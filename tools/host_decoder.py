#!/usr/bin/env python3
"""
SpeedyBee F405 Mini — Binary packet decoder / logger.

Reads fast_packet_t and log_packet_t binary frames from a serial port (or file),
verifies CRC-16/CCITT-FALSE, and prints or logs fields.

Usage:
  python host_decoder.py /dev/ttyUSB0 --baud 2000000
  python host_decoder.py /dev/ttyUSB0 --baud 2000000 --csv out.csv
  python host_decoder.py --file capture.bin
  python host_decoder.py --selftest   # run built-in unit tests
"""

import argparse
import csv
import math
import struct
import sys
import time
from dataclasses import dataclass, field, fields
from typing import Optional

# ---- CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF) ----------------

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    return crc


# ---- Fast packet (96 bytes, little-endian) --------------------------------
# Struct layout mirrors fast_packet_t in fast_packet.h.

FAST_MAGIC   = 0x42F4
FAST_VERSION = 1
FAST_TYPE    = 0x01

FAST_STRUCT_FMT = "<HBBHHQIfffffffffffffffffIH2x"
# Field breakdown (17 floats total):
#   H  magic
#   B  version
#   B  packet_type
#   H  sequence
#   H  _reserved
#   Q  timestamp_us       (8 bytes)
#   I  sample_age_us      (4 bytes)
#   4f q_w/x/y/z          (quaternion)
#   3f roll/pitch/yaw_rad (euler)
#   3f gyro_x/y/z_rad_s
#   3f accel_x/y/z_m_s2
#   3f gravity_body_x/y/z_m_s2
#   1f temperature_deg_c
#   I  health_flags
#   H  crc16
#   2x padding

FAST_PACKET_SIZE = struct.calcsize(FAST_STRUCT_FMT)
assert FAST_PACKET_SIZE == 96, f"fast packet size {FAST_PACKET_SIZE} != 96"

FAST_FIELDS = [
    "magic", "version", "packet_type", "sequence", "_reserved",
    "timestamp_us", "sample_age_us",
    "q_w", "q_x", "q_y", "q_z",
    "roll_rad", "pitch_rad", "yaw_rad",
    "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
    "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
    "gravity_body_x_m_s2", "gravity_body_y_m_s2", "gravity_body_z_m_s2",
    "temperature_deg_c", "health_flags", "crc16",
]

# Health flag bit definitions (from health_monitor.h)
HEALTH_FLAGS = {
    0:  "IMU_OK",
    1:  "BARO_OK",
    2:  "ESTIMATOR_READY",
    3:  "ESTIMATOR_STARTUP",
    4:  "ACCEL_REJECTED",
    5:  "HIGH_LINEAR_ACCEL",
    6:  "HIGH_VIBRATION",
    7:  "GYRO_CLIPPED",
    8:  "ACCEL_CLIPPED",
    9:  "FIFO_OVERFLOW",
    10: "SAMPLE_DROPPED",
    11: "SPI_ERROR",
    12: "I2C_ERROR",
    13: "TIMESTAMP_JITTER",
    14: "PACKET_OVERRUN",
    15: "CONFIG_DIRTY",
    16: "DT_INVALID",
}


def health_flags_str(flags: int) -> str:
    active = [name for bit, name in HEALTH_FLAGS.items() if flags & (1 << bit)]
    return "|".join(active) if active else "OK"


def parse_fast_packet(data: bytes) -> Optional[dict]:
    """Parse and validate a fast packet.  Returns dict or None on error."""
    if len(data) != FAST_PACKET_SIZE:
        return None

    vals = struct.unpack(FAST_STRUCT_FMT, data)
    pkt = dict(zip(FAST_FIELDS, vals))

    # Validate magic, version, type
    if pkt["magic"] != FAST_MAGIC:
        return None
    if pkt["version"] != FAST_VERSION:
        return None
    if pkt["packet_type"] != FAST_TYPE:
        return None

    # Verify CRC (over bytes 0 .. crc16_offset-1)
    crc_offset = FAST_PACKET_SIZE - 4  # 2-byte crc16 + 2 padding at end
    computed = crc16_ccitt(data[:crc_offset])
    if computed != pkt["crc16"]:
        return None

    # Derived: Euler in degrees for display
    pkt["roll_deg"]  = math.degrees(pkt["roll_rad"])
    pkt["pitch_deg"] = math.degrees(pkt["pitch_rad"])
    pkt["yaw_deg"]   = math.degrees(pkt["yaw_rad"])
    pkt["health_str"] = health_flags_str(pkt["health_flags"])

    return pkt


def format_fast_packet(pkt: dict) -> str:
    return (
        f"seq={pkt['sequence']:5d} "
        f"t={pkt['timestamp_us']:10d}µs "
        f"age={pkt['sample_age_us']:4d}µs | "
        f"roll={pkt['roll_deg']:+7.2f}° "
        f"pitch={pkt['pitch_deg']:+7.2f}° "
        f"yaw={pkt['yaw_deg']:+7.2f}° | "
        f"gx={math.degrees(pkt['gyro_x_rad_s']):+6.1f}°/s "
        f"gy={math.degrees(pkt['gyro_y_rad_s']):+6.1f}°/s "
        f"gz={math.degrees(pkt['gyro_z_rad_s']):+6.1f}°/s | "
        f"temp={pkt['temperature_deg_c']:5.1f}°C | "
        f"[{pkt['health_str']}]"
    )


# ---- Serial / file framing -----------------------------------------------

class PacketReader:
    """State-machine byte-by-byte packet synchroniser."""

    MAGIC_LOW  = FAST_MAGIC & 0xFF
    MAGIC_HIGH = (FAST_MAGIC >> 8) & 0xFF

    def __init__(self):
        self.buf   = bytearray()
        self.synced = False
        self.stats = {"total_bytes": 0, "good": 0, "bad_crc": 0, "dropped": 0}

    def feed(self, data: bytes):
        """Feed raw bytes.  Yields decoded packet dicts."""
        for byte in data:
            self.stats["total_bytes"] += 1

            if not self.synced:
                self.buf.append(byte)
                if len(self.buf) >= 2:
                    if (self.buf[-2] == self.MAGIC_LOW and
                            self.buf[-1] == self.MAGIC_HIGH):
                        # Found magic bytes: start collecting from here.
                        self.buf = bytearray([self.MAGIC_LOW, self.MAGIC_HIGH])
                        self.synced = True
                    elif len(self.buf) > 4:
                        self.buf = self.buf[-4:]
            else:
                self.buf.append(byte)
                if len(self.buf) == FAST_PACKET_SIZE:
                    pkt = parse_fast_packet(bytes(self.buf))
                    if pkt is not None:
                        self.stats["good"] += 1
                        yield pkt
                    else:
                        self.stats["bad_crc"] += 1
                    self.buf.clear()
                    self.synced = False


# ---- Self-test -----------------------------------------------------------

def selftest():
    print("Running host decoder self-tests...")

    # CRC known vector
    data = b"123456789"
    crc  = crc16_ccitt(data)
    assert crc == 0x29B1, f"CRC test failed: 0x{crc:04X}"
    print("  CRC-16 known vector: PASS")

    # Build a valid fast packet manually
    q       = (0.9999, 0.001, 0.002, 0.003)
    gyro    = (0.01, -0.02, 0.005)
    accel   = (0.1, 0.2, 9.8)
    gravity = (0.0, 0.0, 9.80665)

    # Pack without CRC first (last 4 bytes = crc16 + pad)
    raw = struct.pack(
        FAST_STRUCT_FMT,
        FAST_MAGIC, FAST_VERSION, FAST_TYPE,
        42, 0,             # sequence, reserved
        123456789, 500,    # timestamp_us, sample_age_us
        *q,                # quaternion
        0.05, -0.1, 0.001, # euler
        *gyro,             # gyro
        *accel,            # accel
        *gravity,          # gravity_body
        25.5,              # temperature
        0x00000003,        # health_flags
        0x0000,            # crc (placeholder)
    )
    crc_offset = FAST_PACKET_SIZE - 4
    correct_crc = crc16_ccitt(raw[:crc_offset])

    # Patch CRC into bytes
    raw_with_crc = raw[:crc_offset] + struct.pack("<H", correct_crc) + b'\x00\x00'
    pkt = parse_fast_packet(raw_with_crc)
    assert pkt is not None, "Valid packet rejected"
    assert pkt["sequence"] == 42
    assert abs(pkt["pitch_rad"] - (-0.1)) < 1e-5, f"Pitch mismatch: {pkt['pitch_rad']}"
    print("  Fast packet encode/decode: PASS")

    # Corrupt one byte → CRC should fail
    corrupted = bytearray(raw_with_crc)
    corrupted[10] ^= 0x01
    pkt_bad = parse_fast_packet(bytes(corrupted))
    assert pkt_bad is None, "Corrupted packet was accepted"
    print("  Corrupted packet rejected: PASS")

    # Health flags
    flags_str = health_flags_str(0x00000003)
    assert "IMU_OK" in flags_str and "BARO_OK" in flags_str, f"Flag parse: {flags_str}"
    print("  Health flags: PASS")

    print("Self-tests PASSED")
    return True


# ---- Main ----------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SpeedyBee F405 packet decoder")
    parser.add_argument("port", nargs="?", help="Serial port (e.g. /dev/ttyUSB0 or COM3)")
    parser.add_argument("--baud", type=int, default=2000000)
    parser.add_argument("--file", help="Read from binary file instead of serial port")
    parser.add_argument("--csv", help="Log decoded packets to CSV file")
    parser.add_argument("--selftest", action="store_true", help="Run self-tests and exit")
    parser.add_argument("--quiet", action="store_true", help="Suppress per-packet console output")
    args = parser.parse_args()

    if args.selftest:
        success = selftest()
        sys.exit(0 if success else 1)

    csv_writer = None
    csv_file   = None
    csv_fields = [
        "timestamp_us", "sequence", "sample_age_us",
        "roll_deg", "pitch_deg", "yaw_deg",
        "q_w", "q_x", "q_y", "q_z",
        "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
        "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
        "gravity_body_x_m_s2", "gravity_body_y_m_s2", "gravity_body_z_m_s2",
        "temperature_deg_c", "health_flags", "health_str",
    ]

    if args.csv:
        csv_file   = open(args.csv, "w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=csv_fields,
                                    extrasaction="ignore")
        csv_writer.writeheader()
        print(f"Logging to {args.csv}")

    reader = PacketReader()
    prev_seq = None
    t_start  = time.monotonic()

    try:
        if args.file:
            with open(args.file, "rb") as f:
                while True:
                    chunk = f.read(256)
                    if not chunk:
                        break
                    for pkt in reader.feed(chunk):
                        _process(pkt, prev_seq, csv_writer, args.quiet)
                        prev_seq = pkt["sequence"]
        else:
            if not args.port:
                parser.error("Specify a serial port or --file / --selftest")

            import serial  # type: ignore
            print(f"Opening {args.port} at {args.baud} baud...")
            with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
                print("Listening for packets (Ctrl-C to stop)...\n")
                while True:
                    chunk = ser.read(256)
                    if chunk:
                        for pkt in reader.feed(chunk):
                            _process(pkt, prev_seq, csv_writer, args.quiet)
                            prev_seq = pkt["sequence"]

    except KeyboardInterrupt:
        elapsed = time.monotonic() - t_start
        s = reader.stats
        print(f"\n--- Summary ---")
        print(f"Elapsed:    {elapsed:.1f} s")
        print(f"Bytes:      {s['total_bytes']}")
        print(f"Good pkts:  {s['good']}")
        print(f"Bad CRC:    {s['bad_crc']}")
        print(f"Rate:       {s['good']/max(elapsed,1):.1f} pkts/s")
    finally:
        if csv_file:
            csv_file.close()


def _process(pkt, prev_seq, csv_writer, quiet):
    if prev_seq is not None:
        expected = (prev_seq + 1) & 0xFFFF
        if pkt["sequence"] != expected:
            drops = (pkt["sequence"] - expected) & 0xFFFF
            print(f"  [WARNING] Sequence gap: expected {expected}, got {pkt['sequence']} ({drops} dropped)")

    if not quiet:
        print(format_fast_packet(pkt))

    if csv_writer:
        csv_writer.writerow(pkt)


if __name__ == "__main__":
    main()
