#include "fast_packet.h"
#include "log_packet.h"
#include "crc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ASSERT_TRUE(expr) assert_true((expr), #expr, __FILE__, __LINE__)
#define ASSERT_FALSE(expr) assert_true(!(expr), "!(" #expr ")", __FILE__, __LINE__)
#define ASSERT_EQ_U16(a, b) do { uint16_t _a=(a),_b=(b); if(_a!=_b){ \
    fprintf(stderr, "%s:%d: %s: expected %u got %u\n",__FILE__,__LINE__,#a,(unsigned)_b,(unsigned)_a); exit(1); } } while(0)
#define ASSERT_EQ_U32(a, b) do { uint32_t _a=(a),_b=(b); if(_a!=_b){ \
    fprintf(stderr, "%s:%d: %s: expected %lu got %lu\n",__FILE__,__LINE__,#a,(unsigned long)_b,(unsigned long)_a); exit(1); } } while(0)

static void assert_true(bool passed, const char *expr, const char *file, int line)
{
    if (!passed) {
        fprintf(stderr, "%s:%d: FAILED: %s\n", file, line, expr);
        exit(1);
    }
}

static void test_fast_packet_size(void)
{
    ASSERT_EQ_U32((uint32_t)sizeof(fast_packet_t), (uint32_t)FAST_PACKET_SIZE);
}

static void test_fast_packet_encode_decode_roundtrip(void)
{
    fast_packet_t pkt;
    uint16_t seq = 42U;

    const float q[4]           = { 0.9999f, 0.001f, 0.002f, 0.003f };
    const float gyro[3]        = { 0.01f, -0.02f, 0.005f };
    const float accel[3]       = { 0.1f, 0.2f, 9.8f };
    const float gravity_body[3]= { 0.0f, 0.0f, 9.80665f };

    fast_packet_encode(&pkt, &seq,
                       123456789ULL, 500U,
                       q, 0.05f, -0.1f, 0.001f,
                       gyro, accel, gravity_body,
                       25.5f, 0x00000003U);

    ASSERT_EQ_U16(pkt.magic,   FAST_PACKET_MAGIC);
    ASSERT_EQ_U16(seq, 43U);  /* sequence incremented */
    ASSERT_EQ_U32(pkt.health_flags, 3U);

    ASSERT_TRUE(fast_packet_decode(&pkt));

    /* Corrupt one byte — CRC should fail. */
    pkt.roll_rad += 0.001f;
    ASSERT_FALSE(fast_packet_decode(&pkt));
}

static void test_fast_packet_crc_covers_all_fields(void)
{
    fast_packet_t pkt1, pkt2;
    uint16_t seq = 0U;

    const float q[4]    = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float z3[3]   = { 0.0f, 0.0f, 0.0f };
    const float g3[3]   = { 0.0f, 0.0f, 9.80665f };

    fast_packet_encode(&pkt1, &seq, 0ULL, 0U, q, 0.0f, 0.0f, 0.0f,
                        z3, z3, g3, 25.0f, 0U);
    seq = 0U;
    fast_packet_encode(&pkt2, &seq, 0ULL, 0U, q, 0.0f, 0.0f, 0.0f,
                        z3, z3, g3, 25.0f, 0U);

    /* Same inputs → same CRC. */
    ASSERT_EQ_U16(pkt1.crc16, pkt2.crc16);

    /* Different pitch → different CRC. */
    seq = 0U;
    fast_packet_encode(&pkt2, &seq, 0ULL, 0U, q, 0.0f, 0.1f, 0.0f,
                        z3, z3, g3, 25.0f, 0U);
    ASSERT_TRUE(pkt1.crc16 != pkt2.crc16);
}

static void test_fast_packet_wrong_magic_rejected(void)
{
    fast_packet_t pkt;
    uint16_t seq = 0U;
    const float q[4]  = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float z3[3] = { 0.0f, 0.0f, 0.0f };
    const float g3[3] = { 0.0f, 0.0f, 9.80665f };

    fast_packet_encode(&pkt, &seq, 0ULL, 0U, q, 0.0f, 0.0f, 0.0f,
                        z3, z3, g3, 25.0f, 0U);
    pkt.magic = 0xDEADU;
    ASSERT_FALSE(fast_packet_decode(&pkt));
}

static void test_fast_packet_sequence_increments(void)
{
    fast_packet_t pkt;
    uint16_t seq = 100U;
    const float q[4]  = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float z3[3] = { 0.0f, 0.0f, 0.0f };
    const float g3[3] = { 0.0f, 0.0f, 9.80665f };

    for (uint16_t expected = 100U; expected < 110U; expected++) {
        fast_packet_encode(&pkt, &seq, 0ULL, 0U, q, 0.0f, 0.0f, 0.0f,
                            z3, z3, g3, 25.0f, 0U);
        ASSERT_EQ_U16(pkt.sequence, expected);
        ASSERT_TRUE(fast_packet_decode(&pkt));
    }
}

int main(void)
{
    test_fast_packet_size();
    test_fast_packet_encode_decode_roundtrip();
    test_fast_packet_crc_covers_all_fields();
    test_fast_packet_wrong_magic_rejected();
    test_fast_packet_sequence_increments();

    puts("test_packets: all tests passed");
    return 0;
}
