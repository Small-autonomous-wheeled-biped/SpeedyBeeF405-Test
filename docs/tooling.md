# Firmware Install And Debug Tooling

Installed tools:

- `dfu-util 0.11` in `C:\Users\marcu\Tools\dfu-util\bin`
- `openocd-xpack 0.12.0-7` through Winget user-scope install
- `arm-none-eabi-gdb` from the existing Arm GNU Toolchain
- `Zadig 2.9` through Winget user-scope install
- VS Code Cortex-Debug extension was already installed

Restart new terminals after installation so the user PATH updates are visible.

## DFU Flashing

Use this when flashing through the board's USB-C port and BOOT button.

1. Hold BOOT.
2. Plug in USB-C.
3. Release BOOT.
4. Run:

```powershell
.\scripts\flash-dfu.ps1
```

If `dfu-util -l` does not show an STM32 DFU device, open Zadig, enable
`Options > List All Devices`, select `STM32 BOOTLOADER`, and install the WinUSB
driver. Be careful to select the STM32 bootloader, not your keyboard, mouse, or
another USB device.

The helper script filters for ST's ROM DFU USB ID `0483:df11` before flashing.
Other DFU-capable devices may appear in `dfu-util -l`; do not flash them.

## SWD Debugging

Use this with an ST-Link connected to the board's SWD pads:

- ST-Link `SWDIO` to board `PA13/SWDIO`
- ST-Link `SWCLK` to board `PA14/SWCLK`
- common `GND`
- target voltage reference to board `3V3`

Start OpenOCD in one terminal:

```powershell
.\scripts\openocd-stlink.ps1
```

Start GDB in another terminal:

```powershell
.\scripts\gdb-openocd.ps1
```

VS Code can also use `.vscode/launch.json` with the Cortex-Debug extension.

## No-Board Testing

Use the host test target to exercise driver logic without flashing firmware:

```powershell
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

The tests link `src/icm42688p.c` and `src/dps310.c` against a fake board I/O
layer under `tests/host`. They do not validate STM32 register writes or physical
pin behavior.

Run the same static analysis used in CI with:

```powershell
cppcheck --enable=warning,style,performance,portability --error-exitcode=1 --std=c11 --inline-suppr -I include src include
```
