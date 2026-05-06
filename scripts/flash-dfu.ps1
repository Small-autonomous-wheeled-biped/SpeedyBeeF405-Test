param(
    [string]$Binary = "$PSScriptRoot\..\build\speedybee_f405_mini_bringup.bin"
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command dfu-util -ErrorAction SilentlyContinue)) {
    throw "dfu-util is not on PATH. Restart the terminal or add C:\Users\marcu\Tools\dfu-util\bin to PATH."
}

$resolvedBinary = Resolve-Path -LiteralPath $Binary

Write-Host "Listing DFU devices..."
dfu-util -l

Write-Host "Flashing $resolvedBinary to STM32 internal flash at 0x08000000..."
dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D $resolvedBinary
