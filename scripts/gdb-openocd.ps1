param(
    [string]$Elf = "$PSScriptRoot\..\build\speedybee_f405_mini_bringup.elf"
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command arm-none-eabi-gdb -ErrorAction SilentlyContinue)) {
    throw "arm-none-eabi-gdb is not on PATH."
}

$resolvedElf = Resolve-Path -LiteralPath $Elf

arm-none-eabi-gdb $resolvedElf `
    -ex "target extended-remote localhost:3333" `
    -ex "monitor reset halt" `
    -ex "load" `
    -ex "break main" `
    -ex "continue"
