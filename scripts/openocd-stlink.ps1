$ErrorActionPreference = "Stop"

if (-not (Get-Command openocd -ErrorAction SilentlyContinue)) {
    throw "openocd is not on PATH. Restart the terminal so Winget's user PATH update is visible."
}

openocd `
    -f interface/stlink.cfg `
    -f target/stm32f4x.cfg `
    -c "adapter speed 4000"
