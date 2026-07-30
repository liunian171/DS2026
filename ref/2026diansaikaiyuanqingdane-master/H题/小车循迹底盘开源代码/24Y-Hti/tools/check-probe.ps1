$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'setup-env.ps1')

& $env:HTI_OPENOCD_EXE `
    -f (Join-Path $projectRoot 'openocd\openocd.cfg') `
    -c 'init; reset halt; mdw 0xE0042000 1; shutdown'

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD connectivity check failed with exit code $LASTEXITCODE."
}
