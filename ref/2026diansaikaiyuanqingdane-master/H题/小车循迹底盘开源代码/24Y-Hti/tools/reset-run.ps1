$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

& $env:HTI_OPENOCD_EXE -f $openOcdConfig `
    -c 'init' `
    -c 'reset run' `
    -c 'shutdown'
if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD reset failed with exit code $LASTEXITCODE."
}
