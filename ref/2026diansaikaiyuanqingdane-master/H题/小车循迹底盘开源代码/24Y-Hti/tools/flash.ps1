$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot 'build\Debug\HtiCar.elf'
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware not found: $firmware. Run tools/build.ps1 first."
}

& $env:HTI_OPENOCD_EXE -f $openOcdConfig `
    -c "program {$firmware} verify reset exit"

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD flash failed with exit code $LASTEXITCODE."
}
