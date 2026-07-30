$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot 'build\Debug\HtiCar.elf'
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware not found: $firmware. Run tools/build.ps1 first."
}

$symbols = & arm-none-eabi-nm.exe --defined-only $firmware

function Get-SymbolAddress([string]$name) {
    $match = $symbols | Select-String -Pattern "^([0-9a-fA-F]+)\s+[A-Za-z]\s+$name$" |
        Select-Object -First 1
    if (-not $match) {
        throw "Symbol not found: $name"
    }
    return $match.Matches[0].Groups[1].Value
}

$leftAddress = Get-SymbolAddress 'g_encoder_left_count'
$rightAddress = Get-SymbolAddress 'g_encoder_right_count'

Write-Host "Left count address : 0x$leftAddress"
Write-Host "Right count address: 0x$rightAddress"

& $env:HTI_OPENOCD_EXE -f $openOcdConfig `
    -c 'init' `
    -c "mdw 0x$leftAddress 1" `
    -c "mdw 0x$rightAddress 1" `
    -c 'shutdown'

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD encoder read failed with exit code $LASTEXITCODE."
}
