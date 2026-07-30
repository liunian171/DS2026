$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

Write-Host 'AFIO MAPR:'
Write-Host 'GPIOA CRH/IDR:'
Write-Host 'GPIOB CRL/CRH/IDR:'
Write-Host 'TIM2 CR1/SMCR/CCER/CNT/ARR and TIM4 equivalents follow.'

& $env:HTI_OPENOCD_EXE -f $openOcdConfig `
    -c 'init' `
    -c 'mdw 0x40010004 1' `
    -c 'mdw 0x40010804 2' `
    -c 'mdw 0x40010c00 3' `
    -c 'mdw 0x40000000 12' `
    -c 'mdw 0x40000800 12' `
    -c 'shutdown'

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD encoder hardware read failed with exit code $LASTEXITCODE."
}
