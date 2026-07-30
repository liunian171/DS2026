$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot 'build\Debug\HtiCar.elf'
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware not found: $firmware. Run tools/build.ps1 first."
}

$openOcd = Start-Process -FilePath $env:HTI_OPENOCD_EXE `
    -ArgumentList @('-f', $openOcdConfig) `
    -WindowStyle Hidden -PassThru

try {
    Start-Sleep -Milliseconds 1000

    arm-none-eabi-gdb --quiet --batch $firmware `
        -ex 'target extended-remote localhost:3333' `
        -ex 'monitor reset halt' `
        -ex 'thbreak main' `
        -ex 'continue' `
        -ex 'frame' `
        -ex 'info registers pc msp' `
        -ex 'monitor reset run' `
        -ex 'detach'

    if ($LASTEXITCODE -ne 0) {
        throw "GDB smoke test failed with exit code $LASTEXITCODE."
    }
}
finally {
    if (-not $openOcd.HasExited) {
        Stop-Process -Id $openOcd.Id -Force
    }
}
