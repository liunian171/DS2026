$ErrorActionPreference = 'Stop'

$armBin = 'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin'
$cmakeBin = 'C:\Program Files\CMake\bin'
$winGetPackages = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages'

$openOcdExe = Get-ChildItem -LiteralPath $winGetPackages -Recurse -Filter 'openocd.exe' |
    Select-Object -First 1 -ExpandProperty FullName
$ninjaExe = Get-ChildItem -LiteralPath $winGetPackages -Recurse -Filter 'ninja.exe' |
    Select-Object -First 1 -ExpandProperty FullName

if (-not (Test-Path -LiteralPath (Join-Path $armBin 'arm-none-eabi-gcc.exe'))) {
    throw 'GNU Arm Embedded Toolchain was not found.'
}
if (-not $openOcdExe) {
    throw 'OpenOCD was not found.'
}
if (-not (Test-Path -LiteralPath (Join-Path $cmakeBin 'cmake.exe'))) {
    throw 'CMake was not found.'
}
if (-not $ninjaExe) {
    throw 'Ninja was not found.'
}

$env:Path = @(
    $armBin
    (Split-Path -Parent $openOcdExe)
    $cmakeBin
    (Split-Path -Parent $ninjaExe)
    $env:Path
) -join [IO.Path]::PathSeparator

$env:HTI_OPENOCD_EXE = $openOcdExe
$env:HTI_NINJA_EXE = $ninjaExe

Write-Host 'STM32 tool environment is ready.'
