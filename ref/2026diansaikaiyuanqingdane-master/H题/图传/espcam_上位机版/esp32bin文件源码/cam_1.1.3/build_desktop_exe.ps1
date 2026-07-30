$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$workPath = Join-Path $projectRoot "desktop_build"
$distPath = Join-Path $projectRoot "desktop_dist"
$logoPath = Join-Path $projectRoot "desktop_assets\company_logo.png"
if (-not (Test-Path -LiteralPath $logoPath)) {
    throw "Desktop logo was not found: $logoPath"
}
$pythonVersion = & python -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
if ($LASTEXITCODE -ne 0) {
    throw "Unable to detect the build Python version"
}

$idfEnvironmentRoot = Join-Path $env:USERPROFILE ".espressif\python_env"
$idfEnvironment = Get-ChildItem -LiteralPath $idfEnvironmentRoot -Directory -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -like "idf*_py${pythonVersion}_env" } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
if ($null -eq $idfEnvironment) {
    throw "No ESP-IDF Python $pythonVersion environment was found; esptool cannot be bundled"
}

$idfPython = Join-Path $idfEnvironment.FullName "Scripts\python.exe"
$idfSitePackages = & $idfPython -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath (Join-Path $idfSitePackages "esptool"))) {
    throw "The selected ESP-IDF Python environment does not contain esptool"
}
$esptoolStubPath = Join-Path $idfSitePackages "esptool\targets\stub_flasher"
if (-not (Test-Path -LiteralPath $esptoolStubPath)) {
    throw "The selected esptool package does not contain ESP32 flasher stub data"
}

New-Item -ItemType Directory -Force -Path $workPath | Out-Null
New-Item -ItemType Directory -Force -Path $distPath | Out-Null

& python -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --windowed `
    --name "ESP32_CAM_Desktop" `
    --add-data "$logoPath;desktop_assets" `
    --add-data "$esptoolStubPath;esptool\targets\stub_flasher" `
    --paths $idfSitePackages `
    --hidden-import "esptool" `
    --collect-all "esptool" `
    --collect-submodules "bitstring" `
    --collect-submodules "bitarray" `
    --collect-submodules "ecdsa" `
    --collect-submodules "intelhex" `
    --hidden-import "reedsolo" `
    --collect-submodules "yaml" `
    --distpath $distPath `
    --workpath $workPath `
    --specpath $workPath `
    (Join-Path $projectRoot "camera_desktop_app.py")

if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller build failed with exit code $LASTEXITCODE"
}

Write-Host "Built: $(Join-Path $distPath 'ESP32_CAM_Desktop.exe')"
