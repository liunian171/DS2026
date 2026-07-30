$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$workPath = Join-Path $projectRoot "desktop_build"
$distPath = Join-Path $projectRoot "desktop_dist"

New-Item -ItemType Directory -Force -Path $workPath | Out-Null
New-Item -ItemType Directory -Force -Path $distPath | Out-Null

& python -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --windowed `
    --name "ESP32_CAM_Desktop" `
    --distpath $distPath `
    --workpath $workPath `
    --specpath $workPath `
    (Join-Path $projectRoot "camera_desktop_app.py")

if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller build failed with exit code $LASTEXITCODE"
}

Write-Host "Built: $(Join-Path $distPath 'ESP32_CAM_Desktop.exe')"
