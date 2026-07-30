# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_submodules
from PyInstaller.utils.hooks import collect_all

datas = [('D:\\ESP32_Project\\2026\\2026_3\\aaa\\desktop_assets\\company_logo.png', 'desktop_assets'), ('C:\\Users\\yyq\\.espressif\\python_env\\idf5.5_py3.10_env\\Lib\\site-packages\\esptool\\targets\\stub_flasher', 'esptool\\targets\\stub_flasher')]
binaries = []
hiddenimports = ['esptool', 'reedsolo']
hiddenimports += collect_submodules('bitstring')
hiddenimports += collect_submodules('bitarray')
hiddenimports += collect_submodules('ecdsa')
hiddenimports += collect_submodules('intelhex')
hiddenimports += collect_submodules('yaml')
tmp_ret = collect_all('esptool')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]


a = Analysis(
    ['D:\\ESP32_Project\\2026\\2026_3\\aaa\\camera_desktop_app.py'],
    pathex=['C:\\Users\\yyq\\.espressif\\python_env\\idf5.5_py3.10_env\\Lib\\site-packages'],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='ESP32_CAM_Desktop',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
