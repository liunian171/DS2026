from __future__ import annotations

import os
import sys
from pathlib import Path


APPLICATION_DATA_DIRECTORY = "ESP32CamDesktop"


def bundled_resource_path(relative_path: str) -> Path:
    """返回源码目录或 PyInstaller 临时目录中的只读资源路径。

    Args:
        relative_path: 相对于项目资源根目录的文件路径。

    Returns:
        源码模式或单文件 EXE 解包模式下对应的绝对资源路径。
    """

    bundled_root = getattr(sys, "_MEIPASS", None)
    resource_root = Path(bundled_root) if bundled_root else Path(__file__).resolve().parent
    return resource_root / relative_path


def runtime_data_directory() -> Path:
    """返回源码模式或 Windows EXE 模式下的可持久化数据目录。

    Args:
        None.

    Returns:
        已创建的数据目录路径。EXE 模式使用
        ``%LOCALAPPDATA%\ESP32CamDesktop``，源码模式使用项目目录。
    """

    if getattr(sys, "frozen", False):
        local_app_data = os.environ.get("LOCALAPPDATA")
        root = Path(local_app_data) if local_app_data else Path.home() / "AppData" / "Local"
        data_directory = root / APPLICATION_DATA_DIRECTORY
    else:
        data_directory = Path(__file__).resolve().parent
    data_directory.mkdir(parents=True, exist_ok=True)
    return data_directory
