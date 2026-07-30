from __future__ import annotations

import os
import sys
from pathlib import Path


APPLICATION_DATA_DIRECTORY = "ESP32CamDesktop"


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
