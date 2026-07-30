from __future__ import annotations

import contextlib
import importlib
import io
import ipaddress
import json
import locale
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
import winreg
from dataclasses import dataclass
from http.server import ThreadingHTTPServer
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from types import ModuleType
from typing import Callable

import cv2
import numpy as np
import serial
from PIL import Image, ImageTk

from camera_web_server import CameraRequestHandler, STATUS_PATH, ensure_storage
from desktop_paths import bundled_resource_path, runtime_data_directory


BASE_DIR = runtime_data_directory()
DESKTOP_CONFIG_PATH = BASE_DIR / "desktop_config.json"
COMPANY_LOGO_PATH = bundled_resource_path("desktop_assets/company_logo.png")
SERIAL_BAUD_RATE = 115200
SERIAL_PROMPT_TIMEOUT_SECONDS = 8.0
WIFI_CONNECT_TIMEOUT_SECONDS = 70.0
DEFAULT_UPLOAD_PORT = 8000
STREAM_PORT = 81
STREAM_PATH = "/stream"
RECORDING_FPS = 15.0
RECORDING_QUEUE_SIZE = 30
FIRMWARE_FLASH_ADDRESS = "0x10000"
FIRMWARE_FLASH_BAUD_RATE = 460800
DEVICE_IP_PATTERN = re.compile(
    r"(?:Device IP:\s*|wifi connected, ip=)(\d{1,3}(?:\.\d{1,3}){3})"
)


@dataclass(frozen=True)
class WifiConfigurationResult:
    """保存一次 ESP32 Wi-Fi 与地址更新操作的结果。

    Attributes:
        success: Wi-Fi 和上传地址是否全部配置成功。
        message: 适合显示给用户的结果说明。
        device_ip: ESP32 获取的新 IPv4 地址；失败或未知时为空。
        computer_ip: 电脑访问 ESP32 时使用的 IPv4 地址；未知时为空。
        upload_url: 已写入 ESP32 的上传地址；未写入时为空。
    """

    success: bool
    message: str
    device_ip: str = ""
    computer_ip: str = ""
    upload_url: str = ""


@dataclass(frozen=True)
class RecordingResult:
    """保存一次 MP4 录制任务的最终结果。

    Attributes:
        output_path: 用户选择的 MP4 输出路径。
        frame_count: 成功写入文件的视频帧数量。
        elapsed_seconds: 从开始录制到写入器关闭的总秒数。
        dropped_frames: 写入队列拥堵时主动丢弃的旧帧数量。
        error: 录制失败原因；成功时为空字符串。
    """

    output_path: Path
    frame_count: int
    elapsed_seconds: float
    dropped_frames: int
    error: str = ""


@dataclass(frozen=True)
class FirmwareFlashResult:
    """保存一次 ESP32-CAM 应用固件烧录任务的结果。

    Attributes:
        success: esptool 是否完成写入、校验和硬复位。
        firmware_path: 用户选择的应用固件 bin 路径。
        message: 适合显示在界面和日志中的结果文本。
    """

    success: bool
    firmware_path: Path
    message: str


def list_serial_ports() -> list[tuple[str, str]]:
    """从 Windows 注册表列出当前存在的串口，不打开 COM 设备。

    Args:
        None.

    Returns:
        按端口名排序的 ``(端口名, 注册表设备名)`` 列表。
    """

    ports: list[tuple[str, str]] = []
    try:
        with winreg.OpenKey(
            winreg.HKEY_LOCAL_MACHINE,
            r"HARDWARE\DEVICEMAP\SERIALCOMM",
        ) as serial_key:
            value_index = 0
            while True:
                try:
                    device_name, port_name, _ = winreg.EnumValue(serial_key, value_index)
                except OSError:
                    break
                ports.append((str(port_name), str(device_name)))
                value_index += 1
    except OSError:
        return []
    return sorted(ports, key=lambda item: item[0])


def serial_port_availability_error(port: str) -> str:
    """尝试独占打开并立即释放串口，检查烧录前是否被占用。

    Args:
        port: 需要检查的 Windows 串口名称，例如 ``COM61``。

    Returns:
        串口可用时返回空字符串；不存在、被占用或无法打开时返回错误文本。
    """

    available_ports = {device.upper() for device, _ in list_serial_ports()}
    if port.upper() not in available_ports:
        return f"未检测到串口 {port}"

    connection = serial.Serial()
    connection.port = port
    connection.baudrate = SERIAL_BAUD_RATE
    connection.timeout = 0.2
    connection.write_timeout = 1.0
    connection.dtr = False
    connection.rts = False
    try:
        connection.open()
        connection.dtr = False
        connection.rts = False
    except (OSError, ValueError, serial.SerialException) as exc:
        return f"串口 {port} 被占用或无法打开：{exc}"
    finally:
        if connection.is_open:
            connection.close()
    return ""


def get_connected_wifi_ssid() -> str | None:
    """读取 Windows 当前连接的 Wi-Fi SSID。

    Args:
        None.

    Returns:
        当前已连接的 SSID；无法检测或未连接时返回 ``None``。
    """

    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            capture_output=True,
            text=True,
            encoding=locale.getpreferredencoding(False),
            errors="replace",
            timeout=5,
            check=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except (OSError, subprocess.SubprocessError):
        return None

    for line in result.stdout.splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key.strip().upper() == "SSID" and value.strip():
            return value.strip()
    return None


def scan_available_wifi_ssids() -> list[str]:
    """使用 Windows 无线网卡扫描并返回附近可见的 Wi-Fi 名称。

    Args:
        None.

    Returns:
        去重并按名称排序的非空 SSID 列表。

    Raises:
        RuntimeError: Windows 无线网络命令执行失败或无线服务不可用。
    """

    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "networks", "mode=bssid"],
            capture_output=True,
            text=True,
            encoding=locale.getpreferredencoding(False),
            errors="replace",
            timeout=10,
            check=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError(f"无法调用 Windows Wi-Fi 扫描：{exc}") from exc
    if result.returncode != 0:
        error_text = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(error_text or "Windows Wi-Fi 扫描失败")

    ssid_pattern = re.compile(r"^\s*SSID\s+\d+\s*:\s*(.*?)\s*$", re.IGNORECASE)
    networks: set[str] = set()
    for line in result.stdout.splitlines():
        match = ssid_pattern.match(line)
        if match is None:
            continue
        ssid = match.group(1).strip()
        if ssid:
            networks.add(ssid)
    return sorted(networks, key=str.casefold)


def is_valid_ipv4(value: str) -> bool:
    """判断文本是否为可用的非回环 IPv4 地址。

    Args:
        value: 待检查的地址文本。

    Returns:
        地址合法且不是回环地址时返回 ``True``，否则返回 ``False``。
    """

    try:
        parsed = ipaddress.ip_address(value.strip())
    except ValueError:
        return False
    return parsed.version == 4 and not parsed.is_loopback and not parsed.is_unspecified


def local_ip_for_peer(peer_ip: str) -> str:
    """获取 Windows 访问指定 ESP32 时实际选择的本机 IPv4 地址。

    Args:
        peer_ip: ESP32 的新 IPv4 地址。

    Returns:
        对应网络接口的本机 IPv4 地址。

    Raises:
        OSError: 系统无法为目标地址选择可用网络路由。
        ValueError: 得到的地址不是有效非回环 IPv4 地址。
    """

    routed_ip = local_ip_from_windows_routes(peer_ip)
    if routed_ip:
        return routed_ip

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.connect((peer_ip, STREAM_PORT))
        local_ip = str(probe.getsockname()[0])
    if not is_valid_ipv4(local_ip):
        raise ValueError("无法确定电脑在目标 Wi-Fi 中的 IPv4 地址")
    return local_ip


def local_ip_from_windows_routes(peer_ip: str) -> str:
    """从 Windows IPv4 路由表中选择匹配 ESP32 最精确的本机接口地址。

    Args:
        peer_ip: ESP32 的目标 IPv4 地址。

    Returns:
        最长前缀匹配路由的本机接口地址；读取失败时返回空字符串。
    """

    try:
        peer = ipaddress.ip_address(peer_ip)
        result = subprocess.run(
            ["route", "print", "-4"],
            capture_output=True,
            text=True,
            encoding=locale.getpreferredencoding(False),
            errors="replace",
            timeout=5,
            check=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except (OSError, ValueError, subprocess.SubprocessError):
        return ""

    route_pattern = re.compile(
        r"^\s*(\d{1,3}(?:\.\d{1,3}){3})\s+"
        r"(\d{1,3}(?:\.\d{1,3}){3})\s+.+?\s+"
        r"(\d{1,3}(?:\.\d{1,3}){3})\s+\d+\s*$"
    )
    best_match: tuple[int, str] | None = None
    for line in result.stdout.splitlines():
        match = route_pattern.match(line)
        if match is None:
            continue
        destination, netmask, interface_ip = match.groups()
        try:
            network = ipaddress.ip_network(f"{destination}/{netmask}", strict=False)
        except ValueError:
            continue
        if peer not in network or not is_valid_ipv4(interface_ip):
            continue
        candidate = (network.prefixlen, interface_ip)
        if best_match is None or candidate[0] > best_match[0]:
            best_match = candidate
    return best_match[1] if best_match is not None else ""


def read_last_device_ip() -> str:
    """从现有图片接收服务状态中读取最近一次 ESP32 地址。

    Args:
        None.

    Returns:
        有效的历史 ESP32 IPv4 地址；没有有效记录时返回空字符串。
    """

    try:
        state = json.loads(STATUS_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return ""
    device_ip = str(state.get("device_ip", "")).strip()
    return device_ip if is_valid_ipv4(device_ip) else ""


def load_desktop_config() -> dict[str, str]:
    """读取不包含密码的桌面端配置。

    Args:
        None.

    Returns:
        包含串口、SSID 和设备 IP 的配置字典；读取失败时返回空字典。
    """

    try:
        data = json.loads(DESKTOP_CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {str(key): str(value) for key, value in data.items() if isinstance(value, str)}


def save_desktop_config(*, port: str, ssid: str, device_ip: str) -> None:
    """保存桌面端非敏感配置，不保存 Wi-Fi 密码。

    Args:
        port: 最近使用的串口名称。
        ssid: 最近配置的 Wi-Fi 名称。
        device_ip: 最近连接成功的 ESP32 IPv4 地址。

    Returns:
        None.
    """

    payload = {"port": port, "ssid": ssid, "device_ip": device_ip}
    DESKTOP_CONFIG_PATH.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def load_esptool_module() -> ModuleType:
    """加载内置 esptool，源码模式下可回退到本机 ESP-IDF Python 环境。

    Args:
        None.

    Returns:
        已导入并可调用 ``main`` 的 esptool 模块。

    Raises:
        RuntimeError: EXE 未包含 esptool，或源码环境中未找到 ESP-IDF 的 esptool。
    """

    try:
        return importlib.import_module("esptool")
    except ImportError:
        pass

    candidate_roots: list[Path] = []
    configured_environment = os.environ.get("IDF_PYTHON_ENV_PATH")
    if configured_environment:
        candidate_roots.append(Path(configured_environment))
    python_environments = Path.home() / ".espressif" / "python_env"
    if python_environments.is_dir():
        candidate_roots.extend(
            sorted(
                python_environments.glob("idf*_py*_env"),
                key=lambda item: item.stat().st_mtime,
                reverse=True,
            )
        )

    for environment_root in candidate_roots:
        site_packages = environment_root / "Lib" / "site-packages"
        if not (site_packages / "esptool").is_dir():
            continue
        site_packages_text = str(site_packages)
        if site_packages_text not in sys.path:
            sys.path.insert(0, site_packages_text)
        try:
            return importlib.import_module("esptool")
        except ImportError:
            continue
    raise RuntimeError("未找到 esptool；请使用包含烧录工具的 EXE，或安装 ESP-IDF 环境")


class EsptoolLogWriter:
    """把 esptool 的标准输出按行转换为上位机日志事件。"""

    def __init__(self, log_callback: Callable[[str], None]) -> None:
        """保存日志回调并初始化未完成行缓冲区。

        Args:
            log_callback: 接收单行 esptool 文本的线程安全回调。

        Returns:
            None.
        """

        self.log_callback = log_callback
        self.pending_text = ""

    def write(self, value: str) -> int:
        """接收 esptool 输出，并将完整内容投递到日志。

        Args:
            value: esptool 写入标准输出或标准错误的文本。

        Returns:
            已接收的字符数量，符合文本流 ``write`` 接口要求。
        """

        self.pending_text += value.replace("\r", "\n")
        while "\n" in self.pending_text:
            line, self.pending_text = self.pending_text.split("\n", 1)
            line = line.strip()
            if line:
                self.log_callback(line)
        return len(value)

    def flush(self) -> None:
        """把尚未换行的剩余 esptool 文本投递到日志。

        Args:
            None.

        Returns:
            None.
        """

        line = self.pending_text.strip()
        self.pending_text = ""
        if line:
            self.log_callback(line)

    def isatty(self) -> bool:
        """声明日志接收器不是交互式终端。

        Args:
            None.

        Returns:
            始终返回 False，避免 esptool 输出交互式控制序列。
        """

        return False


class ExclusiveThreadingHTTPServer(ThreadingHTTPServer):
    """使用独占端口绑定的桌面端 HTTP 服务。"""

    allow_reuse_address = False

    def server_bind(self) -> None:
        """绑定监听端口，并在 Windows 上禁止其他进程复用该端口。

        Args:
            None.

        Returns:
            None.
        """

        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


class EmbeddedUploadServer:
    """在桌面程序内启动现有图片上传 HTTP 服务。"""

    def __init__(self, preferred_port: int = DEFAULT_UPLOAD_PORT) -> None:
        """初始化服务状态，但不立即监听端口。

        Args:
            preferred_port: 首选 HTTP 监听端口，默认使用 8000。

        Returns:
            None.
        """

        self.preferred_port = preferred_port
        self.port = 0
        self.server: ExclusiveThreadingHTTPServer | None = None
        self.thread: threading.Thread | None = None

    def start(self) -> int:
        """启动上传服务，首选端口被占用时依次尝试后续端口。

        Args:
            None.

        Returns:
            实际监听的 TCP 端口。

        Raises:
            OSError: 连续候选端口均无法绑定。
        """

        ensure_storage()
        last_error: OSError | None = None
        for candidate in range(self.preferred_port, self.preferred_port + 11):
            try:
                server = ExclusiveThreadingHTTPServer(
                    ("0.0.0.0", candidate), CameraRequestHandler
                )
            except OSError as exc:
                last_error = exc
                continue
            server.daemon_threads = True
            self.server = server
            self.port = candidate
            self.thread = threading.Thread(
                target=server.serve_forever,
                name="camera-upload-server",
                daemon=True,
            )
            self.thread.start()
            return candidate
        if last_error is not None:
            raise last_error
        raise OSError("无法启动图片上传服务")

    def stop(self) -> None:
        """停止桌面程序内启动的 HTTP 服务。

        Args:
            None.

        Returns:
            None.
        """

        if self.server is None:
            return
        self.server.shutdown()
        self.server.server_close()
        self.server = None


class Esp32SerialConfigurator:
    """通过 ESP32 控制台串口完成 Wi-Fi 和上传地址配置。"""

    def __init__(self, status_callback: Callable[[str], None]) -> None:
        """保存过程状态回调。

        Args:
            status_callback: 接收简短配置进度文本的线程安全回调。

        Returns:
            None.
        """

        self.status_callback = status_callback

    def configure(self, *, port: str, ssid: str, password: str, upload_port: int) -> WifiConfigurationResult:
        """配置 ESP32 Wi-Fi，随后自动写入电脑的新上传地址。

        Args:
            port: ESP32 所在串口，例如 ``COM61``。
            ssid: 目标 Wi-Fi 名称。
            password: 目标 Wi-Fi 密码；开放网络可为空。
            upload_port: 桌面程序图片接收服务实际监听端口。

        Returns:
            包含新设备 IP、电脑 IP 和最终状态的配置结果。
        """

        device_ip = ""
        computer_ip = ""
        upload_url = ""
        try:
            self.status_callback(f"正在打开 {port}...")
            connection = self._open_serial(port)
            try:
                self.status_callback("正在请求 ESP32 进入 Wi-Fi 配置...")
                self._write_line(connection, "wifi set")
                self._wait_for_text(
                    connection,
                    success_markers=("Enter SSID",),
                    failure_markers=("unknown command",),
                    timeout_seconds=SERIAL_PROMPT_TIMEOUT_SECONDS,
                )

                self._write_line(connection, ssid)
                self._wait_for_text(
                    connection,
                    success_markers=("Enter password",),
                    failure_markers=("Invalid SSID",),
                    timeout_seconds=SERIAL_PROMPT_TIMEOUT_SECONDS,
                )

                self._write_line(connection, password)
                self.status_callback(f"ESP32 正在连接 {ssid}，连接及必要恢复最长约 70 秒...")
                wifi_output = self._wait_for_wifi_result(
                    connection,
                    timeout_seconds=WIFI_CONNECT_TIMEOUT_SECONDS,
                )
                device_ip = self._extract_device_ip(wifi_output)
                computer_ip = local_ip_for_peer(device_ip)
                upload_url = f"http://{computer_ip}:{upload_port}/upload"

                self.status_callback(f"ESP32 新地址为 {device_ip}，正在同步上传地址...")
                self._write_line(connection, "server set")
                self._wait_for_text(
                    connection,
                    success_markers=("Enter upload URL",),
                    failure_markers=("unknown command",),
                    timeout_seconds=SERIAL_PROMPT_TIMEOUT_SECONDS,
                )
                self._write_line(connection, upload_url)
                self._wait_for_text(
                    connection,
                    success_markers=("Upload URL saved successfully",),
                    failure_markers=("Invalid URL", "Upload URL save failed"),
                    timeout_seconds=SERIAL_PROMPT_TIMEOUT_SECONDS,
                )
            finally:
                self._release_serial(connection, port)
        except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
            return WifiConfigurationResult(
                success=False,
                message=str(exc),
                device_ip=device_ip,
                computer_ip=computer_ip,
                upload_url=upload_url,
            )

        return WifiConfigurationResult(
            success=True,
            message="Wi-Fi、设备 IP 和上传地址已同步完成",
            device_ip=device_ip,
            computer_ip=computer_ip,
            upload_url=upload_url,
        )

    def _open_serial(self, port: str) -> serial.Serial:
        """打开串口并避免主动拉低 ESP32 的复位控制线。

        Args:
            port: 需要打开的 Windows 串口名称。

        Returns:
            已打开并清空输入缓存的 pyserial 连接。

        Raises:
            serial.SerialException: 串口不存在、被占用或无法打开。
        """

        connection = serial.Serial()
        connection.port = port
        connection.baudrate = SERIAL_BAUD_RATE
        connection.timeout = 0.2
        connection.write_timeout = 2.0
        connection.dtr = False
        connection.rts = False
        connection.open()
        connection.dtr = False
        connection.rts = False
        time.sleep(1.5)
        connection.reset_input_buffer()
        return connection

    def _release_serial(self, connection: serial.Serial, port: str) -> None:
        """显式关闭 Wi-Fi 配置串口并清除端口引用。

        Args:
            connection: 已打开或正在关闭的 pyserial 串口连接。
            port: 用于日志显示的 Windows 串口名称，例如 ``COM61``。

        Returns:
            None. 关闭异常会写入状态日志，不会覆盖原配置结果。
        """

        try:
            connection.close()
            connection.port = None
        except (OSError, ValueError, serial.SerialException) as exc:
            self.status_callback(f"释放串口 {port} 时发生异常：{exc}")
            return
        self.status_callback(f"串口 {port} 已释放")

    def _write_line(self, connection: serial.Serial, value: str) -> None:
        """向 ESP32 发送一行 UTF-8 文本。

        Args:
            connection: 已打开的串口连接。
            value: 不包含行尾的命令或字段内容。

        Returns:
            None.
        """

        connection.write((value + "\r\n").encode("utf-8"))
        connection.flush()

    def _wait_for_text(
        self,
        connection: serial.Serial,
        *,
        success_markers: tuple[str, ...],
        failure_markers: tuple[str, ...],
        timeout_seconds: float,
    ) -> str:
        """读取混合日志，直到出现预期成功或失败标记。

        Args:
            connection: 已打开的 ESP32 串口。
            success_markers: 任意一个出现即可判定本阶段成功的文本。
            failure_markers: 任意一个出现即判定本阶段失败的文本。
            timeout_seconds: 最长等待秒数。

        Returns:
            从阶段开始到成功标记出现为止的串口文本。

        Raises:
            RuntimeError: 收到失败标记或等待超时。
        """

        deadline = time.monotonic() + timeout_seconds
        received_text = ""
        while time.monotonic() < deadline:
            waiting = connection.in_waiting
            chunk = connection.read(waiting if waiting > 0 else 1)
            if not chunk:
                continue
            received_text += chunk.decode("utf-8", errors="replace")
            if len(received_text) > 32_768:
                received_text = received_text[-16_384:]
            for marker in failure_markers:
                if marker in received_text:
                    raise RuntimeError(f"ESP32 返回失败状态：{marker}")
            for marker in success_markers:
                if marker in received_text:
                    return received_text
        expected = " / ".join(success_markers)
        raise RuntimeError(f"等待 ESP32 回复超时，未收到：{expected}")

    def _wait_for_wifi_result(self, connection: serial.Serial, *, timeout_seconds: float) -> str:
        """等待新 Wi-Fi 连接成功，或等待失败后的旧网络恢复完成。

        Args:
            connection: 已打开的 ESP32 串口。
            timeout_seconds: 新网络连接和必要恢复流程的最长总等待秒数。

        Returns:
            从提交密码到新网络保存成功为止的串口文本。

        Raises:
            RuntimeError: 新网络连接失败、保存失败、旧网络恢复失败或等待超时。
        """

        deadline = time.monotonic() + timeout_seconds
        received_text = ""
        connection_failed = False
        while time.monotonic() < deadline:
            waiting = connection.in_waiting
            chunk = connection.read(waiting if waiting > 0 else 1)
            if not chunk:
                continue
            received_text += chunk.decode("utf-8", errors="replace")
            if len(received_text) > 32_768:
                received_text = received_text[-16_384:]

            if "Connected, but saving failed" in received_text:
                raise RuntimeError("ESP32 已连接目标 Wi-Fi，但保存配置失败")
            if "Connected and saved successfully" in received_text:
                return received_text
            if "Connection failed" in received_text:
                connection_failed = True
            if connection_failed and "Previous Wi-Fi restored" in received_text:
                raise RuntimeError("目标 Wi-Fi 连接失败，ESP32 已恢复原网络")
            if connection_failed and "Restore failed" in received_text:
                raise RuntimeError("目标 Wi-Fi 连接失败，且 ESP32 未能恢复原网络")

        if connection_failed:
            raise RuntimeError("目标 Wi-Fi 连接失败，等待 ESP32 恢复原网络超时")
        raise RuntimeError("等待 ESP32 连接目标 Wi-Fi 超时")

    def _extract_device_ip(self, serial_output: str) -> str:
        """从 ESP32 连接成功日志中提取新 IPv4 地址。

        Args:
            serial_output: Wi-Fi 配置阶段收集到的串口文本。

        Returns:
            ESP32 获得的有效非回环 IPv4 地址。

        Raises:
            RuntimeError: 串口输出中没有有效设备地址。
        """

        matches = DEVICE_IP_PATTERN.findall(serial_output)
        for candidate in reversed(matches):
            if is_valid_ipv4(candidate):
                return candidate
        raise RuntimeError("ESP32 已连接，但串口回复中没有找到设备 IP")


class FirmwareFlasher:
    """在后台线程中调用 esptool 烧录 ESP32-CAM 应用固件。"""

    def __init__(self, event_queue: queue.Queue[tuple[str, object]]) -> None:
        """初始化烧录线程状态和 GUI 事件队列。

        Args:
            event_queue: 接收烧录日志和最终结果的线程安全 GUI 队列。

        Returns:
            None.
        """

        self.event_queue = event_queue
        self.state_lock = threading.Lock()
        self.thread: threading.Thread | None = None

    def start(self, *, port: str, firmware_path: Path) -> bool:
        """启动应用固件串口烧录线程。

        Args:
            port: ESP32-CAM 当前使用的串口名称，例如 ``COM61``。
            firmware_path: 需要写入 ``0x10000`` 的应用固件 bin 文件。

        Returns:
            True 表示线程已启动，False 表示已有烧录任务尚未结束。
        """

        with self.state_lock:
            if self.thread is not None and self.thread.is_alive():
                return False
            thread = threading.Thread(
                target=self._run,
                args=(port, firmware_path),
                name="firmware-flasher",
                daemon=True,
            )
            self.thread = thread
        thread.start()
        return True

    def is_running(self) -> bool:
        """查询后台烧录线程是否仍在运行。

        Args:
            None.

        Returns:
            True 表示正在连接、写入或校验固件。
        """

        with self.state_lock:
            return self.thread is not None and self.thread.is_alive()

    def _post_log(self, message: str) -> None:
        """把一行 esptool 输出投递给 GUI 日志队列。

        Args:
            message: 已移除换行符的 esptool 输出文本。

        Returns:
            None.
        """

        self.event_queue.put(("log", f"烧录：{message}"))

    def _run(self, port: str, firmware_path: Path) -> None:
        """加载 esptool 并执行 ESP32 应用固件写入命令。

        Args:
            port: ESP32-CAM 当前使用的串口名称。
            firmware_path: 需要写入 ``0x10000`` 的应用固件 bin 文件。

        Returns:
            None.
        """

        result = FirmwareFlashResult(
            success=False,
            firmware_path=firmware_path,
            message="固件烧录失败",
        )
        log_writer = EsptoolLogWriter(self._post_log)
        arguments = [
            "--chip",
            "esp32",
            "--port",
            port,
            "--baud",
            str(FIRMWARE_FLASH_BAUD_RATE),
            "--before",
            "default_reset",
            "--after",
            "hard_reset",
            "write_flash",
            "--flash_mode",
            "dio",
            "--flash_freq",
            "40m",
            "--flash_size",
            "detect",
            FIRMWARE_FLASH_ADDRESS,
            str(firmware_path),
        ]
        try:
            esptool_module = load_esptool_module()
            with contextlib.redirect_stdout(log_writer), contextlib.redirect_stderr(log_writer):
                esptool_module.main(arguments)
            log_writer.flush()
            result = FirmwareFlashResult(
                success=True,
                firmware_path=firmware_path,
                message="固件烧录、校验和复位已完成",
            )
        except SystemExit as exc:
            log_writer.flush()
            exit_code = exc.code if isinstance(exc.code, int) else 1
            if exit_code == 0:
                result = FirmwareFlashResult(
                    success=True,
                    firmware_path=firmware_path,
                    message="固件烧录、校验和复位已完成",
                )
            else:
                result = FirmwareFlashResult(
                    success=False,
                    firmware_path=firmware_path,
                    message=f"esptool 退出，错误代码：{exit_code}",
                )
        except (OSError, RuntimeError, ValueError) as exc:
            log_writer.flush()
            result = FirmwareFlashResult(
                success=False,
                firmware_path=firmware_path,
                message=str(exc) or exc.__class__.__name__,
            )
        except Exception as exc:
            log_writer.flush()
            result = FirmwareFlashResult(
                success=False,
                firmware_path=firmware_path,
                message=f"esptool 运行失败：{exc}",
            )
        finally:
            with self.state_lock:
                if self.thread is threading.current_thread():
                    self.thread = None
            self.event_queue.put(("firmware_flash_result", result))


class MjpegStreamWorker:
    """在后台读取 ESP32 MJPEG 流并向 GUI 事件队列投递最新帧。"""

    def __init__(
        self,
        event_queue: queue.Queue[tuple[str, object]],
        frame_queue: queue.Queue[tuple[int, Image.Image]],
    ) -> None:
        """初始化视频工作器。

        Args:
            event_queue: 接收视频连接状态的线程安全队列。
            frame_queue: 仅保留最新视频帧的有限队列。

        Returns:
            None.
        """

        self.event_queue = event_queue
        self.frame_queue = frame_queue
        self.stop_event: threading.Event | None = None
        self.generation = 0
        self.state_lock = threading.Lock()
        self.active_response: object | None = None
        self.thread: threading.Thread | None = None

    def start(self, stream_url: str) -> None:
        """停止旧连接并启动指定地址的视频读取线程。

        Args:
            stream_url: ESP32 MJPEG 完整地址。

        Returns:
            None.
        """

        if not self.stop():
            self.event_queue.put(
                (
                    "stream_status",
                    (self.generation, False, "旧视频连接尚未结束，请稍后重试"),
                )
            )
            return
        self.generation += 1
        stop_event = threading.Event()
        self.stop_event = stop_event
        thread = threading.Thread(
            target=self._run,
            args=(stream_url, self.generation, stop_event),
            name="mjpeg-stream",
            daemon=True,
        )
        with self.state_lock:
            self.thread = thread
        thread.start()

    def stop(self) -> bool:
        """关闭当前 HTTP 响应并等待视频线程停止读取。

        Args:
            None.

        Returns:
            True 表示旧连接已经结束，False 表示线程仍未退出。
        """

        stop_event = self.stop_event
        if stop_event is not None:
            stop_event.set()

        with self.state_lock:
            response = self.active_response
            thread = self.thread
        close_response = getattr(response, "close", None)
        if callable(close_response):
            try:
                close_response()
            except OSError:
                pass

        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        stopped = thread is None or not thread.is_alive()
        if stopped:
            with self.state_lock:
                if self.thread is thread:
                    self.thread = None
                if self.active_response is response:
                    self.active_response = None
            self.stop_event = None
        return stopped

    def _run(self, stream_url: str, generation: int, stop_event: threading.Event) -> None:
        """持续连接 MJPEG 地址、解析 JPEG 帧并在断开后重试。

        Args:
            stream_url: ESP32 MJPEG 完整地址。
            generation: 本次连接代次，用于丢弃旧连接事件。
            stop_event: 本次线程独立的停止事件。

        Returns:
            None.
        """

        last_error = ""
        while not stop_event.is_set():
            try:
                request = urllib.request.Request(stream_url, headers={"User-Agent": "ESP32-Desktop/1.0"})
                with urllib.request.urlopen(request, timeout=5) as response:
                    with self.state_lock:
                        self.active_response = response
                    try:
                        self.event_queue.put(("stream_status", (generation, True, "视频已连接")))
                        buffer = bytearray()
                        while not stop_event.is_set():
                            chunk = response.read(4096)
                            if not chunk:
                                raise OSError("视频流已断开")
                            buffer.extend(chunk)
                            self._emit_complete_frames(buffer, generation, stop_event)
                    finally:
                        with self.state_lock:
                            if self.active_response is response:
                                self.active_response = None
            except (OSError, ValueError, urllib.error.URLError) as exc:
                error_text = str(exc) or exc.__class__.__name__
                if "timed out" in error_text.lower():
                    error_text = "连接超时，请关闭浏览器或其他正在占用 ESP32 视频流的程序"
                if error_text != last_error:
                    self.event_queue.put(
                        ("stream_status", (generation, False, f"等待视频：{error_text}"))
                    )
                    last_error = error_text
                stop_event.wait(1.0)

        with self.state_lock:
            if self.thread is threading.current_thread():
                self.thread = None

    def _emit_complete_frames(
        self,
        buffer: bytearray,
        generation: int,
        stop_event: threading.Event,
    ) -> None:
        """从累计字节中提取所有完整 JPEG，并仅保留最新可显示帧。

        Args:
            buffer: MJPEG 响应累计缓冲区；已解析内容会被原地删除。
            generation: 当前视频连接代次。
            stop_event: 当前连接停止事件。

        Returns:
            None.
        """

        while not stop_event.is_set():
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                if len(buffer) > 1_000_000:
                    del buffer[:-2]
                return
            end = buffer.find(b"\xff\xd9", start + 2)
            if end < 0:
                if start > 0:
                    del buffer[:start]
                return
            jpeg_data = bytes(buffer[start : end + 2])
            del buffer[: end + 2]
            try:
                with Image.open(io.BytesIO(jpeg_data)) as decoded:
                    frame = decoded.convert("RGB")
            except (OSError, ValueError):
                continue
            self._replace_latest_frame(generation, frame)

    def _replace_latest_frame(self, generation: int, frame: Image.Image) -> None:
        """向事件队列写入最新帧，并在队列拥堵时丢弃旧帧。

        Args:
            generation: 当前视频连接代次。
            frame: 已解码的 RGB 图像。

        Returns:
            None.
        """

        try:
            self.frame_queue.put_nowait((generation, frame))
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frame_queue.put_nowait((generation, frame))
            except queue.Full:
                pass


class VideoRecorder:
    """在独立线程中将上位机接收到的 RGB 帧写入 MP4 文件。"""

    def __init__(self, event_queue: queue.Queue[tuple[str, object]]) -> None:
        """初始化录像状态和结果事件队列。

        Args:
            event_queue: 接收 ``recording_result`` 结果的线程安全 GUI 队列。

        Returns:
            None.
        """

        self.event_queue = event_queue
        self.state_lock = threading.Lock()
        self.frame_queue: queue.Queue[Image.Image | None] | None = None
        self.thread: threading.Thread | None = None
        self.accepting_frames = False
        self.started_at = 0.0
        self.dropped_frames = 0

    def start(
        self,
        *,
        output_path: Path,
        frame_size: tuple[int, int],
        fps: float,
    ) -> tuple[bool, str]:
        """创建 MP4 写入器并启动后台写入线程。

        Args:
            output_path: 用户选择的 MP4 文件路径。
            frame_size: ``(宽度, 高度)`` 固定输出尺寸。
            fps: MP4 文件使用的固定帧率。

        Returns:
            ``(是否启动成功, 失败原因)``；成功时失败原因为空字符串。
        """

        width, height = frame_size
        if width <= 0 or height <= 0 or fps <= 0:
            return False, "视频尺寸或帧率无效"

        with self.state_lock:
            if self.thread is not None and self.thread.is_alive():
                return False, "上一段视频仍在保存，请稍后重试"

        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return False, f"无法创建保存目录：{exc}"

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
        if not writer.isOpened():
            writer.release()
            return False, "无法创建 MP4 文件，请选择其他保存位置"

        frame_queue: queue.Queue[Image.Image | None] = queue.Queue(
            maxsize=RECORDING_QUEUE_SIZE
        )
        started_at = time.monotonic()
        thread = threading.Thread(
            target=self._run,
            args=(writer, frame_queue, output_path, frame_size, started_at),
            name="mp4-recorder",
            daemon=True,
        )
        with self.state_lock:
            self.frame_queue = frame_queue
            self.thread = thread
            self.accepting_frames = True
            self.started_at = started_at
            self.dropped_frames = 0
        thread.start()
        return True, ""

    def submit(self, frame: Image.Image) -> bool:
        """把一帧图像加入录像队列，队列满时用新帧替换最旧帧。

        Args:
            frame: 当前 ESP32 视频流解码得到的 RGB 图像。

        Returns:
            True 表示帧已进入队列，False 表示当前没有录制或帧未能入队。
        """

        with self.state_lock:
            if not self.accepting_frames or self.frame_queue is None:
                return False
            frame_queue = self.frame_queue

        try:
            frame_queue.put_nowait(frame)
            return True
        except queue.Full:
            try:
                frame_queue.get_nowait()
            except queue.Empty:
                return False
            with self.state_lock:
                self.dropped_frames += 1
            try:
                frame_queue.put_nowait(frame)
                return True
            except queue.Full:
                return False

    def stop(self, *, wait: bool = False) -> bool:
        """停止接收新帧并通知后台线程完成 MP4 封装。

        Args:
            wait: True 时等待后台写入线程结束，适用于程序退出。

        Returns:
            True 表示存在正在录制或保存的任务，False 表示没有录像任务。
        """

        with self.state_lock:
            frame_queue = self.frame_queue
            thread = self.thread
            was_active = self.accepting_frames
            self.accepting_frames = False

        if frame_queue is not None and was_active:
            while True:
                try:
                    frame_queue.put_nowait(None)
                    break
                except queue.Full:
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        continue
                    with self.state_lock:
                        self.dropped_frames += 1

        if wait and thread is not None and thread is not threading.current_thread():
            thread.join(timeout=10.0)
        return was_active or (thread is not None and thread.is_alive())

    def is_recording(self) -> bool:
        """查询录像器当前是否仍接收视频帧。

        Args:
            None.

        Returns:
            True 表示正在录制，False 表示未录制或正在完成文件封装。
        """

        with self.state_lock:
            return self.accepting_frames

    def elapsed_seconds(self) -> float:
        """计算当前录像已经持续的墙钟时间。

        Args:
            None.

        Returns:
            正在录制时返回经过秒数，否则返回 0。
        """

        with self.state_lock:
            if not self.accepting_frames:
                return 0.0
            started_at = self.started_at
        return max(0.0, time.monotonic() - started_at)

    def _run(
        self,
        writer: cv2.VideoWriter,
        frame_queue: queue.Queue[Image.Image | None],
        output_path: Path,
        frame_size: tuple[int, int],
        started_at: float,
    ) -> None:
        """消费录像队列，将 RGB 帧转换后顺序写入 MP4 文件。

        Args:
            writer: 已成功打开的 OpenCV MP4 写入器。
            frame_queue: 本次录像专用的图像帧队列。
            output_path: 本次录像的最终文件路径。
            frame_size: ``(宽度, 高度)`` 固定输出尺寸。
            started_at: 本次录像的 monotonic 起始时间。

        Returns:
            None.
        """

        frame_count = 0
        error = ""
        try:
            while True:
                frame = frame_queue.get()
                if frame is None:
                    break
                rgb_frame = np.asarray(frame, dtype=np.uint8)
                bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                if (bgr_frame.shape[1], bgr_frame.shape[0]) != frame_size:
                    bgr_frame = cv2.resize(bgr_frame, frame_size, interpolation=cv2.INTER_AREA)
                writer.write(bgr_frame)
                frame_count += 1
        except (OSError, ValueError, cv2.error) as exc:
            error = f"写入 MP4 失败：{exc}"
        finally:
            writer.release()
            elapsed_seconds = max(0.0, time.monotonic() - started_at)
            with self.state_lock:
                dropped_frames = self.dropped_frames
                if self.frame_queue is frame_queue:
                    self.frame_queue = None
                if self.thread is threading.current_thread():
                    self.thread = None
                self.accepting_frames = False
            if frame_count == 0 and not error:
                error = "录制期间没有收到可写入的视频帧"
            result = RecordingResult(
                output_path=output_path,
                frame_count=frame_count,
                elapsed_seconds=elapsed_seconds,
                dropped_frames=dropped_frames,
                error=error,
            )
            self.event_queue.put(("recording_result", result))


class CameraDesktopApp:
    """ESP32-CAM Windows 桌面上位机主窗口。"""

    def __init__(self, root: tk.Tk) -> None:
        """创建界面、上传服务和视频工作器。

        Args:
            root: Tk 主窗口对象。

        Returns:
            None.
        """

        self.root = root
        self.root.title("ESP-CAM 图传")
        self.root.geometry("1280x860")
        self.root.minsize(1000, 720)
        self.root.configure(background="#eef1f4")

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.frames: queue.Queue[tuple[int, Image.Image]] = queue.Queue(maxsize=1)
        self.stream_worker = MjpegStreamWorker(self.events, self.frames)
        self.video_recorder = VideoRecorder(self.events)
        self.firmware_flasher = FirmwareFlasher(self.events)
        self.upload_server = EmbeddedUploadServer()
        self.upload_port = 0
        self.current_frame: Image.Image | None = None
        self.current_photo: ImageTk.PhotoImage | None = None
        self.header_logo_photo: ImageTk.PhotoImage | None = None
        self.configuring = False
        self.flashing = False
        self.scanning_wifi = False
        self.video_connected = False
        self.closing = False

        self.port_var = tk.StringVar()
        self.ssid_var = tk.StringVar()
        self.password_var = tk.StringVar()
        self.show_password_var = tk.BooleanVar(value=False)
        self.open_network_var = tk.BooleanVar(value=False)
        self.device_ip_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="未连接视频")
        self.recording_var = tk.StringVar(value="未录制")
        self.server_var = tk.StringVar(value="正在启动图片接收服务...")

        self._configure_styles()
        self._build_layout()
        self._load_initial_values()
        self.refresh_serial_ports()
        self._start_upload_server()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(40, self._process_events)
        self.root.after(250, self.scan_wifi_networks)

    def _configure_styles(self) -> None:
        """配置桌面界面使用的 ttk 颜色和尺寸。

        Args:
            None.

        Returns:
            None.
        """

        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Header.TLabel", font=("Microsoft YaHei UI", 20, "bold"), foreground="#18212b")
        style.configure("Subtle.TLabel", font=("Microsoft YaHei UI", 9), foreground="#66717d")
        style.configure("Section.TLabel", font=("Microsoft YaHei UI", 11, "bold"), foreground="#18212b")
        style.configure("Status.TLabel", font=("Microsoft YaHei UI", 10), foreground="#166534")
        style.configure("Primary.TButton", font=("Microsoft YaHei UI", 10, "bold"), padding=(14, 9))
        style.configure("TButton", font=("Microsoft YaHei UI", 9), padding=(10, 7))
        style.configure("TLabel", font=("Microsoft YaHei UI", 9))
        style.configure("TEntry", padding=7)
        style.configure("TCombobox", padding=6)

    def _build_layout(self) -> None:
        """创建视频区、网络配置区和运行日志区。

        Args:
            None.

        Returns:
            None.
        """

        header = tk.Frame(self.root, background="#ffffff", height=102)
        header.grid(row=0, column=0, columnspan=2, sticky="nsew")
        header.grid_propagate(False)
        for column in range(3):
            header.grid_columnconfigure(column, weight=1, uniform="header")
        header.grid_rowconfigure(0, weight=1)
        self.header_logo_photo = self._load_header_logo(target_height=80)
        if self.header_logo_photo is not None:
            tk.Label(
                header,
                image=self.header_logo_photo,
                background="#ffffff",
                borderwidth=0,
                highlightthickness=0,
            ).grid(row=0, column=0, sticky="w", padx=18, pady=(22, 0))
        ttk.Label(header, text="ESP-CAM 图传", style="Header.TLabel", background="#ffffff").grid(
            row=0, column=1
        )
        ttk.Label(header, textvariable=self.connection_var, style="Status.TLabel", background="#ffffff").grid(
            row=0, column=2, sticky="e", padx=24
        )

        video_area = tk.Frame(self.root, background="#111416")
        video_area.grid(row=1, column=0, sticky="nsew", padx=(18, 8), pady=18)
        video_area.grid_rowconfigure(0, weight=1)
        video_area.grid_columnconfigure(0, weight=1)
        self.video_label = tk.Label(
            video_area,
            text="等待 ESP32 视频",
            foreground="#d8dde3",
            background="#111416",
            font=("Microsoft YaHei UI", 13),
        )
        self.video_label.grid(row=0, column=0, sticky="nsew")
        self.video_label.bind("<Configure>", self._on_video_resize)

        controls = tk.Frame(self.root, background="#ffffff", width=350)
        controls.grid(
            row=1,
            column=1,
            rowspan=2,
            sticky="nsew",
            padx=(8, 18),
            pady=(18, 18),
        )
        controls.grid_propagate(False)
        controls.grid_columnconfigure(0, weight=1)

        ttk.Label(controls, text="设备连接", style="Section.TLabel", background="#ffffff").grid(
            row=0, column=0, sticky="w", padx=20, pady=(20, 8)
        )
        ttk.Label(controls, text="设备 IP", background="#ffffff").grid(
            row=1, column=0, sticky="w", padx=20, pady=(5, 3)
        )
        self.device_ip_entry = ttk.Entry(controls, textvariable=self.device_ip_var)
        self.device_ip_entry.grid(row=2, column=0, sticky="ew", padx=20)
        video_actions = tk.Frame(controls, background="#ffffff")
        video_actions.grid(row=3, column=0, sticky="ew", padx=20, pady=(8, 18))
        video_actions.grid_columnconfigure(0, weight=1)
        video_actions.grid_columnconfigure(1, weight=1)
        self.connect_button = ttk.Button(video_actions, text="连接视频", command=self.connect_video)
        self.connect_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.record_button = ttk.Button(
            video_actions,
            text="开始录制",
            command=self.toggle_recording,
            state="disabled",
        )
        self.record_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        ttk.Label(
            video_actions,
            textvariable=self.recording_var,
            style="Subtle.TLabel",
            background="#ffffff",
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

        ttk.Separator(controls).grid(row=4, column=0, sticky="ew", padx=20)
        ttk.Label(controls, text="更换 ESP32 Wi-Fi", style="Section.TLabel", background="#ffffff").grid(
            row=5, column=0, sticky="w", padx=20, pady=(18, 8)
        )
        ttk.Label(
            controls,
            text="操作前请先让电脑连接下方目标 Wi-Fi。",
            style="Subtle.TLabel",
            background="#ffffff",
            wraplength=300,
        ).grid(row=6, column=0, sticky="w", padx=20, pady=(0, 8))

        port_row = tk.Frame(controls, background="#ffffff")
        port_row.grid(row=7, column=0, sticky="ew", padx=20)
        port_row.grid_columnconfigure(0, weight=1)
        self.port_combo = ttk.Combobox(port_row, textvariable=self.port_var, state="normal")
        self.port_combo.grid(row=0, column=0, sticky="ew")
        ttk.Button(port_row, text="刷新", command=self.refresh_serial_ports).grid(row=0, column=1, padx=(8, 0))

        wifi_label_row = tk.Frame(controls, background="#ffffff")
        wifi_label_row.grid(row=8, column=0, sticky="ew", padx=20, pady=(12, 3))
        wifi_label_row.grid_columnconfigure(0, weight=1)
        ttk.Label(wifi_label_row, text="Wi-Fi 名称", background="#ffffff").grid(
            row=0, column=0, sticky="w"
        )
        self.scan_wifi_button = ttk.Button(
            wifi_label_row,
            text="扫描网络",
            command=self.scan_wifi_networks,
        )
        self.scan_wifi_button.grid(row=0, column=1, sticky="e")
        self.ssid_combo = ttk.Combobox(controls, textvariable=self.ssid_var, state="normal")
        self.ssid_combo.grid(row=9, column=0, sticky="ew", padx=20)

        ttk.Label(controls, text="Wi-Fi 密码", background="#ffffff").grid(
            row=10, column=0, sticky="w", padx=20, pady=(12, 3)
        )
        self.password_entry = ttk.Entry(controls, textvariable=self.password_var, show="*")
        self.password_entry.grid(row=11, column=0, sticky="ew", padx=20)
        password_options = tk.Frame(controls, background="#ffffff")
        password_options.grid(row=12, column=0, sticky="ew", padx=20, pady=(5, 8))
        ttk.Checkbutton(
            password_options,
            text="显示密码",
            variable=self.show_password_var,
            command=self._toggle_password_visibility,
        ).pack(side="left")
        ttk.Checkbutton(
            password_options,
            text="开放网络",
            variable=self.open_network_var,
            command=self._toggle_open_network,
        ).pack(side="right")

        self.configure_button = ttk.Button(
            controls,
            text="更换 Wi-Fi 并同步地址",
            style="Primary.TButton",
            command=self.configure_wifi,
        )
        self.configure_button.grid(row=13, column=0, sticky="ew", padx=20, pady=(4, 14))

        self.flash_button = ttk.Button(
            controls,
            text="选择 BIN 并烧录固件",
            command=self.flash_firmware,
        )
        self.flash_button.grid(row=14, column=0, sticky="ew", padx=20, pady=(0, 12))

        ttk.Label(controls, textvariable=self.server_var, style="Subtle.TLabel", background="#ffffff", wraplength=300).grid(
            row=15, column=0, sticky="w", padx=20, pady=(0, 8)
        )
        log_panel = tk.Frame(self.root, background="#ffffff", height=210)
        log_panel.grid(row=2, column=0, sticky="nsew", padx=(18, 8), pady=(0, 18))
        log_panel.grid_propagate(False)
        log_panel.grid_columnconfigure(0, weight=1)
        log_panel.grid_rowconfigure(1, weight=1)
        ttk.Label(
            log_panel,
            text="运行日志",
            style="Section.TLabel",
            background="#ffffff",
        ).grid(row=0, column=0, sticky="w", padx=14, pady=(10, 6))
        log_scrollbar = ttk.Scrollbar(log_panel, orient="vertical")
        log_scrollbar.grid(row=1, column=1, sticky="ns", padx=(0, 10), pady=(0, 10))
        self.log_text = tk.Text(
            log_panel,
            height=10,
            wrap="word",
            state="disabled",
            relief="solid",
            borderwidth=1,
            background="#f7f8fa",
            foreground="#303944",
            font=("Consolas", 9),
            yscrollcommand=log_scrollbar.set,
        )
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=(14, 6), pady=(0, 10))
        log_scrollbar.configure(command=self.log_text.yview)

        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, minsize=210)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, minsize=350)

    def _load_header_logo(self, *, target_height: int) -> ImageTk.PhotoImage | None:
        """读取、裁剪透明边缘并缩放标题栏 Logo。

        Args:
            target_height: 标题栏中 Logo 的目标显示高度，单位为像素。

        Returns:
            可由 Tkinter 标签持有的 Logo 图像；资源无效时返回 None。
        """

        if target_height <= 0:
            return None
        try:
            with Image.open(COMPANY_LOGO_PATH) as source_logo:
                logo = source_logo.convert("RGBA")
        except (OSError, ValueError):
            return None

        visible_bounds = logo.getbbox()
        if visible_bounds is None:
            return None
        logo = logo.crop(visible_bounds)
        target_width = max(1, round(logo.width * target_height / logo.height))
        resized_logo = logo.resize((target_width, target_height), Image.Resampling.LANCZOS)
        return ImageTk.PhotoImage(resized_logo)

    def _load_initial_values(self) -> None:
        """恢复最近使用的非敏感配置和历史设备地址。

        Args:
            None.

        Returns:
            None.
        """

        config = load_desktop_config()
        self.port_var.set(config.get("port", "COM61"))
        self.ssid_var.set(config.get("ssid", ""))
        device_ip = config.get("device_ip", "") or read_last_device_ip()
        if is_valid_ipv4(device_ip):
            self.device_ip_var.set(device_ip)

    def _start_upload_server(self) -> None:
        """启动内置图片接收服务并更新界面状态。

        Args:
            None.

        Returns:
            None.
        """

        try:
            self.upload_port = self.upload_server.start()
        except OSError as exc:
            self.server_var.set(f"图片接收服务启动失败：{exc}")
            self.append_log(f"图片接收服务启动失败：{exc}")
            return
        self.server_var.set(f"图片接收服务：0.0.0.0:{self.upload_port}")
        self.append_log(f"图片接收服务已启动，端口 {self.upload_port}")
        if self.device_ip_var.get():
            self.connect_video()

    def refresh_serial_ports(self) -> None:
        """刷新串口下拉列表，并保留用户手工填写的端口。

        Args:
            None.

        Returns:
            None.
        """

        available = list_serial_ports()
        values = [f"{device} - {description}" for device, description in available]
        current = self._selected_port()
        if current and all(not value.startswith(current + " ") for value in values):
            values.insert(0, current)
        self.port_combo.configure(values=values)
        if not self.port_var.get().strip() and values:
            self.port_var.set(values[0])
        self.append_log(f"检测到 {len(available)} 个串口")

    def scan_wifi_networks(self) -> None:
        """在后台启动 Windows Wi-Fi 扫描并暂时锁定扫描按钮。

        Args:
            None.

        Returns:
            None.
        """

        if self.scanning_wifi:
            return
        self.scanning_wifi = True
        self.scan_wifi_button.configure(state="disabled", text="扫描中...")
        self.append_log("正在扫描附近 Wi-Fi...")
        thread = threading.Thread(
            target=self._scan_wifi_worker,
            name="wifi-scanner",
            daemon=True,
        )
        thread.start()

    def _scan_wifi_worker(self) -> None:
        """执行阻塞的 Windows Wi-Fi 扫描并把结果投递给主线程。

        Args:
            None.

        Returns:
            None.
        """

        networks: list[str] = []
        error = ""
        try:
            networks = scan_available_wifi_ssids()
        except RuntimeError as exc:
            error = str(exc)
        self.events.put(("wifi_scan_result", (networks, error)))

    def _selected_port(self) -> str:
        """从串口下拉框显示文本中提取纯端口名。

        Args:
            None.

        Returns:
            用户选择或输入的 Windows 串口名。
        """

        return self.port_var.get().split(" - ", 1)[0].strip()

    def _toggle_password_visibility(self) -> None:
        """根据复选框切换密码输入框的遮罩状态。

        Args:
            None.

        Returns:
            None.
        """

        self.password_entry.configure(show="" if self.show_password_var.get() else "*")

    def _toggle_open_network(self) -> None:
        """根据开放网络选项启用或禁用密码输入。

        Args:
            None.

        Returns:
            None.
        """

        if self.open_network_var.get():
            self.password_var.set("")
            self.password_entry.configure(state="disabled")
            return
        self.password_entry.configure(state="normal")
        self._toggle_password_visibility()

    def connect_video(self) -> None:
        """使用设备 IP 启动或重启桌面 MJPEG 视频连接。

        Args:
            None.

        Returns:
            None.
        """

        if self.configuring:
            self.append_log("Wi-Fi 配置进行中，暂不连接视频")
            return
        if self.flashing:
            self.append_log("固件烧录进行中，暂不连接视频")
            return

        if self.video_recorder.is_recording():
            self._stop_recording()

        device_ip = self.device_ip_var.get().strip()
        if not is_valid_ipv4(device_ip):
            messagebox.showerror("设备 IP 无效", "请输入 ESP32 的有效 IPv4 地址。")
            return
        stream_url = f"http://{device_ip}:{STREAM_PORT}{STREAM_PATH}"
        self.video_connected = False
        self.record_button.configure(state="disabled")
        self.connection_var.set(f"正在连接 {device_ip}")
        self.append_log(f"连接视频：{stream_url}")
        self.stream_worker.start(stream_url)
        save_desktop_config(
            port=self._selected_port(),
            ssid=self.ssid_var.get().strip(),
            device_ip=device_ip,
        )

    def toggle_recording(self) -> None:
        """根据当前状态开始或停止 MP4 视频录制。

        Args:
            None.

        Returns:
            None.
        """

        if self.video_recorder.is_recording():
            self._stop_recording()
            return
        self._start_recording()

    def _start_recording(self) -> None:
        """选择 MP4 保存位置并使用当前视频尺寸启动录像器。

        Args:
            None.

        Returns:
            None.
        """

        if not self.video_connected or self.current_frame is None:
            messagebox.showerror("无法录制", "请先等待 ESP32 视频连接并显示画面。")
            return

        recording_directory = BASE_DIR / "recorded_videos"
        try:
            recording_directory.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("无法保存", f"无法创建录像目录：{exc}")
            return

        default_name = time.strftime("ESP32_CAM_%Y%m%d_%H%M%S.mp4")
        selected_path = filedialog.asksaveasfilename(
            parent=self.root,
            title="保存录制视频",
            initialdir=str(recording_directory),
            initialfile=default_name,
            defaultextension=".mp4",
            filetypes=(("MP4 视频", "*.mp4"),),
            confirmoverwrite=True,
        )
        if not selected_path:
            return

        output_path = Path(selected_path)
        if output_path.suffix.lower() != ".mp4":
            output_path = output_path.with_suffix(".mp4")
        started, error = self.video_recorder.start(
            output_path=output_path,
            frame_size=self.current_frame.size,
            fps=RECORDING_FPS,
        )
        if not started:
            messagebox.showerror("无法录制", error)
            return

        self.record_button.configure(text="停止录制", state="normal")
        self.recording_var.set("录制中 00:00")
        self.append_log(f"开始录制 MP4：{output_path}")

    def _stop_recording(self, *, wait: bool = False) -> None:
        """停止当前录像并将按钮切换到文件保存状态。

        Args:
            wait: True 时等待 MP4 完成封装，适用于关闭程序。

        Returns:
            None.
        """

        if not self.video_recorder.stop(wait=wait):
            return
        self.record_button.configure(text="开始录制", state="disabled")
        self.recording_var.set("正在保存 MP4...")
        self.append_log("停止录制，正在保存 MP4...")

    def flash_firmware(self) -> None:
        """选择应用固件 bin，确认后通过当前串口启动后台烧录。

        Args:
            None.

        Returns:
            None.
        """

        if self.configuring:
            messagebox.showwarning("暂时无法烧录", "Wi-Fi 配置进行中，请等待配置结束。")
            return
        if self.flashing or self.firmware_flasher.is_running():
            messagebox.showwarning("正在烧录", "当前固件烧录任务尚未结束。")
            return

        port = self._selected_port()
        if not port:
            messagebox.showerror("缺少串口", "请选择 ESP32-CAM 所在串口。")
            return

        selected_path = filedialog.askopenfilename(
            parent=self.root,
            title="选择 ESP32-CAM 应用固件",
            initialdir=str(Path.cwd() / "build"),
            filetypes=(("ESP32 BIN 固件", "*.bin"),),
        )
        if not selected_path:
            return

        firmware_path = Path(selected_path)
        validation_error = self._validate_firmware_file(firmware_path)
        if validation_error:
            messagebox.showerror("固件文件无效", validation_error)
            return

        confirmed = messagebox.askokcancel(
            "确认烧录固件",
            f"串口：{port}\n"
            f"文件：{firmware_path.name}\n"
            f"地址：{FIRMWARE_FLASH_ADDRESS}\n"
            f"波特率：{FIRMWARE_FLASH_BAUD_RATE}\n\n"
            "烧录期间请勿拔出设备。若自动进入下载模式失败，请将 GPIO0 接地后按一下 RST 再重试。",
        )
        if not confirmed:
            return

        available_ports = {device.upper() for device, _ in list_serial_ports()}
        if port.upper() not in available_ports:
            self.append_log(f"无法开始烧录：未检测到串口 {port}")
            messagebox.showerror(
                "串口不存在",
                f"未检测到串口 {port}\n\n请重新连接设备并刷新串口列表后重试。",
            )
            return

        if not self.stream_worker.stop():
            messagebox.showerror("视频连接忙", "旧视频连接尚未结束，请等待几秒后重试。")
            return
        self._stop_recording()

        self.flashing = True
        self.video_connected = False
        self.connect_button.configure(state="disabled")
        self.configure_button.configure(state="disabled")
        self.record_button.configure(state="disabled")
        self.flash_button.configure(state="disabled", text="正在烧录固件...")
        self.port_combo.configure(state="disabled")
        self.connection_var.set("正在烧录固件")
        self.current_frame = None
        self.current_photo = None
        self.video_label.configure(image="", text="正在烧录 ESP32-CAM 固件")
        self.append_log(
            f"开始烧录：{firmware_path} -> {port} @ {FIRMWARE_FLASH_ADDRESS}，由 esptool 直接打开串口"
        )
        if not self.firmware_flasher.start(port=port, firmware_path=firmware_path):
            self.flashing = False
            self.connect_button.configure(state="normal")
            self.configure_button.configure(state="normal")
            self.flash_button.configure(state="normal", text="选择 BIN 并烧录固件")
            self.port_combo.configure(state="normal")
            messagebox.showerror("无法烧录", "已有烧录线程尚未结束。")

    def _validate_firmware_file(self, firmware_path: Path) -> str:
        """验证用户选择的是非空 ESP32 应用镜像 bin。

        Args:
            firmware_path: 用户从文件选择框选取的固件路径。

        Returns:
            文件有效时返回空字符串，否则返回适合显示的错误原因。
        """

        if firmware_path.suffix.lower() != ".bin":
            return "请选择扩展名为 .bin 的固件文件。"
        try:
            file_size = firmware_path.stat().st_size
            with firmware_path.open("rb") as firmware_file:
                image_magic = firmware_file.read(1)
        except OSError as exc:
            return f"无法读取固件文件：{exc}"
        if file_size <= 1:
            return "固件文件为空。"
        if image_magic != b"\xe9":
            return "文件不是有效的 ESP32 应用镜像，首字节应为 0xE9。"
        return ""

    def configure_wifi(self) -> None:
        """校验输入和电脑 Wi-Fi，确认后启动后台串口配置。

        Args:
            None.

        Returns:
            None.
        """

        if self.configuring:
            return
        if self.flashing:
            messagebox.showwarning("暂时无法配置", "固件烧录进行中，请等待烧录结束。")
            return
        port = self._selected_port()
        ssid = self.ssid_var.get().strip()
        open_network = self.open_network_var.get()
        password = "" if open_network else self.password_var.get()
        if not port:
            messagebox.showerror("缺少串口", "请选择 ESP32 所在串口。")
            return
        if not ssid or len(ssid.encode("utf-8")) > 32:
            messagebox.showerror("Wi-Fi 名称无效", "SSID 必须为 1 到 32 字节。")
            return
        password_size = len(password.encode("utf-8"))
        if not open_network and password_size == 0:
            messagebox.showerror(
                "缺少 Wi-Fi 密码",
                "加密网络必须填写密码；无密码热点请勾选“开放网络”。",
            )
            return
        if password_size > 63 or (0 < password_size < 8):
            messagebox.showerror("Wi-Fi 密码无效", "密码必须为 8 到 63 字节。")
            return
        if self.upload_port <= 0:
            messagebox.showerror("服务未启动", "图片接收服务未启动，暂时不能同步上传地址。")
            return

        connected_ssid = get_connected_wifi_ssid()
        if connected_ssid and connected_ssid != ssid:
            messagebox.showwarning(
                "请先连接目标 Wi-Fi",
                f"电脑当前连接：{connected_ssid}\n目标 Wi-Fi：{ssid}\n\n"
                "请先在 Windows 中连接目标 Wi-Fi，然后再次点击更换按钮。",
            )
            return
        if connected_ssid is None:
            confirmed = messagebox.askokcancel(
                "确认电脑网络",
                f"无法自动读取电脑当前 Wi-Fi。\n\n"
                f"请确认电脑已经连接：{ssid}\n确认后再继续配置 ESP32。",
            )
            if not confirmed:
                return
        else:
            confirmed = messagebox.askyesno(
                "确认更换网络",
                f"电脑已连接目标 Wi-Fi：{ssid}\n\n"
                "现在将更换 ESP32 网络并自动同步全部地址，是否继续？",
            )
            if not confirmed:
                return

        if not self.stream_worker.stop():
            messagebox.showerror("视频连接忙", "旧视频连接尚未结束，请等待几秒后重试。")
            return

        self._stop_recording()
        self.configuring = True
        self.video_connected = False
        self.configure_button.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.record_button.configure(state="disabled")
        self.connection_var.set("正在更换 Wi-Fi")
        self.current_frame = None
        self.current_photo = None
        self.video_label.configure(image="", text="正在更换 Wi-Fi")
        self.append_log(f"开始通过 {port} 配置 Wi-Fi：{ssid}")
        configurator = Esp32SerialConfigurator(self._post_log)
        thread = threading.Thread(
            target=self._configure_wifi_worker,
            args=(configurator, port, ssid, password),
            name="wifi-configurator",
            daemon=True,
        )
        thread.start()

    def _configure_wifi_worker(
        self,
        configurator: Esp32SerialConfigurator,
        port: str,
        ssid: str,
        password: str,
    ) -> None:
        """在线程中执行阻塞串口配置并将结果投递给主界面。

        Args:
            configurator: 已创建的 ESP32 串口配置器。
            port: ESP32 串口名。
            ssid: 目标 Wi-Fi 名称。
            password: 目标 Wi-Fi 密码。

        Returns:
            None.
        """

        result = configurator.configure(
            port=port,
            ssid=ssid,
            password=password,
            upload_port=self.upload_port,
        )
        self.events.put(("configuration_result", result))

    def _post_log(self, message: str) -> None:
        """从后台线程向 GUI 队列投递一条进度日志。

        Args:
            message: 需要显示的进度文本。

        Returns:
            None.
        """

        self.events.put(("log", message))

    def append_log(self, message: str) -> None:
        """在窗口日志框末尾追加带时间的文本。

        Args:
            message: 需要显示的日志正文。

        Returns:
            None.
        """

        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _process_events(self) -> None:
        """在 Tk 主线程中消费后台配置和视频事件。

        Args:
            None.

        Returns:
            None.
        """

        while True:
            try:
                event_name, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event_name == "stream_status":
                self._handle_stream_status(payload)  # type: ignore[arg-type]
            elif event_name == "configuration_result":
                self._handle_configuration_result(payload)  # type: ignore[arg-type]
            elif event_name == "recording_result":
                self._handle_recording_result(payload)  # type: ignore[arg-type]
            elif event_name == "firmware_flash_result":
                self._handle_firmware_flash_result(payload)  # type: ignore[arg-type]
            elif event_name == "wifi_scan_result":
                self._handle_wifi_scan_result(payload)  # type: ignore[arg-type]
            elif event_name == "log":
                self.append_log(str(payload))

        newest_frame: tuple[int, Image.Image] | None = None
        while True:
            try:
                newest_frame = self.frames.get_nowait()
            except queue.Empty:
                break
        if newest_frame is not None:
            generation, frame = newest_frame
            if generation == self.stream_worker.generation:
                self.current_frame = frame
                if self.video_recorder.is_recording():
                    self.video_recorder.submit(frame)
                self._render_current_frame()
        self._update_recording_status()
        self.root.after(40, self._process_events)

    def _handle_stream_status(self, payload: tuple[int, bool, str]) -> None:
        """处理视频工作线程发送的连接状态。

        Args:
            payload: ``(连接代次, 是否成功, 状态文本)`` 元组。

        Returns:
            None.
        """

        generation, connected, message = payload
        if generation != self.stream_worker.generation:
            return
        self.video_connected = connected
        self.connection_var.set(message)
        if connected:
            if not self.configuring and not self.video_recorder.is_recording():
                self.record_button.configure(state="normal")
            return

        self.record_button.configure(state="disabled")
        if self.video_recorder.is_recording():
            self._stop_recording()
        if not connected:
            self.append_log(message)

    def _handle_wifi_scan_result(self, payload: tuple[list[str], str]) -> None:
        """更新 Wi-Fi 下拉列表并恢复扫描按钮。

        Args:
            payload: ``(可见 SSID 列表, 错误文本)`` 元组。

        Returns:
            None.
        """

        networks, error = payload
        self.scanning_wifi = False
        self.scan_wifi_button.configure(state="normal", text="扫描网络")
        if error:
            self.append_log(f"Wi-Fi 扫描失败：{error}")
            return

        current_ssid = self.ssid_var.get().strip()
        connected_ssid = get_connected_wifi_ssid()
        if current_ssid and current_ssid not in networks:
            networks.insert(0, current_ssid)
        if connected_ssid:
            if connected_ssid in networks:
                networks.remove(connected_ssid)
            networks.insert(0, connected_ssid)
        self.ssid_combo.configure(values=networks)
        if not current_ssid and networks:
            self.ssid_var.set(networks[0])
        self.append_log(f"扫描到 {len(networks)} 个 Wi-Fi 网络")

    def _handle_recording_result(self, result: RecordingResult) -> None:
        """处理后台 MP4 写入器完成或失败的结果。

        Args:
            result: 包含输出路径、帧数、耗时和错误信息的录像结果。

        Returns:
            None.
        """

        self.record_button.configure(text="开始录制")
        if self.video_connected and not self.configuring:
            self.record_button.configure(state="normal")
        else:
            self.record_button.configure(state="disabled")

        if result.error:
            self.recording_var.set("录制保存失败")
            self.append_log(f"录制失败：{result.error}")
            if not self.closing:
                messagebox.showerror("录制失败", result.error)
            return

        duration_text = self._format_duration(result.elapsed_seconds)
        self.recording_var.set(f"已保存 {duration_text}")
        self.append_log(
            f"视频已保存：{result.output_path}，{result.frame_count} 帧，"
            f"时长 {duration_text}，丢帧 {result.dropped_frames}"
        )

    def _handle_firmware_flash_result(self, result: FirmwareFlashResult) -> None:
        """恢复界面控件并显示 ESP32-CAM 固件烧录结果。

        Args:
            result: 后台 esptool 烧录任务返回的结构化结果。

        Returns:
            None.
        """

        self.flashing = False
        self.connect_button.configure(state="normal")
        self.configure_button.configure(state="normal")
        self.record_button.configure(state="disabled")
        self.flash_button.configure(state="normal", text="选择 BIN 并烧录固件")
        self.port_combo.configure(state="normal")
        if result.success:
            self.connection_var.set("固件烧录完成")
            self.video_label.configure(image="", text="固件烧录完成，请重新连接视频")
            self.append_log(f"{result.message}：{result.firmware_path}")
            messagebox.showinfo(
                "烧录完成",
                f"{result.message}\n\n设备已复位，等待联网后可重新连接视频。",
            )
            return

        self.connection_var.set("固件烧录失败")
        self.video_label.configure(image="", text="固件烧录失败")
        self.append_log(f"烧录失败：{result.message}")
        messagebox.showerror(
            "烧录失败",
            f"{result.message}\n\n"
            "请确认串口未被占用。若设备未自动进入下载模式，请将 GPIO0 接地，按一下 RST 后重试。",
        )

    def _update_recording_status(self) -> None:
        """刷新录制期间显示的实时时长。

        Args:
            None.

        Returns:
            None.
        """

        if not self.video_recorder.is_recording():
            return
        duration_text = self._format_duration(self.video_recorder.elapsed_seconds())
        self.recording_var.set(f"录制中 {duration_text}")

    @staticmethod
    def _format_duration(seconds: float) -> str:
        """把秒数格式化为固定宽度的分秒文本。

        Args:
            seconds: 非负录像时长秒数。

        Returns:
            ``MM:SS`` 格式文本；超过一小时后分钟数继续累加。
        """

        total_seconds = max(0, int(seconds))
        minutes, remaining_seconds = divmod(total_seconds, 60)
        return f"{minutes:02d}:{remaining_seconds:02d}"

    def _handle_configuration_result(self, result: WifiConfigurationResult) -> None:
        """应用 Wi-Fi 配置结果，并优先重连已获得的新设备 IP。

        Args:
            result: 后台串口配置返回的结构化结果。

        Returns:
            None.
        """

        self.configuring = False
        self.configure_button.configure(state="normal")
        self.connect_button.configure(state="normal")
        if result.device_ip:
            self.device_ip_var.set(result.device_ip)
            self.connect_video()
        if result.success:
            self.password_var.set("")
            self.open_network_var.set(False)
            self._toggle_open_network()
            self.append_log(result.message)
            self.append_log(f"设备 IP：{result.device_ip}")
            self.append_log(f"电脑 IP：{result.computer_ip}")
            self.append_log(f"上传地址：{result.upload_url}")
            messagebox.showinfo("配置完成", result.message)
            return
        self.connection_var.set("Wi-Fi 配置未完成")
        self.append_log(f"配置失败：{result.message}")
        messagebox.showerror(
            "配置未完成",
            f"{result.message}\n\n"
            "如果串口被占用，请关闭串口监视器后重试。",
        )

    def _on_video_resize(self, event: tk.Event) -> None:
        """在视频显示区域尺寸改变时重新缩放当前帧。

        Args:
            event: Tkinter Configure 事件；事件字段无需直接读取。

        Returns:
            None.
        """

        del event
        self._render_current_frame()

    def _render_current_frame(self) -> None:
        """按视频区域大小等比例显示最近一帧图像。

        Args:
            None.

        Returns:
            None.
        """

        if self.current_frame is None:
            return
        width = max(1, self.video_label.winfo_width() - 12)
        height = max(1, self.video_label.winfo_height() - 12)
        source_width, source_height = self.current_frame.size
        scale = min(width / source_width, height / source_height)
        target_size = (
            max(1, int(source_width * scale)),
            max(1, int(source_height * scale)),
        )
        frame = self.current_frame.resize(target_size, Image.Resampling.LANCZOS)
        self.current_photo = ImageTk.PhotoImage(frame)
        self.video_label.configure(image=self.current_photo, text="")

    def close(self) -> None:
        """停止后台服务和视频连接后关闭主窗口。

        Args:
            None.

        Returns:
            None.
        """

        if self.flashing or self.firmware_flasher.is_running():
            messagebox.showwarning("正在烧录", "固件烧录尚未结束，暂时不能关闭程序或拔出设备。")
            return
        self.closing = True
        self._stop_recording(wait=True)
        self.stream_worker.stop()
        self.upload_server.stop()
        self.root.destroy()


def main() -> None:
    """启动 ESP32-CAM Windows 桌面上位机。

    Args:
        None.

    Returns:
        None.
    """

    root = tk.Tk()
    CameraDesktopApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
