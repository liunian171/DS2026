from __future__ import annotations

import io
import ipaddress
import json
import locale
import queue
import re
import socket
import subprocess
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
from dataclasses import dataclass
from http.server import ThreadingHTTPServer
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

import serial
import serial.tools.list_ports
from PIL import Image, ImageTk

from camera_web_server import CameraRequestHandler, STATUS_PATH, ensure_storage
from desktop_paths import runtime_data_directory


BASE_DIR = runtime_data_directory()
DESKTOP_CONFIG_PATH = BASE_DIR / "desktop_config.json"
SERIAL_BAUD_RATE = 115200
SERIAL_PROMPT_TIMEOUT_SECONDS = 8.0
WIFI_CONNECT_TIMEOUT_SECONDS = 40.0
DEFAULT_UPLOAD_PORT = 8000
STREAM_PORT = 81
STREAM_PATH = "/stream"
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


def list_serial_ports() -> list[tuple[str, str]]:
    """列出 Windows 当前可用的串口。

    Args:
        None.

    Returns:
        按端口名排序的 ``(端口名, 描述)`` 列表。
    """

    ports = [(item.device, item.description or "串口设备") for item in serial.tools.list_ports.comports()]
    return sorted(ports, key=lambda item: item[0])


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
        self.server: ThreadingHTTPServer | None = None
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
                server = ThreadingHTTPServer(("0.0.0.0", candidate), CameraRequestHandler)
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
            with self._open_serial(port) as connection:
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
                self.status_callback(f"ESP32 正在连接 {ssid}，最长等待约 40 秒...")
                wifi_output = self._wait_for_text(
                    connection,
                    success_markers=("Connected and saved successfully",),
                    failure_markers=("Connection failed", "Connected, but saving failed"),
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

    def start(self, stream_url: str) -> None:
        """停止旧连接并启动指定地址的视频读取线程。

        Args:
            stream_url: ESP32 MJPEG 完整地址。

        Returns:
            None.
        """

        self.stop()
        self.generation += 1
        stop_event = threading.Event()
        self.stop_event = stop_event
        thread = threading.Thread(
            target=self._run,
            args=(stream_url, self.generation, stop_event),
            name="mjpeg-stream",
            daemon=True,
        )
        thread.start()

    def stop(self) -> None:
        """通知当前视频线程停止读取。

        Args:
            None.

        Returns:
            None.
        """

        if self.stop_event is not None:
            self.stop_event.set()
            self.stop_event = None

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
                    self.event_queue.put(("stream_status", (generation, True, "视频已连接")))
                    buffer = bytearray()
                    while not stop_event.is_set():
                        chunk = response.read(4096)
                        if not chunk:
                            raise OSError("视频流已断开")
                        buffer.extend(chunk)
                        self._emit_complete_frames(buffer, generation, stop_event)
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
        self.root.title("ESP32-CAM 上位机")
        self.root.geometry("1180x760")
        self.root.minsize(920, 620)
        self.root.configure(background="#eef1f4")

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.frames: queue.Queue[tuple[int, Image.Image]] = queue.Queue(maxsize=1)
        self.stream_worker = MjpegStreamWorker(self.events, self.frames)
        self.upload_server = EmbeddedUploadServer()
        self.upload_port = 0
        self.current_frame: Image.Image | None = None
        self.current_photo: ImageTk.PhotoImage | None = None
        self.configuring = False

        self.port_var = tk.StringVar()
        self.ssid_var = tk.StringVar()
        self.password_var = tk.StringVar()
        self.show_password_var = tk.BooleanVar(value=False)
        self.device_ip_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="未连接视频")
        self.server_var = tk.StringVar(value="正在启动图片接收服务...")

        self._configure_styles()
        self._build_layout()
        self._load_initial_values()
        self.refresh_serial_ports()
        self._start_upload_server()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(40, self._process_events)

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
        style.configure("Header.TLabel", font=("Microsoft YaHei UI", 18, "bold"), foreground="#18212b")
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

        header = tk.Frame(self.root, background="#ffffff", height=72)
        header.grid(row=0, column=0, columnspan=2, sticky="nsew")
        header.grid_propagate(False)
        ttk.Label(header, text="ESP32-CAM 上位机", style="Header.TLabel", background="#ffffff").pack(
            side="left", padx=24, pady=18
        )
        ttk.Label(header, textvariable=self.connection_var, style="Status.TLabel", background="#ffffff").pack(
            side="right", padx=24
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
        controls.grid(row=1, column=1, sticky="nsew", padx=(8, 18), pady=18)
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
        ttk.Button(controls, text="连接视频", command=self.connect_video).grid(
            row=3, column=0, sticky="ew", padx=20, pady=(8, 18)
        )

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

        ttk.Label(controls, text="Wi-Fi 名称", background="#ffffff").grid(
            row=8, column=0, sticky="w", padx=20, pady=(12, 3)
        )
        ttk.Entry(controls, textvariable=self.ssid_var).grid(row=9, column=0, sticky="ew", padx=20)

        ttk.Label(controls, text="Wi-Fi 密码", background="#ffffff").grid(
            row=10, column=0, sticky="w", padx=20, pady=(12, 3)
        )
        self.password_entry = ttk.Entry(controls, textvariable=self.password_var, show="*")
        self.password_entry.grid(row=11, column=0, sticky="ew", padx=20)
        ttk.Checkbutton(
            controls,
            text="显示密码",
            variable=self.show_password_var,
            command=self._toggle_password_visibility,
        ).grid(row=12, column=0, sticky="w", padx=20, pady=(5, 8))

        self.configure_button = ttk.Button(
            controls,
            text="更换 Wi-Fi 并同步地址",
            style="Primary.TButton",
            command=self.configure_wifi,
        )
        self.configure_button.grid(row=13, column=0, sticky="ew", padx=20, pady=(4, 14))

        ttk.Label(controls, textvariable=self.server_var, style="Subtle.TLabel", background="#ffffff", wraplength=300).grid(
            row=14, column=0, sticky="w", padx=20, pady=(0, 8)
        )
        self.log_text = tk.Text(
            controls,
            height=8,
            wrap="word",
            state="disabled",
            relief="solid",
            borderwidth=1,
            background="#f7f8fa",
            foreground="#303944",
            font=("Consolas", 9),
        )
        self.log_text.grid(row=15, column=0, sticky="nsew", padx=20, pady=(0, 20))
        controls.grid_rowconfigure(15, weight=1)

        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, minsize=350)

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

    def connect_video(self) -> None:
        """使用设备 IP 启动或重启桌面 MJPEG 视频连接。

        Args:
            None.

        Returns:
            None.
        """

        device_ip = self.device_ip_var.get().strip()
        if not is_valid_ipv4(device_ip):
            messagebox.showerror("设备 IP 无效", "请输入 ESP32 的有效 IPv4 地址。")
            return
        stream_url = f"http://{device_ip}:{STREAM_PORT}{STREAM_PATH}"
        self.connection_var.set(f"正在连接 {device_ip}")
        self.append_log(f"连接视频：{stream_url}")
        self.stream_worker.start(stream_url)
        save_desktop_config(
            port=self._selected_port(),
            ssid=self.ssid_var.get().strip(),
            device_ip=device_ip,
        )

    def configure_wifi(self) -> None:
        """校验输入和电脑 Wi-Fi，确认后启动后台串口配置。

        Args:
            None.

        Returns:
            None.
        """

        if self.configuring:
            return
        port = self._selected_port()
        ssid = self.ssid_var.get().strip()
        password = self.password_var.get()
        if not port:
            messagebox.showerror("缺少串口", "请选择 ESP32 所在串口。")
            return
        if not ssid or len(ssid.encode("utf-8")) > 32:
            messagebox.showerror("Wi-Fi 名称无效", "SSID 必须为 1 到 32 字节。")
            return
        password_size = len(password.encode("utf-8"))
        if password_size > 63 or (0 < password_size < 8):
            messagebox.showerror("Wi-Fi 密码无效", "密码必须为空，或为 8 到 63 字节。")
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

        self.configuring = True
        self.configure_button.configure(state="disabled")
        self.connection_var.set("正在更换 Wi-Fi")
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
                self._render_current_frame()
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
        self.connection_var.set(message)
        if not connected:
            self.append_log(message)

    def _handle_configuration_result(self, result: WifiConfigurationResult) -> None:
        """应用 Wi-Fi 配置结果，并优先重连已获得的新设备 IP。

        Args:
            result: 后台串口配置返回的结构化结果。

        Returns:
            None.
        """

        self.configuring = False
        self.configure_button.configure(state="normal")
        if result.device_ip:
            self.device_ip_var.set(result.device_ip)
            self.connect_video()
        if result.success:
            self.password_var.set("")
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
