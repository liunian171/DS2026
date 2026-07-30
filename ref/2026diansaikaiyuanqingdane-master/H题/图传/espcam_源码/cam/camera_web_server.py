from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import ipaddress
import json
import secrets
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from desktop_paths import runtime_data_directory


BASE_DIR = runtime_data_directory()
UPLOAD_DIR = BASE_DIR / "uploaded_images"
LATEST_IMAGE_PATH = UPLOAD_DIR / "latest.jpg"
OFFLINE_IMAGE_PATH = UPLOAD_DIR / "offline_latest.jpg"
OFFLINE_HISTORY_DIR = UPLOAD_DIR / "offline_history"
STATUS_PATH = UPLOAD_DIR / "status.json"
AUTH_PATH = UPLOAD_DIR / "auth.json"
OFFLINE_HISTORY_PATH = UPLOAD_DIR / "offline_history.json"
STATE_LOCK = threading.Lock()


SESSION_COOKIE_NAME = "esp_session"


def _base_page(*, title: str, body_html: str, extra_head_html: str = "") -> str:
    """生成基础 HTML 页面骨架（白色系）。

    Args:
        title: 页面标题。
        body_html: <body> 内部 HTML 片段。
        extra_head_html: 可选的 <head> 额外内容（例如额外脚本/样式）。

    Returns:
        完整 HTML 字符串。
    """

    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>{title}</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f7f8fb;
      --card: #ffffff;
      --text: #111827;
      --muted: #6b7280;
      --border: rgba(17, 24, 39, 0.08);
      --shadow: 0 12px 36px rgba(17, 24, 39, 0.10);
      --primary: #2563eb;
      --primary-2: #1d4ed8;
      --danger: #dc2626;
      --warn: #d97706;
      --ok: #16a34a;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      background: radial-gradient(circle at 20% 0%, rgba(37, 99, 235, 0.10), transparent 40%),
                  radial-gradient(circle at 80% 0%, rgba(20, 184, 166, 0.10), transparent 45%),
                  var(--bg);
      color: var(--text);
      min-height: 100vh;
    }}
    a {{ color: var(--primary); text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    .container {{
      max-width: 1100px;
      margin: 0 auto;
      padding: 28px;
    }}
    .topbar {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 18px 22px;
      border-radius: 18px;
      background: rgba(255, 255, 255, 0.86);
      border: 1px solid var(--border);
      box-shadow: var(--shadow);
      backdrop-filter: blur(10px);
    }}
    .brand {{
      display: grid;
      gap: 4px;
    }}
    .brand-title {{
      margin: 0;
      font-size: 18px;
      font-weight: 700;
    }}
    .brand-sub {{
      margin: 0;
      font-size: 12px;
      color: var(--muted);
    }}
    .badge {{
      padding: 8px 12px;
      border-radius: 999px;
      background: rgba(37, 99, 235, 0.10);
      border: 1px solid rgba(37, 99, 235, 0.22);
      color: var(--primary-2);
      font-size: 12px;
      white-space: nowrap;
    }}
    .badge.ok {{
      background: rgba(22, 163, 74, 0.10);
      border-color: rgba(22, 163, 74, 0.22);
      color: var(--ok);
    }}
    .badge.warn {{
      background: rgba(217, 119, 6, 0.12);
      border-color: rgba(217, 119, 6, 0.24);
      color: var(--warn);
    }}
    .badge.danger {{
      background: rgba(220, 38, 38, 0.12);
      border-color: rgba(220, 38, 38, 0.24);
      color: var(--danger);
    }}
    .grid {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      margin-top: 18px;
    }}
    .card {{
      background: var(--card);
      border: 1px solid var(--border);
      box-shadow: var(--shadow);
      border-radius: 18px;
      padding: 18px;
    }}
    .card h2 {{
      margin: 0 0 10px 0;
      font-size: 16px;
    }}
    .muted {{ color: var(--muted); }}
    .btn {{
      appearance: none;
      border: 1px solid rgba(37, 99, 235, 0.26);
      background: rgba(37, 99, 235, 0.10);
      color: var(--primary-2);
      border-radius: 12px;
      padding: 10px 14px;
      cursor: pointer;
      font-weight: 600;
    }}
    .btn.primary {{
      background: var(--primary);
      border-color: var(--primary);
      color: #ffffff;
    }}
    .btn.primary:hover {{ background: var(--primary-2); }}
    .btn:hover {{
      border-color: rgba(37, 99, 235, 0.40);
    }}
    .btn-row {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 12px;
    }}
    .form {{
      display: grid;
      gap: 12px;
      margin-top: 12px;
    }}
    .field {{
      display: grid;
      gap: 6px;
    }}
    .label {{
      font-size: 12px;
      color: var(--muted);
    }}
    input[type="text"],
    input[type="password"],
    input[type="file"] {{
      width: 100%;
      padding: 10px 12px;
      border-radius: 12px;
      border: 1px solid rgba(17, 24, 39, 0.14);
      outline: none;
      background: #ffffff;
    }}
    input:focus {{
      border-color: rgba(37, 99, 235, 0.55);
      box-shadow: 0 0 0 4px rgba(37, 99, 235, 0.12);
    }}
    .hint {{
      font-size: 12px;
      color: var(--muted);
      line-height: 1.6;
    }}
    .error {{
      color: var(--danger);
      font-size: 12px;
    }}
    .ok {{
      color: var(--ok);
      font-size: 12px;
    }}
    .image-shell {{
      position: relative;
      overflow: hidden;
      border-radius: 16px;
      border: 1px solid var(--border);
      background: #ffffff;
      min-height: 320px;
      display: flex;
      align-items: center;
      justify-content: center;
    }}
    .image-shell img {{
      max-width: 100%;
      max-height: 70vh;
      display: none;
    }}
    .image-shell canvas {{
      position: absolute;
      left: 0;
      top: 0;
      width: 100%;
      height: 100%;
      pointer-events: none;
    }}
    .sensor-panel {{
      position: absolute;
      left: 12px;
      top: 12px;
      display: grid;
      gap: 6px;
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.92);
      border: 1px solid rgba(17, 24, 39, 0.12);
      box-shadow: 0 10px 24px rgba(17, 24, 39, 0.10);
      backdrop-filter: blur(10px);
      font-size: 12px;
      color: var(--text);
      pointer-events: none;
      max-width: calc(100% - 24px);
    }}
    .sensor-panel .row {{
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      align-items: baseline;
    }}
    .sensor-panel .k {{
      color: var(--muted);
    }}
    .sensor-panel .v {{
      font-weight: 700;
    }}
    .split {{
      display: grid;
      grid-template-columns: minmax(0, 1.3fr) minmax(280px, 0.7fr);
      gap: 16px;
      margin-top: 18px;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-top: 12px;
    }}
    .stat {{
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
      background: rgba(255, 255, 255, 0.9);
    }}
    .stat .k {{
      font-size: 12px;
      color: var(--muted);
    }}
    .stat .v {{
      margin-top: 6px;
      font-size: 20px;
      font-weight: 700;
    }}
    @media (max-width: 940px) {{
      .grid {{
        grid-template-columns: 1fr;
      }}
      .split {{
        grid-template-columns: 1fr;
      }}
      .stats {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
  {extra_head_html}
</head>
<body>
  <div class="container">
    {body_html}
  </div>
</body>
</html>"""


def _login_page(*, error_text: str = "") -> str:
    """生成登录页面。

    Args:
        error_text: 可选的错误提示文本。

    Returns:
        登录页面 HTML。
    """

    error_html = f'<div class="error" id="errorBox">{error_text}</div>' if error_text else '<div class="error" id="errorBox"></div>'
    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">管道破损/泄漏检测</h1>
        <p class="brand-sub">ESP32 在线实时检测 / 离线照片检测</p>
      </div>
      <div class="badge">需要登录</div>
    </section>

    <section class="grid">
      <div class="card">
        <h2>登录</h2>
        <div class="hint">登录后可使用在线实时检测与离线上传检测。</div>
        <form class="form" id="loginForm">
          <div class="field">
            <div class="label">用户名</div>
            <input type="text" id="username" autocomplete="username" required>
          </div>
          <div class="field">
            <div class="label">密码</div>
            <input type="password" id="password" autocomplete="current-password" required>
          </div>
          {error_html}
          <button class="btn primary" type="submit">登录</button>
        </form>
        <div class="hint">还没有账户？ <a href="/register">去注册</a></div>
      </div>
      <div class="card">
        <h2>说明</h2>
        <div class="hint">
          - ESP32 在线：自动拉取最新图片并叠加“检测框”。<br>
          - ESP32 不在线：上传照片进行检测并框选。<br>
          - 当前自动框选为“占位检测”，并提供手动框选工具便于校正。
        </div>
      </div>
    </section>

    <script>
      const form = document.getElementById("loginForm");
      const errorBox = document.getElementById("errorBox");
      form.addEventListener("submit", async (e) => {{
        e.preventDefault();
        errorBox.textContent = "";
        const username = document.getElementById("username").value.trim();
        const password = document.getElementById("password").value;
        const resp = await fetch("/api/login", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify({{ username, password }})
        }});
        const data = await resp.json().catch(() => ({{ ok: false, message: "服务返回异常" }}));
        if (!resp.ok || !data.ok) {{
          errorBox.textContent = data.message || "登录失败";
          return;
        }}
        window.location.href = "/app";
      }});
    </script>
    """
    return _base_page(title="登录 - 管道泄漏检测", body_html=body)


def _register_page(*, error_text: str = "") -> str:
    """生成注册页面。

    Args:
        error_text: 可选的错误提示文本。

    Returns:
        注册页面 HTML。
    """

    error_html = f'<div class="error" id="errorBox">{error_text}</div>' if error_text else '<div class="error" id="errorBox"></div>'
    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">创建账户</h1>
        <p class="brand-sub">本地账号，仅用于当前服务</p>
      </div>
      <div class="badge">注册</div>
    </section>

    <section class="grid">
      <div class="card">
        <h2>注册</h2>
        <form class="form" id="registerForm">
          <div class="field">
            <div class="label">用户名</div>
            <input type="text" id="username" autocomplete="username" required>
          </div>
          <div class="field">
            <div class="label">密码</div>
            <input type="password" id="password" autocomplete="new-password" required>
          </div>
          <div class="field">
            <div class="label">确认密码</div>
            <input type="password" id="password2" autocomplete="new-password" required>
          </div>
          {error_html}
          <button class="btn primary" type="submit">创建账户</button>
        </form>
        <div class="hint">已有账户？ <a href="/login">去登录</a></div>
      </div>
      <div class="card">
        <h2>密码提示</h2>
        <div class="hint">
          - 密码会使用 PBKDF2-HMAC-SHA256 加盐哈希后落盘保存。<br>
          - 服务不支持找回密码，忘记后只能重新注册新用户。
        </div>
      </div>
    </section>

    <script>
      const form = document.getElementById("registerForm");
      const errorBox = document.getElementById("errorBox");
      form.addEventListener("submit", async (e) => {{
        e.preventDefault();
        errorBox.textContent = "";
        const username = document.getElementById("username").value.trim();
        const password = document.getElementById("password").value;
        const password2 = document.getElementById("password2").value;
        if (password !== password2) {{
          errorBox.textContent = "两次输入的密码不一致";
          return;
        }}
        const resp = await fetch("/api/register", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify({{ username, password }})
        }});
        const data = await resp.json().catch(() => ({{ ok: false, message: "服务返回异常" }}));
        if (!resp.ok || !data.ok) {{
          errorBox.textContent = data.message || "注册失败";
          return;
        }}
        window.location.href = "/login";
      }});
    </script>
    """
    return _base_page(title="注册 - 管道泄漏检测", body_html=body)


def _app_home_page(*, username: str) -> str:
    """生成应用首页（两种模式入口）。

    Args:
        username: 当前登录用户名。

    Returns:
        应用首页 HTML。
    """

    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">管道破损/泄漏检测</h1>
        <p class="brand-sub">已登录：{username}</p>
      </div>
      <div class="btn-row">
        <button class="btn" id="logoutBtn">退出登录</button>
      </div>
    </section>

    <section class="grid">
      <div class="card">
        <h2>模式 1：ESP32 在线实时检测</h2>
        <div class="hint">设备在线时，自动拉取最新图片与环境数据，并叠加检测框。</div>
        <div class="btn-row">
          <a class="btn primary" href="/online">进入在线检测</a>
        </div>
      </div>
      <div class="card">
        <h2>模式 2：离线照片上传检测</h2>
        <div class="hint">设备不在线时，上传照片进行检测与框选。</div>
        <div class="btn-row">
          <a class="btn primary" href="/offline">进入离线检测</a>
        </div>
      </div>
    </section>

    <script>
      document.getElementById("logoutBtn").addEventListener("click", async () => {{
        await fetch("/api/logout", {{ method: "POST" }});
        window.location.href = "/login";
      }});
    </script>
    """
    return _base_page(title="模式选择 - 管道泄漏检测", body_html=body)


def _online_page(*, username: str) -> str:
    """生成在线检测页面（拉取 latest.jpg + status.json + 检测框）。

    Args:
        username: 当前登录用户名。

    Returns:
        在线检测页面 HTML。
    """

    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">在线实时检测</h1>
        <p class="brand-sub">已登录：{username} · <a href="/app">返回模式选择</a></p>
      </div>
      <div class="badge" id="deviceBadge">检测中…</div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>最新画面（只对照片内容检测并框选）</h2>
      <div class="image-shell" id="imageShell">
        <img id="img" alt="ESP32 最新照片">
        <canvas id="overlay"></canvas>
        <div class="sensor-panel" id="sensorPanel" style="display:none;"></div>
        <div class="hint" id="emptyHint">等待 ESP32 上传到 /upload</div>
      </div>
      <div class="btn-row">
        <button class="btn" id="manualBoxBtn">手动框选</button>
        <button class="btn" id="clearBtn">清空框</button>
      </div>
      <div class="hint">手动框选：点击“手动框选”后，在图片上按住拖拽绘制。</div>
    </section>

    <script>
      const img = document.getElementById("img");
      const overlay = document.getElementById("overlay");
      const shell = document.getElementById("imageShell");
      const emptyHint = document.getElementById("emptyHint");
      const deviceBadge = document.getElementById("deviceBadge");
      const sensorPanel = document.getElementById("sensorPanel");

      let autoBoxes = [];
      let manualBoxes = [];
      let manualMode = false;
      let drag = null;

      function setBadge(online, label) {{
        deviceBadge.className = "badge " + (online ? "ok" : "warn");
        deviceBadge.textContent = (online ? "ESP32 在线 · " : "ESP32 不在线 · ") + label;
      }}

      function canvasSizeToMatch() {{
        const rect = shell.getBoundingClientRect();
        overlay.width = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
        overlay.height = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
      }}

      /**
       * 把温湿度与涡流传感器数据显示到图片区域左上角。
       *
       * @param {{temperature_c?: number|null, humidity_rh?: number|null, eddy_current_v?: number|null}} status 设备状态对象，可选字段。
       * @returns {{void}}
       */
      function updateSensorPanel(status) {{
        if (!status || typeof status !== "object") {{
          sensorPanel.style.display = "none";
          sensorPanel.innerHTML = "";
          return;
        }}
        const t = status.temperature_c;
        const h = status.humidity_rh;
        const e = status.eddy_current_v;
        const hasAny = (typeof t === "number") || (typeof h === "number") || (typeof e === "number");
        if (!hasAny) {{
          sensorPanel.style.display = "none";
          sensorPanel.innerHTML = "";
          return;
        }}
        const fmt = (v, digits) => (typeof v === "number" ? v.toFixed(digits) : "--");
        sensorPanel.innerHTML = `
          <div class="row"><span class="k">温度</span><span class="v">${{fmt(t, 1)}} ℃</span></div>
          <div class="row"><span class="k">湿度</span><span class="v">${{fmt(h, 1)}} %RH</span></div>
          <div class="row"><span class="k">涡流</span><span class="v">${{fmt(e, 2)}} V</span></div>
        `;
        sensorPanel.style.display = "grid";
      }}

      function drawBoxes() {{
        canvasSizeToMatch();
        const ctx = overlay.getContext("2d");
        ctx.setTransform(window.devicePixelRatio, 0, 0, window.devicePixelRatio, 0, 0);
        ctx.clearRect(0, 0, overlay.width, overlay.height);

        const all = [...autoBoxes, ...manualBoxes];
        if (!img.src || img.style.display === "none") return;

        const imgRect = img.getBoundingClientRect();
        const shellRect = shell.getBoundingClientRect();
        const x0 = imgRect.left - shellRect.left;
        const y0 = imgRect.top - shellRect.top;
        const w = imgRect.width;
        const h = imgRect.height;

        for (const b of all) {{
          const bx = x0 + b.x * w;
          const by = y0 + b.y * h;
          const bw = b.w * w;
          const bh = b.h * h;
          ctx.lineWidth = 2;
          ctx.strokeStyle = b.color || "#dc2626";
          ctx.strokeRect(bx, by, bw, bh);
          ctx.fillStyle = b.color || "#dc2626";
          ctx.font = "12px Segoe UI, Arial, sans-serif";
          const label = b.label || "疑似破损";
          ctx.fillText(label, bx + 4, Math.max(12, by - 6));
        }}
      }}

      function startManualBox(e) {{
        if (!manualMode || img.style.display === "none") return;
        const shellRect = shell.getBoundingClientRect();
        const imgRect = img.getBoundingClientRect();
        const px = e.clientX - imgRect.left;
        const py = e.clientY - imgRect.top;
        if (px < 0 || py < 0 || px > imgRect.width || py > imgRect.height) return;
        drag = {{
          x0: px / imgRect.width,
          y0: py / imgRect.height,
          x1: px / imgRect.width,
          y1: py / imgRect.height
        }};
        overlay.style.pointerEvents = "auto";
        e.preventDefault();
      }}

      function moveManualBox(e) {{
        if (!drag || img.style.display === "none") return;
        const imgRect = img.getBoundingClientRect();
        const px = e.clientX - imgRect.left;
        const py = e.clientY - imgRect.top;
        drag.x1 = Math.min(1, Math.max(0, px / imgRect.width));
        drag.y1 = Math.min(1, Math.max(0, py / imgRect.height));

        manualBoxes = manualBoxes.filter(b => !b._temp);
        const x = Math.min(drag.x0, drag.x1);
        const y = Math.min(drag.y0, drag.y1);
        const w = Math.abs(drag.x1 - drag.x0);
        const h = Math.abs(drag.y1 - drag.y0);
        manualBoxes.push({{ x, y, w, h, label: "手动框", color: "#2563eb", _temp: true }});
        drawBoxes();
        e.preventDefault();
      }}

      function endManualBox(e) {{
        if (!drag) return;
        manualBoxes = manualBoxes.filter(b => !b._temp);
        const x = Math.min(drag.x0, drag.x1);
        const y = Math.min(drag.y0, drag.y1);
        const w = Math.abs(drag.x1 - drag.x0);
        const h = Math.abs(drag.y1 - drag.y0);
        if (w > 0.01 && h > 0.01) {{
          manualBoxes.push({{ x, y, w, h, label: "手动框", color: "#2563eb" }});
        }}
        drag = null;
        drawBoxes();
        e.preventDefault();
      }}

      document.getElementById("manualBoxBtn").addEventListener("click", () => {{
        manualMode = !manualMode;
        const btn = document.getElementById("manualBoxBtn");
        btn.textContent = manualMode ? "退出手动框选" : "手动框选";
        overlay.style.pointerEvents = manualMode ? "auto" : "none";
      }});
      document.getElementById("clearBtn").addEventListener("click", () => {{
        manualBoxes = [];
        drawBoxes();
      }});

      overlay.addEventListener("mousedown", startManualBox);
      window.addEventListener("mousemove", moveManualBox);
      window.addEventListener("mouseup", endManualBox);
      window.addEventListener("resize", drawBoxes);
      img.addEventListener("load", drawBoxes);

      async function refresh() {{
        const dsResp = await fetch("/api/device_status?ts=" + Date.now(), {{ cache: "no-store" }});
        const ds = await dsResp.json().catch(() => ({{ online: false, message: "无法连接" }}));
        setBadge(!!ds.online, ds.message || "未知");

        if (!ds.has_image) {{
          img.style.display = "none";
          emptyHint.style.display = "block";
          autoBoxes = [];
          updateSensorPanel(null);
          drawBoxes();
          return;
        }}

        img.src = "/latest.jpg?ts=" + (ds.updated_unix_ms || Date.now());
        img.style.display = "block";
        emptyHint.style.display = "none";

        const stResp = await fetch("/status.json?ts=" + Date.now(), {{ cache: "no-store" }});
        const st = await stResp.json().catch(() => ({{}}));
        updateSensorPanel(st);

        const bxResp = await fetch("/api/detect/latest?ts=" + Date.now(), {{ cache: "no-store" }});
        const bx = await bxResp.json().catch(() => ({{ boxes: [] }}));
        autoBoxes = (bx.boxes || []).map(b => ({{ ...b, color: "#dc2626" }}));
        drawBoxes();
      }}

      refresh();
      setInterval(refresh, 2000);
    </script>
    """
    return _base_page(title="在线检测 - 管道泄漏检测", body_html=body)


def _live_stream_page() -> str:
    """生成仅显示 ESP32-CAM MJPEG 视频流的全屏页面。

    Args:
        None

    Returns:
        仅包含视频流图像的完整 HTML 字符串。
    """

    return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM</title>
  <style>
    * { box-sizing: border-box; }
    html, body {
      width: 100%;
      height: 100%;
      margin: 0;
      overflow: hidden;
      background: #101214;
    }
    .stream {
      display: block;
      width: 100vw;
      height: 100vh;
      object-fit: contain;
    }
    .record-controls {
      position: fixed;
      right: 28px;
      bottom: 28px;
      z-index: 10;
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .record-timer {
      display: none;
      min-width: 58px;
      padding: 7px 9px;
      border: 1px solid rgba(255, 255, 255, 0.18);
      border-radius: 6px;
      background: rgba(255, 255, 255, 0.94);
      color: #111827;
      font: 600 13px/1 "Segoe UI", Arial, sans-serif;
      text-align: center;
      font-variant-numeric: tabular-nums;
    }
    .record-timer.visible {
      display: block;
    }
    .record-button {
      width: 88px;
      height: 88px;
      padding: 0;
      border: 2px solid rgba(17, 24, 39, 0.18);
      border-radius: 50%;
      background: #ffffff;
      box-shadow: 0 6px 20px rgba(0, 0, 0, 0.34);
      display: grid;
      place-items: center;
      cursor: pointer;
    }
    .record-button:hover {
      background: #f3f4f6;
    }
    .record-button:focus-visible {
      outline: 3px solid rgba(255, 255, 255, 0.72);
      outline-offset: 2px;
    }
    .record-button:disabled {
      cursor: not-allowed;
      opacity: 0.45;
    }
    .record-symbol {
      width: 40px;
      height: 40px;
      border-radius: 50%;
      background: #ef4444;
    }
    .record-button.recording {
      background: #ef4444;
      border-color: #ffffff;
    }
    .record-button.recording .record-symbol {
      width: 32px;
      height: 32px;
      border-radius: 4px;
      background: #ffffff;
    }
  </style>
</head>
<body>
  <img class="stream" src="/stream.mjpg" alt="ESP32-CAM 视频流">
  <div class="record-controls">
    <span class="record-timer" id="recordTimer" aria-live="polite">00:00</span>
    <button class="record-button" id="recordButton" type="button" title="开始屏幕录制" aria-label="开始屏幕录制">
      <span class="record-symbol" aria-hidden="true"></span>
    </button>
  </div>
  <script>
    const recordButton = document.getElementById("recordButton");
    const recordTimer = document.getElementById("recordTimer");
    let displayStream = null;
    let mediaRecorder = null;
    let recordedChunks = [];
    let recordingStartedAt = 0;
    let recordingTimerId = null;

    /**
     * 选择当前浏览器支持的视频录制格式。
     *
     * @param {string[]} candidates 可选 MIME 类型列表，按优先级排列。
     * @returns {string} 支持的 MIME 类型；均不支持时返回空字符串。
     */
    function selectRecordingMimeType(candidates) {
      for (const candidate of candidates) {
        if (MediaRecorder.isTypeSupported(candidate)) {
          return candidate;
        }
      }
      return "";
    }

    /**
     * 根据录制 MIME 类型选择下载文件扩展名。
     *
     * @param {string} mimeType MediaRecorder 实际使用的 MIME 类型，可为空。
     * @returns {string} MP4 格式返回 mp4，其他格式返回 webm。
     */
    function recordingExtensionForMimeType(mimeType) {
      return String(mimeType).toLowerCase().includes("video/mp4") ? "mp4" : "webm";
    }

    /**
     * 将录制秒数格式化为 mm:ss。
     *
     * @param {number} totalSeconds 从录制开始计算的秒数。
     * @returns {string} 分钟和秒组成的计时文本。
     */
    function formatRecordingDuration(totalSeconds) {
      const minutes = Math.floor(totalSeconds / 60);
      const seconds = totalSeconds % 60;
      return String(minutes).padStart(2, "0") + ":" + String(seconds).padStart(2, "0");
    }

    /**
     * 根据当前状态更新录制按钮、提示和计时器可见性。
     *
     * @param {boolean} isRecording true 表示正在录制，false 表示空闲。
     * @returns {void}
     */
    function setRecordingUi(isRecording) {
      recordButton.classList.toggle("recording", isRecording);
      recordButton.title = isRecording ? "停止并保存录制" : "开始屏幕录制";
      recordButton.setAttribute("aria-label", recordButton.title);
      recordTimer.classList.toggle("visible", isRecording);
      if (!isRecording) {
        recordTimer.textContent = "00:00";
      }
    }

    /**
     * 刷新录制计时显示。
     *
     * @param 无。
     * @returns {void}
     */
    function updateRecordingTimer() {
      const elapsedSeconds = Math.max(0, Math.floor((Date.now() - recordingStartedAt) / 1000));
      recordTimer.textContent = formatRecordingDuration(elapsedSeconds);
    }

    /**
     * 停止指定的媒体轨道。
     *
     * @param {MediaStreamTrack} track 需要停止的屏幕或音频轨道。
     * @returns {void}
     */
    function stopMediaTrack(track) {
      track.stop();
    }

    /**
     * 释放一次下载所创建的对象 URL。
     *
     * @param {string} objectUrl 需要释放的 Blob 对象 URL。
     * @returns {void}
     */
    function revokeRecordingUrl(objectUrl) {
      URL.revokeObjectURL(objectUrl);
    }

    /**
     * 在 WebM Info 区块中写入实际时长，使本地播放器能识别完整视频。
     *
     * @param {Blob} blob MediaRecorder 产生的 WebM 文件数据。
     * @param {number} durationMilliseconds 从开始到停止的实际录制毫秒数。
     * @returns {Promise<Blob>} 已写入时长的 WebM；文件头不可识别时返回原 Blob。
     */
    async function addWebmDurationMetadata(blob, durationMilliseconds) {
      const bytes = new Uint8Array(await blob.arrayBuffer());
      const infoId = [0x15, 0x49, 0xa9, 0x66];
      const searchLimit = Math.min(bytes.length - infoId.length, 4096);
      let infoOffset = -1;

      for (let offset = 0; offset <= searchLimit; offset += 1) {
        if (infoId.every((value, index) => bytes[offset + index] === value)) {
          infoOffset = offset;
          break;
        }
      }
      if (infoOffset < 0) {
        return blob;
      }

      const sizeOffset = infoOffset + infoId.length;
      const firstSizeByte = bytes[sizeOffset];
      let sizeLength = 1;
      let marker = 0x80;
      while (sizeLength <= 4 && (firstSizeByte & marker) === 0) {
        sizeLength += 1;
        marker >>= 1;
      }
      if (sizeLength > 4 || marker === 0) {
        return blob;
      }

      let payloadSize = firstSizeByte & (marker - 1);
      for (let index = 1; index < sizeLength; index += 1) {
        payloadSize = (payloadSize * 256) + bytes[sizeOffset + index];
      }

      const payloadStart = sizeOffset + sizeLength;
      const payloadEnd = payloadStart + payloadSize;
      if (payloadEnd > bytes.length) {
        return blob;
      }

      for (let offset = payloadStart; offset < payloadEnd - 1; offset += 1) {
        if (bytes[offset] === 0x44 && bytes[offset + 1] === 0x89) {
          return blob;
        }
      }

      const durationElement = new Uint8Array(11);
      durationElement.set([0x44, 0x89, 0x88], 0);
      new DataView(durationElement.buffer).setFloat64(3, durationMilliseconds, false);

      const updatedPayloadSize = payloadSize + durationElement.length;
      const maximumPayloadSize = Math.pow(2, 7 * sizeLength) - 2;
      if (updatedPayloadSize > maximumPayloadSize) {
        return blob;
      }

      const encodedSize = new Uint8Array(sizeLength);
      let remainingSize = updatedPayloadSize;
      for (let index = sizeLength - 1; index >= 0; index -= 1) {
        encodedSize[index] = remainingSize % 256;
        remainingSize = Math.floor(remainingSize / 256);
      }
      encodedSize[0] |= marker;

      return new Blob([
        bytes.slice(0, sizeOffset),
        encodedSize,
        bytes.slice(payloadStart, payloadEnd),
        durationElement,
        bytes.slice(payloadEnd)
      ], { type: blob.type || "video/webm" });
    }

    /**
     * 将已采集的数据合并为完整视频，修复 WebM 时长后触发下载。
     *
     * @param {BlobPart[]} chunks MediaRecorder 产生的数据块列表。
     * @param {string} mimeType 录制器最终使用的 MIME 类型，可为空。
     * @param {number} durationMilliseconds 录制开始到停止的实际毫秒数。
     * @returns {Promise<void>} 文件元数据处理并开始下载后完成。
     */
    async function downloadRecording(chunks, mimeType, durationMilliseconds) {
      if (!chunks.length) {
        return;
      }
      let blob = new Blob(chunks, { type: mimeType || "video/webm" });
      if (recordingExtensionForMimeType(mimeType) === "webm") {
        blob = await addWebmDurationMetadata(blob, durationMilliseconds);
      }
      const objectUrl = URL.createObjectURL(blob);
      const link = document.createElement("a");
      const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
      const extension = recordingExtensionForMimeType(mimeType);
      link.href = objectUrl;
      link.download = "esp32-cam-" + timestamp + "." + extension;
      link.click();
      window.setTimeout(revokeRecordingUrl, 1000, objectUrl);
    }

    /**
     * 收集 MediaRecorder 输出的数据块。
     *
     * @param {BlobEvent} event 录制器的数据事件对象。
     * @returns {void}
     */
    function handleRecordingData(event) {
      if (event.data && event.data.size > 0) {
        recordedChunks.push(event.data);
      }
    }

    /**
     * 处理录制器停止事件，清理媒体资源并下载视频。
     *
     * @param {Event} event MediaRecorder 停止事件，可选且不读取其字段。
     * @returns {Promise<void>} 完成时长修复、视频下载与媒体资源清理后结束。
     */
    async function handleRecorderStopped(event) {
      void event;
      if (recordingTimerId !== null) {
        window.clearInterval(recordingTimerId);
        recordingTimerId = null;
      }
      if (displayStream !== null) {
        displayStream.getTracks().forEach(stopMediaTrack);
      }
      const mimeType = mediaRecorder ? mediaRecorder.mimeType : "video/webm";
      const durationMilliseconds = Math.max(1, Date.now() - recordingStartedAt);
      try {
        await downloadRecording(recordedChunks, mimeType, durationMilliseconds);
      } catch (error) {
        console.error("Recording download failed", error);
      } finally {
        displayStream = null;
        mediaRecorder = null;
        recordedChunks = [];
        setRecordingUi(false);
      }
    }

    /**
     * 在用户从浏览器共享栏停止共享时同步结束录制。
     *
     * @param {Event} event 屏幕视频轨道结束事件，可选且不读取其字段。
     * @returns {void}
     */
    function handleCapturedTrackEnded(event) {
      void event;
      if (mediaRecorder && mediaRecorder.state !== "inactive") {
        mediaRecorder.stop();
      }
    }

    /**
     * 请求屏幕捕获权限并以 15 FPS 开始录制。
     *
     * @param 无。
     * @returns {Promise<void>} 用户完成选择后启动录制；取消或失败时恢复空闲状态。
     */
    async function startScreenRecording() {
      try {
        displayStream = await navigator.mediaDevices.getDisplayMedia({
          video: { frameRate: { ideal: 15, max: 15 } },
          audio: false,
          preferCurrentTab: true
        });
        const mimeType = selectRecordingMimeType([
          "video/mp4;codecs=avc1.42E01E",
          "video/mp4;codecs=avc1",
          "video/mp4",
          "video/webm;codecs=vp9",
          "video/webm;codecs=vp8",
          "video/webm"
        ]);
        const recorderOptions = { videoBitsPerSecond: 2500000 };
        if (mimeType) {
          recorderOptions.mimeType = mimeType;
        }
        mediaRecorder = new MediaRecorder(displayStream, recorderOptions);
        recordedChunks = [];
        mediaRecorder.addEventListener("dataavailable", handleRecordingData);
        mediaRecorder.addEventListener("stop", handleRecorderStopped, { once: true });
        displayStream.getVideoTracks()[0].addEventListener("ended", handleCapturedTrackEnded, { once: true });
        mediaRecorder.start();
        recordingStartedAt = Date.now();
        recordingTimerId = window.setInterval(updateRecordingTimer, 1000);
        updateRecordingTimer();
        setRecordingUi(true);
      } catch (error) {
        console.warn("Screen recording was not started", error);
        displayStream = null;
        mediaRecorder = null;
        recordedChunks = [];
        setRecordingUi(false);
      }
    }

    /**
     * 停止当前录制并触发保存流程。
     *
     * @param 无。
     * @returns {void}
     */
    function stopScreenRecording() {
      if (mediaRecorder && mediaRecorder.state !== "inactive") {
        mediaRecorder.stop();
      }
    }

    /**
     * 根据当前录制状态切换开始或停止操作。
     *
     * @param {MouseEvent} event 录制按钮的点击事件，可选且不读取其字段。
     * @returns {Promise<void>} 开始录制时等待权限结果，停止时立即完成。
     */
    async function handleRecordButtonClick(event) {
      void event;
      if (mediaRecorder && mediaRecorder.state !== "inactive") {
        stopScreenRecording();
        return;
      }
      await startScreenRecording();
    }

    /**
     * 初始化屏幕录制控件，并在浏览器不支持时禁用按钮。
     *
     * @param 无。
     * @returns {void}
     */
    function initializeRecordingControls() {
      const isSupported = Boolean(
        navigator.mediaDevices && navigator.mediaDevices.getDisplayMedia && window.MediaRecorder
      );
      recordButton.disabled = !isSupported;
      recordButton.addEventListener("click", handleRecordButtonClick);
      setRecordingUi(false);
    }

    initializeRecordingControls();
  </script>
</body>
</html>
"""


def _offline_page(*, username: str) -> str:
    """生成离线检测页面（上传图片，返回检测框）。

    Args:
        username: 当前登录用户名。

    Returns:
        离线检测页面 HTML。
    """

    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">离线照片检测</h1>
        <p class="brand-sub">已登录：{username} · <a href="/app">返回模式选择</a></p>
      </div>
      <div class="btn-row">
        <a class="btn" href="/history">查看历史记录</a>
        <div class="badge">批量检测</div>
      </div>
    </section>

    <section class="split">
      <div class="card">
        <h2>上传并检测（叠加检测框）</h2>
        <div class="form">
          <div class="field">
            <div class="label">检测方式</div>
            <select id="detectMode">
              <option value="single" selected>单张检测</option>
              <option value="batch">批量检测</option>
            </select>
          </div>
          <div class="field">
            <div class="label">选择图片（JPG/PNG）</div>
            <input type="file" id="file" accept="image/*">
          </div>
          <div class="btn-row">
            <button class="btn primary" id="uploadBtn">开始检测</button>
            <button class="btn" id="autoDetectBtn">重新自动检测</button>
            <button class="btn" id="saveBtn">保存当前结果</button>
            <button class="btn" id="manualBoxBtn">手动框选</button>
            <button class="btn" id="clearBtn">清空框</button>
          </div>
          <div class="error" id="err"></div>
          <div class="ok" id="ok"></div>
        </div>
        <div class="image-shell" id="imageShell">
          <img id="img" alt="离线上传图片">
          <canvas id="overlay"></canvas>
          <div class="hint" id="emptyHint">请选择图片并点击“上传并检测”</div>
        </div>
        <div class="hint">自动框选基于照片内容计算；可用手动框选工具进行校正。</div>
      </div>

      <div class="card">
        <h2>批量进度</h2>
        <div class="hint" id="resultHint">等待检测…</div>
        <div class="hint" id="progressList" style="margin-top: 10px; white-space: pre-wrap;"></div>
      </div>
    </section>

    <script>
      const fileEl = document.getElementById("file");
      const detectModeEl = document.getElementById("detectMode");
      const uploadBtn = document.getElementById("uploadBtn");
      const autoDetectBtn = document.getElementById("autoDetectBtn");
      const saveBtn = document.getElementById("saveBtn");
      const err = document.getElementById("err");
      const ok = document.getElementById("ok");
      const resultHint = document.getElementById("resultHint");
      const progressList = document.getElementById("progressList");
      const img = document.getElementById("img");
      const overlay = document.getElementById("overlay");
      const shell = document.getElementById("imageShell");
      const emptyHint = document.getElementById("emptyHint");

      let autoBoxes = [];
      let manualBoxes = [];
      let manualMode = false;
      let drag = null;
      let currentRecordId = null;
      let autoComputeOnLoad = true;

      /**
       * 根据检测方式（单张/批量）更新输入控件与提示文案。
       *
       * @returns {void}
       */
      function updateDetectModeUI() {{
        const mode = detectModeEl.value || "single";
        fileEl.multiple = (mode === "batch");
        uploadBtn.textContent = (mode === "batch") ? "开始批量检测" : "开始检测";
        progressList.textContent = "";
        resultHint.textContent = (mode === "batch") ? "等待批量检测…" : "等待检测…";
      }}
      detectModeEl.addEventListener("change", updateDetectModeUI);
      updateDetectModeUI();

      function clamp01(v) {{
        return Math.max(0, Math.min(1, v));
      }}

      function computeAutoBoxesFromImage(image) {{
        const sw = image.naturalWidth || 0;
        const sh = image.naturalHeight || 0;
        if (sw <= 0 || sh <= 0) return [];

        const maxSide = 640;
        const scale = Math.min(1, maxSide / Math.max(sw, sh));
        const w = Math.max(1, Math.round(sw * scale));
        const h = Math.max(1, Math.round(sh * scale));

        const c = document.createElement("canvas");
        c.width = w;
        c.height = h;
        const ctx = c.getContext("2d", {{ willReadFrequently: true }});
        if (!ctx) return [];
        ctx.drawImage(image, 0, 0, w, h);

        const imgData = ctx.getImageData(0, 0, w, h);
        const data = imgData.data;
        const n = w * h;
        const gray = new Uint8Array(n);

        let sum = 0;
        for (let i = 0, p = 0; p < n; p += 1, i += 4) {{
          const r = data[i];
          const g = data[i + 1];
          const b = data[i + 2];
          const v = (r * 0.299 + g * 0.587 + b * 0.114) | 0;
          gray[p] = v;
          sum += v;
        }}
        const mean = sum / n;

        let vSum = 0;
        for (let p = 0; p < n; p += 1) {{
          const d = gray[p] - mean;
          vSum += d * d;
        }}
        const std = Math.sqrt(vSum / n);

        const thr = Math.max(0, Math.min(255, mean - std * 0.65));
        const mask = new Uint8Array(n);
        for (let y = 1; y < h - 1; y += 1) {{
          for (let x = 1; x < w - 1; x += 1) {{
            const idx = y * w + x;
            const v = gray[idx];
            if (v >= thr) continue;
            const v1 = gray[idx - 1];
            const v2 = gray[idx + 1];
            const v3 = gray[idx - w];
            const v4 = gray[idx + w];
            const localRange = Math.max(v1, v2, v3, v4) - Math.min(v1, v2, v3, v4);
            if (localRange < 14) continue;
            mask[idx] = 1;
          }}
        }}

        const visited = new Uint8Array(n);
        const boxes = [];
        const minArea = Math.max(40, Math.floor(n * 0.0012));

        const qx = [];
        const qy = [];
        for (let y = 0; y < h; y += 1) {{
          for (let x = 0; x < w; x += 1) {{
            const start = y * w + x;
            if (mask[start] !== 1 || visited[start] === 1) continue;

            let head = 0;
            qx.length = 0;
            qy.length = 0;
            qx.push(x);
            qy.push(y);
            visited[start] = 1;

            let minX = x, minY = y, maxX = x, maxY = y, area = 0;
            while (head < qx.length) {{
              const cx = qx[head];
              const cy = qy[head];
              head += 1;
              area += 1;
              if (cx < minX) minX = cx;
              if (cy < minY) minY = cy;
              if (cx > maxX) maxX = cx;
              if (cy > maxY) maxY = cy;

              const nbs = [
                [cx - 1, cy],
                [cx + 1, cy],
                [cx, cy - 1],
                [cx, cy + 1],
              ];
              for (const nb of nbs) {{
                const nx = nb[0];
                const ny = nb[1];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                const ni = ny * w + nx;
                if (visited[ni] === 1) continue;
                if (mask[ni] !== 1) continue;
                visited[ni] = 1;
                qx.push(nx);
                qy.push(ny);
              }}
            }}

            if (area < minArea) continue;
            const bw = (maxX - minX + 1) / w;
            const bh = (maxY - minY + 1) / h;
            if (bw < 0.03 || bh < 0.03) continue;
            boxes.push({{
              x: clamp01(minX / w),
              y: clamp01(minY / h),
              w: clamp01(bw),
              h: clamp01(bh),
              label: "疑似破损/泄漏",
              score: clamp01(area / n)
            }});
          }}
        }}

        boxes.sort((a, b) => (b.score || 0) - (a.score || 0));
        return boxes.slice(0, 3);
      }}

      function canvasSizeToMatch() {{
        const rect = shell.getBoundingClientRect();
        overlay.width = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
        overlay.height = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
      }}

      function drawBoxes() {{
        canvasSizeToMatch();
        const ctx = overlay.getContext("2d");
        ctx.setTransform(window.devicePixelRatio, 0, 0, window.devicePixelRatio, 0, 0);
        ctx.clearRect(0, 0, overlay.width, overlay.height);
        if (!img.src || img.style.display === "none") return;

        const imgRect = img.getBoundingClientRect();
        const shellRect = shell.getBoundingClientRect();
        const x0 = imgRect.left - shellRect.left;
        const y0 = imgRect.top - shellRect.top;
        const w = imgRect.width;
        const h = imgRect.height;

        const all = [...autoBoxes, ...manualBoxes];
        for (const b of all) {{
          const bx = x0 + b.x * w;
          const by = y0 + b.y * h;
          const bw = b.w * w;
          const bh = b.h * h;
          ctx.lineWidth = 2;
          ctx.strokeStyle = b.color || "#dc2626";
          ctx.strokeRect(bx, by, bw, bh);
          ctx.fillStyle = b.color || "#dc2626";
          ctx.font = "12px Segoe UI, Arial, sans-serif";
          ctx.fillText(b.label || "疑似破损", bx + 4, Math.max(12, by - 6));
        }}
      }}

      function startManualBox(e) {{
        if (!manualMode || img.style.display === "none") return;
        const imgRect = img.getBoundingClientRect();
        const px = e.clientX - imgRect.left;
        const py = e.clientY - imgRect.top;
        if (px < 0 || py < 0 || px > imgRect.width || py > imgRect.height) return;
        drag = {{ x0: px / imgRect.width, y0: py / imgRect.height, x1: px / imgRect.width, y1: py / imgRect.height }};
        e.preventDefault();
      }}

      function moveManualBox(e) {{
        if (!drag || img.style.display === "none") return;
        const imgRect = img.getBoundingClientRect();
        const px = e.clientX - imgRect.left;
        const py = e.clientY - imgRect.top;
        drag.x1 = Math.min(1, Math.max(0, px / imgRect.width));
        drag.y1 = Math.min(1, Math.max(0, py / imgRect.height));
        manualBoxes = manualBoxes.filter(b => !b._temp);
        const x = Math.min(drag.x0, drag.x1);
        const y = Math.min(drag.y0, drag.y1);
        const w = Math.abs(drag.x1 - drag.x0);
        const h = Math.abs(drag.y1 - drag.y0);
        manualBoxes.push({{ x, y, w, h, label: "手动框", color: "#2563eb", _temp: true }});
        drawBoxes();
        e.preventDefault();
      }}

      function endManualBox(e) {{
        if (!drag) return;
        manualBoxes = manualBoxes.filter(b => !b._temp);
        const x = Math.min(drag.x0, drag.x1);
        const y = Math.min(drag.y0, drag.y1);
        const w = Math.abs(drag.x1 - drag.x0);
        const h = Math.abs(drag.y1 - drag.y0);
        if (w > 0.01 && h > 0.01) {{
          manualBoxes.push({{ x, y, w, h, label: "手动框", color: "#2563eb" }});
        }}
        drag = null;
        drawBoxes();
        e.preventDefault();
      }}

      document.getElementById("manualBoxBtn").addEventListener("click", () => {{
        manualMode = !manualMode;
        const btn = document.getElementById("manualBoxBtn");
        btn.textContent = manualMode ? "退出手动框选" : "手动框选";
        overlay.style.pointerEvents = manualMode ? "auto" : "none";
      }});
      document.getElementById("clearBtn").addEventListener("click", () => {{
        manualBoxes = [];
        drawBoxes();
      }});
      overlay.addEventListener("mousedown", startManualBox);
      window.addEventListener("mousemove", moveManualBox);
      window.addEventListener("mouseup", endManualBox);
      window.addEventListener("resize", drawBoxes);
      img.addEventListener("load", () => {{
        drawBoxes();
        if (!autoComputeOnLoad) {{
          return;
        }}
        const computed = computeAutoBoxesFromImage(img).map(b => ({{ ...b, color: "#dc2626" }}));
        autoBoxes = computed;
        drawBoxes();
        resultHint.textContent = "检测框数量：" + autoBoxes.length + "（自动） + " + manualBoxes.length + "（手动）";
      }});

      autoDetectBtn.addEventListener("click", () => {{
        err.textContent = "";
        ok.textContent = "";
        if (img.style.display === "none") {{
          err.textContent = "请先上传并显示图片后再自动检测";
          return;
        }}
        const computed = computeAutoBoxesFromImage(img).map(b => ({{ ...b, color: "#dc2626" }}));
        autoBoxes = computed;
        drawBoxes();
        resultHint.textContent = "检测框数量：" + autoBoxes.length + "（自动） + " + manualBoxes.length + "（手动）";
      }});

      async function uploadOne(file) {{
        const resp = await fetch("/api/offline/upload?filename=" + encodeURIComponent(file.name), {{
          method: "POST",
          headers: {{ "Content-Type": file.type || "application/octet-stream" }},
          body: file
        }});
        const data = await resp.json().catch(() => ({{ ok: false, message: "服务返回异常" }}));
        if (!resp.ok || !data.ok) {{
          throw new Error(data.message || "上传失败");
        }}
        return data;
      }}

      async function saveResult(recordId, payload) {{
        const resp = await fetch("/api/offline/save_result", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify({{
            record_id: recordId,
            auto_boxes: payload.auto_boxes || [],
            manual_boxes: payload.manual_boxes || []
          }})
        }});
        const data = await resp.json().catch(() => ({{ ok: false, message: "服务返回异常" }}));
        if (!resp.ok || !data.ok) {{
          throw new Error(data.message || "保存结果失败");
        }}
        return data;
      }}

      function stripBoxes(boxes) {{
        if (!Array.isArray(boxes)) return [];
        return boxes
          .filter(b => b && typeof b === "object")
          .map(b => ({{ x: b.x, y: b.y, w: b.w, h: b.h, label: b.label, score: b.score }}));
      }}

      async function processFile(file, index, total) {{
        autoComputeOnLoad = false;
        try {{
          resultHint.textContent = "处理中：" + (index + 1) + "/" + total + " · " + file.name;
          const up = await uploadOne(file);
          currentRecordId = up.record_id;
          const imageUrl = up.image_url + "?ts=" + Date.now();

          img.src = imageUrl;
          img.style.display = "block";
          emptyHint.style.display = "none";
          autoBoxes = [];
          manualBoxes = [];
          drawBoxes();

          const tmp = new Image();
          tmp.src = imageUrl;
          await new Promise((resolve, reject) => {{
            tmp.onload = resolve;
            tmp.onerror = () => reject(new Error("图片加载失败"));
          }});

          const computed = computeAutoBoxesFromImage(tmp);
          autoBoxes = computed.map(b => ({{ ...b, color: "#dc2626" }}));
          drawBoxes();

          await saveResult(up.record_id, {{
            auto_boxes: computed,
            manual_boxes: manualBoxes
          }});

          const line = "[" + (index + 1) + "/" + total + "] " + file.name + " · 自动框=" + computed.length;
          progressList.textContent = (progressList.textContent ? (progressList.textContent + "\\n") : "") + line;
        }} finally {{
          autoComputeOnLoad = true;
        }}
      }}

      uploadBtn.addEventListener("click", async () => {{
        err.textContent = "";
        ok.textContent = "";
        progressList.textContent = "";
        const mode = detectModeEl.value || "single";
        const files = (fileEl.files ? Array.from(fileEl.files) : []);
        if (!files.length) {{
          err.textContent = (mode === "batch") ? "请先选择图片文件（可多选）" : "请先选择一张图片";
          updateDetectModeUI();
          return;
        }}

        if (mode !== "batch") {{
          ok.textContent = "开始检测…";
          try {{
            await processFile(files[0], 0, 1);
            ok.textContent = "检测完成（已保存到本地历史记录）";
            resultHint.textContent = "完成：1 张";
          }} catch (e) {{
            ok.textContent = "";
            err.textContent = (e && e.message) ? e.message : "处理失败";
          }}
          return;
        }}

        ok.textContent = "开始批量检测…";
        for (let i = 0; i < files.length; i += 1) {{
          try {{
            await processFile(files[i], i, files.length);
          }} catch (e) {{
            const msg = (e && e.message) ? e.message : "处理失败";
            const line = "[" + (i + 1) + "/" + files.length + "] " + files[i].name + " · 失败：" + msg;
            progressList.textContent = (progressList.textContent ? (progressList.textContent + "\\n") : "") + line;
          }}
        }}
        ok.textContent = "批量检测完成";
        resultHint.textContent = "完成：" + files.length + " 张";
      }});

      saveBtn.addEventListener("click", async () => {{
        err.textContent = "";
        ok.textContent = "";
        if (!currentRecordId) {{
          err.textContent = "当前没有可保存的记录，请先上传并检测";
          return;
        }}
        try {{
          await saveResult(currentRecordId, {{
            auto_boxes: stripBoxes(autoBoxes),
            manual_boxes: stripBoxes(manualBoxes)
          }});
          ok.textContent = "已保存到历史记录";
        }} catch (e) {{
          err.textContent = (e && e.message) ? e.message : "保存失败";
        }}
      }});
    </script>
    """
    return _base_page(title="离线检测 - 管道泄漏检测", body_html=body)


def ensure_storage() -> None:
    """确保存储目录与必要 JSON 文件存在。

    Args:
        None

    Returns:
        None
    """

    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    OFFLINE_HISTORY_DIR.mkdir(parents=True, exist_ok=True)
    if not STATUS_PATH.exists():
        STATUS_PATH.write_text(
            json.dumps(
                {
                    "has_image": False,
                    "updated_at": "",
                    "updated_unix_ms": 0,
                    "width": 0,
                    "height": 0,
                    "content_length": 0,
                    "temperature_c": None,
                    "humidity_rh": None,
                    "eddy_current_v": None,
                    "co_ppm": None,
                    "methane_ppm": None,
                    "pm25_ugm3": None,
                    "flame_level_pct": None,
                    "flame_detected": False,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
    if not AUTH_PATH.exists():
        AUTH_PATH.write_text(
            json.dumps({"users": {}, "sessions": {}}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    if not OFFLINE_HISTORY_PATH.exists():
        OFFLINE_HISTORY_PATH.write_text(
            json.dumps({"records": []}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def read_status() -> dict:
    """读取当前设备状态（最新图片元信息 + 传感器数据）。

    Args:
        None

    Returns:
        解析后的 status.json 字典。
    """

    ensure_storage()
    return json.loads(STATUS_PATH.read_text(encoding="utf-8"))


def write_status(width: int,
                 height: int,
                 content_length: int,
                 temperature_c: float | None,
                 humidity_rh: float | None,
                 eddy_current_v: float | None,
                 co_ppm: float | None,
                 methane_ppm: float | None,
                 pm25_ugm3: float | None,
                 flame_level_pct: float | None,
                 flame_detected: bool,
                 device_ip: str) -> dict:
    """写入设备状态，并返回最新 payload。

    Args:
        width: 图片宽度（像素）。
        height: 图片高度（像素）。
        content_length: 图片字节数。
        temperature_c: 温度（℃），可选。
        humidity_rh: 湿度（%RH），可选。
        eddy_current_v: 涡流传感器电压（V），可选。
        co_ppm: 一氧化碳（ppm），可选。
        methane_ppm: 甲烷（ppm），可选。
        pm25_ugm3: PM2.5（μg/m³），可选。
        flame_level_pct: 火焰强度百分比（0-100），可选。
        flame_detected: 是否检测到火焰。
        device_ip: ESP32-CAM 发起上传请求时使用的 IPv4 地址。

    Returns:
        写入到 status.json 的字典对象。
    """

    ensure_storage()
    now = datetime.now()
    payload = {
        "has_image": True,
        "updated_at": now.strftime("%Y-%m-%d %H:%M:%S"),
        "updated_unix_ms": int(now.timestamp() * 1000),
        "width": width,
        "height": height,
        "content_length": content_length,
        "temperature_c": temperature_c,
        "humidity_rh": humidity_rh,
        "eddy_current_v": eddy_current_v,
        "co_ppm": co_ppm,
        "methane_ppm": methane_ppm,
        "pm25_ugm3": pm25_ugm3,
        "flame_level_pct": flame_level_pct,
        "flame_detected": flame_detected,
        "device_ip": device_ip,
    }
    STATUS_PATH.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return payload


def _load_auth_state() -> dict:
    """加载账户与会话状态（auth.json）。

    Args:
        None

    Returns:
        auth 状态字典，结构包含 users 与 sessions。
    """

    ensure_storage()
    try:
        raw = AUTH_PATH.read_text(encoding="utf-8")
        state = json.loads(raw)
    except Exception:
        state = {"users": {}, "sessions": {}}
    if not isinstance(state, dict):
        state = {"users": {}, "sessions": {}}
    state.setdefault("users", {})
    state.setdefault("sessions", {})
    return state


def _save_auth_state(state: dict) -> None:
    """保存账户与会话状态到 auth.json。

    Args:
        state: auth 状态字典（包含 users 与 sessions）。

    Returns:
        None
    """

    ensure_storage()
    AUTH_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def _pbkdf2_hash_password(*, password: str, salt_b64: str | None = None, iterations: int = 200_000) -> dict:
    """对密码进行 PBKDF2-HMAC-SHA256 哈希。

    Args:
        password: 明文密码。
        salt_b64: 可选的 base64 盐；不传则自动生成 16 字节随机盐。
        iterations: PBKDF2 迭代次数（越大越安全但越慢）。

    Returns:
        包含 salt_b64、iterations、hash_b64 的字典，可直接落盘保存。
    """

    if salt_b64 is None:
        salt = secrets.token_bytes(16)
        salt_b64 = base64.b64encode(salt).decode("ascii")
    else:
        salt = base64.b64decode(salt_b64.encode("ascii"))
    dk = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, iterations, dklen=32)
    return {
        "salt_b64": salt_b64,
        "iterations": int(iterations),
        "hash_b64": base64.b64encode(dk).decode("ascii"),
    }


def _verify_password(*, password: str, stored: dict) -> bool:
    """校验明文密码是否匹配已保存的哈希记录。

    Args:
        password: 明文密码。
        stored: 已保存的用户记录字典（应含 salt_b64、iterations、hash_b64）。

    Returns:
        True 表示匹配，False 表示不匹配或记录不完整。
    """

    try:
        salt_b64 = stored["salt_b64"]
        iterations = int(stored["iterations"])
        expected = base64.b64decode(stored["hash_b64"].encode("ascii"))
    except Exception:
        return False
    computed = _pbkdf2_hash_password(password=password, salt_b64=salt_b64, iterations=iterations)
    got = base64.b64decode(computed["hash_b64"].encode("ascii"))
    return hmac.compare_digest(expected, got)


def _create_session(*, username: str, ttl_seconds: int = 24 * 3600) -> tuple[str, dict]:
    """创建会话 token，并返回会话记录。

    Args:
        username: 用户名。
        ttl_seconds: 会话有效期（秒），可选，默认 24 小时。

    Returns:
        (token, session_record) 元组；session_record 包含 username 与 expires_unix_ms。
    """

    token = secrets.token_urlsafe(32)
    expires_unix_ms = int((time.time() + ttl_seconds) * 1000)
    return token, {"username": username, "expires_unix_ms": expires_unix_ms}


def _cleanup_sessions(*, state: dict, now_unix_ms: int | None = None) -> None:
    """清理过期会话。

    Args:
        state: auth 状态字典（会修改其中 sessions）。
        now_unix_ms: 可选的当前时间戳（毫秒）；不传则使用 time.time()。

    Returns:
        None
    """

    if now_unix_ms is None:
        now_unix_ms = int(time.time() * 1000)
    sessions = state.get("sessions", {})
    if not isinstance(sessions, dict):
        state["sessions"] = {}
        return
    expired_tokens = []
    for token, info in sessions.items():
        try:
            if int(info.get("expires_unix_ms", 0)) <= now_unix_ms:
                expired_tokens.append(token)
        except Exception:
            expired_tokens.append(token)
    for token in expired_tokens:
        sessions.pop(token, None)


def _load_offline_history() -> dict:
    """加载离线检测历史记录（offline_history.json）。

    Args:
        None

    Returns:
        历史记录字典，包含 records 列表。
    """

    ensure_storage()
    try:
        raw = OFFLINE_HISTORY_PATH.read_text(encoding="utf-8")
        data = json.loads(raw)
    except Exception:
        data = {"records": []}
    if not isinstance(data, dict):
        data = {"records": []}
    if not isinstance(data.get("records"), list):
        data["records"] = []
    return data


def _save_offline_history(data: dict) -> None:
    """保存离线检测历史记录到 offline_history.json。

    Args:
        data: 历史记录字典（包含 records 列表）。

    Returns:
        None
    """

    ensure_storage()
    OFFLINE_HISTORY_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _new_record_id() -> str:
    """生成新的历史记录 ID。

    Args:
        None

    Returns:
        记录 ID 字符串。
    """

    return secrets.token_urlsafe(12).replace("-", "_")


def _guess_extension_from_content_type(content_type: str) -> str:
    """根据 Content-Type 推测文件扩展名。

    Args:
        content_type: HTTP Content-Type。

    Returns:
        扩展名（含点），未知则返回 .bin。
    """

    ct = (content_type or "").lower()
    if "png" in ct:
        return ".png"
    if "jpeg" in ct or "jpg" in ct:
        return ".jpg"
    return ".bin"


def _add_history_record(*, record: dict) -> None:
    """新增一条离线历史记录。

    Args:
        record: 记录字典。

    Returns:
        None
    """

    data = _load_offline_history()
    records = data.get("records", [])
    records.append(record)
    data["records"] = records
    _save_offline_history(data)


def _update_history_record(*, record_id: str, updates: dict) -> dict | None:
    """更新历史记录字段（按 record_id 匹配）。

    Args:
        record_id: 记录 ID。
        updates: 要更新的字段字典。

    Returns:
        更新后的记录；未找到返回 None。
    """

    data = _load_offline_history()
    records = data.get("records", [])
    for i, rec in enumerate(records):
        if not isinstance(rec, dict):
            continue
        if rec.get("record_id") != record_id:
            continue
        rec.update(updates)
        records[i] = rec
        data["records"] = records
        _save_offline_history(data)
        return rec
    return None


def _get_history_record(*, record_id: str) -> dict | None:
    """获取单条历史记录。

    Args:
        record_id: 记录 ID。

    Returns:
        记录字典；未找到返回 None。
    """

    data = _load_offline_history()
    for rec in data.get("records", []):
        if isinstance(rec, dict) and rec.get("record_id") == record_id:
            return rec
    return None


def _history_page(*, username: str) -> str:
    """生成历史记录列表页面。

    Args:
        username: 当前登录用户名。

    Returns:
        历史记录页面 HTML。
    """

    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">离线检测历史记录</h1>
        <p class="brand-sub">已登录：{username} · <a href="/offline">返回离线检测</a></p>
      </div>
      <div class="badge">History</div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>记录列表</h2>
      <div class="hint">点击任意记录查看详情（图片+框选回放）。</div>
      <div id="list" class="hint" style="margin-top: 12px;"></div>
    </section>

    <script>
      const list = document.getElementById("list");
      function esc(s) {{
        return String(s).replace(/[&<>"']/g, (c) => ({{"&":"&amp;","<":"&lt;",">":"&gt;","\\"":"&quot;","'":"&#39;"}}[c]));
      }}
      async function load() {{
        list.textContent = "加载中…";
        const resp = await fetch("/api/offline/history?ts=" + Date.now(), {{ cache: "no-store" }});
        const data = await resp.json().catch(() => ({{ ok: false, records: [] }}));
        if (!resp.ok || !data.ok) {{
          list.textContent = "加载失败";
          return;
        }}
        const rows = (data.records || []).map(r => {{
          const count = (r.auto_boxes ? r.auto_boxes.length : 0) + (r.manual_boxes ? r.manual_boxes.length : 0);
          return `<div style="padding:10px 0;border-top:1px solid rgba(17,24,39,0.08);">
            <a href="/history/view?id=${{encodeURIComponent(r.record_id)}}">${{esc(r.created_at || "")}} · ${{esc(r.filename || "")}} · 框=${{count}}</a>
          </div>`;
        }});
        list.innerHTML = rows.length ? rows.join("") : "<div>暂无历史记录</div>";
      }}
      load();
    </script>
    """
    return _base_page(title="历史记录 - 离线检测", body_html=body)


def _history_view_page(*, username: str, record_id: str) -> str:
    """生成历史记录详情回放页面（图片叠加框）。

    Args:
        username: 当前登录用户名。
        record_id: 记录 ID。

    Returns:
        详情页面 HTML。
    """

    body = f"""
    <section class="topbar">
      <div class="brand">
        <h1 class="brand-title">历史详情回放</h1>
        <p class="brand-sub">已登录：{username} · <a href="/history">返回列表</a></p>
      </div>
      <div class="badge" id="meta">记录：{record_id}</div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <h2>图片与框选</h2>
      <div class="image-shell" id="imageShell">
        <img id="img" alt="历史图片">
        <canvas id="overlay"></canvas>
        <div class="hint" id="emptyHint">加载中…</div>
      </div>
    </section>

    <script>
      const recordId = {json.dumps(record_id)};
      const img = document.getElementById("img");
      const overlay = document.getElementById("overlay");
      const shell = document.getElementById("imageShell");
      const emptyHint = document.getElementById("emptyHint");
      const meta = document.getElementById("meta");
      let boxes = [];

      function canvasSizeToMatch() {{
        const rect = shell.getBoundingClientRect();
        overlay.width = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
        overlay.height = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
      }}

      function drawBoxes() {{
        canvasSizeToMatch();
        const ctx = overlay.getContext("2d");
        ctx.setTransform(window.devicePixelRatio, 0, 0, window.devicePixelRatio, 0, 0);
        ctx.clearRect(0, 0, overlay.width, overlay.height);
        if (!img.src || img.style.display === "none") return;

        const imgRect = img.getBoundingClientRect();
        const shellRect = shell.getBoundingClientRect();
        const x0 = imgRect.left - shellRect.left;
        const y0 = imgRect.top - shellRect.top;
        const w = imgRect.width;
        const h = imgRect.height;

        for (const b of boxes) {{
          const bx = x0 + b.x * w;
          const by = y0 + b.y * h;
          const bw = b.w * w;
          const bh = b.h * h;
          ctx.lineWidth = 2;
          ctx.strokeStyle = b.color || "#dc2626";
          ctx.strokeRect(bx, by, bw, bh);
          ctx.fillStyle = b.color || "#dc2626";
          ctx.font = "12px Segoe UI, Arial, sans-serif";
          ctx.fillText(b.label || "框", bx + 4, Math.max(12, by - 6));
        }}
      }}

      window.addEventListener("resize", drawBoxes);
      img.addEventListener("load", () => {{
        drawBoxes();
        emptyHint.style.display = "none";
      }});

      async function load() {{
        const resp = await fetch("/api/offline/record?id=" + encodeURIComponent(recordId) + "&ts=" + Date.now(), {{ cache: "no-store" }});
        const data = await resp.json().catch(() => ({{ ok: false }}));
        if (!resp.ok || !data.ok) {{
          emptyHint.textContent = "加载失败";
          return;
        }}
        meta.textContent = (data.created_at || "") + " · " + (data.filename || "") + " · 记录：" + recordId;
        img.src = data.image_url + "?ts=" + Date.now();
        img.style.display = "block";
        boxes = [];
        for (const b of (data.auto_boxes || [])) {{
          boxes.push({{ ...b, color: "#dc2626", label: b.label || "疑似破损/泄漏" }});
        }}
        for (const b of (data.manual_boxes || [])) {{
          boxes.push({{ ...b, color: "#2563eb", label: b.label || "手动框" }});
        }}
        drawBoxes();
      }}
      load();
    </script>
    """
    return _base_page(title="历史详情 - 离线检测", body_html=body)

def _placeholder_pipe_damage_boxes(*, seed: int | None = None) -> list[dict]:
    """生成占位检测框（归一化坐标 0~1）。

    Args:
        seed: 可选随机种子，用于让框的位置随时间略有变化。

    Returns:
        检测框列表，每个框包含 x,y,w,h,label,score（均为可序列化类型）。
    """

    if seed is None:
        seed = 0
    r = (seed * 1103515245 + 12345) & 0x7FFFFFFF
    if (r % 3) != 0:
        return []
    dx = ((r % 1000) / 1000.0 - 0.5) * 0.06
    dy = (((r // 1000) % 1000) / 1000.0 - 0.5) * 0.06
    x = min(0.75, max(0.05, 0.35 + dx))
    y = min(0.75, max(0.05, 0.38 + dy))
    return [
        {"x": x, "y": y, "w": 0.28, "h": 0.18, "label": "疑似破损/泄漏", "score": 0.50},
    ]


def _device_online(*, status: dict, online_window_ms: int = 12_000) -> bool:
    """根据 status.json 判断设备是否在线。

    Args:
        status: status.json 字典。
        online_window_ms: 判定窗口（毫秒）；在此窗口内更新过则视为在线。

    Returns:
        True 表示在线，False 表示不在线或无数据。
    """

    try:
        updated = int(status.get("updated_unix_ms", 0))
    except Exception:
        return False
    if updated <= 0:
        return False
    now_ms = int(time.time() * 1000)
    return (now_ms - updated) <= int(online_window_ms)


class CameraRequestHandler(BaseHTTPRequestHandler):
    server_version = "ESP32CamWeb/1.0"

    def do_GET(self) -> None:
        """处理 GET 请求路由分发。

        Args:
            None

        Returns:
            None
        """

        route = urlparse(self.path).path

        if route in {"/", "/login", "/register", "/app", "/online"}:
            self._send_html(_live_stream_page())
            return

        if route == "/offline":
            username = self._require_auth_page()
            if username is None:
                return
            self._send_html(_offline_page(username=username))
            return

        if route == "/history":
            username = self._require_auth_page()
            if username is None:
                return
            self._send_html(_history_page(username=username))
            return

        if route == "/history/view":
            username = self._require_auth_page()
            if username is None:
                return
            record_id = self._get_query_param("id") or ""
            if not record_id:
                self.send_error(HTTPStatus.BAD_REQUEST, "missing id")
                return
            self._send_html(_history_view_page(username=username, record_id=record_id))
            return

        if route == "/stream.mjpg":
            self._send_live_stream()
            return

        if route == "/latest.jpg":
            if self._require_auth_api() is None:
                return
            self._send_latest_image()
            return

        if route == "/status.json":
            if self._require_auth_api() is None:
                return
            self._send_status()
            return

        if route == "/offline.jpg":
            if self._require_auth_api() is None:
                return
            self._send_offline_image()
            return

        if route == "/api/device_status":
            if self._require_auth_api() is None:
                return
            self._send_device_status()
            return

        if route == "/api/detect/latest":
            if self._require_auth_api() is None:
                return
            self._send_detect_latest()
            return

        if route == "/api/offline/history":
            if self._require_auth_api() is None:
                return
            self._send_offline_history()
            return

        if route == "/api/offline/record":
            if self._require_auth_api() is None:
                return
            self._send_offline_record()
            return

        if route.startswith("/history/image/"):
            if self._require_auth_api() is None:
                return
            record_id = route.removeprefix("/history/image/").strip("/")
            self._send_history_image(record_id=record_id)
            return

        self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

    def do_POST(self) -> None:
        """处理 POST 请求路由分发。

        Args:
            None

        Returns:
            None
        """

        route = urlparse(self.path).path

        if route == "/api/register":
            self._handle_register()
            return

        if route == "/api/login":
            self._handle_login()
            return

        if route == "/api/logout":
            if self._require_auth_api() is None:
                return
            self._handle_logout()
            return

        if route == "/api/detect/upload":
            if self._require_auth_api() is None:
                return
            self._handle_detect_upload()
            return

        if route == "/api/offline/upload":
            if self._require_auth_api() is None:
                return
            self._handle_offline_upload()
            return

        if route == "/api/offline/save_result":
            if self._require_auth_api() is None:
                return
            self._handle_offline_save_result()
            return

        if route != "/upload":
            self.send_error(HTTPStatus.NOT_FOUND, "Not Found")
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            self.send_error(HTTPStatus.BAD_REQUEST, "Empty body")
            return

        temperature_c = self._parse_header_float("X-Temperature-C")
        humidity_rh = self._parse_header_float("X-Humidity-RH")
        eddy_current_v = self._parse_header_float("X-Eddy-Current-V")
        co_ppm = self._parse_header_float("X-CO-PPM")
        methane_ppm = self._parse_header_float("X-Methane-PPM")
        pm25_ugm3 = self._parse_header_float("X-PM25-UGM3")
        flame_level_pct = self._parse_header_float("X-Flame-Level")
        flame_detected = self._parse_header_bool("X-Flame-Detected")
        width = self._parse_header_int("X-Image-Width")
        height = self._parse_header_int("X-Image-Height")
        body = self.rfile.read(content_length)

        with STATE_LOCK:
            ensure_storage()
            LATEST_IMAGE_PATH.write_bytes(body)
            status = write_status(width=width,
                                  height=height,
                                  content_length=content_length,
                                  temperature_c=temperature_c,
                                  humidity_rh=humidity_rh,
                                  eddy_current_v=eddy_current_v,
                                  co_ppm=co_ppm,
                                  methane_ppm=methane_ppm,
                                  pm25_ugm3=pm25_ugm3,
                                  flame_level_pct=flame_level_pct,
                                  flame_detected=flame_detected,
                                  device_ip=self.client_address[0])

        response = {
            "ok": True,
            "message": "image uploaded",
            "saved_to": str(LATEST_IMAGE_PATH),
            "status": status,
        }
        self._send_json(response)

    def log_message(self, format: str, *args) -> None:
        print("%s - - [%s] %s" % (self.client_address[0], self.log_date_time_string(), format % args))

    def _send_html(self, html: str) -> None:
        """发送 HTML 响应。

        Args:
            html: HTML 字符串。

        Returns:
            None
        """

        body = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload: dict, status: int = 200) -> None:
        """发送 JSON 响应。

        Args:
            payload: 可 JSON 序列化的字典对象。
            status: HTTP 状态码，可选，默认 200。

        Returns:
            None
        """

        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_redirect(self, location: str) -> None:
        """发送 303 重定向响应。

        Args:
            location: 跳转目标路径（例如 /login）。

        Returns:
            None
        """

        body = b""
        self.send_response(HTTPStatus.SEE_OTHER)
        self.send_header("Location", location)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self) -> dict | None:
        """读取并解析 JSON 请求体。

        Args:
            None

        Returns:
            解析后的字典；解析失败返回 None。
        """

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return None
        if length <= 0:
            return None
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            return None
        if not isinstance(data, dict):
            return None
        return data

    def _get_query_param(self, name: str) -> str | None:
        """从当前请求 URL 中读取 query 参数。

        Args:
            name: 参数名。

        Returns:
            参数值；不存在返回 None。
        """

        query = urlparse(self.path).query
        params = parse_qs(query, keep_blank_values=True)
        values = params.get(name)
        if not values:
            return None
        value = values[0]
        if not isinstance(value, str):
            return None
        return value

    def _send_latest_image(self) -> None:
        """发送设备最新图片（latest.jpg）。"""

        ensure_storage()
        if not LATEST_IMAGE_PATH.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "No image uploaded yet")
            return

        data = LATEST_IMAGE_PATH.read_bytes()

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_live_stream(self) -> None:
        """优先跳转到 ESP32 原生流，缺少设备 IP 时输出电脑端备用 MJPEG 流。

        Args:
            None

        Returns:
            None。跳转响应完成或客户端断开备用流后结束当前请求线程。
        """

        with STATE_LOCK:
            status = read_status()

        device_ip = str(status.get("device_ip", "")).strip()
        try:
            parsed_ip = ipaddress.ip_address(device_ip)
        except ValueError:
            parsed_ip = None

        if parsed_ip is not None and parsed_ip.version == 4:
            self._send_redirect(f"http://{parsed_ip}:81/stream")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_updated_unix_ms = -1
        try:
            while True:
                frame = None
                updated_unix_ms = last_updated_unix_ms

                with STATE_LOCK:
                    status = read_status()
                    updated_unix_ms = int(status.get("updated_unix_ms", 0) or 0)
                    if updated_unix_ms != last_updated_unix_ms and LATEST_IMAGE_PATH.exists():
                        frame = LATEST_IMAGE_PATH.read_bytes()

                if not frame:
                    time.sleep(0.02)
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                last_updated_unix_ms = updated_unix_ms
        except OSError:
            return

    def _send_offline_image(self) -> None:
        """发送离线上传的最新图片（offline_latest.jpg）。"""

        ensure_storage()
        if not OFFLINE_IMAGE_PATH.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "No offline image uploaded yet")
            return

        data = OFFLINE_IMAGE_PATH.read_bytes()
        content_type = "image/jpeg"
        if data[:8] == b"\x89PNG\r\n\x1a\n":
            content_type = "image/png"

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_status(self) -> None:
        """发送设备状态 JSON（status.json 内容）。"""

        with STATE_LOCK:
            status = read_status()

        body = json.dumps(status, ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_device_status(self) -> None:
        """发送设备在线/离线简要状态（用于前端徽标）。"""

        with STATE_LOCK:
            status = read_status()
        online = _device_online(status=status)
        payload = {
            "ok": True,
            "online": online,
            "has_image": bool(status.get("has_image")),
            "device_ip": status.get("device_ip", ""),
            "updated_at": status.get("updated_at", ""),
            "updated_unix_ms": int(status.get("updated_unix_ms", 0) or 0),
            "message": "最近更新 " + status.get("updated_at", "") if status.get("updated_at") else ("在线" if online else "离线"),
        }
        self._send_json(payload)

    def _send_detect_latest(self) -> None:
        """返回设备最新图的检测框（占位检测）。"""

        with STATE_LOCK:
            status = read_status()
        seed = int(status.get("updated_unix_ms", 0) or 0)
        payload = {"ok": True, "source": "device_latest", "boxes": _placeholder_pipe_damage_boxes(seed=seed)}
        self._send_json(payload)

    def _handle_detect_upload(self) -> None:
        """处理离线图片上传并返回检测框。

        当前实现：
          - 服务端保存图片到 offline_latest.jpg
          - 同时创建一条离线历史记录并保存原图到 offline_history/
          - 检测框由前端基于照片内容计算并叠加显示；可通过 /api/offline/save_result 写回历史记录

        Query:
          - filename: 可选，原始文件名（用于历史列表展示）。
        """

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_json({"ok": False, "message": "无效 Content-Length"}, status=400)
            return
        if content_length <= 0:
            self._send_json({"ok": False, "message": "空请求体"}, status=400)
            return
        if content_length > 8 * 1024 * 1024:
            self._send_json({"ok": False, "message": "图片过大（>8MB）"}, status=400)
            return

        filename = self._get_query_param("filename") or "image"
        content_type = self.headers.get("Content-Type", "application/octet-stream")
        record_id = _new_record_id()
        safe_name = "".join(ch for ch in filename if ch.isalnum() or ch in {"-", "_", ".", " "}).strip()
        if not safe_name:
            safe_name = "image"
        suffix = Path(safe_name).suffix.lower()
        ext = "" if suffix in {".jpg", ".jpeg", ".png"} else _guess_extension_from_content_type(content_type)
        stored_name = f"{record_id}_{safe_name}{ext}"
        image_path = OFFLINE_HISTORY_DIR / stored_name

        body = self.rfile.read(content_length)
        now = datetime.now()
        now_unix_ms = int(now.timestamp() * 1000)
        with STATE_LOCK:
            ensure_storage()
            OFFLINE_IMAGE_PATH.write_bytes(body)
            image_path.write_bytes(body)
            _add_history_record(
                record={
                    "record_id": record_id,
                    "created_at": now.strftime("%Y-%m-%d %H:%M:%S"),
                    "created_unix_ms": now_unix_ms,
                    "filename": filename,
                    "stored_name": stored_name,
                    "content_type": content_type,
                    "content_length": int(content_length),
                    "auto_boxes": [],
                    "manual_boxes": [],
                    "updated_at": now.strftime("%Y-%m-%d %H:%M:%S"),
                }
            )

        self._send_json(
            {
                "ok": True,
                "record_id": record_id,
                "image_url": f"/history/image/{record_id}",
                "boxes": [],
                "detection": "client",
            }
        )

    def _handle_offline_upload(self) -> None:
        """处理离线批量上传：保存图片并创建一条历史记录。

        请求：
          - POST Body: 图片二进制（Content-Type 建议为 image/jpeg 或 image/png）
          - Query: filename（可选）

        返回：
          - JSON: {ok, record_id, image_url}
        """

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_json({"ok": False, "message": "无效 Content-Length"}, status=400)
            return
        if content_length <= 0:
            self._send_json({"ok": False, "message": "空请求体"}, status=400)
            return
        if content_length > 8 * 1024 * 1024:
            self._send_json({"ok": False, "message": "图片过大（>8MB）"}, status=400)
            return

        filename = self._get_query_param("filename") or "image"
        content_type = self.headers.get("Content-Type", "application/octet-stream")
        record_id = _new_record_id()
        safe_name = "".join(ch for ch in filename if ch.isalnum() or ch in {"-", "_", ".", " "}).strip()
        if not safe_name:
            safe_name = "image"
        suffix = Path(safe_name).suffix.lower()
        ext = "" if suffix in {".jpg", ".jpeg", ".png"} else _guess_extension_from_content_type(content_type)
        stored_name = f"{record_id}_{safe_name}{ext}"
        image_path = OFFLINE_HISTORY_DIR / stored_name

        body = self.rfile.read(content_length)
        now = datetime.now()
        now_unix_ms = int(now.timestamp() * 1000)

        with STATE_LOCK:
            ensure_storage()
            image_path.write_bytes(body)
            OFFLINE_IMAGE_PATH.write_bytes(body)
            _add_history_record(
                record={
                    "record_id": record_id,
                    "created_at": now.strftime("%Y-%m-%d %H:%M:%S"),
                    "created_unix_ms": now_unix_ms,
                    "filename": filename,
                    "stored_name": stored_name,
                    "content_type": content_type,
                    "content_length": int(content_length),
                    "auto_boxes": [],
                    "manual_boxes": [],
                }
            )

        self._send_json({"ok": True, "record_id": record_id, "image_url": f"/history/image/{record_id}"})

    def _handle_offline_save_result(self) -> None:
        """保存离线检测结果到历史记录。

        请求体：
          - JSON: {record_id, auto_boxes, manual_boxes}
        """

        data = self._read_json_body()
        if data is None:
            self._send_json({"ok": False, "message": "请求体必须为 JSON"}, status=400)
            return
        record_id = str(data.get("record_id", "")).strip()
        if not record_id:
            self._send_json({"ok": False, "message": "缺少 record_id"}, status=400)
            return

        def normalize_boxes(raw: object) -> list[dict]:
            """规范化前端传入的框列表，过滤无效字段。

            Args:
                raw: 前端传入的 boxes（期望为 list[dict]）。

            Returns:
                规范化后的 boxes 列表。
            """

            if not isinstance(raw, list):
                return []
            out: list[dict] = []
            for item in raw:
                if not isinstance(item, dict):
                    continue
                try:
                    x = float(item.get("x", 0))
                    y = float(item.get("y", 0))
                    w = float(item.get("w", 0))
                    h = float(item.get("h", 0))
                except (TypeError, ValueError):
                    continue
                if w <= 0 or h <= 0:
                    continue
                out.append(
                    {
                        "x": max(0.0, min(1.0, x)),
                        "y": max(0.0, min(1.0, y)),
                        "w": max(0.0, min(1.0, w)),
                        "h": max(0.0, min(1.0, h)),
                        "label": str(item.get("label", "疑似破损/泄漏"))[:64],
                        "score": float(item.get("score", 0.0)) if item.get("score") is not None else 0.0,
                    }
                )
            return out

        auto_boxes = normalize_boxes(data.get("auto_boxes"))
        manual_boxes = normalize_boxes(data.get("manual_boxes"))

        with STATE_LOCK:
            rec = _update_history_record(
                record_id=record_id,
                updates={
                    "auto_boxes": auto_boxes,
                    "manual_boxes": manual_boxes,
                    "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                },
            )
        if rec is None:
            self._send_json({"ok": False, "message": "record_id 不存在"}, status=404)
            return

        self._send_json({"ok": True, "record_id": record_id})

    def _send_offline_history(self) -> None:
        """发送离线历史记录列表（按时间倒序）。"""

        try:
            limit = int(self._get_query_param("limit") or "50")
        except ValueError:
            limit = 50
        limit = max(1, min(200, limit))

        with STATE_LOCK:
            data = _load_offline_history()
        records = [r for r in data.get("records", []) if isinstance(r, dict)]
        records.sort(key=lambda r: int(r.get("created_unix_ms", 0) or 0), reverse=True)
        out = []
        for r in records[:limit]:
            out.append(
                {
                    "record_id": r.get("record_id", ""),
                    "created_at": r.get("created_at", ""),
                    "filename": r.get("filename", ""),
                    "auto_boxes": r.get("auto_boxes", []) if isinstance(r.get("auto_boxes"), list) else [],
                    "manual_boxes": r.get("manual_boxes", []) if isinstance(r.get("manual_boxes"), list) else [],
                }
            )
        self._send_json({"ok": True, "records": out})

    def _send_offline_record(self) -> None:
        """发送单条离线历史记录详情。"""

        record_id = self._get_query_param("id") or ""
        if not record_id:
            self._send_json({"ok": False, "message": "缺少 id"}, status=400)
            return
        with STATE_LOCK:
            rec = _get_history_record(record_id=record_id)
        if rec is None:
            self._send_json({"ok": False, "message": "记录不存在"}, status=404)
            return
        self._send_json(
            {
                "ok": True,
                "record_id": rec.get("record_id", ""),
                "created_at": rec.get("created_at", ""),
                "filename": rec.get("filename", ""),
                "auto_boxes": rec.get("auto_boxes", []) if isinstance(rec.get("auto_boxes"), list) else [],
                "manual_boxes": rec.get("manual_boxes", []) if isinstance(rec.get("manual_boxes"), list) else [],
                "image_url": f"/history/image/{record_id}",
            }
        )

    def _send_history_image(self, *, record_id: str) -> None:
        """发送指定历史记录的原始图片。

        Args:
            record_id: 记录 ID。

        Returns:
            None
        """

        with STATE_LOCK:
            rec = _get_history_record(record_id=record_id)
        if rec is None:
            self.send_error(HTTPStatus.NOT_FOUND, "Record not found")
            return
        stored_name = rec.get("stored_name")
        if not isinstance(stored_name, str) or not stored_name:
            self.send_error(HTTPStatus.NOT_FOUND, "Image not found")
            return

        image_path = OFFLINE_HISTORY_DIR / stored_name
        if not image_path.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "Image not found")
            return

        data = image_path.read_bytes()
        content_type = rec.get("content_type", "application/octet-stream")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _handle_register(self) -> None:
        """处理注册请求。

        请求体：
          - JSON: {username, password}
        """

        data = self._read_json_body()
        if data is None:
            self._send_json({"ok": False, "message": "请求体必须为 JSON"}, status=400)
            return
        username = str(data.get("username", "")).strip()
        password = str(data.get("password", ""))
        if not username or not password:
            self._send_json({"ok": False, "message": "用户名和密码不能为空"}, status=400)
            return
        if len(username) > 32:
            self._send_json({"ok": False, "message": "用户名过长（最多 32 字符）"}, status=400)
            return
        if len(password) < 4:
            self._send_json({"ok": False, "message": "密码过短（至少 4 位）"}, status=400)
            return

        with STATE_LOCK:
            state = _load_auth_state()
            _cleanup_sessions(state=state)
            users = state.get("users", {})
            if username in users:
                self._send_json({"ok": False, "message": "用户名已存在"}, status=400)
                return
            users[username] = {
                **_pbkdf2_hash_password(password=password),
                "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
            _save_auth_state(state)

        self._send_json({"ok": True, "message": "registered"})

    def _handle_login(self) -> None:
        """处理登录请求，并写入会话 Cookie。

        请求体：
          - JSON: {username, password}
        """

        data = self._read_json_body()
        if data is None:
            self._send_json({"ok": False, "message": "请求体必须为 JSON"}, status=400)
            return
        username = str(data.get("username", "")).strip()
        password = str(data.get("password", ""))
        if not username or not password:
            self._send_json({"ok": False, "message": "用户名和密码不能为空"}, status=400)
            return

        with STATE_LOCK:
            state = _load_auth_state()
            _cleanup_sessions(state=state)
            user_record = state.get("users", {}).get(username)
            if not isinstance(user_record, dict) or not _verify_password(password=password, stored=user_record):
                self._send_json({"ok": False, "message": "用户名或密码错误"}, status=401)
                return
            token, session_record = _create_session(username=username)
            state.get("sessions", {})[token] = session_record
            _save_auth_state(state)

        body = json.dumps({"ok": True, "message": "logged_in"}, ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Set-Cookie", f"{SESSION_COOKIE_NAME}={token}; Path=/; HttpOnly; SameSite=Lax")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_logout(self) -> None:
        """处理退出登录请求，并清除会话 Cookie。"""

        token = self._get_session_token()
        if token:
            with STATE_LOCK:
                state = _load_auth_state()
                _cleanup_sessions(state=state)
                state.get("sessions", {}).pop(token, None)
                _save_auth_state(state)
        body = json.dumps({"ok": True}, ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Set-Cookie", f"{SESSION_COOKIE_NAME}=; Path=/; Max-Age=0; HttpOnly; SameSite=Lax")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _get_session_token(self) -> str | None:
        """从 Cookie 读取会话 token。

        Args:
            None

        Returns:
            token 字符串；不存在返回 None。
        """

        cookie = self.headers.get("Cookie", "")
        parts = [p.strip() for p in cookie.split(";") if p.strip()]
        for p in parts:
            if "=" not in p:
                continue
            k, v = p.split("=", 1)
            if k.strip() == SESSION_COOKIE_NAME:
                return v.strip()
        return None

    def _get_current_user(self) -> str | None:
        """根据会话 token 获取当前用户名。

        Args:
            None

        Returns:
            用户名；未登录或会话过期返回 None。
        """

        token = self._get_session_token()
        if not token:
            return None
        with STATE_LOCK:
            state = _load_auth_state()
            _cleanup_sessions(state=state)
            session = state.get("sessions", {}).get(token)
            if not isinstance(session, dict):
                return None
            username = session.get("username")
            if not isinstance(username, str) or not username:
                return None
        return username

    def _require_auth_page(self) -> str | None:
        """页面路由鉴权：未登录则跳转到 /login。

        Args:
            None

        Returns:
            已登录返回用户名；未登录返回 None（已完成重定向）。
        """

        username = self._get_current_user()
        if username is None:
            self._send_redirect("/login")
            return None
        return username

    def _require_auth_api(self) -> str | None:
        """API 路由鉴权：未登录返回 401 JSON。

        Args:
            None

        Returns:
            已登录返回用户名；未登录返回 None（已完成响应）。
        """

        username = self._get_current_user()
        if username is None:
            self._send_json({"ok": False, "message": "未登录"}, status=401)
            return None
        return username

    def _parse_header_int(self, header_name: str) -> int:
        """解析整型 Header。

        Args:
            header_name: Header 名称。

        Returns:
            解析成功返回 int，失败返回 0。
        """

        raw_value = self.headers.get(header_name, "0")
        try:
            return int(raw_value)
        except ValueError:
            return 0

    def _parse_header_float(self, header_name: str) -> float | None:
        """解析浮点型 Header。

        Args:
            header_name: Header 名称。

        Returns:
            解析成功返回 float；Header 缺失或解析失败返回 None。
        """

        raw_value = self.headers.get(header_name)
        if raw_value is None or raw_value == "":
            return None
        try:
            return float(raw_value)
        except ValueError:
            return None

    def _parse_header_bool(self, header_name: str) -> bool:
        """解析布尔型 Header。

        Args:
            header_name: Header 名称。

        Returns:
            True/False。
        """

        raw_value = self.headers.get(header_name, "0")
        return raw_value in {"1", "true", "True", "TRUE", "yes", "YES"}


def main() -> None:
    """程序入口：启动 ThreadingHTTPServer。

    Args:
        None

    Returns:
        None
    """

    parser = argparse.ArgumentParser(description="ESP32-CAM 图片接收与网页展示服务")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    ensure_storage()
    server = ThreadingHTTPServer((args.host, args.port), CameraRequestHandler)
    print(f"Server running at http://127.0.0.1:{args.port}/")
    print("Upload endpoint: /upload")
    server.serve_forever()


if __name__ == "__main__":
    main()
