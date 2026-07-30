# ESP32-CAM 网页视频与录制使用说明

本说明适用于不启动 `ESP32_CAM_Desktop.exe` 上位机、仅使用 Python Web 服务和浏览器查看及录制 ESP32-CAM 视频的场景。

## 一、使用前准备

1. 确认 ESP32-CAM 已烧录当前项目的最新固件。
2. 电脑安装 Python 3，并能在 PowerShell 中执行：

   ```powershell
   python --version
   ```

3. 使用 USB 串口连接 ESP32-CAM。示例端口为 `COM61`，实际使用时以设备管理器显示的端口为准。
4. 电脑先连接 ESP32 即将使用的目标 Wi-Fi。
5. 关闭上位机 EXE、其他串口监视器以及正在访问 ESP32 视频流的其他浏览器页面，避免串口或视频流被占用。

## 二、查询电脑在目标 Wi-Fi 下的 IP

在 PowerShell 中执行：

```powershell
ipconfig
```

找到目标无线网卡的“IPv4 地址”。例如：

```text
192.168.5.61
```

后续配置上传地址时需要使用该地址。不要填写 `127.0.0.1`，因为 ESP32 无法通过该地址访问电脑。

## 三、启动网页服务

进入项目目录：

```powershell
D:\ESP32_Project\2026\2026_3\aaa
```

启动服务：

```powershell
python camera_web_server.py --host 0.0.0.0 --port 8000
```

正常启动后终端会显示：

```text
Server running at http://127.0.0.1:8000/
Upload endpoint: /upload
```

首次运行时，如果 Windows 防火墙弹出提示，请允许 Python 访问“专用网络”，否则 ESP32 可能无法上传图像。

保持该 PowerShell 窗口运行。需要停止服务时，在窗口中按 `Ctrl+C`。

## 四、通过串口更换 ESP32 Wi-Fi

使用串口工具打开 ESP32 串口，参数如下：

```text
波特率：115200
数据位：8
停止位：1
校验位：无
行结束符：CRLF 或换行
```

严格按照设备提示逐项输入，不要把命令、Wi-Fi 名称和密码一次性连续发送。

1. 输入命令并按回车：

   ```text
   wifi set
   ```

2. 等待设备显示：

   ```text
   [WiFi Setup] Enter SSID, then press Enter:
   >
   ```

3. 输入目标 Wi-Fi 名称并按回车。
4. 等待设备提示输入密码。
5. 输入目标 Wi-Fi 密码并按回车。开放网络直接按回车。
6. 等待连接完成。成功后会看到类似信息：

   ```text
   [WiFi Setup] Device IP: 192.168.5.62
   [WiFi Setup] Connected and saved successfully.
   ```

记录设备输出的新 IP，例如 `192.168.5.62`。

## 五、设置 ESP32 图片上传地址

仍在同一个串口终端中操作：

1. 输入命令并按回车：

   ```text
   server set
   ```

2. 等待设备显示：

   ```text
   [Server Setup] Enter upload URL, then press Enter:
   >
   ```

3. 输入电脑的上传地址并按回车。假设电脑 IP 是 `192.168.5.61`：

   ```text
   http://192.168.5.61:8000/upload
   ```

4. 设置成功后会显示：

   ```text
   [Server Setup] Upload URL saved successfully.
   ```

5. ESP32 后续出现以下日志，表示图片已成功上传到电脑：

   ```text
   upload ok, status=200
   ```

## 六、打开视频网页

在运行 Web 服务的电脑上，使用 Microsoft Edge 或 Google Chrome 打开：

```text
http://127.0.0.1:8000/
```

也可以使用电脑的局域网 IP：

```text
http://192.168.5.61:8000/
```

网页会根据 ESP32 最近一次上传请求自动获取设备 IP，并转到 ESP32 的 15 FPS 视频流。更换 Wi-Fi 后如果仍显示旧画面，请等待串口出现 `upload ok, status=200`，然后刷新网页。

## 七、录制视频

1. 确认网页视频已经正常播放。
2. 点击网页右下角的红色圆形录制按钮。
3. 浏览器弹出共享选择窗口后，选择当前显示 ESP32 视频的浏览器标签页或浏览器窗口，然后确认共享。
4. 录制开始后，右下角显示录制计时，按钮变为停止图标。
5. 再次点击右下角按钮停止录制。
6. 等待浏览器自动生成并下载视频文件。根据浏览器支持情况，文件格式为 `.mp4` 或 `.webm`。

停止录制后不要立即关闭网页，应等待下载开始。录制文件通常保存在浏览器的“下载”目录中。

## 八、更换 Wi-Fi 后的处理

每次更换 Wi-Fi 时，需要重新完成以下操作：

1. 先让电脑连接新的目标 Wi-Fi。
2. 使用 `ipconfig` 查询电脑的新 IPv4 地址。
3. 通过 `wifi set` 修改 ESP32 的 Wi-Fi。
4. 通过 `server set` 把上传地址修改为电脑的新 IP。
5. 等待 `upload ok, status=200` 后刷新网页。

如果 Web 服务仍使用 8000 端口，本机浏览器地址始终可以使用 `http://127.0.0.1:8000/`。只有提供给 ESP32 的上传地址必须跟随电脑 IP 变化。

## 九、常见问题

### 1. 网页提示 `ERR_EMPTY_RESPONSE`

- 确认运行 Web 服务的 PowerShell 窗口没有关闭。
- 确认浏览器地址为 `http://127.0.0.1:8000/`。
- 执行以下命令检查 8000 端口：

  ```powershell
  netstat -ano | findstr :8000
  ```

- 如果端口已被其他程序占用，先关闭旧服务，再重新启动 `camera_web_server.py`。

### 2. 网页能打开但没有视频

- 确认串口已经出现 `upload ok, status=200`。
- 确认电脑和 ESP32 位于同一个局域网。
- 关闭其他正在访问 `http://ESP32-IP:81/stream` 的网页或程序。ESP32 视频流不适合同时被多个客户端长期占用。
- 刷新网页，使其读取 ESP32 最新 IP。

### 3. ESP32 上传失败

- `server set` 中必须填写电脑的局域网 IPv4 地址，不能填写 `127.0.0.1`。
- 确认上传地址末尾包含 `/upload`。
- 确认 Windows 防火墙允许 Python 使用专用网络。
- 确认 Python 服务仍在监听 8000 端口。

### 4. 输入 `wifi set` 后没有提示

- 串口波特率应为 `115200`。
- 串口发送应带 CRLF 或换行符。
- 确认上位机、ESP-IDF Monitor 等其他程序没有占用同一串口。
- 如果设备提示 `unknown command`，说明烧录的固件版本较旧，需要重新编译并烧录当前项目固件。

### 5. 录制后视频不完整

- 必须再次点击录制按钮正常停止，不要直接关闭网页或浏览器。
- 停止后等待浏览器完成文件下载。
- 录制期间不要让电脑休眠，也不要断开目标 Wi-Fi。
- 优先使用最新版 Microsoft Edge 或 Google Chrome。

