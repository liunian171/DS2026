#include <errno.h>
#include <limits.h>
#include <netinet/tcp.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include "camera_upload.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sensor_telemetry.h"

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAXIMUM_RETRY 10
#define WIFI_CONNECTION_TIMEOUT_MS 30000
#define WIFI_RESTART_SETTLE_MS 200
#define WIFI_SSID_MAX_LENGTH 32
#define WIFI_PASSWORD_MAX_LENGTH 63
#define WIFI_SERIAL_LINE_MAX_LENGTH 96
#define WIFI_SERIAL_RX_BUFFER_SIZE 512
#define WIFI_SERIAL_IDLE_TIMEOUT_MS 1000
#define WIFI_SERIAL_TASK_STACK_SIZE 4096
#define WIFI_SERIAL_TASK_PRIORITY 4
#define WIFI_NVS_NAMESPACE "wifi_user"
#define WIFI_NVS_SSID_KEY "ssid"
#define WIFI_NVS_PASSWORD_KEY "password"
#define WIFI_NVS_UPLOAD_URL_KEY "upload_url"
#define UPLOAD_URL_MAX_LENGTH 191

#define CAMERA_STREAM_PORT 81
#define CAMERA_STREAM_TARGET_FPS 15
#define CAMERA_STREAM_FRAME_INTERVAL_US (1000000LL / CAMERA_STREAM_TARGET_FPS)
#define CAMERA_STREAM_CONTENT_TYPE "multipart/x-mixed-replace; boundary=frame"
#define CAMERA_STREAM_BOUNDARY "\r\n--frame\r\n"
#define CAMERA_STREAM_PART "Content-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n"

static const char *TAG = "ov2640_capture";
static EventGroupHandle_t wifi_event_group;
static int wifi_retry_count;
static volatile bool wifi_reconfiguration_in_progress;
static char active_wifi_ssid[WIFI_SSID_MAX_LENGTH + 1];
static char active_wifi_password[WIFI_PASSWORD_MAX_LENGTH + 1];
static char active_upload_url[UPLOAD_URL_MAX_LENGTH + 1];
static SemaphoreHandle_t configuration_mutex;
static httpd_handle_t camera_stream_server;
static volatile bool camera_stream_active;

/**
 * @brief 为 MJPEG 客户端套接字禁用 Nagle 算法，减少小数据块的发送等待。
 *
 * @param server HTTP 服务器句柄；由 ESP-IDF 传入，本函数无需修改该对象。
 * @param socket_fd 新建客户端会话的 TCP 套接字文件描述符。
 * @return ESP_OK 表示 TCP_NODELAY 设置成功，ESP_FAIL 表示套接字配置失败。
 */
static esp_err_t configure_stream_socket(httpd_handle_t server, int socket_fd)
{
    (void)server;
    const int no_delay = 1;

    if (setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)) != 0) {
        ESP_LOGE(TAG, "failed to enable TCP_NODELAY: errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static bool camera_has_psram(void)
{
    return heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0;
}

static esp_err_t camera_init(void)
{
    const bool has_psram = camera_has_psram();

    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 14,
        .fb_count = has_psram ? 2 : 1,
        .grab_mode = has_psram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
    };

    ESP_LOGI(TAG, "camera init, psram=%s", has_psram ? "enabled" : "disabled");
    return esp_camera_init(&config);
}

/**
 * @brief 从独立 NVS 命名空间读取用户通过串口保存的 Wi-Fi 凭据。
 *
 * @param ssid 用于接收 SSID 的缓冲区，不可为 NULL。
 * @param ssid_size SSID 缓冲区容量，必须包含字符串结尾空字符。
 * @param password 用于接收密码的缓冲区，不可为 NULL。
 * @param password_size 密码缓冲区容量，必须包含字符串结尾空字符。
 * @return true 表示 SSID 和密码读取成功，false 表示未保存或数据无效。
 */
static bool wifi_credentials_load(char *ssid,
                                  size_t ssid_size,
                                  char *password,
                                  size_t password_size)
{
    if (ssid == NULL || ssid_size == 0 || password == NULL || password_size == 0) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    size_t stored_ssid_size = ssid_size;
    size_t stored_password_size = password_size;
    err = nvs_get_str(handle, WIFI_NVS_SSID_KEY, ssid, &stored_ssid_size);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, WIFI_NVS_PASSWORD_KEY, password, &stored_password_size);
    }
    nvs_close(handle);

    return err == ESP_OK && ssid[0] != '\0';
}

/**
 * @brief 将已验证可连接的 Wi-Fi 凭据持久化到 NVS。
 *
 * @param ssid 需要保存的 Wi-Fi 名称，不可为 NULL 或空字符串。
 * @param password 需要保存的 Wi-Fi 密码；开放网络可传入空字符串。
 * @return ESP_OK 表示写入并提交成功，其他值表示 NVS 操作失败。
 */
static esp_err_t wifi_credentials_save(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0' || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(handle, WIFI_NVS_SSID_KEY, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_PASSWORD_KEY, password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/**
 * @brief 检查上传地址是否为长度合法的 HTTP URL。
 *
 * @param url 需要检查的上传地址，不可为 NULL。
 * @return true 表示地址可以使用，false 表示地址为空、过长或不是 HTTP URL。
 */
static bool upload_url_is_valid(const char *url)
{
    if (url == NULL) {
        return false;
    }

    size_t url_length = strlen(url);
    return url_length > strlen("http://") && url_length <= UPLOAD_URL_MAX_LENGTH &&
           strncmp(url, "http://", strlen("http://")) == 0;
}

/**
 * @brief 从 NVS 读取上次保存的上传地址。
 *
 * @param url 用于接收上传地址的缓冲区，不可为 NULL。
 * @param url_size 缓冲区容量，必须包含字符串结尾空字符。
 * @return true 表示读取到有效地址，false 表示未保存或地址无效。
 */
static bool upload_url_load(char *url, size_t url_size)
{
    if (url == NULL || url_size == 0) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    size_t stored_url_size = url_size;
    err = nvs_get_str(handle, WIFI_NVS_UPLOAD_URL_KEY, url, &stored_url_size);
    nvs_close(handle);
    return err == ESP_OK && upload_url_is_valid(url);
}

/**
 * @brief 初始化上传地址锁和当前地址，优先使用 NVS 保存值。
 *
 * @param 无。
 * @return ESP_OK 表示初始化成功，ESP_ERR_NO_MEM 表示无法创建互斥锁，ESP_ERR_INVALID_ARG 表示编译地址无效。
 */
static esp_err_t upload_url_initialize(void)
{
    configuration_mutex = xSemaphoreCreateMutex();
    if (configuration_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (upload_url_load(active_upload_url, sizeof(active_upload_url))) {
        ESP_LOGI(TAG, "using upload URL saved from serial: %s", active_upload_url);
        return ESP_OK;
    }

    if (!upload_url_is_valid(CONFIG_CAMERA_UPLOAD_URL)) {
        return ESP_ERR_INVALID_ARG;
    }
    snprintf(active_upload_url, sizeof(active_upload_url), "%s", CONFIG_CAMERA_UPLOAD_URL);
    return ESP_OK;
}

/**
 * @brief 将有效上传地址写入 NVS，提交成功后再应用到运行时。
 *
 * @param url 新的 HTTP 上传地址，不可为 NULL。
 * @return ESP_OK 表示保存并应用成功，其他值表示参数、NVS 或同步错误。
 */
static esp_err_t upload_url_save_and_apply(const char *url)
{
    if (!upload_url_is_valid(url) || configuration_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(handle, WIFI_NVS_UPLOAD_URL_KEY, url);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(configuration_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    snprintf(active_upload_url, sizeof(active_upload_url), "%s", url);
    xSemaphoreGive(configuration_mutex);
    return ESP_OK;
}

/**
 * @brief 将当前上传地址安全复制到调用方缓冲区。
 *
 * @param url 用于接收地址的可写缓冲区，不可为 NULL。
 * @param url_size 缓冲区容量，必须大于当前地址长度。
 * @return ESP_OK 表示复制成功，其他值表示参数、长度或同步错误。
 */
static esp_err_t upload_url_copy(char *url, size_t url_size)
{
    if (url == NULL || url_size == 0 || configuration_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(configuration_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    size_t required_size = strlen(active_upload_url) + 1;
    if (required_size > url_size) {
        xSemaphoreGive(configuration_mutex);
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(url, active_upload_url, required_size);
    xSemaphoreGive(configuration_mutex);
    return ESP_OK;
}

/**
 * @brief 根据 SSID 和密码生成 ESP-IDF STA 连接配置。
 *
 * @param config 用于接收配置的 wifi_config_t 对象，不可为 NULL。
 * @param ssid Wi-Fi 名称，长度必须不超过 32 字节。
 * @param password Wi-Fi 密码；空字符串表示开放网络。
 * @return ESP_OK 表示配置有效并已生成，ESP_ERR_INVALID_ARG 表示参数、SSID 或密码无效。
 */
static esp_err_t wifi_config_fill(wifi_config_t *config,
                                  const char *ssid,
                                  const char *password)
{
    if (config == NULL || ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t ssid_length = strlen(ssid);
    size_t password_length = strlen(password);
    if (ssid_length == 0 || ssid_length > WIFI_SSID_MAX_LENGTH ||
        password_length > WIFI_PASSWORD_MAX_LENGTH ||
        (password_length > 0 && password_length < 8)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(config, 0, sizeof(*config));
    memcpy(config->sta.ssid, ssid, ssid_length);
    memcpy(config->sta.password, password, password_length);
    config->sta.threshold.authmode =
        password_length == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config->sta.pmf_cfg.capable = true;
    config->sta.pmf_cfg.required = false;
    return ESP_OK;
}

static bool wifi_is_connected(void)
{
    if (wifi_event_group == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

        if (wifi_reconfiguration_in_progress) {
            return;
        }

        if (wifi_retry_count < WIFI_MAXIMUM_RETRY) {
            wifi_retry_count++;
            esp_wifi_connect();
            ESP_LOGW(TAG, "wifi reconnecting, retry=%d", wifi_retry_count);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "wifi connect failed");
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        wifi_retry_count = 0;
        xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "wifi connected, ip=" IPSTR, IP2STR(&event->ip_info.ip));
        printf("\r\n[WiFi Setup] Device IP: " IPSTR "\r\n", IP2STR(&event->ip_info.ip));
        fflush(stdout);
    }
}

/**
 * @brief 等待 Wi-Fi 连接成功或连续重试失败。
 *
 * @param timeout_ticks 最长等待时间，单位为 FreeRTOS tick。
 * @return ESP_OK 表示已获取 IP，ESP_FAIL 表示重试用尽，ESP_ERR_TIMEOUT 表示等待超时。
 */
static esp_err_t wifi_wait_for_connection(TickType_t timeout_ticks)
{
    if (wifi_event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           timeout_ticks);
    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }
    if (bits & WIFI_FAIL_BIT) {
        return ESP_FAIL;
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief 停止当前 STA，应用新凭据后重新启动并等待获取 IP。
 *
 * @param ssid 目标 Wi-Fi 名称，长度必须为 1–32 字节。
 * @param password 目标 Wi-Fi 密码；空字符串表示开放网络。
 * @return ESP_OK 表示新网络已连接，其他值表示配置、启动或连接失败。
 */
static esp_err_t wifi_reconnect_with_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t wifi_config;
    esp_err_t err = wifi_config_fill(&wifi_config, ssid, password);
    if (err != ESP_OK) {
        return err;
    }

    wifi_reconfiguration_in_progress = true;
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    wifi_retry_count = 0;

    err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
        wifi_reconfiguration_in_progress = false;
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(WIFI_RESTART_SETTLE_MS));
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    if (err != ESP_OK) {
        wifi_reconfiguration_in_progress = false;
        return err;
    }

    wifi_reconfiguration_in_progress = false;
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to disable wifi power save: %s", esp_err_to_name(err));
    }

    return wifi_wait_for_connection(pdMS_TO_TICKS(WIFI_CONNECTION_TIMEOUT_MS));
}

/**
 * @brief 从控制台 UART 读取一行文本，支持回车结束或收到内容后超时结束。
 *
 * @param buffer 用于接收文本的可写缓冲区，不可为 NULL。
 * @param buffer_size 缓冲区总容量，必须至少为 2 字节。
 * @return ESP_OK 表示成功读取一行，ESP_ERR_INVALID_SIZE 表示输入超长，其他值表示参数或 UART 错误。
 */
static esp_err_t serial_read_line(char *buffer, size_t buffer_size)
{
    static uint8_t paired_terminator_to_skip;

    if (buffer == NULL || buffer_size < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    const uart_port_t uart_port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
    size_t length = 0;
    bool overflow = false;

    while (true) {
        uint8_t byte = 0;
        int received = uart_read_bytes(
            uart_port, &byte, 1, pdMS_TO_TICKS(WIFI_SERIAL_IDLE_TIMEOUT_MS));
        if (received < 0) {
            buffer[0] = '\0';
            return ESP_FAIL;
        }
        if (received == 0) {
            if (length > 0 || overflow) {
                buffer[length] = '\0';
                return overflow ? ESP_ERR_INVALID_SIZE : ESP_OK;
            }
            continue;
        }
        if (paired_terminator_to_skip != 0) {
            uint8_t expected_terminator = paired_terminator_to_skip;
            paired_terminator_to_skip = 0;
            if (byte == expected_terminator) {
                continue;
            }
        }
        if (byte == '\r' || byte == '\n') {
            paired_terminator_to_skip = byte == '\r' ? '\n' : '\r';
            buffer[length] = '\0';
            return overflow ? ESP_ERR_INVALID_SIZE : ESP_OK;
        }
        if (byte == '\b' || byte == 0x7f) {
            if (length > 0) {
                length--;
            }
            continue;
        }
        if (length + 1 < buffer_size) {
            buffer[length++] = (char)byte;
        } else {
            overflow = true;
        }
    }
}

/**
 * @brief 交互式读取、验证并保存新的电脑端 HTTP 上传地址。
 *
 * @param 无。
 * @return 无；处理结果通过串口提示信息返回。
 */
static void serial_configure_upload_url(void)
{
    char new_upload_url[UPLOAD_URL_MAX_LENGTH + 1];

    printf("\r\n[Server Setup] Enter upload URL, then press Enter:\r\n> ");
    fflush(stdout);
    esp_err_t err = serial_read_line(new_upload_url, sizeof(new_upload_url));
    if (err != ESP_OK || !upload_url_is_valid(new_upload_url)) {
        printf("\r\n[Server Setup] Invalid URL; use http://<PC-IP>:8000/upload.\r\n");
        fflush(stdout);
        return;
    }

    err = upload_url_save_and_apply(new_upload_url);
    if (err == ESP_OK) {
        printf("\r\n[Server Setup] Upload URL saved successfully.\r\n");
        fflush(stdout);
        ESP_LOGI(TAG, "upload URL updated: %s", new_upload_url);
    } else {
        printf("\r\n[Server Setup] Upload URL save failed.\r\n");
        fflush(stdout);
        ESP_LOGE(TAG, "upload URL update failed: %s", esp_err_to_name(err));
    }
}

/**
 * @brief 处理 `wifi set` 串口命令，交互式读取凭据并执行可回滚的换网。
 *
 * @param argument FreeRTOS 任务可选上下文；当前未使用，应传入 NULL。
 * @return 无；该任务在设备运行期间持续等待命令。
 */
static void wifi_serial_task(void *argument)
{
    (void)argument;
    char command[WIFI_SERIAL_LINE_MAX_LENGTH];
    char new_ssid[WIFI_SSID_MAX_LENGTH + 1];
    char new_password[WIFI_PASSWORD_MAX_LENGTH + 1];
    char previous_ssid[WIFI_SSID_MAX_LENGTH + 1];
    char previous_password[WIFI_PASSWORD_MAX_LENGTH + 1];

    ESP_LOGI(TAG,
             "serial setup ready; commands: 'wifi set' and 'server set' at 115200 baud");

    while (true) {
        esp_err_t err = serial_read_line(command, sizeof(command));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "serial command is too long or could not be read");
            continue;
        }
        if (strcmp(command, "server set") == 0 || strcmp(command, "SERVER SET") == 0) {
            serial_configure_upload_url();
            continue;
        }
        if (strcmp(command, "wifi set") != 0 && strcmp(command, "WIFI SET") != 0) {
            if (command[0] != '\0') {
                ESP_LOGW(TAG, "unknown command; use 'wifi set' or 'server set'");
            }
            continue;
        }

        printf("\r\n[WiFi Setup] Enter SSID, then press Enter:\r\n> ");
        fflush(stdout);
        err = serial_read_line(new_ssid, sizeof(new_ssid));
        if (err != ESP_OK || new_ssid[0] == '\0') {
            printf("\r\n[WiFi Setup] Invalid SSID; enter 'wifi set' to try again.\r\n");
            fflush(stdout);
            ESP_LOGE(TAG, "SSID must contain 1 to 32 bytes");
            continue;
        }

        printf("\r\n[WiFi Setup] Enter password, then press Enter (empty for open Wi-Fi):\r\n> ");
        fflush(stdout);
        err = serial_read_line(new_password, sizeof(new_password));
        printf("\r\n");
        if (err != ESP_OK) {
            printf("[WiFi Setup] Password is too long; enter 'wifi set' to try again.\r\n");
            fflush(stdout);
            ESP_LOGE(TAG, "password is too long; maximum is 63 bytes");
            continue;
        }

        size_t password_length = strlen(new_password);
        if (password_length > 0 && password_length < 8) {
            printf("[WiFi Setup] Password must be empty or 8 to 63 bytes; try again.\r\n");
            fflush(stdout);
            ESP_LOGE(TAG, "password must be empty or contain 8 to 63 bytes");
            continue;
        }

        snprintf(previous_ssid, sizeof(previous_ssid), "%s", active_wifi_ssid);
        snprintf(previous_password, sizeof(previous_password), "%s", active_wifi_password);
        printf("[WiFi Setup] Connecting to SSID '%s'...\r\n", new_ssid);
        fflush(stdout);
        ESP_LOGI(TAG, "connecting to new Wi-Fi SSID: %s", new_ssid);

        err = wifi_reconnect_with_credentials(new_ssid, new_password);
        if (err == ESP_OK) {
            snprintf(active_wifi_ssid, sizeof(active_wifi_ssid), "%s", new_ssid);
            snprintf(active_wifi_password, sizeof(active_wifi_password), "%s", new_password);
            esp_err_t save_err = wifi_credentials_save(new_ssid, new_password);
            if (save_err == ESP_OK) {
                printf("[WiFi Setup] Connected and saved successfully.\r\n");
                fflush(stdout);
                ESP_LOGI(TAG, "new Wi-Fi connected and saved");
            } else {
                printf("[WiFi Setup] Connected, but saving failed.\r\n");
                fflush(stdout);
                ESP_LOGW(TAG, "new Wi-Fi connected but NVS save failed: %s", esp_err_to_name(save_err));
            }
            continue;
        }

        printf("[WiFi Setup] Connection failed; restoring previous Wi-Fi...\r\n");
        fflush(stdout);
        ESP_LOGE(TAG, "new Wi-Fi connection failed: %s; restoring previous network", esp_err_to_name(err));
        esp_err_t restore_err =
            wifi_reconnect_with_credentials(previous_ssid, previous_password);
        if (restore_err == ESP_OK) {
            printf("[WiFi Setup] Previous Wi-Fi restored.\r\n");
            fflush(stdout);
            ESP_LOGI(TAG, "previous Wi-Fi restored");
        } else {
            printf("[WiFi Setup] Restore failed; enter 'wifi set' to configure again.\r\n");
            fflush(stdout);
            ESP_LOGE(TAG,
                     "previous Wi-Fi restore failed: %s; serial setup remains available",
                     esp_err_to_name(restore_err));
        }
    }
}

/**
 * @brief 为默认控制台 UART 安装接收驱动并启动 Wi-Fi 配置任务。
 *
 * @param 无。
 * @return ESP_OK 表示 UART 和任务已就绪，ESP_ERR_NO_MEM 表示任务创建失败，其他值表示 UART 初始化失败。
 */
static esp_err_t wifi_serial_start(void)
{
    const uart_port_t uart_port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
    esp_err_t err = uart_driver_install(
        uart_port, WIFI_SERIAL_RX_BUFFER_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    BaseType_t task_created = xTaskCreate(wifi_serial_task,
                                          "wifi_serial",
                                          WIFI_SERIAL_TASK_STACK_SIZE,
                                          NULL,
                                          WIFI_SERIAL_TASK_PRIORITY,
                                          NULL);
    return task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

/**
 * @brief 初始化 STA，优先连接串口保存的 Wi-Fi，并启动串口换网任务。
 *
 * @param 无。
 * @return ESP_OK 表示 Wi-Fi 子系统和串口配网任务已就绪；初次联网失败不作为初始化错误。
 */
static esp_err_t wifi_init_sta(void)
{
    wifi_config_t wifi_config;
    esp_event_handler_instance_t any_id_handler;
    esp_event_handler_instance_t got_ip_handler;

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &any_id_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &got_ip_handler));

    bool credentials_loaded = wifi_credentials_load(active_wifi_ssid,
                                                     sizeof(active_wifi_ssid),
                                                     active_wifi_password,
                                                     sizeof(active_wifi_password));
    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (credentials_loaded) {
        err = wifi_config_fill(&wifi_config, active_wifi_ssid, active_wifi_password);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "using Wi-Fi credentials saved from serial");
        } else {
            ESP_LOGW(TAG, "saved Wi-Fi credentials are invalid; using build configuration");
        }
    }

    if (!credentials_loaded || err != ESP_OK) {
        err = wifi_config_fill(
            &wifi_config, CONFIG_CAMERA_WIFI_SSID, CONFIG_CAMERA_WIFI_PASSWORD);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "build Wi-Fi credentials are invalid");
            return err;
        }
        snprintf(active_wifi_ssid, sizeof(active_wifi_ssid), "%s", CONFIG_CAMERA_WIFI_SSID);
        snprintf(active_wifi_password,
                 sizeof(active_wifi_password),
                 "%s",
                 CONFIG_CAMERA_WIFI_PASSWORD);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    err = wifi_serial_start();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "wifi connecting to %s", active_wifi_ssid);
    err = wifi_wait_for_connection(pdMS_TO_TICKS(WIFI_CONNECTION_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "initial Wi-Fi connection unavailable: %s; enter 'wifi set' over serial",
                 esp_err_to_name(err));
        return ESP_OK;
    }

    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to disable wifi power save: %s", esp_err_to_name(err));
    }
    return ESP_OK;
}

/**
 * @brief 持续采集 JPEG 帧并通过单个 HTTP 连接输出 MJPEG 视频流。
 *
 * @param request 当前 HTTP GET 请求对象；由 esp_http_server 传入，不可为空。
 * @return ESP_OK 表示客户端正常断开或流正常结束，其他错误表示采集或发送失败。
 */
static esp_err_t camera_stream_handler(httpd_req_t *request)
{
    if (request == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    camera_stream_active = true;
    esp_err_t err = httpd_resp_set_type(request, CAMERA_STREAM_CONTENT_TYPE);
    if (err != ESP_OK) {
        camera_stream_active = false;
        return err;
    }

    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");

    int64_t next_frame_time_us = esp_timer_get_time();
    char part_header[96];

    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            ESP_LOGE(TAG, "stream capture failed");
            err = ESP_FAIL;
            break;
        }

        if (frame->format != PIXFORMAT_JPEG) {
            esp_camera_fb_return(frame);
            ESP_LOGE(TAG, "stream frame is not jpeg");
            err = ESP_ERR_INVALID_STATE;
            break;
        }

        int header_length = snprintf(
            part_header,
            sizeof(part_header),
            CAMERA_STREAM_BOUNDARY CAMERA_STREAM_PART,
            frame->len);
        if (header_length <= 0 || header_length >= (int)sizeof(part_header)) {
            esp_camera_fb_return(frame);
            err = ESP_ERR_INVALID_SIZE;
            break;
        }

        err = httpd_resp_send_chunk(request, part_header, (size_t)header_length);
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(request, (const char *)frame->buf, frame->len);
        }
        esp_camera_fb_return(frame);

        if (err != ESP_OK) {
            break;
        }

        next_frame_time_us += CAMERA_STREAM_FRAME_INTERVAL_US;
        int64_t remaining_us = next_frame_time_us - esp_timer_get_time();
        if (remaining_us > 0) {
            TickType_t delay_ticks = pdMS_TO_TICKS((remaining_us + 999) / 1000);
            if (delay_ticks > 0) {
                vTaskDelay(delay_ticks);
            }
        } else {
            next_frame_time_us = esp_timer_get_time();
        }
    }

    httpd_resp_send_chunk(request, NULL, 0);
    camera_stream_active = false;
    return err == ESP_ERR_HTTPD_RESP_SEND ? ESP_OK : err;
}

/**
 * @brief 在端口 81 启动 ESP32-CAM MJPEG 视频流服务器。
 *
 * @param 无。
 * @return ESP_OK 表示服务器与 /stream 路由启动成功，其他值表示启动失败。
 */
static esp_err_t start_camera_stream_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CAMERA_STREAM_PORT;
    config.core_id = 1;
    config.stack_size = 8192;
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.open_fn = configure_stream_socket;

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = camera_stream_handler,
        .user_ctx = NULL,
    };

    esp_err_t err = httpd_start(&camera_stream_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_register_uri_handler(camera_stream_server, &stream_uri);
    if (err != ESP_OK) {
        httpd_stop(camera_stream_server);
        camera_stream_server = NULL;
        return err;
    }

    ESP_LOGI(TAG,
             "camera stream ready: http://<device-ip>:%d/stream target=%d fps",
             CAMERA_STREAM_PORT,
             CAMERA_STREAM_TARGET_FPS);
    return ESP_OK;
}

static void set_optional_float_header(esp_http_client_handle_t client,
                                      const char *header_name,
                                      bool is_valid,
                                      float value)
{
    char header_value[24];

    if (!is_valid) {
        return;
    }

    snprintf(header_value, sizeof(header_value), "%.2f", value);
    esp_http_client_set_header(client, header_name, header_value);
}

static void set_optional_bool_header(esp_http_client_handle_t client,
                                     const char *header_name,
                                     bool is_valid,
                                     bool value)
{
    if (!is_valid) {
        return;
    }

    esp_http_client_set_header(client, header_name, value ? "1" : "0");
}

static esp_err_t upload_frame(camera_fb_t *frame, const sensor_telemetry_t *telemetry)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->len > INT_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    char width_value[16];
    char height_value[16];
    char length_value[24];
    char upload_url[UPLOAD_URL_MAX_LENGTH + 1];

    esp_err_t err = upload_url_copy(upload_url, sizeof(upload_url));
    if (err != ESP_OK) {
        return err;
    }

    snprintf(width_value, sizeof(width_value), "%u", frame->width);
    snprintf(height_value, sizeof(height_value), "%u", frame->height);
    snprintf(length_value, sizeof(length_value), "%u", (unsigned int)frame->len);

    esp_http_client_config_t config = {
        .url = upload_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_header(client, "X-Image-Width", width_value);
    esp_http_client_set_header(client, "X-Image-Height", height_value);
    esp_http_client_set_header(client, "X-Image-Length", length_value);
    if (telemetry != NULL) {
        set_optional_float_header(client,
                                  "X-Temperature-C",
                                  telemetry->temperature_valid,
                                  telemetry->temperature_c);
        set_optional_float_header(client,
                                  "X-Humidity-RH",
                                  telemetry->humidity_valid,
                                  telemetry->humidity_rh);
        set_optional_float_header(client,
                                  "X-Eddy-Current-V",
                                  telemetry->eddy_current_valid,
                                  telemetry->eddy_current_v);
        set_optional_float_header(
            client, "X-CO-PPM", telemetry->co_valid, telemetry->co_ppm);
        set_optional_float_header(
            client, "X-Methane-PPM", telemetry->methane_valid, telemetry->methane_ppm);
        set_optional_float_header(
            client, "X-PM25-UGM3", telemetry->pm25_valid, telemetry->pm25_ugm3);
        set_optional_float_header(client,
                                  "X-Flame-Level",
                                  telemetry->flame_valid,
                                  telemetry->flame_level_pct);
        set_optional_bool_header(client,
                                 "X-Flame-Detected",
                                 telemetry->flame_valid,
                                 telemetry->flame_detected);
    }
    esp_http_client_set_post_field(client, (const char *)frame->buf, (int)frame->len);

    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        long long response_length = esp_http_client_get_content_length(client);

        if (status_code < 200 || status_code >= 300) {
            ESP_LOGE(TAG,
                     "upload rejected, status=%d response_len=%lld",
                     status_code,
                     response_length);
            err = ESP_FAIL;
        } else {
            ESP_LOGI(TAG,
                     "upload ok, status=%d response_len=%lld",
                     status_code,
                     response_length);
        }
    } else {
        ESP_LOGE(TAG, "http upload failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

static void capture_and_upload(void)
{
    sensor_telemetry_t telemetry;

    if (camera_stream_active) {
        return;
    }

    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "wifi not ready");
        return;
    }

    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == NULL) {
        ESP_LOGE(TAG, "capture failed");
        return;
    }

    ESP_LOGI(TAG,
             "frame ok: width=%d height=%d len=%zu format=%d",
             frame->width,
             frame->height,
             frame->len,
             frame->format);

    if (frame->format == PIXFORMAT_JPEG && frame->len >= 2) {
        ESP_LOGI(TAG, "jpeg header: 0x%02X 0x%02X", frame->buf[0], frame->buf[1]);
    }

    sensor_telemetry_read(&telemetry);

    ESP_LOGI(TAG,
             "telemetry: temp=%.1fC hum=%.1f%% co=%.1fppm ch4=%.1fppm pm2.5=%.1fug/m3 flame=%.1f%% fire=%s",
             telemetry.temperature_c,
             telemetry.humidity_rh,
             telemetry.co_ppm,
             telemetry.methane_ppm,
             telemetry.pm25_ugm3,
             telemetry.flame_level_pct,
             telemetry.flame_detected ? "yes" : "no");

    upload_frame(frame, &telemetry);
    esp_camera_fb_return(frame);
}

esp_err_t camera_upload_app_run(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = upload_url_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "upload URL init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = start_camera_stream_server();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera stream server failed: %s", esp_err_to_name(err));
        return err;
    }

    err = sensor_telemetry_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sensor init failed: %s", esp_err_to_name(err));
        return err;
    }

    while (true) {
        capture_and_upload();
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CAMERA_UPLOAD_INTERVAL_MS));
    }
}
