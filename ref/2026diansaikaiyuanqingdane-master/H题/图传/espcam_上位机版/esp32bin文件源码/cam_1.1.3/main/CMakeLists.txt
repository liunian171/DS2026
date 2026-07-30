idf_component_register(SRCS "main.c" "app/camera_upload.c" "app/sensor_telemetry.c"
                    INCLUDE_DIRS "." "app"
                    REQUIRES esp32-camera esp_driver_uart esp_event esp_http_client esp_http_server esp_netif esp_timer esp_wifi nvs_flash)
