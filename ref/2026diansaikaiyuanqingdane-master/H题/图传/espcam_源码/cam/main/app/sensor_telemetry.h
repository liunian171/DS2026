#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    bool temperature_valid;
    float temperature_c;
    bool humidity_valid;
    float humidity_rh;
    bool eddy_current_valid;
    float eddy_current_v;
    bool co_valid;
    float co_ppm;
    bool methane_valid;
    float methane_ppm;
    bool pm25_valid;
    float pm25_ugm3;
    bool flame_valid;
    float flame_level_pct;
    bool flame_detected;
} sensor_telemetry_t;

esp_err_t sensor_telemetry_init(void);
void sensor_telemetry_read(sensor_telemetry_t *telemetry);
