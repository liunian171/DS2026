#include <string.h>

#include "esp_err.h"
#include "sensor_telemetry.h"

esp_err_t sensor_telemetry_init(void)
{
    return ESP_OK;
}

void sensor_telemetry_read(sensor_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    memset(telemetry, 0, sizeof(*telemetry));

    telemetry->temperature_valid = true;
    telemetry->humidity_valid = true;
    telemetry->eddy_current_valid = true;
    telemetry->co_valid = true;
    telemetry->methane_valid = true;
    telemetry->pm25_valid = true;
    telemetry->flame_valid = true;

    static unsigned int simulated_upload_count;
    unsigned int cycle_position;

    simulated_upload_count++;
    cycle_position = ((simulated_upload_count - 1U) % 6U) + 1U;

    if (cycle_position <= 3U) {
        telemetry->temperature_c = 24.5f + (float)cycle_position * 0.3f;
        telemetry->humidity_rh = 49.0f + (float)cycle_position * 0.5f;
        telemetry->eddy_current_v = 1.20f + (float)cycle_position * 0.05f;
        telemetry->co_ppm = 3.0f + (float)cycle_position * 0.4f;
        telemetry->methane_ppm = 6.5f + (float)cycle_position * 0.6f;
        telemetry->pm25_ugm3 = 12.0f + (float)cycle_position * 1.5f;
        telemetry->flame_level_pct = 6.0f + (float)cycle_position * 1.0f;
        telemetry->flame_detected = false;
        return;
    }

    {
        float gas_offset = (float)(cycle_position - 4U);

        telemetry->temperature_c = 34.0f + gas_offset * 2.5f;
        telemetry->humidity_rh = 36.0f - gas_offset * 1.0f;
        telemetry->eddy_current_v = 2.10f + gas_offset * 0.15f;
        telemetry->co_ppm = 38.0f + gas_offset * 12.0f;
        telemetry->methane_ppm = 90.0f + gas_offset * 22.0f;
        telemetry->pm25_ugm3 = 105.0f + gas_offset * 25.0f;
        telemetry->flame_level_pct = 18.0f + gas_offset * 3.0f;
        telemetry->flame_detected = false;
    }
}
