/**
 * @brief 计算循迹传感器的加权平均值
 *
 * @param sensor      传感器输出数组，检测到黑线为1，白色为0
 * @param weight      每个传感器对应的权重数组
 * @param count       传感器数量
 * @param lost_value  全部传感器都没有检测到黑线时的返回值
 *
 * @return 加权平均后的循迹偏差
 */
float Line_GetWeightedAverage(const uint8_t sensor[], const float weight[], uint8_t count, float lost_value)
{
    float weighted_sum = 0.0f;
    float sensor_sum = 0.0f;

    for (uint8_t i = 0; i < count; i++)
    {
        // 保证传感器值只有0或1
        float value = sensor[i] ? 1.0f : 0.0f;

        weighted_sum += value * weight[i];
        sensor_sum += value;
    }

    // 所有传感器都没有检测到黑线
    if (sensor_sum == 0.0f)
    {
        return lost_value;
    }

    return weighted_sum / sensor_sum;
}

//使用示例：假设是8路传感
#define SENSOR_COUNT 8

uint8_t line_sensor[SENSOR_COUNT] = {0, 1, 1, 0, 0, 0, 0, 0};

const float line_weight[SENSOR_COUNT] =
{
    50.0f,
    40.0f,
    30.0f,
    20.0f,
    -20.0f,
    -30.0f,
    -40.0f,
    -50.0f
};

float last_error = 0.0f;
float line_error;

line_error = Line_GetWeightedAverage(line_sensor, line_weight, SENSOR_COUNT, last_error);

//直接输入到左右电机速度

Car_Speed(V_left + line_error, V_right - line_error);