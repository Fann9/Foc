#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "DengFOC.h"

static const char *TAG = "FOC";

#define MOTOR_ENABLE_PIN  12
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

extern "C" void app_main(void) {
    // 使能电机驱动
    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled (GPIO %d)", MOTOR_ENABLE_PIN);

    // 初始化 FOC：MCPWM + AS5600 编码器
    DFOC_Vbus(12.3f);

    // 电角度校准：极对数 7，方向 -1
    DFOC_alignSensor(7, -1);

    // 角度闭环参数
    float motor_target = M_PI;   // 目标角度 (rad)
    float Kp = 0.133f;           // 比例增益

    ESP_LOGI(TAG, "开始角度闭环控制，目标: %.2f rad", motor_target);

    while (1) {
        float sensor_angle = DFOC_M0_Angle();
        float error = motor_target - (-1) * sensor_angle;
        float Uq = _constrain(Kp * error * 180.0f / M_PI, -6, 6);

        setTorque(Uq, _electricalAngle());

        ESP_LOGI(TAG, "角度: %.2f rad, 误差: %.2f, Uq: %.2f", sensor_angle, error, Uq);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
