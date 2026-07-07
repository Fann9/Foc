// FOC 双线程控制器
// 线程 1：传感器读取 (500Hz) — 高优先级，实时获取编码器角度
// 线程 2：电机控制 (100Hz) — 读取角度，计算 FOC 输出

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "DengFOC.h"

static const char *TAG = "FOC";

#define MOTOR_ENABLE_PIN  12
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static float _normalizeAngle(float angle) {
    float a = fmod(angle, 2 * M_PI);
    return a >= 0 ? a : (a + 2 * M_PI);
}

// ============ 全局实例 ============
static AS5600_t encoder;
static Motor_t motor;
static SemaphoreHandle_t angle_mutex;

// 共享数据：传感器线程写入，控制线程读取
static float shared_angle = 0;           // 多圈累计角度
static float shared_raw_angle = 0;       // 单圈角度 (用于算电角度)

// ============ 传感器线程 (500Hz) ============
static void sensor_task(void *arg) {
    while (1) {
        // 一次性读取两种角度，保持数据一致性
        float raw = AS5600_getAngle_Without_track(&encoder);
        float full = AS5600_getAngle(&encoder);
        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        shared_angle = full;
        shared_raw_angle = raw;
        xSemaphoreGive(angle_mutex);
        vTaskDelay(pdMS_TO_TICKS(2));  // 2ms = 500Hz
    }
}

// ============ 电机控制线程 (100Hz) ============
static void control_task(void *arg) {
    float motor_target = M_PI;   // 目标角度 (rad)
    float Kp = 0.133f;           // 比例增益

    ESP_LOGI(TAG, "开始角度闭环控制，目标: %.2f rad", motor_target);

    while (1) {
        // 从传感器线程读取最新角度（同一把锁读两个值）
        float sensor_angle, raw_angle;
        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        sensor_angle = shared_angle;
        raw_angle = shared_raw_angle;
        xSemaphoreGive(angle_mutex);

        // 用同一时刻的 raw_angle 算电角度，避免数据不同步
        float el_angle = _normalizeAngle(
            (float)(motor.DIR * motor.PP) * raw_angle - motor.zero_electric_angle);

        // PI 控制器
        float error = motor_target - (-1) * sensor_angle;
        float Uq = _constrain(Kp * error * 180.0f / M_PI, -6, 6);

        // 输出力矩
        Motor_setTorque(&motor, Uq, el_angle);

        ESP_LOGI(TAG, "角度: %.2f rad, 误差: %.2f, Uq: %.2f", sensor_angle, error, Uq);
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms = 100Hz
    }
}

// ============ 主函数 ============
extern "C" void app_main(void) {
    // 使能电机驱动
    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled (GPIO %d)", MOTOR_ENABLE_PIN);

    // 初始化编码器 (I2C, SDA=19, SCL=18, 400kHz)
    AS5600_init(&encoder, I2C_NUM_0, 19, 18, 400000);

    // 初始化电机 (12.3V, PWM: 32/33/25)
    Motor_init(&motor, 12.3f, 32, 33, 25, &encoder);

    // 电角度校准：极对数 7，方向 -1
    Motor_alignSensor(&motor, 7, -1);

    // 创建互斥锁
    angle_mutex = xSemaphoreCreateMutex();

    // 创建线程
    // 传感器线程：优先级 5，绑定 CPU1，栈 4KB
    xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 5, NULL, 1);
    // 控制线程：优先级 4，绑定 CPU1，栈 4KB
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "双线程已启动: sensor(5,1kHz) + control(4,100Hz)");
}
