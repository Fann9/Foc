// FOC 双线程控制器
// 线程 1：传感器读取 (500Hz) — 高优先级，实时获取编码器角度 + 速度
// 线程 2：电机控制 (100Hz) — 支持闭环位置 / 闭环速度切换
//
// ==================== 控制模式切换 ====================
// 修改 CONTROL_MODE 宏即可切换, 重新编译烧录生效
#define CONTROL_MODE_POSITION  0   // 闭环位置
#define CONTROL_MODE_VELOCITY  1   // 闭环速度
#define CONTROL_MODE  CONTROL_MODE_VELOCITY   // ← 在这里切换模式
// ======================================================

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "DengFOC.h"
#include "lowpass_filter.h"

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

// 速度低通滤波器 (Tf=0.01s → 截止频率 ~16Hz, 适合 100Hz 控制环)
static LowPassFilter vel_filter(0.01f);

// 共享数据：传感器线程写入，控制线程读取
static float shared_angle = 0;           // 多圈累计角度
static float shared_raw_angle = 0;       // 单圈角度 (用于算电角度)
static float shared_velocity = 0;        // 滤波后速度 (rad/s)

// ============ 传感器线程 (500Hz) ============
static void sensor_task(void *arg) {
    float raw_prev = 0;
    bool first_reading = true;

    while (1) {
        float raw = AS5600_getAngle_Without_track(&encoder);
        float full = AS5600_getAngle(&encoder);

        // 速度计算：用 raw angle 做 wrap-aware 差分
        // 避免多圈追踪在过零点附近因量化噪声导致的速度跳变
        float dt = 0.002f;  // 500Hz 固定周期
        float velocity = 0;
        if (!first_reading) {
            float d_raw = raw - raw_prev;
            if (d_raw > M_PI)       d_raw -= 2.0f * M_PI;  // 正向过零
            else if (d_raw < -M_PI) d_raw += 2.0f * M_PI;  // 反向过零
            velocity = d_raw / dt;
        } else {
            first_reading = false;
        }
        raw_prev = raw;
        float vel_filtered = vel_filter(velocity);

        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        shared_angle = full;
        shared_raw_angle = raw;
        shared_velocity = vel_filtered;
        xSemaphoreGive(angle_mutex);
        vTaskDelay(pdMS_TO_TICKS(2));  // 2ms = 500Hz
    }
}

// ============ 电机控制线程 (100Hz) ============
static void control_task(void *arg) {

#if CONTROL_MODE == CONTROL_MODE_POSITION
    // ── 闭环位置控制 ──
    float motor_target = M_PI;   // 目标角度 (rad)
    float Kp = 0.133f;           // 比例增益

    ESP_LOGI(TAG, ">> 闭环位置模式 | 目标: %.2f rad (%.0f°) | Kp=%.3f",
             motor_target, motor_target * 180.0f / M_PI, Kp);

    while (1) {
        float sensor_angle, raw_angle, velocity;
        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        sensor_angle = shared_angle;
        raw_angle = shared_raw_angle;
        velocity = shared_velocity;
        xSemaphoreGive(angle_mutex);

        float el_angle = _normalizeAngle(
            (float)(motor.DIR * motor.PP) * raw_angle - motor.zero_electric_angle);

        float error = motor_target - (-1) * sensor_angle;
        float Uq = _constrain(Kp * error * 180.0f / M_PI, -6, 6);

        Motor_setTorque(&motor, Uq, el_angle);

        ESP_LOGI(TAG, "角度: %.2f rad, 速度: %.2f rad/s, 误差: %.2f, Uq: %.2f",
                 sensor_angle, velocity, error, Uq);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

#elif CONTROL_MODE == CONTROL_MODE_VELOCITY
    // ── 闭环速度控制 ──
    float motor_target = M_PI;   // 目标速度: π rad/s = 2秒一圈
    float Kp = 0.13f;             // 速度环 P 增益
    float Ki = 0.00f;            // 速度环 I 增益 (消除静差)
    float integral = 0;
    float integral_limit = 3.0f; // 积分抗饱和上限

    ESP_LOGI(TAG, ">> 闭环速度模式 | 目标: %.2f rad/s (%.1f 秒/圈) | Kp=%.3f Ki=%.3f",
             motor_target, 2.0f * M_PI / motor_target, Kp, Ki);

    while (1) {
        float sensor_angle, raw_angle, velocity;
        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        sensor_angle = shared_angle;
        raw_angle = shared_raw_angle;
        velocity = shared_velocity;
        xSemaphoreGive(angle_mutex);

        float el_angle = _normalizeAngle(
            (float)(motor.DIR * motor.PP) * raw_angle - motor.zero_electric_angle);

        // PI 速度控制
        float error = motor_target - velocity;
        integral += error * 0.01f;  // dt = 10ms (100Hz)
        integral = _constrain(integral, -integral_limit, integral_limit);  // 抗饱和
        float Uq = _constrain(Kp * error + Ki * integral, -6, 6);

        Motor_setTorque(&motor, Uq, el_angle);

        ESP_LOGI(TAG, "角度: %.2f rad, 速度: %.2f rad/s, 误差: %.2f, I: %.2f, Uq: %.2f",
                 sensor_angle, velocity, error, integral, Uq);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
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

    ESP_LOGI(TAG, "双线程已启动: sensor(5,500Hz) + control(4,100Hz)");
}
