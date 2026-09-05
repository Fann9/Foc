// DengFOC 第十课双电机速度闭环 ESP-IDF 移植
// 线程 1：双电机 FOC 控制 (1kHz) — 编码器/电流采样、速度环、电流环、PWM 顺序执行
// 线程 2：状态输出 (10Hz) — 打印两台电机状态，不参与控制计算

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "DengFOC.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "current_sense.h"

static const char *TAG = "FOC";

// ==================== 双电机硬件配置 ====================
#define MOTOR_ENABLE_PIN       12

#define M0_PWM_PIN_A           32
#define M0_PWM_PIN_B           33
#define M0_PWM_PIN_C           25
#define M0_ENCODER_SDA_PIN     19
#define M0_ENCODER_SCL_PIN     18
#define M0_CURRENT_PIN_A       39 // VN / ADC1
#define M0_CURRENT_PIN_B       36 // VP / ADC1
#define M0_MCPWM_GROUP         0
#define M0_SENSOR_DIRECTION    -1 // 已在当前电机上验证
#define M0_TARGET_VELOCITY     5.0f

#define M1_PWM_PIN_A           26
#define M1_PWM_PIN_B           27
#define M1_PWM_PIN_C           14
#define M1_ENCODER_SDA_PIN     23
#define M1_ENCODER_SCL_PIN     5
#define M1_CURRENT_PIN_A       35
#define M1_CURRENT_PIN_B       34
#define M1_MCPWM_GROUP         1
#define M1_SENSOR_DIRECTION    -1 // 与 M0 相同的电机和编码器方向
#define M1_TARGET_VELOCITY     7.0f
// ========================================================

// 第十课 Arduino 示例公共参数
#define MOTOR_VBUS             12.6f
#define MOTOR_POLE_PAIRS       7
#define CURRENT_SHUNT_RESISTOR 0.01f
#define CURRENT_AMP_GAIN       50.0f
#define CONTROL_LOOP_HZ        1000
#define TELEMETRY_PERIOD_MS    100
#define DEG_PER_RAD            (180.0f / M_PI)

#define VELOCITY_KP            3.0f
#define VELOCITY_KI            2.0f
#define VELOCITY_KD            0.0f
#define VELOCITY_RAMP          100000.0f
#define VELOCITY_LIMIT         0.5f

#define CURRENT_KP             0.5f
#define CURRENT_KI             50.0f
#define CURRENT_KD             0.0f
#define CURRENT_RAMP           100000.0f
#define CURRENT_LIMIT          MOTOR_VBUS

static float normalizeAngle(float angle) {
    float value = fmodf(angle, 2.0f * M_PI);
    return value >= 0.0f ? value : value + 2.0f * M_PI;
}

typedef struct {
    AS5600_t encoder;
    Motor_t motor;
    CurrSense_t current_sense;
    PID_t velocity_pid;
    PID_t current_pid;
    float target_velocity;
    float full_angle_prev;
    int64_t velocity_timestamp_prev;
} MotorControl_t;

typedef struct {
    float target_velocity;
    float position;
    float velocity;
    float iq_target;
    float iq;
    float uq;
} MotorTelemetry_t;

static MotorControl_t motor0;
static MotorControl_t motor1;
static MotorTelemetry_t telemetry[2];
static SemaphoreHandle_t telemetry_mutex;
static float control_hz = 0.0f;

// 与 Arduino 示例一致：每台电机速度 Tf=10ms，电流 Tf=50ms
static LowPassFilter m0_velocity_filter(0.01f);
static LowPassFilter m0_current_filter(0.05f);
static LowPassFilter m1_velocity_filter(0.01f);
static LowPassFilter m1_current_filter(0.05f);

static void initControlState(MotorControl_t *control) {
    float raw_angle = AS5600_getAngle_Without_track(&control->encoder);
    control->encoder.angle_prev = raw_angle;
    control->full_angle_prev = AS5600_trackAngle(&control->encoder, raw_angle);
    control->velocity_timestamp_prev = esp_timer_get_time();
}

static MotorTelemetry_t updateMotor(MotorControl_t *control,
                                    LowPassFilter &velocity_filter,
                                    LowPassFilter &current_filter) {
    float raw_angle = AS5600_getAngle_Without_track(&control->encoder);
    float full_angle = AS5600_trackAngle(&control->encoder, raw_angle);
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - control->velocity_timestamp_prev) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 1e-3f;

    float position = control->motor.DIR * full_angle;
    float velocity = velocity_filter(
        control->motor.DIR * (full_angle - control->full_angle_prev) / dt);
    float electrical_angle = normalizeAngle(
        (float)(control->motor.DIR * control->motor.PP) * raw_angle
        - control->motor.zero_electric_angle);
    float iq = current_filter(CurrSense_getIq(&control->current_sense, electrical_angle));

    // 速度环输出 Iq 目标，电流环输出 Uq，与第十课 setVelocity() 相同
    float iq_target = PID_update(
        &control->velocity_pid,
        (control->target_velocity - velocity) * DEG_PER_RAD);
    float uq = PID_update(&control->current_pid, iq_target - iq);
    Motor_setTorque(&control->motor, uq, electrical_angle);

    control->full_angle_prev = full_angle;
    control->velocity_timestamp_prev = now;

    MotorTelemetry_t state = {};
    state.target_velocity = control->target_velocity;
    state.position = position;
    state.velocity = velocity;
    state.iq_target = iq_target;
    state.iq = iq;
    state.uq = uq;
    return state;
}

// ============ 双电机 FOC 控制线程 (1kHz) ============
static void control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    int64_t rate_timestamp = esp_timer_get_time();
    uint32_t rate_cycles = 0;

    initControlState(&motor0);
    initControlState(&motor1);

    ESP_LOGI(TAG, ">> 双电机速度-电流串环启动: M0=%.2frad/s, M1=%.2frad/s, loop=%dHz",
             motor0.target_velocity, motor1.target_velocity, CONTROL_LOOP_HZ);

    while (1) {
        MotorTelemetry_t state0 = updateMotor(
            &motor0, m0_velocity_filter, m0_current_filter);
        MotorTelemetry_t state1 = updateMotor(
            &motor1, m1_velocity_filter, m1_current_filter);

        int64_t now = esp_timer_get_time();
        rate_cycles++;
        if (now - rate_timestamp >= 1000000) {
            control_hz = (float)rate_cycles * 1e6f / (float)(now - rate_timestamp);
            rate_cycles = 0;
            rate_timestamp = now;
        }

        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
        telemetry[0] = state0;
        telemetry[1] = state1;
        xSemaphoreGive(telemetry_mutex);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));
    }
}

// ============ 状态输出线程 (10Hz) ============
static void telemetry_task(void *arg) {
    while (1) {
        MotorTelemetry_t state[2];
        float loop_hz;

        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
        state[0] = telemetry[0];
        state[1] = telemetry[1];
        loop_hz = control_hz;
        xSemaphoreGive(telemetry_mutex);

        ESP_LOGI(TAG,
                 "M0 目标: %.2f, 速度: %.2f, 位置: %.2f, Iq目标: %.3f, Iq: %.3f, Uq: %.2f",
                 state[0].target_velocity, state[0].velocity, state[0].position,
                 state[0].iq_target, state[0].iq, state[0].uq);
        ESP_LOGI(TAG,
                 "M1 目标: %.2f, 速度: %.2f, 位置: %.2f, Iq目标: %.3f, Iq: %.3f, Uq: %.2f, 环频率: %.0fHz",
                 state[1].target_velocity, state[1].velocity, state[1].position,
                 state[1].iq_target, state[1].iq, state[1].uq, loop_hz);
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

extern "C" void app_main(void) {
    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor drivers enabled (GPIO %d)", MOTOR_ENABLE_PIN);

    // 两个相同地址的 AS5600 分别连接 ESP32 的 I2C0 和 I2C1
    AS5600_init(&motor0.encoder, I2C_NUM_0,
                M0_ENCODER_SDA_PIN, M0_ENCODER_SCL_PIN, 400000);
    AS5600_init(&motor1.encoder, I2C_NUM_1,
                M1_ENCODER_SDA_PIN, M1_ENCODER_SCL_PIN, 400000);

    // ESP32 的两个 MCPWM 组各驱动一台三相电机
    Motor_init(&motor0.motor, M0_MCPWM_GROUP, MOTOR_VBUS,
               M0_PWM_PIN_A, M0_PWM_PIN_B, M0_PWM_PIN_C, &motor0.encoder);
    Motor_init(&motor1.motor, M1_MCPWM_GROUP, MOTOR_VBUS,
               M1_PWM_PIN_A, M1_PWM_PIN_B, M1_PWM_PIN_C, &motor1.encoder);

    // 四路电流采样共用 ADC1；校准期间电机应保持静止
    CurrSense_init(&motor0.current_sense, M0_CURRENT_PIN_A, M0_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
    CurrSense_init(&motor1.current_sense, M1_CURRENT_PIN_A, M1_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);

    Motor_alignSensor(&motor0.motor, MOTOR_POLE_PAIRS, M0_SENSOR_DIRECTION);
    Motor_alignSensor(&motor1.motor, MOTOR_POLE_PAIRS, M1_SENSOR_DIRECTION);

    PID_init(&motor0.velocity_pid, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD,
             VELOCITY_RAMP, VELOCITY_LIMIT);
    PID_init(&motor0.current_pid, CURRENT_KP, CURRENT_KI, CURRENT_KD,
             CURRENT_RAMP, CURRENT_LIMIT);
    PID_init(&motor1.velocity_pid, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD,
             VELOCITY_RAMP, VELOCITY_LIMIT);
    PID_init(&motor1.current_pid, CURRENT_KP, CURRENT_KI, CURRENT_KD,
             CURRENT_RAMP, CURRENT_LIMIT);

    motor0.target_velocity = M0_TARGET_VELOCITY;
    motor1.target_velocity = M1_TARGET_VELOCITY;

    telemetry_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(telemetry_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    xTaskCreatePinnedToCore(control_task, "dual_control", 8192, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "双线程已启动: dual_control(1kHz) + telemetry(10Hz)");
}
