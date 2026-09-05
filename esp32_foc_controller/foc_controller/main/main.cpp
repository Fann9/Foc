// DengFOC V0.4 第九章 ESP-IDF 等价移植
// 线程 1：FOC 控制 (1kHz) — 编码器/电流采样、外环、电流环、PWM 顺序执行
// 线程 2：串口通信 (事件驱动) — 接收目标值
// 线程 3：状态输出 (20Hz) — 打印控制状态，不参与控制计算
//
// ==================== 控制模式切换 ====================
// 修改 CONTROL_MODE 宏即可切换，重新编译烧录生效
#define CONTROL_MODE_FORCE_ANGLE                 0 // 力位：位置 → Iq → Uq
#define CONTROL_MODE_VELOCITY_CURRENT            1 // 速度：速度 → Iq → Uq
#define CONTROL_MODE_POSITION_VELOCITY_CURRENT   2 // 位置：位置 → 速度 → Iq → Uq
#define CONTROL_MODE_CURRENT                     3 // 电流力矩：Iq → Uq
#define CONTROL_MODE CONTROL_MODE_POSITION_VELOCITY_CURRENT
// ======================================================

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "DengFOC.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "current_sense.h"

static const char *TAG = "FOC";

// ESP32 驱动板引脚
#define MOTOR_ENABLE_PIN       12
#define MOTOR_PWM_PIN_A        32
#define MOTOR_PWM_PIN_B        33
#define MOTOR_PWM_PIN_C        25
#define ENCODER_SDA_PIN        19
#define ENCODER_SCL_PIN        18
#define CURRENT_SENSE_PIN_A    39 // VN / ADC1
#define CURRENT_SENSE_PIN_B    36 // VP / ADC1

// UART 配置 (IO16=ESP_TX, IO17=ESP_RX)
#define UART_PORT              UART_NUM_1
#define UART_TX_PIN            16
#define UART_RX_PIN            17
#define UART_BAUD              115200
#define UART_BUF_SIZE          256

// Arduino 示例参数
#define MOTOR_VBUS             12.6f
#define MOTOR_POLE_PAIRS       7
#define SENSOR_DIRECTION       -1
#define CURRENT_SHUNT_RESISTOR 0.01f
#define CURRENT_AMP_GAIN       50.0f
#define CONTROL_LOOP_HZ        1000
#define TELEMETRY_PERIOD_MS    50
#define DEG_PER_RAD            (180.0f / M_PI)

#if CONTROL_MODE == CONTROL_MODE_FORCE_ANGLE
#define CONTROL_MODE_NAME      "FORCE_ANGLE_CURRENT"
#define ANGLE_KP               0.5f
#define ANGLE_KI               0.0f
#define ANGLE_KD               0.003f
#define ANGLE_RAMP             100000.0f
#define ANGLE_LIMIT            0.1f
#define VELOCITY_KP            0.0f
#define VELOCITY_KI            0.0f
#define VELOCITY_KD            0.0f
#define VELOCITY_RAMP          100000.0f
#define VELOCITY_LIMIT         0.5f
#define CURRENT_KP             1.25f
#define CURRENT_KI             50.0f
#elif CONTROL_MODE == CONTROL_MODE_VELOCITY_CURRENT
#define CONTROL_MODE_NAME      "VELOCITY_CURRENT"
#define ANGLE_KP               0.0f
#define ANGLE_KI               0.0f
#define ANGLE_KD               0.0f
#define ANGLE_RAMP             100000.0f
#define ANGLE_LIMIT            100.0f
#define VELOCITY_KP            3.0f
#define VELOCITY_KI            2.0f
#define VELOCITY_KD            0.0f
#define VELOCITY_RAMP          100000.0f
#define VELOCITY_LIMIT         0.5f
#define CURRENT_KP             0.5f
#define CURRENT_KI             50.0f
#elif CONTROL_MODE == CONTROL_MODE_POSITION_VELOCITY_CURRENT
#define CONTROL_MODE_NAME      "POSITION_VELOCITY_CURRENT"
#define ANGLE_KP               1.0f
#define ANGLE_KI               0.0f
#define ANGLE_KD               0.0f
#define ANGLE_RAMP             100000.0f
#define ANGLE_LIMIT            30.0f
#define VELOCITY_KP            0.02f
#define VELOCITY_KI            1.0f
#define VELOCITY_KD            0.0f
#define VELOCITY_RAMP          100000.0f
#define VELOCITY_LIMIT         0.5f
#define CURRENT_KP             5.0f
#define CURRENT_KI             200.0f
#elif CONTROL_MODE == CONTROL_MODE_CURRENT
#define CONTROL_MODE_NAME      "CURRENT"
#define ANGLE_KP               0.0f
#define ANGLE_KI               0.0f
#define ANGLE_KD               0.0f
#define ANGLE_RAMP             100000.0f
#define ANGLE_LIMIT            100.0f
#define VELOCITY_KP            0.0f
#define VELOCITY_KI            0.0f
#define VELOCITY_KD            0.0f
#define VELOCITY_RAMP          100000.0f
#define VELOCITY_LIMIT         0.5f
#define CURRENT_KP             5.0f
#define CURRENT_KI             200.0f
#else
#error "Invalid CONTROL_MODE"
#endif

#define CURRENT_KD             0.0f
#define CURRENT_RAMP           100000.0f
#define CURRENT_LIMIT          MOTOR_VBUS

static float normalizeAngle(float angle) {
    float value = fmodf(angle, 2.0f * M_PI);
    return value >= 0.0f ? value : value + 2.0f * M_PI;
}

// ============ 全局实例 ============
static AS5600_t encoder;
static Motor_t motor;
static CurrSense_t current_sense;
static PID_t angle_pid;
static PID_t velocity_pid;
static PID_t current_pid;
static SemaphoreHandle_t state_mutex;

// 与 Arduino V0.4 一致：速度 Tf=10ms，电流 Tf=50ms
static LowPassFilter velocity_filter(0.01f);
static LowPassFilter current_filter(0.05f);

// 串口任务写目标，控制任务写其余状态，日志任务只读
static float shared_target = 0.0f;
static float shared_position = 0.0f;
static float shared_velocity_target = 0.0f;
static float shared_velocity = 0.0f;
static float shared_iq_target = 0.0f;
static float shared_iq = 0.0f;
static float shared_uq = 0.0f;
static float shared_control_hz = 0.0f;

// ============ FOC 控制线程 (1kHz) ============
static void control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    float raw_angle = AS5600_getAngle_Without_track(&encoder);
    encoder.angle_prev = raw_angle;
    float full_angle_prev = AS5600_trackAngle(&encoder, raw_angle);
    int64_t velocity_timestamp_prev = esp_timer_get_time();
    int64_t rate_timestamp = velocity_timestamp_prev;
    uint32_t rate_cycles = 0;

    float position = motor.DIR * full_angle_prev;
    float velocity = 0.0f;
    float velocity_target = 0.0f;
    float iq_target = 0.0f;
    float iq = 0.0f;
    float uq = 0.0f;
    float control_hz = 0.0f;

    ESP_LOGI(TAG, ">> Arduino V0.4 等价控制启动: mode=%s, loop=%dHz",
             CONTROL_MODE_NAME, CONTROL_LOOP_HZ);

    while (1) {
        // 等价于 Arduino runFOC()：先更新编码器，再读取相电流
        raw_angle = AS5600_getAngle_Without_track(&encoder);
        float full_angle = AS5600_trackAngle(&encoder, raw_angle);
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - velocity_timestamp_prev) * 1e-6f;
        if (dt <= 0.0f || dt > 0.5f) dt = 1e-3f;

        position = motor.DIR * full_angle;
        velocity = velocity_filter(motor.DIR * (full_angle - full_angle_prev) / dt);
        float electrical_angle = normalizeAngle(
            (float)(motor.DIR * motor.PP) * raw_angle - motor.zero_electric_angle);
        iq = current_filter(CurrSense_getIq(&current_sense, electrical_angle));

        full_angle_prev = full_angle;
        velocity_timestamp_prev = now;

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        float target = shared_target;
        xSemaphoreGive(state_mutex);

        // 以下各控制路径按 Arduino DengFOC.cpp 的调用和单位原样复现
#if CONTROL_MODE == CONTROL_MODE_FORCE_ANGLE
        velocity_target = 0.0f;
        iq_target = PID_update(&angle_pid, (target - position) * DEG_PER_RAD);
#elif CONTROL_MODE == CONTROL_MODE_VELOCITY_CURRENT
        velocity_target = target;
        iq_target = PID_update(&velocity_pid, (target - velocity) * DEG_PER_RAD);
#elif CONTROL_MODE == CONTROL_MODE_POSITION_VELOCITY_CURRENT
        velocity_target = PID_update(&angle_pid, (target - position) * DEG_PER_RAD);
        iq_target = PID_update(&velocity_pid, velocity_target - velocity);
#elif CONTROL_MODE == CONTROL_MODE_CURRENT
        velocity_target = 0.0f;
        iq_target = target;
#endif

        uq = PID_update(&current_pid, iq_target - iq);
        Motor_setTorque(&motor, uq, electrical_angle);

        rate_cycles++;
        if (now - rate_timestamp >= 1000000) {
            control_hz = (float)rate_cycles * 1e6f / (float)(now - rate_timestamp);
            rate_cycles = 0;
            rate_timestamp = now;
        }

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        shared_position = position;
        shared_velocity_target = velocity_target;
        shared_velocity = velocity;
        shared_iq_target = iq_target;
        shared_iq = iq;
        shared_uq = uq;
        shared_control_hz = control_hz;
        xSemaphoreGive(state_mutex);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));
    }
}

// ============ 串口命令 ============
static bool parseNumber(const char *text, float *value) {
    char *end = NULL;
    *value = strtof(text, &end);
    if (end == text || !isfinite(*value)) return false;
    while (*end != '\0' && isspace((unsigned char)*end)) end++;
    return *end == '\0';
}

static void processCommand(const char *command, char *response, size_t response_size) {
    if ((command[0] == 'q' || command[0] == 'Q') && command[1] == '\0') {
        snprintf(response, response_size,
                 "A %.3f/%.3f V %.3f/%.3f C %.3f/%.3f",
                 angle_pid.Kp, angle_pid.Ki,
                 velocity_pid.Kp, velocity_pid.Ki,
                 current_pid.Kp, current_pid.Ki);
        return;
    }

    if ((command[0] == 's' || command[0] == 'S') && command[1] == '\0') {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        shared_target = 0.0f;
        xSemaphoreGive(state_mutex);
        snprintf(response, response_size, "Target = 0.00");
        return;
    }

    float value;
    if (!parseNumber(command, &value)) {
        snprintf(response, response_size, "Invalid command");
        return;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    shared_target = value;
    xSemaphoreGive(state_mutex);
    snprintf(response, response_size, "Target = %.2f", value);
}

static void handleInput(char value, char *buffer, int *length, bool uart_output) {
    if (value == '\n' || value == '\r') {
        if (*length > 0) {
            buffer[*length] = '\0';
            char response[96];
            processCommand(buffer, response, sizeof(response));
            if (uart_output) {
                uart_write_bytes(UART_PORT, response, strlen(response));
                uart_write_bytes(UART_PORT, "\r\n", 2);
            } else {
                printf("%s\n", response);
            }
            *length = 0;
        }
    } else if (value == '\b' || value == 0x7f) {
        if (*length > 0) (*length)--;
    } else if (value >= 0x20 && value < 0x7f && *length < 63) {
        buffer[(*length)++] = value;
    }
}

static void comm_task(void *arg) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = UART_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    const char *help =
        "\n=== DengFOC V0.4 IDF ===\n"
        "  <number>  set target\n"
        "  s         set target to zero\n"
        "  q         show Arduino PID values\n";
    printf("%s", help);
    uart_write_bytes(UART_PORT, help, strlen(help));

    char stdin_buffer[64] = {};
    char uart_buffer[64] = {};
    int stdin_length = 0;
    int uart_length = 0;

    while (1) {
        char value;
        if (read(STDIN_FILENO, &value, 1) > 0) {
            handleInput(value, stdin_buffer, &stdin_length, false);
        }

        uint8_t uart_value;
        if (uart_read_bytes(UART_PORT, &uart_value, 1, 0) > 0) {
            handleInput((char)uart_value, uart_buffer, &uart_length, true);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============ 状态输出线程 (20Hz) ============
static void telemetry_task(void *arg) {
    while (1) {
        float target, position, velocity_target, velocity, iq_target, iq, uq, control_hz;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        target = shared_target;
        position = shared_position;
        velocity_target = shared_velocity_target;
        velocity = shared_velocity;
        iq_target = shared_iq_target;
        iq = shared_iq;
        uq = shared_uq;
        control_hz = shared_control_hz;
        xSemaphoreGive(state_mutex);

        ESP_LOGI(TAG,
                 "模式: %s, 环频率: %.0fHz, 目标: %.2f, 位置: %.2f, "
                 "速度目标: %.2f, 速度: %.2f, Iq目标: %.3f, Iq: %.3f, Uq: %.2f",
                 CONTROL_MODE_NAME, control_hz, target, position,
                 velocity_target, velocity, iq_target, iq, uq);
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

// ============ 主函数 ============
extern "C" void app_main(void) {
    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled (GPIO %d)", MOTOR_ENABLE_PIN);

    AS5600_init(&encoder, I2C_NUM_0, ENCODER_SDA_PIN, ENCODER_SCL_PIN, 400000);
    Motor_init(&motor, MOTOR_VBUS,
               MOTOR_PWM_PIN_A, MOTOR_PWM_PIN_B, MOTOR_PWM_PIN_C, &encoder);

    // 与 Arduino 一致：先在零输出状态校准电流偏置，再进行电角度对齐
    CurrSense_init(&current_sense, CURRENT_SENSE_PIN_A, CURRENT_SENSE_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
    Motor_alignSensor(&motor, MOTOR_POLE_PAIRS, SENSOR_DIRECTION);

    PID_init(&angle_pid, ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_RAMP, ANGLE_LIMIT);
    PID_init(&velocity_pid, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD,
             VELOCITY_RAMP, VELOCITY_LIMIT);
    PID_init(&current_pid, CURRENT_KP, CURRENT_KI, CURRENT_KD,
             CURRENT_RAMP, CURRENT_LIMIT);

    state_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(state_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(comm_task, "comm", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 3072, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "三线程已启动: control(1kHz) + comm + telemetry(20Hz)");
}
