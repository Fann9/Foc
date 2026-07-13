// FOC 三线程控制器
// 线程 1：传感器读取 (500Hz) — 高优先级，实时获取编码器角度 + 速度
// 线程 2：电机控制 (100Hz) — 支持闭环位置 / 闭环速度切换
// 线程 3：串口通信 (事件驱动) — 接收目标速度 + PID 参数
//
// ==================== 控制模式切换 ====================
// 修改 CONTROL_MODE 宏即可切换, 重新编译烧录生效
#define CONTROL_MODE_POSITION  0   // 闭环位置
#define CONTROL_MODE_VELOCITY  1   // 闭环速度
#define CONTROL_MODE  CONTROL_MODE_VELOCITY   // ← 在这里切换模式
// ======================================================

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "DengFOC.h"
#include "lowpass_filter.h"
#include "pid.h"

static const char *TAG = "FOC";

#define MOTOR_ENABLE_PIN  35

// UART 配置 (IO16=ESP_TX, IO17=ESP_RX)
#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     16
#define UART_RX_PIN     17
#define UART_BAUD       115200
#define UART_BUF_SIZE   256

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

// 速度 PID 控制器
static PID_t vel_pid;

// 共享数据：传感器线程写入，控制线程读取
static float shared_angle = 0;           // 多圈累计角度
static float shared_raw_angle = 0;       // 单圈角度 (用于算电角度)
static float shared_velocity = 0;        // 滤波后速度 (rad/s)
static float shared_target = M_PI;       // 目标速度 (rad/s), 默认 π rad/s

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
    // ── 闭环速度控制 (目标由串口设定) ──

    ESP_LOGI(TAG, ">> 闭环速度模式 | 默认目标: %.2f rad/s | Kp=%.3f Ki=%.3f",
             shared_target, vel_pid.Kp, vel_pid.Ki);

    while (1) {
        float raw_angle, velocity, target;
        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        raw_angle = shared_raw_angle;
        velocity = shared_velocity;
        target = shared_target;
        xSemaphoreGive(angle_mutex);

        float el_angle = _normalizeAngle(
            (float)(motor.DIR * motor.PP) * raw_angle - motor.zero_electric_angle);

        // PID 速度控制
        float error = target - velocity;
        float Uq = PID_update(&vel_pid, error);
        Uq = _constrain(Uq, -motor.voltage_power_supply / 2, motor.voltage_power_supply / 2);

        Motor_setTorque(&motor, Uq, el_angle);

        ESP_LOGI(TAG, "目标: %.2f, 速度: %.2f, 误差: %.2f, Uq: %.2f",
                 target, velocity, error, Uq);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}

// ============ 通信线程 — 双通道: stdin (idf.py monitor) + UART ============
// 接收格式: <命令> + Enter
//   3.14  → 设置目标速度 3.14 rad/s
//   0     → 停止
//   -5    → 反向 5 rad/s
//   p0.13 → 设置 Kp
//   i0.01 → 设置 Ki
static void comm_task(void *arg) {
    // ── 初始化 UART (IO16=TX, IO17=RX) ──
    uart_config_t uart_cfg;
    memset(&uart_cfg, 0, sizeof(uart_cfg));
    uart_cfg.baud_rate = UART_BAUD;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity    = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // ── 设置 stdin 为非阻塞 ──
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    const char *help =
        "\n=== FOC Motor Controller ===\n"
        "Commands (type + Enter):\n"
        "  <number>  → set target velocity (rad/s)\n"
        "  p<number> → set Kp\n"
        "  i<number> → set Ki\n"
        "==============================\n";
    // 同时发到 stdout 和 UART
    printf("%s", help);
    uart_write_bytes(UART_PORT, help, strlen(help));

    // 两个通道各自的行缓冲区
    char buf_stdin[64], buf_uart[64];
    int len_stdin = 0, len_uart = 0;

    while (1) {
        // ── 轮询 stdin (idf.py monitor) ──
        char ch;
        int n = read(STDIN_FILENO, &ch, 1);
        if (n > 0) {
            if (ch == '\n' || ch == '\r') {
                if (len_stdin > 0) {
                    buf_stdin[len_stdin] = '\0';
                    float val = atof(buf_stdin);
                    if (buf_stdin[0] == 'p' || buf_stdin[0] == 'P') {
                        PID_setParams(&vel_pid, fabsf(val), vel_pid.Ki, vel_pid.Kd, vel_pid.output_ramp);
                        printf("Kp = %.4f\n", vel_pid.Kp);
                    } else if (buf_stdin[0] == 'i' || buf_stdin[0] == 'I') {
                        PID_setParams(&vel_pid, vel_pid.Kp, fabsf(val), vel_pid.Kd, vel_pid.output_ramp);
                        printf("Ki = %.4f\n", vel_pid.Ki);
                    } else {
                        xSemaphoreTake(angle_mutex, portMAX_DELAY);
                        shared_target = val;
                        xSemaphoreGive(angle_mutex);
                        printf("Target = %.2f rad/s\n", val);
                    }
                    len_stdin = 0;
                }
            } else if (ch == '\b' || ch == 0x7F) {
                if (len_stdin > 0) len_stdin--;
            } else if (len_stdin < (int)sizeof(buf_stdin) - 1) {
                if (ch >= 0x20 && ch < 0x7F) buf_stdin[len_stdin++] = ch;
            }
        }

        // ── 轮询 UART (IO16/17) ──
        uint8_t uch;
        int un = uart_read_bytes(UART_PORT, &uch, 1, 0);  // 非阻塞
        if (un > 0) {
            if (uch == '\n' || uch == '\r') {
                if (len_uart > 0) {
                    buf_uart[len_uart] = '\0';
                    float val = atof(buf_uart);
                    if (buf_uart[0] == 'p' || buf_uart[0] == 'P') {
                        PID_setParams(&vel_pid, fabsf(val), vel_pid.Ki, vel_pid.Kd, vel_pid.output_ramp);
                        char rsp[48];
                        snprintf(rsp, sizeof(rsp), "Kp = %.4f\r\n", vel_pid.Kp);
                        uart_write_bytes(UART_PORT, rsp, strlen(rsp));
                    } else if (buf_uart[0] == 'i' || buf_uart[0] == 'I') {
                        PID_setParams(&vel_pid, vel_pid.Kp, fabsf(val), vel_pid.Kd, vel_pid.output_ramp);
                        char rsp[48];
                        snprintf(rsp, sizeof(rsp), "Ki = %.4f\r\n", vel_pid.Ki);
                        uart_write_bytes(UART_PORT, rsp, strlen(rsp));
                    } else {
                        xSemaphoreTake(angle_mutex, portMAX_DELAY);
                        shared_target = val;
                        xSemaphoreGive(angle_mutex);
                        char rsp[48];
                        snprintf(rsp, sizeof(rsp), "Target = %.2f rad/s\r\n", val);
                        uart_write_bytes(UART_PORT, rsp, strlen(rsp));
                    }
                    len_uart = 0;
                }
            } else if (uch == '\b' || uch == 0x7F) {
                if (len_uart > 0) len_uart--;
            } else if (len_uart < (int)sizeof(buf_uart) - 1) {
                if (uch >= 0x20 && uch < 0x7F) buf_uart[len_uart++] = uch;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // 5ms 轮询周期
    }
}

// ============ 主函数 ============
extern "C" void app_main(void) {
    // 使能电机驱动
    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled (GPIO %d)", MOTOR_ENABLE_PIN);

    // 初始化编码器 (I2C, SDA=19, SCL=18, 400kHz)
    AS5600_init(&encoder, I2C_NUM_0, 15, 14, 400000);

    // 初始化电机 (12.3V, PWM: 32/33/25)
    Motor_init(&motor, 12.3f, 1, 2, 5, &encoder);

    // 电角度校准：极对数 7，方向 -1
    Motor_alignSensor(&motor, 7, -1);

    // 初始化速度 PID (限幅=供电电压/2，与 DengFOC 一致)
    PID_init(&vel_pid, 0.0f, 0.0f, 0.0f, 0.0f, motor.voltage_power_supply / 2);
    PID_setParams(&vel_pid, 0.13f, 0.0f, 0.0f, 0.0f);  // 风格同 DFOC_M0_SET_VEL_PID

    // 创建互斥锁
    angle_mutex = xSemaphoreCreateMutex();

    // 创建线程
    xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 5, NULL, 1);   // 传感器 500Hz
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 4, NULL, 1);  // 控制 100Hz
    xTaskCreatePinnedToCore(comm_task, "comm", 4096, NULL, 2, NULL, 1);        // 串口 事件驱动

    ESP_LOGI(TAG, "三线程已启动: sensor(5,500Hz) + control(4,100Hz) + comm(2,stdin+UART)");
}
