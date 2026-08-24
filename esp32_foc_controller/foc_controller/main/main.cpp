// DengFOC 双电机 ESP-IDF 控制器
// 控制线程：1kHz 顺序执行传感器采样、速度环、电流环和 PWM
// 通信线程：USB 控制台与 UART1 命令解析
// 日志线程：按配置频率输出状态，不参与控制计算

// ==================== 控制模式切换 ====================
#define CONTROL_MODE_DUAL_VELOCITY       0 // M0/M1 独立速度-电流闭环
#define CONTROL_MODE_MASTER_SLAVE_SPEED  1 // M0 手轮输入，M1 单向调速
#define CONTROL_MODE CONTROL_MODE_MASTER_SLAVE_SPEED
// ======================================================

#define _USE_MATH_DEFINES
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
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
#define M0_SENSOR_DIRECTION    -1
#define M0_DEFAULT_VELOCITY    5.0f

#define M1_PWM_PIN_A           26
#define M1_PWM_PIN_B           27
#define M1_PWM_PIN_C           14
#define M1_ENCODER_SDA_PIN     23
#define M1_ENCODER_SCL_PIN     5
#define M1_CURRENT_PIN_A       35
#define M1_CURRENT_PIN_B       34
#define M1_MCPWM_GROUP         1
#define M1_SENSOR_DIRECTION    -1
#define M1_DEFAULT_VELOCITY    7.0f
// ========================================================

// UART1：独立于 ESP-IDF 默认日志串口
#define UART_PORT              UART_NUM_1
#define UART_TX_PIN            16
#define UART_RX_PIN            17
#define UART_BAUD              115200
#define UART_RX_BUF_SIZE       512
#define COMMAND_LINE_SIZE      160
#define COMMAND_QUEUE_LENGTH   12

// 公共控制参数，保持已验证参数不变
#define MOTOR_VBUS             12.6f
#define MOTOR_POLE_PAIRS       7
#define CURRENT_SHUNT_RESISTOR 0.01f
#define CURRENT_AMP_GAIN       50.0f
#define CONTROL_LOOP_HZ        1000
#define DEFAULT_LOG_HZ         10
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

// 主从模式：M0 每转一圈改变 5rad/s，M1 目标限制为 0~10rad/s
#define LINK_DEFAULT_GAIN      5.0f
#define LINK_DEFAULT_MAX_SPEED 10.0f
#define MASTER_INPUT_DEADBAND  0.02f
#define MASTER_INPUT_DIRECTION M0_SENSOR_DIRECTION
#define ABSOLUTE_MAX_SPEED     30.0f
#define ABSOLUTE_MAX_IQ        1.0f

#if CONTROL_MODE == CONTROL_MODE_DUAL_VELOCITY
#define CONTROL_MODE_NAME "DUAL_VELOCITY"
#elif CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
#define CONTROL_MODE_NAME "MASTER_SLAVE_SPEED"
#else
#error "Invalid CONTROL_MODE"
#endif

static float constrainValue(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

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
    bool running;
} MotorControl_t;

typedef struct {
    bool running;
    float target_velocity;
    float position;
    float velocity;
    float iq_target;
    float iq;
    float uq;
} MotorTelemetry_t;

typedef struct {
    float p;
    float i;
    float d;
    float ramp;
    float limit;
} PidParameters_t;

typedef struct {
    MotorTelemetry_t motor[2];
    PidParameters_t velocity_pid[2];
    PidParameters_t current_pid[2];
    float control_hz;
    float link_gain;
    float link_max_speed;
    float link_pending_angle;
    bool master_input_enabled;
} SystemSnapshot_t;

typedef enum {
    CMD_SET_VELOCITY,
    CMD_SET_VELOCITY_PID,
    CMD_SET_CURRENT_PID,
    CMD_SET_RUNNING,
    CMD_SET_LINK_GAIN,
    CMD_SET_LINK_MAX,
    CMD_ZERO_LINK,
} CommandType_t;

typedef struct {
    CommandType_t type;
    uint8_t motor_mask;
    bool enabled;
    float value[5];
} ControlCommand_t;

#define MOTOR_MASK_M0  0x01
#define MOTOR_MASK_M1  0x02
#define MOTOR_MASK_ALL (MOTOR_MASK_M0 | MOTOR_MASK_M1)

static MotorControl_t motor0;
static MotorControl_t motor1;
static SystemSnapshot_t snapshot;
static SemaphoreHandle_t snapshot_mutex;
static SemaphoreHandle_t uart_tx_mutex;
static QueueHandle_t command_queue;
static volatile uint32_t log_hz = DEFAULT_LOG_HZ;

static float link_gain = LINK_DEFAULT_GAIN;
static float link_max_speed = LINK_DEFAULT_MAX_SPEED;
static float link_pending_angle = 0.0f;
static bool master_input_enabled = true;

// 与 Arduino 示例一致：每台电机速度 Tf=10ms，电流 Tf=50ms
static LowPassFilter m0_velocity_filter(0.01f);
static LowPassFilter m0_current_filter(0.05f);
static LowPassFilter m1_velocity_filter(0.01f);
static LowPassFilter m1_current_filter(0.05f);

static PidParameters_t getPidParameters(const PID_t *pid) {
    PidParameters_t params = {};
    params.p = pid->Kp;
    params.i = pid->Ki;
    params.d = pid->Kd;
    params.ramp = pid->output_ramp;
    params.limit = pid->limit;
    return params;
}

static void initControlState(MotorControl_t *control) {
    float raw_angle = AS5600_getAngle_Without_track(&control->encoder);
    control->encoder.angle_prev = raw_angle;
    control->full_angle_prev = AS5600_trackAngle(&control->encoder, raw_angle);
    control->velocity_timestamp_prev = esp_timer_get_time();
}

static MotorTelemetry_t updateClosedLoopMotor(MotorControl_t *control,
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
    float iq_target = 0.0f;
    float uq = 0.0f;

    if (control->running) {
        iq_target = PID_update(
            &control->velocity_pid,
            (control->target_velocity - velocity) * DEG_PER_RAD);
        uq = PID_update(&control->current_pid, iq_target - iq);
    }
    Motor_setTorque(&control->motor, uq, electrical_angle);

    control->full_angle_prev = full_angle;
    control->velocity_timestamp_prev = now;

    MotorTelemetry_t state = {};
    state.running = control->running;
    state.target_velocity = control->target_velocity;
    state.position = position;
    state.velocity = velocity;
    state.iq_target = iq_target;
    state.iq = iq;
    state.uq = uq;
    return state;
}

#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
static MotorTelemetry_t updateMasterInput(void) {
    float raw_angle = AS5600_getAngle_Without_track(&motor0.encoder);
    float full_angle = AS5600_trackAngle(&motor0.encoder, raw_angle);
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - motor0.velocity_timestamp_prev) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 1e-3f;

    float delta_angle = MASTER_INPUT_DIRECTION * (full_angle - motor0.full_angle_prev);
    float position = MASTER_INPUT_DIRECTION * full_angle;
    float velocity = m0_velocity_filter(delta_angle / dt);

    if (master_input_enabled) {
        link_pending_angle += delta_angle;
        if (fabsf(link_pending_angle) >= MASTER_INPUT_DEADBAND) {
            motor1.target_velocity +=
                link_pending_angle * link_gain / (2.0f * M_PI);
            motor1.target_velocity = constrainValue(
                motor1.target_velocity, 0.0f, link_max_speed);
            link_pending_angle = 0.0f;
        }
    }

    motor0.full_angle_prev = full_angle;
    motor0.velocity_timestamp_prev = now;

    MotorTelemetry_t state = {};
    state.running = master_input_enabled;
    state.position = position;
    state.velocity = velocity;
    return state;
}
#endif

static void setMotorRunning(MotorControl_t *control, bool enabled) {
    control->running = enabled;
    if (!enabled) control->target_velocity = 0.0f;
    PID_reset(&control->velocity_pid);
    PID_reset(&control->current_pid);
}

static void setPidParameters(PID_t *pid, const float value[5]) {
    PID_setParams(pid, value[0], value[1], value[2], value[3]);
    PID_setLimit(pid, value[4]);
    PID_reset(pid);
}

static void applyCommand(const ControlCommand_t *command) {
    switch (command->type) {
        case CMD_SET_VELOCITY:
            if (command->motor_mask & MOTOR_MASK_M0)
                motor0.target_velocity = command->value[0];
            if (command->motor_mask & MOTOR_MASK_M1) {
                float target = command->value[command->motor_mask == MOTOR_MASK_ALL ? 1 : 0];
#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
                target = constrainValue(target, 0.0f, link_max_speed);
#endif
                motor1.target_velocity = target;
            }
            break;

        case CMD_SET_VELOCITY_PID:
            if (command->motor_mask & MOTOR_MASK_M0)
                setPidParameters(&motor0.velocity_pid, command->value);
            if (command->motor_mask & MOTOR_MASK_M1)
                setPidParameters(&motor1.velocity_pid, command->value);
            break;

        case CMD_SET_CURRENT_PID:
            if (command->motor_mask & MOTOR_MASK_M0)
                setPidParameters(&motor0.current_pid, command->value);
            if (command->motor_mask & MOTOR_MASK_M1)
                setPidParameters(&motor1.current_pid, command->value);
            break;

        case CMD_SET_RUNNING:
#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
            if (command->motor_mask & MOTOR_MASK_M0) {
                master_input_enabled = command->enabled;
                link_pending_angle = 0.0f;
            }
            if (command->motor_mask & MOTOR_MASK_M1)
                setMotorRunning(&motor1, command->enabled);
#else
            if (command->motor_mask & MOTOR_MASK_M0)
                setMotorRunning(&motor0, command->enabled);
            if (command->motor_mask & MOTOR_MASK_M1)
                setMotorRunning(&motor1, command->enabled);
#endif
            break;

        case CMD_SET_LINK_GAIN:
            link_gain = command->value[0];
            break;

        case CMD_SET_LINK_MAX:
            link_max_speed = command->value[0];
            motor1.target_velocity = constrainValue(
                motor1.target_velocity, 0.0f, link_max_speed);
            break;

        case CMD_ZERO_LINK:
            link_pending_angle = 0.0f;
            motor1.target_velocity = 0.0f;
            PID_reset(&motor1.velocity_pid);
            PID_reset(&motor1.current_pid);
            break;
    }
}

static void updateSnapshot(const MotorTelemetry_t state[2], float current_hz) {
    xSemaphoreTake(snapshot_mutex, portMAX_DELAY);
    snapshot.motor[0] = state[0];
    snapshot.motor[1] = state[1];
    snapshot.velocity_pid[0] = getPidParameters(&motor0.velocity_pid);
    snapshot.velocity_pid[1] = getPidParameters(&motor1.velocity_pid);
    snapshot.current_pid[0] = getPidParameters(&motor0.current_pid);
    snapshot.current_pid[1] = getPidParameters(&motor1.current_pid);
    snapshot.control_hz = current_hz;
    snapshot.link_gain = link_gain;
    snapshot.link_max_speed = link_max_speed;
    snapshot.link_pending_angle = link_pending_angle;
    snapshot.master_input_enabled = master_input_enabled;
    xSemaphoreGive(snapshot_mutex);
}

// ============ 双电机 FOC 控制线程 (1kHz) ============
static void control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    int64_t rate_timestamp = esp_timer_get_time();
    uint32_t rate_cycles = 0;
    float current_hz = 0.0f;

    initControlState(&motor0);
    initControlState(&motor1);
    ESP_LOGI(TAG, ">> mode=%s, loop=%dHz", CONTROL_MODE_NAME, CONTROL_LOOP_HZ);

    while (1) {
        ControlCommand_t command;
        while (xQueueReceive(command_queue, &command, 0) == pdTRUE)
            applyCommand(&command);

        MotorTelemetry_t state[2];
#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
        state[0] = updateMasterInput();
        state[1] = updateClosedLoopMotor(
            &motor1, m1_velocity_filter, m1_current_filter);
#else
        state[0] = updateClosedLoopMotor(
            &motor0, m0_velocity_filter, m0_current_filter);
        state[1] = updateClosedLoopMotor(
            &motor1, m1_velocity_filter, m1_current_filter);
#endif

        int64_t now = esp_timer_get_time();
        rate_cycles++;
        if (now - rate_timestamp >= 1000000) {
            current_hz = (float)rate_cycles * 1e6f / (float)(now - rate_timestamp);
            rate_cycles = 0;
            rate_timestamp = now;
        }
        updateSnapshot(state, current_hz);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));
    }
}

static bool tokenEquals(const char *left, const char *right) {
    while (*left != '\0' && *right != '\0') {
        if (toupper((unsigned char)*left) != toupper((unsigned char)*right))
            return false;
        left++;
        right++;
    }
    return *left == '\0' && *right == '\0';
}

static int splitTokens(char *line, char *token[], int capacity) {
    int count = 0;
    char *save = NULL;
    char *item = strtok_r(line, " \t", &save);
    while (item != NULL && count < capacity) {
        token[count++] = item;
        item = strtok_r(NULL, " \t", &save);
    }
    return count;
}

static bool parseFloat(const char *text, float *value) {
    char *end = NULL;
    *value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(*value);
}

static uint8_t parseMotor(const char *text) {
    if (tokenEquals(text, "M0")) return MOTOR_MASK_M0;
    if (tokenEquals(text, "M1")) return MOTOR_MASK_M1;
    if (tokenEquals(text, "ALL")) return MOTOR_MASK_ALL;
    return 0;
}

static void writeUartLine(const char *line) {
    xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);
    uart_write_bytes(UART_PORT, line, strlen(line));
    uart_write_bytes(UART_PORT, "\r\n", 2);
    xSemaphoreGive(uart_tx_mutex);
}

static void sendResponse(bool uart_output, const char *format, ...) {
    char line[256];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    if (uart_output)
        writeUartLine(line);
    else
        printf("%s\n", line);
}

static bool enqueueCommand(const ControlCommand_t *command, bool uart_output) {
    if (xQueueSend(command_queue, command, 0) != pdTRUE) {
        sendResponse(uart_output, "ERR BUSY");
        return false;
    }
    return true;
}

static SystemSnapshot_t readSnapshot(void) {
    SystemSnapshot_t value;
    xSemaphoreTake(snapshot_mutex, portMAX_DELAY);
    value = snapshot;
    xSemaphoreGive(snapshot_mutex);
    return value;
}

static void sendState(bool uart_output, int motor_index, const SystemSnapshot_t *state) {
    const MotorTelemetry_t *motor = &state->motor[motor_index];
    const char *run_state = motor->running ? "RUN" : "STOP";
#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
    if (motor_index == 0) run_state = motor->running ? "INPUT" : "PAUSE";
#endif
    sendResponse(uart_output,
                 "STATE M%d %s VEL_T=%.3f VEL=%.3f POS=%.3f IQ_T=%.3f IQ=%.3f UQ=%.3f",
                 motor_index, run_state, motor->target_velocity, motor->velocity,
                 motor->position, motor->iq_target, motor->iq, motor->uq);
}

static void sendParameters(bool uart_output, int motor_index,
                           const SystemSnapshot_t *state) {
    const PidParameters_t *v = &state->velocity_pid[motor_index];
    const PidParameters_t *c = &state->current_pid[motor_index];
    sendResponse(uart_output,
                 "PARAM M%d VPID=%.3f,%.3f,%.3f,%.1f,%.3f CPID=%.3f,%.3f,%.3f,%.1f,%.3f",
                 motor_index, v->p, v->i, v->d, v->ramp, v->limit,
                 c->p, c->i, c->d, c->ramp, c->limit);
}

static void sendHelp(bool uart_output) {
    sendResponse(uart_output, "OK COMMANDS");
    sendResponse(uart_output, "RUN|STOP M0|M1|ALL");
    sendResponse(uart_output, "SET M0|M1 VEL value");
    sendResponse(uart_output, "SET ALL VEL m0_value m1_value");
    sendResponse(uart_output, "SET M0|M1|ALL VPID|CPID P I D RAMP LIMIT");
    sendResponse(uart_output, "SET LINK GAIN value | SET LINK MAX value | ZERO LINK");
    sendResponse(uart_output, "GET M0|M1|ALL STATE|PARAM | GET MODE | GET LINK");
    sendResponse(uart_output, "LOG 1..100 | LOG OFF | HELP");
}

static void processCommand(char *line, bool uart_output) {
    char *token[10];
    int count = splitTokens(line, token, 10);
    if (count == 0) return;

    if (count == 1 && tokenEquals(token[0], "HELP")) {
        sendHelp(uart_output);
        return;
    }

    if (count == 2 && tokenEquals(token[0], "LOG")) {
        if (tokenEquals(token[1], "OFF")) {
            log_hz = 0;
            sendResponse(uart_output, "OK LOG OFF");
            return;
        }
        float value;
        if (!parseFloat(token[1], &value) || value < 1.0f || value > 100.0f) {
            sendResponse(uart_output, "ERR BAD_VALUE");
            return;
        }
        log_hz = (uint32_t)value;
        sendResponse(uart_output, "OK LOG %u", (unsigned)log_hz);
        return;
    }

    if (count == 2 && (tokenEquals(token[0], "RUN") || tokenEquals(token[0], "STOP"))) {
        uint8_t motor_mask = parseMotor(token[1]);
        if (motor_mask == 0) {
            sendResponse(uart_output, "ERR BAD_MOTOR");
            return;
        }
        ControlCommand_t command = {};
        command.type = CMD_SET_RUNNING;
        command.motor_mask = motor_mask;
        command.enabled = tokenEquals(token[0], "RUN");
        if (enqueueCommand(&command, uart_output))
            sendResponse(uart_output, "OK %s %s", command.enabled ? "RUN" : "STOP", token[1]);
        return;
    }

    if (count == 2 && tokenEquals(token[0], "ZERO") && tokenEquals(token[1], "LINK")) {
        ControlCommand_t command = {};
        command.type = CMD_ZERO_LINK;
        if (enqueueCommand(&command, uart_output))
            sendResponse(uart_output, "OK ZERO LINK");
        return;
    }

    if (tokenEquals(token[0], "GET")) {
        SystemSnapshot_t state = readSnapshot();
        if (count == 2 && tokenEquals(token[1], "MODE")) {
            sendResponse(uart_output, "MODE %s", CONTROL_MODE_NAME);
            return;
        }
        if (count == 2 && tokenEquals(token[1], "LINK")) {
            sendResponse(uart_output, "LINK GAIN=%.3f MAX=%.3f TARGET=%.3f PENDING=%.4f",
                         state.link_gain, state.link_max_speed,
                         state.motor[1].target_velocity, state.link_pending_angle);
            return;
        }
        if (count == 3) {
            uint8_t motor_mask = parseMotor(token[1]);
            if (motor_mask == 0) {
                sendResponse(uart_output, "ERR BAD_MOTOR");
                return;
            }
            if (tokenEquals(token[2], "STATE")) {
                if (motor_mask & MOTOR_MASK_M0) sendState(uart_output, 0, &state);
                if (motor_mask & MOTOR_MASK_M1) sendState(uart_output, 1, &state);
                return;
            }
            if (tokenEquals(token[2], "PARAM")) {
                if (motor_mask & MOTOR_MASK_M0) sendParameters(uart_output, 0, &state);
                if (motor_mask & MOTOR_MASK_M1) sendParameters(uart_output, 1, &state);
                return;
            }
        }
        sendResponse(uart_output, "ERR BAD_COMMAND");
        return;
    }

    if (tokenEquals(token[0], "SET")) {
        if (count == 4 && tokenEquals(token[1], "LINK")) {
            float value;
            if (!parseFloat(token[3], &value) || value <= 0.0f
                || (tokenEquals(token[2], "MAX") && value > ABSOLUTE_MAX_SPEED)
                || (tokenEquals(token[2], "GAIN") && value > 100.0f)) {
                sendResponse(uart_output, "ERR BAD_VALUE");
                return;
            }
            ControlCommand_t command = {};
            if (tokenEquals(token[2], "GAIN"))
                command.type = CMD_SET_LINK_GAIN;
            else if (tokenEquals(token[2], "MAX"))
                command.type = CMD_SET_LINK_MAX;
            else {
                sendResponse(uart_output, "ERR BAD_COMMAND");
                return;
            }
            command.value[0] = value;
            if (enqueueCommand(&command, uart_output))
                sendResponse(uart_output, "OK SET LINK %s %.3f", token[2], value);
            return;
        }

        if (count >= 4) {
            uint8_t motor_mask = parseMotor(token[1]);
            if (motor_mask == 0) {
                sendResponse(uart_output, "ERR BAD_MOTOR");
                return;
            }

            if (tokenEquals(token[2], "VEL")) {
                int expected = motor_mask == MOTOR_MASK_ALL ? 5 : 4;
                if (count != expected) {
                    sendResponse(uart_output, "ERR ARG_COUNT");
                    return;
                }
#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
                if (motor_mask & MOTOR_MASK_M0) {
                    sendResponse(uart_output, "ERR M0_INPUT_ONLY");
                    return;
                }
#endif
                ControlCommand_t command = {};
                command.type = CMD_SET_VELOCITY;
                command.motor_mask = motor_mask;
                int value_count = motor_mask == MOTOR_MASK_ALL ? 2 : 1;
                for (int i = 0; i < value_count; i++) {
                    if (!parseFloat(token[3 + i], &command.value[i])) {
                        sendResponse(uart_output, "ERR BAD_VALUE");
                        return;
                    }
#if CONTROL_MODE == CONTROL_MODE_DUAL_VELOCITY
                    if (fabsf(command.value[i]) > ABSOLUTE_MAX_SPEED) {
                        sendResponse(uart_output, "ERR BAD_VALUE");
                        return;
                    }
#endif
                }
                if (enqueueCommand(&command, uart_output))
                    sendResponse(uart_output, "OK SET %s VEL", token[1]);
                return;
            }

            if (tokenEquals(token[2], "VPID") || tokenEquals(token[2], "CPID")) {
                if (count != 8) {
                    sendResponse(uart_output, "ERR ARG_COUNT");
                    return;
                }
                ControlCommand_t command = {};
                command.type = tokenEquals(token[2], "VPID")
                    ? CMD_SET_VELOCITY_PID : CMD_SET_CURRENT_PID;
                command.motor_mask = motor_mask;
                for (int i = 0; i < 5; i++) {
                    if (!parseFloat(token[3 + i], &command.value[i])) {
                        sendResponse(uart_output, "ERR BAD_VALUE");
                        return;
                    }
                }
                if (command.value[3] < 0.0f || command.value[4] <= 0.0f) {
                    sendResponse(uart_output, "ERR BAD_VALUE");
                    return;
                }
                if (command.type == CMD_SET_VELOCITY_PID
                    && command.value[4] > ABSOLUTE_MAX_IQ) {
                    sendResponse(uart_output, "ERR BAD_VALUE");
                    return;
                }
                if (command.type == CMD_SET_CURRENT_PID
                    && command.value[4] > MOTOR_VBUS) {
                    sendResponse(uart_output, "ERR BAD_VALUE");
                    return;
                }
                if (enqueueCommand(&command, uart_output))
                    sendResponse(uart_output, "OK SET %s %s", token[1], token[2]);
                return;
            }
        }
    }

    sendResponse(uart_output, "ERR BAD_COMMAND");
}

static void handleInput(char value, char *buffer, int *length, bool uart_output) {
    if (value == '\n' || value == '\r') {
        if (*length > 0) {
            buffer[*length] = '\0';
            processCommand(buffer, uart_output);
            *length = 0;
        }
    } else if (value == '\b' || value == 0x7f) {
        if (*length > 0) (*length)--;
    } else if (value >= 0x20 && value < 0x7f) {
        if (*length < COMMAND_LINE_SIZE - 1)
            buffer[(*length)++] = value;
        else {
            *length = 0;
            sendResponse(uart_output, "ERR LINE_TOO_LONG");
        }
    }
}

static void comm_task(void *arg) {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    char stdin_buffer[COMMAND_LINE_SIZE] = {};
    char uart_buffer[COMMAND_LINE_SIZE] = {};
    int stdin_length = 0;
    int uart_length = 0;

    sendHelp(false);
    sendHelp(true);

    while (1) {
        char value;
        if (read(STDIN_FILENO, &value, 1) > 0)
            handleInput(value, stdin_buffer, &stdin_length, false);

        uint8_t uart_value;
        if (uart_read_bytes(UART_PORT, &uart_value, 1, 0) > 0)
            handleInput((char)uart_value, uart_buffer, &uart_length, true);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void outputTelemetryLine(const char *line) {
    ESP_LOGI(TAG, "%s", line);
    writeUartLine(line);
}

// ============ 状态输出线程 ============
static void telemetry_task(void *arg) {
    while (1) {
        uint32_t current_log_hz = log_hz;
        if (current_log_hz == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        SystemSnapshot_t state = readSnapshot();
        char line[256];
#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
        snprintf(line, sizeof(line),
                 "TEL M0 %s VEL=%.3f POS=%.3f M1_TARGET=%.3f",
                 state.master_input_enabled ? "INPUT" : "PAUSE",
                 state.motor[0].velocity, state.motor[0].position,
                 state.motor[1].target_velocity);
        outputTelemetryLine(line);
#endif
        snprintf(line, sizeof(line),
                 "TEL M1 %s VEL_T=%.3f VEL=%.3f POS=%.3f IQ_T=%.3f IQ=%.3f UQ=%.3f HZ=%.0f",
                 state.motor[1].running ? "RUN" : "STOP",
                 state.motor[1].target_velocity, state.motor[1].velocity,
                 state.motor[1].position, state.motor[1].iq_target,
                 state.motor[1].iq, state.motor[1].uq, state.control_hz);
        outputTelemetryLine(line);
#if CONTROL_MODE == CONTROL_MODE_DUAL_VELOCITY
        snprintf(line, sizeof(line),
                 "TEL M0 %s VEL_T=%.3f VEL=%.3f POS=%.3f IQ_T=%.3f IQ=%.3f UQ=%.3f",
                 state.motor[0].running ? "RUN" : "STOP",
                 state.motor[0].target_velocity, state.motor[0].velocity,
                 state.motor[0].position, state.motor[0].iq_target,
                 state.motor[0].iq, state.motor[0].uq);
        outputTelemetryLine(line);
#endif
        uint32_t delay_ms = 1000 / current_log_hz;
        if (delay_ms == 0) delay_ms = 1;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void initUart(void) {
    uart_config_t config = {};
    config.baud_rate = UART_BAUD;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void initPid(MotorControl_t *control) {
    PID_init(&control->velocity_pid, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD,
             VELOCITY_RAMP, VELOCITY_LIMIT);
    PID_init(&control->current_pid, CURRENT_KP, CURRENT_KI, CURRENT_KD,
             CURRENT_RAMP, CURRENT_LIMIT);
}

extern "C" void app_main(void) {
    snapshot_mutex = xSemaphoreCreateMutex();
    uart_tx_mutex = xSemaphoreCreateMutex();
    command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(ControlCommand_t));
    ESP_ERROR_CHECK(snapshot_mutex != NULL && uart_tx_mutex != NULL && command_queue != NULL
                    ? ESP_OK : ESP_ERR_NO_MEM);
    initUart();

    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 0);
    ESP_LOGI(TAG, "Motor drivers disabled during initialization (GPIO %d)",
             MOTOR_ENABLE_PIN);

    AS5600_init(&motor0.encoder, I2C_NUM_0,
                M0_ENCODER_SDA_PIN, M0_ENCODER_SCL_PIN, 400000);
    AS5600_init(&motor1.encoder, I2C_NUM_1,
                M1_ENCODER_SDA_PIN, M1_ENCODER_SCL_PIN, 400000);

#if CONTROL_MODE == CONTROL_MODE_MASTER_SLAVE_SPEED
    // M0 只作为手轮输入：三路 PWM 保持低电平，不进行 FOC 对齐。
    gpio_set_direction((gpio_num_t)M0_PWM_PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)M0_PWM_PIN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)M0_PWM_PIN_C, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)M0_PWM_PIN_A, 0);
    gpio_set_level((gpio_num_t)M0_PWM_PIN_B, 0);
    gpio_set_level((gpio_num_t)M0_PWM_PIN_C, 0);
    motor0.motor.DIR = M0_SENSOR_DIRECTION;
    motor0.motor.PP = MOTOR_POLE_PAIRS;
    motor0.motor.encoder = &motor0.encoder;

    Motor_init(&motor1.motor, M1_MCPWM_GROUP, MOTOR_VBUS,
               M1_PWM_PIN_A, M1_PWM_PIN_B, M1_PWM_PIN_C, &motor1.encoder);
    CurrSense_init(&motor1.current_sense, M1_CURRENT_PIN_A, M1_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor drivers enabled (GPIO %d)", MOTOR_ENABLE_PIN);
    Motor_alignSensor(&motor1.motor, MOTOR_POLE_PAIRS, M1_SENSOR_DIRECTION);

    motor0.target_velocity = 0.0f;
    motor0.running = false;
    motor1.target_velocity = 0.0f;
    motor1.running = true;
#else
    Motor_init(&motor0.motor, M0_MCPWM_GROUP, MOTOR_VBUS,
               M0_PWM_PIN_A, M0_PWM_PIN_B, M0_PWM_PIN_C, &motor0.encoder);
    Motor_init(&motor1.motor, M1_MCPWM_GROUP, MOTOR_VBUS,
               M1_PWM_PIN_A, M1_PWM_PIN_B, M1_PWM_PIN_C, &motor1.encoder);
    CurrSense_init(&motor0.current_sense, M0_CURRENT_PIN_A, M0_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
    CurrSense_init(&motor1.current_sense, M1_CURRENT_PIN_A, M1_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor drivers enabled (GPIO %d)", MOTOR_ENABLE_PIN);
    Motor_alignSensor(&motor0.motor, MOTOR_POLE_PAIRS, M0_SENSOR_DIRECTION);
    Motor_alignSensor(&motor1.motor, MOTOR_POLE_PAIRS, M1_SENSOR_DIRECTION);

    motor0.target_velocity = M0_DEFAULT_VELOCITY;
    motor1.target_velocity = M1_DEFAULT_VELOCITY;
    motor0.running = true;
    motor1.running = true;
#endif

    initPid(&motor0);
    initPid(&motor1);

    xTaskCreatePinnedToCore(control_task, "dual_control", 10240, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(comm_task, "comm", 6144, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "Tasks started: control(1kHz) + comm + telemetry");
}
