// DengFOC 双电机 ESP-IDF 控制器
// 控制线程：1kHz 顺序执行传感器采样、速度环、电流环和 PWM
// 通信线程：USB 控制台与 UART1 命令解析
// 日志线程：按配置频率输出状态，不参与控制计算

// ==================== 控制模式 ====================
#define CONTROL_MODE_DUAL_VELOCITY       0 // M0/M1 独立速度-电流闭环
#define CONTROL_MODE_MASTER_SLAVE_SPEED  1 // M0 手轮输入，M1 单向调速
#define CONTROL_MODE_BILATERAL_COUPLING  2 // M0/M1 双向位置同步与力反馈
#define CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD 3 // M0易堵停，M1增流抗负载
#define DEFAULT_CONTROL_MODE CONTROL_MODE_BILATERAL_COUPLING

// 单电机硬件测试：M1 未接线时设为 0，接好后改回 1。
#define MOTOR1_ENABLED 1
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
#include "multi_button.h"

static const char *TAG = "FOC";

// ==================== 双电机硬件配置 ====================
#define MOTOR_ENABLE_PIN       35
#define MODE_BUTTON_PIN        0

#define M0_PWM_PIN_A           1
#define M0_PWM_PIN_B           2
#define M0_PWM_PIN_C           5
#define M0_ENCODER_SDA_PIN     15
#define M0_ENCODER_SCL_PIN     14
#define M0_CURRENT_PIN_A       4
#define M0_CURRENT_PIN_B       9
#define M0_MCPWM_GROUP         0
#define M0_SENSOR_DIRECTION    -1
#define M0_DEFAULT_VELOCITY    5.0f

#define M1_PWM_PIN_A           6
#define M1_PWM_PIN_B           7
#define M1_PWM_PIN_C           8
#define M1_ENCODER_SDA_PIN     13
#define M1_ENCODER_SCL_PIN     12
#define M1_CURRENT_PIN_A       10
#define M1_CURRENT_PIN_B       11
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

// 双向电子轴：位置刚度(A/rad)、速度阻尼(A/(rad/s))和安全限制
#define LINK_DEFAULT_KP             0.80f
#define LINK_DEFAULT_KD             0.030f
#define LINK_DEFAULT_CURRENT_LIMIT  0.15f
#define LINK_DEFAULT_CURRENT_RAMP   10.0f
#define LINK_DEFAULT_RATIO          1.0f
#define LINK_POSITION_ERROR_LIMIT   1.5f
#define LINK_POSITION_DEADBAND      0.005f
#define LINK_VELOCITY_DEADBAND      0.05f
#define LINK_VELOCITY_FILTER_TF     0.005f

// 参考第九章力位闭环，双向模式使用更快的电流环参数。
#define BILATERAL_CURRENT_KP        1.25f
#define BILATERAL_CURRENT_KI        50.0f

// 独立负载模式：速度 PI 输出 Iq 目标，电流 PI 输出 Uq。
#define LOAD_DEFAULT_VELOCITY       (16.0f * M_PI)
#define LOAD_MAX_VELOCITY           80.0f
#define LOAD_VELOCITY_KP            0.03f
#define LOAD_VELOCITY_KI            0.20f
#define LOAD_VELOCITY_RAMP          5.0f
#define M0_NORMAL_IQ_LIMIT          0.12f
#define M0_SOFT_STOP_IQ             0.02f
#define M0_ARM_SPEED_RATIO          0.80f
#define M0_LOAD_SPEED_RATIO         0.70f
#define M0_STALL_SPEED_RATIO        0.20f
#define M0_RELEASE_SPEED_RATIO      0.05f
#define M0_TRIP_CURRENT             0.08f
#define M0_ARM_CONFIRM_CYCLES       200
#define M0_TRIP_CONFIRM_CYCLES      30
#define M0_RELEASE_CONFIRM_CYCLES   100
#define M0_RECOVER_TIMEOUT_CYCLES   500
#define M1_IQ_LIMIT                 0.50f
#define M1_OVERCURRENT_LIMIT        1.0f
#define M1_OVERCURRENT_CYCLES       50
#define M1_FAULT_COOLDOWN_CYCLES    1000

#if !MOTOR1_ENABLED
#error "Button mode switching requires MOTOR1_ENABLED=1"
#endif

typedef uint8_t ControlMode_t;

static const char *controlModeName(ControlMode_t mode) {
    switch (mode) {
        case CONTROL_MODE_DUAL_VELOCITY: return "DUAL_VELOCITY";
        case CONTROL_MODE_MASTER_SLAVE_SPEED: return "MASTER_SLAVE_SPEED";
        case CONTROL_MODE_BILATERAL_COUPLING: return "BILATERAL_COUPLING";
        case CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD:
            return "INDEPENDENT_VOLTAGE_LOAD";
        default: return "UNKNOWN";
    }
}

static float constrainValue(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float normalizeAngle(float angle) {
    float value = fmodf(angle, 2.0f * M_PI);
    return value >= 0.0f ? value : value + 2.0f * M_PI;
}

static float applySoftDeadband(float value, float deadband) {
    if (fabsf(value) <= deadband) return 0.0f;
    return value - copysignf(deadband, value);
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
    float position;
    float velocity;
    float electrical_angle;
    float iq;
} MotorSample_t;

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
    float link_kp;
    float link_kd;
    float link_current_limit;
    float link_current_ramp;
    float link_ratio;
    float link_position_error;
    float link_velocity_error_raw;
    float link_velocity_error;
    float link_iq_request;
    float link_iq_command;
    uint8_t m0_load_state;
    uint8_t m1_load_state;
    ControlMode_t control_mode;
} SystemSnapshot_t;

typedef enum {
    M0_LOAD_STARTING,
    M0_LOAD_ARMED,
    M0_LOAD_SOFT_STOP,
    M0_LOAD_RECOVERING,
} M0LoadState_t;

typedef enum {
    M1_LOAD_DRIVE,
    M1_LOAD_OVERCURRENT,
} M1LoadState_t;

typedef enum {
    CMD_SET_VELOCITY,
    CMD_SET_VELOCITY_PID,
    CMD_SET_CURRENT_PID,
    CMD_SET_RUNNING,
    CMD_SET_LINK_GAIN,
    CMD_SET_LINK_MAX,
    CMD_SET_LINK_KP,
    CMD_SET_LINK_KD,
    CMD_SET_LINK_CURRENT,
    CMD_SET_LINK_RAMP,
    CMD_SET_LINK_RATIO,
    CMD_SET_LINK_PARAMS,
    CMD_ZERO_LINK,
    CMD_SET_MODE,
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
static ControlMode_t control_mode = DEFAULT_CONTROL_MODE;
static Button mode_button;

static float link_gain = LINK_DEFAULT_GAIN;
static float link_max_speed = LINK_DEFAULT_MAX_SPEED;
static float link_pending_angle = 0.0f;
static bool master_input_enabled = true;

static float bilateral_link_kp = LINK_DEFAULT_KP;
static float bilateral_link_kd = LINK_DEFAULT_KD;
static float bilateral_current_limit = LINK_DEFAULT_CURRENT_LIMIT;
static float bilateral_current_ramp = LINK_DEFAULT_CURRENT_RAMP;
static float bilateral_ratio = LINK_DEFAULT_RATIO;
static float bilateral_zero_position[2] = {};
static float bilateral_position_error = 0.0f;
static float bilateral_position_error_prev = 0.0f;
static float bilateral_velocity_error_raw = 0.0f;
static float bilateral_velocity_error = 0.0f;
static float bilateral_iq_request = 0.0f;
static float bilateral_iq_command = 0.0f;
static int64_t bilateral_timestamp_prev = 0;
static bool bilateral_zero_pending = false;

static M0LoadState_t m0_load_state = M0_LOAD_STARTING;
static M1LoadState_t m1_load_state = M1_LOAD_DRIVE;
static uint32_t m0_load_counter = 0;
static uint32_t m0_recover_counter = 0;
static uint32_t m1_overcurrent_counter = 0;
static uint32_t m1_fault_counter = 0;

// 速度滤波保持 10ms；双向力反馈缩短电流滤波以降低相位滞后。
static LowPassFilter m0_velocity_filter(0.01f);
static LowPassFilter m0_current_filter(0.01f);
static LowPassFilter m1_velocity_filter(0.01f);
static LowPassFilter m1_current_filter(0.01f);
static LowPassFilter bilateral_velocity_filter(LINK_VELOCITY_FILTER_TF);

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

static MotorSample_t sampleClosedLoopMotor(MotorControl_t *control,
                                            LowPassFilter &velocity_filter,
                                            LowPassFilter &current_filter) {
    float raw_angle = AS5600_getAngle_Without_track(&control->encoder);
    float full_angle = AS5600_trackAngle(&control->encoder, raw_angle);
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - control->velocity_timestamp_prev) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 1e-3f;

    MotorSample_t sample = {};
    sample.position = control->motor.DIR * full_angle;
    sample.velocity = velocity_filter(
        control->motor.DIR * (full_angle - control->full_angle_prev) / dt);
    sample.electrical_angle = normalizeAngle(
        (float)(control->motor.DIR * control->motor.PP) * raw_angle
        - control->motor.zero_electric_angle);
    sample.iq = current_filter(
        CurrSense_getIq(&control->current_sense, sample.electrical_angle));

    control->full_angle_prev = full_angle;
    control->velocity_timestamp_prev = now;
    return sample;
}

static MotorTelemetry_t applyCurrentTarget(MotorControl_t *control,
                                            const MotorSample_t *sample,
                                            float iq_target) {
    float uq = 0.0f;
    if (control->running)
        uq = PID_update(&control->current_pid, iq_target - sample->iq);
    else
        iq_target = 0.0f;
    Motor_setTorque(&control->motor, uq, sample->electrical_angle);

    MotorTelemetry_t state = {};
    state.running = control->running;
    state.target_velocity = control->target_velocity;
    state.position = sample->position;
    state.velocity = sample->velocity;
    state.iq_target = iq_target;
    state.iq = sample->iq;
    state.uq = uq;
    return state;
}

static MotorTelemetry_t updateClosedLoopMotor(MotorControl_t *control,
                                              LowPassFilter &velocity_filter,
                                              LowPassFilter &current_filter) {
    MotorSample_t sample = sampleClosedLoopMotor(
        control, velocity_filter, current_filter);
    float iq_target = 0.0f;

    if (control->running)
        iq_target = PID_update(
            &control->velocity_pid,
            (control->target_velocity - sample.velocity) * DEG_PER_RAD);
    return applyCurrentTarget(control, &sample, iq_target);
}

static void updateBilateralCoupling(MotorTelemetry_t state[2]) {
    MotorSample_t sample[2];
    sample[0] = sampleClosedLoopMotor(
        &motor0, m0_velocity_filter, m0_current_filter);
    sample[1] = sampleClosedLoopMotor(
        &motor1, m1_velocity_filter, m1_current_filter);

    bool zeroed = bilateral_zero_pending;
    if (zeroed) {
        bilateral_zero_position[0] = sample[0].position;
        bilateral_zero_position[1] = sample[1].position;
        bilateral_position_error = 0.0f;
        bilateral_position_error_prev = 0.0f;
        bilateral_velocity_error_raw = 0.0f;
        bilateral_velocity_error = 0.0f;
        bilateral_iq_request = 0.0f;
        bilateral_iq_command = 0.0f;
        bilateral_velocity_filter.reset();
        bilateral_timestamp_prev = esp_timer_get_time();
        PID_reset(&motor0.current_pid);
        PID_reset(&motor1.current_pid);
        bilateral_zero_pending = false;
    }

    float q0 = sample[0].position - bilateral_zero_position[0];
    float q1 = sample[1].position - bilateral_zero_position[1];
    bilateral_position_error = q0 - q1 / bilateral_ratio;

    int64_t now = esp_timer_get_time();
    float dt = (float)(now - bilateral_timestamp_prev) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 1e-3f;
    bilateral_timestamp_prev = now;

    bilateral_velocity_error_raw = zeroed
        ? 0.0f
        : (bilateral_position_error - bilateral_position_error_prev) / dt;
    bilateral_position_error_prev = bilateral_position_error;
    bilateral_velocity_error = bilateral_velocity_filter(
        bilateral_velocity_error_raw);

    float limited_error = constrainValue(
        bilateral_position_error, -LINK_POSITION_ERROR_LIMIT,
        LINK_POSITION_ERROR_LIMIT);
    float controlled_error = applySoftDeadband(
        limited_error, LINK_POSITION_DEADBAND);
    float controlled_velocity = applySoftDeadband(
        bilateral_velocity_error, LINK_VELOCITY_DEADBAND);
    bilateral_iq_request = bilateral_link_kp * controlled_error
                         + bilateral_link_kd * controlled_velocity;
    float requested_iq = constrainValue(bilateral_iq_request,
                                        -bilateral_current_limit,
                                        bilateral_current_limit);

    if (motor0.running && motor1.running) {
        float max_step = bilateral_current_ramp * dt;
        float delta = constrainValue(requested_iq - bilateral_iq_command,
                                     -max_step, max_step);
        bilateral_iq_command += delta;
    } else {
        bilateral_iq_command = 0.0f;
    }

    // RATIO=1 时两台相同电机采用大小相同、方向相反的 Iq。
    float iq_target_m0 = -bilateral_iq_command;
    float iq_target_m1 = constrainValue(
        bilateral_iq_command / bilateral_ratio,
        -bilateral_current_limit, bilateral_current_limit);
    state[0] = applyCurrentTarget(&motor0, &sample[0], iq_target_m0);
    state[1] = applyCurrentTarget(&motor1, &sample[1], iq_target_m1);
}

static float updateLoadSpeedPid(MotorControl_t *control,
                                const MotorSample_t *sample) {
    if (!control->running) return 0.0f;
    // 本模式的速度 PI 直接使用 rad/s 误差，输出单位为 A。
    return PID_update(&control->velocity_pid,
                      control->target_velocity - sample->velocity);
}

static void resetM0LoadState(void) {
    m0_load_state = M0_LOAD_STARTING;
    m0_load_counter = 0;
    m0_recover_counter = 0;
    PID_reset(&motor0.velocity_pid);
    PID_reset(&motor0.current_pid);
}

static void updateIndependentVoltageLoad(MotorTelemetry_t state[2]) {
    MotorSample_t sample[2];
    sample[0] = sampleClosedLoopMotor(
        &motor0, m0_velocity_filter, m0_current_filter);
#if MOTOR1_ENABLED
    sample[1] = sampleClosedLoopMotor(
        &motor1, m1_velocity_filter, m1_current_filter);
#endif

    float m0_target_abs = fabsf(motor0.target_velocity);
    float m0_speed_abs = fabsf(sample[0].velocity);
    float m0_iq_target = 0.0f;

    if (!motor0.running || m0_target_abs < 0.01f) {
        resetM0LoadState();
    } else {
        switch (m0_load_state) {
            case M0_LOAD_STARTING:
                m0_iq_target = updateLoadSpeedPid(&motor0, &sample[0]);
                if (m0_speed_abs >= m0_target_abs * M0_ARM_SPEED_RATIO)
                    m0_load_counter++;
                else
                    m0_load_counter = 0;
                if (m0_load_counter >= M0_ARM_CONFIRM_CYCLES) {
                    m0_load_state = M0_LOAD_ARMED;
                    m0_load_counter = 0;
                }
                break;

            case M0_LOAD_ARMED:
                m0_iq_target = updateLoadSpeedPid(&motor0, &sample[0]);
                if ((m0_speed_abs <= m0_target_abs * M0_LOAD_SPEED_RATIO
                     && fabsf(sample[0].iq) >= M0_TRIP_CURRENT)
                    || m0_speed_abs <= m0_target_abs * M0_STALL_SPEED_RATIO)
                    m0_load_counter++;
                else
                    m0_load_counter = 0;
                if (m0_load_counter >= M0_TRIP_CONFIRM_CYCLES) {
                    m0_load_state = M0_LOAD_SOFT_STOP;
                    m0_load_counter = 0;
                    PID_reset(&motor0.velocity_pid);
                    PID_reset(&motor0.current_pid);
                    m0_iq_target = copysignf(M0_SOFT_STOP_IQ,
                                             motor0.target_velocity);
                }
                break;

            case M0_LOAD_SOFT_STOP:
                m0_iq_target = copysignf(M0_SOFT_STOP_IQ,
                                         motor0.target_velocity);
                if (m0_speed_abs >= m0_target_abs * M0_RELEASE_SPEED_RATIO)
                    m0_load_counter++;
                else
                    m0_load_counter = 0;
                if (m0_load_counter >= M0_RELEASE_CONFIRM_CYCLES) {
                    m0_load_state = M0_LOAD_RECOVERING;
                    m0_load_counter = 0;
                    m0_recover_counter = 0;
                    PID_reset(&motor0.velocity_pid);
                    PID_reset(&motor0.current_pid);
                }
                break;

            case M0_LOAD_RECOVERING:
                m0_iq_target = updateLoadSpeedPid(&motor0, &sample[0]);
                m0_recover_counter++;
                if (m0_speed_abs >= m0_target_abs * M0_ARM_SPEED_RATIO) {
                    m0_load_counter++;
                    if (m0_load_counter >= M0_ARM_CONFIRM_CYCLES) {
                        m0_load_state = M0_LOAD_ARMED;
                        m0_load_counter = 0;
                        m0_recover_counter = 0;
                    }
                } else {
                    m0_load_counter = 0;
                }
                if (m0_recover_counter >= M0_RECOVER_TIMEOUT_CYCLES
                    && m0_speed_abs <= m0_target_abs * M0_RELEASE_SPEED_RATIO) {
                    m0_load_state = M0_LOAD_SOFT_STOP;
                    m0_load_counter = 0;
                    m0_recover_counter = 0;
                    PID_reset(&motor0.velocity_pid);
                    PID_reset(&motor0.current_pid);
                }
                break;
        }
    }

#if MOTOR1_ENABLED
    float m1_iq_target = 0.0f;
    if (!motor1.running) {
        m1_load_state = M1_LOAD_DRIVE;
        m1_overcurrent_counter = 0;
        m1_fault_counter = 0;
    } else if (m1_load_state == M1_LOAD_OVERCURRENT) {
        m1_fault_counter++;
        if (m1_fault_counter >= M1_FAULT_COOLDOWN_CYCLES) {
            m1_load_state = M1_LOAD_DRIVE;
            m1_overcurrent_counter = 0;
            m1_fault_counter = 0;
            PID_reset(&motor1.velocity_pid);
            PID_reset(&motor1.current_pid);
        }
    } else {
        m1_iq_target = updateLoadSpeedPid(&motor1, &sample[1]);
        if (fabsf(sample[1].iq) >= M1_OVERCURRENT_LIMIT)
            m1_overcurrent_counter++;
        else
            m1_overcurrent_counter = 0;
        if (m1_overcurrent_counter >= M1_OVERCURRENT_CYCLES) {
            m1_load_state = M1_LOAD_OVERCURRENT;
            m1_overcurrent_counter = 0;
            m1_fault_counter = 0;
            PID_reset(&motor1.velocity_pid);
            PID_reset(&motor1.current_pid);
            m1_iq_target = 0.0f;
        }
    }

    state[1] = applyCurrentTarget(&motor1, &sample[1], m1_iq_target);
#else
    state[1] = {};
    m1_load_state = M1_LOAD_DRIVE;
#endif
    state[0] = applyCurrentTarget(&motor0, &sample[0], m0_iq_target);
}

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
    Motor_setTorque(&motor0.motor, 0.0f, 0.0f);
    return state;
}

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

static void configureControlMode(ControlMode_t mode) {
    Motor_setTorque(&motor0.motor, 0.0f, 0.0f);
    Motor_setTorque(&motor1.motor, 0.0f, 0.0f);
    motor0.running = false;
    motor1.running = false;

    m0_velocity_filter.reset();
    m1_velocity_filter.reset();
    m0_current_filter.reset();
    m1_current_filter.reset();
    m0_current_filter.Tf = mode == CONTROL_MODE_BILATERAL_COUPLING
                           || mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD
        ? 0.01f : 0.05f;
    m1_current_filter.Tf = m0_current_filter.Tf;
    initControlState(&motor0);
    initControlState(&motor1);

    if (mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD) {
        PID_init(&motor0.velocity_pid, LOAD_VELOCITY_KP, LOAD_VELOCITY_KI,
                 0.0f, LOAD_VELOCITY_RAMP, M0_NORMAL_IQ_LIMIT);
        PID_init(&motor1.velocity_pid, LOAD_VELOCITY_KP, LOAD_VELOCITY_KI,
                 0.0f, LOAD_VELOCITY_RAMP, M1_IQ_LIMIT);
    } else {
        PID_init(&motor0.velocity_pid, VELOCITY_KP, VELOCITY_KI,
                 VELOCITY_KD, VELOCITY_RAMP, VELOCITY_LIMIT);
        PID_init(&motor1.velocity_pid, VELOCITY_KP, VELOCITY_KI,
                 VELOCITY_KD, VELOCITY_RAMP, VELOCITY_LIMIT);
    }

    float current_kp = mode == CONTROL_MODE_BILATERAL_COUPLING
        ? BILATERAL_CURRENT_KP : CURRENT_KP;
    float current_ki = mode == CONTROL_MODE_BILATERAL_COUPLING
        ? BILATERAL_CURRENT_KI : CURRENT_KI;
    PID_init(&motor0.current_pid, current_kp, current_ki, CURRENT_KD,
             CURRENT_RAMP, CURRENT_LIMIT);
    PID_init(&motor1.current_pid, current_kp, current_ki, CURRENT_KD,
             CURRENT_RAMP, CURRENT_LIMIT);

    control_mode = mode;
    link_pending_angle = 0.0f;
    bilateral_iq_command = 0.0f;
    bilateral_iq_request = 0.0f;

    switch (mode) {
        case CONTROL_MODE_DUAL_VELOCITY:
            motor0.target_velocity = M0_DEFAULT_VELOCITY;
            motor1.target_velocity = M1_DEFAULT_VELOCITY;
            motor0.running = true;
            motor1.running = true;
            break;

        case CONTROL_MODE_MASTER_SLAVE_SPEED:
            motor0.target_velocity = 0.0f;
            motor1.target_velocity = 0.0f;
            master_input_enabled = true;
            motor1.running = true;
            break;

        case CONTROL_MODE_BILATERAL_COUPLING:
            motor0.target_velocity = 0.0f;
            motor1.target_velocity = 0.0f;
            motor0.running = true;
            motor1.running = true;
            bilateral_zero_pending = true;
            bilateral_timestamp_prev = esp_timer_get_time();
            break;

        case CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD:
            motor0.target_velocity = LOAD_DEFAULT_VELOCITY;
            motor1.target_velocity = LOAD_DEFAULT_VELOCITY;
            motor0.running = true;
            motor1.running = true;
            m0_load_state = M0_LOAD_STARTING;
            m1_load_state = M1_LOAD_DRIVE;
            m0_load_counter = 0;
            m0_recover_counter = 0;
            m1_overcurrent_counter = 0;
            m1_fault_counter = 0;
            break;
    }
    ESP_LOGI(TAG, "Mode switched: %u - %s", (unsigned)mode,
             controlModeName(mode));
}

static void applyCommand(const ControlCommand_t *command) {
    switch (command->type) {
        case CMD_SET_VELOCITY:
            if (command->motor_mask & MOTOR_MASK_M0)
                motor0.target_velocity = command->value[0];
            if (command->motor_mask & MOTOR_MASK_M1) {
                float target = command->value[command->motor_mask == MOTOR_MASK_ALL ? 1 : 0];
                if (control_mode == CONTROL_MODE_MASTER_SLAVE_SPEED)
                    target = constrainValue(target, 0.0f, link_max_speed);
                motor1.target_velocity = target;
            }
            if (control_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD) {
                if (command->motor_mask & MOTOR_MASK_M0)
                    resetM0LoadState();
                if (command->motor_mask & MOTOR_MASK_M1) {
                    m1_load_state = M1_LOAD_DRIVE;
                    m1_overcurrent_counter = 0;
                    m1_fault_counter = 0;
                    PID_reset(&motor1.velocity_pid);
                    PID_reset(&motor1.current_pid);
                }
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
            if (control_mode == CONTROL_MODE_MASTER_SLAVE_SPEED) {
                if (command->motor_mask & MOTOR_MASK_M0) {
                    master_input_enabled = command->enabled;
                    link_pending_angle = 0.0f;
                }
                if (command->motor_mask & MOTOR_MASK_M1)
                    setMotorRunning(&motor1, command->enabled);
            } else if (control_mode == CONTROL_MODE_BILATERAL_COUPLING) {
                // 电子轴必须成对启停，避免只驱动一侧产生意外转矩。
                setMotorRunning(&motor0, command->enabled);
                setMotorRunning(&motor1, command->enabled);
                bilateral_iq_command = 0.0f;
                bilateral_zero_pending = command->enabled;
            } else if (control_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD) {
                if (command->motor_mask & MOTOR_MASK_M0) {
                    if (command->enabled && fabsf(motor0.target_velocity) < 0.01f)
                        motor0.target_velocity = LOAD_DEFAULT_VELOCITY;
                    setMotorRunning(&motor0, command->enabled);
                    resetM0LoadState();
                }
                if (command->motor_mask & MOTOR_MASK_M1) {
                    if (command->enabled && fabsf(motor1.target_velocity) < 0.01f)
                        motor1.target_velocity = LOAD_DEFAULT_VELOCITY;
                    setMotorRunning(&motor1, command->enabled);
                    m1_load_state = M1_LOAD_DRIVE;
                    m1_overcurrent_counter = 0;
                    m1_fault_counter = 0;
                }
            } else {
                if (command->motor_mask & MOTOR_MASK_M0)
                    setMotorRunning(&motor0, command->enabled);
                if (command->motor_mask & MOTOR_MASK_M1)
                    setMotorRunning(&motor1, command->enabled);
            }
            break;

        case CMD_SET_LINK_GAIN:
            link_gain = command->value[0];
            break;

        case CMD_SET_LINK_MAX:
            link_max_speed = command->value[0];
            motor1.target_velocity = constrainValue(
                motor1.target_velocity, 0.0f, link_max_speed);
            break;

        case CMD_SET_LINK_KP:
            bilateral_link_kp = command->value[0];
            break;

        case CMD_SET_LINK_KD:
            bilateral_link_kd = command->value[0];
            break;

        case CMD_SET_LINK_CURRENT:
            bilateral_current_limit = command->value[0];
            bilateral_iq_command = constrainValue(
                bilateral_iq_command, -bilateral_current_limit,
                bilateral_current_limit);
            break;

        case CMD_SET_LINK_RAMP:
            bilateral_current_ramp = command->value[0];
            break;

        case CMD_SET_LINK_RATIO:
            bilateral_ratio = command->value[0];
            bilateral_zero_pending = true;
            break;

        case CMD_SET_LINK_PARAMS:
            bilateral_link_kp = command->value[0];
            bilateral_link_kd = command->value[1];
            bilateral_current_limit = command->value[2];
            bilateral_ratio = command->value[3];
            bilateral_iq_command = 0.0f;
            bilateral_zero_pending = true;
            break;

        case CMD_ZERO_LINK:
            if (control_mode == CONTROL_MODE_BILATERAL_COUPLING) {
                bilateral_zero_pending = true;
            } else {
                link_pending_angle = 0.0f;
                motor1.target_velocity = 0.0f;
                PID_reset(&motor1.velocity_pid);
                PID_reset(&motor1.current_pid);
            }
            break;

        case CMD_SET_MODE:
            if ((ControlMode_t)command->value[0] != control_mode)
                configureControlMode((ControlMode_t)command->value[0]);
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
    snapshot.link_kp = bilateral_link_kp;
    snapshot.link_kd = bilateral_link_kd;
    snapshot.link_current_limit = bilateral_current_limit;
    snapshot.link_current_ramp = bilateral_current_ramp;
    snapshot.link_ratio = bilateral_ratio;
    snapshot.link_position_error = bilateral_position_error;
    snapshot.link_velocity_error_raw = bilateral_velocity_error_raw;
    snapshot.link_velocity_error = bilateral_velocity_error;
    snapshot.link_iq_request = bilateral_iq_request;
    snapshot.link_iq_command = bilateral_iq_command;
    snapshot.m0_load_state = (uint8_t)m0_load_state;
    snapshot.m1_load_state = (uint8_t)m1_load_state;
    snapshot.control_mode = control_mode;
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
    configureControlMode(control_mode);
    ESP_LOGI(TAG, ">> mode=%s, loop=%dHz", controlModeName(control_mode),
             CONTROL_LOOP_HZ);

    while (1) {
        ControlCommand_t command;
        while (xQueueReceive(command_queue, &command, 0) == pdTRUE)
            applyCommand(&command);

        MotorTelemetry_t state[2];
        switch (control_mode) {
            case CONTROL_MODE_MASTER_SLAVE_SPEED:
                state[0] = updateMasterInput();
                state[1] = updateClosedLoopMotor(
                    &motor1, m1_velocity_filter, m1_current_filter);
                break;
            case CONTROL_MODE_BILATERAL_COUPLING:
                updateBilateralCoupling(state);
                break;
            case CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD:
                updateIndependentVoltageLoad(state);
                break;
            default:
                state[0] = updateClosedLoopMotor(
                    &motor0, m0_velocity_filter, m0_current_filter);
                state[1] = updateClosedLoopMotor(
                    &motor1, m1_velocity_filter, m1_current_filter);
                break;
        }

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

static const char *m0LoadStateName(uint8_t state) {
    switch ((M0LoadState_t)state) {
        case M0_LOAD_STARTING: return "STARTING";
        case M0_LOAD_ARMED: return "ARMED";
        case M0_LOAD_SOFT_STOP: return "SOFT_STOP";
        case M0_LOAD_RECOVERING: return "RECOVERING";
        default: return "UNKNOWN";
    }
}

static const char *m1LoadStateName(uint8_t state) {
    return state == M1_LOAD_OVERCURRENT ? "OVERCURRENT" : "DRIVE";
}

static void sendState(bool uart_output, int motor_index, const SystemSnapshot_t *state) {
    const MotorTelemetry_t *motor = &state->motor[motor_index];
    const char *run_state = motor->running ? "RUN" : "STOP";
    if (state->control_mode == CONTROL_MODE_MASTER_SLAVE_SPEED
        && motor_index == 0)
        run_state = motor->running ? "INPUT" : "PAUSE";
    else if (state->control_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD
             && motor->running)
        run_state = motor_index == 0
            ? m0LoadStateName(state->m0_load_state)
            : m1LoadStateName(state->m1_load_state);
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
    sendResponse(uart_output, "SET LINK GAIN|MAX value (master/slave)");
    sendResponse(uart_output, "SET LINK KP|KD|CURRENT|RAMP|RATIO value (bilateral)");
    sendResponse(uart_output, "SET LINK PARAM kp kd current ratio | ZERO LINK");
    sendResponse(uart_output, "GET LOAD (independent voltage/load mode)");
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
            sendResponse(uart_output, "MODE %u %s", (unsigned)state.control_mode,
                         controlModeName(state.control_mode));
            return;
        }
        if (count == 2 && tokenEquals(token[1], "LINK")) {
            if (state.control_mode == CONTROL_MODE_BILATERAL_COUPLING)
                sendResponse(uart_output,
                             "LINK KP=%.4f KD=%.4f CURRENT=%.3f RAMP=%.3f RATIO=%.3f E=%.4f DE_RAW=%.3f DE=%.3f IQ_REQ=%.3f IQ=%.3f",
                             state.link_kp, state.link_kd, state.link_current_limit,
                             state.link_current_ramp, state.link_ratio,
                             state.link_position_error, state.link_velocity_error_raw,
                             state.link_velocity_error, state.link_iq_request,
                             state.link_iq_command);
            else
                sendResponse(uart_output,
                             "LINK GAIN=%.3f MAX=%.3f TARGET=%.3f PENDING=%.4f",
                             state.link_gain, state.link_max_speed,
                             state.motor[1].target_velocity,
                             state.link_pending_angle);
            return;
        }
        if (count == 2 && tokenEquals(token[1], "LOAD")) {
            if (state.control_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD)
                sendResponse(uart_output,
                             "LOAD M0=%s TARGET=%.3f LOAD_VEL=%.3f TRIP_IQ=%.2fA HOLD=%.2fA M1=%s TARGET=%.3f IQ_MAX=%.2fA OC=%.2fA",
                             m0LoadStateName(state.m0_load_state),
                             state.motor[0].target_velocity,
                             fabsf(state.motor[0].target_velocity) * M0_LOAD_SPEED_RATIO,
                             M0_TRIP_CURRENT, M0_SOFT_STOP_IQ,
                             m1LoadStateName(state.m1_load_state),
                             state.motor[1].target_velocity, M1_IQ_LIMIT,
                             M1_OVERCURRENT_LIMIT);
            else
                sendResponse(uart_output, "ERR LOAD_MODE_ONLY");
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
        ControlMode_t active_mode = readSnapshot().control_mode;
        if (count == 7 && tokenEquals(token[1], "LINK")
            && tokenEquals(token[2], "PARAM")) {
            ControlCommand_t command = {};
            command.type = CMD_SET_LINK_PARAMS;
            for (int i = 0; i < 4; i++) {
                if (!parseFloat(token[3 + i], &command.value[i])) {
                    sendResponse(uart_output, "ERR BAD_VALUE");
                    return;
                }
            }
            if (command.value[0] < 0.0f || command.value[0] > 10.0f
                || command.value[1] < 0.0f || command.value[1] > 1.0f
                || command.value[2] <= 0.0f || command.value[2] > ABSOLUTE_MAX_IQ
                || fabsf(command.value[3]) < 0.1f || fabsf(command.value[3]) > 10.0f) {
                sendResponse(uart_output, "ERR BAD_VALUE");
                return;
            }
            if (enqueueCommand(&command, uart_output))
                sendResponse(uart_output, "OK SET LINK PARAM");
            return;
        }

        if (count == 4 && tokenEquals(token[1], "LINK")) {
            float value;
            if (!parseFloat(token[3], &value)) {
                sendResponse(uart_output, "ERR BAD_VALUE");
                return;
            }
            ControlCommand_t command = {};
            if (tokenEquals(token[2], "GAIN") && value > 0.0f && value <= 100.0f)
                command.type = CMD_SET_LINK_GAIN;
            else if (tokenEquals(token[2], "MAX") && value > 0.0f
                     && value <= ABSOLUTE_MAX_SPEED)
                command.type = CMD_SET_LINK_MAX;
            else if (tokenEquals(token[2], "KP") && value >= 0.0f && value <= 10.0f)
                command.type = CMD_SET_LINK_KP;
            else if (tokenEquals(token[2], "KD") && value >= 0.0f && value <= 1.0f)
                command.type = CMD_SET_LINK_KD;
            else if (tokenEquals(token[2], "CURRENT") && value > 0.0f
                     && value <= ABSOLUTE_MAX_IQ)
                command.type = CMD_SET_LINK_CURRENT;
            else if (tokenEquals(token[2], "RAMP") && value > 0.0f && value <= 100.0f)
                command.type = CMD_SET_LINK_RAMP;
            else if (tokenEquals(token[2], "RATIO") && fabsf(value) >= 0.1f
                     && fabsf(value) <= 10.0f)
                command.type = CMD_SET_LINK_RATIO;
            else {
                sendResponse(uart_output, "ERR BAD_VALUE");
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
                if (active_mode == CONTROL_MODE_MASTER_SLAVE_SPEED
                    && (motor_mask & MOTOR_MASK_M0)) {
                    sendResponse(uart_output, "ERR M0_INPUT_ONLY");
                    return;
                }
                if (active_mode == CONTROL_MODE_BILATERAL_COUPLING) {
                    sendResponse(uart_output, "ERR LINK_MODE_ONLY");
                    return;
                }
                ControlCommand_t command = {};
                command.type = CMD_SET_VELOCITY;
                command.motor_mask = motor_mask;
                int value_count = motor_mask == MOTOR_MASK_ALL ? 2 : 1;
                for (int i = 0; i < value_count; i++) {
                    if (!parseFloat(token[3 + i], &command.value[i])) {
                        sendResponse(uart_output, "ERR BAD_VALUE");
                        return;
                    }
                    float max_velocity =
                        active_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD
                        ? LOAD_MAX_VELOCITY : ABSOLUTE_MAX_SPEED;
                    if (fabsf(command.value[i]) > max_velocity) {
                        sendResponse(uart_output, "ERR BAD_VALUE");
                        return;
                    }
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
                float velocity_limit_max = ABSOLUTE_MAX_IQ;
                if (active_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD)
                    velocity_limit_max = (motor_mask & MOTOR_MASK_M0)
                        ? M0_NORMAL_IQ_LIMIT : M1_IQ_LIMIT;
                if (command.type == CMD_SET_VELOCITY_PID
                    && command.value[4] > velocity_limit_max) {
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
        if (state.control_mode == CONTROL_MODE_MASTER_SLAVE_SPEED) {
            snprintf(line, sizeof(line),
                     "TEL M0 %s VEL=%.3f POS=%.3f M1_TARGET=%.3f",
                     state.master_input_enabled ? "INPUT" : "PAUSE",
                     state.motor[0].velocity, state.motor[0].position,
                     state.motor[1].target_velocity);
            outputTelemetryLine(line);
        } else if (state.control_mode == CONTROL_MODE_BILATERAL_COUPLING) {
            snprintf(line, sizeof(line),
                     "TEL LINK E=%.4f DE_RAW=%.3f DE=%.3f IQ_REQ=%.3f IQ=%.3f KP=%.3f KD=%.4f LIMIT=%.3f",
                     state.link_position_error, state.link_velocity_error_raw,
                     state.link_velocity_error, state.link_iq_request,
                     state.link_iq_command, state.link_kp, state.link_kd,
                     state.link_current_limit);
            outputTelemetryLine(line);
        } else if (state.control_mode == CONTROL_MODE_INDEPENDENT_VOLTAGE_LOAD) {
            snprintf(line, sizeof(line),
                     "TEL LOAD M0=%s M1=%s",
                     m0LoadStateName(state.m0_load_state),
                     m1LoadStateName(state.m1_load_state));
            outputTelemetryLine(line);
        }
        snprintf(line, sizeof(line),
                 "TEL M1 %s VEL_T=%.3f VEL=%.3f POS=%.3f IQ_T=%.3f IQ=%.3f UQ=%.3f HZ=%.0f",
                 state.motor[1].running ? "RUN" : "STOP",
                 state.motor[1].target_velocity, state.motor[1].velocity,
                 state.motor[1].position, state.motor[1].iq_target,
                 state.motor[1].iq, state.motor[1].uq, state.control_hz);
        outputTelemetryLine(line);
        if (state.control_mode != CONTROL_MODE_MASTER_SLAVE_SPEED) {
            snprintf(line, sizeof(line),
                     "TEL M0 %s VEL_T=%.3f VEL=%.3f POS=%.3f IQ_T=%.3f IQ=%.3f UQ=%.3f",
                     state.motor[0].running ? "RUN" : "STOP",
                     state.motor[0].target_velocity, state.motor[0].velocity,
                     state.motor[0].position, state.motor[0].iq_target,
                     state.motor[0].iq, state.motor[0].uq);
            outputTelemetryLine(line);
        }
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

static uint8_t readButtonLevel(uint8_t button_id) {
    return (uint8_t)gpio_get_level((gpio_num_t)button_id);
}

static void modeButtonClick(void *button) {
    uint8_t click_count = button_get_repeat_count((Button *)button);
    if (click_count < 1 || click_count > 4) return;

    ControlCommand_t command = {};
    command.type = CMD_SET_MODE;
    command.value[0] = (float)(click_count - 1);
    if (xQueueSend(command_queue, &command, 0) != pdTRUE)
        ESP_LOGW(TAG, "Mode button command queue is full");
}

static void button_task(void *arg) {
    while (1) {
        button_ticks();
        vTaskDelay(pdMS_TO_TICKS(mode_button.ticks_interval));
    }
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

    gpio_config_t button_config = {};
    button_config.pin_bit_mask = 1ULL << MODE_BUTTON_PIN;
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&button_config));
    button_init(&mode_button, readButtonLevel, 0, MODE_BUTTON_PIN);
    button_attach(&mode_button, BTN_SINGLE_CLICK, modeButtonClick);
    button_attach(&mode_button, BTN_DOUBLE_CLICK, modeButtonClick);
    button_attach(&mode_button, BTN_TRIPLE_CLICK, modeButtonClick);
    button_attach(&mode_button, BTN_QUADRUPLE_CLICK, modeButtonClick);
    ESP_ERROR_CHECK(button_start(&mode_button) == 0 ? ESP_OK : ESP_FAIL);

    AS5600_init(&motor0.encoder, I2C_NUM_0,
                M0_ENCODER_SDA_PIN, M0_ENCODER_SCL_PIN, 400000);
#if MOTOR1_ENABLED
    AS5600_init(&motor1.encoder, I2C_NUM_1,
                M1_ENCODER_SDA_PIN, M1_ENCODER_SCL_PIN, 400000);
#endif

    Motor_init(&motor0.motor, M0_MCPWM_GROUP, MOTOR_VBUS,
               M0_PWM_PIN_A, M0_PWM_PIN_B, M0_PWM_PIN_C, &motor0.encoder);
#if MOTOR1_ENABLED
    Motor_init(&motor1.motor, M1_MCPWM_GROUP, MOTOR_VBUS,
               M1_PWM_PIN_A, M1_PWM_PIN_B, M1_PWM_PIN_C, &motor1.encoder);
#else
    // 共用使能拉高前，将未使用的 M1 三相输入固定为低电平。
    gpio_set_direction((gpio_num_t)M1_PWM_PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)M1_PWM_PIN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)M1_PWM_PIN_C, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)M1_PWM_PIN_A, 0);
    gpio_set_level((gpio_num_t)M1_PWM_PIN_B, 0);
    gpio_set_level((gpio_num_t)M1_PWM_PIN_C, 0);
#endif
    CurrSense_init(&motor0.current_sense, M0_CURRENT_PIN_A, M0_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
#if MOTOR1_ENABLED
    CurrSense_init(&motor1.current_sense, M1_CURRENT_PIN_A, M1_CURRENT_PIN_B,
                   CURRENT_SHUNT_RESISTOR, CURRENT_AMP_GAIN);
#endif
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor drivers enabled (GPIO %d)", MOTOR_ENABLE_PIN);
    Motor_alignSensor(&motor0.motor, MOTOR_POLE_PAIRS, M0_SENSOR_DIRECTION);
#if MOTOR1_ENABLED
    Motor_alignSensor(&motor1.motor, MOTOR_POLE_PAIRS, M1_SENSOR_DIRECTION);
#endif
    motor0.running = false;
    motor1.running = false;

    xTaskCreatePinnedToCore(control_task, "dual_control", 10240, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(button_task, "mode_button", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(comm_task, "comm", 6144, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "Tasks started: control(1kHz) + button(5ms) + comm + telemetry");
}
