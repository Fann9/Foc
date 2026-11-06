// author: al1s
// 移植DengFOC库, Arduino -> ESP-IDF支持函数
// 使用ESP-IDF API (MCPWM)
// 主要重构为实例化结构体，支持多电机并存

#define _USE_MATH_DEFINES
#include "DengFOC.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "DengFOC";

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _3PI_2 4.71238898038f

#define MCPWM_FREQ_HZ      30000    // PWM 频率 30kHz
#define MCPWM_RESOLUTION_HZ 10000000 // 10MHz 分辨率

// 归一化角度到 [0, 2PI]
static float normalizeAngle(float angle) {
    float a = fmod(angle, 2 * M_PI);
    return a >= 0 ? a : (a + 2 * M_PI);
}

// 设置三相 PWM 占空比（电压值 → 占空比 → MCPWM 比较值）
static void motor_setPwm(Motor_t *motor, float Ua, float Ub, float Uc) {
    float dc_a = _constrain(Ua / motor->voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / motor->voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / motor->voltage_power_supply, 0.0f, 1.0f);

    uint32_t period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ;
    mcpwm_comparator_set_compare_value(motor->comparator[0], (uint32_t)(dc_a * period_ticks));
    mcpwm_comparator_set_compare_value(motor->comparator[1], (uint32_t)(dc_b * period_ticks));
    mcpwm_comparator_set_compare_value(motor->comparator[2], (uint32_t)(dc_c * period_ticks));
}

// 初始化电机：创建 MCPWM 定时器 + 3 路 operator/comparator/generator
void Motor_init(Motor_t *motor, int mcpwm_group, float voltage,
                int pin_a, int pin_b, int pin_c, AS5600_t *encoder) {
    memset(motor, 0, sizeof(Motor_t));
    motor->mcpwm_group = mcpwm_group;
    motor->voltage_power_supply = voltage;
    motor->pwm_pins[0] = pin_a;
    motor->pwm_pins[1] = pin_b;
    motor->pwm_pins[2] = pin_c;
    motor->encoder = encoder;
    motor->PP = 1;
    motor->DIR = 1;

    // 创建 MCPWM 定时器
    mcpwm_timer_config_t timer_config;
    memset(&timer_config, 0, sizeof(timer_config));
    timer_config.group_id = mcpwm_group;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = MCPWM_RESOLUTION_HZ;
    timer_config.period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor->timer));

    // 为 U/V/W 各创建 operator + comparator + generator
    for (int i = 0; i < 3; i++) {
        mcpwm_operator_config_t oper_config;
        memset(&oper_config, 0, sizeof(oper_config));
        oper_config.group_id = mcpwm_group;
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &motor->oper[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor->oper[i], motor->timer));

        mcpwm_comparator_config_t cmp_config;
        memset(&cmp_config, 0, sizeof(cmp_config));
        cmp_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(motor->oper[i], &cmp_config, &motor->comparator[i]));

        mcpwm_generator_config_t gen_config;
        memset(&gen_config, 0, sizeof(gen_config));
        gen_config.gen_gpio_num = motor->pwm_pins[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(motor->oper[i], &gen_config, &motor->generator[i]));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->comparator[i], 0));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor->generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor->generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator[i], MCPWM_GEN_ACTION_LOW)));
    }

    // 启动定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(motor->timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Motor initialized (MCPWM group %d, 30kHz, GPIO %d/%d/%d)",
             mcpwm_group, pin_a, pin_b, pin_c);
}

// 设置 Uq 电压和电角度 → Park 逆变换 + Clarke 逆变换 → 三相 PWM
void Motor_setTorque(Motor_t *motor, float Uq, float angle_el) {
    Uq = _constrain(Uq, -motor->voltage_power_supply / 2, motor->voltage_power_supply / 2);
    angle_el = normalizeAngle(angle_el);

    // Park 逆变换
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    // Clarke 逆变换
    float Ua = Ualpha + motor->voltage_power_supply / 2;
    float Ub = (sqrtf(3) * Ubeta - Ualpha) / 2 + motor->voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + motor->voltage_power_supply / 2;
    motor_setPwm(motor, Ua, Ub, Uc);
}

// 计算电角度 = 机械角度 × 极对数 × 方向 - 零电角度偏移
float Motor_electricalAngle(Motor_t *motor) {
    return normalizeAngle(
        (float)(motor->DIR * motor->PP) * AS5600_getAngle_Without_track(motor->encoder)
        - motor->zero_electric_angle
    );
}

// 电角度校准：施加 Uq=3V 指向 3PI/2 方向，1 秒后读取实际电角度作为零偏
void Motor_alignSensor(Motor_t *motor, int pp, int dir) {
    motor->PP = pp;
    motor->DIR = dir;
    Motor_setTorque(motor, 3, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor->zero_electric_angle = Motor_electricalAngle(motor);
    Motor_setTorque(motor, 0, _3PI_2);
    ESP_LOGI(TAG, "零电角度: %.4f (PP=%d, DIR=%d)", motor->zero_electric_angle, pp, dir);
}

// 获取累计角度（多圈）
float Motor_getAngle(Motor_t *motor) {
    return AS5600_getAngle(motor->encoder);
}
