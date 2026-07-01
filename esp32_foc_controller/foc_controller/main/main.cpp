#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "AS5600.h"

static const char *TAG = "FOC";

// 三相 PWM 输出引脚
#define PWM_A  32
#define PWM_B  33
#define PWM_C  25

// 电机使能引脚
#define MOTOR_ENABLE_PIN  12

// MCPWM 配置
#define MCPWM_FREQ_HZ       30000     // 30kHz PWM 频率
#define MCPWM_RESOLUTION_HZ 10000000  // 10MHz 分辨率

// FOC 参数
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static float voltage_power_supply = 12.3f;
static float voltage_limit = 10.0f;
static float zero_electric_angle = 0;
static int PR = 7;     // 极对数
static int DIR = -1;   // 旋转方向

// MCPWM 句柄：3 个 operator 对应 U/V/W 三相
static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper[3] = {NULL};
static mcpwm_cmpr_handle_t comparator[3] = {NULL};
static mcpwm_gen_handle_t generator[3] = {NULL};

// 角度归一化到 [0, 2PI]
static float normalizeAngle(float angle) {
    float a = fmod(angle, 2 * M_PI);
    return a >= 0 ? a : (a + 2 * M_PI);
}

// 计算电角度
static float electricalAngle() {
    return normalizeAngle((float)(DIR * PR) * getAngle_Without_track() - zero_electric_angle);
}

// 设置三相 PWM 占空比
static void setPWM(float Ua, float Ub, float Uc) {
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    uint32_t period = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ;
    mcpwm_comparator_set_compare_value(comparator[0], (uint32_t)(dc_a * period));
    mcpwm_comparator_set_compare_value(comparator[1], (uint32_t)(dc_b * period));
    mcpwm_comparator_set_compare_value(comparator[2], (uint32_t)(dc_c * period));
}

// Park 逆变换 + Clarke 逆变换，将 Uq/Ud 转为三相电压
static void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = normalizeAngle(angle_el);

    // Park 逆变换
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    // Clarke 逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrtf(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + voltage_power_supply / 2;
    setPWM(Ua, Ub, Uc);
}

// 初始化 MCPWM 三路 PWM
static void initPWM() {
    // 创建定时器
    mcpwm_timer_config_t timer_config;
    memset(&timer_config, 0, sizeof(timer_config));
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = MCPWM_RESOLUTION_HZ;
    timer_config.period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // 为 U/V/W 各创建 operator + comparator + generator
    int gpios[3] = {PWM_A, PWM_B, PWM_C};
    for (int i = 0; i < 3; i++) {
        mcpwm_operator_config_t oper_config;
        memset(&oper_config, 0, sizeof(oper_config));
        oper_config.group_id = 0;
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper[i], timer));

        mcpwm_comparator_config_t cmp_config;
        memset(&cmp_config, 0, sizeof(cmp_config));
        cmp_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper[i], &cmp_config, &comparator[i]));

        mcpwm_generator_config_t gen_config;
        memset(&gen_config, 0, sizeof(gen_config));
        gen_config.gen_gpio_num = gpios[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(oper[i], &gen_config, &generator[i]));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], 0));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW)));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "MCPWM initialized (30kHz, GPIO %d/%d/%d)", PWM_A, PWM_B, PWM_C);
}

// FOC 初始化
static void initFOC() {
    initPWM();

    // 使能电机驱动
    gpio_set_direction((gpio_num_t)MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled (GPIO %d)", MOTOR_ENABLE_PIN);

    // 初始化 AS5600 编码器
    BeginSensor();

    // 电角度校准：施加固定电压对齐转子，记录零电角度
    setPhaseVoltage(3, 0, 3 * M_PI / 2);
    vTaskDelay(pdMS_TO_TICKS(3000));
    zero_electric_angle = electricalAngle();
    setPhaseVoltage(0, 0, 3 * M_PI / 2);
    ESP_LOGI(TAG, "零电角度: %.4f", zero_electric_angle);
}

extern "C" void app_main(void) {
    initFOC();

    // 角度闭环参数
    float motor_target = M_PI;   // 目标角度 (rad)
    float Kp = 0.133f;           // 比例增益

    ESP_LOGI(TAG, "开始角度闭环控制，目标: %.2f rad", motor_target);

    while (1) {
        float sensor_angle = getAngle();
        float error = motor_target - DIR * sensor_angle;
        float Uq = _constrain(Kp * error * 180.0f / M_PI, -6, 6);

        setPhaseVoltage(Uq, 0, electricalAngle());

        ESP_LOGI(TAG, "角度: %.2f rad, 误差: %.2f, Uq: %.2f", sensor_angle, error, Uq);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
