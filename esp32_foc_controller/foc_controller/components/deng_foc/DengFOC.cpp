// 灯哥开源，遵循GNU协议，转载请著名版权！
// 基于原版代码移植到 ESP-IDF 原生 API (MCPWM)

#define _USE_MATH_DEFINES
#include "DengFOC.h"
#include "AS5600.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "DengFOC";

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _3PI_2 4.71238898038f

#define MCPWM_FREQ_HZ      30000   // PWM 频率 30kHz
#define MCPWM_RESOLUTION_HZ 10000000 // 10MHz 分辨率

// U/V/W 三相 GPIO 引脚
static int pwmA = 32;
static int pwmB = 33;
static int pwmC = 25;

static float voltage_power_supply = 0;
static float zero_electric_angle = 0;
static int PP = 1;
static int DIR = 1;

// MCPWM 句柄：3 个 operator 对应 U/V/W 三相
static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper[3] = {NULL};
static mcpwm_cmpr_handle_t comparator[3] = {NULL};
static mcpwm_gen_handle_t generator[3] = {NULL};

// 归一化角度到 [0, 2PI]
float _normalizeAngle(float angle) {
    float a = fmod(angle, 2 * M_PI);
    return a >= 0 ? a : (a + 2 * M_PI);
}

// 设置三相 PWM 占空比 (电压值)
void setPwm(float Ua, float Ub, float Uc) {
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    // MCPWM 占空比：period_ticks 对应 100%，resolution/freq = 333 ticks
    uint32_t period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ;
    mcpwm_comparator_set_compare_value(comparator[0], (uint32_t)(dc_a * period_ticks));
    mcpwm_comparator_set_compare_value(comparator[1], (uint32_t)(dc_b * period_ticks));
    mcpwm_comparator_set_compare_value(comparator[2], (uint32_t)(dc_c * period_ticks));
}

// 设置 Uq 电压和电角度，经过 Park 逆变换 + Clarke 逆变换输出三相电压
void setTorque(float Uq, float angle_el) {
    Uq = _constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);
    angle_el = _normalizeAngle(angle_el);

    // Park 逆变换
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    // Clarke 逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrtf(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + voltage_power_supply / 2;
    setPwm(Ua, Ub, Uc);
}

// FOC 初始化：配置 MCPWM 三路 PWM 输出 + 初始化 AS5600 编码器
void DFOC_Vbus(float power_supply) {
    voltage_power_supply = power_supply;

    // 创建 MCPWM 定时器
    mcpwm_timer_config_t timer_config;
    memset(&timer_config, 0, sizeof(timer_config));
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = MCPWM_RESOLUTION_HZ;
    timer_config.period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // 为 U/V/W 各创建一个 operator + comparator + generator
    int gpios[3] = {pwmA, pwmB, pwmC};
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

        // 初始占空比为 0
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], 0));

        // 计数器空时输出高电平，比较值匹配时输出低电平
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW)));
    }

    // 启动定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM initialized (30kHz, GPIO %d/%d/%d)", pwmA, pwmB, pwmC);
    BeginSensor();
}

// 计算电角度 = 机械角度 × 极对数 - 零电角度偏移
float _electricalAngle() {
    return _normalizeAngle((float)(DIR * PP) * getAngle_Without_track() - zero_electric_angle);
}

// 电角度校准：施加固定电压使转子对齐到已知位置，记录零电角度
void DFOC_alignSensor(int _PP, int _DIR) {
    PP = _PP;
    DIR = _DIR;
    setTorque(3, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(3000));
    zero_electric_angle = _electricalAngle();
    setTorque(0, _3PI_2);
    ESP_LOGI(TAG, "零电角度: %.4f", zero_electric_angle);
}

// 获取累计角度（多圈）
float DFOC_M0_Angle() {
    return getAngle();
}
