// author: al1s
// 移植DengFOC库, Arduino -> ESP-IDF支持函数

#ifndef DENG_FOC_H
#define DENG_FOC_H

#include <stdint.h>
#include "driver/mcpwm_prelude.h"
#include "AS5600.h"

#ifdef __cplusplus
extern "C" {
#endif

// 电机实例结构体：每个电机一个
typedef struct {
    // MCPWM 硬件句柄
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper[3];
    mcpwm_cmpr_handle_t comparator[3];
    mcpwm_gen_handle_t generator[3];
    int mcpwm_group;             // MCPWM 组号，每台电机使用独立组
    // 电机参数
    int pwm_pins[3];             // U/V/W 三相 GPIO
    float voltage_power_supply;  // 电源电压
    float zero_electric_angle;   // 零电角度偏移
    int PP;                      // 极对数
    int DIR;                     // 方向 (+1 或 -1)
    // 关联的编码器
    AS5600_t *encoder;
} Motor_t;

// 初始化电机：配置指定 MCPWM 组的三路 PWM，关联编码器
void Motor_init(Motor_t *motor, int mcpwm_group, float voltage,
                int pin_a, int pin_b, int pin_c, AS5600_t *encoder);

// 电角度校准：施加固定电压对齐转子，记录零电角度
void Motor_alignSensor(Motor_t *motor, int pp, int dir);

// 设置力矩：Uq 电压 + 电角度 → 三相 PWM 输出
void Motor_setTorque(Motor_t *motor, float Uq, float angle_el);

// 获取累计角度（多圈）
float Motor_getAngle(Motor_t *motor);

// 获取当前电角度
float Motor_electricalAngle(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif
