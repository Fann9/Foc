#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "esp_adc/adc_oneshot.h"

// 两相在线电流检测实例：由 Ia、Ib 推导 Iq
typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t channel_a;
    adc_channel_t channel_b;
    int pin_a;
    int pin_b;
    float offset_ia;
    float offset_ib;
    float gain_a;
    float gain_b;
    float current_a;
    float current_b;
} CurrSense_t;

// 初始化 ADC 并在无电流状态下校准零偏
void CurrSense_init(CurrSense_t *dev, int pin_a, int pin_b, float shunt_resistor, float amp_gain);

// 读取 A/B 两相电流 (A)
void CurrSense_getPhaseCurrents(CurrSense_t *dev);

// 由两相电流和电角度计算 q 轴电流 Iq (A)
float CurrSense_getIq(CurrSense_t *dev, float angle_el);

#endif
