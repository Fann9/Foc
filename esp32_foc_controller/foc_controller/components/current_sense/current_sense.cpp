#include <math.h>
#include <string.h>
#include "current_sense.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "CurrentSense";
static adc_oneshot_unit_handle_t adc1_handle = NULL;

#define ADC_VOLTAGE       3.3f
#define ADC_RESOLUTION    4095.0f
#define ADC_CONV          (ADC_VOLTAGE / ADC_RESOLUTION)
#define CALIBRATION_ROUNDS 1000
#define _1_SQRT3          0.57735026919f
#define _2_SQRT3          1.15470053838f

static float readADCVoltage(CurrSense_t *dev, adc_channel_t channel) {
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(dev->adc_handle, channel, &raw));
    return (float)raw * ADC_CONV;
}

void CurrSense_init(CurrSense_t *dev, int pin_a, int pin_b, float shunt_resistor, float amp_gain) {
    memset(dev, 0, sizeof(CurrSense_t));
    dev->pin_a = pin_a;
    dev->pin_b = pin_b;

    adc_unit_t unit_a, unit_b;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pin_a, &unit_a, &dev->channel_a));
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pin_b, &unit_b, &dev->channel_b));
    ESP_ERROR_CHECK(unit_a == ADC_UNIT_1 && unit_b == ADC_UNIT_1 ? ESP_OK : ESP_ERR_INVALID_ARG);

    // 两台电机的四路采样均在 ADC1，共用同一个 oneshot unit。
    if (adc1_handle == NULL) {
        adc_oneshot_unit_init_cfg_t unit_cfg;
        memset(&unit_cfg, 0, sizeof(unit_cfg));
        unit_cfg.unit_id = ADC_UNIT_1;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1_handle));
    }
    dev->adc_handle = adc1_handle;

    adc_oneshot_chan_cfg_t chan_cfg;
    memset(&chan_cfg, 0, sizeof(chan_cfg));
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(dev->adc_handle, dev->channel_a, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(dev->adc_handle, dev->channel_b, &chan_cfg));

    float volts_to_amps_ratio = 1.0f / shunt_resistor / amp_gain;
    dev->gain_a = -volts_to_amps_ratio;
    dev->gain_b = -volts_to_amps_ratio;

    for (int i = 0; i < CALIBRATION_ROUNDS; i++) {
        dev->offset_ia += readADCVoltage(dev, dev->channel_a);
        dev->offset_ib += readADCVoltage(dev, dev->channel_b);
        esp_rom_delay_us(1000);
    }
    dev->offset_ia /= CALIBRATION_ROUNDS;
    dev->offset_ib /= CALIBRATION_ROUNDS;
    ESP_LOGI(TAG, "Current sense initialized (GPIO %d/%d, offsets %.3f/%.3fV)",
             pin_a, pin_b, dev->offset_ia, dev->offset_ib);
}

void CurrSense_getPhaseCurrents(CurrSense_t *dev) {
    dev->current_a = (readADCVoltage(dev, dev->channel_a) - dev->offset_ia) * dev->gain_a;
    dev->current_b = (readADCVoltage(dev, dev->channel_b) - dev->offset_ib) * dev->gain_b;
}

float CurrSense_getIq(CurrSense_t *dev, float angle_el) {
    CurrSense_getPhaseCurrents(dev);
    float i_alpha = dev->current_a;
    float i_beta = _1_SQRT3 * dev->current_a + _2_SQRT3 * dev->current_b;
    return i_beta * cosf(angle_el) - i_alpha * sinf(angle_el);
}
