#include <math.h>
#include <string.h>
#include "current_sense.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "CurrentSense";
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

#define ADC_VOLTAGE       3.3f
#define ADC_RESOLUTION    4095.0f
#define ADC_CONV          (ADC_VOLTAGE / ADC_RESOLUTION)
#define CALIBRATION_ROUNDS 1000
#define _1_SQRT3          0.57735026919f
#define _2_SQRT3          1.15470053838f

static adc_oneshot_unit_handle_t getADCHandle(adc_unit_t unit) {
    adc_oneshot_unit_handle_t *handle = NULL;
    if (unit == ADC_UNIT_1)
        handle = &adc1_handle;
    else if (unit == ADC_UNIT_2)
        handle = &adc2_handle;
    else
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);

    if (*handle == NULL) {
        adc_oneshot_unit_init_cfg_t unit_cfg;
        memset(&unit_cfg, 0, sizeof(unit_cfg));
        unit_cfg.unit_id = unit;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, handle));
    }
    return *handle;
}

static float readADCVoltage(adc_oneshot_unit_handle_t handle,
                            adc_channel_t channel) {
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(handle, channel, &raw));
    return (float)raw * ADC_CONV;
}

void CurrSense_init(CurrSense_t *dev, int pin_a, int pin_b, float shunt_resistor, float amp_gain) {
    memset(dev, 0, sizeof(CurrSense_t));
    dev->pin_a = pin_a;
    dev->pin_b = pin_b;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(
        pin_a, &dev->unit_a, &dev->channel_a));
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(
        pin_b, &dev->unit_b, &dev->channel_b));
    dev->adc_handle_a = getADCHandle(dev->unit_a);
    dev->adc_handle_b = getADCHandle(dev->unit_b);

    adc_oneshot_chan_cfg_t chan_cfg;
    memset(&chan_cfg, 0, sizeof(chan_cfg));
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        dev->adc_handle_a, dev->channel_a, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        dev->adc_handle_b, dev->channel_b, &chan_cfg));

    float volts_to_amps_ratio = 1.0f / shunt_resistor / amp_gain;
    dev->gain_a = -volts_to_amps_ratio;
    dev->gain_b = -volts_to_amps_ratio;

    for (int i = 0; i < CALIBRATION_ROUNDS; i++) {
        dev->offset_ia += readADCVoltage(dev->adc_handle_a, dev->channel_a);
        dev->offset_ib += readADCVoltage(dev->adc_handle_b, dev->channel_b);
        esp_rom_delay_us(1000);
    }
    dev->offset_ia /= CALIBRATION_ROUNDS;
    dev->offset_ib /= CALIBRATION_ROUNDS;
    ESP_LOGI(TAG,
             "Current sense initialized (GPIO %d/ADC%d, GPIO %d/ADC%d, offsets %.3f/%.3fV)",
             pin_a, (int)dev->unit_a + 1, pin_b, (int)dev->unit_b + 1,
             dev->offset_ia, dev->offset_ib);
}

void CurrSense_getPhaseCurrents(CurrSense_t *dev) {
    dev->current_a = (readADCVoltage(dev->adc_handle_a, dev->channel_a)
                      - dev->offset_ia) * dev->gain_a;
    dev->current_b = (readADCVoltage(dev->adc_handle_b, dev->channel_b)
                      - dev->offset_ib) * dev->gain_b;
}

float CurrSense_getIq(CurrSense_t *dev, float angle_el) {
    CurrSense_getPhaseCurrents(dev);
    float i_alpha = dev->current_a;
    float i_beta = _1_SQRT3 * dev->current_a + _2_SQRT3 * dev->current_b;
    return i_beta * cosf(angle_el) - i_alpha * sinf(angle_el);
}
