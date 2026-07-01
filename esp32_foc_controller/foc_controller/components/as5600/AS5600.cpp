// 灯哥开源，遵循GNU协议，转载请著名版权！
// 基于原版代码移植到 ESP-IDF 原生 API

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include "AS5600.h"
#include "esp_log.h"

// 抑制旧版 I2C 驱动的废弃警告 (ESP-IDF v6.0)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "driver/i2c.h"
#pragma GCC diagnostic pop

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AS5600";

#define AS5600_ADDR        0x36
#define AS5600_RAW_ANG_HI  0x0C
#define AS5600_RAW_ANG_LO  0x0D

#define I2C_SDA_PIN  19
#define I2C_SCL_PIN  18
#define I2C_FREQ_HZ  400000

static int32_t full_rotations = 0;
static float angle_prev = 0;

void BeginSensor() {
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_SDA_PIN;
    conf.scl_io_num = (gpio_num_t)I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "AS5600 initialized (SDA=%d, SCL=%d, %dkHz)", I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ / 1000);
}

static uint16_t readTwoBytes(uint8_t reg_hi, uint8_t reg_lo) {
    uint8_t buf[2];

    // 读高位寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_hi, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, &buf[1], 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    // 读低位寄存器
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_lo, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, &buf[0], 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (buf[1] << 8) | buf[0];
}

uint16_t getRawAngle() {
    return readTwoBytes(AS5600_RAW_ANG_HI, AS5600_RAW_ANG_LO);
}

float getAngle_Without_track() {
    return getRawAngle() * 0.08789f * M_PI / 180.0f;
}

float getAngle() {
    float val = getAngle_Without_track();
    float d_angle = val - angle_prev;
    if (fabsf(d_angle) > (0.8f * 6.28318530718f)) {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
}
