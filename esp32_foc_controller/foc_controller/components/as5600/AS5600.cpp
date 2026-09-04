// AS5600 磁编码器驱动 — 实例化版本
// 每个 AS5600_t 结构体对应一个物理编码器，支持多编码器并存

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

// 初始化 AS5600：配置 I2C 总线并安装驱动
void AS5600_init(AS5600_t *dev, i2c_port_t port, int sda, int scl, uint32_t freq) {
    dev->i2c_port = port;
    dev->sda_pin = sda;
    dev->scl_pin = scl;
    dev->freq_hz = freq;
    dev->full_rotations = 0;
    dev->angle_prev = 0;

    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)sda;
    conf.scl_io_num = (gpio_num_t)scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "AS5600 initialized (port=%d, SDA=%d, SCL=%d, %dkHz)",
             port, sda, scl, freq / 1000);
}

// 通过一次 I2C 事务读取两个连续寄存器字节并合并为 16 位值
static uint16_t readTwoBytes(AS5600_t *dev, uint8_t reg_hi) {
    uint8_t buf[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_hi, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[1], I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100)));
    i2c_cmd_link_delete(cmd);

    return (buf[0] << 8) | buf[1];
}

// 读取原始角度 (0-4095, 12 位分辨率)
uint16_t AS5600_getRawAngle(AS5600_t *dev) {
    return readTwoBytes(dev, AS5600_RAW_ANG_HI);
}

// 转换为单圈弧度 (0 ~ 2PI)
float AS5600_getAngle_Without_track(AS5600_t *dev) {
    return AS5600_getRawAngle(dev) * 0.08789f * M_PI / 180.0f;
}

// 多圈角度追踪：检测过零点并累计圈数
float AS5600_trackAngle(AS5600_t *dev, float angle) {
    float d_angle = angle - dev->angle_prev;
    if (fabsf(d_angle) > (0.8f * 6.28318530718f)) {
        dev->full_rotations += (d_angle > 0) ? -1 : 1;
    }
    dev->angle_prev = angle;
    return (float)dev->full_rotations * 6.28318530718f + dev->angle_prev;
}

float AS5600_getAngle(AS5600_t *dev) {
    return AS5600_trackAngle(dev, AS5600_getAngle_Without_track(dev));
}
