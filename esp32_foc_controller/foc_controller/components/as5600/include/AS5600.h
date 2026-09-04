#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include "driver/i2c.h"

// AS5600 实例结构体：每个编码器一个
typedef struct {
    i2c_port_t i2c_port;
    int sda_pin;
    int scl_pin;
    uint32_t freq_hz;
    int32_t full_rotations;
    float angle_prev;
} AS5600_t;

// 初始化 AS5600 编码器（配置 I2C 并启动）
void AS5600_init(AS5600_t *dev, i2c_port_t port, int sda, int scl, uint32_t freq);

// 读取原始角度值 (0-4095)
uint16_t AS5600_getRawAngle(AS5600_t *dev);

// 读取单圈角度 (0 ~ 2PI rad)
float AS5600_getAngle_Without_track(AS5600_t *dev);

// 读取累计角度（多圈追踪）
float AS5600_getAngle(AS5600_t *dev);

// 用已读取的单圈角度更新并返回累计角度，避免同一控制周期重复读取 I2C
float AS5600_trackAngle(AS5600_t *dev, float angle);

#endif
