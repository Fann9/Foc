// 低通滤波器 — 改编自 Arduino 版本，适配 ESP-IDF
// 使用 esp_timer_get_time() 替代 micros()
// 公式: y = alpha*y_prev + (1-alpha)*x,  alpha = Tf/(Tf+dt)

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdint.h>

class LowPassFilter
{
public:
    /**
     * @param Tf  低通滤波时间常数 (秒), 越大平滑越强
     *            典型值: 速度滤波 0.005~0.02s
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    /**
     * @param x  当前输入值
     * @return   滤波后的输出值
     */
    float operator()(float x);

    float Tf;  // 低通滤波时间常数

private:
    int64_t timestamp_prev;  // 上次调用时间戳 (微秒)
    float   y_prev;          // 上一拍的滤波输出
};

#endif
