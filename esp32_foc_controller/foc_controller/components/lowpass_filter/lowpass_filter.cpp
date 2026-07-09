// 低通滤波器 — 改编自 Arduino 版本，适配 ESP-IDF
// 替换 micros() → esp_timer_get_time()
// 替换 unsigned long → int64_t

#include "lowpass_filter.h"
#include "esp_timer.h"

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = esp_timer_get_time();
}

float LowPassFilter::operator()(float x)
{
    int64_t timestamp = esp_timer_get_time();
    float dt = (float)(timestamp - timestamp_prev) * 1e-6f;

    // 容错：时间异常时的回退策略
    if (dt < 0.0f) {
        dt = 1e-3f;                    // 时间倒退 → 假定 1ms
    } else if (dt > 0.3f) {
        y_prev = x;                    // 间隔太长 → 直接复位
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = Tf / (Tf + dt);
    float y = alpha * y_prev + (1.0f - alpha) * x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}
