#include "pid.h"
#include "esp_timer.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float ramp, float limit) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;
    pid->output_prev = 0.0f;
    pid->timestamp_prev = esp_timer_get_time();
}

void PID_setParams(PID_t *pid, float Kp, float Ki, float Kd, float ramp) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_ramp = ramp;
}

void PID_setLimit(PID_t *pid, float limit) {
    pid->limit = limit;
}

float PID_update(PID_t *pid, float error) {
    int64_t now = esp_timer_get_time();
    float Ts = (float)(now - pid->timestamp_prev) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    // P 环
    float P = pid->Kp * error;

    // I 环 (梯形积分 / Tustin 变换)
    pid->integral += pid->Ki * Ts * 0.5f * (error + pid->error_prev);
    pid->integral = _constrain(pid->integral, -pid->limit, pid->limit);

    // D 环
    float D = pid->Kd * (error - pid->error_prev) / Ts;

    // 合路
    float output = P + pid->integral + D;
    output = _constrain(output, -pid->limit, pid->limit);

    // 输出变化率限幅
    if (pid->output_ramp > 0.0f) {
        float rate = (output - pid->output_prev) / Ts;
        if (rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }

    pid->error_prev = error;
    pid->output_prev = output;
    pid->timestamp_prev = now;
    return output;
}
