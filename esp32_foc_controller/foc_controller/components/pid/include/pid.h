/**
 * @file pid.h
 * @brief 位置型 PID 控制器 (C 语言接口)
 *
 * 算法与 DengFOC 库一致：
 *   P   = Kp * error
 *   I   = Ki * Ts * (error + error_prev) / 2   (梯形/Tustin 积分)
 *   D   = Kd * (error - error_prev) / Ts
 * 带输出限幅 (limit) 和变化率限幅 (output_ramp)。
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益
    float output_ramp;      // 输出变化率限幅 (>0 生效)
    float limit;            // 输出绝对值限幅
    float error_prev;       // 上次误差
    float integral;         // 积分累加值
    float output_prev;      // 上次输出
    int64_t timestamp_prev; // 上次调用时间戳 (μs)
} PID_t;

/**
 * @brief 初始化 PID 控制器
 * @param pid   PID 实例指针
 * @param Kp    比例增益
 * @param Ki    积分增益
 * @param Kd    微分增益
 * @param ramp  输出变化率限幅 (0 = 不限)
 * @param limit 输出绝对值限幅
 */
void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float ramp, float limit);

/**
 * @brief 运行时设定 PID 参数（不重置积分/误差历史）
 *
 * 等价于 DengFOC 的 DFOC_M0_SET_VEL_PID / DFOC_M0_SET_ANGLE_PID。
 * 可在控制循环中随时调用，不会破坏积分累加值或时间戳。
 *
 * @param pid  PID 实例指针
 * @param Kp   比例增益
 * @param Ki   积分增益
 * @param Kd   微分增益
 * @param ramp 输出变化率限幅 (0 = 不限)
 */
void PID_setParams(PID_t *pid, float Kp, float Ki, float Kd, float ramp);

/**
 * @brief 运行时设定输出限幅
 * @param pid   PID 实例指针
 * @param limit 输出绝对值限幅
 */
void PID_setLimit(PID_t *pid, float limit);

/**
 * @brief 清空积分、误差和输出历史，保留当前 PID 参数
 * @param pid PID 实例指针
 */
void PID_reset(PID_t *pid);

/**
 * @brief 执行一次 PID 计算
 * @param pid    PID 实例指针
 * @param error  当前误差 (设定值 - 测量值)
 * @return PID 输出量
 */
float PID_update(PID_t *pid, float error);

#ifdef __cplusplus
}
#endif

#endif
