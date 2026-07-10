# 问题记录

## 1. ESP-IDF v6.0 组件名变更

**现象**：`driver/gpio.h: No such file or directory`

**原因**：v6.0 把 `driver` 拆分为 `esp_driver_gpio`、`esp_driver_mcpwm`、`esp_driver_ledc` 等

**解决**：CMakeLists.txt 中 `PRIV_REQUIRES` 改为对应的新组件名

---

## 2. C++ 不支持嵌套指定初始化

**现象**：MCPWM/I2C 配置结构体 `-Wmissing-field-initializers` 报错

**原因**：C++ 不支持 `.master.clk_speed = xxx` 这种嵌套指定初始化语法

**解决**：先 `memset(&config, 0, sizeof(config))`，再逐个字段赋值

---

## 3. 旧版 I2C 废弃警告

**现象**：`driver/i2c.h` 触发 `#pragma message` CRITICAL WARNING

**原因**：v6.0 标记旧版 I2C 为 EOL，v7.0 将移除

**解决**：用 `#pragma GCC diagnostic ignored "-Wdeprecated-declarations"` 包裹 include

---

## 4. int 到 gpio_num_t 类型转换

**现象**：`-fpermissive` 错误，int 不能隐式转为枚举

**原因**：C++ 比 C 严格，不允许 int → enum 隐式转换

**解决**：加显式转换 `(gpio_num_t)pin_number`

---

## 5. REQUIRES vs PRIV_REQUIRES 传播性

**现象**：`driver/mcpwm_prelude.h: No such file or directory`（main 编译时）

**原因**：公共头文件 include 了其他组件的头文件，但依赖写在 `PRIV_REQUIRES` 里，include 路径不会传播

**解决**：公共头文件用到的组件必须放 `REQUIRES`，只有 .cpp 私用的才放 `PRIV_REQUIRES`

---

## 6. 双线程角度不同步导致电机振荡

**现象**：电机左右跳动

**原因**：控制线程用 `shared_angle` 算误差，但用 `Motor_electricalAngle()` 算电角度，后者直接读编码器，两次读数有时间差

**解决**：传感器线程同时读多圈和单圈角度，控制线程统一用共享值计算，不再直接读编码器

---

## 7. 速度获取 — 传感器侧无速度接口

**现象**：需要实现闭环速度环，但 AS5600 驱动和 DengFOC 只提供角度读取。

**解决**：在传感器线程（500Hz）中直接用角度差分计算速度 `velocity = (angle_now - angle_prev) / 0.002`，并加入低通滤波器平滑信号。不修改 AS5600 组件，保持其接口简洁。

---

## 8. 低通滤波器库 — Arduino 依赖适配

**现象**：找到的 `lowpass_filter.cpp/.h` 使用了 `micros()` 和 `Arduino.h`，无法在 ESP-IDF 下编译。

**解决**：
- `micros()` → `esp_timer_get_time()`（来自 `esp_timer.h`）
- `unsigned long` → `int64_t`
- 去掉了 `Arduino.h`，改为 `<stdint.h>`
- 封装为独立 component：`components/lowpass_filter/`，包含 CMakeLists.txt

---

## 9. 电机周期性顿挫 — 过零点速度计算噪声 (关键)

**现象**：速度环下电机匀速旋转一段时间后会突然"顿一下"，周期约 2 秒（等于 π rad/s 下转一圈的时间），周而复始。

**根因**：速度计算使用了 `AS5600_getAngle()` 返回的多圈累计角度做差分。AS5600 是 12 位编码器（4096 counts/rev），在 raw angle 过零点附近，两次读数的物理位移极小（仅 ~4 LSB = 0.006 rad），导致量化噪声占比高达 ±25~50%：

```
过零前: full = N*2π + 6.280   (raw ≈ 2π - 4 LSB)
过零后: full = (N+1)*2π + 0.003   (raw ≈ 2 LSB)
物理位移 = 0.006 rad  ← 仅 4 LSB
速度 = 0.006 / 0.002 = 3.0 rad/s  (理论值 π ≈ 3.14 rad/s, 误差 ~5%)
任意 ±1 LSB 抖动 → 速度误差 ±25~50%
```

坏数据经低通滤波后仍残留扰动 → Uq 突变 → 电机周期性顿挫。

**解决**：改用 raw angle + wrap-aware 差分，绕过对多圈追踪状态的依赖：

```cpp
float d_raw = raw - raw_prev;
if (d_raw > M_PI)       d_raw -= 2.0f * M_PI;  // 正向过零
else if (d_raw < -M_PI) d_raw += 2.0f * M_PI;  // 反向过零
velocity = d_raw / dt;
```

阈值 π 在 500Hz 采样率下对应 ~15000 RPM，实际转速远达不到，不会误判。过零点速度计算完全平滑。

---

## 10. 双模式切换 — 位置环 / 速度环

**需求**：通过一个宏定义切换闭环位置和闭环速度控制，方便验证对比。

**实现**：
```cpp
#define CONTROL_MODE_POSITION  0
#define CONTROL_MODE_VELOCITY  1
#define CONTROL_MODE  CONTROL_MODE_VELOCITY  // ← 改这一行切换
```

| 模式 | 反馈量 | 控制器 | 目标值 |
|------|--------|--------|--------|
| 位置环 | 多圈累计角度 | P | π rad (180°) |
| 速度环 | 低通滤波速度 | PI + 抗饱和 | π rad/s (2秒/圈) |
