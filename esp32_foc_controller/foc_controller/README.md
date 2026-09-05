# ESP32 双电机 FOC 控制器

基于 ESP32 + ESP-IDF v6.0 的双电机 FOC 控制器。本版本按照 `第十课a+b 双电机闭环FOC代码` 中实际启用的示例，实现两台电机相互独立的速度—电流串级闭环。

## 硬件配置

| 功能 | M0 | M1 |
|------|----|----|
| PWM U / V / W | GPIO32 / GPIO33 / GPIO25 | GPIO26 / GPIO27 / GPIO14 |
| MCPWM 组 | Group 0 | Group 1 |
| AS5600 SDA / SCL | GPIO19 / GPIO18（I2C0） | GPIO23 / GPIO5（I2C1） |
| A / B 相电流采样 | GPIO39 / GPIO36 | GPIO35 / GPIO34 |
| 传感器方向 | `-1` | `-1` |
| 初始目标速度 | `5rad/s` | `7rad/s` |

两路驱动器共用 GPIO12 使能。两个 AS5600 地址均为 `0x36`，因此分别使用 ESP32 的 I2C0 和 I2C1。四路电流采样均属于 ADC1，并共享同一个 ADC oneshot unit。

公共硬件参数：

- 驱动器供电电压：`12.6V`
- 电机极对数：`7`
- 分流电阻：`0.01Ω`
- 运放增益：`50`
- PWM：`30kHz`

## 控制结构

每台电机都有独立的编码器、电流采样、滤波器、速度 PID 和电流 PID：

```text
目标速度 → 速度 PID → Iq 目标 → 电流 PID → Uq → FOC/PWM
```

第十课 Arduino 原始参数：

| 控制器 | P | I | D | 输出限制 |
|--------|---|---|---|----------|
| 速度 PID | 3 | 2 | 0 | ±0.5A |
| 电流 PID | 0.5 | 50 | 0 | ±12.6V |

所有 PID 的输出变化率限制为 `100000`。速度低通滤波时间常数为 `10ms`，电流低通滤波时间常数为 `50ms`。

双电机控制在同一个固定 1kHz 高优先级任务中顺序执行，日志在另一个任务中以 10Hz 输出。当前暂不处理 UART 命令，目标速度由 [main.cpp](D:/Word/Code/Gitcode/Foc/esp32_foc_controller/foc_controller/main/main.cpp) 中的宏决定：

```cpp
#define M0_TARGET_VELOCITY 5.0f
#define M1_TARGET_VELOCITY 7.0f
```

## 编译与测试

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

烧录前应确认两台电机、两个 AS5600 和四路电流采样均已连接。上电时程序依次进行两台电机的电角度对齐，然后自动开始运行。

正常日志应同时出现 M0 和 M1：

```text
M0 目标: 5.00, 速度: ..., 位置: ..., Iq目标: ..., Iq: ..., Uq: ...
M1 目标: 7.00, 速度: ..., 位置: ..., Iq目标: ..., Iq: ..., Uq: ..., 环频率: ...Hz
```

稳定后 M0、M1 的速度应分别接近 5 和 7rad/s，位置应持续按各自方向变化，环频率应接近 1000Hz。
