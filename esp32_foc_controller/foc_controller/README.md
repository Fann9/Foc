# ESP32 FOC 电机控制器

基于 ESP32 + ESP-IDF v6.0 的 FOC 电机控制器。本版本按 `第九章a+b 位置闭环，速度闭环升级版（加上电流环）` 中 DengFOC V0.4 示例的控制链路和参数移植。

## 硬件

| 功能 | ESP32 GPIO |
|------|------------|
| PWM U / V / W 相 | 32 / 33 / 25 |
| 电机使能 | 12 |
| AS5600 SDA / SCL | 19 / 18 |
| A 相电流采样（VN） | 39 |
| B 相电流采样（VP） | 36 |

- 驱动器供电电压：`12.6V`
- 电机极对数：`7`
- 传感器方向：`-1`
- 分流电阻：`0.01Ω`
- 运放增益：`50`
- PWM：`30kHz`，8 位分辨率

## 控制模式

在 `main/main.cpp` 中修改 `CONTROL_MODE`，重新编译烧录后生效：

```cpp
#define CONTROL_MODE_FORCE_ANGLE                 0 // 位置 → 电流 → 电压
#define CONTROL_MODE_VELOCITY_CURRENT            1 // 速度 → 电流 → 电压
#define CONTROL_MODE_POSITION_VELOCITY_CURRENT   2 // 位置 → 速度 → 电流 → 电压
#define CONTROL_MODE_CURRENT                     3 // 电流 → 电压

#define CONTROL_MODE CONTROL_MODE_VELOCITY_CURRENT
```

默认启用 Arduino 示例中实际运行的速度—电流串级控制。

### Arduino 原始参数

| 模式 | 角度 PID | 速度 PID | 电流 PID |
|------|----------|----------|----------|
| 力位 | `0.5 / 0 / 0.003`，限幅 `0.1A` | — | `1.25 / 50 / 0` |
| 速度—电流 | — | `3 / 2 / 0`，限幅 `0.5A` | `0.5 / 50 / 0` |
| 位置—速度—电流 | `1 / 0 / 0`，限幅 `30rad/s` | `0.02 / 1 / 0`，限幅 `0.5A` | `5 / 200 / 0` |
| 电流力矩 | — | — | `5 / 200 / 0` |

所有 PID 的输出变化率均为 `100000`。速度低通滤波时间常数为 `10ms`，电流低通滤波时间常数为 `50ms`。

## 软件结构

控制计算集中在一个固定周期任务中，保持与 Arduino `loop()` 相同的先后关系：

```text
control_task，1kHz
  编码器采样 → 速度计算/滤波 → 电流采样/滤波
             → 外环 PID → 电流 PID → FOC/PWM

comm_task             串口命令，不参与控制时序
telemetry_task，20Hz  状态日志，不参与控制计算
```

将控制链拆成三个不同频率的控制任务会额外引入跨线程数据时序问题，因此基准移植版只使用一个控制任务；通信和日志仍独立运行。

## 串口命令

命令以回车结束，可通过 USB 控制台或 UART1（TX=GPIO16，RX=GPIO17）输入：

| 命令 | 说明 |
|------|------|
| `<number>` | 设置当前模式目标值 |
| `s` | 将目标值设为 0 |
| `q` | 查看角度、速度、电流 PID 的 P/I 参数 |

速度模式的目标单位是 `rad/s`，`6.28rad/s` 约等于每秒一圈。电流模式的目标单位是 A，位置模式的目标单位是 rad。

## 编译、烧录和监控

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash
idf.py -p COM3 monitor
```

上电时会进行约 1 秒的电角度对齐。默认速度模式完整保留 Arduino 的高增益参数；编码器静止噪声可能使速度环输出快速触及 `±0.5A`，该现象属于原参数基准的测试结果，后续调参应在此基准上单独进行。

## 主要组件

```text
foc_controller/
├── main/main.cpp                 控制模式、参数、任务和串口
├── components/as5600/            AS5600 编码器驱动
├── components/current_sense/     双路在线电流采样
├── components/deng_foc/          FOC 与三相 PWM
├── components/lowpass_filter/    低通滤波器
├── components/pid/               位置型 PID
└── sdkconfig.defaults            FreeRTOS 1kHz tick
```
