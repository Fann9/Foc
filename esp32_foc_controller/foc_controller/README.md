# ESP32 双电机 FOC 控制器

基于 ESP32 + ESP-IDF v6.0 的双电机 FOC 控制器，控制算法和参数来自 DengFOC 第九、十课示例。目前支持：

- M0、M1 两台电机独立的速度—电流串级闭环。
- M0 作为手动输入、控制 M1 单向速度的主从模式。
- USB 控制台和 UART1 分别设置两台电机的目标、PID、启停和日志。

当前默认编译 `MASTER_SLAVE_SPEED`。完整模式说明、命令格式和测试步骤见 [README_MODES_UART.md](D:/Word/Code/Gitcode/Foc/esp32_foc_controller/foc_controller/README_MODES_UART.md)。

## 硬件配置

| 功能 | M0 | M1 |
|------|----|----|
| PWM U / V / W | GPIO32 / GPIO33 / GPIO25 | GPIO26 / GPIO27 / GPIO14 |
| MCPWM 组 | Group 0 | Group 1 |
| AS5600 SDA / SCL | GPIO19 / GPIO18（I2C0） | GPIO23 / GPIO5（I2C1） |
| A / B 相电流采样 | GPIO39 / GPIO36 | GPIO35 / GPIO34 |
| 传感器方向 | `-1` | `-1` |

两路驱动共用 GPIO12 使能。公共参数如下：

- 驱动器供电电压：`12.6V`
- 电机极对数：`7`
- 分流电阻：`0.01Ω`
- 运放增益：`50`
- PWM：`30kHz`
- 控制任务：`1kHz`

## 默认 PID

| 控制器 | P | I | D | RAMP | LIMIT |
|--------|---|---|---|------|-------|
| 速度 PID | 3 | 2 | 0 | 100000 | 0.5A |
| 电流 PID | 0.5 | 50 | 0 | 100000 | 12.6V |

速度低通滤波时间常数为 `10ms`，电流低通滤波时间常数为 `50ms`。`Motor_setTorque()` 还会将最终 Uq 限制为母线电压的一半。

## UART1

```text
TX：GPIO16
RX：GPIO17
115200，8N1
```

例如在默认主从模式查询并修改参数：

```text
GET MODE
GET LINK
SET LINK GAIN 5
SET LINK MAX 10
SET M1 VPID 3 2 0 100000 0.5
SET M1 CPID 0.5 50 0 100000 12.6
ZERO LINK
```

## 编译与烧录

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

默认主从模式上电时只有 M1 执行电角度对齐。对齐完成后 M1 目标为 0，手动正转 M0 才会提高 M1 速度，反转 M0 会降低速度。
