# 双电机控制模式与 UART 协议

本文档说明双电机控制模式、M0 主输入/M1 从电机调速逻辑，以及 USB 控制台和 UART1 共用的文本命令协议。

## 模式选择

在 `main/main.cpp` 顶部修改 `CONTROL_MODE`，然后重新编译烧录：

```cpp
#define CONTROL_MODE_DUAL_VELOCITY       0
#define CONTROL_MODE_MASTER_SLAVE_SPEED  1

#define CONTROL_MODE CONTROL_MODE_MASTER_SLAVE_SPEED
```

### DUAL_VELOCITY

M0、M1 都运行独立的速度—电流串级闭环：

```text
目标速度 → 速度 PID → Iq 目标 → 电流 PID → Uq → FOC/PWM
```

默认目标为 M0=`5rad/s`、M1=`7rad/s`，可以分别通过 UART 修改。两台电机都会进行电角度对齐。

### MASTER_SLAVE_SPEED

M0 是手动输入，M1 是从电机：

```text
转动 M0 → AS5600 角度增量 → 累加/减少 M1 速度目标
                                ↓ 限制到 0～MAX
                    M1 速度 PID → 电流 PID → FOC/PWM
```

- M0 正转提高 M1 速度，反转降低 M1 速度。
- M1 速度目标不会低于 0，因此不会被命令持续反转。
- M1 减速时允许速度环短暂输出负 Iq，用于主动制动。
- M0 不运行 FOC、不进行电角度对齐，三路 PWM GPIO 保持低电平。
- 两路驱动共用 GPIO12 使能，M0 能否完全自由滑行仍取决于驱动芯片的输入真值表。
- 上电只对 M1 对齐；M1 初始目标为 0，等待 M0 输入。

默认主从参数：

```text
GAIN = 5rad/s/圈     M0 每转一圈，M1 目标改变 5rad/s
MAX  = 10rad/s       M1 最高目标速度
输入累计死区 = 0.02rad（约 1.15°）
```

`MASTER_INPUT_DIRECTION` 默认沿用 M0 已验证的编码器方向。如果实际操作时正反调速方向与预期相反，只需单独反转这个宏，不会影响双速度模式下 M0 的 FOC 方向。

在上限继续正转不会增加目标，在下限继续反转不会产生负目标。退出上下限后，反方向输入立即有效。

## 串口配置

```text
UART1 TX：GPIO16
UART1 RX：GPIO17
波特率：115200
格式：8N1
命令结尾：CR、LF 或 CRLF
大小写：不敏感
```

同一套命令也可以从 ESP-IDF monitor 所连接的 USB 控制台输入。

## 运行和停止

```text
RUN M0
RUN M1
RUN ALL
STOP M0
STOP M1
STOP ALL
```

在 `MASTER_SLAVE_SPEED` 模式中：

- `RUN/STOP M0` 表示启用或暂停 M0 手轮输入，不会驱动 M0。
- `RUN/STOP M1` 控制从电机闭环输出。
- `STOP M1` 同时将 M1 目标清零并重置 PID。
- `STOP ALL` 暂停手轮输入、停止 M1 并清零目标。

在 `DUAL_VELOCITY` 模式中，M0/M1 都表示对应电机的闭环启停。

## 设置目标速度

```text
SET M0 VEL 5
SET M1 VEL 7
SET ALL VEL 5 7
```

单位为 `rad/s`。`SET ALL` 只用于 `DUAL_VELOCITY`；主从模式下 M0 是输入端，只允许直接设置 M1：

```text
SET M1 VEL 4
```

主从模式会自动把直接设置的 M1 目标限制到 `0～LINK MAX`。

## 设置 PID

参数顺序统一为：

```text
P I D RAMP LIMIT
```

速度 PID：

```text
SET M0 VPID 3 2 0 100000 0.5
SET M1 VPID 3 2 0 100000 0.5
SET ALL VPID 3 2 0 100000 0.5
```

电流 PID：

```text
SET M0 CPID 0.5 50 0 100000 12.6
SET M1 CPID 0.5 50 0 100000 12.6
SET ALL CPID 0.5 50 0 100000 12.6
```

整组参数会在控制周期边界一次性生效，并清空相应 PID 的积分、误差和输出历史。主从模式中 M0 的 PID 不参与控制，但仍可配置并在切回双速度模式后使用。

速度 PID 的 `LIMIT` 是 Iq 目标上限，UART 最大允许设置为 `1A`；电流 PID 的电压输出上限最大允许设置为 `12.6V`。FOC 输出还会在 `Motor_setTorque()` 中把最终 Uq 限制为母线电压的一半。

## 设置主从参数

```text
SET LINK GAIN 5
SET LINK MAX 10
ZERO LINK
GET LINK
```

- `GAIN`：M0 每旋转一圈造成的 M1 目标速度变化量，范围 `(0,100]`。
- `MAX`：M1 目标速度上限，范围 `(0,30]rad/s`，固件硬上限为 `30rad/s`。
- `ZERO LINK`：M1 目标立即归零，清除未处理的 M0 角度增量并重置 M1 PID。

响应示例：

```text
LINK GAIN=5.000 MAX=10.000 TARGET=2.500 PENDING=0.0040
```

## 查询

```text
GET MODE
GET LINK
GET M0 STATE
GET M1 STATE
GET ALL STATE
GET M0 PARAM
GET M1 PARAM
GET ALL PARAM
```

状态响应：

```text
STATE M0 INPUT VEL_T=0.000 VEL=1.230 POS=4.560 IQ_T=0.000 IQ=0.000 UQ=0.000
STATE M1 RUN VEL_T=5.000 VEL=4.960 POS=20.100 IQ_T=0.120 IQ=0.116 UQ=0.830
```

## 周期日志

```text
LOG 10
LOG 20
LOG OFF
```

数字表示每秒日志条数，允许 `1～100`。日志同时输出到 USB 控制台和 UART1：

```text
TEL M0 INPUT VEL=1.230 POS=4.560 M1_TARGET=5.000
TEL M1 RUN VEL_T=5.000 VEL=4.960 POS=20.100 IQ_T=0.120 IQ=0.116 UQ=0.830 HZ=990
```

## 返回格式

成功：

```text
OK RUN M1
OK SET M1 VEL
OK SET LINK MAX 10.000
```

失败：

```text
ERR BAD_COMMAND
ERR BAD_MOTOR
ERR BAD_VALUE
ERR ARG_COUNT
ERR M0_INPUT_ONLY
ERR LINE_TOO_LONG
ERR BUSY
```

输入 `HELP` 可以查看固件内置的简要命令列表。

## 主从模式首次测试

1. 确认 M0/M1 编码器和 M1 驱动、电流采样均已连接。
2. 上电时只有 M1 应执行约 1 秒的电角度对齐，M0 不应主动转动。
3. 对齐完成后 M1 保持停止。
4. 缓慢正转 M0，确认 `M1_TARGET` 从 0 增加且 M1 正向加速。
5. 反转 M0，确认目标下降；持续反转后目标停在 0。
6. 正转到上限，确认目标不超过 `10rad/s`。
7. 输入 `ZERO LINK`，确认 M1 减速至停止。
8. 输入 `STOP ALL`，确认手轮输入暂停且 M1 无控制输出。
