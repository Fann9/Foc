# 双电机控制模式与 UART 协议

本文档说明双电机控制模式、单向主从调速、双向电子轴力反馈，以及 USB 控制台和 UART1 共用的文本命令协议。

## 模式选择

在 `main/main.cpp` 顶部修改 `CONTROL_MODE`，然后重新编译烧录：

```cpp
#define CONTROL_MODE_DUAL_VELOCITY       0
#define CONTROL_MODE_MASTER_SLAVE_SPEED  1
#define CONTROL_MODE_BILATERAL_COUPLING  2

#define CONTROL_MODE CONTROL_MODE_BILATERAL_COUPLING
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

### BILATERAL_COUPLING

M0、M1 组成对称的双向电子轴，两台电机都可以作为输入，也都可以作为输出：

```text
M0/M1 位置与速度差 → 虚拟弹簧阻尼 → -Iq 给 M0
                                      → +Iq 给 M1
                          两路 Iq → 各自电流 PID → FOC/PWM
```

耦合电流计算为：

```text
位置误差 E  = M0相对位置 - M1相对位置 / RATIO
速度误差 DE = (E - 上一周期E) / dt，再经过5ms低通滤波
Iq = KP × 限幅(E) + KD × DE
```

- 转动任意一台电机，另一台都会追随。
- 一台电机受到阻挡后，位置误差会产生大小相同、方向相反的两路 Iq，阻力会反馈到另一台电机。
- 此模式不使用速度 PID，只有虚拟联轴器和两台电流 PID。
- 上电完成两台电机对齐后保持 `STOP`，不会主动产生耦合力。
- `RUN M0`、`RUN M1` 和 `RUN ALL` 均会成对启动两台电机，并以启动瞬间的位置建立同步零点。
- 任意 `STOP` 命令也会成对停止两台电机，防止只驱动一侧。
- `ZERO LINK` 使用当前位置重新建立同步零点，不会要求电机回到上电位置。

默认参数：

```text
KP      = 0.80A/rad
KD      = 0.030A/(rad/s)
CURRENT = 0.15A
RAMP    = 10.0A/s
RATIO   = 1.0
位置误差参与计算的硬限制 = ±1.5rad
位置软死区 = 0.005rad
相对速度软死区 = 0.05rad/s
```

双向模式参考第九章力位闭环，默认电流 PID 使用 `P=1.25、I=50、D=0`，电流低通时间常数为 `10ms`。其他两个模式仍使用已经验证的 `P=0.5、I=50` 和 `50ms` 电流滤波。

`RATIO=1` 表示两台电机相对转角相同，此时固件使用 `Iq0=-Iq`、`Iq1=+Iq` 近似实现相同的轴端反作用力。负值可以反转跟随方向；非 1 传动比时 M1 电流会按比例换算并再次限流。机械摩擦、电流零偏和电机转矩常数误差仍会影响实际手感。

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

在 `BILATERAL_COUPLING` 模式中，电子轴必须作为整体启停，因此 M0、M1、ALL 三种电机选择写法都会被接受，但效果相同。

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

双向电子轴模式没有速度目标，使用 `SET ... VEL` 会返回：

```text
ERR LINK_MODE_ONLY
```

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

## 设置双向电子轴参数

可以逐项设置：

```text
SET LINK KP 0.8
SET LINK KD 0.03
SET LINK CURRENT 0.15
SET LINK RAMP 10
SET LINK RATIO 1.0
```

也可以一次设置 KP、KD、最大电流和传动比：

```text
SET LINK PARAM 0.8 0.03 0.15 1.0
```

参数范围：

```text
KP       0～10A/rad
KD       0～1A/(rad/s)
CURRENT  (0,1]A
RAMP     (0,100]A/s
RATIO    -10～-0.1 或 0.1～10
```

修改 `RATIO` 或整组 `PARAM` 时会自动以当前位置重新建立同步零点。`GET LINK` 返回示例：

```text
LINK KP=0.8000 KD=0.0300 CURRENT=0.150 RAMP=10.000 RATIO=1.000 E=0.0250 DE_RAW=0.420 DE=0.310 IQ_REQ=0.029 IQ=0.029
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
ERR LINK_MODE_ONLY
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

## 双向电子轴首次测试

1. 让两个电机轴都处于可以自由转动的状态，上电时不要用手握住电机。
2. 等待两台电机依次完成电角度对齐，确认日志显示 `MODE BILATERAL_COUPLING` 且两台状态均为 `STOP`。
3. 输入 `GET LINK`，确认默认最大电流为 `0.150A`。
4. 输入 `RUN ALL`；命令生效瞬间会自动把当前位置设为同步零点。
5. 很慢地转动 M0 小角度，确认 M1 同向跟随；再反向测试 M1 带动 M0。
6. 如果另一台电机反向运动，立即输入 `STOP ALL`，然后使用 `SET LINK RATIO -1` 后重新测试。
7. 轻轻捏住 M1，再小幅转动 M0，确认 M0 阻力增加；交换 M0/M1 重复测试。
8. 输入 `GET LINK` 和 `GET ALL STATE`，正常情况下 `M0 IQ_T` 与 `M1 IQ_T` 大小相同、符号相反，链接 `IQ` 不超过设定的 `CURRENT`。`DE_RAW` 是位置差直接求导，`DE` 是参与控制的滤波值。
9. 稳定后再逐步提高 `KP` 或 `CURRENT`；出现高频振动时先增大少量 `KD`，仍振动则降低 `KP`。
10. 测试结束输入 `STOP ALL`。

首次测试不要直接提高到 `0.5A`，也不要在一台电机被完全锁死时长时间保持较大的位置差。
