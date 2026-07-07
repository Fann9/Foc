# ESP32 FOC 电机控制器

基于 ESP32-S3 + ESP-IDF v6.0 的 FOC (磁场定向控制) 电机控制器。

## 硬件

| 组件 | 型号/参数 |
|------|----------|
| 主控 | ESP32-S3 |
| 电机驱动 | 外置驱动板（自带互补 PWM 和死区控制） |
| 编码器 | AS5600 磁编码器 (I2C) |
| 电机 | 7 极对数 BLDC |

### 引脚分配

| 功能 | GPIO |
|------|------|
| PWM U 相 | 32 |
| PWM V 相 | 33 |
| PWM W 相 | 25 |
| 电机使能 | 12 |
| I2C SDA | 19 |
| I2C SCL | 18 |

## 软件架构

```
foc_controller/
├── main/
│   └── main.cpp              # 主程序：双线程启动
├── components/
│   ├── deng_foc/             # FOC 算法库 (MCPWM 驱动)
│   │   ├── DengFOC.cpp
│   │   └── include/DengFOC.h
│   └── as5600/               # AS5600 编码器驱动 (I2C)
│       ├── AS5600.cpp
│       └── include/AS5600.h
└── CMakeLists.txt
```

### 双线程模型

```
┌─────────────────┐    mutex     ┌──────────────────┐
│  sensor_task     │ ──────────→ │  control_task      │
│  优先级 5, 1kHz   │  shared     │  优先级 4, 100Hz    │
│  读 AS5600 编码器  │  angle      │  FOC 计算 + PWM 输出 │
└─────────────────┘             └──────────────────┘
```

- **传感器线程** (sensor_task): 高优先级，每 1ms 读取编码器角度，通过互斥锁写入共享变量
- **控制线程** (control_task): 每 10ms 执行一次 FOC 闭环控制（Park 逆变换 → Clarke 逆变换 → 三相 PWM）

### 实例化设计

所有驱动均采用结构体实例化，支持多电机扩展：

```c
// 每个电机/编码器独立实例
AS5600_t encoder;
Motor_t motor;

// 初始化时绑定硬件和关联关系
AS5600_init(&encoder, I2C_NUM_0, 19, 18, 400000);
Motor_init(&motor, 12.3f, 32, 33, 25, &encoder);
```

扩展第二个电机：

```c
AS5600_t encoder2;
Motor_t motor2;
AS5600_init(&encoder2, I2C_NUM_1, 21, 22, 400000);
Motor_init(&motor2, 12.3f, 4, 16, 17, &encoder2);
```

## FOC 控制流程

```
目标角度 ──→ PI 控制器 ──→ Uq 电压
                              │
                    电角度 ←── 编码器
                              │
                     Park 逆变换 (Uq → Uα, Uβ)
                              │
                    Clarke 逆变换 (Uα,Uβ → Ua,Ub,Uc)
                              │
                    MCPWM 三相 PWM 输出 (30kHz)
```

## 编译和烧录

```bash
# 设置目标芯片
idf.py set-target esp32s3

# 编译
idf.py build

# 烧录 + 监控
idf.py flash monitor
```

## API 参考

### AS5600 编码器

| 函数 | 说明 |
|------|------|
| `AS5600_init(dev, port, sda, scl, freq)` | 初始化编码器 |
| `AS5600_getRawAngle(dev)` | 读取原始角度 (0-4095) |
| `AS5600_getAngle_Without_track(dev)` | 单圈角度 (0~2π rad) |
| `AS5600_getAngle(dev)` | 多圈累计角度 |

### Motor 电机

| 函数 | 说明 |
|------|------|
| `Motor_init(motor, voltage, a, b, c, encoder)` | 初始化电机 MCPWM |
| `Motor_alignSensor(motor, pp, dir)` | 电角度校准 |
| `Motor_setTorque(motor, Uq, angle_el)` | 设置力矩 |
| `Motor_getAngle(motor)` | 获取累计角度 |
| `Motor_electricalAngle(motor)` | 获取当前电角度 |

## 依赖

- ESP-IDF v6.0+
- 灯哥开源 DengFOC 库 (已移植为 ESP-IDF 原生 API)
