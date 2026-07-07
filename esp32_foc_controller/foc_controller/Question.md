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
