# CESC FOC 软件代码导读

## 1. 不要从 `main()` 一行行读到底

读电机控制工程最有效的方法是同时沿三条线追踪：

- 初始化线：硬件如何进入安全的 READY 状态；
- 快环线：一次 ADC 中断如何由电流和角度算出下一周期 PWM；
- 命令线：上位机目标如何经过外环成为 Id/Iq 或 Vq。

## 2. 软件分层

| 层 | 目录/文件 | 职责 |
|---|---|---|
| 启动与 HAL | `Core/Src/`、`.ioc` | 时钟、GPIO、ADC、TIM、I2C、SPI、FreeRTOS |
| 应用编排 | `Application/App/` | 初始化和任务入口 |
| 通信 | `Application/Communication/` | USB CDC、协议解析、遥测 |
| 器件驱动 | `Application/Devices/` | DRV8301、AS5600 |
| 功率级/状态机 | `MotorControl/PowerStage/power_stage.c` | 采样、保护、使能、外环、辨识、调度 FOC |
| FOC 数学 | `MotorControl/FOC/foc.c` | 坐标变换、电流 PI、反变换、SPWM/SVM |
| 观测器 | `MotorControl/FOC/foc_observer.c` | 磁链观测与 PLL，目前默认仅诊断且关闭 |
| 配置 | `MotorControl/Control/` | 电机参数、限值、增益和 Flash 存储 |

`foc.c` 刻意不依赖 STM32 外设，适合先学算法和做单元测试。`power_stage.c` 很长，是因为它集中承担硬件、安全、控制模式和参数辨识；不要试图第一次就全部读完。

## 3. 启动调用链

从 `Core/Src/main.c` 开始，只看以下链路：

```text
HAL_Init / SystemClock_Config
 -> GPIO, I2C2, ADC1/2/3, SPI3, TIM1, TIM8 初始化
 -> application_init()
     -> motor_config_store_init()
     -> power_stage_init()
     -> cesc_protocol_init()
 -> MX_FREERTOS_Init()
 -> osKernelStart()
```

`power_stage_init()` 的安全逻辑值得逐句读：

1. 先调用 `power_stage_disable()`，撤销 PWM 输出和栅极使能。
2. 清状态和控制器变量。
3. 初始化 DRV8301，检查 `nFAULT`。
4. 配置 TIM1→TIM8→三 ADC 的同步采样链。
5. 启动 ADC 注入转换和 TIM1 CH4。
6. 进入 `POWER_STAGE_CALIBRATING`，收集 1024 个零电流样本。
7. 校准完成后进入 `POWER_STAGE_READY`，并不会自动转动电机。

状态机为：

```text
UNINITIALIZED -> CALIBRATING -> READY -> RUNNING
                        ^          |        |
                        +----------+        v
                                      READY或FAULT
```

## 4. 三类执行上下文

### 4.1 20 kHz ADC 中断：真正的快环

入口是 `power_stage.c` 中的 `HAL_ADCEx_InjectedConvCpltCallback()`。这是最重要、实时性最高的函数，主要完成：

```text
读取ADC1/2/3
 -> 零偏校正与不可观测相重构
 -> 软件过流检查
 -> 获取/预测电角度
 -> 相电流(A/B/C)转换为 Id/Iq
 -> 根据当前模式产生定向Vq，或运行d/q电流PI
 -> 反Park得到 alpha/beta
 -> SPWM或SVM得到三相CCR
 -> 写TIM1 CCR1/2/3
```

中断中还有命令超时的独立关断，避免 USB 或低优先级任务阻塞时电机持续通电。读这个函数时先只标出上述主干，跳过电阻、电感、磁链辨识和统计代码，第二遍再看分支。

### 4.2 500 Hz 传感器任务

`StartSensorTask()` 每 2 ms 调用 `angle_sensor_process()`：读取 AS5600，处理 0/4095 跨圈，累计多圈位置，再计算机械角和电角。共享数据使用序列号快照；快环发现写入进行中时会放弃本周期读取，不会在中断里自旋等待。

### 4.3 100 Hz 状态/外环任务

`StartStatusTask()` 每 10 ms 调用 `application_status_process()`，继而执行 `power_stage_process()`。这里负责：

- 命令看门狗与传感器有效性；
- 速度、位置、轨迹、触觉等外环；
- 速度/位置目标转换为 Iq 目标或直接 Vq 命令；
- 母线电压和 DRV8301 故障处理；
- 参数辨识状态机；
- 协议周期处理、配置存储和 LED 状态。

协议任务在 USB 数据到达时被信号量唤醒，优先级高于传感器任务；它解析命令后调用 `power_stage_set_*()` 系列接口。

## 5. 一次 FOC 快环的数学与代码

### 5.1 相电流与 Clarke 变换

偏置校正后得到 `Ia/Ib/Ic`。`foc_clarke_park()` 先把三相量映射到静止坐标系：

```text
Ialpha = 2/3 Ia - 1/3 Ib - 1/3 Ic
Ibeta  = (Ib - Ic) / sqrt(3)
```

若只可靠测得两相，代码也提供利用 `Ia+Ib+Ic=0` 的简化形式。先在调试器里确认三相静止时接近零、转动时和接近零，再判断变换结果。

### 5.2 Park 变换

用电角度 `theta_e` 把静止坐标旋转到转子坐标：

```text
Id =  cos(theta) Ialpha + sin(theta) Ibeta
Iq =  cos(theta) Ibeta  - sin(theta) Ialpha
```

本工程的角度用 12 位环形量表示：0~4095 对应 0~360°。`foc_sine_raw()` 使用近似计算，`raw+1024` 表示加 90°取得余弦。

表贴式永磁电机常用 `Id_target=0`，`Iq_target` 控制转矩。若相序、电流符号或编码器方向任意一个错误，Id/Iq 解耦都会失效。

### 5.3 d/q 电流 PI

`foc_current_control()` 对 d、q 两轴分别执行：

```text
error = target - measured
integral += error × Ki × dt
voltage = Kp × error + integral
```

当前 `dt=50 us`。积分项先限幅，随后对 `(Vd,Vq)` 矢量整体限幅，保留方向并避免超出可用调制范围。返回结构中的三个 saturation 标志和 `request_counts` 是理解调参是否受电压限制的重要诊断量。

注意本工程 PI 输出的单位是“TIM1 PWM counts”，不是伏特。母线电压变化会改变同一 count 对应的实际相电压，因此它不是严格的母线归一化电流控制器。

### 5.4 反 Park 与 PWM

反 Park：

```text
Valpha = cos(theta) Vd - sin(theta) Vq
Vbeta  = sin(theta) Vd + cos(theta) Vq
```

随后有两条调制路径：

- `foc_voltage_to_pwm()`：正弦 PWM，三相围绕 ARR/2 变化；
- `foc_svm_voltage_to_pwm()`：六扇区空间矢量调制。

默认 `high_speed_svm_enabled=0`，所以不能因为函数存在就认为当前固件已在使用 SVM。

## 6. 电角度如何进入 20 kHz 快环

AS5600 只有 500 Hz。`angle_sensor_process()` 先计算：

```text
electrical_raw_unaligned = mechanical_raw × pole_pairs mod 4096
electrical_raw = electrical_raw_unaligned - electrical_zero mod 4096
```

对齐测试通过给定转子定向电压，把转子锁到已知方向，再调用 `angle_sensor_calibrate_electrical_zero()` 记录零偏。快环通过非阻塞快照取得最新电角度；高速路径可用 PLL 估计速度并逐周期预测相位，还加入传感器延迟相位补偿。

阅读角度代码时重点检查四个量：机械方向、极对数、电角零偏、预测角度。不要仅凭“电机能转”判断角度正确；相差几十电角度仍可能转动，但电流、发热和转矩会很差。

## 7. 控制模式不是同一条路径

`power_stage_control_mode_t` 包括电流、速度、位置、位置轨迹和触觉模式。外环最终有两类输出：

```text
A. Iq目标 -> d/q电流PI -> PWM
B. Vq counts -> 编码器定向电压 -> PWM
```

当前默认配置中，速度电流 FOC 被关闭，速度/位置控制大量使用 B 路径。配置中虽有低速/高速切换、PLL、SVM 和观测器实现，但多个功能开关默认关闭。因此学习时建议先跟踪一次具体命令：

1. 在 `cesc_protocol.c` 找到命令分支。
2. 跳到对应 `power_stage_set_iq_current_ma()`、`power_stage_set_speed_millidegrees_per_second()` 或位置接口。
3. 查看它设置的 `control_mode`、目标和 `current_foc_active`。
4. 在 `power_stage_process()` 看 10 ms 外环如何更新目标。
5. 在 ADC 回调末段看该模式最终进入直接 Vq 还是电流 PI 分支。

## 8. 参数从哪里改

集中查看 `Application/MotorControl/Control/motor_control_config.c`：

| 参数组 | 先理解的字段 |
|---|---|
| 电机本体 | `pole_pairs`, `torque_constant_nm_per_amp`, `motor_resistance_ohm`, `motor_inductance_h`, `motor_flux_linkage_wb` |
| 采样 | `current_adc_amps_per_count`, `current_loop_period_seconds` |
| 电流环 | `current_kp_pwm_counts_per_amp`, `current_ki_pwm_counts_per_amp_second`, `maximum_iq_ma` |
| 编码器 | PLL 增益、相位提前、样本年龄限制 |
| 速度/位置 | 速度 PI、斜坡、最大速度与位置 |
| 安全 | 母线范围、命令超时 |
| 功能开关 | `speed_current_foc_enabled`, `high_speed_svm_enabled`, `observer_diagnostic_enabled` |

不要先随意调 PI。正确顺序应是：确认采样比例和极性 → 确认极对数/相序/编码器方向 → 标定电角零位 → 辨识 R/L/磁链 → 根据带宽设计电流 PI → 再调速度和位置外环。

## 9. 推荐的逐周实验

### 实验 1：工程构建与静态地图

安装 Arm GNU Toolchain、CMake 和 Ninja 后执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

先确认生成 ELF/HEX/BIN，再用 IDE 的 Call Hierarchy 查本文列出的函数。

### 实验 2：只采样、不使能桥

保持 `DRV_EN_GATE` 为低，观察状态从 CALIBRATING 到 READY；记录三相 `current_offset` 和 `bus_voltage_mv`。母线换算应与万用表一致，三相零偏应稳定。

### 实验 3：编码器

手转电机，观察 raw 0~4095、连续 `position_counts`、机械角和未对齐电角。确认一机械圈产生 `pole_pairs` 个电角周期，跨零点位置不跳变。

### 实验 4：纯数学验证

为 `foc.c` 建立主机测试：

- 三相平衡正弦输入经 Clarke/Park 后 Id/Iq 是否为常量；
- 电角度加 90° 后 d/q 是否按预期交换；
- PI 积分和矢量限幅是否触发正确标志；
- 六扇区输入是否输出合法的 0~ARR 比较值。

### 实验 5：低能量功率级检查

使用限流电源和空载电机，先运行项目已有的 commissioning/alignment 流程。示波器依次确认六路互补 PWM、死区、栅极波形、相线波形和故障关断。不要跳过电角对齐直接闭环。

### 实验 6：电流环

从很小 Iq 阶跃开始，记录 Id、Iq、目标、Vd/Vq、饱和标志和母线电压。先让 Id 接近 0、Iq 平稳跟踪，再提高带宽。若只有几码电流变化，优先承认采样硬件限制，不要用极高增益掩盖量化噪声。

### 实验 7：外环

电流环可靠后再闭合速度环，最后才是位置/轨迹。外环带宽必须显著低于电流环；每加一环都保留限幅、斜坡和命令看门狗。

## 10. 常见现象的定位顺序

| 现象 | 优先检查 |
|---|---|
| 一使能就过流 | 相序、上下桥 PWM 对应、死区、电角零点、电流极性 |
| 电机抖动不转 | 极对数、编码器方向/磁铁、对齐零偏、Vq 符号 |
| 能转但电流很大 | 电角相位误差、Id 不为零、采样比例、PWM 饱和 |
| 低速跳动 | AS5600 量化/安装、直接电压增益、静摩擦、电流采样分辨率 |
| 高速失步 | 角度延迟、PLL/相位提前、母线电压不足、电压饱和 |
| 速度波动 | 先看电流环和角度质量，再调速度 PI，不要反向处理 |
| 偶发突然停机 | `drv_faults`、软件故障位、命令超时、编码器样本年龄 |

## 11. 第一遍阅读的最小文件集

按这个顺序即可形成闭环认识：

1. `CESC_Application.ioc`
2. `Core/Src/tim.c` 与 `Core/Src/adc.c`
3. `Application/App/application.c` 与 `application_tasks.c`
4. `Application/MotorControl/Control/motor_control_config.c`
5. `Application/Devices/AngleSensor/angle_sensor.c`
6. `Application/MotorControl/FOC/foc.h` 与 `foc.c`
7. `power_stage_init()` 和 `power_stage_disable()`
8. `HAL_ADCEx_InjectedConvCpltCallback()` 的主干
9. 一个 `power_stage_set_*()` 命令及其外环路径
10. 最后再读参数辨识、观测器和完整协议实现

完成第一遍后，你应该能回答：PWM 何时更新、ADC 何时采样、角度从哪里来、Iq 目标从哪里来、故障如何关断。这五个问题比记住所有函数更重要。

