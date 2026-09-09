# CESC FOC 硬件平台导读

## 1. 硬件功能分块

本平台可按能量流和信号流分为五块：

```text
直流母线 -> 三相MOSFET逆变桥 -> 电机 U/V/W
                ^        |
                |        +-> 三相分流器 + AD8418 -> ADC1/2/3
STM32 PWM -> DRV8301
                ^
                +-> nFAULT / SPI状态

电机轴磁铁 -> AS5600 -> I2C2 -> STM32电角度
PC上位机 <-> USB CDC <-> STM32命令与遥测
```

控制核心是 STM32F405RGT6。DRV8301 把 MCU 的六路互补 PWM 转换为 MOSFET 栅极驱动，并通过 `EN_GATE`（代码名 `DRV_EN_GATE`）、`nFAULT` 和 SPI 与 MCU 配合。原理图文件位于 `Document/Schematic_Prints.pdf` 和 `Document/schematic_copy.pdf`。

## 2. MCU 与主要信号

下表由 `CESC_Application.ioc`、`Core/Inc/main.h` 和外设初始化代码交叉整理。

| 功能 | MCU 引脚/外设 | 工程信号 | 代码位置 |
|---|---|---|---|
| U/V/W 高侧 PWM | PA8/PA9/PA10，TIM1 CH1/2/3 | 三相 PWM 正端 | `Core/Src/tim.c` |
| U/V/W 低侧 PWM | PB13/PB14/PB15，TIM1 CH1N/2N/3N | 三相 PWM 互补端 | `Core/Src/tim.c` |
| 相电流 1/2/3 | PC0/PC1/PC2 | `CURRENT_1/2/3` | `Core/Src/adc.c` |
| 母线电压 | PC3，ADC1 IN13 | 未单独命名 | `power_stage_process()` |
| 栅极使能 | PB5 GPIO | `DRV_EN_GATE` | `drv8301.c`, `power_stage.c` |
| 驱动故障 | PB7 EXTI7，下降沿 | `DRV_FAULT_N` | `HAL_GPIO_EXTI_Callback()` |
| DRV8301 SPI | PC10/11/12 + PC9 | SCK/MISO/MOSI/CS | `Core/Src/spi.c`, `drv8301.c` |
| AS5600 I2C | PB10/PB11，I2C2 | SCL/SDA | `angle_sensor.c` |
| USB CDC | PA11/PA12 | USB DM/DP | `USB_DEVICE/` |
| 调试下载 | PA13/PA14 | SWDIO/SWCLK | `.ioc` |
| 状态灯 | PB0/PB1 | 绿灯/红灯 | `application.c` |

## 3. PWM 发生器

TIM1 的 APB2 定时器时钟是 168 MHz，预分频为 0，采用中心对齐计数，ARR=4199。因此：

```text
fPWM = 168 MHz / [2 × (4199 + 1)] = 20 kHz
```

CH1~CH3 和互补通道驱动三相桥。CCR1~CCR3 是三相占空比命令。中心对齐的优点是开关谐波对称，也方便在零矢量附近安排电流采样。

死区配置值为 84。对 STM32 高级定时器而言，死区的实际时间需要按芯片参考手册的 BDTR.DTG 分段编码解释，不能在改变时钟后简单把它永久当作固定纳秒值。当前值落在线性区时约为 84/168 MHz = 0.5 us，修改前仍应结合 MOSFET、DRV8301 传播延迟和示波器波形确认。

TIM1 CH4 没有输出到 GPIO，只服务于采样/测量时序。`power_stage_disable()` 会关闭 CH1~CH3 的六个输出和 `EN_GATE`，但在需要持续采样时允许 CH4 继续运行。

## 4. 电流采样及同步

三相电流分别进入 ADC1/ADC2/ADC3 的注入通道，均为 12 位、15 周期采样。TIM1 在中心对齐计数的两端产生更新事件；该事件复位 TIM8，TIM8 再延时 200 个 168 MHz 时钟后用 CC2 同时触发三路 ADC：

```text
TIM1零矢量边界 -> TIM8复位 -> 200 ticks(约1.19 us) -> ADC1/2/3注入转换
```

这样给栅极驱动和电流放大器留出建立时间。ADC1 注入转换完成后进入 `HAL_ADCEx_InjectedConvCpltCallback()`，同时读取三只 ADC 的 JDR，形成约 20 kHz 的控制节拍。

代码注释给出的模拟链为：每相 0.5 mΩ 分流器、AD8418 增益 20 V/V、VDDA=3.3 V。因此理想换算为：

```text
每ADC码电压 = 3.3 / 4095
每ADC码电流 = (3.3 / 4095) / (20 × 0.0005)
              ≈ 0.080586 A/count
```

由于原理图中放大器输入方向与软件定义的“桥到电机为正”相反，配置值为 `-0.08058608 A/count`。这一负号非常关键：若硬件版本或相线/放大器输入改变，Park 变换后的 Iq 符号也会改变。

启动时栅极关闭，软件累计 1024 组样本得到每相零偏。运行时还会依据 PWM 矢量判断某一相是否不可观测，并利用 `Ia+Ib+Ic=0` 重构该相。调试电流采样应检查：

1. 三相静态零偏是否稳定并远离 ADC 上下边界。
2. 手动给电机相线注入已知小电流时，ADC 变化量和换算是否一致。
3. 正方向定义是否与编码器角度、相序和 Iq 正方向一致。
4. PWM 开关后，采样点是否避开尖峰并有足够建立时间。

当前每码约 80.6 mA，而默认最大 Iq 只有 300 mA，只有数个 ADC 码的有效变化，属于明显的控制精度限制。后续若改板，优先评估分流电阻、放大器增益/带宽、偏置范围、ADC 噪声和过流范围之间的折中。

## 5. 母线电压采样

PC3/ADC1 IN13 是母线电压常规通道。代码假设分压电阻为 39 kΩ 和 2.2 kΩ，换算式为：

```text
Vbus_mV = ADCraw × 3300 × (39000 + 2200) / (4095 × 2200)
```

软件每 100 ms 采样一次，默认只允许 6~15 V 母线使能测试或控制。修改分压电阻、VDDA 或额定母线后，必须同时检查换算常量、MOSFET/电容/DRV8301 的硬件额定值和配置校验上限。

## 6. 转子位置

AS5600 是 12 位绝对磁编码器，一圈 4096 码，I2C 地址由驱动定义，I2C2 运行于 400 kHz。传感器任务每 2 ms 读取一次，即 500 Hz；20 kHz 快环不能等待 I2C，而是读取共享快照，并通过 PLL/相位预测把低速角度采样插值到 PWM 节拍。

机械角到电角的关系是：

```text
theta_e = pole_pairs × theta_m - electrical_zero
```

默认 `pole_pairs=11`。电角零点不是 AS5600 的机械零点，必须通过对齐流程得到。磁铁偏心、气隙不正确或磁场过强/过弱都会直接造成转矩脉动甚至失控。

## 7. 驱动与保护链

软件有多层关断：

- `DRV_FAULT_N` 下降沿外部中断；
- DRV8301 状态寄存器故障读取；
- 连续 3 次超过 40 ADC counts 的软件过流；
- 速度超过配置上限 10%；
- 编码器失效或样本过旧；
- 外部控制命令 1000 ms 未刷新；
- Cortex 故障捕获代码直接撤销 TIM1 MOE、通道使能和 `EN_GATE`。

这些保护是学习和调试的底线，但不替代保险丝、限流电源、硬件过流设置、正确的功率布局以及额定值校核。尤其注意 TIM1 的 Break 输入当前配置为禁用，异步硬件保护主要依赖 DRV8301 自身和 `nFAULT` 路径。

## 8. 看原理图时的顺序

打开 `Document/Schematic_Prints.pdf`，建议按以下问题逐页追踪：

1. 电源入口：允许电压、反接/浪涌/保险保护、母线电容和 3.3 V 来源。
2. 每个半桥：MOSFET 型号、耐压、栅电阻、上下管连接和相线端子。
3. DRV8301：PVDD/GVDD/DVDD、bootstrap、电荷泵、OC_ADJ、模式脚及 SPI 接法。
4. 三个分流器和 AD8418：阻值、Kelvin 走线、输入极性、滤波与 ADC 量程。
5. 母线分压：确认 39 kΩ/2.2 kΩ 与 PC3 一致。
6. AS5600：供电、上拉、电容、接口和机械安装要求。
7. SWD、USB、LED 和测试点：确定安全调试与示波器接入位置。

若原理图、BOM、实物丝印与代码注释不一致，以实际装配版本为准，并先修正文档/配置再上电。
