**CESC 与本地 Benjamin VESC 的实现对比（2026-10-28）**

对比基准：CESC 当前源码，HEAD `3b8c6a5`；`D:/Desk/Folders/bldc` 的 HEAD `db6ba047`，2019-09-17，固件版本 **3.61**，默认硬件选择 HW60。以下结论针对这些源码和默认配置，不代表最新 VESC，也不代表板上正在运行的固件及 Flash 参数。版本依据：[VESC conf_general.h](D:/Desk/Folders/bldc/conf_general.h:28)。

本次是静态源码核对，未进行烧录、电机运行、示波器测量或动态性能测试。“代码已存在”“默认启用”“已经经过实机验证”是三个不同层次；报告只把前两项作为源码可确认的事实。

**1. 最主要的区别：默认控制链路不同**

VESC FOC 以 d/q 电流闭环作为主要执行层。速度 PID 输出 Iq，位置 PID 也直接输出 Iq；两者不是必须串接的位置—速度—电流三环。

CESC 已有完整的 Clarke/Park、d/q PI、逆变换和 PWM 数学函数，但默认速度、普通位置、位置轨迹主要采用编码器定向的直接 Vq 控制。电流、力矩、触觉模式才进入电流环。

| 模式 | 本地 VESC 3.61 | CESC 当前实现 |
|---|---|---|
| 电流 | Iq 目标 → 电流环 | 已实现，默认命令范围 ±300 mA，Id 通常为 0 |
| 力矩 | 主要以电流命令表达 | 显式 mN·m 接口，按 `Iq = T / Kt` 换算；不是力矩传感器反馈 |
| 速度 | eRPM PID → Iq → 电流环 | 机械速度斜坡 → 速度 PI + 前馈 → Vq；高速电流分支默认关闭 |
| 普通位置 | 环绕角度误差 PID → Iq | 累计位置误差 P → Vq，默认没有位置积分或微分 |
| 位置轨迹 | 此核心位置接口没有对应的加减速轨迹发生器 | 独立加速/减速、制动距离规划、轨迹位置误差修正速度，再进入速度控制 |
| 触觉 | 此版本核心接口没有 CESC 这样的统一齿槽/阻尼/端止挡命令 | 已有齿槽、阻尼、位置边界 → Iq 的实现；阻尼速度更新另见第 13 项 |
| 制动 | 独立电流制动、手刹及低速短接逻辑 | 可输出反向电流，但没有等价的完整制动/回充限流模式 |

直接 Vq 控制仍然使用编码器和速度/位置反馈，不应称为“整个系统开环”。区别是它没有以 Iq 反馈闭合最内层，所以电流和力矩对负载、绕组电阻、母线电压变化更敏感。

依据：[VESC 位置 PID](D:/Desk/Folders/bldc/mcpwm_foc.c:2645)、[速度 PID](D:/Desk/Folders/bldc/mcpwm_foc.c:2712)、[CESC 外环](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1075)、[力矩换算](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:2448)。

**2. 功能开关必须以配置及调用路径为准**

| CESC 配置 | 默认值 | 实际含义 |
|---|---:|---|
| `speed_current_foc_enabled` | 0 | 不会按 20 RPM / 15 RPM 门槛进入/退出高速电流环 |
| `high_speed_svm_enabled` | 0 | 六扇区 SVM 函数存在，但默认不用于控制输出 |
| `observer_diagnostic_enabled` | 0 | 磁链观测器的旁路诊断也默认关闭 |
| `high_speed_pll_prediction_enabled` | 1 | 编码器相位预测启用；不等于无感控制 |

这些开关不在 41 字节用户配置载荷中，正常 Flash 用户参数加载不会把它们覆盖为启用。`MotorControl/README.md` 中“20 RPM 以上转电流环、使用六扇区 SVPWM”的叙述已落后于默认源码。

依据：[默认配置](D:/Desk/Folders/CESC_Application/Application/MotorControl/Control/motor_control_config.c:117)、[用户参数结构](D:/Desk/Folders/CESC_Application/Application/MotorControl/Control/motor_control_config.h:102)、[速度切换条件](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1274)、[SVM 调用条件](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3796)。

**3. 电流环数学相近，但电压单位和输出调制不同**

两者均有 d/q PI、积分限幅、二维电压矢量限幅及逆 Park。不能把 CESC 描述为“还没有电流 FOC”。

VESC 的 PI 输出以伏特表示，再按实时母线电压换算调制比；输出统一进入六扇区 SVM。它还计算 d/q 电流滤波值、母线电流估计，并在供观测器使用的电压估计中加入死区修正。此处死区修正作用于估算的 `v_alpha/v_beta`，不是直接把补偿量加到最终 PWM。

CESC 的 PI 输出直接以 PWM timer counts 表示；限幅也以 `ARR / divisor` 表示。母线电压变化会改变相同 PI 参数对应的物理电压增益。默认 PWM 是以 50% 为中心的三相正弦比较值，没有 SVM 的共模平移；六扇区函数虽存在，但受前述开关和速度模式条件限制。

当前电流模式调制上限为 `ARR / 20`；直接电压速度模式可逐步放宽到 1000 counts。两条路径可用电压范围明显不同，不能仅把高速电流开关置 1 就认为与原速度模式等效。

本地 VESC 的电流 PI同样采用较简单的积分限幅。没有在这份 3.61 FOC 核心中发现现代版本常被讨论的 HFI、MTPA、完整弱磁和显式 d/q 交叉耦合前馈实现，不应把它们算作这份 VESC 已有而 CESC 缺失的功能。

依据：[VESC 电流环](D:/Desk/Folders/bldc/mcpwm_foc.c:2432)、[CESC 电流环](D:/Desk/Folders/CESC_Application/Application/MotorControl/FOC/foc.c:46)、[CESC 默认正弦调制](D:/Desk/Folders/CESC_Application/Application/MotorControl/FOC/foc.c:197)、[电压限幅](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3730)。

**4. 角度来源：VESC 的观测器参与控制，CESC 仍依赖 AS5600**

VESC 的磁链观测器实际参与换相角生成，支持无感、编码器、Hall。编码器模式下，`correct_encoder()` 按 eRPM 门槛和 5% 迟滞选择编码器或观测器角度；停驱时仍利用三相端电压跟踪反电势，并预置电压积分状态。编码器驱动包含 ABI、AS5047P SPI、AD2S1205、Sin/Cos，部分功能取决于硬件。

CESC 的 `foc_observer.c` 已移植同类六次迭代磁链观测器、PLL、角度选择函数。但是调用处明确丢弃角度选择返回值，观测器没有接管 PWM；默认甚至不运行这段诊断。CESC 没有实现等价的无感运行、Hall 换相、停驱反电势跟踪链路。

CESC 的实际角度增强来自 AS5600 的采样时间戳、速度外推、固定延迟补偿和连续相位校正。高转速时使用预测相位仍然依赖编码器，不等于切到磁链观测器。

依据：[VESC 角度选择](D:/Desk/Folders/bldc/mcpwm_foc.c:1963)、[停驱跟踪](D:/Desk/Folders/bldc/mcpwm_foc.c:2054)、[编码器驱动类型](D:/Desk/Folders/bldc/encoder.c:68)、[CESC 观测器](D:/Desk/Folders/CESC_Application/Application/MotorControl/FOC/foc_observer.c:26)、[旁路调用](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3132)。

**5. 传感器带宽与小电流量程，是 CESC 的现实约束**

CESC AS5600 为 12 位机械角度，每 2 ms 轮询一次，标称 500 Hz；VESC AS5047 采样定时器配置为 20 kHz。CESC 为降低总线负担和处理延迟，增加了机械位置 PLL、0.4 ms 相位提前和 DWT 时间戳插值。

按 CESC 默认 11 极对数计算：一个机械编码器 count 约为 0.0879°机械角、0.967°电角度；500 RPM 时两个 2 ms 样本之间约跨越 66°电角度。这解释了高速时为何相位时间戳及插值变得敏感，不能把它简单归因于 PID 没调好。

CESC 标称相电流分辨率约 80.6 mA/count，300 mA 仅相当于约 3.7 个原始 ADC counts。它限制的是单次相电流采样信息量，不等于电流平均值绝不可能更细，但噪声、偏置和采样时刻将直接影响小电流闭环。

**本地 VESC HW60 也配置了 0.5 mΩ 分流电阻与 20 倍增益**，在相同 ADC 参考电压下有相近的标称量化步长。区别在于 VESC 通用默认电流上限是 60 A，而 CESC 上限是 0.3 A；不能说 VESC 天生使用了高精度小电流采样。

依据：[CESC 轮询及角度换算](D:/Desk/Folders/CESC_Application/Application/Devices/AngleSensor/angle_sensor.c:174)、[电流采样说明](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:117)、[VESC 20 kHz 传感器定时](D:/Desk/Folders/bldc/encoder.c:31)、[HW60 采样增益](D:/Desk/Folders/bldc/hwconf/hw_60.h:111)、[VESC 默认电流范围](D:/Desk/Folders/bldc/mcconf/mcconf_default.h:38)。

**6. ADC、定时器与调度实现不同**

| 项目 | VESC | CESC |
|---|---|---|
| 底层及 RTOS | STM32 标准外设库、ChibiOS 3.0.2 | STM32 HAL、FreeRTOS/CMSIS-RTOS2 |
| 构建 | Makefile、多 `hwconf` 板级配置 | CMake/CubeMX，当前 STM32F405 板 |
| 电流采样 | 三 ADC 规则同步扫描、DMA 完成进入控制 | 三 ADC 注入组使用共同 TIM8 CC2 触发，ADC1 完成中断读取三个结果 |
| 采样时刻 | TIM1/TIM8 同步，按硬件和 `foc_sample_v0_v7` 决定有效控制时刻 | TIM1 上下边界复位 TIM8，200 timer counts 后采样 |
| 电流重构 | 按 V0/V7、比较寄存器及采样模式选择可靠通道 | 已有相同思路，但控制用输入与保护用输入的选择并不完全一致 |
| 速度外环 | 专门线程，名义 1 ms | 状态任务，名义 10 ms |
| 位置环 | 在 FOC ADC 控制路径调用 | 状态任务，名义 10 ms |
| 实时诊断 | 保存 ADC ISR 耗时 | DWT 主要用于角度时间戳；未见同等的每周期 ISR 耗时统计 |

CESC 的状态任务还承担母线/温度轮询、遥测和配置保存；电流控制在中断中执行。三个电机控制编译单元单独使用 `-O2`。这些是调度与实现选择，不能仅从 RTOS 名称判断性能优劣。

依据：[VESC ADC/DMA 初始化](D:/Desk/Folders/bldc/mcpwm_foc.c:302)、[采样筛选](D:/Desk/Folders/bldc/mcpwm_foc.c:1664)、[CESC 同步采样](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:732)、[CESC 任务](D:/Desk/Folders/CESC_Application/Application/App/application_tasks.c:9)、[CESC 构建](D:/Desk/Folders/CESC_Application/CMakeLists.txt:63)。采样周期一致性问题见第 13 项。

**7. 参数辨识已有对应方法，但自动应用程度不同**

两者都实现了锁定电流测电阻和脉冲测电感，并使用相应的 2/3 参数换算。CESC 目前有效电阻结果来自正向测量，旧的反向字段被置零，不能按字段名认为当前在做正反向消偏平均。

VESC 的电阻测量固定角度后使用 Iq，并对电压/电流矢量幅值求平均；CESC 使用固定角度 Id、斜坡/稳定/采样状态机，累计 Vd/Id。CESC 电感路径包含针对本板的基线扣除、脉冲搜索和定时器切换，不是完全无差别复制。

磁链方面，VESC 有开环 FOC 测磁链及整套自动辨识流程。CESC 使用编码器辅助的旋转测量，以 Vq、Iq、已测电阻和电角速度估算磁链，再发布 Ke/Kv。

VESC 的 `detect_apply_all_foc()` 会测量、选择电流范围、计算电流 PI 和观测器增益、识别传感器并按选项保存。源码中有 `Kp = L × bandwidth`、`Ki = R × bandwidth`。CESC 的 R/L/磁链结果主要是诊断结果；本工程未见同等“测完全部参数 → 自动重算控制器 → 自动应用”的闭环流程。修改用户 R/L/磁链也不会自动重算当前的 PWM-count PI。

编码器方面，VESC 能检测 offset、ratio、inverted；CESC 重点实现给定极对数下的零位校准和保存，没有对应的完整传感器自动选择及传动比/方向配置体系。

依据：[VESC R/L](D:/Desk/Folders/bldc/mcpwm_foc.c:1303)、[VESC 自动整定](D:/Desk/Folders/bldc/conf_general.c:1032)、[增益计算](D:/Desk/Folders/bldc/conf_general.c:1130)、[编码器辨识](D:/Desk/Folders/bldc/mcpwm_foc.c:1074)、[CESC R 结果](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:550)、[磁链结果](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:626)、[配置应用](D:/Desk/Folders/CESC_Application/Application/MotorControl/Control/motor_control_config.c:202)。

**8. 保护：CESC 已有基本闭环，但 VESC 统一限流覆盖更广**

CESC 已实现：驱动器故障中断关断、相电流连续超限关断、直接速度电压模式快速降压、命令超时中断关断、编码器陈旧检查、运行中母线欠压/过压、速度模式超速、板上 NTC 异常及过温保护。

默认值包括母线 18–30 V、85°C 开始温度降额、100°C 关断、命令超时 1 s。不要把 CESC 评价为“没有温度保护”。温度转换的 Beta=3380 是当前假设，源码注明还需匹配实装 NTC。

但 CESC 有几个不同含义的电流门槛：

| 门槛 | 当前默认 | 作用 |
|---|---:|---|
| 用户 Iq 命令上限 | 300 mA | 电流/力矩等命令范围 |
| 速度电压模式慢限流 | 800 mA | 依赖 `control_iq_ma` 的降压逻辑，更新问题见第 13 项 |
| 速度电压模式快速相电流限制 | 30 ADC counts，约 2.42 A | 连续触发后降低 Vq |
| 软件硬过流 | 40 ADC counts，约 3.22 A，连续 3 样本 | 关断、锁存故障 |

因此 **300 mA 不是所有模式下统一生效的实际相电流硬上限**。

VESC 在 `mc_interface` 汇总电机/输入电流正负限值、母线功率、低电池电压、MOS/电机温度、转速降额，再进入 FOC Iq 限幅；还有输入电流估计、回充限制、Ah/Wh、采样偏置越界、电流不平衡、故障瞬间数据记录。CESC 当前没有对应的完整统一限额体系，也没有电机绕组温度反馈。

VESC 另外启用 IWDG，并检查多个执行路径是否持续喂狗；CESC 应用中未见 IWDG/WWDG 初始化。CESC 的 ADC 中断命令超时保护能处理通信卡住，但不能等价覆盖 CPU/中断执行本身停滞。

VESC 的故障状态有等待时间后解除的机制；CESC 的硬故障锁存在本应用路径中主要通过重新初始化/复位清除。两者恢复策略不同，不应无条件套用同一上位机重试逻辑。

依据：[CESC 温度及母线保护](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1522)、[中断命令超时](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:2957)、[快速相电流限制](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3077)、[VESC 限额汇总](D:/Desk/Folders/bldc/mc_interface.c:1693)、[FOC 输入电流限幅](D:/Desk/Folders/bldc/mcpwm_foc.c:2031)、[硬件看门狗](D:/Desk/Folders/bldc/timeout.c:35)、[故障记录](D:/Desk/Folders/bldc/mc_interface.c:1338)。

**9. 配置存储：CESC 已经具备完整的基本持久化设计**

VESC 用 `mc_configuration` / `app_configuration`、配置序列化及 Flash EEPROM 仿真，覆盖大量控制与应用参数。

CESC 将运行配置、默认配置、用户可持久化参数分开。Flash A/B 槽位于 `0x08040000` / `0x08060000`，记录有 magic、版本、长度、序号、CRC32 和最后写入的 commit 标志；启动选择最新有效记录，无效时回退默认值。编码器电角零位也已保存。协议提供读取、暂存、保存、重载、恢复默认和保存状态。

这是 CESC 已经完成的工程能力；其限制是用户参数面较窄，PI、相位预测、温度参数和功能开关仍主要由源码默认值决定。README 中“将来替换成 RAM/非易失配置”的描述也已过时。

依据：[CESC 槽位校验](D:/Desk/Folders/CESC_Application/Application/MotorControl/Control/motor_config_store.c:9)、[保存事务](D:/Desk/Folders/CESC_Application/Application/MotorControl/Control/motor_config_store.c:162)、[配置协议](D:/Desk/Folders/CESC_Application/Application/Communication/Cesc/cesc_protocol.c:1150)、[VESC EEPROM 配置](D:/Desk/Folders/bldc/conf_general.c:268)。

**10. 通信协议与升级：同类需求，线协议不兼容**

VESC 使用 `packet.c` 的长度分帧、payload CRC16、结束符及 `COMM_*` 命令；USB、UART、CAN 等接入命令层，并有 CAN 转发和多种输入应用。

CESC 当前是 USB CDC 上的自定义 CE 协议：`0x43 0x45`、版本、消息类型、service、command、sequence、length、CRC16；有 HELLO 会话、状态码、通信统计、异步遥测、配置和升级服务。不能直接用 VESC Tool 的原协议访问。`SERVICE_DIAGNOSTIC` 虽已分配编号，但分发时返回 `NOT_SUPPORTED`；大量现有诊断数据走 motor/status 接口。

两者都有暂存镜像和跳转 Bootloader 的升级思路。CESC 增加顺序 offset 检查、重复块内容核对、整镜像 CRC32，并生成长度+CRC16 的 Bootloader 头。暂存区 `0x08080000`、Bootloader `0x080E0000` 与 VESC 的布局思路对应；本次未审查另一个 Bootloader 工程，不能把应用层存在升级代码当成升级全链路已验证。

依据：[VESC 帧格式](D:/Desk/Folders/bldc/packet.c:64)、[VESC 命令](D:/Desk/Folders/bldc/commands.c:250)、[CESC 帧格式](D:/Desk/Folders/CESC_Application/Application/Communication/Cesc/cesc_protocol.c:274)、[服务分发](D:/Desk/Folders/CESC_Application/Application/Communication/Cesc/cesc_protocol.c:1240)、[升级块处理](D:/Desk/Folders/CESC_Application/Application/Services/FirmwareUpdate/firmware_update.c:189)、[VESC Flash 布局](D:/Desk/Folders/bldc/flash_helper.c:34)。

**11. 遥测与验证工具的侧重点不同**

VESC 有通用电流、电压、温度、能量统计、终端、故障快照、最多 2000 点的内部采样缓冲，以及 `virtual_motor.c` 虚拟电机和 packet recovery 测试。

CESC 有大量面向调试的 ADC、重构、PI 饱和、R/L/磁链、编码器和控制状态统计，也有串口 smoke、限时速度及阶段验收脚本。当前可订阅流枚举的是 7 个角度/状态/温度/温度限额通道；接口最多容纳 8 个通道，不代表已经具备 8 路电流高速波形流。周期推送由 10 ms 状态任务调用，也不等于 ADC 采样频率。

这些脚本的存在说明已有测试入口，但本次没有执行它们，不能据此声称所有模式或全部保护已通过实机测试。

依据：[VESC 采样缓冲](D:/Desk/Folders/bldc/mc_interface.c:80)、[虚拟电机](D:/Desk/Folders/bldc/virtual_motor.c:88)、[CESC 通道枚举](D:/Desk/Folders/CESC_Application/Application/Communication/Cesc/cesc_protocol.c:100)、[周期发送](D:/Desk/Folders/CESC_Application/Application/Communication/Cesc/cesc_protocol.c:1365)、[阶段验收脚本](D:/Desk/Folders/CESC_Application/scripts/cesc_phase1_validation.ps1:1)。

**12. 模块划分：CESC 已分目录，但运行控制职责仍集中**

VESC 主要分为 applications、commands/packet、mc_interface、mcpwm_foc/mcpwm、encoder/hwconf。它支持 FOC、六步 BLDC、DC 及 GPD 等执行模块，板级适配范围更广。

CESC 已把 FOC 数学、角度传感器、驱动器、通信、配置及升级拆开，便于独立理解。但 `power_stage.c` 同时承载硬件、保护、速度/位置/触觉、相位预测、辨识状态机与统计；其职责显著超出纯功率级驱动。后续扩展时可优先把这些控制状态及限额管理拆开，而不是仅继续增加目录或功能开关。

依据：[CESC 模块编译清单](D:/Desk/Folders/CESC_Application/CMakeLists.txt:44)、[CESC 控制入口](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1034)、[VESC 多执行模块](D:/Desk/Folders/bldc/mc_interface.c:1386)。

**13. 对比中发现的具体一致性问题**

以下区分源码能直接确认的条件遗漏、需测量的时序推断，以及需要实物核验的约定差异。

**A. 采样触发频率与固定 dt 存在高优先级疑点。** CESC TIM1 使用 168 MHz、中心对齐、ARR=4199、RCR=0，PWM 完整周期约 50 μs。TIM8 又在上下边界都被复位并触发 ADC，因此从配置推算正常注入采样约 40 kHz、间隔 25 μs。回调中没有像 VESC 那样跳过一半零矢量，但电流环 dt 固定为 50 μs，限流保持时间也按每毫秒 20 个周期计算。若硬件运行符合此触发配置，则积分速度约为预期两倍、周期计数保持时间约减半。应通过 ISR 计数/DWT 或 GPIO 示波确认，不能把注释中的“20 kHz ISR”当成测量结果。

VESC 自己的 `foc_f_sw` 命名也应谨慎：默认 20000、ARR=`168 MHz / foc_f_sw`，中心对齐完整 PWM 约 10 kHz；默认跳过 V7 后控制 dt 按 `1 / (foc_f_sw / 2)` 计算。不能只对比两个文件里“20000”这个数字。

依据：[CESC TIM1](D:/Desk/Folders/CESC_Application/Core/Src/tim.c:46)、[TIM8 触发](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:732)、[固定 dt](D:/Desk/Folders/CESC_Application/Application/MotorControl/Control/motor_control_config.c:105)、[保持周期换算](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3117)、[VESC dt](D:/Desk/Folders/bldc/mcpwm_foc.c:1832)。

**B. 默认直接 Vq 速度模式不会进入声称的降频 d/q 诊断。** `direct_voltage_control=true` 会使 `current_foc_required=false`；随后 Clarke/Park 外层条件只包含若干测试模式和 `current_foc_required`，没有普通直接电压速度/位置模式。正常 `test_kind=NONE` 时，`control_iq_ma` 的赋值不会执行。速度慢限流却读取该字段，所以可能消费初始化值或旧值，不能把 800 mA 慢限流视为可靠的实时 Iq 限流。原始相电流快速限制和硬关断是另外的路径，仍然存在。

依据：[模式分类](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3196)、[遗漏的外层条件](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3341)、[Iq 唯一采样更新处](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3626)、[慢限流读取](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1467)。

**C. `observer_*` 诊断字段混用了两种角度来源。** 磁链诊断先写入这些字段，后面的连续编码器预测又覆盖 `speed_observer_phase_raw`、`speed_observer_erpm` 和 `speed_observer_using_encoder`。最后一个字段甚至由 `precise_encoder_blend < 1` 决定，因此它变成 false 不能证明无感接管；即使打开磁链诊断，同一回调后段仍可能覆盖结果。

依据：[磁链诊断赋值](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3176)、[编码器预测覆盖](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3294)、[标志语义](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3326)。

**D. 触觉模式的阻尼项没有独立更新其速度输入。** 触觉入口先走普通位置初始化，将 `control_speed_actual_mdps` 清零并关闭 `control_speed_current_foc`；外环机械速度更新仅覆盖速度/轨迹模式，ISR 的电气 PLL 更新也仅覆盖速度/轨迹电流模式。触觉阻尼表达式虽存在，但按当前路径其速度输入不会随手动转动正常更新，阻尼效果不能仅凭参数存在就认定实现完整。

依据：[触觉入口](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:2701)、[位置初始化](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:2591)、[机械速度更新条件](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1090)、[阻尼项](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1200)。

**E. 电流源和符号约定需要统一核验。** CESC 虽先构造重构后的 `corrected[]`，但正式控制模式的 `full_clarke_currents` 为 true，实际使用原始 `centered_samples[]`，并对 `CURRENT_ADC_TO_AMPS` 再取负号。旁路观测器使用重构数据和原符号。数学全 Clarke 本身并非错误；是否正确取决于板上采样拓扑、相序、角度与电流定义。但“保护、闭环、观测器已经使用同一套电流”这一判断不成立。应以实测相电流方向及有效采样窗口核对，不能只照搬 VESC 的一个负号。

依据：[电流标度约定](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:117)、[观测器输入](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3139)、[正式控制输入选择](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3604)。

**F. 后续启用高速电流轨迹模式前还需修正输出分支。** 外环允许 POSITION_PROFILE 进入 `control_speed_current_foc`，但 ISR 后面的直接电压输出条件无条件包含 POSITION_PROFILE，会优先走 Vq 输出而跳过电流 PI。此问题当前被默认关闭的高速电流开关遮蔽。

依据：[外环覆盖速度/轨迹](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1075)、[高速切换](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:1274)、[输出分支](D:/Desk/Folders/CESC_Application/Application/MotorControl/PowerStage/power_stage.c:3706)。

**建议推进顺序**

1. 先核实 ADC 触发频率、ISR 耗时、dt，以及相电流符号/采样窗口；修正 d/q 诊断更新和字段来源。
2. 根据实际小电流范围评估采样增益、偏置、噪声与可用带宽，建立可信的 Iq 闭环验证基线。
3. 再逐模式接入统一电流环，处理切换连续性、轨迹分支、触觉速度反馈和统一限流；不直接批量打开现有开关。
4. 补充独立看门狗、故障快照、明确的故障恢复及母线输入/回充限流。
5. 最后完善参数自动整定和观测器验证/接管；若目标是编码器低速伺服，完整无感控制不必先于前四项。

本报告新增分析文档，未修改固件实现。
