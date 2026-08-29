# CESC 上位机通信协议

本文档对应当前 `CESC_Application` 固件实现，供上位机 UI 开发使用。本文中的多字节整数和 `float32` 均采用小端序，带符号量使用二进制补码。

## 1. 传输接口

- 接口：USB CDC 虚拟串口
- 当前测试端口：由 Windows 动态分配，例如 `COM15`
- 串口参数：`115200, 8N1`（USB CDC 实际传输不依赖物理 UART 波特率）
- 单帧最大 Payload：1024 字节
- 请求必须串行编号；推荐同一时刻只保留一个未完成请求

## 2. 帧格式

| 偏移 | 长度 | 字段 | 值/说明 |
|---:|---:|---|---|
| 0 | 1 | Magic 0 | `0x43`，ASCII `C` |
| 1 | 1 | Magic 1 | `0x45`，ASCII `E` |
| 2 | 1 | Version | 当前为 `1` |
| 3 | 1 | Message Type | `0` 请求，`1` 响应，`3` 数据流 |
| 4 | 1 | Service | 服务号 |
| 5 | 1 | Command | 命令号 |
| 6 | 2 | Sequence | `uint16`，小端；响应原样返回 |
| 8 | 2 | Payload Length | `uint16`，小端 |
| 10 | N | Payload | 命令数据 |
| 10+N | 2 | CRC16 | 小端 |

CRC 使用 `CRC-16/CCITT` 多项式 `0x1021`，初始值 `0x0000`，不反射，无最终异或。CRC 覆盖从 `Version` 到 Payload 末尾，即字节 `[2, 10+N)`，不包含 Magic 和 CRC 自身。

所有响应 Payload 的前两个字节都是 `status:uint16`。只有 `status=0` 时，后续字节才是响应数据。

状态码：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | OK | 成功 |
| 1 | INVALID_SERVICE | 服务号无效 |
| 2 | INVALID_COMMAND | 命令号无效 |
| 3 | INVALID_LENGTH | Payload 长度错误 |
| 4 | INVALID_ARGUMENT | 参数格式或枚举值错误 |
| 5 | NOT_READY | 未握手、未校准、故障锁存或功率级未就绪 |
| 6 | BUSY | 固件正在执行其他操作 |
| 10 | NOT_SUPPORTED | 当前固件未实现 |
| 11 | VERSION_MISMATCH | 协议版本不兼容 |
| 14 | OUT_OF_RANGE | 参数超出允许范围 |
| 15 | VERIFY_FAILED | 校验失败 |

## 3. 建立会话

打开串口后必须首先发送：

### HELLO

- Service：`0x00`（System）
- Command：`0x00`
- 请求 Payload（6 字节）：

| 偏移 | 类型 | 字段 | 推荐值 |
|---:|---|---|---:|
| 0 | uint8 | 客户端最低协议版本 | 1 |
| 1 | uint8 | 客户端最高协议版本 | 1 |
| 2 | uint32 | 客户端标识/随机数 | 任意 |

成功响应数据（不含开头的 `status`）：`version:uint8`、`maxPayload:uint16`、`capabilities:uint64`、`sessionId:uint32`。

除 System 服务外，其余命令在 HELLO 成功前均返回 `NOT_READY`。

## 4. 电机控制命令

电机服务 Service 固定为 `0x05`。

| Command | 功能 | 请求 Payload | 当前范围 |
|---:|---|---|---|
| `0x02` | STOP | 空 | 立即释放控制、关闭 PWM/驱动，并清除命令超时锁存 |
| `0x03` | 编码器电角度对齐 | 空 | 电机会短暂运动；首次安装或机械关系改变后执行 |
| `0x09` | 设置 Iq 电流 | `iqMa:int32` | `-300..300 mA`；0 表示释放 |
| `0x0A` | 设置速度 | `speedMdps:int32` | `-3000000..3000000 mdps`，即 `-500..500 RPM` |
| `0x0B` | 设置绝对位置 | `positionMdeg:int32` | `-3600000..3600000 mdeg`，即编码器累计位置 `-10..10` 圈 |
| `0x0C` | 位置轨迹 | 见下表 | 目标位置、最高速度、加速度和减速度 |
| `0x0D` | 旋钮/触觉模式 | 见下表 | 当前算法已实现，但现有电流采样分辨率下不建议开放给普通用户 |
| `0x0E` | 设置估算轴力矩 | `torqueMNm:int32` | `-69..69 mN·m`，由当前 `Kt=0.23 N·m/A` 和 `±300 mA` 推导 |
| `0x0F` | 读取精简控制状态 | 空 | 推荐 UI 轮询 |
| `0x00` | 读取完整功率级诊断 | 空 | 调试页面使用，响应较长且布局会随诊断扩展 |

正负号表示两个旋转方向。UI 不应写死“正转=顺时针”，因为视觉方向取决于电机安装方向和编码器方向。

### 4.1 速度单位换算

固件速度单位为机械角 `millidegrees/second`：

```text
speedMdps = RPM * 6000
RPM = speedMdps / 6000
```

例如：100 RPM=`600000 mdps`，500 RPM=`3000000 mdps`。

速度、Iq、力矩、位置保持和位置轨迹命令都受 1000 ms 命令看门狗保护。上位机必须以 100–200 ms 周期重复发送当前命令。不要只发送一次。通信中断超过 1000 ms 后固件会停止，并锁存超时；恢复通信后必须先发送 STOP，再重新下发运动命令。

### 4.2 位置轨迹 `0x0C`

推荐使用 16 字节形式：

| 偏移 | 类型 | 字段 | 范围 |
|---:|---|---|---|
| 0 | int32 | `targetPositionMdeg` | `-3600000..3600000` |
| 4 | int32 | `maxSpeedMdps` | `50..3000000` |
| 8 | int32 | `accelerationMdps2` | `100..600000`；0 仅在兼容形式中表示默认值 |
| 12 | int32 | `decelerationMdps2` | `100..600000`；0 仅在兼容形式中表示默认值 |

固件也接受 8 字节兼容形式（仅目标位置和最高速度），此时加减速度均采用默认值 `10000 mdps²`。

目标位置是累计机械绝对位置，不是相对位移。UI 若提供“转 N 圈”，应先读取当前位置，再计算：

```text
targetPositionMdeg = currentPositionMdeg + turns * 360000
```

### 4.3 旋钮模式 `0x0D`

Payload 固定 20 字节：

| 偏移 | 类型 | 字段 | 范围 |
|---:|---|---|---|
| 0 | int32 | `detentSpacingMdeg` | `100..360000` |
| 4 | int32 | `detentStrengthMa` | `0..300` |
| 8 | int32 | `dampingMaPerDps` | `0..100` |
| 12 | int32 | `minimumPositionMdeg` | `>= -3600000` |
| 16 | int32 | `maximumPositionMdeg` | `<= 3600000` 且必须大于最小位置 |

## 5. 控制状态 `0x0F`

成功响应去掉前两个字节 `status` 后，当前数据长度为 96 字节。UI 的主状态页建议只解析稳定的前 46 字节；后面的 CPU/调试字段用于诊断。

| 数据偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | uint8 | `powerState` | 0 未初始化，1 校准，2 就绪，3 运行，4 故障 |
| 1 | uint8 | `flags` | bit0 Gate 开；bit1 PWM 开；bit2 nFAULT 有效；bit3 母线电压有效 |
| 2 | uint16 | `drvFaults` | 0 为正常 |
| 4 | uint32 | `busVoltageMv` | 母线电压 mV |
| 8 | uint8 | `controlMode` | 0 停止，1 Iq/力矩，2 速度，3 位置，4 位置轨迹，5 旋钮 |
| 9 | uint8 | `speedCurrentFoc` | 速度模式内部诊断标志 |
| 10 | int32 | `speedTargetMdps` | 用户目标速度 |
| 14 | int32 | `speedReferenceMdps` | 加减速轨迹后的速度参考 |
| 18 | int32 | `speedActualMdps` | 实测速度 |
| 22 | int32 | `idMa` | d 轴电流 |
| 26 | int32 | `iqMa` | q 轴电流 |
| 30 | int32 | `iqTargetMa` | q 轴目标电流 |
| 34 | uint16 | `predictedElectricalRaw` | 预测电角度，0..4095 |
| 36 | uint32 | `encoderAgeMs` | 编码器样本年龄 |
| 40 | int16 | `predictionErrorRaw` | 预测误差 |
| 42 | uint32 | `timeoutRemainingMs` | 看门狗剩余时间 |

UI 安全逻辑建议：只要 `powerState=4`、`drvFaults!=0`、flags bit2=1、母线电压无效，或命令响应不是 OK，应立即停止周期运动命令并发送一次 STOP。

## 6. 用户可见参数与当前固件能力

需要明确区分两类参数：

### 6.1 UI 现在就能设置的运行参数

- 控制模式：停止、Iq、速度、位置、位置轨迹、力矩
- Iq 目标（mA）
- 力矩目标（mN·m）
- 速度目标（RPM 或 mdps）
- 绝对位置/相对圈数（UI 换算为绝对 mdeg）
- 位置轨迹最高速度、加速度、减速度
- 旋钮齿距、力度、阻尼和软限位（建议暂时隐藏为实验功能）

这些参数通过 Motor Service 设置，仅保存在 RAM 中，不写入 Flash。

### 6.2 用户安装电机时需要配置的持久化参数

Configuration Service 为 `0x04`，当前已实现以下命令：

| Command | 功能 | Payload/响应数据 |
|---:|---|---|
| `0x00` | 获取配置能力 | 响应：版本 `uint16`、配置长度 `uint16`、能力位 `uint32` |
| `0x01` | 读取当前生效配置 | 响应：41 字节用户配置 |
| `0x02` | 校验并写入 RAM | 请求：41 字节用户配置 |
| `0x03` | 请求保存 RAM 配置到 Flash | 空；立即应答，随后后台双槽交替保存 |
| `0x04` | 从 Flash 重新加载 | 空 |
| `0x05` | 将 RAM 恢复为编译期默认值 | 空；需再发 `0x03` 才持久化 |
| `0x06` | 获取保存状态 | 来源、dirty、A/B 有效性、序号、pending、lastSaveOk |

41 字节配置 Payload：

| 偏移 | 类型 | 参数 |
|---:|---|---|
| 0 | uint8 | 极对数 |
| 1 | float32 | Kt，N·m/A |
| 5 | float32 | 相电阻，Ω |
| 9 | float32 | 相电感，H |
| 13 | float32 | 磁链，Wb |
| 17 | int32 | 最大 Iq，mA |
| 21 | int32 | 最大速度，mdps |
| 25 | int32 | 最大累计绝对位置，mdeg |
| 29 | uint32 | 最低母线电压，mV |
| 33 | uint32 | 最高母线电压，mV |
| 37 | uint32 | 命令看门狗，ms |

`0x02–0x05` 只允许功率级处于 READY 状态。`SAVE` 返回 OK 表示请求已接受，不表示 Flash 已写完；上位机应每 250 ms 查询 `0x06`，直到 `pending=0`，再检查 `lastSaveOk=1`、`dirty=0`。

`0x06` 响应数据固定 10 字节：`source:uint8`、`dirty:uint8`、`slotAValid:uint8`、`slotBValid:uint8`、`sequence:uint32`、`savePending:uint8`、`lastSaveOk:uint8`。

当前默认值：

| 参数 | 当前值 | 用途 |
|---|---:|---|
| 极对数 | 11 | 机械角和电角度换算 |
| 力矩常数 Kt | 0.23 N·m/A | 力矩命令换算 |
| 相电阻 | 2.20 Ω | FOC/观测器模型 |
| 相电感 | 1.16 mH | FOC/观测器模型 |
| 磁链 | 0.023 Wb | 观测器和反电动势模型 |
| 最大用户 Iq | 300 mA | 电流/力矩命令限幅 |
| 最大速度 | 500 RPM | 速度命令限幅 |
| 最大累计位置 | ±10 圈 | 位置命令限幅 |
| 最低/最高母线电压 | 6/15 V | 功率级使能保护 |
| 命令看门狗 | 1000 ms | 通信失联保护 |

配置记录带 Magic、版本、长度、递增序号、CRC32 和最后写入的提交标记，分别存放在 STM32F405 Sector 6/7。上电选择 CRC 有效且序号最新的槽；两槽均无效时回退到编译期安全默认值。

## 7. 推荐的 UI 操作顺序

1. 打开 USB CDC 端口。
2. 发送 HELLO，确认协议版本为 1 且 capabilities 包含 Motor bit（bit4）。
3. 读取控制状态 `Service=0x05, Command=0x0F`。
4. 确认 `powerState=2`、`drvFaults=0`、母线电压有效。
5. 若编码器尚未对齐，显式提示用户并执行 `0x03`；不要自动执行会转动电机的校准。
6. 用户按下运行后，每 100–200 ms 重发当前目标命令。
7. 用户松开按钮、关闭窗口、切换模式或发生通信错误时，发送 STOP `0x02`。
8. 程序退出前再次发送 STOP，并关闭串口。

## 8. 固件 Flash 占用

当前 Debug 构建结果：

- 链接脚本给应用程序预留：`0x3FFFA = 262138 bytes`（约 256 KiB）
- 当前实际 Flash 使用：`122648 bytes`（约 119.77 KiB）
- 占预留空间：`46.79%`
- 剩余：`139490 bytes`（约 136.22 KiB）

当前固件没有超过新的应用 Flash 边界，余量充足。文件系统中的 `.elf` 包含调试符号和段信息，不等于烧入 MCU 的大小；判断 Flash 占用应看链接器统计或 `.bin`。
