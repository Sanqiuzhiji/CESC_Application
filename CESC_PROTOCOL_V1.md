# CESC 协议版本 1

状态：**规范性规范**
协议版本：**1**
最后更新：**2026-08-20**

## 1. 范围

本文档定义 CESC 固件和 CESC Tool 之间的二进制应用协议。它与传输方式无关，适用于 USB CDC 串口、硬件 UART、TCP 和其他可靠字节流。

协议包括：连接协商和设备识别、请求/响应事务、固件升级、传感器访问、遥测发现与流传输、诊断，以及未来的配置和电机控制服务。

CESC 协议独立于 CESC Tool 协议编辑器创建的用户协议；协议编辑器工作区不可用于描述或修改本协议。

关键字 **MUST**、**MUST NOT**、**REQUIRED**、**SHOULD**、**SHOULD NOT** 和 **MAY** 按规范性要求解释。

## 2. 约定

除非明确说明：

- 多字节整数均为小端序；有符号整数采用二进制补码；浮点数采用 IEEE-754 binary32/binary64。
- 字符串为无结尾空字符的 UTF-8，编码为 `uint8 length` 后接 `length` 字节；长度以字节计。
- 保留字段发送时 MUST 为零、接收时 MUST 忽略；未定义枚举值 MUST 视为不支持。
- 接收方 MUST 在读取字段前验证长度。

主机是 CESC Tool 一侧，设备是 CESC 固件一侧。

## 3. 传输要求

### 3.1 字节流行为

传输层 MAY 将一个帧拆分到多次读取，或将多个帧合并到一次读取。因此解码器 MUST 支持：不完整帧、连续帧、有效帧前的任意垃圾数据、损坏帧后的恢复，以及跨读取边界的魔数。一次传输写入不保证在接收端等于一个协议帧。

### 3.2 串口默认值

硬件 UART 的默认设置：

```text
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
```

USB CDC 波特率设置仅供参考，MUST NOT 改变协议行为。

### 3.3 连接状态

打开传输层不等于建立 CESC 会话。主机状态机：

```text
Disconnected -> TransportOpen -> Negotiating -> Ready
                                      |           |
                                      +-> Error <-+
```

仅在 `SYSTEM/HELLO` 成功交换后会话才进入 `Ready`。此前除 `SYSTEM/HELLO` 之外的应用请求 SHOULD NOT 发送。

## 4. 帧格式

### 4.1 布局

```text
偏移量  大小  字段
0       2     Magic
2       1     Version
3       1     MessageType
4       1     ServiceId
5       1     CommandId
6       2     Sequence
8       2     PayloadLength
10      N     Payload
10+N    2     CRC16
```

帧总长为 `12 + PayloadLength` 字节。

| 字段 | 编码 | 含义 |
|---|---|---|
| Magic | `43 45` | ASCII `CE`，用于同步 |
| Version | `uint8` | 本规范必须为 `1` |
| MessageType | `uint8` | 请求、响应、事件或流 |
| ServiceId | `uint8` | 目标服务 |
| CommandId | `uint8` | 服务内命令 |
| Sequence | `uint16` | 事务或流序列号 |
| PayloadLength | `uint16` | Payload 的字节数 |
| Payload | bytes | 类型特定内容 |
| CRC16 | `uint16` | 覆盖 Version 到 Payload 的 CRC16-CCITT |

没有尾部分隔符。魔数、经验证的长度和 CRC 用于流重新同步。

### 4.2 消息类型

```c
enum CescMessageType {
    CESC_MESSAGE_REQUEST  = 0x00,
    CESC_MESSAGE_RESPONSE = 0x01,
    CESC_MESSAGE_EVENT    = 0x02,
    CESC_MESSAGE_STREAM   = 0x03
};
```

- `REQUEST` 由主机发送，并期待一个 `RESPONSE`。
- `RESPONSE` MUST 从请求复制 ServiceId、CommandId 和 Sequence。
- `EVENT` 是设备主动发起的低频通知。
- `STREAM` 是设备主动发送的遥测数据，使用设备生成的 Sequence。

版本 1 未定义主机发起的事件或流。

### 4.3 载荷限制

版本 1 的 PayloadLength 绝对上限为 4096 字节。设备 MAY 在 `SYSTEM/HELLO` 中声明更小值；双方 MUST 使用协商后的较小值。初始固件实现 SHOULD 声明 1024 字节。

解码器 MUST 拒绝 PayloadLength 超过配置上限的帧，且不得按该不可信长度分配内存。

### 4.4 CRC16

```text
名称：       使用零初始化的 CRC-16/CCITT
多项式：     0x1021
初始值：     0x0000
RefIn：      false
RefOut：     false
XorOut：     0x0000
校验值：     CRC("123456789") = 0x31C3
```

覆盖范围从偏移量 2 的 Version 至最后一个 Payload 字节；不覆盖 Magic 和 CRC 字段。结果以小端序附加。

### 4.5 解码器恢复

解码器 MUST：

1. 搜索 `43 45`。
2. 等待固定十字节头部可用。
3. 验证 Version、MessageType、PayloadLength。
4. 等待完整候选帧可用。
5. 验证 CRC。
6. 成功时消费整个帧。
7. 头部或 CRC 无效时，仅丢弃第一个魔数字节并重新搜索；MUST NOT 丢弃整个声称的帧。

若候选帧在 1000 ms 内未收到更多字节，SHOULD 放弃该不完整候选帧。传输断开时始终清除解码器状态。

## 5. 事务与响应

### 5.1 Sequence 分配

主机 MUST 为每个请求分配非零 Sequence。其 SHOULD 按模 65536 递增并跳过零。某 Sequence 仍有未完成请求时 MUST NOT 重用。序列号零保留给有意不与请求关联的消息。

### 5.2 响应状态

每个响应 Payload 以小端 `uint16 Status` 开始，随后是命令响应字段（如有）。

```c
enum CescStatus {
    CESC_STATUS_OK               = 0,
    CESC_STATUS_INVALID_SERVICE  = 1,
    CESC_STATUS_INVALID_COMMAND  = 2,
    CESC_STATUS_INVALID_LENGTH   = 3,
    CESC_STATUS_INVALID_ARGUMENT = 4,
    CESC_STATUS_NOT_READY        = 5,
    CESC_STATUS_BUSY             = 6,
    CESC_STATUS_TIMEOUT          = 7,
    CESC_STATUS_CRC_ERROR        = 8,
    CESC_STATUS_IO_ERROR         = 9,
    CESC_STATUS_NOT_SUPPORTED    = 10,
    CESC_STATUS_VERSION_MISMATCH = 11,
    CESC_STATUS_INTERNAL_ERROR   = 12,
    CESC_STATUS_ACCESS_DENIED    = 13,
    CESC_STATUS_OUT_OF_RANGE     = 14,
    CESC_STATUS_VERIFY_FAILED    = 15
};
```

未通过头部或 CRC 验证的畸形帧不接收响应，因为其路由和 Sequence 不可信。语法有效但命令 Payload 无效的请求 MUST 收到错误响应。

### 5.3 超时与重试

| 操作 | 主机默认超时 |
|---|---:|
| HELLO、PING、信息查询 | 1000 ms |
| 传感器/配置请求 | 1000 ms |
| 固件 BEGIN/擦除 | 30000 ms |
| 固件 WRITE | 3000 ms |
| 固件 FINISH/验证 | 10000 ms |
| RESET/ACTIVATE | 确认延迟后无需响应 |

主机 MAY 对超时请求最多重试两次；重试 MUST 使用相同 Sequence 和完全相同的 Payload。

设备 SHOULD 为每个活动传输缓存最近响应至少两秒。若收到相同 Sequence 的完全相同重复请求，SHOULD 重发缓存响应，而非重复副作用。固件命令还遵守第 8 节幂等规则。

同一时间只能有一个固件操作未完成。其他服务 MAY 支持多个未完成请求，但初始主机实现 SHOULD 限制为八个。

## 6. 服务标识符

```c
enum CescServiceId {
    CESC_SERVICE_SYSTEM        = 0x00,
    CESC_SERVICE_FIRMWARE      = 0x01,
    CESC_SERVICE_SENSOR        = 0x02,
    CESC_SERVICE_TELEMETRY     = 0x03,
    CESC_SERVICE_CONFIGURATION = 0x04,
    CESC_SERVICE_MOTOR         = 0x05,
    CESC_SERVICE_DIAGNOSTIC    = 0x06,
    CESC_SERVICE_DEVELOPMENT   = 0x7F
};
```

Configuration 和 Motor ID 在版本 1 中保留。命令格式规定前，实现 MUST 返回 `NOT_SUPPORTED`。

## 7. 系统服务（`0x00`）

### 7.1 `HELLO`（`0x00`）

打开传输后的第一个请求。

```text
请求 Payload：
uint8  minimumVersion
uint8  maximumVersion
uint32 hostCapabilities
```

版本 1 未定义主机能力位，`hostCapabilities` 为零。

```text
成功响应中 Status 后的字段：
uint8  selectedVersion
uint16 maximumPayloadLength
uint64 deviceCapabilities
uint32 sessionId
```

`sessionId` 启动时生成、复位后改变。主机发现不同会话 ID 时 MUST 丢弃未完成事务和流状态。若无共同版本，设备在可能情况下以接收帧版本返回 `VERSION_MISMATCH`，且不含附加字段。

```text
设备能力位：
bit 0  固件升级
bit 1  传感器服务
bit 2  遥测流
bit 3  配置服务
bit 4  电机服务
bit 5  诊断/日志事件
bits 6..63 保留
```

### 7.2 `GET_DEVICE_INFO`（`0x01`）

请求 Payload：空。

```text
成功响应中 Status 后的字段：
uint16 firmwareMajor
uint16 firmwareMinor
uint16 firmwarePatch
uint16 bootloaderMajor
uint16 bootloaderMinor
uint16 bootloaderPatch
uint8  uuid[12]
string hardwareName
string buildIdentifier
```

不可用版本编码为三个零值。`buildIdentifier` SHOULD 包含简短 Git 提交 ID 或可复现构建 ID。

### 7.3 `PING`（`0x02`）

```text
请求 Payload：
uint32 token

成功响应中 Status 后的字段：
uint32 token
uint32 deviceUptimeMs
```

token 不透明，MUST 原样返回。

### 7.4 `GET_CAPABILITIES`（`0x03`）

请求 Payload：空。

```text
uint64 deviceCapabilities
uint16 maximumPayloadLength
uint8  maximumOutstandingRequests
uint8  reserved
```

### 7.5 `GET_COMM_STATS`（`0x04`）

请求 Payload：空。

```text
uint32 receivedBytes
uint32 transmittedBytes
uint32 validFrames
uint32 crcErrors
uint32 lengthErrors
uint32 discardedBytes
uint32 receiveOverflows
uint32 transmitOverflows
uint32 unsupportedRequests
```

计数器按模 2^32 自然回绕，设备重启时复位。

### 7.6 `RESET`（`0x05`）

```text
请求 Payload：
uint8  resetMode
uint16 delayMs

复位模式：
0 正常应用程序复位
1 进入引导加载程序
```

设备 MUST 在复位前发送 `OK`，并 SHOULD 将 `delayMs` 限制至 50..5000 ms，确保响应可以发出。

## 8. 固件服务（`0x01`）

### 8.1 通用规则

只能存在一个升级会话。升级时设备 SHOULD 拒绝新遥测订阅并暂停活动流；PING、GET_STATUS 等查询保持可用。

固件镜像完整性使用 CRC-32/ISO-HDLC：

```text
多项式：     0x04C11DB7
初始值：     0xFFFFFFFF
RefIn：      true
RefOut：     true
XorOut：     0xFFFFFFFF
校验值：     CRC32("123456789") = 0xCBF43926
```

### 8.2 `BEGIN`（`0x00`）

```text
请求 Payload：
uint32 imageSize
uint32 imageCrc32
uint16 requestedChunkSize
uint16 flags

成功响应中 Status 后的字段：
uint32 updateSessionId
uint16 acceptedChunkSize
uint32 nextExpectedOffset
```

版本 1 的 flags 为零。设备在确认前验证可用存储。Chunk size MUST 非零，且完整 WRITE 请求必须可放入协商的最大 PayloadLength。

当同一升级活动时，重复完全相同的 BEGIN 返回相同会话 ID 和当前 `nextExpectedOffset`；忙碌时不同 BEGIN 返回 `BUSY`。

### 8.3 `WRITE`（`0x01`）

```text
请求 Payload：
uint32 updateSessionId
uint32 offset
uint16 dataLength
uint8  data[dataLength]

成功响应中 Status 后的字段：
uint32 nextExpectedOffset
```

`dataLength` MUST 等于剩余 Payload 字节数，且 MUST NOT 超过已接受的块大小。

- 若 `offset == nextExpectedOffset`，写入并推进偏移量。
- 若整个数据块位于 `nextExpectedOffset` 之前，验证已存数据一致，然后不重复写入而返回当前偏移量。
- 有间隙或内容冲突的重复数据返回 `INVALID_ARGUMENT` 或 `VERIFY_FAILED`。
- 因而成功重试是幂等的。

### 8.4 `FINISH`（`0x02`）

```text
请求 Payload：
uint32 updateSessionId

成功响应中 Status 后的字段：
uint32 verifiedSize
uint32 calculatedCrc32
```

设备验证镜像大小、CRC32 和平台特定镜像头。FINISH 成功前镜像 MUST NOT 标记为可启动。

### 8.5 `ACTIVATE`（`0x03`）

```text
uint32 updateSessionId
uint16 delayMs
```

设备返回 `OK`，标记已验证镜像待激活，等待响应发送后复位。主机随后重新连接并执行 HELLO 和 GET_DEVICE_INFO。`OK` 后传输消失是预期行为。

### 8.6 `ABORT`（`0x04`）

请求 Payload：`uint32 updateSessionId`。设备取消当前升级。它 MAY 保留暂存字节，但这些字节 MUST NOT 被视为可启动。

### 8.7 `GET_STATUS`（`0x05`）

请求 Payload：空。

```text
uint8  state
uint8  lastError
uint16 acceptedChunkSize
uint32 updateSessionId
uint32 imageSize
uint32 nextExpectedOffset
uint32 expectedCrc32

状态：0 Idle；1 Erasing；2 Receiving；3 Verifying；4 ReadyToActivate；5 Failed
```

## 9. 传感器服务（`0x02`）

传感器 ID 是由固件分配的稳定数值标识符。版本 1 保留传感器 ID 零用于轴角传感器。

### 9.1 `ENUMERATE`（`0x00`）

请求 Payload：空。

```text
uint8 sensorCount
重复 sensorCount 次：
    uint8 sensorId
    uint8 sensorType
    uint8 sensorCapabilities
    string name
```

传感器类型：0 Unknown、1 AS5600、2 Encoder、3 Hall sensors、4 IMU。

### 9.2 `GET_SAMPLE`（`0x01`）

请求 Payload：`uint8 sensorId`。

对于轴角传感器，成功响应中 Status 后的字段：

```text
uint8  sensorId
uint8  sensorType
uint8  sensorStatus
uint8  reserved
uint16 rawAngle
float32 angleDegrees
uint64 timestampUs
```

传感器状态：0 Uninitialized、1 OK、2 NotFound、3 NoMagnet、4 MagnetWeak、5 MagnetStrong、6 IoError。`timestampUs` 为设备采样时刻的单调时间戳。

### 9.3 `GET_STATUS`（`0x02`）

请求 Payload：`uint8 sensorId`。

```text
uint8  sensorId
uint8  sensorStatus
uint16 reserved
uint32 sampleAgeUs
uint32 ioErrorCount
```

## 10. 遥测服务（`0x03`）

### 10.1 数据类型

```c
enum CescDataType {
    CESC_DATA_UINT8   = 0,
    CESC_DATA_INT8    = 1,
    CESC_DATA_UINT16  = 2,
    CESC_DATA_INT16   = 3,
    CESC_DATA_UINT32  = 4,
    CESC_DATA_INT32   = 5,
    CESC_DATA_UINT64  = 6,
    CESC_DATA_INT64   = 7,
    CESC_DATA_FLOAT32 = 8,
    CESC_DATA_FLOAT64 = 9
};
```

流内数值采用通道枚举返回的类型和顺序。

### 10.2 `ENUM_CHANNELS`（`0x00`）

```text
请求 Payload：
uint16 firstChannelId
uint8  maximumCount

成功响应中 Status 后的字段：
uint16 totalChannelCount
uint8  returnedCount
重复 returnedCount 次：
    uint16 channelId
    uint8  dataType
    float32 scale
    float32 offset
    string name
    string unit
```

枚举分页进行，以使响应符合协商的 Payload 限制。指定硬件和协议主版本的通道 ID MUST 稳定。物理值为 `raw * scale + offset`。

### 10.3 `SUBSCRIBE`（`0x01`）

```text
请求 Payload：
uint32 requestedPeriodUs
uint8  samplesPerFrame
uint8  channelCount
uint16 channelIds[channelCount]

成功响应中 Status 后的字段：
uint16 streamId
uint32 actualPeriodUs
uint8  actualSamplesPerFrame
uint8  channelCount
uint16 channelIds[channelCount]
```

设备 MAY 增大周期或降低批量，以满足带宽和 CPU 限制。流 ID 的作用域为当前设备会话。

### 10.4 `UNSUBSCRIBE`（`0x02`）

请求 Payload：`uint16 streamId`。成功响应仅含 Status。

### 10.5 `STOP_ALL`（`0x03`）

请求 Payload：空。成功响应仅含 Status。

### 10.6 `GET_STREAM_STATUS`（`0x04`）

请求 Payload：`uint16 streamId`。

```text
uint16 streamId
uint32 producedFrames
uint32 droppedFrames
uint32 producedSamples
```

### 10.7 流数据（`CommandId 0x80`）

设备以 `MessageType = STREAM` 发送。

```text
uint16 streamId
uint32 sampleSequence
uint64 firstTimestampUs
uint32 samplePeriodUs
uint8  sampleCount
uint8  channelCount
重复 sampleCount 次：
    每个已订阅通道的值，按协商后的通道顺序
```

帧头 `Sequence` 在此流的每个帧递增。`sampleSequence` 在每个样本递增并标识帧中第一个样本；二者可使主机检测帧和样本丢失。若 `samplePeriodUs` 非零，样本 `i` 的时间戳为：

```text
firstTimestampUs + i * samplePeriodUs
```

流传输为尽力而为。带宽耗尽时，固件 SHOULD 丢弃遥测数据，而不得阻塞控制、请求响应或安全任务。

## 11. 电机服务（`0x05`）

### 11.1 `GET_POWER_STAGE_STATUS`（`0x00`）

请求 Payload：空。该命令只读，不会使能功率桥。成功响应数据为：

```text
uint8  state              // 0=未初始化, 1=校准中, 2=就绪, 3=运行, 4=故障
uint8  flags              // bit0 EN_GATE, bit1 六路PWM, bit2 nFAULT有效, bit3母线电压有效
uint16 drvFaults
uint16 busVoltageRaw
uint32 busVoltageMv
uint32 currentSequence
重复三相 A、B、C：
    uint16 currentRaw
    uint16 currentOffset
    int32  currentCentered
uint8  testState          // 0=空闲, 1=运行, 2=完成, 3=中止
uint8  testStepsCompleted // 本次标定测试已完成的换相步数
```

`currentCentered = currentRaw - currentOffset`，单位为 ADC count。电流放大器增益确认前不得将其解释为安培。

功率级基础诊断字段之后保留现有 commissioning 统计字段。响应偏移 204 起追加相电阻辨识数据（所有多字节字段均为小端）：

```text
uint8  resistancePhase       // 0空闲, 1校零, 2待使能, 3..10测量阶段, 11完成
uint8  resistanceValid       // 1表示正反向结果均通过有效性检查
uint32 forwardSamples
uint32 reverseSamples
int32  idMa                  // 实时FOC内部Id，mA
int32  iqMa                  // 实时FOC内部Iq，mA
int32  vdMv                  // 实时FOC内部Vd，mV
int32  vqMv                  // 实时FOC内部Vq，mV
int32  liveResistanceMilliOhms
int32  forwardResistanceMilliOhms
int32  reverseResistanceMilliOhms
int32  averageResistanceMilliOhms
```

### 11.4 `MEASURE_RESISTANCE`（`0x06`）

请求 Payload：空。命令仅在功率级 READY、无锁存故障且母线电压为 6–10 V 时启动。辨识流程保持功率输出关闭并重新平均 1024 次 ADC offset，随后固定电角度，令 `Iq_target=0`，依次注入 `+0.5 A Id` 和 `-0.5 A Id`。每个方向包含 500 ms 斜坡、800 ms 稳定等待和 1000 ms 多点采样；方向切换前回零 300 ms，结束时斜坡降流 500 ms。

固件直接累计FOC电流环内部的 `Id` 和限幅后的 `Vd`，不从PWM duty反算电压：

```text
Rs_forward = average(Vd_forward) / average(Id_forward)
Rs_reverse = average(Vd_reverse) / average(Id_reverse)
Rs          = (Rs_forward + Rs_reverse) / 2
```

相电阻辨识单独允许最大50%电压矢量。辨识电流暂定为0.4 A；更高电流在当前ADC/PWM采样结构下表现出调制度相关偏差，并接近20-count软件保护的瞬态边界。与参考VESC提交db6ba047一致，只执行一次正向锁定电流测量；所得电压电流比乘以2/3，输出星形等效FOC相电阻。兼容字段reverseSamples和reverseMilliohms返回0。该限制不改变其他commissioning与FOC控制路径的调制度上限；ADC软件过流、DRV8301 nFAULT和母线电压限制始终有效。

### Phase Inductance 与 Flux Linkage 辨识扩展

`MOTOR_MEASURE_INDUCTANCE (0x07)` 无请求载荷，要求当前会话已经完成且保留有效的相电阻结果。实现移植自参考VESC提交db6ba047：TIM1临时切换到3 kHz测量时基，从2% duty开始按1.5倍递增，直至三相短脉冲平均电流达到约0.5 A；随后依次对三组相线施加200轮短脉冲，在脉冲末端同步采样对应相电流。使用 `L=V*t/I`、50 timer-count上升时间补偿及2/3 FOC相参数归一化。完成、超时或故障后恢复20 kHz控制时基。兼容字段forward保存VESC三相平均结果，reverse字段返回0。

`MOTOR_MEASURE_FLUX (0x08)` 请求载荷为一个 `int8` 方向（`+1` 或 `-1`），同样要求有效 Rs。模块使用 AS5600 实际机械位置产生受限的 60 rpm 旋转电压矢量，仅接收 300–420 degree/s、方向一致且传感器新鲜的稳态样本。三路 AD8418 使用 inline full-Clarke 电流变换。原始调制矢量电压按参考VESC的FOC相电压约定乘以2/3，稳态 PMSM 关系为：

`lambda = (2/3) * (Vq - Rs * Iq) / omega_e`

`Ke_mechanical = pole_pairs * lambda`

`KV = 60 / (2*pi*pole_pairs*lambda)`

建议依次执行 Rs、L，再对 `0x08` 执行正转和反转，将两次 `lambda` 平均后重新计算。协议中的 Ke/KV 是由该FOC相磁链派生的内部量，不等同于铭牌常见的线间RMS定义。

功率级状态响应在相电感字段（偏移 246–275）之后追加：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 276 | uint8 | flux_valid |
| 277 | uint32 | flux_samples |
| 281 | int32 | mechanical_speed_millidegrees_per_second |
| 285 | int32 | Iq_mA |
| 289 | int32 | Vq_mV |
| 293 | uint32 | flux_linkage_uWb |
| 297 | uint32 | mechanical_Ke_uV_per_rad_s |
| 301 | uint32 | KV_milliRPM_per_V |

正式电流控制字段追加在偏移 305：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 305 | uint8 | control_mode：0关闭，1 Iq，2速度，3位置，4限速位置轨迹，5旋钮触觉 |
| 306 | int32 | 实时 Id，mA |
| 310 | int32 | 实时 Iq，mA |
| 314 | int32 | Iq 目标，mA |
| 318 | uint32 | 控制命令看门狗剩余时间，ms |
| 322 | int32 | 速度目标，millidegree/s |
| 326 | int32 | PLL估算机械速度，millidegree/s |
| 330 | int32 | 多圈机械位置目标，millidegree |
| 334 | int32 | 多圈机械位置反馈，millidegree |

### 11.5 `SET_IQ_CURRENT`（`0x09`）

请求 Payload 为小端 `int32 iqTargetMa`。这是参考 VESC
`CONTROL_MODE_CURRENT` 的第一版正式有感 FOC 电流模式：`Id_target=0`，正负
`Iq` 决定正反力矩。当前安全范围为 `-300..+300 mA`；绝对值小于 10 mA
（包括 0）立即关闭 PWM 和 EN_GATE，释放电机。

非零命令要求功率级 READY、母线 6–10 V、无锁存故障、没有正在运行的
commissioning 测试，并且 AS5600 电角度零位已经校准且样本新鲜。进入模式后，
主机必须以短于 500 ms 的周期重复发送目标值；超时、编码器数据失效、功率级
状态异常或保护触发都会由固件独立关断。`STOP (0x02)` 同样立即退出该模式。

### 11.6 `SET_SPEED`（`0x0A`）

请求 Payload 为小端 `int32 speedTargetMillidegreesPerSecond`。第一版正式有感
速度模式采用分段有感控制。当前范围限制为
`-3000000..+3000000 millidegree/s`（±500 RPM），默认加速度限制为
600 degree/s²。低速直接电压段单独限制为 20 degree/s²，避免给电压路径施加
中高速所需的激进斜坡。低于 20 RPM 使用编码器位置轨迹误差直接产生受限 `Vq`；达到
20 RPM 后切换到 VESC 式电气 RPM 速度 PI，PI 输出经过 1 A/s 斜坡后成为
`Iq` 目标，再由电流 PI 和六扇区 SVPWM 执行。编码器 PLL 只在新 AS5600 样本
到达时校正。AS5600 以 500 Hz 读取，并在 20 kHz 电流环中预测电角度。速度降至
15 RPM 以下时切回
低速路径，形成迟滞以防止反复切换。中高速路径使用 ±50 mA 方向性前馈克服
起动齿槽。
绝对目标小于 50 millidegree/s 时释放电机。非零命令的准入条件和 500 ms
看门狗与 `SET_IQ_CURRENT` 相同。

### 11.7 `SET_POSITION`（`0x0B`）

请求 Payload 为小端 `int32 positionTargetMillidegrees`，范围为±3600000
millidegree（±10圈），零点与 Sensor 服务的连续 `positionCounts` 一致。控制采用
VESC式结构：位置P环直接生成 `Iq` 目标，再由电流PI执行。当前比例增益为
0.04 A/degree，误差绝对值超过0.5 degree时叠加0.02 A静摩擦补偿，`Iq`硬限制为
±0.15 A；编码器检查和500 ms命令看门狗保持不变。位置命令不会因目标为0而释放电机，必须使用
`STOP (0x02)`退出保持。

### 11.8 `SET_POSITION_PROFILE`（`0x0C`）

推荐请求 Payload 为四个小端 `int32`：绝对多圈位置目标
`positionTargetMillidegrees`、正数速度上限 `maximumSpeedMillidegreesPerSecond`、
加速度 `accelerationMillidegreesPerSecondSquared` 和减速度
`decelerationMillidegreesPerSecondSquared`。位置范围为 ±3600000 millidegree（±10圈），
速度范围为 50..90000 millidegree/s，加减速度范围均为 100..90000
millidegree/s²。固件生成带加速和提前减速的梯形/三角形位置轨迹，再由编码器位置误差
直接产生受限 `Vq` 电压矢量，因此能够按指定速度转到指定圈数或角度。

为兼容旧上位机，两个 `int32` 的 8 字节 Payload 仍然接受，此时加速度和减速度均使用
10000 millidegree/s²。新实现 SHOULD 使用 16 字节 Payload。
命令必须在 500 ms 看门狗期限内刷新，到达目标后继续保持位置，使用 `STOP (0x02)`释放。

### 11.9 `SET_HAPTIC`（`0x0D`）

请求 Payload 为五个小端 `int32`：卡点间隔（millidegree）、卡点强度（mA）、
阻尼系数（mA/(degree/s)）、最小位置和最大位置（millidegree）。允许范围分别为
100..360000、0..300、0..100，位置边界必须递增且位于 ±3600000 内。
固件输出最近卡点的回正力矩、与转速成比例的阻尼以及位置边界的虚拟限位力矩，
总 `Iq` 硬限制为 ±300 mA。该命令同样需要每 500 ms 内刷新并使用 `STOP` 退出。

### 11.10 `SET_TORQUE`（`0x0E`）

请求 Payload 为小端 `int32 torqueTargetMillinewtonMetres`，范围为 -69..+69 mN·m。
固件按厂家参数 `Kt = 0.23 N·m/A` 换算为 `Iq`，并复用带 500 ms 看门狗的电流环；
零力矩释放输出。实际轴力矩还会受到厂家力矩常数误差、温升和摩擦影响，因此这是估算力矩，
若需要计量级力矩必须用力矩传感器标定。

### 11.2 `START_COMMISSIONING_TEST`（`0x01`）

请求 Payload：`int8 direction`，只接受 `+1` 或 `-1`。这是受限调试命令：仅允许母线 6–10 V、功率级 READY、无锁存故障时启动；当前实现固定约 ±8.3% 调制度、500 ms 对齐、精确 30 个换相步和 200 ms 末端稳定保持，并设有 3 s 独立总超时。占空比另受 ±10% 绝对上限约束，并独立执行 ADC 过流与 nFAULT 关断。

### 11.3 `STOP`（`0x02`）

请求 Payload：空。立即关闭六路 PWM 和 EN_GATE。主机应在异常、超时或关闭连接时尽力发送此命令；固件自身的 3 s 超时不依赖该请求。

## 12. 诊断服务（`0x06`）

### 12.1 `SET_LOG_LEVEL`（`0x00`）

请求 Payload：`uint8 minimumLevel`。级别：Debug=0、Info=1、Warning=2、Error=3、Off=255。

### 12.2 日志事件（`CommandId 0x80`）

设备以 `MessageType = EVENT` 发送：

```text
uint8  level
uint16 code
uint64 timestampUs
string message
```

MUST NOT 将未封帧 ASCII 日志插入 CESC 协议流。

## 13. 流量控制与优先级

设备发送优先级（由高到低）：

1. 安全和致命诊断事件；
2. 请求响应；
3. 固件确认；
4. 普通事件；
5. 遥测流；
6. 调试日志。

发送队列满时，低优先级遥测和日志 MAY 被丢弃。响应 MUST NOT 被静默丢弃；设备 SHOULD 预留至少一个最大尺寸响应的队列空间。

主机 SHOULD 基于未完成事务限制命令速率。固件升级期间，它 MUST 在 BEGIN 前停止遥测，且 MUST NOT 发送无关的状态变更命令。

## 14. 安全与功能安全

版本 1 提供完整性检测，不提供认证或加密。CRC 无法抵御恶意篡改。

实现 MUST：

- 访问内存或 Flash 前验证每个长度和范围；
- 绝不执行 CRC 无效帧的命令；
- 拒绝固件暂存区以外的写入；
- 激活前验证完整固件镜像；
- 独立于主机请求应用安全限制；
- 启用 Motor 服务前定义控制命令超时；
- 传输或控制会话丢失时停止危险输出。

在可信网络以外使用的网络传输需要未来的认证协议层；本规范不提供安全保护。

## 15. 兼容性规则

- Version 字段决定帧及公共语义兼容性。
- MAY 在已有服务中增加新命令。
- 仅当能力位或新命令/版本明确表示其存在时，MAY 在响应后追加新字段。
- 版本 1 内，已有字段顺序、大小、含义和字节序 MUST NOT 改变。
- 未知 ServiceId 返回 `INVALID_SERVICE`；已知服务中的未知 CommandId 返回 `INVALID_COMMAND`。
- 帧验证后，未知事件和流 MUST 忽略；保留值 MUST NOT 以不兼容方式重新利用。

## 16. 必需测试向量

### 15.1 空 PING 请求

该向量结构有效，尽管 PING 通常需要 token；接收方应解码并响应 `INVALID_LENGTH`。

```text
Magic:         43 45
Version:       01
MessageType:   00
ServiceId:     00
CommandId:     02
Sequence:      01 00
PayloadLength: 00 00
CRC input:     01 00 00 02 01 00 00 00
CRC value:     0x75E4
CRC bytes:     E4 75

完整帧：
43 45 01 00 00 02 01 00 00 00 E4 75
```

### 15.2 带 token `0x12345678` 的 PING 请求

```text
Sequence:      0x1234
Payload:       78 56 34 12
CRC input:     01 00 00 02 34 12 04 00 78 56 34 12
CRC value:     0x0D60
CRC bytes:     60 0D

完整帧：
43 45 01 00 00 02 34 12 04 00 78 56 34 12 60 0D
```

两个实现 MUST 都能复现这些精确字节。

## 17. 版本 1 最小实现里程碑

在开始电机控制开发前，固件和 CESC Tool SHOULD 共同达到以下里程碑：

1. 增量帧编解码器通过共享向量和损坏数据测试。
2. HELLO 建立已验证的 CESC 会话。
3. 重新连接后设备信息和 PING 可用。
4. 通信统计暴露 CRC 和溢出错误。
5. 固件 BEGIN/WRITE/FINISH/ACTIVATE 支持重试和最终 CRC32。
6. 可经 Sensor 服务查询 AS5600。
7. 可订阅并利用设备时间戳绘制角度遥测。
8. CESC 协议传输中不输出未封帧 ASCII。
9. USB 拔插和畸形输入可无需重启地恢复。
10. 一小时遥测压力测试无解析器死锁或响应饥饿。

## 附录 A. 已分配命令摘要

| 服务 | 命令 | ID | 消息类型 |
|---|---|---:|---|
| System | HELLO | `0x00` | Request/Response |
| System | GET_DEVICE_INFO | `0x01` | Request/Response |
| System | PING | `0x02` | Request/Response |
| System | GET_CAPABILITIES | `0x03` | Request/Response |
| System | GET_COMM_STATS | `0x04` | Request/Response |
| System | RESET | `0x05` | Request/Response |
| Firmware | BEGIN | `0x00` | Request/Response |
| Firmware | WRITE | `0x01` | Request/Response |
| Firmware | FINISH | `0x02` | Request/Response |
| Firmware | ACTIVATE | `0x03` | Request/Response |
| Firmware | ABORT | `0x04` | Request/Response |
| Firmware | GET_STATUS | `0x05` | Request/Response |
| Sensor | ENUMERATE | `0x00` | Request/Response |
| Sensor | GET_SAMPLE | `0x01` | Request/Response |
| Sensor | GET_STATUS | `0x02` | Request/Response |
| Telemetry | ENUM_CHANNELS | `0x00` | Request/Response |
| Telemetry | SUBSCRIBE | `0x01` | Request/Response |
| Telemetry | UNSUBSCRIBE | `0x02` | Request/Response |
| Telemetry | STOP_ALL | `0x03` | Request/Response |
| Telemetry | GET_STREAM_STATUS | `0x04` | Request/Response |
| Telemetry | STREAM_DATA | `0x80` | Stream |
| Motor | GET_POWER_STAGE_STATUS | `0x00` | Request/Response |
| Motor | START_COMMISSIONING_TEST | `0x01` | Request/Response |
| Motor | STOP | `0x02` | Request/Response |
| Motor | CALIBRATE_ENCODER_ZERO | `0x03` | Request/Response |
| Motor | START_ENCODER_VOLTAGE_TEST | `0x04` | Request/Response |
| Motor | START_CURRENT_FOC_TEST | `0x05` | Request/Response |
| Motor | MEASURE_PHASE_RESISTANCE | `0x06` | Request/Response |
| Motor | MEASURE_PHASE_INDUCTANCE | `0x07` | Request/Response |
| Motor | MEASURE_FLUX_LINKAGE | `0x08` | Request/Response |
| Motor | SET_IQ_CURRENT | `0x09` | Request/Response |
| Motor | SET_SPEED | `0x0A` | Request/Response |
| Motor | SET_POSITION | `0x0B` | Request/Response |
| Motor | SET_POSITION_PROFILE | `0x0C` | Request/Response |
| Motor | SET_HAPTIC | `0x0D` | Request/Response |
| Motor | SET_TORQUE | `0x0E` | Request/Response |
| Motor | MEASURE_RESISTANCE | `0x06` | Request/Response |
| Diagnostic | SET_LOG_LEVEL | `0x00` | Request/Response |
| Diagnostic | LOG | `0x80` | Event |

## 附录 B. 从原型协议迁移

原型固件和 CESC Tool 使用 CESC 兼容外层帧以及固件升级的原型命令 ID 0 至 3；该格式不是 CESC 协议版本 1。

迁移 SHOULD 在两个仓库中原子地进行：

1. 两端实现并测试版本 1 编解码器；
2. 实现 HELLO 和 System 命令；
3. 将固件升级迁移到 Firmware 服务；
4. 迁移传感器查询和遥测；
5. 移除周期性 `samples:` ASCII 输出；
6. 验证新升级器后移除旧数据包解码器。

临时双协议固件 MAY 在迁移期间识别两种魔数格式，但 CESC Tool MUST 精确选择一种会话协议，且 MUST NOT 并发发送两种格式。
