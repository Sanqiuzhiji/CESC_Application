# CESC 上位机通信协议 v1

## 1. 分层

协议沿用 VESC 的成熟分层方式：

1. **传输层**：当前为 USB CDC，全双工字节流；以后可替换 UART、CAN
   网关或 BLE，而不改变上层帧和命令。
2. **帧层**：负责长度、CRC16、粘包/断包解析和错误后的重新同步。
3. **应用层**：VESC 兼容命令和 CESC 原生命令并存。
4. **服务层**：按 System、Sensor、Control、Configuration、Firmware
   等服务扩展。

所有多字节整数均使用**大端序**。协议中不发送 C `float`，物理量使用带
单位的定点整数，避免上下位机的浮点格式和精度差异。

## 2. 外层帧

短帧（Payload 长度 1～255）：

```
02 LEN8 PAYLOAD... CRC_H CRC_L 03
```

长帧（Payload 长度 256～1024）：

```
03 LEN_H LEN_L PAYLOAD... CRC_H CRC_L 03
```

- CRC：CRC16-CCITT，初值 `0x0000`，多项式 `0x1021`，只覆盖 Payload。
- 接收端遇到非法起始字节、长度、CRC 或结束字节时逐字节重新同步。
- 单个 Payload 最大 1024 字节。
- USB CDC 只是字节流边界，不能把一次 USB 收包当成一帧。

## 3. 应用层负载

VESC 兼容命令继续使用原有格式：Payload 第一个字节为 VESC 命令编号。
固件版本和固件升级命令 `0～3` 保持不变。

CESC 原生命令统一使用入口字节 `0xC8`：

### 请求

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 1 | Marker：`0xC8` |
| 1 | 1 | ProtocolVersion：当前为 `1` |
| 2 | 1 | MessageType：请求为 `0` |
| 3 | 2 | Sequence：上位机生成，大端序 |
| 5 | 1 | Service |
| 6 | 1 | Opcode |
| 7 | N | RequestData |

### 应答

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 1 | Marker：`0xC8` |
| 1 | 1 | ProtocolVersion：`1` |
| 2 | 1 | MessageType：应答为 `1` |
| 3 | 2 | Sequence：原样返回 |
| 5 | 1 | Service：原样返回 |
| 6 | 1 | Opcode：原样返回 |
| 7 | 1 | Status |
| 8 | N | ResponseData |

上位机同一连接内应使用递增 Sequence，并以
`Sequence + Service + Opcode` 匹配应答。当前 USB 实现采用请求—应答停止等待
模式：收到当前应答或确认超时前，不发送下一条请求。这样无需在 MCU 中为多个
最大长度应答分配发送队列；未来加入异步发送队列后可以开放多请求并行。

## 4. 通用状态码

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | OK | 成功 |
| 1 | BAD_LENGTH | 数据长度不正确 |
| 2 | BAD_ARGUMENT | 参数值无效 |
| 3 | UNSUPPORTED | 服务或操作不支持 |
| 4 | NOT_READY | 设备或数据尚未就绪 |
| 5 | BAD_VERSION | 协议版本不支持 |

CRC 错误和不完整帧不产生错误应答，因为此时无法可靠识别请求。上位机应设置
超时，普通查询建议 200 ms，超时后最多重试两次。写配置和固件升级命令必须
使用更长的命令专用超时，并保证重复请求安全。

## 5. 已实现服务

### System（Service `0x00`）

#### Ping（Opcode `0x00`）

请求无数据。成功应答数据：

| 字段 | 类型 | 单位 |
|---|---|---|
| Uptime | uint32 | ms |

#### GetInfo（Opcode `0x01`）

请求无数据。成功应答数据：

| 字段 | 类型 |
|---|---|
| FirmwareMajor | uint8 |
| FirmwareMinor | uint8 |
| FirmwarePatch | uint8 |
| NameLength | uint8 |
| Name | uint8[NameLength]，UTF-8，不带 `\0` |

### Sensor（Service `0x01`）

#### GetSample（Opcode `0x00`）

请求数据：

| 字段 | 类型 | 当前值 |
|---|---|---:|
| SensorId | uint8 | `0`：主轴角度传感器 |

成功应答数据：

| 字段 | 类型 | 说明 |
|---|---|---|
| SensorId | uint8 | `0` |
| SensorType | uint8 | `1`：AS5600 |
| SensorStatus | uint8 | 见 `angle_sensor_status_t` |
| Raw | uint16 | AS5600 原始值，0～4095 |
| Angle | uint32 | 千分之一度，`123456` 表示 123.456° |
| Timestamp | uint32 | 采样时的下位机运行时间，ms |

读取角度的 Payload 请求示例（Sequence 为 1）：

```
C8 01 00 00 01 01 00 00
```

外层短帧中的 LEN 为 `08`，CRC 由完整的上述 Payload 计算。

## 6. 上位机行为

1. 打开串口后发送 System/Ping，再发送 System/GetInfo，确认协议版本。
2. 需要显示角度时按固定周期主动发送 Sensor/GetSample，建议 20～50 Hz。
3. 校验帧 CRC、MessageType、Sequence、Service、Opcode 和 Status 后再使用数据。
4. UI 显示角度时使用 `Angle / 1000.0`；控制算法可直接使用整数。
5. 断线重连后清空接收缓存并重新执行握手，不复用旧的未完成请求。

## 7. 扩展规则

- 新功能优先增加 Service/Opcode，不占用更多 VESC 命令号。
- 已发布字段的含义和单位不得修改；需要不兼容变更时升级 ProtocolVersion。
- 新增应答字段只能追加在尾部，上位机必须按实际 Payload 长度解析。
- 周期遥测以后使用 MessageType `2`（Event），订阅操作应包含周期和字段掩码。
- 控制和配置服务必须增加参数范围检查、明确应答和失联安全策略。
