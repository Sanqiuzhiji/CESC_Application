# CESC Protocol Version 1

Status: **Normative specification**  
Protocol version: **1**  
Last updated: **2026-08-20**

## 1. Scope

This document defines the binary application protocol between CESC firmware and
CESC Tool. It is transport-independent and is intended for USB CDC serial,
hardware UART, TCP, and other reliable byte streams.

The protocol covers:

- connection negotiation and device identification;
- request/response transactions;
- firmware update;
- sensor access;
- telemetry discovery and streaming;
- diagnostics and future configuration and motor-control services.

The CESC Protocol is independent of user protocols created with CESC Tool's
Protocol Editor. Protocol Editor workspaces are not used to describe or modify
this protocol.

The key words **MUST**, **MUST NOT**, **REQUIRED**, **SHOULD**, **SHOULD NOT**,
and **MAY** are to be interpreted as normative requirements.

## 2. Conventions

Unless explicitly stated otherwise:

- every multi-byte integer is little-endian;
- signed integers use two's-complement representation;
- IEEE-754 binary32 and binary64 are used for floating-point values;
- strings are UTF-8 without a terminating null byte;
- a string is encoded as `uint8 length` followed by `length` bytes;
- lengths are measured in bytes;
- reserved fields MUST be transmitted as zero and ignored when received;
- enum values not defined by this version MUST be treated as unsupported;
- receivers MUST validate lengths before reading fields.

The host is the CESC Tool side. The device is the CESC firmware side.

## 3. Transport requirements

### 3.1 Byte-stream behavior

A transport MAY split one frame across multiple reads or combine multiple
frames in one read. A decoder MUST therefore support:

- partial frames;
- multiple consecutive frames;
- arbitrary garbage before a valid frame;
- recovery after a corrupted frame;
- a magic sequence split across reads.

One transport write is not guaranteed to equal one protocol frame on receive.

### 3.2 Serial defaults

For a hardware UART, the default settings are:

```text
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
```

USB CDC baud-rate settings are informational and MUST NOT change protocol
behavior.

### 3.3 Connection states

Opening a transport does not establish a CESC session. The host state machine
is:

```text
Disconnected -> TransportOpen -> Negotiating -> Ready
                                      |           |
                                      +-> Error <-+
```

The session enters `Ready` only after a successful `SYSTEM/HELLO` exchange.
Application requests other than `SYSTEM/HELLO` SHOULD NOT be sent before that.

## 4. Frame format

### 4.1 Layout

```text
Offset  Size  Field
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

Total frame length is `12 + PayloadLength` bytes.

| Field | Encoding | Meaning |
|---|---|---|
| Magic | `43 45` | ASCII `CE`, used for synchronization |
| Version | `uint8` | Must be `1` for this specification |
| MessageType | `uint8` | Request, response, event, or stream |
| ServiceId | `uint8` | Target service |
| CommandId | `uint8` | Command within the service |
| Sequence | `uint16` | Transaction or stream sequence number |
| PayloadLength | `uint16` | Number of Payload bytes |
| Payload | bytes | Type-specific content |
| CRC16 | `uint16` | CRC16-CCITT over Version through Payload |

There is no trailing delimiter. Magic, validated length, and CRC provide stream
re-synchronization.

### 4.2 Message types

```c
enum CescMessageType {
    CESC_MESSAGE_REQUEST  = 0x00,
    CESC_MESSAGE_RESPONSE = 0x01,
    CESC_MESSAGE_EVENT    = 0x02,
    CESC_MESSAGE_STREAM   = 0x03
};
```

- A `REQUEST` is sent by the host and expects one `RESPONSE`.
- A `RESPONSE` MUST copy ServiceId, CommandId, and Sequence from its request.
- An `EVENT` is an unsolicited, low-rate device notification.
- A `STREAM` is unsolicited telemetry and uses a device-generated Sequence.

Version 1 does not define host-originated events or streams.

### 4.3 Payload limits

The absolute Version 1 maximum PayloadLength is 4096 bytes. A device MAY
advertise a smaller maximum in `SYSTEM/HELLO`; both endpoints MUST use the
smaller negotiated value.

The initial firmware implementation SHOULD advertise 1024 bytes.

A decoder MUST reject a frame whose PayloadLength exceeds its configured limit
without allocating memory based on that untrusted length.

### 4.4 CRC16

CRC parameters:

```text
Name:       CRC-16/CCITT with zero initialization
Polynomial: 0x1021
Initial:    0x0000
RefIn:      false
RefOut:     false
XorOut:     0x0000
Check:      CRC("123456789") = 0x31C3
```

The covered bytes begin at Version, offset 2, and end at the last Payload byte.
Magic and the CRC field itself are not covered. The resulting CRC is appended
little-endian.

### 4.5 Decoder recovery

A decoder MUST use the following behavior:

1. Search for `43 45`.
2. Wait until the fixed ten-byte header is available.
3. Validate Version, MessageType, and PayloadLength.
4. Wait until the entire candidate frame is available.
5. Validate CRC.
6. On success, consume the entire frame.
7. On invalid header or CRC, discard only the first magic byte and search
   again. It MUST NOT discard the entire claimed frame.

An incomplete candidate SHOULD be abandoned if no additional byte arrives for
1000 ms. Transport disconnect always clears decoder state.

## 5. Transactions and responses

### 5.1 Sequence allocation

The host MUST allocate a nonzero Sequence for every request. It SHOULD increase
the value modulo 65536 and skip zero. A Sequence MUST NOT be reused while a
request with that Sequence is outstanding.

Sequence zero is reserved for messages that are intentionally not correlated
with a request.

### 5.2 Response status

Every response Payload begins with a little-endian `uint16 Status`. Command
response fields, if any, follow it.

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

Malformed frames that fail header or CRC validation do not receive a response,
because their routing and Sequence fields cannot be trusted. A syntactically
valid request with an invalid command Payload MUST receive an error response.

### 5.3 Timeout and retry

Default host timeouts are:

| Operation | Timeout |
|---|---:|
| HELLO, PING, information query | 1000 ms |
| Sensor/configuration request | 1000 ms |
| Firmware BEGIN/erase | 30000 ms |
| Firmware WRITE | 3000 ms |
| Firmware FINISH/verification | 10000 ms |
| RESET/ACTIVATE | No response required after acknowledged delay |

The host MAY retry a timed-out request up to two times. A retry MUST use the
same Sequence and identical Payload.

The device SHOULD cache the most recent response for each active transport for
at least two seconds. If it receives an identical repeated request with the
same Sequence, it SHOULD resend the cached response rather than repeat side
effects. Firmware commands additionally obey the idempotency rules in section
8.

Only one firmware operation may be outstanding. Other services MAY support
multiple outstanding requests, but the initial host implementation SHOULD
limit itself to eight.

## 6. Service identifiers

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

Configuration and Motor IDs are reserved in Version 1. Implementations MUST
return `NOT_SUPPORTED` until their command formats are specified.

## 7. System service (`0x00`)

### 7.1 `HELLO` (`0x00`)

This is the first request after opening a transport.

Request Payload:

```text
uint8  minimumVersion
uint8  maximumVersion
uint32 hostCapabilities
```

Version 1 defines no host capability bits; `hostCapabilities` is zero.

Successful response after Status:

```text
uint8  selectedVersion
uint16 maximumPayloadLength
uint64 deviceCapabilities
uint32 sessionId
```

`sessionId` is generated on boot and changes after reset. The host MUST discard
outstanding transactions and stream state if it observes a different session
ID.

If there is no common version, the device responds using the received frame
version when possible, with `VERSION_MISMATCH` and no additional fields.

Device capability bits:

```text
bit 0  Firmware update
bit 1  Sensor service
bit 2  Telemetry streaming
bit 3  Configuration service
bit 4  Motor service
bit 5  Diagnostic/log events
bits 6..63 reserved
```

### 7.2 `GET_DEVICE_INFO` (`0x01`)

Request Payload: empty.

Successful response after Status:

```text
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

An unavailable version is encoded as three zero values. `buildIdentifier`
SHOULD contain a short Git commit or reproducible build ID.

### 7.3 `PING` (`0x02`)

Request Payload:

```text
uint32 token
```

Successful response after Status:

```text
uint32 token
uint32 deviceUptimeMs
```

The token is opaque and MUST be returned unchanged.

### 7.4 `GET_CAPABILITIES` (`0x03`)

Request Payload: empty.

Successful response after Status:

```text
uint64 deviceCapabilities
uint16 maximumPayloadLength
uint8  maximumOutstandingRequests
uint8  reserved
```

### 7.5 `GET_COMM_STATS` (`0x04`)

Request Payload: empty.

Successful response after Status:

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

Counters wrap naturally modulo 2^32 and reset on device reboot.

### 7.6 `RESET` (`0x05`)

Request Payload:

```text
uint8  resetMode
uint16 delayMs
```

Reset modes:

```text
0 Normal application reset
1 Enter bootloader
```

The device MUST send an `OK` response before resetting and SHOULD clamp
`delayMs` to 50..5000 ms so the response can leave the transport.

## 8. Firmware service (`0x01`)

### 8.1 General rules

Only one update session may exist. While updating, the device SHOULD reject
new telemetry subscriptions and pause active streams. Queries such as PING and
GET_STATUS remain available.

Firmware image integrity uses CRC-32/ISO-HDLC:

```text
Polynomial: 0x04C11DB7
Initial:    0xFFFFFFFF
RefIn:      true
RefOut:     true
XorOut:     0xFFFFFFFF
Check:      CRC32("123456789") = 0xCBF43926
```

### 8.2 `BEGIN` (`0x00`)

Request Payload:

```text
uint32 imageSize
uint32 imageCrc32
uint16 requestedChunkSize
uint16 flags
```

Version 1 flags are zero.

Successful response after Status:

```text
uint32 updateSessionId
uint16 acceptedChunkSize
uint32 nextExpectedOffset
```

The device validates available storage before acknowledging. Chunk size MUST
be nonzero and small enough that a complete WRITE request fits the negotiated
maximum PayloadLength.

Repeating an identical BEGIN while the same update is active returns the same
session ID and current `nextExpectedOffset`. A different BEGIN while busy
returns `BUSY`.

### 8.3 `WRITE` (`0x01`)

Request Payload:

```text
uint32 updateSessionId
uint32 offset
uint16 dataLength
uint8  data[dataLength]
```

`dataLength` MUST equal the remaining Payload bytes and MUST NOT exceed the
accepted chunk size.

Successful response after Status:

```text
uint32 nextExpectedOffset
```

Rules:

- If `offset == nextExpectedOffset`, write and advance.
- If the entire block is below `nextExpectedOffset`, verify that stored data
  matches, then return the current offset without writing again.
- Gaps and conflicting repeated data return `INVALID_ARGUMENT` or
  `VERIFY_FAILED`.
- A successful retry is therefore idempotent.

### 8.4 `FINISH` (`0x02`)

Request Payload:

```text
uint32 updateSessionId
```

The device verifies image size, CRC32, and any platform-specific image header.

Successful response after Status:

```text
uint32 verifiedSize
uint32 calculatedCrc32
```

The image MUST NOT be marked bootable before FINISH succeeds.

### 8.5 `ACTIVATE` (`0x03`)

Request Payload:

```text
uint32 updateSessionId
uint16 delayMs
```

The device returns `OK`, marks the verified image for activation, waits long
enough to transmit the response, and resets. The host then reconnects and
performs HELLO and GET_DEVICE_INFO. Transport disappearance after the `OK`
response is expected behavior.

### 8.6 `ABORT` (`0x04`)

Request Payload:

```text
uint32 updateSessionId
```

The device cancels the active update. It MAY retain staged bytes, but they MUST
not be considered bootable.

### 8.7 `GET_STATUS` (`0x05`)

Request Payload: empty.

Successful response after Status:

```text
uint8  state
uint8  lastError
uint16 acceptedChunkSize
uint32 updateSessionId
uint32 imageSize
uint32 nextExpectedOffset
uint32 expectedCrc32
```

States:

```text
0 Idle
1 Erasing
2 Receiving
3 Verifying
4 ReadyToActivate
5 Failed
```

## 9. Sensor service (`0x02`)

Sensor IDs are stable numeric identifiers assigned by firmware. Version 1
reserves sensor ID zero for the shaft-angle sensor.

### 9.1 `ENUMERATE` (`0x00`)

Request Payload: empty.

Successful response after Status:

```text
uint8 sensorCount
repeated sensorCount times:
    uint8 sensorId
    uint8 sensorType
    uint8 sensorCapabilities
    string name
```

Sensor types:

```text
0 Unknown
1 AS5600
2 Encoder
3 Hall sensors
4 IMU
```

### 9.2 `GET_SAMPLE` (`0x01`)

Request Payload:

```text
uint8 sensorId
```

For the shaft-angle sensor, successful response after Status:

```text
uint8  sensorId
uint8  sensorType
uint8  sensorStatus
uint8  reserved
uint16 rawAngle
float32 angleDegrees
uint64 timestampUs
```

Sensor status values:

```text
0 Uninitialized
1 OK
2 NotFound
3 NoMagnet
4 MagnetWeak
5 MagnetStrong
6 IoError
```

`timestampUs` is the device monotonic timestamp at sampling time.

### 9.3 `GET_STATUS` (`0x02`)

Request Payload:

```text
uint8 sensorId
```

Successful response after Status:

```text
uint8  sensorId
uint8  sensorStatus
uint16 reserved
uint32 sampleAgeUs
uint32 ioErrorCount
```

## 10. Telemetry service (`0x03`)

### 10.1 Data types

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

Values in a stream use the type and ordering returned by channel enumeration.

### 10.2 `ENUM_CHANNELS` (`0x00`)

Request Payload:

```text
uint16 firstChannelId
uint8  maximumCount
```

Successful response after Status:

```text
uint16 totalChannelCount
uint8  returnedCount
repeated returnedCount times:
    uint16 channelId
    uint8  dataType
    float32 scale
    float32 offset
    string name
    string unit
```

Enumeration is paged so the response remains within the negotiated Payload
limit. Channel IDs MUST remain stable for a given hardware and protocol major
version. Physical value is `raw * scale + offset`.

### 10.3 `SUBSCRIBE` (`0x01`)

Request Payload:

```text
uint32 requestedPeriodUs
uint8  samplesPerFrame
uint8  channelCount
uint16 channelIds[channelCount]
```

Successful response after Status:

```text
uint16 streamId
uint32 actualPeriodUs
uint8  actualSamplesPerFrame
uint8  channelCount
uint16 channelIds[channelCount]
```

The device MAY increase the period or reduce the batch size to meet bandwidth
and CPU limits. A stream ID is scoped to the current device session.

### 10.4 `UNSUBSCRIBE` (`0x02`)

Request Payload:

```text
uint16 streamId
```

Successful response contains Status only.

### 10.5 `STOP_ALL` (`0x03`)

Request Payload: empty. Successful response contains Status only.

### 10.6 `GET_STREAM_STATUS` (`0x04`)

Request Payload:

```text
uint16 streamId
```

Successful response after Status:

```text
uint16 streamId
uint32 producedFrames
uint32 droppedFrames
uint32 producedSamples
```

### 10.7 Stream data (`CommandId 0x80`)

The device sends this as `MessageType = STREAM`.

Payload:

```text
uint16 streamId
uint32 sampleSequence
uint64 firstTimestampUs
uint32 samplePeriodUs
uint8  sampleCount
uint8  channelCount
repeated sampleCount times:
    values for each subscribed channel, in negotiated channel order
```

`Sequence` in the frame header increments for every frame in this stream.
`sampleSequence` increments for every sample and identifies the first sample in
the frame. These counters allow the host to detect both frame and sample loss.

When `samplePeriodUs` is nonzero, sample `i` has timestamp:

```text
firstTimestampUs + i * samplePeriodUs
```

Stream delivery is best-effort. When bandwidth is exhausted, firmware SHOULD
drop telemetry rather than block control, request responses, or safety tasks.

## 11. Diagnostic service (`0x06`)

### 11.1 `SET_LOG_LEVEL` (`0x00`)

Request Payload:

```text
uint8 minimumLevel
```

Levels are Debug=0, Info=1, Warning=2, Error=3, Off=255.

### 11.2 Log event (`CommandId 0x80`)

The device sends this as `MessageType = EVENT`.

```text
uint8  level
uint16 code
uint64 timestampUs
string message
```

Logs MUST NOT be inserted as unframed ASCII into a CESC protocol stream.

## 12. Flow control and prioritization

Device transmit priority, highest first:

1. safety and fatal diagnostic events;
2. request responses;
3. firmware acknowledgements;
4. ordinary events;
5. telemetry streams;
6. debug logs.

If a transmit queue is full, lower-priority telemetry and logs MAY be dropped.
Responses MUST NOT be silently dropped; the device SHOULD reserve queue space
for at least one maximum-sized response.

The host SHOULD limit command rate based on outstanding transactions. During a
firmware update it MUST stop telemetry before BEGIN and MUST NOT send unrelated
state-changing commands.

## 13. Security and safety

Version 1 provides integrity detection, not authentication or encryption.
CRC values do not protect against malicious modification.

Implementations MUST:

- validate every length and range before accessing memory or Flash;
- never execute a command from a frame with invalid CRC;
- reject writes outside the firmware staging area;
- verify a complete firmware image before activation;
- apply safe limits independently of host requests;
- define a control-command timeout before the Motor service is enabled;
- stop dangerous outputs when the transport or control session is lost.

Network transports used outside a trusted network require a future authenticated
protocol layer and are not secured by this specification.

## 14. Compatibility rules

- The Version field governs frame and common semantic compatibility.
- New commands MAY be added within an existing service.
- New fields MAY only be appended to a response when a capability bit or a new
  command/version explicitly indicates their presence.
- Existing field order, size, meaning, and byte order MUST NOT change within
  Version 1.
- Unknown ServiceId returns `INVALID_SERVICE`.
- Unknown CommandId in a known service returns `INVALID_COMMAND`.
- Unknown events and streams MUST be ignored after frame validation.
- Reserved values MUST NOT be repurposed incompatibly.

## 15. Required test vectors

### 15.1 Empty PING request

This vector is structurally valid even though PING normally requires a token;
the receiver should decode it and respond `INVALID_LENGTH`.

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

Complete frame:
43 45 01 00 00 02 01 00 00 00 E4 75
```

### 15.2 PING request with token `0x12345678`

```text
Sequence:      0x1234
Payload:       78 56 34 12
CRC input:     01 00 00 02 34 12 04 00 78 56 34 12
CRC value:     0x0D60
CRC bytes:     60 0D

Complete frame:
43 45 01 00 00 02 34 12 04 00 78 56 34 12 60 0D
```

Both implementations MUST reproduce these exact bytes.

## 16. Minimum Version 1 implementation milestone

Before motor-control development begins, the firmware and CESC Tool SHOULD
jointly pass the following milestone:

1. Incremental frame codec passes shared vectors and corruption tests.
2. HELLO establishes a verified CESC session.
3. Device information and PING work after reconnect.
4. Communication statistics expose CRC and overflow errors.
5. Firmware BEGIN/WRITE/FINISH/ACTIVATE supports retry and final CRC32.
6. AS5600 can be queried through the Sensor service.
7. Angle telemetry can be subscribed and plotted using device timestamps.
8. No unframed ASCII is emitted on the CESC protocol transport.
9. USB unplug/replug and malformed input recover without reboot.
10. A one-hour telemetry stress test reports no parser deadlock or response
    starvation.

## Appendix A. Assigned command summary

| Service | Command | ID | Message type |
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
| Diagnostic | SET_LOG_LEVEL | `0x00` | Request/Response |
| Diagnostic | LOG | `0x80` | Event |

## Appendix B. Migration from the prototype protocol

The prototype firmware and CESC Tool use a CESC-compatible outer frame and
prototype command IDs 0 through 3 for firmware update. That format is not CESC
Protocol Version 1.

Migration SHOULD be atomic across the two repositories:

1. implement and test the Version 1 codec on both sides;
2. implement HELLO and System commands;
3. migrate firmware update to the Firmware service;
4. migrate sensor queries and telemetry;
5. remove periodic `samples:` ASCII output;
6. remove the legacy packet decoder after the new updater is verified.

Temporary dual-protocol firmware MAY recognize both magic formats during
migration, but CESC Tool MUST select exactly one session protocol and MUST NOT
send both formats concurrently.
