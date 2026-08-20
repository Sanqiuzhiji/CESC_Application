# Application protocol

The normative wire-protocol specification is
[`../CESC_PROTOCOL_V1.md`](../CESC_PROTOCOL_V1.md).

The firmware implements CESC Protocol Version 1. The legacy CESC-compatible
prototype framing is intentionally not used for normal communication.

Application-layer protocol code is located in
`Application/Communication/Cesc/`, including the shared CRC16 primitive.
