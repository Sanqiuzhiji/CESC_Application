# Application code

This directory contains product-owned code. STM32CubeMX-generated code remains
in `Core` and `USB_DEVICE`; changes there should stay inside `USER CODE` blocks.

## Structure

```text
Application/
|-- App/                         Product lifecycle and module orchestration
|-- Communication/
|   `-- Vesc/                    Framing, CRC, and CESC command handling
|-- Devices/
|   `-- AngleSensor/             Angle-sensor facade and AS5600 driver
|-- Services/
|   `-- FirmwareUpdate/          Firmware staging and bootloader handoff
|-- PROTOCOL.md                  Wire-protocol documentation
`-- README.md
```

Headers live beside their implementation so a component can be read and moved
as one unit. Directory names describe product responsibilities rather than C
file types.

## Dependency direction

```text
CubeMX entry points -> App -> Communication / Devices
Communication/Vesc  -> Devices/AngleSensor / Services/FirmwareUpdate
Devices/AngleSensor -> STM32 HAL I2C
Services            -> STM32 HAL / USB device stack
```

Keep dependencies flowing in this direction. Low-level device code must not
call application orchestration or protocol code. New product behavior should
normally be introduced as a focused component and connected through `App`.

## Adding a component

1. Create a responsibility-based directory under the closest category.
2. Keep its public header and implementation together.
3. Add its source and include directory to the application target in the root
   `CMakeLists.txt`.
4. Expose only the operations needed by its callers; keep HAL details inside
   device or service implementations.
