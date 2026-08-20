# CESC Application USB firmware update interface

This repository is the application project. It only receives VESC protocol
commands, stores the update package in the staging sectors, and transfers
control to the separately built bootloader. Firmware installation and copying
are not implemented in this project.

## Flash layout

| Region | Address | Purpose |
|---|---:|---|
| Sectors 0-7 | `0x08000000` - `0x0807FFFF` | Application |
| Sectors 8-10 | `0x08080000` - `0x080DFFFF` | Download staging area |
| Sector 11 | `0x080E0000` - `0x080FFFFF` | Reserved for the separate CESC_Bootloader project |

The application linker limits the generated image to 393210 bytes. The last
six bytes of the 384 KiB staging area are reserved because the VESC bootloader
format prepends a four-byte big-endian length and a two-byte CRC16.

## Build

Application:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Output: `build/Debug/CESC_Application.bin`

## First programming

Build the bootloader from the separate `D:\Desk\Folders\CESC_Bootloader`
project. Program these two images with SWD:

- `CESC_Application.bin` at `0x08000000`
- The separate project's `CESC_Bootloader.hex`, or its BIN at `0x080E0000`

Do not perform a full-chip erase after installing the bootloader. A full-chip
erase removes sector 11 and makes USB updates unable to install the downloaded
image.

## USB updates

The USB interface implements the legacy Benjamin/VESC framing and commands:

- `COMM_FW_VERSION` (`0`)
- `COMM_JUMP_TO_BOOTLOADER` (`1`)
- `COMM_ERASE_NEW_APP` (`2`)
- `COMM_WRITE_NEW_APP_DATA` (`3`)

Select `CESC_Application.bin` as a custom firmware image. Compression support
is owned by the separate CESC_Bootloader project.

During download PB0 indicates staging Flash activity. PB1 indicates an erase
or write error. LED polarity can be changed in
`Application/Services/FirmwareUpdate/firmware_update.c` if the board
uses active-low LEDs.
