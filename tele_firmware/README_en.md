# WL1 Remote-Control Firmware

[中文](README.md) | [English](README_en.md) | [Back to project README](../README_en.md)

`tele_firmware` is the handheld remote-control firmware for the WL1
wheeled-legged robot. It targets the STM32F411CEU6. The firmware samples four
joystick channels and two buttons, sends control commands every 50 ms to the
`car_firmware` in this repository through an nRF24L01+, and displays the current
commands on an SSD1306 OLED.

The current implementation prioritizes simplicity, deterministic behavior, and
debuggability:

- STM32 HAL + FreeRTOS;
- Fixed memory usage with no dynamic allocation in periodic tasks;
- DMA and interrupt synchronization for ADC, SPI, and UART;
- Four application tasks with clearly separated responsibilities;
- A 32-byte wireless protocol compatible with `car_firmware`.

For more detailed design information, see the
[architecture document](docs/architecture.md). For power-on or communication
issues, see [debugging and troubleshooting](docs/troubleshooting.md).

## Quick Start

### 1. Prepare the Tools

- CMake 3.22 or later;
- Ninja;
- Arm GNU Toolchain, providing `arm-none-eabi-gcc`, `g++`, and `objcopy`;
- OpenOCD for ST-Link flashing and debugging.

Verify that these commands are available in `PATH`, then run the following in
this directory:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

For an optimized build intended for normal operation:

```sh
cmake --preset Release
cmake --build --preset Release
```

Build outputs are placed in `build/Debug/` or `build/Release/`:

- `WL1_F411CEU6_Tele.elf`: debugging and flashing;
- `WL1_F411CEU6_Tele.hex`: Intel HEX image;
- `WL1_F411CEU6_Tele.bin`: raw binary image;
- `WL1_F411CEU6_Tele.map`: linker map and memory analysis.

### 2. Flash with ST-Link

Connect SWDIO, SWCLK, GND, and the target-board reference voltage, then run:

```sh
openocd -f STlink.cfg \
  -c "program build/Release/WL1_F411CEU6_Tele.elf verify reset exit"
```

After flashing, the OLED should display `WL1 REMOTE`. Moving the joysticks should
change the values and progress bars, and the PC13 activity LED should toggle as
wireless frames are sent.

## Controls

### Joysticks

| Control | ADC pin | Output range | Wireless meaning |
| --- | --- | ---: | --- |
| Roll | PA1 / ADC1_IN1 | -18.0° to 18.0° | Body roll target |
| Leg | PA2 / ADC1_IN2 | 44.5 to 78.5 mm | Leg-height target |
| Turn | PA3 / ADC1_IN3 | -100.0 to 100.0 | Steering target |
| Speed | PA5 / ADC1_IN5 | -100.0 to 100.0 | Velocity target |

All four joysticks use a calibration range of `0..4095`, a center value of
`2048`, and a center dead zone of `±300` ADC counts. Inputs outside the
calibrated range are clamped.

### Buttons

The buttons use internal pull-ups, so a low level means pressed.

| Button | Pin | Short press | Long press (at least 1 s) |
| --- | --- | --- | --- |
| Leg | PB0 | Lock/unlock the current leg height | Print the Buttons task's remaining stack through UART |
| Roll | PB1 | Lock/unlock the current roll angle | Print the Buttons task's remaining stack through UART |

When a value is locked, the joystick's live position continues to update, but
the corresponding target sent to the car remains unchanged. Another short press
immediately switches to the joystick's latest position. A long press prints only
diagnostic information; releasing it does not trigger a short press.

## Hardware Connections

### nRF24L01+

| nRF signal | MCU pin | Description |
| --- | --- | --- |
| SCK | PB13 | SPI2_SCK |
| MISO | PB14 | SPI2_MISO |
| MOSI | PB15 | SPI2_MOSI |
| CSN | PA8 | Software-controlled chip select, active low |
| CE | PB12 | TX/RX mode control |
| IRQ | PA12 | Falling-edge EXTI |
| VCC | 3.3 V | Do not connect to 5 V |
| GND | GND | Common ground with the MCU |

Place a decoupling capacitor near the nRF module's power pins. If wireless
communication fails, first confirm that the 3.3 V supply is stable and that the
module and MCU share a common ground.

### OLED

The SSD1306 128×64 OLED uses u8g2 software I²C:

| Signal | MCU pin | Parameters |
| --- | --- | --- |
| SCL | PB8 | Open-drain output |
| SDA | PB9 | Open-drain output |
| Address | — | 7-bit `0x3C` |

### Debug Interface

| Function | MCU pin | Parameters |
| --- | --- | --- |
| USART1 TX | PA9 / PA15 | Both output the same UART data, 115200, 8-N-1 |
| USART1 RX | PA10 | Serial commands through DMA + IDLE reception |
| Activity LED | PC13 | Toggles whenever a wireless transmission is submitted |

PA9 supports the common USB serial wiring; PA15 retains the original remote
board wiring. The PA9 mapping lives in the USER CODE section of
`Core/Src/usart.c` so CubeMX regeneration preserves it.

Connect the USB adapter's **TXD to PA10, RXD to PA9 (or PA15), and GND to GND**.
Use 115200, 8-N-1, with no flow control. With only TXD connected, commands may
execute even though the PC cannot receive their replies.

## Serial Commands and Feedback

The legacy `nrfsend <car command>` forwards a 1–32 byte ASCII payload, padded
with zeroes to 32 bytes. The prefix does not count toward that limit. Oversized
commands are rejected, never silently truncated. End commands with CR, LF, or
CRLF. For serial assistants without line endings, 50 ms of idle time also ends
a command; keep pauses between fragments shorter than 50 ms.
The queue holds eight commands and processes at most one every 50 ms; keep
sustained input at or below 20 commands per second.

| Command | Behavior |
| --- | --- |
| `nrfsend R 0 0 0 61.5` | Send zero turn, speed, and roll with a 61.5 mm leg height |
| `R 0 0 0 61.5` | Shorthand for the previous command |
| `nrfsend nrfshow -mr 0` | Ask the car to enable radio telemetry |
| `nrfsend nrfshow -nn` | Ask the car to disable radio telemetry |
| `joystick off` | Pause automatic joystick packets for manual serial control |
| `joystick on` | Resume 50 ms joystick packets; enabled by default after reset |
| `status` | Report ACK, retry, receive, driver, UART, and stack counters |
| `help` | Show serial command help |

For manual control, send `joystick off`, then `nrfsend R 0 0 0 61.5` on separate
lines. Pausing joystick packets does not stop the car: it retains its last
target. Serial `R` values follow the car's Turn, Speed, Roll, Leg order without
an additional speed sign inversion. Send `joystick on` after debugging.
The OLED continues to show joystick values and locks; inspect serial feedback
for the payload sent manually.

`uart: sending ...` reports submission. `nRF: send success [uart]: ...` is
printed only after `TX_DS` confirms an auto ACK. Joystick packets report
`nRF: send success [joystick]: ...`. Exhausted retries report
`nRF: send fail [uart]: no ACK; ...`, and car replies report `receive: ...`.
An ACK confirms radio delivery; command execution also depends on the car's
parser and operating state.

## Wireless Protocol

The remote control always sends a 32-byte payload. The valid content is
space-separated ASCII text:

```text
R <turn> <-speed> <roll_degrees> <leg_height_mm>
```

Example:

```text
R 0.0 -0.0 0.0 61.5
```

Each value has one decimal place, and the text is padded to 32 bytes with
`0x00`. The speed field is negated during encoding. This is the existing
coordinate-system convention between the remote control and the car and should
not be changed on only one side.

| Parameter | Current setting |
| --- | --- |
| Data rate | 2 Mbps |
| RF channel | 2 (2402 MHz) |
| Address width | 5 bytes |
| TX / Pipe 0 address | `11 52 01 31 41` |
| Payload width | 32 bytes |
| RF power | 0 dBm |
| Auto ACK | Enabled |
| Auto retry | 500 μs interval, up to 15 retries |
| Application period | 50 ms (20 Hz) |

The protocol format, field order, address, channel, data rate, and payload length
must be changed in sync with `car_firmware`.

## Software Structure

```text
tele_firmware/
├── Component/
│   ├── HardWare/
│   │   ├── OLED/                 # SSD1306/u8g2 board-level adaptation
│   │   └── Telecontrol/          # Joystick conversion and nRF24L01+ driver
│   ├── Peripheral/               # Buttons and asynchronous UART logging
│   ├── UserApp/                  # Tasks, state, protocol, and display logic
│   └── Module/include/etl/       # Embedded Template Library
├── Core/                         # STM32CubeMX-generated code
├── Drivers/                      # STM32 HAL / CMSIS
├── Middlewares/                  # FreeRTOS
├── ThirdParty/u8g2/              # OLED graphics library
├── cmake/                        # Toolchain and CubeMX CMake targets
├── CMakeLists.txt
├── CMakePresets.json
├── STlink.cfg
└── WL1_F411CEU6_Tele.ioc
```

The application entry point is `CPP_Main()` in `Component/UserApp/main.cpp`,
called by `MX_FREERTOS_Init()` before the scheduler starts.

## Development Guidelines

- Add handwritten code to CubeMX files under `Core/` only inside `USER CODE` sections;
- Explicitly add new application source files to the top-level `CMakeLists.txt`;
- Do not call regular FreeRTOS APIs from an ISR; use only the `...FromISR` variants;
- Interrupts that call FreeRTOS ISR APIs must not have an NVIC numerical priority higher than the currently permitted value of 5;
- Keep periodic tasks free of heap allocation, indefinite waits, and high-frequency UART output;
- When changing the protocol or RF parameters, check `car_firmware` at the same time;
- Before committing, complete at least Debug and Release builds and run `git diff --check`.

The current control frame and OLED values still use `snprintf` floating-point
formatting, so the linker configuration retains `-u _printf_float`. If the
encoding implementation is replaced later, validate both the wireless payload
and the OLED display.
