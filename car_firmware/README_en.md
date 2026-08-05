# WL1 Car Firmware

[中文](README.md) | [English](README_en.md) | [Back to project README](../README_en.md)

`car_firmware` is the vehicle-side firmware for the WL1 wheeled-legged robot. It
targets the STM32F411CEU6. The firmware reads the MPU6050 and the left and right
wheel encoders, runs cascaded PID control, drives two DC motors through a TB6612
and two leg servos, and receives remote-control commands over an nRF24L01+.

The current runtime path uses STM32 HAL, FreeRTOS, and C++23:

- A 10 ms attitude loop that calculates left and right wheel PWM;
- A 50 ms loop for speed, steering, roll, and leg-height control;
- Fixed 32-byte wireless commands over the nRF24L01+;
- USART1 DMA transmission and reception for online status monitoring and control-parameter updates;
- Fixed-capacity ETL containers for command queues and UART buffers.

Further reading:

- [Software architecture](docs/architecture.md): startup flow, tasks, control loops, and concurrency model;
- [Development conventions](docs/development.md): component boundaries, CubeMX ownership, and validation gates;
- [Command reference](docs/commands.md): serial/wireless commands, default parameters, and tuning sequence;
- [Debugging and troubleshooting](docs/troubleshooting.md): power-on checks, common faults, and CubeMX regeneration checks.

> [!WARNING]
> The wheeled-legged self-balancing controller drives the motor PWM immediately
> after its task starts. During initial flashing, control-direction changes, or
> PID tuning, raise the wheels off the ground, disconnect motor power, or use a
> current-limited power supply, and make sure power can be cut immediately.

## Quick Start

### 1. Prepare the Tools

- CMake 3.28 or later;
- Ninja, Make, or another CMake-supported build tool;
- Arm GNU Toolchain with the `arm-none-eabi-` command prefix;
- OpenOCD and an ST-Link for flashing and debugging;
- STM32CubeMX (optional), required only when changing `.ioc` peripheral settings.

Verify that the tools are available in `PATH`:

```sh
cmake --version
arm-none-eabi-gcc --version
openocd --version
```

### 2. Build

Run the following commands in the `car_firmware` directory:

```sh
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --parallel
```

For an optimized build intended to run on the robot:

```sh
cmake -S . -B build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release --parallel
```

If Ninja is unavailable, replace `-G Ninja` with an available generator, such as
`-G "MinGW Makefiles"` on Windows. Use a new build directory when changing
generators.

Build outputs are placed in the selected build directory:

- `WL1_F411CEU6.elf`: debugging and flashing;
- `WL1_F411CEU6.hex`: Intel HEX image;
- `WL1_F411CEU6.bin`: raw binary image;
- `WL1_F411CEU6.map`: linker map and memory analysis.

Build configurations:

| `CMAKE_BUILD_TYPE` | Compiler options | Purpose |
| --- | --- | --- |
| `Debug` or unspecified | `-Og -g` | Debugging, stepping, and variable inspection |
| `Release` | `-Ofast` | Normal operation |
| `RelWithDebInfo` | `-Ofast -g` | Optimized execution with debug information |
| `MinSizeRel` | `-Os` | Minimize image size |

The project always uses the Cortex-M4F hard-float ABI (`fpv4-sp-d16`), C11, and
C++23.

### 3. Flash with ST-Link

Connect SWDIO, SWCLK, GND, and the target-board reference voltage, then run:

```sh
openocd -f STlink.cfg \
  -c "program build/Release/WL1_F411CEU6.elf verify reset exit"
```

After flashing, USART1 should print `CPPMain: success`, followed by `MPU: success`
when MPU6050 initialization succeeds. The PC13 activity LED toggles approximately
every 100 ms.

## First Power-On

Use the following sequence to narrow down faults safely:

1. Disconnect motor power or raise the wheels off the ground, and power only the logic circuitry and ST-Link;
2. Confirm that USART1 receives the startup log;
3. Run `showimu -y` and verify that the attitude values remain stable while the robot is stationary;
4. Run `showrpm -y`, turn each wheel by hand, and confirm that both encoders respond;
5. Power the servos and use `legheight 44.5` to check the direction and mechanical limits on both sides;
6. Center and power on the remote control, then confirm that wireless commands update the target values;
7. Finally, connect motor power and check the feedback direction using a current-limited power supply and a stand.

For detailed checks, see [Debugging and troubleshooting](docs/troubleshooting.md).

## Hardware Connections

### Core and Debug Interfaces

| Function | MCU pin | Parameters |
| --- | --- | --- |
| USART1 TX | PA15 | 115200, 8-N-1, DMA2 Stream 7 |
| USART1 RX | PA10 | 115200, 8-N-1, DMA2 Stream 5, receive-to-idle |
| SWDIO | PA13 | ST-Link |
| SWCLK | PA14 | ST-Link |
| Activity LED | PC13 | Toggles every 100 ms |
| HSE | PH0/PH1 | 25 MHz |
| LSE | PC14/PC15 | 32.768 kHz |

PA15 is not the usual default USART1_TX pin. When connecting a serial adapter,
follow this table and `Core/Src/usart.c`.

### MPU6050

| Signal | MCU pin | Parameters |
| --- | --- | --- |
| SCL | PB6 | I2C1, 400 kHz |
| SDA | PB7 | I2C1, 400 kHz |
| Address | — | 7-bit `0x68` (the driver uses the left-shifted value `0xD0`) |

The current configuration uses a ±1000 °/s gyroscope range, a ±4 g accelerometer
range, and VQF for attitude fusion.

### Motors and Encoders

| Function | MCU pin | Peripheral |
| --- | --- | --- |
| Left-wheel PWM / TB6612 A | PA8 | TIM1_CH1, approximately 10 kHz |
| Right-wheel PWM / TB6612 B | PA9 | TIM1_CH2, approximately 10 kHz |
| AIN1 / AIN2 | PA6 / PA7 | Left-wheel direction |
| BIN1 / BIN2 | PB0 / PB1 | Right-wheel direction |
| Left encoder A / B | PA5 / PA1 | TIM2_CH1 / CH2 |
| Right encoder A / B | PB4 / PB5 | TIM3_CH1 / CH2 |

The current PID path inverts the TB6612 B-channel direction and configures a
minimum compare value of 50 counts for both PWM channels. The final PWM command
is limited to `-1000..1000`; a zero command keeps compare at 0 and does not apply
dead-zone compensation.

### Leg Servos

| Function | MCU pin | Peripheral |
| --- | --- | --- |
| Left-leg servo | PA2 | TIM9_CH1 |
| Right-leg servo | PA3 | TIM9_CH2 |

TIM9 generates 100 Hz PWM. The software limits the target leg height to
`44.5..78.5 mm` and converts it to a servo angle using four-bar inverse
kinematics. Mechanical dimensions and coordinate-system definitions are in
`Component/UserApp/CtrlAlgorithm/LegKinematics.hpp`.

### nRF24L01+

| nRF signal | MCU pin | Description |
| --- | --- | --- |
| SCK | PB13 | SPI2_SCK |
| MISO | PB14 | SPI2_MISO |
| MOSI | PB15 | SPI2_MOSI |
| CSN | PA4 | Software-controlled chip select, active low |
| CE | PB12 | TX/RX mode control |
| IRQ | PA12 | Falling-edge EXTI |
| VCC | 3.3 V | Do not connect to 5 V |
| GND | GND | Common ground with the MCU |

The SPI2 clock is approximately 6.25 Mbit/s, and both transmission and reception
use DMA. Place a decoupling capacitor near the nRF module's power pins. If the
wireless link behaves abnormally, first check the 3.3 V supply and common ground.

### Reserved OLED Interface

The driver in `Component/HardWare/OLED` is compiled, but the current application
does not initialize or refresh the OLED. CubeMX reserves these software-I²C pins:

| Signal | MCU pin |
| --- | --- |
| O_SCL | PB10 |
| O_SDA | PB3 |

## Wireless Protocol

The car enters nRF receive mode after power-on. A valid payload is an ASCII
command padded to 32 bytes with `0x00`. The recommended control frame is:

```text
R <turn_target> <velocity_target> <roll_degrees> <leg_height_mm>
```

Example:

```text
R 0.0 -0.0 0.0 61.5
```

The field order must remain steering, velocity, roll, and leg height. The
`tele_firmware` in this repository negates the remote-control velocity before
placing it in the second field. This is the existing coordinate-system
convention between the two devices.

| Parameter | Current setting |
| --- | --- |
| Data rate | 2 Mbps |
| RF channel | 2 (2402 MHz) |
| Address width | 5 bytes |
| RX Pipe 0 / TX address | `11 52 01 31 41` |
| Payload width | 32 bytes |
| RF power | 0 dBm |
| CRC | 1 byte |
| Auto ACK | Enabled |
| Auto retry | 500 μs interval, up to 15 retries |

The RF parameters and `R` command format must be changed in sync with
`tele_firmware`. For all available commands and their limits, see the
[command reference](docs/commands.md).

## Software Structure

```text
car_firmware/
├── Component/
│   ├── App/                       # Module lifecycle, registry, and rollback
│   ├── AppModules/                # Communication, servo, and control tasks
│   ├── HardWare/
│   │   ├── IMU/                  # MPU6050 and VQF
│   │   ├── Motor/                # Encoders, TB6612, and servos
│   │   ├── OLED/                 # OLED driver, currently unused
│   │   └── Telecontrol/          # nRF24L01+ driver
│   ├── Module/
│   │   ├── Basic/                # TaskReactor and SafeQueue
│   │   └── include/etl/          # Embedded Template Library
│   ├── Peripheral/               # USART DMA wrapper
│   └── UserApp/
│       ├── CtrlAlgorithm/        # PID, LQR, and leg kinematics
│       └── main.cpp              # Application module composition only
├── cmake/                        # Firmware target and toolchain policy
├── Core/                         # STM32CubeMX-generated code
├── Drivers/                      # STM32 HAL / CMSIS
├── Middlewares/                  # FreeRTOS
├── tests/                        # Host tests that need no development board
├── CMakeLists.txt
├── CMakeLists_template.txt
├── STlink.cfg
└── WL1_F411CEU6.ioc
```

The C entry point is `Core/Src/main.c`. Before the scheduler starts,
`MX_FREERTOS_Init()` calls `CPP_Main()`. A fixed-capacity `AppManager` registers and
starts the communication, servo, and motion-control modules in order. If any task
creation fails, already-created tasks are rolled back in reverse order and startup
reports `CPPMain: fail`.

PID is the active control path. The LQR path remains available for experiments but
does not currently run. `MainControl.*` and the OLED module are likewise not
connected to the application path.

## Development Guidelines

- Add handwritten code to CubeMX files under `Core/` only inside `USER CODE` sections;
- `CMakeLists.txt` is marked as a template-generated file, so persistent changes should also be applied to `CMakeLists_template.txt`;
- Explicitly list new component sources in that component's `CMakeLists.txt`, then rerun CMake;
- Do not call regular FreeRTOS APIs from an ISR; use only the `...FromISR` variants;
- Interrupts that call FreeRTOS ISR APIs must have an NVIC numerical priority of 5 or greater;
- Avoid dynamic allocation, blocking I/O, and high-frequency UART output in control tasks;
- After changing wheel direction, encoder direction, or attitude signs, raise the wheels off the ground and verify the closed-loop feedback direction;
- When changing the wireless protocol, update `tele_firmware` and the documentation for both sides;
- Before committing, complete Debug and Release builds, host tests, and `git diff --check`.

Run the host tests with:

```sh
cmake -S tests -B build/host-tests -G "MinGW Makefiles"
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```
