# WL1 遥控器固件

[中文](README.md) | [English](README_en.md) | [返回项目主页](../README.md)

`tele_firmware` 是 WL1 轮腿小车的手持遥控器固件，目标芯片为
STM32F411CEU6。固件采集四路摇杆和两个按键，通过 nRF24L01+ 每 50 ms
向同一仓库下的 `car_firmware` 发送控制命令，并在 SSD1306 OLED 上显示当前指令。

当前实现以简单、确定和可调试为原则：

- STM32 HAL + FreeRTOS；
- 固定内存，不在周期任务中动态分配；
- ADC、SPI、UART 使用 DMA 与中断同步；
- 四个职责单一的应用任务；
- 保持与 `car_firmware` 兼容的 32 字节无线协议。

更详细的设计说明见 [架构说明](docs/architecture.md)，上电或通信异常见
[调试与故障排查](docs/troubleshooting.md)。

## 快速开始

### 1. 准备工具

- CMake 3.22 或更新版本；
- Ninja；
- Arm GNU Toolchain，提供 `arm-none-eabi-gcc`、`g++`、`objcopy`；
- OpenOCD，用于 ST-Link 烧录和调试。

确认上述命令可以从 `PATH` 中找到，然后在本目录执行：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

用于实际运行的优化版本：

```sh
cmake --preset Release
cmake --build --preset Release
```

构建输出位于 `build/Debug/` 或 `build/Release/`：

- `WL1_F411CEU6_Tele.elf`：调试和烧录；
- `WL1_F411CEU6_Tele.hex`：Intel HEX；
- `WL1_F411CEU6_Tele.bin`：裸二进制镜像；
- `WL1_F411CEU6_Tele.map`：链接映射和内存分析。

### 2. 使用 ST-Link 烧录

连接 SWDIO、SWCLK、GND、目标板参考电压后执行：

```sh
openocd -f STlink.cfg \
  -c "program build/Release/WL1_F411CEU6_Tele.elf verify reset exit"
```

烧录后，OLED 应显示 `WL1 REMOTE`，推动摇杆时数值和进度条会变化，PC13
活动指示灯随无线帧发送翻转。

## 操作说明

### 摇杆

| 控制量 | ADC 引脚 | 输出范围 | 无线含义 |
| --- | --- | ---: | --- |
| Roll | PA1 / ADC1_IN1 | -18.0° ～ 18.0° | 车体横滚目标 |
| Leg | PA2 / ADC1_IN2 | 44.5 ～ 78.5 mm | 腿高目标 |
| Turn | PA3 / ADC1_IN3 | -100.0 ～ 100.0 | 转向目标 |
| Speed | PA5 / ADC1_IN5 | -100.0 ～ 100.0 | 速度目标 |

四路摇杆的标定范围均为 `0..4095`，中心值为 `2048`，中心死区为
`±300` ADC counts。超出标定范围的输入会被限幅。

### 按键

按键使用内部上拉，低电平表示按下。

| 按键 | 引脚 | 短按 | 长按（至少 1 s） |
| --- | --- | --- | --- |
| Leg | PB0 | 锁定/解锁当前腿高 | 通过 UART 输出 Buttons 任务剩余栈 |
| Roll | PB1 | 锁定/解锁当前横滚角 | 通过 UART 输出 Buttons 任务剩余栈 |

锁定后，摇杆的实时位置仍会更新，但发给小车的对应目标保持不变；再次短按会
立即切换到摇杆的最新位置。长按只输出诊断信息，松开时不会再触发短按。

## 硬件连接

### nRF24L01+

| nRF 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SCK | PB13 | SPI2_SCK |
| MISO | PB14 | SPI2_MISO |
| MOSI | PB15 | SPI2_MOSI |
| CSN | PA8 | 软件片选，低有效 |
| CE | PB12 | 收发模式控制 |
| IRQ | PA12 | 下降沿 EXTI |
| VCC | 3.3 V | 不可接 5 V |
| GND | GND | 与主控共地 |

建议在 nRF 模块电源引脚附近放置去耦电容。无线收发异常时，首先确认 3.3 V
供电稳定和模块共地。

### OLED

SSD1306 128×64 OLED 使用 u8g2 软件 I²C：

| 信号 | MCU 引脚 | 参数 |
| --- | --- | --- |
| SCL | PB8 | 开漏输出 |
| SDA | PB9 | 开漏输出 |
| Address | — | 7-bit `0x3C` |

### 调试接口

| 功能 | MCU 引脚 | 参数 |
| --- | --- | --- |
| USART1 TX | PA9 / PA15 | 同时输出同一路串口，115200, 8-N-1 |
| USART1 RX | PA10 | DMA + IDLE 接收串口命令 |
| Activity LED | PC13 | 每次提交无线发送时翻转 |

PA9 用于常见 USB 串口接法，PA15 保留原遥控器接法；两者输出相同数据。
PA9 映射放在 `Core/Src/usart.c` 的 USER CODE 中，CubeMX 重新生成时会保留。

USB 串口模块的 **TXD 接 PA10，RXD 接 PA9（或 PA15），GND 共地**，参数设为
115200、8-N-1、无流控。只接 TXD 时，遥控器可以执行命令，但电脑看不到回复。

## 串口控制与反馈

兼容旧版 `nrfsend <小车命令>`，每次发送 32 字节，短命令补零。
`nrfsend` 前缀不占用无线 payload；超过 32 字节的内容会报错，不会截断发送。
建议用 CR、LF 或 CRLF 结束命令。串口助手不带换行时，连续空闲 50 ms
也会结束一条命令；逐字键入或分段发送时，段间停顿应小于 50 ms。
命令队列最多保存八条，每 50 ms 处理一条；持续发送请控制在每秒 20 条以内。

| 串口命令 | 行为 |
| --- | --- |
| `nrfsend R 0 0 0 61.5` | 将零转向、零速度、零横滚、61.5 mm 腿高发送给小车 |
| `R 0 0 0 61.5` | 上一条命令的简写 |
| `nrfsend nrfshow -mr 0` | 请求小车开启无线姿态回传 |
| `nrfsend nrfshow -nn` | 请求小车关闭无线回传 |
| `joystick off` | 暂停摇杆周期发送，供串口手动控制 |
| `joystick on` | 恢复摇杆每 50 ms 发送，重启后默认开启 |
| `status` | 查看发送成功、无 ACK、接收、驱动错误、串口丢弃和剩余栈计数 |
| `help` | 查看串口命令帮助 |

手动控制时按行发送：

```text
joystick off
nrfsend R 0 0 0 61.5
status
```

`joystick off` 只暂停自动发包，小车保留其最后收到的目标；它不是停车命令。
串口的 `R` 参数直接采用小车协议中的 Turn、Speed、Roll、Leg，不额外翻转
Speed。调试结束后发送 `joystick on` 恢复摇杆。OLED 仍显示摇杆值及锁定状态，
串口发送的内容请以串口反馈为准。

正常反馈示例：

```text
uart: sending R 0 0 0 61.5
nRF: send success [uart]: R 0 0 0 61.5
```

第一行表示开始提交发送；第二行只在 nRF 的 `TX_DS` 置位、收到自动 ACK 后
输出。摇杆帧也输出 `nRF: send success [joystick]: ...`。达到重试上限会输出
`nRF: send fail [uart]: no ACK; ...`，收到小车回传则输出 `receive: ...`。
无线 ACK 证明对端无线模块收到包，小车是否执行还取决于它的命令解析和运行状态。

## 无线协议

遥控器固定发送 32 字节 payload。有效内容是以空格分隔的 ASCII 文本：

```text
R <turn> <-speed> <roll_degrees> <leg_height_mm>
```

示例：

```text
R 0.0 -0.0 0.0 61.5
```

每个值保留一位小数，文本后补 `0x00` 到 32 字节。速度字段在编码时取反，
这是遥控器与小车坐标系之间的既有约定，不应在单端单独修改。

| 参数 | 当前设置 |
| --- | --- |
| Data rate | 2 Mbps |
| RF channel | 2（2402 MHz） |
| Address width | 5 bytes |
| TX / Pipe 0 address | `11 52 01 31 41` |
| Payload width | 32 bytes |
| RF power | 0 dBm |
| Auto ACK | 开启 |
| Auto retry | 500 μs 间隔，最多 15 次 |
| Application period | 50 ms（20 Hz） |

协议格式、字段顺序、地址、频道、速率和 payload 长度必须与
`car_firmware` 同步修改。

## 软件结构

```text
tele_firmware/
├── Component/
│   ├── HardWare/
│   │   ├── OLED/                 # SSD1306/u8g2 板级适配
│   │   └── Telecontrol/          # 摇杆转换与 nRF24L01+ 驱动
│   ├── Peripheral/               # 按键与异步 UART 日志
│   ├── UserApp/                  # 任务、状态、协议和显示逻辑
│   └── Module/include/etl/       # Embedded Template Library
├── Core/                         # STM32CubeMX 生成代码
├── Drivers/                      # STM32 HAL / CMSIS
├── Middlewares/                  # FreeRTOS
├── ThirdParty/u8g2/              # OLED 图形库
├── cmake/                        # 工具链与 CubeMX CMake 目标
├── CMakeLists.txt
├── CMakePresets.json
├── STlink.cfg
└── WL1_F411CEU6_Tele.ioc
```

应用代码入口是 `Component/UserApp/main.cpp` 中的 `CPP_Main()`，由
`MX_FREERTOS_Init()` 在调度器启动前调用。

## 开发约定

- `Core/` 中的 CubeMX 文件只在 `USER CODE` 区域添加手写代码；
- 新的应用源文件必须显式加入顶层 `CMakeLists.txt`；
- 不要在 ISR 中调用普通 FreeRTOS API，只使用 `...FromISR` 版本；
- 会调用 FreeRTOS ISR API 的中断，NVIC 数值优先级不得高于当前允许值 5；
- 保持任务周期内无堆分配、无无限等待、无高频 UART 打印；
- 修改协议或射频参数时，同时检查 `car_firmware`；
- 提交前至少完成 Debug、Release 构建和 `git diff --check`。

当前控制帧和 OLED 数值仍使用 `snprintf` 浮点格式化，因此链接配置保留
`-u _printf_float`。后续若替换编码实现，应同时验证无线 payload 和 OLED 显示。
