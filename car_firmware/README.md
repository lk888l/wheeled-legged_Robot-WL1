# WL1 小车端固件

[中文](README.md) | [English](README_en.md) | [返回项目主页](../README.md)

`car_firmware` 是 WL1 轮腿机器人的车体端固件，目标芯片为
STM32F411CEU6。固件读取 MPU6050 和左右轮编码器，运行串级 PID 控制，
驱动 TB6612 双路直流电机与两路腿部舵机，并通过 nRF24L01+ 接收遥控器命令。

当前运行路径使用 STM32 HAL、FreeRTOS 和 C++23：

- 10 ms 姿态环，计算左右轮 PWM；
- 50 ms 速度、转向和横滚/腿高控制；
- 柔和、普通、运动三档控制参数，切换时用 400 ms 渐变；
- 原始 PID/俯仰偏置在线调节也使用限幅渐变，跳跃武装、动作或 Fault 锁存期间锁定；
- nRF24L01+ 固定 32 字节无线命令；
- 250 ms 遥控失联保护和 nRF 自动重初始化；
- USART1 DMA 收发，可在线查看状态和修改控制参数；
- PC13 两秒状态图案，正常慢心跳并区分 IMU、无线和跳跃状态；
- ETL 固定容量容器，用于命令队列和 UART 缓冲。

进一步阅读：

- [软件架构](docs/architecture.md)：启动流程、任务、控制环和并发模型；
- [开发规范](docs/development.md)：组件边界、CubeMX 所有权和验证门槛；
- [命令参考](docs/commands.md)：串口/无线命令、默认参数和调参顺序；
- [调试与故障排查](docs/troubleshooting.md)：上电检查、常见故障和
  CubeMX 重新生成检查项。

> [!WARNING]
> 轮腿自平衡控制会在任务启动后直接输出电机 PWM。首次烧录、修改控制方向或
> 调整 PID 时，应架空车轮、断开电机功率或使用限流电源，并确保可以立即断电。
> 当前默认构建使用 `WL1_REQUIRE_UPRIGHT_STARTUP=OFF`，IMU 获得可信姿态后会在
> 任意俯仰/横滚角直接接管车轮；倒地上电也可能输出接近满占空比。需要恢复扶正后
> 才启动的保护时，以 `-DWL1_REQUIRE_UPRIGHT_STARTUP=ON` 重新配置并构建。

## 快速开始

### 1. 准备工具

- CMake 3.28 或更新版本；
- Ninja、Make 或其他 CMake 支持的构建工具；
- Arm GNU Toolchain，命令前缀为 `arm-none-eabi-`；
- OpenOCD 和 ST-Link，用于烧录与调试；
- STM32CubeMX（可选），仅在修改 `.ioc` 外设配置时需要。

确认工具可以从 `PATH` 中找到：

```sh
cmake --version
arm-none-eabi-gcc --version
openocd --version
```

### 2. 构建

在 `car_firmware` 目录执行：

```sh
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --parallel
```

用于实际上车的优化构建：

```sh
cmake -S . -B build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release --parallel
```

如果系统没有 Ninja，可将 `-G Ninja` 换成可用生成器，例如 Windows 上的
`-G "MinGW Makefiles"`。切换生成器时使用新的构建目录。

构建输出位于所选构建目录：

- `WL1_F411CEU6.elf`：调试和烧录；
- `WL1_F411CEU6.hex`：Intel HEX；
- `WL1_F411CEU6.bin`：裸二进制镜像；
- `WL1_F411CEU6.map`：链接映射和内存分析。

构建配置说明：

| `CMAKE_BUILD_TYPE` | 编译选项 | 用途 |
| --- | --- | --- |
| `Debug` 或未指定 | `-Og -g` | 调试、单步和变量观察 |
| `Release` | `-O2` | 实际运行；保留控制安全路径中的 NaN/Inf 检查 |
| `RelWithDebInfo` | `-O2 -g` | 优化运行并保留调试信息 |
| `MinSizeRel` | `-Os` | 优先减小镜像 |

工程固定使用 Cortex-M4F 硬浮点 ABI（`fpv4-sp-d16`）、C11 和 C++23。

跳跃执行有单独的编译安全门，默认关闭。普通构建只运行可观测的 dry-run
状态机，不会用跳跃轨迹覆盖舵机目标：

```sh
cmake -S . -B build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DWL1_ENABLE_EXPERIMENTAL_JUMP=OFF
```

`ON` 仅预留给完成机械限位、舵机时序和架空测试后的实验固件。

### 3. 使用 ST-Link 烧录

连接 SWDIO、SWCLK、GND 和目标板参考电压后执行：

```sh
openocd -f STlink.cfg \
  -c "program build/Release/WL1_F411CEU6.elf verify reset exit"
```

烧录后，USART1 应输出 `CPPMain: success`，MPU6050 初始化成功时输出
`MPU: success`。PC13 使用 2 秒状态图案；正常状态仅亮 200 ms，明显慢于故障提示。

## 首次上电

建议按以下顺序缩小故障范围：

1. 断开电机功率或架空车轮，只给逻辑部分和 ST-Link 供电；
2. 确认 USART1 能收到启动日志；
3. 执行 `showimu -y`，静止时观察姿态值是否稳定；
4. 执行 `showrpm -y`，手动转动车轮，确认左右编码器有响应；
5. 给舵机供电，使用 `legheight 44.5` 检查两侧方向和机械限位；
6. 遥控器置中并上电，确认无线命令可以更新目标值；
7. 最后接通电机功率，先用限流电源和支架检查反馈方向。

详细检查方法见 [调试与故障排查](docs/troubleshooting.md)。

## 硬件连接

### 核心与调试接口

| 功能 | MCU 引脚 | 参数 |
| --- | --- | --- |
| USART1 TX | PA15 | 115200, 8-N-1，DMA2 Stream 7 |
| USART1 RX | PA10 | 115200, 8-N-1，DMA2 Stream 5，Receive-to-idle |
| SWDIO | PA13 | ST-Link |
| SWCLK | PA14 | ST-Link |
| Activity LED | PC13 | 低电平点亮，100 ms 时间槽状态图案 |
| HSE | PH0/PH1 | 25 MHz |
| LSE | PC14/PC15 | 32.768 kHz |

PA15 不是常见的 USART1_TX 默认引脚；接串口工具时应以本表和
`Core/Src/usart.c` 为准。

### MPU6050

| 信号 | MCU 引脚 | 参数 |
| --- | --- | --- |
| SCL | PB6 | I2C1，400 kHz |
| SDA | PB7 | I2C1，400 kHz |
| Address | — | 7-bit `0x68`（驱动中使用左移后的 `0xD0`） |

当前配置为陀螺仪 ±2000 °/s、加速度计 ±8 g、100 Hz，DLPF 配置为 3。
驱动每周期从 `0x3B` 连续读取 14 字节，并使用 VQF 融合。线加速度模长不在
`0.8..1.2 g`、偏离已学习静止模长、与陀螺预测重力方向相差超过 12°，或原始值
接近饱和时，不把该加速度写入倾角修正；回到 6° 内才重新接纳。融合丢失后，
只有外部轮速/命令门确认近似静止，且稳定重力持续 300 ms，才会硬重捕获，避免
大动作后的缓慢回正和动态加速度误校正。

### 电机与编码器

| 功能 | MCU 引脚 | 外设 |
| --- | --- | --- |
| 左轮 PWM / TB6612 A | PA8 | TIM1_CH1，约 10 kHz |
| 右轮 PWM / TB6612 B | PA9 | TIM1_CH2，约 10 kHz |
| AIN1 / AIN2 | PA6 / PA7 | 左轮方向 |
| BIN1 / BIN2 | PB0 / PB1 | 右轮方向 |
| 左编码器 A / B | PA5 / PA1 | TIM2_CH1 / CH2 |
| 右编码器 A / B | PB4 / PB5 | TIM3_CH1 / CH2 |

当前 PID 路径将 TB6612 B 通道方向反相，并给两路 PWM 都配置 50 counts
最小比较值。PWM 命令最终限幅为 `-1000..1000`；零命令会保持 compare 为 0，
不会应用死区补偿。

### 腿部舵机

| 功能 | MCU 引脚 | 外设 |
| --- | --- | --- |
| 左腿舵机 | PA2 | TIM9_CH1 |
| 右腿舵机 | PA3 | TIM9_CH2 |

TIM9 产生 100 Hz PWM。软件将目标腿高限制为 `44.5..78.5 mm`，经四连杆
逆运动学换算为舵机角度。机械尺寸和坐标系定义位于
`Component/UserApp/CtrlAlgorithm/LegKinematics.hpp`。舵机没有轴位置回读；固件只会
确认 PWM 任务存活、最新目标已消费以及软件平滑轨迹完成，不能据此证明机械到位。

### nRF24L01+

| nRF 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SCK | PB13 | SPI2_SCK |
| MISO | PB14 | SPI2_MISO |
| MOSI | PB15 | SPI2_MOSI |
| CSN | PA4 | 软件片选，低有效 |
| CE | PB12 | 收发模式控制 |
| IRQ | PA12 | 下降沿 EXTI |
| VCC | 3.3 V | 不可接 5 V |
| GND | GND | 与主控共地 |

SPI2 时钟约 6.25 Mbit/s，收发均使用 DMA。建议在 nRF 模块电源附近放置
去耦电容；无线异常时首先检查 3.3 V 供电和共地。

### 预留 OLED 接口

`Component/HardWare/OLED` 驱动会参与编译，但当前应用没有初始化或刷新
OLED。CubeMX 保留了软件 I²C 引脚：

| 信号 | MCU 引脚 |
| --- | --- |
| O_SCL | PB10 |
| O_SDA | PB3 |

## 无线协议

小车在上电后进入 nRF 接收模式。有效 payload 是以 `0x00` 补齐到 32 字节
的 ASCII 命令，推荐控制帧为：

```text
R <turn_target> <velocity_target> <roll_degrees> <leg_height_mm> [profile [flags]]
```

示例：

```text
R 0.0 -0.0 0.0 61.5
```

字段顺序必须保持为转向、速度、横滚、腿高。仓库中的 `tele_firmware`
会把遥控器速度取反后放入第二个字段，这是两端现有坐标系约定。
可选 `profile` 为 `0/1/2`（柔和/普通/运动）；`flags` 的 bit0 为跳跃武装，
bit1 为跳跃请求。旧遥控器不发送可选字段时仍兼容运动控制，但始终解除跳跃
武装。任意有效 `R` 帧超过 250 ms 未刷新时，车端会将速度、转向、横滚目标
清零并解除跳跃武装；真实跳跃还要求无线来源的 `R` 帧持续新鲜。

| 参数 | 当前设置 |
| --- | --- |
| Data rate | 2 Mbps |
| RF channel | 2（2402 MHz） |
| Address width | 5 bytes |
| RX Pipe 0 / TX address | `11 52 01 31 41` |
| Payload width | 32 bytes |
| RF power | 0 dBm |
| CRC | 1 byte |
| Auto ACK | 开启 |
| Auto retry | 500 μs 间隔，最多 15 次 |

射频参数和 `R` 命令格式必须与 `tele_firmware` 同步修改。全部可用命令和
限制见 [命令参考](docs/commands.md)。

## 软件结构

```text
car_firmware/
├── Component/
│   ├── App/                       # 模块生命周期、注册、失败回滚
│   ├── AppModules/                # 通信、舵机和运动控制任务
│   ├── HardWare/
│   │   ├── IMU/                  # MPU6050 与 VQF
│   │   ├── Motor/                # 编码器、TB6612、舵机
│   │   ├── OLED/                 # 当前未启用的 OLED 驱动
│   │   └── Telecontrol/          # nRF24L01+ 驱动
│   ├── Module/
│   │   ├── Basic/                # TaskReactor、SafeQueue
│   │   └── include/etl/          # Embedded Template Library
│   ├── Peripheral/               # USART DMA 封装
│   └── UserApp/
│       ├── CtrlAlgorithm/        # PID、LQR、腿部运动学
│       └── main.cpp              # 只负责注册和启动应用模块
├── cmake/                        # 固件目标、编译与链接策略
├── Core/                         # STM32CubeMX 生成代码
├── Drivers/                      # STM32 HAL / CMSIS
├── Middlewares/                  # FreeRTOS
├── tests/                        # 不依赖开发板的主机单元测试
├── CMakeLists.txt
├── CMakeLists_template.txt
├── STlink.cfg
└── WL1_F411CEU6.ioc
```

C 入口是 `Core/Src/main.c`。`MX_FREERTOS_Init()` 在调度器启动前调用
`CPP_Main()`；入口使用固定容量的 `AppManager` 按顺序注册并启动通信、舵机和
运动控制模块。任一任务创建失败时，已创建任务会按反序回滚，启动日志会输出
`CPPMain: fail`。

PID 是当前运行的控制路径。LQR 路径仍保留作实验，但当前不会运行。
`MainControl.*` 和 OLED 模块同样尚未接入应用路径。

## 开发约定

- `Core/` 中的 CubeMX 文件只在 `USER CODE` 区域添加手写代码；
- `CMakeLists.txt` 标记为模板生成文件，持久修改应同步到
  `CMakeLists_template.txt`；
- 新增组件源文件时，在该组件的 `CMakeLists.txt` 中显式登记，并重新运行 CMake；
- 不要在 ISR 中调用普通 FreeRTOS API，只使用 `...FromISR` 版本；
- 会调用 FreeRTOS ISR API 的中断，其 NVIC 数值优先级不得小于 5；
- 控制任务中避免动态分配、阻塞式 I/O 和高频 UART 输出；
- 修改轮向、编码器方向或姿态符号后，必须架空验证闭环反馈方向；
- 修改无线协议时同时更新 `tele_firmware` 和两端文档；
- 提交前至少完成 Debug、Release、主机测试和 `git diff --check`。

主机测试命令：

```sh
cmake -S tests -B build/host-tests -G "MinGW Makefiles"
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```
