# WL1 小车端固件

[中文](README.md) | [English](README_en.md) | [返回项目主页](../README.md)

`car_firmware` 是 WL1 轮腿机器人的车体端固件，目标芯片为
STM32F411CEU6。固件读取 MPU6050 和左右轮编码器，运行串级 PID 控制，
驱动 TB6612 双路直流电机与两路腿部舵机，并通过 nRF24L01+ 接收遥控器命令。

当前运行路径使用 STM32 HAL、FreeRTOS 和 C++23：

- 10 ms 姿态环，计算左右轮 PWM；
- 50 ms 速度、转向和横滚/腿高控制；
- nRF24L01+ 固定 32 字节无线命令；
- USART1 DMA 收发，可在线查看状态和修改控制参数；
- ETL 固定容量容器，用于命令队列和 UART 缓冲；
- PA0 板载 KEY 单击、双击和长按，5 ms 低优先级扫描、静态任务和有界事件队列；
- 心跳、命令、舵机、运动和按键业务分别封装为继承 `AppTask` 的任务类；
- 在 `main.cpp` 中逐个显式初始化硬件模块，用轻量报告记录全部结果；
- 任一硬件初始化失败时进入安全模式：不创建平衡任务，轮电机 PWM 保持为 0，
  同时尽可能保留串口或无线命令应答。

进一步阅读：

- [软件架构](docs/architecture.md)：启动流程、任务、控制环和并发模型；
- [命令参考](docs/commands.md)：串口/无线命令、默认参数和调参顺序；
- [PA0 按键](docs/button-a0.md)：事件时序、内存、业务接入及实时性边界；
- [2026-09-05 工程审查](docs/engineering-review-2026-09-05.md)：已修复问题、验证结果和待整改项；
- [调试与故障排查](docs/troubleshooting.md)：上电检查、常见故障和
  CubeMX 重新生成检查项。

> [!WARNING]
> 只有全部硬件模块和必要应用任务均启动成功后，固件才允许创建并运行平衡任务。
> 这道门控不能替代物理安全措施；首次烧录、修改控制方向或调整 PID 时，仍应
> 架空车轮、断开电机功率或使用限流电源，并确保可以立即断电。

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

初始化、按键、任务生命周期、命令业务与 PID 等回归测试可在主机独立运行：

```sh
cmake -S tests -B build/host-tests -G Ninja
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
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
| `Release` | `-Ofast` | 实际运行 |
| `RelWithDebInfo` | `-Ofast -g` | 优化运行并保留调试信息 |
| `MinSizeRel` | `-Os` | 优先减小镜像 |

工程固定使用 Cortex-M4F 硬浮点 ABI（`fpv4-sp-d16`）、C11 和 C++23。

### 3. 使用 ST-Link 烧录

连接 SWDIO、SWCLK、GND 和目标板参考电压后执行：

```sh
openocd -f STlink.cfg \
  -c "program build/Release/WL1_F411CEU6.elf verify reset exit"
```

若已经用万用表确认主控板供电正常，但特定 ST-Link 仍误报低 Vref，可使用仓库内
经过本板验证的 100 kHz HLA 兼容配置：

```sh
openocd -f STlink_hla.cfg \
  -c "adapter speed 400; init; halt; flash write_image erase build/Release/WL1_F411CEU6.elf; verify_image build/Release/WL1_F411CEU6.elf; reset run; shutdown"
```

400 kHz 已完成整片写入与校验；配置文件会在复位时降回 100 kHz，供稳定调试。
HLA 是兼容后端，只用于已确认属于测量误报的调试器；其他 ST-Link 仍优先使用
`STlink.cfg` 的标准 SWD 后端。

烧录后，USART1 会按顺序输出每个初始化步骤，例如：

```text
[app] WL1 startup begin
[task][ OK ] Heartbeat
[init][ OK ] command-uart
[init][FAIL] imu-mpu6050
...
[app] state=init-failed control=off hw_fail=... task_fail=0
```

每个失败都会立即输出，但不会中断后续初始化。最终只有
`state=ready control=on` 才表示平衡与电机输出已获准运行。

## 启动、安全门控与 LED 心跳

FreeRTOS 调度器启动后，`AppBootstrap` 才调用 C++ 组合入口 `CPP_Main()`。
`Component/UserApp/main.cpp` 直接按以下顺序调用 8 个模块的初始化方法：

1. USART1 命令接收；
2. MPU6050；
3. 左编码器；
4. 右编码器；
5. TB6612 轮电机 PWM；
6. 左舵机 PWM；
7. 右舵机 PWM；
8. nRF24L01+。

每次调用后记录结果、输出日志并延时 3 ms，失败时继续初始化后续模块。
`InitializationReport::all_succeeded(bsp::kRequiredHardwareMask)` 统一检查报告有效、
必要模块全部尝试且全部成功；漏掉必要模块也不能通过门控。检查未通过时，
不创建 `ServoControl` 和 `MotionControl`，
强制轮电机 compare 为 0、方向脚为低，并停止两路舵机 PWM。独立的
`Heartbeat` 仍运行；USART1 或 nRF 中任一可用时，`CommandService` 仍运行，
可使用 `ping`、`status` 和原有参数命令。会导致输出的请求在安全模式下会被拒绝。

PC13 LED 按低电平点亮处理。一个“闪”表示约 120 ms 亮，模式如下：

| 状态 | LED 模式 | 周期 | 含义 |
| --- | --- | ---: | --- |
| `booting` | 100 ms 亮 / 100 ms 灭连续交替 | 200 ms | 正在逐项初始化 |
| `ready` | 80 ms 亮 / 920 ms 灭 | 1 s | 全部初始化成功，控制已启用 |
| `init-failed` | 连闪 2 次后停顿 | 1 s | 至少一个硬件步骤失败，安全模式 |
| `task-failed` | 连闪 3 次后停顿 | 1.12 s | FreeRTOS 应用任务创建失败，安全模式 |
| `runtime-fault` | 80 ms 亮 / 80 ms 灭连续快闪 | 160 ms | 运行期 IMU 连续读取失败，输出已关闭 |

没有串口适配器时，可通过 ST-Link/GDB 读取
`g_app_system_state`、`g_app_hardware_attempted_mask`、
`g_app_hardware_failed_mask`、`g_app_task_failed_mask` 和
`g_app_control_enabled`。位定义与上述初始化顺序一致，从 bit 0 开始。

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
| Status LED | PC13 | 低有效；模式见“启动、安全门控与 LED 心跳” |
| Onboard KEY | PA0 | 上拉输入、按下接地；5 ms 扫描，不启用 EXTI |
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

当前配置为陀螺仪 ±1000 °/s、加速度计 ±4 g，姿态融合使用 VQF。

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
最小比较值。PWM 命令最终限幅为 `-1000..1000`；零命令是专门的安全分支，
不会再应用死区，compare 保持为 0。

### 腿部舵机

| 功能 | MCU 引脚 | 外设 |
| --- | --- | --- |
| 左腿舵机 | PA2 | TIM9_CH1 |
| 右腿舵机 | PA3 | TIM9_CH2 |

TIM9 产生 100 Hz PWM。软件将目标腿高限制为 `44.5..78.5 mm`，经四连杆
逆运动学换算为舵机角度。机械尺寸和坐标系定义位于
`Component/UserApp/CtrlAlgorithm/LegKinematics.hpp`。

### nRF24L01+

| nRF 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SCK | PB13 | SPI2_SCK |
| MISO | PB14 | SPI2_MISO |
| MOSI | PB15 | SPI2_MOSI |
| CSN | PA4 | 软件片选，低有效 |
| CE | PB12 | 收发模式控制 |
| IRQ | PA12 | 上拉输入、下降沿 EXTI；模块缺失时避免悬空 |
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

nRF 初始化和寄存器回读验证成功后，小车进入接收模式。有效 payload 是以
`0x00` 补齐到 32 字节的 ASCII 命令，推荐控制帧为：

```text
R <turn_target> <velocity_target> <roll_degrees> <leg_height_mm>
```

示例：

```text
R 0.0 -0.0 0.0 61.5
```

字段顺序必须保持为转向、速度、横滚、腿高。仓库中的 `tele_firmware`
会把遥控器速度取反后放入第二个字段，这是两端现有坐标系约定。

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
│   ├── Application/               # 初始化报告、AppTask、运行状态、纯按键状态机
│   ├── Bsp/                       # 板级硬件门面、稳定模块 ID 与名称
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
│       ├── Tasks/                # 五个继承 AppTask 的业务任务类
│       ├── ControlState.hpp      # 参数、反馈及腿目标快照
│       └── main.cpp              # 组合入口、初始化、安全门控
├── Core/                         # STM32CubeMX 生成代码
├── Drivers/                      # STM32 HAL / CMSIS
├── Middlewares/                  # FreeRTOS
├── CMakeLists.txt
├── CMakeLists_template.txt
├── STlink.cfg
├── STlink_hla.cfg                 # Vref 已确认误报时的低速兼容配置
├── tests/                          # 主机状态机、并发与业务回归测试
└── WL1_F411CEU6.ioc
```

C 入口是 `Core/Src/main.c`。`MX_FREERTOS_Init()` 只创建 `AppBootstrap`；
调度器启动后，该任务调用 `CPP_Main()` 进行硬件组合、逐项初始化和应用任务
创建，随后删除自身。这与参考工程的 app-main task 模型一致，同时避免在
调度器启动前创建软件定时器、信号量或调用会阻塞的初始化代码。

这里借鉴 [esp_idf_template](https://github.com/lk888l/esp32_idf/tree/main/esp_idf_template)
的入口组合、模块封装和分层思路，并按本项目需要调整启动方式：在 `main.cpp`
逐模块调用初始化，失败后保留诊断通道，最后统一安全门控。初始化不再经过工厂、
函数指针表或观察回调；业务仍由独立 FreeRTOS 任务运行。具体取舍和新增模块步骤见
[软件架构](docs/architecture.md)。

`MotionControlTask` 保留串级 PID、10 ms/50 ms 周期和原优先级，只在必要任务与
硬件安全门控通过后启动。`LQR` 算法类保留作实验，未调度的旧入口循环已移除。
`MainControl.*` 和 OLED 模块同样尚未接入应用路径。

## 开发约定

- `Core/` 中手写逻辑放在 `USER CODE`；GPIO 声明和配置变更同步 `.ioc`，避免重新生成丢失；
- `CMakeLists.txt` 标记为模板生成文件，持久修改应同步到
  `CMakeLists_template.txt`；
- 源文件 glob 使用 `CONFIGURE_DEPENDS`，新增任务文件会触发 CMake 重新检查；
- 任务继承 `AppTask`，在组合入口注入依赖，禁止复制或销毁运行中的任务；
- 新硬件在 `Bsp/HardwareModule.hpp` 分配稳定 ID，并在 `main.cpp` 显式初始化、记录结果；
- 控制共享数据通过 `ControlState` 快照传递，不跨任务暴露可写引用；
- 不要在 ISR 中调用普通 FreeRTOS API，只使用 `...FromISR` 版本；
- 会调用 FreeRTOS ISR API 的中断，其 NVIC 数值优先级不得小于 5；
- 控制任务中避免动态分配、阻塞式 I/O 和高频 UART 输出；
- 修改轮向、编码器方向或姿态符号后，必须架空验证闭环反馈方向；
- 修改无线协议时同时更新 `tele_firmware` 和两端文档；
- 提交前至少完成 Debug、Release 构建和 `git diff --check`。
