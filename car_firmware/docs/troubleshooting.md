# 调试与故障排查

本文按“安全与供电 → 启动日志 → 传感器 → 执行器 → 无线协议”的顺序排查。
不要在传感器方向和硬件接线尚未确认时直接提高 PID。

## 最小安全上电检查

1. 架空车轮，或断开 TB6612 的电机功率；
2. 使用限流电源，确认逻辑 3.3 V 和舵机/电机电源没有接反；
3. ST-Link、串口、nRF、传感器和电机驱动共地；
4. nRF24L01+ 只接 3.3 V，并在模块附近放置去耦电容；
5. 烧录后执行 verify，再复位运行；
6. USART1 使用 PA15/PA10、115200, 8-N-1；
7. 启动日志包含 `CPPMain: success` 和 `MPU: success`；
8. 暂不开启电机功率，分别检查 IMU、编码器和舵机。

## 启动日志

| 日志 | 含义 |
| --- | --- |
| `CPPMain: success` | 任务创建状态检查通过，但当前 OR 累积逻辑不能证明三个任务都成功 |
| `CPPMain: fail` | 创建状态检查失败，优先检查 FreeRTOS heap 和任务栈 |
| `MPU: success` | MPU6050 初始化事务成功 |
| `MPU: fail` | I2C、地址、供电或器件身份检查失败 |
| `nRF: send success` | TX 完成且收到 ACK |
| `nRF: send fail` | 达到最大自动重发次数 |
| `receive: ...` | 收到无法匹配的文本命令 |

当前 nRF 初始化返回值没有启动日志，因此“没有 nRF 日志”不能证明初始化成功。

## 常见现象

| 现象 | 优先检查 |
| --- | --- |
| 无任何日志、LED 不翻转 | 供电、BOOT0、复位、时钟、HardFault、镜像地址 |
| `CPPMain: fail` | 32 KiB FreeRTOS heap、任务栈、重复创建对象 |
| `MPU: fail` | PB6/PB7、0x68 地址、上拉、电源和共地 |
| IMU 值跳变或缓慢漂移 | 安装方向、振动、陀螺零偏、采样周期 |
| 手转轮子但 RPM 为 0 | TIM2/TIM3 引脚、编码器供电、相线和计数器 |
| 电机一上电就全速 | 反馈符号、Pitch 偏置、编码器左右映射、PWM 方向 |
| 左右轮纠偏方向相反 | TB6612 B 反相配置、左右电机接线、差速符号 |
| 舵机顶到机械限位 | PA2/PA3 映射、舵机装配零位、`-10°` 偏置、连杆尺寸 |
| 遥控器有发送但小车不响应 | 地址、频道、速率、payload 长度、PA12 IRQ |
| 串口偶尔少日志 | UART 固定发送缓冲已满；高频日志会被丢弃 |
| PC13 翻转不稳定 | Reactor 被高频事件唤醒、控制临界区过长 |

## IMU 排查

执行：

```text
showimu -y
```

静止放置时 Roll/Pitch 应稳定，Yaw 可以随时间漂移。分别绕各轴缓慢转动车体，
确认符号和安装坐标系与控制算法一致，然后执行：

```text
showimu -n
```

当前陀螺零偏在 `MotionControlFunc_PID()` 中固定为：

```text
X = 2.5, Y = 0.7, Z = 0.9
```

更换 MPU6050 或机械安装后应重新标定。控制代码还会根据腿高动态计算
`Angle_bias`；不要只通过 `anglebias` 命令判断持久偏置。

## 编码器和电机排查

先保持电机断电，执行：

```text
showrpm -y
```

手动分别转动左右轮，确认：

- 左轮只影响 `A`，右轮只影响 `B`；
- 同向转动时符号符合预期；
- 静止时数值回到 0；
- 50 ms 采样窗口内没有明显随机跳变。

完成后执行 `showrpm -n`。

闭环方向检查时，将车体固定在支架上并短时接通电机功率。车体向前倾时，车轮
应向前追赶重心；若相反，应先修正 IMU/PWM 方向，不能靠增大 PID 解决。

`motor` 命令在当前 PID 路径中不会直接驱动电机，不能用它作为硬件点动测试。
另外，TB6612 驱动当前会在零命令分支之后重新应用 50 counts 最小 PWM；
如果需要可靠滑行/制动状态，应先修正零值处理并在支架上复验。

## 舵机与腿部机构

从最低安全高度开始：

```text
legheight 44.5
```

再小步增加，例如：

```text
legheight 50
legheight 60
```

任务会把实际目标限制为 `44.5..78.5 mm`。若两侧方向不同或顶住：

1. 检查 PA2 是否连接左腿、PA3 是否连接右腿；
2. 检查右舵机反向脉宽映射；
3. 检查两侧装配零位和代码中的 `-10°` 偏置；
4. 对照 `LegKinematics.hpp` 的连杆长度；
5. 断电手动确认机构全行程没有死点。

不要依赖软件限幅代替机械限位。

## 无线链路

两端必须同时满足：

1. RF channel 为 2；
2. data rate 为 2 Mbps；
3. 地址为 `11 52 01 31 41`；
4. Pipe 0 payload width 为 32；
5. auto ACK 开启；
6. payload 是补零的 ASCII 文本；
7. 字段顺序为 Turn、Velocity、Roll、Height；
8. 小车 PA12 配置为下降沿 EXTI；
9. SPI2 RX/TX DMA 中断和 EXTI15_10 中断正常。

典型中心帧：

```text
R 0.0 -0.0 0.0 61.5
```

若遥控器持续报告 `radio: no ACK`：

- 检查小车是否已经上电并进入 RX；
- 检查 nRF 3.3 V 在发射瞬间是否跌落；
- 检查 CE、CSN 和 IRQ 是否接反；
- 在 `NRF24L01P::signal_IRQEvent()` 设置断点，确认 RX_DR；
- 检查 SPI DMA 信号量能否在 100 tick 超时前释放。

`nrfsend` 和 `nrfshow` 会让小车临时进入 TX。排查正常遥控接收时先执行
`nrfshow -nn`，并避免主动发送测试帧。

## 构建问题

### 找不到构建工具

```sh
cmake --version
arm-none-eabi-gcc --version
ninja --version
```

如果没有 Ninja，可使用：

```sh
cmake -S . -B build/Debug -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --parallel
```

不要在同一个构建目录中切换 CMake 生成器。生成器或工具链路径变化后，使用
新的构建目录重新配置。

### 找不到 `arm-none-eabi-*`

将 Arm GNU Toolchain 的 `bin` 目录加入 `PATH`，确保 `gcc`、`g++`、
`objcopy`、`objdump` 和 `size` 来自同一套工具链。

### 链接空间不足

构建时会打印 RAM/FLASH 占用，详细来源见 `WL1_F411CEU6.map`。优先检查：

- 新增任务的栈深度；
- 全局/静态缓冲；
- 浮点格式化和未使用的 C++ 模板实例；
- 是否误把新的第三方源码目录全部加入递归 glob。

## OpenOCD 与 GDB

烧录并校验：

```sh
openocd -f STlink.cfg \
  -c "program build/Debug/WL1_F411CEU6.elf verify reset exit"
```

启动 GDB server：

```sh
openocd -f STlink.cfg
```

另一个终端连接：

```sh
arm-none-eabi-gdb build/Debug/WL1_F411CEU6.elf
```

```gdb
target remote localhost:3333
monitor reset halt
load
monitor reset run
```

若 OpenOCD 报告 target voltage 过低，应先用万用表确认目标板 VCC 和 ST-Link
Vref，不能仅凭“仍能识别芯片”忽略欠压。

## CubeMX 重新生成检查项

使用 `WL1_F411CEU6.ioc` 重新生成后至少检查：

1. MCU 仍为 STM32F411CEU6，HSE 25 MHz，SYSCLK 100 MHz；
2. `Core/Src/freertos.c` 的 USER CODE 中仍调用 `CPP_Main()`；
3. FreeRTOS tick 仍为 1 kHz、heap 仍满足任务创建需求；
4. USART1 仍映射 PA15/PA10，RX/TX DMA 分别为 Stream 5/7；
5. SPI2 仍映射 PB13/PB14/PB15，DMA 分别为 Stream 3/4；
6. PA12 仍为下降沿 EXTI，优先级为 6；
7. SPI/UART DMA 中断优先级仍为 5；
8. TIM1/TIM2/TIM3/TIM9 的引脚、prescaler 和 period 未改变；
9. I2C1 仍为 PB6/PB7、400 kHz；
10. CMake 中的应用 include 和 source glob 仍完整；
11. Debug 和 Release 都能完成编译、链接并生成 ELF/HEX/BIN。
