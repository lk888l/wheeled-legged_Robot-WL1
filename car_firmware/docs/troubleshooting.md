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
7. 启动日志列出 8 个 `[init]` 结果，并以 `[app] state=...` 结束；
8. 暂不开启电机功率，分别检查 IMU、编码器和舵机。

## 启动日志

| 日志 | 含义 |
| --- | --- |
| `[init][ OK ] <name>` | 该硬件步骤完成；固件继续下一项 |
| `[init][FAIL] <name>` | 该硬件步骤失败；固件记录失败位并继续下一项 |
| `[task][ OK ] <name>` | 对应应用任务创建成功 |
| `[task][FAIL] <name>` | 任务创建失败，优先检查 FreeRTOS heap 和任务栈 |
| `[app] state=ready control=on ...` | 所有启动门控通过，平衡控制已启用 |
| `[app] state=init-failed control=off ...` | 至少一个硬件步骤失败，执行器保持安全状态 |
| `[runtime][FAIL] imu read; control stopped` | 连续 3 次 IMU 读取失败，已在运行期关断输出 |
| `nRF: send success` | TX 完成且收到 ACK |
| `nRF: send fail` | 达到最大自动重发次数 |
| `receive: ...` | 收到无法匹配的文本命令 |

nRF 初始化会回读 RF channel 和 address width；SPI DMA 虽然完成但模块缺失、
MISO 悬空或寄存器值不匹配时，`radio-nrf24` 仍会报告失败。

## 常见现象

| 现象 | 优先检查 |
| --- | --- |
| 无任何日志、LED 不翻转 | 供电、BOOT0、复位、时钟、HardFault、镜像地址 |
| `[task][FAIL] ...` | 32 KiB FreeRTOS heap、任务栈、重复创建对象 |
| `[init][FAIL] imu-mpu6050` | PB6/PB7、0x68 地址、上拉、电源和共地 |
| IMU 值跳变或缓慢漂移 | 安装方向、振动、陀螺零偏、采样周期 |
| 手转轮子但 RPM 为 0 | TIM2/TIM3 引脚、编码器供电、相线和计数器 |
| 电机一上电就全速 | 反馈符号、Pitch 偏置、编码器左右映射、PWM 方向 |
| 左右轮纠偏方向相反 | TB6612 B 反相配置、左右电机接线、差速符号 |
| 舵机顶到机械限位 | PA2/PA3 映射、舵机装配零位、`-10°` 偏置、连杆尺寸 |
| 遥控器有发送但小车不响应 | 地址、频道、速率、payload 长度、PA12 IRQ |
| 串口偶尔少日志 | UART 固定发送缓冲已满；高频日志会被丢弃 |
| PC13 连闪 2 次 | 硬件初始化失败；执行 `status` 或读失败位图 |
| PC13 连闪 3 次 | 应用任务创建失败；检查 heap 和任务栈 |
| PC13 连续快闪 | 运行期 IMU 读取故障，输出已关闭 |

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
TB6612 的零命令有独立安全分支，compare 保持 0，不再重新应用 50 counts
死区。进入安全模式时还会把四个方向脚拉低。

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
8. 小车 PA12 配置为上拉输入、下降沿 EXTI；
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

没有连接串口或外部模块时，等待约 1 秒后中断目标并读取启动状态：

```gdb
monitor reset run
shell sleep 1
monitor halt
p g_app_system_state
p/x g_app_hardware_attempted_mask
p/x g_app_hardware_failed_mask
p/x g_app_task_failed_mask
p g_app_control_enabled
```

仅连接主控板时，MPU6050（bit 1）和 nRF24L01+（bit 7）应检测失败，因此典型
`g_app_hardware_failed_mask` 为 `0x82`，状态为 `initialization_failed`（数值 3），
`g_app_control_enabled` 必须为 0。编码器、电机和舵机步骤只能验证 MCU 定时器
是否成功启动，无法在没有反馈/识别引脚的情况下判断板外器件是否物理存在。

本次仅连接主控板的实测结果如下，连续 3 次复位结果一致：

| 检查项 | 实测值 | 结论 |
| --- | --- | --- |
| 硬件尝试 / 失败位图 | `0xFF / 0x82` | 8 项全部执行；IMU、nRF 失败后仍继续 |
| 任务尝试 / 失败位图 | `0x03 / 0x00` | 仅 Heartbeat、CommandService 创建成功 |
| 系统状态 / 控制门 | `3 / 0` | `initialization_failed`，控制关闭 |
| TIM1 CCR1 / CCR2 | `0 / 0` | 左右轮 PWM 为零 |
| PA6、PA7、PB0、PB1 | 全部低电平 | TB6612 四个方向输入关闭 |
| TIM9 CCER、CCR1、CCR2 | `0 / 0 / 0` | 两路舵机 PWM 已停止 |
| PC13 LED | 两次约 120 ms 点亮，随后约 640 ms 熄灭 | `init-failed` 心跳符合设计 |

ST-Link VCP（COM22，115200 8-N-1）串口实测同时覆盖 LF 和 CRLF 帧：

| 输入 | 实测应答 |
| --- | --- |
| `ping` | `pong state=init-failed control=off` |
| `status` | `status=init-failed control=off hw_fail=130 task_fail=0` |
| `motor 100 100` | `motor rejected: control is in safe mode` |
| `unknown_probe` | `receive: unknown_probe` |

执行被拒绝的 `motor` 命令后再次读取寄存器，TIM1/TIM9 CCR 仍全部为 0，四个
TB6612 方向位仍为低电平。

若 OpenOCD 报告 target voltage 过低，应先用万用表确认目标板 VCC 和 ST-Link
Vref，不能仅凭“仍能识别芯片”忽略欠压。若已独立确认供电正常且该探头属于
已知测量误报，可改用 `STlink_hla.cfg`：复位和寄存器调试使用 100 kHz，整片
Flash 写入显式切换为已实测的 400 kHz，避免 100 kHz 下 Flash 算法超时。本项目
所连接的 ST-LINK/V2 已通过 CPUID 读取、ELF 写入校验和多次复位运行验证。
标准探头仍应优先使用 `STlink.cfg`。

## CubeMX 重新生成检查项

使用 `WL1_F411CEU6.ioc` 重新生成后至少检查：

1. MCU 仍为 STM32F411CEU6，HSE 25 MHz，SYSCLK 100 MHz；
2. `Core/Src/freertos.c` 的 `StartAppBootstrap()` USER CODE 中仍在调度器启动后
   调用 `CPP_Main()`，而 `MX_FREERTOS_Init()` 不提前调用它；
3. FreeRTOS tick 仍为 1 kHz、heap 仍满足任务创建需求；
4. USART1 仍映射 PA15/PA10，RX/TX DMA 分别为 Stream 5/7；
5. SPI2 仍映射 PB13/PB14/PB15，DMA 分别为 Stream 3/4；
6. PA12 仍为上拉输入、下降沿 EXTI，优先级为 6；
7. SPI/UART DMA 中断优先级仍为 5；
8. TIM1/TIM2/TIM3/TIM9 的引脚、prescaler 和 period 未改变；
9. I2C1 仍为 PB6/PB7、400 kHz；
10. CMake 中 `Component/Application`、`Component/Bsp` 和 UserApp 的 include/source
    glob 仍完整；
11. Debug 和 Release 都能完成编译、链接并生成 ELF/HEX/BIN。
