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
| `CPPMain: success` | 三个模块注册成功，且三个任务均创建成功 |
| `CPPMain: fail` | 创建状态检查失败，优先检查 FreeRTOS heap 和任务栈 |
| `MPU: success` | MPU6050 初始化事务成功 |
| `MPU: fail` | I2C、地址、供电或器件身份检查失败；电机 PWM 保持为 0 |
| `nRF: send success` | TX 完成且收到 ACK |
| `nRF: send fail` | 达到最大自动重发次数 |
| `nRF init success` | SPI 寄存器回读正确并已进入 RX |
| `nRF init failed, retrying` | 初始化失败；保持安全目标并每 1 s 自动重试 |
| `receive: ...` | 收到无法匹配的文本命令 |

执行 `nrfstatus` 可查看初始化次数、补轮询、有效收包、SPI 错误和队列丢包。

## 常见现象

| 现象 | 优先检查 |
| --- | --- |
| 无任何日志、LED 无任何图案 | 供电、BOOT0、复位、时钟、HardFault、镜像地址 |
| `CPPMain: fail` | 32 KiB FreeRTOS heap、任务栈、重复创建对象 |
| `MPU: fail` | PB6/PB7、0x68 地址、上拉、电源和共地 |
| IMU 值跳变或缓慢漂移 | 安装方向、振动、陀螺零偏、采样周期 |
| 手转轮子但 RPM 为 0 | TIM2/TIM3 引脚、编码器供电、相线和计数器 |
| 电机一上电就全速 | 反馈符号、Pitch 偏置、编码器左右映射、PWM 方向 |
| 左右轮纠偏方向相反 | TB6612 B 反相配置、左右电机接线、差速符号 |
| 舵机顶到机械限位 | PA2/PA3 映射、舵机装配零位、`-10°` 偏置、连杆尺寸 |
| 遥控器有发送但小车不响应 | 地址、频道、速率、payload 长度、PA12 IRQ |
| 串口偶尔少日志 | UART 固定发送缓冲已满；高频日志会被丢弃 |
| PC13 图案停顿或错乱 | Reactor/SPI 长时间阻塞、tick 异常或高优先级任务占用过久 |

PC13 为低电平点亮，每个字符表示 100 ms，图案每 2 秒重复：

| 图案 | 状态 |
| --- | --- |
| `██..................` | 正常，单次慢心跳 |
| `█████.....█████.....` | nRF 已就绪，但 250 ms 内没有有效无线 `R` 帧 |
| `██..██..............` | 跳跃已武装 |
| `██..██..██..........` | nRF 初始化或 SPI 故障，正在重试 |
| `██..██..██..██......` | 跳跃 Fault 已锁存，等待明确解除武装 |
| `██..██..██..██..██..` | IMU 无效或控制正在恢复 |
| `████████████████████` | 跳跃动作阶段（默认构建仍是 dry-run） |

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

当前 MPU6050 使用 ±2000 °/s、±8 g、100 Hz 和 2 ms I2C 超时。每次只做一次
14 字节 burst；连续三次姿态无效会把两个轮电机 PWM 置零。`controlstatus` 可查看
I2C 失败和原始饱和计数。

陀螺零偏在 `Component/AppModules/src/motion_control_module.cpp` 中固定为：

```text
X = 2.5, Y = 0.7, Z = 0.9
```

更换 MPU6050 或机械安装后应重新标定。控制代码还会根据腿高动态计算
`Angle_bias`；`anglebias auto` 可从手动覆盖恢复自动值。

大动作后回正慢时同时观察 `a=...` 和 `ok=...`：加速度先要求模长在
`0.8..1.2 g`、接近已学习静止模长，还会与陀螺预测的重力方向比较。方向误差
超过 12° 时拒绝，回到 6° 内才重新接纳，因此“水平 0.5 g + 竖直 1 g”不会被
误当成倾角。若陀螺预测曾丢失，只有轮速不超过 5 rpm、遥控目标居中、非跳跃动作，
且静止重力方向/模长稳定 300 ms，才允许硬重捕获；首次静止模长窗口为
`0.90..1.10 g`，之后使用 `max(0.03 g, 3%)` 容差。去除已学习零偏后的陀螺
模长还需不超过 5°/s。`ok=0` 时未饱和的陀螺预测仍继续。正常可信重力修正的
VQF 倾角时间常数为 0.5 s；建议台架验收为静止后 1.5 s 内
Roll/Pitch 回到 2° 内。出现饱和计数则说明该段已接近约 1900 °/s 或 7.6 g，
不应继续依赖该段姿态完成动作。

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
TB6612 零命令应使 TIM1 compare 保持为 0；非零小命令才会应用 50 counts
死区补偿。排查“零目标仍轻微驱动”时应直接观察 TIM1 compare 值。

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

先执行：

```text
nrfstatus
controlstatus
```

正常接收时 `ready=1` 且 `rx` 持续增加。车端先上电、遥控器先上电、遥控持续
发送时复位车端三种顺序都应恢复；IRQ 在绑定前已经拉低时，任务会主动补服务，
SPI 故障则会退出 ready 并在约 1 s 内重新初始化。关闭遥控器超过 250 ms 后，
`controlstatus` 的 `remote` 应变为 0，速度/转向/横滚归零。

若遥控器持续报告 `radio: no ACK`：

- 检查小车是否已经上电并进入 RX；
- 检查 nRF 3.3 V 在发射瞬间是否跌落；
- 检查 CE、CSN 和 IRQ 是否接反；
- 在 `NRF24L01P::signal_IRQEvent()` 设置断点，确认 RX_DR；
- 检查 SPI DMA 信号量能否在 10 ms 超时前释放。

`nrfsend` 和 `nrfshow` 会让小车临时进入 TX。排查正常遥控接收时先执行
`nrfshow -nn`，并避免主动发送测试帧。

## 跳跃状态机（默认 dry-run）

先保持电机线断开、腿部机构架空，依次执行 `jump arm`、等待车体稳定、再执行
`jump trigger`，用 `jump status` 或 `controlstatus` 观察状态。默认构建不会改变
舵机目标。舵机 API 中的当前角度只是软件命令估计，不是物理位置反馈；启用
`WL1_ENABLE_EXPERIMENTAL_JUMP=ON` 前必须单独完成机械限位、阶段时序、落地冲击和
失联/IMU 故障测试。实验构建还要求舵机任务 ready 且心跳不超过 200 ms；待跳
稳定门要求最新目标序号已消费、舵机软件平滑完成，且控制档位和原始 PID/偏置
渐变均已结束。动作及 Fault 中原始参数保持冻结。动作中任务失联或控制周期超过
30 ms 会进入 Fault；解除武装后仍需 abort 目标的软件确认和 200 ms 保持才复位。

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
7. PA4 / SPI2_CSN 的初始输出电平仍为 High，PB12 / CE 初始为 Low；
8. SPI/UART DMA 中断优先级仍为 5；
9. TIM1/TIM2/TIM3/TIM9 的引脚、prescaler 和 period 未改变；
10. I2C1 仍为 PB6/PB7、400 kHz；
11. CubeMX 生成的 include/source 列表仍由顶层 CMake 传给 `cmake/firmware.cmake`，
    各手写组件的显式源文件列表仍完整；
12. Debug 和 Release 都能完成编译、链接并生成 ELF/HEX/BIN。
