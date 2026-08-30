# 调试与故障排查

本文按“先供电和接线，再外设，最后协议”的顺序排查。不要一开始就修改任务
优先级、无线地址或控制算法，否则容易同时引入多个变量。

## 最小上电检查

1. 目标板 3.3 V 稳定，nRF24L01+ 不接 5 V；
2. ST-Link、遥控器和外部串口共地；
3. Release 或 Debug 镜像烧录后通过 verify；
4. OLED 出现 `WL1 REMOTE`；
5. 推动摇杆时 OLED 数值变化；
6. PC13 随发送翻转；
7. USART1 TX（PA15）能输出 `radio: ready`。

调试串口参数为 115200, 8-N-1。应用当前只使用 TX，接收线路不是必需的。

## 常见现象

| 现象 | 优先检查 |
| --- | --- |
| 遥控器完全不启动 | 供电、BOOT0、复位、烧录地址、HardFault |
| OLED 不亮但小车可控制 | PB8/PB9、0x3C 地址、OLED 供电；显示故障不影响无线 |
| OLED 数值不变化 | PA1/PA2/PA3/PA5 接线、ADC DMA、`input: ADC failure` |
| PC13 不翻转 | Radio 任务是否创建、nRF 初始化或 SPI DMA 是否失败 |
| 有发送但小车不动作 | 两端协议、地址、频道、速率和 payload width |
| 持续 `radio: no ACK` | 小车/nRF 是否上电、接收端是否进入 RX、射频参数是否一致 |
| `radio: SPI transmit failed` | SPI2 DMA 中断、CSN/CE 接线、HAL SPI 状态 |
| 按键连击或无响应 | PB0/PB1 上拉、低有效接法、接地、机械抖动 |
| UART 日志偶尔缺失 | 日志固定缓冲已满；这是保护实时任务的预期行为 |

## UART 日志解释

| 日志 | 含义 |
| --- | --- |
| `radio: ready` | SPI 配置事务完成，驱动已请求进入接收模式；不等同于已收到对端 ACK |
| `radio: init failed (attempt N)` | SPI/寄存器配置失败，1 s 后自动重试 |
| `radio: no ACK (count N)` | 达到最大自动重发次数，TX FIFO 已清空 |
| `radio: IRQ handling failed` | 读取/清除 nRF 状态时 SPI 事务失败 |
| `radio: SPI transmit failed` | payload 写入或切换 TX 模式失败 |
| `radio: command encoding failed` | 控制文本无法放入 32 字节 payload |
| `input: ADC failure (count N)` | ADC DMA 启动、超时或 HAL 错误 |
| `leg: locked/live` | 腿高锁定状态已切换 |
| `roll: locked/live` | 横滚锁定状态已切换 |
| `buttons: stack N words` | 长按按键输出 Buttons 任务历史最小剩余栈 |

故障日志经过限频，不代表计数只增加了一次。

## 无法控制小车

先确认遥控器确实在发送合法帧。Debug 构建下可在
`NRF24L01P::send()` 设置断点并检查：

```text
data   -> 32 字节 payload
length -> 32
```

中心位置的典型内容是：

```text
R 0.0 -0.0 0.0 61.5
```

随后逐项对比 `tele_firmware` 和 `car_firmware`：

1. RF channel 都是 2；
2. data rate 都是 2 Mbps；
3. 地址都是 `11 52 01 31 41`；
4. Pipe 0 payload width 都是 32；
5. 两端都启用 auto ACK；
6. 小车端解析顺序为 Turn、Speed、Roll、Leg；
7. 小车端已进入 RX 模式且 IRQ 中断正常。

若小车能动作但仍报告 no ACK，重点检查小车端 EN_AA、RX_ADDR_P0 和 IRQ
处理；这说明上行 payload 可能到达，但自动应答链路不完整。

反过来，若小车向遥控器发送探测包时能收到 ACK，但小车始终收不到 `R` 帧，
只能说明遥控器上的 nRF 仍在 PRX 模式并由硬件自动应答，不能证明遥控器 MCU、
Radio 任务或 SPI DMA 正常运行。此时按下面顺序检查：

1. PC13 是否持续随 50 ms 发送周期翻转；
2. Radio 任务是否仍在运行，保存的 PC 是否位于 `radioTask()` 循环；
3. `hspi2.State` 是否长期为 BUSY，DMA1 Stream 3/4 的 NDTR 是否停止变化；
4. PA12 IRQ 是否持续为低，以及 nRF 的 `STATUS`、`FIFO_STATUS`；
5. 板上 Flash 是否确实是当前构建的遥控器镜像。

当前驱动会在 SPI DMA 启动失败、超时或 HAL 状态异常时拉高 CSN、执行
`HAL_SPI_Abort()` 并清理完成信号量；Radio 任务也会在每个发送周期补查 IRQ
低电平，避免只依赖下降沿通知而永久丢失中断。若仍卡在 BUSY，应继续检查 DMA
中断、HardFault 和供电，而不是仅反复重试 nRF 初始化。

## ADC 排查

ADC DMA 缓冲顺序固定为：

```text
sample[0] = PA1 = Roll
sample[1] = PA2 = Leg
sample[2] = PA3 = Turn
sample[3] = PA5 = Speed
```

正常中心值约为 2048。若某一路长期为 0 或 4095，通常是摇杆电源、公共地或
模拟信号接线问题，而不是死区设置。

## OpenOCD

烧录并校验：

```sh
openocd -f STlink.cfg \
  -c "program build/Debug/WL1_F411CEU6_Tele.elf verify reset exit"
```

启动 GDB server：

```sh
openocd -f STlink.cfg
```

另一个终端连接：

```sh
arm-none-eabi-gdb build/Debug/WL1_F411CEU6_Tele.elf
```

```gdb
target remote localhost:3333
monitor reset halt
load
monitor reset run
```

若 OpenOCD 报告 target voltage 过低，应先用万用表确认目标板 VCC 和 ST-Link
Vref 接线。部分 ST-Link 克隆器的电压读数可能不准确，但不能仅凭“还能烧录”
就忽略真实欠压风险。

## 构建问题

### 找不到编译器或 Ninja

确保以下命令能直接运行：

```sh
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

删除对应构建目录后重新配置，通常可以清除旧工具链绝对路径：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

### 浮点数据显示异常

控制帧和 OLED 当前使用 `snprintf` 浮点格式化。顶层 CMake 已通过
`-u _printf_float` 为 `nano.specs` 启用支持。若绕过本项目 CMake 直接链接，
需要保留等价链接参数。

ETL 头文件可能输出 GCC 7.1 参数传递 ABI 的 `note`；它不是编译失败。真正的
项目警告仍应处理后再提交。
