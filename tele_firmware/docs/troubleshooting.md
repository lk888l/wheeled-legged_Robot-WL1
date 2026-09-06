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
7. USART1 TX（PA9 或 PA15）能输出 `radio: ready`。

调试串口参数为 115200, 8-N-1、无流控。USB 串口 TXD 接 PA10，RXD 接 PA9
或 PA15，GND 共地。修复前固件仅启用了 PA15，若 RXD 接 PA9 就会完全没有输出；
现在两路同时输出相同数据。

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
| 串口命令能执行但电脑没有回复 | USB 串口 RXD 是否接 PA9/PA15；只接 PA10 只能发送命令 |
| 串口控制值很快恢复成摇杆值 | 手动调试前发送 `joystick off`，完成后用 `joystick on` 恢复 |
| 串口命令没有识别 | 使用 `nrfsend <小车命令>`，发送 CR/LF；无换行需连续空闲 50 ms |

## UART 日志解释

| 日志 | 含义 |
| --- | --- |
| `radio: ready` | SPI 配置事务完成，驱动已请求进入接收模式；不等同于已收到对端 ACK |
| `uart: ready (115200 8N1); type help` | 已启动串口 DMA 接收 |
| `uart: sending ...` | 已解析串口命令，开始提交无线发送 |
| `nRF: send success [uart/joystick]: ...` | `TX_DS` 确认无线 ACK；不等同于小车已执行命令 |
| `nRF: send fail [uart]: no ACK; ...` | 该串口命令达到无线重试上限 |
| `receive: ...` | 收到对端无线 payload，并转发到串口 |
| `uart: command too long ...` | 拒绝超长命令，未截断发送 |
| `uart: RX error/overflow ...` | 接收错误或队列溢出；检查接线、波特率及发送速率 |
| `uart: command queue full ...` | PC 发包超过八槽命令队列容量 |
| `radio: TX result timeout` | 30 ms 内未取得 TX 结果，驱动将重新初始化 |
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

输入 `status` 查看累计计数、UART 丢弃数和 Radio 任务剩余栈。
测试无线回传时可依次发送 `joystick off`、`nrfsend nrfshow -mr 0`，观察
`receive: ...`，再用 `nrfsend nrfshow -nn` 关闭回传并发送 `joystick on`。

串口解析的主机回归测试（需要本机 C++ 编译器，不使用 Arm 交叉编译器）：

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I Component/UserApp tests/serial_commands_test.cpp Component/UserApp/SerialCommandQueue.cpp -o build/serial_commands_test
./build/serial_commands_test
```

连接遥控器和已上电的小车后，可在 PowerShell 中运行实机回归（端口按实际修改）：

```powershell
./tests/device_serial_test.ps1 -Port COM7
```

脚本验证零速度控制帧、无换行/分段输入、连续四条命令、payload 边界及姿态回传，
结束时关闭姿态回传并恢复 `joystick on`。日志默认保存到
`build/uart-radio/hardware-test.log`。

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
