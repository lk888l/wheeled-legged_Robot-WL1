# ZX-D30 蓝牙串口适配（2026-09-13）

当前固件已适配并烧录到 STM32F411CEU6，**9600 8N1，nRF 默认关闭**。
详细接线、微信操作和本轮实际无线测试见 [ZX-D30 联调记录](zx-d30.md)。
下文是 2026-09-12 的 JDY-31 历史记录，其中的 115200 和 SPP 限制不代表当前硬件。

---

# 历史：蓝牙串口适配与验证（2026-09-12）

本说明对应 `feature/SoftEngine` 工作区的蓝牙适配，保留本轮之前修复的运行时 PID
调参、重心补偿和 IMU 启动门控。未提交或推送 Git。

**当前配置：115200 8N1、nRF 关闭。** 用户将模块改为 115200 后，已同步并烧录固件，
经 PA15/PA10 收到真实版本回包 `+VERSION=JDY-31-V1.35,Bluetooth V3.0`。
下文保留首轮 9600 测试历史；最新烧录和串口实测见文末“115200 同步与实测”。

## 模块和微信的兼容边界

用户提供的《JDY-31 蓝牙底板使用手册》第 3 页明确标注 **Bluetooth 3.0 SPP**，
第 5 页列出默认串口波特率 **9600**。这是经典蓝牙串口；手册个别表项写了 BLE，
不能据此判断支持 BLE GATT。普通 HC-05 也属于经典 SPP。

小程序项目 `D:/kk/wechat-app/cf-studio_wechat-app` 使用 `wx.createBLEConnection` /
`wx.writeBLECharacteristicValue`。其已有实测记录显示模块为
`JDY-31-V1.35 / Bluetooth V3.0 / JDY-31-SPP`、9600 8N1。
因此，**仅修改 STM32 固件不能让这个 SPP 模块直接连接微信小程序**。
微信端需要具备 UART 透传服务的 BLE 模块；保留 JDY-31 时可使用 Windows 或原生
Android 的 SPP 客户端发送相同命令。没有修改模块名称、配对密码或 AT 配置。

参考：[厂商资料](https://telesky.yuque.com/bdys8w/01/hao54g0d16drugbe?singleDoc)、
[微信 BLE API](https://developers.weixin.qq.com/miniprogram/dev/api/device/bluetooth-ble/wx.writeBLECharacteristicValue.html)。

## 接线与构建

| 项目 | 本次默认配置 |
| --- | --- |
| 模块 TXD | 接 MCU PA10 / USART1 RX |
| 模块 RXD | 接 MCU PA15 / USART1 TX |
| GND | 共地 |
| 串口 | 115200、8 数据位、无校验、1 停止位、无流控 |
| 模块供电 | 用户提供的 JDY-31 **底板**手册为 3.6–6 V、建议 5 V；裸模块请按其资料 |
| STATE / EN | 固件不依赖这些引脚，不等待配对或 AT 应答 |
| nRF24L01+ | 默认不初始化 SPI2 和 nRF 驱动，也不接收 nRF 遥控 |

PA15 的 USART1_TX 复用已与
[STM32F411 数据手册](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf)核对。
串口引脚启用上拉，模块未接时保持空闲高电平。

从 `car_firmware` 目录构建：

```sh
cmake -S . -B build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release -DWL1_COMMAND_UART_BAUD=115200 -DWL1_ENABLE_NRF24=OFF
cmake --build build/Release --parallel
```

用户已将模块改为 115200，固件默认值、CubeMX 配置和当前构建同步为 115200。
若使用仍为出厂 9600 的模块，改用 `-DWL1_COMMAND_UART_BAUD=9600`；已有 CMake
构建目录会保留缓存，切换波特率时必须显式传入这个参数。
需要实体 nRF 遥控器时可显式选择 `-DWL1_ENABLE_NRF24=ON`；此时 nRF 失败仍不会
阻止控制启动。不要同时发送两路运动命令：当前以最后收到的有效命令为准。
配置默认值也保存在 `Core/Inc/communication_config.h`；CubeMX 重新生成后应检查
波特率宏及 `MX_SPI2_Init()` 的条件编译是否保留。

## 启动行为

控制必需硬件为 IMU、左右编码器、电机定时器与左右舵机，掩码 **0x7E**。
串口位 0 与 nRF 位 7 是可选通信硬件；它们失败只影响诊断位图。
命令任务创建失败、两路通信均不可用，也不会阻止 ServoControl / MotionControl。
Heartbeat、ServoControl、MotionControl 仍为必要任务。

默认成功启动的硬件尝试掩码为 **0x7F**，nRF 显示 `[init][SKIP]`；串口 DMA 初始化
成功仅表示 MCU 接收器已启动，不能证明蓝牙模块存在或手机已连接。
IMU 必须连续满足原有的 500 ms 静稳条件才允许轮电机输出，连续 3 次 IMU 故障仍会
锁存关闭输出。通信超时不会触发这个 IMU 故障锁存。

## 命令与分帧

现有 `R <turn> <velocity> <roll> <height>`、`anglepid`、`velocitypid`、`differpid`、
`rollpid` 和 `anglebias` 命令继续有效。`anglepid -p` 保持手动 Kp，腿高变化不覆盖
它；`anglepid -auto` 恢复按腿高计算 Kp。

旧客户端仍可一次发送一条完整的短命令，以 UART 空闲事件为边界。**旧格式不支持
跨空闲事件拼接**。新蓝牙客户端使用以下显式格式（`\n` 表示单个 LF 字节）：

```text
@R 0 0 0 44.5\n
@R -100 -100 -18 78.5\n
@anglepid -p 75\n
@anglebias 10.5\n
@controlstate\n
```

- `@` 开始新命令并清除未完成的旧帧；LF/CRLF 结束命令。
- 正文最多 32 字节；超长、非法字符和接收错误导致的残帧被丢弃。
- 显式帧须在 300 ms 内收齐；延迟尾包不能重新启动已过期的请求。
- 支持逐字节分包及同一个 DMA 接收块内的多条有结束符命令。
- UART DMA 关闭半传输通知，只在 IDLE/TC 时交接缓冲；错误后重新接收。
- 命令队列深度 4，建议 100 ms 发送一条运动命令，不堆积历史运动目标。

小程序 `main` 配置保留原来的最多 20 字节单包格式。新的 `SoftEngine` 配置采用
`@R ...\n`，最长 22 字节；BLE 写入按最多 20 字节顺序拆分，片间留 25 ms。
两条命令的分片不会交错，断开或超时后不补发旧尾包。切换固件配置先发送旧格式
归零命令，随后需要重新启用本地控制。小程序界面中的控制开关不是固件电源开关。

## 运动指令超时

有效的 `R`、`VandD`、`target_roll` 记录时间。超过 **500 ms** 没有更新时：

- 生效的速度、转向、横滚目标归零；共同腿高保持最近设置。
- 姿态 PID、速度反馈、腿部补偿和舵机任务继续工作，输出不强制归零。
- PID 参数、重心基准和手动 Kp 模式保留；调参、查询、非法命令不会延长运动指令寿命。
- 新的局部运动命令不会恢复已经过期的其他运动分量。

MCU 将速度/转向限制为 ±100、横滚限制为 ±18°、腿高限制为 44.5–78.5 mm。
`controlstate` 增加 `remote_timeout` 和三个实际生效目标。原始参数快照保留最后请求，
运动任务在本地副本上归零，所以诊断时应看反馈中的生效目标。

## 首轮 9600 验证及限制

| 验证 | 结果 |
| --- | --- |
| 主机 C++ 回归 | 43/43，通过真实应用入口、命令任务、控制任务和 UART 驱动测试 |
| 通信可选启动 | nRF 开/关、任一或全部通信失败、命令任务创建失败均覆盖 |
| 必需硬件/任务失败 | 仍保持输出门控；没有放宽 IMU 和执行器错误处理 |
| UART 分帧 | 每种切分位置、逐字节、粘包、长帧、错误、超时、tick 回绕 |
| UART DMA | 半传输不交接缓冲、128 字节完整接收、缓冲耗尽、错误和重启失败恢复 |
| 运行时调参 | 原来的全部 PID 增益回归继续通过；分包调参不提前执行 |
| 小程序 | 原项目在 Windows / Node 20.18 下完整 `npm run check` 通过：格式、严格 TypeScript、73/73 测试；微信 wcc/wcsc 模板编译也通过 |
| ST-Link | 已烧录并校验；最终 247,124 字节 Flash 读回与构建产物逐字节一致，读回前后实时数据区未改变 |
| 最终板上运行 | 复位后 257.14 秒、25,711 次控制循环；最大间隔 10 ms、超期 0；ready / armed / IMU 有效，CFSR/HFSR 为 0 |
| 备用构建 | 115200 + nRF 开启的 Release 构建通过，当时未替换 9600 / nRF 关闭镜像 |
| 板上通信失败注入 | 绕过串口初始化并返回失败，nRF 关闭；状态 ready、control=1、硬件失败位=1、任务尝试=0x1D，控制周期 10 ms |
| 板上分包与超时 | 从 UART 已收缓冲队列注入：半条 PID 命令不执行，收齐后 Kp=75；R 生效后超时目标归零，armed/IMU 保持有效 |

板上注入经过真实 `LkUart::signal_RxComplete`、分帧器、命令任务和 PID 任务，
**没有把注入数据当作物理 RX 或手机蓝牙收发证明**。经实际 UART TX 发出的只读
`AT+VERSION` 查询暂未收到模块回复；本轮未完成手机→模块→PA10 的实际链路验证。
首轮的 9600 与 PA10/PA15 依据资料及小程序原有记录配置；用户随后将模块改为 115200。

小程序测试入口改为 `node --test`，解决 Windows / Node 20 不展开测试路径通配符的问题；
格式检查接受现有换行符，并排除微信开发者工具维护的 `project.config.json`。
该文件的用户原有修改按字节保留；两个锁定的开发依赖从本地缓存安装，未运行安装脚本。

小车固定在测试架上，轮电机和舵机线按用户之前的确认保持断开。本轮可以验证启动、
计算周期、门控、PID 参数和定时器输出，**不能证明接上负载后的实际自平衡稳定性**。
本轮复位测试为调试器复位，不等价于重新接通全部电源的冷启动。

构建、烧录前整片 Flash 备份、板上日志及测试日志位于工作区的
`car_firmware/build/bluetooth-20260912/`（该目录被 Git 忽略）。烧录校验完成后必须
复位再观察控制；运行后的镜像核对使用 Flash 读回与主机字节比较，不让目标 CRC
算法占用实时任务的数据区。

最终烧录文件：`build/bluetooth-20260912/firmware/WL1_F411CEU6.bin`。
SHA-256：`1dfdea56f69b44112c6bfc4469fb8ac9671778e88638a549d8d81f35cfe37681`。
当前镜像占用 Flash 247,124 字节、静态 RAM 47,664 字节；运行时可用 FreeRTOS heap
11,032 字节、最低 8,552 字节。调试后的临时参数和故障注入均已通过复位清除，主控继续运行。

### 后续 CRLF 查询复测

按用户要求再次核对并发送实际 CRLF 字节（`0D 0A`），不是字面量 `/r/n` 或 `\\r\\n`。
上次查询脚本也已包含 CRLF；此次增加发送缓冲、长度和 DMA 完成状态记录。
该次 UART 配置为 9600 8N1，依次发送 `AT+VERSION\r\n`、`AT\r\n`、
`AT+VERSION?\r\n`、`AT+VERSION\r\n`，每次运行并等待 2 秒。

四次均确认 TX DMA 剩余计数为 0、UART 发送状态 READY、错误计数无增加；RX DMA
剩余计数保持 128，接收回调计数无增加，未捕获物理接收字节。发送完成只能确认
MCU 发送路径完成，不能证明模块收到请求。该结果还需结合模块实际波特率、接线和
AT 可用状态定位。本次未重烧固件、未修改模块配置或 PID 参数。

复测前后控制任务均为 ready、IMU 有效、最大循环间隔 10 ms、超期 0；此次观察到
姿态 pitch 约 -43.5°，启动门控尚未放行（armed=false，左右 PWM=0），不与此前
257 秒测试的姿态和 armed 状态混用。调试已脱离，主控继续运行。
日志：`build/bluetooth-20260912/at-crlf-retry.log`；脚本：同目录 `at-crlf-retry.gdb`。

### 115200 同步与实测

用户确认已将模块波特率改为 115200 后，固件的 CMake 默认值、生成模板、头文件回退
值、CubeMX `.ioc` 和小程序接线提示已同步为 115200。新的构建显式使用
`-DWL1_COMMAND_UART_BAUD=115200 -DWL1_ENABLE_NRF24=OFF`，板上读到
`huart1.Init.BaudRate=115200`。本次未改动 PID 算法、通信可选启动行为或模块 AT 配置。

| 查询（末尾均为实际 CRLF 字节） | 实际 RX 结果 |
| --- | --- |
| `AT+VERSION\r\n`，两次 | 两次均收到 `+VERSION=JDY-31-V1.35,Bluetooth V3.0` |
| `AT+VERSION?\r\n`，一次 | 收到相同版本回复 |
| `AT\r\n`，一次 | 等待 2 秒无回复 |

以上版本数据来自真实 UART DMA 接收缓冲，不是命令队列注入；调试器只借用软件
`ping` 触发正常 TX 路径，并在 HAL 发送入口替换为 AT 查询。TX DMA 均完成，
三次版本查询各产生一次新的物理接收事件，UART 错误和丢包计数均为 0。
这确认了当前 115200 配置下 MCU 与模块的串口双向链路；手机蓝牙透传仍未验证，
SPP 与微信 BLE 的兼容边界不变。

新固件构建及烧录校验通过，247,124 字节 Flash 读回与构建文件逐字节一致；烧录前
整片备份的程序区与之前 9600 镜像一致。查询结束时控制任务 ready、IMU 有效，
循环计数 36,851、tick=368,540 ms、最大间隔 10 ms、超期 0，CFSR/HFSR 为 0。
当时 pitch 约 -43.6°，启动门控保持 armed=false、左右 PWM=0；未绕过姿态保护。
调试已脱离，主控继续运行。小程序本次仅修改三处波特率说明，格式检查通过。

本次产物、备份与日志目录：`build/bluetooth-115200-20260912/`。
当前烧录文件：`build/bluetooth-115200-20260912/firmware/WL1_F411CEU6.bin`。
SHA-256：`d661d3dd4e54b02ab6d4143c48916e93a0ea13fef01aaeaba3aa54753577cc1d`。
