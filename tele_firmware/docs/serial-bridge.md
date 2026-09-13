# 串口无线调参（bridge v1）

本版遥控器固件为 WL1 Studio 的“连接遥控器”模式提供串口到 nRF24L01+
调参转发。旧版 `tele_firmware` 没有启动 UART 接收，升级上位机后还需重新
编译并烧录本版遥控器固件。小车继续使用原有的 32 字节 ASCII 无线命令协议。

## 接线和输入协议

- 3.3 V USB-UART TX → PA10（遥控器 RX），RX → PA15（遥控器 TX），GND 共地。
- USART1：115200 baud、8 data bits、no parity、1 stop bit、no flow control。
- 每条命令以 LF 结束，可使用 CRLF；命令正文最多 31 个可打印 ASCII 字节。
- 一条命令可拆成多次串口写入；收到 LF 前不会执行。多条命令按 LF 拆分。
- 建议命令间隔至少 120 ms。不要通过删除间隔一次性批量灌入大量参数。
- 无线 payload 为命令正文后补零到 32 字节；LF / CRLF 不进入无线包。

支持的命令：

| 命令 | 参数 |
| --- | --- |
| `anglepid` | `-p`、`-i`、`-d` + 数值，或 `auto` |
| `velocitypid` | `-p`、`-i`、`-d` + 数值 |
| `differpid` | `-p`、`-i`、`-d` + 数值 |
| `rollpid` | `-p`、`-i` + 数值 |
| `anglebias` | 数值，或 `auto` |

`auto` 为旧版本协议兼容转发，能否执行取决于小车固件；当前模块化小车固件
不实现自动整定。数值支持普通十进制和科学记数法，小车最终检查数值有效性。
本链路不提供参数读取或应用执行确认。

例如发送 `anglepid -p 2.5\n`。`R`、`VandD`、`legheight`、`target_roll`、
`showimu`、`showrpm`、`nrfshow`、`nrfsend` 和其他未知命令会在遥控器端拒绝。
实体遥控器保留运动、腿高和横滚控制权，避免串口命令与每 50 ms 的摇杆帧互相覆盖。

## 输出和传输边界

| 日志 | 含义 |
| --- | --- |
| `bridge: ready v1 ...` | 遥控器串口桥接接收已启动；不代表小车在线 |
| `bridge: queued <command>` | 完整有效命令已进入遥控器发送队列 |
| `bridge: radio ACK <command>` | nRF24L01+ 收到了对端无线模块 ACK |
| `bridge: no ACK <command>` | 本包用完无线硬件重试次数仍未收到 ACK |
| `bridge: rejected ...` | 格式、长度、能力、队列、过期或 UART 错误，命令未提交发送 |
| `bridge: radio timeout` / `bridge: SPI transmit failed` | 无线驱动异常，本次调参投递未确认 |

无线模块 ACK 只证明无线包被接收，不证明小车已执行参数，也不代表参数写入 Flash。
遥控器不会自动重发已完成或结果不明的调参命令。需要重试时应核对当前输入后重新发送。
日志沿用固定缓冲非阻塞 UART logger，拥塞时可能丢日志，不能将“没有失败日志”视为成功。
当前固件不把小车的无线遥测回传给上位机；上位机仅显示遥控器串口日志。

## 调度与故障处理

串口中断只向 128 字节环形缓冲写入数据。Radio 任务解析完整行，使用容量为 4 的
固定命令队列；未发送命令超过 500 ms 会丢弃。UART 溢出/帧错误会清空队列并丢弃
受损行直到下一个 LF，恢复后请先发送一个 LF，再重新发送完整命令。

Radio 任务每 25 ms 服务一次，将调参插入到每 50 ms 的实体摇杆帧之间。所有 SPI
和无线发包都由该任务执行，同一时间只保留一个待完成发送；未完成发送不会被下一包
覆盖。无线状态寄存器也会周期轮询，以恢复丢失的 IRQ 通知。驱动重新初始化时不重放
离线期间接收的命令。实体摇杆帧及原有射频配置保持兼容。

## 验证

Debug / Release 构建命令与主 README 相同。可在有 C++23 主机编译器的环境中，
从 `tele_firmware` 目录运行无硬件协议和队列测试：

```sh
g++ -std=c++23 -Wall -Wextra -Werror -I tests/bridge_stubs -I Component/UserApp tests/serial_bridge_test.cpp Component/UserApp/SerialCommandBridge.cpp -o build/serial_bridge_test
./build/serial_bridge_test
```

测试覆盖分片和连续帧、LF/CRLF、31/32 字节边界、命令白名单、受损帧恢复、
队列溢出/过期、离线半行和 UART 接收重启。构建与主机测试不能替代实机无线验证：
烧录后应确认无车时 `no ACK`、车辆在线时 `radio ACK`，并由小车本体串口或行为
验证实际参数生效，以及持续调参时摇杆控制周期正常。
