# 主机回归与参数持久化验证

模块化任务与 Flash 保存统一使用 tests/CMakeLists.txt。Windows PowerShell：

```powershell
./tests/run_host_tests.ps1 -Compiler 'D:/rj/CLion 2024.1/bin/mingw/bin/g++.exe'
```

可用 `-NinjaPath <ninja.exe>` 指定 Ninja；否则优先使用 PATH 中的 Ninja，再使用
编译器旁的 mingw32-make.exe。脚本运行 Debug/Release 全部 CTest，并检查 VQF
拒绝不安全的 -Ofast。也可按主 README 用 CMake/CTest 运行单一配置。

回归覆盖启动依赖和故障锁存、任务通知、UART DMA/BLE 分片、按键、VQF、控制环、
遥控超时，以及以下持久化集成行为：

- 每组 PID 的 P/I/D 命令实际影响控制输出，重心/腿高补偿与 Kp 基准保持一致；
- Flash 全字段恢复、CRC、逐字中断、回读失败、写满和显式回收；
- v1/v2 记录混合、相同参数不写入、仅模式变化也持久化；
- 真实命令任务经 UART 分片/nRF 输入保存，重启时瞬态指令和诊断开关不恢复；
- 保存恰好抢占解锁周期时保持零 PWM，并重新计满 500 ms；
- 连续 control off/on 也重置门控，故障状态不能恢复控制。

`tests/fakes` 为主机的 RTOS/硬件/Flash 替身，不进入 STM32 镜像。IMU 等阻塞 I/O
必须在临界区外，最终 PWM 寄存器写入与保存互锁原子提交。主机测试不验证真实 BLE
传输、STM32 Flash 时序或闭环负载。

## 合并后板上复验步骤

1. 按项目 README 准备设备，烧录时保留扇区 7；用 USART1/BLE 发送 `@params\n`。
2. `@control off\n` 后等待 armed=false。调节 15 项参数与角度模式，发 `@save\n`。
3. 再发 save 确认 unchanged；改 RAM 参数但不保存，完整断电后确认恢复已保存值。
4. 分别用 -p 80 和 -manual 80，在 44.5/61.5/78.5 mm 核对自动值 74.9/80/85.1
   和固定值 80/80/80；保存、重启后模式及基准一致。
5. control on 后重新满足稳定窗口；运行中 save 返回 busy，不会异步补写。

遥控器串口桥接沿用远端白名单，只支持有限调参，不转发 save 或任意 R 帧；以上复验
直接连接小车命令 UART/BLE。协议及回包见 [命令参考](../docs/commands.md)。
下面的实板记录属于合并前镜像，保留作为 v1 数据来源；合并后版本另行复验。

## 2026-09-13 合并前实板 Flash 验证记录

在电机线与舵机线断开的 STM32F411（512 KB Flash）上完成以下验证。
ST-Link 报告低目标电压，但芯片连接、固件烧录、整段读回和下述保存测试均成功。
测试用 `RelWithDebInfo` 镜像的 BIN 与合并前 Release BIN 完全一致，SHA256 为
`339ab862d2137a5b72ae082855847a2b84c7aa87a674646bdd6a5be9d9a71581`。

- 烧录前备份完整 512 KB Flash；烧录后逐字节核对固件，并确认参数扇区未被覆盖。
- 通过 SWD 将文本送入现有 UART 接收队列，由正常命令任务执行 15 个参数的修改和 `save`。
  此次未验证 PC 串口、蓝牙或 nRF 的实际传输链路。
- 读回参数扇区，独立使用 Python `zlib.crc32` 核对记录 CRC、提交标记与全部 15 个字段；
  重复 `save all` 后整个参数扇区保持相同。
- 修改部分 RAM 参数但不保存，复位后全部恢复到上次保存值。
- 用户断开全部供电并重新上电后，重新连接 SWD；确认 Flash 逐字节不变，
  RAM 中全部 15 个参数自动恢复，`Have_saved_parameters=true`。
- 随后恢复测试前默认参数，执行 `save` 并再次复位核对；最终 `unsaved=false`，
  重心基准 9.5°、姿态 Kp 基准 75.35、腿高 44.5 mm、横滚目标 0。
  当前会话最后发送了 `control off`；该开关不持久化，重新上电仍按正常启动条件使能。

本机原始备份、GDB 脚本、Flash/RAM 读回文件和结果 JSON 位于
`build/flash-persistence-20260913/`，其中 `validation-summary.json` 汇总验证结果。
本次没有实车负载测试，也没有在实板上执行日志写满后的 `save recycle` 掉电测试。
