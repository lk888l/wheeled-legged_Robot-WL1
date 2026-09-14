# 运行中保存运动参数

## 数据流

1. 开机时读取 Flash 最新有效记录，将参数恢复到 RAM；没有有效记录时使用编译默认值。
2. 遥控命令只修改 RAM，平衡任务从 ControlState 获取 RAM 快照。不自动保存、不定时保存。
3. save/save all 处理时复制整组运动参数，将快照追加到 Flash。之后断电恢复这组数据。
   写入期间不关闭 control，不清除 PID 历史，不重新等待 500 ms 稳定窗口。
4. 修改后未发送 save 的参数只在本次运行有效。v3 仍使用 84 字节槽，并兼容读取
   v1/v2；旧记录的新增电机死区采用编译默认值 0。

小程序按钮发送 `@save\n`，收到 `save: ok` 或 `save: unchanged` 视为保存成功；
其他回包和保存字段见 [命令参考](commands.md#flash-参数保存)。

## 追加编程与擦除分开处理

STM32F411 的 Flash 操作仍可能短暂延迟指令读取，因此不能把“写空白字”和“擦除
整个扇区”当成同一种操作处理。ST 的 DS10314 Rev 8 表 45 给出的 x32 单字编程时间
为典型 16 us、表列最大 100 us（该最大值为特性评估，非逐片生产测试保证）；128 KiB
扇区 x32 擦除时间为典型 1 s、表列最大 2 s。
参考：[STM32F411 数据手册，第 93 页](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf)。

普通 save 从不擦除，只使用空白的 84 字节槽。每成功编程一个 32 位字（含最后的提交
标记），存储适配层调用 vTaskDelay(1)，在当前 1 kHz RTOS tick 下让出一个调度 tick。
运动任务优先级 29 高于命令任务的 28；Flash 全程不持有 RTOS 临界区，避免连续编程
21 个字形成一段不可抢占的长突发。已保持为全 1 的字也经过相同的让出点。
这使正常编程造成的单次 Flash 读取停顿局限于一个字；不等于硬件完全没有时序扰动，
完整控制负载与通信条件下的实际抖动仍需实板测量。

MotionStorageInterlock 分开记录 busy（防并发写入）与 blocks_control（维护期间禁止
控制）。普通 save 只占用 busy；独占 save recycle 才检查 armed=false 和左右 PWM=0，
并重置稳定门控。若日志已满，普通 save 返回 full，保持平衡且不排队自动擦除。
回收必须另行支撑车体、停止控制后执行 save recycle，不能绑到普通保存按钮上。

记录先写头部与参数、回读校验，再写最终提交字。只有最终回读验证成功才返回 ok。
普通追加中断可回退前一条有效记录；首次保存未完成或显式回收擦除后中断则可能回退默认值。

## 本次验证

- Debug/Release 各 47 项主机 CTest 通过，VQF 的 -Ofast 拒绝检查通过。
- 新增逐字编程必须经过让出点的检查；保存期间真实运动任务仍保持 armed、非零 PWM，
  保存完成不留下控制重置请求。
- 真实命令任务测试覆盖运行中 save、recycle 拒绝、只调 RAM 不写 Flash、日志写满不擦除；
  既有逐字中断、CRC、版本兼容、断电数据恢复模型、control off/on 和故障锁存回归仍通过。
- 车端 Debug、Release、RelWithDebInfo 与可选 nRF Release 均构建通过。默认 Release
  使用 Flash 267684 B、静态 RAM 47776 B，参数扇区继续独立预留 128 KiB。
- 本次尝试连接 ST-Link 返回 open failed；Windows 当前未枚举到 ST-Link/STM32 设备。
  尚未重新烧录或完成运行中保存的实板时序验证，不能把上述模型测试当成闭环实车验收。

构建、测试及连接日志位于 Git 忽略的 build/runtime-save-20260913/。
