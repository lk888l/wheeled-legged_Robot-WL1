# Flash 保存与模块化固件的合并兼容说明

> 本文记录合并时的行为。后续普通 save 已改为保持平衡运行，只有 recycle 仍要求停机；
> 当前实现见 [运行时 Flash 保存](runtime-flash-save.md) 和 [命令参考](commands.md)。


本次合并连接本地 `bd0955b`（Flash 保存）与远端 `ed66a59`（模块化运行控制）。
两者都基于 `9b58e9e`，一边仍修改旧 main.cpp，一边已把业务迁移到独立任务，
因此不能整文件选择 ours 或 theirs。

## 保留与适配

- 保留远端 BoardHardware/AppTask/ControlState 架构、BLE @ 分帧、9600 默认波特率、
  可选 nRF、500 ms 遥控超时、故障锁存和遥控器串口调参桥接。
- 将本地 save/params/control 命令接入 CommandServiceTask，通过 MotionPersistence
  显式保存运动参数；在启动控制任务前恢复。重心默认基准保留本地最后标定的 9.5°。
- `anglepid -p` 调整 61.5 mm 基准并启用线性腿高补偿；`-manual <值>` 保留固定 Kp
  能力；`-auto` 启用补偿但不重置基准。小程序常规 P 调参仍使用 -p。
- Flash 扇区地址、84 字节记录长度和 15 个浮点字段顺序不变。v2 增加模式位；
  后续 v3 在版本字高位保存双轮共用电机死区，继续兼容读取 v1/v2，旧记录使用
  默认死区 0。无有效记录才使用全部默认值。
- 保存抢占解锁周期、瞬间完成保存及连续 control off/on 都重新触发 500 ms 稳定窗口。

完整协议与小程序保存按钮回包见 [命令参考](commands.md#flash-参数保存)。
遥控器固件与本次远端版本内容一致；串口桥接白名单未扩展为任意命令转发。

## 本次验证范围

- 主机 Debug / Release：各 47 项 CTest 全部通过，并确认 VQF 拒绝 -Ofast。
- 新增 v1/v2 混合读取、模式持久化、真实命令任务保存与恢复、解锁边界抢占回归。
- 车端 Debug、Release（9600 / nRF OFF）及 Release（nRF ON）均成功生成 ELF/HEX/BIN。
  Release 默认配置占用 Flash 267644 B，静态 RAM 47776 B；128 KiB 参数区仍独立预留。
- 遥控器 Debug 编译和串口桥接主机测试通过。
- 本次合并没有重新烧录设备。tests/README.md 中完整断电恢复证据属于合并前 v1 镜像，
  不能视为合并后新镜像的实板验收。

原本地提交保留在 `codex/backup-flash-before-merge`。合并前工作区及 Git 合并状态
备份位于被 Git 忽略的 `build/merge-recovery-20260913/working-state.zip`，同目录保存
构建与测试日志。该 ZIP 用于人工核对和恢复，不应直接覆盖当前 .git。
