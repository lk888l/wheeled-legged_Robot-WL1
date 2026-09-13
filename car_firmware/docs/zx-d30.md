# ZX-D30 接线、微信遥控与验证（2026-09-13）

## 当前配置

| 项目 | 本轮实测 / 配置 |
| --- | --- |
| 模块版本 | `ZX-D30_V1.2.7`，实际 UART 查询回包 |
| 广播名称 | `D30SP_126BB2`（名字含 SP，但已实测支持 BLE GATT） |
| 地址 | `C8:47:80:12:6B:B2` |
| UART | **9600 8N1**，无流控；`AT+BAUD?` 返回 `OK+G_BAUD=3` |
| 模块 TX → MCU RX | PA10 / USART1_RX |
| 模块 RX ← MCU TX | PA15 / USART1_TX |
| GND | 共地 |
| 服务 | `0000FFE0-0000-1000-8000-00805F9B34FB` |
| 写入 | FFE2（推荐）或 FFE1，均实测支持 write / write without response |
| 返回通知 | FFE1，两种写入方式均订阅此特征 |
| FFE3 | 资料单 BLE 版本的 IO 控制通道；本机未暴露，`AT+IUUID?` 返回 ERROR |

使用现有四根串口线即可；STATE/KEY 不参与固件启动判断。
用户提供的底板手册标注 VCC 3.2–6 V，裸模块技术手册标注 1.8–3.6 V、建议 3.3 V，
串口为 3.3 V TTL；供电应根据实际是否带底板区分。
本次没有改动模块名称、波特率、UUID 或恢复出厂设置。

## 固件变更

- CMake、生成模板、头文件回退值和 CubeMX `.ioc` 默认波特率同步为 9600。
- 在外设初始化后、调度器启动前等待 MCU 上电满 300 ms，满足模块启动阶段不接收 UART 数据的要求。
- 丢弃模块 `OK`、`ERROR` 等应答，避免未知命令回显与模块 AT 错误形成循环；这些应答不延长运动指令寿命。
- 复用 `@<命令>\n` 分帧，正文最多 32 字节、300 ms 收齐；兼容原来的单块短命令。
- 复用 500 ms 运动指令超时归零，保留腿高。平衡计算、IMU 门控和 PID 算法保持原逻辑。

构建时显式指定波特率，避免已有 CMake 缓存保留 115200：

```sh
cmake -S car_firmware -B car_firmware/build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release -DWL1_COMMAND_UART_BAUD=9600 -DWL1_ENABLE_NRF24=OFF
cmake --build car_firmware/build/Release --parallel
```

如将模块另行配置为 115200，可用 `-DWL1_COMMAND_UART_BAUD=115200`；本次模块保持出厂 9600。
AT 查询仅在未连接时可用，本轮使用实际 CRLF 字节，版本查询是 `AT+VERS?\r\n`。
固件正常启动不自动发 AT 配置命令。

## 微信小程序

项目：`D:/kk/wechat-app/cf-studio_wechat-app`。

1. 用微信开发者工具打开该目录，在手机上预览 / 真机调试。
2. 选择 **WL1 · SoftEngine**（页面初始仍是旧 main，需要手动切换）。
3. 关闭 SIM，进入连接页搜索 **D30SP_126BB2**。
4. 选择 FFE0 服务的 **FFE2** 写入通道，返回通知为 **FFE1**；FFE1 写入也已测试。
5. 启用本地控制，再操作摇杆；松手归零，断连后固件在 500 ms 超时归零。

小程序按最多 20 字节、25 ms 片间隔串行发送，100 ms 周期更新控制目标。
本次修正 FFE2→FFE1 通知配对、排除 FFE3 IO 通道、支持短 UUID 与完整 UUID 等价比较，
并更新模块名称和接线提示。未知模块仍保留通用服务发现流程。
小程序的本地控制开关不等于固件解除 IMU 姿态门控。

## 验证结果

| 验证 | 结果 |
| --- | --- |
| STM32 编译 | RelWithDebInfo / O3，通过；Flash 247,640 B，静态 RAM 47,664 B |
| 主机 C++ 回归 | 45/45 通过，含新增 AT 应答不回显、不刷新运动时间测试 |
| 小程序 | 格式、严格 TypeScript、76/76 测试；wcc / wcsc 模板编译通过 |
| ST-Link 烧录 | 写入、verify_image 通过；247,640 字节读回与最终构建逐字节相同 |
| 真实 UART | 版本、名称、波特率、地址、FFE0/FFE1/FFE2 查询收到真实回复 |
| 真实 BLE | Windows BLE → ZX-D30 → STM32；FFE1/FFE2 两种写入均收到 `pong state=ready control=on` |
| 返回分包 | FFE1 多个通知片段组合得到完整 `controlstate` |
| 真实 BLE 分片调参 | `@anglepid -p 7` + `5\n` 后 Kp=75；超时 400 ms 的 `9` + `0\n` 尾包不执行 |
| 真实 BLE 运动帧 | 22 字节 `@R -100 -100 -18 78.5\n` 按 20+2 拆包，状态回传目标 -100/-100/-18 |
| 无命令超时 | 停止发送 700 ms 后，状态回传 remote_timeout=1，三个运动目标归零 |

无线测试使用电脑 BLE 适配器，**未冒充手机微信真机测试**。微信 API 测试使用模拟适配器，
手机权限、微信版本及触屏遥控还需在手机实际运行。用户已断开电机线和舵机线，
因此不包含带负载的自平衡性能验证。测试时机器倾斜，IMU 有效但 armed=false，PWM=0，未绕过门控。

所有产物、原 Flash 整片备份、UART/BLE 日志位于 `car_firmware/build/zx-d30-20260913/`（Git 忽略）。
最终镜像：`firmware/WL1_F411CEU6.bin`。
SHA-256：`d948bf3700341664c56955fc5b5daebc7b8071aa089e74766911a0065c141c4d`。
原 Flash 为 524,288 字节，已备份为 `original-flash.bin`；原镜像与此前保存的构建产物不一致。
ST-Link 电压低提示不影响本轮已完成的烧录和读回。

## 最终板上检查

- 发送非零目标后断开真实 BLE：SWD 读到最后请求仍为速度 20 / 转向 10 / 横滚 4、腿高 61.5，实际三个运动目标为零、remote_timeout=true，证明已收到最后一帧且断连后超时生效。
- 断连检查时累计 UART 接收错误计数为 1、丢包计数 0，接收已恢复；混合调试会话中未定位该单次错误的准确来源，不将整个测试过程描述为零错误。
- 随后复位清除全部临时运动和 PID 参数；首条 UART DMA 发送时 uwTick=301 ms。
- 复位运行 20 s：ready，硬件失败=0、任务失败=0、1997 次控制循环、最大周期 10 ms、超期 0、UART 错误=0、丢包=0、CFSR/HFSR=0。
- 最终参数 Kp=70、自动腿高 Kp 模式开启、腿高 44.5、速度/转向/横滚均为零。可用 RTOS heap 11,032 B，最低 8,552 B。
- 调试器已脱离、主控继续运行；电脑 BLE 已断开，可供手机微信连接。复位测试不等价于重新接通全部电源的冷启动。
