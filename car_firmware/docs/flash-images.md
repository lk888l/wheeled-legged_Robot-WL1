# 首次烧录、更新与参数保留

STM32F411CEU6 的程序使用 `0x08000000..0x0805FFFF`（384 KiB，扇区 0–6），
运动参数日志使用 `0x08060000..0x0807FFFF`（128 KiB，扇区 7）。

## 选择烧录方式

| 芯片当前内容 | 使用方式 | 烧录后参数 |
| --- | --- | --- |
| 空芯片或内容未知、其他工程固件 | factory | 当前编译默认值，已写入 Flash |
| 已安装 WL1，需要保留调参 | update | 原扇区 7 完整保留，启动加载最新兼容有效记录 |
| 已安装 WL1，需要恢复出厂 | factory | 原记录全部清除，恢复当前编译默认值 |

factory 会清除原有调参。普通更新不要使用全片擦除，也不要误选 factory 镜像。

在 `car_firmware` 中构建；默认构建同时生成两种 HEX/BIN，不会自动连接设备：

```sh
cmake -S . -B build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release --parallel
```

两种烧录目标会先构建，再按固定扇区范围擦除、写入、校验和复位运行：

```sh
# 首次安装 / 其他固件转入 / 恢复出厂
cmake --build build/Release --target flash_factory

# 保留参数更新
cmake --build build/Release --target flash_update
```

配置阶段需要找到 OpenOCD 才会提供烧录目标；不安装 OpenOCD 仍可生成所有镜像和脚本。
也可直接执行 `openocd -f STlink.cfg -f build/Release/flash_factory.cfg`，
更新则把最后的脚本改为 `flash_update.cfg`。
已确认 Vref 误报的探头使用 `STlink_hla.cfg`；本次连接的 V2J35M26 已通过此配置验证。
使用其他 OpenOCD 路径时设置 `WL1_OPENOCD_EXECUTABLE`，使用其他配置时设置
`WL1_OPENOCD_CONFIG` 为配置文件绝对路径。

## 产物与擦除要求

| 文件 | 内容 / 用途 |
| --- | --- |
| `WL1_F411CEU6_update.hex`、`_update.bin` | 只有程序区，默认更新产物 |
| `WL1_F411CEU6_factory.hex` | 程序区 + 地址 `0x08060000` 的一条 84 字节默认记录 |
| `WL1_F411CEU6_factory.bin` | 从 `0x08000000` 开始的连续镜像，中间空洞填 `0xFF`，总长 393300 字节 |
| `WL1_F411CEU6_defaults.bin` | 构建中间产物，仅一条记录，不是完整固件 |
| `WL1_F411CEU6.elf`、`.hex`、`.bin` | 保持旧用法；均不含参数扇区内容 |
| `flash_factory.cfg`、`flash_update.cfg` | 与对应 HEX 放在同一目录的 OpenOCD 烧录脚本 |

使用其他烧录器时，HEX 自带地址；两个完整 BIN 的烧录起始地址都是 `0x08000000`。
factory 必须先擦除扇区 0–7；update 只擦除扇区 0–6。不要写入 MCU option bytes。

HEX/BIN 不包含“擦除哪个扇区”的指令。当前日志按物理位置选择最后一条有效记录，
只把默认记录写到槽 0 会留下后面的旧有效记录，启动仍可能加载旧参数；
因此 factory 脚本明确执行 `flash erase_sector 0 0 7`，然后写入及校验镜像。
update 脚本使用 `flash erase_sector 0 0 6`。
OpenOCD 的自动 `write_image erase` 还可能擦除镜像区段之间的空洞，详见
[OpenOCD Flash Commands](https://openocd.org/doc/html/Flash-Commands.html)。

## 默认值与运行时行为

默认记录由 ARM 编译器从 `MotionSettings::Parameters{}` 生成，使用与运行时 `save`
相同的记录编码、版本和 CRC 函数，不另外手填一份参数或 CRC。RAM 启动参数也使用
同一份定义。当前记录为 v3、序号 1、自动角度 Kp 模式、共享电机死区 0；
重心基准 9.5°、角度 Kp 基准 75.35、腿高 44.5 mm、横滚目标 0。

factory 首次启动应显示 `flash_valid=1 unsaved=0`。未修改参数就执行 `save` 返回
`unchanged`；修改后保存会追加下一条记录，复位后恢复已保存值。

update 烧到全空芯片也能启动：没有有效记录时使用编译默认值，`flash_valid=0`，
第一次 `save` 写入空白槽。启动本身不会写 Flash。
但 update 不清理其他固件遗留的数据：若参数区被占满，普通 `save` 会返回 `full`；
若还存在旧的兼容有效记录，启动会加载它。内容未知的芯片应选 factory。
既有设备日志写满时，支撑车体并停止控制后可显式执行 `save recycle`。
普通保存仍只追加，不在运动中自动擦除整个扇区。

## 验证

```sh
python tests/check_firmware_images.py build/Release
```

此检查独立解析 HEX、验证地址边界、默认记录 CRC/提交标记、factory 空洞填充、
兼容文件名及脚本擦除范围。主机 `motion_parameters_test` 还覆盖 factory 记录加载、
首次保存、旧固件占满日志、尾部旧记录覆盖默认记录的反例和完整擦除后的恢复。
实板记录见 [2026-09-19 烧录验证](flash-images-validation-2026-09-19.md)。
