# 喜德盛 AD350 功率计转发

这是一个基于 nRF52832 的喜德盛 XDS 功率计转发固件。设备通过 BLE Central 连接喜德盛私有功率计，解析真实功率和踏频数据，再通过 ANT+ Bicycle Power Profile 广播出去，让支持 ANT+ 功率计的码表可以发现并读取数据。

## 从 Release 下载固件

打开 GitHub 仓库的 `Releases` 页面，下载最新版本中的固件产物。

推荐直接下载：

```text
xds_ble_to_ant_bpwr_s332_firmware.zip
```

解压后需要用到两个 hex 文件：

```text
ANT_s332_nrf52_7.0.1.hex
xds_ble_to_ant_bpwr_s332_app.hex
```

其中：

- `ANT_s332_nrf52_7.0.1.hex` 是 BLE 和 ANT 协议栈
- `xds_ble_to_ant_bpwr_s332_app.hex` 是本项目应用固件

如果你不下载 zip，也可以在 Release 页面分别下载这两个文件。

## 使用 nRF Connect Programmer 烧录

1. 打开 `nRF Connect for Desktop`
2. 进入 `Programmer`
3. 连接 J-Link 或开发板调试器
4. 点击 `Erase all`
5. 点击 `Add file`
6. 添加 `ANT_s332_nrf52_7.0.1.hex`
7. 再次点击 `Add file`
8. 添加 `xds_ble_to_ant_bpwr_s332_app.hex`
9. 确认内存布局中有两段固件
10. 点击 `Write`
11. 烧录完成后点击 `Reset`

正常情况下，SoftDevice 会位于 Flash 起始区域，应用固件会从 `0x30000` 附近开始。

## 使用 nrfjprog 烧录

如果使用命令行，可以执行：

```powershell
nrfjprog -f nrf52 --eraseall
nrfjprog -f nrf52 --program ANT_s332_nrf52_7.0.1.hex --sectorerase --verify
nrfjprog -f nrf52 --program xds_ble_to_ant_bpwr_s332_app.hex --sectorerase --verify
nrfjprog -f nrf52 --reset
```

## 使用方式

烧录完成后，让喜德盛功率计保持唤醒状态。设备会尝试通过 BLE 搜索并连接功率计，收到功率数据后转换为 ANT+ Bicycle Power 广播。

在码表上添加传感器时，请选择 ANT+ 功率计。不要选择蓝牙功率计，因为本项目不会把数据重新通过蓝牙广播出去。

## 注意事项

- 喜德盛功率计通常只能同时连接一个 BLE Central
- 测试时请关闭手机 App 或原码表对功率计的蓝牙连接
- 码表搜索不到 ANT+ 设备时，先确认两个 hex 都已经烧录
- 只烧录应用固件无法运行，必须同时烧录 S332 SoftDevice
- 当前工程目录名中仍有 `s212`，但实际构建和链接使用的是 S332
