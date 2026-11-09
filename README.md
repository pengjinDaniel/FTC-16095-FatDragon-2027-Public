# FTC 16095 FatDragon

上传前，在 Android Studio 构建 `TeamCode` 的调试 APK，并确认电脑已安装 Python 3.8 或更新版本和 Android SDK Platform-Tools。

- macOS 可以双击 `无线传程序.command`，也可以在终端运行 `python3 upload-offline.py`。
- Linux 在终端运行 `python3 upload-offline.py`。Windows 在终端运行 `py -3 upload-offline.py`。
- 无线方式先连接 Control Hub 的 Wi-Fi，再选 `1`。用 USB 数据线时选 `2`。核对目标设备后输入 `y` 安装。
- 加上 `--check` 可检查本机的 ADB 和 APK，不连接机器人。

工具会从 `PATH`、Android SDK 环境变量、项目的 `local.properties` 或系统默认的 SDK 目录查找 ADB。
