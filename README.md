# FTC 16095 FatDragon

确认电脑已安装 Android Studio（或可用 JDK）、Python 3.8 或更新版本和 Android SDK Platform-Tools，首次使用先联网准备好 Gradle 依赖。

- macOS 双击 `无线传程序.command` 会先执行离线 `clean :TeamCode:assembleDebug`，成功后才进入上传。编译失败会停止，不会上传旧 APK。
- `无线传程序.command --build-only` 只编译并检查本地上传文件；`--check` 跳过编译，仅检查，不连接机器人。
- 直接运行 `python3 upload-offline.py` 仍只上传已有 APK，需事先自行构建。
- Linux 在终端运行 `python3 upload-offline.py`。Windows 在终端运行 `py -3 upload-offline.py`。
- 无线方式先连接 Control Hub 的 Wi-Fi，再选 `1`。用 USB 数据线时选 `2`。核对目标设备后输入 `y` 安装。
- 加上 `--check` 可检查本机的 ADB 和 APK，不连接机器人。

工具会从 `PATH`、Android SDK 环境变量、项目的 `local.properties` 或系统默认的 SDK 目录查找 ADB。
