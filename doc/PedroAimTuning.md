> 2026-10-13：用户要求暂时停用这套独立 Pedro 测试／调参代码；对应 Java 和主机检查已逐行注释，下面内容仅为历史记录。正式 Blue / Red 使用原 AutoAimController；本次只新增 RT 持续期间固定 CELL，未改其 PID。

# 新增 Pedro 自瞄与调参程序

这些程序独立于原有比赛程序，原有 Drive、Vision、自瞄测试和比赛逻辑保持不变。

手机 Driver Station 的 Test 分组新增：

- **Pedro Aim Tune AUTO**：Start 后自动双向起转探测、双区间 PD 扫描；卡住时在上限内增加摩擦补偿，固定参数通过四次角度测试才推荐。结果不会自动套用。
- **Pedro Aim Tune MANUAL**：手动方向确认和启动扫描。
- **Pedro Aim Blue / Pedro Aim Red**：独立摄像头自瞄测试，按住 RT 瞄准，松开恢复手动；A 重新校准并选择目标；B 停止至 RT 松开。只驱动底盘，不发射。

已载入实际底盘测试选出的参数：近区 P/I/D = 0.012/0/0.002，远区 = 0.025/0/0.003；角误差阈值 5°/20°，功率上限 0.30/0.35，CW/CCW 摩擦补偿 0.091183/0.115504。I 尚未搜索；摩擦补偿不是电机 SDK 的 kF。

摄像头测试在一次 RT 持续期间固定目标坐标，用 Pinpoint 连续角度控制；视觉校准发生在初始化、RT 按下或 A 按下时。D 使用带符号角速度，停止容差带使用回差。此持续瞄准修订已通过主机检查与 APK 编译，尚未完成实机验证；此前版本出现过持续震荡，不能据底盘阶跃通过宣称摄像头自瞄已调好。

调参日志：Robot Controller 的 FIRST/pid-tests/turn-*.csv 与结果文本；摄像头日志：FIRST/pid-tests/aim-*.csv，STOP 保存。Dashboard 的 AimTuneConstants 可查看新增控制参数。

Pedro 依赖固定为 revhub/core 3.0.1；仅启用手动底盘与 Pinpoint 定位，路径动力学未标定，路径跟随与 hold 被拒绝。保留原 Pinpoint -96/-74 mm 设置。

构建验证使用 JDK 17、Gradle 8.13、Android SDK Platform 30。仓库原 Gradle wrapper 配置未修改。local-maven 包含官方 FTC Dashboard 0.6.0 的 AAR、core JAR 和 POM（MIT；来源 https://github.com/acmerobotics/ftc-dashboard 及 https://acmerobotics.z13.web.core.windows.net/），供附加的本地仓库配置解析依赖。local-tests 保存七组独立 Java 主机检查；APK 和本机缓存、设备日志不提交。
