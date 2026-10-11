# FR3C 原厂伺服与目标跟随

`servo_probe` 是第三步的独立测试程序，一个 C++ 程序拥有一个原厂 SDK 连接，同时读取反馈和执行固定姿态平移。使用项目固定的 SDK 2.3.2，直接调用 `ServoMoveStart → ServoCart → StopMotion → ServoMoveEnd`；退出后关闭连接。没有视觉跟随、模式切换、自动使能或故障复位。

首次构建：

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
colcon build --packages-select fr3c_follow_driver --symlink-install
```

默认只读检查，打印实际法兰、工具/工件坐标、模式和能否开始运动：

```bash
scripts/target_follow/servo_demo.sh
```

不连接机器人验证软件流程：

```bash
scripts/target_follow/servo_demo.sh --mock --move --axis x --distance-mm 5
scripts/target_follow/servo_demo.sh --mock --move --axis y --distance-mm -5 --stop-after-s .5
```

Mock 默认 ROS 域 189，与相机和机械臂的域 187 分开；报告会明确写 mock，不代表实机运动验证。

实机一次平移，默认速度 5 mm/s、候选周期 8 ms：

```bash
scripts/target_follow/servo_demo.sh --move --axis x --distance-mm 5
```

方向是机器人基坐标 X/Y/Z，距离可以为正或负。一次测试上限 10 mm；固定旋转时，法兰和夹爪 TCP 的平移量相同。程序核对当前工具 0、工件坐标 0 均为零变换，指令控制法兰，不依赖未知的控制器夹爪工具表。

机械臂需处于自动模式、已使能且没有故障，其他机器人程序停止。关闭原只读反馈/预览的启动终端，让本程序提供反馈；相机和标签检测可以继续运行。脚本检查同 ROS 域的反馈发布者，实际执行时还需确保其他机械臂运动客户端已经退出。

程序从最新实际位置开始，以五次平滑曲线生成小步绝对参考。速度和加速度由轨迹控制，而非 SDK 暂不开放的 acc/vel 字段。每次指令推进一个周期，调用变慢时减慢推进，避免补发积压命令。

到终点后保持 0.5 秒，再调用 StopMotion/ServoMoveEnd。停止方式：

- 当前终端 Ctrl-C。
- 同 ROS 域另一终端运行 `scripts/target_follow/stop_servo.sh`。
- 测试中途停止：追加 `--stop-after-s .5`。

停止请求会在当前 SDK 调用返回后处理。停止服务返回表示请求已接收，实际 SDK 停止结果以状态和报告为准；物理急停仍用于软件无法响应的情况。

每次运行在 `evidence/target_follow/stage3/` 新建目录，保存逐次命令和反馈 `samples.jsonl`、汇总 `report.json`。报告包括实测发送间隔、调用耗时、位移、旋转变化和停止返回码。即便 SDK 返回成功，也要查看实际位移；mock 和只读检查不会写成实机通过。

`servo_probe` 发布现有 `/target_follow/flange_pose`、`/target_follow/robot_feedback` 和法兰 TF，另外发布 `/target_follow/servo_state`、提供 `/target_follow/servo_stop`。模拟发布只能用于模拟域。第四步的 `position_follow` 复用同一套 SDK 客户端和停止清理，接入位置目标。

第三步已实机验证三轴各 5 mm 往返、保持和停止服务。实际轴向位移约 4.99–5.03 mm；发送间隔中位数约 9.7–10.0 ms，SDK 的 8 ms 参数不等于实机严格 125 Hz。测试完成后机器人停止，已返回起始位置附近。汇总位于 `evidence/target_follow/stage3/live_summary.json`。

选项见 `ros2 run fr3c_follow_driver servo_probe --help`。详细实机结果见开发记录；第四步运行方法如下。


## 第四步：固定姿态 XYZ 跟随

`position_follow` 是第四步新增的 C++ 执行程序。它独占原厂 SDK，提供法兰反馈，订阅 `/target_follow/tag_pose_base`，在自己的伺服循环中计算位置误差、滤波和速度/加速度限幅。不会把第二步的 50 Hz `preview_reference` 当作执行指令。

坐标换算继续使用已验证的 `follow_preview`，在 `transform_only=true` 时只做图像时刻的反馈插值、手眼/光学坐标变换和实际 TCP 显示。偏移、固定旋转与期望 TCP 都由执行端管理。软件 TCP 使用当前标定；原厂指令仍用工具 0、工件 0 的法兰目标。

首次构建：

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
colcon build --packages-select fr3c_follow_driver fr3c_target_follow --symlink-install
```

先关闭第二步的反馈/预览 launch 和第三步的伺服程序，避免多 SDK 所有者。相机和标签检测可保留。三个终端：

```bash
# 终端 1：相机和标签检测；已有该链路时继续复用
cd /home/industio/fairino_ws
scripts/target_follow/start_pose.sh
```

```bash
# 终端 2：坐标换算和真实执行端，启动后先待机
cd /home/industio/fairino_ws
scripts/target_follow/start_follow.sh
```

```bash
# 终端 3：显示和操作
cd /home/industio/fairino_ws
scripts/target_follow/view_pose.sh
```

机械臂处于自动模式、已使能且没有故障；当前工具 0、工件 0 为零变换。标签在画面内、末端处于合适位置后，查看器按 **S** 开始跟随。S 会重新记录当前位置偏移和固定旋转；**X** 主动停止；**T** 在停止时单独记录偏移。Q/Esc 只关闭查看器，要停止运动先按 X。启动程序不会自动开始运动。

也可在另一终端操作：

```bash
cd /home/industio/fairino_ws
scripts/target_follow/follow_control.sh start
scripts/target_follow/follow_control.sh stop
# 仅记录偏移，不运动；start 时仍会重新记录
scripts/target_follow/follow_control.sh teach
```

默认域 187。已有 `stop_servo.sh` 仍可请求停止；执行端终端 Ctrl-C 会停止伺服并关闭连接。直接调用停止服务返回的是已接收请求；`follow_control.sh stop` 会继续等待驱动停止状态并显示 SDK 停止结果。SDK 返回码也可在 `/target_follow/follow_state` 和日志中查看。开始服务返回成功表示已记录偏移并成功调用 ServoMoveStart，后续跟随状态仍需观察。

当前参数在 `src/fr3c_follow_driver/config/position_follow.yaml`：Kp 2.0/s、最大速度 20 mm/s、最大加速度 60 mm/s²、目标低通时间常数 0.10 s；目标超时 350 ms、反馈超时 200 ms。跟随范围是每次开始时实际 TCP 周围半径 1.00 m（保留现场修改值），按现场可用空间调整 `follow_radius_m`。这些是低速 Demo 起点，不要求额外误差或帧率指标达标。

目标丢失超过超时后调用 StopMotion/ServoMoveEnd，状态变为 `paused_target`，不继续推进旧目标。新鲜目标重新出现且仍在范围内时保留原偏移、重置滤波和速度，从实际位置平滑接续。显式停止后不自动恢复，下次 S/start 重新记录偏移。越界、反馈过期、控制器故障或 SDK 错误进入 `fault` 并停止；处理原因后重启执行端。查看器会显示具体 Reason；范围越界显示 `FOLLOW RANGE LIMIT`。范围判断的是目标对应期望 TCP 相对开始位置的三维直线距离，组合轴移动也按这个距离计算。软件不切模式、不使能、不复位控制器。

SDK 周期参数继续为 8 ms；每次参考推进最多一个周期，调用变慢时不会补发积压指令。实际发送周期由现场日志判断，不宣称固定 125 Hz。参考持续积分，但误差来自实际 TCP；复用第三步的 15 mm 参考超前检查，避免反馈没有跟上时参考持续跑远。

新增输出和服务：

| 名称 | 内容 |
| --- | --- |
| `/target_follow/follow_state` | idle / following / paused_target / paused_collision / stopped / fault，实际和期望 TCP、误差、速度、观测年龄、停止返回码 |
| `/target_follow/desired_tcp_pose` | 执行端的期望夹爪 TCP，固定旋转 |
| `/target_follow/servo_state` | 与 follow_state 相同，兼容已有停止观察 |
| `/target_follow/follow_start` | Trigger，重新记录偏移并开始跟随 |
| `/target_follow/teach_offset` | Trigger，停止时只记录当前偏移 |
| `/target_follow/servo_stop` | Trigger，请求停止 |

每轮记录位于 `evidence/target_follow/stage4/日期时间_PID/`，包含 `samples.jsonl`、`report.json`、ROS 日志。采样记录实际反馈、期望 TCP、误差相关位置、速度、图像时间戳年龄、SDK 命令和返回码。报告中的 `live_follow_validated` 默认 false，实机演示结果单独记录，不以软件内部数值自动宣称通过。

不连接机械臂的启动检查：

```bash
scripts/target_follow/start_follow.sh mock:=true
```

Mock 默认域 189，不自带模拟视觉目标；单独启动会保持 idle。完整自动模拟验证使用隔离域，包含三轴、图像时刻坐标链路、组合移动、遮挡恢复、重新开始和越界停止：

```bash
source scripts/camera/activate.bash
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 FR3C_RUN_ROS_TESTS=1 \
  ROS_LOG_DIR="$PWD/evidence/target_follow/stage4/test_ros_logs" \
  python3 -m pytest -q src/fr3c_follow_driver/test/test_follow_mock.py
```

第四步原有位置跟随已由用户确认实机效果可用；首轮目标位移约 100.3 mm 时触发当时的 100 mm 软件范围停止，SDK 停止返回 0，当前配置半径为 1.00 m。第五步已整理为统一 Demo，不要求额外一分钟演示；日常操作见 [Demo 使用说明](../../docs/目标跟踪Demo使用说明-20261008.md)。模拟证据位于 `evidence/target_follow/stage4/mock_integration/`，不等同于机械臂跟随实测。


遇到范围 fault 时，在执行端终端 Ctrl-C 退出，再运行 `start_follow.sh`；方块处于合适位置后按 S，新一轮会重新记录偏移和范围中心。需要更大演示范围时，依据现场空间调整 `config/position_follow.yaml` 的 `follow_radius_m`，然后重启执行端加载；速度和加速度可以沿用当前低速设置。

## 第四步追加：轻量防碰撞

模型/桌面碰撞检查为启动时可选，现场卡顿复测后默认恢复之前的跟踪流程。`start_follow.sh collision_enabled:=false` 不加载模型、不启动检查/逆解线程；`collision_enabled:=true` 开启新增检查，接近障碍减速、进入预测余量时暂停。`start_demo.sh` 支持同一参数。命令行覆盖参数文件，不传时读取配置；切换需停止并重启，启动后参数只读。状态始终报告 `collision.enabled` 和 `tracking_mode`，关闭模式不进行模型自碰撞/桌面预检；原有 SDK 故障、反馈超时和停止处理保留。当前 ServoCart、速度/范围和开启模式算法保留。详细步骤见 [使用说明](../../docs/目标跟踪轻量防碰撞使用说明-20261008.md)。

开启模式尤其 Z 上升时曾出现频繁启停，模式开关没有修复此前卡顿；当前默认 `collision_enabled: false`。日常确认模式可运行 `follow_control.sh status`，或看查看器的 `Model collision: OFF/ON`。

## 第八步：独立小角度 TCP 姿态伺服

新增 `rotation_probe`，沿用唯一 SDK 所有者、ServoCart 和停止路径。默认只读；显式 `--move` 后从当前实际 TCP 开始，绕基坐标轴缓慢旋转并往返，默认 2.5°、3°/s、10°/s²。完整 `T_flange_tcp` 每周期换算法兰平移和旋转，包含非零 TCP 旋转标定；内部使用 SO(3)，内部选择邻近上一指令的固定轴 RPY，SDK 绝对位姿请求将各角度规范到 ±180°，两种表达保持相同旋转。启动前采样原厂参考逆解，运动中记录关节和反馈，保留异常停止。

```bash
/home/industio/fairino_ws/scripts/target_follow/rotation_demo.sh --mock --move --axis x --angle-deg 2.5
# 现场先关闭其他 SDK 客户端，只读检查后再逐轴执行
/home/industio/fairino_ws/scripts/target_follow/rotation_demo.sh
/home/industio/fairino_ws/scripts/target_follow/rotation_demo.sh --move --axis x --angle-deg 2.5
```

`--axis x|y|z|xyz` 指基坐标轴；`xyz` 是对角组合轴。默认往返，可选 `--one-way`。Ctrl-C、既有 `stop_servo.sh` 和 `--stop-after-s` 都会停止，中断后不自动回位；下一轮从新的实际位置开始。日志在 `evidence/target_follow/stage8/`，保存完整 TCP/法兰、关节、参考逆解、调用节拍及停止结果。

当前不消费视觉、不启动模型碰撞预测。Mock 只验证流程与几何，逆解/关节不是机构仿真。2026-10-10 首轮 X 轴 2.5°实机往返已通过，用户确认运动，516 条伺服返回 0，TCP 计算位移最大约 0.204 mm，正常结束停止返回 0；随后 Y 轴日志正常完成；Z 轴首轮因跨 180°的 SDK 表达在预检返回 112，已修复绝对位姿编码，随后两轮 Z 实机往返通过，用户确认无问题。随后 2°组合往返及预定中途停止也正常完成，当前第八步 Demo 检查范围已完成。停止轮实际转角约 1.28°/1.23°，程序按预期停在中途，停止返回 0 且反馈确认；不要求重复或精密测量。以明显平滑的动作与可停止为 Demo 标准，无需逐度准确。详细命令、参数和报告说明见 [第八步操作](../../docs/目标跟踪第八步姿态伺服操作-20261010.md)。

## 第九步：视觉六自由度位姿跟随

新增 `pose_follow`，从 `/target_follow/pose_observation` 读取同一图像整数时间戳配对的可信完整目标，自己持有 SDK 反馈与伺服，按实际 TCP 的位置和 SO(3) 旋转误差生成连续参考。每周期通过完整 `T_flange_tcp` 换算法兰位置和姿态，再使用第八步的 SDK 绝对位姿编码。旧 `position_follow` 继续可用。

```bash
/home/industio/fairino_ws/scripts/target_follow/start_pose_follow.sh
```

启动待机；T 只记录、S 从当前目标和实际 TCP 重新记录完整关系并开始、X 主动停止。默认 0.8 s 姿态滤波、6°/4°旋转死区、0.5 旋转映射（位置绕转同步减幅），以允许残差和延迟的稳定 Demo 为标准。角速度/加速度为 3°/s、10°/s²；按最新用户要求独立位置范围为 1 米，转角为 90°，参考超前限制 15 mm/5°。参数见 `config/pose_follow.yaml`。

第九步检测使用显式 35°候选旋转差异容差，质量元数据保留 `orientation_approximate` 和容差；Python 质量检查与 C++ 驱动一致接受该近似策略，未声明双解已正确消除。没有新字段时仍使用旧 5°规则。T/S 允许短时软状态下使用 350 ms 内的最后可信目标；硬拒绝、丢失和过期仍不能开始，自动暂停恢复仍需新可信图像。三轮诊断与修复回放见 [现场调参](../../evidence/target_follow/stage9/field_tuning_20261010/README.md)。

后续倾斜现场会话因观测跳变暂停，再因原始恢复转角 20.24°超过旧 20°阈值而等待重记录。该次修正将第九步原始恢复转角容差设为 40°，基坐标观测抖动余量由独立配置放宽到 25°；当时半幅映射、滤波/死区、角速度和 20°实际运动范围生效。后续恢复从实际 TCP 开始，保留原关系。诊断与新 Mock 倾斜/恢复验证见 [倾斜调参](../../evidence/target_follow/stage9/tilt_tuning_20261010/README.md)。

短时软歧义保持上次完整目标，但不刷新可信图像时间戳；丢失、硬拒绝、过期或反馈异常结束伺服并暂停。反馈与新的可信观测恢复后，从实际 TCP 重新锚定并清零速度，保留原关系。大幅重定位或原始目标越界提示 S 重新记录；主动 X 停止后不自动恢复。可选 `collision_enabled:=true` 使用完整旋转扫掠、参考超前和角制动预测，默认关闭。

初版验证：36 项 C++、40 项 Python、8 项新驱动 ROS 模拟、1 项完整入口模拟及 6 项原位置跟随回归通过，1 项 GUI 跳过。三轮现场诊断后的修复已重新构建安装：37 项 C++、60 项 Python、10 项驱动 ROS 模拟和 1 项入口模拟通过，1 项 GUI 跳过。本轮执行验证使用 MockClient，真实命令为 0；更新后的真实视觉效果及现场旋转防碰撞待复测。详情见 [第九步操作](../../docs/目标跟踪第九步位姿跟随操作-20261010.md) 和 [验证证据](../../evidence/target_follow/stage9/README.md)。

最新现场 `20261010_140121_208977` 的 14049 条 ServoCart 全返回 0，最后半幅期望转角 20.36°超过旧 20°限制而暂停。当前范围为 500 mm/90°，暂停恢复差值 150 mm/120°。新增完整目标锁存与到位保持：4 mm/2°内固定参考、按既有加速度减速；滤波期望 TCP 从固定视觉基准累计超过 8 mm 或 4°且新可信帧同向确认 0.2 s 后才更新。锁存覆盖姿态引起的位置绕转，滤波尾差和反馈微抖动不会持续重开矫正。`settled_hold`、`goal_pending` 随状态及伺服日志保存，窗口显示 POSE HOLD。软件与 Mock 验证见 [范围/保持修正](../../evidence/target_follow/stage9/range_hold_tuning_20261010/README.md)，更新后的实机效果待确认。

随后按用户要求将 `follow_radius_m` 改为 1.00 m，默认值与 YAML 一致，速度仍为 20 mm/s、3°/s。最新现场最终 SDK 112（目标位姿无法到达），没有触发旧软件半径；扩大软件范围不能改变真实可达空间。日志诊断与本次构建见 [1 米区域修正](../../evidence/target_follow/stage9/radius_1m_20261010/README.md)。
