# 三步抓取启动与执行命令

当前流程用于真实 FR3-C、DaBai DCW2 相机和 YDI-J22 夹爪。启动脚本会连接真实机械臂，并解锁
FT232H 夹爪 PWM 输出；执行前必须确认工作区无人、急停及停止方式可用、线缆余量充足，且没有
其他程序或人员控制机械臂。

## 1. 启动完整抓取栈（三种模式三选一）

在终端 1 先加载环境：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

然后根据场景选择且只执行下面一个启动命令。

### 1.1 蓝色单目标模式（默认，兼容正方体和长方体）

```bash
scripts/run_moveit_camera_rviz.sh detection_mode:=blue
```

也可以省略参数，直接运行 `scripts/run_moveit_camera_rviz.sh`。该模式使用蓝色分割，每帧只允许
一个目标，兼容以下两种物体：

- 30×30×30 mm 蓝色正方体；
- 60×30×30 mm 蓝色长方体，60×30 mm 面水平放置。

系统会根据顶面轴比自动区分正方体和长方体，并同步选择对应尺寸、碰撞体和抓取角度对称性。

### 1.2 四色正方体分拣模式

```bash
scripts/run_moveit_camera_rviz.sh detection_mode:=colors
```

该模式识别红、黄、蓝、绿的 30×30×30 mm 正方体，并自动启用 A4 四区分拣。可先使用人工三步
服务验证单块；调用 `/fr3c_first_grasp/start_continuous` 时会按红→黄→蓝→绿连续抓取、放置并
返航。该模式当前只接受正方体，不接受 60×30×30 mm 长方体。详见
[四色分拣说明](four_color_sorting.md)。

### 1.3 YOLO 单目标模式

首次使用时先安装工作区内的 YOLO 运行环境：

```bash
scripts/setup_yolo_runtime.sh
```

安装完成后启动：

```bash
scripts/run_moveit_camera_rviz.sh detection_mode:=yolo
```

YOLO 模式每帧只允许一个目标，会自动使用本工作区的 RKNNLite 环境，不需要指向旧项目。当前
模型只识别旧项目的 16 种垃圾类别，不是通用方块检测器；目标物必须具有对应类别外观。YOLO
提供候选框后，系统仍会使用 RGB-D 顶面几何区分 30 mm 正方体和 60×30×30 mm 长方体。详见
[YOLO 融合说明](yolo_blue_cube_fusion.md)。

`blue`、`colors`、`yolo` 三种模式不能同时启动，切换模式前必须在原启动终端按 `Ctrl+C`，等待
旧节点完全退出，再执行新的启动命令。

该命令会启动机械臂控制、MoveIt 2、RViz、相机、手眼 TF、真实夹爪、所选方块定位、三步抓取
服务和 P6 安全门禁。启动本身不会让机械臂执行抓取；机械臂在收到第一步服务命令后才会运动。

等待启动终端中的 P6 检查输出 `passed=true`，再按所选模式放置目标并保持静止：

- `blue`：放置一个蓝色正方体或一个蓝色长方体；
- `colors`：放置一个或多个红、黄、蓝、绿的 30 mm 正方体；
- `yolo`：放置一个带当前模型可识别类别外观的正方体或长方体。

然后在终端 2 加载环境：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

可选：执行前检查定位和抓取状态：

```bash
ros2 topic echo /perception/blue_cube_localizer/status --once
ros2 topic echo /perception/blue_cube_localizer/top_pose --once
ros2 topic echo /perception/blue_cube_localizer/object_size --once
ros2 topic echo /fr3c_first_grasp/state --once
```

只有定位状态为当前有效、稳定且时间戳新鲜时，才执行第一步。

当前配置会从顶面三维点云自动分类。状态中应看到
`object_class=cube_30mm` 或 `object_class=rectangular_box_60x30mm`，以及对应的
`nominal_object_size_m`、`top_axis_ratio`和 `grasp_axis_yaw_std_rad`（应不大于 `0.0873`，即 5°）。
正方体使用顶面边方向和 90° 对称性；长方体使用长轴方向和 180° 对称性。抓取节点保持夹爪
竖直，自动使用对应的 30×30×30 mm 或 60×30×30 mm 碰撞体。每个等价角都从同一起点
完成接近、下降、提升预规划，再根据真实关节轨迹选择。单轮第六轴累计转角 90°、
距 URDF 限位至少 10° 为默认软偏好；若没有候选满足，仍会选择超出软偏好最少的完整可行路径，
并在日志中标记。URDF 硬限位、碰撞和路径完整性检查不会放宽。轴比位于
`1.25`～`1.55` 的模糊样本也会被拒绝。

首次验证动态 yaw 或新检测模式时，建议先以仅规划模式启动。下面以蓝色模式为例；验证
`colors` 或 `yolo` 时，将最后的模式参数替换成对应值：

```bash
FR3C_FIRST_GRASP_EXECUTE=false scripts/run_moveit_camera_rviz.sh detection_mode:=blue
```

此模式仍会连接真实控制栈，但 `/start` 只生成并显示完整轨迹，不会发送夹爪或机械臂运动命令。
先在 RViz 中用至少 0°、30°、60°、90°四种摆角分别确认正方体/长方体 Marker、夹爪方向和轨迹，再重启为
默认执行模式并使用下面的人工三步流程。首次动态 yaw 测试不要使用连续执行服务。

## 2. 分三步执行抓取

### 一键连续执行（无人工停顿）

多块红、黄、蓝、绿按优先级连续识别并自动放置到 RViz 对应 A4 分区的操作见
[四色分拣说明](four_color_sorting.md)。

确认目标位置正确、整段路径净空且方块能够可靠夹持后，可用一个命令连续完成三步：

```bash
ros2 service call /fr3c_first_grasp/start_continuous std_srvs/srv/Trigger '{}'
```

在 `blue` 和 `yolo` 模式下，该命令只连续完成当前单个目标的接近、下降、闭爪和提升。在
`detection_mode:=colors` 下，该命令会逐块完成抓取、分区放置和返航，返航后重新识别，连续
10 秒没有可处理方块才结束。目标的完整路径在机械臂运动前规划；某块的规划失败会跳过该块，
但机械臂开始运动后的任何执行、夹爪或门禁失败都会终止整批流程。此模式会跳过高位对中和闭爪
后的两次人工确认，仅可在已经验证过目标位置、夹持效果及完整运动路径的条件下使用。

以下命令保留原有的人工分步确认方式。

### 指定颜色单次 Action（大模型／语音层推荐接口）

在 `detection_mode:=colors` 下，可明确指定本次颜色和任务类型。抓起后保持：

```bash
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_hold \
  fairino_msgs/action/PickColor \
  "{color: red}"
```

抓取指定颜色、放入对应分区并返航：

```bash
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_sort \
  fairino_msgs/action/PickColor \
  "{color: blue}"
```

颜色可替换为 `red`、`yellow`、`blue`、`green`。每个 Goal 只执行一次；不存在或不唯一时返回错误，
不会自动改抓其他颜色。`pick_hold` 成功后必须先真实放稳／移除物体并确认夹爪不再持物，再同步
软件状态：

```bash
ros2 service call /fr3c_first_grasp/acknowledge_object_released \
  std_srvs/srv/Trigger '{}'
```

该确认命令不移动机械臂或夹爪。完整接口、安全边界和现场证据见
[P2 颜色任务 Action 技术报告](voice/FR3C离线语音抓取P2颜色任务Action技术报告-20260916.md)。

### 第一步：预张开夹爪并移动到方块上方

```bash
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'
```

等待机械臂停止后，目视确认夹爪在方块上方且对中。状态应进入
`waiting_for_descend_command`；未确认对中前不要执行第二步。

### 第二步：下降并闭合夹爪

```bash
ros2 service call /fr3c_first_grasp/descend_and_close std_srvs/srv/Trigger '{}'
```

等待下降和闭合完成，状态应进入 `waiting_for_manual_grip_confirmation`，再现场确认方块已被
夹住。夹爪为开环控制，仅凭服务返回成功不能证明已经夹持成功；未确认夹住前不要执行第三步。

### 第三步：确认夹持并提升

```bash
ros2 service call /fr3c_first_grasp/confirm_grip_and_lift std_srvs/srv/Trigger '{}'
```

机械臂将竖直提升 50 mm。完成后的状态为 `complete_holding`。

## 3. 取消与下一轮

需要中止当前动作时执行：

```bash
ros2 service call /fr3c_first_grasp/cancel std_srvs/srv/Trigger '{}'
```

取消不会自动松开夹爪、放下方块或恢复运动，后续必须按现场状态人工安全处置。

开始下一轮前，先确认机械臂已停止，取下并重新放置方块，结束法兰按钮拖动，等待定位重新变为
有效且稳定，然后从第一步 `/start` 重新执行。成功完成一轮后无需重启抓取栈，也无需重置定位节点。

停止整套系统时，在终端 1 按 `Ctrl+C`，并等待所有节点退出后再重新启动，禁止叠加启动第二套
控制栈。
