# 三步抓取启动与执行命令

当前流程用于真实 FR3-C、DaBai DCW2 相机和 YDI-J22 夹爪。启动脚本会连接真实机械臂，并解锁
FT232H 夹爪 PWM 输出；执行前必须确认工作区无人、急停及停止方式可用、线缆余量充足，且没有
其他程序或人员控制机械臂。

## 1. 启动完整抓取栈

在终端 1 执行：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
scripts/run_moveit_camera_rviz.sh
```

该命令会启动机械臂控制、MoveIt 2、RViz、相机、手眼 TF、真实夹爪、蓝色方块定位、三步抓取
服务和 P6 安全门禁。启动本身不会让机械臂执行抓取；机械臂在收到第一步服务命令后才会运动。

等待启动终端中的 P6 检查输出 `passed=true`。将蓝色方块放在相机视野内并保持静止，然后在终端 2
加载环境：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

可选：执行前检查定位和抓取状态：

```bash
ros2 topic echo /perception/blue_cube_localizer/status --once
ros2 topic echo /perception/blue_cube_localizer/top_pose --once
ros2 topic echo /fr3c_first_grasp/state --once
```

只有定位状态为当前有效、稳定且时间戳新鲜时，才执行第一步。

## 2. 分三步执行抓取

### 一键连续执行（无人工停顿）

确认目标位置正确、整段路径净空且方块能够可靠夹持后，可用一个命令连续完成三步：

```bash
ros2 service call /fr3c_first_grasp/start_continuous std_srvs/srv/Trigger '{}'
```

该命令不使用固定延时：预张开、接近、下降、闭爪和提升会在前一动作成功后立即衔接。任一动作或
门禁失败都会终止后续流程。此模式会跳过高位对中和闭爪后的两次人工确认，仅可在已经验证过目标
位置、夹持效果及完整运动路径的条件下使用。

以下命令保留原有的人工分步确认方式。

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
