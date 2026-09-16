# P2 颜色抓取任务 Action 接口

日期：2026-09-16  
范围：离线大模型语音抓取 10 阶段规划的第 2 阶段。阶段已于 2026-09-16 完成软件与代表性真机验收并关闭；完整结论见 [P2 颜色任务 Action 技术报告](FR3C离线语音抓取P2颜色任务Action技术报告-20260916.md)。

## 已实现行为

`fr3c_first_grasp` 现在同时提供两个单目标 ROS 2 Action：

| Action | 行为 | 成功终态 |
|---|---|---|
| `/fr3c_first_grasp/pick_hold` | 指定颜色，接近、下降、闭爪、抬升 | `complete_holding`，保持夹持，不放置 |
| `/fr3c_first_grasp/pick_sort` | 指定颜色，抓取、按既有 A4 映射放置、释放、返航 | `complete_sorted_home`，只处理本次指定颜色 |

两者使用同一个 `fairino_msgs/action/PickColor` 类型，Goal 只有 `color`。合法值为 `red`、`yellow`、`blue`、`green`。Feedback 包含 `phase` 和 `detail`；Result 包含 `success`、稳定错误码、说明、颜色和是否真实执行。

错误码包括：`INVALID_COLOR`、`BUSY`、`TARGET_NOT_FOUND`、`TARGET_NOT_UNIQUE`、`TARGET_STALE`、`PRECONDITION_FAILED`、`PLANNING_FAILED`、`EXECUTION_FAILED`、`CANCELED`、`HOLDING_OBJECT`、`PERCEPTION_UNAVAILABLE`、`INTERNAL_ERROR`。成功为 `OK=0`。

`pick_hold` 与 `pick_sort` 都只执行一次，不使用原 `/start_continuous` 的整批逻辑。Action 自动经过已经现场验证过的抓取阶段，但仍保留 P6、工作区、桌面、碰撞、完整轨迹、关节限制、到位和夹爪 Action 检查。闭爪仍是开环，因此成功信息只表示动作链完成，不声称传感器确认抓牢。

`pick_hold` 成功后状态保持为 `complete_holding`，后续 Action 返回 `HOLDING_OBJECT`，防止软件在仍可能持物时直接覆盖场景。操作员已在安全位置真正松爪并移除／放稳物体后，调用以下接口同步 MoveIt 场景：

```bash
ros2 service call /fr3c_first_grasp/acknowledge_object_released \
  std_srvs/srv/Trigger '{}'
```

该确认接口不发送机械臂或夹爪命令；它清除先前附着物、把上次 Action 的单色感知过滤恢复为全颜色，并回到 `waiting_for_target`。若物体实际上仍在夹爪中，不得调用；下一次任务的预张开动作可能使物体掉落。

## 视觉过滤与快照约束

感知节点新增 `/perception/blue_cube_localizer/set_target_color`（`fairino_msgs/srv/SetTargetColor`）。每次 Action：

1. 设置请求颜色并取得单调递增的 `generation`。
2. 感知清空旧稳定窗口、旧位姿、旧尺寸和旧跳过记录。
3. 新 RGB-D 帧完成 HSV、深度、方块顶面和桌面约束验证。
4. 只在请求颜色恰好有一个三维有效目标时开始稳定窗口；0 个不回退到其他颜色，2 个及以上不任意选择。
5. 抓取节点只接受 `target_filter_color`、`filter_generation`、`color`、`sample_stamp`、Pose 和 Size 全部匹配且未超时的快照。
6. 接受后复制不可变快照，后续相机更新不改变本次目标。

原四色批处理不设置颜色过滤时，仍按红→黄→蓝→绿顺序工作。感知回调带过滤代次防护：旧代次中尚未处理完的帧不能覆盖新代次状态。

## 启动和仅规划示例

首次必须使用仅规划模式：

```bash
cd /home/industio/fairino_ws
FR3C_FIRST_GRASP_EXECUTE=false scripts/run_moveit_camera_rviz.sh detection_mode:=colors
```

另开终端：

```bash
cd /home/industio/fairino_ws
scripts/grasp/send_color_task.sh hold red
scripts/grasp/send_color_task.sh sort yellow
```

等价的原生命令：

```bash
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_hold fairino_msgs/action/PickColor "{color: red}"
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_sort fairino_msgs/action/PickColor "{color: blue}"
```

执行模式沿用同一启动命令，将 `FR3C_FIRST_GRASP_EXECUTE` 改为 `true`。不得在未检查 RViz 完整路径、P6 门禁、区域标定和现场净空前启用。

## 验证记录

本次完成以下非运动验证：

- `fairino_msgs` 接口生成和 `fairino_mtc_demo` C++ 编译通过。
- `ros2 interface show` 可解析 `PickColor.action` 与 `SetTargetColor.srv`。
- 感知模块、跟踪窗口、区域映射和 P1 基线相关测试共 24 项通过。
- 工作区完整 Python 回归共 79 项通过；验证记录保存在 `evidence/voice/P2/20260916/software_verification.txt`。
- 新增测试覆盖“多色中只选指定色”“指定色不存在不回退”“同色两个目标拒绝唯一选择”。
- `python3 -m py_compile` 通过。

自动验证本身未发出机械臂或夹爪命令。其后现场完成了红色 `pick_hold`、红色 `pick_sort`、黄色 `pick_sort` 和两次蓝色 `pick_sort`，累计 5 次代表性成功运行；验收证据汇总见 `evidence/voice/P2/20260916/field_acceptance_summary.md`。

## 现场验收清单

依次在 `execute=false` 下验证：只有红色、多色含红色、没有红色、两个红色、稳定目标超过时效、任务忙；确认前两种只锁定红色，后三种分别返回预期错误且无运动。

再在低速受控现场用单块验证：

- `pick_hold(red)` 抬升后停住并保持夹持，不进入放置轨迹。
- 人工安全解除持物状态后，`pick_sort(red)` 只处理红色并返回起始关节位置。
- 人工释放后调用 `acknowledge_object_released`，确认状态回到 `waiting_for_target`，随后可连续接受下一条 Action。
- 执行中取消不会自动松爪、重试或转抓其他颜色。
- 快速重复发 Goal 不产生两次动作。

代表性主链路及连续 Action 状态恢复已有真机证据，第二阶段验收关闭。绿色真机循环、更多耐久循环和夹持传感器闭环作为后续整机验证／硬件增强项保留。

## 改动文件

- `src/fairino_msgs/action/PickColor.action`
- `src/fairino_msgs/srv/SetTargetColor.srv`
- `src/fairino_msgs/CMakeLists.txt`
- `src/fr3c_camera/fr3c_camera/blue_cube_localizer.py`
- `src/fr3c_camera/test/test_cube_perception.py`
- `src/fr3c_camera/package.xml`
- `src/fairino_mtc_demo/src/first_grasp_node.cpp`
- `src/fairino_mtc_demo/config/first_grasp.yaml`
- `src/fairino_mtc_demo/CMakeLists.txt`
- `src/fairino_mtc_demo/package.xml`
- `src/fairino_mtc_demo/README.md`
- `scripts/grasp/send_color_task.sh`
- `evidence/voice/P2/20260916/software_verification.txt`
