# P5：首次 RGB 方块三步任务开发记录

日期：2026-09-11  
范围：完成 P5 首版任务执行器、编译和拒绝不完整现场配置的离线门禁；本记录不宣称已执行真机运动。

## 当前结论

已新增一次性 `first_grasp_node`，保留原 `mtc_node` 仅作上游示例。新节点直接消费 P3 冻结的
`/perception/blue_cube_localizer/top_pose`、`valid` 和 `status`，使用 P4 冻结值：

```text
接近净空 0.015 m
抓取深度 0.015 m（因此接近位到抓取位下降 0.030 m）
提升 0.150 m
预张开 0.060 m
闭合 0.026 m
```

当前朝下姿态候选冻结为 `base_link` 下四元数 `[0, 1, 0, 0]`，即工具 Z 严格朝向
`base_link -Z`。现场只读得到的 P4 后当前姿态为 RPY 约
`[-2.997, 0.000, -3.113] rad`，与严格朝下候选接近但仍有约 8.3° 倾角；候选值必须先在
接近位审查，必要时只调整 yaw，不修改已冻结的深度和 26 mm 闭合量。

## 实现边界

- 默认 `execute=false`，只规划和发布 RViz 预览，不发送机械臂或夹爪命令。
- 动态把视觉顶面转换为 30 mm 方块碰撞体。按用户决定，将 RViz `base_link` XY Grid 的
  `Z=0` 作为下边界，并在 MoveIt 中生成 2×2 m、仅位于 Z=0 下方的碰撞薄板；三个 TCP 目标
  也显式拒绝低于 Z=0。RViz Grid 本身仍只是显示，安全约束来自新增的碰撞体和目标门禁。
- 接近使用 MoveIt 规划管线，可绕开障碍；下降和提升使用 `base_link` 绝对目标姿态的
  Cartesian path，比例低于 `0.999999` 或返回码失败就拒绝执行。
- 预览提升时方块以只允许四个手指接触的 attached object 参与碰撞检查，预览结束恢复为世界物体。
- 真机流程显式分段：`start` 预张开并执行接近；`descend_and_close` 才下降并闭合；人工确认后
  `confirm_grip_and_lift` 才附着并提升。
- 机械臂执行成功后再核对 TCP 位置/姿态误差；夹爪 action 成功后再等待 MoveIt 中开度估计到达
  60/26 mm。26 mm 仍是开环命令，不能替代人工夹持确认。
- 规划失败、执行失败、目标失效/改变、夹爪失败或取消都会停止状态机；不自动松爪、分离物体、重试或循环。
- 每次运行保存目标、关键参数、三段轨迹 YAML、事件和返回码到 `evidence/P5/runs/<time_ns>/`。

## 已执行验证

构建命令：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --packages-select fairino_mtc_demo \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
```

结果：通过。

首版“桌面必须实测”配置曾验证零值拒绝启动；用户随后明确允许直接采用 RViz 虚拟平面，正式配置
已改为 `ground_plane_z_m=0.0`，不再依赖桌面外廓尺寸。

首次准备隔离 mock 规划时发现旧 `mtc_demo_env.launch.py` 的注释和 README 虽称 fake/mock，
当前组合模型却已固定加载真实 `fairino_hardware/FairinoHardwareInterface`。进程在打印
`Connecting to FAIRINO controller` 后被立即终止；控制器 spawner 始终未连接，日志中没有
`ServoMoveStart`、`ServoJ`、轨迹执行或夹爪命令。现已给模型增加显式 `hardware_plugin` xacro 参数，
并把该 mock 入口固定传入 `mock_components/GenericSystem`，防止后续离线测试误连真机。

修复后在独立 `ROS_DOMAIN_ID=77` 重新运行，硬件立即完成 GenericSystem 激活，且不再出现控制器 IP、
`Connecting to FAIRINO controller` 或任何 FAIRINO SDK 日志。使用 P4 的顶面坐标、严格朝下候选姿态和
仅用于离线检查的局部桌面片，调用一次 `/fr3c_first_grasp/start`，结果为：

| 项目 | mock plan-only 结果 |
| --- | --- |
| 接近规划 | MoveIt 返回码 1（SUCCESS） |
| 下降 Cartesian path | fraction 1.0，32 点，返回码 1 |
| 方块 attached object | 成功，只允许四个指碰撞链接接触 |
| 提升 Cartesian path | fraction 1.0，151 点，返回码 1 |
| 最终状态 | `plan_only_complete` |
| 硬件命令 | 无；`execute=false`，未启动夹爪 action |

证据目录：`evidence/P5/runs/1789100104150143759/`。其中包含目标/参数、三段轨迹 YAML 和逐事件返回码。
该结果关闭代码链和完整路径比例检查，不替代真实当前关节姿态、实测完整桌面、RViz 视觉审查及真机执行。

旧 mock launch 仍有一个与 P5 仅规划无关的遗留问题：它尝试 spawn 已删除的
`fairino3mt_controller`，而当前 ros2_control 内部控制器名是 `fairino3mt_execution_controller`；
因此 mock 轨迹执行入口尚未恢复。此次只规划依靠 joint state broadcaster，结果不受影响。

## 当前姿态在线 plan-only

采用正式 Z=0 虚拟地面后，先在 mock 零位验证地面门禁：该零位的 wrist2 低于平面，接近规划被
`wrist2_link/rviz_ground_plane` 碰撞正确拒绝。随后在当前真机只读关节状态下重新冻结目标：

```text
base_link [-0.0920, -0.3566, 0.0355] m
```

首次提升被 MoveIt 相对平均关节跳变启发式在 21.19% 截断。为避免把启发式误触发当成真实碰撞，
改为保持碰撞检查开启、计算完整 Cartesian path，再逐点检查绝对关节步长上限 0.10 rad。重新规划结果：

| 项目 | 当前真机姿态 plan-only 结果 |
| --- | --- |
| Z=0 碰撞地面和方块 | 应用成功；方块底面约 Z=5.5 mm，未低于平面 |
| 接近规划 | MoveIt 返回码 1；保存后 36 点，约 3.480 s |
| 下降 | fraction 1.0；最大相邻关节步长 0.033404 rad；保存后 11 点，约 0.982 s |
| 附着提升 | fraction 1.0；最大相邻关节步长 0.073762 rad；保存后 47 点，约 4.556 s |
| 最终状态 | `plan_only_complete` |
| 硬件动作 | 无；未调用夹爪 action，未执行任何轨迹 |

正式证据目录：`evidence/P5/runs/1789105230679601877/`。

P5 剩余现场工作只包括：在 RViz 人工查看这次预览是否符合期望；完成 P6 的单一指令源、负载、
150 mm 空间和停止方式确认；随后按服务分段进行真机接近、下降闭合和人工确认后的提升。

## 2026-09-11 P6 接入补充

- 操作者已审查当前三段规划并反馈路径规划无问题。
- 修复 `first_grasp.launch.py execute:=true` 被任务 YAML 中 `execute:false` 抢先覆盖的问题；执行模式
  现由 launch 参数唯一控制，默认仍为 false。
- 真机参数读回确认 `execute=True` 后，失败的 P6 gate 能在 `start` 前拒绝任务；执行控制器保持
  inactive，未预张开、未发送轨迹。
- P5 现要求 600 s 内新鲜的 P6 passing gate，并在三个分段授权点及实际 approach 前重复核对。

## 2026-09-11 首次真机分段执行

操作者明确确认现场条件，并决定在首次低速效果验证中暂不标定末端负载/质心。P6 以有记录的原型
豁免通过后，使用正式 P3 冻结目标和法奥 MoveIt/关节 ServoJ 链完成一次 `execute=true` 分段执行：

| 阶段 | 真机结果 |
| --- | --- |
| 预张开 | action 成功，到达 60 mm |
| 接近 | 返回码 1；位置误差 0.290 mm，姿态误差 0.0267° |
| 垂直下降 | fraction 1.0；返回码 1；位置误差 0.188 mm，姿态误差 0.0154° |
| 闭爪 | action 成功；命令 26 mm，状态估计 26.571 mm |
| 垂直提升 | fraction 1.0；返回码 1；位置误差 0.149 mm，姿态误差 0.00155° |
| 最终状态 | `complete_holding`，保持提升位和 26 mm 命令 |

执行证据：`evidence/P5/runs/1789108218709764726/`。控制链已经完整成功；腕部相机在闭爪位只看到
标定板，无法从软件画面确认方块是否被实际带起，因此最终实物夹持效果由现场观察补记。

后续重复测试中，FT232H 在 USB 重枚举后仍由旧进程持有失效句柄，下一次 60 mm 预张开收到
`FtdiError: UsbError: [Errno 5] Input/Output Error`，gripper action 正确返回 ABORTED，P5 停在 FAILED，
没有进入机械臂接近。现已让 FT232H 后端在写线程退出后由下一条显式 action 自动重新打开当前设备，
并新增故障恢复测试；8 项夹爪测试在隔离 ROS domain 全部通过。该恢复不会自动重发导致故障的目标，
FAILED 的 P5 实例仍必须重启，避免动作在操作者不知情时继续。

## 每轮自动重新定位

发现仅重启 P5 会继续消费 P3 的上一轮冻结坐标后，已把重新定位握手接入 `/start`：先调用
`/perception/blue_cube_localizer/reset`，再要求 8 个新稳定样本、`frozen=true` 和晚于 reset 的图像
时间戳，之后才记录目标、预张开和规划。在线 plan-only 验证中旧/新样本秒时间戳分别为
`1789111807` 和 `1789112342`，新目标为 `[-0.083503,-0.472544,0.037075] m`；事件证据位于
`evidence/P5/runs/1789112340234563231/events.jsonl`，最终 `plan_only_complete`，未发送硬件动作。
