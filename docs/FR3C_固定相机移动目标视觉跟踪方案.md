# FR3C 固定相机移动目标视觉跟踪方案

> 适用场景：ROS 2 Humble、固定外部相机（eye-to-hand）、桌面平面目标、FR3C 机械臂，目标是在安全高度保持末端姿态，连续跟随移动物体。本文是架构设计与实施清单；频率、接口、性能阈值必须以实机测量为准。

## 1. 结论：哪些方案可以直接采用

**MoveIt Servo 可以直接承担机械臂的连续伺服计算，但不是一个开箱即用的“识别并追踪指定物体”的完整应用。** Humble 官方文档明确列出视觉伺服作为用途，接受带时间戳的 `TwistStamped` 或 `JointJog`，提供关节限位、奇异点和碰撞处理。它要求有效的 URDF/SRDF、关节反馈和接收位置或速度命令的控制器。最终仍需完成目标感知、相机与机器人坐标变换、视觉控制律，以及与 FR3C 驱动接口的对接。[MoveIt Servo Humble 官方教程](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)

| 环节 | 推荐做法 | 工作归属 |
| --- | --- | --- |
| 检测和目标身份 | 现有 HSV/YOLO 检测；多目标时增加关联与跟踪 | 项目实现 |
| 位置与速度估计 | 时间戳对齐，滤波并估计平面目标速度 | 项目实现 |
| 外部相机标定 | 工作平面像素到机器人基坐标变换；核验 TCP 偏置 | 项目标定 |
| 视觉控制 | 用目标与实际 TCP 的平面位置误差生成末端 XY 速度 | 项目实现 |
| 运动学和约束 | MoveIt Servo | 采用官方组件并配置 |
| 最终执行 | 实际 FR3C 驱动及 ServoJ/可用速度接口 | 验证厂商接口并适配 |

## 2. 固定相机下必须明确的几何关系

固定在外部的相机拍摄桌面时，**机械臂运动不会使桌上的静止目标在图像中移动**。所以先前把“机器人 XY 速度映射为目标像素速度”的式子
`[u̇_target,v̇_target] = J [Vx,Vy]` 用于此场景是不成立的。

正确的误差应是**目标与 TCP 在同一参考系下的相对位置**，不是“目标到画面中心的距离”，除非目标任务真的规定 TCP 投影始终固定在画面中心。例如把目标底部接触点和 TCP 都投影到桌面，再转换到机器人 `base_link`：

```text
相机图像中的目标接触点 ─┐
                      ├─→ 同一时刻、同一机器人基坐标系下的 XY 误差
机器人关节反馈 → FK → TCP ┘
                                  ↓
                             末端 XY Twist
                                  ↓
                              MoveIt Servo
                                  ↓
                         FR3C 连续执行接口
```

如果平面、相机内参和外参稳定，可以使用平面单应或深度加手眼外参求得 `p_target^base`。根据关节状态、工具坐标和运动学计算 `p_tcp^base`。当目标有高度变化或抓取要求三维定位时，改用深度/三维位姿估计；平面映射不能外推到不同高度。

> 这里选择任务参考点（如夹爪闭合后的实际接触中心），并将 TCP 标定与夹爪姿态保持一致。画面中心可以作为界面提示，不应自动等同于机器人跟踪的几何目标。

## 3. 建议的控制律

设目标和 TCP 在机器人基坐标系中的平面位置分别为 `p_t` 与 `p_r`，估计目标平面速度为 `v̂_t`，从视觉采样时刻到命令实际生效的估计延迟为 `τ`：

```text
p̂_t(now + τ) = p_t(measured) + v̂_t · τ
e_xy          = p̂_t(now + τ) - p_r(now)
v_tcp,xy      = sat(v̂_t + K · e_xy)
```

第一版可令 `v̂_t = 0`、`τ = 0`，先完成静态目标的连续跟随；再依次加入速度估计与延迟补偿。该控制律是**项目实现建议**，不是 MoveIt 官方提供的现成视觉追踪算法。速度前馈在目标运动且位置误差暂为零时仍会驱动末端跟随；反馈项负责减小相对偏差。使用实测时间戳估算延迟，不把示意延迟写死成参数。

将控制结果作为 `TwistStamped` 发布。明确 `header.frame_id` 为该 Twist 表达的参考坐标系，例如机器人 `base_link`；`linear.x/y` 的单位是 m/s；`linear.z` 和角速度按任务约束设置。Humble 的 `command_in_type` 若使用物理速度应配置为 `speed_units`，同时按该版本配置核对输入话题、输出类型和发布周期。[MoveIt Servo Humble 配置说明](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html#servo-configs)

感知抖动可通过测量静态像素噪声、滤波和速度限幅处理。若使用小死区，仅作用于**位置反馈修正**；不能在“进入中心区”时直接清零总速度，否则会丢掉目标速度前馈。任何限幅、加速度和急动度处理都应和驱动侧约束核对，避免同一命令被多次积分或多层平滑造成明显滞后。

## 4. ROS 2 节点与数据接口

| 节点/模块 | 输入 | 输出或职责 |
| --- | --- | --- |
| `target_tracker` | 带采集时间戳的图像、相机参数 | 目标 ID、接触点、置信度、速度估计及有效性 |
| `target_transform` | 目标观测、标定、TF | 带时间戳的 `base_link` 下目标 XY |
| `visual_servo_controller` | 目标状态、实际 `joint_states`/TCP 位姿、Servo 状态 | 带时间戳的 `TwistStamped` 和状态标志 |
| `moveit_servo` | Twist、机器人状态、规划场景 | 按实际配置输出关节位置/速度命令 |
| `fairino_servo_adapter` | Servo 的实际输出和机器人状态 | 对接已验证的 FR3C 连续执行接口；独立处理超时与停机 |

话题名、Servo 输出消息类型、FR3C 命令周期和接口模式均从**当前部署的配置与驱动**确认，不在文档中假定。MoveIt Servo Humble 可配置输出至 JointTrajectoryController，也可面向 JointGroupPosition/JointGroupVelocity 类控制器；**JTC 本身不是“用错了”**，需要看具体更新频率、轨迹字段、覆盖策略和实机跟随误差。[MoveIt Servo Humble 官方教程](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)

针对现有“10 Hz 单点 JointTrajectory → JTC → ServoJ”的链路，先记录各层发布时间、关节反馈和延迟，再比较以下两种链路：

1. 保留现有 JTC，发送实际控制器能连续插值的正确字段，测量是否满足跟随要求；
2. 若驱动支持持续且安全的关节命令，则试验 Servo 输出直接接 FR3C 连续执行适配器，比较延迟、抖动和故障处理。

Humble 的 JTC 文档指出：仅给位置时是线性插值，节点处速度可能不连续；位置加速度字段分别影响三次、五次插值。这个事实**不能单独证明现有抖动就是 JTC 引起的**。[ros2_control Humble 轨迹表示](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)

## 5. 分阶段落地与验收

| 阶段 | 操作 | 验收重点 |
| --- | --- | --- |
| A. 核查现状 | 核对 MoveIt/Servo Humble 配置、关节状态频率、控制器接口、FR3C 驱动命令和反馈周期 | 画出真实消息链；记录延迟与丢帧 |
| B. 坐标验证 | 标定桌面目标点到 `base_link`，检查 TCP 和目标在不同位置的同系坐标 | 至少多点核对误差及方向；异常不自动运动 |
| C. 静态闭环 | 限速、离桌面安全高度、无抓取动作，使用位置误差生成连续 Twist | 目标静止时平稳收敛，无方向反转 |
| D. 动态跟随 | 加入目标身份保持、速度估计、时间戳与预测 | 目标匀速/启停时统计误差、超调、滞后 |
| E. 执行层比较 | 比较原 JTC 链和直接适配链，固定相同感知与控制参数 | 依据实测跟随误差、时延、丢命令行为选择 |
| F. 任务集成 | 加入抓取触发、运动边界、急停和目标丢失恢复 | 目标丢失、超范围、异常时停止并可安全恢复 |

**关键安全门槛：** 校验目标数据新鲜度、关节状态新鲜度、TF 可用性、目标置信度、规划场景和机器人工作空间；失效时停止输出持续运动命令，并确认底层驱动在命令超时后会安全减速或停止。先仿真/低速空载验证，再上实机并逐步提高速度。MoveIt Servo 的碰撞和限位功能依赖正确的机器人模型、场景与配置，不应把它当成硬件急停的替代品。

## 6. 直接可复用的官方资料

- [MoveIt Servo：ROS 2 Humble 官方教程及新机器人配置要求](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [ros2_control：Humble JointTrajectoryController 轨迹插值与替换](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)

**最终推荐：** 采用“检测与状态估计 → 目标/TCP 同系误差和速度前馈 → Twist → MoveIt Servo → 经验证的 FR3C 连续执行接口”。是否去掉现有 JTC，以驱动能力和实测结果决定。
