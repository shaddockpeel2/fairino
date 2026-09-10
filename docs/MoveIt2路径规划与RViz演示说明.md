# MoveIt 2 路径规划与 RViz 演示说明

更新日期：2026-09-08。工作区：`/home/industio/fairino_ws`。

本文记录 FR3-C（本项目使用 FR3MT 模型）在 ROS 2 Humble 下的规划演示流程及当前验证情况，不作为实机验收记录。

## 1. 当前进展

- 已通过 RViz 发起规划，日志出现 `Motion plan was computed successfully`，示例规划耗时约 0.014～0.019 秒。
- 已发起 Execute，日志显示 `fairino3mt_controller` 接收轨迹并返回 `SUCCEEDED`。
- 后续一次 Plan & Execute 提示目标已满足，无需运动。
- 上述执行日志中，从控制器接受目标到报告到达仅约 7 毫秒。因此可以确认规划与执行接口有成功返回，但仍需结合实机位移、关节反馈及实际加载的插件确认非零轨迹的实机执行效果。
- 此前出现过 J5 起点越界和 `ServoJ` 错误码 14；后续日志没有提供对应修复过程，不能视为已完成根因修复。

## 2. 规划与控制流程

```text
RViz 拖动目标 / 程序设置末端位姿
    → MoveIt move_group：逆运动学、路径规划、轨迹时间参数化
    → Execute：通过 FollowJointTrajectory action 发送轨迹
    → fairino3mt_controller（ros2_control 关节轨迹控制器）
    → fairino_hardware/FairinoHardwareInterface
    → FAIRINO SDK ServoJ → 实际机器人

实际关节角 → 硬件插件 read → joint_state_broadcaster
    → /joint_states → MoveIt 当前状态与 RViz 模型
```

Plan 只计算轨迹；Execute 执行已规划轨迹；Plan & Execute 合并这两步。
当前规划组为 `fairino3mt_v6_group`，规划链为 `base_link` 到 `wrist3_link`，控制器为 `fairino3mt_controller`。

## 3. 启动与演示

### 网络检查

机器人地址为 `192.168.58.2`，上位机专用有线地址配置为 `192.168.58.10/24`。
本次排查发现：NetworkManager 连接名仍为 `eth1`，但相同 MAC 地址的设备已变为 `enP4p65s0`，旧接口绑定导致静态配置无法启用。

若仍是这张网卡，可在本机终端修正原配置：

```bash
sudo nmcli connection modify eth1 connection.interface-name enP4p65s0
sudo nmcli connection up eth1 ifname enP4p65s0
ip route get 192.168.58.2
ping -c 3 192.168.58.2
nc -vz -w 2 192.168.58.2 8080
```

路由应直接走机器人有线网口，而不是 Wi-Fi 网关。端口可连接仅证明网络层连通，不代表完整 SDK 通信和运动条件均已满足。

### 确认硬件插件

启动使用的是安装目录中的配置：

`install/fairino3mt_v6_moveit2_config/share/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.ros2_control.xacro`

- `mock_components/GenericSystem`：模拟关节反馈，不驱动实机。
- `fairino_hardware/FairinoHardwareInterface`：实机插件。

本次早期检查时，安装目录为实机插件，而源码目录仍为模拟插件。重新编译可能覆盖安装目录修改，因此每次构建后应核对实际配置。`FakeSystem` 是组件名称，不能单独用于判断是否为模拟模式。

### 启动 RViz / MoveIt

为避开此前直接启动时的渲染环境问题，使用已出现过正常 RViz 初始化结果的命令：

```bash
cd /home/industio/fairino_ws
env -u LD_LIBRARY_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fairino3mt_v6_moveit2_config demo.launch.py
'
```

### 演示步骤

1. 确认加载模式、当前关节反馈及 RViz 姿态；实机模式下检查模型与实机一致。
2. 在 MotionPlanning 中选择 `fairino3mt_v6_group`，使用当前状态作为起点。
3. 拖动末端交互标记设置一个可达目标，点击 Plan，观察轨迹预览。
4. 实机演示前完成下述待确认项，确认现场运动条件后再点击 Execute。
5. 同时观察执行结果、实机关节变化及最终位姿；不要仅凭 `SUCCEEDED` 判定实机完成运动。

当前插件的硬件更新循环会调用 ServoJ，即使没有点击 Execute，也可能发送保持当前位置的指令。实机模式启动本身就会进入硬件控制流程。

## 4. 使用 4×4 位姿矩阵代替 RViz 拖动

可以由程序提供目标齐次变换矩阵：

```text
base_T_tool = [ R(3×3)  p(3×1) ]
              [ 0 0 0     1    ]
```

其中 R 是合法旋转矩阵，p 是位置。将 R 转为归一化四元数，将 p 转为以米为单位的位置，然后设置 MoveIt 位姿目标并调用规划、执行接口即可。ROS 关节角使用弧度，当前硬件插件负责与 SDK 的角度单位转换。

需要统一两项定义：

- **参考坐标系**：相机给出的目标必须先通过标定变换换算到基座，例如 `base_T_tool = base_T_camera × camera_T_tool`。
- **末端坐标系**：当前模型末端为 `wrist3_link`。若目标是夹爪 TCP，应配置 TCP 连杆，或换算 `base_T_wrist3 = base_T_tool × inverse(wrist3_T_tool)`。

单个目标矩阵只规定终点，不规定中间路径。普通 OMPL 规划的末端路径可能是曲线；直线接近、插入等动作需要另行设置笛卡尔路径要求。满足终点关节限位也不保证存在无碰撞路径或一定在给定时间内规划成功。

## 5. 已遇到的问题与待确认项

| 现象 | 已确认的信息 / 后续处理 |
| --- | --- |
| SDK 连接失败，控制器一直等待 | 当时机器人路由错误地走 Wi-Fi，旧 `eth1` 配置绑定失效；先恢复有线通信。 |
| J5 起点无效 | 日志反馈约 -97.27°，源码 URDF 范围为 0～350°；核对控制器软限位与角度约定，不能靠放大起点容差掩盖。 |
| ServoJ 错误码 14 | 当前 SDK 定义为“指令执行失败”；需核对控制器模式、使能及报警。源码未调用 ServoMoveStart，且失败后仍返回 OK、继续发送，需要进一步完善。 |
| 缺少碰撞几何 | 2026-09-08 已补齐 7 个本体连杆 collision，并加入固定夹爪占位包络，离线检测通过；真实安装对齐及环境碰撞体仍需核对，见 [夹爪占位模型与碰撞检测说明](夹爪占位模型与碰撞检测说明.md)。 |
| 缺少加速度限制 | 日志使用默认 1 rad/s²；应按匹配机型和控制器参数配置。 |
| RViz / MoveIt 退出崩溃 | 曾在 Ctrl+C 后出现退出崩溃和 launch 残留；与规划成功、实机执行成功分别判断。 |

下一阶段建议先完成配置与模型核对、ServoJ 初始化及错误处理，再记录一次明确的非零位移演示：起点关节角、目标位姿、规划轨迹时长、执行结果和实机最终反馈。本文记录期间未为了演示修改控制器软限位或自动切换机器人使能状态。
