# FR3-C + RK3588 端侧实验箱：抓取项目总体设计与进程规划

> 文档状态：可执行开发基线 / V0.9
>
> 日期：2026-09-10（Asia/Shanghai）；本次同步 FR3-C 运动学/手眼、MoveIt 真机验证及 YDI-J22 ROS/MoveIt 软件接入进度
>
> 工作区：`/home/industio/fairino_ws`
>
> 当前机器人控制器：`192.168.58.2`，软件 V3.9.2
>
> 本期范围：先完成安全、可重复、可验收的 RGB-D 视觉抓取；暂不规划 LLM/VLM、语音和 RAG。

> **前置资源提醒：先查法奥官方开源资源和本地资源池，再决定是否自研或重新下载。**
>
> - 法奥官方 GitHub 组织及仓库总入口：[FAIR-INNOVATION / FAIRINO 开源仓库](https://github.com/FAIR-INNOVATION?tab=repositories)；
> - 本机法奥开源资源统一放在：`/home/industio/Fairino`；
> - `/home/industio/Fairino/frcobot_ros2` 是完整 Git clone。当前工作树为较新的 `main`/V3.9.9 系列，**不能直接复制当前工作树用于本机 V3.9.2 控制器**；项目取材时必须显式读取 `V3.0.0_RobotV3.9.2`（commit `b6de550dd695ab15c56c1ae5f700f499202a80d2`）；
> - `/home/industio/Fairino/fairino-cpp-sdk-main` 和 `fairino-python-sdk-main` 当前是没有 `.git` 元数据的源码快照，只能用于查阅和候选代码比较，不能仅凭目录名认定版本；
> - 正式真机 C++ SDK 仍以 `v2.3.2-robot3.9.2`、本工作区已验证的 ARM64 `libfairino.so.2.3.2` 为准；
> - `/home/industio/Fairino` 定位为“只读上游资源池”，不在里面直接做项目定制。需要使用的官方包先核对 tag/commit/API，再以可追溯方式引入 `/home/industio/fairino_ws/src`；
> - 不要同时 source `/home/industio/Fairino` 下的构建结果和本工作区，避免同名 `fairino_hardware/fairino_msgs` 包或不同 SDK 版本互相覆盖。
> - `/home/industio/DofbotPro_JetsonNano_0106/dofbot` 中已验证的 DaBai DCW2 P4 方案只作为配置、门禁和融合策略参考；本项目不 source 其 overlay，也不复制其旧 SDK/构建产物。

## 1. 总体结论

本项目不应重新编写机械臂通信协议、运动学、轨迹规划器、相机驱动或完整抓取状态机。推荐主线为：

1. 继续使用已经跑通的法奥 V3.9.2 官方 C++ SDK 和 ROS 2 `fairino_hardware`；
2. 保留 `fairino3mt_v6_moveit2_config` 的工程入口和六关节接口，但机器人运动学改用经 V3.9.2 控制器 FK 实测确认的标准 `fairino3_v6` 链，并在项目兼容副本中显式定义 `flange_link`；不得再把未经核对的型号名直接等同于运动学型号；
3. 使用已从本机法奥 Robot V3.7.7 资源取得并移植的 `fairino_mtc_demo` 循环抓取样例，在 V3.9.2 控制链上继续做版本适配；
4. 相机方案固定为 Orbbec DaBai DCW2，使用本工作区 Orbbec ROS 2 Wrapper；已舍弃无法稳定取得深度图的 Astra Stereo S U3（此前称 Gemini Pro），不自行开发 USB/深度驱动；
5. 感知第一版不用大模型，也暂不依赖 YOLO：用 RGB-D、桌面平面、颜色/几何或 ChArUco/AprilTag 完成确定性抓取；
6. 自研代码只保留在“设备适配、坐标标定、抓取参数和任务编排”这一薄层；
7. 真机运动之前必须先解决 FR3-C 模型核对、`ros2_control` 伺服模式初始化、末端负载/TCP、夹爪电气适配和急停验证。

预计在硬件资料齐全、单人开发且不含采购等待的情况下，30～45 个有效工程日可以完成“桌面随机位置积木抓取并放入固定料框”的第一版闭环演示。相机和夹爪的不涉及机械臂运动的台架测试从第一周开始，可与机器人模型/仿真工作并行；未通过单项门禁的硬件不得进入整机联调。

## 2. 当前基线与实际进度

### 2.1 已确认的基础环境

| 项目 | 当前状态 | 说明 |
| --- | --- | --- |
| RK3588 | 已完成 | AArch64，Ubuntu 22.04.5，ROS 2 Humble |
| 法奥网络链路 | 已完成 | RK3588 `192.168.58.10/24` 到控制器 `192.168.58.2` 可达 |
| ROS 2 工作区 | 已完成 | `fairino_msgs`、`fairino_hardware` 可全量编译 |
| SDK 架构适配 | 已完成 | 使用官方 `fairino-cpp-sdk` 的 V3.9.2 对应标签本机构建 ARM64 动态库 |
| 只读通信 | 已完成 | ROS 节点可连接控制器，SDK 版本与状态话题可读取 |
| 运动接口审核 | 部分完成 | 服务端运动参数已做过检查，但尚未完成现场真机运动验收 |
| MoveIt 2 | 基础真机规划执行已验证，P07/P11 仍开发中 | `fairino3mt_v6_moveit2_config` 已切换到控制器一致的 `fairino3c_v6`；操作者已在有人监护、低速条件下通过 RViz 末端交互球和单关节面板完成多组 Plan/Execute，实体机械臂运动正常；异常退出、取消/断线、ServoJ 生命周期/周期、归档的 20/100 组门禁仍待验收 |
| RGB-D 相机 | DaBai DCW2 已接入，P03 开发中 | `CH82C5201C9`、固件 `RD1017`、USB2；RGB/硬件 D2C 深度、严格 20 秒快速门禁及相机独立 RViz 的 RGB/深度/彩色点云显示已通过，尺量精度和长期稳定性待验收 |
| 夹爪 | 台架动作与 ROS/MoveIt 软件链已验证，P04/P09 开发中 | YDI-J22 四指柔性夹爪，独立 5 V 三线 PWM；FT232H AD7 点动已通过，`GripperCommand`、mock、诊断、MoveIt controller 和受控 FT232H backend 已实现；真实 action、机械行程、持续夹持及循环验收待完成（见第 6.6 节） |
| 手眼标定 | 核心数值门禁与正式 TF 已通过，P13 开发中 | 最终支架与 640×480 相机配置下采集 40 组 ChArUco；25 组求解、15 组独立验证，平移中位数/P95 为 2.274/6.327 mm；`flange_link → camera_link` 已通过重启和 TF 唯一性检查，固定点/9 宫格和拆装复验待完成 |

现阶段可以定义为：通信和构建底座完成；MoveIt 已证明 RViz 笛卡尔目标和单关节目标能够规划并驱动实体机械臂，但 P11 的生命周期与故障门禁尚未关闭；DaBai DCW2 已完成可用深度图快速验证、独立 RViz 和 MoveIt 组合可视化，但 P03 长稳与精度门禁尚未关闭；FR3-C ROS 运动学已按控制器 FK 纠正；eye-in-hand 手眼数值门禁及正式 TF 接入完成，但 P13 的固定点、9 宫格和拆装复验尚未关闭；夹爪 ROS/MoveIt 软件链和 mock 动作已接通，真实 FT232H action 及机械/带载验收尚未完成。原约 15% 为早期粗估，本次不按单项成果推算新的整机完成百分比；以第 10 节阶段台账为准。

### 2.2 必须保留的版本基线

- 机器人 WebAPP/控制器：V3.9.2；
- ROS 2 源码标签：`V3.0.0_RobotV3.9.2`；
- C++ SDK 标签：`v2.3.2-robot3.9.2`；
- ROS 2：Humble；
- 系统：Ubuntu 22.04；
- RGB-D 相机：DaBai DCW2，SN `CH82C5201C9`，固件 `RD1017`；
- 相机运行时：Orbbec ROS wrapper 1.5.15、Orbbec SDK 1.10.35，默认配置 `src/fr3c_camera/config/dabai_dcw2.yaml`。
- 相机可视化：独立 profile `src/fr3c_camera/config/dabai_dcw2_rviz.yaml` 仅用于 RViz/点云观察，不替代上述生产基线。
- 机器人运动学：`src/fairino_description/urdf/fairino3c_v6.urdf`，组合模型显式定义 `wrist3_link → flange_link` 为 +Z 0.1 m；
- 当前手眼外参：`evidence/P13/manual_30pose_20260910/flange_to_dabai_dcw2.yaml`，绑定相机 SN `CH82C5201C9`、`final_mount_v1` 和 640×480；发布父子帧为 `flange_link → camera_link`。

法奥最新在线手册当前已高于本机控制器版本。最新手册可用于理解功能和硬件结构，但每一个要调用的 API 都要以 V3.9.2 标签内的头文件和源码为准，不能把新版本接口直接用于旧控制器。

## 3. 本期目标与非目标

### 3.1 本期最终演示目标

在固定桌面作业区内，FR3-C 使用法兰侧固定的 Orbbec DaBai DCW2 RGB-D 相机识别指定积木，在实体碰撞建模和宽松任务范围校验下完成：

`观察位 -> 识别 -> 规划 -> 预抓取 -> 直线接近 -> 夹取 -> 提升 -> 料框上方 -> 放置 -> 撤退 -> 回观察位`

推荐第一版对象：表面不反光、尺寸已知、颜色明显的立方体或圆柱体；推荐第一版放置目标：固定、宽口、带明显边界的料框。

### 3.2 预设验收目标

| 目标 | 初始验收值 |
| --- | --- |
| 固定示教点抓取 | 20 次中成功至少 19 次 |
| 随机平面位置抓取 | 50 次中成功至少 45 次 |
| 放入固定料框 | 50 次中成功至少 48 次 |
| 手眼标定点验证 | 平移误差中位数不大于 3 mm，P95 不大于 8 mm |
| 感知位置稳定性 | 静止目标 3D 中心 P95 抖动不大于 5 mm |
| 碰撞与硬越界 | 0 次；只有超出机器人原厂关节限制、进入真实障碍物或明显不可达的候选才硬拒绝，普通任务区边缘只提示 |
| 连续运行 | 100 个循环无进程崩溃、无未处理故障；成功率至少 95% |
| 故障行为 | 相机超时、深度无效、规划失败、夹爪失败时不继续盲动 |

这些是第一轮工程目标。若相机近距离深度、夹爪机械间隙或支架刚度不支持该误差，需要先整改硬件，不能用软件阈值掩盖。

### 3.3 暂不纳入本期

- LLM/VLM、ReAct Agent、自然语言控制；
- YOLO/RKNN/NPU 模型部署；
- 任意类别、透明/反光/柔性物体抓取；
- 动态目标、传送带跟踪和视觉伺服；
- 复杂堆叠、乱序箱抓和 6D 通用抓取；
- 云端服务、语音识别、数字人；
- OpenHarmony 双系统。

本期仍会使用标准 ROS 接口，使以后模型只替换“目标选择/目标检测”模块，不进入安全运动控制内环。

## 4. 官方内容复用清单

| 能力 | 优先复用内容 | 本项目只做什么 | 不做什么 |
| --- | --- | --- | --- |
| 机器人通信 | V3.9.2 对应 `fairino-cpp-sdk` | ARM64 构建、错误封装、日志与健康检查 | 不重写 XML-RPC/TCP 协议 |
| ROS 2 API | `fairino_msgs`、`fairino_hardware` | 保留已审计的少量 ARM64/安全补丁 | 不另写一套机械臂驱动 |
| 机器人模型 | `fairino_description` | 增加 FR3-C 核对结果、相机/支架/夹爪/TCP 叠加模型 | 不手写 FR3 运动学 |
| 路径规划 | `fairino3mt_v6_moveit2_config` + MoveIt 2 | 建立实验箱场景、限速、碰撞体和末端执行器配置 | 不自研 RRT、IK 或插补器 |
| 抓取流程 | 法奥 `fairino_mtc_demo`；备选 MoveIt MTC 官方 Pick/Place | 替换机器人组名、抓取姿态、夹爪动作和场景参数 | 不从零写复杂状态机 |
| 末端参数 | SDK `SetToolCoord`、`SetLoadWeight`、`SetLoadCoord` | 标定、应用和开机一致性检查 | 不在应用层伪造 TCP |
| YDI-J22 夹爪 | 已有 `test/jiazhua` PWM 台架工具与交接记录；`fr3c_gripper_driver` 已实现 | 本原型正式控制继续使用独立 5 V + FT232H AD7，增加 ROS action、标定与故障处理；若 USB 抖动不能通过持续保持验收再换硬件定时 PWM | 不把有限时长点动结果当成持续夹持和真实反馈 |
| 法奥末端接口 | SDK 夹爪 API、`SetToolDO`、末端 IO/485 仅作其他夹爪候选 | 更换设备时重新核对适配型号和电气规格 | 不将本夹爪接入 24 V，不用 ToolDO 模拟 PWM |
| RGB-D 驱动 | Orbbec `OrbbecSDK_ROS2`，DaBai DCW2 | 固定 SN/USB2 启动 YAML、设备预检、RGB/深度/点云 RViz、话题检查、录包与稳定性门禁 | 不写 UVC/OpenNI/点云转换驱动，不回退复制旧项目 SDK |
| 手眼标定 | MoveIt Calibration ROS 2 分支或 OpenCV `calibrateHandEye` | 采样、TF 对接、误差验证和结果固化 | 不自研 AX=XB 求解器 |
| 抓取几何 | SDK `ComputePrePick/ComputePostPick` 可作结果对照；主流程使用 MTC Cartesian stages | 设定 approach/lift/retreat 距离 | 不手写轨迹插补 |

重要发现：法奥人员确认 `fairino3mt` 是 FR3-C 的历史资源命名，但实机 FK 已证明原 `fairino3_mt_v6` 运动学不匹配当前 V3.9.2 控制器。项目保留 `fairino3mt_v6_moveit2_config` 包名和控制接口，内部改用经控制器验证的 `fairino3c_v6` 兼容模型。`fairino_mtc_demo` 已从本机 Robot V3.7.7 资源取得完整源码、迁入本工作区并完成 ARM64 编译，但真机运行仍须验证 V3.9.2 `fairino_hardware` 兼容性。

## 5. 总体软件架构

```text
                         ┌──────────────────────────────┐
                         │ RViz2 / CLI / 后续 Agent     │
                         │ PickPlace Action Client       │
                         └──────────────┬───────────────┘
                                        │ 目标ID/料框ID
                         ┌──────────────▼───────────────┐
                         │ fr3c_manipulation             │
                         │ MTC任务、场景、失败恢复、仲裁 │
                         └───────┬─────────────┬────────┘
                                 │             │
                   规划/轨迹      │             │ GripperCommand
              ┌──────────────────▼───┐   ┌─────▼────────────────┐
              │ MoveIt2 + ros2_control│   │ fr3c_gripper_driver  │
              │ 法奥FR3配置派生包     │   │ PWM标定/限幅/诊断    │
              └──────────┬───────────┘   └─────┬────────────────┘
                         │ ServoJ/官方SDK       │ USB/UART（待定）
                    ┌────▼────┐       ┌───────▼──────────────────┐
                    │ FR3-C   │       │ 硬件定时PWM控制器/MCU    │
                    └─────────┘       └───────┬──────────────────┘
                                              │ 持续50 Hz PWM
                                     YDI-J22（独立稳压5 V供电）

   Orbbec RGB-D ──官方ROS2 Wrapper──> fr3c_perception ──Detection3D──┐
           │                                                        │
           ├── Image/Depth/PointCloud2 ──> 相机独立/MoveIt组合 RViz │
           └── CameraInfo/TF --> fr3c_calibration ------------------┘

   工作台、料框、相机支架、夹爪碰撞模型 ──> MoveIt PlanningScene
   非实时状态、急停、故障码、motion_done ──> safety supervisor/diagnostics
```

### 5.1 接口原则

- 感知输出优先使用 `vision_msgs/Detection3DArray`；
- 位姿使用 `geometry_msgs/PoseStamped`，每条数据必须带时间戳和坐标系；
- 夹爪优先实现 `control_msgs/action/GripperCommand`；
- 整体任务提供一个薄的 `PickPlace.action`；
- 健康状态进入 `/diagnostics`；
- 所有真机任务必须可录制为 rosbag，以便离线复现；
- 大模型以后只能调用 `PickPlace.action`，不能直接调用 `MoveJ/MoveL/ServoJ/ToolDO`。

## 6. 关键设计决策

### 6.1 FR3-C 运动学以控制器 FK 实测为准

法奥人员曾说明 ROS 资源中的 `fairino3mt` 是 FR3-C 的旧型号命名，但 2026-09-10 的实机核对证明：型号名不能直接作为运动学正确性的依据。原 `fairino3_mt_v6.urdf` 在 3 个现场姿态下无法与控制器法兰保持固定刚体关系，最佳固定 `wrist3_link → controller flange` 仍有 10.991 mm / 3.036° 离散，因此禁止继续作为本机 V3.9.2 控制器的 FK 真值。

本地候选模型核对结果：

- `/home/industio/franka_description/robots` 下的 `fr3`、`fr3v2` 等属于 Franka Robotics 七轴 FR3，不适用于 FAIRINO FR3-C 六轴数据；
- FAIRINO `FR3WML` 为长臂几何，不匹配；`FR3WMS` 较接近但仍不如标准模型一致；
- 标准 FAIRINO `fairino3_v6` 六轴链与控制器 `GetForwardKin`/`GetActualToolFlangePose` 一致；控制器法兰位于 `wrist3_link` 自身 +Z 方向 100 mm；
- 项目新增 `fairino3c_v6.urdf`，只对工作区标准 `fairino3_v6.urdf` 做 robot/joint 名称兼容，将关节保持为硬件接口使用的 `joint1`～`joint6`；组合 xacro 显式增加 `flange_link`；
- 3 个直接现场姿态的 `base_link → flange_link` 最大误差为 0.01062 mm / 0.000792°；40 组历史关节角经控制器 FK 重建后的最大误差为 0.00981 mm / 0.000443°，均通过 ≤1 mm / ≤0.2° 门槛；
- URDF 可解析，默认位与两个命名位无自碰撞，随机自碰撞和各实体障碍碰撞检查可检出。

后续处理原则：

1. 保留 `fairino3mt_v6_moveit2_config` 包名和既有控制器接口，内部运动学使用已核对的 `fairino3c_v6.urdf`；
2. 继续索取 FR3-C/FR3MT 对应 STEP、尺寸图和 V3.9.2 版本资料，用于核对惯量、碰撞外形和关节限制，不再反向覆盖已通过实测的 FK；
3. 保持独立 `wrist3_link`、`flange_link` 和 `grasp_frame`；本项目直接以 `grasp_frame` 表示工具 TCP，不增设含义重复的 `tool0`，也不得把 mesh 原点、控制器法兰和夹爪 TCP 当成同一点；
4. 未来更换本体模型或控制器版本时，必须重新执行多姿态 FK 交叉验证；
5. FK 通过不等于 MoveIt 真机轨迹门禁通过，P07 的 100 组规划和 P11 的低速真实轨迹仍须独立关闭。

详细模型比较与数据边界见 `evidence/P13/manual_30pose_20260910/robot_model_comparison.md`。

### 6.2 机器人只能有一个运动指令源

当前有两条官方控制路径：

- `/fairino_remote_command_service`：适合连通性、单条 `MoveJ/MoveL` 和 SDK API 验证；
- `ros2_control -> FairinoHardwareInterface -> ServoJ`：适合 MoveIt 轨迹执行。

两条路径不能同时下发运动。启动文件要提供互斥模式：

- `control_mode:=sdk_service`：调试与单指令验收；
- `control_mode:=moveit`：正式抓取运行；
- `control_mode:=mock`：纯仿真。

正式抓取时只允许 MoveIt/ros2_control 控制手臂。SDK 服务端即使保留，也必须禁用所有运动命令或不启动。

### 6.3 当前 ros2_control 路径有真机前置缺口

当前 V3.9.2 `FairinoHardwareInterface` 在激活后直接周期性调用 `ServoJ()`，没有在 `on_activate()` 中显式进入 `ServoMoveStart()`；同时 `std::any_of` 的结束指针没有覆盖第 6 轴。较新官方目录仍可见同类实现，公开 issue 也报告过真实机器人阻塞问题。

真机 MoveIt 门禁：

1. 先在隔离分支做最小 backport：同步当前关节位置、复位错误、使能、停止旧运动、结束旧伺服模式、进入 `ServoMoveStart()`；
2. 退出时执行 `ServoMoveEnd()`、`StopMotion()`、`CloseRPC()`；
3. 修正 6 轴有限值检查；
4. 任何 SDK 返回码非 0 都使 hardware lifecycle 激活失败；
5. 在假硬件和无负载低速真机上分别验证；
6. 不为了获得新代码而盲目升级控制器，仍保持 V3.9.2 ABI/数据结构匹配。

### 6.4 相机推荐固定在法兰侧独立支架

推荐“eye-in-hand”安装：相机固定在法兰转接板上，夹爪安装在同一转接板中心；不建议把相机固定在可拆夹爪外壳上。

原因：

- 更换夹爪时不破坏 `flange -> camera` 手眼标定；
- 相机支架刚度和重复装配更容易控制；
- 可以把相机、支架和夹爪分别建模；
- 有利于定义稳定的 `grasp_frame`。

机械注意事项：

- 相机视轴需要看见夹爪前方 150～450 mm 的抓取区；
- 支架尽量短、厚、带定位销或止口；
- USB 线必须有应力释放和腕部转动余量，不能跨越夹爪运动区；
- 相机、支架、夹爪、线缆和最大工件的总质量与质心都计入末端负载；
- 按 FR3-C 额定 3 kg 进行本项目设计，不把扩展 5 kg 能力当常规预算；
- 支架调整、相机重新安装、分辨率或 D2C 模式改变后都要重做标定验证。

### 6.5 DaBai DCW2 已替换原 Astra Stereo S U3 方案

原 Astra Stereo S U3（此前称 Gemini Pro）在本项目测试中始终未能形成满足后续抓取要求的可靠深度图，方案已明确舍弃。替换设备已由本工作区 SDK 实测枚举为
`DaBai DCW2`，序列号 `CH82C5201C9`，固件 `RD1017`；USB 深度/控制接口为
`2bc5:06a0`，RGB 接口为 `2bc5:0561`，两个接口均以 USB 2.0 / 480 Mbit/s 连接。

本项目借鉴旧 Dofbot 项目已经用于 RGB-D 抓取的低带宽方案，固定以下默认数据契约：

1. RGB 640×480@10 FPS、MJPG；原始深度 640×400@10 FPS、Y11；
2. 使用硬件 D2C，发布 640×480 `16UC1` 毫米深度；
3. 关闭 IR、点云、设备软滤波、LDP、heartbeat 和固件不支持的 frame sync；
4. RGB 与对齐深度使用 `camera_color_optical_frame`，但仍按各自 `CameraInfo` 处理畸变；
5. 启动前核对 SN、两个 USB PID、480 Mbit/s、授权状态和设备节点权限；
6. RGB-D 上层按时间戳近似配对，台架审计上限 60 ms，正式抓取融合沿用旧项目更严格的 50 ms 门禁；
7. 本工作区继续使用已验证兼容的 wrapper 1.5.15 / SDK 1.10.35，不复制旧项目 SDK 1.10.18 或 source 其 overlay。

2026-09-09 严格 20 秒快速验证全部门禁通过：RGB/对齐深度各 194 帧、均为
9.69 FPS，形成 187 组有效 RGB-D 对，时间差 P50/P95 为 26.9/56.2 ms，整图有效深度比例中位数为 96.88%，时间戳无回退。该结果证明当前链路已取得可消费的深度图，但不替代尺量距离精度、材质覆盖、60 分钟稳定性和装机工作距离验收。详细记录见
[DaBai DCW2 接入说明](../hardware/DaBai_DCW2_接入说明.md)。

在不改变上述生产抓取基线的前提下，已经新增 `dabai_dcw2_rviz.yaml` 可视化 profile：
保留相同 RGB/硬件 D2C 参数，仅启用 `/camera/depth_registered/points` 彩色点云，并使用
`SENSOR_DATA/BEST_EFFORT` QoS。相机独立入口 `scripts/camera/run_rviz.sh` 会先清理本机
Qt/OGRE 环境、执行 USB 预检，再启动预配置 RViz；其 Fixed Frame 为
`camera_color_optical_frame`，因此不依赖手眼标定即可查看 RGB、对齐深度和点云。

2026-09-09 实机验证中，RViz 使用 OpenGL 3.1 正常创建窗口，RGB/深度约 10 Hz，RViz
与三个数据话题的 Best Effort QoS 匹配，且收到 frame 为 `camera_color_optical_frame` 的彩色
点云。当前 ARM 主机生成 640×480 彩色点云的更新率低于图像，现阶段定位为实时观察工具；
需要更高更新率时再单独评估无颜色 XYZ、降采样或显示分辨率，不能直接修改生产抓取 profile。
MoveIt 组合入口在未提供标定时可发布明确标记为“仅可视化”的相机外壳估位 TF；正式运行时只有显式加载
`calibrated: true` 且结构有效的 `flange_link → camera_link` 文件才发布精确 TF，并自动停用外壳预览 TF。
2026-09-10 已使用最终文件完成重启加载，确认只有 `/dabai_hand_eye_tf` 发布 `camera_link`，不存在多父坐标系。
实现、验证和接手边界见
[DaBai DCW2、RViz 与 MoveIt 2 集成技术交接](../DaBai_DCW2_RViz与MoveIt集成技术交接.md)。

### 6.6 YDI-J22 四指柔性夹爪：ROS/MoveIt 软件链已接通，真实 action 待验收

本节依据 [YDI-J22 四指夹爪测试与技术交接](../../test/jiazhua/YDI-J22四指夹爪测试与技术交接.md)、[台架 README](../../test/jiazhua/README.md) 和 [`fr3c_gripper_driver`](../../src/fr3c_gripper_driver/README.md)，记录截至 2026-09-10 的结论；本次软件验证未向真实夹爪发送新 PWM。

| 项目 | 已确认内容与证据边界 |
| --- | --- |
| 型号/控制 | YDI-J22 四指柔性夹爪，内部三线 5 V、0～180° 舵机；控制为 PWM，不是 UART 字节流 |
| 接线 | 棕线 GND、红线独立 +5 V、黄线 FT232H `ADBUS7/AD7`；电源负极与 FT232H GND 共地 |
| 供电 | 独立稳压 5 V，建议持续能力至少 2 A；商品图片标称堵转电流约 1.6 A，尚无实测电流验收记录；禁止使用 FT232H 小板 5 V 脚给夹爪供电 |
| 转换器 | FTDI FT232H，VID:PID `0403:6014`；交接时枚举 `/dev/ttyUSB0`，by-id 为 `/dev/serial/by-id/usb-FTDI_Single_RS232-HS-if00-port0`；程序通过 PyFtdi 接管 GPIO，不向串口发送动作字节 |
| 信号 | 20 ms 周期（50 Hz）；FT232H GPIO 高电平为 3.3 V，禁止向该脚输入 5 V，不使用真正 RS-232 的 ± 电压端口 |
| 电气范围 | 500～2500 µs 对应 0～180°，来自 2026-09-07 用户向卖家客服确认；书面规格仍待归档 |
| 方向/实测范围 | 脉宽减小闭合、增大张开，1500 µs 为舵机中位；已测试若干离散点，最低 500 µs、最高 1700 µs，完整点位见交接文档；不代表区间内全部点位及负载均已验收 |
| 机械边界 | 真实 action 实测550 µs闭合净开度0 mm、1650 µs张开净开度约130 mm；图纸166 mm是展开外廓，不对应已知脉宽；2500 µs尚未验证当前装配的机械安全性 |
| 软件 | 原台架脚本继续保留；新增 `fr3c_gripper_driver`，提供 mock/FT232H backend、标准 `GripperCommand`、限幅/渐变/timeout/cancel/busy/USB fault、诊断和 MoveIt controller；异步 bit-bang 默认 100 kHz，脉宽分辨率 10 µs |
| 保持/反馈 | 最后一次记录为 1700 µs、1 秒；程序结束停止 PWM、AD7 恢复输入，不持续保持；现有工具没有真实开度、夹持力或夹持完成反馈 |

机械资料位于 `/home/industio/YDI-J22柔性机械夹爪-三维模型`。交接文档记载其包含二/三/四指 STEP 与尺寸图，四指最大展开尺寸 166 mm、最大夹持角约 117°；这些是资料参数，不等于本装配的实测工作行程。舵机具体型号和厂家协议表仍缺失。

本原型正式路线冻结为“RK3588 → USB → FT232H AD7 → YDI-J22”，夹爪独立稳压 5 V。ROS backend 连续刷新 50 Hz 波形，真实输出需要配置参数与环境变量双重解锁，启动节点不发 PWM。FT232H 异步 bit-bang 仍是 USB 开环方案；若持续保持、抖动或断连测试不通过，应直接替换为硬件定时 PWM/MCU，ROS action 层保持不变。当前无证据表明 YDI-J22 可直接使用法奥 `SetGripperConfig/ActGripper/MoveGripper`；不走末端 485/开关量路线，不用 `SetToolDO` 模拟 PWM，不将三线夹爪直接接 FR3-C M12/24 V。

剩余工作：

1. 当前真实 action 工作范围冻结为 `operating_close_us=550`、`operating_open_us=1650`，对应实测净开度0～130 mm；已将0/24/60/100/110/130 mm六点分段线性映射写入驱动，500/2500 µs保留为电气边界。
2. 固化保护、共地、线束和应力释放设计，补实测电流、温升、动作时间、上电/失电/信号丢失行为及 100 次循环记录。
3. 验证 FT232H backend 的持续 50 Hz、限幅、渐变、timeout、cancel 和 USB 断连行为；无电流/堵转反馈的缺口通过外部传感或流程限制处理。
4. 业务层采用 0% 安全闭合、100% 安全张开的标定语义；ROS `GripperCommand` 的位置单位和几何映射在 P09 明确定义，不直接把百分比或舵机角度填入标准位置字段。
5. 增加夹持确认手段；命令发送完成不等于抓取成功，缺少真实反馈时必须明确估计值和验证方式。

2026-09-10 真实联调曾观察到旧张开波形延迟输出。候选根因是首版 FT232H 连续线程未按
波形时长限制生产速度，导致主机/USB 缓冲积压。现已改为按实际采样率节拍发送、缓存块从
10帧降至2帧（约40 ms），增加帧/字节计数、goal/有效脉宽日志及发送速率测试；组合脚本
默认启动夹爪节点，但拒绝与既有夹爪节点并存。修复后的真实多目标切换复测是 P09-04、
P09-12 关闭前的必测项。

### 6.7 坐标系与标定链

统一使用以下 TF：

```text
world / table
└── base_link
    └── ... -> wrist3_link
        └── flange_link
            ├── camera_link
            │   └── camera_color_optical_frame
            └── gripper_base_link
                └── grasp_frame (= 实际TCP)
```

眼在手上的物体转换：

```text
T_base_object = T_base_flange × T_flange_camera × T_camera_object
```

2026-09-10 当前实机已确认：`wrist3_link → flange_link` 为沿 wrist3 自身 +Z 的 0.1 m 固定变换；
正式 `flange_link → camera_link` 来自 40 组 ChArUco 数据，文件为
`evidence/P13/manual_30pose_20260910/flange_to_dabai_dcw2.yaml`。该 TF 已在系统重启后加载并验证唯一，
不能再同时发布 `camera_housing_link → camera_link` 预览变换。

必须避免以下错误：

- mm 与 m 混用；
- degree 与 radian 混用；
- 相机光学坐标与 ROS 普通坐标混用；
- 把 `wrist3_link`、法兰中心和夹爪 TCP 当成同一点；
- 读取图像时使用了另一个时刻的机器人 TF；
- 修改工具负载后未在 WebAPP/SDK 应用；
- 手眼矩阵方向取反。

推荐标定流程：

1. 固定 ChArUco 板到工作台；
2. 验证相机出厂内参与 D2C 后的 `CameraInfo`；
3. 低速移动到至少 20 个覆盖视场、距离和姿态的采样位；
4. 同步记录 `base -> flange` 与 `camera -> target`；
5. 使用 MoveIt Calibration ROS 2 分支；若兼容性不满足，则只用 OpenCV 官方 `calibrateHandEye` 求解；
6. 将结果保存为带设备序列号、支架版本和时间戳的 YAML；
7. 用未参加求解的 10 个验证姿态测量已知点，达到误差门槛后再抓取。

### 6.8 第一版感知不用神经网络

建议按以下顺序实施：

1. 订阅彩色图、对齐深度和动态 `CameraInfo`；
2. 对工作区做 ROI；
3. RANSAC 去除桌面平面；
4. 通过 HSV 颜色或已知尺寸几何分割积木；
5. 用目标区域 5×5 或更大有效深度中位数，过滤 0、NaN、边缘飞点和范围外点；
6. 反投影得到相机坐标 3D 点；
7. 经 TF 变换到 `base_link`；
8. 连续 5 帧位置稳定后发布目标，附带置信度、时间戳、尺寸和抓取轴；
9. 目标超时、深度无效或位置越界时拒绝任务。

第一版采用竖直向下的 top-down grasp。先把目标 yaw 限制为 0° 或 90°，再逐步增加平面角度估计。这样可以尽早验证坐标链，而不是把问题混入识别模型。

### 6.9 MTC 抓取任务阶段

优先从法奥官方 `fairino_mtc_demo` 修改，阶段建议为：

1. `Preflight`：控制模式、急停、使能、错误码、相机新鲜度、标定版本、负载/TCP 检查；
2. `CurrentState`：获取真实关节状态；
3. 更新工作台、箱体、料框、相机支架、夹爪和目标碰撞体；
4. 打开夹爪；
5. 连接到预抓取姿态（OMPL/Pilz PTP）；
6. 直线接近目标（MTC Cartesian/Pilz LIN）；
7. 关闭夹爪并等待完成/超时；
8. 将目标 attach 到 `grasp_frame`；
9. 直线提升；
10. 连接到料框上方；
11. 直线下降；
12. 打开夹爪；
13. detach 并更新 PlanningScene；
14. 直线撤退；
15. 返回观察位。

规划与执行分开：先得到完整可行解并显示，再经用户确认执行；自动化成熟后才允许无确认连续运行。抓取过程中不重新使用陈旧物体位姿，失败最多做一次“回观察位后重新感知”，不做原地盲重试。

### 6.10 工作区与运动限制采用“必要安全、避免重复收紧”

本项目不建立一个很小的笛卡尔硬盒子把机器人锁在里面。过去常见的“限制移动”往往不是一个限制造成，而是原厂关节软限位、WebAPP 安全墙、MoveIt 关节限制、碰撞体 padding、应用层 XYZ 范围同时生效，边界还不一致。多层重复收紧会让正常轨迹频繁失败，也难以判断到底是哪一层拒绝。

限制分为四层：

| 层级 | 本项目策略 | 是否硬拒绝 |
| --- | --- | --- |
| 机器人原厂安全能力 | 保留急停、安全停止、原厂关节软限位和经现场确认的碰撞检测；不为了绕过报错关闭这些能力 | 是 |
| 控制器自定义关节/空间限制 | 初期不额外缩小；启动脚本不得调用 `SetLimitPositive/SetLimitNegative` 改写关节范围。若现场确有不可进入的实体危险区，才单独配置并保存修改前后值 | 仅有实体风险时 |
| MoveIt PlanningScene | 只建模真实桌面、箱壁、底座、料框和末端实体；碰撞余量依据模型/标定误差设置，不叠加大而不明的通用 padding | 与真实碰撞相关时 |
| 感知 ROI / 抓取任务区 | 用于减少误识别和选择目标，不限制机器人返回观察位、绕行或维护动作；接近边缘先告警，明显不可达或进入实体障碍才拒绝 | 默认软提示 |

使用两个配置档：

- `commissioning`：保留原厂关节软限位、急停、安全停止和低速；不启用额外笛卡尔硬工作区，MoveIt 只放真实障碍物，任务边界以告警为主；
- `demo`：在 commissioning 已跑通的基础上，只增加已经由实物测量证明必要的实体禁入区，不复制另一套更小的 XYZ 盒子。

以下项目不能因“太严格”而关闭：实体急停、安全停止、原厂关节软限位、单运动源、TCP/负载正确性、真实桌面/箱壁碰撞。可以调松或取消的是未经测量的自定义安全墙、重复的应用层边界、过大的碰撞 padding 和把感知 ROI 错当成机械臂运动范围的逻辑。

出现“限制移动”时必须先定位拒绝源，不能同时修改多个参数：

1. 保存法奥主/子错误码、`RobotNonrtState`、当前六关节、TCP 和目标点；
2. 用 `GetJointSoftLimitDeg` 只读核对原厂/当前关节软限位，不直接写 `SetLimitPositive/Negative`；
3. 检查 WebAPP 是否启用了自定义安全墙、缩小后的关节范围或错误的安装方式；
4. 检查当前 tool/work/load 编号和 TCP 是否让控制器判定目标不可达；
5. 检查 MoveIt 是“无 IK”“起始状态碰撞”“路径碰撞”还是控制器拒绝；
6. 检查应用层是感知 ROI 告警、抓取任务区告警还是硬物理禁区；
7. 一次只调整一层，复测并记录；不能用扩大关节软限位或删除真实碰撞体掩盖模型/TCP 错误。

因此，`workspace_limits.yaml` 更名为语义更准确的 `motion_boundaries.yaml`：默认记录原厂限位快照、真实障碍物和任务告警区，不默认向控制器写入自定义硬限制。

## 7. 建议工作区结构

```text
fairino_ws/
├── src/
│   ├── fairino_msgs/                    # 已有，官方V3.9.2 + 最小补丁
│   ├── fairino_hardware/                # 已有，官方V3.9.2 + ARM64/安全补丁
│   ├── fairino_description/             # 补入官方V3.9.2原包
│   ├── fairino3mt_v6_moveit2_config/    # 法奥确认对应FR3-C的MoveIt基准
│   ├── fairino_mtc_demo/                # 已移植并适配FR3MT的官方循环抓取样例
│   ├── fairino_fr3c_description/        # 自有：FR3-C、相机、支架、夹爪叠加模型
│   ├── fairino_fr3c_moveit_config/      # 自有：基于官方FR3配置的派生包
│   ├── fr3c_gripper_driver/             # 自有：标准GripperCommand薄适配
│   ├── fr3c_camera/                     # 自有：DaBai启动、USB预检、RGB-D/点云RViz与验收门禁
│   ├── fr3c_calibration/                # 自有：采样配置、结果YAML、验证工具
│   ├── fr3c_perception/                 # 自有：RGB-D几何/颜色检测
│   ├── fr3c_manipulation/               # 自有：MTC任务、场景、PickPlace action
│   ├── fr3c_bringup/                    # 自有：mock/service/moveit互斥启动
│   └── fr3c_interfaces/                 # 仅保存确实缺少的少量接口
├── config/
│   ├── devices.yaml                     # SN/VID/PID/固件/夹爪/支架版本
│   ├── motion_boundaries.yaml            # 原厂限位快照、真实障碍物、任务告警区
│   └── acceptance_thresholds.yaml
├── calibration/
│   ├── camera_intrinsics/
│   ├── handeye/
│   └── tcp_payload/
├── bags/                                # 默认gitignore，仅留说明和元数据
├── docs/
└── scripts/                             # 一键构建、启动、验收，不放算法主体
```

官方包应保留 upstream 标签/commit 和本地补丁清单。建议后续每次升级都通过明确的 vendor 更新提交完成，不要直接跟踪 `main`。

## 8. 分阶段进度与门禁

> 工期为有效工程日，不含夹爪资料、转接板加工、官方样例索取等等待时间。

### 8.1 执行规则与状态口径

本节按参考文档的结构，把每个阶段都固定为“目标、边界、前置条件、官方复用、开发步骤、验收门槛、计划产物”。阶段编号也是日志、配置、测试报告和问题单的前缀。

- `[x]`：已有证据确认完成；
- `[ ]`：尚未完成或尚未实物验收；
- “软件通过”：只代表构建、静态检查、仿真或无动作测试通过；
- “实物通过”：代表在规定现场条件下完成真机/真设备验收并保存证据；
- “阻塞”：存在明确外部阻塞项，例如夹爪无规格书；不能用“暂时没做”代替阻塞；
- 任一门禁未通过，下一阶段最多做不依赖该门禁的离线工作，不得带病进入真机动作；
- 每项实物验收至少保留配置快照、原始日志、结果汇总和现场照片/视频中的两类证据；
- 已完成的环境底座不重复开发，只补“可重复验收脚本、配置冻结和证据归档”。

建议证据目录统一为：

```text
evidence/
└── Pxx/
    └── YYYYMMDD_HHMM/
        ├── manifest.yaml
        ├── commands.log
        ├── result.json
        ├── rosbag2_*/
        ├── config/
        └── media/
```

`manifest.yaml` 至少记录：机器人 SN/软件版本、SDK/ROS 包 commit、相机 SN/固件、夹爪型号、转接板版本、TCP/负载编号、配置 SHA256、操作者、测试区域和结论。

### 8.2 总阶段图与并行关系

```text
P00 基线冻结
  └─ P01 安全与工作区
      ├─ P02 法奥资源/FR3-C资料冻结 ── P07 模型与MoveIt仿真 ── P11 MoveIt真机 ─┐
      ├─ P03 相机无运动台架 ───────── P08 末端机械/布线 ── P12 装机相机 ── P13 手眼 ── P14 感知 ── P15 悬停 ─┐
      ├─ P04 夹爪无机器人台架 ─────── P08 末端机械/布线 ── P09 夹爪装机 ── P10 固定点抓取 ─────────────────┤
      └─ P05 只读健康门禁 ─────────── P06 SDK低速运动 ──────────────────────────────────────────────────────┤
                                                                                                             ▼
                                                                                                      P16 视觉抓放
                                                                                                             ▼
                                                                                                      P17 可靠性交付
```

P03、P04、P05 可在 P02 资料索取期间并行。P13 核心数值门禁及正式 TF 已通过，P14 可以使用当前 `base_link` 坐标链继续开发；在 P13 固定点/9 宫格尚未关闭前，不得声明完整抓取区的绝对坐标精度已验收。P16 必须同时依赖 P09、P10、P11、完整 P13、P14、P15。

### 8.3 阶段总览

| 阶段 | 目标 | 预计工期 | 当前状态 | 硬退出条件 |
| --- | --- | ---: | --- | --- |
| P00 | 冻结已有可用底座和证据格式 | 0.5～1 天 | 开发中 | 当前环境可从干净终端复现 |
| P01 | 建立必要安全与宽松限制基线 | 1～2 天 | 未开始 | 急停/停止有效；无重复、过紧的自定义运动限制 |
| P02 | 冻结 V3.9.2 法奥资源和 FR3-C 资料 | 1～2 天 | 开发中 | 来源/版本/本地补丁/缺件全部可追溯 |
| P03 | 相机不随机械臂运动的独立验收 | 2～3 天 | 开发中（DaBai 已接入；快速门禁和独立 RViz 可视化通过） | 型号明确，D2C、深度、时间戳、稳定性达标 |
| P04 | 夹爪不接机器人的电气/协议验收 | 2～5 天 | 开发中 | YDI-J22 PWM 点动已验证；待机械行程、供电保护、持续 PWM 方案及循环验收 |
| P05 | 把已有只读通信变成自动健康门禁 | 1 天 | 软件基础完成 | 连续采样和异常识别可重复 |
| P06 | 建立官方 SDK 低速运动基线 | 2～3 天 | 未开始 | MoveJ/MoveL/停止/断线行为实物通过 |
| P07 | 建立 FR3-C 派生模型和 MoveIt mock | 3～5 天 | 开发中（控制器一致 FK、法兰帧和基础碰撞检查已通过；mock/100 组规划未通过） | FK、限位、碰撞和 100 组规划通过 |
| P08 | 完成相机/夹爪/线缆机械集成 | 2～4 天 | 未开始 | 质量、质心、刚度、视场、线缆均通过 |
| P09 | 完成夹爪装机和 ROS 标准闭环 | 2～4 天 | 开发中：action/mock/诊断/MoveIt 控制器已实现，真实 FT232H action 待验收 | 装机 100 次开合及故障行为通过 |
| P10 | 完成 TCP/负载和无视觉固定点抓放 | 2～4 天 | 开发中 | 实测 TCP 已冻结；固定点 20 次至少 19 次成功 |
| P11 | 完成 ros2_control/MoveIt 低速真机 | 3～5 天 | 开发中（RViz 末端拖拽和单关节多组 Plan/Execute 真机通过；生命周期/故障门禁待完成） | 20 条审查轨迹无跳变、超限或阻塞 |
| P12 | 完成装机相机工作距离和支架复核 | 2～4 天 | 开发中（最终支架、相机配置和标定工作距离已用于 40 姿态采集） | 所有计划观察/抓取位都有有效深度 |
| P13 | 完成 eye-in-hand 手眼标定 | 3～5 天 | 开发中（25+15 独立验证及正式 TF 接入通过；固定点/9 宫格/拆装复验待完成） | 独立验证中位数 ≤3 mm、P95 ≤8 mm |
| P14 | 完成确定性 RGB-D 目标位姿 | 3～5 天 | 未开始 | 静态 P95 抖动 ≤5 mm，明显不可达/陈旧目标拒绝 |
| P15 | 完成视觉规划和无接触悬停 | 2～3 天 | 未开始 | 30 次目标上方悬停无误动作 |
| P16 | 完成 MTC 视觉抓放闭环 | 3～5 天 | 未开始 | 随机点 50 次至少 45 次成功 |
| P17 | 完成故障注入、100 循环和交付 | 3～5 天 | 未开始 | 100 循环 ≥95%，交付可复现 |

并行后推荐排期为 30～45 个有效工程日。P04 若需要重新设计隔离/供电板，采购和加工等待不计入该估算。

## P00：已有工程基线冻结

### 阶段目标

把当前已经完成的 ARM64 SDK、ROS 2 构建、机器人网络和只读通信固化成可复现基线，避免后续联调时反复怀疑底层环境。

### 阶段边界

P00 不发送有效运动，不连接未知夹爪，不升级控制器/相机固件，不把其他工作区 source 进本工作区。

### 已有基础

- [x] RK3588 / Ubuntu 22.04 / ROS 2 Humble 已确认；
- [x] `eth1=192.168.58.10/24` 到 `192.168.58.2` 已连通；
- [x] `fairino_msgs`、`fairino_hardware` 已干净构建；
- [x] V3.9.2 对应 ARM64 `libfairino.so.2.3.2` 已加载；
- [x] `/fairino_remote_command_service` 和 `/nonrt_state_data` 已发现；
- [x] `GetSDKVersion()` 与只读状态已验证；
- [x] 命令服务器退出双析构和 5 条运动参数未初始化问题已有修复记录。

### 开发步骤

- [ ] P00-01 保存当前 `git status --short`、HEAD、未提交补丁和文件 SHA256；
- [ ] P00-02 保存 ROS、ros2_control、MoveIt、编译器和内核版本；
- [ ] P00-03 从干净终端执行一次全量构建并归档 `colcon list`、构建日志；
- [ ] P00-04 检查源码和安装目录所有 ELF 架构，确认无 x86-64 动态库；
- [ ] P00-05 执行 `ldd` 检查，确认没有 `not found`；
- [ ] P00-06 保存专用网口地址、路由、ping 和控制端口连通性；
- [ ] P00-07 启动命令服务器，调用只读 API，正常 SIGINT 退出并检查无残留进程；
- [ ] P00-08 导出 ROS 节点、服务、话题和接口定义快照；
- [ ] P00-09 建立 `devices.yaml`、`acceptance_thresholds.yaml`、`issue_register.md` 初始文件清单；
- [ ] P00-10 明确当前用户改动与官方原版的 diff，后续不覆盖现有修改。

### 验收门槛

- [ ] 两个当前包从干净目录构建成功；
- [ ] SDK 架构、SONAME 和 SHA256 与交接文档一致；
- [ ] 连续启动/退出 10 次，无崩溃、内存损坏或残留进程；
- [ ] 只读连接成功且未记录任何运动调用；
- [ ] 任一开发者按 P00 记录可在新终端复现。

### 计划产物

- `baseline/P00_environment.yaml`
- `baseline/P00_files.sha256`
- `evidence/P00/<timestamp>/`
- 更新后的环境交接文档和未关闭问题表

## P01：必要安全、实验箱与宽松限制基线

### 阶段目标

在任何运动前明确操作者站位、停止方式、低速条件和真实障碍物，同时清理重复、过紧的自定义限制，使安全措施不会无理由阻断正常路径。

### 阶段边界

本阶段只做现场检查、WebAPP 配置记录和不产生位移的状态验证。默认不新增笛卡尔硬工作区，也不改写原厂关节软限位。软件安全检查不替代机器人系统风险评估、实体急停和现场监护。

### 前置条件

- P00 完成；
- 操作者能接触 WebAPP、控制箱和实体急停；
- 机器人周围可清空，底座已可靠固定。

### 法奥资源/API 复用

- 法奥安装和安全说明；
- `RobotNonrtState.emg`、`safety_stop1_state`、`safety_stop2_state`、`collision_err`；
- `GetSafetyStopState`、`GetRobotErrorCode`、`SetSpeed`、`SetAnticollision`；
- `StopMotion` 只作为软件停止；实体急停仍需独立验证。

### 开发与现场步骤

- [ ] P01-01 拍摄底座、控制箱、末端、急停、围挡和作业区全景；
- [ ] P01-02 检查底座紧固、供电、接地、控制柜散热和线缆应力；
- [ ] P01-03 标记机器人最大可达区、教学低速区、操作者站位和禁止进入区；
- [ ] P01-04 测量桌面、箱壁、底座、料框的 `base_link` 近似坐标和尺寸；
- [ ] P01-05 记录 WebAPP 当前安装方式、工具号、工件号、负载号、碰撞等级和全局速度；
- [ ] P01-06 记录 WebAPP 当前自定义安全墙、关节正/负限位和其他运动限制；先读出并截图，不直接修改；
- [ ] P01-07 区分原厂关节软限位、现场自定义限制和应用层任务范围；
- [ ] P01-08 若曾频繁出现“限制移动”，保存一个可复现目标及完整主/子错误码；
- [ ] P01-09 将首次运动速度限制为 5%～10%，保存修改前后截图；
- [ ] P01-10 确认拖动示教已退出、作业程序停止、无远程自动任务；
- [ ] P01-11 验证实体急停按下/释放时状态字段正确变化；释放后不得自动恢复运动；
- [ ] P01-12 验证安全停止输入状态，未接安全设备也要明确记录真实配置；
- [ ] P01-13 验证 WebAPP 停止、SDK `StopMotion` 和控制箱断使能的职责差异；
- [ ] P01-14 定义每次真机动作前口头确认和现场监护规则；
- [ ] P01-15 记录观察位、测试位和真实实体禁入区；普通任务区边缘只作告警；
- [ ] P01-16 默认使用 `commissioning` 配置：不额外启用笛卡尔硬工作区；
- [ ] P01-17 只有存在桌外跌落、箱壁撞击等真实风险时，才增加最小必要硬禁区；
- [ ] P01-18 将原厂限位快照、真实障碍物和任务告警范围写入 `motion_boundaries.yaml`；
- [ ] P01-19 建立真机测试 checklist，未全勾选不允许运动。

### 验收门槛

- [ ] 急停、软件停止、下使能的状态变化均有记录；
- [ ] 速度、碰撞等级、工具/工件/负载编号已记录；
- [ ] 原厂限位与自定义限制已区分，自定义限制不存在重复收紧；
- [ ] `commissioning` 模式没有额外笛卡尔硬盒子；
- [ ] 工作台、箱壁和料框尺寸可用于 PlanningScene；
- [ ] 起始观察位不接近关节限位、奇异点或箱壁；
- [ ] 任何参与者都能在 2 秒内触达停止装置。

### 计划产物

- `config/motion_boundaries.yaml`
- `config/safety_profile.yaml`
- `docs/hardware/P01_workcell_safety.md`
- `evidence/P01/<timestamp>/`

## P02：法奥 V3.9.2 配套资源与 FR3-C 资料冻结

### 阶段目标

明确“哪些直接使用法奥官方资源、哪些只参考、哪些必须自研适配”，并为每份资源固定版本。

### 阶段边界

保持控制器 V3.9.2，不因在线手册已更新而直接升级控制器或混用新版二进制。新版本 FR3-C Gazebo/模型资源只能先用于几何对照，不能未经验证替换 V3.9.2 控制链。

### 法奥资源/API 复用

- ROS 2 标签 `V3.0.0_RobotV3.9.2`；
- C++ SDK 标签 `v2.3.2-robot3.9.2`；
- 官方 `fairino_description`、`fairino3mt_v6_moveit2_config`；
- 已从本机 Robot V3.7.7 资源移植的 `fairino_mtc_demo`；
- 在线手册中新增的 `fr3c_ros2_control` 可作为 FR3-C 仿真几何线索，但必须记录来源版本并单独验证。

### 开发步骤

- [ ] P02-01 记录两个官方仓库 URL、tag、commit、许可证和获取日期；
- [ ] P02-02 用 `git ls-tree` 确认 V3.9.2 标签确有 FR3 MoveIt 和 description 包；
- [ ] P02-03 把官方包原样补入工作区，首次提交不混入项目定制；
- [ ] P02-04 建立 `upstream_manifest.yaml`，记录每个官方目录来源；
- [ ] P02-05 保存当前工作区相对官方标签的补丁清单；
- [ ] P02-06 向法奥索取 FR3-C 对应 STEP、尺寸图、URDF/xacro、DH/关节限制和惯量资料；
- [x] P02-07 从本机 Robot V3.7.7 资源取得完整 `fairino_mtc_demo`，迁入工作区并完成 FR3MT/ARM64 编译适配；
- [x] P02-08 经法奥人员确认 FR3-C 在历史 ROS 资源中使用 `fairino3mt` 命名；该结论仅用于资源追溯，不再等同于运动学模型选择；
- [ ] P02-09 向法奥确认 V3.9.2 `ServoJ` 推荐周期及 ros2_control 激活/退出序列；
- [ ] P02-10 核对本地 `robot.h`，生成项目会用到的 API 白名单；
- [ ] P02-11 将在线最新版手册标注为“功能解释来源”，本地 V3.9.2 头文件标注为“签名真值来源”；
- [ ] P02-12 建立升级禁令：未单独立项前不升级控制器、不追 ROS 仓库 `main`；
- [ ] P02-13 为无法取得的资料建立阻塞问题和替代验证方法。
- [ ] P02-14 验证 V3.7.7 MTC 样例与 V3.9.2 `fairino_hardware` 的真机运行兼容性。

### 验收门槛

- [ ] 官方包来源和本地改动 100% 可追溯；
- [x] FR3-C 的历史 ROS 型号命名已由法奥人员确认；运动学已另行按控制器 FK 实测选择，不以名称推断；
- [ ] 所有计划使用的 SDK API 都能在 V3.9.2 `robot.h` 找到；
- [x] `fairino_mtc_demo` 已获得完整源码并完成 FR3MT/ARM64 编译适配；
- [ ] 新版资源未进入 V3.9.2 真机运行时。

### 计划产物

- `baseline/upstream_manifest.yaml`
- `docs/design/P02_fairino_resource_matrix.md`
- `patches/fairino_v3_9_2/README.md`
- 法奥售后问题清单和答复归档

## P03：RGB-D 相机无运动台架测试

### 阶段目标

在相机还没有装上机械臂、机器人完全不运动时，确认准确型号、官方驱动路线、可用工作距离、D2C 和长期稳定性。

### 阶段边界

相机放置在独立支架上测试；不做手眼标定、不发布 `base_link` 目标、不因驱动能出图就宣称满足抓取精度。

### 前置条件

- P00 完成；
- DaBai DCW2、线缆和稳定 USB 2.0 / 480 Mbit/s 端口可用；
- 能读取相机铭牌。

### 官方资源复用

- 使用本工作区已在 ARM64 构建并验证兼容 DaBai DCW2 的 OrbbecSDK ROS 2 Wrapper；
- 使用 `list_devices_node`、`list_camera_profile_mode_node`、`list_depth_work_mode_node`；
- 使用与准确型号一致的 launch 和 `device_preset`；
- D2C 使用官方 `depth_registration`/对齐输出及测试叠加图，不自行实现深度注册。

### 开发与测试步骤

- [ ] P03-01 拍摄相机正反面铭牌，记录商品名与完整产品型号；
- [x] P03-02 记录 `lsusb`、VID/PID、USB 拓扑、序列号和固件版本；
- [ ] P03-03 已用厂商工具枚举 DaBai 设备；仍需归档该设备完整支持的 stream profile 列表；
- [ ] P03-04 对照官方支持矩阵选择 wrapper 主版本、分支和 commit；
- [x] P03-05 安装官方 udev 规则，验证普通用户可访问；不使用永久 root 运行规避权限；
- [x] P03-06 首次测试不升级固件；若确需升级，先保存旧版本和官方升级/回退说明；
- [x] P03-07 固定 DaBai 彩色、深度、FPS、格式和硬件 D2C 基线配置；
- [ ] P03-08 启动彩色流，核对分辨率、FPS、曝光和 50 Hz 防闪烁；
- [x] P03-09 启动深度流，核对深度单位、`CameraInfo`、畸变模型和光学坐标系；
- [ ] P03-10 开启 D2C，使用边缘清晰的平面物体检查彩深重合；
- [ ] P03-11 在 150、200、250、300、400 mm 等候选距离测有效深度率；低于型号最小距离的点跳过并记录；
- [ ] P03-12 对固定平面采集 300 帧，计算中心/边缘 ROI 的均值、标准差、P95 和无效像素率；
- [ ] P03-13 用已知距离/尺寸物体检查深度比例和空间尺寸，不只看画面；
- [ ] P03-14 测试不同材质、颜色、照度和桌面背景；
- [ ] P03-15 连续运行至少 60 分钟，记录掉帧、USB reset、CPU、内存和温度；
- [ ] P03-16 冷启动/热插拔 10 次，确认相同 SN、命名空间和话题；
- [ ] P03-17 为 DaBai 重新录制一组彩色、对齐深度、`CameraInfo`、TF 的 rosbag；旧 Gemini bag 不作为替换相机证据；
- [ ] P03-18 形成“抓取可用工作距离”和推荐 preset 结论，不凭产品宣传参数下结论。
- [x] P03-19 启动前只读核对 DaBai SN、`06a0/0561` 两个 USB PID、480 Mbit/s 和设备节点权限；
- [x] P03-20 以 60 ms 配对上限完成 20 秒快速门禁，确认 RGB、D2C 深度、CameraInfo、TF、时间戳和非零深度均可用。
- [x] P03-21 建立不依赖手眼标定的相机独立 RViz 入口，实机确认 RGB、对齐深度、彩色点云和相机内部 TF 可消费；该项不替代 P03 距离精度和长期稳定性门禁。

### 验收门槛

- [ ] 准确型号、SN、固件、VID/PID 已实测；wrapper/SDK 压缩包的可追溯上游 commit 仍待补齐；
- [ ] 计划抓取距离内中心 ROI 有效深度率至少 95%；
- [ ] 静止平面深度 P95 波动不大于 5 mm；若达不到则调整 preset/距离/曝光后重测；
- [ ] 彩深边缘没有系统性明显错位，D2C 配置与 `CameraInfo` 一致；
- [ ] 时间戳单调、TF 完整，60 分钟无节点退出和持续 USB reset；
- [ ] 10 次重启均绑定同一设备且不依赖手改代码。

### 计划产物

- `src/fr3c_camera/config/dabai_dcw2.yaml`
- `src/fr3c_camera/config/dabai_dcw2_rviz.yaml`
- `src/fr3c_camera/launch/camera_rviz.launch.py`
- `src/fr3c_camera/rviz/dabai_dcw2.rviz`
- `src/fr3c_camera/fr3c_camera/preflight.py`
- `docs/hardware/DaBai_DCW2_接入说明.md`
- `docs/DaBai_DCW2_RViz与MoveIt集成技术交接.md`
- `evidence/camera/20260909_dabai_migration/`；后续补 DaBai 可回放 rosbag

## P04：夹爪身份、电气与协议无机器人台架测试

### 阶段目标

在已完成 YDI-J22 身份、三线 PWM 控制和 FT232H AD7 动作验证的基础上，完成安全机械行程、供电保护、反馈能力和可靠性验收，冻结外置硬件定时 PWM 控制器/MCU 方案。当前为“开发中”，不等于 P04 实物通过。

### 阶段边界

在完整型号、供电、峰值电流、线序和信号协议未确认前，禁止接 FR3-C M12、控制箱 IO 或 RK3588 GPIO。万用表猜出的线序不能直接作为接线依据。

### 已有资源与路线

- 复用 `test/jiazhua` 台架脚本、软件自检和技术交接记录；参数和边界见第 6.6 节。
- 已确认三线 PWM，不再把法奥夹爪 API、24 V Tool IO 或 RS485 作为当前设备待选路线。
- 正式控制器/MCU 尚未选定；台架工具不提供持续夹持和真实夹持反馈。

### 开发与测试步骤

- [ ] P04-01 已知型号 YDI-J22 四指；补齐品牌来源、购买链接、铭牌和插头两面照片归档；
- [ ] P04-02 归档原厂说明书/客服书面答复和接线图；5 V、约 1.6 A 堵转为现有资料值，补峰值/堵转实测；
- [ ] P04-03 棕 GND、红独立 +5 V、黄 AD7 及共地已确认；补插针方向照片与正式线束编号；
- [x] P04-04 已确认 50 Hz 三线 PWM，卖家确认 500～2500 µs，FT232H AD7 实物点动成功（证据见第 6.6 节交接链接）；
- [ ] P04-05 当前无真实开度/夹持完成反馈；确定新增电流/力/视觉确认方案及其能力边界；
- [ ] P04-06 断电测量线间电阻/短路，仅作为手册交叉检查；
- [ ] P04-07 补充台架电源型号、限流设定和上电记录；正式控制器首次上电从保守限流开始；
- [ ] P04-08 记录静态、开合、堵转瞬态电流和动作时间；
- [ ] P04-09 用示波器/逻辑分析仪实测 PWM 电平、频率、脉宽和持续输出稳定性；
- [ ] P04-10 验证失电状态、上电默认动作和信号悬空状态；
- [ ] P04-11 标定并记录安全闭合/张开值；已测试到 500/1700 µs 不直接作为生产端点；
- [ ] P04-12 在张开端逐步标定并记录机械干涉、抖动、异响和压降；2500 µs 尚未验收；
- [ ] P04-13 设计独立 5 V 稳压供电、限流/保险、反接保护和共地策略；
- [ ] P04-14 选定硬件定时 PWM 控制器/MCU 与 USB/UART 链路，验证持续 50 Hz 输出；
- [ ] P04-15 固定控制器通信协议、启动状态、看门狗、超时/取消和掉线行为；
- [ ] P04-16 画出“电源—保护—控制—夹爪—FR3-C/RK3588”的完整接线图；
- [ ] P04-17 对转接方案做同伴复核，未知项必须为 0；
- [ ] P04-18 台架完成 100 次开合和 10 次堵转/空夹测试；
- [ ] P04-19 选择最终接入路线并冻结 BOM、原理图和固件版本。

### 验收门槛

- [ ] 每根线、每个电平、最大电流和协议均有原厂或实测证据；
- [ ] 首次接 FR3-C 前接线图已评审，含保险、隔离、续流和应力释放；
- [ ] 台架 100 次开合无复位、异常发热或失控；
- [ ] 堵转不会损坏电源/驱动，超时后可进入安全状态；
- [ ] 已明确是否能获得真实夹持反馈；若不能，风险已进入 P09/P17 门禁。

### 计划产物

- `config/devices/gripper.yaml`
- `docs/hardware/P04_gripper_characterization.md`
- `hardware/gripper_adapter/` 设计文件清单
- `evidence/P04/<timestamp>/`

## P05：机器人只读健康检查与运动前门禁

### 阶段目标

把当前“能连接、能读状态”提升为每次动作前自动运行的 preflight，先阻断错误状态，再谈运动。

### 阶段边界

不发送 `RobotEnable(1)`、`ResetAllError`、`MoveJ`、`MoveL`、`ServoJ` 等可能改变机器人状态/位置的命令。错误清除必须先由操作者理解原因。

### 法奥资源/API 复用

- `/nonrt_state_data`；
- `GetSDKVersion`、`GetActualJointPosDegree`、`GetActualTCPPose`、`GetActualToolFlangePose`；
- `GetRobotMotionDone`、`GetRobotErrorCode`、`GetSafetyStopState`；
- `GetTargetPayload`、`GetTargetPayloadCog`、`GetTCPOffset`。

### 开发步骤

- [ ] P05-01 定义 preflight 输入、返回码和 `READY/BLOCKED/STALE` 状态；
- [ ] P05-02 检查控制器 IP、SDK 连接、控制器版本和状态话题频率；
- [ ] P05-03 检查 `rbt_main_code/rbt_sub_code`、`emg`、两路安全停止；
- [ ] P05-04 检查 `rbt_enable_state`、`robot_mode`、`motion_done`、`collision_err`；
- [ ] P05-05 检查关节角、TCP、法兰位姿均为有限值且在预设范围；
- [ ] P05-06 检查当前 tool/work/load 编号与项目配置一致；
- [ ] P05-07 检查负载重量/质心与末端版本一致，未知时只允许只读模式；
- [ ] P05-08 检查状态时间戳/接收时间和更新频率，陈旧数据不得视为安全；
- [ ] P05-09 保存一次机器可读 `preflight.json` 和人可读摘要；
- [ ] P05-10 连续运行 30 分钟，统计状态频率、断点和错误；
- [ ] P05-11 停止命令服务器、拔网线等方式注入通信中断，验证门禁立即变为 BLOCKED；
- [ ] P05-12 恢复连接后要求重新完整 preflight，不沿用旧的 READY；
- [ ] P05-13 为每个阻塞字段给出明确处置建议，但不自动清错/使能；
- [ ] P05-14 将 preflight 作为 P06 之后所有真机启动的强制依赖。

### 验收门槛

- [ ] 连续 100 次 preflight 结果稳定且字段完整；
- [ ] 任一错误、急停、安全停止、状态陈旧或非有限值都会拒绝运动；
- [ ] 断网/节点退出能在规定超时内识别；
- [ ] preflight 本身不调用任何运动或自动复位 API；
- [ ] 每次结果都可追溯到状态原始值。

### 计划产物

- `scripts/fr3c_preflight`
- `config/preflight_rules.yaml`
- `docs/test/P05_readonly_preflight.md`
- `evidence/P05/<timestamp>/`

## P06：法奥 SDK 低速基础运动验收

### 阶段目标

先用当前已跑通的法奥 SDK/ROS 服务路径建立最小、可停止、可重复的单指令运动基线，为后续 MoveIt 排除机器人通信和基础运动问题。

### 阶段边界

只在 `control_mode:=sdk_service` 下运行；不启动 ros2_control/MoveIt 轨迹控制，不安装相机/夹爪或仅保留经过 P08 验收的假负载。所有目标点先由 WebAPP/离线检查确认。

### 前置条件

- P01、P05 实物通过；
- 现场净空、急停可触达、操作者明确确认；
- TCP/负载未标定时使用空法兰或已知保守配置，不做接触任务。

### 法奥资源/API 复用

- `SetSpeed`、`Mode`、`RobotEnable`；
- `MoveJ`、`MoveL`、`GetRobotMotionDone`；
- `GetActualJointPosDegree`、`GetActualTCPPose`、`GetRobotErrorCode`；
- `StopMotion`；错误原因明确后才可人工决定 `ResetAllError`。

### 开发与现场步骤

- [ ] P06-01 编写单运动源检查，发现 ros2_control/其他 SDK 客户端即拒绝启动；
- [ ] P06-02 保存动作前 preflight 和完整六关节/TCP 起始状态；
- [ ] P06-03 设全局速度 5%，确认 WebAPP 显示与 API 设置一致；
- [ ] P06-04 选择远离箱壁、奇异点和关节限位的测试姿态；
- [ ] P06-05 对单个关节规划 ±1° 小步 `MoveJ`，先执行一次再检查方向；
- [ ] P06-06 六个关节分别做保守小步测试，记录目标、实到、耗时、返回码；
- [ ] P06-07 执行已审查的多关节短 `MoveJ`，使用到位模式并等待 `motion_done`；
- [ ] P06-08 在安全方向执行 5 mm `MoveL`，核对坐标轴方向；
- [ ] P06-09 分别验证 X/Y/Z 小位移，不在首次测试中改变姿态；
- [ ] P06-10 验证运动中 `StopMotion`，记录停止距离和最终状态；
- [ ] P06-11 在静止时断开网络，确认门禁和恢复流程；
- [ ] P06-12 仅在风险评估允许时验证运动中通信异常；否则记录为 P17 受控测试；
- [ ] P06-13 任何 API 非 0、主/子错误码变化或 motion_done 超时都立即终止本轮；
- [ ] P06-14 完成 50 次观察位—测试位低速往返；
- [ ] P06-15 比较命令目标、`GetActualJointPosDegree` 和 `GetActualTCPPose` 重复性；
- [ ] P06-16 固化安全测试点，不把临时点位写进通用生产配置；
- [ ] P06-17 记录手动、自动模式切换要求及重启后的默认状态。

### 验收门槛

- [ ] 六轴方向与模型预期一致；
- [ ] 50 次往返全部返回成功，无主/子错误码、碰撞或超时；
- [ ] `MoveJ`、`MoveL` 到位判断可重复，运动后状态无 NaN/跳变；
- [ ] `StopMotion` 和急停均能终止运动，释放后不自动续跑；
- [ ] 任一异常都要求重新 P05，而不是直接重发动作。

### 计划产物

- `config/poses/sdk_test_poses.yaml`
- `scripts/acceptance/P06_sdk_motion`
- `docs/test/P06_sdk_motion_report.md`
- `evidence/P06/<timestamp>/`

## P07：法奥模型、FR3-C 派生描述与 MoveIt mock

### 阶段目标

保留法奥 MoveIt 配置与控制接口，以当前 V3.9.2 控制器 FK 实测选择 FR3-C 运动学，建立包含末端设备和工作台碰撞体的仿真系统。

### 阶段边界

P07 只运行 fake/mock 或仿真控制器，不连接真机硬件插件。官方包作为只读基准，自有派生包承载 FR3-C、相机、夹爪和场景定制。

### 前置条件

- P02 完成，至少拿到 FR3-C 尺寸/法兰/关节范围确认；
- P01 工作区尺寸可用；
- P03/P04 即使未完成，也可先使用保守包络体。

### 法奥资源复用

- `fairino_description`；
- `fairino3mt_v6_moveit2_config` 的 URDF xacro、SRDF、KDL、Pilz、controller 配置和 RViz；
- 本地 V3.9.2 `GetForwardKin` 用于运动学真值对比；
- 若取得法奥 FR3-C 专用模型，优先使用其几何数据，不手工重画连杆。

### 开发步骤

- [ ] P07-01 将两个官方包原样构建，启动官方 `demo.launch.py`；
- [x] P07-02 记录官方组名、tip link、关节名、controller 名和单位；当前规划 tip 已改为 `flange_link`，六轴保持 `joint1`～`joint6`；
- [x] P07-03 已在工作区 description 包建立 `fairino3c_v6.urdf` 兼容副本，并由组合 xacro 引用；只读上游资源池未修改；
- [x] P07-04 明确定义 `base_link`、`wrist3_link`、`flange_link` 和实际抓取末端 `grasp_frame`；本项目直接以 `grasp_frame` 表示工具 TCP，不再增设含义重复的 `tool0`；
- [ ] P07-05 核对 FR3-C DH/连杆长度、关节正方向、零位和关节软限位；
- [x] P07-06 为 FR3-C 本体使用官方 mesh/collision 几何，末端相机与夹爪继续使用保守占位碰撞体；
- [ ] P07-07 加入相机、支架、夹爪、转接板和线缆保守包络；
- [ ] P07-08 加入实验箱、桌面、底座、料框和固定障碍物；
- [x] P07-09 已在现有 `fairino3mt_v6_moveit2_config` 工程入口中派生 URDF xacro/SRDF，保留控制器、KDL、Pilz 和 RViz 配置；
- [x] P07-10 已审查 SRDF 禁碰撞对并完成离线自碰撞/障碍碰撞回归；后续精细末端模型仍需复审；
- [ ] P07-11 核对官方 100 Hz controller 配置与本项目后续真机周期，但此阶段不下发 `ServoJ`；
- [x] P07-12 使用 40 组历史关节角重建控制器 `GetForwardKin`，并增加 3 个现场停稳姿态直接核对；
- [x] P07-13 已定位原 `fairino3_mt_v6` 运动学不匹配，切换为标准 `fairino3_v6` 链并定义 +Z 0.1 m 的 `flange_link`；
- [ ] P07-14 建立观察位、维护位、抓取准备位命名姿态；
- [ ] P07-15 生成 100 组工作区内目标规划，检查关节限位、自碰撞和箱壁碰撞；
- [ ] P07-16 对超原厂关节限位、桌面/箱壁实体内部的目标确认规划前拒绝；仅超出常用抓取区的目标只提示；
- [ ] P07-17 测试 OMPL 连接段和 Pilz PTP/LIN 段；
- [ ] P07-18 保存 RViz 场景截图、规划轨迹和失败样例。
- [x] P07-19 建立 MoveIt MotionPlanning 与 DaBai RGB/深度/彩色点云的组合 RViz 配置和启动入口；未标定时仅允许明确标记的外壳估位 TF，当前已切换为通过验收的正式手眼 TF；本项不代表真机轨迹门禁通过。

### 验收门槛

- [x] 40 组重建 FK 最大平移/姿态差 0.00981 mm / 0.000443°；3 个现场姿态最大 0.01062 mm / 0.000792°，满足 ≤1 mm / ≤0.2°；
- [x] 六关节名与当前 `fairino_hardware`/controller 配置使用的 `joint1...joint6` 一致；
- [ ] 末端所有实体均进入 collision model；
- [ ] 100 组有效目标规划无穿模和越界；
- [ ] 10 组涉及真实碰撞、原厂关节越限或明显不可达的目标均在执行前被拒绝；
- [x] 标准 `fairino3_v6` 与项目 `fairino3c_v6` 的差异限定为 robot/joint 命名兼容，模型选择和核对证据已归档。

### 计划产物

- `src/fairino_description/urdf/fairino3c_v6.urdf`
- `src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.urdf.xacro`
- `src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.srdf`
- `src/fairino3mt_v6_moveit2_config/config/moveit_camera.rviz`
- `src/fairino3mt_v6_moveit2_config/launch/camera_moveit_demo.launch.py`
- `docs/test/P07_model_fk_collision.md`
- `evidence/P07/<timestamp>/`

## P08：末端机械、载荷预算、相机支架与线缆集成

### 阶段目标

完成相机、支架、夹爪和转接板的可重复装配，使视场、TCP、刚度、负载和线缆都满足后续标定与抓取。

### 阶段边界

本阶段以机械安装和静态检查为主。支架版本未冻结前不做正式手眼标定；P04 未通过前夹爪电气仍不接机器人。

### 前置条件

- P01 工作区尺寸完成；
- P03 得到相机工作距离；
- P04 得到夹爪外形、供电和接线路线；
- P07 有可更新的末端模型。

### 法奥资源/API 复用

- 法奥 FR3-C 额定负载、末端法兰/M12 安装资料；
- `SetLoadWeight`、`SetLoadCoord` 仅在 P10 使用，本阶段先完成测量；
- 法奥官方 STEP/法兰图优先用于支架设计。

### 开发与现场步骤

- [ ] P08-01 建立末端 BOM，逐项称量相机、支架、夹爪、转接板、线缆活动段和最大工件；
- [ ] P08-02 估算/测量整体质心相对法兰的 XYZ；
- [ ] P08-03 按 FR3-C 额定 3 kg 做预算，不把扩展 5 kg 当常规能力；
- [ ] P08-04 预留动态和估算误差裕量，超预算立即减重/缩短支架；
- [ ] P08-05 设计法兰侧独立相机支架，夹爪拆装不改变相机安装基准；
- [ ] P08-06 使用定位销、止口或明确扭矩的紧固方案提高重装重复性；
- [ ] P08-07 确定相机视轴、夹爪中心线和计划 150～450 mm 观察区；
- [ ] P08-08 在 CAD/RViz 检查相机是否被夹爪手指、线缆或工件遮挡；
- [ ] P08-09 检查最大开口、闭合、抓持工件时的机械干涉；
- [ ] P08-10 规划 USB、电源/信号线的固定点、弯曲半径和应力释放；
- [ ] P08-11 手动慢速转动 J4/J5/J6 全计划范围，观察线缆是否拉紧、摩擦或进入夹爪区；
- [ ] P08-12 建立线缆包络并加入碰撞/禁转范围；
- [ ] P08-13 在相机外壳和支架上做版本/方向标记；
- [ ] P08-14 拆装支架 5 次，测量基准点重复性；
- [ ] P08-15 更新 URDF visual/collision/inertial 和 `devices.yaml`；
- [ ] P08-16 完成机械与电气图纸评审后再进入装机通电。

### 验收门槛

- [ ] 末端总质量/质心在 FR3-C 允许范围并留有明确裕量；
- [ ] 计划动作范围内无干涉、无拉线、无接头受力；
- [ ] 相机能看见夹爪前方全部计划抓取区；
- [ ] 支架拆装 5 次后基准重复性满足手眼误差预算；
- [ ] 末端实际版本与 URDF/配置版本一一对应。

### 计划产物

- `hardware/end_effector/BOM.yaml`
- `hardware/end_effector/assembly.*`
- `config/devices.yaml` 更新
- `docs/hardware/P08_end_effector_integration.md`
- `evidence/P08/<timestamp>/`

## P09：YDI-J22 装机、硬件 PWM 适配与 ROS 集成

### 阶段目标

把 YDI-J22 和 FT232H 安全接到整机，对上提供标准 `GripperCommand`，对下实现持续 PWM、标定映射、保护与显式开环状态。2026-09-10 已完成软件实现和 mock 验证，真实硬件 action 待操作者监护下验收。

### 阶段边界

先测试夹爪，机械臂保持安全静止。未证明失电/堵转安全前，不执行携带物体的机械臂运动。

2026-09-10 推进结果：已直接实现自有薄驱动而非等待不匹配的第三方协议节点。mock action、越界拒绝、诊断、URDF/SRDF 和 MoveIt controller 已通过离线验证；FT232H 未解锁模式确认不占用设备且拒绝 goal。真实输出仍不默认启用，也不把“命令发送完成”声明为“夹持成功”。

当前实现顺序：纯映射/PWM 单元测试 → mock action → MoveIt 接口 → FT232H 未解锁测试均已完成；下一步在机械臂静止时执行真实 550/1650 µs action，再做中间开度、cancel、断连和持续保持验收，最后进入 MTC。

### 前置条件

- P01、P04、P08 通过；
- 接线图、保险/隔离、线束和外壳已完成；
- 夹爪真实控制路线已冻结。

### 资源与接口复用

- 从 `test/jiazhua` 复用已验证的方向和参数依据，生产脉宽使用 P04 冻结的安全值。
- 对下使用已选硬件 PWM 控制器协议；不调用法奥夹爪激活/运动 API，也不将 `RobotNonrtState` 的夹爪字段当作 YDI-J22 反馈。
- 对上实现标准 action 和诊断；仅在实际传感器提供时发布测量位置、电流等，指令估计值须显式区分。

### 开发与测试步骤

- [ ] P09-01 上电前逐针导通/短路检查，确认机器人末端禁止热插拔；
- [ ] P09-02 限流条件下首次装机上电，机械臂保持下使能或静止；
- [ ] P09-03 加载控制器身份/固件、供电与 PWM 配置，核对 P04 安全脉宽和方向；
- [ ] P09-04 验证控制器初始化、通信握手、持续 PWM 与看门狗，启动时不得意外跳转端点；
- [ ] P09-05 在已验收行程内以保守脉宽渐变测试 0%、25%、50%、75%、100% 开度，不宣称具备未实现的力控制；
- [x] P09-06 已核对闭合/张开方向，实测0～130 mm净开度，并将六点分段线性映射写入 ROS 位置命令；
- [ ] P09-07 实测开合时间，设置动作 timeout 和 settle time；
- [ ] P09-08 有反馈时采集 motion_done、position、current、voltage、temp；
- [x] P09-09 无反馈路线以 `open_loop_estimated`、NaN effort 和诊断区分“命令估计到达”与“夹持已确认”；
- [x] P09-10 实现 `control_msgs/action/GripperCommand` 薄封装并接入 MoveIt simple controller manager；
- [ ] P09-11 增加 busy、timeout、fault、cancel、not_active、communication_lost 错误；
- [ ] P09-12 重复 goal、并发 goal 和取消必须幂等，不产生第二次意外动作；
- [ ] P09-13 测试空载 100 次完整开合；
- [ ] P09-14 测试软物体、标准块、空夹和受控堵转各 10 次；
- [ ] P09-15 测试急停、末端断电、驱动节点退出和通信超时；
- [ ] P09-16 连续测试中记录温升、电流和错误分布；
- [ ] P09-17 MoveIt 已加入 `gripper_command_joint` 及 open/closed 状态；待实测指尖开度和可变碰撞几何后关闭本项。

### 验收门槛

- [ ] 100 次空载开合无失控、异常复位或持续过流；
- [ ] 夹爪动作完成判据不只依赖固定 sleep；
- [ ] 堵转/超时/断线不会继续推进抓取状态机；
- [ ] ROS action 取消和重复调用行为明确；
- [ ] 失电后夹持是否保持已有明确风险结论。

### 计划产物

- `src/fr3c_gripper_driver/`
- `config/gripper.yaml`
- `docs/test/P09_gripper_installed.md`
- `evidence/P09/<timestamp>/`

## P10：TCP、负载与无视觉固定点抓放

### 阶段目标

不引入相机误差，先证明“机器人 + 夹爪 + TCP + 负载 + 抓放顺序”能稳定抓住固定示教位置的标准块。

### 阶段边界

目标和料框位置由示教/测量给出，不使用视觉坐标。主控制仍采用 P06 SDK service 路线，便于直接核对法奥 API；不与 MoveIt 同时发运动。

### 前置条件

- P06、P08、P09 通过；
- 标准工件、固定治具和宽口料框就绪；
- 末端版本和质量/质心已冻结。

### 法奥资源/API 复用

- 工具标定优先使用法奥 WebAPP；需要程序化时使用 `SetToolPoint/ComputeTool` 或 `SetTcp4RefPoint/ComputeTcp4`；
- `SetToolCoord`、`GetTCPOffset`；
- `SetLoadWeight`、`SetLoadCoord`、`GetTargetPayload`、`GetTargetPayloadCog`；
- `ComputePrePick`、`ComputePostPick` 用作预抓取/撤退点计算与 MTC 结果对照；
- 运动使用 `MoveJ/MoveL`，夹爪使用 P09 接口。

### 开发与现场步骤

- [x] P10-01 以四个柔性指最终共同闭合的最尖端定义 `grasp_frame`；
- [x] P10-02 已以 `[0,0,0.163] m` 建立第一版名义 TCP，并在固定尖点采集 12 个法兰姿态执行实测修正；
- [x] P10-03 已冻结实用 TCP v2：`flange_link -> grasp_frame = xyz[-0.000847397,0.003027928,0.178] m, rpy[0,0,0]`；X/Y 来自 12 姿态固定点求解，Z 来自法兰盘到尖端 178 mm 直接测量；厂商确认原 160 mm 尺寸有问题，全量求解的 Z=184.614 mm 未采用；机器可读结果为 `src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml`；
- [ ] P10-04 将 TCP 保存到专用 tool ID，不覆盖未知现用工具；
- [ ] P10-05 读回 `GetTCPOffset` 或状态字段，核对 tool ID 和数值；
- [ ] P10-06 写入实测负载重量和质心到专用 load ID；
- [ ] P10-07 读回重量/质心并与 `devices.yaml` 比较；
- [ ] P10-08 定义固定工件坐标、抓取位、预抓取位、提升位、放置位和撤退位；
- [ ] P10-09 用 `ComputePrePick/PostPick` 生成结果，与手工/MTC 坐标约定对比；
- [ ] P10-10 在夹爪张开状态空跑全路径，离工件至少 50 mm；
- [ ] P10-11 分段降低到抓取高度，每段要求操作者确认；
- [ ] P10-12 执行低力夹取、垂直提升 20～30 mm、暂停检查；
- [ ] P10-13 执行到料框上方、下降、释放、撤退；
- [ ] P10-14 失败时回安全位重新开始，不在原地盲夹第二次；
- [ ] P10-15 完成固定位置 20 次抓放；
- [ ] P10-16 分别记录未夹到、夹偏、滑落、放置失败和运动失败；
- [ ] P10-17 在携带/不携带工件两种状态检查负载和碰撞行为。

### 验收门槛

- [x] TCP 固定点标定的姿态覆盖、残差和留一法稳定性达到普通固定点抓取要求；
- [ ] 配置中的 tool/load ID 与机器人当前状态一致；
- [ ] 空跑路径无碰撞和近碰；
- [ ] 固定点 20 次抓取成功至少 19 次；
- [ ] 失败不会继续带着错误的 attach/夹持状态运动。

### 计划产物

- `calibration/tcp_payload/<end_effector_version>.yaml`
- `config/poses/fixed_pick_place.yaml`
- `docs/test/P10_fixed_pick_place.md`
- `evidence/P10/<timestamp>/`

## P11：ros2_control 生命周期修正与 MoveIt 真机空载

### 阶段目标

在 P06 已证明 SDK 运动正常后，修正并验收官方 `fairino_hardware` 的真机伺服生命周期，使 MoveIt 轨迹能安全送入 FR3-C。

### 阶段边界

`control_mode:=moveit` 时只能有一个运动源；命令服务器若启动，只允许只读接口。先空载/无抓取轨迹，后续才接任务层。

2026-09-10 现场进度：操作者已在有人监护、急停可用和低速条件下，使用 RViz MotionPlanning 的末端交互球改变笛卡尔目标并执行 Plan/Execute，也使用单关节目标多次规划执行；实体 FR3-C 均按规划运动，未观察到明显跳变、规划后不执行或控制器阻塞。这证明 MoveIt → JointTrajectoryController → `fairino_hardware` → 实体机械臂的基础路径可用，也为手眼采样提供了多姿态运动能力。

上述操作没有形成逐条轨迹、返回码、周期/跟踪误差和故障注入归档，不能替代本阶段的 20 条审查轨迹、取消/断线、deactivate/error/shutdown、ServoJ 周期及第 6 轴 NaN/Inf 门禁。尤其当前源码仍可见 `std::any_of(...[5])` 未覆盖第 6 轴、100 Hz 配置与 `ServoJ(..., 0.008, ...)` 不一致、`unique_ptr.release()` 以及未在硬件生命周期中明确调用 `ServoMoveStart/End`，这些项继续保持未完成。

### 前置条件

- P05、P06、P07 通过；
- 若末端已安装，P08/P10 负载和碰撞体必须有效；
- 法奥对 `ServoJ` 周期/进入退出序列的答复已记录，或完成保守实验验证。

### 法奥资源/API 复用

- 官方 `FairinoHardwareInterface`、`JointTrajectoryController`；
- `ServoMoveStart`、`ServoJ`、`ServoMoveEnd`；
- `GetActualJointPosDegree`、`StopMotion`、`CloseRPC`；
- MoveIt controller 名称沿用官方 `fairino3_controller` 后再在派生包中规范。

### 开发与测试步骤

- [ ] P11-01 对当前 hardware interface 做逐行生命周期审计；
- [ ] P11-02 修正有限值检查范围，确保包含第 6 轴；
- [ ] P11-03 激活时连接 RPC 后先读取当前六轴并同时同步 state/command；
- [ ] P11-04 激活前检查错误、急停、安全停止、使能和拖动示教状态；
- [ ] P11-05 明确执行旧运动停止/旧伺服结束的顺序；
- [ ] P11-06 进入 `ServoMoveStart`，非 0 返回码使 lifecycle 激活失败；
- [ ] P11-07 将 `ServoJ cmdT` 与 controller_manager 实际 update period 对齐；
- [ ] P11-08 解决官方配置 100 Hz 与当前硬编码 0.008 s 的不一致，禁止保留“配置 10 ms、下发声称 8 ms”的状态；
- [ ] P11-09 `write()` 中任一 `ServoJ` 非 0 立即返回 ERROR 并停止控制器；
- [ ] P11-10 增加连续超时、状态陈旧和跟踪误差保护；
- [ ] P11-11 deactivate/error/shutdown 均执行 `ServoMoveEnd → StopMotion → CloseRPC`；
- [ ] P11-12 使用 `unique_ptr.reset()` 正常释放，不用 `release()` 泄漏实例；
- [ ] P11-13 建立 mock FRRobot 或接口封装，测试所有失败返回路径；
- [ ] P11-14 在无机器人环境测试 activate/deactivate 失败不会崩溃；
- [x] P11-15 组合入口已多次激活真实控制器并长时间保持/采样，操作者未观察到激活首帧位置跳变；
- [x] P11-16 已通过 RViz 单关节目标和末端交互球分别执行多组单关节/多关节低速短轨迹；
- [ ] P11-17 已有多组 RViz Plan/Execute 成功的现场经验，但尚未按固定清单归档 20 条轨迹及每条结果；
- [ ] P11-18 中途取消轨迹，验证控制器/机器人状态一致；
- [ ] P11-19 重启、断线、SDK 返回错误后必须重新完整激活流程；
- [ ] P11-20 记录控制周期、抖动、ServoJ 返回码和跟踪误差。

### 验收门槛

- [x] 现场多次启动、保持和连续采样期间未观察到首帧跳变或静止漂移；
- [ ] 20 条轨迹全部完成，0 超限、0 碰撞、0 长周期阻塞；
- [ ] 第 6 轴 NaN/Inf 单元测试会被拒绝；
- [ ] `ServoJ` 周期与 ros2_control 更新周期有明确一致结论；
- [ ] 取消、deactivate、错误退出均能结束伺服并释放 RPC；
- [ ] SDK service 与 MoveIt 不能同时下发运动。

### 计划产物

- `src/fairino_hardware` 最小生命周期补丁
- `tests/P11_hardware_interface/`
- `docs/test/P11_moveit_real_robot.md`
- `evidence/P11/<timestamp>/`

## P12：相机装机后的视场、近距离深度与支架验证

### 阶段目标

把 P03 台架相机参数带到真实 eye-in-hand 安装中，验证每个计划观察/抓取姿态下都能看清目标，并确认支架/线缆不会引入明显深度或位姿漂移。

### 阶段边界

先使用名义 CAD 变换，仅做视场和深度质量评估；不能把名义 `flange→camera` 当作正式手眼结果。

2026-09-10 进度：相机已固定在最终末端支架上，640×480、内参和 D2C 配置已冻结，并在人工低速监护下完成 40 个可见标定板的多平移/多旋转姿态采集。该事实支持进入 P13，但不替代本阶段尚未完成的全抓取区深度覆盖、夹爪开闭遮挡、10 分钟漂移、冷启动和支架受力复测。

### 前置条件

- P03、P08、P11 通过；
- 相机支架版本冻结；
- P07 中相机/线缆碰撞体已更新。

### 官方资源复用

- 相机厂商官方 launch、preset、D2C 和诊断；
- 法奥 `MoveJ`/MoveIt 只用于到达已审查观察姿态；
- `GetActualToolFlangePose`/状态法兰位姿用于记录采样姿态。

### 开发与测试步骤

- [ ] P12-01 固定相机 SN、USB 口/udev 名称和命名空间；
- [x] P12-02 已先使用外壳估位 preview TF；P13 通过后已由正式 `flange_link→camera_link` 标定 TF 取代；
- [ ] P12-03 在观察位检查整个工作区是否进入彩色/深度视场；
- [ ] P12-04 在预抓取、抓取上方、提升、料框上方检查夹爪遮挡；
- [ ] P12-05 测量每个姿态下目标到相机距离，与 P03 可用区比较；
- [ ] P12-06 若型号支持，比较 High Accuracy、High Density、近距离等官方 preset；
- [x] P12-07 已固定本次手眼标定使用的 640×480、`CameraInfo` 和硬件 D2C 配置；任何后续变更都必须重新验证；
- [ ] P12-08 在 9 宫格工作区和多个高度采集深度有效率；
- [ ] P12-09 开合夹爪，检查彩深画面是否受电气干扰、遮挡或振动；
- [ ] P12-10 低速转动 J4/J5/J6，观察 USB 带宽、接头和线缆张力；
- [ ] P12-11 保持机器人静止 10 分钟，分析目标/标定板深度漂移；
- [ ] P12-12 对支架轻微受力前后重复拍摄，评估刚度；
- [ ] P12-13 冷启动 10 次检查相机参数是否一致；
- [ ] P12-14 更新观察位；必要时调整支架后重新从 P08 验收。

### 验收门槛

- [ ] 所有计划抓取位置处于有效深度和清晰彩色视场；
- [ ] 夹爪在开/闭状态不遮住目标关键区域；
- [ ] 腕部计划范围内无掉线、拉线和持续 USB reset；
- [ ] 相机参数重启后不漂移；
- [ ] 支架变形/振动不占用手眼误差预算的大头。

### 计划产物

- `config/camera/eye_in_hand.yaml`
- `config/poses/observation_poses.yaml`
- `docs/test/P12_mounted_camera.md`
- `evidence/P12/<timestamp>/`

## P13：eye-in-hand 手眼标定与独立验证

### 阶段目标

得到可追溯的 `T_flange_camera`，并用未参加求解的姿态证明它能把相机目标稳定转换到机器人基坐标系。

### 阶段边界

求解数据与验证数据严格分开；“标定软件返回成功”不等于验收通过。更换相机、支架、分辨率、D2C 模式或工具机械结构后必须重新验证。

2026-09-10 当前结论：已在最终相机支架和冻结的 640×480 配置下采集 40 组 ChArUco 数据，前 25 组求解、后 15 组独立验证。原会话同步保存的 ROS `base→wrist3` 因旧 URDF 错误而被判定不可用；没有用前 25 组拟合新 FK，而是从历史姿态恢复关节角，再只读调用控制器 `GetForwardKin` 重建 `base→controller flange`。TSAI 最佳结果的验证平移误差中位数/P95/最大值为 2.274/6.327/6.609 mm，旋转中位数/P95 为 0.418°/0.825°。新 `flange_link` 经 3 个现场姿态交叉检查，固定关系离散最大 0.004513 mm / 0.000483°；系统重启后正式 YAML 与完整 `base_link→camera_link` TF 链均已验证。

这表示核心手眼数值门禁和 TF 接入已经通过，但 P13 尚未整体关闭：仍缺已知固定点/探针、真实抓取区 9 宫格与不同高度测量、相机支架拆装复验，以及 preset/生成时间/文件 SHA256 的最终元数据固化。

### 前置条件

- P10 TCP/负载、P11 真机、P12 装机相机通过；
- ChArUco/AprilTag 标定板尺寸经实物测量；
- 机器人和标定板固定可靠。

### 官方/标准资源复用

- MoveIt Calibration ROS 2 可用分支或 OpenCV `calibrateHandEye`；
- 法奥 `GetActualToolFlangePose` / `GetActualJointPosDegree`；
- 相机官方 `CameraInfo` 和 D2C；
- 不自研 AX=XB 求解器。

### 开发与测试步骤

- [x] P13-01 实测并固化 ChArUco：11×8 方格、checker 15 mm、marker 11 mm、DICT_4X4_50、legacy pattern；
- [x] P13-02 标定板已牢固固定到工作台，40 组采样期间保持不动；
- [x] P13-03 已确认 `CameraInfo` 对应当前 640×480 和硬件 D2C 配置；
- [x] P13-04 实际采集 40 个覆盖平移、距离及 roll/pitch/yaw 的手动停稳姿态；
- [x] P13-05 采集器执行角点数、重投影、图像/机器人短时稳定性门禁；40 组图像与质量字段完整；
- [x] P13-06 每个姿态由操作者确认完全停稳，并连续采集 12 个通过图像/板位姿/机器人位姿稳定性门禁的观测；
- [ ] P13-07 已同步记录 `camera→target`、时间戳和检测误差；原 `base→wrist3` 受旧 URDF 影响，本次已用可追溯的控制器 FK 重建补救，未来采集必须直接同步记录控制器 `base→flange`；
- [x] P13-08 已在采集器、求解器和报告中明确 m/mm、rad/deg 及 `camera_link`/光学帧转换；
- [x] P13-09 已比较 TSAI、PARK、HORAUD、ANDREFF、DANIILIDIS 五种 OpenCV 方法，按留出验证选择 TSAI；
- [x] P13-10 已确认并保存 `flange_link→camera_link`，未把逆矩阵作为发布 TF；
- [x] P13-11 未人工删除 40 组样本；历史第 38 姿态的 IK 多分支选择依据和替代分支残差已写入 provenance；
- [x] P13-12 已固化全部原始/角点叠加图、40 组 JSON、控制器 FK 重建、求解报告及正式 YAML；
- [x] P13-13 使用未参与求解的后 15 个姿态进行独立验证；
- [ ] P13-14 用已知固定点/探针验证 `camera→base` 转换；
- [ ] P13-15 在工作区 9 宫格和不同高度统计平移/姿态误差；
- [x] P13-16 已重启组合入口并加载正式 YAML；`tf2_echo flange_link camera_link` 与文件一致；
- [ ] P13-17 拆装相机支架一次后只做验证；超门槛则证明需要重标定；
- [ ] P13-18 给 YAML 写入相机 SN、支架版本、分辨率、preset、时间和 SHA256。

### 验收门槛

- [x] 独立验证平移误差中位数 2.274 mm、P95 6.327 mm，满足 ≤3 mm / ≤8 mm；
- [ ] 验证点覆盖真实抓取区，不只集中在图像中心；
- [x] 单位、坐标轴、矩阵方向和 TF 链已经离线残差、三姿态 FK 交叉检查及在线 `tf2_echo` 核对；
- [ ] YAML 与相机 SN、支架版本和相机配置严格绑定；
- [x] 重启加载后仅 `/dabai_hand_eye_tf` 发布 `flange_link→camera_link`，外壳 preview TF 未运行，无重复发布/方向取反。

### 计划产物

- `calibration/handeye/<camera_sn>_<mount_version>.yaml`
- `calibration/handeye/raw/<session>/`
- `docs/test/P13_handeye_validation.md`
- `evidence/P13/<timestamp>/`

本次实际产物集中在 `evidence/P13/manual_30pose_20260910/`：

- `flange_to_dabai_dcw2.yaml`：正式 `calibrated: true` 外参；
- `P13_handeye_analysis_controller_fk.md` / `solve_report_controller_fk.json`：25+15 求解与验证结果；
- `controller_fk_reconstruction.json`：历史控制器 FK 重建及 provenance；
- `robot_pose_crosscheck_corrected.json`：修正模型后三个现场姿态交叉检查；
- `robot_model_comparison.md`：候选 URDF、控制器 FK 和模型选择依据。

## P14：确定性 RGB-D 目标检测与 3D 位姿

### 阶段目标

用桌面平面、颜色/几何和深度获得稳定、带时间戳和置信度的标准积木 3D 位姿，不把神经网络误差混入第一版坐标链。

### 阶段边界

只支持已知、非透明、非强反光的少量标准物体；不做任意类别、6D 通用抓取或动态目标。可先离线 rosbag 开发，再在线无动作验收。

### 前置条件

- P03 有可回放数据；
- 在线 `base_link` 输出验收依赖 P13；
- 工作区 ROI、工件尺寸和颜色范围已定义。

### 官方/标准资源复用

- 相机官方彩色、对齐深度、`CameraInfo`；
- ROS 2 `image_transport`、`tf2`、`vision_msgs/Detection3DArray`；
- OpenCV/RANSAC/点云标准能力；
- 不重写相机投影模型和 TF 系统。

### 开发步骤

- [ ] P14-01 定义检测输入同步策略和最大时间差；
- [ ] P14-02 建立感知 ROI 和桌面高度/平面先验；ROI 只控制检测范围，不改机器人运动限制；
- [ ] P14-03 对齐彩深并过滤 0、NaN、范围外值和边缘飞点；
- [ ] P14-04 用 RANSAC/高度带移除桌面平面；
- [ ] P14-05 用 HSV 颜色和已知尺寸/轮廓分割第一类积木；
- [ ] P14-06 对候选做面积、长宽比、高度和抓取区评分；靠近常用区边缘先告警；
- [ ] P14-07 在目标内部腐蚀 ROI 取有效深度中位数，而非单像素；
- [ ] P14-08 用当前 `CameraInfo` 反投影到光学坐标；
- [ ] P14-09 使用与图像时间一致的 TF 转换到 `base_link`；
- [ ] P14-10 发布中心、尺寸、yaw、置信度、时间戳、frame_id 和拒绝原因；
- [ ] P14-11 连续至少 5 帧稳定后才标记可抓；
- [ ] P14-12 目标过期、深度不足、TF 超时、明显不可达、进入真实障碍物或多目标歧义时拒绝；
- [ ] P14-13 先固定 yaw=0/90°，再评估是否需要轮廓主轴角；
- [ ] P14-14 建立不少于 200 帧的标注/测量数据集；
- [ ] P14-15 覆盖 9 宫格位置、至少 3 个高度、3 种照度和 2 种背景；
- [ ] P14-16 对静止目标计算 3D 抖动、系统偏差、漏检和误检；
- [ ] P14-17 与尺量/机器人已知点比对，而不是只看 RViz；
- [ ] P14-18 提供 debug image、点云、平面和候选可视化；
- [ ] P14-19 所有在线测试录包，可离线复现同一输出。

### 验收门槛

- [ ] 静止目标 `base_link` 中心 P95 抖动 ≤5 mm；
- [ ] 9 宫格测量误差满足 P13/P16 总误差预算；
- [ ] 目标移除后在 freshness 超时内失效；
- [ ] 深度为 0、TF 缺失、明显不可达和真实障碍物内部目标 100% 被拒绝；普通任务区边缘不会无故硬拒绝；
- [ ] 相同 rosbag 重放产生可解释、基本一致的检测结果。

### 计划产物

- `src/fr3c_perception/`
- `config/perception/*.yaml`
- `datasets/P14/README.md`
- `docs/test/P14_rgbd_pose.md`
- `evidence/P14/<timestamp>/`

## P15：视觉坐标链、规划预览与无接触悬停

### 阶段目标

在夹爪接触工件前，用“检测目标上方悬停”把视觉、TF、碰撞场景和 MoveIt 规划串起来，捕获轴反、单位错、手眼方向错等高风险问题。

### 阶段边界

夹爪保持张开，TCP 不低于目标上方保守高度，禁止闭合和接触。默认“只规划”，人工确认后才执行到悬停位。

### 前置条件

- P07、P11、P13、P14 通过；
- P08 末端碰撞模型有效；
- P05 preflight 集成到启动流程。

### 法奥资源/API 复用

- MoveIt 规划/碰撞和法奥 `fairino_hardware`；
- SDK `GetForwardKin/GetActualTCPPose` 用于执行后独立对照；
- `ComputePrePick` 可对照预抓取方向，但不作为第二运动源。

### 开发与现场步骤

- [ ] P15-01 只发布目标碰撞体，不发送运动，检查 RViz 位置；
- [ ] P15-02 将目标、桌面、料框、箱壁和末端同步到 PlanningScene；
- [ ] P15-03 对 `camera→base` 结果检查原厂关节可达性、真实障碍物、桌面高度和姿态；普通任务区只设告警带；
- [ ] P15-04 生成目标上方 100 mm 预抓取位，检查抓取轴方向；
- [ ] P15-05 先只规划并显示完整轨迹；
- [ ] P15-06 规划结果经过关节限位、碰撞、路径长度和最低 Z 检查；
- [ ] P15-07 人工确认后低速执行到目标上方 100 mm；
- [ ] P15-08 用直尺/俯视图检查 TCP XY 是否对准目标；
- [ ] P15-09 分级降低到 70 mm、50 mm，仍不接触；
- [ ] P15-10 执行后读取真实 TCP，与规划终点比较；
- [ ] P15-11 目标被移动时，旧检测必须过期且禁止执行；
- [ ] P15-12 在 9 宫格完成至少 30 次悬停；
- [ ] P15-13 注入错误手眼版本/SN，系统必须拒绝；
- [ ] P15-14 规划失败只回观察位重新感知，不原地盲重试；
- [ ] P15-15 保存每次检测位姿、规划解、执行终点和人工测量误差。

### 验收门槛

- [ ] 30 次均无闭爪、接触、碰撞和越界；
- [ ] 目标上方 XY 偏差满足进入抓取的误差预算；
- [ ] 陈旧目标和错误标定文件 100% 被拒绝；
- [ ] 规划和执行的终点差异可解释；
- [ ] 任一坐标方向/单位错误会在接触前暴露。

### 计划产物

- `src/fr3c_manipulation` 的 dry-run 模式
- `config/manipulation/pregrasp.yaml`
- `docs/test/P15_visual_hover.md`
- `evidence/P15/<timestamp>/`

## P16：法奥 MTC/MoveIt 视觉抓放闭环

### 阶段目标

在所有单链路均已通过后，完成“观察—识别—规划—预抓取—接近—夹取—提升—放置—撤退—返回”的第一版闭环。

### 阶段边界

继续修改已迁入并完成 FR3MT 编译适配的法奥 `fairino_mtc_demo`。自研只保留机器人/夹爪命名、目标参数、场景、安全门禁和失败恢复。

### 前置条件

- P09、P10、P11、P13、P14、P15 全部实物通过；
- 固定点抓取和视觉悬停均已稳定；
- 现场仍使用低速和人工执行确认。

### 官方资源/API 复用

- 法奥 `fairino_mtc_demo` 或 MoveIt Task Constructor；
- 法奥 description/MoveIt controller/hardware interface；
- MTC `CurrentState`、`Connect`、`MoveRelative`、场景 attach/detach；
- 夹爪调用 P09 标准 action，对下使用 YDI-J22 硬件 PWM 控制器；夹持确认不以发送完成代替；
- `ComputePrePick/PostPick` 只做离线/日志对照。

### 开发步骤

- [ ] P16-01 冻结 `PickPlace.action` 输入、结果、反馈和错误码；
- [ ] P16-02 建立全局单任务互斥和唯一 task_id；
- [ ] P16-03 Preflight 检查控制模式、机器人状态、相机新鲜度、标定版本、TCP/负载和夹爪；
- [ ] P16-04 获取新的稳定目标，锁定本轮 pose 和时间戳；
- [ ] P16-05 更新桌面、箱壁、料框、目标和末端 PlanningScene；
- [ ] P16-06 打开夹爪并确认完成；
- [ ] P16-07 `CurrentState` 获取真实关节状态；
- [ ] P16-08 `Connect` 规划到预抓取姿态；
- [ ] P16-09 `MoveRelative`/Pilz LIN 沿抓取轴直线接近；
- [ ] P16-10 关闭夹爪，等待完成/超时/故障；
- [ ] P16-11 只有夹持判定成立后才 attach 目标；
- [ ] P16-12 直线提升并检查最低安全高度；
- [ ] P16-13 连接到料框上方；
- [ ] P16-14 直线下降到释放高度；
- [ ] P16-15 打开夹爪，确认释放；
- [ ] P16-16 detach 目标并更新目标/料框场景；
- [ ] P16-17 直线撤退并返回观察位；
- [ ] P16-18 规划和执行分开，完整解可视化后人工确认；
- [ ] P16-19 取消在每个阶段都可达安全终态；
- [ ] P16-20 相机/目标/规划/夹爪失败时不推进下一阶段；
- [ ] P16-21 只允许一次“回观察位—重新感知”恢复，不原地盲重试；
- [ ] P16-22 先做 5 次固定视觉目标，再做 20 次单区域随机点；
- [ ] P16-23 扩展到 9 宫格 50 次随机位置；
- [ ] P16-24 分别统计检测、规划、抓取、运输、放置失败原因；
- [ ] P16-25 保存 task JSONL、rosbag、MoveIt trajectory 和状态事件。

### 验收门槛

- [ ] 固定视觉位置连续 20 次至少 19 次成功；
- [ ] 随机平面位置 50 次至少 45 次成功；
- [ ] 放入固定料框 50 次至少 48 次；
- [ ] 0 碰撞、0 越界、0 未确认动作、0 双运动源；
- [ ] 所有失败都有明确阶段/错误码且不会盲动；
- [ ] 用户无需改源码即可改颜色、ROI、工作区、料框和高度参数。

### 计划产物

- `src/fr3c_manipulation/`
- `src/fr3c_interfaces/action/PickPlace.action`
- `config/manipulation/*.yaml`
- `docs/test/P16_visual_pick_place.md`
- `evidence/P16/<timestamp>/`

## P17：故障注入、连续运行、发布与交接

### 阶段目标

证明系统不仅“能演示一次”，而且能在常见故障下安全停止、恢复可控，并由其他人按文档复现。

### 阶段边界

不在本阶段新增识别模型、VLM/LLM、语音或新硬件；只关闭第一版 RGB-D 抓取的可靠性和交付问题。

### 前置条件

- P16 实物通过；
- 所有配置、固件、支架和末端版本冻结；
- 故障注入逐项完成风险评审。

### 开发与测试步骤

- [ ] P17-01 建立一键 `mock/sdk_service/moveit` 互斥启动；
- [ ] P17-02 启动前自动运行 P05 preflight 和版本/SN/配置校验；
- [ ] P17-03 注入相机节点退出/USB 断开；
- [ ] P17-04 注入深度全 0、低有效率和时间戳陈旧；
- [ ] P17-05 注入 TF 缺失、手眼 SN/支架版本不匹配；
- [ ] P17-06 分别注入“仅超常用任务区”“明显不可达”“桌面/箱壁内部”和多目标歧义，核对软提示与硬拒绝没有混淆；
- [ ] P17-07 注入规划无解、碰撞路径和轨迹取消；
- [ ] P17-08 注入夹爪未激活、超时、堵转和反馈丢失；
- [ ] P17-09 注入机器人未使能、主/子错误码、急停和安全停止；
- [ ] P17-10 注入任务中目标被移走；
- [ ] P17-11 注入客户端重复 goal、取消和节点重启；
- [ ] P17-12 检查每类故障都停止推进并输出唯一错误码；
- [ ] P17-13 故障恢复必须重新 preflight/感知/规划，禁止从中间盲续；
- [ ] P17-14 完成 10 次冷启动和 10 次正常关机；
- [ ] P17-15 完成 60 分钟连续运行，记录资源、温度、USB 和控制周期；
- [ ] P17-16 完成 100 个抓放循环；
- [ ] P17-17 统计成功率、P50/P95 周期、各类失败和人工干预次数；
- [ ] P17-18 固化依赖、upstream commit、本地补丁和所有配置 SHA256；
- [ ] P17-19 编写安装、启动、正常操作、维护、标定、故障和回退手册；
- [ ] P17-20 由未参与开发者按文档完成一次冷启动和 5 次抓放；
- [ ] P17-21 生成最终验收报告，未达标项不得写成“通过”。

### 验收门槛

- [ ] 100 循环成功率至少 95%，无进程崩溃；
- [ ] 所有故障注入均不继续盲动；
- [ ] 10 次冷启动配置、设备绑定和 TF 一致；
- [ ] 普通使用者可在不改源码情况下启动、抓放、停止和读取错误；
- [ ] 版本清单、接线图、标定、日志、rosbag 和回退方法完整。

### 计划产物

- `src/fr3c_bringup/`
- `scripts/diagnostics/`、`scripts/acceptance/`
- `docs/user/FR3C操作手册.md`
- `docs/test/P17_final_acceptance.md`
- `release/manifest.yaml`
- `evidence/P17/<timestamp>/`

## 9. 前 15 个工作日的建议执行顺序

> 这是启动排期，不代替 P00～P17 门禁。相机、夹爪和机器人三条链路要并行推进、分别验收；同一天可以采集多条链路的数据，但不能把多个未知问题混在一次真机动作中。

### 第 1 天：P00 基线封存

- 复现当前两包干净构建、动态链接、网口和只读通信；
- 保存 Git/SDK/ROS/系统版本与 SHA256；
- 建立证据目录、设备清单和问题登记表；
- 明确当前 `command_server.cpp` 的用户修改，不覆盖、不格式化。

### 第 2～3 天：P01/P02 安全和法奥资源

- 在现场完成急停、安全停止、低速和底座检查，并盘点/清理重复过紧的自定义限制；
- 固定控制器 V3.9.2 与两个官方 tag/commit；
- 补入法奥官方 `fairino_description` 和 FR3-C 对应的 `fairino3mt_v6_moveit2_config`；
- 已取得并移植 `fairino_mtc_demo`；继续向法奥索取 FR3-C/FR3MT STEP、V3.9.2 版本资料和 ServoJ 周期建议；
- 在线新版 FR3-C 仿真包只登记为对照来源，不进入 V3.9.2 真机链。

### 第 2～5 天并行线 A：P03 相机无运动台架

- 识别准确型号、SN、固件、VID/PID 和官方 wrapper 分支；
- 先做彩色/深度/D2C/CameraInfo/TF，再测工作距离；
- 在候选抓取距离做平面 300 帧噪声和有效率统计；
- 完成 60 分钟稳定运行、10 次重启和一组可回放 rosbag；
- 第 5 天形成“可用工作距离 + preset + 固定启动 YAML”结论。

### 第 2～6 天并行线 B：P04 夹爪无机器人台架

- 继承 YDI-J22 型号、线序、5 V 供电和 FT232H AD7 PWM 动作记录；
- 补齐书面规格与电流/波形测量，完成机械安全行程标定；
- 选定持续硬件 PWM 控制器/MCU，冻结接线图、保护电路、BOM 和固件；
- 完成 100 次台架开合及受控故障测试；当前仍为开发中，不因点动成功跳过验收。

### 第 4～5 天并行线 C：P05 只读健康门禁

- 把 `/nonrt_state_data` 中错误、急停、安全停止、使能、motion_done、tool/load 等字段做成 preflight；
- 连续执行 100 次并测试节点退出、断网和状态陈旧；
- 所有异常只提示/阻断，不自动 `ResetAllError` 或自动使能。

### 第 6～7 天：P06 SDK 低速运动

- 现场确认后仅启动 `sdk_service` 运动源；
- 六轴分别做保守小角度 `MoveJ`；
- 确认坐标方向后做 5 mm X/Y/Z `MoveL`；
- 验证 `StopMotion`、motion_done、错误码和恢复流程；
- 完成 50 次观察位—测试位往返。

### 第 6～10 天并行：P07 模型与 MoveIt mock

- 原样启动官方 FR3 MoveIt 配置；
- 建立 FR3-C 派生 description/config；
- 加相机、夹爪、支架、桌面、实验箱和料框；
- 完成至少 20 组 SDK FK 对比和 100 组规划；
- 禁止在 FK/碰撞门禁通过前连接 MoveIt 真机。

### 第 8～11 天：P08 末端机械/线缆

- 汇总相机/夹爪台架结果后再定支架和电气转接；
- 称重、测质心、检查负载余量和相机视场；
- 完成 USB/夹爪线缆应力释放和腕部全计划范围检查；
- 更新 URDF collision/inertial，拆装重复性不达标则先改支架。

### 第 10～12 天：P09 夹爪装机

- 按评审接线首次通电；
- 验证 YDI-J22 硬件 PWM 控制器初始化、持续输出、安全开度映射和看门狗；
- 完成 GripperCommand、反馈/超时/取消和诊断；
- 先 20 次冒烟，再 100 次装机开合及受控堵转。

### 第 11～13 天：P10 TCP/负载和固定点抓放

- 已冻结实用 TCP v2 `[-0.847397,3.027928,178.0] mm`：固定点法提供 X/Y，现场直测提供 Z；后续以该值执行控制器写入和抓放微调；
- `SetToolCoord`、`SetLoadWeight/SetLoadCoord` 后读回核对；
- 先空跑，再逐级下降、夹取、提升和放置；
- 固定点完成 20 次，至少 19 次成功后才引入视觉。

### 第 12～15 天：P11 MoveIt 真机最小验收

- 修正 `ServoMoveStart/ServoJ/ServoMoveEnd` 生命周期、错误传播和第 6 轴检查；
- 先解决 100 Hz 配置与 0.008 s 硬编码的周期不一致；
- 保持当前位置 30 秒无跳变后，再执行 20 条已审查短轨迹；
- 第 15 天只要求关闭 MoveIt 真机底座；P12～P17 按阶段继续，不压缩手眼和视觉验收。

## 10. 累计验收台账

后续每关闭一个阶段，更新下表并附证据路径。状态只使用“未开始、开发中、软件通过、实物通过、阻塞”；没有证据路径不得标为通过。

| 阶段 | 当前状态 | 验收日期 | 结论摘要 | 主要证据 | 未关闭问题 |
| --- | --- | --- | --- | --- | --- |
| P00 基线冻结 | 开发中 | — | 环境、ARM64 SDK、构建、网络和只读通信已有基础，待形成统一快照 | `docs/env-deal/FAIRINO_ROS2_环境处理与技术交接.md` | P00 证据目录/manifest 未建立 |
| P01 安全与限制基线 | 未开始 | — | — | — | 急停、现场监护及现有自定义限制待盘点 |
| P02 法奥资源冻结 | 开发中 | 2026-09-10（运动学型号纠偏） | 已确认 `fairino3mt` 是历史命名，但原 `fairino3_mt_v6` 运动学不匹配当前控制器；MTC 源码已迁入并完成 ARM64 编译 | 本文第 2.2、4、6.1 节，`evidence/P13/manual_30pose_20260910/robot_model_comparison.md` | FR3-C/FR3MT STEP、惯量/碰撞外形/关节限制的 V3.9.2 正式资料、ServoJ 周期答复 |
| P03 相机台架 | 开发中 | 2026-09-09（快速门禁与 RViz 实机验证） | Astra Stereo S U3/Gemini Pro 已舍弃；DaBai DCW2 `CH82C5201C9` 已完成 USB 预检、10 FPS RGB+硬件 D2C 接入和 20 秒快速门禁；相机独立 RViz 已实机显示 RGB、深度和彩色点云，点云 frame/相机内部 TF/QoS 均验证 | [DaBai 接入说明](../hardware/DaBai_DCW2_接入说明.md)、[RViz/MoveIt 集成交接](../DaBai_DCW2_RViz与MoveIt集成技术交接.md)、`evidence/camera/20260909_dabai_migration/quick_validation_strict/` | 铭牌照片/官方 commit、DaBai stream profile 归档、尺量距离精度、彩深边缘、DaBai rosbag、60 分钟和 10 次重启；彩色点云更新率优化非 P03 硬门禁 |
| P04 夹爪台架 | 开发中 | 2026-09-10（软件链） | YDI-J22 独立 5 V、FT232H AD7、50 Hz PWM 已点动验证；连续 PWM backend 与软件门禁已实现 | [夹爪技术交接](../../test/jiazhua/YDI-J22四指夹爪测试与技术交接.md)、[`fr3c_gripper_driver`](../../src/fr3c_gripper_driver/README.md) | 安全机械行程、书面规格、电流/温升、持续输出实测、100 次循环及证据归档 |
| P05 只读门禁 | 开发中 | — | 只读状态已验证，自动 preflight 未实现 | 环境交接文档第 7、10 节 | 连续采样、异常注入、机器可读结果 |
| P06 SDK 低速运动 | 未开始 | — | 非法参数拒绝已验证，有效运动未现场验收 | 环境交接文档第 10.1 节 | 现场净空、急停、低速轨迹 |
| P07 模型/MoveIt mock | 开发中 | 2026-09-10（FK 纠正、碰撞回归与真机组合显示） | 保留现有 MoveIt 包/六关节接口，运动学切换为 `fairino3c_v6` 并定义 `flange_link`；3 个现场姿态与 40 组重建姿态 FK 均远低于 1 mm/0.2° 门槛；URDF/SRDF、命名位、自碰撞及障碍碰撞回归通过；正式手眼 TF 已接入 | 本文第 6.1、P07 节，[模型核对报告](../../evidence/P13/manual_30pose_20260910/robot_model_comparison.md)、[RViz/MoveIt 集成交接](../DaBai_DCW2_RViz与MoveIt集成技术交接.md) | 全场景/精细末端碰撞体、mock 与 100 组规划；ServoJ 生命周期/周期和真实轨迹执行仍未验收 |
| P08 末端机械/布线 | 未开始 | — | — | — | 质量、质心、支架、线缆 |
| P09 夹爪装机 | 开发中 | 2026-09-10（真实 action 与开度标定） | 自有薄驱动已完成 `GripperCommand`、mock/FT232H backend、诊断、MoveIt controller；真实 action 已驱动夹爪，550～1650 µs对应0～130 mm六点开度映射已写入 | 本文第 6.6 节、P09，[驱动 README](../../src/fr3c_gripper_driver/README.md) | 持续保持时长、cancel/断连、100次循环、夹持确认、动态碰撞几何及带载联动待验收 |
| P10 固定点抓放 | 开发中 | 2026-09-10（实用 TCP v2 冻结） | `grasp_frame` 已接入 URDF/MTC；最终采用 `[-0.847397,3.027928,178.0] mm`，X/Y 来自固定点求解、Z 来自法兰盘到尖端直测 | `src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml`、`docs/calibration/FR3C夹爪TCP固定点标定采集说明.md`、本文 P10 | 控制器专用 tool ID、TCP 读回、负载/质心、固定点 20 次抓放及必要微调 |
| P11 MoveIt 真机 | 开发中 | 2026-09-10（RViz 多组真机 Plan/Execute） | 操作者已用 RViz 末端交互球和单关节目标完成多组低速 Plan/Execute，实体机械臂运动正常，基础 MoveIt→ros2_control→FR3-C 链路通过 | 本文 P11；现场操作记录（尚未形成独立 P11 证据包） | 归档 20 条审查轨迹；修复/验证 ServoMove 生命周期、第 6 轴有限值、100 Hz/0.008 s 周期、错误返回、取消/断线/deactivate 及跟踪误差 |
| P12 装机相机 | 开发中 | 2026-09-10（最终支架下完成手眼采集） | DaBai 已固定在最终末端支架，640×480/内参/D2C 固定；40 个多平移/多旋转姿态均能检测标定板并通过单姿态稳定性门禁 | [手眼采集说明](../calibration/FR3C手眼标定数据采集说明.md)、`evidence/P13/manual_30pose_20260910/` | 全抓取区有效深度、夹爪开闭遮挡、10 分钟漂移、支架受力/拆装、10 次冷启动和线缆全范围复测 |
| P13 手眼标定 | 开发中 | 2026-09-10（核心数值门禁和正式 TF 通过） | 40 组数据按 25 求解+15 独立验证；TSAI 平移中位数/P95 2.274/6.327 mm，旋转中位数/P95 0.418°/0.825°；三姿态确认 `flange_link` 与控制器法兰一致；正式 YAML 已重启加载且 TF 唯一 | [手眼分析报告](../../evidence/P13/manual_30pose_20260910/P13_handeye_analysis_controller_fk.md)、[正式 YAML](../../evidence/P13/manual_30pose_20260910/flange_to_dabai_dcw2.yaml)、[模型核对报告](../../evidence/P13/manual_30pose_20260910/robot_model_comparison.md) | 已知固定点/探针验证、真实抓取区 9 宫格与不同高度误差、支架拆装复验；YAML 仍需补 preset、生成时间和 SHA256；未来采集须直接同步记录控制器 `base→flange` |
| P14 RGB-D 感知 | 未开始 | — | — | — | 可用 DaBai 快照先开发静态算法；需等待 P03-17 的 DaBai rosbag 才能做时序离线验证 |
| P15 无接触悬停 | 未开始 | — | — | — | 依赖模型/MoveIt/手眼/感知 |
| P16 视觉抓放 | 未开始 | — | — | — | 依赖所有主链门禁 |
| P17 可靠性交付 | 未开始 | — | — | — | 依赖 P16 |

## 11. 测试矩阵

### 11.1 单模块

- 机器人：版本、连接、状态频率、返回码、使能/停止/急停、50 次低速往返；
- 模型：FK 对比、关节限位、自碰撞、相机/夹爪碰撞体；
- 夹爪：100 次开合、堵转、断电、超时、反复启动、线缆摆动；
- 相机：分辨率/FPS、D2C、深度尺度、有效像素、时间戳、60 分钟稳定性；
- 标定：训练姿态之外的点验证、不同距离/角度验证、拆装后重复性；
- 感知：不同位置、颜色、照度和背景，记录漏检、误检和 3D 抖动。

### 11.2 系统故障注入

- 拔掉相机或停止相机节点；
- 目标深度为 0、明显不可达或进入真实障碍物；
- 规划无解；
- 夹爪动作超时；
- 机器人未使能、有故障或急停；
- 运行时物体被移走；
- 标定文件 SN 与当前相机不匹配；
- TF 超时或图像时间戳陈旧；
- 任务进行中客户端取消。

每一种情况的合格行为都是：停止推进任务、保持或退回安全姿态、给出明确错误码和日志；不能继续执行下一阶段。

## 12. 主要风险与控制措施

| 风险 | 影响 | 控制措施 |
| --- | --- | --- |
| 把 FR3 URDF 直接当 FR3-C | 碰撞检测和 TCP 可能错误 | 官方 CAD + FK 对比 + 保守碰撞几何 |
| ServoJ 模式未初始化 | 控制循环阻塞、轨迹无法激活 | 真机前修正生命周期并做低速门禁 |
| ros2_control 100 Hz 与 ServoJ 0.008 s 不一致 | 插补周期、跟踪和阻塞行为不可预测 | 以法奥 V3.9.2 建议和实测确定唯一周期，配置与实际 period 一致 |
| SDK/控制器版本不匹配 | 数据结构或行为不可预测 | 固定 V3.9.2 标签，不追 `main` |
| 两个运动源同时工作 | 指令冲突 | mock/service/moveit 三模式互斥 |
| YDI-J22 供电或信号误接 | 烧毁夹爪、转换器或机器人 IO | 独立稳压 5 V、棕地红电黄信号、共地和保护；不接 24 V/RS232，不用 FT232H 小板供电 |
| 把电气上限当机械安全行程 | 连杆顶死、过流和温升 | 2500 µs 尚未实测，冻结带裕量的机械工作端点 |
| 点动结束或通信丢失导致夹持失效 | 工件掉落，任务误判成功 | 持续硬件 PWM、看门狗、夹持确认及故障策略验收；不把命令完成当抓取成功 |
| 相机安装不刚性 | 手眼结果漂移 | 法兰侧定位支架，拆装后验证 |
| 眼在手上线缆缠绕 | 拉脱 USB、限制 J6 | 应力释放、线缆环、把线缆纳入运动范围检查 |
| 末端负载/质心未设置 | 拖动与碰撞检测异常 | 实测质量/质心并应用官方负载参数 |
| RGB-D 边缘飞点 | 抓取点跳变 | ROI 中值、平面约束、稳定帧、置信门限 |
| 把外壳估位或全零 TF 当手眼结果 | 点云和抓取目标在 `base_link` 下系统性错位 | 相机独立 RViz 固定在光学坐标；组合入口默认不发布外参，只接受明确标为已标定且结构有效的 YAML |
| 只看规划成功就执行 | 模型误差造成真机危险 | 仿真审查、低速、分段门禁、现场确认 |
| 多层边界重复收紧 | 正常目标频繁报“限制移动”，难以定位拒绝源 | 原厂限位只读快照；不默认加硬工作区；任务区用软提示；每次只调一层 |

## 13. 项目完成定义（Definition of Done）

只有同时满足以下条件，才可把“第一版抓取”标为完成：

- 设备型号、版本、SN、接线、负载和 TCP 有文档；
- 官方代码来源、标签、commit 和本地补丁可追溯；
- mock、SDK service、MoveIt 真机三种模式一键启动且互斥；
- FR3-C 模型与 SDK FK 一致，末端设备在碰撞模型中；
- 夹爪有标准 ROS action、超时和安全失效行为；
- 相机稳定性和手眼标定通过独立验收；
- MTC 完成完整 Pick/Place，失败不盲动；
- 固定点、随机点和 100 循环指标达到目标；
- 有 rosbag、日志、验收报告、接线图和操作步骤；
- 普通使用者不需要编辑源码即可更改颜色、感知/抓取任务区、料框和抓取高度；这些任务参数不改写机器人原厂关节限位。

## 14. 开工前需要补齐的信息

1. FR3-C 铭牌照片，以及 WebAPP“关于”页的完整版本截图；
2. DaBai DCW2 正反面铭牌照片；SDK/`lsusb` 身份、SN、固件和 USB PID 已记录；
3. YDI-J22 已知型号、线序、5 V 与 PWM 方式；补品牌/购买来源、舵机型号、厂家书面规格和峰值/堵转电流实测；
4. 夹爪安全机械行程和生产脉宽、持续 PWM 控制器/固件、保护与接线图、真实夹持确认方案；现有台架工具无位置或夹持反馈；
5. 相机支架、夹爪、转接板和最大工件的质量；
6. 实验箱桌面尺寸、机械臂底座位置、料框位置和目标工件尺寸；
7. 是否已有实体急停、围挡/安全扫描或固定的低速教学区域。

夹爪身份和初步台架接线已确认；第 3、4 项的剩余验证须在 P04/P09 关闭后才能进入整机携物运动，不再将夹爪列为“型号/协议未知”的阻塞。DaBai 的软件身份已经确认，但第 2 项铭牌照片及第 5 项末端质量/几何仍须在真机 MoveIt 和标定前关闭；旧 Gemini 的外壳占位和任何标定结果不得沿用。

## 15. 官方参考资料

> 在线手册会继续更新。本文编写时在线页面已高于控制器 V3.9.2；涉及函数签名、枚举、数据结构和真机控制行为时，以本地 V3.9.2 `robot.h`、对应 tag 源码及法奥对该版本的书面答复为准。

- [法奥 FAIR-INNOVATION GitHub 官方组织及全部开源仓库](https://github.com/FAIR-INNOVATION?tab=repositories)
- [法奥协作机器人在线手册](https://manual.fairino.support/latest/index.html)
- [法奥 FR3-C 参数、DH 与坐标系](https://manual.fairino.support/latest/CobotsManual/robot_brief_introduction.html)
- [法奥安装、末端 M12 IO/485 与 FR3-C 负载说明](https://manual.fairino.support/latest/CobotsManual/installation.html)
- [法奥工具坐标和末端负载设置](https://manual.fairino.support/latest/CobotsManual/parameter_setting.html)
- [法奥 ROS 2 / MoveIt 2 / MTC 官方说明](https://manual.fairino.support/latest/ROSGuide/moveIt2.html)
- [法奥 V3.9.2 对应 ROS 2 标签](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/V3.0.0_RobotV3.9.2)
- [法奥 V3.9.2 对应 FR3 MoveIt 2 配置](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/V3.0.0_RobotV3.9.2/fairino3_v6_moveit2_config)
- [法奥 V3.9.2 对应机器人描述](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/V3.0.0_RobotV3.9.2/fairino_description)
- [法奥 V3.9.2 对应 C++ SDK](https://github.com/FAIR-INNOVATION/fairino-cpp-sdk/tree/v2.3.2-robot3.9.2)
- [Orbbec 官方 ROS 2 Wrapper](https://github.com/orbbec/OrbbecSDK_ROS2)
- [Orbbec 设备枚举、固件与 profile 查询工具](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/6_benchmark/device_query_tools.html)
- [Orbbec ROS 2 启动参数与 D2C 配置](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/4_application_guide/launch_parameters.html)
- [Orbbec 相机预设与抓取/近距离模式说明](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/predefined_presets.html)
- [MoveIt 官方 MTC Pick/Place 教程](https://github.com/moveit/moveit2_tutorials/blob/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.rst)
- [MoveIt Task Constructor 官方仓库](https://github.com/moveit/moveit_task_constructor)
- [MoveIt Calibration 官方仓库](https://github.com/moveit/moveit_calibration)
