# FR3-C 夹爪、相机模型与本体碰撞技术交接

> 日期：2026-09-08
>
> 工作区：`/home/industio/fairino_ws`
>
> 状态：模型接入及离线碰撞验证通过；安装位置为近似值，尚未完成真机对齐验收。
>
> 本文以当前源码为准，覆盖相机下移至底面贴合法兰外缘后的最终配置。

## 1. 交接结论

本次完成三项工作：

1. 为 FR3-C（项目使用官方 FR3MT 模型）增加固定四指夹爪占位外观、垫片及碰撞包络。
2. 为法兰侧 Gemini Pro 相机增加固定长方体外观及碰撞盒，并按用户确认修正底面相切位置。
3. 给原来只有 visual 的 7 个机械臂本体连杆补充 collision，配置必要的碰撞排除对，使 MoveIt 能检查本体自碰撞及末端附件与本体的碰撞。

**本次没有修改官方六轴关节角度上下限、六轴运动学、原有速度配置，也没有修改控制器软限位、碰撞保护或下发真机运动指令。**

加入 collision 后，某些以前能规划的姿态可能因几何碰撞被拒绝。这属于碰撞检查生效，不是关节角度限位收紧。

## 2. 文件入口

以下路径均以 `/home/industio/fairino_ws` 为根目录。

| 文件 | 用途 |
| --- | --- |
| `src/fairino_description/urdf/fairino3_mt_v6.urdf` | 官方 FR3MT 本体；本次为 7 个连杆补充 collision |
| `src/fairino_description/urdf/ydi_j22_placeholder.xacro` | 固定夹爪、垫片的外观和碰撞包络 |
| `src/fairino_description/urdf/gemini_pro_placeholder.xacro` | 相机外壳的长方体外观和带余量碰撞盒 |
| `src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.urdf.xacro` | 模型总入口；夹爪、相机安装和尺寸参数集中在这里 |
| `src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.srdf` | 六轴规划组、示例姿态和碰撞排除对 |
| `src/fairino3mt_v6_moveit2_config/config/joint_limits.yaml` | 原有 MoveIt 速度/加速度配置，本次未修改 |
| `src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.ros2_control.xacro` | 当前使用 `fairino_hardware/FairinoHardwareInterface` 真机插件（复核修正） |
| `test/model_collision/check_model.cpp` | 只加载模型的离线碰撞检查程序 |
| `test/model_collision/CMakeLists.txt` | 独立编译上述检查程序 |
| `test/model_collision/COLCON_IGNORE` | 避免测试目录被常规 colcon 构建自动发现 |

已有实现说明见 [夹爪占位模型与碰撞检测说明](夹爪占位模型与碰撞检测说明.md)。本文为集中交接入口。

## 3. 模型连接关系

```text
原机械臂六轴链 → wrist3_link
                  └─ gripper_mount_link  名义法兰安装坐标
                       ├─ gripper_spacer_link  历史垫片占位（实物是否存在待拆装确认）
                       ├─ gripper_link  夹爪外观及整体碰撞盒
                       │    └─ grasp_frame  柔性指共同闭合尖端 TCP
                       └─ camera_housing_link  相机外壳
```

新增关节均为 fixed，不增加可控自由度。当前完整模型有 11 个 link，其中 10 个有碰撞几何；`gripper_mount_link` 只提供安装坐标。

规划组仍为 `fairino3mt_v6_group`，可动关节仍只有 `joint1`～`joint6`；链终点现为固定连接的 `grasp_frame`，因此 RViz/MTC 的笛卡尔末端对应真实夹爪尖端。夹爪四根手指仍只是 `gripper_link` 下的多个 visual，不是四个可动关节。

组合模型现已单独定义经控制器 FK 交叉核对的 `flange_link`。2026-09-10 最终采用实用 TCP v2：`grasp_frame` 相对 `flange_link` 为 `xyz=[-0.000847397,0.003027928,0.178] m`、姿态与法兰一致。X/Y 保留 12 姿态固定点求解结果，Z 使用现场从法兰盘到夹爪尖端直接测得的 178 mm；厂商已说明原 160 mm 尺寸有问题，不再作为依据。固定点原始全量求解的 Z=184.614 mm 与实物测量不符，因此只采用其 X/Y。后续真实抓放若仍有稳定偏差，再做毫米级微调。控制器专用 tool ID 和负载参数尚未写入。`camera_housing_link` 只是外壳中心，不是相机光学坐标。

## 4. 夹爪模型

### 4.1 资料与依据

夹爪为 YDI-J22 四指柔性夹爪。用户确认底部方形板连接法兰，可能存在约 3 mm 垫片。

本地资料：

- `/home/industio/YDI-J22柔性机械夹爪-三维模型/YDI-J22四指柔性夹爪.STEP`
- `/home/industio/YDI-J22柔性机械夹爪-三维模型/舵机四指尺寸图.jpg`

尺寸图标注最大展开 166 mm、图示高度 160 mm、侧面总宽 80.59 mm、底板 56×56 mm。当前没有转换 STEP，而是用基本几何体制作固定占位模型。

### 4.2 当前参数

参数位于模型总入口 xacro；长度单位米，角度单位弧度。

| 参数 | 当前默认值 | 说明 |
| --- | --- | --- |
| `gripper_mount_xyz` | `0 0 0` | 与已核对的 `flange_link` 原点重合 |
| `gripper_mount_rpy` | `0 0 0` | 与 `flange_link` 同向 |
| `gripper_spacer` | `0.003` | 历史碰撞/外观占位；实物是否存在尚未确认，不参与标定 TCP 变换 |
| `gripper_tcp_from_body_xyz` | `-0.000847397 0.003027928 0.175` | 相对 `gripper_link`；加 3 mm 历史占位后得到法兰 TCP Z=178 mm |
| `gripper_tcp_rpy` | `0 0 0` | 机械同轴定义；固定点法不观测方向 |
| `gripper_envelope_x` | `0.180` | 整体包络 X 尺寸 |
| `gripper_envelope_y` | `0.100` | 整体包络 Y 尺寸 |
| `gripper_envelope_z` | `0.200` | 整体包络 Z 尺寸 |

`flange_link` 已按控制器 `GetActualToolFlangePose` 与 ROS FK 多姿态核对；夹爪安装坐标直接使用该法兰坐标。实际转接板和绕法兰安装角仍应在拆装时复核。

垫片仍按 56×56×3 mm 作为历史占位几何显示，现场目前认为可能没有该垫片；下次拆装时应确认后再删改模型。为保持既有碰撞净空，夹爪碰撞体仍位于该占位之后；`grasp_frame` 相对夹爪本体的 Z 已减去 3 mm，所以完整 `flange_link -> grasp_frame` 不会重复叠加垫片。夹爪外观为简化本体和四根张开的手指；碰撞体为 180×100×200 mm 实心盒，从夹爪根部向工具伸出方向覆盖。这是偏保守的占位尺寸，不是厂家认证的全行程包络。

### 4.3 使用边界

整体实心盒包含抓取开口，因此目标物进入开口也会被判碰撞。当前适合检查净空和本体碰撞，不适合直接用于接触感知的抓取任务。

正式抓取前应拆分夹爪本体与手指碰撞体，根据实际开合状态更新几何，并针对目标物与预期接触部位配置接触规则。不要通过全局关闭夹爪碰撞解决抓取受阻。

## 5. 相机模型

### 5.1 尺寸与朝向

依据用户提供的实物照片和测量：相机长 100 mm、高 35 mm、厚 20 mm。外观为黑色长方体；碰撞盒每侧增加 3 mm，得到 106×41×26 mm。

局部轴约定：X 沿相机长度，Y 沿高度并指向法兰径向外侧，Z 沿厚度及名义工具伸出方向。相机固定到 `gripper_mount_link`，不会跟随夹爪手指开合。

| 参数 | 当前默认值 | 说明 |
| --- | --- | --- |
| `camera_mount_xyz` | `0 0.0575 0.015` | 相对名义法兰安装坐标的外壳中心 |
| `camera_mount_rpy` | `0 0 0` | 相对安装坐标的旋转 |
| `camera_size_x` | `0.100` | 长度 |
| `camera_size_y` | `0.035` | 高度 |
| `camera_size_z` | `0.020` | 厚度 |
| `camera_collision_padding` | `0.003` | 每侧增加的碰撞余量 |

### 5.2 底面相切修正

用户观察 RViz 后确认，相机底面应贴合法兰外缘。当前按模型名义法兰半径 40 mm 计算：

```text
相机中心径向高度 = 40 mm + 35 mm / 2 = 57.5 mm
相机外观底面高度 = 57.5 mm - 17.5 mm = 40 mm
```

因此最终 Y 偏移为 57.5 mm；早期 75 mm 估位已废弃，相当于下移 17.5 mm。轴向 Z 偏移仍为照片估计的 15 mm。

这里的“相切”是外观底面与名义圆周外缘相切，不代表已精确重建真实转接盘。绕法兰的方位、轴向偏移及实际转接盘半径仍需实测核对。3 mm 碰撞余量不能覆盖所有安装估计误差。

### 5.3 未覆盖内容

当前只有相机外壳盒，没有相机驱动、图像接入、光学坐标、手眼标定，也没有支架、完整转接板和线缆几何。后续补支架时应按实际突出范围加入碰撞体。

## 6. 本体碰撞与关节限位

### 6.1 本体几何

此前本体 URDF 只有 visual。本次给以下 7 个 link 增加 collision：

`base_link`、`shoulder_link`、`upperarm_link`、`forearm_link`、`wrist1_link`、`wrist2_link`、`wrist3_link`。

各自复用对应官方 STL 和原有 visual 原点，没有改 STL 文件，没有另写运动学，也没有对本体 mesh 增加额外几何膨胀。后续若规划耗时偏高，可制作简化碰撞网格，但须核对几何覆盖范围。

### 6.2 角度限制与碰撞检查是独立机制

| 机制 | 当前状态 | 能解决什么 |
| --- | --- | --- |
| URDF 单关节 lower/upper | 保留原值 | 限制每个关节角度范围 |
| MoveIt 速度配置 | 保留原值 | 约束规划速度等动态参数 |
| 本体自碰撞 | 已补几何并离线验证 | 检查非排除连杆是否相交 |
| 附件对本体碰撞 | 已接入并离线验证 | 检查相机、夹爪与手臂的相交 |
| 环境碰撞 | 需要 PlanningScene 中存在实体障碍物 | 桌面、料框等不会由 RViz 网格自动生成 |
| 控制器软限位及碰撞保护 | 本次未配置或验证 | 属于真机控制器侧机制 |

当前源码的角度限位快照如下，单位 rad；此表不表示已与控制器逐项读回核对。

| 关节 | lower | upper |
| --- | --- | --- |
| joint1 | -3.054 | 3.054 |
| joint2 | -4.6251 | 1.4835 |
| joint3 | -2.618 | 2.618 |
| joint4 | -4.6251 | 1.4835 |
| joint5 | 0 | 6.1087 |
| joint6 | -3.054 | 3.054 |

关节角度全部合法并不意味着没有碰撞；离线采样已找到角度合法但相机与前臂相交的姿态。

### 6.3 SRDF 碰撞排除清单

当前共 9 对，均在 `fairino3mt_v6_robot.srdf` 中标记 `reason="Adjacent"`。该字符串只是原因标签，实际含义是对整个 link 对跳过碰撞检查，不是仅排除某个接触面。

| link1 | link2 | 原因 |
| --- | --- | --- |
| base_link | shoulder_link | 相邻关节连接处 |
| shoulder_link | upperarm_link | 相邻关节连接处 |
| upperarm_link | forearm_link | 相邻关节连接处 |
| forearm_link | wrist1_link | 相邻关节连接处 |
| wrist1_link | wrist2_link | 相邻关节连接处 |
| wrist2_link | wrist3_link | 相邻关节连接处 |
| wrist3_link | gripper_spacer_link | 腕部与历史垫片占位接触 |
| gripper_spacer_link | gripper_link | 历史垫片占位与夹爪根部重叠 |
| camera_housing_link | gripper_link | 固定相机盒与夹爪大包络根部存在占位几何重叠 |

最后一对是在相机下移后加入的。它不关闭相机与前臂、腕部、其他本体连杆或环境的检测，但相机与 `gripper_link` 整体不会互检。当前两者固定且夹爪为单个实心包络；改成可动夹爪后必须重新审查此排除，不能直接沿用到手指。

其余具有碰撞几何的组合继续检查，运行时 PlanningScene 若另改 Allowed Collision Matrix，则还需检查运行时配置。

## 7. 构建与启动

修改参数后，在原工作区构建并重新启动原有环境；不要在已有控制栈之上叠加第二套。

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fairino_description fairino3mt_v6_moveit2_config --symlink-install
source /home/industio/fairino_ws/install/setup.bash
```

用户此前采用的 RViz 启动方式：

```bash
env -u LD_LIBRARY_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fairino3mt_v6_moveit2_config demo.launch.py
'
```

已运行节点不会自动重载修改后的 URDF/SRDF。相机下移同时涉及模型和 SRDF，必须让 MoveIt 与 RViz 一起加载新配置。

RViz 可启用碰撞几何显示，核对相机位置、夹爪包络及本体几何。碰撞盒与外观尺寸不同是预期现象。

**2026-09-08 复核修正：当前插件已经是 `fairino_hardware/FairinoHardwareInterface`，能够读取真机关节角并通过 ServoJ 下发指令。用户确认已通过 RViz 操作带动真机。** `FakeSystem` 仍作为 ros2_control 实例名保留，但名称不能证明使用模拟硬件；应检查实际 plugin 和运行日志。早前 14:40 的日志对应模拟配置，不能用于断言后续运行状态。

最新检查的 `2026-09-08-15-36-13-142039-ido-369863/launch.log` 中存在真实硬件接口日志及多次 `ServoJ` 返回码 14；因此“真机链路已接入”不等于本次所有执行均成功或整体验收完成。具体错误触发原因需另行核对。当前启动命令应按可能控制真机对待。

## 8. 已完成验证与复现

两个模型相关包构建通过；已安装的 MoveItConfigsBuilder 配置可加载末端模型。相机下移并加入最终排除规则后，已有离线检查通过：

- 默认姿态、pos1、pos2 无自碰撞。
- 确定性随机采样找到角度限位内的 `camera_housing_link/forearm_link` 自碰撞。
- 7 个本体连杆、夹爪、垫片和相机均能检测到放在对应位置的测试障碍物。
- 远处测试障碍物无接触。

复现命令：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
cmake -S test/model_collision -B /tmp/fr3c_collision_build
cmake --build /tmp/fr3c_collision_build -j2
xacro src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.urdf.xacro initial_positions_file:=/home/industio/fairino_ws/src/fairino3mt_v6_moveit2_config/config/initial_positions.yaml > /tmp/fr3c_with_gripper.urdf
/tmp/fr3c_collision_build/check_model /tmp/fr3c_with_gripper.urdf src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.srdf
```

合格输出包含 `camera_housing_link obstacle_detected=1`、`far obstacle: no contact; PASS`，进程退出码为 0。

测试程序只加载模型和构造测试场景，不连接控制器、不执行轨迹。它验证碰撞链路生效，不是全姿态穷举、真实安装精度验收或连续轨迹安全证明。

## 9. 后续接手顺序

1. 对照实物核对夹爪及相机绕法兰的方位、安装偏移、转接板和垫片厚度；优先调整安装参数，不改本体关节原点。
2. 补支架、转接盘突出部分及实际工作台、料框碰撞体，避免未建模物体被遗漏。
3. 真机接口已接入；继续核对六轴名称、角度单位、状态更新和 RViz 与实物姿态的一致性，避免模拟与真机同时发布到同一状态话题。
4. ROS/MoveIt 已采用实用 TCP v2 `[-0.847397,3.027928,178.0] mm`；正式抓取前将其写入并读回控制器专用 tool ID，同时细化夹爪碰撞几何并重新审查相机—夹爪排除规则。
5. 相机独立台架的数据接入、D2C 和相机坐标反投影已通过；见
   [P03 相机台架交接](hardware/P03_camera_bench.md)。重新装机后再做手眼标定，
   外壳估位不得替代标定。
6. 真机轨迹执行另行验证控制链、当前状态初始化及执行行为，不能把此次模型验收等同于真机运动验收。

## 10. 常见问题

| 现象 | 先检查什么 |
| --- | --- |
| RViz 模型初始姿态与真机不一致 | 检查实际硬件 plugin、joint_states 发布源和更新情况，不能仅凭 FakeSystem 实例名判断 |
| 相机仍像悬空 | 是否加载最新模型；最终 Y 为 0.0575 m，不是旧值 0.075 m |
| 修改后模型没变化 | 是否 source 正确工作区、构建安装，并重启了 MoveIt/RViz |
| 初始姿态突然碰撞 | 确认 URDF 与 SRDF 同时更新，查看具体碰撞 link 对 |
| 夹爪无法接近目标物 | 当前实心包络填满开口，需细化抓取几何和接触规则 |
| 规划拒绝但角度未越界 | 查看自碰撞或环境碰撞，不要直接扩大关节限位 |
| 相机穿过未建模支架/桌面 | 外观或 RViz 网格不等于 PlanningScene 实体碰撞体 |
