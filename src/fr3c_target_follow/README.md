# FR3C 独立目标跟随：位姿测量和位置预览

第五步已经提供统一位置 Demo 入口：`scripts/target_follow/start_demo.sh`，一次启动所需节点并复用已有相机和标签检测，执行端先待机。查看器 S 开始、X 停止、T 在停止时记录偏移；Demo 自己打开的窗口关闭时一并结束执行端。完整构建、日常操作和日志见 [Demo 使用说明](../../docs/目标跟踪Demo使用说明-20261008.md)。前四步资源和接口继续保留。

目标是已有 30 mm 方块顶面的 **AprilTag 36h11 / ID 1**。用户实测标签边长为 **20 mm**。标签节点输出完整位置、姿态和质量状态。第二步由独立 `fr3c_follow_feedback` 节点只读连接机械臂，换算基坐标并生成位置预览；第二步预览链路没有运动命令。第四步新增独立 C++ 执行端，真实 XYZ 跟随的构建、启动和操作见 [执行端说明](../fr3c_follow_driver/README.md#第四步固定姿态-xyz-跟随)。

## 构建和启动

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
colcon build --packages-select fr3c_target_follow --symlink-install
scripts/target_follow/start_pose.sh
```

脚本默认使用 ROS 域 187，复用该域已有的相机；没有相机话题时，预检后启动现有 DaBai 15 FPS 配置。相机被其它 ROS 域占用时，应设置 `FR3C_FOLLOW_DOMAIN_ID` 复用正确的域，不能另开相机实例。

新终端显示实时图像：

```bash
cd /home/industio/fairino_ws
scripts/target_follow/view_pose.sh
```

Q / Esc 关闭显示窗口。启动终端 Ctrl-C 关闭检测；若相机由本次 launch 启动，也会随 launch 一起关闭。复用的独立相机不随检测退出。

## 输出和坐标

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/target_follow/tag_pose` | `geometry_msgs/PoseStamped` | 相机光学坐标下的标签位姿，米、四元数 xyzw |
| `/target_follow/quality` | `std_msgs/String`，JSON | 识别状态、角点、处理时间、图像年龄、重投影误差、姿态歧义 |
| `/target_follow/image_annotated` | `sensor_msgs/Image` | 标签框、三维坐标轴、XYZ/RPY 读数 |
| `/tf` | TF | `camera_color_optical_frame → follow_tag_1`，有效观测时更新 |

位姿原点是**顶面标签中心**，不是方块中心。标签坐标 X 向其编码方向的右侧、Y 向上、Z 朝顶面外侧；相机光学坐标 X 向图像右、Y 向下、Z 向前。检测角点按照编码方向排序，所以当前标签旋转约四分之一圈是正常的。

`marker_size_m: 0.020` 是四个检测角点之间的实测边长。`cube_edge_m: 0.030` 仅记录物体尺寸，不参与标签 PnP 的尺度计算，也不自动将位姿平移到方块中心。

小标签成像相位和视角变化可能使单一阈值漏检。检测先使用阈值常量 13；目标 ID 未识别时依次尝试 21、25，找到目标即停止重试。仍使用原字典解码和原位姿质量检查，不会强制填入目标 ID。质量 JSON 的 `detector_threshold` 和 `detector_passes` 记录实际使用的检测步骤。

PoseStamped 保留原图时间戳。无目标、同 ID 多个目标、图像过期、时间戳回退、内参/坐标系不匹配时不发布新位姿。TF 缓存仍可能保存最后一次变换，后续跟踪必须检查观测时间和质量，不能仅请求 latest TF。

位置和姿态都已经求解并输出。`orientation_ambiguous: true` 表示平面标记候选解接近，当前图像对旋转的约束较弱；这不是机械臂控制信号。本阶段姿态只用于观察和记录。

## 实机采样

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
ros2 run fr3c_target_follow pose_audit --duration-s 30 \
  --output /home/industio/fairino_ws/evidence/target_follow/manual_pose_audit
```

输出 `report.json`、逐帧 `samples.jsonl` 和 `annotated.png`。如果方块或相机在动，报告中的位置/姿态离散度包含真实运动，不能将其解读为测量噪声。静止采样的离散度也不是绝对定位精度。

2026-10-08 的实机数据位于 `evidence/target_follow/20261008/`。初始检测、小标签参数对比和最终实时复测分开保存。

## 验证

兼容系统 OpenCV 4.5.4 和用户环境 OpenCV 4.10 的两种 ArUco API。使用项目相机环境脚本，保留 `PYTHONNOUSERSITE=1`，避免改变其它 ROS 相机依赖。

```bash
source /home/industio/fairino_ws/scripts/camera/activate.bash
export PYTHONPATH="/home/industio/fairino_ws/src/fr3c_target_follow:${PYTHONPATH:-}"
python3 -m pytest /home/industio/fairino_ws/src/fr3c_target_follow/test/test_marker_pose.py -q

# 用合成图像在独立 ROS 域验证位姿发布、丢失、过期和恢复。
FR3C_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/fr3c_target_follow_contract_ros \
  python3 -m pytest /home/industio/fairino_ws/src/fr3c_target_follow/test/test_pose_node_ros.py -q
```

只读机械臂反馈、拍摄时刻坐标变换和位置 PBVS 预览已完成，见下一节。后续步骤是原厂伺服执行端和真实位置跟随，按 `docs/目标跟踪Demo分步开发规划-20261008.md` 继续。第一版固定末端姿态。

## 第二步：基坐标和位置 PBVS 预览

先启动上一节的相机和标签检测，再启动预览：

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
colcon build --packages-select fr3c_follow_feedback fr3c_target_follow --symlink-install
scripts/target_follow/start_preview.sh
```

预览脚本复用同 ROS 域已有的只读反馈；没有反馈话题时启动独立 SDK 读节点。该读节点只使用 `RPC`、`GetRobotRealTimeState` 和 `CloseRPC`，不切模式、不使能、不发运动。当前版本不与已有 ros2_control 控制连接合并，后续实际执行需要由一个 SDK 所有者统一处理。

两个启动脚本检查实际发布者数量，避免把仅有订阅者的同名话题误当成运行中的数据源。已经运行的检测、反馈和预览会复用；部分节点退出时，只补启动缺少的节点。

记录当前偏移：在实时查看器按 **T**，或使用服务：

```bash
source /home/industio/fairino_ws/scripts/camera/activate.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
ros2 service call /target_follow/teach_offset std_srvs/srv/Trigger '{}'
```

记录的是当前实际 TCP 到标签中心的基坐标偏移和当前 TCP 旋转。随后目标平移改变期望 TCP 位置，目标旋转不改变期望 TCP 旋转。T 只修改软件里的偏移，不触发机械臂。

| 话题 | 含义 |
| --- | --- |
| `/target_follow/flange_pose` | 新 SDK 反馈对应的法兰位姿，基坐标、米/四元数 |
| `/target_follow/robot_feedback` | SDK 返回码、反馈年龄、控制器工具/工件坐标、状态 |
| `/target_follow/tag_pose_base` | 结合拍摄时刻法兰位姿换算后的标签位姿，保留原图时间戳 |
| `/target_follow/tcp_pose_base` | 由法兰和已标定 TCP 计算的实际夹爪 TCP |
| `/target_follow/desired_tcp_pose` | 标签位置加记录偏移，保持记录时的 TCP 旋转 |
| `/target_follow/preview_velocity` | 限速度、限加速度的位置比例反馈预览，角速度始终为零 |
| `/target_follow/preview_reference` | 从实际 TCP 出发的一步位置参考，不能当作累计运动轨迹 |
| `/target_follow/preview_state` | 位置误差、新鲜度、插值间隔和状态，运动/姿态控制均关闭 |

新反馈包使用主机首次观察到新 SDK 包的时刻作为时间戳；旧缓存包不会被重新打时间戳。当前控制器显示时钟与主机相差约 330 秒，因此没有直接将控制器日期与相机时间混用。主机观察时刻仍有传输/轮询偏差，小幅平移测试验证了当前低速场景，未证明高速运动的时序精度。

目标变换按图像时间在线性插值法兰位置、SLERP 插值旋转，然后串联 `flange_link → camera_link → camera_color_optical_frame`。无历史包围、反馈间隔过大、反馈过期或目标过期时不生成新位置参考，速度预览归零。

默认预览参数在 `config/preview.yaml`：50 Hz、位置增益 1.5/s、速度上限 10 mm/s、加速度上限 30 mm/s²。目标保持时间上限 350 ms，参考已有约 268 ms 的有效观测间隔设置；实际执行阶段的停止路径需单独验证。

采样：

```bash
source /home/industio/fairino_ws/scripts/camera/activate.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
ros2 run fr3c_target_follow preview_audit --duration-s 20 \
  --output /home/industio/fairino_ws/evidence/target_follow/manual_preview_audit
```

报告和原始逐帧数据会写到输出目录。原厂执行端实机验证及第四步原有位置跟随的实机效果见 `docs/目标持续跟踪开发记录-20261008.md`；新增模型防碰撞的现场反馈另有记录。


## 第四步接入方式

`position_follow.launch.py` 启动已验证的坐标换算节点（`transform_only=true`）和 `fr3c_follow_driver/position_follow`。坐标节点保留原图时间戳，只输出基坐标标签、实际 TCP 和换算状态，不提供偏移服务、不发布新的期望 TCP 或预览参考。执行端独占 SDK，按自己的周期生成运动参考。

`start_follow.sh` 启动后待机；查看器 **S** 记录当前偏移并开始跟随、**X** 停止、**T** 在停止时记录偏移。查看器优先显示 follow_state，区分实机运动、模拟、暂停和停止；跟随状态过期时显示提示。Q/Esc 仅关闭查看器。原有 start_pose.sh、start_preview.sh 的测量/预览行为继续保留。

第四步原有位置跟随的实机效果已由用户确认可用，首轮约 100.3 mm 目标位移触发当时的 100 mm 范围停止，当前配置半径为 1.00 m。第五步不要求额外一分钟演示。查看器显示范围停止及具体 Reason，当前参数和操作见 [Demo 使用说明](../../docs/目标跟踪Demo使用说明-20261008.md)。

新增模型防碰撞由 `start_demo.sh` 或 `start_follow.sh` 的 `collision_enabled:=false/true` 选择，当前默认 false。显式参数覆盖配置文件，切换需停止并退出后重启；查看器 OFF/ON、实时 `collision.enabled` 和 `tracking_mode` 表示当前模式。开启模式此前出现过尤其 Z 上升时的频繁启停，开关功能未修复该卡顿，详见 [轻量防碰撞使用说明](../../docs/目标跟踪轻量防碰撞使用说明-20261008.md)。

## 第六步：只读姿态质量检查

软件实现、模拟链路和一轮真实只读采样已完成。此入口只运行检测、只读机械臂反馈、基坐标换算、姿态检查和记录；没有运动执行端。姿态仍未用于控制，方块三轴旋转和示教旋转的现场验证尚待完成。

先停止并退出位置 Demo/伺服启动终端，构建后运行：

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
colcon build --packages-select fr3c_follow_feedback fr3c_target_follow --symlink-install
scripts/target_follow/start_orientation_check.sh scenario:=static duration_s:=30
```

脚本自动启动缺少的相机、标签检测、只读反馈和基坐标换算，复用同域已有只读数据源。发现运动执行端、重复发布者或不完整反馈源时拒绝启动。默认域仍为 187。无窗口加 `--no-view`；窗口标题为 READ ONLY，只有 Q/Esc 退出，T/S/X 不创建或调用服务。记录完成、关闭窗口或 Ctrl-C 均关闭本次启动的节点，外部复用的数据源保留。

SSH 实测建议加 `--no-view`，并显式传入 `output_dir:="$stage6_run_dir/static"` 等输出目录；仅定义变量不会改变脚本的默认保存位置。可直接复制的三组命令、操作顺序和完成判断见 [第六步姿态实测操作](../../docs/目标跟踪第六步姿态实测操作-20261009.md)。已修复本机 OpenCV 4.5.4 GTK3 不支持窗口可见性属性时误退出的问题。

以可展示的粗略姿态为目标时可加 `quality_profile:=demo`，使用 0.10 px 歧义阈值、2 帧确认；默认 `conservative` 仍用 0.15 px、5 帧。此选项只控制本次启动的检测/检查参数，不会修改复用的外部检测器；显式 `params_file` / `tag_params_file` 优先，会话记录实际请求文件。该入口仍无运动输出，放宽并不提高原始姿态精度。回放对比和后续死区/滤波设计见 [Demo 阈值说明](../../evidence/target_follow/stage6/manual_20261009_101620/demo_threshold_analysis/README.md)。

旧标签检测进程若尚未重启，可能没有新增整数时间戳/候选质量字段；状态会显示 `missing_exact_image_stamp` 或质量缺失，应先重启该检测进程，不用浮点时间近似配对。

分别做与本步相关的三类现场检查，时长可自行调整：

| 场景 | 运行参数 | 现场操作与判断 |
| --- | --- | --- |
| 静止观察 | `scenario:=static` | 固定方块和机械臂，观察可信/歧义比例及基坐标姿态变化 |
| 方块转动 | `scenario:=target_rotation` | 机器人不动，缓慢小幅转动方块的三个轴，观察方向、连续性与标签可见范围 |
| 机械臂转动 | `scenario:=robot_rotation` | 固定方块，通过示教器小角度转动机械臂；基坐标方块姿态应基本不变 |

场景参数只给记录加标签，不会指挥机器人或验证目标是否真的静止。转动机械臂由现场操作完成；本入口不切换控制器模式或使能。改变观察角度前先看实时图像，保持标签完整可见。

| 新话题 | 类型与含义 |
| --- | --- |
| `/target_follow/pose_observation` | String/JSON：同一原图整数时间戳配对的基坐标位姿、原始质量、位置/姿态可用性和拒绝原因；丢失/过期时也发布失效事件 |
| `/target_follow/orientation_state` | String/JSON：20 Hz 状态、数据年龄、连续确认次数和配对统计；心跳不改变原图 `stamp` |

`stamp={sec,nanosec}` 是原图精确时间；`published_stamp_s` 只是诊断发送时间，后续消费者必须检查原图年龄。`valid_position` 与 `valid_orientation` 分开：歧义姿态仍可有有效位置，现有位置话题继续输出。`orientation_measurement_valid` 表示本帧通过候选解检查，`valid_orientation` 还要求基坐标连续性与连续确认。

默认检查参数在 `config/orientation_check.yaml`：连续 5 个不同图像观测，目标/反馈超时 350/200 ms，旋转变化上限为 `3° + 60°/s × 实际图像间隔`。角速度检查在 `base_link` 中进行，不把相机随机械臂转动当作标签跳变；拒绝的跳变不替换上一可信基准。歧义、跳变、缺失质量、目标丢失或反馈中断会撤销可用状态，恢复后重新确认。计数不随心跳增加。

原 PnP 选解规则与位置链路保持原样。本步增加了候选误差、候选夹角、选中候选索引；若历史连续性选择了明显不同的次优分支，也拒绝该帧姿态。连续性只能检查观测轨迹，不能证明平面双解已经正确消除。

每次输出到 `evidence/target_follow/stage6/<会话>/`，包含 `session.json`、`samples.jsonl`、`report.json` 与 ROS 日志。报告分别给出全部测量与通过确认的姿态、基坐标旋转向量各轴变化和法兰运动，不能把真实运动产生的离散度解释为噪声或绝对精度。

`samples.jsonl` 同时保留已有 `/target_follow/robot_feedback` 的状态和工具/工件坐标元数据（`kind=feedback`），用于排查机器人换位时的坐标一致性，不新增 SDK 调用。单独采样和回放可用：

```bash
source /home/industio/fairino_ws/scripts/camera/activate.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
ros2 run fr3c_target_follow orientation_audit --duration-s 30 --scenario target_rotation \
  --output /home/industio/fairino_ws/evidence/target_follow/stage6/manual_rotation
ros2 run fr3c_target_follow orientation_audit \
  --replay /home/industio/fairino_ws/evidence/target_follow/stage6/manual_rotation/samples.jsonl \
  --scenario target_rotation --output /home/industio/fairino_ws/evidence/target_follow/stage6/replay_rotation
```

`start_sources:=false` 只启动检查/审计，用于外部合成数据的独立 ROS 验证，不打开相机或机器人。已验证的启动/退出、错配、丢失恢复与 eye-in-hand 旋转案例见 `evidence/target_follow/stage6/`。

2026-10-09 的 10 秒真实只读采样成功配对 99 个观测，79 个通过连续确认，4 个姿态歧义，16 个处于重新确认；未发送运动指令。该轮场景标记为 unspecified，未由现场确认方块完全静止，不能视为三轴转动或转角范围验证。证据为 `live_readonly_20261009/report.json`。

## 第七步：完整位姿只读预览

已完成软件、模拟、窗口及一轮 90 秒真实只读预览，达到当前 Demo 检查范围。机械臂保持不动，T 记录目标到 TCP 的完整相对变换；缓慢转动方块，查看期望 TCP 的位置绕转和姿态。2026-10-10 默认改用 Demo 质量与 `config/pose_preview_demo_stable.yaml` 的 0.8 秒姿态滤波、6°/4°死区滞回和 0.5 映射；旧 `pose_preview.yaml` 保留可选。真实记录分析见 [20261009_122550_231933](../../evidence/target_follow/stage7/20261009_122550_231933/analysis/README.md)，尚未验证真实姿态运动或绝对转角精度。

```bash
export FR3C_FOLLOW_DOMAIN_ID=187
/home/industio/fairino_ws/scripts/target_follow/start_pose_preview.sh duration_s:=90
```

SSH 用 `--no-view auto_teach:=true`，等 `recorded=True` 再转动方块；自动记录只保存预览关系，不启动运动。默认 0.5 同时减弱末端旋转和位置绕转；可加 `rotation_mapping_gain:=1.0` 恢复完整幅度，死区允许几度残差。每次 T 从当前实际位姿重新记录，无需复现历史起点。详细步骤见 [第七步位姿预览操作](../../docs/目标跟踪第七步位姿预览操作-20261009.md)。

本入口只启动缺少的只读相机/检测/反馈/换算与姿态检查，拒绝运动执行端、重复源及旧姿态检查；复用外部检测器时不改其阈值。查看器 `--pose-preview` 只有 T 记录客户端，没有 S/X 运动客户端。所有位姿、速度和状态在 `/target_follow/pose_preview/` 下，不发布执行端期望目标；唯一操作服务为 `/target_follow/pose_preview/teach`。

`raw_target`、`filtered_target`、`mapped_target`、`actual_tcp`、`desired_tcp`、`reference_tcp` 和 `reference_flange` 为数据话题；`state` 保存原图时间戳、质量原因、角/位置误差和参数，`velocity` 是预览速度。参考始终从实际反馈出发，并换算完整 TCP/法兰位姿。短时歧义冻结上一目标，最后可信图像超过 350 ms、标签丢失、显著跳变或反馈过期时速度归零，保持不能延长可信时间。

每次记录到 `evidence/target_follow/stage7/<会话>/`，包含会话参数、原始状态/观测/质量/反馈、报告与 ROS 日志。证据、用户静止数据离线回放与曲线也在 `stage7/`；尚未接入真实机器人旋转执行端。

## 第九步：视觉位姿跟随入口

新增 `pose_follow.launch.py` 与 `scripts/target_follow/start_pose_follow.sh`，复用本包图像时刻坐标换算与 Demo 姿态质量检查，将配对后的完整观测交给 `fr3c_follow_driver/pose_follow`。坐标换算使用 `transform_only=true`，不创建执行目标发布者或额外 SDK 所有者；驱动独占实际反馈与期望 TCP 发布。入口检查冲突源，只启动缺少的相机/检测器；关闭本次窗口、Ctrl-C 或任一自有组件退出时关闭本次节点，外部复用的源保留。

```bash
/home/industio/fairino_ws/scripts/target_follow/start_pose_follow.sh
```

默认使用 Demo 质量（2 帧确认、0.10 px 歧义阈值），启动待机。窗口 T 记录、S 重新记录当前完整关系并开始、X 停止。显示 `POSE`、实际/期望姿态、角差、角速度、旋转映射、质量和防碰撞状态。默认 0.8 s 姿态滤波、6°/4°死区与 0.5 映射，使末端旋转和位置绕转共同减幅；无需复现之前起点。旧 `start_demo.sh` 仍使用位置跟随，第十步再统一模式入口。

三轮现场测试中，后两轮被近正视小标签的候选歧义挡住启动。第九步现在单独使用 `tag_pose_demo_follow.yaml`，将候选旋转差异容差从 5°放宽到 35°，明确标记近似使用；原始位置、PnP 选解、旧只读配置不变。窗口显示 `APPROX DEMO`，执行端仍使用 6°/4°死区和半幅旋转。该次修正的位姿位置范围为 200 mm。

后续现场平移可运行，但倾斜被基坐标连续性和恢复转角阈值卡住。第九步改用 `orientation_check_pose_demo.yaml`：25°观测抖动余量、原有 60°/s 速率项及 2 帧确认；该次修正的驱动原始恢复角差为 40°，实际运动范围为 20°。旧 `orientation_check_demo.yaml` 的 6°余量保留，旋转滤波、死区和映射不变。

软件/模拟已通过，现场已有平移与倾斜动作，最新范围和保持处理待确认。可以缓慢移动和转动方块，检查静止是否稳定、旋转是否明显平滑、遮挡恢复及 X/S 是否可用；无需重复第八步独立旋转测试或精密误差验收。命令、当前范围与日志说明见 [第九步操作](../../docs/目标跟踪第九步位姿跟随操作-20261010.md)，模拟证据见 [stage9](../../evidence/target_follow/stage9/README.md)。

最新现场触发旧 20°转角范围后，当前执行端扩大为 500 mm/90°，恢复原始目标差值 150 mm/120°，并新增 4 mm/2°到位保持、8 mm/4°新目标阈值及 0.2 s 可信帧确认。窗口 `POSE HOLD` 表示已到位，保留固定参考而忽略小视觉误差；`motion pending` 表示正确认明显变化。固定完整期望 TCP，旋转噪声引起的位置绕转也一并冻结。最新诊断与模拟见 [范围与静止保持](../../evidence/target_follow/stage9/range_hold_tuning_20261010/README.md)。

用户随后明确要求区域改为 1 米、速度不变。当前 `pose_follow` 半径为 1.00 m，90°姿态范围及静止保持继续生效。最新现场 SDK 112 属于目标位姿无法到达，未触发软件半径；记录见 [1 米区域修正](../../evidence/target_follow/stage9/radius_1m_20261010/README.md)。
