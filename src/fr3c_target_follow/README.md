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

报告和原始逐帧数据会写到输出目录。原厂执行端实机验证和第四步软件接入的结果见 `docs/目标持续跟踪开发记录-20261008.md`；真实位置跟随效果待现场验证。


## 第四步接入方式

`position_follow.launch.py` 启动已验证的坐标换算节点（`transform_only=true`）和 `fr3c_follow_driver/position_follow`。坐标节点保留原图时间戳，只输出基坐标标签、实际 TCP 和换算状态，不提供偏移服务、不发布新的期望 TCP 或预览参考。执行端独占 SDK，按自己的周期生成运动参考。

`start_follow.sh` 启动后待机；查看器 **S** 记录当前偏移并开始跟随、**X** 停止、**T** 在停止时记录偏移。查看器优先显示 follow_state，区分实机运动、模拟、暂停和停止；跟随状态过期时显示提示。Q/Esc 仅关闭查看器。原有 start_pose.sh、start_preview.sh 的测量/预览行为继续保留。

第四步已编译并通过模拟验证，实机首轮用户确认跟踪良好，约 100.3 mm 目标位移触发默认范围停止；继续补齐完整演示。查看器增加范围停止及具体 Reason 提示。具体方法和参数见上述执行端说明。
