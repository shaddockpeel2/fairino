# DaBai DCW2 与 MoveIt 2 RViz 可视化说明

更新日期：2026-09-10。

## 1. 功能边界

本工作区提供两个入口：

- 相机独立 RViz：显示 RGB、D2C 对齐深度和彩色点云，不启动机械臂控制，也不依赖手眼标定；
- MoveIt 组合 RViz：在一个 RViz 中显示真实关节状态驱动的机械臂、MotionPlanning、相机图像和点云。默认发布“外壳估位”预览 TF 以便同屏观察；要把点云用于测量、抓取或精确叠加，仍必须提供有效手眼标定。

相机可视化使用独立的 `dabai_dcw2_rviz.yaml`。原有 `dabai_dcw2.yaml` 是已经验证过的生产基线，继续保持关闭点云。

默认只发布彩色点云 `/camera/depth_registered/points`，不同时发布 `/camera/depth/points`，避免对同一深度帧执行两次点云转换。

## 2. 构建

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_camera fairino3mt_v6_moveit2_config --symlink-install
source install/setup.bash
```

## 3. 相机独立 RViz

推荐通过脚本启动。脚本会先执行 DaBai USB 身份、速率和权限预检：

```bash
cd /home/industio/fairino_ws
scripts/camera/run_rviz.sh
```

RViz 的 Fixed Frame 是 `camera_color_optical_frame`，应出现：

- `DaBai RGB Image` 对应 `/camera/color/image_raw`；
- `DaBai Aligned Depth` 对应 `/camera/depth/image_raw`；
- `DaBai RGB Point Cloud` 对应 `/camera/depth_registered/points`。

如果只想启动 launch 且已经自行完成预检：

```bash
ros2 launch fr3c_camera camera_rviz.launch.py
```

## 4. MoveIt 组合 RViz

下面的启动会加载当前 MoveIt demo 和 ros2_control 配置。当前配置为实机硬件插件，启动本身可能向机器人发送保持指令；必须先完成机器人现场安全检查。

推荐通过清理过 Qt/OGRE 环境并执行相机预检的脚本启动。默认将相机驱动的 `camera_link` 以零变换连接到 URDF 中照片估算的 `camera_housing_link`，所以未标定时也能在 `base_link` 下同时看到机械臂和点云：

```bash
cd /home/industio/fairino_ws
scripts/run_moveit_camera_rviz.sh
```

该默认 TF 仅供 RViz 联调，精度由外壳占位模型决定，不能用于计算抓取位姿。如果只想核对机器人而不显示未标定点云，可关闭它：

```bash
scripts/run_moveit_camera_rviz.sh publish_mount_preview_tf:=false
```

手眼标定文件采用：

```yaml
calibrated: true
parent_frame: wrist3_link
child_frame: camera_link
xyz: [x, y, z]            # 米
rpy: [roll, pitch, yaw]    # 弧度
```

标定完成后启动：

```bash
scripts/run_moveit_camera_rviz.sh \
  publish_calibrated_tf:=true \
  hand_eye_file:=/absolute/path/to/wrist3_to_dabai_dcw2.yaml
```

启动文件会拒绝不存在的文件、`calibrated: false`、非有限数值以及不是 `camera_link` 的 child frame，防止无意发布全零占位 TF。

## 5. 话题和 TF 验证

相机独立 `run_rviz.sh` 默认使用隔离的 ROS domain 187，所以检查独立入口时先设置相同环境。组合脚本不改 domain；检查正在运行的组合入口时使用启动终端原有的 `ROS_DOMAIN_ID`（当前默认通常为 0）：

```bash
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth_registered/points
ros2 topic echo /camera/depth_registered/points --field header --once --no-lost-messages

# 仅相机内部 TF
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame

# 组合入口下执行；默认预览 TF 或有效手眼标定启用时应该成功
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

预期 RGB 和深度图约为 10 Hz；彩色点云帧率取决于主机算力。点云的 frame 应为 `camera_color_optical_frame`，XYZ 单位是米，深度图像素值单位是毫米。

RViz 中能看到点云只代表可视化链路正常。若要让 MoveIt 把点云用于避障，还需要自体滤除、工作空间裁剪、体素降采样及 OctoMap/PlanningScene 接入，不能直接把原始点云当成碰撞环境。

## 6. 本机实测

2026-09-09 在序列号 `CH82C5201C9` 的 DaBai DCW2 上验证：

- 启动脚本的 USB 预检通过；
- RViz 使用 OpenGL 3.1 正常创建窗口；
- RViz 对 RGB、深度和彩色点云均建立了 `BEST_EFFORT` 订阅；
- RGB 和深度约 10 Hz；
- 已收到 `/camera/depth_registered/points`，frame 为 `camera_color_optical_frame`；
- `camera_link → camera_color_optical_frame` TF 可查询。

2026-09-10 又完成一次真机组合启动验证：`FairinoHardwareInterface` 成功连接，`fairino3mt_controller` 与 `joint_state_broadcaster` 均为 `active`，`/joint_states` 约 125 Hz，MoveGroup ready；组合 RViz 中机械臂模型、交互标记和彩色点云已同屏显示，`base_link → camera_color_optical_frame` 可连续查询。

2026-09-10 又完成一次真机组合启动验证：`FairinoHardwareInterface` 成功连接，`fairino3mt_controller` 与 `joint_state_broadcaster` 均为 `active`，`/joint_states` 约 125 Hz，MoveGroup ready，组合 RViz 中机械臂模型和交互标记正常显示。

当前 ARM 主机生成 640×480 彩色点云的实测速率低于图像帧率，适合查看但会比图像更新慢；若后续要求更高点云帧率，应增加降采样配置或切换为无颜色 XYZ 点云。
