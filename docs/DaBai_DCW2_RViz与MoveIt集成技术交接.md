# DaBai DCW2、RViz 与 MoveIt 2 集成技术交接

> 日期：2026-09-09
>
> 工作区：`/home/industio/fairino_ws`
>
> 状态：相机独立 RViz 已完成实机启动和数据链验证；MoveIt 组合入口已完成软件实现与静态检查，但未在本次交接中启动真机控制栈。

## 1. 交接结论

本次已经形成两条清晰、互不混淆的使用路径：

1. **相机独立 RViz**：不启动机器人和 MoveIt，不需要手眼标定，可以直接实时查看 RGB、硬件 D2C 对齐深度和彩色点云。
2. **MoveIt 组合 RViz**：在同一个 RViz 中显示真机状态驱动的机械臂、MotionPlanning、RGB、深度和彩色点云。默认外壳估位 TF 只解决未标定阶段的同屏预览；点云要正确叠加到 `base_link`，必须加载通过验收的 `wrist3_link → camera_link` 标定变换。

相机驱动已经原生提供图像、`CameraInfo`、点云和相机内部 TF，本次没有重复编写图像或点云转发节点。原有生产基线 `dabai_dcw2.yaml` 保持不变，新增独立 RViz profile 开启彩色点云，避免把可视化负载混入已经验证的抓取数据契约。

本次没有生成手眼外参，也没有把点云接入 MoveIt PlanningScene/OctoMap。组合入口会明确发布 `camera_housing_link → camera_link` 的可视化预览 TF；它只用于让点云与机器人同屏，不能冒充标定结果参与抓取计算。

## 2. 交付文件

以下路径均以工作区根目录为基准。

| 文件 | 用途 |
| --- | --- |
| `src/fr3c_camera/config/dabai_dcw2_rviz.yaml` | DaBai 可视化 profile；沿用 640×480@10 FPS RGB、硬件 D2C 深度，开启彩色点云 |
| `src/fr3c_camera/launch/camera_rviz.launch.py` | 组合相机 bench launch 与相机专用 RViz |
| `src/fr3c_camera/rviz/dabai_dcw2.rviz` | RGB、深度、彩色点云和 TF 的 RViz 布局；Fixed Frame 为 `camera_color_optical_frame` |
| `scripts/camera/run_rviz.sh` | 推荐的相机独立入口；清理 Qt/OGRE 环境、设置隔离 ROS domain、执行 USB 预检后启动 |
| `src/fairino3mt_v6_moveit2_config/config/moveit_camera.rviz` | MotionPlanning 与相机数据显示的组合布局；Fixed Frame 为 `base_link` |
| `src/fairino3mt_v6_moveit2_config/launch/camera_moveit_demo.launch.py` | MoveIt demo、相机和组合 RViz 的入口；默认发布外壳估位预览 TF，也可选择加载经校验的手眼 TF |
| `src/fairino3mt_v6_moveit2_config/config/wrist3_to_dabai_dcw2.example.yaml` | 手眼标定文件格式示例，默认 `calibrated: false`，不能直接用于发布 TF |
| `scripts/run_moveit_camera_rviz.sh` | 组合入口包装脚本；清理渲染环境、执行相机预检并明确警告当前为真机硬件插件 |
| `docs/DaBai_DCW2与MoveIt2_RViz可视化说明.md` | 面向操作者的构建、启动、验证和故障说明 |

同时更新了 `fr3c_camera/setup.py` 的 RViz 文件安装规则，以及两个包的运行依赖。

## 3. 节点、话题与坐标契约

### 3.1 运行节点

相机独立入口主要启动：

```text
/camera/camera_container
└─ /camera/camera    orbbec_camera::OBCameraNodeDriver

/camera_rviz         rviz2
```

默认没有启动 `rgbd_probe`；如需中心像素三维探针，可传 `probe:=true`。

### 3.2 主要话题

| 话题 | 类型 | frame/单位 | 默认用途 |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `camera_color_optical_frame` | RGB 画面 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `camera_color_optical_frame`；`16UC1` 毫米 | 硬件 D2C 对齐深度 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色光学坐标 | 彩色内参与畸变 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色光学坐标 | 对齐深度内参 |
| `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` | `camera_color_optical_frame`；XYZ 米 | RGB 彩色点云 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 相机内部坐标树 | `camera_link` 到各 stream/optical frame |

图像和点云均使用 `SENSOR_DATA/BEST_EFFORT` QoS。RViz 配置显式使用 `Best Effort`，避免 Reliable 订阅者无法匹配 Best Effort 发布者。

可视化 profile 默认仅启用 `/camera/depth_registered/points`，不同时发布 `/camera/depth/points`，避免对同一深度帧做第二次 XYZ 点云转换。驱动只有发现点云订阅者时才执行点云生成；RViz 启用 PointCloud2 display 后会触发发布。

## 4. 相机独立 RViz 使用方法

### 4.1 构建

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_camera fairino3mt_v6_moveit2_config --symlink-install
```

### 4.2 推荐启动

```bash
cd /home/industio/fairino_ws
scripts/camera/run_rviz.sh
```

该脚本会：

1. 在干净 shell 中重建 ROS 库路径，去除交互环境中可能污染 RViz 的 `LD_LIBRARY_PATH` 和 `QT_QPA_PLATFORM_PLUGIN_PATH`；
2. source ROS 2 Humble 和本工作区；
3. 默认设置 `ROS_DOMAIN_ID=187`、`ROS_LOCALHOST_ONLY=1`；
4. 校验 DaBai 序列号、两个 USB PID、480 Mbit/s、授权和设备节点权限；
5. 启动相机及预配置 RViz。

RViz 左侧 Displays 中默认启用：

- `DaBai RGB Image`；
- `DaBai Aligned Depth`；
- `DaBai RGB Point Cloud`；
- `TF`。

如确需让相机加入其他 ROS domain：

```bash
FR3C_CAMERA_DOMAIN_ID=0 FR3C_CAMERA_LOCALHOST_ONLY=1 scripts/camera/run_rviz.sh
```

直接运行 `ros2 launch fr3c_camera camera_rviz.launch.py` 不会自动执行 USB 预检，也不会自动清理调用它的 shell 环境，因此日常操作优先使用包装脚本。

## 5. MoveIt 组合入口与安全边界

### 5.1 启动风险

当前 `fairino3mt_v6_moveit2_config` 加载的是：

```text
fairino_hardware/FairinoHardwareInterface
```

它不是 mock 插件。现有硬件更新循环在没有点击 Execute 时也可能发送 ServoJ 保持指令。因此组合脚本只能在机器人网络、急停、净空、使能状态和现场监护都确认后使用：

```bash
cd /home/industio/fairino_ws
scripts/run_moveit_camera_rviz.sh
```

2026-09-10 经用户明确授权实际运行了真机组合入口：硬件接口连接成功，两个控制器均激活，关节状态约 125 Hz，MoveGroup ready，RViz 已显示真实关节状态驱动的机械臂模型和交互标记。

### 5.2 未标定状态

默认 `publish_calibrated_tf:=false publish_mount_preview_tf:=true`：

- RGB 与深度 Image display 可以正常显示；
- RViz 的 Fixed Frame 是 `base_link`；
- 发布 `camera_housing_link → camera_link` 零变换，把相机内部 TF 树连接到 URDF 的外壳估位；
- 彩色点云可以与机器人同屏预览，但位置和朝向未经手眼标定，不能用于测量或抓取。

如需禁用预览连接，启动时添加 `publish_mount_preview_tf:=false`。不要把组合 RViz 中“已经看见点云”当成手眼标定完成。

### 5.3 标定文件

经过 P13 手眼标定和独立误差验收后，保存例如：

```yaml
calibrated: true
parent_frame: wrist3_link
child_frame: camera_link
xyz: [x, y, z]            # 米
rpy: [roll, pitch, yaw]    # 弧度
```

启动方式：

```bash
scripts/run_moveit_camera_rviz.sh \
  publish_calibrated_tf:=true \
  hand_eye_file:=/absolute/path/to/wrist3_to_dabai_dcw2.yaml
```

启动文件在启动相机和 ros2_control 之前检查：

- 文件存在且是映射；
- `calibrated` 必须严格为 `true`；
- parent/child 非空且不相同；
- child 必须为 Orbbec TF 树根 `camera_link`；
- `xyz`、`rpy` 都必须恰好有三个有限数值。

示例文件故意为 `calibrated: false`。实测传入该示例并要求发布 TF 时，launch 在启动相机和机器人进程前拒绝执行。

正式 P13 结果还应绑定相机 SN、支架版本、分辨率、D2C 模式、标定日期和结果 SHA256；当前最小 loader 尚未校验这些扩展元数据，后续应在 `fr3c_calibration` 中实现完整 schema。

## 6. 本次验证记录

### 6.1 软件检查

已执行并通过：

```text
Python launch py_compile：通过
YAML/RViz safe_load：通过
Bash 脚本 bash -n：通过
colcon build：fr3c_camera、fairino3mt_v6_moveit2_config 均通过
ros2 launch --show-args：两个新入口均可展开
colcon test：命令通过；两个包当前没有实际测试用例
```

### 6.2 DaBai 实机验证

相机：DaBai DCW2，SN `CH82C5201C9`，固件 `RD1017`。

实测确认：

- USB 预检通过，`06a0` 深度功能和 `0561` RGB 功能均为 480 Mbit/s，设备节点可读写；
- wrapper 1.5.15 / SDK 1.10.35 成功连接设备；
- RViz 使用 OpenGL 3.1 正常创建窗口；
- RGB 和深度约 10 Hz；
- RViz 对 RGB、深度和彩色点云均建立 `BEST_EFFORT` 订阅；
- 收到 `/camera/depth_registered/points`，header frame 为 `camera_color_optical_frame`；
- `camera_link → camera_color_optical_frame` TF 可查询。

当前 ARM 主机生成 640×480 彩色点云的更新速度明显低于 10 Hz 图像，命令行并发订阅测试约为 2～3 Hz。它适合实时观察，但若后续要求更高点云帧率，应评估无颜色 `/camera/depth/points`、点云降采样或降低可视化分辨率；不能未经回归直接改生产抓取 profile。

## 7. 已知现象与排查

| 现象 | 结论与处理 |
| --- | --- |
| RViz 报 render window/OgreWindow 创建失败 | 使用 `scripts/camera/run_rviz.sh`；脚本会清理污染的 Qt/OGRE 环境后重新 source ROS |
| 新终端 `ros2 topic list` 看不到相机 | 相机独立脚本默认 domain 187；新终端设置 `ROS_DOMAIN_ID=187`、`ROS_LOCALHOST_ONLY=1` |
| RGB/深度有画面但点云没有 | 检查 PointCloud2 display 是否启用、话题是否为 `/camera/depth_registered/points`、QoS 是否 Best Effort、Fixed Frame 是否能变换到点云 frame |
| 组合 RViz 点云报 No transform | 检查 `publish_mount_preview_tf` 是否关闭，或检查已标定 TF 文件；默认预览模式应存在 `base_link → camera_color_optical_frame` 链路 |
| 启动日志提示设备不支持 frame sync | 当前固件已明确不支持，profile 设置为关闭；上层仍按时间戳近似同步 |
| 彩色点云比图像慢 | 640×480 RGB 点云转换占用 ARM CPU；按第 9 节顺序优化 |
| Ctrl+C 时 RViz 退出码 `-11` | 与项目既有 RViz 退出崩溃现象一致；本次运行期显示正常，但退出问题尚未根治 |
| 关闭相机时出现 USB cancel/timeout warning | 本次发生在主动 Ctrl+C 清理阶段；相机进程最终正常结束，不能与运行期 USB reset 混为一谈 |

## 8. 验证命令

相机独立脚本启动后，在第二个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth_registered/points
ros2 topic echo /camera/depth_registered/points \
  --field header --once --no-lost-messages
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

发布有效手眼标定并启动组合栈后才检查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 9. 后续接手顺序

1. 先继续关闭 P03：完整 stream profile 归档、尺量距离精度、彩深边缘、材质/照度覆盖、DaBai rosbag、60 分钟稳定性和 10 次重启。
2. 若彩色点云交互明显卡顿，先测 CPU/内存/发布率，再比较无颜色 XYZ、体素降采样和较低显示分辨率；一次只改一项并保留 RGB-D 抓取基线。
3. 完成真实 DaBai 外壳、支架、转接板和线缆尺寸后，替换旧 `gemini_pro_placeholder.xacro`，重新验收碰撞模型。
4. P08/P12 冻结最终安装和相机参数后再执行 P13 手眼标定。
5. 扩展手眼 YAML schema，强制校验相机 SN、支架版本、profile 摘要和 SHA256，保证 TF 唯一发布。
6. 手眼通过后在组合 RViz 验证点云随腕部运动且稳定叠加到 `base_link`；先做无运动/低速观察，不直接进入抓取。
7. 若要让 MoveIt 使用点云避障，另行实现工作空间裁剪、自体滤除、体素降采样和 OctoMap/PlanningScene 接入；当前 RViz display 只是观察者。

## 10. 交接边界

截至本交接：

- 可以声明“DaBai RGB、对齐深度和彩色点云能够在相机独立 RViz 中实时显示”；
- 可以声明“MoveIt 与相机组合入口、预览 TF 和标定保护逻辑已经完成软件实现，并已完成一次真机显示验证”；
- 不能声明“点云已经准确叠加到机械臂”；
- 不能声明“手眼标定、MoveIt 点云避障或视觉抓取已经完成”；
- 不能因为 RViz 中看见点云就关闭 P03、P12、P13、P14 或 P15 门禁。

操作说明见 [DaBai DCW2 与 MoveIt 2 RViz 可视化说明](DaBai_DCW2与MoveIt2_RViz可视化说明.md)，相机数据基线见 [DaBai DCW2 接入说明](hardware/DaBai_DCW2_接入说明.md)。
