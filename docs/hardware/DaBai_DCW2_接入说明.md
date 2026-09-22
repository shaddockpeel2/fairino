# FR3-C DaBai DCW2-DW2 接入说明

> 切换日期：2026-09-09  
> 状态：默认相机方案已由 Orbbec Astra Stereo S U3（Gemini Pro）切换为 DaBai DCW2。

## 1. 固定的软件与设备基线

本项目借鉴
`/home/industio/DofbotPro_JetsonNano_0106/dofbot` 中已经用于 RGB-D 抓取的 P4
相机方案，但继续使用本项目已构建的新版驱动，不复制旧工作区的构建产物或旧 SDK。

| 项目 | 本机实测值 |
| --- | --- |
| SDK 枚举型号 | `DaBai DCW2` |
| 序列号 | `CH82C5201C9` |
| 固件 | `RD1017` |
| USB 功能 | `2bc5:06a0` 深度、`2bc5:0561` RGB |
| 连接 | 两个功能均为 USB 2.0 / 480 Mbit/s |
| ROS wrapper | 1.5.15 |
| Orbbec SDK | 1.10.35，ARM64 |

启动前的只读预检会同时核对序列号、两个 USB PID、480 Mbit/s 速率、授权状态和
设备节点读写权限。发现另一台 Orbbec、接口缺失或速率不符时拒绝启动，防止驱动误选设备。

## 2. 默认数据契约

默认配置是 `src/fr3c_camera/config/dabai_dcw2.yaml`：

- RGB：640×480@10 FPS，MJPG；
- 原始深度：640×400@10 FPS，Y11；
- 发布深度：硬件 D2C 后 640×480，`16UC1`，单位毫米；
- RGB 与对齐深度均发布在 `camera_color_optical_frame`；
- 关闭 IR、点云、设备软滤波、LDP 和 heartbeat；
- 固件不支持 frame sync，因此关闭 frame sync，上层严格按时间戳配对；
- 默认允许的 RGB/Depth 配对时间差为 60 ms，后续抓取融合建议沿用旧项目更严格的
  50 ms 门禁。

话题名称保持不变，后续视觉模块无需因换相机改订阅接口：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
/tf_static
```

旧 `gemini*.yaml` 与 `astra*.yaml` 只保留为历史诊断配置，不再是默认启动入口。

需要在 RViz 中查看 RGB、对齐深度和彩色点云时，使用独立的
`dabai_dcw2_rviz.yaml` 和 `camera_rviz.launch.py`，不要修改上述生产基线。
完整操作见 [DaBai DCW2 与 MoveIt 2 RViz 可视化说明](../DaBai_DCW2与MoveIt2_RViz可视化说明.md)。

## 3. 使用与验证

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_camera --symlink-install

# 终端一：先预检，再启动默认 DaBai 配置
scripts/camera/run_bench.sh

# 终端二：默认预热 10 秒、检查 60 秒
scripts/camera/audit.sh
```

单独执行预检：

```bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fr3c_camera dabai_camera_preflight
```

如确需启动历史配置，可给 `bench.launch.py` 显式传 `config:=...`，并设置
`FR3C_CAMERA_SKIP_PREFLIGHT=1`。该开关仅用于诊断，不应进入 DaBai 的生产启动脚本。

## 4. 2026-09-09 实机快速验证

在当前连接的相机上，以默认配置和严格 60 ms 时序门禁完成 20 秒检查，全部门禁通过：

| 指标 | 结果 |
| --- | ---: |
| RGB | 194 帧，9.69 FPS，最大接收间隔 0.210 s |
| 对齐深度 | 194 帧，9.69 FPS，最大接收间隔 0.312 s |
| 合格 RGB-D 对 | 187 对 |
| 时间差 P50 / P95 | 26.9 / 56.2 ms |
| 深度有效像素比例中位数 | 96.88% |
| 时间戳回退 | RGB 0，Depth 0 |

彩色和深度 CameraInfo 均为 640×480，frame 均为
`camera_color_optical_frame`，内参 K 一致。彩色带畸变参数，对齐深度畸变为 0；项目现有
`rgbd.py` 已按各自 CameraInfo 做彩色像素到 D2C 深度网格映射。

报告与快照位于
`evidence/camera/20260909_dabai_migration/quick_validation_strict/`。

## 5. 后续开发边界

本次只完成相机数据链适配，没有生成 DaBai 到机械臂的外参。现有机器人描述中的
`gemini_pro_placeholder.xacro` 仍是旧相机的外壳碰撞占位，不能当作 DaBai 的准确几何或
光学坐标。相机正式安装后需要重新测量安装姿态和包络尺寸、更新碰撞模型，并重新完成
eye-in-hand 手眼标定；在此之前只能输出相机光学坐标下的三维点，不能直接用于机械臂抓取。

MoveIt/RViz 的未标定预览会发布 `camera_housing_link -> camera_link`。该变换不能使用单位旋转：驱动还会发布标准的 `camera_link -> camera_color_optical_frame` 轴变换，单位旋转会令光轴沿外壳 X 轴，使水平桌面在 `base_link` 中显示成竖直面。当前预览使用四元数 `[0.5, -0.5, 0.5, 0.5]`（xyzw）抵消名义光学轴转换，使光学 Z 对齐相机外壳/工具伸出方向 Z。它只修正坐标轴约定，不代替手眼标定外参。

快速测试只证明已稳定获得有效深度，不替代尺量距离精度、材质覆盖、60 分钟稳定性、
冷启动/热插拔以及完整抓取闭环验收。

## 6. 15 FPS 试验配置（2026-09-22）

当前 MoveIt/RViz 路径默认加载 `dabai_dcw2_rviz.yaml`，其中彩色和深度均显式设为
10 FPS。设备历史 profile 枚举同时列出了彩色 `640×480 MJPG@15` 和深度
`640×400 Y11@15`；但这只证明单路模式存在，不证明两路同时开启并硬件 D2C 时可稳定
得到 15 FPS。此前 Astra 的 15 FPS 通过记录不能作为 DaBai 的验收结果。DaBai 先前的
15 FPS 原始深度单路测试虽然达到 15.07 FPS，但在当时启用 LDP、近距遮挡的条件下，
整图有效深度仅约 0.82%，也不能作为 RGB-D 质量合格证据。

`src/fr3c_camera/config/dabai_dcw2_rviz_15fps.yaml` 是与当前 MoveIt/RViz 配置相比
仅将 `color_fps`、`depth_fps` 改为 15 的试验配置。它保留硬件 D2C、
`full_frame` 聚合、彩色点云，以及关闭软滤波和 LDP 等条件，以便直接比较。生产默认值
仍为 10 FPS，直至同一相机、同一场景完成彩深同步和深度质量复测。
另有 `dabai_dcw2_15fps.yaml`，仅相对不发布点云的生产基线提高两路帧率，
可用于判断彩色点云生成及 RViz 负载是否妨碍达到目标帧率。驱动当前在同一彩色帧
处理线程中依次解码 MJPG、生成彩色点云、发布彩色图像；RViz 订阅点云时，这部分
计算会直接占用彩色帧发布路径。因此若带点云配置达不到目标，先对比无点云配置，
再决定是否调整可视化负载。驱动在没有点云订阅者时跳过点云计算，单独运行
`bench.launch.py` 不代表 RViz 点云负载。

在当前相机节点和机器人任务结束后，先用原配置记录一轮基线，再单独启动 15 FPS
配置；不要并行启动两个相机驱动实例：

```bash
scripts/camera/run_bench.sh config:=/home/industio/fairino_ws/src/fr3c_camera/config/dabai_dcw2_rviz_15fps.yaml
```

另一终端在相同 ROS 域运行 60 秒审计：

```bash
FR3C_CAMERA_MINIMUM_FPS=14 FR3C_CAMERA_MINIMUM_PAIRED_FPS=12 \
  FR3C_CAMERA_MAXIMUM_RECEIVE_GAP=0.3 \
  scripts/camera/audit.sh /tmp/dabai_dcw2_15fps_audit 60
```

报告需同时满足彩色与对齐深度实测至少 14 FPS、每秒至少 12 组有效彩深配对、
彩深时间差 P95 不超过 60 ms、无时间戳回退、最大接收空窗低于 0.3 秒，且深度有效率及固定
哑光平面 ROI 的稳定性不低于同场景 10 FPS 基线。再在实际 MoveIt/RViz 与感知负载下
复测；其中带点云的负载测试须让 RViz 实际订阅 `/camera/depth_registered/points`。
只有这些结果均通过，才将正式启动配置切到 15 FPS。
