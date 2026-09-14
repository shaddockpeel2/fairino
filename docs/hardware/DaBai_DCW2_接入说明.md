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
