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

## 6. 帧率标定与 15 FPS 默认配置（2026-09-22）

原 MoveIt/RViz 路径默认加载 `dabai_dcw2_rviz.yaml`，其中彩色和深度均显式设为
10 FPS。设备历史 profile 枚举同时列出了彩色 `640×480 MJPG@15` 和深度
`640×400 Y11@15`；但这只证明单路模式存在，不证明两路同时开启并硬件 D2C 时可稳定
得到 15 FPS。此前 Astra 的 15 FPS 通过记录不能作为 DaBai 的验收结果。DaBai 先前的
15 FPS 原始深度单路测试虽然达到 15.07 FPS，但在当时启用 LDP、近距遮挡的条件下，
整图有效深度仅约 0.82%，也不能作为 RGB-D 质量合格证据。

`src/fr3c_camera/config/dabai_dcw2_rviz_15fps.yaml` 相对 `dabai_dcw2_rviz.yaml`
仅将 `color_fps`、`depth_fps` 改为 15，保留硬件 D2C、`full_frame` 聚合、彩色点云，
以及关闭软滤波和 LDP 等条件。另有 `dabai_dcw2_15fps.yaml`，仅相对不发布点云的
生产基线提高两路帧率。

### 6.0 设备只接受离散帧率

设备 profile 枚举里**没有 12 FPS 这个模式**，可选值只有 `5/10/15/20/25/30`（彩色）
与 `5/10/15`（深度）。把 `color_fps` 或 `depth_fps` 设为 12 会被驱动直接拒绝：

```
[warning] Invalid input, No matched video stream profile found!
[ERROR] Failed to get color profile: ... FPS: 12, Format: OB_FORMAT_MJPG
[ERROR] The device might be connected via USB 2.0. ...
```

因此"把帧率设成 12"不可行；要达到 ~12 FPS 的实际输出，只能**请求 15 FPS 模式**
并接受当前两路实测约 12–13.5 FPS。这也解释了为什么 10 FPS 请求只得到约 9：
请求值本身就是节流目标，请求 10 就把上限设在 10。

### 6.1 实测结论（2026-09-22 已完成）

已在机器人抓取栈与相机驱动全部退出、无其它进程占用相机 USB 节点的条件下完成实测
（60 s 审计、10 s 预热）。完整数据与复现命令见
[evidence/optimization/20260922_camera_fps](../../evidence/optimization/20260922_camera_fps/README.md)。

| 配置 | 彩色 FPS | 深度 FPS | 对齐配对 | 深度有效率 |
|---|---|---|---|---|
| `dabai_dcw2_rviz.yaml`（10 FPS） | 9.80 | 9.00 | 526 | 95.6% |
| `dabai_dcw2_rviz_15fps.yaml`（15 FPS） | 13.59 | 12.34 | 681 | 94.3% |
| `dabai_dcw2.yaml`（10 FPS） | 8.97 | 8.82 | 477 | 94.3% |
| `dabai_dcw2_15fps.yaml`（15 FPS） | 13.67 | 12.15 | 670 | 94.3% |

同域配对对比：彩色 8.97 → 13.67 FPS（+52%），深度 8.82 → 12.15 FPS（+38%），
每秒对齐配对 7.95 → 11.17（+41%）。

隔离实验定位了瓶颈：

- 仅深度 @15 FPS 稳定 **15.01 FPS**（最大空窗 95 ms），仅彩色 @15 FPS 达 **14.32 FPS**，
  证明设备单路能力足够，15 FPS 不是配置或器件能力问题。
- 驱动进程仅占单核约 21%，整机 8 核空闲 86%，**不是 CPU 瓶颈**。
- 关闭彩色点云后彩色 13.67、深度 12.15，与开启点云的 13.59/12.34 在噪声范围内，
  **点云不是瓶颈**（原"点云占用彩色处理线程"的推测已被实测否定）。
- 相机当前经两级 USB 2.0 Hub 挂在 EHCI（USB 2.0）根控制器上，链路锁定 480 Mbps；
  而本机 `usb2`/`usb8` 为 5 Gbps xHCI 控制器。USB 2.0 链路可能影响双路吞吐，
  但现有实验尚未单独证实带宽是唯一瓶颈。
- 关闭 D2C 对齐后深度由 12.15 升到 14.85 FPS：对齐需等待彩色帧，在带宽饱和时进一步
  放大深度等待。

### 6.2 处置决定（已执行）

**运行路径默认值已切到 15 FPS。** 依据是"请求值即节流目标"：请求 10 只得到约 9，
请求 15 得到彩色 13.5、深度 12.6，深度恰好落在期望的 ~12 FPS。同一域（192）匹配复测：

| 配置 | 彩色 FPS | 深度 FPS | 对齐配对 | 深度有效率 |
|---|---|---|---|---|
| 10 FPS（`dabai_dcw2_rviz.yaml`） | 9.80 | 9.05 | 531 | 95.6% |
| 15 FPS（`dabai_dcw2_rviz_15fps.yaml`） | 13.53 | 12.58 | 691 | 94.3% |

即彩色 +38%、深度 +39%、每秒对齐配对 +30%，深度有效率仅由 95.6% 降到 94.3%。

已切换的调用点：

- `src/fairino_mtc_demo/launch/full_first_grasp.launch.py`（生产抓取主链路）
- `src/fairino3mt_v6_moveit2_config/launch/camera_moveit_demo.launch.py`（MoveIt 显示）
- `src/fr3c_camera/launch/camera_rviz.launch.py`（独立 RViz 可视化）

切换后经安装路径解析实际默认配置复测：**彩色 13.79 / 深度 12.53 FPS、707 组对齐配对、
深度有效率 94.2%**，确认生产链路加载的就是 15 FPS profile。

**未切换**（刻意保守）：

- `dabai_dcw2.yaml`（10 FPS 无点云生产基线）保持不变，仍被
  `bench.launch.py` 台架审计与`docs/calibration/`手眼标定流程引用。
  帧率不改变空间标定结果（D2C 已把深度重投影到彩色视角），但标定采集期间
  不应同时变更多个变量。
- 各感知节点（`blue_cube_localizer`、`color_target_2d`、`probe_node` 等）只订阅
  `/camera/*` 话题，不读取帧率配置，无需改动。

残留观察项：15 FPS 下深度最大空窗实测 0.28–1.02 s 波动，10 FPS 下也出现过约 0.80 s，
**不是帧率改动引入的**，需单独排查（与 USB 2.0 突发调度或 SDK 帧聚合有关）。

若以后排查性能，可将相机直连另一主机端口做对照，再按 6.3 重跑验收。
当前两个相机 USB 功能都以 USB 2.0 枚举；换到 5 Gbps 主机口并不保证设备本身会以
USB 3.0 工作，也不能预先保证两路达到 15 FPS。

### 6.3 复现与复验步骤

下列步骤已完成过多轮（结果见 6.0–6.2）。运行默认值**已切到 15 FPS**；
若以后改变连接方式，需用同一套命令复验。不要并行启动两个相机驱动实例：

```bash
# 终端 1：启动相机（15 FPS 当前默认；对比 10 FPS 基线可换成 dabai_dcw2_rviz.yaml）
scripts/camera/run_bench.sh config:=/home/industio/fairino_ws/src/fr3c_camera/config/dabai_dcw2_rviz_15fps.yaml
```

```bash
# 终端 2：在相同 ROS 域运行 60 秒审计
FR3C_CAMERA_MINIMUM_FPS=11.5 FR3C_CAMERA_MINIMUM_PAIRED_FPS=10 \
  FR3C_CAMERA_MAXIMUM_RECEIVE_GAP=1.5 \
  scripts/camera/audit.sh /tmp/dabai_dcw2_15fps_audit 60
```

以上门槛用于复验当前接受的约 12 FPS 模式；检验理想 15 FPS 目标需另行提高门槛。

理想的 15 FPS 目标：彩色与对齐深度实测至少 14 FPS、每秒至少 12 组
有效彩深配对、彩深时间差 P95 不超过 60 ms、无时间戳回退、最大接收空窗低于 0.3 秒，
且深度有效率不低于同场景 10 FPS 基线。

当前 USB 2.0 链路的实际达成（已接受为当前最优，见 6.2）：彩色 13.79 / 深度 12.53 FPS、
每秒 11.8 组配对、深度有效率 94.2%。**未达到上述 14 FPS 目标**，瓶颈尚未单独确认；
"最大接收空窗低于 0.3 秒"在 10 FPS 基线上也未通过（实测约 0.80 s），需单独排查。

### 6.4 项目内调用点（切换状态一览）

生产抓取链路的相机配置由 `fairino_mtc_demo` 的启动文件唯一注入；显示与标定路径各有自己的
默认值。**2026-09-22 已将运行路径切到 15 FPS**，当前状态如下：

| 位置 | 当前默认 | 作用 | 状态 |
|---|---|---|---|
| `src/fairino_mtc_demo/launch/full_first_grasp.launch.py:100` | `dabai_dcw2_rviz_15fps.yaml` | 生产抓取主链路 `camera_config` | 已切 15 |
| `src/fairino3mt_v6_moveit2_config/launch/camera_moveit_demo.launch.py:127` | `dabai_dcw2_rviz_15fps.yaml` | MoveIt 显示 demo | 已切 15 |
| `src/fr3c_camera/launch/camera_rviz.launch.py:34` | `dabai_dcw2_rviz_15fps.yaml` | 独立 RViz 相机可视化 | 已切 15 |
| `src/fr3c_camera/launch/bench.launch.py:26` | `dabai_dcw2.yaml` | 台架审计入口 | 刻意保持 10 |
| `docs/calibration/FR3C手眼标定数据采集说明.md` | `dabai_dcw2.yaml` | 标定采集命令 | 刻意保持 10 |

`dabai_dcw2.yaml` 与 `dabai_dcw2_15fps.yaml` 保持为 10 FPS 基线与无点云对照，供台架审计
和手眼标定使用：帧率不改变空间标定结果（D2C 已把深度重投影到彩色视角），但标定采集期间
不应同时变更多个变量。

各感知节点（`blue_cube_localizer`、`color_target_2d`、`probe_node`、`quality`、`audit`
等）只订阅 `/camera/*` 话题，不读取帧率配置，因此帧率变更**无需**修改这些节点。
注意 `run_moveit_camera_rviz.sh` 未透传 `camera_config`，切换默认值是改上述启动文件的
`default_value`，不是改脚本。
