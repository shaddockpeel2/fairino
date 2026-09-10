# FR3-C 手眼标定数据采集说明

## 1. 本次固定参数

```yaml
模式: eye-in-hand
机器人变换: base_link -> flange_link
输出外参: flange_link -> camera_link
彩色光学坐标: camera_color_optical_frame
相机序列号: CH82C5201C9
图像: /camera/color/image_raw
内参: /camera/color/camera_info
ChArUco 方格数: 11 x 8
方格边长: 0.015 m
Marker 边长: 0.011 m
字典: DICT_4X4_50
OpenCV legacyPattern: true
```

采集工具只订阅图像、`CameraInfo` 和 TF，没有机械臂运动发布接口。机械臂由操作者
使用法奥 WebAPP、RViz 或手动方式低速移动。

## 2. 首次构建

只需执行一次：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_calibration --symlink-install
```

## 3. 每个姿态的操作

### 启动与相机异常恢复

采集前启动机器人 TF 和相机的组合入口（会启动真实机器人控制器，按现场操作要求执行）：

```bash
cd /home/industio/fairino_ws
scripts/run_moveit_camera_rviz.sh \
  camera_config:=/home/industio/fairino_ws/src/fr3c_camera/config/dabai_dcw2.yaml
```

此处使用关闭点云的 RGB-D 基线配置；手眼采集不需要点云。2026-09-10 实测订阅彩色点云时
彩色帧率降至约 5.23 FPS，未达到 8 FPS 检查门限。

组合入口已包含相机驱动，不要再同时运行 `scripts/camera/run_rviz.sh` 或
`scripts/camera/run_bench.sh`。采集终端与组合入口的 `ROS_DOMAIN_ID`、
`ROS_LOCALHOST_ONLY` 必须一致。相机独立脚本默认使用 domain 187，仅启动它不能提供
`base_link -> flange_link`，不能代替完整手眼采集启动。

若连续出现 `Failed to decompress color frame` / `color frame is not decoded`：
先 Ctrl+C 停止启动进程，等进程退出；相机 USB 完整拔下至少 10 秒后重插，
有独立电源时一起断电，再重新启动。2026-09-10 实测该故障在 USB 软件重置后仍存在，
断电重插后恢复；具体硬件根因尚未确定。480 Mbps USB 预检通过并不能保证图像流正常。

可用独立相机检查验证恢复（两个终端分别执行，检查结束后停止独立相机再启动组合入口）：

```bash
# 终端一
/home/industio/fairino_ws/scripts/camera/run_bench.sh
# 终端二：默认预热 10 秒、检查 60 秒，结果须为 "pass": true
/home/industio/fairino_ws/scripts/camera/audit.sh
```

### 逐姿态采集

本次建议使用一个全新的固定目录：

```text
/home/industio/fairino_ws/evidence/P13/manual_30pose_20260910
```

每一组都按相同顺序操作：

1. 现场有人监护，确认急停可用，将速度限制在低速档。
2. 移动机械臂，使末端位置或朝向与上一组明显不同。
3. 保证标定板仍清晰可见，机械臂完全停稳后松开操作界面。
4. 运行下面同一条命令：

```bash
/home/industio/fairino_ws/scripts/calibration/capture_handeye_sample.sh \
  /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910
```

脚本会先执行 DaBai USB 预检。深度和 RGB 两个 Orbbec 功能都必须以 480 Mbit/s
枚举；如果设备降到 12 Mbit/s，脚本会在采样前拒绝运行，避免把相机掉线后的旧时间戳或
不完整话题误认为有效观测。

5. 看到 `"captured": true` 才表示本组成功并已追加。输出中的
   `sample_count` 是当前有效组数。
6. 如果显示 `"captured": false`，本组不会写入数据集。保持当前姿态不动，排查画面
   或等待振动消失后原样重跑命令，不要先移动到下一姿态。

不要同时运行两个采集命令，也不要手工编辑 `samples.json`。

允许并且需要旋转法兰。不要只原地旋转第 6 关节；应组合改变法兰的 roll、pitch、yaw，
并搭配平移和观察距离变化。大角度姿态下必须确认 USB/夹爪线缆保持松弛，不得牵拉相机
支架。只要机械臂已完全停稳、标定板仍清晰可见，就可以采集。

查询已采集数量：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fr3c_calibration handeye_capture \
  --session /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910 \
  --status
```

## 4. 30个姿态如何分布

不要只改变关节角而让相机末端位姿基本不变。建议按下面分布，所有位置都必须先由操作者
确认无碰撞，并确保 USB 线不受拉扯：

| 组号 | 视图与姿态建议 |
| --- | --- |
| 1～5 | 棋盘大致居中，使用约 330、360、390、420、450 mm 的不同观察距离 |
| 6～10 | 距离约 380～420 mm，让棋盘分别位于画面左、右、上、下及中心附近 |
| 11～15 | 绕末端一个横向轴使用约 -20°、-10°、0°、+10°、+20° 倾角 |
| 16～20 | 绕另一个横向轴使用约 -20°、-10°、0°、+10°、+20° 倾角 |
| 21～25 | 使用约 ±10°、±20° 的光轴旋转，并组合少量平移和倾斜 |
| 26～30 | 采用此前未重复的组合姿态，覆盖真实抓取区的边缘、中心和不同高度 |

角度和距离是目标分布，不是自动运动指令；无需精确到整数。优先保证安全、画面清晰和
姿态多样性。建议每组至少检测到20个 ChArUco 角点，实际最好保持40个以上。棋盘可以
部分出画，但前期尽量让整板可见。不要让所有姿态都保持竖直俯视。

30组可用于求解和交叉验证。如果初步分析发现姿态激励不足或留出验证数据不足，再补采
5组不同姿态，不需要推倒已有数据。

## 5. 自动生成的数据

会话目录包含：

```text
manual_30pose_20260910/
├── samples.json                  # 全部样本、内参、板参数、时间戳和质量指标
└── images/
    ├── sample_001_rgb.png        # 每组一张原始彩色图
    ├── sample_001_detected.png   # Marker/ChArUco 角点叠加图
    └── ...
```

每个有效姿态记录12个同步原始观测以及该姿态的稳健平均值：

- `base_from_wrist`：为了兼容旧数据保留的字段名，新会话中表示图像时间戳对应的
  `base_link <- flange_link`；
- `camera_from_board`：已转换到 `camera_link <- board`，不是光学帧外参；
- 角点数、Marker 数和重投影 RMS；
- 机械臂静止度、棋盘位姿的短时平移/旋转抖动；
- 与相机 SN、当前 `CameraInfo` 和标定板尺寸绑定的会话元数据。

完成30组后保留整个会话目录，并将目录路径发给我。后续再进行异常样本审计、姿态覆盖
检查、多算法手眼求解、留出验证和最终 YAML 生成；采集阶段不会提前发布未经验证的 TF。

## 6. 离线分析

数据采集完成后执行：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fr3c_calibration handeye_analyze \
  /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910
```

退出码 `0` 表示留出验证通过；退出码 `2` 表示分析已完成但未通过验收。未通过时只生成
`flange_to_dabai_dcw2.candidate.yaml`，其中 `calibrated: false`，不得加载为正式 TF。

若离线报告提示跨姿态刚体关系不一致，可在三个明显不同且已经停稳的法兰姿态各执行一次：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fr3c_calibration handeye_pose_crosscheck \
  /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910/robot_pose_crosscheck_corrected.json
```

该命令只读取 `/joint_states`、TF 和法奥控制器的实际关节/法兰位姿，不发送运动命令。
修正后默认比较 `flange_link` 和控制器 ToolFlangePose。必须使用新文件，不要与旧
`robot_pose_crosscheck.json` 的 `wrist3_link` 样本混合。在三个明显不同的停稳姿态下，平移和旋转
离散应接近零，用于确认 ROS FK 与控制器法兰一致。

### 2026-09-10 的 40 组旧模型会话

该会话采样时使用了错误的 `fairino3_mt_v6` ROS FK，所以不直接使用
`samples.json` 中的 `base_from_wrist`。使用已重建的控制器 FK 和修正模型的三姿态
现场核对文件执行：

```bash
ros2 run fr3c_calibration handeye_analyze \
  /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910 \
  --robot-poses /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910/controller_fk_reconstruction.json \
  --frame-crosscheck /home/industio/fairino_ws/evidence/P13/manual_30pose_20260910/robot_pose_crosscheck_corrected.json
```

只有同时输出 `accepted: true` 和 `deployable_in_current_ros_tf: true` 时，
`flange_to_dabai_dcw2.yaml` 才会标记为 `calibrated: true`。

## 7. 加载正式手眼 TF

先停止当前组合启动进程，等所有节点退出后重新启动：

```bash
cd /home/industio/fairino_ws
scripts/run_moveit_camera_rviz.sh \
  camera_config:=/home/industio/fairino_ws/src/fr3c_camera/config/dabai_dcw2.yaml \
  publish_calibrated_tf:=true \
  publish_mount_preview_tf:=false \
  hand_eye_file:=/home/industio/fairino_ws/evidence/P13/manual_30pose_20260910/flange_to_dabai_dcw2.yaml
```

启动入口会拒绝加载 `calibrated` 不是 `true` 的文件。加载正式 TF 时不能再发布
`camera_housing_link → camera_link` 预览 TF，否则 `camera_link` 会出现多父坐标系。
