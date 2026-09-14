# P3 蓝色方块定位开发与现场只读验证记录

日期：2026-09-11

对象：DaBai DCW2、当前视野内单个蓝色 30×30×30 mm 方块。

## 实现

- 新增 `fr3c_camera.cube_perception`：蓝色 HSV 分割、候选几何筛选、候选内部掩码、D2C 畸变模型映射、批量反投影、基座变换和水平顶面提取。
- 新增 `blue_cube_localizer`：RGB/深度/两路 CameraInfo 近似同步、数据新鲜度检查、机械臂静止检查、图像时刻 TF、在线手眼 YAML 一致性校验、唯一目标门禁、工作区/尺寸/深度质量门禁和多帧稳定冻结。
- 已把之前验收的手眼参数部署到 `dabai_dcw2_handeye.yaml`，并把组合启动默认值切换为标定 TF；首次抓取定位入口同时加载 `ydi_j22_tcp.yaml`，核对在线 `flange_link -> grasp_frame`。
- 输出冻结顶面中心、有效标志、JSON 状态、RViz Marker 和调试图；提供显式 reset 服务。
- 对纯色立方体不可观测的 yaw 不做虚假估计，使用显式 `fixed_yaw_rad`。

## 离线验证

命令：

```bash
PYTHONPATH=/home/industio/fairino_ws/src/fr3c_camera \
python3 -m pytest -q src/fr3c_camera/test/test_cube_perception.py
```

结果：4 passed。覆盖单目标、双目标不合并、D2C 反投影/顶面拟合和手眼四元数差异判断。

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_camera --symlink-install
```

结果：通过。

## 当前现场结果

当前运行的 `camera_moveit_demo.launch.py` 未传 `publish_calibrated_tf:=true`，节点树中存在 `/dabai_mount_preview_tf`。在线 `flange_link -> camera_link` 是安装预览值，而不是已验收文件：

```text
evidence/P13/manual_30pose_20260910/flange_to_dabai_dcw2.yaml
```

严格模式成功拒绝发布可执行位姿，状态为：

```text
online_hand_eye_mismatch:0.040508m:0.134856rad
```

这两个数只是在线预览变换与已验收手眼变换的参数差，不代表已验收手眼精度变差。手眼验证统计仍为平移中位数 2.274 mm、P95 6.327 mm。

初次开发时 TCP 使用 `[-0.847397, 3.027928, 178.0] mm`，严格在线一致性检查通过。随后用户在实物指尖接触桌面时发现 RViz 仍显示约 6 mm 间隙，这与固定点求解 Z=184.614459 mm 相对 178 mm 的差值 6.614459 mm 高度吻合。源码提升到 184.614459 mm 并同步延长手指模型后，用户通过法兰按钮手动拖拽到桌面接触，确认 RViz 与实物严密对齐。重启后的在线 `flange_link -> grasp_frame` 已读回为 `[-0.000847397, 0.003027928, 0.184614459] m`，现接受用于首次 RGB 抓取。

同次在线核对确认 `flange_link -> camera_link` 已是验收手眼值 `xyz=[-0.0219964,-0.0729000,0.0453301] m`、`rpy=[2.888921,-1.440795,-1.354010] rad`，不再是照片预览 TF。

为覆盖分割、深度、顶面和稳定冻结代码路径，进行了一次明确标记为“不可用于运动”的宽容差诊断运行。当前目标统计：

| 指标 | 结果 |
| --- | --- |
| 蓝色候选数 | 1 |
| 候选轮廓面积 | 约 1912.5～1942.0 px² |
| 候选内部有效深度比例 | 约 99.4%～99.6% |
| 腐蚀后顶面观测跨度 | 约 21.4～24.3 mm |
| 顶面 Z 的 P10～P90 展宽 | 约 2.83～2.94 mm |
| 5 帧位置标准差 | XY 约 0.019/0.029 mm，Z 约 0.013 mm |
| 稳定冻结 | 通过 |

上述重复性只证明当前静止画面下的分割和深度稳定；由于在线 TF 是预览值，本次诊断产生的 `base_link` 坐标全部作废，不得用于机械臂运动。

## 结论与未关闭项

P3 的定位模块和失败门禁已经实现，当前蓝色方块能被唯一稳定识别。安全门也实际阻止了错误预览 TF 进入抓取位姿。

在正式手眼 TF 和 184.614459 mm TCP 均在线一致后，运行：

```bash
ros2 launch fairino3mt_v6_moveit2_config first_grasp_perception.launch.py
```

严格门禁下完成 8 帧冻结，未发送任何机械臂或夹爪命令：

| 指标 | 正式配置结果 |
| --- | --- |
| 冻结顶面中心 | `base_link [-0.0907, -0.3774, 0.0368] m` |
| 8 帧位置标准差 | X 0.057 mm、Y 0.067 mm、Z 0.015 mm |
| 内部有效深度比例 | 约 94.1%～97.4% |
| 腐蚀后顶面跨度 | 约 22.2～24.3 mm |
| 顶面 Z 的 P10～P90 展宽 | 约 1.78～1.86 mm |
| 手眼/TCP 一致性门禁 | 通过 |
| 稳定冻结 | 通过 |

尚未完成 P3 的绝对落点验收：需要完成 RViz 方块 Marker 对照、中心及周边至少 3 个位置的重复采样和独立 XY/Z 偏差测量。默认工作区范围也必须按实测桌面收紧后，才允许 P5 消费位姿。
