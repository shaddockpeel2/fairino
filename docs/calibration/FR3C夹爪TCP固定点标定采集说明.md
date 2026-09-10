# FR3-C YDI-J22 夹爪 TCP 固定点标定与复现步骤

日期：2026-09-10

## 1. 标定对象与方法

TCP 定义为 YDI-J22 四根柔性指最终闭合时的共同尖端，坐标关系为
`flange_link -> grasp_frame`。本次采用固定尖点（pivot）法：让 TCP 在多个法兰姿态下接触
同一个固定基准点，并读取控制器 `GetActualToolFlangePose(0)`。每组观测满足：

```text
base_p_flange(i) + base_R_flange(i) * flange_p_tcp = base_p_fixed
```

将全部姿态联合最小二乘即可求出 `flange_p_tcp` 和固定点位置。该方法只观测 TCP 的 XYZ，
不能确定工具坐标轴方向；本项目夹爪按法兰同轴安装，因此 RPY 继续定义为 `[0,0,0]`。

## 2. 2026-09-10 求解复核与最终采用值

数据目录：`evidence/P10/tcp_pivot_20260910`

| 项目 | 结果 |
| --- | --- |
| 采集姿态 | 12 组 |
| 有效姿态 | 11 组；第 11 组固定点残差 3.410 mm，作为离群点排除 |
| 姿态跨度 | 163.170° |
| 方程条件数 | 5.609，激励充分 |
| 有效样本固定点 RMS | 1.181 mm |
| 有效样本最大残差 | 1.913 mm |
| 留一法 TCP 最大变化 | 0.748 mm |
| 数学候选 TCP XYZ | `[-0.847397, 3.027928, 184.614459] mm` |
| 候选 TCP 长度 | 184.641 mm |
| TCP RPY | `[0,0,0] deg`，来源为机械同轴定义 |
| 现场直接测量 | 法兰盘到夹爪尖端 Z 方向距离 178 mm |
| 最终采用 TCP | `[-0.847397, 3.027928, 178.000000] mm` |
| 最终 TCP 三维长度 | 178.028 mm |

旧名义值 `[0,0,163] mm` 来自厂商 160 mm 尺寸和假设垫片 3 mm。厂商现已说明该夹爪的
160 mm 尺寸有问题，现场直接测得法兰盘到尖端 Z 方向距离为 178 mm，因此最终不使用
160 mm 名义尺寸，也不采用固定点全量求解的 Z=184.614 mm。最终值采用混合来源：X/Y
保留固定点求解结果，Z 使用现场直接测量 178 mm，RPY 使用机械同轴定义。相对旧名义值
变化 `[-0.847397,+3.027928,+15.000000] mm`，三维差值 15.326 mm。

用保存关节角重新调用控制器 `GetForwardKin`，与采集法兰位姿最大字段差约 0.0013；用
URDF 独立 FK 核对控制器，最大差约 0.011 mm / 0.00042°，两条路径均重现 184.62 mm。
因此控制器状态、FK 和 RPY 约定没有发现问题；固定点接触几何对 Z 产生了系统偏差。该批
数据仍用于估计 X/Y，Z 则以更直接的实物测量覆盖。最终值面向普通抓取，后续通过实际抓放
做毫米级微调。

## 3. 标定前准备

1. 准备刚性固定、不会移动的尖锐锥尖或小锥孔，保证各姿态接触的是同一个物理点。不要使用球、圆柱侧面、大圆头螺钉，也不要让四指包住基准后对准其中心；这些做法会把基准半径加入 TCP 长度。
2. 夹爪在全部姿态保持相同的最终闭合状态，不中途改变 PWM、开度或指尖装配。
3. 只允许轻触基准，不压弯柔性指；基准被碰动后必须新建会话重新采集。
4. 保持低速、有人监护、急停可达，并检查相机支架和线缆在所有倾斜姿态下有余量。
5. 确认控制器地址为 `192.168.58.2`，工作区已构建：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_calibration --symlink-install
source install/setup.bash
```

## 4. 新建与采集会话

为每次重新安装后的标定使用全新目录，例如：

```text
/home/industio/fairino_ws/evidence/P10/tcp_pivot_YYYYMMDD_HHMM
```

每个姿态按以下顺序执行：

1. 低速移动机械臂，使闭合共同尖端轻触同一基准点。
2. 松开操作界面，等待机械臂完全停稳。
3. 执行一次采集命令：

```bash
/home/industio/fairino_ws/scripts/calibration/capture_tcp_pivot_sample.sh \
  /home/industio/fairino_ws/evidence/P10/tcp_pivot_YYYYMMDD_HHMM
```

4. 只有输出 `"captured": true` 才移动到下一姿态。采集器会在约 1.1 秒内连续读取
   15 组法兰位姿和六轴角；未停稳的数据不会写入。
5. 若输出 `robot_not_stationary`，保持当前姿态不变，等待后重采。不要为凑数量保留失败组。

建议采集 12 组，最少 8 组。应包含中性姿态、工具 X/Y 方向约 ±15°、±25° 以及若干
X/Y 组合倾斜。不要只转第 6 轴；两两最大姿态跨度至少应达到 25°。

查询已采数量：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fr3c_calibration tcp_pivot_capture \
  --session /home/industio/fairino_ws/evidence/P10/tcp_pivot_YYYYMMDD_HHMM --status
```

## 5. 求解、异常检查与采用条件

执行：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fr3c_calibration tcp_pivot_analyze \
  /home/industio/fairino_ws/evidence/P10/tcp_pivot_YYYYMMDD_HHMM
```

默认采用条件如下：

| 检查项 | 门限 |
| --- | --- |
| 样本数 | 至少 8 |
| 有效样本 | 至少 6 且不少于总数 75% |
| 方程秩 | 6 |
| 姿态跨度 | 至少 25° |
| 条件数 | 不大于 100 |
| 有效样本固定点 RMS | 不大于 1.5 mm |
| 有效样本最大残差 | 不大于 2.0 mm |
| 留一法 TCP 最大变化 | 不大于 1.0 mm |
| TCP 三维长度 | 当前装配不大于 180 mm，并须与 178 mm 现场直测一致 |

相对旧名义值超过 10 mm 会在报告中提示人工复核。当前工具的自动上限设为 180 mm；超过
上限即使残差和留一法稳定性很好也不得直接部署。

会话目录产物：

```text
tcp_samples.json                 # 全部原始读数与单组停稳质量
tcp_solve_report.json            # 求解结果、离群点、门限和逐组残差
ydi_j22_tcp_v2.yaml              # 通过门限的结果
ydi_j22_tcp_v2.candidate.yaml    # 未通过门限时的诊断候选
final_tcp_decision.yaml          # 人工测量修正后的最终采用值与来源
```

## 6. 部署步骤

1. 将通过复核的 XYZ 写入
   `src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml`。
2. URDF 当前为保持碰撞模型净空仍保留 3 mm 历史占位，因此本次
   `gripper_tcp_from_body_xyz=[-0.000847397,0.003027928,0.175] m`；与占位相加后完整法兰
   TCP 为 `[-0.000847397,0.003027928,0.178] m`。保持 `gripper_tcp_rpy="0 0 0"`。
3. 重新构建 `fairino_description`、`fairino3mt_v6_moveit2_config` 和使用该 TCP 的任务包。
4. 完全停止并重新启动 MoveIt/RViz，核对 `flange_link -> grasp_frame` 的 TF 与 YAML 一致。
5. 在控制器中选择一个明确的专用 tool ID，写入
   `[-0.847397,3.027928,178.0,0,0,0]`，不得覆盖来源不明的现用工具。
6. 使用 `GetTCPOffset` 或按 tool ID 读取接口回读，核对 ID、XYZ 和 RPY。
7. 在固定基准点附近以多个姿态重新接近，确认 TCP 围绕同一点旋转时没有明显系统偏移。
8. 完成低速空跑和 20 次固定点抓放；至少 19 次成功后，才将 P10 固定抓放验收标记完成。

ROS/MoveIt 已更新为最终实用 TCP v2。控制器专用 tool ID、负载重量和质心目前仍未写入；
TCP 标定也不能替代负载标定。

## 7. 本次采集说明

首次尝试曾因采集器把控制器毫米值再次乘以 1000，误报法兰平移波动 3.99 mm。原始关节
和尖端波动实际约为 0.004 mm，该次失败没有写入样本。单位问题修正并加入回归检查后，
正式采集的 12 组全部通过单姿态停稳门限。

该 12 组会话保留原始全量求解证据；`tcp_solve_report.json` 和
`ydi_j22_tcp_v2.candidate.yaml` 记录未经 Z 修正的 184.614 mm 数学候选。正式混合结果以
`src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml` 为准。若以后重新执行完整固定点
标定，必须使用新的会话目录，不能向这 12 组后面追加数据。
