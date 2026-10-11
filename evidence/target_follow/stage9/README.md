# 第九步：位姿跟随的软件与模拟证据

2026-10-10：第九步完整位姿驱动、独立启动入口与可选旋转预测已实现、构建安装并通过软件/模拟验证。下列初版软件执行检查使用 MockClient，没有连接真实机械臂或相机，真实伺服命令数为 0。随后用户进行了三轮现场测试，第一轮已有运动、后两轮未开始；诊断与阈值修复见 [field_tuning_20261010](field_tuning_20261010/README.md)。后续实机记录与最新修正说明见下方各次现场调参。

操作见 [第九步说明](../../../docs/目标跟踪第九步位姿跟随操作-20261010.md)。初版汇总、测试数量、当时源码及报告哈希见 [validation_summary.json](validation_summary.json)；后续修复的验证单独保存在 `field_tuning_20261010/`。

随后现场平移已可跟随，倾斜触发跳变及 20.24°原始恢复角差超限；进一步将第九步观测抖动余量设为 25°、该次恢复角差设为 40°，实际运动范围仍为 20°。该次诊断、回放及 96 项软件/Mock 验证见 [tilt_tuning_20261010](tilt_tuning_20261010/README.md)。

最新会话 `20261010_140121_208977` 已发送 14049 条正常伺服，最后映射转角 20.36°触发旧 20°范围。当前扩大到 500 mm/90°、恢复 150 mm/120°，并新增完整期望位姿锁存与到位保持，容忍静止视觉/反馈噪声。最新原始诊断、构建与 101 项软件／Mock 检查见 [range_hold_tuning_20261010](range_hold_tuning_20261010/README.md)；此版真实效果待确认。

随后用户明确要求扩大区域至 1 米、速度不变，当前半径已改为 1.00 m。最新现场错误是 SDK 112（目标位姿无法到达），未触发原软件半径。诊断与此轮构建见 [radius_1m_20261010](radius_1m_20261010/README.md)。

## 已通过验证

| 证据 | 结果与范围 |
| --- | --- |
| `validation/build_release.log` | `fr3c_follow_driver`、`fr3c_target_follow` 构建安装成功 |
| `validation/cpp.log`、`validation/cpp_xml/` | 8 个 CTest 容器、36 项 C++ 用例通过；含 12 项新位姿/旋转碰撞/反馈用例及既有执行回归 |
| `validation/python.log`、`python.xml` | 40 项通过，1 项 GUI 用例因无显示环境跳过；含源检查、位姿几何与旧预览/窗口回归 |
| `validation/ros_driver.log`、`ros_driver.xml` | 8 项新位姿驱动 ROS 模拟通过 |
| `validation/ros_launch.log`、`ros_launch.xml` | 1 项完整启动入口 ROS 模拟通过 |
| `validation/ros_position.log`、`ros_position.xml` | 6 项原位置跟随及可选碰撞 ROS 回归通过 |

合计 91 项通过、1 项跳过。xacro 的 Python 3.11 私有变量提示为既有依赖警告。

`mock_integration/` 包含三轴完整相对关系与非零 TCP、主动停止后重新开始、短时姿态抖动/歧义、可信原图过期暂停、硬跳变/丢失恢复、越界和大幅重定位后 S 重新记录、非法/错配/未来观测拒绝、实际 Python 图像时刻换算与配对质量链、开启真实机构模型/FCL 的位姿跟随。

`launch_mock/` 用合成 eye-in-hand 输入与现有手眼标定启动实际入口，检查稳定 Demo 默认值、独占期望 TCP 发布、启动待机、T/S/X 和旋转跟随、退出报告及自有节点清理，确认外部源仍保留。相机与标签输入均为合成数据。

`regression_position/` 保存本轮位置回归，未覆盖原 stage4 证据。`invalid_input_range/report.json` 的 `fault` 是越界拒绝用例的预期结果。

纯 C++ 用例还覆盖 SO(3) 最短旋转与确定的 180°方向、速度/加速度与收敛、原始目标范围先于滤波检查、反馈冻结导致的参考超前、暂停基准保留、精确整数时间配对，以及纯旋转/参考超前/角制动造成的附件桌面扫掠和缓存过期。`test_pose_feedback` 注入重复包、SDK 读取失败与 NaN，实际运行驱动验证停止、等待新可信图像、原关系恢复及恢复零速度。

## 复现

下列执行均为隔离域的 Mock 测试，不启动真实相机或机器人。先在工作区构建安装：

```bash
cd /home/industio/fairino_ws
source scripts/camera/activate.bash
colcon build --packages-select fr3c_follow_driver fr3c_target_follow --symlink-install
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/evidence/target_follow/stage9/validation/replay_ros_logs"
ROS_DOMAIN_ID=196 ctest --test-dir build/fr3c_follow_driver --output-on-failure
```

Python 几何、源检查及预览回归：

```bash
python3 -m pytest -q \
  src/fr3c_target_follow/test/test_demo_graph.py \
  src/fr3c_target_follow/test/test_follow_geometry.py \
  src/fr3c_target_follow/test/test_orientation_source_plan.py \
  src/fr3c_target_follow/test/test_pose_follow_source_plan.py \
  src/fr3c_target_follow/test/test_pose_preview.py \
  src/fr3c_target_follow/test/test_viewer_window.py
```

新驱动和完整入口：

```bash
export FR3C_RUN_ROS_TESTS=1
ROS_DOMAIN_ID=197 python3 -m pytest -q src/fr3c_follow_driver/test/test_pose_follow_mock.py
ROS_DOMAIN_ID=198 python3 -m pytest -q src/fr3c_target_follow/test/test_pose_follow_launch_ros.py
```

旧位置跟随回归单独输出：

```bash
export FR3C_POSITION_TEST_OUTPUT_ROOT="$PWD/evidence/target_follow/stage9/regression_position"
ROS_DOMAIN_ID=199 python3 -m pytest -q \
  src/fr3c_follow_driver/test/test_follow_mock.py \
  src/fr3c_follow_driver/test/test_collision_mock.py
```

模拟证明流程与几何可运行，不证明视觉绝对精度或现场稳定性。此前约 40 mm 跨视角基坐标差异仍未解释；首轮按操作说明观察固定方块是否明显漂移或引起持续追赶，允许残差与延迟，无需精密误差验收。
