# P2 夹爪模型与 MoveIt 状态链验证记录

日期：2026-09-11

## 结论

P2 的碰撞模型拆分已经加载，但首次实现漏掉了真实 FT232H 后端启动时的夹爪关节状态。`move_group` 因此每秒报告：

```text
complete state not known. Missing gripper_command_joint
```

根因不是法奥机械臂状态或 MoveIt 配置：`fr3c_gripper_driver` 原先只在 mock 后端初始化 `startup_position_m`，真实后端必须等到第一条夹爪命令后才发布 `/joint_states`。

现已让 mock 和 FT232H 后端都在启动后发布 `gripper_command_joint`。FT232H 没有位置反馈，因此该启动值被明确标记为 `startup_parameter_unverified`；默认 0.130 m 代表全开估计。发布状态不会打开 FTDI、不会输出 PWM。实际利用夹爪开口规划抓取前，仍须明确发送一次预张开命令。

## 验证

- `python3 -m pytest -q src/fr3c_gripper_driver/test`：7 passed。
- `colcon build --packages-select fr3c_gripper_driver --symlink-install`：通过。
- FT232H 后端且硬件输出关闭时，`/joint_states` 收到 `gripper_command_joint: 0.13`，后端未产生 PWM。
- 运行 `scripts/run_moveit_camera_rviz.sh`：启动初期各状态源汇合时出现一次瞬态缺失提示，随后不再周期性重复；MoveIt 输出 `You can start planning now!`。

## 尚未覆盖

- 真实桌面和 30 mm 方块碰撞体。
- 接近、下降、目标接触许可、附着方块及携物提升的现场规划验证。
- `Ctrl+C` 时 RViz/move_group 的退出段错误属于独立的关闭顺序问题，不是本次关节状态缺失，也不影响启动后的规划就绪结论。

## 相机侧向修正

2026-09-11 用户对照 RViz 与实物确认相机落在法兰盘对侧。旧的照片估计将相机中心暂放在法兰 `+Y=57.5 mm`；已修正为 `-Y=57.5 mm`。这是安装位置符号修正，未给相机自身增加无依据的 180°旋转。

同日截取运行中的 RViz 并审计实时点云：Fixed Frame 为 `base_link`，主平面占有效点 95.0%，法向与基座 Z 轴夹角 80.65°，确认其被错误显示成竖直面。根因为预览 `camera_housing_link -> camera_link` 使用单位旋转，与驱动固有的约 `roll=-90°、yaw=-90°` 光学轴变换叠加后令光轴沿外壳 X。预览 TF 已改用逆向名义光学轴四元数 `[0.5,-0.5,0.5,0.5]`（xyzw），使点云光学 Z 对齐外壳/工具 Z；正式抓取仍以手眼标定外参为准。
