# FR3C 跟随链路的只读反馈

独立 C++ ROS 节点，链接项目固定的 ARM64 FAIRINO SDK 2.3.2。仅调用 `RPC`、`GetRobotRealTimeState`、`CloseRPC`，没有运动、使能或模式切换接口。

从新的状态包读取 `flange_cur_pos`，将毫米/固定轴角度转换为米/四元数，在 `/target_follow/flange_pose` 和 `base_link → flange_link` TF 发布。新包通过序号和控制器时钟字段识别，重复缓存包不发布新位姿，也不更新其采样时间。

默认以 125 Hz 轮询 SDK 缓存。实机新包发布约 118 Hz；这不是原厂伺服命令发送能力的测量。状态和调用耗时发布在 `/target_follow/robot_feedback`。

使用主机首次观察到新反馈包的时间作为 ROS 时间戳，控制器原始时钟仅作诊断。本机已观察到控制器时间比主机快约 5 分半，未修改控制器时钟。

```bash
source /home/industio/fairino_ws/scripts/camera/activate.bash
colcon build --packages-select fr3c_follow_feedback --symlink-install
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ros2 run fr3c_follow_feedback robot_feedback
```

优先使用 `scripts/target_follow/start_preview.sh`，它会复用已有的反馈话题。与另一个 SDK 控制客户端共同运行前需由执行端明确 SDK 连接所有权；目前独立预览没有启动控制客户端。

三项 C++ 测试覆盖单位/旋转约定、非有限位姿拒绝、缓存包新鲜度与序号回绕。测试本身不连接机器人。
