# 第九步：按用户要求将位姿跟随区域改为 1 米

用户明确要求扩大区域、保持速度参数。将配置、控制核心和驱动默认值的 `follow_radius_m` 从 0.50 改为 **1.00 m**。这是每次 S 开始时实际 TCP 周围的三维半径。90°姿态范围、20 mm/s／3°/s、加速度、半幅映射、静止保持与已有停止处理保持原值；没有新增关节速度限制。

最新现场会话 `../20261010_142209_226469/` 的最终错误为 `ServoCart failed: 112`，3397 条调用成功，最后 1 条返回 112。目标暂停和恢复均为 0。停止、结束和关闭返回 0。原始记录不改写，本次只读诊断没有新的真实 SDK 连接或运动。

本地原厂 SDK [robot_error.h](../../../../src/fairino_hardware/libfairino/include/robot_error.h) 定义 112 为 `ERR_TARGET_POSE_CANNOT_REACHED`（目标位姿无法到达），不是软件半径超限。TCP 指令速度未超过 20 mm/s／3°/s；第三关节接近 0°时，按新反馈包的主机观测时间估算，关节速度峰值约 89.47°/s。低 TCP 速度与高速关节运动可以同时出现，记录支持接近运动学边界的判断，但未据此声称取得控制器内部完整诊断。详情与原始哈希见 [diagnosis.json](diagnosis.json)。

扩大软件半径不能改变真实机械臂可达空间。112 仍由已有错误停止流程处理，本次按用户最新要求完成区域修改。

执行端构建安装通过，41 项 C++ 和 1 项完整入口 Mock 共 42 项检查通过；入口会话实际加载 `follow_radius_m=1.00`，所有线／角速度和加速度与现场旧配置相同。构建与验证保存于 `validation/`，汇总见 [validation_summary.json](validation_summary.json)。执行测试均为 MockClient 与合成视觉，无新硬件指令。

退出旧入口后重新运行原命令，再按 S，才会加载本轮参数：

```bash
/home/industio/fairino_ws/scripts/target_follow/start_pose_follow.sh
```
