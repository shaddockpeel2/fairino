# P1 执行错误传播与周期：代码及离线验证记录

日期：2026-09-10

## 结论

P1 的代码修复和离线故障注入已通过；真机低速短轨迹、取消和实际周期日志尚未执行，因此当前结论是“代码/离线通过，真机门禁待验”，不是完整真机通过。

## 版本与周期核对

- 本地 SDK：`src/fairino_hardware/libfairino/include/robot.h`。该头文件明确给出 `ServoMoveStart()`、`ServoJ()`、`ServoMoveEnd()` 配对生命周期；`ServoJ::cmdT` 注释的建议范围是 0.001～0.0016 s。
- 法奥官方 `V3.0.0_RobotV3.9.2` 标签的 `fairino_hardware_interface.cpp` 实际将 `ServoJ::cmdT` 固定为 0.008 s，而同标签全部 MoveIt2 控制器配置为 100 Hz（0.010 s）。本地源码与该标签一致，原状态确实存在 8 ms/10 ms 不一致。
- 当前仍使用该版本官方 XML-RPC ServoJ 路径，不冒进切换到未经本机验证的高频 UDP 路径。保留官方插件实际使用的 `cmdT=0.008 s`，把 `controller_manager.update_rate` 改为 125 Hz，使名义控制周期一致为 8 ms。
- 插件首次写周期会比较 ros2_control 传入的实测 `period` 与配置的 `cmdT`；偏差超过 2 ms 会输出 WARN，否则输出匹配记录。真机验收时必须保存该日志。这个决策解决了原始的名义周期矛盾，但不能替代真机对 V3.9.2 控制器节拍的验证。

官方版本对照：<https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/V3.0.0_RobotV3.9.2>

## 已实现行为

1. 位置和预留扭矩命令均用完整六元素容器检查有限值，第六轴 NaN/Inf 会在进入 SDK 前被拒绝。
2. 激活顺序为 `RPC → 读取六轴 → 同步 state/command → ServoMoveStart`；`ServoMoveStart` 非零会使激活失败。
3. `ServoJ` 任意非零返回码会立即：锁存写失败、执行 `ServoMoveEnd → StopMotion → CloseRPC`、向 ros2_control 返回 `ERROR`。
4. 错误锁存后，即使控制循环再次调用 `write()`，也不会再次调用 `ServoJ`。
5. `deactivate`、`shutdown` 和 `on_error` 统一执行幂等清理；原来的 `unique_ptr.release()` 已替换为正常释放。
6. SDK 客户端边界可替换，离线测试可注入关键 SDK 返回码，不连接机器人。

## 离线验证

执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select fairino_hardware fairino3mt_v6_moveit2_config --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
build/fairino_hardware/test_fairino_hardware_interface
```

结果：6 个测试全部通过。

- 完整六轴有限值检查：通过；第六轴 NaN/Inf 均返回 false。
- 生命周期与 8 ms 参数：通过；调用顺序为 `RPC → read → ServoMoveStart → ServoJ → ServoMoveEnd → StopMotion → CloseRPC`。
- 第六轴 NaN：通过；`write()` 返回 `ERROR`，`ServoJ` 调用次数为 0。
- `ServoJ=14`：通过；首拍 `write()` 返回 `ERROR`，第二拍仍返回 `ERROR`，`ServoJ` 总调用次数保持为 1。
- ros2_control 传播：通过；真实 `ResourceManager::write()` 报告 `FairinoTestSystem` 失败，六个 `jointN/position` 命令接口全部变为 unavailable，并触发硬件 `on_error` 清理。
- `ServoMoveStart=14`：通过；激活返回 `ERROR`，并执行停止和断连。

构建同时通过 xacro 展开、XML 解析以及本次新增/修改 C++ 文件的 uncrustify 检查。全包 `ament_lint_auto` 仍会报告法奥原包中命令服务器、SDK 头文件和示例的既有格式问题；这些与本次 P1 功能测试无关，未做大范围重排。

## 上层阻断边界

离线测试已证明 ros2_control 收到硬件失败、撤销六轴命令接口，且硬件层不再下发任何运动命令。当前 `fairino_mtc_demo` 对 MoveIt/MTC 执行返回值已有失败即 `break` 的处理；但它仍是固定圆柱的一体化示例，不是真实“三段运动 + 夹爪”任务，因此尚不能声称已完成真实任务层的下降、闭爪、提升逐段故障注入。该项要在 P5 分段任务形成后复用本门禁做集成测试。

## 真机待执行门禁

条件：人工清场、无物体、低速、短轨迹、急停可用、仅启动 MoveIt/ros2_control 一个运动源。

1. 启动后确认存在 `ServoJ mode started with cmdT=0.008000 s`，并保存实测 period 匹配日志。
2. 执行一条低速小位移关节轨迹，确认全程无 SDK 非零返回码和明显控制周期超时。
3. 中途取消另一条短轨迹，确认 action 返回取消、机器人停止、控制器状态可解释。
4. 只在无物体条件下注入可恢复故障；确认 action 非成功、后续任务阶段未启动，重新执行完整激活流程后才能继续。
