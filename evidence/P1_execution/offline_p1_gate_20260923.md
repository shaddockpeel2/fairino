# P1 Servo 专用执行后端：门禁记录

> 2026-09-23 复核补充：以下原始试验与未通过判定保留。后续 FK 复算发现目标领先反馈
> 约 0.0993 mm，与 5 mm/s × 20 ms 相符，不支持重复缩放解释；“下一步必须转
> ServoCart/UDP”应改为先隔离连续参考跟随能力。详见
> [欠速复核与修复路线](../../docs/FR3C_P1欠速复核与修复路线-20260923.md)。尚未完成真机修复。

日期：2026-09-23  
工作区：`/home/industio/fairino_ws`  
范围：离线构建/测试、`mock_components/GenericSystem`，以及一次由现场操作者
执行的 `0.005 m/s × 0.5 s` forward-position 真机短脉冲。

## 构建

```text
colcon build --symlink-install --packages-select \
  fairino_hardware fairino3mt_v6_moveit2_config fr3c_visual_servo

Summary: 3 packages finished
```

## 自动测试

```text
fr3c_visual_servo: 46 tests, 0 errors, 0 failures, 0 skipped
fairino_hardware gtest: 12 tests passed
P1/诊断定向 pytest: 31 passed
ament_flake8（8 个本次相关 Python 文件）: No problems found
git diff --check（本次相关文件）: passed
```

全量 `fairino_hardware` CTest 仍有一个既存红项：厂商目录中的 10 个未触及源文件不符合
`uncrustify`。测试输出同时确认本次涉及的
`fairino_hardware_interface.hpp/.cpp` 和
`test_fairino_hardware_interface.cpp` 无格式差异。未对厂商源文件做无关的批量格式化。

原始测试结果：

- `build/fr3c_visual_servo/pytest.xml`
- `build/fairino_hardware/test_results/fairino_hardware/test_fairino_hardware_interface.gtest.xml`
- `build/fairino_hardware/test_results/fairino_hardware/uncrustify.xunit.xml`

## controller_manager mock 门禁

使用 `ROS_DOMAIN_ID=97`，将机器人硬件插件替换为
`mock_components/GenericSystem`。没有连接 FAIRINO 控制器。

初始加载状态：

```text
joint_state_broadcaster              active
fairino3mt_execution_controller      inactive
fairino3mt_servo_position_controller inactive
```

依次执行严格切换后的状态：

```text
# 激活 JTC
joint_state_broadcaster              active
fairino3mt_execution_controller      active
fairino3mt_servo_position_controller inactive

# 同一 switch 请求停 JTC、启 forward position
joint_state_broadcaster              active
fairino3mt_execution_controller      inactive
fairino3mt_servo_position_controller active

# 释放 Servo 控制
joint_state_broadcaster              active
fairino3mt_execution_controller      inactive
fairino3mt_servo_position_controller inactive
```

三个 switch 请求均返回 `Successfully switched controllers`。mock controller manager 随后经
SIGINT 正常停用控制器并关闭 GenericSystem。

## 真机 0.5 s x+ 短脉冲

原始证据：
[`forward_020_xplus_0p5s.json`](forward_020_xplus_0p5s.json)。

| 指标 | 结果 | 判定 |
|---|---:|---|
| 后端 / Servo 周期 | forward position / 20 ms | 目标配置 |
| 名义位移 | 2.500 mm | — |
| 指令阶段沿 x+ 位移 | 0.026 mm | 实现率 1.04% |
| 释放后最终沿 x+ 位移 | 0.155 mm | 名义位移的 6.20% |
| 零命令后 0.5 s 行程 | 0.131 mm | 通过 `<=0.3 mm` |
| ServoJ interval P99 / max | 11.676 / 14.514 ms | 通过 `<=12 / <20 ms` |
| Servo 状态 | 仅 0 | 无硬故障 |
| forward 命令数 | 25 | 后端输出已观测 |
| 释放后状态 | `monitoring_manual_ready` | 通过 |
| 操作者观察 | “基本看不到明显机械臂变化” | 位移过小，无法完成平滑性验收 |

证据中 forward 位置目标在运动窗内的六关节合成变化仅约
`9.21e-4 rad`，同期反馈变化约 `7.66e-4 rad`。这说明硬件确实跟随了
forward controller 的小目标；主要欠速仍是 MoveIt Servo 按“实时状态 + 单周期增量”
生成目标与 ServoJ 跟随动态的组合结果，不是 JTC 插值导致。与旧
`T=0.02 s` JTC 证据约 5% 的结果基本相同。

## 当前结论

P1 的代码、离线门禁和首个真机短脉冲已完成，但 forward-position 后端
**没有消除欠速，P1 真机性能门禁未通过**。按 V2 交接规则，停止 2.0 s、
其他方向、0.010 m/s 和 P2 真机开发；不用后续阶段掩盖本问题。下一技术分支应为
FAIRINO ServoCart/UDP 受控 A/B 或取得厂商对 ServoJ 推荐流式用法的正式确认。
