# P6：首次 RGB 抓取运行配置与现场门禁开发记录

日期：2026-09-11  
范围：只读核对法奥控制器、ros2_control、ROS TF 和单一运动源，并为 P5 单次低速实物执行签发门禁。

## 结论

P6 自动预检链已经实现并在当前真机栈上运行。操作者在 2026-09-11 明确确认工作区、150 mm 净空、
线缆、断电/停止方式和单一运动源满足本次验证，并决定原型阶段暂不进行末端负载/质心标定。
配置因此启用有记录的 prototype waiver：报告继续保存控制器实时 `0 kg`、`[0,0,0] mm`，但不把它
作为本次 5% 低速单循环的阻塞项。其余自动检查全部通过，P6 当前结果为 **通过**。

## 实现

- `fairino_hardware` 使用其已经持有的唯一法奥 SDK 连接读取 `ROBOT_STATE_PKG`，通过 ros2_control
  只读 `fairino_status/*` 状态接口发布。没有启动第二个 SDK/运动服务进程。
- 状态包含程序/机器人状态、主副故障、模式、工具/工件号、急停、安全停止、碰撞、使能、负载/质心、
  工具/工件坐标和控制器法兰位姿。
- `p6_preflight` 同时核对控制器状态、执行控制器占用、六轴命令接口、轨迹桥状态、关节状态发布源、
  ROS FK 对法奥法兰、184.614459 mm TCP、验收手眼 TF、必需/禁用节点和现场确认项。
- 每次运行保存完整 JSON 报告，并原子覆盖 `evidence/P6/p6_execution_gate.txt`；任何失败都会写
  `passed=false`。
- P5 在 `start`、`descend_and_close`、`confirm_grip_and_lift` 及实际 approach 前检查 gate 的签名、
  结果、控制路径和 600 s 新鲜度。
- 修复了 P5 launch 中 `execute:=true` 被 YAML 内 `execute:false` 抢先覆盖的问题；执行模式现在由 launch
  参数唯一控制，默认仍为 false。

## 当前真机结果

首次执行前报告：[`report.json`](../runs/1789108188154535168/report.json)  
执行后复核报告：[`report.json`](../runs/1789108493793796096/report.json)

| 项目 | 结果 |
| --- | --- |
| 控制路径 | `moveit_joint_servoj` |
| 法奥程序/机器人状态 | 均为 1（停止） |
| 主/副故障 | `[0,0]` |
| 急停/两路安全停/碰撞 | 均为 0 |
| 使能/运动到位 | 均为 1 |
| 控制器工具/工件 | ID 0 / 0，坐标均为零；仅记录，不参与关节 ServoJ 坐标变换 |
| 控制器负载/质心 | `0 kg` / `[0,0,0] mm`；已记录原型豁免，不阻塞本次 5% 低速单循环 |
| ros2_control | 硬件 active；执行控制器 inactive；六轴接口 available/unclaimed |
| 轨迹桥 | `monitoring_manual_ready` |
| 关节状态源 | 仅 `joint_state_broadcaster` 发布六轴；夹爪驱动只发布夹爪关节 |
| ROS FK 对法奥法兰 | 平移 0.010110 mm，旋转 0.000218° |
| `flange_link -> grasp_frame` | 与 TCP YAML 完全一致，0 mm / 0° |
| `flange_link -> camera_link` | 与验收手眼 YAML 完全一致且来源唯一，0 mm / 0° |
| 必需/禁用节点 | 必需节点均唯一存在；未发现通用法奥命令服务器或旧 MTC 运动节点 |
| 五项现场确认 | 操作者明确确认，通过 |

P3 同时重新冻结当前蓝色方块顶面约 `[-0.0916,-0.3567,0.0349] m`；这只证明当前感知链有效，
不替代 P3 三位置绝对落点测量。

## 失败门禁实测

在失败 gate 保持有效时启动：

```bash
ros2 launch fairino_mtc_demo first_grasp.launch.py execute:=true
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'
```

节点参数读回 `execute=True`，服务返回：

```text
success=False
Start refused: P6 gate reports failed; inspect .../1789106952064467968/report.json
```

拒绝后 `fairino3mt_execution_controller` 仍为 `inactive`，未生成新的 P5 run，未发送夹爪 action 或机械臂轨迹。

## 后续精度/速度提升前事项

1. 实测并记录相机、支架、夹爪、转接和随动线缆在法兰后的总质量与法兰坐标系质心；
2. 通过法奥 WebAPP/官方负载接口应用该值，再由本工具实时读回；不得用产品目录单件重量冒充总成实测值；
3. 将同一实测值和依据写入 `config/p6_preflight.yaml`；
4. 将 `payload.required` 恢复为 `true` 后重跑同一预检。

```bash
ros2 run fairino_mtc_demo p6_preflight \
  --config /home/industio/fairino_ws/src/fairino_mtc_demo/config/p6_preflight.yaml \
  --evidence-root /home/industio/fairino_ws/evidence/P6 \
  --confirm-workcell-clear \
  --confirm-lift-clearance-150mm \
  --confirm-cable-slack \
  --confirm-stop-method-ready \
  --confirm-no-external-motion-source
```

本轮原型执行使用的门禁已经输出 `passed: true`；上述事项保留给精度、动力学和更高速度验证。
