# P4：30 mm 方块夹持深度与持续保持验证

日期：2026-09-11  
对象：FR3-C、YDI-J22 四指柔性夹爪、30×30×30 mm 蓝色方块。  
范围：只关闭首次 RGB 抓取所需的 P4 夹持门禁，不替代 P09 的断连、失电、100 次循环和自动夹持反馈验收。

## 结论

操作者在方块由桌面支撑、机械臂不提升的条件下完成现场夹持测试，并确认下列组合能够稳定夹住方块：

| 参数 | 冻结值 |
| --- | ---: |
| 预张开净开度 | 60 mm |
| 抓取深度 `d` | 顶面以下 15 mm |
| 闭合净开度命令 | 26 mm |
| 映射脉宽 | 836 µs |
| 硬件有效脉宽 | 840 µs（100 kHz 输出的 10 µs 分辨率） |
| 持续保持 | 超过 30 s |
| 夹持确认 | 人工现场确认 |
| 自动位置、力或夹持反馈 | 无 |

P4 按“首次抓取门禁”通过。后续 P5 必须继续把 action 成功与人工夹持确认区分开，不得把 `commanded_position_estimated` 当作传感器确认。

## 几何核对

第一次冻结的方块顶面为：

```text
base_link [-0.081295936, -0.360040586, 0.036089889] m
```

相邻桌面处用最低指尖接触并读取 `base_link -> grasp_frame`，Z 约为 0.006 m。两者之差约 30.09 mm，与 30 mm 方块一致，说明定位节点输出的是 `base_link` 下的顶面绝对高度，不能把桌面错误假定为 `z=0`。

重新摆放后的正式冻结顶面为：

```text
base_link [-0.111081809, -0.344718258, 0.035460172] m
```

采用 `d=0.015 m` 后，抓取 TCP 目标为：

```text
base_link [-0.111081809, -0.344718258, 0.020460172] m
```

按桌面 Z 约 0.006 m 估算，名义最低点桌面净空约 14.46 mm。操作者先到顶面以下约 10 mm 的检查点，再分段下降到 15 mm，并确认没有碰桌。

需要特别保留的动作定义：接近位为 `z_top + 15 mm`，抓取位为 `z_top - 15 mm`，所以 P5 从接近位到抓取位的竖直下降总量是 30 mm，而不是 15 mm。

## 在线驱动证据

成功会话中在线 `/diagnostics` 连续读回：

```text
backend: ft232h
armed: true
feedback_mode: open_loop_estimated
position_source: commanded_open_loop_estimate
last_result: commanded_position_estimated
estimated_position_m: 0.026000
last_target_m: 0.026000
current_pulse_us: 840
actual_sample_rate_hz: 100000.0
frames_written: 6962 -> 7002（持续增长）
```

帧计数以约 50 Hz 增长；采样时已经输出约 6962 帧，即约 139 s，覆盖本次不少于 30 s 的保持要求。操作者同时确认实物方块保持稳定。

## 故障边界

成功试验前曾发生一次 FT232H USB 重新枚举。旧会话记录到：

```text
FtdiError: UsbError: [Errno 5] Input/Output Error
```

驱动锁存故障并将 26 mm action 置为 `ABORTED`，夹爪未执行该次闭合。重启组合栈、确认 FT232H 重新在线后才进行了上述成功试验。失败会话不计入P4成功结果，但证明了当前 action 不会把通信错误伪报为成功。

本次意外重新枚举的根因尚未作为长期可靠性问题关闭。进入无人值守或重复抓取前，P09 仍需验证USB线缆/Hub稳定性、断连处理、失电行为、温升和循环寿命。

## P5 固定输入

P5 首版应使用：

```text
pre_open_position_m = 0.060
approach_clearance_m = 0.015
grasp_depth_m = 0.015
close_position_m = 0.026
lift_distance_m = 0.150
hold_confirmation = manual
```

尚未由P4冻结的是精确朝下四元数和绕基座Z轴的 yaw；P5 必须在仅规划及高净空检查阶段明确它们。夹持期间保持夹爪节点、FT232H和独立5 V电源运行；试验收尾时先把方块放回支撑面，再命令张开到60 mm。
