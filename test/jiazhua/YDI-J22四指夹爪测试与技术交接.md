# YDI-J22 四指柔性夹爪测试与技术交接

## 当前冻结方案（2026-09-17）

- 控制链：RK3588 → USB → FT232H AD7 → YDI-J22 三线舵机。
- 供电：夹爪独立稳压 5 V，建议持续能力至少 2 A；与 FT232H 共地。
- 信号：50 Hz，500–2500 us 对应 0–180°，使用 PyFtdi asynchronous
  bit-bang，不向 `/dev/ttyUSB*` 发送 UART 动作字节。
- 生产业务只允许两态：`0.026 m` 抓取位 = 台架 `31°`；
  `0.060 m` 张开/释放位 = 台架 `70°`。
- 台架脚本和 ROS backend 都使用 10 kHz 采样率。31°的标称脉宽为
  844.4 us，实际表示为 8 个高样本 = 800 us；70°标称 1277.8 us，
  实际为 13 个高样本 = 1300 us。

## 替换旧实现的原因

旧 ROS backend 使用后台线程持续分块写入 PWM，现场出现过旧波形延后执行和夹爪
来回抽动。现实现封装了效果较好的 `gripper_ft232h.py` 方式：每条命令只打开一次
FT232H，一次性写入 0.8 s / 40 帧 / 8000 字节，等待波形发送完成，再将 AD7 拉低
并关闭。不再存在后台补充队列、扫角或循环往返。

## 软件位置

- `test/jiazhua/gripper_ft232h.py`：独立台架入口，只接受 31°/70°。
- `src/fr3c_gripper_driver/fr3c_gripper_driver/ft232h_servo.py`：有限波形封装。
- `src/fr3c_gripper_driver/fr3c_gripper_driver/ft232h_backend.py`：ROS 后端适配。
- `src/fr3c_gripper_driver/fr3c_gripper_driver/node.py`：`GripperCommand` 两态门禁。
- `src/fr3c_gripper_driver/config/ydi_j22_ft232h.yaml`：工作参数配置。

## 台架复测

以下命令会产生真实动作，必须独立 5 V 供电、共地并现场监护：

```bash
cd /home/industio/fairino_ws/test/jiazhua
.venv/bin/python gripper_ft232h.py --angles 31
.venv/bin/python gripper_ft232h.py --angles 70
```

两条命令应分开执行，先确认 31°抓取位稳定，再执行 70°释放。当前软件验证不会自动
触发这两条硬件命令。

## 上层接口与限制

ROS action 仍为 `/ydi_j22_gripper_controller/gripper_cmd`，MoveIt/MTC 保持标准
`GripperCommand` 封装。`close` 映射 0.026 m，`open` 映射 0.060 m，其他位置
目标全部拒绝。URDF、SRDF、MoveIt joint limits 和 MTC 配置已同步。

硬件无位置、力、电流、温度或堵转反馈，所以 action 成功只表示有限 PWM 命令已完成，
不等于确认抓取成功。若需长时间保力或可验证夹持力，应增加硬件定时 PWM/MCU 与电流
或力反馈，上层 action 接口可保持不变。
