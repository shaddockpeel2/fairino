# YDI-J22 四指柔性夹爪测试与技术交接

## 1. 当前结论

截至 2026-09-10，YDI-J22 四指柔性夹爪已通过 FT232H 在 AD7 引脚成功完成 PWM
动作测试，并已增加 ROS 2 标准 action 与 MoveIt 控制器适配；真实硬件 action 尚待本轮现场验收。

- 控制类型：三线 PWM 舵机，不是向 `/dev/ttyUSB0` 发送 UART 字节。
- 电源：独立稳压 5 V；建议持续供电能力至少 2 A。
- 接线：棕线接 GND，红线接独立 +5 V，黄线接 FT232H AD7；电源与 FT232H 共地。
- USB 芯片：FTDI FT232H，VID:PID 为 `0403:6014`。
- Linux 枚举：`/dev/ttyUSB0`；稳定路径为
  `/dev/serial/by-id/usb-FTDI_Single_RS232-HS-if00-port0`。
- PWM 周期：20 ms（50 Hz）。
- PWM 范围：500–2500 µs；该范围由用户于 2026-09-07 向卖家客服确认。
- 实测方向：500 µs方向为闭合，2500 µs方向为张开，1500 µs为舵机中位。
- 本轮已验证脉宽：500、550、600、650、700、750、800、850、900、950、1000、
  1111、1167、1222、1333、1389、1444、1500、1556、1600、1611、1667、1700 µs。
- 已实测上限为 1700 µs；2500 µs是客服确认的舵机电气上限，但尚未在当前夹爪装配上
  验证机械安全性。

最后一次台架指令为 1700 µs、持续 1 秒。测试程序随后停止 PWM 并将 AD7 恢复为输入，
所以当前不是持续保持状态。

当前 ROS 驱动工作值为 `operating_close_us=550`、`operating_open_us=1650`；真实
action 已确认能驱动夹爪。实测相对指尖净开度映射为：550 µs→0 mm、820 µs→24 mm、
1100 µs→60 mm、1380 µs→100 mm、1480 µs→110 mm、1650 µs→130 mm。脉宽按
100 kHz bit-bang 的 10 µs实际分辨率记录；500/2500 µs仍只作为电气边界保留。

## 2. 资料边界

本地资料目录：`/home/industio/YDI-J22柔性机械夹爪-三维模型`。

其中包含二指、三指、四指 STEP 模型和尺寸图，没有舵机具体型号或厂家 PWM 协议表。
四指尺寸图给出的机械参数包括最大展开尺寸 166 mm、最大夹持角约 117°。商品图片还给出
5 V、0–180°、PWM、15 kg 金属舵机、堵转电流约 1.6 A。500–2500 µs范围来自本轮
客服确认，不是从本地文件中读取的；若用于验收或量产，应向卖家索取书面规格书归档。

需要区分两个范围：

1. 500–2500 µs是舵机允许的电气控制范围。
2. 夹爪装配后的安全机械范围可能更窄，受舵盘零位、连杆安装和软指形变影响。

2026-09-10 实测说明：1650 µs时相对指尖净开度约130 mm。图纸166 mm是展开外廓，
不是净开度，也不能据此推断166 mm对应2500 µs。

## 3. 测试软件

目录：`/home/industio/fairino_ws/test/jiazhua`

- `gripper_test.py`：设备检测、角度映射、直接脉宽输出和保守扫动。
- `test_gripper_test.py`：角度映射、PWM帧结构与边界检查。
- `requirements.txt`：固定 `pyftdi==0.57.2`。
- `.venv`：本机已创建的隔离环境，不进入 Git。

程序使用 PyFtdi 异步 bit-bang 模式，将 AD7 作为 GPIO 输出。默认采样率为100 kHz，
每个样本10 µs，因此脉宽分辨率为10 µs。程序需要显式安全确认参数才会输出PWM。

## 4. 常用命令

```bash
cd /home/industio/fairino_ws/test/jiazhua
```

只检测设备，不产生动作：

```bash
python3 gripper_test.py detect
```

按舵机角度点动；默认映射为500–2500 µs，默认AD7：

```bash
.venv/bin/python gripper_test.py move --angle 90 --duration 1 \
  --execute I_HAVE_EXTERNAL_5V_POWER
```

直接按脉宽点动，推荐用于继续标定：

```bash
.venv/bin/python gripper_test.py pulse --us 1700 --duration 1 \
  --execute I_HAVE_EXTERNAL_5V_POWER
```

运行软件自检：

```bash
python3 -m unittest -v test_gripper_test.py
```

## 5. 后续机械行程标定

闭合端已经测试到500 µs。由于柔性手指能够被人工掰动，不能以“手指能否掰开”判断
舵机是否到位；应观察舵机轴、摇臂和连杆是否继续运动，并监听堵转声。

张开端建议从1700 µs继续，每次增加100 µs；靠近机械极限后改为50 µs。任一步出现
连杆顶死、持续抖动、异常声音、电源压降或舵机快速升温，应立即断电，并将安全上限设在
发生异常值以下至少50–100 µs。不要仅因客服给出2500 µs就直接将装配后的夹爪跳转至
2500 µs。

最终应记录：

- `close_us`：无堵转的完全闭合值。
- `open_us`：无机械干涉的最大张开值。
- `operating_close_us` / `operating_open_us`：在两端各保留安全裕量后的生产值。

## 6. 已知限制与正式集成建议

当前工具只发送有限时长PWM；信号结束后舵机不会获得持续的位置指令，软指可被反向掰动
不等于闭合标定失败。

FT232H异步bit-bang适合台架验证，但USB调度和数据缓冲不是硬实时机制。正式接入机器人时
建议使用具有硬件定时器的PWM控制器、MCU或机器人控制柜PWM输出，并持续刷新50 Hz信号。
上层接口建议暴露“开度百分比”而不是舵机角度：0%映射安全闭合脉宽，100%映射安全张开
脉宽；同时实现脉宽限幅、渐变、超时停止和电流/堵转保护。

夹持时不能仅依靠命令到达端点来建立夹持力。柔性夹爪应根据工件尺寸留出形变行程；若需
稳定可重复的夹持力，建议增加电流检测或力传感器，并限制持续堵转电流。

## 7. ROS 2 / MoveIt 接入（2026-09-10）

新增包：`/home/industio/fairino_ws/src/fr3c_gripper_driver`。

- 提供 `/ydi_j22_gripper_controller/gripper_cmd`，类型为
  `control_msgs/action/GripperCommand`；
- 提供 mock 与 FT232H 两个 backend；启动节点本身不输出 PWM，FT232H 只在第一个已接受
  goal 到来时打开；
- 真实输出需要参数与环境变量双重解锁；
- 实现位置/脉宽限幅、400 µs/s 脉宽渐变、8 s timeout、cancel 冻结、USB 异常 abort、
  busy 拒绝、`/diagnostics` 和估计 `/joint_states`；
- 没有传感器反馈，因此 effort 发布为 NaN，正数 `max_effort` 被拒绝，诊断明确标记
  `open_loop_estimated`；
- MoveIt 已添加 `ydi_j22_gripper` 组、open/closed 命名状态和
  `GripperCommand` controller。当前碰撞体仍为保守固定包络，命令关节不驱动柔性手指几何。

构建、mock/真机启动及动作命令见
`/home/industio/fairino_ws/src/fr3c_gripper_driver/README.md`。软件验证已通过 4 个映射/PWM
单元测试、URDF 解析、mock 完整动作、越界拒绝、诊断及 FT232H 未解锁拒绝；验证过程没有
向真实夹爪发送新的 PWM。

2026-09-10 首版连续 backend 在真实测试中出现过“旧张开波形延后输出”的现象：夹爪从
约60 mm状态在没有新操作的情况下张到最大外廓。高概率原因是生产线程没有按波形时长限速，
使旧 PWM 数据积压在主机/USB 缓冲。已改为按实际采样率节拍发送、每块由10帧缩到2帧
（约40 ms），并增加发送速率单元测试、帧/字节计数和 goal/有效脉宽审计日志。MoveIt+相机
脚本继续默认启动夹爪节点，但拒绝与已有夹爪节点并存；无解锁 token 时节点在线但拒绝真实
goal。修复后的真实持续输出与多次目标切换仍需复测后才能关闭该异常。
