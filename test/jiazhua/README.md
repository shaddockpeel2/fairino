# YDI-J22 四指夹爪台架测试

该夹爪内部是三线 5 V、0–180° 舵机，黄色线需要的是约 50 Hz 的舵机
PWM，不是 UART 字节流。FT232H 可切换成 GPIO 位操作模式输出 PWM。当前台架实测
信号接在 `ADBUS7/AD7`，程序也默认使用 7 号脚。

2026-09-07 经用户向卖家客服确认，舵机 PWM 范围为 500–2500 µs，对应 0–180°。
台架实测方向为：脉宽减小则夹爪闭合，脉宽增大则夹爪张开。

## 接线和上电前检查

- 棕线：夹爪 GND；接独立 5 V 电源负极，同时接 FT232H GND（必须共地）。
- 红线：夹爪 +5 V；接独立、稳压的 5 V 电源，建议持续电流能力至少 2 A。
- 黄线：信号；当前接 FT232H `ADBUS7/AD7`，不要接真正 RS-232 的 ± 电压端口。
- **不要用 FT232H 小板的 5 V 引脚给夹爪供电。** 商品参数给出的堵转电流是
  1.6 A，普通转换板、USB 口及其细导线未必能安全承受。
- 首次动作时卸下负载并让夹爪悬空，手放在电源开关旁。机械顶死、异响或电源掉压时
  立即断电。

如果转换板输出是标有 `RS232` 的 DB9 接口而非 3.3/5 V TTL GPIO，请不要连接黄色线。
FT232H 的 GPIO 高电平为 3.3 V，一般可被 5 V 舵机识别，禁止向该脚输入 5 V。

## 使用

只检测 USB 设备（不会输出 PWM）：

```bash
cd /home/industio/fairino_ws/test/jiazhua
python3 gripper_test.py detect
```

安装执行动作所需依赖：

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

首次只测试中位，确认外部 5 V 供电和共地后执行（AD7 已是默认值）：

```bash
.venv/bin/python gripper_test.py move --angle 90 \
  --execute I_HAVE_EXTERNAL_5V_POWER
```

标定时建议直接使用脉宽，避免把舵机角度和夹爪开度混淆：

```bash
.venv/bin/python gripper_test.py pulse --us 1500 \
  --execute I_HAVE_EXTERNAL_5V_POWER
```

中位正常后，小范围扫动：

```bash
.venv/bin/python gripper_test.py sweep \
  --execute I_HAVE_EXTERNAL_5V_POWER
```

本台架实测为低角度闭合、高角度张开。新装配或调整过舵盘后，不要一开始就测试
0° 和 180°。若机械行程在到达端点前已经顶死，可用
`--min-us`、`--max-us` 缩窄范围，例如 `--min-us 1100 --max-us 1900`。

`move` 和 `pulse` 都只在 `--duration` 指定的时间内输出 PWM，结束后 AD7 恢复为输入，
因此它们适合台架点动，不提供持续夹持。机器人正式运行应使用持续、硬件定时的 PWM。

程序控制 FT232H 时会暂时从 Linux 的 `ftdi_sio` 驱动接管设备，结束后设备会重新成为
`/dev/ttyUSB*`。如果 ModemManager 或其他程序占用串口，先关闭占用程序。

## 自检

```bash
python3 -m unittest -v test_gripper_test.py
```
