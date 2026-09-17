# YDI-J22 四指夹爪台架测试

当前夹爪使用 FT232H `ADBUS7/AD7` 输出 50 Hz 舵机 PWM，不是 UART 数据。
红线必须由独立、足够电流的 5 V 电源供电，棕线接电源 GND，黄线接 AD7；电源
GND 与 FT232H GND 必须共地。不要用 FT232H 小板给夹爪供电。

本项目只保留两个已选工作位：

| 命令 | 项目语义 | 10 kHz 实际波形 |
| --- | --- | --- |
| `--angles 31` | 原 26 mm 抓取位 | 每帧高 8 个样本，约 800 us |
| `--angles 70` | 原 60 mm 张开/释放位 | 每帧高 13 个样本，约 1300 us |

程序源自 `/home/industio/YDI-J22柔性机械夹爪-三维模型/gripper_ft232h.py`，
已移入本项目并限制为上述两个角度。它一次性写入 0.8 秒波形，等待发送完成后将
AD7 拉低并关闭 FT232H；没有后台循环、扫角或持续补充数据。

安装依赖：

```bash
cd /home/industio/fairino_ws/test/jiazhua
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

现场监护下分别执行一次：

```bash
.venv/bin/python gripper_ft232h.py --angles 31
.venv/bin/python gripper_ft232h.py --angles 70
```

不要把两个角度写成来回重复序列。出现机械顶死、异响、电源压降或抖动时立即断电。

不连接硬件的软件自检：

```bash
.venv/bin/python -m unittest -v test_gripper_test.py
```
