# 桌面红块倾斜抓取测试

范围：30 mm 桌面红块，倾斜接近、闭合、保持姿态沿 base Z 抬高 50 mm，最后保持夹持。
默认 30°、从顶面下降 10 mm，支持 `--angle-deg` 指定正负 5°～45°，
支持 `--depth-mm` 指定 5～15 mm。每次试验只用一个绝对倾角，
仍由现有规划器依次尝试方块的等价 yaw。正式 action 会自动搜索 0、±10、±20、±30°；
此脚本把相对倾角候选固定为 `[0]`，用于隔离验证命令行指定的姿态。

标定文件 `ydi_j22_tcp.yaml` 已将 `grasp_frame` 定义在共同闭合指尖。
零接触偏移时，TCP 目标是红块顶面中心沿世界 Z 下移指定深度。
从 15 mm 改成 10 mm 会把整个夹爪提高 5 mm，为倾斜后较低的夹指边角留出桌面间隙。
姿态为 `Rz(object_yaw - pi/2 + symmetry) * Ry(pi) * Rx(tilt)`，
绕局部开合轴 X 倾斜，保持开合轴水平。接近点在顶面上方 50 mm，
从该点保持倾角竖直下降；附着物使用现有完整相对变换，抬升时方块保持直立。
第二种“边角夹持”由 `--contact-toward-flange-mm` 表达：它是从 TCP 沿夹持面朝法兰方向
到实际接触点的距离。程序对每个候选姿态计算 `TCP = 方块接触点 - R × 接触点偏移`，
因此 TCP 可以落在方块区域之外，而指定夹爪边角仍对准方块。55 mm 是允许的接触面总长度上限，
不应直接作为默认偏移；需先在 RViz 中从小偏移验证实际方向和碰撞模型。
非零接触偏移当前只允许 `pick_hold` 测试。抬升时已有完整的方块—TCP 相对变换；正式接入
分拣或码垛前，还需让放置目标也从该相对变换反解 TCP 位姿，不能继续使用原来的竖直硬编码高度。

测试使用现有 P5 程序和配置的独立副本，不修改普通 action 的配置。
夹爪碰撞几何仍是近似模型，规划通过不代表真实接触和摩擦已经验证。
夹爪没有夹持反馈，action 成功只代表命令/轨迹完成；需观察方块是否随抬升离桌。

## 启动与运行

先正常退出当前完整抓取 launch，再将红块单独放在桌面，移开之前的堆叠和夹指扫掠范围内的物体。
测试启动器发现已有机器人栈会拒绝重复启动；不会替用户杀进程或移动机械臂。
顶面 Z 仅接受 25～40 mm，以排除之前塔顶的红块。

终端一，只规划模式启动（不会自动发送抓取目标）：

```bash
cd /home/industio/fairino_ws
bash scripts/grasp/test_tilted_red.sh launch --angle-deg 30 --depth-mm 10
```

终端二，发送一次红块测试：

```bash
cd /home/industio/fairino_ws
bash scripts/grasp/test_tilted_red.sh run --angle-deg 30 --depth-mm 10
```

查看 RViz 完整接近、下降和带物抬升轨迹。随后退出测试 launch；实机测试在两个命令中均加 `--execute`：

```bash
bash scripts/grasp/test_tilted_red.sh launch --angle-deg 30 --depth-mm 10 --execute
# 另一个终端，待栈启动且 P6 通过：
bash scripts/grasp/test_tilted_red.sh run --angle-deg 30 --depth-mm 10 --execute
```

实机流程会先发开爪命令，再规划并执行；开始前夹爪应为空。
速度和加速度缩放均为 0.03。使用原有新鲜视觉快照、颜色筛选、完整笛卡尔路径、
关节跳变检查、碰撞检查及 P6 门控。run 会核对服务器实际倾角、执行模式和测试参数，
避免把命令误发给普通生产节点。超时或 Ctrl+C 会尝试取消 action。
成功后保持夹持，不自动松爪、回家或码垛；后续仍按原有 held-object 释放确认流程处理。

生成的独立配置保存在 `evidence/tilted_red_grasp/configs/`；目标、候选结果和轨迹保存在
`evidence/tilted_red_grasp/runs/`。仅导出配置、不连接 ROS：

```bash
python3 tools/tilted_red_grasp_test.py config --angle-deg 20
python3 -m pytest -q tools/test_tilted_red_grasp_test.py
```

第二种边角方案先只做规划，例如接触点从 TCP 朝法兰移动 15 mm：

```bash
bash scripts/grasp/test_tilted_red.sh launch --angle-deg 30 --depth-mm 10 \
  --contact-toward-flange-mm 15
bash scripts/grasp/test_tilted_red.sh run --angle-deg 30 --depth-mm 10 \
  --contact-toward-flange-mm 15
```
