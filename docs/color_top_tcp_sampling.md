# 颜色顶面与 TCP 接触点配对采集

这个工具只读取定位状态和 TF，不发送机械臂运动命令。每个位置用相同的样本名分别保存：

- `color_top`：夹爪移开、方块无遮挡时，从 `/perception/blue_cube_localizer/status` 采集 8 个新的稳定顶面结果。
- `tcp_contact`：TCP 轻触同一方块上表面时，采集 12 个新的 `base_link → grasp_frame` TF。

结果默认写入一个 JSON 会话。脚本按感知帧的 `sample_stamp` 去重，并拒绝脚本启动前的旧结果；所以不会把定位节点心跳重复发布的旧顶面误存成新样本。

## 每个位置的操作

先启动现有相机、定位、机器人 TF 环境，并确认以下命令能输出一条状态：

```bash
ros2 topic echo /perception/blue_cube_localizer/status --once
```

将一个 30 mm 方块放在待测位置，夹爪移开，等定位稳定后执行：

```bash
/home/industio/fairino_ws/scripts/calibration/capture_surface_contact_sample.sh color \
  --session /home/industio/fairino_ws/evidence/color_top_tcp_contact.json \
  --sample P01 --expected-color blue
```

然后保持方块不动，使 TCP 轻触同一方块顶面并保持静止，执行：

```bash
/home/industio/fairino_ws/scripts/calibration/capture_surface_contact_sample.sh tcp \
  --session /home/industio/fairino_ws/evidence/color_top_tcp_contact.json \
  --sample P01
```

换一个桌面 XY 位置，依次使用 `P02`、`P03` 等新名字。每个位置必须先采颜色顶面，再接触；否则夹爪遮挡会使颜色定位无效。建议至少采 5 个 XY 位置。如果要区分不同相机距离或工具姿态，可用 `near_P01`、`far_P01`、`tilt_P01` 等名字。

查看全部配对结果：

```bash
/home/industio/fairino_ws/scripts/calibration/capture_surface_contact_sample.sh summary \
  --session /home/industio/fairino_ws/evidence/color_top_tcp_contact.json
```

采错某一半时，在对应的 `color` 或 `tcp` 命令末尾加 `--replace`。要重新开始整个会话可执行 `reset`；旧 JSON 会先被重命名归档，不会直接删除。

采完后把 `evidence/color_top_tcp_contact.json` 发来分析。关键量 `tcp_minus_color_top_xyz_mm` 的 Z 若持续约为 `-5`，说明视觉顶面整体高约 5 mm；若它随 XY/相机姿态或距离系统变化，则更像手眼外参或深度尺度问题。
