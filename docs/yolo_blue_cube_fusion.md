# 蓝色方块 + YOLO 检测融合

启动前用 `detection_mode` 选择两个独立场景之一：`blue` 使用原有蓝色分割，`yolo` 使用垃圾分类模型检测带类别标签的方块。
两个模式都要求同帧只有一个目标，之后共用 D2C 深度、手眼变换、顶面几何、稳定窗口和原有抓取话题。定位节点名保留为 `blue_cube_localizer` 以维持话题兼容；抓取节点与执行逻辑没有改动。YOLO 模式将类别和置信度写入定位节点的 `status`，调试图用紫色框显示目标。

移植来源是旧项目 `dofbot_pro_yolov11/rknn_detector.py` 和 `best.rknn`。
模型 SHA-256 为 `422a853f3f46f535aff054899da470e77144c3f5e5cf42461e11841ad53f4275`，输入为 640×640 RGB，输出契约固定为 16 种垃圾类别的九个张量。
它**没有通用方块类别**：无标签的蓝色方块不能指望通过 YOLO 检查；需要使用旧项目训练类别对应的方块样本，或另行训练、转换新模型并调整类别与输出解码。

`detection_mode` 默认是 `blue`，原有蓝色方块流程保持原样。RKNNLite 2.3.2 wheel 和 ruamel 运行文件已复制到本项目的 `third_party/rknn/`；一次性安装到工作区的 `.venv-yolo`，不修改系统 Python。使用平常的完整启动命令，最后加模式参数：

```bash
cd /home/industio/fairino_ws
scripts/setup_yolo_runtime.sh  # 首次运行一次；已在本机完成
source /opt/ros/humble/setup.bash
source install/setup.bash
scripts/run_moveit_camera_rviz.sh detection_mode:=yolo
```

蓝色场景用原命令 `scripts/run_moveit_camera_rviz.sh`，或显式加 `detection_mode:=blue`。启动脚本也接受 `FR3C_DETECTION_MODE=yolo` 环境变量，命令行参数优先。模型路径可用 `yolo_model_path:=...` 覆盖。建议先观察 `/perception/blue_cube_localizer/status` 和 `/perception/blue_cube_localizer/debug_image`，确认持续得到 `valid=true`、正确的 `yolo_class` 和匹配框，再走现有抓取前检查。YOLO 没有唯一目标、目标框大小不合适、深度顶面不像支持的方块时，该帧均为 `valid=false`。模式切换需要重启定位节点，不会在抓取期间热切换。

离线测试：

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/fr3c_camera:$PYTHONPATH \
  .venv-yolo/bin/python -m pytest -q \
  src/fr3c_camera/test/test_cube_perception.py \
  src/fr3c_camera/test/test_tracking_window.py \
  src/fr3c_camera/test/test_yolo_cube_candidate.py
```

隔离执行环境不映射 `/dev/dri`，但已在宿主机权限下用当前项目的本地 RKNNLite 环境与模型完成合成图 NPU 推理：RKNNLite/运行时 2.3.2、驱动 0.9.8、九个输出张量匹配。合成空图没有目标，尚未验证当前机械臂场景中的实物方块检测、深度定位或抓取。
