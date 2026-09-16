# 多目标四色连续识别、抓取和分区放置

使用已有 A4 TCP 标定文件 `evidence/a4_paper_grid.json` 的投影区域中心。现场确认的映射为蓝→A、黄→B、红→C、绿→D；各中心由启动时读取的 JSON 提供，重新采集或投影四角后重启抓取栈即可更新。标定文件必须是 `base_link` 中 `base_z` 投影，区域平面 Z 必须与任务配置的地面平面 Z 一致。RViz 的彩色区域只是标记，实际放置位由这份标定文件决定。

将一个或多个 30 × 30 × 30 mm 方块放在相机画面和抓取范围内，保持机械臂静止。四色模式同时检查红、绿、蓝、黄四种 HSV 掩膜，再用深度、方形顶面和桌面高度排除画面中的平面色块。视觉允许多个三维有效方块同时存在，并按 **红→黄→蓝→绿**、同色按 base X/Y 的固定顺序提供下一个目标。状态中的 `rgb_color_regions` 是颜色区域数，`physical_cube_count` 是通过三维验证的方块数，`actionable_cube_count` 是未被本批任务跳过的方块数，`rejected_color_regions` 记录排除原因。检测状态里的 `color`、`sample_stamp` 必须与本次稳定目标匹配；当前仅允许等边方块进入分拣流程。红色跨 OpenCV 色相 0/179 接缝。不同灯光下可调整 `src/fr3c_camera/config/blue_cube_localizer.yaml` 中四组 `*_hue_intervals`、饱和度和亮度下限，并先查看 `/perception/blue_cube_localizer/debug_image` 与 `status`。

第一次先仅规划，检查 RViz 中抓取、目标区域、释放与返航整段轨迹：

```bash
cd /home/industio/fairino_ws
FR3C_FIRST_GRASP_EXECUTE=false scripts/run_moveit_camera_rviz.sh detection_mode:=colors
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'
```

规划成功后，关闭旧栈、移除规划期间可能留在 MoveIt 场景里的旧方块，再以执行模式启动。人工单块服务仍为 `/start`、`/descend_and_close`、`/confirm_grip_and_lift`。若单块和整段轨迹已经现场确认，在 `colors` 执行模式下调用一次 `/start_continuous` 即启动整批任务：每块完成放置并返回任务起始关节位置后重新识别，连续 10 秒确认 `actionable_cube_count=0` 才结束。

```bash
ros2 service call /fr3c_first_grasp/start_continuous std_srvs/srv/Trigger '{}'
ros2 topic echo /fr3c_first_grasp/state
```

每个目标仍会在机械臂运动前规划完整的抓取、放置和返航路径。完整路径规划失败、目标超出任务范围或对应颜色已无空余放置槽时，该目标仅在本批任务中被跳过，然后继续下一个。机械臂开始运动之后发生的动作、夹爪反馈、到位检查或场景更新失败会中止整批任务，不会自动跳到下一个目标。

“上方 5 mm”按**方块底面到区域平面的间距**定义。夹爪 TCP 位于约 30 mm 方块的中心，所以释放时 TCP 约在区域平面上方 20 mm。直接让 TCP 到平面上方 5 mm 会把方块压入桌面。张爪后，规划场景把方块视为落在区域平面上；随后先垂直抬离，再返回起始关节位置。

2026-09-16 起，感知节点对视觉顶面应用默认 `top_pose_z_compensation_m=-0.004`。它把发布、桌面判定和抓取使用的顶面 Z 下移 4 mm，但不直接修改上述放置 TCP 公式。由于抓取 TCP 会随修正后的顶面下降 4 mm，实际方块相对 TCP 的夹持关系会更接近任务模型，所以预计能消除此前“命令底面间隙 5 mm、实物却几乎贴面释放”的联动误差。首次启用仍必须先仅规划并单块低速验证。

每个颜色区域会根据标定区域尺寸生成一个简单固定的 2×2 放置槽序列；区域不足以安全容纳 2×2 时只使用中心槽。已经放下的方块继续以独立碰撞体保留在 MoveIt 场景，视觉重新看到本批任务的源位置残留或已放置方块时会跳过，避免反复抓取。最终状态为 `batch_complete` 或 `batch_complete_with_skips`，并给出成功和跳过数量。启动命令默认使用 `evidence/a4_paper_grid.json`，可用 `grid_file:=/absolute/path/to/grid.json` 指定其他标定文件。`detection_mode:=blue` 和 `yolo` 保留原有单目标行为。真实机械臂的多块分区轨迹、开环夹持和 5 mm 释放间距仍需先在现场用单块与仅规划模式核对。
