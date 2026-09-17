# P3 软件验证摘要

日期：2026-09-16

- `colcon build --symlink-install --packages-select fr3c_camera fr3c_voice`：2 个包通过。
- `python3 -m pytest -q src/fr3c_voice/test src/fr3c_camera/test`：39 passed。
- `ROS_DOMAIN_ID=77 python3 -m pytest -q test src/fr3c_camera/test src/fr3c_gripper_driver/test src/fairino_mtc_demo/test src/fr3c_voice/test`：96 passed，1 个既有 Python 3.11 前瞻 warning。
- `python3 -m py_compile`：P3 Python 包、相机门禁和三个验收脚本通过。
- `git diff --check`：通过。

未隔离 ROS domain 的一次扩大回归为 95 passed、1 failed：现场机械栈正在发布 `/joint_states`，夹爪测试订阅同名 topic 后最后收到的是六轴机械臂消息。切换到 `ROS_DOMAIN_ID=77` 后同一测试集合 96/96 通过，确认是测试域污染而非实现回归。

板端证据：

- `acceptance.jsonl`：30 次文字、3 次视觉、busy、cancel recovery、worker-exit recovery 全通过；
- `timeout_recovery.json`：50 ms 真实超时注入和恢复通过；
- `yolo_switch.json`：4 次 YOLO 暂停/恢复、恢复后新帧确认通过；
- `yolo_switch.trace.jsonl`：逐次 owner、pause/resume、首 token 和 generation end 时间线。
