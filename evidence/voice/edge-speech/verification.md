# 端侧语音感知开发验证（2026-10-08）

新增页面复用现有模型和接口。没有为验证启动真实机械臂任务或重启现有业务服务。

- 新增页面和资源隔离测试：30 项通过，含真实 Tk 布局与文本传递。
- 完整语音模块回归：253 项通过、1 项跳过。跳过项为环境缺少 ROS 测试依赖。
  既有 ALSA 播放取消用例报告一条 `PcmCancelled` 线程警告；本次未修改其实现。
- 完成 TTS 取消时的定稿等待处理后，最终定向回归：70 项通过。
  覆盖新页面、原 VoiceInput、原唤醒和原 GUI 契约。
- 修改的 Python 文件通过语法检查；本次源文件没有新增行尾空白。
- `preview.png` 为实际 Tk 页面截图，识别文本为展示示例，音频和机器人接口使用模拟对象。

最终定向回归命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/fr3c_voice/test/test_edge_speech_page.py \
  src/fr3c_voice/test/test_voice_input.py \
  src/fr3c_voice/test/test_dialogue_gui_contract.py \
  src/fr3c_voice/test/test_wake_input.py
```

仍需现场确认：真人单句识别、实际 TTS 朗读、原唤醒语音控制回归，以及断网运行。
当前 GUI 内已验证会话归属和互斥；原服务的全局停止协议不提供跨外部客户端的原子隔离。
