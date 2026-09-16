# 第一阶段统一追踪约定

每行一个 JSON 对象（JSONL），schema_version=1。目录为 `evidence/voice/P<阶段>/<日期>/<运行条件>/`，每次重测使用新目录，保存命令、配置校验值、退出状态、trace.jsonl、resources.jsonl 与摘要。

必填：`run_id`、`request_id`、`event`、`clock`、`monotonic_ns`、`wall_time_ns`、`pid`。同一请求跨 ASR、模型、TTS、播放器、动作沿用 request_id；单机进程均使用 CLOCK_MONOTONIC 纳秒，wall_time_ns 仅用于对照日志，不用于相减算延迟。跨重启不可相减，运行元数据记录 boot_id。ROS 图像时间戳单独保存，不能与 monotonic_ns 混减。

| event | 时间点 |
|---|---|
| input_end | 用户结束输入；测试中为固定文本注入时刻 |
| asr_final | ASR 定稿到达 |
| queue_enter / queue_leave | 请求入队 / 获得执行权 |
| qwen_first_token | Controller 收到第一个 token |
| first_text_segment | 首个可合成文本段提交 |
| tts_first_pcm | 合成器第一次 PCM 回调 |
| first_valid_pcm_write | 播放器写入含有效语音的首帧，需记录阈值与采样率 |
| generation_end | 生成完成（模块可用 component 区分） |
| playback_end | ALSA drain 完成，不能以最后一次 enqueue 代替 |
| action_state | 底层动作状态，带任务标识、状态、错误码 |

缺失事件记在 unavailable_events，不能用 0、请求完成时刻或推测时间补齐。软件首帧写入不代表真实可闻首声，声学采样另记 acoustic_first_sound。

第一阶段测试以固定文本直接驱动生产 Kokoro 回调；Qwen 并发请求独立运行，**不是已接通 Qwen→TTS 的端到端链路**。第一段文本不是 Qwen 输出。没有播放的测试不生成 first_valid_pcm_write/playback_end，没有 ASR 的文本测试不生成 asr_final。旧日志仅作历史对照，不伪造为当前统一追踪结果。

资源记录：系统 MemAvailable、swap 使用；各进程 RSS、累计 CPU ticks、start_ticks（识别 PID 重用）；NPU 原始读取或权限错误。CPU 单核百分比为 Δticks/HZ/Δ秒×100，可超过100%；系统余量用 MemAvailable，不能把共享页重复计算的 RSS 求和当成整机内存。250 ms 采样的峰值是观测峰值，不保证捕捉更短瞬态。

测试集为 `config/voice/p1_texts.json`，按 Unicode 字符数固定 10/30/60/120。cold 指新建 TTS backend，包含模型加载，但保留操作系统文件缓存；warm 指已加载 backend。Qwen 冷启动单独用 model_load_begin/model_ready 计时，循环中的 cold 标签不表示每次重启 Qwen。至少记录单独 TTS、单独 Qwen、colors 整机并发、YOLO 整机感知；Qwen 与 YOLO 在当前缺少暂停确认接口时分开运行，不用无仲裁的共同推理作为正式配置。
