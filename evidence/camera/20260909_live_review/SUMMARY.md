# 2026-09-09 Gemini Pro 续测摘要

- 实时枚举：`SV1301S_U3`、PID `0x0614`、SN `AY23B520016`、USB 3.0、端口 `2-1.1.2`。
- 设备启动信息：固件 `RD3013`、wrapper 1.5.15、SDK 1.10.35。
- 首次连流约 1 秒后发生一次断开，驱动自动复位并重新连接；其后 60 秒审计通过。
- 彩色 14.25 FPS，深度 13.97 FPS；最大空窗 0.276/0.268 秒；时间戳无回退。
- 808 组有效配对；彩深时间差 P50/P95 19.8/52.5 ms；最大消息年龄 0.121 秒。
- 彩色 `rgb8`、深度 `16UC1` 毫米；两路和两份 CameraInfo 均为 640×480、
  `camera_color_optical_frame`，K 矩阵一致。彩色畸变非零，D2C 深度畸变为零。
- 整图有效深度比例中位数 31.6%。当前天花板/复杂桌面场景存在大量空洞和混合深度，
  不满足抓取距离平面质量验收，也不能据此声明绝对精度失败。
- 新增实时审阅界面；自动更新 `live_review.png` 和 `live_report.json`。
- 固定 300 帧蓝色垫 ROI：失败；深度中值 290 mm，有效率中位 41.6%，时间 P95 跨度 182 mm。
- 固定 300 帧木桌 ROI：失败；深度中值 237 mm，有效率中位 48.0%，时间 P95 跨度 61.1 mm。
- 两个 ROI 的时间契约均通过，当前异常不是时间戳、深度单位或陈旧数据导致；当前场景的
  对齐深度质量确实不满足 95% 有效率和 5 mm 时间跨度门槛。
- 未闭环：铭牌完整 SKU；150/200/250/300/400 mm 尺量平面各 300 帧的偏差、有效率和抖动。

## Windows 复测对照与下一轮方案

- Windows Orbbec SDK 1.5.7 复测确认设备实际为 `Astra Stereo S U3`（内部名同为
  `SV1301S_U3`），原始深度 640×400，软件 D2C 为 640×480，深度单位均为毫米。
- Windows 约 0.75 m 静态场景的 D2C 中心 100×100 ROI 有效率为 98.75%，帧中值标准差
  1.99 mm、峰峰值 7 mm。Linux 最初两个 ROI 约 0.24～0.29 m，接近 0.25 m 起测边界；
  但后续约 0.56 m ROI 仍失败，因此工作距离只能解释早期近距结果，不能解释全部差异。
- 主深度画面采用与 Windows 复测代码相同的当前单帧 D2C 有效像素 2%～98% 自适应伪彩，
  D2C 彩深融合也使用这份未经时间筛选的原始毫米深度。5 个独立帧的保守时间一致性仅作为
  附加置信度指标，不再遮蔽主深度画面。无效值保持无效，不做补洞；稳定掩码要求最新帧
  有效且 5 帧最小～最大跨度默认不超过 15 mm。
- 修复旧实时界面按 GUI 重绘次数累计“300 帧”的问题；现在按深度消息时间戳去重。独立
  `rgbd_roi_quality` 原本就是逐消息采样，不受该问题影响。
- 便捷脚本默认只把审阅画面旋转 180°，点击坐标会反算回原始 optical frame；ROS 图像、
  CameraInfo 和后续三维坐标均不旋转，避免破坏成像模型。
- 下一轮实物门禁改为先做 400、600、800、1000 mm 哑光平面，每点 300 帧；250/300 mm
  只作为近距边界表征。通过后再测试抓取物材质、静止检测 ROI 和三维反投影。
- 新增 `config/astra_stereo_u3_sw.yaml`，保持分辨率、帧率和滤波参数不变，仅把 Linux
  D2C 从硬件模式切换为软件模式，用于与 Windows 软件 D2C 做单变量对照；在对照结论
  形成前不替换生产默认配置。

### 2026-09-09 Linux 现场修正版结果

- 修正了把“5 帧严格稳定掩码”当主深度画面的错误；修正版主 pane 和融合 pane 均直接
  使用当前单帧 D2C 毫米深度，伪彩公式与 Windows `camera_retest.py::save_visuals()` 一致。
  5 帧稳定结果只保留为 `stable_valid_fraction`，供未来抓取候选置信度使用。
- 硬件 D2C 修正版快照：`evidence/camera/20260909_105400_live/`。主深度画面不再因稳定
  门禁而全黑；当前复杂动态画面自适应色标约为 152～4180 mm。
- 同一地面 ROI `(479,319)` 的 300 帧单变量对照：硬件 D2C 原始有效率 32.6%、中值
  560.5 mm、时间 P95 跨度 215.1 mm；软件 D2C 原始有效率 26.8%、中值 548.0 mm、
  时间 P95 跨度 187.4 mm。软件 D2C 没有恢复 Windows 约 0.75 m 静态目标的质量，说明
  HW/SW 对齐模式不是本轮差异的充分解释。
- Windows 脚本符号名虽请求 `OB_PY_FORMAT_Y16`，实测 report 中 format code 为 11；当前
  Linux SDK 的 code 11 名为 `Y11`，并注明 SDK 默认解包为 Y16。不能仅凭新旧 SDK 的枚举
  名称差异判定板端使用了错误像素格式。
- Windows 脚本依赖 CPython 3.9 Win64 `.pyd` 与 `OrbbecSDK.dll`，不能直接加载到 Linux
  ARM64；已复用其深度解码语义、自适应伪彩、软件 D2C 对照和质量统计思路。生产抓取仍应
  消费 `16UC1` 毫米原值，而不是消费伪彩图颜色。
- 将 Windows 保存的 `aligned_depth_mm.png` 作为完全相同输入，在 ARM 上执行迁移后的
  自适应伪彩，输出 `evidence/camera/20260909_arm_from_windows_depth.png`；与 Windows
  `aligned_depth_color.png` 的 307200 个像素逐像素完全一致，最大通道差 0，两个 PNG 的
  SHA256 同为 `13880d381417a7b86d6b620cd2c40f57009ca19c49a5a6434f448d00d66bbf62`。
  因此 ARM 可以生成电脑端图像，当前差异确定来自实时深度输入而非绘图能力。
- 新增原始深度审计与 `config/astra_stereo_u3_raw_30fps.yaml`。D2C 关闭后采满 120 帧：
  原始深度 640×400@30.20 FPS、`camera_depth_optical_frame`、整图有效率中位 63.0%，但
  中心 100×100 ROI 有效率 63.0%、帧中值时间 P95 跨度 124.5 mm，仍未通过。这把问题
  继续前移到原始深度/现场条件/SDK或主机链路，而不是 D2C 或 YOLO。
- 只读状态：laser=true、LDP=true、LDP protection=false，当前没有触发近距激光保护；
  深度曝光 3000、增益 2000。下一次有意义的对照必须清空运动物体，并在镜头前约 0.75 m
  固定哑光平面；当前数据不得进入抓取坐标链。

机器可读 60 秒结果见 `audit_60s/report.json`。

## 2026-09-09 第二台 DaBai DCW2 与参考抓取工程复现

- 新接入设备精确枚举为 `DaBai DCW2`、PID `0x06a0`、SN `CH82C5201C9`、固件
  `RD1017`、USB 2.0；RGB UVC 接口 PID 为 `0x0561`。原 Astra 仍接在 USB 3.0，所有
  启动均按序列号选择，未混淆两台设备。
- 第一次仅开 DaBai 原始深度时，消息契约为 `640x400 16UC1`、小端、step 1280、data
  512000 bytes；直接按消息字节解码与项目 `depth_metres()` 的结果在全部 120 帧逐像素
  相等。此时整图有效率仅 0.82%，但相机状态同时明确报告 `laser=false`、
  `LDP protection=true`、保护测距约 104 mm，不能把这组黑图归因于 Python reshape、
  端序或毫米/米换算。
- 用户提供的已验证工程位于 `/home/industio/DofbotPro_JetsonNano_0106/dofbot`。其 P4
  路径使用 Orbbec SDK 1.10.18，参数为 RGB 640x480 + Depth 640x400 Y11 @10 FPS、
  硬件 D2C、关闭设备软滤波和 LDP，发布 `640x480 16UC1` 对齐深度。抓取融合并非只取
  单像素：bbox 中央 40% ROI 经有效率、MAD/IQR、标准差和 5 帧位置稳定门禁后才发布
  可抓目标。
- 原样运行参考 SDK 1.10.18 的第一次复现仍只有 5.4% 有效深度；同步 RGB 快照显示相机
  倒置且镜头前大面积紧贴透明反光塑料，远处主要是天花板。调整现场后再次运行同一代码，
  整图有效率中位升至 83.76%，中心探针中值 679 mm，深度轮廓恢复连续。这证明“同一
  软件由黑变正常”受现场遮挡/保护状态直接控制。
- 新增 `config/dabai_dcw2_reference.yaml`，在当前 SDK 1.10.35 上复刻参考工程全部相机
  参数。该路径同样输出连续的 RGB/对齐深度，RGB/Depth P95 时间差 32.5 ms；固定桌面
  ROI `(320,455)`、20x20、120 帧验收为 **PASS**：有效率 100%、中值 291 mm、帧中值
  P95 跨度 2.0 mm、标准差 0.93 mm、逐像素时间标准差中位 0.93 mm。消息字节直解与
  项目解码仍在全部帧完全相等。
- 结论：当前 ARM 能获得并处理可用于抓取的对齐深度，项目 Python 深度读取和 SDK
  1.10.35 均未复现“必然乱码”故障。此前全黑的充分原因是第二台相机近距保护/遮挡；
  原 Astra 当前的异常仍需在同一固定哑光靶、相同视场下单独判断设备/固件差异。后续抓取
  应以 `dabai_dcw2_reference.yaml` 为候选输入配置，并保持 bbox ROI 质量与时间稳定门禁，
  不能仅凭伪彩图或单个中心像素放行。

本轮主要机器可读证据：

- `evidence/camera/20260909_dabai_raw/raw_depth_report.json`
- `evidence/camera/20260909_astra_raw_bytecheck/raw_depth_report.json`
- `evidence/camera/20260909_dabai_reference_sdk_repeat/stability.json`
- `evidence/camera/20260909_dabai_current_sdk_reference_config/stability.json`
- `evidence/camera/20260909_dabai_current_sdk_reference_config/table_roi_audit/raw_depth_report.json`
