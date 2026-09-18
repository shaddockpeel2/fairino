# P5 中文数字与颜色读音评审表

语料 51 条，候选 5 个。WAV 位于 `<候选>/<条目>.wav`。
判定以人工听感为准；ASR 不作判据。

## kokoro-multi-lang-v1_1

- 采样率 24000 Hz，说话人数 103，使用 sid=3

| 条目 | 文本 | 应读作 | 音频 | 峰值 | 判定 |
|---|---|---|---|---:|---|
| num-0 | 零 | líng | 0.51s | -10.6 | |
| num-1 | 一 | yī | 0.52s | -12.0 | |
| num-2 | 二 | èr | 0.54s | -13.4 | |
| num-3 | 三 | sān | 0.54s | -8.1 | |
| num-4 | 四 | sì | 0.53s | -10.3 | |
| num-5 | 五 | wǔ | 0.48s | -10.8 | |
| num-6 | 六 | liù | 0.51s | -10.4 | |
| num-7 | 七 | qī | 0.54s | -10.2 | |
| num-8 | 八 | bā | 0.49s | -7.5 | |
| num-9 | 九 | jiǔ | 0.52s | -6.0 | |
| num-10 | 十 | shí | 0.32s | -9.6 | |
| num-21 | 二十一 | èr shí yī | 0.79s | -10.2 | |
| num-105 | 一百零五 | yī bǎi líng wǔ | 1.15s | -9.2 | |
| num-2024 | 二零二四 | èr líng èr sì (digit-by-digit) | 1.10s | -10.7 | |
| num-10000 | 一万 | yī wàn | 0.69s | -9.9 | |
| num-date | 2024年12月31号 | èr líng èr sì nián shí èr yuè sān shí yī hào | 2.60s | -9.6 | |
| num-police | 拨打110 | bō dǎ yāo yāo líng (幺幺零) | 1.56s | -8.9 | |
| num-phone | 一八九二零二四零五一一 | digit-by-digit yī bā jiǔ èr líng èr sì líng wǔ yī yī | 2.28s | -6.4 | |
| num-money | 123456块钱 | shí èr wàn sān qiān sì bǎi wǔ shí liù kuài qián | 2.81s | -8.5 | |
| num-decimal | 三点一四 | sān diǎn yī sì | 1.18s | -8.3 | |
| num-percent | 百分之八十五 | bǎi fēn zhī bā shí wǔ | 1.51s | -11.0 | |
| num-negative | 负十二度 | fù shí èr dù | 1.06s | -9.2 | |
| num-coord | 坐标X等于三百二十五，Y等于负一百二十 | zuò biāo X děng yú sān bǎi èr shí wǔ, Y děng yú fù yī bǎi èr shí | 4.38s | -6.2 | |
| num-mixed | 误差不超过0.5毫米，速度降到百分之三十 | wù chā bù chāo guò líng diǎn wǔ háo mǐ, sù dù jiàng dào bǎi fēn zhī sān shí | 4.52s | -6.8 | |
| num-code | 错误码1234 | cuò wù mǎ yī èr sān sì | 2.36s | -8.7 | |
| col-red | 红色 | hóng sè | 0.77s | -8.7 | |
| col-yellow | 黄色 | huáng sè | 0.79s | -8.6 | |
| col-blue | 蓝色 | lán sè | 0.60s | -10.2 | |
| col-green | 绿色 | lǜ sè | 0.74s | -9.7 | |
| col-purple | 紫色 | zǐ sè | 0.77s | -10.1 | |
| col-black | 黑色 | hēi sè | 0.75s | -7.8 | |
| col-white | 白色 | bái sè | 0.76s | -11.1 | |
| col-grey | 灰色 | huī sè | 0.73s | -7.3 | |
| col-orange | 橙色 | chéng sè | 0.76s | -11.2 | |
| col-pink | 粉色 | fěn sè | 0.76s | -7.8 | |
| col-light-blue | 浅蓝色 | qiǎn lán sè | 0.84s | -8.3 | |
| col-dark-green | 深绿色 | shēn lǜ sè | 0.82s | -6.5 | |
| col-block | 红色方块 | hóng sè fāng kuài | 1.26s | -7.3 | |
| col-all | 红色、黄色、蓝色、绿色 | hóng sè, huáng sè, lán sè, lǜ sè | 2.18s | -8.8 | |
| col-sentence | 已经找到红色方块，正在接近目标 | yǐ jīng zhǎo dào hóng sè fāng kuài, zhèng zài jiē jìn mù biāo | 3.59s | -8.2 | |
| col-none | 桌面上没有红色的方块 | zhuō miàn shàng méi yǒu hóng sè de fāng kuài | 2.30s | -9.1 | |
| rob-emoji | 好的 ✅ 马上执行 |  | 3.53s | -7.7 | |
| rob-emoji2 | 抓取完成 🎉 |  | 1.98s | -8.0 | |
| rob-markdown | **加粗** 与 `代码` 与 ~~删除~~ |  | 6.05s | -8.1 | |
| rob-json | {"intent": "pick_hold", "color": "red"} |  | 1.61s | -10.0 | |
| rob-symbols | 温度 25℃ 角度 90° 长度 3.5mm |  | 4.32s | -7.7 | |
| rob-latin | 使用 RGB-D 相机识别 YOLO 目标 |  | 3.60s | -8.4 | |
| rob-long-punct | 第一步，移动到位；第二步，下降抓取；第三步，抬升返回。 |  | 5.50s | -8.3 | |
| rob-quote | 他说：“好的”。 |  | 1.55s | -9.0 | |
| rob-space | 红 色 方 块 |  | 1.26s | -7.5 | |
| rob-newline | 第一行
第二行 |  | 1.55s | -9.8 | |

## matcha-icefall-zh-baker

- 采样率 22050 Hz，说话人数 1，使用 sid=0

| 条目 | 文本 | 应读作 | 音频 | 峰值 | 判定 |
|---|---|---|---|---:|---|
| num-0 | 零 | líng | 0.35s | -15.8 | |
| num-1 | 一 | yī | 0.27s | -14.5 | |
| num-2 | 二 | èr | 0.30s | -7.9 | |
| num-3 | 三 | sān | 0.32s | -11.9 | |
| num-4 | 四 | sì | 0.33s | -14.0 | |
| num-5 | 五 | wǔ | 0.30s | -16.0 | |
| num-6 | 六 | liù | 0.24s | -11.8 | |
| num-7 | 七 | qī | 0.37s | -13.1 | |
| num-8 | 八 | bā | 0.28s | -9.5 | |
| num-9 | 九 | jiǔ | 0.28s | -7.7 | |
| num-10 | 十 | shí | 0.36s | -12.2 | |
| num-21 | 二十一 | èr shí yī | 0.85s | -4.2 | |
| num-105 | 一百零五 | yī bǎi líng wǔ | 0.88s | -5.5 | |
| num-2024 | 二零二四 | èr líng èr sì (digit-by-digit) | 1.03s | -4.6 | |
| num-10000 | 一万 | yī wàn | 0.55s | -12.0 | |
| num-date | 2024年12月31号 | èr líng èr sì nián shí èr yuè sān shí yī hào | 2.76s | -2.4 | |
| num-police | 拨打110 | bō dǎ yāo yāo líng (幺幺零) | 1.21s | -1.7 | |
| num-phone | 一八九二零二四零五一一 | digit-by-digit yī bā jiǔ èr líng èr sì líng wǔ yī yī | 2.26s | -5.7 | |
| num-money | 123456块钱 | shí èr wàn sān qiān sì bǎi wǔ shí liù kuài qián | 2.88s | -6.6 | |
| num-decimal | 三点一四 | sān diǎn yī sì | 0.98s | -9.7 | |
| num-percent | 百分之八十五 | bǎi fēn zhī bā shí wǔ | 1.45s | -5.3 | |
| num-negative | 负十二度 | fù shí èr dù | 1.09s | -6.3 | |
| num-coord | 坐标X等于三百二十五，Y等于负一百二十 | zuò biāo X děng yú sān bǎi èr shí wǔ, Y děng yú fù yī bǎi èr shí | 3.85s | -6.3 | |
| num-mixed | 误差不超过0.5毫米，速度降到百分之三十 | wù chā bù chāo guò líng diǎn wǔ háo mǐ, sù dù jiàng dào bǎi fēn zhī sān shí | 4.15s | -6.1 | |
| num-code | 错误码1234 | cuò wù mǎ yī èr sān sì | 2.38s | -7.8 | |
| col-red | 红色 | hóng sè | 0.56s | -10.7 | |
| col-yellow | 黄色 | huáng sè | 0.57s | -7.8 | |
| col-blue | 蓝色 | lán sè | 0.61s | -3.9 | |
| col-green | 绿色 | lǜ sè | 0.58s | -10.5 | |
| col-purple | 紫色 | zǐ sè | 0.48s | -14.1 | |
| col-black | 黑色 | hēi sè | 0.55s | -11.1 | |
| col-white | 白色 | bái sè | 0.50s | -7.4 | |
| col-grey | 灰色 | huī sè | 0.58s | -8.5 | |
| col-orange | 橙色 | chéng sè | 0.60s | -8.0 | |
| col-pink | 粉色 | fěn sè | 0.52s | -3.2 | |
| col-light-blue | 浅蓝色 | qiǎn lán sè | 0.90s | -4.2 | |
| col-dark-green | 深绿色 | shēn lǜ sè | 0.75s | -7.4 | |
| col-block | 红色方块 | hóng sè fāng kuài | 1.05s | -8.3 | |
| col-all | 红色、黄色、蓝色、绿色 | hóng sè, huáng sè, lán sè, lǜ sè | 2.39s | -9.3 | |
| col-sentence | 已经找到红色方块，正在接近目标 | yǐ jīng zhǎo dào hóng sè fāng kuài, zhèng zài jiē jìn mù biāo | 3.15s | -7.6 | |
| col-none | 桌面上没有红色的方块 | zhuō miàn shàng méi yǒu hóng sè de fāng kuài | 1.99s | -7.5 | |
| rob-emoji | 好的 ✅ 马上执行 |  | 1.27s | -4.4 | |
| rob-emoji2 | 抓取完成 🎉 |  | 1.06s | -6.7 | |
| rob-markdown | **加粗** 与 `代码` 与 ~~删除~~ |  | 1.89s | -7.2 | |
| rob-json | {"intent": "pick_hold", "color": "red"} |  | 1.03s | -8.6 | |
| rob-symbols | 温度 25℃ 角度 90° 长度 3.5mm |  | 3.12s | -8.7 | |
| rob-latin | 使用 RGB-D 相机识别 YOLO 目标 |  | 1.79s | -4.2 | |
| rob-long-punct | 第一步，移动到位；第二步，下降抓取；第三步，抬升返回。 |  | 5.11s | -7.1 | |
| rob-quote | 他说：“好的”。 |  | 1.11s | -7.6 | |
| rob-space | 红 色 方 块 |  | 1.06s | -8.5 | |
| rob-newline | 第一行
第二行 |  | 1.48s | -11.7 | |

## vits-zh-aishell3

- 采样率 8000 Hz，说话人数 174，使用 sid=21

| 条目 | 文本 | 应读作 | 音频 | 峰值 | 判定 |
|---|---|---|---|---:|---|
| num-0 | 零 | líng | 0.60s | -14.0 | |
| num-1 | 一 | yī | 0.52s | -21.3 | |
| num-2 | 二 | èr | 0.51s | -9.2 | |
| num-3 | 三 | sān | 0.47s | -14.9 | |
| num-4 | 四 | sì | 0.40s | -21.9 | |
| num-5 | 五 | wǔ | 0.48s | -29.7 | |
| num-6 | 六 | liù | 0.55s | -10.6 | |
| num-7 | 七 | qī | 0.51s | -19.8 | |
| num-8 | 八 | bā | 0.51s | -9.5 | |
| num-9 | 九 | jiǔ | 0.57s | -17.7 | |
| num-10 | 十 | shí | 0.50s | -17.0 | |
| num-21 | 二十一 | èr shí yī | 0.94s | -12.1 | |
| num-105 | 一百零五 | yī bǎi líng wǔ | 1.25s | -11.9 | |
| num-2024 | 二零二四 | èr líng èr sì (digit-by-digit) | 1.33s | -6.1 | |
| num-10000 | 一万 | yī wàn | 0.69s | -11.4 | |
| num-date | 2024年12月31号 | èr líng èr sì nián shí èr yuè sān shí yī hào | 2.82s | -7.4 | |
| num-police | 拨打110 | bō dǎ yāo yāo líng (幺幺零) | 1.40s | -7.0 | |
| num-phone | 一八九二零二四零五一一 | digit-by-digit yī bā jiǔ èr líng èr sì líng wǔ yī yī | 3.09s | -9.8 | |
| num-money | 123456块钱 | shí èr wàn sān qiān sì bǎi wǔ shí liù kuài qián | 3.32s | -7.6 | |
| num-decimal | 三点一四 | sān diǎn yī sì | 1.14s | -13.0 | |
| num-percent | 百分之八十五 | bǎi fēn zhī bā shí wǔ | 1.51s | -12.5 | |
| num-negative | 负十二度 | fù shí èr dù | 0.97s | -12.2 | |
| num-coord | 坐标X等于三百二十五，Y等于负一百二十 | zuò biāo X děng yú sān bǎi èr shí wǔ, Y děng yú fù yī bǎi èr shí | 3.99s | -9.4 | |
| num-mixed | 误差不超过0.5毫米，速度降到百分之三十 | wù chā bù chāo guò líng diǎn wǔ háo mǐ, sù dù jiàng dào bǎi fēn zhī sān shí | 5.13s | -9.0 | |
| num-code | 错误码1234 | cuò wù mǎ yī èr sān sì | 2.34s | -9.4 | |
| col-red | 红色 | hóng sè | 0.72s | -16.3 | |
| col-yellow | 黄色 | huáng sè | 0.75s | -12.1 | |
| col-blue | 蓝色 | lán sè | 0.72s | -15.3 | |
| col-green | 绿色 | lǜ sè | 0.83s | -11.7 | |
| col-purple | 紫色 | zǐ sè | 0.69s | -13.9 | |
| col-black | 黑色 | hēi sè | 0.83s | -13.7 | |
| col-white | 白色 | bái sè | 0.69s | -15.1 | |
| col-grey | 灰色 | huī sè | 0.72s | -16.3 | |
| col-orange | 橙色 | chéng sè | 0.79s | -17.7 | |
| col-pink | 粉色 | fěn sè | 0.71s | -13.8 | |
| col-light-blue | 浅蓝色 | qiǎn lán sè | 0.98s | -15.0 | |
| col-dark-green | 深绿色 | shēn lǜ sè | 1.06s | -13.3 | |
| col-block | 红色方块 | hóng sè fāng kuài | 1.17s | -12.9 | |
| col-all | 红色、黄色、蓝色、绿色 | hóng sè, huáng sè, lán sè, lǜ sè | 2.10s | -12.1 | |
| col-sentence | 已经找到红色方块，正在接近目标 | yǐ jīng zhǎo dào hóng sè fāng kuài, zhèng zài jiē jìn mù biāo | 3.85s | -11.5 | |
| col-none | 桌面上没有红色的方块 | zhuō miàn shàng méi yǒu hóng sè de fāng kuài | 2.31s | -7.0 | |
| rob-emoji | 好的 ✅ 马上执行 |  | 1.48s | -9.0 | |
| rob-emoji2 | 抓取完成 🎉 |  | 1.18s | -13.4 | |
| rob-markdown | **加粗** 与 `代码` 与 ~~删除~~ |  | 1.93s | -10.6 | |
| rob-json | {"intent": "pick_hold", "color": "red"} |  | 0.51s | -34.7 | |
| rob-symbols | 温度 25℃ 角度 90° 长度 3.5mm |  | 4.00s | -8.4 | |
| rob-latin | 使用 RGB-D 相机识别 YOLO 目标 |  | 2.54s | -9.3 | |
| rob-long-punct | 第一步，移动到位；第二步，下降抓取；第三步，抬升返回。 |  | 5.21s | -8.9 | |
| rob-quote | 他说：“好的”。 |  | 1.64s | -11.9 | |
| rob-space | 红 色 方 块 |  | 1.16s | -11.2 | |
| rob-newline | 第一行
第二行 |  | 1.59s | -12.8 | |

## vits-melo-tts-zh_en

- 采样率 44100 Hz，说话人数 1，使用 sid=0
- ⚠ 静音条目：num-4, num-6, num-9

| 条目 | 文本 | 应读作 | 音频 | 峰值 | 判定 |
|---|---|---|---|---:|---|
| num-0 | 零 | líng | 0.28s | -24.7 | |
| num-1 | 一 | yī | 0.26s | -36.1 | |
| num-2 | 二 | èr | 0.17s | -35.6 | |
| num-3 | 三 | sān | 0.31s | -26.2 | |
| num-4 | 四 | sì | 0.19s | -39.2 | |
| num-5 | 五 | wǔ | 0.36s | -29.9 | |
| num-6 | 六 | liù | 0.06s | -80.8 | |
| num-7 | 七 | qī | 0.18s | -32.6 | |
| num-8 | 八 | bā | 0.34s | -35.7 | |
| num-9 | 九 | jiǔ | 0.07s | -80.8 | |
| num-10 | 十 | shí | 0.29s | -25.2 | |
| num-21 | 二十一 | èr shí yī | 0.73s | -20.6 | |
| num-105 | 一百零五 | yī bǎi líng wǔ | 0.80s | -19.0 | |
| num-2024 | 二零二四 | èr líng èr sì (digit-by-digit) | 0.94s | -16.6 | |
| num-10000 | 一万 | yī wàn | 0.52s | -21.7 | |
| num-date | 2024年12月31号 | èr líng èr sì nián shí èr yuè sān shí yī hào | 2.08s | -10.8 | |
| num-police | 拨打110 | bō dǎ yāo yāo líng (幺幺零) | 0.92s | -17.5 | |
| num-phone | 一八九二零二四零五一一 | digit-by-digit yī bā jiǔ èr líng èr sì líng wǔ yī yī | 1.66s | -14.9 | |
| num-money | 123456块钱 | shí èr wàn sān qiān sì bǎi wǔ shí liù kuài qián | 2.29s | -16.2 | |
| num-decimal | 三点一四 | sān diǎn yī sì | 0.80s | -16.1 | |
| num-percent | 百分之八十五 | bǎi fēn zhī bā shí wǔ | 1.13s | -17.2 | |
| num-negative | 负十二度 | fù shí èr dù | 0.81s | -16.9 | |
| num-coord | 坐标X等于三百二十五，Y等于负一百二十 | zuò biāo X děng yú sān bǎi èr shí wǔ, Y děng yú fù yī bǎi èr shí | 3.34s | -13.9 | |
| num-mixed | 误差不超过0.5毫米，速度降到百分之三十 | wù chā bù chāo guò líng diǎn wǔ háo mǐ, sù dù jiàng dào bǎi fēn zhī sān shí | 3.51s | -13.1 | |
| num-code | 错误码1234 | cuò wù mǎ yī èr sān sì | 1.76s | -16.5 | |
| col-red | 红色 | hóng sè | 0.50s | -24.1 | |
| col-yellow | 黄色 | huáng sè | 0.65s | -23.5 | |
| col-blue | 蓝色 | lán sè | 0.46s | -23.3 | |
| col-green | 绿色 | lǜ sè | 0.57s | -22.6 | |
| col-purple | 紫色 | zǐ sè | 0.65s | -20.8 | |
| col-black | 黑色 | hēi sè | 0.51s | -21.5 | |
| col-white | 白色 | bái sè | 0.51s | -23.2 | |
| col-grey | 灰色 | huī sè | 0.53s | -21.3 | |
| col-orange | 橙色 | chéng sè | 0.33s | -37.7 | |
| col-pink | 粉色 | fěn sè | 0.42s | -33.2 | |
| col-light-blue | 浅蓝色 | qiǎn lán sè | 0.52s | -29.9 | |
| col-dark-green | 深绿色 | shēn lǜ sè | 0.69s | -22.0 | |
| col-block | 红色方块 | hóng sè fāng kuài | 1.08s | -15.4 | |
| col-all | 红色、黄色、蓝色、绿色 | hóng sè, huáng sè, lán sè, lǜ sè | 2.11s | -17.8 | |
| col-sentence | 已经找到红色方块，正在接近目标 | yǐ jīng zhǎo dào hóng sè fāng kuài, zhèng zài jiē jìn mù biāo | 2.89s | -13.9 | |
| col-none | 桌面上没有红色的方块 | zhuō miàn shàng méi yǒu hóng sè de fāng kuài | 1.85s | -17.1 | |
| rob-emoji | 好的 ✅ 马上执行 |  | 1.25s | -13.7 | |
| rob-emoji2 | 抓取完成 🎉 |  | 0.80s | -19.8 | |
| rob-markdown | **加粗** 与 `代码` 与 ~~删除~~ |  | 1.40s | -17.4 | |
| rob-json | {"intent": "pick_hold", "color": "red"} |  | 2.33s | -12.8 | |
| rob-symbols | 温度 25℃ 角度 90° 长度 3.5mm |  | 2.87s | -15.0 | |
| rob-latin | 使用 RGB-D 相机识别 YOLO 目标 |  | 2.67s | -13.0 | |
| rob-long-punct | 第一步，移动到位；第二步，下降抓取；第三步，抬升返回。 |  | 4.54s | -15.3 | |
| rob-quote | 他说：“好的”。 |  | 1.00s | -22.8 | |
| rob-space | 红 色 方 块 |  | 1.08s | -16.2 | |
| rob-newline | 第一行
第二行 |  | 1.14s | -11.6 | |

## vits-zh-hf-fanchen-C

- 采样率 16000 Hz，说话人数 187，使用 sid=0
- ⚠ 静音条目：num-2, rob-json

| 条目 | 文本 | 应读作 | 音频 | 峰值 | 判定 |
|---|---|---|---|---:|---|
| num-0 | 零 | líng | 0.69s | -21.6 | |
| num-1 | 一 | yī | 0.20s | -37.3 | |
| num-2 | 二 | èr | 0.24s | -38.4 | |
| num-3 | 三 | sān | 0.46s | -20.8 | |
| num-4 | 四 | sì | 0.35s | -25.9 | |
| num-5 | 五 | wǔ | 0.33s | -32.0 | |
| num-6 | 六 | liù | 0.63s | -19.0 | |
| num-7 | 七 | qī | 0.51s | -25.3 | |
| num-8 | 八 | bā | 0.61s | -16.2 | |
| num-9 | 九 | jiǔ | 0.40s | -21.1 | |
| num-10 | 十 | shí | 0.16s | -38.4 | |
| num-21 | 二十一 | èr shí yī | 1.24s | -13.4 | |
| num-105 | 一百零五 | yī bǎi líng wǔ | 1.59s | -12.5 | |
| num-2024 | 二零二四 | èr líng èr sì (digit-by-digit) | 1.26s | -13.0 | |
| num-10000 | 一万 | yī wàn | 0.75s | -14.9 | |
| num-date | 2024年12月31号 | èr líng èr sì nián shí èr yuè sān shí yī hào | 3.47s | -10.3 | |
| num-police | 拨打110 | bō dǎ yāo yāo líng (幺幺零) | 1.52s | -12.5 | |
| num-phone | 一八九二零二四零五一一 | digit-by-digit yī bā jiǔ èr líng èr sì líng wǔ yī yī | 4.00s | -14.7 | |
| num-money | 123456块钱 | shí èr wàn sān qiān sì bǎi wǔ shí liù kuài qián | 3.81s | -11.7 | |
| num-decimal | 三点一四 | sān diǎn yī sì | 1.40s | -15.7 | |
| num-percent | 百分之八十五 | bǎi fēn zhī bā shí wǔ | 1.88s | -9.2 | |
| num-negative | 负十二度 | fù shí èr dù | 1.13s | -12.8 | |
| num-coord | 坐标X等于三百二十五，Y等于负一百二十 | zuò biāo X děng yú sān bǎi èr shí wǔ, Y děng yú fù yī bǎi èr shí | 4.89s | -11.6 | |
| num-mixed | 误差不超过0.5毫米，速度降到百分之三十 | wù chā bù chāo guò líng diǎn wǔ háo mǐ, sù dù jiàng dào bǎi fēn zhī sān shí | 5.85s | -9.5 | |
| num-code | 错误码1234 | cuò wù mǎ yī èr sān sì | 3.43s | -12.6 | |
| col-red | 红色 | hóng sè | 0.83s | -16.4 | |
| col-yellow | 黄色 | huáng sè | 0.87s | -16.1 | |
| col-blue | 蓝色 | lán sè | 0.96s | -16.0 | |
| col-green | 绿色 | lǜ sè | 0.93s | -12.9 | |
| col-purple | 紫色 | zǐ sè | 0.95s | -13.8 | |
| col-black | 黑色 | hēi sè | 0.79s | -14.4 | |
| col-white | 白色 | bái sè | 0.76s | -14.7 | |
| col-grey | 灰色 | huī sè | 0.71s | -16.5 | |
| col-orange | 橙色 | chéng sè | 1.01s | -16.7 | |
| col-pink | 粉色 | fěn sè | 0.89s | -16.5 | |
| col-light-blue | 浅蓝色 | qiǎn lán sè | 1.13s | -16.9 | |
| col-dark-green | 深绿色 | shēn lǜ sè | 1.01s | -15.0 | |
| col-block | 红色方块 | hóng sè fāng kuài | 1.47s | -14.4 | |
| col-all | 红色、黄色、蓝色、绿色 | hóng sè, huáng sè, lán sè, lǜ sè | 2.84s | -12.3 | |
| col-sentence | 已经找到红色方块，正在接近目标 | yǐ jīng zhǎo dào hóng sè fāng kuài, zhèng zài jiē jìn mù biāo | 4.44s | -11.5 | |
| col-none | 桌面上没有红色的方块 | zhuō miàn shàng méi yǒu hóng sè de fāng kuài | 2.90s | -14.1 | |
| rob-emoji | 好的 ✅ 马上执行 |  | 1.85s | -10.0 | |
| rob-emoji2 | 抓取完成 🎉 |  | 1.47s | -10.6 | |
| rob-markdown | **加粗** 与 `代码` 与 ~~删除~~ |  | 2.41s | -13.4 | |
| rob-json | {"intent": "pick_hold", "color": "red"} |  | 0.12s | -42.9 | |
| rob-symbols | 温度 25℃ 角度 90° 长度 3.5mm |  | 4.32s | -12.5 | |
| rob-latin | 使用 RGB-D 相机识别 YOLO 目标 |  | 2.52s | -11.6 | |
| rob-long-punct | 第一步，移动到位；第二步，下降抓取；第三步，抬升返回。 |  | 6.76s | -10.3 | |
| rob-quote | 他说：“好的”。 |  | 1.18s | -13.5 | |
| rob-space | 红 色 方 块 |  | 1.34s | -15.7 | |
| rob-newline | 第一行
第二行 |  | 1.85s | -13.9 | |

