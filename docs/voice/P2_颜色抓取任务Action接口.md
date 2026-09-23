# P2 颜色抓取任务 Action 接口

日期：2026-09-16  
范围：离线大模型语音抓取 10 阶段规划的第 2 阶段。阶段已于 2026-09-16 完成软件与代表性真机验收并关闭；完整结论见 [P2 颜色任务 Action 技术报告](FR3C离线语音抓取P2颜色任务Action技术报告-20260916.md)。

## 已实现行为

`fr3c_first_grasp` 现在同时提供五个 ROS 2 Action：

| Action | 行为 | 成功终态 |
|---|---|---|
| `/fr3c_first_grasp/pick_hold` | 指定颜色，接近、下降、闭爪、抬升 | `complete_holding`，保持夹持，不放置 |
| `/fr3c_first_grasp/pick_sort` | 指定颜色，抓取、按既有 A4 颜色映射放置、释放、返航 | `complete_sorted_home`，只处理本次指定颜色 |
| `/fr3c_first_grasp/pick_place` | 指定颜色 + 指定区域/槽位，抓取、放置、释放、返航 | `complete_sorted_home`，放置目标与颜色无关 |
| `/fr3c_first_grasp/pick_stack` | 指定上方颜色 + 下方颜色，抓取上方方块、对齐后码到下方方块顶面、释放、返航 | `complete_stacked_home`，只堆这一次，无论成败都回到起始关节位 |
| `/fr3c_first_grasp/pick_stack_auto` | **不带任何参数**：自动识别桌面现有颜色，按固定顺序一次码完整座塔 | `complete_stacked_home`，一条指令码完整座塔；任一层失败即停止并交还任务槽 |

五者使用同一个 `fairino_msgs/action/PickColor` 类型。Goal 为 `color`、`zone` 和 `support_color`：`color` 合法值为 `red`、`yellow`、`blue`、`green`；`zone` 只由 `pick_place` 使用，其余任务必须留空，否则返回 `INVALID_ZONE`；`support_color` 只由 `pick_stack` 使用，其余任务必须留空，否则返回 `INVALID_SUPPORT_COLOR`；`pick_stack_auto` 三个字段必须**全为空**，颜色顺序由机械臂端决定。Feedback 包含 `phase` 和 `detail`；Result 包含 `success`、稳定错误码、说明、颜色、实际使用的区域、实际使用的下方颜色和是否真实执行。

错误码包括：`INVALID_COLOR`、`BUSY`、`TARGET_NOT_FOUND`、`TARGET_NOT_UNIQUE`、`TARGET_STALE`、`PRECONDITION_FAILED`、`PLANNING_FAILED`、`EXECUTION_FAILED`、`CANCELED`、`HOLDING_OBJECT`、`PERCEPTION_UNAVAILABLE`、`INTERNAL_ERROR`、`INVALID_ZONE`、`ZONE_UNAVAILABLE`、`INVALID_SUPPORT_COLOR`、`STACK_GEOMETRY_REJECTED`。成功为 `OK=0`。

五个 Action 都只执行一次任务单元，不使用原 `/start_continuous` 的整批逻辑。Action 自动经过已经现场验证过的抓取阶段，但仍保留 P6、工作区、桌面、碰撞、完整轨迹、关节限制、到位和夹爪 Action 检查。闭爪仍是开环，因此成功信息只表示动作链完成，不声称传感器确认抓牢。唯一的例外是 `pick_stack_auto`：它内部按层依次调用 `pick_stack`，因此一条 goal 会包含多次“抓取→放置”，但**每一层仍然是同一条经过验证的单步路径**（见下文）。

`pick_hold` 成功后状态保持为 `complete_holding`，后续 Action 返回 `HOLDING_OBJECT`，防止软件在仍可能持物时直接覆盖场景。操作员已在安全位置真正松爪并移除／放稳物体后，调用以下接口同步 MoveIt 场景：

```bash
ros2 service call /fr3c_first_grasp/acknowledge_object_released \
  std_srvs/srv/Trigger '{}'
```

该确认接口不发送机械臂或夹爪命令；它清除先前附着物、把上次 Action 的单色感知过滤恢复为全颜色，并回到 `waiting_for_target`。若物体实际上仍在夹爪中，不得调用；下一次任务的预张开动作可能使物体掉落。

## pick_place 的区域与槽位

`pick_place` 把放置目标从“颜色固定映射”改成“调用方显式指定”。区域来自启动时读取的 A4 标定文件，因此**每个标定区域都是可寻址的**，不限于红→C、黄→B、蓝→A、绿→D 这四个。

`zone` 语法：

| 写法 | 含义 |
|---|---|
| `A` | 区域 A 中第一个未被占用的槽位 |
| `A3` | 区域 A 的第 3 个槽位（1 起算） |
| `a` / ` a ` | 大小写和空白不敏感 |

区域名来自标定文件 `grid.regions` 的键，启动时转成小写。槽位顺序沿用启动脚本生成的 2×2 序列：区域不足以安全容纳 2×2 时只有一个中心槽。当前 `evidence/a4_paper_grid.json` 提供 `a`、`b`、`c`、`d` 四个区域，每个 4 个槽位。

关键约束：

- **区域与颜色解耦**：`pick_place(red, zone=D)` 是合法请求，放置到 D 区。程序只校验区域存在、槽位可用、路径可规划，不校验颜色与区域是否"匹配"。
- **槽位占用来自实时场景**：某个槽位被判为已占用，是因为 MoveIt 场景里已有物体落在该槽位 25 mm（`zone_slot_clearance_m`）以内，而不是靠调用计数。这样操作员清理桌面并重启抓取栈后，占用状态自动恢复正确。当前目标和地面碰撞板不计入占用。
- **不回退**：区域不存在、槽位号超范围或槽位已占用时返回 `INVALID_ZONE` / `ZONE_UNAVAILABLE`，不会退回到该颜色的默认区域，也不会换一个槽位。
- **失败在运动之前**：区域解析和槽位分配在任何感知等待和机械臂运动之前完成，因此这类拒绝不产生任何运动。

`pick_hold` 与 `pick_sort` 的语义保持第二阶段现场验证时的原样，未做改动。

## 视觉过滤与快照约束

感知节点新增 `/perception/blue_cube_localizer/set_target_color`（`fairino_msgs/srv/SetTargetColor`）。每次 Action：

1. 设置请求颜色并取得单调递增的 `generation`。
2. 感知清空旧稳定窗口、旧位姿、旧尺寸和旧跳过记录。
3. 新 RGB-D 帧完成 HSV、深度、方块顶面和桌面约束验证。
4. 只在请求颜色恰好有一个三维有效目标时开始稳定窗口；0 个不回退到其他颜色，2 个及以上不任意选择。
5. 抓取节点只接受 `target_filter_color`、`filter_generation`、`color`、`sample_stamp`、Pose 和 Size 全部匹配且未超时的快照。
6. 接受后复制不可变快照，后续相机更新不改变本次目标。

原四色批处理不设置颜色过滤时，仍按红→黄→蓝→绿顺序工作。感知回调带过滤代次防护：旧代次中尚未处理完的帧不能覆盖新代次状态。

## 启动和仅规划示例

首次必须使用仅规划模式：

```bash
cd /home/industio/fairino_ws
FR3C_FIRST_GRASP_EXECUTE=false scripts/run_moveit_camera_rviz.sh detection_mode:=colors
```

`detection_mode:=colors` 会同时打开 `sort_enabled` 和 `stack_enabled`；也可以显式覆盖：`FR3C_STACK_ENABLED=false` 只关掉码垛，或 `FR3C_STACK_ENABLED=true` 单独打开。`stack_enabled` 依赖 `sort_enabled`（需要四色感知与方块几何），缺少时节点启动即拒绝。

另开终端：

```bash
cd /home/industio/fairino_ws
scripts/grasp/send_color_task.sh hold red
scripts/grasp/send_color_task.sh sort yellow
scripts/grasp/send_color_task.sh place blue A
scripts/grasp/send_color_task.sh place red D3
scripts/grasp/send_color_task.sh stack red blue     # 红色码到蓝色上面
```

等价的原生命令：

```bash
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_hold fairino_msgs/action/PickColor "{color: red}"
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_sort fairino_msgs/action/PickColor "{color: blue}"
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_place fairino_msgs/action/PickColor "{color: red, zone: D3}"
ros2 action send_goal --feedback \
  /fr3c_first_grasp/pick_stack fairino_msgs/action/PickColor \
  "{color: red, support_color: blue}"
```

先查看当前标定文件里有哪些区域和槽位：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 param get /fr3c_first_grasp place_zone_names
ros2 param get /fr3c_first_grasp place_zone_a_slots_m
ros2 topic echo /fr3c_first_grasp/state   # target_zone 显示本次实际使用的区域
```

执行模式沿用同一启动命令，将 `FR3C_FIRST_GRASP_EXECUTE` 改为 `true`。不得在未检查 RViz 完整路径、P6 门禁、区域标定和现场净空前启用。

## 验证记录

本次完成以下非运动验证：

- `fairino_msgs` 接口生成和 `fairino_mtc_demo` C++ 编译通过。
- `ros2 interface show` 可解析 `PickColor.action` 与 `SetTargetColor.srv`。
- 感知模块、跟踪窗口、区域映射和 P1 基线相关测试共 24 项通过。
- 工作区完整 Python 回归共 79 项通过；验证记录保存在 `evidence/voice/P2/20260916/software_verification.txt`。
- 新增测试覆盖“多色中只选指定色”“指定色不存在不回退”“同色两个目标拒绝唯一选择”。
- `python3 -m py_compile` 通过。

自动验证本身未发出机械臂或夹爪命令。其后现场完成了红色 `pick_hold`、红色 `pick_sort`、黄色 `pick_sort` 和两次蓝色 `pick_sort`，累计 5 次代表性成功运行；验收证据汇总见 `evidence/voice/P2/20260916/field_acceptance_summary.md`。

## 现场验收清单

依次在 `execute=false` 下验证：只有红色、多色含红色、没有红色、两个红色、稳定目标超过时效、任务忙；确认前两种只锁定红色，后三种分别返回预期错误且无运动。

再在低速受控现场用单块验证：

- `pick_hold(red)` 抬升后停住并保持夹持，不进入放置轨迹。
- 人工安全解除持物状态后，`pick_sort(red)` 只处理红色并返回起始关节位置。
- 人工释放后调用 `acknowledge_object_released`，确认状态回到 `waiting_for_target`，随后可连续接受下一条 Action。
- 执行中取消不会自动松爪、重试或转抓其他颜色。
- 快速重复发 Goal 不产生两次动作。

代表性主链路及连续 Action 状态恢复已有真机证据，第二阶段验收关闭。绿色真机循环、更多耐久循环和夹持传感器闭环作为后续整机验证／硬件增强项保留。

## pick_place 追加验证（2026-09-17）

`pick_place` 是第二阶段接口的后续扩展，不改变 `pick_hold` / `pick_sort` 已验证的行为。

- `fairino_msgs` 与 `fairino_mtc_demo` 重新编译通过；`ros2 interface show fairino_msgs/action/PickColor` 显示新增的 `zone` 字段与 `INVALID_ZONE`、`ZONE_UNAVAILABLE` 错误码。
- 加载真实 `evidence/a4_paper_grid.json` 后节点正常启动，`ros2 action list` 同时列出 `pick_hold`、`pick_sort`、`pick_place`。
- 区域参数校验可端到端失败：在标定文件中加入一个非颜色映射、中心 Z 偏离网格平面的区域 E 后，节点以 `invalid e placement zone center or grid plane Z` 拒绝启动（退出码 2）。该错误只可能由本次新增的 zone 校验产生，证明 launch→节点的区域参数通路有效。
- 工作区完整 Python 回归 195 项通过、4 项跳过；新增 `src/fairino_mtc_demo/test/test_place_zone.py`（区域语法接受/拒绝、三个 Action 的 `zone` 互斥、槽位占用读实时场景）与 `test_sort_regions.py` 的两个区域参数用例。
- 区域语法用生产代码编译成独立可执行文件测试，不接受空串、纯数字、`A0`、`A123`、`A-1`、`A1B`、`!`；解析失败时不写出半成品请求。
- 详细记录见 `evidence/voice/P2/20260916/pick_place_zone_software_verification.txt`。

未完成的验证：本沙箱内 DDS 只能完成发现、无法传输数据（`ros2 topic echo /fr3c_first_grasp/state` 与未改动的 `pick_hold` 目标同样无响应），因此**运行时目标往返未测**。真机／正常 DDS 环境下仍需按下面的清单补测。

### pick_place 现场验收清单

先只规划（`FR3C_FIRST_GRASP_EXECUTE=false`），逐条确认无运动且返回预期错误码：

- `zone` 留空 → `INVALID_ZONE`。
- `zone=Z` 或标定文件里不存在的区域 → `INVALID_ZONE`，消息列出可用区域。
- `zone=A9`（超出该区域槽位数）→ `ZONE_UNAVAILABLE`，无运动。
- 把一个方块预先放进 A1，再请求 `zone=A1` → `ZONE_UNAVAILABLE`；请求 `zone=A` 应自动选到 A2。
- 往已用于 `pick_sort` 的场景里再加一个 `pick_place`：确认此前放置的方块被归档为障碍且被正确判为占用。

再在执行模式下低速单块验证：

- `pick_place(red, zone=D)` 把红色方块放到 D 区，而不是红色默认的 C 区；`/fr3c_first_grasp/state` 的 `target_zone` 显示 `d1`。
- 连续两次 `pick_place(red, zone=A)` 依次落在 A1、A2，不重叠。
- `pick_place(red, zone=D)` 后重新观察，D 区方块被记为障碍，下次规划绕开。
- 取消、`HOLDING_OBJECT` 和快速重复发 Goal 的行为与 `pick_sort` 一致。

## pick_stack 堆叠码垛（2026-09-17 追加，2026-09-22 支持四层）

`pick_stack` 把“红色放在蓝色上面、黄色放在绿色上面”这类请求做成一次独立任务：抓取 `color` 方块，旋转对齐后放到 `support_color` 方块的顶面，释放并返航。

**一次请求只加一个方块**。塔上的方块就是下一层的 `support_color`，所以在同一个塔上再发一条 `pick_stack` 就能继续往上加：`pick_stack(blue, red)` → `pick_stack(green, blue)` → `pick_stack(yellow, green)`，三次请求得到四层塔（red 在最下、yellow 在最上）。默认上限 `maximum_stack_cubes: 4`，即塔最多四个方块；已经满四层时再请求，会以 `STACK_GEOMETRY_REJECTED` 在运动前拒绝。

### 与 pick_place 的区别

`pick_place` 把方块放到 A4 标定区的槽位；`pick_stack` 的放置目标**来自实时视觉测到的下方方块顶面**，不使用任何标定坐标。因此码垛位置随现场方块摆放变化，不需要预先标定“塔位”。

### 初始位姿一次锁定两个方块

机械臂处于初始位姿时，先设置待抓取的 `color` 过滤器，等该方块得到新鲜稳定位姿。同一条 RGB-D 状态里的 `detected_targets` 仍然包含当前画面内所有物理有效的彩色方块，因此 `pick_stack` 会一次性：

1. 确认画面中恰好有一个 `color` 和一个 `support_color`；缺失或同色多个时在运动前拒绝。
2. 从同一个 `sample_stamp` 的状态同时冻结两者的 XYZ、尺寸、偏航角和高度层级（`stack_tier`）。
3. 确认 `color` 是桌面上的 `stack_tier == 0` 方块、`support_color` 顶面没有别的方块、且这层塔加完不超过上限，然后才做 MoveIt2 整路径可达与碰撞规划。
4. 机械臂一旦开始运动，后续抓取、旋转、放置全部使用这一份不可变快照，**不回初始位、不重新识别下方方块**。

`support_color` **允许已经是塔顶**（`stack_tier > 0`），这正是继续加高的方式。相机只能看到塔顶那一个方块，塔身被挡住，所以除了塔顶方块本身，规划场景里还会补一个从桌面平面到塔顶方块底面的虚拟立柱障碍物（`stack_support_id` + `_column`），避免路径从塔身中间穿过去。下方方块仍在桌面时没有立柱，两层码垛的场景与之前完全一致。

### 旋转角度计算

方块顶面是正方形，因此“抓到方块时的抓取偏航角”加 90 度的任意整数倍都是同一个物理抓取。放置时要求上方方块顶面偏航角与下方方块一致，手腕需要额外旋转的角度为：

```
θ = wrap(下方方块偏航角 − 固定边缘偏置 − 本次抓取偏航角, 90°)
```

`wrap(·, 90°)` 取最近的等价角，即减去最接近的 90 度整数倍。等价抓取共有 4 个，彼此相差整 90 度，代码对 4 个都尝试完整轨迹规划，先取预计执行时间最短的一个，时长相同时再取关节空间路径较短的一个，因此**不需要预先知道方块朝向就能完成对齐**。

关键约束与实测边界：

- 正方形顶面每 90 度就物理等价。`wrap(·, 90°)` 取最近等价角后，手腕需要增加的对齐旋转不会超过半个对称步长，即 45 度（0.785398 rad）。
- `maximum_stack_placement_rotation_rad` 默认 `0.7853981633974483`（45 度）。超过该值的候选在**任何轨迹规划之前**被丢弃并记录事件；如果全部候选都没有完整无碰轨迹，任务在机械臂仍停在起始位时失败。
- 可以为更保守的现场减小该参数，但某些原始朝向组合会因所需旋转超限而在运动前被拒绝。

### 高度与姿态

- 放置高度：下方方块**实测顶面 Z** + `release_bottom_gap_m` + 方块高 − `grasp_depth_m`。因为附着方块中心比 TCP 高 `grasp_depth − 高度/2`，这样张爪时上方方块底面正好在下方块顶面上方 5 mm，然后自由落下约 5 mm 坐到下方方块上。
- 工具姿态仍是竖直向下（`grasp_orientation_xyzw` 不变），只是叠加了上一步算出的偏航旋转，因此放置旋转不会让工具轴倾斜。
- 接近点比放置点高 `stack_approach_clearance_m`（默认 30 mm），让夹着方块的手腕在旋转时有足够净空越过塔。
- 下方方块在整个任务期间作为规划障碍留在 MoveIt 场景中（`stack_support_id`，默认 `stack_support_cube`）。任务结束或取消后由下一个任务清除，不会泄漏给 `pick_sort`／`pick_place`。

### 最高四层，且不覆盖已有方块

- 待抓取的 `color` 必须是**落在地面平面上的**桌面方块（`stack_tier == 0`，底面与 `ground_plane_z_m` 的偏差不超过 `maximum_cube_plane_gap_m`）。塔上的方块只能当 `support_color`，不会被当成抓取目标。
- `support_color` 可以是桌面方块，也可以是塔顶方块，但**它自己顶面不能已经有方块**：感知对它所在水平 footprint（`stack_footprint_tolerance_m`）内、位置更高的方块会直接拒绝，报 `STACK_GEOMETRY_REJECTED`，不会悄悄码成比请求更高的塔。
- `support_color` 的层数受上限约束：`stack_tier + 1 >= maximum_stack_cubes`（默认 4）时拒绝，说明里给出实际层数和上限。
- `support_color` 报告的层级必须与它实测的底面高度自洽：塔顶方块底面应在桌面上方 `stack_tier × 方块高度` 处。因为每个真实方块的高度与标称值都有偏差、误差会随层数累加，这里比较的是“该层数隐含的单块平均高度”，容差 `stack_height_tolerance_m`（默认 12 mm），而不是一个绝对毫米窗口。
- 当 `support_color` 还在桌面（`stack_tier == 0`）时，两个方块的底面高度差必须落在 `stack_height_tolerance_m` 内，且水平 footprint 不得重叠。`support_color` 已经是塔顶时不再比较两者底面——它按定义就高一层或多层。

感知侧的对应配置是 `stacking_cube_height_offsets_m`：**每个已叠层需要一个高度偏移**，默认 `[0.030, 0.060, 0.090]` 覆盖到第 3 层（即四层塔的塔顶）。层高判定窗口用 `stacking_tier_tolerance_m`（默认 12 mm），比桌面的 `sorting_max_cube_plane_gap_m`（10 mm）宽，用来吸收上面提到的累加高度误差——否则真实方块略矮于标称时，四层塔的塔顶会直接不再被识别为方块。

### 无论成败都释放任务

这是本任务与其余任务最大的行为差异。`pick_stack` 承诺“一次请求、一次尝试、无论结果都把任务槽交还”，失败时：

- 已经产生运动：自动规划一条返航轨迹（`stack_failure_return_home`），把机械臂带回本次任务的起始关节位；**不会自动松爪、不会自动重试、不会换颜色**。
- 返航也失败：状态置为 `failed`，并在说明里明确要求人工检查现场后再发下一条。
- 尚未产生运动（目标不唯一、几何被拒、规划失败）：直接返回可重试的失败，机械臂原地不动。

收尾后的状态是 `complete_stacked_home`（成功）或可重试失败，两者都在 `stateAllowsStart` 白名单里，因此**下一条 Action 可以立刻发**，不需要任何人工复位。

唯一的例外：如果失败发生在闭爪之后，上方方块仍可能被夹在爪中，场景里的附着物也还在。此时下一条任务返回 `HOLDING_OBJECT`，必须先由操作员实际取下方块；这与 `pick_hold` 的持物保护一致，防止下一条任务的预张开动作把方块掉在桌上。

### pick_stack 现场验收清单

先只规划（`FR3C_FIRST_GRASP_EXECUTE=false`，且 `stack_enabled:=true`）：

- `support_color` 留空 → `INVALID_SUPPORT_COLOR`。
- `support_color` 非法值或与 `color` 相同 → `INVALID_SUPPORT_COLOR`，无运动。
- `pick_hold`／`pick_sort`／`pick_place` 带 `support_color` → `INVALID_SUPPORT_COLOR`，无运动。
- 视野里没有下方颜色方块 → `TARGET_NOT_FOUND`，无运动。
- 视野里有同色两个方块 → `TARGET_NOT_UNIQUE`，无运动。
- 指定的 `support_color` 顶面已经有一个方块 → `STACK_GEOMETRY_REJECTED`，无运动。
- 塔已经到 `maximum_stack_cubes`（默认四层）→ `STACK_GEOMETRY_REJECTED`，说明里给出实际层数和上限，无运动。
- 正常两方块：RViz 里确认下方方块作为障碍存在、完整路径包含“抓取→抬升→旋转→放置→退让→返航”七段。

再在执行模式下低速单块验证：

- `pick_stack(red, blue)`：红色方块落正在蓝色方块顶面，四边大致对齐；`/fr3c_first_grasp/state` 的 `support_color` 为 `blue`、`stack_support_tier` 为 `0`、`stack_placement_yaw_rad` 是本次实际旋转角。
- 完成后立即发第二条 `pick_stack`：应被接受并正常执行，不需要人工复位。
- 开始运动后不得移动或抽走下方方块：设计上刻意不做二次视觉重定位，任务会信任初始快照，现场必须保持两块方块静止。
- 执行中取消：返航逻辑同样生效；若已闭爪，下一条任务返回 `HOLDING_OBJECT`。
- 故意把两方块摆成同一朝向、再摆成相差 45 度：两种都能对齐，日志里 `stack_candidate_selected` 的 `placement_yaw_offset_rad` 应分别为接近 0 和接近 45 度。

### 四层塔验收（2026-09-22 追加）

默认 `maximum_stack_cubes: 4`，塔最多四个方块。在两层验证通过之后，用同一条链路继续加高：

- 依次发 `pick_stack(red, blue)`、`pick_stack(green, red)`、`pick_stack(yellow, green)`：三次请求得到蓝—红—绿—黄四层塔。
- 每一步的 `/fr3c_first_grasp/state`：`stack_support_tier` 依次为 `0`、`1`、`2`，`stack_lower_color` 分别是上一次放上去的颜色；成功说明里 “cube N of 4” 依次为 2、3、4。
- 第二步起 RViz 规划场景中除塔顶方块外还有 `<stack_support_id>_column` 立柱障碍，覆盖塔顶方块以下整根塔身；`stack_column` 只在 `stack_support_tier > 0` 时出现，两层码垛场景与改动前一致。
- 塔满四层后再发 `pick_stack(任意颜色, 顶层颜色)` → `STACK_GEOMETRY_REJECTED`，说明里给出实际层数与 `maximum_stack_cubes`，机械臂原地不动。
- 故意只发 `pick_stack(green, blue)`（跳过中间层，蓝色已被红色压住）：蓝色塔身被遮挡、不在 `detected_targets` 里 → `TARGET_NOT_FOUND`，无运动。这是预期行为：只能往塔顶加，不能插到中间。
- 只规划模式下把 `fr3c_camera/config/blue_cube_localizer.yaml` 的 `stacking_cube_height_offsets_m` 改短（例如只留 `[0.030]`）：三层塔的塔顶不再被识别为方块，第三次请求在目标获取阶段以 `TARGET_NOT_FOUND` 失败，不会带着错误高度去码垛。验收后请恢复三个偏移。
- 四层塔放好后，用 `/fr3c_first_grasp/state` 的 `stack_support_top_xyz_m` 与尺量结果对比，塔顶 Z 应约为 `4 × 方块高`（默认 120 mm）；偏差随层数累加，超过 `stacking_tier_tolerance_m` 时先查方块实际高度是否与 `cube_size_m` 标称一致。

## pick_stack_auto 自动码垛（2026-09-22 追加）

一条指令码完整座塔：`pick_stack_auto` 不带任何参数，节点自己看桌面上有什么颜色，按固定顺序把整座塔码完。

### 颜色顺序与缺色规则

顺序固定为 **红 → 黄 → 蓝 → 绿，自上而下**（红在最上面、绿在最下面），也就是从桌面往上依次放：绿、蓝、黄、红。

- 某种颜色桌面上没有 → 直接跳过，其余颜色保持相对顺序。
  - 四种都在 → 红/黄/蓝/绿 四层
  - 缺黄色 → 红/蓝/绿 三层
  - 缺黄、缺蓝 → 红/绿 两层
- 桌面上的颜色种类少于 2 → `TARGET_NOT_FOUND`（一个方块没法码垛），运动前拒绝。
- 桌面上的颜色种类多于 `maximum_stack_cubes` → `STACK_GEOMETRY_REJECTED`，说明里给出颜色数与上限；不会只码一部分。
- 同一种颜色在桌面上出现两个 → `TARGET_NOT_UNIQUE`（无法确定用哪一个），运动前拒绝。
- 顺序由 C++ 常量 `kAutoStackOrder` 固定，语音/上位机**不能**指定颜色或顺序。

### 用法

```bash
# 终端（新增 auto 模式）
scripts/grasp/send_color_task.sh auto

# 直接发 goal：三个字段必须都为空
ros2 action send_goal /fr3c_first_grasp/pick_stack_auto \
  fairino_msgs/action/PickColor "{color: '', zone: '', support_color: ''}" --feedback

# 语音/大模型：说“自动码垛”“自动堆叠”“一键码垛”“全自动码垛”“自动码放”等。
# 必须带“自动/一键/全自动”这类标记；只说“码垛/堆叠”仍然是原来的 pick_stack。
```

### 与其他任务的关系

- 塔的每一层都是**真正的内部 `pick_stack` goal**：`pick_stack_auto` 只是编排器，它通过私有 `/fr3c_first_grasp/internal/pick_stack` 端点按顺序发 `pick_stack(上层颜色, 下层颜色)`，因此每一层都走同一套几何校验、P6 门与碰撞规划；公开 `/pick_stack` 在整座塔完成前始终保持关闭，避免外部 goal 冒充内部步骤。
- 自动模式只在开始时采集一次四色坐标。后续源方块继续使用这份冻结坐标；每放好一层，只按已执行的放置几何在内存中抬高塔顶及层号，不重新切颜色过滤器、不等待相机重新识别塔顶。
- 中间层张爪后只退到塔顶安全位，直接从当前关节状态规划下一次抓取；最后一层才返回整项自动码垛开始前的关节位。首层已经开爪，后续层也不重复发送抓取前开爪命令。
- 四个等价抓取姿态按本层实际会执行的轨迹总时长选最短者（时长相同时再比关节路径长度）。高塔场景会把抓取后的垂直抬升自动提高到塔顶安全高度，避免第四块从低于三层塔顶的位置横穿而导致放置接近规划超时。
- 第二层开始，从上一层塔顶退让位到下一块源方块接近位使用带碰撞检查和关节跳变门限的笛卡尔直达轨迹；直达不完整就换下一个等价姿态，不执行先绕向远端 IK 构型再折返的自由空间轨迹。
- 自动码垛期间**整个任务槽被它占用**：其它任务的 goal 一律返回 `BUSY`，不会插到两层之间；塔结束（成功、失败或取消）后立即交还任务槽。
- 只加高、不改顺序：第一层放最下面的颜色，最后一层放红色。
- 失败语义：某一层失败就**停止**，不重试、不换颜色、不自动松爪。Result 的 `message`（中文）给出失败层号、失败原因、塔上现有层数（自下而上列出颜色）；`color`/`support_color` 分别是本次计划塔顶与塔底的颜色。已码好的部分留在原地，可人工用 `pick_stack` 继续。
- 取消：`pick_stack_auto` 的取消会转发给当前正在执行的那一层并立即停止后续层；Result 报 `CANCELED` 并说明已完成几步。非取消类的执行失败若触发自动返航，会使用整项自动码垛开始前保存的关节位。
- **只规划模式（`execute=false`）下拒绝**：不发真实运动时塔不会真的变高，第二层之后的支撑位置就不成立，所以直接返回 `PRECONDITION_FAILED`。预演请用 `send_color_task.sh stack <上> <下>` 逐对验证。

### 前置条件

- `stack_enabled:=true`（并因此 `sort_enabled:=true`、四色感知与 `use_target_yaw`/`use_target_size` 打开），与 `pick_stack` 相同。
- 建议桌面上每种颜色各一个方块；已有预置塔时，塔上的方块不会被当作取料来源（只统计 `stack_tier == 0` 的方块）。
- 感知配置 `stacking_cube_height_offsets_m` 必须覆盖到 `maximum_stack_cubes - 1` 层，否则塔顶不再被识别为方块，后续层会在目标获取阶段失败。

### 状态与反馈

- Feedback：`auto_stack_discovery`（正在识别桌面颜色）、`auto_stack_step`（第 k/N 步：某色放到某色上面，并转发该层 `pick_stack` 的进度）。
- `/fr3c_first_grasp/state` 新增 `auto_stack_active`、`auto_stack_order_top_to_bottom`；运行期间 `task_mode` 显示 `pick_stack_auto`。
- GUI：成功时直接显示这条动作自己的中文总结（而不是通用的一句“任务已实际完成：X 方块”），因为它描述的是整座塔。

### 自动码垛验收清单

先按“四层塔验收”确认单步 `pick_stack` 正常，再验自动码垛：

- 桌面摆好红黄蓝绿各一个 → goal 被接受，Feedback 先 `auto_stack_discovery`，随后 `auto_stack_step` 显示“第 1/3 步：蓝色放到绿色上面”；三层依次为 蓝→绿、黄→蓝、红→黄；最后 Result 成功，`message` 为“自动码垛完成：自上而下 红色、黄色、蓝色、绿色，共 4 层”。
- 拿走黄色再发 → 顺序自动变成 红/蓝/绿（第 1/2 步是 蓝色放到绿色上面），塔为三层。
- 只留一个颜色 → `TARGET_NOT_FOUND`，无运动。
- 同色摆两个 → `TARGET_NOT_UNIQUE`，无运动。
- 临时把 `maximum_stack_cubes` 改成 2 再发 → `STACK_GEOMETRY_REJECTED`，说明里给出颜色数与上限，无运动。
- 自动码垛运行期间另发一条 `pick_sort` 或 `pick_hold` → `BUSY`（说明为自动码垛正在码塔），无运动；塔结束后再发 → 正常接受，说明任务槽已交还。
- 中途取消 → 当前层返航，Result 为 `CANCELED`，说明里给出已完成步数；塔上已有的方块留在原处。
- `execute=false` 下发送 → `PRECONDITION_FAILED`，提示必须实际执行，无运动。

## 改动文件

`pick_stack` 追加（2026-09-17）涉及：

- `src/fairino_msgs/action/PickColor.action`（新增 `support_color` 与 `INVALID_SUPPORT_COLOR`、`STACK_GEOMETRY_REJECTED`）
- `src/fr3c_camera/fr3c_camera/cube_perception.py`（新增 `tabletop_cube_tier`，`is_tabletop_cube` 改为薄封装）
- `src/fr3c_camera/fr3c_camera/blue_cube_localizer.py`（感知层高分层、`detected_targets` 增加偏航角／尺寸／`stack_tier`）
- `src/fr3c_camera/config/blue_cube_localizer.yaml`（`stacking_cube_height_offsets_m`、`stacking_tier_tolerance_m`）
- `src/fr3c_camera/test/test_cube_perception.py`
- `src/fairino_mtc_demo/src/first_grasp_node.cpp`（`pick_stack` 动作、双快照、偏航对齐、返航保证）
- `src/fairino_mtc_demo/config/first_grasp.yaml`
- `src/fairino_mtc_demo/launch/first_grasp.launch.py`、`full_first_grasp.launch.py`
- `src/fairino_mtc_demo/test/test_stack_task.py`、`test_place_zone.py`
- `src/fairino_mtc_demo/CMakeLists.txt`
- `scripts/grasp/send_color_task.sh`、`scripts/run_moveit_camera_rviz.sh`
- `evidence/voice/P2/20260917/pick_stack_software_verification.txt`

四层支持（2026-09-22）改动：

- `src/fairino_mtc_demo/src/first_grasp_node.cpp`：新增 `maximum_stack_cubes` 参数（默认 4）、`stackSupportTierAcceptable()`（塔顶可继续加高、层数与实测高度自洽、超限拒绝）、`StackDestination.stack_tier`、`stackColumnObject()`／`stackColumnPresent()`／`stackColumnId()`（塔身虚拟立柱障碍），`validateStackPair()` 的同台面比较改为仅对 `stack_tier == 0` 生效，成功说明里给出“cube N of M”。
- `src/fairino_mtc_demo/config/first_grasp.yaml`：新增 `maximum_stack_cubes: 4`。
- `src/fr3c_camera/config/blue_cube_localizer.yaml`：`stacking_cube_height_offsets_m` 由 `[0.030]` 扩到 `[0.030, 0.060, 0.090]`。
- `src/fr3c_camera/fr3c_camera/blue_cube_localizer.py`：同上的默认值。
- `src/fr3c_camera/fr3c_camera/cube_perception.py`：`tabletop_cube_tier()` 的 `tier > 0` 判定改用 `stack_tolerance_m` 的对称窗口（原先只用桌面的 `maximum_plane_gap_m` 和 2 mm 穿透容差，四层塔的累加高度误差会让塔顶直接失效）。
- `src/fairino_mtc_demo/test/test_stack_task.py`、`test_source_height.py`、`src/fr3c_camera/test/test_cube_perception.py`。

四层支持的重新验证命令：

```bash
python3 -m pytest src/fairino_mtc_demo/test/ src/fr3c_camera/test/ -q
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF \
  --packages-select fairino_mtc_demo fr3c_camera
```

真机四层码垛尚未验证，现场按上面“四层塔验收”逐条走一遍。

自动码垛（2026-09-22）改动：

- `src/fairino_mtc_demo/src/first_grasp_node.cpp`：新增 `pick_stack_auto` Action 服务器（`createAutoStackServer`、`executeAutoStackAction`）、颜色发现与排序（`discoverAutoStackOrder`、`waitForColourInventory`、`kAutoStackOrder`）、逐步编排（`runAutoStackStep`，内部 Action 客户端通过私有端点复用 `pick_stack`）、并发互斥（`auto_stack_active_`／`auto_stack_running_`）、取消转发（`cancelInternalStackGoal`）、线程退出回收、中文结果与反馈。
- `src/fairino_mtc_demo/test/test_auto_stack_task.py`：新增，含编译运行顺序配对算术与全部契约断言。
- `src/fr3c_voice/fr3c_voice/intent.py`：新增 `pick_auto_stack` 任务、`AUTO_STACK_CUE`（必须带“自动/一键/全自动”标记）、`_has_action_cue`、语义与提示词规则。
- `src/fr3c_voice/fr3c_voice/action_bridge.py`：新增 `AUTO_STACK_ACTION`／`pick_auto_stack` 客户端，`send()` 支持三字段全空的 goal。
- `src/fr3c_voice/fr3c_voice/dialogue_gui.py`：自动码垛成功时显示该动作自己的中文总结。
- `src/fr3c_voice/test/test_intent.py`、`test_action_bridge_ros.py`、`test_dialogue_gui_contract.py`：新增用例。
- `config/voice/p4_intent_corpus.json`（`auto_stack` 类 7 条）、`config/voice/asr_hotwords.txt`。
- `scripts/grasp/send_color_task.sh`：新增 `auto` 模式。

自动码垛的重新验证命令：

```bash
cd /home/industio/fairino_ws
python3 -m pytest src/fr3c_voice/test/ src/fairino_mtc_demo/test/ -q
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF \
  --packages-select fairino_mtc_demo fr3c_camera fr3c_voice
ros2 launch fairino_mtc_demo full_first_grasp.launch.py \
  detection_mode:=colors sort_enabled:=true stack_enabled:=true
```

若 `~/.ros` 不可写（只读镜像），先设 `export ROS_LOG_DIR=$PWD/.roslog`（目录需存在），否则涉及 rclpy 的用例会因写日志失败而全部报错。

第二阶段原有改动：

- `src/fairino_msgs/action/PickColor.action`
- `src/fairino_msgs/srv/SetTargetColor.srv`
- `src/fairino_msgs/CMakeLists.txt`
- `src/fr3c_camera/fr3c_camera/blue_cube_localizer.py`
- `src/fr3c_camera/test/test_cube_perception.py`
- `src/fr3c_camera/package.xml`
- `src/fairino_mtc_demo/src/first_grasp_node.cpp`
- `src/fairino_mtc_demo/config/first_grasp.yaml`
- `src/fairino_mtc_demo/launch/first_grasp.launch.py`
- `src/fairino_mtc_demo/test/test_sort_regions.py`
- `src/fairino_mtc_demo/test/test_place_zone.py`
- `src/fairino_mtc_demo/CMakeLists.txt`
- `src/fairino_mtc_demo/package.xml`
- `src/fairino_mtc_demo/README.md`
- `scripts/grasp/send_color_task.sh`
- `evidence/voice/P2/20260916/software_verification.txt`
