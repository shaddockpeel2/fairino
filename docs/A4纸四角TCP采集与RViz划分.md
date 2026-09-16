# A4 纸四角 TCP 采集与 RViz 四区显示

此工具只读取实时 TF，不发送任何机械臂运动命令。它记录夹爪尖端 `grasp_frame` 到达纸张四角时在 `base_link` 中的位置，默认沿 `base_link` 的 Z 方向把四点投影到 RViz XY 网格平面 `z=0`，拟合 210 × 297 mm 矩形并划分为四个等面积矩形。A/B/C/D 分别为左上、右上、右下、左下。保存的 JSON 中分别保留**原始 TCP 测量**、投影后的拟合结果、各区域顶点和中心坐标，单位均为米。投影只是可视化/分区坐标修正，不证明 TCP 实际接触了纸面。

## 准备

1. 将 A4 纸固定在工作台上。确定从纸面正面看四角的方向：`TL` 左上、`TR` 右上、`BR` 右下、`BL` 左下；整个采集过程保持这个命名。
2. 启动已有的**真实硬件** MoveIt/RViz 环境，确认 RViz 中机器人跟随真机、`base_link → grasp_frame` TF 正常且持续更新。已运行的 RViz 须重启，才能加载新增的 “A4 Paper Grid” 显示项。
3. 使用当前经过确认的夹爪 TCP。让 TCP 的局部 Z 轴沿纸面法线，指向上或下均可；四点采集期间保持相同工具朝向。**TCP 尖端轻触**四个角点，不要只悬停在纸面上方；不要让纸张移动。
4. 在终端运行 `source /opt/ros/humble/setup.bash` 和 `source /home/industio/fairino_ws/install/setup.bash`。如系统 ROS 发行版不是 Humble，按实际安装路径 source。

在另一个终端先启动 RViz 发布器（它会等待采样文件出现，并在每次采样后更新）：

```bash
ros2 run fr3c_calibration paper_grid serve --session /home/industio/fairino_ws/evidence/a4_paper_grid.json
```

手动将机械臂移到纸张 `TL` 左上角，TCP 稳定轻触该点，保持不动后运行：

```bash
ros2 run fr3c_calibration paper_grid capture --session /home/industio/fairino_ws/evidence/a4_paper_grid.json --corner TL
```

按同样方式依次移动并采集 `TR`、`BR`、`BL`，每次只把命令末尾的 `--corner` 改成相应名称。每次命令会输出采集的 TCP 坐标和剩余角点；第四点成功后输出四个区域的全部坐标，RViz 同步显示彩色轮廓和 A/B/C/D 标签。采集使用约 12 个不同时间戳的实时 TF 样本；TCP 抖动超过 2 mm、轴向变化超过 2°、TF 过期时不会保存。

采错某点时，移动到正确位置后追加 `--replace` 重新采集该角。程序会重新检查完整四角。四点需要形成近似 A4 矩形：宽/高分别在标称尺寸的 ±25 mm 内，拟合误差最多 12 mm，非共面误差最多 4 mm，TCP Z 轴在各点之间最多相差 8°，且与纸面法线偏差最多 12°。第四点不通过时，该点不会覆盖已有文件；检查纸张方向、TCP 接触位置和工具姿态后重采。

如果要**从 TL 开始全部重采**，先执行下列命令。它把已有文件（包括空文件或损坏的 JSON）归档为同目录下带时间戳的 `.backup-*.json`，然后建立空会话；运行中的 RViz 发布器会清除旧标记。

```bash
ros2 run fr3c_calibration paper_grid reset --session /home/industio/fairino_ws/evidence/a4_paper_grid.json
```

空文件不能直接用于 `capture`，因为它不是有效 JSON；`TL already captured` 表示原会话仍含 TL，此时单点重采应使用 `--replace`，全部重采则用 `reset`。

已采完的旧会话可直接重新投影，无须再次移动机械臂：

```bash
ros2 run fr3c_calibration paper_grid project --session /home/industio/fairino_ws/evidence/a4_paper_grid.json --plane-z 0
```

命令会先备份旧 JSON，再更新区域坐标和 RViz 标记。四角原始 `xyz_m` 不变。若 RViz 网格曾被手动设置到其他高度，`--plane-z` 应使用那个高度。此投影后的纸面法线为 `±base_link Z`；程序仍检查 TCP 局部 Z 轴与它近似平行。RViz 显示投影后的四角、四个半透明填充矩形和较大的 A/B/C/D 字符；现场颜色定义为 **A 蓝、B 黄、C 红、D 绿**。

`/paper_grid/markers` 是 `visualization_msgs/MarkerArray`，采用 transient-local QoS；已更新项目的 `moveit.rviz` 和 `moveit_camera.rviz`。若使用自定义 RViz 布局，手动添加 MarkerArray 显示项并将 Topic 设为 `/paper_grid/markers`，Fixed Frame 设为 `base_link`。

图中轮廓仅是可视化结果，不会自动加入 MoveIt 碰撞场景或驱动机械臂。纸张/桌面高度来自四角真实接触位置；如果 TCP 停在纸面上方，划分结果也会悬空。
