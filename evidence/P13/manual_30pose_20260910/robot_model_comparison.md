# FR3-C 机器人运动学模型核对

## 结论

`/home/industio/franka_description/robots` 中的 `fr3`、`fr3v2` 等是 Franka Robotics
的七轴 FR3 模型，而现场机器人是法奥 FAIRINO FR3-C 六轴机器人。关节数、连杆尺寸和
坐标系定义均不同，这些 Franka URDF 不能用于本次 40 组数据。

控制器的实际 FK 与标准 FAIRINO `fairino3_v6` 六轴运动学链一致，与原项目使用的
`fairino3_mt_v6` 不一致。本项目不需要、也没有使用前 25 组数据拟合新 FK。

## 核对结果

| 候选模型 | 结果 |
|---|---|
| Franka `fr3` / `fr3v2` / `fr3v2_1` | 七轴 Franka 机器人，不适用 |
| FAIRINO `fairino3_mt_v6` | 3 个实测姿态的最佳固定 `wrist3_link → controller flange` 仍有 10.991 mm / 3.036° 离散，不兼容 |
| FAIRINO `FR3WML` | 长臂几何，与控制器位姿最大约 72 mm 离散，不兼容 |
| FAIRINO `FR3WMS` | 运动关系较接近，但最大约 0.355 mm 离散，不如标准 `fairino3_v6` 一致 |
| FAIRINO `fairino3_v6` | 与控制器 FK 一致；法兰为 `wrist3_link` 沿自身 +Z 方向 100 mm |

为兼容现有硬件接口的 `joint1`～`joint6` 命名，已从工作区的标准
`fairino3_v6.urdf` 生成仅替换 robot/joint 名称的
`src/fairino_description/urdf/fairino3c_v6.urdf`，并在组合 xacro 中显式增加
`wrist3_link → flange_link: xyz=[0, 0, 0.1] m`。

## 数值验证

- 3 个直接读取的现场停稳姿态：新 URDF `base_link → flange_link` 与控制器
  `GetActualToolFlangePose` 的平移误差中位数 0.00920 mm、最大 0.01062 mm；旋转误差
  中位数 0.000516°、最大 0.000792°。
- 40 组历史关节角重建姿态：平移误差中位数 0.00953 mm、最大 0.00981 mm；
  旋转误差中位数 0.000368°、最大 0.000443°。这 40 组中的关节角是从旧会话
  ROS 位姿反求得到，再通过法奥控制器官方 `GetForwardKin` 读取运动学真值；因此它是
  历史数据重建验证，不替代修正模型启动后的新现场交叉检查。
- MoveIt 离线模型检查通过：默认位与两个命名位无自碰撞，自碰撞和所有机械臂/工具
  连杆的障碍物碰撞均可检出。

## 40 组手眼数据的用法

前 25 组只用于求解固定 `controller flange → camera_link` 手眼变换，后 15 组用于
独立验证。使用控制器 FK 后，TSAI 结果的验证平移误差中位数为 2.274 mm、
P95 为 6.327 mm；旋转误差中位数为 0.418°、P95 为 0.825°。手眼数值已通过验收门限，
新 ROS 模型启动后已完成 3 个新姿态现场交叉检查，`flange_link` 与控制器法兰的
平移离散最大 0.004513 mm，旋转离散最大 0.000483°。最终
`flange_to_dabai_dcw2.yaml` 已标记 `calibrated: true` 和
`deployable_in_current_ros_tf: true`。
