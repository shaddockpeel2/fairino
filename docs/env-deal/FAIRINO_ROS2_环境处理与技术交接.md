# FAIRINO ROS 2 环境处理与技术交接

> 文档日期：2026-09-07（Asia/Shanghai）  
> 工作区：`/home/industio/fairino_ws`  
> 目标平台：Industio RK3588，Ubuntu 22.04，AArch64  
> 目标机器人控制器：`192.168.58.2`，软件版本 V3.9.2

## 1. 交付结论

`fairino_ws` 已整理为独立、标准的 ROS 2 Humble colcon 工作区。官方
V3.9.2 ROS 2 源码已配套使用 ARM64 版 FAIRINO SDK，能够在当前 RK3588
开发板上完成全量编译、加载运行库、启动命令服务器并连接机器人控制器。

本次没有升级系统、替换系统 ROS、修改 shell 全局环境或改动其他 ROS 2
项目。`/home/industio/frcobot_ros2` 官方克隆保持不变。

当前结论可以概括为：**环境构建与通信链路已经交付，机器人运动功能尚未
达到安全验收条件。**

## 2. 当前完成进度与验证精度

下面的百分比表示任务完成度，不代表机器人定位精度或轨迹精度。

| 项目 | 完成度 | 状态 | 验证依据 |
| --- | ---: | --- | --- |
| 标准 colcon 工作区 | 100% | 已完成 | 根目录只包含两个源码包及标准 `build/install/log` 目录 |
| ROS 2 基础环境复用 | 100% | 已完成 | ROS 2 Humble 可用，未覆盖其他项目环境 |
| ros2_control 依赖 | 100% | 已完成 | 系统包 2.54.0，能够被 CMake 和运行时解析 |
| V3.9.2 源码匹配 | 100% | 已完成 | 使用官方 `V3.0.0_RobotV3.9.2` 对应源码 |
| ARM64 SDK 替换 | 100% | 已完成 | SDK 为 AArch64 ELF，工作区中 x86-64 ELF 数量为 0 |
| 全量干净编译 | 100% | 已完成 | 2 个包均成功完成编译和安装 |
| 动态链接验证 | 100% | 已完成 | 在 source 后执行 `ldd`，无 `not found` |
| 网络物理链路 | 100% | 已完成 | `eth1` 到 `192.168.58.2` ping 正常 |
| ROS 节点连接控制器 | 100% | 已完成 | 日志出现 `Robot connected!` |
| 只读 API 验证 | 100% | 已完成 | 已发现服务/话题并调用 `GetSDKVersion()` |
| 网卡地址持久化 | 100% | 已完成 | NetworkManager 已设置 `eth1` 静态地址和自动连接，断开/重连验证通过 |
| 运动 API 审核 | 100% | 已完成 | 5 条运动路径已增加严格参数校验，相关未初始化警告已消除 |
| 真机运动与精度验收 | 20% | 待现场确认 | 无故障/无急停/运动完成等读取预检通过；未在无人现场确认时下发有效运动 |

环境部署范围的完成度为 **100%**；若把现场运动安全、轨迹验证和精度标定纳入
整体交付，则当前仅能定义为“基础环境和只读通信阶段完成”，不能声明整机
运动功能验收完成。

## 3. 环境基线

### 3.1 操作系统和架构

```text
OS: Ubuntu 22.04.5 LTS
Kernel: 6.1.118
Architecture: aarch64
ROS_DISTRO: humble
```

不要直接将本机升级到 Ubuntu 24.04。当前 ROS 2 Humble 工作环境基于
Ubuntu 22.04 Jammy，系统跨版本升级会改变 ROS、Python、编译器和依赖库
组合，可能破坏现有项目。

### 3.2 ROS 与 ros2_control

```text
ros-humble-ros2-control: 2.54.0-1jammy.20260805.082815
ros-humble-ros2-controllers: 2.53.3-1jammy.20260805.002729
```

本机已有完整的 Humble 二进制安装，因此没有按照旧版 FAIRINO 文档再次
从源码编译 ros2_control。这样可以减少重复 overlay，也避免影响开发板上
现有 ROS 2 项目。

### 3.3 网络

```text
开发板接口: eth1
开发板地址: 192.168.58.10/24
控制器地址: 192.168.58.2
```

当前 ping 已验证成功。ROS 源码中的控制器地址位于：

- `src/fairino_hardware/include/fairino_hardware/data_type_def.h`
- `src/fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp`

`eth1` 已由 NetworkManager 持久化为 `192.168.58.10/24`，连接名为
`eth1`，并设为开机自动连接、不接管系统默认路由。IPv6 在该专用
控制链路上已禁用。手动断开并重新激活连接后，地址、路由及
`192.168.58.2` 的 ping 均恢复正常。

## 4. 工作区结构

```text
/home/industio/fairino_ws/
├── README.md
├── docs/
│   └── env-deal/
├── src/
│   ├── fairino_msgs/
│   └── fairino_hardware/
├── build/
├── install/
└── log/
```

包清单：

```text
fairino_hardware  src/fairino_hardware  (ros.ament_cmake)
fairino_msgs      src/fairino_msgs      (ros.ament_cmake)
```

`build/`、`install/`、`log/` 已加入根目录 `.gitignore`，避免误提交生成物。

## 5. 源码与 SDK 来源

### 5.1 ROS 2 包

机器人 V3.9.2 在 FAIRINO ROS 2 仓库主分支中没有独立的同名版本目录，实际
适配源码来自官方 Git 标签：

```text
Repository: https://github.com/FAIR-INNOVATION/frcobot_ros2.git
Tag: V3.0.0_RobotV3.9.2
Commit: b6de550dd695ab15c56c1ae5f700f499202a80d2
```

### 5.2 FAIRINO C++ SDK

官方 V3.9.2 ROS 包自带的 `libfairino.so.2.2.7` 和
`libfairino.so.2.3.2` 是 x86-64 文件，不能在 RK3588 上加载。本次使用对应
的官方 C++ SDK 源码进行 AArch64 本机构建：

```text
Repository: https://github.com/FAIR-INNOVATION/fairino-cpp-sdk.git
Tag: v2.3.2-robot3.9.2
Commit: 2ba16e2373a541d4880cb970f421bbd249a749c1
Output: libfairino.so.2.3.2
```

当前 SDK 文件：

```text
/home/industio/fairino_ws/src/fairino_hardware/libfairino/lib/libfairino.so.2.3.2
ELF: 64-bit LSB shared object, ARM aarch64
SHA-256: 2dfa828ce9d42281d600039ba4536b65a6e351b3370fad684b61adfbb3307915
SONAME: libfairino.so.2
```

工作区经过逐文件 ELF 检查，不再包含 x86-64 ELF。

## 6. 已实施的修改

### 6.1 架构替换

1. 删除 ROS 包中原有的 x86-64 FAIRINO 动态库。
2. 从官方对应 SDK 标签本机构建 ARM64 `libfairino.so.2.3.2`。
3. 将 ARM64 SDK 放入 `fairino_hardware/libfairino/lib/`。
4. 修改 CMake，使目标直接链接版本化 SDK 文件。
5. 安装阶段创建相对 SONAME 链接：

```text
libfairino.so -> libfairino.so.2
libfairino.so.2 -> libfairino.so.2.3.2
```

相对链接避免工作区移动后仍指向旧的绝对路径。

### 6.2 退出时内存损坏修复

上游 `robot_command_thread` 持有 `std::unique_ptr<FRRobot>`，但析构函数又
手工调用了 `_ptr_robot->~FRRobot()`，导致对象被析构两次。现已改成：

```cpp
robot_command_thread::~robot_command_thread()
{
    if (_ptr_robot) {
        _ptr_robot->CloseRPC();
    }
}
```

随后由 `unique_ptr` 自动且仅执行一次对象析构。修复前 SIGINT 退出曾出现：

```text
malloc(): mismatching next->prev_size
```

修复后多次 SIGINT 验证均能正常退出。

### 6.3 构建元数据修正

- 修正 `fairino_msgs/CMakeLists.txt` 接口声明缩进。
- 调整 `fairino_msgs/package.xml` 中测试依赖与接口组声明顺序。
- 将错误测试依赖 `ament_add_gmock` 改为 `ament_cmake_gmock`。
- SDK 链接不再依赖源码目录中的绝对路径符号链接。

没有对上游约一万行命令服务器代码做自动格式化，避免为了通过 style lint
产生大面积、难以审核的控制代码变更。

## 7. 构建和验收记录

### 7.1 干净构建

最终交付前已将旧 `build/install/log` 移出工作区，并从空构建目录执行：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

结果：

```text
Summary: 2 packages finished
```

`fairino_hardware` 编译过程仍有上游警告，详见第 10 节；这些警告没有阻止
编译和链接，但其中部分警告会影响运动调用的安全判断。

### 7.2 动态链接

在完整 source 环境后验证：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ldd install/fairino_hardware/lib/fairino_hardware/ros2_cmd_server | grep 'not found'
```

无输出，即没有缺失的运行时动态库。

### 7.3 实机连接

从最终的全新 `install/` 启动，记录到：

```text
Robot ip:192.168.58.2
fairino_hardware:V3.0.1
Adapt to software version of robot:V3.9.2,
Robot connected!
signal_handler(SIGINT/SIGTERM)
```

该验证只建立连接并等待，然后使用 SIGINT 退出，没有发送运动指令。

### 7.4 ROS 图和只读接口

连接成功后已观察到：

```text
Node:
  /fr_command_server

Service:
  /fairino_remote_command_service
  type: fairino_msgs/srv/RemoteCmdInterface

Topic:
  /nonrt_state_data
  type: fairino_msgs/msg/RobotNonrtState
```

只读 `GetSDKVersion()` 调用已返回：

```text
0,SDKVersion:83
```

此前直接查询控制器软件版本曾返回 V3.9.2 / V3.9.12-LA，和所选 ROS 包及
SDK 标签一致。

## 8. 日常操作

### 8.1 编译

建议每次使用干净 shell：

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

不要预先 source 其他项目的 `install/setup.bash`，除非明确需要建立 overlay。

### 8.2 启动

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fairino_hardware ros2_cmd_server
```

### 8.3 查看 ROS 接口

另开一个干净终端：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 node list
ros2 service list -t
ros2 topic list -t
ros2 interface show fairino_msgs/srv/RemoteCmdInterface
```

### 8.4 停止

在服务器终端按 `Ctrl+C`，确认出现：

```text
signal_handler(SIGINT/SIGTERM)
```

再检查没有残留进程：

```bash
pgrep -af ros2_cmd_server
```

## 9. 快速健康检查

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ip -brief address show eth1
ping -c 3 192.168.58.2
colcon list
file src/fairino_hardware/libfairino/lib/libfairino.so.2.3.2
ldd install/fairino_hardware/lib/fairino_hardware/ros2_cmd_server | grep 'not found'
ros2 pkg executables fairino_hardware
```

预期结果：

- `eth1` 为 `192.168.58.10/24` 或同网段的其他非冲突地址；
- ping 无丢包或仅有偶发网络丢包；
- `colcon list` 只列出两个 FAIRINO 包；
- SDK 架构显示 `ARM aarch64`；
- `ldd ... | grep 'not found'` 无输出；
- 可看到 `ros2_cmd_server` 和 `srv_test_client`。

## 10. 已知风险和未完成事项

### 10.1 已解决：运动命令参数安全审核

编译器原在以下上游命令路径报告参数可能未初始化：

- `MoveJ`：`user`、`tool`、`speed`；
- `MoveL`：`user`、`tool`、`speed`；
- `MoveC`：`user`、`tool`、`speed`；
- `SplinePTP`：`user`、`tool`、`speed`；
- `NewSplinePoint`：`user`、`tool`、`speed`、`lastflag`。

现已对上述路径实施以下修复：

- 参数数量不匹配时立即返回 `-1`，不调用 SDK 运动函数；
- `speed` 限定为 0–100，`tool`/`user` 限定为 0–14，`lastflag` 限定为 0/1；
- 拒绝 `JNT0`/`CART0` 及超出容器范围的点位索引；
- 数字参数必须完整解析，拒绝带尾随字符的值；
- 拒绝 `NaN`/无穷大等非有限扩展轴数值；
- 修复 `MoveC` 的局部变量遮蔽，确保 JNT 路径传入的是已取出的关节点；
- `MoveC` 使用自身的 `MoveC_blendR` 参数，不再误用 `MoveL_blendR`。

重新编译后，上述未初始化警告已全部消除；5 条非法命令的真机服务调用均返回
`-1`，过程中未下发有效运动。生产使用仍需完成现场低速运动与精度验收。

### 10.2 已解决：非 void 函数的无返回路径

`robot_command_thread::getVariable(std::string)` 的失败路径已统一返回命令协议现有的
`-1`，`control reaches end of non-void function` 警告已消除。

### 10.3 中优先级：官方 lint 不通过

启用 `BUILD_TESTING=ON` 后，上游代码会触发大量 `uncrustify`、
`lint_cmake` 等格式检查。一次记录为：

```text
56 tests, 0 errors, 32 failures, 14 skipped
```

这些主要是格式/风格失败，不是编译或运行失败。当前生产构建命令明确使用
`-DBUILD_TESTING=OFF`。后续若要建立 CI，应将第三方 SDK 头文件排除在格式
检查之外，并单独逐步修复本项目源码。

### 10.4 已解决：网卡地址持久化

已确认本机使用 NetworkManager，并仅修改专用机器人网口 `eth1`。配置为静态
`192.168.58.10/24`、自动连接优先级 100、`never-default=yes`。断开/重新激活验证通过，
未修改承载 SSH 的 `wlan0`。

### 10.5 尚未进行定位/轨迹精度验收

真机只读预检已完成：主/子错误码均为 0，`emg=0`，`motion_done=1`，
`collision_err=0`，两路安全停止状态均为 0。检查时机器人为使能状态
`rbt_enable_state=1`。

由于无法远程确认围栏内无人、急停可达、TCP/负载参数正确和验收轨迹无碰撞风险，
本次未下发任何有效运动命令。同时尚无外部测量工具数据，因此不能认定重复定位精度、
绝对定位精度、TCP 标定误差或轨迹误差已验收。

## 11. 故障排查

### 11.1 `Exec format error` 或加载到 x86 库

```bash
file /home/industio/fairino_ws/src/fairino_hardware/libfairino/lib/libfairino.so.2.3.2
file /home/industio/fairino_ws/install/fairino_hardware/lib/libfairino.so.2.3.2
```

两者都必须显示 `ARM aarch64`。若不是，停止运行，不要用 x86 文件覆盖当前
SDK。

### 11.2 `libfairino.so.2 => not found`

先确认 source 顺序：

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
```

再检查：

```bash
ls -l /home/industio/fairino_ws/install/fairino_hardware/lib/libfairino.so*
ldd /home/industio/fairino_ws/install/fairino_hardware/lib/fairino_hardware/ros2_cmd_server
```

不要把 SDK 复制到 `/usr/lib`，这会污染全局环境并可能影响其他程序。

### 11.3 一直显示连接失败

按顺序检查：

```bash
ip -brief address show eth1
ip route get 192.168.58.2
ping -c 3 192.168.58.2
nc -vz -w 2 192.168.58.2 8080
```

同时确认没有另一个 `ros2_cmd_server` 占用连接：

```bash
pgrep -af ros2_cmd_server
```

### 11.4 ROS 2 找不到包

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 pkg prefix fairino_hardware
```

如果仍找不到，重新执行第 8.1 节构建命令。避免在同一终端混合 source 多个
包含同名包的工作区。

### 11.5 修改后希望完全重编译

先不要直接删除。将旧生成目录移到明确的临时备份目录，然后重新构建：

```bash
cd /home/industio/fairino_ws
backup_dir="/tmp/fairino_ws_rebuild_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
mv build install log "$backup_dir"/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

## 12. 回退信息

本次整理前的工作区内容暂存于：

```text
/tmp/fairino_ws_legacy_JlTP51
```

SDK 源码构建目录暂存于：

```text
/tmp/fairino_cpp_sdk_v2_3_2
```

这些位于 `/tmp`，不应视为长期备份，可能在重启或系统清理时被删除。当前
正式工作区已经包含运行所需 ARM64 SDK，不依赖上述临时目录。

## 13. 建议的后续交接任务

下一位工程师应按以下顺序继续：

1. 为当前源码建立独立 Git 仓库或补丁集，记录相对官方标签的全部变更。
2. 在现场确认围栏、急停、人员撤离、TCP/负载和限速设置后，执行单轴/小范围低速运动测试。
3. 使用外部测量工具重复运行验收轨迹，记录重复定位、绝对定位、TCP 标定和轨迹误差。

## 14. 交接验收清单

- [x] Ubuntu 22.04 / AArch64 基线确认
- [x] ROS 2 Humble 可用
- [x] ros2_control 可用
- [x] 标准 colcon 工作区建立
- [x] FAIRINO V3.9.2 ROS 源码落位
- [x] x86 SDK 从正式工作区清除
- [x] ARM64 SDK 构建和 SHA-256 记录
- [x] 两个包全量干净编译成功
- [x] 运行时动态库完整
- [x] 控制器网络连通
- [x] 命令服务器连接成功
- [x] 只读 SDK 版本查询成功
- [x] SIGINT 双重析构问题修复并验证
- [x] 网卡静态地址持久化
- [x] 运动命令安全审计
- [ ] 真机低速运动验收
- [ ] 机器人定位和轨迹精度验收

---

交接时必须保留的核心判断：**当前系统已经具备 ARM64 环境下编译、启动、
连接和通信能力，且高风险运动命令的参数解析已审计；它尚未完成现场低速运动和
机械精度验收。**
