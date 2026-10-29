# ROS 2 控制实验：单轴水平位置变化与悬停 30 s

版本：`build=2026-10-13-v2`。适用当前 Ubuntu 24.04.5 ARM64、ROS 2 Jazzy、PX4 v1.17、Gazebo Harmonic X500 SITL。新增包不覆盖 `px4_ab_mission` r8，不修改 PX4 控制参数。

## 本次搭建了什么

| 文件 | 作用 |
|---|---|
| `config/single_axis.yaml` | 科目 1：相对起点起飞 3 m，初始稳定后向北移动 2 m，到点后保持 5 s |
| `config/hover_30s.yaml` | 科目 2：相对起点起飞 3 m，初始稳定后保持同一目标 30 s |
| `src/control_experiment.cpp` | ROS 2 通信、读取参数、发布目标、保存记录和提前结束服务 |
| `include/px4_control_experiment/mission_logic.hpp` | 共用状态机、NED 坐标、计时与结束条件 |
| `test/mission_logic_test.cpp` | 不依赖 ROS 的任务逻辑检查 |

起点为启动条件满足时的 PX4 局部位置。高度目标为 `home.z - 3`，北向目标为 `home.x + 2`，其余轴目标保持不变。航向固定为启动时的估计航向。北向不是机头的“前方”。

科目 1 的输入是位置目标阶跃，PX4 负责具体飞行响应。科目 2 的 30 s 在起飞稳定、初始保持后开始，不包含起飞与降落；出现漂移不会重置该窗口。

## 1. 将完整包放入 Ubuntu 并编译（只做一次）

先把 `px4_control_experiment_2026-10-13-v2.zip` 传到 **Ubuntu 的 `~/Downloads/`**，以下全部命令在 Ubuntu 中执行。本包不需要重新安装 PX4、Gazebo、QGC 或 ROS 2。

```bash
mkdir -p ~/px4_ros2_ws/src
unzip -o ~/Downloads/px4_control_experiment_2026-10-13-v2.zip -d ~/px4_ros2_ws/src
ls ~/px4_ros2_ws/src/px4_control_experiment/src/control_experiment.cpp

cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 pkg prefix px4_msgs
colcon build --symlink-install \
  --packages-select px4_control_experiment \
  --cmake-clean-cache --cmake-clean-first \
  --event-handlers console_direct+
source ~/px4_ros2_ws/install/setup.bash
ros2 pkg prefix px4_control_experiment
strings ~/px4_ros2_ws/install/px4_control_experiment/lib/px4_control_experiment/control_experiment | grep 'control_experiment build='
```

最后应能看到 `build=2026-10-13-v2`。`unzip -o` 会更新同名 v2 文件；如果你改过源码，先备份再覆盖。若构建出现 error，保留完整首条错误，先不要启动任务。

可选逻辑检查：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
colcon test --packages-select px4_control_experiment --event-handlers console_direct+
colcon test-result --all --verbose
```

## 2. 准备仿真时钟桥（首次检查）

MicroXRCEAgent 桥接 PX4 数据；本包另外通过 `ros_gz_bridge` **单向读取 Gazebo 的 `/clock`**。这样可以按真实仿真进度计时，不假定 PX4 DDS 消息时间戳等于 Gazebo 时间。

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix ros_gz_bridge
```

如果返回包路径，直接进入启动步骤。如果明确显示 `Package not found`，只补装这个时钟桥：

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-bridge
```

如果 apt 找不到该包，先提供错误信息，暂不更改系统源或重装现有环境。没有有效 `/clock` 时任务只等待，不自动起飞。

## 3. 启动顺序：四个终端

每个终端只启动一份对应程序。同一架飞机只运行一个控制节点：关闭旧的 A/B 控制任务，关闭 QGC 虚拟摇杆。飞机应在地面且上锁；QGC 可观察，无需点击起飞。

### 终端 1：PX4 + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

出现 `pxh>` 后，可在这里读取 PX4 内部状态。

### 终端 2：PX4 ↔ ROS 2 通信桥

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

### 终端 3：Gazebo → ROS 2 仿真时钟

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

保持此终端运行；`[` 表示单向 Gazebo → ROS。每次只有一份时钟桥。任务节点不启用 ROS 的 `use_sim_time`，而是单独读取 `/clock` 来计算实验时长；ROS 命令时间戳沿用现有通信方式。

### 终端 4：先检查，再运行科目 1

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo --once /clock rosgraph_msgs/msg/Clock --qos-reliability best_effort
ros2 topic echo --once /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus --qos-reliability best_effort

ros2 run px4_control_experiment control_experiment --ros-args \
  --params-file ~/px4_ros2_ws/src/px4_control_experiment/config/single_axis.yaml
```

`echo --once` 收到一条数据便结束；如果一直没有数据，按 Ctrl+C 停止这个只读检查并检查对应桥，不要先运行飞行任务。预检未通过时，任务会打印 `preflight=no` 并等待。

科目 1 正常日志顺序：

```text
state -> PRIME_OFFBOARD
state -> TAKEOFF
requested Offboard/arm; awaiting status confirmation
state -> INITIAL_HOLD
state -> SINGLE_AXIS_RESPONSE
state -> HOLD
hold complete: ... sim seconds
state -> LANDING
landing handoff: stop Offboard heartbeat AND trajectory targets
state -> IDLE
RESULT: SUCCESS (landed and disarmed confirmed)
```

PX4 可能自动上锁，因此正常日志不一定出现节点发出的 disarm 请求；以新鲜的 PX4 `landed=true` 与 disarmed 状态共同确认。

### 科目 1 完成后，在终端 4 运行科目 2

先确认 `RESULT: SUCCESS`、飞机落地且上锁，再 Ctrl+C 结束已处于 IDLE 的节点。其余三个终端继续运行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run px4_control_experiment control_experiment --ros-args \
  --params-file ~/px4_ros2_ws/src/px4_control_experiment/config/hover_30s.yaml
```

科目 2 为 `PRIME_OFFBOARD → TAKEOFF → INITIAL_HOLD → HOLD（30 s 仿真时间）→ LANDING → IDLE`，没有单轴移动阶段。仿真慢时，30 s 仿真时间可能花费更久的现实时间。

## 4. 修改配置

先结束已落地上锁的节点，然后修改相应 YAML；下一次启动会重新读取。文件中的 `px4_control_experiment: / ros__parameters:` 缩进和节点名必须保留。

| 参数 | 当前值 | 意义 |
|---|---|---|
| `scenario` | `single_axis`／`hover` | 选择科目 |
| `axis` | `north` | 单轴科目可选 `north` 或 `east`；悬停科目忽略 |
| `displacement_m` | 科目 1 为 2.0 | 相对起点的位移，可负；悬停科目忽略 |
| `altitude_m` | 3.0 | 相对起点的飞行高度 |
| `initial_hold_s` | 2.0 | 起飞到点后，额外连续稳定保持的仿真时间 |
| `hold_s` | 科目 1 为 5.0；科目 2 为 30.0 | 正式保持窗口，按仿真时间 |
| `position_tolerance_m` | 0.3 | 三维目标距离容差 |
| `speed_tolerance_mps` | 0.3 | 到点稳定时允许的三维速度 |
| `stable_duration_s` | 1.0 | 起飞／单轴到点需连续满足容差的仿真时间 |
| `takeoff_timeout_s` | 45.0 | 仿真时间起飞超时 |
| `response_timeout_s` | 30.0 | 初始稳定／单轴响应各自的仿真超时 |
| `landing_timeout_s` | 60.0 | 仿真时间着陆诊断超时；不强制上锁 |
| `wall_stage_timeout_s` | 180.0 | 各阶段现实时间看门狗，处理仿真长期停顿 |
| `message_timeout_s` | 2.0 | 状态／时钟接收新鲜度阈值，按现实时间 |

程序使用启动时的配置，运行期间不支持调参生效；不要用 `ros2 param set` 改飞行中的科目。本包不会改 PX4 控制器增益。默认参数是首轮试运行值，不是已经完成控制优化的值。

v2 的启动门槛与 r8 对齐：仍要求时钟、位置、速度、状态、着陆、已上锁、有效航向和非 failsafe；`pre_flight_checks_pass` 只记录，不再阻止 SITL 科目启动。它不会绕过 PX4 自己的解锁检查：PX4 仍可能拒绝解锁，节点会记录失败并进入降落处理。

v2 也不再把估计器 reset 直接判为中止。reset 会写入 `events.csv`，并在 `result.txt` 标记 `estimator_reset_seen=true` 和 `data_quality=review_required`。这样优先保证完整采集，但 Agent 不能未经复核把这次数据当作干净调参证据。

## 5. 记录输出与本轮验收

默认每次创建一个独立目录：`~/px4_experiments/<scenario>_<运行编号>/`。启动日志会打印完整路径。

| 文件 | 内容 |
|---|---|
| `parameters.yaml` | 本次实际节点参数，可追溯科目配置 |
| `metadata.txt` | 程序版本、单位、坐标和时间域说明 |
| `samples.csv` | 墙钟经过时间、Gazebo 时间、PX4 消息时间戳、目标、位置／速度、模式、状态、新鲜度与坐标重置计数 |
| `events.csv` | 状态转换、请求、超时、接管与完成事件 |
| `result.txt` | 最终状态、飞行是否成功、记录状态、预检标记、估计器 reset 标记和数据质量状态 |

当前 CSV 是节点按现实时间约 20 Hz 记录的最新接收状态，会有重复消息时间戳；不是原始 IMU、逐条 ROS 消息或完整控制器输出。PX4 ULog 需另行保留。此包尚未实现正式指标和图表计算，`flight_success=true` 也不表示 IMU 数据质量已通过。

## 6. 给 Agent 的数据：当前有和还缺什么

Agent 要判断“这个工况该改什么参数”，不能只看一张飞行截图，也不能只看最终是否到点。每次运行至少要把以下内容作为一个实验包：

| 数据层 | 当前记录／需要补充 | Agent 能判断什么 |
|---|---|---|
| 工况定义 | `scenario`、轴、目标位移、高度、保持时长、坐标系、任务阶段 | 当前是在单轴响应、悬停，还是 A/B 回归；目标是什么 |
| 目标与实际状态 | `target_x/y/z/yaw`、实际 `x/y/z`、`vx/vy/vz`、有效性 | 跟踪误差、超调、稳定时间、漂移和阶段差异 |
| 飞控状态 | `arming_state`、`nav_state`、`landed`、`failsafe`、`preflight_ok` | 控制性能问题与模式／预检／故障问题的区分 |
| 时间与质量 | Gazebo `/clock`、ROS 接收时间、PX4 消息时间戳、消息新鲜度、reset 计数、events | 延迟、丢失、reset、异常窗口；是否可以作为调参证据 |
| 任务结果 | `flight_success`、完成阶段、超时、人工接管、降落结果 | 是否真正完成工况，失败原因是什么 |
| 参数上下文 | **当前节点只保存实验配置，不保存 PX4 控制参数**；每轮需另存 PX4 参数快照和源码版本 | 候选参数的当前值、变化幅度、回退点和可复现性 |
| 控制作用证据 | 当前没有完整的姿态／推力／执行器输出；需另存 PX4 ULog，后续再解析 | 判断误差来自位置目标、速度环、姿态／推力限制，避免盲目调参 |

三个科目的 Agent 关注点不同：

| 科目 | 主要问题 | 首要指标 | 首版 Agent 判断方向 |
|---|---|---|---|
| 单轴水平位置变化 | 阶跃后跟踪慢、超调大、振荡或稳态误差大 | 主轴误差 RMSE、最大超调、稳定时间、横向／高度耦合误差 | 先判断位置／速度外环参数是否与现象一致；一次只建议白名单内一个候选 |
| 悬停保持 30 s | 位置漂移、速度波动、低频摆动或控制目标抖动 | 三轴位置 RMS、速度 RMS、最大漂移、目标／状态延迟 | 判断是否为参数问题，还是传感器／时间质量问题；异常标记时不直接调参 |
| A/B 任务链 | 改参数后完整任务是否仍可靠 | 完成率、阶段耗时、A/B 误差、降落与上锁结果 | 作为回归门，不能仅凭 A/B 成功证明参数改善 |

因此当前数据链的实际用途是：**先让 Agent 识别工况和异常，再让它解释控制表现，最后才提出参数候选。** 它不能仅凭现有 CSV 直接证明某个 PX4 参数是根因。

参数快照建议在 PX4 `pxh>` 中对已经选定的白名单逐项执行并保存输出：

```text
param show <参数名1>
param show <参数名2>
param show <参数名3>
```

不要在尚未核对 PX4 v1.17 参数作用前凭名称猜参数。每个 `run_id` 同时保存参数值、源码 commit、任务配置和 PX4 ULog。

本轮验收：科目 1 只改变一个水平轴目标并完成流程；科目 2 连续保持至少 30 s 仿真时间；两者都落地上锁并出现 `RESULT: SUCCESS`，且生成完整记录文件。已知 IMU timeout／时间戳异常仍要保留，不能用本包宣称已经修复。

## 7. 提前结束和异常处理

飞行过程中需要提前结束，先在额外终端请求正常降落：

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 service call /px4_control_experiment/abort std_srvs/srv/Trigger '{}'
```

服务成功仅表示收到结束请求；继续观察 PX4，确认落地上锁后再退出。提前结束记为失败／中止实验，不伪装成科目成功。

若人工切换模式或 PX4 failsafe 接管，节点停止发布并进入 `RELEASED`，不抢回控制。此时由已接管的 PX4／QGC 完成处置。Ctrl+C 仅退出节点，不保证自动降落，不能当作飞行中的停止按钮。

| 输出／现象 | 先检查什么 |
|---|---|
| `waiting: clock=no` | 终端 3、ROS `/clock` 是否有数据；Gazebo 是否暂停 |
| `position=no`／`status=no`／`land=no` | 终端 2、工作区 source、实际话题名；本版本位置／状态带 `_v1`，着陆不带 |
| `preflight=no` | PX4 预检错误、电池健康、传感器状态；不要绕过预检 |
| 重复请求模式／解锁但未起飞 | 在 PX4 终端查看解锁拒绝原因与 `commander status`；请求发送不等于被接受 |
| `DATA FLAG: estimator reset observed` | 本次运行出现估计器坐标／航向 reset；任务继续采集，但本轮必须标为 `review_required`，检查估计器／仿真异常后再决定是否用于调参 |
| `RELEASE` | 模式切换、人工接管或 failsafe；节点不再发控制目标 |
| `LANDING TIMEOUT` | 节点继续观察，不强制上锁；用 PX4 `commander status` 和着陆话题检查实际状态 |

如果 `/clock` 不出现，在 Gazebo 已启动时检查实际时钟话题：

```bash
gz topic -l
```

若只有 `/world/default/clock`，关闭旧时钟桥，把终端 3 改成以下单向桥，并在终端 4 重新检查 `/clock`：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  --ros-args -r /world/default/clock:=/clock
```

## 检查范围

2026-10-13，本地纯 C++ 状态机检查通过，包括单轴／悬停完整流程、仿真时间计时、降落交接、接管、过期、坐标／时钟重置与超时；AddressSanitizer／UndefinedBehaviorSanitizer 未报告问题。YAML 字段、包 XML 和文档 Bash 命令语法检查通过。

开发机没有 ROS 2，完整节点尚未在 ROS 2 下编译；DDS 通信、Gazebo 时钟桥与 SITL 飞行均需要在 Ubuntu 验证。不要把本地逻辑检查通过标成科目已飞行验收。
