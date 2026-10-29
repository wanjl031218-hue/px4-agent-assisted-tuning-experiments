# px4_ab_mission

当前版本：`build=2026-10-22-r9`。修正降落阶段仍发送 B 点悬停目标的问题，并补齐科目 3 的 ROS 记录。

这是今天的第一版自动任务节点。它把任务逻辑放在 ROS 2，把姿态、位置和电机控制留给 PX4：

```text
记录启动位置 home
  → 起飞到 home.z - 3 m
  → A = home + (2, 0, 0)，到 A 后稳定 1 s
  → DO_TASK：在 A 点悬停 5 s
  → B = home + (2, 2, 0)，到 B 后稳定 1 s
  → 发送 NAV_LAND
  → 检测 landed，发送 disarm
  → IDLE
```

坐标是 PX4 局部 NED：x 向北、y 向东、z 向下。因此上升 3 m 是 `z = home_z - 3`。A/B 的数字是相对任务启动点的偏移，避免依赖 Gazebo 当前世界原点。

## 在 Ubuntu 中部署

把本目录完整复制到 Ubuntu 的 ROS 2 工作区：

```bash
mkdir -p ~/px4_ros2_ws/src
cp -r ~/Desktop/quadrotor/tutorials/px4_ab_mission ~/px4_ros2_ws/src/
```

如果文件位于 Mac 而不是 Ubuntu，请用共享文件夹、拖放或压缩包把整个目录传入 Ubuntu，然后确认：

```bash
ls ~/px4_ros2_ws/src/px4_ab_mission/src/ab_mission.cpp
```

编译：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_ab_mission
source ~/px4_ros2_ws/install/setup.bash
```

## 启动顺序

必须先启动：

1. PX4 SITL + Gazebo；
2. `MicroXRCEAgent udp4 -p 8888`；
3. QGC 可打开观察，但不需要点击起飞；
4. ROS 2 节点。

启动节点时 X500 应已经处于地面、未解锁状态。节点会等待位置有效、`landed=true` 且 `armed_state=1`，然后才记录 home 并开始任务；如果当前飞机正在飞行，节点不会接管它。

ROS 2 节点的一条启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run px4_ab_mission ab_mission
```

这条命令会自动等待位置数据，然后自动解锁、进入 Offboard、起飞、执行 A→DO_TASK→B→降落。首版只允许在 SITL/Gazebo 测试，真实飞行前必须重新设计安全检查和人工接管。

## 修改 A/B 和任务参数

默认参数是 A=(+2, 0)、B=(+2, +2)、高度 3 m、任务 5 s。可以在同一条命令中覆盖：

```bash
ros2 run px4_ab_mission ab_mission --ros-args \
  -p a_x:=2.0 -p a_y:=0.0 \
  -p b_x:=2.0 -p b_y:=2.0 \
  -p altitude:=3.0 -p task_seconds:=5.0
```

用于实验采集时，使用统一脚本，让 ROS 记录和本次更新的 ULog 进入同一运行目录：

```bash
python3 ~/px4_tools/run_px4_capture.py ab_mission \
  --output-root "$HOME/px4_experiments"
```

任务回到 `IDLE` 后按 `Ctrl+C` 退出节点；脚本随后等待 PX4 关闭本次 ULog，再生成顶层的 `samples.csv`、`events.csv`、`result.txt`、`parameters.yaml`、`metadata.txt` 和 `flight.ulg`。ROS 原始记录仍保留在该运行目录的 `ros/` 下。

## 观测与停止

节点日志应按顺序出现：

```text
state -> TAKEOFF
state -> GO_TO_A
state -> DO_TASK: holding at A for 5.0 seconds
state -> GO_TO_B
B reached; requesting automatic landing
state -> LANDING
landing handoff: stopping Offboard heartbeat and trajectory setpoints; PX4 owns automatic descent
landing detected by VehicleLandDetected
requested disarm; waiting for PX4 confirmation
state -> IDLE
    任务已完成：已回到 IDLE，飞机已落地并上锁。
```

上锁只在 PX4 的 `VehicleLandDetected.landed=true` 后请求。仅仅位置接近地面或速度接近零不再作为上锁依据，因为 PX4 可能仍报告 `not landed`。

进入 `LANDING` 后，节点停止发布 `/fmu/in/trajectory_setpoint` 和 `/fmu/in/offboard_control_mode`，由 PX4 自动降落负责下降目标；节点继续订阅位置、着陆状态和解锁状态。PX4 内部自动模式与外部 Offboard 目标使用同一个 `trajectory_setpoint` 通道，旧版持续发布 B 点高度可能覆盖下降目标。此修订需要在 Ubuntu SITL 中实际复测，不代表 IMU 时间戳异常已解决。

30 秒阶段超时仍按电脑实际时间计算，超时后节点继续观察着陆状态，不强制上锁。如果仿真明显变慢，应另行检查仿真速度与超时计时的关系。

更新后，必须重新编译，并确认启动第一行包含 `build=2026-10-22-r9; ROS recording enabled`，只检查源码中的版本号不能证明运行了新程序。可检查已安装的可执行文件：

```bash
strings ~/px4_ros2_ws/install/px4_ab_mission/lib/px4_ab_mission/ab_mission | grep 'A/B mission build='
```

同一架仿真飞机每次只运行一个任务节点。确认着陆且上锁后，再进行下一次任务。连续三次观察到 A → DO_TASK → B → 降落 → IDLE，才把此问题标为复测通过。

只有节点先观察到局部 NED 高度相对 home 上升至少约 0.3 m，才把后续的 `landed=true` 作为正常降落确认。如果起飞阶段超时且飞机从未离地，节点会记录起飞失败并安全处理地面状态，不会伪造一次成功的降落流程。

同时可在另一个终端观察：

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo --once /fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus
```

按 `Ctrl+C` 只停止 ROS 2 任务节点。若任务正在飞行，先让它完成或在 QGC 中切回安全模式并降落，再停止节点。

## 这版的验收标准

- ROS 2 节点能够自动进入 Offboard 并解锁；
- X500 起飞到约 3 m；
- 位置相对任务启动点先到 A，再在 A 停留约 5 s；
- 再到 B；
- B 点触发自动降落，检测 landed 后上锁；
- 日志回到 `IDLE`；
- 终端日志中的 A/B 坐标与 `VehicleLocalPosition` 的 NED 坐标一致。

如果没有进入 Offboard、位置无效、阶段超过 30 s，节点会记录错误并尝试切入降落流程；首版仍需结合 PX4/QGC 状态共同判断，不把单条日志当作安全保证。
