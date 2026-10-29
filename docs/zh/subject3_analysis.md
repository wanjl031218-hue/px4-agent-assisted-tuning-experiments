# 科目 3 A/B 完整任务控制性能分析

## 1. 数据质量表

| run_id | flight_success | data_quality | reset | preflight | failsafe | 人工接管 | 关键文件是否齐全 | 是否允许用于参数判断 |
|---|---:|---|---|---|---|---|---|---|
| `ab_mission_capture_20261022T094811Z_aa723460` | true | `review_required` | 是（heading 1→2；xy/z 稳定） | 是（ULog 预检告警） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺 px4_params/version/snapshot_time；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T094949Z_17c3ca3c` | true | `review_required` | 是（heading 2→3；xy/z 稳定） | 是（ROS 起飞首样本） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T095514Z_6801ed9b` | true | `review_required` | 是（heading 1→2；xy/z 稳定） | 是（ULog 预检告警） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T095808Z_727a8115` | true | `review_required` | 是（heading 1→2；xy/z 稳定） | 否 | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T100054Z_43b27de7` | true | `review_required` | 是（heading 3→4；xy/z 稳定） | 是（ROS 起飞首样本） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T095945Z_51fd207d` | true | `review_required` | 是（heading 2→3；xy/z 稳定） | 是（ROS 起飞首样本） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T100254Z_f33b3ccc` | true | `review_required` | 是（heading 5→6；xy/z 稳定） | 是（ROS 起飞首样本） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T100813Z_5969fc15` | true | `review_required` | 是（heading 1→2；xy/z 稳定） | 否 | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T100934Z_e77dd1c2` | true | `review_required` | 是（heading 2→3；xy/z 稳定） | 是（ROS 起飞首样本） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |
| `ab_mission_capture_20261022T100151Z_7e6ac24c` | true | `review_required` | 是（heading 4→5；xy/z 稳定） | 是（ROS 起飞首样本） | 否（ROS/ULog 均为 0） | 否（user_took_over=0） | 部分（缺三份快照；ULog/ROS 时间线未完全证实） | 否 |

所有运行的 `capture_manifest.json` 均为 `scenario=ab_mission`，任务参数相同，build 为 `2026-10-22-r9`，`capture.sha256` 均通过。ROS 记录包含 `samples.csv/events.csv/parameters.yaml/metadata.txt`，但实际附件没有 `px4_params.txt`、`px4_version.txt`、`snapshot_time.txt`；采集清单还明确 ULog 可能含启动或其他飞行，不能仅凭文件存在证明与 ROS 是严格同一飞行时间线。

## 2. 阶段诊断表

| 阶段 | 首要现象 | 核心指标 | 对应 ROS/ULog 字段 | 数据限制 | 诊断结论 |
|---|---|---|---|---|---|
| TAKEOFF | 10/10 进入 `GO_TO_A`；切换前目标高度误差绝对值均值 0.182 m（0.171–0.196），`|vz|` 均值 0.092 m/s（0.075–0.109），起飞耗时均值 9.515 s（9.300–9.750） | `z-target_z`、`vz`、起飞耗时、armed/nav | `state,target_z,z,vz,arming_state,nav_state,preflight_ok,heading_reset`；ULog `vehicle_local_position`、`vehicle_status`、`estimator_status` | 起飞首样本存在 preflight 标记，所有运行有 heading reset；无独立 freshness 标志；切换已在任务容差内发生 | 没有可重复的起飞失败或模式切换故障；残余高度误差不足以开放 `MPC_Z_*` |
| GO_TO_A | A 到达误差均值 0.086 m（0.031–0.230），阶段耗时均值 3.485 s（3.398–3.600）；实测水平速度 RMS 均值 0.823 m/s（0.780–0.842） | A 三维到达误差、阶段耗时、实测水平速度 RMS；移动过程最大误差约 2 m 仅反映起点到 A 的路段 | ROS `target_x/y/z,x/y/z,vx,vy,speed,xy_valid,z_valid,v_xy_valid,v_z_valid`；ULog `vehicle_local_position_setpoint.vx/vy/acceleration[0/1]` | ROS 没有目标速度字段；移动目标的全过程误差不能直接当作超调；reset、时间戳 age 和 ULog/ROS 时间线限制归因 | 没有重复的 A 点位置或速度失稳证据；暂不开放 `MPC_XY_P_ACC` 或 `MPC_XY_P` |
| DO_TASK | A 点位置 RMS 均值 0.053 m（0.023–0.126），最大漂移均值 0.089 m（0.031–0.245），速度 RMS 均值 0.049 m/s（0.018–0.113），DO_TASK 时长 5.035 s（5.000–5.051） | A 点三轴位置 RMS、最大漂移、速度 RMS、带符号速度均值 | ROS `state=DO_TASK,target_*,x/y/z,vx/vy/vz,speed,valid,reset`；ULog `vehicle_local_position`、`vehicle_local_position_setpoint` | `vx/vy` 均值整体略负但跨运行幅度不一致，run1/run3/run8 有 0.109–0.157 s 间隔；run3 的时间戳异常在着陆段，不在该指标窗口 | 保持稳定且未超过 0.30 m 容差；没有足够的持续带符号偏差证据，暂不开放 `MPC_XY_VEL_I_ACC` |
| GO_TO_B | B 到达误差均值 0.053 m（0.021–0.103），阶段耗时均值 3.471 s（3.400–3.560）；实测水平速度 RMS 均值 0.828 m/s（0.820–0.835） | B 三维到达误差、阶段耗时、实测水平速度 RMS、到达前误差 | ROS `target_x/y/z,x/y/z,vx/vy/vz,speed,valid,nav_state`；ULog `vehicle_local_position_setpoint`、`vehicle_status` | ROS 没有目标速度；移动段的约 2 m 最大误差不是静态超调定义；部分 freshness age 异常，不能把速度波形归因于参数 | 没有重复的 B 点过冲或到达慢问题；ULog 任务段内水平加速度设定最大 3.694 m/s²、速度设定最大 1.921 m/s，未触及 5.0/12.0 限值，不开放 `MPC_XY_VEL_P_ACC` 或限幅参数 |
| LANDING | 10/10 记录 `landing detected`、`landed=1`、disarm 后回到 `IDLE`；末样本 `|vz|` 均值 0.009 m/s（0.000–0.030） | NAV_LAND 交接、着陆耗时均值 6.144 s（6.000–6.400）、末端 z/vz、最终 arming state | `events.csv` 的 `LANDING/IDLE`；ROS `z,vz,landed,arming_state,nav_state,failsafe`；ULog `vehicle_land_detected`、`vehicle_status`、`failsafe_flags` | Offboard signal lost 出现在预期的降落交接语境；ULog 可能含其他飞行，且没有独立版本/快照时间文件 | 降落交接和上锁可靠；不能把降落交接现象归因于水平参数，也不开放垂直参数 |

## 3. 参数建议表

| 参数 | 当前值 | 建议值 | 代表控制环节 | 证据 | 预期影响 | 风险 | 回退值 | 下一轮验证 |
|---|---:|---|---|---|---|---|---|---|
| `MPC_XY_VEL_P_ACC` | 1.900（仅 ULog embedded initial parameter；`px4_params.txt` 缺失） | **不修改** | 水平速度环 P | GO_TO_A/GO_TO_B 的实测水平速度 RMS 分别约 0.823/0.828 m/s，A/B 误差均值 0.086/0.053 m，任务 10/10 完成；ULog `vehicle_local_position_setpoint` 的水平加速度最大 3.694 m/s²、速度最大 1.921 m/s，未显示 5.0 m/s² 或 12.0 m/s 限幅触发；同时所有运行有 reset 或时间线/freshness 限制 | 不修改，不宣称有改善；当前数据不足以建立可归因的水平速度环问题 | 盲目增益变化可能改变 A/B 到达时间、过冲和 DO_TASK 稳定性，并掩盖数据质量问题 | 1.900（仅作为下一轮快照读回值，不能用缺失快照猜测） | 补齐每次运行的 `px4_params.txt`、`px4_version.txt`、`snapshot_time.txt`；确认 ULog 与 ROS 同一飞行且 freshness/reset 合格；候选与基线固定任务各至少 10 次，比较 GO_TO_A/GO_TO_B 速度 RMS、到达误差/耗时、全任务完成率、着陆上锁，并完成科目 1/2 回归 |

## 4. 结论

10 次任务均完成 A→DO_TASK→B→自动降落并上锁，GO_TO_A、DO_TASK、GO_TO_B 的阶段指标没有显示可重复的控制性能问题。保持 `MPC_XY_VEL_P_ACC=1.900` 及其余参数不变，当前不允许进入参数修改轮；现有数据只能支持阶段现象审查。先补测并补齐参数、版本、快照时间和严格的 ULog/ROS 同次运行证据，至少复测 10 次后再判断是否开放水平速度环候选。补测通过后还必须检查完整任务完成率、降落上锁、reset/freshness 和科目 1/2 回归。

候选结论：collect_more_data
