# PX4 科目基准报告

## 1. 数据质量表

| 科目 | run_id | flight_success | data_quality | 主要原因 | reset | preflight | failsafe | 是否纳入基准 |
|---|---|---:|---|---|---|---|---|---|
| 科目3 | ab_mission_capture_20261023T070453Z_605b8b43 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 否 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T024211Z_0e45f9ea | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T070654Z_17becf08 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T070905Z_a56db4b4 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T082615Z_b49b4546 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 否 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T024657Z_b955a01d | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T082729Z_152707e2 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T071003Z_fd4ba379 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T073333Z_b40f8c43 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 否 | 否（ROS=0；ULog=0） | 是 |
| 科目3 | ab_mission_capture_20261023T025356Z_065c69f2 | true | clean_candidate | 环境性 reset/freshness 已按约定排除；任务链窗口可用 | 是 | 是 | 否（ROS=0；ULog=0） | 是 |

## 2. 指标汇总表

| 科目 | run_id | 完成 | TAKEOFF→GO_TO_A (s) | GO_TO_A→DO_TASK (s) | DO_TASK (s) | GO_TO_B→LANDING (s) | LANDING→IDLE (s) | A 误差 (m) | B 误差 (m) | 降落结果 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 3 | ab_mission_capture_20261023T070453Z_605b8b43 | 是 | 9.250 | 3.550 | 5.050 | 3.500 | 6.250 | 0.102 | 0.106 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T024211Z_0e45f9ea | 是 | 9.250 | 3.500 | 5.050 | 3.500 | 6.050 | 0.074 | 0.065 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T070654Z_17becf08 | 是 | 10.549 | 3.550 | 5.051 | 3.499 | 6.101 | 0.075 | 0.092 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T070905Z_a56db4b4 | 是 | 9.800 | 3.600 | 5.050 | 3.550 | 6.200 | 0.052 | 0.055 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T082615Z_b49b4546 | 是 | 9.150 | 3.450 | 5.000 | 3.550 | 6.100 | 0.074 | 0.109 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T024657Z_b955a01d | 是 | 9.099 | 3.502 | 5.048 | 3.550 | 6.302 | 0.108 | 0.070 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T082729Z_152707e2 | 是 | 9.200 | 3.500 | 5.000 | 3.750 | 6.201 | 0.100 | 0.068 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T071003Z_fd4ba379 | 是 | 9.350 | 3.550 | 5.050 | 3.500 | 6.350 | 0.085 | 0.079 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T073333Z_b40f8c43 | 是 | 10.101 | 3.548 | 5.000 | 3.550 | 6.150 | 0.096 | 0.108 | 是（landed=1，IDLE） |
| 3 | ab_mission_capture_20261023T025356Z_065c69f2 | 是 | 10.048 | 3.549 | 5.001 | 3.503 | 5.999 | 0.099 | 0.068 | 是（landed=1，IDLE） |
| 3 | 汇总 | 10/10（100%） | 9.580 (9.099–10.549) | 3.530 (3.450–3.600) | 5.030 (5.000–5.051) | 3.545 (3.499–3.750) | 6.170 (5.999–6.350) | 0.087 (0.052–0.108); n=10/10 | 0.082 (0.055–0.109); n=10/10 | 10/10 |

阶段耗时使用 events.csv 的 wall_elapsed_s（任务计时为 steady wall clock）。A/B 误差按本审计的 0–0.2 s freshness 判据，取状态切换前最后一条同时满足有限值、xy_valid=z_valid=1、sim_time_s 有效且 position timestamp freshness 在 0–0.2 s 的样本的三维 PX4 local NED 误差范数；不满足条件写 unavailable，未用 NaN 填 0。

## 3. 说明

本版将原指标汇总表第 1、3、4、5、7、8、9 条替换为 2026-10-23 07:04–08:29 采集的新包；10 个 capture_manifest.json 均为 scenario=ab_mission，任务配置 A=(2,0)、B=(2,2)、高度 3 m、DO_TASK=5 s、接受半径 0.30 m 一致，因此本批只支持科目 3。10/10 ROS 运行完成并进入 IDLE，10/10 capture.sha256 校验通过，最终 landed=1，ROS failsafe 全为 0，解析到的 ULog failsafe 也全为 0（但 ULog 未按飞行时间线匹配）；所有 ROS valid 字段均为 1，所有运行都有 estimator reset，除替换后的第 1、5、9 条外其余运行有运行级 preflight 标记。所有 ULog 版本均为 PX4 v1.17.0、同一 commit、关键主题存在且 dropout=0；选定 10 条中嵌入 ULog 的 MPC_XY_VEL_P_ACC 初始参数证据均为 1.800，但外部 PX4 参数快照未采集，且 ULog 仍未按飞行分段，因此该参数证据只作补充。替换的第 5 条和第 7 条均能在 A/B 状态切换前找到满足可靠窗口条件的样本；第 7 条 B 窗口末段 freshness 略超过 0.2 s，但仍有更早的可靠样本，因此未记为 unavailable。环境性 freshness、时间戳和 sim_time_s 异常仍按本次实验约定不作为基准指标；完成率、阶段耗时、A/B 可靠窗口误差和降落结果可用于同环境对比。

## 4. 结论

科目 3 基准可用：替换后任务完成率 100%，DO_TASK 平均 5.030 s，A/B 误差可用率均为 100%，降落结果 10/10；该结论适用于当前 SITL 环境和任务配置。
