# PX4 + ROS 2 + Gazebo + Agent Experiment SOP

Version: v1.0  
Date: 2026-10-28  
Environment: Ubuntu 24.04.5 ARM64 VM, PX4 v1.17, Gazebo Harmonic 8.15.0, ROS 2 Jazzy, X500 SITL

## 1. Objective

Build a reproducible loop for bounded PX4 parameter experiments:

```text
Gazebo → PX4 estimation/control → MicroXRCEAgent → ROS 2 task and recorder
→ baseline data → Agent analysis → parameter candidate → replay → regression → conclusion
```

The first-stage question is whether an offline Agent can identify a control issue and propose a reversible PX4 parameter candidate. The Agent does not control the vehicle online, write parameters, change task code, change PX4 controller code, or change the Gazebo model.

### Fixed system roles

| Component | Role |
|---|---|
| Gazebo | Physics, sensors and simulation clock |
| PX4 | State estimation, mode management, controllers and actuators |
| MicroXRCEAgent | PX4–ROS 2 transport over UDP 8888 |
| ROS 2 | Task state machine, setpoints, subscriptions and CSV recording |
| QGroundControl | Ground monitoring only |
| Agent | Offline audit, diagnosis, candidate proposal and comparison |

### Subjects and representative parameters

| Subject | Flow | Main observations | First parameter scope |
|---|---|---|---|
| 1. Single-axis response | Takeoff 3 m → north +2 m → hold 5 s → land | Main-axis RMSE, overshoot, settling time, speed RMS | `MPC_XY_VEL_P_ACC`; later `MPC_XY_P` only with evidence |
| 2. Hover stability | Takeoff 3 m → hover 30 s → land | Position drift, position/velocity RMS, steady-state error | `MPC_XY_VEL_P_ACC`; `MPC_XY_VEL_I_ACC` only with persistent signed bias |
| 3. A/B mission | Takeoff → A → hold 5 s → B → land | Task completion, phase times, A/B error, landing reliability | Start with `MPC_XY_VEL_P_ACC`; open limits only when ULog proves saturation |

Every run has a unique `run_id` and its own directory containing ROS records, ULog, configuration, version, parameter and time metadata. Keep failed and `review_required` runs. Do not fill NaN with zero. Judge each metric separately: `usable`, `usable_with_caveats`, or `invalid_for_metric`.

## 2. Method and execution path

### 2.1 Preconditions

Run only one PX4, one Gazebo, one MicroXRCEAgent, one `/clock` bridge and one task node. Before starting, confirm the vehicle is landed and disarmed; QGroundControl Virtual Stick is off.

### 2.2 Four terminals

**Terminal 1 — PX4 + Gazebo**

```bash
unset GZ_CONFIG_PATH
export PATH=/usr/bin:/bin:$PATH
hash -r
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Keep the `pxh>` prompt open.

**Terminal 2 — PX4/ROS 2 bridge**

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — Gazebo clock bridge**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

**Terminal 4 — QGroundControl**

```bash
cd ~/Downloads
./QGroundControl-aarch64.AppImage
```

QGroundControl monitors status only; it does not issue task points or manual takeover commands.

### 2.3 Logger procedure for every run

At the PX4 `pxh>` prompt, before starting ROS:

```text
logger status
logger on
logger status
```

Start the ROS task only after logging is confirmed. After landing and disarm, press `Ctrl+C` in the ROS terminal, then return to `pxh>`:

```text
logger off
logger status
```

Archive the ULog only after `logger off`. Never select a ULog merely because it is the newest file when more than one log exists.

### 2.4 Task commands

**Subject 1 — single-axis response**

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/px4_tools/run_px4_capture.py single_axis \
  --output-root "$HOME/px4_experiments"
```

**Subject 2 — hover stability**

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/px4_tools/run_px4_capture.py hover \
  --output-root "$HOME/px4_experiments"
```

**Subject 3 — A/B mission**

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/px4_tools/run_px4_capture.py ab_mission \
  --output-root "$HOME/px4_experiments"
```

After each run, preserve `samples.csv`, `events.csv`, `result.txt`, `parameters.yaml`, `metadata.txt`, `px4_params.txt`, `px4_version.txt`, `snapshot_time.txt`, `flight.ulg`, `capture_manifest.json`, `capture.sha256` and the `ros/` directory.

### 2.5 Parameter change example

At `pxh>`, save and verify the current value before changing it:

```text
param show MPC_XY_VEL_P_ACC
param set MPC_XY_VEL_P_ACC 1.900
param show MPC_XY_VEL_P_ACC
param save
```

Rollback:

```text
param set MPC_XY_VEL_P_ACC 1.800
param show MPC_XY_VEL_P_ACC
param save
```

Change only the agreed candidate. Freeze task YAML, PX4/Gazebo versions, model, start sequence and scorer.

### 2.6 Experiment chain

```text
Gazebo physics/sensors → PX4 estimation and control → UDP 8888 bridge
→ ROS 2 task/recording → samples.csv + events.csv + flight.ulg
→ baseline audit → Agent diagnosis → human ground-side parameter change
→ same-condition replay → Subject 1/2/3 regression → report
```

The task node sends targets. PX4 decides how to execute them. Gazebo supplies the simulated response. ROS records feedback. The Agent is outside the real-time flight-control loop.

## 3. Experiment boundaries

| Boundary | Fixed condition |
|---|---|
| Wind | No wind or external disturbance |
| Temperature | Fixed Gazebo default |
| Battery | Same initial state; record voltage and remaining capacity |
| Motors | Four motors healthy; no injected fault |
| Initial position | Same local NED origin |
| Initial velocity | Zero |
| Initial attitude/rates | Level, zero angular rate |
| Heading | Fixed |
| Vehicle state | Landed and disarmed |
| QGroundControl | Virtual Stick off; no takeover |
| Task configuration | YAML, A/B coordinates and tolerances fixed |
| Software | PX4, ROS 2, Gazebo versions, commits and node build fixed |
| Data | One unique run directory; ROS records and ULog belong to the same run |

SITL CPU scheduling, communication jitter, timestamp issues, estimator resets, non-critical NaN values and preflight warnings are recorded as limitations. They do not automatically invalidate a run. Exclude only the affected metric when the issue breaks its target window or identity.

## 4. Subjects and parameter scope

| Subject | Representative parameters | Current values |
|---|---|---|
| 1. Single-axis response | `MPC_XY_P`, `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC` | `0.9500`, `1.800`, `0.400` |
| 2. Hover stability | `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC` | `1.800`, `0.400` |
| 3. A/B mission | First `MPC_XY_VEL_P_ACC`; then evidence-based `MPC_XY_P`, `MPC_XY_VEL_I_ACC`, `MPC_ACC_HOR_MAX`, `MPC_XY_VEL_MAX` | `1.800`, `0.9500`, `0.400`, `5.000`, `12.000` |

The first pass changes one candidate parameter at a time. Candidate and baseline groups should each contain at least 10 runs. A candidate must also pass Subject 2 and Subject 3 regression before being accepted as a general improvement.

## 5. Data interpretation rules

- Separate task metrics (completion, phase time, landing/disarm) from control-attribution metrics (tracking error, setpoint, limits, actuator evidence).
- `review_required` can contribute to a task metric when its target window remains valid; it is excluded from a clean-only mean.
- Missing A/B error, speed setpoint or limit evidence makes only that metric `unavailable`; do not delete the whole run.
- Do not use position CSV alone to claim a velocity-loop, acceleration-limit or actuator-saturation cause.
- Keep all failures, resets, NaNs, communication events and takeovers in the audit table.
- A successful single task or a single improved metric is not sufficient to accept a parameter.

## 6. Agent prompts

### 6.0 Common principle

```text
This project runs PX4 SITL in an Ubuntu VM. CPU scheduling, DDS/UDP transport and simulator timing can cause local freshness gaps, timestamp jitter, probe dropouts, non-critical NaN values, estimator resets and startup preflight warnings. Do not automatically invalidate the whole run.

Judge task metrics and control-attribution metrics separately. A review_required run may support a task metric when its target window is valid. Only mark the affected metric invalid when the issue breaks that metric window, task identity or parameter identity. Never replace NaN with zero. Write unavailable when evidence is missing.
```

### 6.1 Baseline construction prompt — `px4-baseline-agent-v1.2`

```text
You are the baseline-data assistant for three PX4 subjects: Subject 1 single-axis response, Subject 2 30 s hover, and Subject 3 A→DO_TASK→B mission. Read the ROS records, ULog, task configuration, parameter files and version metadata. Use PX4 local NED.

Output exactly one file: baseline_report.md. Do not create CSV, JSON, figures or extra files. Keep only:
1. a quality table with run_id, completion, data_quality, key anomaly, task-metric availability and attribution availability;
2. a subject-specific metric summary table;
3. a short limitations note;
4. one conclusion sentence.

Confirm subject, task, version, parameter identity and event windows first. Classify each metric as usable, usable_with_caveats or invalid_for_metric. Keep review_required runs when the target window is valid. Report Subject 1 RMSE/overshoot/settling/speed RMS; Subject 2 position RMS/drift/speed RMS/steady-state error; Subject 3 completion, phase time, A/B error, A-hold and landing. Do not propose parameter changes and do not invent missing values.
```

### 6.2 Optimization recommendation prompt — `px4-param-agent-v0.6`

```text
You are the offline PX4 control-performance assistant. Read the baseline report and experiment directories for Subject 1, 2 or 3.

Output exactly one file: optimization_recommendation.md. Keep only:
1. a problem table with subject, phase, observation, metric, evidence status and limitation;
2. a parameter table with parameter, current value, proposed value or “do not change”, control stage, evidence, risk, rollback value and next test;
3. a 3–5 sentence conclusion.

First verify task, version, node build, parameter identity and scorer. Inspect target/measured state, validity, timestamps, resets, mode, failsafe and takeover. Treat review_required as usable with caveats when the target window is valid. Do not reject all data because of non-critical SITL errors. Never invent a missing parameter snapshot. The Agent proposes only; it does not run param set and does not change code, task or model.

Parameter order: start with MPC_XY_VEL_P_ACC for Subjects 1/2 and A/B horizontal evidence in Subject 3. Consider MPC_XY_P only with position-loop evidence; MPC_XY_VEL_I_ACC only with persistent signed velocity bias; MPC_ACC_HOR_MAX or MPC_XY_VEL_MAX only when the ULog proves the limit was reached. Give a small reversible step or request targeted data when evidence is insufficient.
```

### 6.3 Comparison prompt — `px4-data-compare-v0.3`

```text
You are the PX4 replay-data comparison assistant. Compare baseline_runs and candidate_runs for the same subject using identical task, version, parameter records, event windows, fields and formulas.

Output exactly one file: comparison_report.md. Keep only:
1. a quality table with group, run count, task-metric count, control-metric count, failures and review_required count;
2. a metric comparison table with baseline, candidate, change, assessment and limitation;
3. a short missing-data/confounder note;
4. one result: improved, not_improved or inconclusive;
5. one next action: accept, rollback or collect more data.

Use valid review_required runs for task completion, phase time and landing when their windows are intact. Use only valid windows for A/B error, velocity-loop evidence, limits and parameter attribution. Keep unavailable metrics instead of deleting the run. Do not fill NaN, do not delete failed runs, and do not accept a parameter because one metric or one run improved. The overall result is inconclusive only when task identity, parameter identity, sample size or principal evidence cannot be compared.
```

## 7. Reports and repository files

| Subject | English report | Source report/data |
|---|---|---|
| Subject 1 | [subject1_report.md](subject1_report.md) | `figures/subject1/` in this folder |
| Subject 2 | [subject2_report.md](subject2_report.md) | `figures/subject2/` in this folder |
| Subject 3 | [subject3_report.md](subject3_report.md) | The corresponding Chinese baseline, comparison and analysis files in the parent project |

## 8. Final conclusion

| Subject | Result | Evidence |
|---|---|---|
| 1. Single-axis response | `provisional improved` | Main-axis RMSE, overshoot, settling time and speed RMS decreased; task completion stayed at 100%. Local to Subject 1 SITL. |
| 2. Hover stability | `inconclusive` | Several metrics decreased, but main-axis RMSE increased, settling time had no discrimination, and only 9 effective 1.900 runs were available. |
| 3. A/B mission | `not_improved` | Completion and landing were unchanged; A error increased about 36% while B error decreased about 20%, so both waypoints did not improve together. |

**Overall result: `not_improved`.** This experiment does not demonstrate that `MPC_XY_VEL_P_ACC=1.900` improves all three subjects. The parameter decision is to roll back to `1.800`. The Agent-assisted analysis, candidate, replay and regression loop is established, but the conclusion is limited to this PX4 SITL/Gazebo/ROS 2 configuration and must not be generalized to real hardware.
