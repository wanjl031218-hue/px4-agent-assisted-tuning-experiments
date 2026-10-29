# PX4 Agent-Assisted Tuning Experiments

Can an AI agent use flight data to suggest PX4 parameter changes that improve flight performance? This project tests that question in simulation through a complete **collect → analyze → tune → retest → compare** workflow.

ROS 2 runs repeatable flight tasks, PX4 controls the vehicle, and Gazebo simulates its response. Flight data is recorded for an offline agent to analyze. A human reviews and applies the proposed parameter changes, then repeats the experiments to check whether performance actually improves.

**[中文文档](docs/zh/README.md) · [English documentation](docs/en/README.md)**

## Experiment workflow

| Step | Action | Responsibility and output |
|---|---|---|
| 1. Start the simulation | Launch PX4 SITL and Gazebo | PX4 runs estimation and flight control; Gazebo provides the X500 physics, sensors and simulation clock. |
| 2. Connect the system | Start MicroXRCEAgent on UDP 8888 and the Gazebo `/clock` bridge; open QGroundControl | MicroXRCEAgent carries PX4–ROS 2 messages. The clock bridge provides simulation time. QGroundControl monitors vehicle state and modes. |
| 3. Run a subject | Start one ROS 2 mission through [`run_px4_capture.py`](tools/run_px4_capture.py) | The mission node sends targets, observes PX4 feedback, and records samples, events and task results. |
| 4. Archive the flight | Enable the PX4 logger before the task; close it after landing and disarm | The capture script archives ROS records and the uniquely identified, closed ULog in one run directory. Parameter, version and snapshot-time records are saved separately. |
| 5. Build a baseline | Repeat the same subject with fixed settings | Audit the data and summarize per-run metrics using the baseline prompt in the SOP. Keep anomalies and document their effect on each metric. |
| 6. Request a candidate | Give the baseline, run data and optimization prompt to the analysis agent | The agent identifies a possible control issue and proposes a bounded parameter change, expected effect, risk and rollback value. |
| 7. Apply and retest | Review the proposal, change the parameter in PX4, read it back and repeat the subject | A human applies the change. Keep the task, environment and scoring rules fixed; collect the candidate runs. |
| 8. Compare and decide | Compare baseline and candidate metrics; run the other subjects as regression checks | Report `improved`, `not_improved` or `inconclusive`, then accept, roll back or collect targeted evidence. |

The analysis agent works offline. Parameter changes and flight execution remain separate, reviewable steps.

## Flight subjects

| Subject | ROS 2 node | Task | Main measurements |
|---|---|---|---|
| 1 — Single-axis response | [`px4_control_experiment`](tutorials/px4_control_experiment/) | Take off to 3 m → move 2 m north → hold 5 s → land and disarm | Main-axis RMSE, overshoot, settling time, speed RMS |
| 2 — Hover stability | [`px4_control_experiment`](tutorials/px4_control_experiment/) | Take off to 3 m → hover 30 simulation seconds → land and disarm | Position RMSE, drift, speed RMS, steady-state error |
| 3 — A/B mission | [`px4_ab_mission`](tutorials/px4_ab_mission/) | Take off → A → hold 5 s → B → land and disarm | Completion, phase durations, A/B arrival errors, landing reliability |

## Run the workflow

The experiments use **Ubuntu 24.04.5 ARM64 VM, PX4 v1.17, Gazebo Harmonic 8.15.0, ROS 2 Jazzy and X500 SITL**. PX4, Gazebo, ROS 2, matching `px4_msgs`, MicroXRCEAgent and QGroundControl must be installed before running the task packages.

Follow the [English SOP](docs/en/SOP.md#2-method-and-execution-path) or [中文 SOP](docs/zh/SOP.md#2-实验方法和路径) for the terminal commands, subject launches, logger operations, parameter changes and three analysis prompts. Each flight is stored in its own directory under `~/px4_experiments/`.

## Reports and findings

The first comparison tested **`MPC_XY_VEL_P_ACC: 1.800 → 1.900`**.

| Document | Chinese | English | Finding |
|---|---|---|---|
| Workflow and prompts | [SOP](docs/zh/SOP.md) | [SOP](docs/en/SOP.md) | Complete experiment procedure |
| Subject 1 | [Report](docs/zh/subject1_report.md) | [Report](docs/en/subject1_report.md) | Provisional improvement in the tested single-axis condition |
| Subject 2 | [Report](docs/zh/subject2_report.md) | [Report](docs/en/subject2_report.md) | Inconclusive: mixed metric directions and 9 effective candidate runs |
| Subject 3 | [Report](docs/zh/subject3_report.md) | [Report](docs/en/subject3_report.md) | Not improved: A-point error increased while B-point error decreased |

**The candidate was not accepted as an overall improvement; rollback to 1.800 is recommended.** The experiment establishes the analysis-and-retest workflow, but does not demonstrate consistent improvement or a tuning-efficiency advantage. Results are limited to the tested SITL conditions. English reports are concise editions; detailed per-run tables remain in the Chinese reports.

## Prepared Ubuntu VM for Apple Silicon Macs

The author's **Ubuntu 24.04.5 ARM64** experiment VM has been packaged for **Apple Silicon Macs using VMware Fusion**. Verified configuration: Fusion **13.6.4**, 5 virtual CPUs, 5,000 MB RAM and a 70 GiB virtual disk; PX4 **v1.17.0**, Gazebo **8.15.0** and ROS 2 **Jazzy**. This is not an Intel Mac or x86 PC image.

**Upload status: pending.** The 35.4 GB archive is split into 18 parts; no complete public VM download is available yet. VM attachments will be distributed through a GitHub Release. The [`vm-distribution` branch](https://github.com/wanjl031218-hue/px4-agent-assisted-tuning-experiments/tree/vm-distribution) stores the environment manifest and import instructions.

See the [English import guide](docs/en/VM_DISTRIBUTION.md) or [中文导入说明](docs/zh/VM_DISTRIBUTION.md) for verified versions, login, checksums, disk requirements and extraction commands. VM boot/login and archive integrity were verified; a fresh-import flight test remains pending.
