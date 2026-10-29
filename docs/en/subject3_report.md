# Subject 3 Parameter Replay Comparison Report

Candidate parameter: `MPC_XY_VEL_P_ACC` 1.800 → 1.900.  
Task: takeoff → A=(2,0) → hold 5 s → B=(2,2) → automatic landing and disarm. Altitude: 3 m. Acceptance radius: 0.30 m.

## 1. Data quality and scope

| Group | Runs | Task metrics usable | Parameter attribution usable | Failures | Main limitation |
|---|---:|---:|---:|---:|---|
| Baseline 1.8000 | 10 | 10 | 0 | 0 | Baseline ULog/task timeline and external parameter snapshots are incomplete |
| Candidate main set 1.9000 | 10 | 10 | 0 | 0 | Reset, freshness and preflight covariates remain |
| All candidate attempts | 15 | 15 | 0 | 0 | Sensitivity check only; not mixed into the main comparison |

Both groups completed the task and entered `IDLE`; all landings reported `landed=1`. No ROS/ULog failsafe or manual takeover was observed. A/B error windows were computable, but strict parameter attribution remains limited by ULog segmentation and missing complete parameter snapshots.

## 2. Metric comparison

| Metric | Baseline | Candidate | Change | Relative change | Assessment |
|---|---:|---:|---:|---:|---|
| Task completion | 10/10 | 10/10 | 0 | 0% | Unchanged |
| TAKEOFF → GO_TO_A (s) | 9.580 | 9.245 | -0.335 | -3.50% | Lower |
| GO_TO_A → DO_TASK (s) | 3.530 | 3.680 | +0.150 | +4.24% | Higher |
| DO_TASK (s) | 5.030 | 5.030 | ≈0 | ≈0% | Unchanged |
| GO_TO_B → LANDING (s) | 3.545 | 3.495 | -0.050 | -1.42% | Lower |
| LANDING → IDLE (s) | 6.170 | 6.180 | +0.010 | +0.16% | Unchanged |
| A 3-D position error (m) | 0.087 | 0.118 | +0.031 | +36.01% | Worse |
| B 3-D position error (m) | 0.082 | 0.066 | -0.016 | -19.76% | Better |
| A/B simultaneous improvement | 0.087 / 0.082 | 0.118 / 0.066 | +0.031 / -0.016 | +36.01% / -19.76% | **Not improved** |
| Landing result | 10/10 | 10/10 | 0 | 0% | Unchanged |
| Velocity tracking / limit evidence | unavailable | unavailable | unavailable | unavailable | Inconclusive |

## 3. Notes and limitations

Stage durations use `events.csv` `wall_elapsed_s`. A/B error uses the last valid PX4 local NED sample before each stage transition, with the declared freshness window. NaN values are not replaced with zero. Reset, preflight and freshness anomalies are retained as environment covariates and are not attributed to the gain.

A error increased by about 36% while B error decreased by about 20%; the two waypoints did not improve together. This is a task-performance result for this batch, not a statistically significant or strict causal parameter claim. No additional figure is necessary because the A/B and phase tables directly show the result without hiding the ULog/ROS timeline limitation.

## 4. Conclusion

**Result: `not_improved`.** `MPC_XY_VEL_P_ACC=1.900` preserved task completion and landing reliability but increased A-point error. Roll back to `1.800`; the rollback is recommended and was not executed by this report.
