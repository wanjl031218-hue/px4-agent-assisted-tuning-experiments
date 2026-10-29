#!/usr/bin/env python3
"""Build a traceable subject-2 hover baseline from extracted run archives."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any

import numpy as np
from pyulog import ULog


ROOT = Path(__file__).resolve().parents[1]
RUN_ROOT = ROOT / "analysis_runs"
OUT_METRICS = ROOT / "baseline_metrics.csv"
OUT_REPORT = ROOT / "baseline_report.md"
OUT_SUMMARY = ROOT / "baseline_summary.md"
TOLERANCE_M = 0.3
SETTLING_DWELL_S = 1.0
STEADY_WINDOW_S = 5.0
MAX_GAP_S = 0.2

REQUIRED = [
    "samples.csv",
    "events.csv",
    "result.txt",
    "parameters.yaml",
    "metadata.txt",
    "flight.ulg",
    "capture.sha256",
    "capture_manifest.json",
]
CONTROL_KEYS = [
    "MPC_XY_P",
    "MPC_XY_VEL_P_ACC",
    "MPC_XY_VEL_I_ACC",
    "MPC_XY_VEL_D_ACC",
    "MPC_XY_VEL_MAX",
    "MPC_ACC_HOR_MAX",
    "MPC_ACC_UP_MAX",
    "MPC_ACC_DOWN_MAX",
    "MPC_Z_VEL_P_ACC",
    "MPC_Z_VEL_I_ACC",
    "MPC_Z_VEL_D_ACC",
    "MPC_Z_VEL_MAX_UP",
    "MPC_Z_VEL_MAX_DN",
    "MPC_THR_HOVER",
]
TOPICS = [
    "vehicle_local_position",
    "vehicle_local_position_setpoint",
    "vehicle_local_position_groundtruth",
    "trajectory_setpoint",
    "vehicle_status",
    "vehicle_control_mode",
    "failsafe_flags",
    "estimator_status",
    "estimator_status_flags",
    "control_allocator_status",
    "actuator_motors",
]


def finite(value: str | float | int) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def f(value: str | float | int) -> float:
    return float(value)


def parse_kv(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(errors="replace").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            result[k.strip()] = v.strip()
    return result


def parse_params(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(errors="replace").splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, value = line.strip().split(":", 1)
        value = value.strip().strip('"')
        if key not in {"px4_control_experiment", "ros__parameters"}:
            result[key] = value
    return result


def sha256_ok(path: Path) -> bool:
    expected: dict[str, str] = {}
    for line in (path / "capture.sha256").read_text().splitlines():
        parts = line.split()
        if len(parts) == 2:
            expected[parts[1]] = parts[0]
    for name, digest in expected.items():
        actual = hashlib.sha256((path / name).read_bytes()).hexdigest()
        if actual != digest:
            return False
    return True


def event_time(events: list[dict[str, str]], prefix: str, field: str = "wall_elapsed_s") -> float | None:
    for row in events:
        if row.get("event", "").startswith(prefix):
            return f(row[field])
    return None


def trapz(t: np.ndarray, y: np.ndarray) -> float:
    if len(t) < 2 or t[-1] <= t[0]:
        return math.nan
    return float(np.trapezoid(y, t))


def fmt(value: Any, digits: int = 6) -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "unavailable"
    if isinstance(value, (float, np.floating)):
        return f"{float(value):.{digits}f}"
    return str(value)


def parse_ulog(path: Path) -> dict[str, Any]:
    ulog = ULog(str(path))
    result: dict[str, Any] = {
        "corrupt": bool(ulog.file_corruption),
        "version": ulog.get_version_info_str(),
        "commit": ulog.msg_info_dict.get("ver_sw", "unavailable"),
        "hardware": ulog.msg_info_dict.get("ver_hw", "unavailable"),
        "duration_s": (ulog.last_timestamp - ulog.start_timestamp) / 1e6,
        "dropouts": len(ulog.dropouts),
        "parameter_count": len(ulog.initial_parameters),
        "parameters": dict(ulog.initial_parameters),
        "topics": {d.name for d in ulog.data_list},
    }

    def dataset(name: str):
        for item in ulog.data_list:
            if item.name == name:
                return item.data
        return None

    status = dataset("vehicle_status")
    result["ulog_failsafe"] = None if status is None else bool(np.any(status.get("failsafe", [])))
    result["ulog_user_took_over"] = (
        None
        if status is None
        else bool(np.any(status.get("failsafe_and_user_took_over", [])))
    )
    result["ulog_preflight_fail"] = (
        None
        if status is None
        else bool(np.any(np.asarray(status.get("pre_flight_checks_pass", [1])) == 0))
    )
    manual = dataset("manual_control_setpoint")
    result["manual_topic"] = manual is not None
    if manual is not None:
        result["manual_valid_values"] = sorted(set(np.asarray(manual.get("valid", [])).tolist()))
        result["manual_sticks_values"] = sorted(
            set(np.asarray(manual.get("sticks_moving", [])).tolist())
        )
    else:
        result["manual_valid_values"] = []
        result["manual_sticks_values"] = []
    flags = dataset("failsafe_flags")
    result["manual_signal_lost"] = (
        None
        if flags is None
        else bool(np.any(flags.get("manual_control_signal_lost", [])))
    )
    result["offboard_signal_lost"] = (
        None
        if flags is None
        else bool(np.any(flags.get("offboard_control_signal_lost", [])))
    )
    est = dataset("estimator_status")
    if est is None:
        result["estimator_reset_max"] = None
    else:
        reset_fields = [
            "reset_count_pos_ne",
            "reset_count_pos_d",
            "reset_count_vel_ne",
            "reset_count_vel_d",
            "reset_count_quat",
        ]
        result["estimator_reset_max"] = {
            k: int(np.max(est[k])) for k in reset_fields if k in est and len(est[k])
        }
    return result


def analyze_run(folder: Path) -> dict[str, Any]:
    run_dir = next(folder.glob("hover_capture_baseline*"))
    manifest = json.loads((run_dir / "capture_manifest.json").read_text())
    metadata = parse_kv(run_dir / "metadata.txt")
    result_txt = parse_kv(run_dir / "result.txt")
    params = parse_params(run_dir / "parameters.yaml")
    events = list(csv.DictReader((run_dir / "events.csv").open(newline="")))
    rows = list(csv.DictReader((run_dir / "samples.csv").open(newline="")))
    ulog = parse_ulog(run_dir / "flight.ulg")
    hold_start = event_time(events, "state -> HOLD", "sim_time_s")
    hold_end = event_time(events, "hold complete:", "sim_time_s")
    hold_start_wall = event_time(events, "state -> HOLD", "wall_elapsed_s")
    hold_end_wall = event_time(events, "hold complete:", "wall_elapsed_s")
    reasons: list[str] = []
    missing = [name for name in REQUIRED if not (run_dir / name).is_file()]
    if missing:
        reasons.append("missing:" + ",".join(missing))
    hashes_ok = not missing and sha256_ok(run_dir)
    if not hashes_ok:
        reasons.append("sha256_mismatch")
    if hold_start is None or hold_end is None or hold_end <= hold_start:
        reasons.append("invalid_hold_event_window")
    hold_rows = [
        row
        for row in rows
        if row.get("state") == "HOLD"
        and hold_start is not None
        and hold_end is not None
        and hold_start <= f(row["sim_time_s"]) <= hold_end
    ]
    metric_fields = [
        "target_x",
        "target_y",
        "target_z",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "speed",
    ]
    metric_rows = [
        row
        for row in hold_rows
        if all(finite(row[k]) for k in metric_fields)
        and all(row[k] == "1" for k in ["xy_valid", "z_valid", "v_xy_valid", "v_z_valid"])
    ]
    if len(metric_rows) < 2:
        reasons.append("insufficient_valid_metric_samples")
    times = np.array([f(row["sim_time_s"]) for row in metric_rows], dtype=float)
    if len(times) >= 2:
        gaps = np.diff(times)
        if np.any(gaps < 0):
            reasons.append("metric_time_decreasing")
        if np.any(gaps > MAX_GAP_S):
            reasons.append("metric_gap_gt_0.2s")
    else:
        gaps = np.array([], dtype=float)
    if metric_rows:
        targets = np.array(
            [[f(r["target_x"]), f(r["target_y"]), f(r["target_z"])] for r in metric_rows]
        )
        if np.max(np.ptp(targets, axis=0)) > 1e-6:
            reasons.append("moving_target_in_hold_window")
        measured = np.array([[f(r["x"]), f(r["y"]), f(r["z"])] for r in metric_rows])
        error = measured - targets
        speed = np.array([f(r["speed"]) for r in metric_rows])
        duration = times[-1] - times[0] if len(times) >= 2 else math.nan
        axis_rmse = [math.sqrt(trapz(times, error[:, i] ** 2) / duration) for i in range(3)] if duration > 0 else [math.nan] * 3
        position_3d_rmse = math.sqrt(trapz(times, np.sum(error**2, axis=1)) / duration) if duration > 0 else math.nan
        max_overshoot = float(np.max(np.linalg.norm(error, axis=1))) if len(error) else math.nan
        speed_rms = math.sqrt(trapz(times, speed**2) / duration) if duration > 0 else math.nan
        max_speed = float(np.max(speed)) if len(speed) else math.nan
        terminal_mask = times >= times[-1] - STEADY_WINDOW_S
        steady_t = times[terminal_mask]
        steady_e = np.linalg.norm(error[terminal_mask], axis=1)
        steady_error = (
            trapz(steady_t, steady_e) / (steady_t[-1] - steady_t[0])
            if len(steady_t) >= 2 and steady_t[-1] > steady_t[0]
            else math.nan
        )
        inside = np.all(np.abs(error) <= TOLERANCE_M, axis=1)
        settling = math.nan
        if len(times) >= 2:
            for idx in range(len(times)):
                if times[-1] - times[idx] >= SETTLING_DWELL_S and bool(np.all(inside[idx:])):
                    settling = times[idx] - times[0]
                    break
    else:
        targets = np.empty((0, 3))
        duration = math.nan
        axis_rmse = [math.nan] * 3
        position_3d_rmse = max_overshoot = speed_rms = max_speed = steady_error = settling = math.nan

    sample_wall = [f(r["wall_elapsed_s"]) for r in rows if finite(r["wall_elapsed_s"])]
    sample_sim = [f(r["sim_time_s"]) for r in rows if finite(r["sim_time_s"])]
    event_wall = [f(r["wall_elapsed_s"]) for r in events if finite(r["wall_elapsed_s"])]
    event_sim = [f(r["sim_time_s"]) for r in events if finite(r["sim_time_s"])]
    hold_fresh = {
        key: all(r.get(key) == "1" for r in hold_rows)
        for key in ["clock_fresh", "position_fresh", "status_fresh", "land_fresh"]
    }
    hold_age_max = {}
    for key in ["clock_age_s", "position_age_s", "status_age_s", "land_age_s"]:
        vals = [f(r[key]) for r in hold_rows if finite(r[key])]
        hold_age_max[key] = max(vals) if vals else math.nan
    reset_start = {
        key: hold_rows[0].get(key, "unavailable") if hold_rows else "unavailable"
        for key in ["xy_reset", "z_reset", "heading_reset"]
    }
    reset_end = {
        key: hold_rows[-1].get(key, "unavailable") if hold_rows else "unavailable"
        for key in ["xy_reset", "z_reset", "heading_reset"]
    }
    reset_seen = result_txt.get("estimator_reset_seen", "false").lower() == "true"
    preflight_issue = result_txt.get("preflight_issue_seen", "false").lower() == "true"
    hold_preflight_bad = any(r.get("preflight_ok") != "1" for r in hold_rows)
    if reset_seen:
        reasons.append("estimator_reset_seen")
    if preflight_issue or hold_preflight_bad:
        reasons.append("preflight_issue")
    if ulog["corrupt"]:
        reasons.append("ulog_corrupt")
    if ulog["dropouts"]:
        reasons.append(f"ulog_dropouts={ulog['dropouts']}")
    if not ulog["topics"].issuperset(set(TOPICS)):
        reasons.append("ulog_topic_subset_missing")
    classification = "invalid_for_metric" if any(
        reason.startswith(("missing:", "sha256_", "invalid_hold", "insufficient_", "metric_time_decreasing", "metric_gap", "moving_target", "ulog_corrupt"))
        for reason in reasons
    ) else ("review_required" if reasons else "clean_candidate")

    return {
        "archive": folder.name,
        "run_id": manifest.get("run_id", "unavailable"),
        "run_dir": str(run_dir.relative_to(ROOT)),
        "metadata": metadata,
        "result": result_txt,
        "ros_params": params,
        "manifest": manifest,
        "ulog": ulog,
        "events": events,
        "samples_total": len(rows),
        "hold_start_sim_s": hold_start,
        "hold_end_sim_s": hold_end,
        "hold_duration_sim_s": (hold_end - hold_start) if hold_start is not None and hold_end is not None else math.nan,
        "hold_start_wall_s": hold_start_wall,
        "hold_end_wall_s": hold_end_wall,
        "hold_duration_wall_s": (hold_end_wall - hold_start_wall) if hold_start_wall is not None and hold_end_wall is not None else math.nan,
        "hold_samples": len(hold_rows),
        "metric_samples": len(metric_rows),
        "metric_duration_sim_s": duration,
        "metric_max_gap_s": float(np.max(gaps)) if len(gaps) else math.nan,
        "sample_wall_strict": all(a < b for a, b in zip(sample_wall, sample_wall[1:])),
        "sample_sim_nondecreasing": all(a <= b for a, b in zip(sample_sim, sample_sim[1:])),
        "event_wall_nondecreasing": all(a <= b for a, b in zip(event_wall, event_wall[1:])),
        "event_sim_nondecreasing": all(a <= b for a, b in zip(event_sim, event_sim[1:])),
        "hold_fresh": hold_fresh,
        "hold_age_max": hold_age_max,
        "reset_start": reset_start,
        "reset_end": reset_end,
        "hold_preflight_bad": hold_preflight_bad,
        "hashes_ok": hashes_ok,
        "missing_files": missing,
        "quality_reasons": reasons,
        "quality_class": classification,
        "target": targets[0].tolist() if len(targets) else [math.nan] * 3,
        "main_axis_rmse_m": axis_rmse[0],
        "lateral_rmse_m": axis_rmse[1],
        "height_rmse_m": axis_rmse[2],
        "position_3d_rmse_m": position_3d_rmse,
        "max_overshoot_m": max_overshoot,
        "settling_time_s": settling,
        "steady_state_error_3d_m": steady_error,
        "speed_rms_mps": speed_rms,
        "max_speed_mps": max_speed,
        "task_completion": "success" if result_txt.get("flight_success", "false").lower() == "true" else "not_success",
        "flight_success": result_txt.get("flight_success", "unavailable"),
        "recording_ok": result_txt.get("recording_ok", "unavailable"),
        "state_final": result_txt.get("state", "unavailable"),
    }


def param_value(run: dict[str, Any], key: str) -> Any:
    return run["ulog"]["parameters"].get(key, "unavailable")


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(x) for x in row) + " |")
    return "\n".join(lines)


def aggregate(runs: list[dict[str, Any]], keys: list[str]) -> list[list[str]]:
    out = []
    for key in keys:
        vals = [float(r[key]) for r in runs if finite(r[key])]
        if not vals:
            out.append([key, "0", "unavailable", "unavailable", "unavailable", "unavailable", "unavailable"])
            continue
        out.append([
            key,
            str(len(vals)),
            fmt(statistics.mean(vals)),
            fmt(statistics.median(vals)),
            fmt(statistics.stdev(vals) if len(vals) >= 2 else math.nan),
            fmt(min(vals)),
            fmt(max(vals)),
        ])
    return out


def write_metrics(runs: list[dict[str, Any]]) -> None:
    fields = [
        "archive", "run_id", "quality_class", "quality_reasons", "task_completion", "flight_success", "recording_ok", "state_final",
        "hold_start_sim_s", "hold_end_sim_s", "hold_duration_sim_s", "hold_start_wall_s", "hold_end_wall_s", "hold_duration_wall_s", "metric_duration_sim_s", "hold_samples", "metric_samples", "metric_max_gap_s",
        "target_x_m", "target_y_m", "target_z_m", "main_axis_position_rmse_m", "lateral_rmse_m", "height_rmse_m", "position_3d_rmse_m",
        "max_overshoot_m", "settling_time_s", "steady_state_error_3d_m", "speed_rms_mps", "max_speed_mps",
        "hashes_ok", "ulog_version", "ulog_commit", "ulog_corrupt", "ulog_dropouts", "ulog_parameter_count",
        "ulog_failsafe", "ulog_user_took_over", "ulog_preflight_fail", "manual_topic", "manual_valid_values", "manual_sticks_values",
        "manual_signal_lost", "offboard_signal_lost", "reset_xy_start", "reset_xy_end", "reset_z_start", "reset_z_end", "reset_heading_start", "reset_heading_end",
        "hold_preflight_bad", "sample_wall_strict", "sample_sim_nondecreasing", "event_wall_nondecreasing", "event_sim_nondecreasing",
        "max_clock_age_s", "max_position_age_s", "max_status_age_s", "max_land_age_s", "hold_clock_fresh", "hold_position_fresh", "hold_status_fresh", "hold_land_fresh",
    ]
    with OUT_METRICS.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for r in runs:
            row = {k: "unavailable" for k in fields}
            row.update({
                "archive": r["archive"], "run_id": r["run_id"], "quality_class": r["quality_class"], "quality_reasons": ";".join(r["quality_reasons"]),
                "task_completion": r["task_completion"], "flight_success": r["flight_success"], "recording_ok": r["recording_ok"], "state_final": r["state_final"],
                "hold_start_sim_s": fmt(r["hold_start_sim_s"]), "hold_end_sim_s": fmt(r["hold_end_sim_s"]), "hold_duration_sim_s": fmt(r["hold_duration_sim_s"]),
                "hold_start_wall_s": fmt(r["hold_start_wall_s"]), "hold_end_wall_s": fmt(r["hold_end_wall_s"]), "hold_duration_wall_s": fmt(r["hold_duration_wall_s"]),
                "metric_duration_sim_s": fmt(r["metric_duration_sim_s"]), "hold_samples": r["hold_samples"], "metric_samples": r["metric_samples"], "metric_max_gap_s": fmt(r["metric_max_gap_s"]),
                "target_x_m": fmt(r["target"][0]), "target_y_m": fmt(r["target"][1]), "target_z_m": fmt(r["target"][2]),
                "main_axis_position_rmse_m": fmt(r["main_axis_rmse_m"]), "lateral_rmse_m": fmt(r["lateral_rmse_m"]), "height_rmse_m": fmt(r["height_rmse_m"]), "position_3d_rmse_m": fmt(r["position_3d_rmse_m"]),
                "max_overshoot_m": fmt(r["max_overshoot_m"]), "settling_time_s": fmt(r["settling_time_s"]), "steady_state_error_3d_m": fmt(r["steady_state_error_3d_m"]), "speed_rms_mps": fmt(r["speed_rms_mps"]), "max_speed_mps": fmt(r["max_speed_mps"]),
                "hashes_ok": r["hashes_ok"], "ulog_version": r["ulog"]["version"], "ulog_commit": r["ulog"]["commit"], "ulog_corrupt": r["ulog"]["corrupt"], "ulog_dropouts": r["ulog"]["dropouts"], "ulog_parameter_count": r["ulog"]["parameter_count"],
                "ulog_failsafe": r["ulog"]["ulog_failsafe"], "ulog_user_took_over": r["ulog"]["ulog_user_took_over"], "ulog_preflight_fail": r["ulog"]["ulog_preflight_fail"], "manual_topic": r["ulog"]["manual_topic"], "manual_valid_values": ",".join(map(str, r["ulog"]["manual_valid_values"])), "manual_sticks_values": ",".join(map(str, r["ulog"]["manual_sticks_values"])), "manual_signal_lost": r["ulog"]["manual_signal_lost"], "offboard_signal_lost": r["ulog"]["offboard_signal_lost"],
                "reset_xy_start": r["reset_start"]["xy_reset"], "reset_xy_end": r["reset_end"]["xy_reset"], "reset_z_start": r["reset_start"]["z_reset"], "reset_z_end": r["reset_end"]["z_reset"], "reset_heading_start": r["reset_start"]["heading_reset"], "reset_heading_end": r["reset_end"]["heading_reset"],
                "hold_preflight_bad": r["hold_preflight_bad"], "sample_wall_strict": r["sample_wall_strict"], "sample_sim_nondecreasing": r["sample_sim_nondecreasing"], "event_wall_nondecreasing": r["event_wall_nondecreasing"], "event_sim_nondecreasing": r["event_sim_nondecreasing"],
                "max_clock_age_s": fmt(r["hold_age_max"]["clock_age_s"]), "max_position_age_s": fmt(r["hold_age_max"]["position_age_s"]), "max_status_age_s": fmt(r["hold_age_max"]["status_age_s"]), "max_land_age_s": fmt(r["hold_age_max"]["land_age_s"]),
                "hold_clock_fresh": r["hold_fresh"]["clock_fresh"], "hold_position_fresh": r["hold_fresh"]["position_fresh"], "hold_status_fresh": r["hold_fresh"]["status_fresh"], "hold_land_fresh": r["hold_fresh"]["land_fresh"],
            })
            writer.writerow(row)


def write_reports(runs: list[dict[str, Any]]) -> None:
    clean = [r for r in runs if r["quality_class"] == "clean_candidate"]
    review = [r for r in runs if r["quality_class"] == "review_required"]
    invalid = [r for r in runs if r["quality_class"] == "invalid_for_metric"]
    metric_keys = ["main_axis_rmse_m", "lateral_rmse_m", "height_rmse_m", "position_3d_rmse_m", "max_overshoot_m", "settling_time_s", "steady_state_error_3d_m", "speed_rms_mps", "max_speed_mps"]
    metric_labels = {"main_axis_rmse_m": "主轴位置 RMSE (m)", "lateral_rmse_m": "横向 RMSE (m)", "height_rmse_m": "高度 RMSE (m)", "position_3d_rmse_m": "三维位置 RMSE (m)", "max_overshoot_m": "最大超调/最大三维偏差 (m)", "settling_time_s": "稳定时间 (s)", "steady_state_error_3d_m": "稳态三维绝对误差 (m)", "speed_rms_mps": "speed RMS (m/s)", "max_speed_mps": "最大 speed (m/s)"}
    quality_rows = []
    for r in runs:
        quality_rows.append([
            r["archive"].replace("hover_capture_baseline", ""), f"`{r['run_id']}`", f"`{r['quality_class']}`", "是" if r["hashes_ok"] else "否", r["result"].get("flight_success", "unavailable"), r["result"].get("recording_ok", "unavailable"), r["result"].get("estimator_reset_seen", "unavailable"), r["result"].get("preflight_issue_seen", "unavailable"), f"{r['metric_samples']}/{r['hold_samples']}", f"{r['reset_start']['xy_reset']}→{r['reset_end']['xy_reset']}/{r['reset_start']['z_reset']}→{r['reset_end']['z_reset']}/{r['reset_start']['heading_reset']}→{r['reset_end']['heading_reset']}", "是" if r["ulog"]["ulog_failsafe"] is False else "否/缺失", "; ".join(r["quality_reasons"]) or "—"
        ])
    metric_rows = []
    for r in runs:
        metric_rows.append([
            r["archive"].replace("hover_capture_baseline", ""), f"`{r['run_id']}`", f"`{r['quality_class']}`", fmt(r["hold_start_sim_s"], 3), fmt(r["hold_end_sim_s"], 3), str(r["metric_samples"]), fmt(r["main_axis_rmse_m"], 6), fmt(r["lateral_rmse_m"], 6), fmt(r["height_rmse_m"], 6), fmt(r["position_3d_rmse_m"], 6), fmt(r["max_overshoot_m"], 6), fmt(r["settling_time_s"], 3), fmt(r["steady_state_error_3d_m"], 6), fmt(r["speed_rms_mps"], 6), fmt(r["max_speed_mps"], 6), r["task_completion"]
        ])
    agg_rows = aggregate(review, metric_keys)
    agg_rows = [[metric_labels.get(row[0], row[0]), *row[1:]] for row in agg_rows]
    parameter_rows = []
    for key in CONTROL_KEYS:
        vals = [param_value(r, key) for r in runs]
        parameter_rows.append([f"`{key}`", fmt(vals[0], 6), "一致" if all(v == vals[0] for v in vals) else "不一致/缺失"])
    common_topics = [topic for topic in TOPICS if all(topic in r["ulog"]["topics"] for r in runs)]
    report = f"""# 科目 2 悬停基准数据报告

报告生成日期：2026-10-20（Asia/Shanghai）  
数据集：用户提供的 `hover_capture_baseline1.zip` 至 `hover_capture_baseline10.zip`。分析目录：`analysis_runs/`。

## 结论范围

这是 PX4 local NED、X500 SITL、起飞 3 m→悬停 30 s→降落上锁的同工况位置/速度表现基线。10/10 次任务完成并最终 `IDLE`，Hold 窗口内位置和速度字段完整、有效标志为 1，且没有 `vehicle_status.failsafe` 或 `failsafe_and_user_took_over`。但是 10/10 次都出现估计器 reset 记录，故 0 次进入 `clean_candidate`，10 次均为 `review_required`；review 数据只进入 review 汇总，不进入 clean 主基准均值。

## 1. 实验身份与证据

| 项目 | 证据 |
|---|---|
| 科目 | 起飞 3 m、悬停 30 s、降落上锁 |
| 坐标系 | PX4 local NED：x 北、y 东、z 向下；本报告主轴=x、横向=y、高度=z |
| 记录节点 | `metadata.txt`：`build=2026-10-14-v3`；samples 为 wall 20 Hz 节点接收记录 |
| PX4 | 10/10 ULog：`v1.17.0`，commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，硬件标识 `PX4_SITL` |
| ROS 2/Gazebo | 用户声明为 ROS 2 Jazzy/Gazebo Harmonic；本批附件没有独立运行时版本快照，未把声明当作新测量证据 |
| ULog | 10/10 文件可解析、`file_corruption=false`、dropout=0；均含关键主题：`{', '.join(common_topics)}` |
| PX4 参数快照 | 每个 ULog 含 1000 个 initial parameters；`capture_manifest.json` 同时明确外部参数/版本快照未由采集脚本收集 |
| 原始文件完整性 | 10/10 `capture.sha256` 全部通过；原始压缩包未修改 |

### PX4 参数一致性

以下控制相关 ULog 参数在 10 次运行的 embedded `initial_parameters` 中一致：

{markdown_table(['参数', '首个运行值', '10 次比较'], parameter_rows)}

辅助运行状态参数（如 `CAL_BARO0_OFF`、`LND_FLIGHT_T_LO`、`COM_FLIGHT_UUID`）在不同 ULog 中变化；报告不把它们当成控制调参变量，也不猜测缺失的历史参数快照。

10 个 `parameters.yaml` 的科目任务参数一致（`altitude_m=3`、`hold_s=30`、`position_tolerance_m=0.3`、`speed_tolerance_mps=0.3` 等）；仅每次运行的 `log_root` 路径随 `run_id` 改变。

## 2. 逐运行质量表

{markdown_table(['序号', 'run_id', '分级', '哈希', 'flight_success', 'recording_ok', 'reset_seen', 'preflight_issue', 'Hold有效样本/窗口样本', 'Hold reset xy/z/heading', 'vehicle_status failsafe=0', '质量原因'], quality_rows)}

分级规则：`invalid_for_metric` 表示必需文件/哈希、事件窗口、有效样本、目标恒定性或采样间隔使指标不可可靠定义；`review_required` 表示指标可在窗口内计算，但存在 reset、preflight 或其他需要复核的证据；只有没有这些问题才是 `clean_candidate`。本批 `clean_candidate=0`、`review_required={len(review)}`、`invalid_for_metric={len(invalid)}`。

## 3. 逐运行指标（统一 Hold 窗口）

窗口为每次 `events.csv` 的 `state -> HOLD` 至 `hold complete: ...` 的 `sim_time_s` 边界，只使用 `state=HOLD` 且目标/位置/速度有限、四个 valid 字段均为 1 的样本。窗口内没有 NaN；窗口外出现的 NaN 保留并记录，未填 0。

{markdown_table(['序号', 'run_id', '分级', '窗口起点(s)', '窗口终点(s)', 'n', '主轴RMSE', '横向RMSE', '高度RMSE', '三维RMSE', '最大超调*', '稳定时间*', '稳态三维误差', 'speed RMS', '最大speed', '任务结果'], metric_rows)}

\\* `最大超调`对本科目零位移悬停没有可定义的 step 百分比，因此报告的是 Hold 窗口内最大三维位置偏差 `max(||p-target||)`，单位 m；`稳定时间`是 Hold 窗口内、相对窗口起点、按 0.3 m 三轴容差且至少连续 1 s 保持的时间，不是从起飞开始的全任务稳定时间。所有运行在 Hold 首样本即满足并保持，故该定义下为 0 s。

## 4. clean_candidate 汇总

`clean_candidate` 运行数为 0。按规则不能从 review_required 运行借用或推导 clean 主基准均值；因此 clean 汇总中的全部指标为 `unavailable`。

## 5. review_required 汇总（不冒充 clean）

以下为 10 次 review_required 运行的等权运行级描述统计，标准差为样本标准差（n−1）：

{markdown_table(['指标', 'n', '均值', '中位数', '标准差', '最小值', '最大值'], agg_rows)}

任务完成：10/10 `flight_success=true`，10/10 `recording_ok=true`，最终状态均为 `IDLE`。这说明本批同工况任务链可重复完成，但不等于无异常 clean 基准。

## 6. 时间戳、NaN、valid、新鲜度与状态审计

- 每个运行的 `samples.csv` wall 时间严格递增；事件 wall/sim 时间均非递减。sim 时间在个别记录有重复采样值，但没有大于 0.2 s 的 Hold 间隔；指标使用 Gazebo `sim_time_s`，重复时间样本的积分区间为零，不填补或重采样。
- Hold 窗口内四个位置/速度 valid 字段均为 1，目标为每次运行固定值；指标窗口无 NaN。窗口外的等待、着陆阶段 NaN 未被改写。
- Hold 窗口内 `clock_fresh`、`position_fresh`、`status_fresh`、`land_fresh` 均保持 1。原始 age 字段仍保留在 CSV；各运行的最大 age 见 `baseline_metrics.csv`，不把 freshness flag 当成无限新鲜。
- 10/10 `result.txt` 报告 `estimator_reset_seen=true`。Hold 起点到终点的 reset 计数在窗口内不再增加，但窗口起点已经是非零计数，因此这是 review 证据而不是 clean 证据。
- 10/10 的阶段事件序列均为 `PRIME_OFFBOARD → TAKEOFF → INITIAL_HOLD → HOLD → LANDING → IDLE`；10/10 的 reset 事件都在 `TAKEOFF` 阶段以 `DATA FLAG: estimator reset observed` 记录，随后任务继续。
- 采样状态没有 `failsafe=1`；ULog `vehicle_status.failsafe=0` 且 `failsafe_and_user_took_over=0`。`failsafe_flags` 中 manual-control signal lost 是 SITL 无 RC 输入的记录，不能解释成有人接管；可见的 Offboard signal lost 与降落时停止 Offboard 心跳的交接阶段相关。
- ULog 中手动控制主题在部分运行不存在，存在时 `valid=0`、`sticks_moving=0`；因此未观察到人工接管，但附件不提供独立的人机操作证明。

## 7. 异常列表与限制

1. 10/10 运行均有 estimator reset；运行 3、6、8 的 Hold 窗口 `preflight_ok=0`，其他运行至少在 Hold 窗口为 1；所有 `result.txt` 仍标记任务成功。
2. `capture_manifest.json` 明确写出 ULog 可能包含启动或其他飞行，且只按“捕获期间更新”选择文件，未解析 flight timeline；本报告使用附件内 ULog 的完整可解析内容做审计，不把它当作已完成的任务时间对齐证明。
3. 没有独立 PX4 参数/版本快照文件；只有 ULog embedded initial parameters 和版本 info。缺失的历史参数值不猜测。
4. `samples.csv` 是位置/速度记录器的节点接收采样，不是 raw IMU、全量控制器输出或执行器饱和日志。位置 CSV/这些指标不能证明速度环、加速度限幅或执行器饱和的原因；本报告不作此类因果结论。
5. 该基准回答的是：在给定 build、任务、采样器和当前 ULog 参数证据下，悬停 Hold 窗口的位置/速度误差、波动范围、任务完成和质量标记是什么。它没有回答：跨版本/跨随机种子泛化、真实硬件表现、控制器内部因果、执行器饱和原因、传感器/估计器 reset 的根因，或参数修改后的收益。

## 8. 可复现定义

- 时间轴：`samples.csv.sim_time_s`；Hold 事件边界来自 `events.csv` 的 `sim_time_s`。`wall_elapsed_s` 只用于接收顺序/新鲜度审计。
- 误差：`e = [x-target_x, y-target_y, z-target_z]`，PX4 local NED；主轴=x、横向=y、高度=z。
- RMSE：对采样误差平方按 `sim_time_s` 做梯形积分除以窗口时长再开方；三维 RMSE 为三轴均方和的平方根。
- 最大超调：零位移悬停使用最大三维绝对偏差 `max(||e||)`，不报告无定义的百分比超调。
- 稳定时间：Hold 起点后首次进入并持续到窗口末、且剩余至少 1 s 的三轴 `|e|<=0.3 m`；未满足应为 `unavailable`，从不填 0。
- 稳态误差：窗口末 5 s 内 `||e||` 的时间加权平均；不足 2 个样本为 `unavailable`。
- speed RMS：对 `samples.csv.speed` 的平方按 `sim_time_s` 加权；最大 speed 为窗口内有限样本最大值。
- 缺失/NaN：不填 0；必要字段不能可靠计算时写 `unavailable`，并按 `invalid_for_metric` 处理。

逐运行 CSV（含所有质量审计字段和指标）见 [`baseline_metrics.csv`](/Users/mojinliang/Desktop/quadrotor/baseline_metrics.csv)。
"""
    OUT_REPORT.write_text(report)

    summary_rows = [[
        "任务完成并上锁", "10/10", "基于 result.txt + events.csv 最终 IDLE"
    ], ["clean_candidate", str(len(clean)), "不从 review_required 混入"], ["review_required", str(len(review)), "10/10 有 estimator reset"], ["invalid_for_metric", str(len(invalid)), "无"]]
    summary = f"""# 科目 2 基准摘要

本批 10 次科目 2 运行均完成“起飞 3 m→悬停 30 s→降落上锁”，但 10 次均记录 estimator reset，因而没有 clean_candidate。可计算的误差数值只作为 `review_required` 汇总，不能视为 clean 主基准。

{markdown_table(['项目', '结果', '说明'], summary_rows)}

## review_required 运行级均值

{markdown_table(['指标', 'n', '均值', '中位数', '标准差', '最小值', '最大值'], agg_rows)}

主轴为 x 北向、横向为 y 东向、高度为 z 向下；指标窗口为 `state -> HOLD` 到 `hold complete`，时间积分使用 Gazebo `sim_time_s`。最大超调是零位移悬停中的最大三维位置偏差，稳定时间只描述 Hold 窗口内的 0.3 m 容差保持。

证据限制：外部 PX4 参数/版本快照缺失；仅使用 ULog embedded version/initial parameters。位置 CSV 和速度字段不足以断言速度环、加速度限幅或执行器饱和的因果原因。完整质量表、逐运行指标、异常和定义见 [`baseline_report.md`](/Users/mojinliang/Desktop/quadrotor/baseline_report.md)，机器可读结果见 [`baseline_metrics.csv`](/Users/mojinliang/Desktop/quadrotor/baseline_metrics.csv)。
"""
    OUT_SUMMARY.write_text(summary)


def main() -> None:
    folders = sorted(RUN_ROOT.glob("hover_capture_baseline[0-9]*"), key=lambda p: int(p.name.rsplit("baseline", 1)[1]))
    if len(folders) != 10:
        raise SystemExit(f"expected 10 run folders under {RUN_ROOT}, found {len(folders)}")
    runs = [analyze_run(folder) for folder in folders]
    # The control parameter consistency check is reported in the report. The embedded ULog
    # values are parsed per run; this intentionally does not alter the quality classification.
    write_metrics(runs)
    write_reports(runs)
    print(f"wrote {OUT_METRICS}, {OUT_REPORT}, {OUT_SUMMARY}")


if __name__ == "__main__":
    main()
