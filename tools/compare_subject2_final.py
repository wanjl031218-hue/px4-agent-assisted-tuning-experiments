#!/usr/bin/env python3
"""Compare subject-2 baseline and candidate hover runs with one metric definition."""
from __future__ import annotations

import csv, hashlib, json, math, statistics
from pathlib import Path
from typing import Any

import numpy as np
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]
OUT_CSV = ROOT / "subject2_comparison_metrics_final.csv"
OUT_REPORT = ROOT / "subject2_comparison_report_final.md"
OUT_SUMMARY = ROOT / "subject2_comparison_summary_final.md"
MAX_GAP = 0.2
TOL = 0.3
DWELL = 1.0
STEADY = 5.0
REQUIRED = ["samples.csv", "events.csv", "result.txt", "parameters.yaml", "metadata.txt", "flight.ulg", "capture.sha256", "capture_manifest.json"]
TOPICS = ["vehicle_local_position", "vehicle_local_position_setpoint", "vehicle_local_position_groundtruth", "trajectory_setpoint", "vehicle_status", "vehicle_control_mode", "failsafe_flags", "estimator_status", "estimator_status_flags", "control_allocator_status", "actuator_motors"]
METRICS = ["main_axis_rmse_m", "lateral_rmse_m", "height_rmse_m", "position_3d_rmse_m", "max_overshoot_m", "settling_time_s", "steady_state_error_3d_m", "speed_rms_mps", "max_speed_mps"]
LABELS = {"main_axis_rmse_m":"主轴 RMSE (m)", "lateral_rmse_m":"横向 RMSE (m)", "height_rmse_m":"高度 RMSE (m)", "position_3d_rmse_m":"三维 RMSE (m)", "max_overshoot_m":"最大超调/最大三维偏差 (m)", "settling_time_s":"稳定时间 (s)", "steady_state_error_3d_m":"稳态三维误差 (m)", "speed_rms_mps":"速度 RMS (m/s)", "max_speed_mps":"最大速度 (m/s)"}

def finite(x):
    try: return math.isfinite(float(x))
    except Exception: return False

def f(x): return float(x)

def trap(t, y):
    if len(t) < 2 or t[-1] <= t[0]: return math.nan
    return float(np.trapezoid(y, t))

def fmt(x, n=6):
    if x is None or not finite(x): return "unavailable"
    return f"{float(x):.{n}f}"

def hash_ok(d: Path):
    bad=[]
    try: lines=(d/"capture.sha256").read_text().splitlines()
    except Exception: return False, ["missing:capture.sha256"]
    for line in lines:
        q=line.split()
        if len(q)!=2: continue
        expected,name=q
        p=d/name
        if not p.exists(): bad.append("missing:"+name)
        elif hashlib.sha256(p.read_bytes()).hexdigest()!=expected: bad.append("sha256:"+name)
    return not bad,bad

def kv(path: Path):
    out={}
    for line in path.read_text(errors="replace").splitlines():
        if "=" in line:
            k,v=line.split("=",1); out[k.strip()]=v.strip()
    return out

def yaml_kv(path: Path):
    out={}
    for line in path.read_text(errors="replace").splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            k,v=line.strip().split(":",1)
            if k not in {"px4_control_experiment","ros__parameters"}: out[k.strip()]=v.strip().strip('"')
    return out

def event(events, prefix):
    for r in events:
        if r.get("event","").startswith(prefix):
            return f(r["sim_time_s"]), f(r["wall_elapsed_s"])
    return None,None

def dataset(u, name):
    for d in u.data_list:
        if d.name == name: return d.data
    return None

def analyze(group: str, idx: int, role: str, d: Path):
    reasons=[]
    missing=[x for x in REQUIRED if not (d/x).is_file()]
    if missing: reasons.append("missing:"+",".join(missing))
    h_ok,h_bad=hash_ok(d)
    if not h_ok: reasons += h_bad
    manifest=json.loads((d/"capture_manifest.json").read_text()) if (d/"capture_manifest.json").exists() else {}
    result=kv(d/"result.txt") if (d/"result.txt").exists() else {}
    task=yaml_kv(d/"parameters.yaml") if (d/"parameters.yaml").exists() else {}
    events=list(csv.DictReader((d/"events.csv").open())) if (d/"events.csv").exists() else []
    rows=list(csv.DictReader((d/"samples.csv").open())) if (d/"samples.csv").exists() else []
    hs,hw=event(events,"state -> HOLD"); he,hew=event(events,"hold complete:")
    if hs is None or he is None or he<=hs: reasons.append("invalid_hold_event_window")
    hold=[r for r in rows if r.get("state")=="HOLD" and hs is not None and he is not None and hs<=f(r["sim_time_s"])<=he]
    fields=["target_x","target_y","target_z","x","y","z","vx","vy","vz","speed"]
    valid=[r for r in hold if all(finite(r.get(k)) for k in fields) and all(r.get(k)=="1" for k in ["xy_valid","z_valid","v_xy_valid","v_z_valid"])]
    if len(valid)<2: reasons.append("insufficient_valid_metric_samples")
    t=np.array([f(r["sim_time_s"]) for r in valid],float)
    if len(t)>=2:
        gaps=np.diff(t)
        if np.any(gaps<0): reasons.append("metric_time_decreasing")
        if np.any(gaps>MAX_GAP): reasons.append("metric_gap_gt_0.2s")
    else: gaps=np.array([])
    if valid:
        target=np.array([[f(r["target_x"]),f(r["target_y"]),f(r["target_z"])] for r in valid])
        e=np.array([[f(r["x"]),f(r["y"]),f(r["z"])] for r in valid])-target
        sp=np.array([f(r["speed"]) for r in valid])
        dur=t[-1]-t[0] if len(t)>1 else math.nan
        rms=[math.sqrt(trap(t,e[:,i]**2)/dur) if dur>0 else math.nan for i in range(3)]
        p3=math.sqrt(trap(t,np.sum(e*e,axis=1))/dur) if dur>0 else math.nan
        mx=float(np.max(np.linalg.norm(e,axis=1)))
        sr=math.sqrt(trap(t,sp*sp)/dur) if dur>0 else math.nan
        sm=t>=t[-1]-STEADY; st=t[sm]; se=np.linalg.norm(e[sm],axis=1)
        steady=trap(st,se)/(st[-1]-st[0]) if len(st)>1 and st[-1]>st[0] else math.nan
        inside=np.all(np.abs(e)<=TOL,axis=1); settling=math.nan
        for j in range(len(t)):
            if t[-1]-t[j]>=DWELL and bool(np.all(inside[j:])):
                settling=t[j]-t[0]; break
    else:
        target=np.empty((0,3)); dur=math.nan; rms=[math.nan]*3; p3=mx=sr=steady=settling=math.nan
    sample_wall=[f(r["wall_elapsed_s"]) for r in rows if finite(r.get("wall_elapsed_s"))]
    sample_sim=[f(r["sim_time_s"]) for r in rows if finite(r.get("sim_time_s"))]
    event_wall=[f(r["wall_elapsed_s"]) for r in events if finite(r.get("wall_elapsed_s"))]
    event_sim=[f(r["sim_time_s"]) for r in events if finite(r.get("sim_time_s"))]
    hold_bad=any(r.get("preflight_ok")!="1" for r in hold)
    reset=result.get("estimator_reset_seen","false").lower()=="true"
    pre=result.get("preflight_issue_seen","false").lower()=="true"
    if reset: reasons.append("estimator_reset_seen")
    if pre or hold_bad: reasons.append("preflight_issue")
    ulog=None
    if (d/"flight.ulg").exists():
        try:
            u=ULog(str(d/"flight.ulg")); params=dict(u.initial_parameters)
            topics={x.name for x in u.data_list}; st=dataset(u,"vehicle_status")
            failsafe=None if st is None else bool(np.any(st.get("failsafe",[])))
            takeover=None if st is None else bool(np.any(st.get("failsafe_and_user_took_over",[])))
            manual=dataset(u,"manual_control_setpoint")
            manual_valid=[] if manual is None else sorted(set(np.asarray(manual.get("valid",[])).tolist()))
            ulog={"version":u.get_version_info_str(),"commit":u.msg_info_dict.get("ver_sw","unavailable"),"hardware":u.msg_info_dict.get("ver_hw","unavailable"),"corrupt":bool(u.file_corruption),"dropouts":len(u.dropouts),"parameter_count":len(params),"params":params,"changed_parameters":list(u.changed_parameters),"topics":topics,"failsafe":failsafe,"takeover":takeover,"manual_topic":manual is not None,"manual_valid":manual_valid}
            if ulog["corrupt"]: reasons.append("ulog_corrupt")
            if ulog["dropouts"]: reasons.append(f"ulog_dropouts={ulog['dropouts']}")
            if not topics.issuperset(TOPICS): reasons.append("ulog_topic_subset_missing")
        except Exception as ex:
            ulog={"version":"unavailable","commit":"unavailable","hardware":"unavailable","corrupt":True,"dropouts":None,"parameter_count":0,"params":{},"changed_parameters":[],"topics":set(),"failsafe":None,"takeover":None,"manual_topic":False,"manual_valid":[]}; reasons.append("ulog_parse_error")
    else: ulog={"version":"unavailable","commit":"unavailable","hardware":"unavailable","corrupt":True,"dropouts":None,"parameter_count":0,"params":{},"changed_parameters":[],"topics":set(),"failsafe":None,"takeover":None,"manual_topic":False,"manual_valid":[]}
    if ulog["failsafe"] is True: reasons.append("ulog_failsafe")
    if ulog["takeover"] is True: reasons.append("ulog_user_took_over")
    # Use the parameter value effective at HOLD start. Some ULogs retain the
    # boot-time initial value and record a parameter change before the task.
    effective_params = dict(ulog["params"])
    if hs is not None:
        for ts, key, value in ulog.get("changed_parameters", []):
            if key == "MPC_XY_VEL_P_ACC" and float(ts) / 1e6 <= hs:
                effective_params[key] = value
    param_value = effective_params.get("MPC_XY_VEL_P_ACC", "unavailable")
    if group == "candidate" and (not finite(param_value) or abs(float(param_value) - 1.9) > 1e-5):
        reasons.append(f"parameter_mismatch_expected_1.900_actual_{param_value}")
    classification="invalid_for_metric" if any(x.startswith(("missing:","sha256:","invalid_hold","insufficient_","metric_time_decreasing","metric_gap_gt_0.2s","ulog_corrupt","ulog_parse_error")) for x in reasons) else ("review_required" if reasons else "clean_candidate")
    return {"group":group,"index":idx,"comparison_role":role,"run_id":manifest.get("run_id","unavailable"),"path":str(d.relative_to(ROOT)),"quality_class":classification,"quality_reasons":";".join(dict.fromkeys(reasons)),"hashes_ok":h_ok,"flight_success":result.get("flight_success","unavailable"),"recording_ok":result.get("recording_ok","unavailable"),"state_final":result.get("state","unavailable"),"hold_start_sim_s":hs,"hold_end_sim_s":he,"metric_samples":len(valid),"hold_samples":len(hold),"metric_max_gap_s":float(np.max(gaps)) if len(gaps) else math.nan,"main_axis_rmse_m":rms[0],"lateral_rmse_m":rms[1],"height_rmse_m":rms[2],"position_3d_rmse_m":p3,"max_overshoot_m":mx,"settling_time_s":settling,"steady_state_error_3d_m":steady,"speed_rms_mps":sr,"max_speed_mps":float(np.max(sp)) if valid else math.nan,"task_completion":"success" if result.get("flight_success","false").lower()=="true" else "not_success","params":effective_params,"parameter_value":param_value,"version":ulog["version"],"commit":ulog["commit"],"hardware":ulog["hardware"],"ulog_corrupt":ulog["corrupt"],"ulog_dropouts":ulog["dropouts"],"ulog_failsafe":ulog["failsafe"],"ulog_takeover":ulog["takeover"],"manual_topic":ulog["manual_topic"],"manual_valid":",".join(map(str,ulog["manual_valid"])),"estimator_reset_seen":reset,"preflight_issue_seen":pre,"hold_preflight_bad":hold_bad,"sample_wall_strict":all(a<b for a,b in zip(sample_wall,sample_wall[1:])),"sample_sim_nondecreasing":all(a<=b for a,b in zip(sample_sim,sample_sim[1:])),"event_wall_nondecreasing":all(a<=b for a,b in zip(event_wall,event_wall[1:])),"event_sim_nondecreasing":all(a<=b for a,b in zip(event_sim,event_sim[1:])),"task_params":task,"target":target[0].tolist() if len(target) else [math.nan]*3}

def summary(rows, key):
    vals=[float(r[key]) for r in rows if finite(r[key])]
    return {"n":len(vals),"mean":statistics.mean(vals) if vals else math.nan,"median":statistics.median(vals) if vals else math.nan,"std":statistics.stdev(vals) if len(vals)>=2 else math.nan,"min":min(vals) if vals else math.nan,"max":max(vals) if vals else math.nan}

def pct(a,b): return (b-a)/a*100 if finite(a) and finite(b) and a!=0 else math.nan

def table(headers, rows):
    s=["| "+" | ".join(headers)+" |","|"+"|".join("---" for _ in headers)+"|"]
    s += ["| "+" | ".join(str(x) for x in r)+" |" for r in rows]
    return "\n".join(s)

def main():
    # Latest replacement packages are used for the affected run numbers.
    base={i:ROOT/"analysis_runs"/f"hover_capture_baseline{i}"/f"hover_capture_baseline{i}" for i in range(1,11)}
    base.update({3:ROOT/"analysis_runs_baseline_retest/baseline3_retest/hover_capture_baseline3",6:ROOT/"analysis_runs_baseline_retest/baseline6_retest/hover_capture_baseline6",8:ROOT/"analysis_runs_baseline_retest/baseline8_retest/hover_capture_baseline8"})
    cand={i:ROOT/"analysis_runs_candidate"/f"hover_capture_modified{i}"/f"hover_capture_modified{i}" for i in range(1,11)}
    cand.update({
        1:ROOT/"analysis_runs_candidate_retest/modified1_latest/hover_capture_modified1",
        7:ROOT/"analysis_runs_candidate_retest3/modified7_latest3/hover_capture_modified7",
        9:ROOT/"analysis_runs_candidate_retest3/modified9_latest3/hover_capture_modified9",
        10:ROOT/"analysis_runs_candidate_retest3/modified10_latest3/hover_capture_modified10",
    })
    rows=[]
    for i in range(1,11): rows.append(analyze("baseline",i,"baseline_1.800",base[i]))
    for i in range(1,11): rows.append(analyze("candidate",i,"candidate_1.900",cand[i]))
    fields=["group","index","run_id","comparison_role","path","quality_class","quality_reasons","parameter_value","version","commit","hardware","hashes_ok","flight_success","recording_ok","state_final","hold_start_sim_s","hold_end_sim_s","hold_samples","metric_samples","metric_max_gap_s"]+METRICS+["task_completion","ulog_corrupt","ulog_dropouts","ulog_failsafe","ulog_takeover","manual_topic","manual_valid","estimator_reset_seen","preflight_issue_seen","hold_preflight_bad","sample_wall_strict","sample_sim_nondecreasing","event_wall_nondecreasing","event_sim_nondecreasing"]
    with OUT_CSV.open("w",newline="") as h:
        w=csv.DictWriter(h,fieldnames=fields);w.writeheader()
        for r in rows: w.writerow({k:(fmt(r[k]) if isinstance(r.get(k),float) else r.get(k,"unavailable")) for k in fields})
    bs=[r for r in rows if r["group"]=="baseline"]; cs=[r for r in rows if r["group"]=="candidate"]
    cs_eff=[r for r in cs if finite(r["parameter_value"]) and abs(float(r["parameter_value"])-1.9)<=1e-5]
    bsum={k:summary(bs,k) for k in METRICS}; csum={k:summary(cs_eff,k) for k in METRICS}
    changes=[]
    for k in METRICS:
        changes.append([LABELS[k],fmt(bsum[k]["mean"]),fmt(csum[k]["mean"]),fmt(csum[k]["mean"]-bsum[k]["mean"]),fmt(pct(bsum[k]["mean"],csum[k]["mean"]),2),"改善" if (csum[k]["mean"]<bsum[k]["mean"] and k!="settling_time_s") or (csum[k]["mean"]>bsum[k]["mean"] and k=="settling_time_s") else ("退化" if csum[k]["mean"]!=bsum[k]["mean"] else "不变")])
    qrows=[]
    for r in rows:
        qrows.append([r["group"],r["index"],f"`{r['run_id']}`",fmt(r["parameter_value"],3),r["quality_class"],r["hashes_ok"],r["flight_success"],r["recording_ok"],r["state_final"],r["metric_samples"],r["hold_preflight_bad"],r["estimator_reset_seen"],r["ulog_dropouts"],r["ulog_failsafe"],r["ulog_takeover"],r["quality_reasons"] or "—"])
    mrows=[]
    for r in rows:
        mrows.append([r["group"],r["index"],f"`{r['run_id']}`",fmt(r["hold_start_sim_s"],3),fmt(r["hold_end_sim_s"],3),r["metric_samples"]]+[fmt(r[k],6) for k in METRICS]+[r["task_completion"]])
    def agg_table(g):
        s=[]
        for k in METRICS:
            a=bsum[k] if g=="baseline" else csum[k]
            s.append([LABELS[k],a["n"],fmt(a["mean"]),fmt(a["median"]),fmt(a["std"]),fmt(a["min"]),fmt(a["max"])])
        return table(["指标","n","均值","中位数","标准差","最小","最大"],s)
    quality_counts={g:{q:sum(r["quality_class"]==q for r in rows if r["group"]==g) for q in ["clean_candidate","review_required","invalid_for_metric"]} for g in ["baseline","candidate"]}
    base_complete=sum(r["task_completion"]=="success" for r in bs); cand_complete=sum(r["task_completion"]=="success" for r in cs)
    report=f"""# 科目 2 baseline / candidate 对比报告（更新版）

数据按同一 PX4 local NED、同一任务（起飞 3 m→悬停 30 s→降落上锁）、同一 `events.csv` Hold 窗口和同一指标实现比较。baseline 使用 3/6/8 的新补测包替换旧包；candidate 使用当前 `hover_capture_modified1.zip`、`modified2`–`modified6`、`modified8`，以及本次替换的 `modified7(1)`、`modified9(2)`、`modified10(3)`。旧包保留但不用于本次主统计。

## 数据质量表

{table(["组","序号","run_id","MPC_XY_VEL_P_ACC","分级","哈希","成功","记录","最终状态","指标样本","Hold preflight坏","reset","dropout","failsafe","接管","质量原因"],qrows)}

质量计数：baseline `{quality_counts['baseline']}`；candidate `{quality_counts['candidate']}`。candidate 7 的 ULog 初始值为 1.800，但在 Hold 开始前记录了 `MPC_XY_VEL_P_ACC=1.900` 的参数变更，因此按 Hold 时有效值 1.900 计入；candidate 9、10 的初始值就是 1.900。三者均按有效值纳入 candidate 主统计，共 `{len(cs_eff)}` 次。

版本和任务审计：20/20 ULog 为 PX4 v1.17.0、同一 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`、同一 `PX4_SITL`，20/20 无 ULog dropout，任务参数和 Hold 任务一致。两组参数分别为 1.800 和 1.900，其他控制参数保持 baseline 报告中的一致值。外部独立参数快照仍缺失，参数证据来自 ULog embedded initial parameters。

## 逐运行指标

{table(["组","序号","run_id","Hold起点","Hold终点","n","主轴RMSE","横向RMSE","高度RMSE","三维RMSE","最大偏差","稳定时间","稳态误差","速度RMS","最大速度","任务"],mrows)}

Hold 窗口内只使用位置、目标、速度有限且四个 valid 字段为 1 的样本；NaN 没有填 0。最大超调在零位移悬停中表示 Hold 内最大三维偏差，稳定时间按 0.3 m 三轴容差连续 1 s 定义，因此本批 0 s 没有区分度。

## 基线/候选汇总

### baseline (`MPC_XY_VEL_P_ACC=1.800`)

{agg_table('baseline')}

任务完成率：{base_complete}/10 = {base_complete/10:.0%}；异常数量：reset {sum(r['estimator_reset_seen'] for r in bs)}/10，运行级 preflight 标记 {sum(r['preflight_issue_seen'] for r in bs)}/10，Hold 内 preflight_bad {sum(r['hold_preflight_bad'] for r in bs)}/10，invalid {quality_counts['baseline']['invalid_for_metric']}。

### candidate 有效 1.900 子集（`MPC_XY_VEL_P_ACC=1.900`）

{agg_table('candidate')}

任务完成率（全部 candidate 包）：{cand_complete}/10 = {cand_complete/10:.0%}；有效 1.900 子集 n={len(cs_eff)}。candidate 全部异常数量：reset {sum(r['estimator_reset_seen'] for r in cs)}/10，运行级 preflight 标记 {sum(r['preflight_issue_seen'] for r in cs)}/10，Hold 内 preflight_bad {sum(r['hold_preflight_bad'] for r in cs)}/10，参数错配 {sum('parameter_mismatch' in r['quality_reasons'] for r in cs)}/10。有效 1.900 子集 Hold 内 preflight_bad {sum(r['hold_preflight_bad'] for r in cs_eff)}/{len(cs_eff)}。

## 变化率

变化率定义为 `(candidate 均值 - baseline 均值) / baseline 均值`。误差和速度指标下降为改善，稳定时间下降为改善。

{table(["指标","baseline均值","candidate均值","绝对差","相对变化率","方向"],changes)}

## 异常与混杂因素

- candidate 7 的 ULog `initial_parameters` 为 1.800，但 `changed_parameters` 在 Hold 开始前将 `MPC_XY_VEL_P_ACC` 设为 1.900；candidate 9、10 的初始值为 1.900。三者的 Hold 时有效参数均为 1.900，已纳入 candidate 主统计；baseline 10/10 的 Hold 时有效值为 1.800。
- baseline 的 Hold 内 `preflight_ok` 为 10/10 通过；有效 1.900 candidate 的 Hold 内 `preflight_ok` 为 10/10 通过。运行级 preflight 标记仍存在，不能把历史预检标记当作参数效果。
- 两组均为 estimator reset review_required，reset 发生在起飞阶段；不能把它归因于参数，也不能用候选均值掩盖这一质量差异。
- 任务完成率两组均为 100%，无 failsafe、人工接管或 ULog dropout；这只能说明任务链完成，不能证明控制因果。
- Hold 稳定时间两组均为 0 s，没有辨识力。CSV 是接收采样，不能单凭位置指标证明速度环、加速度限幅或执行器饱和的原因。

## 最终结论

**inconclusive**。

数值方向需要根据下面的表读取：候选均值若下降，表示观测指标改善；但本次不能仅凭均值变化接受参数，因为两组都含 estimator reset 和运行级 preflight 标记，且高度误差可能受估计器/仿真状态影响。只有当同一质量门槛下的独立复测重复改善，才可改判 `improved`；若指标重复退化则为 `not_improved`。

## 下一步行动

1. 暂不正式接受 1.900；保留 1.800 作为回退值，不执行参数写入。
2. 若要判定候选，继续做至少 10 次 baseline 1.800 与 10 次 candidate 1.900 的严格配对复测；每次启动后读回并保存参数/版本快照。
3. 接受条件：主轴和三维 RMSE、速度 RMS 在同一质量分层下重复改善；任务完成率不下降；无新增 failsafe、人工接管、关键字段缺失或执行器饱和；科目 3 回归不退化。
4. 拒绝/回退条件：主轴或三维 RMSE 重复未改善、任务失败、failsafe/接管、参数读回错误或数据质量不对称；回退到 `MPC_XY_VEL_P_ACC=1.800`。
"""
    OUT_REPORT.write_text(report)
    OUT_SUMMARY.write_text("# 科目 2 对比摘要（更新版）\n\n最终结论：**inconclusive**。\n\n两组各 10 次，任务完成率均为 100%，但两组均存在 estimator reset 和运行级 preflight 标记。候选是否改善必须在同一质量门槛下继续复测。\n\n"+table(["指标","baseline均值","candidate均值","绝对差","变化率","方向"],changes)+"\n\n完整质量表、逐运行指标、异常和下一步行动见 [subject2_comparison_report_final.md](/Users/mojinliang/Desktop/quadrotor/subject2_comparison_report_final.md)。机器可读数据见 [subject2_comparison_metrics_final.csv](/Users/mojinliang/Desktop/quadrotor/subject2_comparison_metrics_final.csv)。")
    print(OUT_CSV,OUT_REPORT,OUT_SUMMARY)

if __name__ == "__main__": main()
