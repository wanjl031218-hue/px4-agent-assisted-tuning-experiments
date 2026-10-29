#!/usr/bin/env python3
"""Ubuntu SITL: run a ROS scenario, then archive its ROS records and ULog."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid


ROS_FILES = ("samples.csv", "events.csv", "result.txt", "parameters.yaml", "metadata.txt")
AB_MISSION_CAPTURE_MARKER = b"A/B mission build=2026-10-22-r9; ROS recording enabled"


def scenario_command(scenario, run_dir, control_share=None):
    """Build the foreground ROS command for one capture scenario."""
    if scenario == "ab_mission":
        return [
            "ros2", "run", "px4_ab_mission", "ab_mission", "--ros-args",
            "-p", "a_x:=2.0", "-p", "a_y:=0.0",
            "-p", "b_x:=2.0", "-p", "b_y:=2.0",
            "-p", "altitude:=3.0", "-p", "task_seconds:=5.0",
            "-p", "acceptance_radius:=0.30", "-p", "max_stage_seconds:=30.0",
            "-p", f"log_root:={run_dir / 'ros'}",
        ]
    if scenario not in ("single_axis", "hover"):
        raise ValueError(f"unknown scenario: {scenario}")
    if control_share is None:
        raise ValueError("control_share is required for control_experiment scenarios")
    config_name = "single_axis.yaml" if scenario == "single_axis" else "hover_30s.yaml"
    config = Path(control_share) / "config" / config_name
    return [
        "ros2", "run", "px4_control_experiment", "control_experiment", "--ros-args",
        "--params-file", str(config), "-p", f"log_root:={run_dir / 'ros'}",
    ]


def verify_ab_mission_binary(package_prefix):
    """Reject a stale A/B executable before starting a flight."""
    executable = Path(package_prefix) / "lib" / "px4_ab_mission" / "ab_mission"
    if not executable.is_file():
        raise RuntimeError(f"找不到已安装的科目 3 节点：{executable}")
    try:
        binary = executable.read_bytes()
    except OSError as error:
        raise RuntimeError(f"无法读取科目 3 节点：{executable}") from error
    if AB_MISSION_CAPTURE_MARKER not in binary:
        raise RuntimeError(
            "当前 px4_ab_mission 仍是旧二进制，未包含 ROS 记录功能。"
            "请先复制 r9 源码、colcon build，并重新 source ~/px4_ros2_ws/install/setup.bash。"
        )
    return executable


def inventory(log_root):
    result = {}
    for path in log_root.rglob("*.ulg"):
        stat = path.stat()
        result[str(path.resolve())] = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
    return result


def candidates(before, after):
    return [Path(name) for name, identity in after.items() if before.get(name) != identity]


def open_by_px4(path, proc_root=Path("/proc")):
    """Check Linux PX4 file descriptors, not merely a quiet file's modification time."""
    if not proc_root.is_dir():
        raise RuntimeError("此脚本需要在 Ubuntu 虚拟机运行，不能在 Mac 上采集。")
    target = path.resolve()
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes().split(b"\0")[0]
            if Path(os.fsdecode(command)).name != "px4":
                continue
            for fd in (process / "fd").iterdir():
                try:
                    if Path(os.readlink(fd)).resolve() == target:
                        return True
                except FileNotFoundError:
                    continue
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as error:
            raise RuntimeError("无法核对 PX4 日志是否仍打开；请使用运行 SITL 的同一 Ubuntu 用户。") from error
    return False


def ros_directory(run_dir):
    ros_root = run_dir / "ros"
    if not ros_root.is_dir():
        raise RuntimeError(
            f"未找到 ROS 记录目录：{ros_root}。"
            "请确认 Ubuntu 已重新编译并 source 新的 px4_ab_mission；"
            "旧二进制不会创建 samples.csv/result.txt。"
        )
    results = [p for p in ros_root.rglob("result.txt") if "agentdata" not in p.parts]
    if len(results) != 1:
        ros_root = run_dir / "ros"
        entries = sorted(str(path.relative_to(run_dir)) for path in ros_root.rglob("*") if path.is_file())
        detail = "；当前 ROS 文件：" + ", ".join(entries[:8]) if entries else "；ROS 目录为空或不存在"
        raise RuntimeError(
            "未找到唯一的 ROS result.txt。通常表示运行了旧版 px4_ab_mission，"
            "或节点没有使用 log_root。原始 ROS 目录保留，不能标为采集完成" + detail
        )
    directory = results[0].parent
    for name in ROS_FILES:
        path = directory / name
        if not path.is_file() or not path.stat().st_size:
            raise RuntimeError(f"ROS 文件缺失或为空：{path}")
    result = dict(line.split("=", 1) for line in results[0].read_text().splitlines() if "=" in line)
    if result.get("state") != "IDLE":
        raise RuntimeError("ROS 结果尚未回到 IDLE；保留数据，未完成自动归档。")
    # The node writes result.txt after its final sample and then stops writing
    # samples. Therefore the final CSV row may still be a LANDING transition;
    # do not use that row as a stricter completion gate than result.txt.
    return directory


def collect(run_dir, log_root, before, wait_seconds=120):
    source_ros = ros_directory(run_dir)
    for name in ROS_FILES:
        shutil.copy2(source_ros / name, run_dir / name)
    deadline = time.monotonic() + wait_seconds
    announced = False
    while True:
        changed = candidates(before, inventory(log_root))
        if len(changed) > 1:
            raise RuntimeError("多个 ULog 更新，未自动选择。ROS 已保留，请核对后手动复制：\n" + "\n".join(map(str, changed)))
        if changed and not open_by_px4(changed[0]):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("等待 ULog 超时：未找到本次更新的日志，或日志仍被 PX4 打开。ROS 数据已保留。")
        if not announced:
            print("等待 ULog 关闭。飞机落地上锁后，请在 pxh> 执行 logger off；此处会自动继续。", flush=True)
            announced = True
        time.sleep(1)
    source = changed[0]
    identity = source.stat()
    with source.open("rb") as stream:
        if stream.read(7) != b"ULog\x01\x12\x35":
            raise RuntimeError(f"不是可识别的 ULog 文件：{source}")
    pending = run_dir / "flight.ulg.partial"
    shutil.copy2(source, pending)
    after = source.stat()
    if (identity.st_ino, identity.st_size, identity.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns) or open_by_px4(source):
        raise RuntimeError("复制时 ULog 又发生写入；保留 partial 文件，未标为完成。")
    pending.rename(run_dir / "flight.ulg")
    return {
        "ros_source": str(source_ros),
        "ulog_source": str(source),
        "ulog_existed_before_launch": str(source) in before,
        "ulog_scope": "may include startup or other flights; segment before analysis",
        "matching": "only ULog updated during capture; flight timeline not parsed",
        "ulog_validation": "magic header and file closure only; topics and data quality not checked",
        "px4_parameter_and_version_snapshots": "not collected by this script",
    }


def launch(command):
    process = subprocess.Popen(command)
    try:
        return process.wait()
    except KeyboardInterrupt:
        # Foreground ROS receives the same Ctrl+C. Do not send forced termination.
        print(
            "\n正在停止 ROS 任务节点。请在 PX4 SITL 的 pxh> 终端执行：\n"
            "  logger off\n"
            "  logger status\n"
            "完成后本脚本会继续归档 ULog。",
            flush=True,
        )
        return process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=("single_axis", "hover", "ab_mission"))
    parser.add_argument("--output-root", type=Path, default=Path.home() / "px4_experiments")
    parser.add_argument("--px4-root", type=Path, default=Path.home() / "PX4-Autopilot")
    args = parser.parse_args()
    if not Path("/proc").is_dir():
        parser.error("请在 Ubuntu 虚拟机执行，不是在 Mac 或 pxh> 中。")
    if not shutil.which("ros2"):
        parser.error("请先 source ROS Jazzy 和 px4_ros2_ws/install/setup.bash。")
    log_root = args.px4_root.expanduser().resolve() / "build/px4_sitl_default/rootfs/log"
    if not log_root.is_dir():
        parser.error(f"找不到 PX4 日志目录：{log_root}")
    package = "px4_ab_mission" if args.scenario == "ab_mission" else "px4_control_experiment"
    package_prefix = Path(subprocess.check_output(["ros2", "pkg", "prefix", package], text=True).strip())
    if args.scenario == "ab_mission":
        executable = verify_ab_mission_binary(package_prefix)
        print(f"已确认科目 3 采集节点：{executable}", flush=True)
    share = subprocess.check_output(["ros2", "pkg", "prefix", "--share", package], text=True).strip()
    control_share = None if args.scenario == "ab_mission" else share
    if control_share is not None:
        config_name = "single_axis.yaml" if args.scenario == "single_axis" else "hover_30s.yaml"
        config = Path(control_share) / "config" / config_name
        if not config.is_file():
            parser.error(f"找不到任务配置：{config}")
    run_id = f"{args.scenario}_capture_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:8]}"
    run_dir = args.output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    before = inventory(log_root)
    command = scenario_command(args.scenario, run_dir, control_share)
    manifest = {"run_id": run_id, "scenario": args.scenario, "command": command,
                "capture_started_utc": datetime.now(timezone.utc).isoformat(),
                "status": "running", "ulog_inventory_before": before}
    manifest_path = run_dir / "capture_manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(f"本次输出目录：{run_dir}\n任务即将启动。完成且落地上锁后，Ctrl+C 退出 ROS 节点。", flush=True)
    try:
        manifest["ros_exit_code"] = launch(command)
        manifest.update(collect(run_dir, log_root, before))
        manifest["status"] = "ros_and_ulog_collected"
        with (run_dir / "capture.sha256").open("w") as hashes:
            for name in ROS_FILES + ("flight.ulg",):
                with (run_dir / name).open("rb") as stream:
                    digest = hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(hashlib, "file_digest") else hashlib.sha256(stream.read()).hexdigest()
                hashes.write(f"{digest}  {name}\n")
        print(f"采集归档完成：ROS 五个文件 + flight.ulg\n{run_dir}", flush=True)
        return 0
    except (RuntimeError, OSError, KeyboardInterrupt) as error:
        manifest["status"] = "incomplete"
        manifest["error"] = str(error) or "interrupted"
        print(f"未完成归档：{manifest['error']}\n已生成的数据保留在：{run_dir}", file=sys.stderr)
        return 1
    finally:
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    sys.exit(main())
