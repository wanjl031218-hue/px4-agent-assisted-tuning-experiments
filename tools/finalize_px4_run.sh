#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
用法：
  finalize_px4_run.sh RUN_DIR PX4_PARAMS PX4_VERSION SNAPSHOT_TIME FLIGHT_ULG [ROS_DATA_DIR]

参数：
  RUN_DIR       单次运行根目录，例如 ~/px4_experiments/single_axis_<run_id>
  PX4_PARAMS    从 PX4 pxh> 的 param show 输出保存的文本文件
  PX4_VERSION   ver all、git hash 和工作树状态文本文件
  SNAPSHOT_TIME 参数快照时间文本文件
  FLIGHT_ULG    本次运行对应的 PX4 ULog
  ROS_DATA_DIR  本次 ROS 完整记录目录（v3 为 allfile_*，不要选 agentdata）
                省略时要求 ROS 文件已经在 RUN_DIR 根目录

归档前请确认 ULog 和 ROS 文件属于同一次运行，且 ULog 已停止写入。
本脚本检查文件齐全性，不解析 ULog，不证明两者的时间对应或飞行成功。
EOF
  exit 2
}

[[ $# -eq 5 || $# -eq 6 ]] || usage

run_dir=$1
px4_params=$2
px4_version=$3
snapshot_time=$4
flight_ulg=$5
ros_data_dir=${6:-$run_dir}

[[ -d "$run_dir" ]] || { echo "运行目录不存在: $run_dir" >&2; exit 1; }
[[ -d "$ros_data_dir" ]] || { echo "ROS 记录目录不存在: $ros_data_dir" >&2; exit 1; }
ros_names=(samples.csv events.csv result.txt parameters.yaml metadata.txt)
for name in "${ros_names[@]}"; do
  [[ -s "$ros_data_dir/$name" ]] || {
    echo "ROS 记录缺失或为空: ${ros_data_dir}/${name}；未归档。v3 请选择本次 allfile_* 目录。" >&2
    exit 1
  }
done
sources=("$px4_params" "$px4_version" "$snapshot_time" "$flight_ulg")
names=(px4_params.txt px4_version.txt snapshot_time.txt flight.ulg)
for name in "${ros_names[@]}"; do
  sources+=("$ros_data_dir/$name")
  names+=("$name")
done
for source_file in "${sources[@]}"; do
  [[ -s "$source_file" ]] || { echo "文件不存在或为空: $source_file" >&2; exit 1; }
done

# 先检查全部目标，避免发现冲突之前已覆盖部分原始记录。
for i in "${!sources[@]}"; do
  target="$run_dir/${names[$i]}"
  if [[ -e "$target" ]] && ! cmp -s -- "${sources[$i]}" "$target"; then
    echo "目标已有不同内容，拒绝覆盖: ${target}。请使用新的归档目录。" >&2
    exit 1
  fi
done
for i in "${!sources[@]}"; do
  target="$run_dir/${names[$i]}"
  if [[ ! -e "$target" ]]; then
    cp -- "${sources[$i]}" "$target"
  fi
done

if command -v sha256sum >/dev/null 2>&1; then
  (
    cd "$run_dir"
    sha256sum "${names[@]}" > artifacts.sha256
  )
fi

echo "完整性检查通过（9/9 文件，尚未验证时间对应和数据质量）: $run_dir"
