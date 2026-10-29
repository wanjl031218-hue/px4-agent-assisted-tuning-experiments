# PX4 + ROS 2 + Gazebo + Agent 实验总 SOP

版本：v1.0  
日期：2026-10-19  
适用环境：Ubuntu 24.04.5 ARM64 VM、PX4 v1.17、Gazebo Harmonic、ROS 2 Jazzy、X500 SITL

## 1. 目标

在固定仿真工况、固定任务配置、固定参数和固定评价规则下，建立可复现的 PX4 控制实验链：

```text
Gazebo 仿真 → PX4 状态估计与控制 → MicroXRCEAgent → ROS 2 任务/记录节点
→ 基准数据 → Agent 数据分析 → PX4 参数候选 → 复测对比 → 实验结论
```

第一阶段要回答：Agent 能否根据实验数据识别控制问题，并提出可验证、可回退的 PX4 参数优化建议。

边界：Agent 只做离线分析和建议，不实时控制飞机，不直接写参数，不修改 ROS 任务代码、PX4 控制算法或 Gazebo 模型。

当前固定版本：

| 项目 | 固定值 |
|---|---|
| PX4 | v1.17；运行版本和 source commit 每次记录 |
| Gazebo | Harmonic 8.15.0 |
| ROS 2 | Jazzy |
| 通信桥 | `MicroXRCEAgent udp4 -p 8888` |
| 任务节点 | `px4_control_experiment` v3 |
| A/B 回归节点 | `px4_ab_mission` r9 |
| Agent | GPT-6.1-sol，离线分析 |

### 1.1 实验目标与链路

本 SOP 的核心目标是：在固定仿真环境、任务、参数和评分规则下，形成“采集 → 审计 → 诊断 → 单参数候选 → 复测 → 回归 → 结论”的可追溯闭环。Agent 只进行离线数据分析和参数建议，不参与实时控制，不执行 PX4 参数写入，不修改 ROS 2、PX4 控制算法或 Gazebo 模型。

实验链路固定为：

```text
Gazebo 物理与传感器 → PX4 估计/模式/控制/执行器 → MicroXRCEAgent（UDP 8888）
→ ROS 2 任务与记录节点 → samples.csv/events.csv + flight.ulg
→ 基准审计 → 参数候选 → 人工地面修改 → 同工况复测 → 科目 1/2/3 回归
```

三类任务及其主要用途：

| 科目 | 固定流程 | 主要观察 | 首轮相关参数 |
|---|---|---|---|
| 科目 1 | 起飞 3 m → 北向 +2 m → 保持 5 s → 降落上锁 | 单轴 RMSE、超调、稳定时间、速度 RMS | `MPC_XY_VEL_P_ACC`；速度环证据充分后再看 `MPC_XY_P` |
| 科目 2 | 起飞 3 m → 悬停 30 s → 降落上锁 | 三轴位置误差、漂移、速度 RMS、稳态误差 | `MPC_XY_VEL_P_ACC`；只有持续带符号速度偏差才看 `MPC_XY_VEL_I_ACC` |
| 科目 3 | 起飞 → A → `DO_TASK` 5 s → B → 自动降落上锁 | 完整任务可靠性、A/B 跟踪、A 点保持、起降过程 | 首轮开放 `MPC_XY_VEL_P_ACC`；其余参数按证据条件开放 |

每次运行必须保留唯一 `run_id`，并将以下文件放在同一运行目录：`samples.csv`、`events.csv`、`result.txt`、`parameters.yaml`、`metadata.txt`、`px4_params.txt`、`px4_version.txt`、`snapshot_time.txt`、`flight.ulg`、`capture_manifest.json`、`capture.sha256` 和 `ros/`。其中 `parameters.yaml` 是 ROS 任务配置，`px4_params.txt` 才是 PX4 `pxh>` 的参数读回。必须逐项核对哈希、文件内容、时间戳单调性、NaN、`valid` 字段、消息新鲜度、事件窗口、reset、failsafe、人工接管、任务状态、版本和参数。缺少 ULog、参数快照或关键字段时，只能标记证据缺口，不能猜测历史值。

运行分为 `clean_candidate`、`review_required` 和 `invalid_for_metric`。失败、reset、NaN、通信波动、预检异常和人工接管的运行都保留；`review_required` 不得混入 clean 主均值，但在任务配置、版本和任务结果一致且目标任务指标窗口可用时，可以进入任务基准并标记限制。参数归因仍必须单独检查参数快照、匹配 ULog 和控制窗口。指标必须在相同坐标系（PX4 local NED，x 北、y 东、z 向下）、相同事件窗口、相同筛选规则下计算，NaN 不填 0，无法可靠计算写 `unavailable`。

参数复测的最小闭环是：记录修改前后 `param show`、`ver all`、源码 commit 和时间；一次只改变一个候选参数；保存当前值作为回退值；固定任务、版本、模型、初始状态和评分器；候选与基线各至少 10 次；除科目 1/2 主指标外必须检查科目 3 回归。最终结论只能为 `improved`、`not_improved` 或 `inconclusive`：单次变好、任务成功一次或单个指标变好都不足以宣布优化成功。

## 2. 实验方法和路径

### 2.1 实验前提

每次只运行一份 PX4、Gazebo、MicroXRCEAgent、`/clock` 桥和一个任务节点。开始前确认飞机已落地并上锁，QGroundControl 的 Virtual Stick 已关闭。三个科目都使用同一个采集脚本；脚本为每次运行创建独立目录，并同时保存 ROS 记录和本次更新的 ULog。

### 2.2 启动四个终端

#### 终端 1：PX4 + Gazebo

```bash
unset GZ_CONFIG_PATH
export PATH=/usr/bin:/bin:$PATH
hash -r
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

出现 `pxh>` 后保持终端运行。实验前和实验后都在这里执行 logger 命令。

#### 终端 2：PX4 ↔ ROS 2 通信桥

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

#### 终端 3：Gazebo `/clock` 桥

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

#### 终端 4：QGroundControl

```bash
cd ~/Downloads
./QGroundControl-aarch64.AppImage
```

QGroundControl 只用于观察状态和模式，不提供本实验的任务点，也不执行人工接管。

### 2.3 每次实验的 logger 操作

任务启动前，在终端 1 的 `pxh>` 执行：

```text
logger status
logger on
logger status
```

确认 logger 已开始记录后，再启动 ROS 任务。任务完成、飞机已落地并上锁后，先在运行 ROS 任务的终端按一次 `Ctrl+C`，再回到 `pxh>` 执行：

```text
logger off
logger status
```

`logger off` 后才能归档本次 `flight.ulg`。如果仍显示 `Not logging`、日志未关闭或采集脚本报多个 ULog，不要用“最新文件”猜测匹配关系；保留该次目录并单独标记。

### 2.4 三个科目的启动代码

三个命令都在新的 Ubuntu 普通终端执行；每次实验前重新 `source`，每次运行自动生成新的目录。

#### 科目 1：单轴水平位置响应

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/px4_tools/run_px4_capture.py single_axis \
  --output-root "$HOME/px4_experiments"
```

任务流程：起飞 3 m → 北向移动 2 m → 保持 5 s → 自动降落 → `IDLE`。

#### 科目 2：悬停稳定性

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/px4_tools/run_px4_capture.py hover \
  --output-root "$HOME/px4_experiments"
```

任务流程：起飞 3 m → 悬停 30 s → 自动降落 → `IDLE`。

#### 科目 3：A/B 完整任务

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 ~/px4_tools/run_px4_capture.py ab_mission \
  --output-root "$HOME/px4_experiments"
```

任务流程：起飞 → 到达 A → `DO_TASK` 保持 5 s → 到达 B → PX4 自动降落并上锁 → `IDLE`。

### 2.5 参数修改示例

以下示例在 PX4 `pxh>` 中把 `MPC_XY_VEL_P_ACC` 从 `1.800` 改为 `1.900`。修改前先保存参数快照，修改后立即读回确认：

```text
param show MPC_XY_VEL_P_ACC
param set MPC_XY_VEL_P_ACC 1.900
param show MPC_XY_VEL_P_ACC
param save
```

如需回退：

```text
param set MPC_XY_VEL_P_ACC 1.800
param show MPC_XY_VEL_P_ACC
param save
```

一次候选实验只修改约定的参数；任务配置、PX4 版本、Gazebo 模型和评价规则保持不变。

### 2.6 目录内容

GitHub 项目根目录保留：

```text
README.md                         
PX4_AI实验总SOP.md
figures/subject1/                   # 科目 1 SVG 图表
figures/subject2/          # 科目 2 SVG 图表
实验报告_科目1_MPC_XY_VEL_P_ACC_1.800_vs_1.900.md
subject2/subject2_comparison_report_final2.md
baseline_report.md                # 科目 3 基准报告
comparison_report.md              # 科目 3 基线/候选对比
optimization_recommendation_subject3.md
tools/
tutorials/
```

每次实验的 `~/px4_experiments/<run_id>/` 独立保存：

```text
samples.csv
events.csv
result.txt
parameters.yaml
metadata.txt
px4_params.txt
px4_version.txt
snapshot_time.txt
flight.ulg
capture_manifest.json
capture.sha256
ros/
```

## 3. 实验链路

```text
Gazebo：物理世界、传感器和仿真时间
  ↓
PX4：状态估计、模式管理、位置环、速度环、姿态控制和执行器
  ↓  uXRCE-DDS / UDP 8888
MicroXRCEAgent：PX4 与 ROS 2 的通信桥
  ↓
ROS 2：任务状态机、轨迹设定、状态订阅和 CSV 记录
  ↓
flight.ulg + samples.csv + events.csv
  ↓
基准助手：质量审计和指标统计
  ↓
优化助手：识别控制问题并提出 PX4 参数候选
  ↓
人工核对、地面修改、同工况复测
  ↓
数据处理与对比助手：improved / not_improved / inconclusive
```

控制权划分：任务节点发送目标，PX4 计算如何执行，Gazebo 模拟执行结果，ROS 读取反馈；Agent 不直接参与飞行控制闭环。

## 4. 实验科目和代表参数

| 科目 | 代表参数 | 当前参数 |
|---|---|---|
| 科目 1：单轴水平位置响应 | `MPC_XY_P`、`MPC_XY_VEL_P_ACC`、`MPC_XY_VEL_I_ACC` | `0.9500`、`1.800`、`0.400` |
| 科目 2：悬停稳定性 | `MPC_XY_VEL_P_ACC`、`MPC_XY_VEL_I_ACC` | `1.800`、`0.400` |
| 科目 3：A/B 完整任务 | 首轮 `MPC_XY_VEL_P_ACC`；后续按证据查看 `MPC_XY_P`、`MPC_XY_VEL_I_ACC`、`MPC_ACC_HOR_MAX`、`MPC_XY_VEL_MAX` | `1.800`、`0.9500`、`0.400`、`5.000`、`12.000` |

## 5. 边界与工况

| 条件 | 固定设置 |
|---|---|
| 风场 | 无风，不加入外部扰动 |
| 温度 | Gazebo 默认且固定 |
| 电池 | 每次同一初始状态，目标 SOC 100%；记录电压和剩余容量 |
| 电机 | 四个电机正常，不注入故障 |
| 初始位置 | 同一 local NED 原点 |
| 初始速度 | 0 |
| 初始姿态 | 水平 |
| 初始角速度 | 0 |
| 初始航向 | 固定 |
| 飞机状态 | 落地、上锁 |
| QGC | Virtual Stick 关闭，不做人工接管 |
| 任务配置 | 科目 YAML、A/B 坐标和容差固定 |
| PX4/ROS/Gazebo | 版本、commit、节点 build 固定 |
| 数据记录 | 每次独立 `run_id`；ROS 文件和 ULog 必须属于同一次运行 |

基准样本量：科目 1 至少 10 次；候选参数每组至少 10 次；完成科目 1 后用科目 2 和科目 3 回归。
## 6. 提示词和输出

### 6.0 统一数据解释原则

```text
本项目在 Ubuntu 虚拟机中的 PX4 SITL、Gazebo 和 DDS/UDP 通信会受到算力和调度影响。局部 freshness 超时、时间戳抖动、探针信息缺失、非关键 NaN、estimator reset 或启动阶段 preflight warning，不自动判定整次运行无效。

请按“任务指标”和“控制归因指标”分别判断数据：
- 任务指标：任务完成率、阶段耗时、降落上锁、任务状态；
- 控制归因指标：位置/速度误差、限幅、控制设定和参数变化因果。

`review_required` 运行可以用于任务表现，也可以在目标窗口仍可靠时作为 `usable_with_caveats` 证据。只有问题发生在目标指标窗口，并直接破坏该指标、任务身份或参数身份时，才将受影响的指标标记为 `invalid_for_metric`。缺少某个指标的数据时只写 `unavailable`，不要把整次运行或其他指标一起删除，不要把 NaN 填成 0。
```

### 6.1 基准构建 Prompt：`px4-baseline-agent-v1.2`

```text
你是 PX4 三科目实验的基准数据分析助手。输入可能是科目 1 单轴水平响应、科目 2 30 秒悬停或科目 3 A→DO_TASK→B 完整任务。读取实验目录中的 ROS 记录、ULog、任务配置、参数和版本信息；坐标系使用 PX4 local NED。

只输出一个 `baseline_report.md`，不要生成 CSV、JSON、图表或其他文件。报告只保留：
1. 数据质量表：run_id、任务是否完成、data_quality、主要异常、任务指标是否可用、控制归因是否可用；
2. 指标汇总表：按科目填写核心指标；
3. 简短说明：哪些数据可用、哪些指标缺失、限制是什么；
4. 一句结论：可建立任务基准、可建立参数归因基准，或只能做部分指标基准。

按以下规则分析：
- 先确认科目、任务配置、版本和参数身份；缺少外部参数快照或严格匹配 ULog 时，保留运行，但不要声称可以完成参数因果归因；
- 按阶段和指标判断 `usable`、`usable_with_caveats` 或 `invalid_for_metric`，不要按整包一次性判定；
- SITL 算力造成的非关键 reset、freshness、时间戳或 preflight 问题，只作为限制说明；只有它们破坏目标窗口时才排除对应指标；
- 科目 1 统计单轴误差、超调、稳定时间和速度 RMS；科目 2 统计位置 RMS、速度 RMS、最大漂移和稳态误差；科目 3 统计任务完成率、阶段耗时、A/B 误差、A 点保持和降落结果；
- A/B 误差或某个速度指标缺失时写 `unavailable`，仍可报告任务完成率和阶段耗时；
- 保留失败、reset、NaN、通信波动和 `review_required` 运行；不提出参数修改建议，不猜测缺失值。
```

### 6.2 生成优化建议 Prompt：`px4-param-agent-v0.6`

```text
你是 PX4 三科目实验的离线控制性能分析助手。输入为基准报告和一个或多个实验目录，科目可以是：
- 科目 1：单轴水平位置响应；
- 科目 2：30 秒悬停稳定性；
- 科目 3：A→DO_TASK→B 完整任务。

只输出一个 `optimization_recommendation.md`，不要生成其他文件。报告只保留：
1. 问题表：科目、阶段、现象、核心指标、证据状态、限制；
2. 参数建议表：参数、当前值、建议值或“不修改”、控制环节、证据、风险、回退值、下一轮验证；
3. 3—5 句结论：修改、保持不变或补测。

分析顺序：
1. 确认任务、版本、参数、任务节点 build 和评价规则是否一致；
2. 按阶段检查样本和 ULog 的目标/实测状态、valid、时间戳、reset、模式、failsafe 和人工接管；
3. 先区分控制现象与 SITL 算力、通信、估计器、预检或记录问题；
4. 对每个目标指标单独标记 `usable`、`usable_with_caveats` 或 `invalid_for_metric`。

数据规则：
- `review_required` 不等于整次运行不可用；只要目标窗口连续、字段有效且现象可解释，就可以提出带限制条件的建议；
- 局部 NaN、freshness 波动、时间戳异常或 estimator reset 若不影响目标窗口，只在限制栏说明；
- 缺少 A/B 误差、速度设定或 ULog 时，只限制对应指标；不能用位置数据强行证明速度环或限幅问题；
- 缺少参数快照时，不编造当前值或建议值；可以给出控制环假设和需要补齐的证据；
- Agent 只提出候选，不执行 `param set`，不修改任务、评分器、代码或模型。

参数顺序：
- 科目 1/2 首轮观察 `MPC_XY_VEL_P_ACC`；
- 科目 3 首轮观察 A/B 水平速度与到达表现，再决定是否提出 `MPC_XY_VEL_P_ACC`；
- 只有位置环证据充分才考虑 `MPC_XY_P`；
- 只有持续带符号速度偏差才考虑 `MPC_XY_VEL_I_ACC`；
- 只有 ULog 证明触发限制才考虑 `MPC_ACC_HOR_MAX` 或 `MPC_XY_VEL_MAX`；
- 垂直参数只在起飞或降落存在可重复垂直问题时考虑。

证据足够时给出一个小步长、可回退、可验证的候选；证据不足时写“不修改”和针对性补测，不要因为非关键环境报错直接拒绝所有建议。
```

### 6.3 输出处理与对比 Prompt：`px4-data-compare-v0.3`

```text
你是 PX4 三科目复测数据处理与对比助手。比较同一科目的 baseline_runs 和 candidate_runs，并使用相同任务、版本、参数记录、事件窗口、字段和指标公式。

只输出一个 `comparison_report.md`，不要生成其他文件。报告只保留：
1. 数据质量表：组别、运行数、任务指标可用数、控制指标可用数、失败数、review_required 数量；
2. 指标对比表：科目、指标、基线值、候选值、变化、评价、数据限制；
3. 简短说明：异常、缺失和是否影响结论；
4. 一句结论：`improved`、`not_improved` 或 `inconclusive`；
5. 一句下一步：接受、回退或补测。

处理规则：
- 任务完成率、阶段耗时、降落上锁可以使用任务指标有效的 `review_required` 运行；
- A/B 误差、速度环、限幅和参数因果只使用对应窗口有效的证据，并写明限制；
- 某个指标缺失时只将该指标写为 `unavailable` 或 `inconclusive`，仍保留其他可用指标；
- SITL 算力、通信和 estimator reset 造成的非关键波动作为限制说明，不自动否定整组数据；
- 不填补 NaN，不删除失败运行，不因为单个指标或单次运行变好就宣布改善；
- 只有任务、参数身份、样本量或主要指标证据无法比较时，整体参数优化结论才为 `inconclusive`。
```

## 7. 实验报告与文件索引

本节只记录各科目的报告和对应数据文件；详细指标、逐运行结果和限制以链接文件为准。

| 科目 | 实验报告 | 相关数据/分析文件 |
|---|---|---|
| 科目 1：单轴水平响应 | [实验报告_科目1_MPC_XY_VEL_P_ACC_1.800_vs_1.900.md](subject1_report.md) | [科目1参数优化闭环计划.md](../../科目1参数优化闭环计划.md)、[figures/subject1/](figures/subject1/)、对应运行目录中的 `samples.csv`、`events.csv`、`flight.ulg` |
| 科目 2：悬停稳定性 | [subject2/subject2_comparison_report_final2.md](subject2_report.md) | [subject2/baseline_summary.md](../../subject2/baseline_summary.md)、[subject2/subject2_comparison_summary_final.md](../../subject2/subject2_comparison_summary_final.md)、[figures/subject2/](figures/subject2/)、对应运行目录 |
| 科目 3：A/B 完整任务 | [baseline_report.md](subject3_baseline.md)、[comparison_report.md](subject3_report.md) | [optimization_recommendation_subject3.md](subject3_analysis.md)、对应运行目录中的 ROS 记录、参数快照和 ULog |

所有报告均应保留原始运行目录、Agent 输入输出、参数修改记录和最终结论。
## 8. 结论

本轮三个科目的最终结果如下：

| 科目 | 结果 | 证据摘要 |
|---|---|---|
| 科目 1：单轴水平响应 | `provisional improved` | 1.900 在同工况下使主轴 RMSE、最大超调、稳定时间和 speed RMS 下降；任务完成率保持 100%。结论只适用于科目 1 的当前 SITL 工况。 |
| 科目 2：悬停稳定性 | `inconclusive` | 横向、高度、三维和速度指标下降，但主轴 RMSE 上升，稳定时间无区分度，且有效 1.900 candidate 只有 9 次。 |
| 科目 3：A/B 完整任务 | `not_improved` | 任务完成率和降落可靠性不变，但 A 点误差增加约 36%，B 点误差下降约 20%，没有同时改善。 |

整体结论：**`not_improved`**。本轮没有证明 `MPC_XY_VEL_P_ACC=1.900` 能改善三个科目的综合飞行表现；参数决策为**回退到 `1.800`**。Agent 辅助分析、参数候选、复测、回归和报告闭环已经建立，但结论仍限定在当前 PX4 SITL、Gazebo、ROS 2 和任务配置，不能外推到真实硬件。reset、preflight、freshness、时间戳和 ULog/ROS 匹配缺口作为实验限制保留，不把它们误写成参数导致的故障。

每一轮实验最后只允许形成以下三类结论：

| 结论 | 含义 |
|---|---|
| `improved` | 候选在预先定义的主指标上改善，约束指标和科目 2/3 回归均未退化，证据充分 |
| `not_improved` | 候选没有改善或出现明确退化，回退参数 |
| `inconclusive` | 数据质量、样本量、参数快照、ULog 或控制证据不足，先补测 |

不得因为任务成功一次、单个指标变好或 Agent 的文字判断就宣布优化成功。最终报告必须保留原始实验包、Agent 输入输出、参数修改记录、复测结果和限制。
