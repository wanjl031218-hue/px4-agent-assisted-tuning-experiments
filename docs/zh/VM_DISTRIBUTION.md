# Apple Silicon Mac 用 Ubuntu ARM64 实验虚拟机

状态：已打包并完成本地校验；GitHub Release 附件尚未上传完成，目前没有完整的公开下载。

这是项目作者用于 PX4 实验的 Ubuntu 环境。在 **Apple Silicon（M 系列芯片）Mac** 上安装 VMware Fusion 后，可打开 `.vmwarevm` 使用。镜像为 ARM64，不适用于 Intel Mac 或 x86 PC 的直接启动。

## 已核实的版本

| 组件 | 版本／配置 |
|---|---|
| Ubuntu | 24.04.5 LTS，ARM64 |
| 内核 | 7.0.0-38-generic |
| 打包主机 | macOS 26.6.1，arm64 |
| VMware Fusion | 13.6.4，虚拟硬件版本 21 |
| 虚拟机资源 | 5 个虚拟 CPU、5,000 MB 内存、70 GiB 虚拟磁盘 |
| PX4 | v1.17.0 |
| PX4 commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| PX4 工作树 | 有修改，详见版本原始输出 |
| Gazebo Sim | 8.15.0-1~noble（Harmonic） |
| ROS 2 | Jazzy |
| ROS–Gazebo bridge | 1.0.24-1noble.20260921.043401 |
| 科目 1、2 节点 | `2026-10-14-v3` |
| 科目 3 节点 | `2026-10-22-r9` |

以上信息于 2026-10-29 从发布副本中读取。[版本清单](../../distribution/environment.json) · [原始输出](../../distribution/verified_versions.txt)。

## 登录与导入

作者已授权公开的登录账号：用户名 `wjl`，密码 `jlkj`。导入个人副本后请更换密码。

待 Release 上传完成后，将全部 **18 个分卷（part000 至 part017）** 和 `SHA256SUMS` 下载到同一文件夹。总下载量约 35.4 GB（33.0 GiB），解压后约 63 GiB；下载和解压建议预留 **110 GiB** 主机磁盘空间。

在 Mac 终端进入下载目录，先校验：

```bash
shasum -a 256 -c SHA256SUMS
```

全部显示 `OK` 后，直接合并解压，无需再保存一份合并包：

```bash
cat PX4_Ubuntu_ARM64_2026-10-29.tar.gz.part??? | tar -xzf -
open "PX4_Ubuntu_ARM64.vmwarevm"
```

VMware 若询问移动还是复制，选择“我已复制”。安装 ISO 不再需要，发布配置已移除原主机上的 ISO 路径。

后续按 [实验总 SOP](SOP.md#2-实验方法和路径) 操作。Ubuntu 内的项目路径为 `~/PX4-Autopilot`、`~/px4_ros2_ws`、`~/px4_tools`、`~/px4_experiments`。重新采集基准前先读取实际 PX4 参数。

## 验证范围与仓库结构

发布副本已经启动、验证登录并正常关机；完整压缩流通过 macOS `tar` 读取校验，各分卷包含 SHA-256。尚未执行“重新解压后完成一次飞行和采集”的验证，其他主机／VMware 版本的兼容性也未验证。

`vm-distribution` 分支保存环境信息和导入说明；大体积镜像使用 GitHub Release 附件分发，不进入 Git 提交历史。原工作虚拟机保留。
