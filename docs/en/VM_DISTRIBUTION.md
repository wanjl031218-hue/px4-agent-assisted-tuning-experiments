# Ubuntu ARM64 VM for Apple Silicon Macs

Status: packaged and locally verified; GitHub Release attachment upload is pending. There is no complete public VM download yet.

This is the project author's prepared Ubuntu environment for the PX4 experiments. Open the `.vmwarevm` bundle in VMware Fusion on an **Apple Silicon Mac (M-series)**. It is an ARM64 guest, not an Intel Mac or x86 PC image. VMware Fusion must be installed separately.

## Verified environment

| Component | Version / configuration |
|---|---|
| Guest | Ubuntu 24.04.5 LTS, ARM64 |
| Guest kernel | 7.0.0-38-generic |
| Packaging host | macOS 26.6.1, arm64 |
| VMware Fusion | 13.6.4; virtual hardware version 21 |
| VM resources | 5 virtual CPUs, 5,000 MB RAM, 70 GiB virtual disk |
| PX4 | v1.17.0 |
| PX4 source commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| PX4 working tree | Modified; see `verified_versions.txt` for the recorded status |
| Gazebo Sim | 8.15.0-1~noble (Harmonic) |
| ROS 2 | Jazzy |
| ROS–Gazebo bridge | 1.0.24-1noble.20260921.043401 |
| Subjects 1/2 executable | `control_experiment build=2026-10-14-v3` |
| Subject 3 executable | `A/B mission build=2026-10-22-r9` |

Version evidence was collected inside the distribution VM on 2026-10-29. [Machine-readable manifest](../../distribution/environment.json) · [Raw version output](../../distribution/verified_versions.txt).

## Login

The owner has explicitly provided this public distribution login:

- Username: `wjl`
- Password: `jlkj`

Change the guest password after importing your own copy.

## Download and import

Once the Release upload is complete, download **all 18 parts** (`part000` through `part017`) and `SHA256SUMS` into one folder. Total download size is 35,430,763,970 bytes (35.4 GB / approximately 33.0 GiB). Each part is below GitHub's 2 GiB attachment limit. The archive expands to approximately 63 GiB; allow at least **110 GiB of free host disk space** for download and extraction.

In macOS Terminal, enter that folder and run:

```bash
shasum -a 256 -c SHA256SUMS
```

Proceed only if every part reports `OK`. Extract directly from the parts without creating another large merged archive:

```bash
cat PX4_Ubuntu_ARM64_2026-10-29.tar.gz.part??? | tar -xzf -
open "PX4_Ubuntu_ARM64.vmwarevm"
```

In VMware Fusion, choose **I copied it** if prompted. Log in with the account above. The installation ISO is not required; its host-specific path was removed from the distribution configuration.

Then follow the [experiment SOP](SOP.md#2-method-and-execution-path). The guest project locations are `~/PX4-Autopilot`, `~/px4_ros2_ws`, `~/px4_tools` and `~/px4_experiments`. Use the existing installed environment; read back the active PX4 parameters before collecting a new baseline.

## Verification and publication layout

- A separate distribution copy was created; the original working VM was preserved.
- The distribution copy booted, accepted the documented login, and shut down normally.
- Every archive part has a SHA-256 checksum. macOS `tar` successfully read the complete combined compressed archive.
- A fresh extraction and complete flight/capture test have **not** been performed. Compatibility beyond the recorded host/Fusion combination is unverified.
- The `vm-distribution` branch contains the manifest and import documentation. Large VM parts belong in GitHub Release attachments, not in Git history.
