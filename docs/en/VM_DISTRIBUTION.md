# Planned Ubuntu VM Distribution

Status: planned; no VM image has been exported or uploaded.

A prepared Ubuntu VM can provide a convenient starting environment. The current guest is Ubuntu 24.04.5 ARM64, so users need a compatible ARM64 host and hypervisor. An x86 PC requires a separate environment build; downloading the ARM64 image alone does not make it compatible.

## Distribution layout

| Location | Contents |
|---|---|
| GitHub repository | Code, bilingual documentation, configuration and environment manifest |
| Release or external artifact hosting | Compressed VM image, SHA-256 checksum and import instructions |
| Optional dataset download | Example ROS + ULog runs, distributed separately from the VM |

Keep the VM out of Git history. Select hosting after checking the exported image size and the host's current upload limits. If the image is too large for a single release asset, use external artifact hosting and link it from the release.

## Preparation before release

1. Create a separate distribution copy of the VM, preserving the working VM.
2. Remove personal documents, browser sessions, API keys, tokens, SSH keys and saved credentials from the distribution copy. Do not embed a private agent API key.
3. Record the guest architecture, hypervisor/export format, allocated CPU/RAM/disk, PX4 commit, ROS/Gazebo/px4_msgs versions and task builds. Document upstream licenses for redistributed dependencies.
4. Shut down the guest, export/package the VM, and generate a SHA-256 checksum.
5. Import it as a new VM and verify PX4/Gazebo, UDP 8888, `/clock`, one mission, ROS records, logger closure and ULog archival.
6. Publish the artifact and import instructions, then add its download link to the project README.

Use “download, import, then follow setup instructions” until a clean import has actually passed the startup and capture test.
