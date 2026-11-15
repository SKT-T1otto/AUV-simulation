# AUV-simulation

ROS 2 Jazzy + Gazebo Harmonic 单艇三维航点跟踪工程示例。
六自由度艇体、八个推进器、中性浮力、水动力阻尼与附加质量，使用真值里程计反馈闭环控制。
控制过程包含姿态稳定、有界推力分配、推进器一阶响应与变化率限制。

此模型尚未经实艇标定，未接入强化学习，不能视为经过验证的 BlueROV2 高保真模型。
Gazebo 负责运动积分；Python 节点只计算控制指令和记录指标。

## 入口

- `crk_auv_singleboat_v1/`：Linux ROS 2 工作空间，含单艇模型、控制器、测试和运行脚本。
- `crk_auv_singleboat_v1/README_zh.md`：安装、运行、验收和问题定位说明。
- `package_singleboat_delivery.py`：按需生成 tar.gz、ZIP 包和校验清单。
- `crk_auv_singleboat_v1/VALIDATION.md`：离线验证结果与实际运行验收边界。

## 使用

前提：已有安装 ROS 2 Jazzy、`ros-jazzy-ros-gz` 和 `ros-dev-tools` 的 Ubuntu 24.04 环境，
且 NVIDIA GPU、X11 和 Gazebo 窗口已验证可用。宿主机可以保持 Ubuntu 20.04，仿真在 Ubuntu 24.04 容器内运行。

使用已有 `crk-ros2-dev` 容器时，在 Ubuntu 宿主机执行：

```bash
git clone https://github.com/SKT-T1otto/AUV-simulation.git
sudo docker start crk-ros2-dev
sudo docker cp AUV-simulation crk-ros2-dev:/root/
sudo docker exec -it crk-ros2-dev bash
```

在容器内执行（首次复制安装；再次更新先保留既有运行结果）：

```bash
cd /root/AUV-simulation/crk_auv_singleboat_v1
sha256sum -c SHA256SUMS
bash install_and_build.sh
bash run_demo.sh
```

先用 Ctrl+C 停止之前的 `shapes.sdf` 与测试时钟桥接。
`run_demo.sh` 在一个终端中管理 Gazebo、消息桥接及控制器，结果保存在工作空间 `results/run_*` 下。
完整的压缩包传输方案和诊断步骤见 `crk_auv_singleboat_v1/README_zh.md`。

默认航点为 `(0,0,-2)`、`(2,0,-2)`、`(2,2,-3)`、`(0,2,-1.5)`、`(0,0,-2)`。
世界坐标 `z` 向上，航点处于水下。每个航点要求距离不超过 0.2 m、速度不超过 0.1 m/s，持续 2 s；最终保持 10 s。
实际完成状态以本次 `summary.json` 的 `termination_reason`、`waypoints_accepted` 和 `engineering_acceptance_passed` 为准。

本项目独立于既有论文实验，不改变论文既有任务、奖励、观测、动作、通信或成功判据。

## 打包

在本目录执行 `python package_singleboat_delivery.py`。
压缩包包含 Linux 工作空间；不包含 Python 缓存、构建产物、实验结果或工作区历史资料。
生成压缩包、运行数据、机器路径清单及本地工作区迁移审计不提交到 Git。

## 离线检查

不依赖 ROS 的数值检查需要 NumPy、SciPy：

```bash
cd crk_auv_singleboat_v1
python3 -m unittest discover -s tests -v
```

## 验证范围

已完成离线控制、几何、浮力配平和脚本语法检查；Gazebo 闭环仍需用户在 Ubuntu 容器内实际验收。
