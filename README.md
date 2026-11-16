# AUV-simulation

ROS 2 Jazzy + Gazebo Harmonic 单艇三维航点跟踪工程示例。
六自由度艇体、八个推进器、中性浮力、水动力阻尼与附加质量，使用真值里程计反馈闭环控制。
控制过程包含姿态稳定、有界推力分配、推进器一阶响应与变化率限制。

此模型尚未经实艇标定，不能视为经过验证的 BlueROV2 高保真模型。
已提供 28D 观测 / 3D 残差的 Python 策略接口；尚未选定和加载用户的历史训练模型。
Gazebo 负责运动积分；Python 节点只计算控制指令和记录指标。

## 入口

- `crk_auv_singleboat_v1/`：Linux ROS 2 工作空间，含单艇模型、控制器、测试和运行脚本。
- `crk_auv_ch3_v1/`：第三章分阶段改造；G3 静态场景已验收，新增 G4 四艇运动控制测试。
- `tools/run_ch3_motion.sh`：四艇运动入口；参数依据和验收见 `crk_auv_ch3_v1/G4_MOTION.md`。
- `crk_auv_ch3_v1/IMPLEMENTATION_PLAN.md`：阶段顺序；G4 需要 Linux 实测通过后再加入地图/任务。
- `crk_auv_singleboat_v1/README_zh.md`：安装、运行、验收和问题定位说明。
- `tools/run_linux.sh`：宿主机启动入口，配置目录挂载并直接保存结果。
- `crk_auv_singleboat_v1/HOST_OUTPUT.md`：`/home/legion/AUV-simulation` 的一次性配置与日常运行。
- `tools/update_source_manifest.py`：更新源文件校验清单，不生成压缩包。
- `crk_auv_singleboat_v1/POLICY_INTEGRATION.md`：策略、外部推理话题、TorchScript 导出契约。
- `crk_auv_singleboat_v1/VALIDATION.md`：离线验证结果与实际运行验收边界。

## 使用

前提：已有安装 ROS 2 Jazzy、`ros-jazzy-ros-gz` 和 `ros-dev-tools` 的 Ubuntu 24.04 环境，
且 NVIDIA GPU、X11 和 Gazebo 窗口已验证可用。宿主机可以保持 Ubuntu 20.04，仿真在 Ubuntu 24.04 容器内运行。

使用已有 `crk-ros2-dev` 容器时，在 Ubuntu 宿主机执行一次：

```bash
mkdir -p /home/legion/AUV-simulation
git clone https://github.com/SKT-T1otto/AUV-simulation.git /home/legion/AUV-simulation/source
bash /home/legion/AUV-simulation/source/tools/run_linux.sh --setup-only
```

已存在 source 仓库时先 `git -C /home/legion/AUV-simulation/source pull --ff-only`，不重复克隆。
配置脚本保留已安装环境及旧容器，将源码和输出目录挂载到新容器 `crk-ros2-home`，首次自动编译。
后续在宿主机启动：

```bash
bash /home/legion/AUV-simulation/source/tools/run_linux.sh
```

先用 Ctrl+C 停止之前的 `shapes.sdf` 与测试时钟桥接。
`run_demo.sh` 在一个终端中管理 Gazebo、消息桥接及控制器。
结果实时保存到宿主机 `/home/legion/AUV-simulation/results/run_*`，图表与报告使用同一根目录。
无需运行结束后复制；从 ToDesk 可直接访问。配置与路径说明见 `crk_auv_singleboat_v1/HOST_OUTPUT.md`。

默认使用 5 Hz 零残差策略 + 原参考公式的低速航点先验，执行层为 50 Hz。
测试外部 Python 推理往返时，可用：

```bash
CRK_POLICY_MODE=external CRK_EXTERNAL_ZERO=1 \
    bash /home/legion/AUV-simulation/source/tools/run_linux.sh
```

诊断 actor 仍是零残差，不加载训练模型。真实 actor 支持外部 ROS 推理进程及可选的无状态 TorchScript，
具体步骤见 `POLICY_INTEGRATION.md`；任务/团队上下文需要另外核对，不能凭维数一致就直接加载。

默认航点为 `(0,0,-2)`、`(2,0,-2)`、`(2,2,-3)`、`(0,2,-1.5)`、`(0,0,-2)`。
世界坐标 `z` 向上，航点处于水下。每个航点要求距离不超过 0.2 m、速度不超过 0.1 m/s，持续 2 s；最终保持 10 s。
实际完成状态以本次 `summary.json` 的 `termination_reason`、`waypoints_accepted` 和 `engineering_acceptance_passed` 为准。

本项目独立于既有论文实验，不改变论文既有任务、奖励、观测、动作、通信或成功判据。

## 源码更新

直接通过 Git 克隆和更新，不需要压缩包。在仓库根目录运行 `python3 tools/update_source_manifest.py`
可更新 `SHA256SUMS`。该命令不生成压缩包；运行数据、机器路径清单及本地审计不提交到 Git。
已有 `package_singleboat_delivery.py` 仅作为旧交付工具保留，不在构建或启动流程调用。

## 离线检查

不依赖 ROS 的数值检查需要 NumPy、SciPy：

```bash
cd crk_auv_singleboat_v1
python3 -m unittest discover -s tests -v
```

## 验证范围

已完成离线控制、几何、浮力配平和脚本语法检查；Gazebo 闭环仍需用户在 Ubuntu 容器内实际验收。
