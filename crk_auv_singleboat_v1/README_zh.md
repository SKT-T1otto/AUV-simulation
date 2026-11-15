# 单艇三维航点跟踪与策略执行接口

本工程位于 Windows `E:\gym\code\WORKSPACE\CRK-AUV-Simulation`，直接通过 GitHub 同步到 Linux。
目标环境为 Ubuntu 24.04 容器、ROS 2 Jazzy、Gazebo Harmonic（Sim 8），需有正常的 GPU/OpenGL/X11 支持。

## 结构与模型

- `policy.py`：28D 观测、原参考动作缩放/先验公式、5 Hz 策略调度及超时检查。
- `actors.py`：可选无状态 TorchScript actor，校验导出契约和文件哈希。
- `core.py`：50 Hz 期望加速度执行、姿态稳定和八推进器分配；不再包含航点先验。
- `controller.py`：ROS 里程计反馈、观测/动作/加速度话题、航点和日志。
- `external_zero_actor.py`：外部 Python 往返诊断，不是训练策略。
- `config/vehicle.json`：18 kg、0.6 x 0.3 x 0.1 m 中性浮力艇体及八推进器布局。
- `config/policy_demo.json`：低速示例；`policy_reference_search_balanced.json` 是原环境构造默认值的参考。

六自由度动力学、浮力、水阻、附加质量和推进器在 Gazebo 中执行。水密度均匀，海底 z=-6 m，
不模拟自由液面、波浪、声呐成像、定位误差或通信。世界 z 向上，负 z 为水下。
物理参数是未标定工程起点，不是实艇高保真复现。

原论文目录、checkpoint、奖励、任务成功判据不由本项目改写。单艇默认上下文不替代四艇任务。

## Linux 启动

Ubuntu 宿主机首次获取代码：

```bash
git clone https://github.com/SKT-T1otto/AUV-simulation.git
sudo docker start crk-ros2-dev
sudo docker cp AUV-simulation crk-ros2-dev:/root/
sudo docker exec -it crk-ros2-dev bash
```

容器内：

```bash
cd /root/AUV-simulation/crk_auv_singleboat_v1
sha256sum -c SHA256SUMS
bash install_and_build.sh
bash run_demo.sh
```

先用 Ctrl+C 结束旧 shapes 场景及其桥接，不重复创建同名容器。
若代码以前位于 `/root/crk_auv_singleboat_v1`，以实际工作空间路径为准，更新时保留旧结果。
不需要生成或上传压缩包。

`run_demo.sh` 在一个终端管理场景、桥接和控制器。蓝色艇体 auv_0 位于 z=-2 m，金色球体为航点。
路径：(0,0,-2) -> (2,0,-2) -> (2,2,-3) -> (0,2,-1.5) -> (0,0,-2)。
每点需误差 <=0.2 m、速度 <=0.1 m/s 持续 2 s，最终保持 10 s。
低速控制已不同于旧 v1；以实际轨迹验收，超过 180 s 仿真时间会记录超时。

## 外部/神经网络策略

外部零残差消息往返：

```bash
CRK_POLICY_MODE=external CRK_EXTERNAL_ZERO=1 bash run_demo.sh
```

真实模型的外部推理、TorchScript 加载、观测字段、三维动作和时效协议见 [策略接口说明](POLICY_INTEGRATION.md)。
选定并核验 checkpoint 前，不宣称已经运行该训练策略。

## 结果

`results/run_*` 每次使用新目录，不覆盖旧结果。

- trajectory.csv：位置、误差、姿态、推力、策略序号、世界系期望加速度。
- policy.jsonl：观测与接受动作的映射诊断。
- summary.json：终止原因、航点数、驻留 RMSE、策略模式及配置。
- gazebo.log、bridge.log：启动/消息问题；自动外部零 actor 另有 actor.log。

在容器内工作空间查看最近结果：

```bash
CRK_LAST_RUN=$(ls -dt results/run_* | head -n 1)
cat "$CRK_LAST_RUN/summary.json"
```

工程验收要求 `termination_reason=completed`、`waypoints_accepted=5`、`engineering_acceptance_passed=true`。
工程判据不是论文原任务判据，日志保留本机并由 Git 忽略。

## 更新与诊断

- 容器内克隆：停止运行后，在仓库根目录 `git pull --ff-only`，重新编译后启动。
- 宿主机克隆后复制：先更新宿主机仓库，再同步源文件，保留容器 results 和本地实验修改。
- `SHA256SUMS` 用于干净克隆；修改源码后在仓库根运行 `python3 tools/update_source_manifest.py`，不会打包。
- 无时钟/里程计或外部订阅者：初始 45 s 壁钟超时后退出，检查启动日志。
- 外部动作过期/无效：终止测试并发零推力，不静默换成另一策略。
- 坐标错误：核对 map、auv_0/base_link、策略 R/t，不混用艇体系与世界系速度。
- 图形授权失败：更新宿主机 `.crk-ros2.xauth`，无需重装 ROS。
- Windows 开发环境未执行 Gazebo 联合运行；离线数值、TorchScript 测试不替代 Linux 实机验收。
