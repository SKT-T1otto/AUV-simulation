# G4 四艇运动与控制验收

G3 已有用户 Linux SDF 校验、Gazebo 8.15 场景加载日志及视觉确认。
本阶段让四艘动态水下机器人通过独立推进器实际运动，尚不运行地图、搜索任务或学习策略。
当前 G4 已有本地数值与接口检查，**实际 Gazebo 动力学、桥接和控制验收仍待 Linux 运行**。
G5 及后续阶段尚未开始。
G4 不导入 Active 算法目录。进入 G5 前需要再次核对锁定来源，不能自动混入来源冻结后的新模块。

## 运行

先用 Ctrl+C 停止旧的静态预览。在 Ubuntu 宿主机运行：

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_motion.sh
```

入口复用现有容器及 GPU/X11 与目录挂载；每个新 shell 显式加载 Jazzy。
先校验源文件、ROS 服务类型、运行本地测试，随后生成世界并执行 `gz sdf -k`。
全部通过后自动启动 Gazebo、状态/推进器桥、世界步进服务桥和控制进程。
**不要手动按 Play。**窗口可能显示暂停按钮状态，但脚本正在通过服务逐步推进物理。
默认打开 GUI，并将每个控制周期的墙钟耗时下限设为 20 ms，方便观察；计算较慢时总耗时会更长。
完成后自动结束场景，保留所有日志和结果。中途 Ctrl+C 会中止本次测试并清理子进程。

无窗口验收可用：

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_motion.sh --headless
```

`--generate-only` 只生成资源，不启动物理；对应摘要标记 g4_assets_only，不作为运动通过证据。

## 应当看到的运动

| 仿真时间，扣除 0.04 s 启动采样 | 阶段 |
| --- | --- |
| 0–1 s | 四艇定点保持 |
| 1–7 s | auv_0 沿世界 x 正方向移动 1 m，其余保持 |
| 7–13 s | auv_1 沿世界 y 正方向移动 1 m，其余保持 |
| 13–19 s | auv_2 向上移动 0.75 m，其余保持 |
| 19–25 s | auv_3 斜向移动，其余保持 |
| 25–33 s | 四艇同时向各自新目标运动 |
| 33–37 s | 四艇最终定点保持与速度检查 |

前三艘测试艇为蓝、绿、紫色，执行器测试艇为橙色。此次四艇采用同一物理配置，
颜色/名称代表未来任务角色，不宣称已验证物理异构或第三章搜索效果。
目标标记/障碍仍仅为场景参照，控制程序不追踪原任务目标。

## 物理模型依据与近似

采用 BlueROV2 Heavy 的文献参数作为可更换的运动控制基准。它的实际原型是 ROV，
不是已经标定的无缆 AUV；本项目中的 auv_i 命名用于后续任务接口兼容。
八推进器来自参考平台构型，与策略动作方向数无关。

质量、排水体积、惯量、重浮心和阻尼/附加质量取自
[von Benzon et al., JMSE 2022, Table A1](https://doi.org/10.3390/jmse10121898)。
推进器安装按该表的旋转公式转录，FRD 到 FLU 使用 diag(1,-1,-1)；
原表的近似角度及第三个水平方向 3π/4 也保留，不能称为厂家 CAD 精确复现。
原表数据和工程假设逐项记录在 config/vehicle_g4.json。

已移除旧示例的安装中心重合问题，控制矩阵由安装位置/方向计算；
基体质量、质心及惯量扣除八个数值转子部件的贡献，以保持整艇参数一致。
排水体积由单一碰撞盒表示，浮心位于重心上方 0.01 m。外观和碰撞几何是简化近似。
阻尼使用负 SNAME 导数；附加质量使用正的物理值，在 SDFormat 1.11 的
`base_link/inertial/fluid_added_mass` 中配置，并指定 DART 物理引擎。
DART 将其与刚体惯性联合求解；Hydrodynamics 只计算阻尼，关闭旧式附加质量和科氏项，
不再写入 `xDotU` 等旧字段，避免重复计算。当前零海流下的附加质量科氏项由 DART 处理。
重力取参考参数值 9.82 m/s²；不把附加质量算作额外艇重或排水体积。

2026-11-16 的旧 v1 接入曾在启动后触发 ODE 碰撞断言。该版本采用插件根据上一子步
加速度施力的显式附加质量方式，垂向附加质量 18.68 kg 大于刚体质量 13.5 kg，
存在数值发散风险。v2 改为官方推荐的原生惯性方式，保留全部文献物理数值和控制判据。
本地回归复现了孤立显式递推的发散并检查新 SDF，但修复后的真实 Gazebo 验收仍须 Linux 重跑。

15 N 对称降额推力边界、0.12 s 执行器一阶响应、变化率和控制增益是工程测试设定。
使用 Gazebo 牛顿推力命令，不宣称有真实 PWM/转速/功耗映射；推力系数仅作名义转速换算。
没有缆绳、海流、测量噪声、推进器间干扰或完整桨反扭矩模型，也没有复现论文整套实验。
全部参数都保存在每次结果中；通过 G4 只表示该参考模型的控制链路可运行。

## 控制、同步与停止

- 四个独立控制器；每艇 8 路推力，各自 `/auv_i/...` 命名空间。
- 位置/速度反馈、姿态稳定、重浮力差与标称水阻补偿、有界推力分配、执行器延迟。
- Gazebo 是唯一运动积分器。Python 不覆盖艇位，不直接设置速度，不调用学习环境推进运动。
- 控制周期 0.02 s，物理子步 0.002 s，每次服务推进 10 子步，暂停等待下一指令。
- 四艇里程计必须具有相同时间戳，且距离请求时刻不超过一个物理子步；否则超时失败。
- 启动时先推进 0.04 s 零推力，用于排除里程计有限差分初始化瞬态。
- DDS 确认只证明 ROS 桥收到消息，不构成 32 路执行器在 Gazebo 中原子应用的证明。
  此阶段不宣称比特级确定性；应用时间、状态时间和推力均记录，后续严格事务仍需单独验证。
- 关闭前发送零推力并请求暂停，监督进程按控制器、桥、Gazebo 顺序终止并等待子进程。
- 固定 ROS_DOMAIN_ID=75，Gazebo 分区每次唯一，宿主机输出根目录内有运行互斥锁。

## 结果与判据

所有结果直接保存到：

```text
/home/legion/AUV-simulation/results/ch3/motion_日期_时间_后缀/
  summary.json
  vehicle_profile.json
  scenario.json
  truth_scene.json
  world.sdf
  bridge.yaml
  trajectories.csv
  commands.jsonl
  events.jsonl
  sdf_validation.log
  gazebo.log
  bridge.log
  world_control_bridge.log
  controller.log
  ros_logs/
```

查看最新摘要：

```bash
CRK_LAST_RUN=$(ls -dt /home/legion/AUV-simulation/results/ch3/motion_* | head -n 1)
cat "$CRK_LAST_RUN/summary.json"
```

要求 linux_sdf_valid=true、termination_reason=completed、motion_started=true、
engineering_acceptance_passed=true，且 phases 中七阶段均 passed=true。
工程阈值：阶段末位置误差≤0.2 m；单艇测试时其他艇相对阶段起点漂移≤0.15 m；
运动方向进度至少达到指令位移的一半；最大横滚/俯仰<10°；最终速度<0.1 m/s。
这些是 G4 工程测试判据，不替换论文任务成功规则。

异常必须保留 error、日志和失败摘要；不会以零进程退出码替代成功判据。
基础进程提前退出时，摘要增加 `failed_processes`，记录清理前已退出的日志文件与退出码；
终端直接显示对应日志末尾。停止其他进程产生的 SIGINT 不作为根因。
修复版结果还应包含 `model_profile=bluerov2_heavy_literature_port_v2`、
`added_mass_mode=dart_native_implicit`，便于区分旧失败记录。
mission_executed、map_executed、checkpoint_loaded、performance_claim 始终为 false。
与 G3 不同，这次 motion_started 应在实际收到推进后的反馈时变为 true。

## 实现依据

- [Gazebo ROS service bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge#example-4-service-bridge)
- [Jazzy WorldControl](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_interfaces/msg/WorldControl.msg)
- [Gazebo Thruster](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Thruster.html)
- [Gazebo OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)
- [Harmonic 原生附加质量与阻尼示例](https://gazebosim.org/api/sim/8/adding_system_plugins.html)
- [Gazebo 附加质量迁移说明](https://github.com/gazebosim/gz-sim/blob/main/Migration.md)
- [DART 对原生附加质量与刚体重力的处理](https://github.com/gazebosim/gz-physics/blob/gz-physics7/dartsim/src/SimulationFeatures.cc)

本地验证包含独立控制/分配、质量惯量配平、SDF/话题一致性、状态屏障、实际客户端方法的
ROS 替身测试、监督进程错误处理，以及有限简化六自由度回归。
它们不代替真实 Gazebo，G4 仍等待用户 Linux 输出与实际运动观察。
