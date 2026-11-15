# Python 策略到 Gazebo 的接口

本次增加完整的观测—残差动作—期望加速度—推进器链，暂未选定、加载用户历史 checkpoint。
默认零残差仅验证航点先验和执行层。支持外部 Python 推理进程，也支持普通无状态 actor 的 TorchScript 导出。

## 分层与坐标

1. Gazebo 真值里程计：位姿在 map，twist 在 auv_0/base_link。
2. ObservationAdapter：艇体系速度转世界系，再转到策略坐标；生成 28 维观测。
3. PolicyPipeline：每 0.2 s 仿真时间采样、推理/请求、缩放残差并组合先验，期间保持期望加速度。
4. ControlLaw.execute_acceleration：每 0.02 s 将世界系期望加速度转成艇体力，稳定姿态，再做八推进器分配、限幅、延迟。
5. Gazebo 独自负责实际动力学积分。代码不再从动作直接更新位置和速度。

下层执行器没有第二套航点先验，不会将同一先验重复叠加。它也不会再次缩放残差或按示例速度阈值截断动作。
执行层沿用单艇示例的零海流标称阻力前馈，将期望加速度映射为艇体力。它与原 Python 环境的被动阻力、
速度硬裁剪和位置积分不构成逐步等价；这里保留的是策略接口与动作组合公式，需要独立评估后端差异。

策略变换：p_policy = R*p_world + t，v_policy = R*R_body_to_world*v_body。
期望加速度反变换：a_world = R.T*a_policy。平移不作用于速度、加速度。
R 必须为保持 z 向上的正交旋转，禁止用单轴反射假装三维坐标旋转。
默认 R=I、t=(5,5,5)，仅用于让本示例水下坐标映射到正坐标范围；并非任何训练场景的已验证映射。

## 28 维观测（零起始下标）

| 下标 | 内容 |
|---|---|
| 0:3 | 策略系位置，m |
| 3:6 | 策略系速度，m/s |
| 6:9 | 航点相对位置，m |
| 9:12 | 航点方向，距离分母下限 1e-6 |
| 12:15 | 自己已知的任务目标相对位置；未知时为零 |
| 15 | clip(导航距离/10,0,1) |
| 16 | clip(速度/(v_xy_max+1e-6),0,1) |
| 17 | tanh(沿航点方向速度/(v_xy_max+1e-6)) |
| 18 | clip(最近障碍距离/10,0,1) |
| 19 | 航点进度 |
| 20 | 已完成标志 |
| 21 | 驻留进度 |
| 22:26 | 四角色 one-hot |
| 26:28 | 未知/已知目标阶段 one-hot |

布局和动作公式对应参考代码 core/env/uav_env.py 的 _get_obs、_compute_waypoint_prior_acc、_apply_agent_dynamics。
本示例没有搜索探测、目标交接、障碍感知或团队通信。TaskContext 显式提供这些字段；默认无目标知识、
最近障碍距离 10 m（归一化后为 1）、角色 1。最终驻留仍在控制，故不会把演示航点完成误写为原任务终止。
该上下文不代表完整论文任务。绝不使用 Gazebo 未知目标真值填充已知目标字段。

## 动作契约

actor 输出是 (3,) 归一化残差，先 clip 到 [-1,1]。

    residual = residual_scale * [a_xy_max,a_xy_max,a_z_max] * action
    final = clip(prior_strength * prior + residual, -a_max, a_max)

use_residual_prior=false 时 prior 为零。enabled=false 时整个期望加速度为零；它不是旧环境中的位置冻结机制。
prior 的水平期望速度按水平距离范数计算，垂向单独计算；公式、角色缩放和各轴限幅在 policy.py 中实现。

- policy_demo.json：低速工程基线，v_xy=0.5、v_z=0.3、a_xy=0.6、a_z=0.4。
- policy_reference_search_balanced.json：参考环境 search_balanced 的构造默认值，v_xy=2.2、v_z=1.0、a_xy=1.0、a_z=0.7。
  这份文件不是从用户选定 checkpoint 自动导出的配置，也不保证当前 18 kg/15 N 示例平台能达到这些速度。

checkpoint_compatible=false 明确记录当前未完成训练配置核对。
接入时必须核对模型来源、角色、归一化、先验参数、决策周期、坐标映射和任务上下文。
绝不能为了让旧策略运行而静默修改这些含义；若调整，要使用新的、单独标识的实验配置。

## 零残差与外部接口测试

在已经编译的工作空间目录（容器内）运行：

```bash
bash run_demo.sh
```

测试真正跨 ROS 消息的外部 Python 零残差往返，脚本自动启动诊断 actor：

```bash
CRK_POLICY_MODE=external CRK_EXTERNAL_ZERO=1 bash run_demo.sh
```

它没有神经网络，不应称为训练策略实验。

## 真实外部 Python 推理进程

不要设置 CRK_EXTERNAL_ZERO。启动场景：

```bash
CRK_POLICY_MODE=external bash run_demo.sh
```

另开宿主机终端进入同一容器，加载 /opt/ros/jazzy/setup.bash 和本工作空间 install/setup.bash，
设置 ROS_DOMAIN_ID=73、ROS_LOCALHOST_ONLY=1、use_sim_time=true，再启动自己的推理节点。
推理节点若在其他 Python/Conda 环境，需先确保该环境能正确使用 ROS 2 rclpy；默认示例不依赖 PyTorch。

话题均为 std_msgs/msg/String，data 为 JSON。观测 /auv_0/policy/observation：

```json
{
  "schema": "crk.observation28.v1",
  "seq": 1,
  "sim_s": 5.02,
  "frame": "policy",
  "profile_id": "singleboat_low_speed_v2",
  "policy_contract_sha256": "本次配置的规范化JSON指纹",
  "role_index": 1,
  "observation": ["这里为28个有限浮点数"],
  "enabled": true,
  "singleboat_test_context": true
}
```

动作 /auv_0/policy/action 必须回复同一 seq、sim_s 和配置指纹：

```json
{
  "schema": "crk.residual3.v1",
  "seq": 1,
  "observation_sim_s": 5.02,
  "policy_contract_sha256": "与请求一致",
  "action": [0.1, -0.2, 0.0]
}
```

使用训练工程原生加载入口恢复模型并关闭探索噪声，再将 actor 原始三维输出填入 action。
不能在外部先乘加速度系数或叠加先验，否则会重复映射。
重复/旧序号回复会被拒绝；错误结构、配置指纹、时间戳、非有限动作或超过 0.15 s 仿真时间的回复会终止测试。
首次等待观测订阅者最多 45 s 壁钟。初始尚无动作时保持零期望加速度；后续有效动作之间为零阶保持。
不要只在终端用 ros2 topic pub 发送没有对应观测的任意动作。

## TorchScript 本地 actor

只支持 [1,28] float32 输入、[1,3] float32 输出的 CPU 无状态 actor。
需要原生加载历史 checkpoint 后导出纯推理模型；当前仓库没有自动读取任意 checkpoint 的逻辑。
需要四艇联合边界、通信上下文或递归状态的网络不能直接截成单艇 actor，应使用外部接口并提供真实上下文，
或者等四艇任务实现后接入；禁止填充虚构队友观测来绕过依赖。

在自己的训练环境中安装/使用匹配版本 PyTorch，导出 actor.pt。与它一起提供 actor.json：

```json
{
  "schema": "crk.actor_export.v1",
  "observation_dim": 28,
  "action_dim": 3,
  "output_semantics": "normalized_acceleration_residual",
  "requires_team_context": false,
  "recurrent": false,
  "model_sha256": "actor.pt实际字节的SHA256",
  "policy_config_sha256": "所选policy配置文件实际字节的SHA256"
}
```

元数据用于防止模型/配置混用，不代替对训练语义的人工核对，也不是性能或安全认证。
模型放在仓库外的目录，容器必须可读；默认不下载或安装 PyTorch。

```bash
CRK_POLICY_MODE=torchscript \
CRK_POLICY_FILE=/path/to/verified_policy.json \
CRK_ACTOR_PATH=/path/to/actor.pt \
CRK_ACTOR_METADATA_PATH=/path/to/actor.json \
bash run_demo.sh
```

可用 CRK_POLICY_FILE 指定新的显式配置；不用覆盖原参考配置。
本地同步推理超过 0.15 s 壁钟预算会终止；实际机器还需检查 5 Hz 推理与状态时效是否满足要求。

## 记录和验收

- trajectory.csv 新增 policy_seq 和世界系期望加速度。
- policy.jsonl 保存每次观测与接受动作、先验、残差和最终加速度。
- /auv_0/reference/acceleration 发布 map 系、m/s^2 的最终执行参考及 seq，仅作诊断。
- summary.json 记录模式、决策数、动作接受/拒绝数、完整配置及 actor 元数据。
- 每次运行仍输出 Gazebo/桥接日志，自动诊断 actor 另有 actor.log。

先验证零残差，再验证外部零残差，最后接实际导出的模型。三者结果分别保存，不能混为同一方法。
Gazebo 端到端验收仍需用户 Linux 实机运行；离线检查不是 Gazebo 实测。
