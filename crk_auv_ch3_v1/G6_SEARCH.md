# G6：ZERO 四艇 Gazebo 搜索闭环

G5 的 Linux 日志已显示全部来源/接口检查及四组回放一致性通过，输出目录为
`mission_check_20261117_041319_3u0t69lc`。本阶段接入真实 Gazebo 运动。
代码与本地接口测试已完成，真实 Linux G6 联调及搜索成功验收仍待运行。

## 数据流和实际执行

原 D2 规划 → 原 28D 观测/动作门控 → ZERO 归一化残差 → 原基础导航加速度
→ 世界系到艇体系转换 → 质量/附加质量、阻力和重浮力补偿 → 有界推力分配
→ 32 路推进器 → Gazebo 积分 → 四艇同时间状态 → 原地图/探测/目标/交接/奖励/终止。

每个任务步 0.2 s，含 10 个 0.02 s 控制周期，每周期服务推进 10 个 0.002 s
物理子步。完成同一批步进与四艇状态屏障后才进入下一批。不要手动按 Play。
任务计算期间世界暂停；暂停按钮显示状态不代表仿真没有推进。
启动的 0.04 s 零推力预热用于排除里程计初始化瞬态，随后设置任务时间基准。
预热后艇位距原场景初值须不超过 0.02 m，偏差保留在实际反馈中。
任务时钟基准在 D2 初始化、目标显示更新以及新鲜的 WorldStatistics 暂停确认之后
才固定。启动期间最多接纳一个物理子步（2 ms）的同步差，并记录 startup_clock_delta_ns；
更大的未授权推进会报错。进入任务后基准不再调整，每个任务步末确认暂停时间严格匹配。
暂停服务返回只代表收到请求，不能单独证明世界已经暂停。时钟错误记录具体期望/实际值。

Python 不积分艇位/速度、不对 AUV 调用设置位置服务。设置位置客户端仅更新
非碰撞的黄色目标真值显示球，位置来自 Python 原目标运动规则，不能当成策略输入。
目标显示为任务时间离散更新（5 Hz），水下艇体为 50 Hz 控制/500 Hz 物理。

地图与感知仍使用固定来源中的 Python 实现：四艇状态驱动 neighbor26 扫描、
共享占据图与信念更新，探测、一步可靠交接与五步捕获保持规则照旧。
当前地图数组在结果中记录，Gazebo 窗口显示艇、目标、障碍，不叠加地图体素。
没有加载 `.pt`、启用训练或更改 400 步任务时限。

## 先在 Ubuntu 宿主机运行短联调

先停止其他 G4/G6 场景，然后执行：

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_search.sh --mode smoke
```

复用已经安装的 G5 CPU 环境、ROS Jazzy、Gazebo Harmonic、GPU/X11 和宿主机挂载。
运行前校验源码、ROS 类型与 42 项基础检查、14 项来源/任务/接口检查。
脚本不会重新下载 PyTorch；若缺少 G5 环境，会提示先执行 G5 入口。

smoke 使用远目标场景，运行 16 个任务步（3.2 s 仿真时间），检查三艘搜索艇都至少
移动 0.02 m，并要求地图扫描、四艇状态、控制周期和正常停止完整。
它会启动窗口，结束后自动关闭。墙钟耗时包括启动、规划和消息等待，通常大于仿真时间。
`--headless` 可追加到任一运行命令，用于只运行服务器排查物理/通信。

短联调通过后验证近距离发现、交接和捕获：

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_search.sh --mode capture
```

capture 夹具使用与 G5 相同的近目标布局，检验任务链路；该夹具成功不能作为搜索
性能数据。它仍用真实 Gazebo 反馈运行，不能用回放的第六步成功替代其实际结果。

在 `summary.json` 中同时核对 `engineering_acceptance_passed=true`、
`g6_capture_acceptance_passed=true` 和 `capture_evidence.verified=true`。
`capture_trace.jsonl` 每个任务步写一行，记录发现者、执行艇收到信息的状态、
执行艇两端的 Gazebo 实测位置、目标两端的位置和原生连续保持计数。坐标使用原生米制坐标系，
目标位置只用于事后验收。发现后的下一步应收到交接；成功结束时应有连续五个任务步，
每步起止距离均在配置半径内，且与原生保持计数一致。夹具中执行艇初始位置已在近目标半径内，
因此该结果仅检验近距离交接与保持。这里的捕获指仿真任务的距离与时间判据。
capture 模式的进程退出状态反映该模式的验收结果。未通过时保留部分日志，
检查 `capture_evidence.failure_tags`、`termination_reason` 和 `controller.log`。

两项通过后运行完整 ZERO 搜索：

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_search.sh --mode search
```

search 使用远目标和障碍布局，最多运行原有 400 步，即 80 s 仿真时间，提前成功或
任务终止则结束。可以显式选择原格式场景清单，文件应位于容器可见的宿主机挂载目录：

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_search.sh --mode search \
  --manifest /home/legion/AUV-simulation/scenes.json --scenario-id your_scene_id
```

场景必须包含原有字段与 `planner_seed`；默认夹具的 planner_seed=2729。
`--generate-only` 只生成场景资源和未开始摘要，不启动仿真，不构成任何运动通过证据。

## 物理与任务边界

沿用已通过 G4 的同一文献参数硬件模型、原生 DART 附加质量、15 N 单推进器上限、
0.12 s 响应及推力变化率。G6 将 G5 合成后的加速度直接转为力，不额外叠加航点先验。
低层使用当前工程模型的 0.35 m/s 速度调节和每轴 0.45 m/s² 加速度限制；触发调节时
记录请求值和调节值。原 Python 的角色速度/加速度约束、探测距离仍保留在任务层，
不把角色参数伪装成已标定的不同硬件。

该物理模型的可达速度低于原简化环境中的角色上限，且推进器会饱和或滞后，因此
完整搜索可能超时。必须如实记录；不能延长任务时限、降低捕获要求或瞬移来制造成功。
源简化运动中的 flow/drag 积分不再执行；Gazebo 负责当前零海流水动力。
实际加速度误差使用滤波里程计世界速度有限差分估计，不是独立 IMU 真值。

原任务的已完成艇冻结、碰撞首触点投影与边界规则在 G5 接口中保持不变。
G6 另存原始 Gazebo 艇位、任务艇位和差值。冻结艇由低层控制保持在任务点，
真实保持误差大于 0.2 m 则工程失败；真实中心越出原任务区域则停止。
姿态大于 20° 或艇体速度大于 2 m/s 也按工程异常停止。
这些工程守卫不能替代或改变任务的奖励与成功规则；保持位移偏差仍是模型近似限制。

每艇船壳接触传感器通过 ROS 桥记录 Gazebo 实体接触，观测到事件即暂停停止并标记
physical_contact。原任务的点线段碰撞单独记录在 task_outcome 中，两者不是同一判据。
接触传感器只在发生接触时发布，消息异步；订阅发现与零事件不证明接触检测完整无漏报。
摘要始终保留 contact_absence_proven=false，不将零事件冒充严格无碰撞证明。

## 结果与验收

结果直接保存到宿主机：

```text
/home/legion/AUV-simulation/results/ch3/search_模式_时间_后缀/
  summary.json
  scenario.json / truth_scene.json / vehicle_profile.json
  resolved_config.json / role_specs.json
  world.sdf / bridge.yaml / sdf_validation.log
  control.jsonl          每个控制周期的真实状态、指令、加速度估计与控制诊断
  transitions.jsonl      每个任务步的观测、ZERO 动作、奖励、任务及原始/投影位置
  events.jsonl           原探测/扫描公共事件
  maps.jsonl             首步、每十步及结束时的地图数组快照
  physical_contacts.jsonl（观测到实体接触时）
  gazebo.log / bridge.log / world_control_bridge.log
  target_marker_bridge.log / controller.log / ros_logs/
  error_traceback.txt    （异常时）
```

```bash
CRK_G6_RUN=$(ls -dt /home/legion/AUV-simulation/results/ch3/search_* | head -n 1)
cat "$CRK_G6_RUN/summary.json"
```

- smoke：engineering_acceptance_passed=true，仅表示短程接通。
- capture：engineering_acceptance_passed=true 且 g6_capture_acceptance_passed=true。
- 完整 search：g6_search_acceptance_passed=true 才表示本次搜索任务通过，要求原任务
  success、发现与交接均有记录、五步捕获满足，并在发现前真实搜索位移至少 0.02 m。
- 超时或原任务碰撞可能仍具有 engineering_acceptance_passed=true，表示受控运行
  完成；task_success 和 g6_search_acceptance_passed 仍为 false，不能解释为任务成功。
- infrastructure failure、physical_contact、engineering_failure、interrupted 都是未通过。

关闭前发送零推力、等待 ROS 桥确认并暂停世界，随后清理子进程。最终速度仍会记录；
此处没有追加物理减速/漂移观察过程，不能宣称艇体已经在流体中完全静止。
ROS_DOMAIN_ID=76，每次独立 Gazebo 分区；与 G4 共用运行互斥锁，避免同时启动。
所有输出保留，异常会记录已退出进程的名字与日志。

本地验证使用数学检查、ROS/运动替身以及固定来源的真实任务封装；它们不代替 Linux
Gazebo 接触、服务、控制与任务运行验收。后续 `.pt` 推理仍留在 G7。

实现参考：
- [Gazebo Contact 官方示例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/contact_sensor.sdf)
- [Jazzy Contacts 消息](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_interfaces/msg/Contacts.msg)
- [Jazzy SetEntityPose 服务](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_interfaces/srv/SetEntityPose.srv)

独立的32步D2搜索重新分配诊断见 [D2_REASSIGNMENT_DIAGNOSTIC.md](D2_REASSIGNMENT_DIAGNOSTIC.md)。
它必须显式启用；先运行baseline32并审阅，再运行defer条件，不构成完整搜索或模型验收。

可恢复的逐艇进度提交适配器见 [PROGRESS_COMMIT_V1.md](PROGRESS_COMMIT_V1.md)。
先完成独立短程工程夹具的到达/释放核对，再运行原搜索场景32步诊断；默认G6不自动启用。
