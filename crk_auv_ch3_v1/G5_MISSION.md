# G5：当前 Active 来源固定与任务移植

G4 已依据用户提供的 `motion_20261116_142040_p6qhzsi0` 真实 Linux 记录验收。
G5 复用当前 Active 的地图、探测、目标运动、固定可靠交接、奖励、观测和终止逻辑，
提供外部运动状态接口。最终 `.pt` 权重不参与本阶段。

## 当前来源

`native_g5/` 保存用户授权时的 253 个运行代码/配置文件，逐字节复制。
`config/source_lock_g5.json` 记录 SHA-256、文件清单与相对旧 G1 锁的变化；
旧 `source_lock.json` 保留。新增的五个 `r123_governed` 文件被记录为当前源码，
但此次运行只选择原始 D2/ZERO，不启用这些研究分支。

来源总摘要：`1da086f7a7418dcca6914168d71843c4cf93d86c0f8ae95c1a4d6260137299d1`。
运行前校验全部源码，已导入其他仓库同名模块时拒绝继续。Git 保留快照原始换行。
用户后续更新训练代码时，需新建来源版本并重新验证，不能直接覆盖此快照或旧结果。

`configs/b2.json` 用于读取原有任务、奖励、D2 规划和动作门控配置；此处未构造 B2
网络。ZERO 输出四个三维零归一化残差，基础导航控制仍照常计算。
当前启用的是 original D2，不自动切换 NEW 地图、R123 或其他实验配置。

## 接入边界

`ExternalMissionRuntime` 复用原任务封装与完整 step 顺序，只替换艇体积分块：

1. 原观测、门控、航点先验、残差缩放与加速度裁剪生成四艇加速度请求。
2. `MotionRequest` 含步号、唯一标识、0.2 s 区间、艇位、世界系速度、加速度和停止标志。
3. 外部运动提供器返回四艇共同时间的 `MotionFeedback`；错误坐标系、旧数据、
   未来数据或步号不匹配会失败，失败后必须重建任务，不能重试半完成的 step。
4. Python 用返回状态执行原任务碰撞、地图扫描、目标运动、发现、交接、捕获、
   奖励和下一步观测/规划。Python 中不再运行原艇体速度/位置积分。

`feedback_from_gazebo` 已提供位置坐标变换与 FLU 船体系速度旋转，时间相对仿真
启动基准计算。G6 仍需连接实际 ROS 步进和推进器控制回调。

原任务会在碰撞时投影到接触点、在边界处裁剪，并冻结已完成艇。G5 保留这些任务
规则用于等价性验证，同时分别记录原始运动反馈、任务位置和投影距离；不得把任务
投影当成 Gazebo 真实位置，也不得发位置指令瞬移。G6 必须明确处理这些差异，并
把 Gazebo 实体接触与原任务点线段碰撞分别记录。

原 Python 的角色加速度/速度上限、传感距离仍是任务接口参数。G4 四艇使用同一
文献物理模型，不能因此宣称真实硬件异构已验证。真实水动力与原简化积分器的差异
由 G6 测量，不能以本阶段回放一致就宣称运动等价。

## Linux 运行

在 Ubuntu 宿主机执行：

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_mission_check.sh
```

复用现有 `crk-ros2-home` 容器及挂载。首次创建独立 CPU Python 环境并安装 PyTorch
2.6.0 CPU 与 NumPy 1.26.4；下载失败时保留环境，修复网络后重跑相同命令即可。
此环境用于任务验证，不改宿主机 AUV conda 环境；未来 `.pt` 接入需另外核对训练版本。

这次不会打开 Gazebo 窗口；执行的是运动记录回放的一致性验收。先做 6 项接口/来源
测试，再执行最多 8 个原始/回放实例、每实例最多 32 步的验证，无训练或模型加载。
工程夹具用于覆盖分支，不能作为论文成功率数据。

检查包含非零动作合成、碰撞优先终止、超时、地图更新、目标反射、发现、一步交接、
五步捕获，以及静止运动反馈不被二次积分、失败事务不能重复使用。完整任务逐步比较
28D 观测、动作门控、地图数组、事件、奖励、目标状态、规划指令和随机数状态。
碰撞夹具显式注入接触前状态，超时夹具将两侧测试上限设为 2 步；正式配置仍为 400 步。

输出直接在宿主机：

```text
/home/legion/AUV-simulation/results/ch3/mission_check_时间_后缀/
  summary.json
  g5_mapping_reflection/{scenario.json,recorded_motion.jsonl,comparisons.jsonl,case_summary.json}
  g5_discovery_handoff_capture/{scenario.json,recorded_motion.jsonl,comparisons.jsonl,case_summary.json}
  error_traceback.txt  （失败时）
```

```bash
CRK_G5_RUN=$(ls -dt /home/legion/AUV-simulation/results/ch3/mission_check_* | head -n 1)
cat "$CRK_G5_RUN/summary.json"
```

通过需要 `termination_reason=completed`、`semantic_parity_passed=true`、四个 case
都 `parity_passed=true`。`gazebo_motion_executed=false` 是本阶段正常标记。
Windows 本地检查已通过；Linux 环境验证待该命令的实际输出。通过后进入 G6 ZERO
四艇搜索闭环，再进入 G7 `.pt` 推理。
