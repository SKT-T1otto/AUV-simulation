# D2 重新分配对路径执行影响：工程诊断 v1

本入口只运行 D2＋ZERO。没有选择、加载或训练第三章模型；不构成 G6 完整搜索通过。
原 G4/G6 默认入口不变，native_g5、来源锁、400步任务配置、艇体和控制器不变。

## 唯一主动实验变量

- `baseline32`：原D2正常规划和提交，外部预算32步。
- `defer_search_reassignment32`：第1～19步照旧；**第20步物理运动结束之后**，
  在原航点稳定化完成、updates/commit之前，将搜索分配替换为进入边界前的有效搜索分配和路径。
  直到32步，不再提交新搜索分配；不冻结PathTracker索引。

第20步运动仍使用第19步末guidance。运动对照窗口是21～32步，最多2.4秒；
不得把20步的运动归于干预。原任务max_steps=400，没有延长、截短或改写任务终止规则。
正常预算结束标签为 `diagnostic_budget_completed`，任务本身仍未终止。
`diagnostic_valid`表示该次受控诊断完整有效；不是任务成功或物理性能通过。

提议、地图、信念、事件和执行艇规划继续计算。原生联合评分在门控前完成执行艇选择；
混合后的实际分配以 `rescore(finalize=False)`重新评分，固定这个原生执行艇选择，
不再次调用联合优化来迎合被保留的搜索分配。后续事件、冷却和执行艇决策可能因干预反馈而不同，
这是该提交干预的下游效应，不保证执行艇后半程轨迹相同。日志明确区分原始策略分值、
稳定化后的原始提议、门控提议以及真正安装结果；原始分值不是实际混合分配的分值。

检查最新公共几何中的剩余路径。发现/交接/任务终止、旧路径不安全或不可达、
安全桥需要改变旧路径、接触或工程守卫触发时停止并标为无效/失败。
不关闭安全功能，不向艇体设置位置，不为追求进展调整规划器、限速或控制器。

## Ubuntu宿主机：先只运行 baseline

关闭其他G4/G6运行后，在宿主机执行；不需要进入容器。

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_reassignment.sh baseline32
```

无GUI时可追加 `--headless`；成对运行必须使用相同选项和机器环境。
入口只检查/启动已有 `crk-ros2-home` 容器，校验源码和宿主机绑定目录，复用
`/opt/crk-g5-cpu-v1/bin/python`；不安装依赖、不构建新容器。
与G4/G6共用 `.container/g4_motion.lock`，暂停步进、四艇状态屏障、接触监测和子进程回收沿用原入口。
不要手动按Play，也不要让两个条件共用正在运行的Gazebo。

每次生成唯一目录，终端会打印实际路径：

```text
/home/legion/AUV-simulation/results/ch3/d2_baseline32_时间_随机后缀/
```

检查该目录 `summary.json`、`DIAGNOSTIC_REPORT.md`，先把baseline结果提供给审阅者。
预期 `termination_reason=diagnostic_budget_completed`、`completed_task_steps=32`、
`control_steps=320`、`diagnostic_valid=true`、`failure_tags=[]`、
`g6_search_acceptance_passed=false`、`task_success=false`。
零退出码只表示入口的工程诊断检查通过，不能替代检查原始物理记录。
异常非零退出并保留已有记录；原任务若提前发现目标，也会使本搜索阶段对照失效。

## baseline确认后才运行第二个条件

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_reassignment.sh defer_search_reassignment32
```

它会单独启动并清理一套Gazebo进程，产生 `d2_defer_search_reassignment32_...` 新目录。
不要恢复/续跑第一个实例，也不要扩大预算窗口。

## 两次完成后进行只读配对

将下面两行路径改成终端打印的**完整实际目录**；不要用另一次场景或不明来源的latest结果替代。

```bash
BASELINE_DIR='/home/legion/AUV-simulation/results/ch3/d2_baseline32_实际时间_实际后缀'
DEFER_DIR='/home/legion/AUV-simulation/results/ch3/d2_defer_search_reassignment32_实际时间_实际后缀'
bash /home/legion/AUV-simulation/source/tools/run_ch3_reassignment.sh --compare "$BASELINE_DIR" "$DEFER_DIR"
```

分析不启动任务、不加载模型，输出新的 `results/ch3/d2_pair_.../REPORT.md` 和 `comparison.json`。
输入JSON/JSONL在分析前后核对SHA-256，不改原运行文件。配对不充分会返回非零码和具体issue。

## 事先规定的可比性门槛

`reassignment_contract.py` 在运行前写入metadata，两次必须一致：

| 核对项 | 最大允许差异 |
| --- | --- |
| 初始化四艇位置/世界速度 | 0.002 m / 0.01 m/s |
| 运动第1～20步全部20ms端点位置/世界速度 | 0.01 m / 0.02 m/s |
| 初始化及上述端点姿态夹角 | 1° |
| 前20步请求加速度向量范数差 | 0.05 m/s² |
| 初始化及步后1～19 guidance分配ID、路径、跟踪点、保持及可达状态 | 精确相同，不取整 |
| 上述任务状态、扫描/地图计数 | 精确相同 |
| 每个控制端点相对任务时钟 | 严格对应20ms批次 |
| 场景、来源、适配源码清单、配置、角色、物理配置、固定旧目标/路径 | 一致 |

这些是保守工程对照容差（1cm远小于0.75m切换阈值），不是统计等价或动力学标定结论。
额外核对运动20用于保护“该动作尚未受到干预”的时序。
不要求两次真实Gazebo前缀逐位相同，不重置位置来制造相同。
任一条件无效、前缀超差/路线分叉或未真正暂缓任何搜索提交，都不给出因果归因。
分配哈希单列差异：沿用原 `OnlineAllocation.allocation_sha256`，不改精度、不取整。
此前Windows/Linux信念浮点评分造成的哈希差异没有被本诊断“修复”。

## 记录和指标

- `initial.jsonl`：初始化观测、任务状态、实际四艇位姿/世界速度转换所需数据、guidance和tracker。
- `step_start.jsonl`：本次运动之前真正安装的guidance、allocation、tracker、实际状态及地图计数。
- `planning.jsonl`：检测事件/诊断、原始actionable/primary及策略决策、原始allocator提议、
  稳定化原始提议、门控后提议、实际安装、规范哈希、刷新原因、图哈希、规划/地图版本。
- `transitions.jsonl`：动作/观测/奖励原记录、步后分配和guidance、tracker签名/索引/剩余路径、
  实际四艇位姿、世界速度换算数据、地图/扫描计数、安全状态与原任务碰撞。
- `control.jsonl`：50Hz四艇位姿与世界速度、请求/调节后加速度、响应估计、推力与分配误差。
  仅在本诊断记录回调补充body_states，没有修改低层控制实现或推力计算。
- `retained_reference.json`：运动20结束、规划提交前保留的旧跟踪点、剩余路径和状态。
- `events.jsonl`（有事件才创建）、`maps.jsonl`、`physical_contacts.jsonl`（有接触才创建）、
  `final_state.jsonl`和summary：扫描、地图快照、停止和失败信息。
- 自动生成 `diagnostic_metrics.json`、`DIAGNOSTIC_REPORT.md`、`trajectory_xy.svg`；
  失败/中断也生成部分报告，缺失指标不补造。

配对重点比较21～32步到**固定旧跟踪点**的距离减少、沿同一旧剩余折线的投影进展、
累计路程/净位移、扫描/地图计数变化、实际应用有限路径段误差、请求及调节后加速度响应误差。
当前跟踪点距离序列另存，不能作为两组不同目标的直接距离对比。
投影取最近有限线段，重叠路线可能有歧义；扫描/未知格变化不称论文搜索覆盖率。
减速定义为20ms速度差分低于−0.02 m/s²；转向为与20步末入口速度方向夹角首次达到90°；
反向为该方向投影速度低于−0.05 m/s。入口速度不高于0.05 m/s则方向未定义。
“未观测到”不等于不会发生；2.4秒窗口可能连制动都未完成，应如实报未分辨。

## 本地验证边界

`diagnostics/verify_reassignment.py` 是显式调用的有预算验证程序，不属于常规unittest自动发现。
它在每个原生任务实例构建前写预算、每次advance前记录尝试步数；不分段续跑。
先mechanism阶段验证原流程/门控/索引/无效路径，再entry阶段用真实入口和反馈替身验证记录及报告。
运行示例（只限另一次明确批准的本地验证预算，不要在Linux实验前自动重复）：

```text
python diagnostics/verify_reassignment.py --phase mechanism --audit-dir AUDIT --input-run ORIGINAL
python diagnostics/verify_reassignment.py --phase entry --audit-dir AUDIT --input-run ORIGINAL
```

AUDIT必须预先具有budget_plan.json（总上限8、每实例32）。Windows当前已消耗全部8个实例，
包括一次OpenMP冲突中止，未恢复；后续不再新增实例。具体证据保存在Windows verification审计，
不随源码上传。本地替身验证不是真实Linux/Gazebo证据。
