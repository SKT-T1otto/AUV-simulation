# progress_commit_v1：可恢复的进度感知搜索提交适配器

独立工程诊断，显式启用；原G4/G6默认入口、D2固定源码和旧baseline/defer模式不变。
不加载.pt、不训练、不选第三章最终模型，不改变物理/控制/地图/感知/奖励/观测动作接口/目标或成功条件。

## 实现契约

搜索场景中第1～19步照旧。运动20完成后，在原航点稳定化与提交边界拦截原D2已接受的提议。
逐搜索艇保存最新待提交意图、原决策步、规范提议哈希、旧有效路径签名和旧跟踪点。
旧路径有效且距该跟踪点≥原阈值0.75m时，继续旧路径；不冻结索引或艇体。
距离严格小于原PathTracker阈值时，尝试释放。新的原生搜索提议可替代待处理意图；
原生执行艇单独提议不取消无关搜索意图。

释放可在没有新D2事件的步后提交边界发生。不得重新安装缓存路径：
搜索入口用原extract_planning_state生成独立只读公共路线视图，经公共几何过滤，再调用原TravelCostService，
从当前任务测量位置查询原提议终点。读取视图不替换provider.cached/runtime.state，不调用provider强制刷新；
原D2事件及信念刷新节奏照旧。缺少连接器、不可达或最新公共几何拒绝时，丢弃意图，等待原生新提议。
没有任意直线或真值障碍回退。实际混合评分仍使用原生决策状态，finalize=False固定当前原生执行艇选择。

路线和分值预检完成后再更新controller和WaypointManager；原生桥继续编译guidance/Tracker并刷新观测。
实际搜索切换时间写入WaypointManager，一次延迟安装不重复调用原策略mark_replan，不伪造新原生事件或求解。
native_controller_result保存原生控制器经过门控后的返回；未干预提议和策略决策另存于raw_stabilized_proposal、
raw_policy_decision。延迟安装的applied_controller_result显式标记
PROGRESS_DELAYED_COMMIT / adapter_search_commit，原生与实际分值、时间及计数不混写。
多艇可分别释放，因此实际搜索组合可能不等于原始联合最优提议，记录其混合评分，不另选执行艇来掩盖耦合。

旧路径失效/原生安全改路会取消待处理意图，原安全控制器和桥照常处置，随后当前对照标无效并停止。
发现、交接、终止时先取消队列并允许原任务控制器处理；搜索诊断随后停止为不适用。
接触、时钟、工程守卫、中断或预算结束保留已有日志并清空未提交意图，不恢复/续跑。
32为外部预算，正式任务max_steps仍400；diagnostic_budget_completed不代表任务成功或超时。

## 两层运行与预算

Windows本阶段原生任务实例预算0，旧阶段8实例预算不重置。
本地组件、原生路径/Tracker/WaypointManager类和真实ROS入口替身检查不代表Linux/Gazebo验证。
建议Linux本阶段最多4个独立运行，每次32个0.2s周期；先工程夹具，再原搜索，不分段续跑。

工程夹具复用原搜索世界/初始位置、艇体、G6低层加速度控制、互斥锁、暂停步进、四艇屏障和接触停止。
它没有原生任务实例、地图/感知/奖励/目标运动或D2求解；提议和路线是明确的工程输入。
使用既有envelope航点加速度参考（与原D2基础导航不同），请求每0.2s保持10个20ms控制周期。
三搜索艇初始旧点相对起点[0,1.5,0]，第5周期运动后注入新终点[1,2.5,0.5]；执行艇固定保持。
声明的安全走廊位于原障碍左侧，安全检查保留；合成路线为走廊内新起点到终点的直段。
该直段回调只用于工程夹具，原搜索入口仍使用原生公共图查询。
fixture_progress32必须记录先WAITING、后COMMITTED、三艇均释放且重建起点对应实际位置；
32周期内未观察到释放会返回失败，不追加运行时间或补造通过。

## Ubuntu宿主机先只运行第一项

停止其他G4/G6运行；不要按Play。复用已有crk-ros2-home和/opt/crk-g5-cpu-v1，不安装任何依赖。

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_progress.sh fixture_baseline32
```

无GUI可追加--headless，但一对条件使用相同选项。每次创建唯一目录：
/home/legion/AUV-simulation/results/ch3/progress_v1_条件_时间_随机后缀/。
预期summary：diagnostic_budget_completed、completed_fixture_cycles=32、control_steps=320、
native_task_instances=0、mission_executed=false、task_success=false、diagnostic_valid=true。
0退出码只代表该诊断的入口检查通过，仍要审阅实际状态、接触和时钟记录。

第一项结果确认后再单独运行：

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_progress.sh fixture_progress32
```

两项完整有效、前缀可比且实际释放通过后，再用同一新代码版本运行原搜索两项：

```bash
bash /home/legion/AUV-simulation/source/tools/run_ch3_progress.sh search_baseline32
bash /home/legion/AUV-simulation/source/tools/run_ch3_progress.sh search_progress32
```

两条分别运行和审阅，不并行、不共用Gazebo实例。search条件都仍为原场景seed1/planner_seed2729、D2＋ZERO。
当前场景32步可能尚未到旧点；pending_before_stop>0、release_observed=false可为有效有界观察，
不能据此宣称搜索中的恢复提交已真实验证。不可用旧c68caec基线替代新版本成对前缀检查。

## 成对只读分析

将两条路径替换为同一层次实际产生的新目录，不能混合工程夹具和搜索，也不能用未知latest替代。

```bash
BASELINE_DIR='/home/legion/AUV-simulation/results/ch3/实际baseline目录'
PROGRESS_DIR='/home/legion/AUV-simulation/results/ch3/实际progress目录'
bash /home/legion/AUV-simulation/source/tools/run_ch3_progress.sh --compare "$BASELINE_DIR" "$PROGRESS_DIR"
```

输出唯一progress_pair_...目录的comparison.json和REPORT.md；不启动任务、不覆盖原报告。
原绝对可比性门槛保留：初始化位置2mm/速度0.01m/s；控制前缀位置1cm、速度0.02m/s、
姿态1°、请求加速度0.05m/s²，离散guidance精确相同，不取整分配哈希。
夹具核对guidance0～4及运动1～5；搜索核对guidance0～19及运动1～20，并检查任务/地图计数和旧固定参考一致。
夹具执行艇保持模式比较其真实控制参考加速度；不把被blocked的无效搜索请求当作实际输入。
前缀、时钟、来源、配置不符或运行无效时不作因果判断。夹具未验证延迟释放也判对照不充分。

## 记录与验收

共同：initial、step_start、control、final_state、summary及PROGRESS_REPORT.md/progress_metrics.json。
夹具另有program.json、cycles.jsonl；搜索沿用planning、transitions、events、maps、retained_reference，
停止时另有progress_stop.jsonl。所有位置来自Gazebo反馈，不调用艇体设置位置服务，不在Python积分艇位。
COMMITTED逐艇记录原决策/实际提交步、提议哈希、旧跟踪点、路线查询图/地图版本及重建起点。
实际提交与原始提议分别记账；失败/中断也保存部分报告，不把缺失指标补为0通过。
搜索报告分别给固定旧目标/路径进展、当前应用路径误差、加速度误差及地图增量。
不同应用路径误差不直接当作相同参考胜负；扫描/未知格变化不称论文搜索覆盖率。

只有工程夹具真实到点/释放及原搜索链路实际证据分别通过后，才能讨论后续正式工程适配。
本版本不自动启用正式G6，不跑400步，不把永久冻结当最终方法。
