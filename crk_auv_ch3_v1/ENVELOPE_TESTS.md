# 当前艇体运动包线测试 v1

2026-11-18。独立于搜索任务的短程工程测试，实际 Gazebo 验收待 Linux 运行。
复用当前 BlueROV2 Heavy 文献参数移植、DART 原生附加质量、32 路推进器与 G6 加速度控制。
不加载模型、不实例化任务环境、不训练、不改变奖励/观测/动作/任务时限或历史场景。

## Ubuntu 宿主机运行

先停止其他 G4/G6 场景，不要手动按 Gazebo Play。已有容器/环境不重新安装。

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_envelope.sh x --headless
bash /home/legion/AUV-simulation/source/tools/run_ch3_envelope.sh y --headless
bash /home/legion/AUV-simulation/source/tools/run_ch3_envelope.sh z --headless
bash /home/legion/AUV-simulation/source/tools/run_ch3_envelope.sh track --headless
```

按顺序逐个执行、检查摘要；失败时停止后续项目并保留失败输出。去掉 `--headless` 可显示窗口，
每次完成后自动关闭正常。脚本只进入已有 `crk-ros2-home`，不执行安装、容器重建或训练。
每次运行包括现有本地检查，墙钟耗时大于模拟时间，监督器墙钟上限600 s。

## 固定工程夹具与激励

四艇相同物理模型，在无障碍水域以任务坐标 (6,6,4)、(14,6,4)、(6,14,4)、(14,14,4) 放置。
这是独立运动夹具，原正式搜索场景未修改；没有目标搜索或地图更新。
四艇并行施加相同参考；不能据此声称角色硬件异构或四个独立统计重复。
无艇体位姿/速度设置服务调用，Gazebo 是唯一艇体积分器。

| case | 阶段 | 模拟时长 |
| --- | --- | --- |
| x / y / z（每次） | 初始保持1 s → 正加速度6 s → 零加速度请求2 s → 主动制动4 s → 负加速度6 s → 主动制动4 s → 继续制动2 s | 25 s，1250控制步 |
| track | 初始保持1 s → 相对航点(1.2,0,0) 6 s → (1.2,1.2,0.6) 7 s → (0,1.2,-0.4) 7 s → 原点7 s → 最终保持3 s | 31 s，1550控制步 |

x/y轴请求±1.3 m/s²，z轴请求±0.75 m/s²；保持当前控制器每轴±0.45 m/s²裁剪。
速度调节仍为0.35 m/s、单推进器±15 N、电机一阶响应0.12 s、变化率50 N/s。
这些激励只用于物理测试，不改变强化学习3D归一化动作契约。

“零加速度请求”仍含阻力/重浮力补偿，不是零推力滑行；制动请求为 −1.2*v_world。
负向测试发生在正向制动后，记录负速度建立时间，尚非带明显正初速度的直接反推测试。
航点模式使用当前G4相同的0.9位置增益、0.35 m/s期望速度范数限制和1.2速度增益，
生成加速度后送入同一个G6 AccelerationControl；这是工程参考输入，不叠加D2规划。
航点按固定时刻连续切换，误差据实记录，不通过等待到达延长测试。
三维转弯指轨迹方向改变；姿态控制仍稳定世界零航向，不宣称已测转艏能力。

## 记录与诊断分层

所有结果直接保存到宿主机：

```text
/home/legion/AUV-simulation/results/ch3/envelope_CASE_时间_后缀/
  summary.json / program.json / scenario.json / vehicle_profile.json
  truth_scene.json / world.sdf / bridge.yaml / sdf_validation.log
  control.jsonl                  每20ms四艇输入状态/输出状态、姿态和时钟
  physical_contacts.jsonl         事件流；零事件不证明严格无接触
  controller.log / gazebo.log / bridge.log / world_control_bridge.log
  target_marker_bridge.log        复用桥接，只为统一节点就绪检查；不设置艇位
  error_traceback.txt             异常时
  report/REPORT.md               当前仿真模型可达能力报告
  report/envelope_metrics.json   每阶段、每艇的详细数值
  report/response.csv            便于Windows画图、比较加速/制动过程
```

四层分别记录：

1. requested_acceleration → speed_governed_acceleration：软件速度反馈调节。
2. speed_governed → governed：每轴加速度裁剪。
3. requested_wrench → allocated_wrench：有界分配，分别报告力N和力矩Nm误差。
4. allocated_thrust → 发布推力 → 实测状态：执行器命令响应与实际闭环运动。

allocated_wrench 和 commanded_wrench 均由安装矩阵推导，不是力传感器实测。
实际加速度由滤波里程计世界速度有限差分估算；其误差包含执行器、水动力/姿态耦合和估计误差，
不能据此唯一反演水动力参数。原始四元数、艇体/世界速度、角速度、推力和状态时间戳均保留。

## 判据和结论范围

工程运行通过：SDF校验成功、完整步数、时钟/共同状态有效、未越任务几何边界、姿态≤20°、
速度≤2 m/s、未观测到接触、正常发零推力并暂停、报告生成成功。
它不代表任务搜索成功，不要求每个阶段都达到稳定速度或完成制动。

稳态诊断：完整驱动阶段末1 s的50点，轴向速度极差≤0.03 m/s且线性趋势绝对值≤0.02 m/s²。
不满足就记录“本次时长内未观察到稳态”，不能把末速度称为稳定速度。
制动诊断：三维速度≤0.05 m/s连续0.5 s；记录该保持段起始时间/路程和确认时间。
未满足则停止时间/距离为null，保留观察窗口与路径。阈值是本工程诊断定义，不替代原捕获判据。
路径误差相对相邻命令航点的有限三维线段，另存阶段末航点误差和终点超调。

当前不解除软件调节，因此测到的是当前软件+艇体的闭环可达范围，不是无约束硬件极限。
不能因为速度调节频繁，就排除推力或控制误差；应结合分配误差、触边比例和实测响应分别解释。
通过本测试也不是厂家参数标定、实艇验证或论文性能结论。正式物理参数/控制上限变化留待单独论证。

Windows完成代码和本地替身验证后，实际范围仍为PENDING_LINUX_EVIDENCE。
请回传四次运行目录（至少summary、program、profile、control及report），再做独立只读复核。
不要通过只发“程序运行成功”替代原始运动记录。原有G4和G6入口保持默认行为。
