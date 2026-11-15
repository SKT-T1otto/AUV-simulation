# 策略执行接口验证记录

日期：2026-11-15。对应新增的策略接口 v2，原 VALIDATION.md 保留 v1 记录。

## 修改范围

新增 policy.py、actors.py、external_zero_actor.py、两份策略配置、协议说明、接口测试及无压缩包的清单更新工具。
调整 controller.py、core.py、setup.py、run_demo.sh 和数值闭环测试。艇体模型、水动力系数和推进器布局保持原样。
vehicle.json 只去除已转移到策略配置的旧控制参数字段，避免执行层重复规划或限幅。

默认 5 Hz 决策、50 Hz 执行。新增外部动作序号/时间戳/配置指纹检查、过期终止、零推力退出。
原策略先验和残差公式是独立适配实现，不导入/改写受保护旧仓库，也不宣称不同物理后端逐步等价。

## 离线验证

`python -m unittest discover -s tests -v`：11 项检查全部通过，包括：

- 28D 观测布局、目标知识门控、坐标旋转和平移。
- 原参考先验公式、归一化残差缩放、各轴限幅和禁用状态。
- 5 Hz 零阶保持、动作序号匹配、时间戳与配置错误拒绝、外部动作超时。
- 策略输出经反坐标变换进入执行器，没有重复叠加航点先验。
- TorchScript 导出元数据/哈希检查，拒绝声明需要团队上下文的单艇模型。
- 实际保存、加载并推理一个临时 TorchScript 线性网络；不是用户训练权重。
- 原艇体几何、浮力配平、推力分配约束和驻留条件。
- 新 5 Hz 策略 + 50 Hz 执行的简化六自由度模型完成五个航点。

简化模型 47.62 s 接受全部航点，额外驻留后位置误差约 0.004862 m。
这是离线测试模型结果，不是 Gazebo 或实艇测量精度。

两个 Bash 脚本通过 `bash -n`。Python 源码通过 AST 语法检查。源文件 SHA256SUMS 使用 LF 换行并更新。
没有生成压缩包，没有运行长训练，也没有修改或加载历史实验 checkpoint。

## 待用户 Linux 验收

当前 Windows 开发环境没有 ROS 2/Gazebo。需要依次在用户容器执行：

1. `bash install_and_build.sh`，包括真实 SDF 验证及 colcon 编译。
2. `bash run_demo.sh`，验收零残差三维航点闭环。
3. `CRK_POLICY_MODE=external CRK_EXTERNAL_ZERO=1 bash run_demo.sh`，验收真实 ROS 消息往返和动作时效。
4. 明确选定 checkpoint 与导出方式、核对配置后，再运行训练 actor。

checkpoint_compatibility_verified 始终为 false；当前没有完成用户具体模型的训练语义核验。
四艇任务与通信仍未接入单艇演示上下文，不能用占位队友观测替代。
