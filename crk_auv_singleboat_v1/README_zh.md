# 单艇三维航点跟踪工程示例 v1

Windows 项目位置：`E:\gym\code\WORKSPACE\CRK-AUV-Simulation`。源码及交付包已于 2026-11-15 从 C 盘迁到此处。
Linux 容器路径仍为 `/root/crk_auv_singleboat_v1`，下述运行命令保持不变。

目标环境：Ubuntu 24.04 容器、ROS 2 Jazzy、Gazebo Harmonic（Sim 8），已通过 NVIDIA OpenGL / X11 检查。

这是独立的工程示例，不是经过实艇标定的 BlueROV2，不是论文旧任务的等价环境，也没有接入 RL。
既有 CH3、CH4、CH5 代码、奖励、观测、成功判据、历史模型和实验结果均不由本包改写。

## 模型和闭环

- 六自由度刚体，18 kg 总质量。艇体碰撞箱 0.6 x 0.3 x 0.1 m，淡水密度 1000 kg/m^3；排水质量等于总质量，静水中性浮力。
- 八个推进器：四个水平矢量推进器，四个垂向推进器。单推进器指令限制 +/-15 N。
- 几何位置和推力方向统一记录在 config/vehicle.json，同时生成 SDF 和控制分配矩阵。
- 使用 Gazebo 原生 Buoyancy、Hydrodynamics、Thruster、OdometryPublisher 系统。
- 采用保守对角附加质量的插件配置，兼容 Sim 8 小版本；不同时设置 fluid_added_mass。新版本可后续迁移到 DART 原生附加质量。
- 全空间均匀水密度；不模拟自由液面、波浪、真实声呐、DVL、定位误差或水声通信。海底位于 z=-6 m。
- 世界坐标系 map：z 向上，负 z 为水下。里程计位姿在 map，速度在艇体系，控制器显式变换速度。
- ROS 2 /clock 驱动控制时钟。控制频率 50 Hz，物理步长 0.002 s。里程计为 Gazebo 真值。
- 控制链：航点位置误差 -> 期望速度 -> 期望加速度/姿态力矩 -> 有界最小二乘推力分配 -> 一阶响应和推力变化率限制 -> Gazebo。

## 文件结构

src/crk_auv_demo 是标准 ament_python 包，包含控制器、可独立测试的控制核心、配置、世界 SDF 和桥接配置。
install_and_build.sh 安装小量依赖、重新生成模型、运行数值检查和 SDF 校验，并执行 colcon build。
run_demo.sh 在一个终端内管理 Gazebo、桥接、控制器和退出清理；每次结果保存到独立目录。
tests/test_numerics.py 是离线设计检查，不替代 Gazebo 的实际运行验收。

## Linux 操作

1. 将 crk_auv_singleboat_v1.tar.gz 传到 Ubuntu 宿主机 /home/legion/Downloads。
2. 在宿主机执行：

    sudo docker start crk-ros2-dev
    sudo docker cp /home/legion/Downloads/crk_auv_singleboat_v1.tar.gz crk-ros2-dev:/root/
    sudo docker exec -it crk-ros2-dev bash

3. 在容器执行（首次解压，若目录已存在先保留旧目录，不覆盖旧结果）：

    cd /root
    tar -xzf crk_auv_singleboat_v1.tar.gz
    cd /root/crk_auv_singleboat_v1
    sha256sum -c SHA256SUMS
    bash install_and_build.sh

4. 先用 Ctrl+C 结束之前 shapes.sdf 测试及其时钟桥接进程。不要退出容器的主 bash。
5. 在容器执行：

    cd /root/crk_auv_singleboat_v1
    bash run_demo.sh

此时只需要一个操作终端，脚本启动三个进程。Gazebo 窗口应出现蓝色艇体、黑色推进器和金色航点标记。
目标坐标依次为 (0,0,-2)、(2,0,-2)、(2,2,-3)、(0,2,-1.5)、(0,0,-2)。
每个航点要求距离 <=0.2 m、速度 <=0.1 m/s，持续 2 s；最后驻留 10 s 后自动结束。
示例通常约一分钟内完成，但实际结果以本机日志为准。180 s 仿真时间仍未完成会记录超时。
关窗或 Ctrl+C 会中断测试；控制器尽可能先发零推力，再结束桥接和仿真。

## 查看结果

每次运行保存 results/run_时间_随机后缀/：

- trajectory.csv：10 Hz 的位置、航点、误差、速度、姿态、推力和分配残差。
- summary.json：实际终止原因、接受航点数、最终驻留 RMSE、姿态、饱和率及参数快照。
- gazebo.log 和 bridge.log：启动和桥接问题定位。

在容器查看最后一次 summary：

    CRK_LAST_RUN=$(ls -dt /root/crk_auv_singleboat_v1/results/run_* | head -n 1)
    cat "$CRK_LAST_RUN/summary.json"

工程验收要求 summary.json 中 termination_reason=completed、waypoints_accepted=5、engineering_acceptance_passed=true。
工程判据为完成全部航点及驻留、最终驻留 RMSE<0.2 m、全过程最大横滚/俯仰<5 度；不是原论文的任务成功判据。
记录 0.35 m/s 为各世界轴速度指令分量上限，不是三维速度模长上限。

从宿主机复制结果出来，目标目录使用新名字保留既有结果：

    sudo docker cp crk-ros2-dev:/root/crk_auv_singleboat_v1/results /home/legion/Downloads/crk_auv_results_v1

源码暂存在容器文件系统内。删除容器会丢失其文件；上述 docker cp 可导出结果，源码压缩包也要保留。
后续正式项目应迁移到授权的生产目录，并通过宿主机绑定挂载管理源码与结果。

## 问题定位

- 先看终端最早报错，不要重复创建同名容器。
- 无里程计/时钟：控制器 45 s 壁钟等待后退出；查看本次 bridge.log 和 gazebo.log。
- GUI 看不到艇：按左侧模型列表定位 auv_0，并调整相机；模型位于 z=-2，不在原点平面。
- X11 授权失败：重建宿主机 .crk-ros2.xauth 中的授权内容，再启动容器。不要关闭 X11 访问控制。
- 修改 vehicle.json/waypoints.json 后重新执行 install_and_build.sh，保证 SDF 与控制器使用一致参数。
- 待本机基线通过后，再加入海流、定位误差或 RL 残差；每次只改变一个因素。

## 参数可信度和验证边界

此包的质量、阻力、附加质量和推进器系数均为工程起点，未经过水池或 CFD 标定，不能写成实艇高保真模型。
离线检查只证明几何、坐标与控制设计的内部一致性以及简化六自由度模型上的收敛；Gazebo 联合运行尚需用户本机验证。
正式策略接入须先匹配 checkpoint 的观测、归一化、动作映射、角色和团队上下文，不能直接把三维残差当推进器指令。

参考接口：
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Thruster.html
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Buoyancy.html
https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems/odometry_publisher
https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md
