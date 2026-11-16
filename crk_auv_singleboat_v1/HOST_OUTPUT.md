# Linux 宿主机直接保存结果

本项目后续结果统一保存在 Ubuntu 宿主机 `/home/legion/AUV-simulation` 下：

```text
/home/legion/AUV-simulation/
├── results/run_日期_时间_随机后缀/   # CSV、策略记录、summary、运行日志
├── figures/                       # 后续生成的图表
├── reports/                       # 后续生成的分析报告
├── source/                        # 建议的 Git 源码目录
└── .container/xauth               # 本机图形授权，不导出、不提交 Git
```

results、figures、reports 在配置时创建；图表和分析报告只在相应分析功能实际执行后产生。
每次实验使用独立结果目录。数据在仿真过程中直接写入宿主机，无须结束后 docker cp 或 tar 导出。
结果和本机图形授权均不提交 Git。直接用 ToDesk 打开上述目录即可手动导出结果。

## 一次性配置

如果提示符是 `root@...#`，先 `exit` 回到 `(base) legion@...$`。
停止旧的 Gazebo 场景及桥接后，在 Ubuntu 20.04 宿主机执行：

```bash
mkdir -p /home/legion/AUV-simulation
if [ -d /home/legion/AUV-simulation/source/.git ]; then
    git -C /home/legion/AUV-simulation/source pull --ff-only
else
    git clone https://github.com/SKT-T1otto/AUV-simulation.git /home/legion/AUV-simulation/source
fi
bash /home/legion/AUV-simulation/source/tools/run_linux.sh --setup-only
```

脚本会从已有 `crk-ros2-dev` 容器创建本地环境镜像，再创建 `crk-ros2-home`。
沿用已安装的 ROS 2、Gazebo 和依赖，不重新安装 ROS。旧容器及其结果保留，不会被删除。
首次会将旧容器工作空间 results 中的结果复制到宿主机，跳过已有同名文件；此后采用目录挂载直接写入。
环境快照会额外占用磁盘空间，大小取决于旧容器的可写层。

新容器将宿主机结果根目录挂载到相同路径，将当前宿主机源码仓库挂载到 `/root/AUV-simulation`。
首次在新工作空间编译一次，build/install/log 保留在 source 工作空间内，不作为实验结果导出。
脚本需要当前桌面的 DISPLAY 与 xauth 授权；每次刷新 cookie。
授权文件所在目录整体挂载，因此不会因 xauth 原子替换文件而继续使用旧 cookie。

脚本会校验复用容器的挂载类型、源目录、目标目录和可写状态；配置不匹配时退出。
可以显式指定新的 `CRK_CONTAINER` 名称配置另一容器，无须删除已有容器。

## 后续运行

在 Ubuntu 宿主机执行，不加 sudo：

```bash
bash /home/legion/AUV-simulation/source/tools/run_linux.sh
```

脚本自行调用 sudo 启动 Docker。启动后可立即从宿主机查看 results/run_*。
目录归桌面用户所有，运行结束后将本次目录内文件的所有权交回该用户；不更改历史结果内容。
Ctrl+C 会停止本次实验并执行清理。旧的 run_and_export.sh 不再是推荐入口。

外部零残差接口测试仍通过同一个入口：

```bash
CRK_POLICY_MODE=external CRK_EXTERNAL_ZERO=1 \
    bash /home/legion/AUV-simulation/source/tools/run_linux.sh
```

CRK_POLICY_FILE、CRK_ACTOR_PATH、CRK_ACTOR_METADATA_PATH 也会传入容器。
模型及配置路径须位于已挂载目录内，例如 `/home/legion/AUV-simulation/models/`。
这不意味着任意历史 checkpoint 已经兼容；原策略语义核对要求不变。

## 检查本次摘要

在宿主机执行：

```bash
CRK_LAST_RUN=$(ls -dt /home/legion/AUV-simulation/results/run_* | head -n 1)
cat "$CRK_LAST_RUN/summary.json"
```

首次闭环验收检查 termination_reason=completed、waypoints_accepted=5、engineering_acceptance_passed=true，
并确认 policy_mode。后续需要分析实验性能时，再导出完整轨迹、策略记录和图表。

## 直接在容器内运行

推荐使用宿主机入口。若手动进入新容器，运行方式是：

```bash
sudo docker exec -it crk-ros2-home bash
cd /root/AUV-simulation/crk_auv_singleboat_v1
bash run_demo.sh
```

旧的、没有挂载宿主机输出目录的容器会明确报错，不再静默保存到容器 /root 或 /tmp。
单独启动 controller 时，默认结果也放到上述根目录下的新 results/manual_*；自定义 output_dir 必须在根目录内。

实现依据：[Docker bind mounts](https://docs.docker.com/engine/storage/bind-mounts/)、
[Docker commit](https://docs.docker.com/reference/cli/docker/container/commit/)。
本地已做脚本及存储路径检查；真实 Docker 挂载、显卡与 Gazebo 组合需要在 Linux 按以上步骤验收。
