# 脚本入口

按用途组织；保留探索实现，不表示已经升级为正式算法包。算法、模型参数、采样和守门规则保持原样。

```text
scripts/
├── models/ur5/             UR5 场景、网格、来源与许可证
├── simulation/            运动、施力检查与可视化
├── data_collection/       试采、高频记录与区间监督工具
├── DeLuca_2003_GeneralizedMomenta/  广义动量与执行器故障复现
├── Zhang_2022_JointAcceleration/   GM辅助关节加速度估计与消融
└── rigid_baseline/
    ├── identification/    刚体回归、SVD 与评价
    ├── physical_constraints/  物理约束求解与评价
    ├── reference_kf/      参考动力学与动量 KF 数值核
    ├── model_comparison/  模型查询、KF 重放及对比
    ├── covariance_checks/ 协方差检查与重放
    ├── config_examples/   历史配置示例，旧数据路径已失效
    └── environment_tools/ 历史环境构建及数值库诊断工具
```

## 从哪里开始

| 要做的事情 | 文件／说明 |
|---|---|
| 查看模型来源与默认场景 | [模型说明](models/ur5/INDEX.md) |
| 使用只读等效电流 | [equivalent_current.py](simulation/equivalent_current.py)，由位置控制启动文件加载所选配置 |
| 检查六轴小齿隙配置 | [check_backlash_configuration.py](simulation/check_backlash_configuration.py)，仅静态检查，不推进仿真 |
| 检查运动与反馈 | [check_motion.py](simulation/check_motion.py) |
| 检查末端外力施加 | [check_external_force.py](simulation/check_external_force.py) |
| 打开 MuJoCo 窗口 | [view_motion.py](simulation/view_motion.py)，在宿主机桌面终端调用 |
| 采集无外力轨迹 | [collect_pilot.py](data_collection/collect_pilot.py)，须指定输出位置 |
| 记录每个物理步 | [collect_highrate.py](data_collection/collect_highrate.py) |
| 查看区间算子与固定设置 | [interval_loss.py](data_collection/interval_loss.py)、[supervision_settings.json](data_collection/supervision_settings.json) |
| 使用传统辨识＋KF | [专用入口](rigid_baseline/INDEX.md) |
| 使用Zhang关节加速度估计 | [代码与Docker入口](Zhang_2022_JointAcceleration/README.md) |
| 回顾科学结论 | [结论索引](../docs/conclusion/INDEX.md) |

## 调用约定

以下命令在项目根目录执行，只显示参数，不启动实验：

```bash
# 已有容器内：仿真和采集使用主环境
/opt/piakf-venv/bin/python scripts/simulation/check_motion.py --help
/opt/piakf-venv/bin/python scripts/data_collection/collect_pilot.py --help
/opt/piakf-venv/bin/python scripts/data_collection/collect_highrate.py --help

# 已有容器内：刚体回归使用独立环境
/opt/piakf-rigid-venv/bin/python scripts/rigid_baseline/identification/experiment.py --help

# 宿主机桌面终端：查看可视化入口参数
python3 scripts/simulation/view_motion.py --help
```

运动和采集脚本默认读取本仓库的 `models/ur5/scene.xml`，不依赖终端当前目录。可视化入口将项目中的模型路径转换为 Docker 挂载下的 `/workspace/...` 路径。

环境以 [docker/DEPENDENCIES.md](../docker/DEPENDENCIES.md) 为准。`environment_tools/` 是历史辅助程序，部分会安装依赖或构建镜像，不是日常运行前的必选步骤。

## 现有输入边界

旧数据、归一化与辨识参数已删除。`config_examples/` 保留字段与数值设置，**不能直接作为当前实验配置运行**；重新使用前需绑定新的输入和显式输出目录。高频采集的 `--original-source` 需要基础采集的配置及模型快照，`--collector` 指向当前的 `data_collection/collect_pilot.py`。

历史 PINN 对比只保留两份必要模型支持源码，没有训练入口。区间工具仍含原 Torch 辅助函数，未在整理中重写算法。

旧 `step01/`、`step02/`、`ur5_model/` 及其旧文件名已替换，不再提供兼容链接。历史实验结果已按用户要求清理，结论统一见结论索引。Zhang代码迁移的核对仅使用既有数据和独立预检，没有新增物理仿真、训练、KF或机器人运行。
