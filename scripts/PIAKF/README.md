# PIAKF 本地重实现与阶段归档

当前完成结构化动力学学习、动量 Kalman 滤波、静止监督配对和冻结分量诊断。对象为 MuJoCo 中的 UR5，不是原论文 SN7B 数值复现；TCN、Gaussian 换向补偿、变窗口 WMA 和完整消融尚未实现。

阶段结果见[摘要](conclusion/summary.md)，续接工作从[HANDOFF](../../reproductions/PIAKF/HANDOFF.md)开始，模型与训练设计见[方法论](../../reproductions/PIAKF/PINN机器人动力学估计方法论.md)。结果均为 EXPLORE 的有限条件观察，不是 claim-level evidence。

## 代码入口

保留现有模块路径，避免归档整理改变科学实现。

| 分工 | 文件 |
|---|---|
| 结构化惯性、重力、摩擦及位置导数 | [structured.py](structured.py) |
| 已知模型、几何与动力学接口 | [dynamics.py](dynamics.py) |
| 动量 KF、离散化与协方差更新 | [observer.py](observer.py) |
| 仿真、评价与基础检查 | [simulation.py](simulation.py)、[checks.py](checks.py)、[learning_checks.py](learning_checks.py) |
| 动态数据采集、训练和加载回放 | [learning.py](learning.py)、[run.py](run.py) |
| 持续运动评价 | [motion_evaluation.py](motion_evaluation.py) |
| 静态重力与静止监督配对 | [static_gravity.py](static_gravity.py)、[static_constraint.py](static_constraint.py) |
| 冻结分量替换与误差抵消诊断 | [diagnosis.py](diagnosis.py)、[motion_diagnosis.py](motion_diagnosis.py)、[paired_motion_diagnosis.py](paired_motion_diagnosis.py) |
| 归档完整性与32帧端点回归 | [verify_archive.py](verify_archive.py) |

## 保留资产

- `reference_data/learning/`：六条各6秒的四字段测量，整轨迹4／1／1划分；不是未使用的封存测试。
- `reference_data/checkpoints/`：初始化、原动态模型、配对基线、静止增强模型及原训练统计量；最后三者都是1000次更新终点。
- `reference_data/load/`：无力、20 N阶跃、20 N／2 Hz正弦的测量与独立参考。
- `reference_data/native/`：原生 KF 的时间与wrench回归数组，已去掉协方差和新息历史。
- `reference_data/static_training.npz`、`static_evaluation.npz`、`shared_dynamic_batches.npz`：静止监督／评价姿态与配对动态采样序列。
- `conclusion/`：阶段摘要、完整指标JSON／CSV、关键PNG／SVG和原八个Run的紧凑索引。

[archive_manifest.json](archive_manifest.json)记录每项资产的来源与校验值，以及已跟踪机器人和环境依赖的校验值。配置仅迁移资产路径；八份原配置保留在`conclusion/config_snapshots/`，校验程序比较迁移前后的科学字段。

## 校验归档

从仓库根目录运行只读文件检查：

```bash
python3 scripts/PIAKF/verify_archive.py
```

已有镜像中的数值回归检查：

```bash
docker compose -f docker/compose.yaml run --rm --no-deps --pull never -T piakf python scripts/PIAKF/verify_archive.py --runtime
```

这只读取归档，核对数组、权重、训练统计和已有32帧结果；不训练、不调用物理积分、不启动ROS。原Run尚在时可加`--compare-originals`检查搬迁前后内容。

当前环境为Python3.12.3、MuJoCo3.12.0、CPU PyTorch2.9.1、NumPy1.26.4、SciPy1.14.1、Matplotlib3.9.4，2 CPU／4 GiB、单数值线程。镜像及来源信息见[provenance.json](provenance.json)。

## 后续运行边界

`configs/`保留这轮的科学参数，资产路径已指向本目录。后续科研计算须先准备相应TASK及`tools/research-workflow.mjs`请求，并为新Run显式指定输出目录；这些配置不会自动启动实验。

原驱动器日志、完整协方差、输入副本和优化器状态不保留。可用保留数据重做对应计算和端点回归，但不能从紧凑归档恢复原逐Run文件树，也不能把此归档追溯标记为FORMAL冻结证据。
