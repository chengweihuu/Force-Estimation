# 传统刚体辨识＋动量 KF

保留原实现，按功能组织；目录整理不改变动力学方程、损失、控制设置和检查阈值。

| 目录 | 主要入口 | 作用 |
|---|---|---|
| [identification](identification/) | [rigid.py](identification/rigid.py)、[experiment.py](identification/experiment.py) | 已知运动学下的回归矩阵、区间算子、正对照、联合求解与评价 |
| [physical_constraints](physical_constraints/) | [constrained.py](physical_constraints/constrained.py)、[evaluate.py](physical_constraints/evaluate.py) | 惯性物理可行性与摩擦约束求解、评价；recover.py 保留原恢复入口 |
| [reference_kf](reference_kf/) | [momentum_observer.py](reference_kf/momentum_observer.py)、[oracle_kf.py](reference_kf/oracle_kf.py)、[run.py](reference_kf/run.py) | KF 数值核、参考动力学与仿真基线 |
| [model_comparison](model_comparison/) | [models.py](model_comparison/models.py)、[replay.py](model_comparison/replay.py)、[evaluate.py](model_comparison/evaluate.py) | 模型查询、共同 KF 重放及结果对比 |
| [covariance_checks](covariance_checks/) | [guarded_replay.py](covariance_checks/guarded_replay.py) | 原协方差检查、核心段重放与报告 |
| [config_examples](config_examples/) | 同名 JSON 配置 | 历史数值和字段示例；旧 Run 输入已删除 |
| [environment_tools](environment_tools/) | build_compose.py、build_physical_image.py、create_legacy_venv.py、check_svd_runtime.py | 历史构建、安装及库诊断，运行算法不需要先执行这些程序 |

依赖集中在 [UR5 模型](../models/ur5/INDEX.md)、[采集工具](../data_collection/) 和本目录。历史 PINN 对照保留 [pinn_support](model_comparison/pinn_support/) 的模型类及适配器；传统刚体模型查询不要求 PINN 权重。

## 使用顺序

1. 根据 [采集方法](../../docs/conclusion/UR5_Simulation_Data_Method.md) 准备完整物理步记录、模型快照和训练归一化。
2. 完成真实参数正对照，再构造训练回归矩阵与求解。不能把任意参数当作辨识结果。
3. 检查物理可行性及开发表现，再准备同步反馈用于 KF 重放。

这些是重新开展工作的顺序，整理没有执行上述步骤。历史三模型评价程序还需要原 PINN 对照输入；不能因为入口能够导入就宣称数据链可直接重放。

## 配置与环境

五个配置已集中到 `config_examples/`，源码路径已更新；原始数据路径保留为历史示例。使用前创建与新输入对应的配置，并传入 `--config` 和输出位置。旧 workflow 请求已删除。

在已有容器的 `/workspace` 下查看参数：

```bash
/opt/piakf-rigid-venv/bin/python scripts/rigid_baseline/identification/experiment.py --help
/opt/piakf-rigid-venv/bin/python scripts/rigid_baseline/physical_constraints/constrained.py --help
/opt/piakf-rigid-venv/bin/python scripts/rigid_baseline/model_comparison/models.py --help
```

依赖沿用 [Docker 环境说明](../../docker/DEPENDENCIES.md)。源码仍属探索实现，原数据、theta.npy、归一化和历史权重不随源码恢复。[方法与历史结论](../../docs/conclusion/UR5_Dynamics_Learning_Conclusion.md)。
