# De Luca（2003）广义动量方法：UR5 仿真复现代码

论文：*Actuator Failure Detection and Isolation Using Generalized Momenta*，De Luca、Mattone，2003，DOI：10.1109/ROBOT.2003.1241665。本目录是项目自己的 UR5 适配实现，不是作者官方代码；来源见 `provenance.json`。论文原文与推导保留在 `reproductions/DeLuca_2003_GeneralizedMomenta/`。

## 代码分类

| 文件 | 职责 |
|---|---|
| `observer.py` | 广义动量残差核心与 UR5 动力学适配 |
| `external_torque.py` | 静止、运动、单轴和并发外力矩验证，含评价 |
| `signed_joint_torques.py` | 六关节正负力矩覆盖 |
| `signed_joint_torques_evaluation.py` | 六轴响应、通道隔离及符号对称性评价 |
| `actuator_loss.py` | 无故障、J2部分失效、J2/J5并发部分失效 |
| `actuator_loss_evaluation.py` | 缺失力矩重构与实际力矩输入对照 |
| `configs/` | 三组已有实验的原配置，未改模型或参数 |

输出符号：核心 `residual` 是正外部关节力矩残差；论文故障输出 `fault_residual=-residual`。外力估计使用实际驱动力矩，故障估计使用名义命令力矩。所有关节力矩单位为 N·m，观测频率为 1 kHz。

## 通用依赖与最小运行资产

- `scripts/rigid_baseline/reference_kf/oracle_kf.py`：只调用 `Model`，不运行 KF。
- `scripts/simulation/ur5_helpers.py`：共用轨迹、关节与碰撞守门，原函数逻辑未改。
- `scripts/models/ur5/nominal_16khz.mjb`：已有16 kHz模型原样迁出，不修改机器人参数。
- `reference_data/measurements.npz`、`reference_data/estimates.npz`：仅保留无外力健康回归输入，供原等价性守门使用；不是完整实验记录。

已解除对旧 scratch/runs 的运行依赖。历史物理日志、逐轮报告和临时代码已按要求清理，结论见 [De Luca阶段结论](../../docs/conclusion/record/DeLuca_阶段结论.md)。模型二进制需使用现有兼容 MuJoCo Docker 环境。

## 在现有 Docker 容器中运行

在项目根目录 `/workspace` 执行。三个入口接受相同参数，必须显式提供输出目录。下面以执行器故障验证为例，`RUN` 应换成当前获批任务的**新输出目录**：

```bash
cd /workspace
RUN=runs/<task-id>/<new-run-id>
python3 scripts/DeLuca_2003_GeneralizedMomenta/actuator_loss.py \
  --phase preflight --run-root "$RUN" --output-dir "$RUN/preflight"
python3 scripts/DeLuca_2003_GeneralizedMomenta/actuator_loss.py \
  --phase simulate --run-root "$RUN" --output-dir "$RUN/simulate"
python3 scripts/DeLuca_2003_GeneralizedMomenta/actuator_loss.py \
  --phase evaluate --run-root "$RUN" --output-dir "$RUN/evaluate"
```

使用安装了现有依赖的 Python；`piakf:h022-physical` 镜像中为 `/opt/piakf-venv/bin/python`。需要指定配置时加 `--config <json>`；模型相对路径按项目根目录解析。

科研执行仍由当前 TASK 与 research-workflow 管理：新请求绑定本目录入口、评价文件、配置及通用依赖。不要改写旧请求或旧运行输出。每阶段先检查 `metrics.json` 的 `passed`，失败即停止；不要直接连续运行后续阶段。本文命令只说明入口，不授权新增实验。

历史报告与旧工作流已清理；新实验须生成新的请求及输出，不能恢复旧Run。
