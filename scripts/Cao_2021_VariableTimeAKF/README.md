# Cao 2021：电流预滤波与动量KF

本目录保存本地UR5仿真复现代码，不是作者官方软件。当前主入口完成“匀速标定 → 加载运动 → RAW/HMA/MSMA配套协方差 → 三个配对噪声种子评价”。电流系数和摩擦已知，反馈及KF为125 Hz。

[阶段结论与主要结果](../../docs/conclusion/Cao_2021_VariableTimeAKF/Cao_电流滤波与估力结论.md) · [论文与方法说明](../../reproductions/Cao_2021_VariableTimeAKF/UR5_MuJoCo_Reproduction.md) · [代码来源](provenance.json)

## 先看这几个文件

```text
Cao_2021_VariableTimeAKF/
├── experiment/                  当前完整实验入口
│   ├── prepare.py               生成新配置和workflow请求；不执行实验
│   ├── experiment.py            六阶段调度、标定表构造
│   ├── engineering_checks.py    电流、协方差、时序和控制检查
│   ├── evaluation.py            三种子配对估计、六轴指标和曲线
│   └── request.template.json    固定六阶段与预算模板
├── configs/
│   ├── core_comparison.json     当前固定比较参数，不绑定旧Run
│   ├── base.json                模型、采样、控制器及KF基础参数
│   ├── thresholds.json          固定六轴电流阈值
│   └── …                       历史分项实验配置
├── current_interface.py         实际力矩与等效电流接口
├── msma.py                      因果WMA/HMA/MSMA
├── integral_controller.py       前馈＋PD＋受限积分
└── …                            主入口复用模块及历史诊断
```

本轮已验证的三个实验脚本从`scratch`归入`experiment/`，仅调整导入路径；计算函数、比较规则和固定参数保持原样。历史模块继续保留原位置，避免破坏本地导入和旧运行快照。当前入口不读取旧运行，也不依赖原`scratch`代码。

通用动力学/KF仍在`../rigid_baseline/reference_kf/`，仿真工具在`../simulation/`，UR5模型在`../models/ur5/`；本目录没有复制这些公共实现。这里仍是论文复现代码，不因归类而升级为正式证据或`src/`公共接口。

## 当前实验的六个阶段

|顺序／mode|工作|输出及限制|
|---|---|---|
|1 `units`|37项滤波检查、等效电流、KF等价性和协方差时序|453次预检KF；无物理步|
|2 `path`|固定TCP方向的6 s竖直路径|6001点；位置/速度/加速度一致性|
|3 `precheck`|控制器顺序、抗饱和及局部稳定性检查|仅751个新路径状态；不是非线性稳定证明|
|4 `calibrate`|12段正反0.05 rad/s、每段8 s|1536000物理步；312项残差方差表|
|5 `collect`|一次竖直运动与偏置等效加载|96000物理步；运动门槛通过才继续|
|6 `compare`|三组×种子8801/8802/8803|6183次KF；六轴、四窗口、配对差、响应和图片|

窗口50、阈值和KF其余参数固定；三组各自递推P/K。估计从0.512 s开始，每组687帧。科学运行预算300 s，2 CPU/4 GiB；失败即停止，不自动重采或调参。详细定义见`configs/core_comparison.json`，算法和统计口径见结论文档。

## Docker运行方式

先在项目根目录准备新绑定。以下`H-101`只是命令格式示例，实际应使用已批准的Active TASK ID：

```bash
python3 scripts/Cao_2021_VariableTimeAKF/experiment/prepare.py --task-id H-101
```

命令生成`scratch/<task-id>/cao-<新UTC Run ID>/config.json`和`request.json`，并打印路径。它只写配置，不运行Docker、物理仿真或KF，也不修改TASK。目标Run或绑定已存在时拒绝覆盖。

在项目TASK中引用生成的请求、确认范围及预算后，通过已有workflow执行：

```bash
node tools/research-workflow.mjs flow start
node tools/research-workflow.mjs flow run
```

请求会在已有`piakf`容器中执行六阶段，代码挂载到`/workspace`，每步明确绑定`/output`。全部物理日志、估计、指标和失败记录写到新Run。缺少路径、预检或采集通过记录时不能跳过检查直接比较。生成新Run不是自动获得科研授权。

查看当前入口而不启动计算：

```bash
docker compose -f docker/compose.yaml run --rm --no-deps -T \
  -e PYTHONDONTWRITEBYTECODE=1 piakf /opt/piakf-venv/bin/python -B \
  scripts/Cao_2021_VariableTimeAKF/experiment/experiment.py --help
```

`--config`、`--output-dir`和`--mode`均为显式参数。`prepare.py`只绑定路径和身份，不更改科学参数；不能直接拿历史`paper_controller.json`中的旧Run路径启动当前实验。

## 主入口复用的计算模块

|职责|文件|
|---|---|
|电流输入与滤波|`current_interface.py`、`msma.py`|
|接口与滤波性质检查|`checks.py`、`filter_checks.py`|
|匀速轨迹与采集|`constant_speed_v2.py`|
|竖直参考路径|`paper_motion.py`|
|控制与实际物理采集|`integral_controller.py`、`controlled_collection.py`|
|标定表读取与KF协方差|`akf_comparison.py`中的`CaptureObserver`、`read_table`|
|同一数据上的滤波与重放|`paper_compare.py`中的`filter_inputs`、`replay`|
|数组记录、JSON/CSV及动态加载|`runner.py`、`input_statistics.py`|

复用模块有历史CLI；当前实验调用其中已验证的函数，不绕过历史CLI对原始数据的检查，也不伪造旧检查通过。

## 历史诊断的用途

|问题|保留代码|
|---|---|
|初始接口、白噪声与短段接入|`runner.py`、`filter_probe.py`|
|运动误差统计、噪声与失真分解|`input_statistics.py`、`frozen_schedule.py`|
|旧窗口标定与迁移|`constant_speed.py`、`constant_speed_v2.py`、`window_transfer.py`|
|人工协方差与同口径RAW对照|`akf_comparison.py`、`raw_calibrated.py`|
|旧单种子竖直加载与跟踪诊断|`paper_compare.py`、`tracking_diagnostic.py`、`controlled_collection.py`|
|同序列三倍噪声对照|`noise_comparison.py`|

`configs/constant_speed.json`、`paper_controller.json`和`noise_comparison.json`是历史配置，部分字段引用已清理的数据。它们用于理解原设计，不是数据齐备的一键运行包。需要旧离线分析时，使用路径修正后的`constant_speed_v2.py`，显式提供新获授权的数据绑定；不要删守门假装旧数据存在。

19个原模块的恢复来源仍保存在`provenance.json`。本次保留原源码和旧快照，不重建或删除旧运行，不改变滤波核、积分修正或KF默认配置。
