# Zhang（2022）关节加速度估计：UR5复现代码

论文：Jiexin Zhang、Pingyun Nie、Yuhang Chen、Bo Zhang，*A Joint Acceleration Estimation Method Based on a High-Order Disturbance Observer*，IEEE RA-L 7(4), 12615–12622，DOI：[10.1109/LRA.2022.3220501](https://doi.org/10.1109/LRA.2022.3220501)。

本目录是按论文自行实现的UR5适配，不是作者官方代码，也未升级为正式算法包。论文与推导在 [reproductions](../../reproductions/Zhang_2022_JointAcceleration/UR5_MuJoCo_Reproduction.md)，结果及未解决边界统一见 [阶段结论](../../docs/conclusion/record/Zhang_关节加速度估计结论.md)。旧探索目录和全部旧运行结果已按要求删除；本目录不保存结果或回归数据副本。

## 代码职责

| 文件 | 职责 |
|---|---|
| `acceleration.py` | HODO三层同步更新、两层消融、GM与名义加速度组合；迁移时原样保留 |
| `common.py` | 共享模型加载、配置、日志读写及固定评价窗口 |
| `collect.py` | 原UR5控制、J2等效加速度施力、反馈／真值隔离及物理守门 |
| `replay.py` | 配置指定的干净、噪声、阻尼失配和采样对照 |
| `evaluate.py` | 六轴全段／核心段误差、幅相、噪声、采样、频点对照与图 |
| `preflight.py` | 独立更新公式、消融、初始化、GM符号和模型检查；迁移时原样保留 |
| `run.py` | 四阶段CLI，前级失败时停止，禁止覆盖已存在的阶段结果 |
| `docker_step.py` | 已有Docker镜像的资源限制与只读代码挂载 |
| `make_request.py` | 为指定任务生成新工作流请求；不自动执行 |
| `configs/baseline.json` | 原七场景、十种子速度噪声、单独阻尼失配及限定2 kHz对照 |
| `configs/frequencies.json` | 无力运动基线＋0.5/1/2/3/4/6/7/8 Hz，仅无噪声准确模型 |

两份配置的机器人、控制器、增益及评价窗口一致。`baseline`保持原场景顺序和噪声种子映射；`frequencies`从头生成自己的无力运动基线，不再读取过去的Run。历史频点曾分轮采集；此处合并的是可重跑配置，不声称合并后的整套物理采集已经重新执行。

## 运行边界

- 输入：时间戳s、六关节位置rad、速度rad/s、实际驱动力矩N·m；输出：五方法的六轴加速度rad/s²。输出时间与当前反馈时间一致。
- 模型顺序：shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3。固定基座UR5，模型资产不修改。
- 名义模型独立于Plant；仅从反馈计算惯性、偏置、惯性方向导数和黏性摩擦。真实加速度、施力及Plant惯性只用于施力／评价，不作为观测器输入。
- 所有方程、正外力GM符号、同步显式Euler、增益及无输出滤波条件保持。J2等效加速度经Plant惯性矩阵转换为广义力，一般不是仅向J2施加力矩。
- 物理16 kHz、控制1 kHz、记录2 kHz；主要观测1 kHz。全段保留，静止核心0.2–2 s，其余2–8 s。0.5 Hz核心只有3个周期。
- `passed`只表示工程检查通过，不代表三层优于其他方法。噪声是仿真假设；2 kHz对照不等于物理步长收敛。无KF、训练、真机或加速度反馈控制。

通用依赖继续位于原目录：`scripts/DeLuca_2003_GeneralizedMomenta/observer.py`（动量核）及其`external_torque.py`（已有静态预检）；`scripts/rigid_baseline/reference_kf/oracle_kf.py`（只调用`Model`，不运行KF）；`scripts/simulation/ur5_helpers.py`；`scripts/models/ur5/nominal_16khz.mjb`。

## Docker与重新运行

沿用已有`piakf:h022-physical`：Python3.12.3、MuJoCo3.3.7、NumPy1.26.4、SciPy1.14.1、Matplotlib3.9.4；2 CPU、4 GiB、BLAS单线程，不需要安装依赖或重建镜像。容器内Python为`/opt/piakf-venv/bin/python`。

在项目根目录查看入口，不启动实验：

```bash
python3 scripts/Zhang_2022_JointAcceleration/make_request.py --help
```

后续科学计算仍需当前TASK批准及新的UTC Run ID。以下变量替换为获批任务的实际值；选择`baseline`或`frequencies`，每次请求只运行一种预设：

```bash
python3 scripts/Zhang_2022_JointAcceleration/make_request.py \
  --task-id "$TASK_ID" --run-id "$RUN_ID" \
  --preset frequencies --request "$REQUEST_PATH"
```

将生成的准确请求路径绑定到Active TASK的`Execution request`，再使用：

```bash
node tools/research-workflow.mjs flow start
node tools/research-workflow.mjs flow run
```

新请求四阶段依次为`preflight → collect → replay → evaluate`。输出为`runs/<task-id>/<run-id>/`；观测和采集不依赖历史结果。请求保留1200秒墙钟上限，重放限制500万次逻辑更新、100万次模型查询；具体任务还应声明物理步数和累计预算，不因生成请求自动获得计算授权。

容器内单阶段入口如下，`RUN`与`CONFIG`必须指向新Run和选定配置，前级检查必须通过：

```bash
/opt/piakf-venv/bin/python scripts/Zhang_2022_JointAcceleration/run.py \
  --phase preflight --config "$CONFIG" \
  --run-root "$RUN" --output-dir "$RUN/preflight"
```

没有旧工作流恢复入口。迁移只改变目录、运行绑定、配置展开和表格汇总；原核心公式、参数、时间窗及数值比较容差不变。

迁移验证：独立公式与模型预检通过；2001帧含加载反馈的全部输出／内部状态逐值一致；配置驱动的静止重放一致；九张原指标表以及八频点、六轴比较表完全一致。验证复用旧数据，未推进物理仿真；检查结束后删除了历史数据及临时验证输出。
