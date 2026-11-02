# PIAKF 结构化动力学学习阶段交接

## 当前可以接续的内容

已完成MuJoCo／UR5上的结构化动力学网络、动量KF、无接触及已知加载评价、静止监督配对和分量诊断。当前最明确的结果是：静止监督改善了独立静态姿态的重力预测，但持续运动和加载指标仍有改善与退化，尚不能宣布估力可用或完整论文复现成功。

后续无需从聊天记录或旧Run目录恢复。先读[阶段摘要与图表](../../scripts/PIAKF/conclusion/summary.md)，再读[PINN动力学方法论](PINN机器人动力学估计方法论.md)。源码与使用入口见[scripts/PIAKF](../../scripts/PIAKF/README.md)。

## 方法和信息边界

论文为Hu、Ma、Lu、Zhuang的 *Physics-informed adaptive Kalman filter for contact force estimation in industrial robots considering model uncertainty*，DOI [10.1016/j.rcim.2026.103292](https://doi.org/10.1016/j.rcim.2026.103292)。本地是基于笔记和正文的重实现，没有作者源码，UR5／MuJoCo替代原SN7B／Simscape。

当前网络从位置输出正定惯性和重力，从同一惯性函数推导速度项，另学习非负黏性摩擦。三层128节点ReLU、float64。动量KF使用`y=Mhat*v`和`u=tau_ref+Mdot_hat*v-c_hat-G_hat-friction_hat`，wrench为作用于tool0、世界轴表达的六维量。

动态基线用四条无接触运动的实际驱动力矩训练，加速度由速度差分得到，训练不使用真实`M／c／G`标签。增强组在相同初始化、动态采样和1000次更新预算下，额外使用25个合成静止平衡力矩标签，静止损失权重1。共享隐藏层可能使静止监督间接改变惯性。

KF没有参与网络反向传播。TCN、Gaussian换向补偿、变窗口WMA、论文完整摩擦与四方法消融尚未实现；不可把当前程序称为完整PIAKF或原论文数值复现。

## 阶段结论的使用方式

保留以下观察及其不利结果，不扩展其适用范围：

- 原训练集归一化力矩MSE约0.00171，开发集约0.551，存在明显泛化差距。
- 静止增强组在48个非训练静止姿态上改善重力；J2重力RMSE从1.157降至0.142 N·m。
- 运动核心`[1,5)`秒：轨迹04的Fx RMSE从1.068降至0.409 N，轨迹05从0.555升至0.623 N。
- 已有阶跃加载误差改善，正弦幅值更接近参考，但正弦延迟和RMSE略增。
- 轨迹05共同协方差诊断中，仅换增强重力就使Fx RMSE从0.555升至0.648 N；单换惯性或摩擦略有改善。误差信号具有交叉项，RMSE不能相加为贡献百分比。

这些是单种子、少量已有轨迹上的EXPLORE观察，不是已接受的claim-level evidence。两条运动评价轨迹已多次用于开发与诊断，不是封存测试。静止评价只有轴向偏移，不能证明全工作空间泛化。

## 环境和归档校验

从仓库根目录执行：

```bash
python3 scripts/PIAKF/verify_archive.py
docker compose -f docker/compose.yaml run --rm --no-deps --pull never -T piakf python scripts/PIAKF/verify_archive.py --runtime
```

第二条检查归档数组、四份权重、原训练统计和已有32帧的15组诊断及原生端点回归；0训练更新、0物理积分步，不启动ROS或真实机器人。不要把归档校验误当作新增实验。

已有镜像为`piakf:jazzy-mujoco3.12.0-control0.1.2`，image ID为`sha256:60362f5b84c36e35e00d0e495e4bc2a70553f0af58541ebdc0345d93d9dd5c23`。Python3.12.3、MuJoCo3.12.0、CPU PyTorch2.9.1、NumPy1.26.4、SciPy1.14.1、Matplotlib3.9.4；Docker限制2 CPU／4 GiB、单数值线程、无网络。

模型为已跟踪的`scripts/models/ur5/scene_equivalent_current.xml`，仅黏性阻尼、零干摩擦／armature，理想关节侧力矩，不包含实机电流换算误差。模型、控制设计和环境文件的校验值保留在[归档清单](../../scripts/PIAKF/archive_manifest.json)；这些依赖本次不修改。

## 保留资产与精度限制

`scripts/PIAKF/reference_data/`保存六条动态测量、三条加载测量与参考、初始化／动态／配对基线／增强权重、静止姿态及配对动态batch序列。原生输出只保留端点回归需要的时间和wrench，保留前后数组逐项一致。

`scripts/PIAKF/conclusion/`保存完整逐方向指标、CSV、关键PNG／SVG、原科学配置和八个原Run的紧凑索引。活跃配置的资产路径已改到归档；原历史请求中的`runs/`路径只说明来源。

归档不保留优化器状态、完整协方差／新息历史、载体日志或重复输入快照。保留的数据和源码可以支撑局部重算及回归；不能恢复原Run的全部文件树、逐输入副本或Formal级不可变绑定。原始大目录的清理不会把这些探索结果升级为正式证据。

## 后续研究决策

当前优先级是确定实际使用范围、增加有辨识信息的姿态／速度／加速度组合，并固定独立轨迹和加载验收。已有数据没有比较“同一网络只增加有效覆盖”与“只改变优化方法”，因此不能把根因直接判为数据量或训练技巧。

逐关节追踪到Fx可以暂缓；只有它会改变数据或模型选择时才继续。更有价值的候选包括多关节组合静止姿态、多速度正反向运动和不同加速度激励，不能把相似轨迹的更多相邻帧当作独立信息。

用户提出的 *Physics informed machine learning model for inverse dynamics in robotic manipulators*，DOI [10.1016/j.asoc.2024.111877](https://doi.org/10.1016/j.asoc.2024.111877)，目前仅核对摘要和公开章节，未取得完整训练细节、未复现E2NN。精读时关注数据效率、方程嵌入及能否提供动量KF所需的惯性和导数。DeLaN与激励轨迹设计文献可作为互补参照。

本交接不自动启动下一轮训练。新增数据处理或训练treatment按仓库规则准备任务，先固定最小可判别对照；不要重用已分析轨迹冒称独立holdout，不同时改变数据、网络和KF。
