# KF 外力估计的仿真验证设计

记录日期：2026-10-21。

状态：**讨论稿（THINK）**。本文不是 Frozen Protocol，不是 Experiment，不构成 Evidence；
执行前需要单独的 TASK 和用户确认。

适用对象：`reproductions/KF/KF_Wrench_Estimation.md` 所述的“给定动力学模型，用
Kalman Filter 递推估计末端 wrench”。

## 1. 问题与判断

如果 observer 直接读取 MuJoCo 的 `M(q)`、`C(q,q̇)q̇ + g(q)`、`J(q)`，plant 与 observer
的刚体模型在参数和结构上完全一致，那么 KF 外力估计的仿真只能验证递推实现，无法反映
模型误差下的估计性能。

这个担心真实存在，而且正好对应《KF_Wrench_Estimation.md》的“模型误差与真实外力的可
区分性”一节（`:292`）：模型误差与真实外力的可区分性，往往是精度的主要限制。完美模型
仿真无法检验这个限制。

但结论不是“仿真没用”。MuJoCo 的价值不是“物理真实”，而是 **真值可知、误差可控、实验
可重复**。真机上模型误差不可控、不可知；仿真里可以把它当作 treatment 操纵，直接观察它
如何映射到 wrench 误差。不要用 MuJoCo 冒充真机，要用它分解真机上分不开的误差。

## 2. 为什么完美模型仿真不够

### 2.1 verification 与 validation

- **Verification**：递推、`C` 约定、`Q/R` 尺度、离散化、可观测性预测是否正确。完美模型
  仿真可以做，而且应该做。
- **Validation**：在真实模型误差、传感噪声和时序条件下，估计性能如何。完美模型仿真不能
  做。

把 E0 的误差收敛曲线当成“估计性能”，就是拿 unit test 当论文结果。

### 2.2 连 Q/R 都不可迁移

真实系统里，动量过程噪声 `Q_p` 的主要作用就是吸收未建模动态。模型误差为零时，最优
`Q_p` 会偏小，滤波器过度自信，对突变接触响应变慢。在完美模型上调好的 `Q/R`，到真机上
往往要重调。因此 E0 的结论只能是“实现正确”，不能是“参数最优”。

## 3. “完全一致”的准确含义

即使 observer 直接读 MuJoCo 的 `M`、`Cq̇+g`、`J`，也不等于 plant 与 observer 完全
一致。至少还有四层：

1. **离散化/积分器**：plant 用 MuJoCo 的积分器和约束求解器，observer 用采样周期 `T` 的
   一阶离散。
2. **摩擦与被动力**：`damping`、`frictionloss`、armature、弹簧等；`mj_rne` 不包含这些。
3. **执行器与力矩信号**：observer 拿到的是指令/电流换算，不是 `qfrc_actuator`。
4. **传感与同步**：`q` 量化、`q̇` 差分/滤波、采样保持、延迟。

反过来，如果把 plant 做成理想刚体 + 理想力矩源，observer 又读同样的三项，那才是真正的
“完全一致”；这时它确实只是 unit test。若 plant 有上述效应而 observer 只读三项，本身
就已经有结构误差——只是这个误差是**无意中引入的、不可解释的**。实验设计的目标，是把它
变成**受控的**。

## 4. 误差阶梯

研究意义不来自“跑通”，而来自“在什么条件下成立/失效”。建议按以下阶梯组织：

| 层级 | plant | observer | 回答的问题 | 能支持的说法 |
|---|---|---|---|---|
| E0 | 真参数、无噪声、无延迟、同一积分器 | 同参数 | 递推、`C` 约定、`Q/R` 尺度是否正确；与 batch MAP 是否一致 | 实现正确性；不能支持性能 |
| E1 | 真参数 + 真实传感链（量化、`q̇` 差分、力矩换算、延迟） | 同参数 | 只有噪声时误差下限是多少；`Q/R` 是否与经验新息协方差一致 | 噪声-带宽权衡（条件性） |
| E2 | 真参数 + 真实传感链 | 标称参数（有依据的独立模型或扰动） | 参数误差如何映射为 wrench 偏差、哪个参数最敏感 | 给定不确定性下的条件性能 |
| E3 | 真参数 + 结构误差（摩擦、armature、执行器、工具偏移、非末端接触） | 简化模型 | 哪些误差不可辨识、会伪装成什么 wrench | 可辨识性边界/失效模式 |
| E4 | 时变/未建模（负载变化、柔性、基座） | 标称模型 | 鲁棒性与失效边界 | 条件性鲁棒性 |

E0 有用，但它是 unit test；E2/E3 才是复现的研究内容。

### 4.1 E2 的两种 nominal model

E2 里要区分两种 nominal model：

- **独立来源**的标称模型（CAD vs 辨识、PINN vs 辨识），回答“真实机器人上会怎样”；
- **从真值随机扰动**的 Monte Carlo，回答“对哪个参数最敏感”。

两者不能互相替代；只做随机扰动，研究的是自己的假设，不是真实模型误差。plant 和
observer 的参数集必须来自不同的、可解释的来源，不能只是同一组参数加随机数。

### 4.2 E3 的非末端接触

E3 里“非末端接触”尤其值得做：文档 `:304–312` 指出，真实关节力矩可能是
`Σᵢ Jᵢᵀ wᵢ`，把全部残差归到末端可能不存在一个准确的末端 wrench；在仿真里很容易构造
（在中连杆施加已知 wrench），而且没有任何参数调参能修复。它能直接回答“KF 输出的六个
数什么时候不代表真实末端接触”。

## 5. MuJoCo 实现要点

### 5.1 两个模型对象，而不是同一组项

- plant 用 true 参数，observer 用 nominal 参数的第二个 `MjModel` + 自己的 `MjData`。
- 同 XML 修改 `body_mass`/`body_inertia`/`body_ipos`/`dof_damping`/`dof_frictionloss`/
  `dof_armature`/`actuator_gainprm`。
- 不要让 observer 直接调用 plant 的 `mj_fullM`/`mj_rne`/`mj_jac`；即使参数相同，也应
  在自己的模型上计算。这样 E0 的“一致”是“两个模型参数相同”，而不是“共享同一份计算”。
- 同库不同模型可以避免把 MuJoCo 与 Pinocchio 的实现差异误当成模型误差；Pinocchio 可用
  来做独立交叉校验（`rnea`/`crba`），但不必做 observer。项目镜像里两者都已固定
  （`docker/DEPENDENCIES.md`），不需要新依赖。
- observer 内部 `M`、`C`、`g`、`J` 必须来自同一个 nominal model，测量方程和预测方程要
  用同一个 `M`；不要一半用 plant、一半用 nominal。

### 5.2 真实 wrench 的注入与真值

- 用 `mj_applyFT` 在 site 点施加已知 `[f; m]`，或先核对 `xfrc_applied` 的 torque 参考点
  再用。不同接口对 torque 是关于 body CoM、body origin 还是指定 point 容易混，先做小
  实验核对。
- 换参考点用 `m_e = m_p + (p − e) × f`。
- 不要把 `qfrc_constraint` 当 truth——它是约束求解器的输出，参考点和方向都要换算。

### 5.3 观测器输入链要像真机

- `q` 量化 + 噪声。
- `q̇` 由 `q` 差分/滤波得到，不要直接给 `qvel`。
- `τ` 用指令/电流换算（增益误差、偏置、延迟、饱和），不要给 `qfrc_actuator`。
- observer 的 `M`/`C`/`g`/`J` 要在**测量到的** `q, q̇` 上算，不是在 plant 真状态上算。
- 所有估计器（KF、naive、momentum observer）必须用同一条传感链，否则比较不公平。
- 文档 `:154` 已经提醒测量噪声会进入 `J`、`b`、`y_p` 并引入相关性；仿真里要显式检验
  这一点，而不是假设 `R` 对角。

### 5.4 plant 打开真实效应，observer 省略/扰动它们

- plant 打开 `damping`、`frictionloss`、`armature`、执行器动态、工具负载、接触约束。
- 典型失败模式：observer 用纯黏性摩擦，plant 有 Coulomb + Stribeck，估计出的 wrench
  会随速度方向出现“假力”。
- 注意 MuJoCo 的 `qfrc_passive` 是广义力（阻尼项为负），文档的 `τ_f` 是正阻力矩，符号
  要统一。

### 5.5 坐标系与参考点

- 统一 world frame、site 点、`[linear; angular]` 顺序。
- 工具坐标系偏移作为误差因子：1 cm × 20 N = 0.2 N·m，常常不可忽略。
- 如果 wrench 在 tool frame，随机游走先验在旋转坐标系里不再合适（文档 `:111` 已提醒），
  要单独处理。

### 5.6 C 约定、离散化与数值检查

- `mj_rne(q,v,0)` 给 `Cq̇+g`；`mj_fullM` 给 `M`。
- 用 `mj_rne(q,v,a) ?= mj_fullM @ a + mj_rne(q,v,0)` 做单元测试，确认 armature 是否被
  两边一致地包含。
- `Cᵀq̇ = 0.5 ∂(q̇ᵀ M q̇)/∂q`，不能把任意满足 `Cq̇` 的矩阵直接转置代入。
- 离散化方面，`A` 是幂零的，`exp(AT) = I + AT` 对冻结 `J` 精确；真正的误差来自步内
  `J`/`b` 变化和测量保持，不要把它当成主要误差源。
- 文档 `:186` 提醒连续噪声谱密度不能直接当离散协方差；用 NIS/NEES 检验尺度。

### 5.7 离线重放

- 记录 `qpos`/`qvel`/`qacc`/`qfrc_*` 和真值，后处理重放 observer。
- 比在线更可重复、可审计，也方便同一份数据跑多个 nominal model。
- 注意时间对齐：估计第 `k` 步只能用第 `k` 步及之前的测量；真值施加也按零阶保持对齐，
  否则频响会出现假的相位滞后。

## 6. 误差幅度的依据

误差幅度不能拍脑袋。现有材料包括：

- H-022 的 `theta.npy` 与 CAD 参数的差异；
- H-033 对完整惯性矩阵的约束及其不确定性（以实际 Run 产物为准）；
- 1 kHz `feedback` 的噪声谱和量化；
- 已知的工具质量/质心；
- 文献里的摩擦/执行器误差范围。

用这些来定扰动分布和噪声模型，比“±10%”有说服力。然后先冻结误差模型（分布、幅度、
seed schedule），再看结果。否则 “simulate model error” 很容易变成“调到故事好看”。

这正是 VALIDATE 的纪律：观察 endpoint 前固定 treatment、baseline、endpoint、population、
inferential unit、seed schedule。

## 7. 场景、指标与基线

### 7.1 场景

至少包括：

- 零外力（测假阳性/漂移）；
- 常值、阶跃、斜坡；
- 正弦扫频 0.1–10 Hz；
- 脉冲；
- 非末端接触；
- 负载突变。

### 7.2 指标

不要只报六轴 RMSE。至少包括：

- 每轴 bias、稳态 std；
- 90% 建立时间、超调；
- 频响幅值/相位；
- 零外力下的虚假 wrench；
- NIS/NEES；
- `σ_min(J)`/条件数；
- 误差协方差的最差方向（特征向量）。

### 7.3 基线

至少要有：

- naive `Jᵀ` residual inversion（`q̈` 差分 + 滤波）；
- momentum observer；
- batch least squares。

KF 只有在同一 mismatch 下优于这些才有意义。

### 7.4 其他纪律

- 调参轨迹和评估轨迹分开；`Q/R` 要报敏感性，不只报一个调好的值。
- 做 noise-only vs model-error-only vs combined 的分解——线性系统下近似可叠加，可以验证
  这个分解本身。

## 8. 最小 probe

如果只做一个最小 probe：

- E0：精确模型、无噪声；
- E1：同一模型 + 真实传感链；
- E2：标称模型（用 H-022 theta 或 CAD 参数，独立来源 + 5 个扰动样本）+ 真实传感链；
- 轨迹两条（准静态 + 动态激励）；
- wrench 四种（零、常值、阶跃、扫频）；
- 每条 5 个噪声种子；
- 指标只保留：每轴 bias/RMSE、零外力虚假 wrench、90% 建立时间、扫频幅值/相位、
  `σ_min(J)`。

这已经足以回答“完美模型仿真缺了什么”，而且成本可控。

## 9. 复现定位与 claim 边界

- 如果原论文在真机上验证，那么完美模型 MuJoCo 不是对其性能 claim 的复现，而是“算法
  重实现 + 新的仿真研究”；按 `reproductions/AGENTS.md` 记录 deviation。
- 如果原论文本身是仿真且没建模 mismatch，它的性能结论也有同样问题——这本身是一个可
  报告的发现。
- 复现的研究意义可以来自两处：
  1. 做原论文没做的 model-error sensitivity，回答“在什么误差下失效、哪个误差最致命”；
  2. 把 KF 作为项目方法的 baseline，建立一个固定 plant/sensor/trajectory/mismatch 协议
     的 benchmark。
- 前者是科学贡献，后者是基础设施贡献，但都要写清楚 claim 的边界。
- 仿真结果只能支持“在给定误差模型下”的条件性能，不能声称真实机器人性能。

## 10. 真机真值锚点

如果最终要声称真机性能，仿真只能缩小候选、给出预期和失效方向，不能替代真机。

最便宜的真值锚点是**已知负载**：

- 末端刚性固定一个已知质量/质心的负载，让机器人做无接触运动；
- 真值 wrench 就是负载重力对末端点的等效 wrench（`f = mg`，`m_e = r × f`；动态时还要
  加负载惯性 wrench）；
- 不需要力/力矩传感器，就能在真实数据上检验 KF 的偏差和方向依赖性。

项目已有的无接触轨迹（如果 `collect18/19/20` 是自由运动）也可以当“零外力”真值，用来
测真实数据上的虚假 wrench。这两个真值锚点可以反过来校准仿真里的误差幅度。

前提是 nominal model 不包含这个负载，或明确把负载当作外部 wrench；两种定义不能混。

## 11. 结论

问题真实，但不是“不要用 MuJoCo”的理由，而是“不要用完美模型 MuJoCo 做性能 claim”的
理由。

- 完美模型仿真保留为 unit test / regression test；
- 科学 claim 来自带 mismatch 的受控实验；
- 误差模型必须有独立依据，不能随手拍。

做到这三点，“给定动力学模型下用 KF 估计末端 wrench”的复现就有研究意义：它不仅复现了
递推，还回答了“模型误差如何变成 wrench 误差、哪些方向最脆弱、KF 相对更简单的方法在
什么条件下值得用”。

## 12. 与当前 TASK 的关系

本文是 THINK 讨论稿，不修改 H-033 的合同、进度或任何已接受状态。若要把这个仿真设计
落成行动，需要先确认研究方向，再单独准备一个 TASK；按项目规则，确认后只准备 TASK，
不在同一步执行。

## 13. 目录状态

`reproductions/KF/` 目前只有本设计稿和 `KF_Wrench_Estimation.md`，缺少
`reproductions/AGENTS.md` 要求的方法 `AGENTS.md`。若要把 KF 做成正式 reproduction，
需要补充：

- 论文题目、作者和稳定引用；
- 官方仓库或代码来源；
- 固定 commit、tag 或版本；
- 许可证；
- 本地修改和修改理由；
- 输入输出适配与单位、坐标系；
- 使用它的 Protocol 和 Experiment；
- 已知的复现差异。

## 参考

- `reproductions/KF/KF_Wrench_Estimation.md`
- `reproductions/AGENTS.md`
- `state/AGENTS.md`
- `docker/DEPENDENCIES.md`
