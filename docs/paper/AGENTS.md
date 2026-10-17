# Paper 目录准入

## 用途

这里承载论文文本、图表入口和投稿材料。`drafts/` 允许保存已初步
成稿但尚未完成实验验证的预证据论文稿，使其可以受 Git 管理和持续编辑。

论文稿只是 Idea、Protocol 和 Experiment 的叙述性投影，不是 Claim、科学结论或
证据的权威来源。论文措辞与权威对象冲突时，以 Idea、Protocol 和 Experiment 为准，
不得用修改论文稿的方式改变研究状态。

## `drafts/` 准入

每份受管理的论文稿放在 `docs/paper/drafts/<manuscript-name>/` 中。论文主文件的开头
或同目录的稿件索引必须显式记录：

```text
ManuscriptStatus: PreEvidence | EvidenceAligned | SubmissionCandidate
EvidenceStatus: Unvalidated | Aligned
IdeaClaims: [I-NNN:C-NN, ...]
ExperimentEvidence: [E-NNN, ...] | None
UnvalidatedSections: [section locators, ...] | None
```

- `IdeaClaims` 只列论文实际陈述的 Claim；不能用论文稿新建或改写 Claim。
- `ExperimentEvidence` 只列已存在的正式 Experiment；Run、scratch 产物和预期数值不是论文证据。
- 引言、相关工作、方法描述、理论推导、预期贡献和待验证图表可在 `PreEvidence`
  阶段编写，但必须保持来源和证据状态可辨识。
- 未验证的结果性陈述、数值、图表和贡献必须在正文中就近标记为 `Expected`、
  `Hypothesis` 或 `Placeholder`，并在 `UnvalidatedSections` 中给出可定位入口。

## 稿件状态

### `PreEvidence`

初步成稿即可进入，不要求已有正式 Experiment。此时必须为
`EvidenceStatus: Unvalidated`；`ExperimentEvidence` 可为 `None`，但所有未验证部分都必须
显式标记和定位。完稿程度不会使其自动晋级。

### `EvidenceAligned`

只有同时满足以下可检查条件才能进入：

1. 每个科学主张都指向存在的 Idea Claim；
2. 每个实验事实、数值、结果图表和结果性陈述都指向正式 Experiment；
3. 稿件对 `Supported`、`Refuted`、`Inconclusive`、`Invalid`、`Pending` 或 `Disputed`
   的表述与权威记录一致，不隐藏不利、无结论或失效结果；
4. `UnvalidatedSections: None`，且没有作为结果或结论使用的预期值或占位图表。

满足后记录 `ManuscriptStatus: EvidenceAligned` 和 `EvidenceStatus: Aligned`。该状态只表示
稿件已与当前权威证据对齐，不改变任何 Experiment 的 `ResearcherVerification`。

### `SubmissionCandidate`

只有稿件已是 `EvidenceAligned`，且研究者在内容级 Closeout 中批准准备投稿的
准确文本、图表入口和证据指针后，才能记录为 `SubmissionCandidate`。此时
`EvidenceStatus` 仍为 `Aligned`。该状态不表示论文已接收、结论永久正确或未来证据
不能推翻当前结论。

## 准入

- 只放当前论文直接使用的文本、表格定义、图表入口和投稿材料。
- 方法主张引用 Idea 或 Protocol；作为已观察事实的图表和数字必须引用正式 Experiment。
- 稿件状态或证据指针改变时，必须在对应 TASK 的内容级 Closeout 中给出拟议值或
  可读 diff；不根据写作完成度自动晋级。

## 禁止

- 不放研究讨论、临时图片、原始实验结果或 Codex 过程总结。
- 不在本目录产生新的实验结论；结论先写入 Experiment。
- 不把 `PreEvidence` 稿件、scratch 输出、占位图表或预期数值引用为正式证据。
- 不因为论文叙述需要而改写冻结 Protocol、删除不利结果或把研究者尚未确认的
  解释写成已接受结论。

## 生命周期

论文稿从 `PreEvidence` 开始，只在满足上述证据条件后进入 `EvidenceAligned`，
再经研究者内容级批准进入 `SubmissionCandidate`。新证据使引用失效或出现未验证
部分时，必须降回 `PreEvidence` 和 `Unvalidated`，不通过补写论文掩盖差异。

投稿历史由 Git 保存，不创建“最终版”、“最终版-v2”或日期副本。文件重命名、稿件移动和
清理必须在 Closeout 的 `Evidence` / `Next scientific action` 中给出每个准确路径、操作和 diff 摘要，
并在执行前获得用户内容级批准。
