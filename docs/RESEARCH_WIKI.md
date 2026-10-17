# Research Wiki

此文件是研究记忆的可检索入口，不是 Idea、Protocol、Experiment 或 Evidence 的权威来源。
它帮助研究者把不可避免的弯路保留为更便宜、可复用的下一次检查，而不是删除失败或重复同一条路。

## 使用规则

- 每个 `Rejected` Idea、`Refuted` Experiment 或 `Inconclusive` Experiment 都保留其权威对象；
  在下表增加或更新一条短索引。
- 每行必须指向权威 Idea/Experiment/Protocol，并列出证据指针（如已有）、适用边界、可复用教训
  与重启条件。没有正式 Evidence 时明确写“无”；不可用普通 LLM transcript 代替。
- Wiki 不复制科学结论、指标、原始输出或完整失败分析；这些内容仍只属于链接的权威对象。
- 使用状态、边界或重启条件检索；重新启动时先检查既有失败记忆能否用一个更低成本、可判别的
  阶段排除同一问题。

实际入口：`node tools/research-workflow.mjs wiki query <关键词>` 检索，`wiki audit` 检查
权威失败对象的遗漏和失效指针。执行失败先进入 Run 的 `failure-memory.pending.json`；
权威对象经审阅后用 `wiki propose` 生成精确索引 diff，获用户接受后 `wiki apply --accept`。
Pi 模型工具不能直接接受索引。完整格式见 [实验与记忆接口](research_workflow.md)。

## Failure-memory index

| Memory ID | Status | Authoritative object | Evidence pointer | Applicable boundary | Reusable lesson | Restart condition |
|---|---|---|---|---|---|---|
| _Add after an authority object exists_ | `Rejected` / `Refuted` / `Inconclusive` | Idea / Experiment / Protocol link | Evidence link or `None` | Conditions where the result applies | Smallest next discriminating check | What new condition justifies reopening |
