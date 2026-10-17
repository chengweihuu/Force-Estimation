# 科研工作流

## 第一原则

一次只推进一个研究问题。除非有助于回答它，否则不要扩大范围、修复无关问题或保留过程。
本模板面向个人研究者与单 Agent：任务内自主，研究含义、证据接受和阶段变化由用户确认。

科研生命周期只有三个阶段：`EXPLORE` 快速探索 hypothesis / mechanism，
`VALIDATE` 固定比较设计并做受控验证，`FORMAL` 在确实需要 claim-level evidence 时
才启用完整冻结和审计。总节奏是：`Explore fast. Validate cleanly. Formalize once.`
严格性按阶段启用，不默认覆盖整个科研生命周期。

## 权威地图与读取顺序

开始工作时按需读取：

1. `state/AGENTS.md`：准备、执行或关闭任务时读取；它是 TASK 生命周期的唯一权威；
2. `state/TASK.md`：若为 Active，它定义当前唯一行动、预算和权限；
3. `state/CURRENT.md`：当前接受状态、对象指针和下一项决策；
4. `state/PROJECT.md`：仅在需要长期目标、安全或项目边界时；
5. TASK 精确引用的 Idea、Protocol、Experiment 或代码；
6. 准备写入子目录时，该目录最近的 `AGENTS.md`。

不要扫描整个 `docs/`。README 只服务人类快速理解；除非用户明确要求解释目录或 README，
不要读取或把它当作任务上下文。子目录 AGENTS 负责局部准入。文件规则冲突时停止报告，
不自行拼接。

目录地图：研究知识见 `docs/AGENTS.md`；正式 Python 代码见 `src/AGENTS.md`；探索和外部实现
见 `scratch/AGENTS.md`、`reproductions/AGENTS.md`；环境见 `docker/AGENTS.md`。

## Git 前置关卡

Git 是覆盖更新、审计和回滚的安全网。修改项目文件前先确认 `git rev-parse --is-inside-work-tree`
成功；若模板尚未初始化，停止执行并向用户提出 `git init`、初始提交和默认分支方案。没有
有效 Git 基线不得开始 Active `VALIDATE` 或 `FORMAL`，也不得覆盖 `CURRENT.md` 或 Idea。

`VALIDATE` 若要修改或提交受控比较所需的配置/实现，从干净、已接受的基线创建
`task/<task-id>` 分支；`FORMAL` 只运行干净的冻结 commit。`EXPLORE` 默认使用可丢弃输出，
不要求 Formal-grade binding 或 hash。
科学任务关闭的内容审批、如有需要的提交和干净工作树要求见 `state/AGENTS.md`。

## THINK 与任务边界

没有 Active TASK 时，科学研究处于 THINK：可以对话、只读检查必要文件、比较方案并形成
待确认计划，但不得直接执行科学任务。用户确认研究方向后先只准备 TASK，不在同一步执行。
用户已授权的普通维护可以直接修改和验证，无需 TASK；边界见
[`state/AGENTS.md` 的普通维护规则](state/AGENTS.md#普通维护)。为具体科学问题跨对话保存探索
代码、计算或长篇分析时，把它纳入该 H-task 的 `EXPLORE`，不要建立额外 THINK 档案；
不得因 execution-only 内容需要持久化而单独创建 H-task。

Active TASK 只能处于 `EXPLORE`、`VALIDATE` 或 `FORMAL`。其精确形状、阶段合同、Progress、
Budget、分支和 Closeout 规则只在 `state/AGENTS.md` 定义。

只有 scientific hypothesis、treatment、endpoint、population、inferential unit 或 Frozen
scientific rule 发生实质变化时才创建新 H-task。阶段升级本身不创建新 H-task；
用户确认后在同一 H-task 内更新阶段和对应合同。CLI、schema、runner、carrier、heartbeat、
ID plumbing、logging、测试和小型 refactor 默认留在当前 task；若超出已批准的范围或预算，
先 Stop and Ask 以修订当前 TASK，不因执行层工作自动新建 H-task。没有 Active TASK 时，
普通维护按会话中用户已授权的范围执行，TASK 保持 Inactive，不单独创建 H-task。

以下情况必须停止并询问：

- 实质改变 scientific hypothesis、treatment、endpoint、population、inferential unit 或
  Frozen scientific rule；这类变化需要新 H-task；
- 阶段变化尚未获得用户确认；确认后保留同一 H-task identity；
- 超出 Allowed Changes 或 TASK 的文件、计算、时间、机器人预算，或需要新依赖、
  长期公共接口、权威实现、架构抽象、启发式、阈值或降级策略；若科学边界未变，
  有 Active TASK 时批准后修订当前 TASK；普通维护则修订会话中的授权范围，不新建 TASK；
- 将探索结果提升为正式代码或证据；
- Harness 缺口会威胁当前结果有效性，或修复它需要改变规则；
- 执行任何可能让真实机器人运动的命令。

新信息使任务失效时直接报告，不扩大任务挽救原计划。无关问题最多报告三个，不修复。

## 科学与执行分层

依赖只允许沿以下方向：

```text
Scientific Core → Generic Execution Contract → Experiment Binding → Carrier/Host
```

Scientific Core 只实现科学输入、输出和规则，不得依赖具体 Experiment ID、Protocol
version、Task/Run ID、foreground/detached、tmux/systemd 或 carrier/host 细节。Experiment identity
必须由 config/request 注入 Experiment Binding；禁止 core implementation 硬编码具体
Experiment ID，或根据具体实验名称分支科学逻辑。Carrier/Host 只处理运输、调度、生命周期和
状态呈现，不为 Scientific Core 注入隐式科学默认值。

runner、carrier、host、schema、CLI、logging、测试、ID plumbing 和小型 refactor 的变化不得
单独导致 Scientific Protocol 升版。若这些变化影响已冻结 FORMAL 运行的可复现性，应创建
新的 immutable request / Experiment Binding / Run，并保留旧绑定；只有科学规则改变才升级
Scientific Protocol。

## 资产与证据硬边界

- `EXPLORE` 默认只写 Git 忽略的 `scratch/<task-id>/`，可修改设计，不要求 Frozen
  Protocol、environment binding 或 artifact hash；其结果不能成为正式依赖或 claim-level evidence；
- `VALIDATE` 在观察 endpoint 前固定 treatment、baseline、endpoint、population、inferential unit
  和 seed schedule，保存可重放的 config/request 与代码修订；验证结果用于受控比较和
  是否进入 FORMAL 的决策，不得追溯性地标为 claim-level evidence；
- 正式代码按最近的目录 AGENTS 落地；`src/`、`ros2_ws/` 不得依赖 `scratch/`；
- 运行输出进入 `runs/<task-id>/<run-id>/`，程序必须接收明确 `output_dir`；
- Run ID 使用 UTC：`R-YYYYMMDDTHHMMSSZ-<short-id>`，例如 `R-20260816T073015Z-a7f2`；
- 只有 `FORMAL` 的 claim-level evidence 遵循
  `Claim → Frozen Protocol → immutable request → Run → Experiment → Idea Evidence Map`；
- `docs/RESEARCH_WIKI.md` 只作可检索的研究记忆入口：失败 Idea 和不利/无结论 Experiment
  必须保留到其权威对象的指针，但 Wiki 不复制结论或成为 Evidence；
- Protocol 规则见 `docs/protocols/AGENTS.md`，结果状态与研究者接受规则见
  `docs/ideas/AGENTS.md`；
- 不提交 scratch、runs、日志、bag、完整参数扫描或大型原始数据。

Closeout 中的 `Evidence` 可以是 diff、测试或受控比较等工程/验证依据；它不因字段名称
自动成为科学 Evidence。只有满足 FORMAL 链条和研究者接受的结果才能支持 Claim。

科学判断不得用 AI 自评替代。任何可能让真实机器人运动的命令，都要先报告准确命令、硬件、
限制、预期行为和停止方式并等待批准；失败后不自动重试。

## 实验经济学与外部审阅

实验计算使用 [`tools/research-workflow.mjs`](tools/research-workflow.mjs) 或 Pi 的
`research_workflow` 工具；请求字段与恢复方式见 [`docs/research_workflow.md`](docs/research_workflow.md)。
Agent 根据当前已批准设计生成请求，只列实际需要的步骤；普通维护不使用实验阶梯。
开始/恢复科学工作时检索相关失败记忆；关卡失败自动留下待审条目，不能自动否定 Idea。

- 每个阶段先做当前预算内最低成本、仍可判别的失败检查；前级失败即停止，不因
  “已投入”扩大。`EXPLORE` 只需声明当前 probe 的成本上限和停止条件，不需预绑定后续
  FORMAL 阶梯；`VALIDATE` 在观察结果前固定比较设计、失败判据和预算；`FORMAL` 只对已通过
  受控验证且需要 claim-level evidence 的主张执行冻结协议和 holdout。
- `ProtocolOutcome` 只能在 `FORMAL` 中由冻结的判定规则应用于可核验 Run 得到。LLM 可以翻译、
  摘要或提出建议，不能判定论据、生成 Evidence 或代替研究者接受。
- External Reviewer 仅可在显式最小上下文中做 `plain-language` 或 `scope-check` 建议；其
  provider/model 必须显式配置，单次注入调用、无工具、无读写权，输出固定为 advisory-only 且
  evidence-ineligible。不得自动写入权威对象或触发阶段升级。

## Harness Gardening

用 diff、测试、性质检查、校验值和审查发现偏差。一次性摩擦记入剩余不确定性。
若连续两次因同类 binding、schema、identity、carrier 或 interface 问题失败，停止逐个 patch；
在当前 task 中触发一次 execution-layer architecture review 或经批准的小范围 refactor，不再
做第三个局部补丁，也不因此自动创建新 H-task 或 Scientific Protocol version。修改 AGENTS 后
重启 Codex CLI/TUI，再用真实任务验证。

## 完成回复

日常解释使用清楚的中文、具体原因和必要的例子。普通维护只报告改动、验证结果和剩余问题，
无需科研 Closeout。科学任务关闭时只返回：`Result`、`Evidence`、`Remaining uncertainty`、
`Next scientific action`。
`Result` 必须分别声明 `Scientific information gain` 和 `Engineering information gain`。
若只产生工程进展，必须原文写：`No new scientific evidence was generated.`
若关闭需要用户批准，在这四项内附最小的精确批准内容，不增加一般性过程叙述。

## 环境、冻结与计算闸门

- `EXPLORE` 优先科研速度：只声明当前计算/硬件预算、停止条件和理解输出所需的
  最小环境信息；可修改设计，不要求依赖锁、image digest、主机指纹或 artifact hash。
- `VALIDATE` 优先受控比较：在观察 endpoint 前固定 treatment、baseline、endpoint、population、
  inferential unit、seed schedule、代码/配置修订、预算和停止规则，并记录足以重放的环境模式与依赖参照。
- `FORMAL` 才优先 provenance、immutability 和完整审计：必须绑定 Frozen Protocol、holdout、
  immutable request、环境合同、依赖锁、数据清单、代码 commit、Docker image digest 或主机指纹，
  以及必要的校验 hash；绑定不得事后覆盖。
- Scientific Protocol 只在 treatment、baseline、endpoint、population、inferential unit、判定/排除规则或
  其他科学语义变化时升版。执行层变化可要求新 binding/request/Run，但不升级 Protocol。
  旧文件、commit、运行和不利结果保留；不改变科学语义的纯排版或链接修正标记为
  `Editorial correction`。
- 计算规模在当前阶段内按需由最小 probe 升到 pilot / scale；只为实际计划的级别写通过条件、
  预算和停止条件。`FORMAL` 是证据阶段，不是所有计算都必须到达的默认末级。

## Agent context contract

- 持久状态只保存结构化事实、指针和决策；原始工具输出和长日志写入任务输出目录，
  由短记录引用路径和摘要。只有 `FORMAL` 绑定或完整性确有需要时强制 hash；
  `EXPLORE` 不因持久状态要求而承担 Formal-grade hashing。
- 检索采用渐进披露：先读控制面和索引，再按对象/章节加载最小片段，只有明确需要时才读原始产物。
- 子 agent 默认不调用。委派必须发送 bounded Delegation Packet，只接收 bounded Result Packet；详细过程留在子任务输出中。
- 格式见 [`state/CONTEXT.md`](state/CONTEXT.md)。若运行平台不支持外部状态、独立 context 或 agent 隔离，必须报告能力缺口。
