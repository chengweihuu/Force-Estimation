# 实验、失败记忆与主线恢复

实现入口是 [`tools/research-workflow.mjs`](../tools/research-workflow.mjs)。不需要新模型或新依赖；
Node 24 可运行 CLI，Pi 使用项目已有的依赖。普通维护继续直接完成，不创建 H-task 或激活 TASK。
下面的实验入口只用于已批准的 Active 科学 TASK。

| 需要的行为 | 实际机制 |
|---|---|
| 便宜地发现失败，再支付更高成本 | 按预声明成本递增执行；每步预留成本和时间；通过固定判据才允许下一步 |
| 失败成为下一次的记忆 | 失败自动留下待审条目；权威对象确定后生成 Wiki 行的精确 diff；用户接受后可检索；audit 检查遗漏 |
| 主线不会被支线替换 | TASK 问题、范围、预算、请求和输入的快照校验；只接受下一项已批准动作；支线暂存；每次 Pi 发起对话重新恢复上下文 |

## 1. 准备当前实验

仍按 `state/AGENTS.md` 批准科学问题、阶段、范围和预算。只有需要启动计算时才准备一个 JSON
执行请求；它留在当前 H-task，不另建任务。用户审批 TASK 时也审批其引用的请求，尤其是实际
argv、判据和预算。程序不会把文件存在视为用户已批准科学设计。
这些字段由 Agent 根据研究意图和已批准设计填写，并给出可审阅的请求；不要求研究者手填 JSON。

在 TASK 的 `Context` 增加一个精确指针，并在 `Outputs` 列出本次目录：

```markdown
## Context

- Idea: `docs/ideas/I-001_path_error.md#c-01`
- Execution request: `scratch/H-101/request.json`

## Outputs

- `runs/H-101/R-20260905T000000Z-demo/`
```

这只是待批准 TASK 中两个章节的示例，不是可替代整个 TASK 的文件。
`Budget` 的文字约束也必须覆盖请求预算。只有当前 probe 时，`steps` 只有一项，
不要求预填未来 pilot、scale 或 FORMAL。真正批准了多级计算，才添加后续步骤。

一个最小 EXPLORE 请求如下。ID、路径、程序和判据均为示例，不能直接作为实际研究设计：

```json
{
  "version": 1,
  "taskId": "H-101",
  "stage": "EXPLORE",
  "hardware": "local-compute",
  "environment": "Python 3.12; CPU; standard library only",
  "outputDir": "runs/H-101/R-20260905T000000Z-demo",
  "budget": { "maxCostUnits": 1, "maxWallTimeMs": 10000 },
  "inputs": ["scratch/H-101/probe.py"],
  "steps": [
    {
      "id": "probe",
      "relation": "direct",
      "purpose": "检查当前问题在零曲率两侧是否存在符号跳变",
      "costUnits": 1,
      "timeoutMs": 10000,
      "argv": ["python3", "scratch/H-101/probe.py", "--output-dir", "{output_dir}"],
      "resultFile": "metrics.json",
      "pass": [{ "field": "sign_discontinuities", "op": "eq", "value": 0 }]
    }
  ]
}
```

程序接收 `{output_dir}` 替换后的绝对路径，写 `metrics.json`，例如
`{"sign_discontinuities": 0}`。科学计算只接收其输入和输出目录，不需要知道 TASK ID 或 Pi。
也会提供 `HARNESS_OUTPUT_DIR` 环境变量，但 argv 中仍必须显式传递目录。

每个后续 step 都写明它如何直接回答问题（`direct`）或为回答问题提供必要条件
（`prerequisite`）。`costUnits` 必须不递减；预算按步骤声明值累计预留，不因失败退还。
步骤 ID 不得使用运行元数据的目录名 `inputs`、`review`，避免与输入快照或手动审阅包冲突。
这些是用户定义的计算成本单位，不是测得的美元、token 或 GPU 账单。墙钟预算累计命令耗时，
不包含人类暂停思考的时间；剩余额度不足以容纳下一步的 timeout 时，不启动该步骤。
每步 timeout 不超过 Node 定时器上限 2,147,483,647 ms；停止时终止整个本地进程组。

`pass` 的全部规则都须通过。支持 `eq/lt/lte/gt/gte`、点分字段路径和严格类型比较。
缺失字段、错误类型、无效 JSON、非零退出、超时或取消都会停止阶梯；`metrics.json` 上限 1 MiB，
原始数据写其他产物。规则来自已批准设计，不能看完结果后修改来放行。
`inputs` 列出会影响当前计算的准确代码、配置和输入文件；启动时保存副本，继续前逐字节检查。
大数据优先列准确清单，数据内容的完整性按阶段合同验证，不无目的复制整个数据目录。

```bash
node tools/research-workflow.mjs flow start
node tools/research-workflow.mjs flow next probe
node tools/research-workflow.mjs context
```

`start` 只做前置检查和快照，不启动实验。`next` 执行一项，`resume` 从当前检查点执行下一项；
`run` 连续执行请求内已批准的步骤，遇到任何停止即退出。失败/预算停止的 CLI 退出码为 2，
参数或状态错误为 1。不能跳步、复用 Run ID、自动重试或自动升级阶段。

输出目录保存 `request.snapshot.json`、`inputs/`、`workflow-state.json` 和每步的 stdout、stderr、
结果与执行记录。TASK 只原地更新单行 `Progress` 指针。运行输出不提交 Git。

### 阶段差异

- **EXPLORE**：最小环境说明、当前 probe、预算和停止规则；输入副本不要求 hash。
  修改设计后使用新 Run；是否需要新 H-task 仍按科学边界判断。
- **VALIDATE**：完整 Validation Contract，观察 endpoint 前固定设计。
  请求增加 `replay: {"codeRevision": "完整 commit", "dependencies": "依赖参照", "implementationChanged": true}`。
  当前代码/config 必须是可重放的干净修订，TASK Progress 可变化；若更改受控实现/config，
  `implementationChanged: true` 要求 `task/<task-id>` 分支；只读运行已接受实现可为 false。
  不强制 FORMAL hash。
- **FORMAL**：用户先确认阶段升级和证据需要。请求必须绑定同一任务已完成的 VALIDATE 记录；
  `formal.validationRun` 指向它的 `workflow-state.json`。科学实现从独立、干净的冻结 checkout
  运行，`formal.codeRoot` 是它相对项目根的路径，避免 TASK Progress 使冻结代码变脏。
  请求增加 `replay` 与以下 `formal` 字段，相关引用均为项目根的相对路径：

```json
{
  "codeRoot": "scratch/H-101/frozen-code",
  "codeCommit": "冻结实现的完整 commit",
  "validationRun": "runs/H-101/R-20260905T000000Z-validation/workflow-state.json",
  "protocol": "docs/protocols/P-001_accuracy.md",
  "holdout": "data/holdout.json",
  "dependencyLock": "requirements.lock",
  "dataManifest": "data/manifest.json",
  "environmentContract": "docker/ENVIRONMENT_CONTRACT.md",
  "hostFingerprint": "host-fingerprint 命令的输出",
  "sha256": { "每个 inputs 路径": "对应文件的 SHA-256" }
}
```

FORMAL 的上述 validation/protocol/holdout/lock/manifest/environment 指针必须都列入 `inputs`，
所有 inputs 都校验 hash。Frozen commit 必须与 Context 及 checkout 一致，Protocol 必须 Frozen，
Context 必须指向该 Protocol，Experiment Contract 必须包含相同协议 ID 和实际 request 路径；
运行前后检查冻结 checkout 与输入；输出结果和日志记录 hash。本机指纹覆盖 OS、架构、内核、
主机名和 Node 版本；完整 Python/CUDA/数据环境仍由引用的环境合同、锁和清单定义与核验。
此 CLI 实现本地 host 模式；容器 digest、集群计费和机器人要走各自批准的执行载体。

这些检查不会判定实验设计是否科学有效，也不会自动产生 `ProtocolOutcome`、
`ResearcherVerification` 或 Idea Evidence Map。工作流检查点始终 `evidenceEligible: false`；
FORMAL 原始产物能否支持 Claim 仍需满足已有完整证据链和研究者接受。

## 2. 失败记忆进入下一次工作

失败、预算停止或中断自动生成 `failure-memory.pending.json`。它记录观察和指针，不把命令失败
改写成 Idea 被否定。先审阅权威 Idea/Experiment；只有其已有 `Rejected`、`Refuted` 或
`Inconclusive` 状态，才准备长期 Wiki 索引。探索阶段可以指向 Rejected Idea，不必创建 FORMAL。

准备一个条目 JSON，例如：

```json
{
  "id": "M-001",
  "status": "Rejected",
  "authority": "docs/ideas/I-001_path_error.md",
  "evidence": "None",
  "boundary": "零曲率附近；只适用于该 Idea 所列定义",
  "lesson": "先做两侧符号连续性检查",
  "restart": "定义或适用边界出现新依据"
}
```

```bash
node tools/research-workflow.mjs wiki propose scratch/H-101/entry.json scratch/H-101/wiki-proposal.json
node tools/research-workflow.mjs wiki apply scratch/H-101/wiki-proposal.json --accept
node tools/research-workflow.mjs wiki query 零曲率
node tools/research-workflow.mjs wiki audit
```

先向用户展示 `propose` 返回的精确 diff，获得接受后才调用 `apply --accept`；该参数表达操作者
持有的授权，不是身份验证系统。Pi 的模型工具只有 propose，没有 apply；人类使用
`/research-memory-accept scratch/H-101/wiki-proposal.json`。Wiki 或权威对象在审阅后改变会拒绝应用。
只能改一行索引，不能夹带科学结论。索引里无正式 Evidence 时写 None；证据指针只接受已完成、
研究者接受的 Protocol Experiment，原始日志或模型 transcript 不算 Evidence。

`audit` 检查全部 Idea 和 Experiment 的失败状态及 Wiki 指针，报告缺漏；不扫描整个 docs。
有争议的 Refuted/Inconclusive 结果仍须索引，不能因研究者不认同解释而消失。
查询按词、对象 ID 和中文相邻字检索，返回命中总数与省略数量；它不是语义裁判。
关闭失败任务前执行 audit，补齐获批索引。失败仍待解释时保留 pending 指针，不伪造结论。

## 3. 恢复主线并处理支线

```bash
node tools/research-workflow.mjs context
node tools/research-workflow.mjs flow park "以后再考虑重写可视化界面"
node tools/research-workflow.mjs flow resume
```

`context` 从 TASK 和检查点恢复原问题、研究对象、范围、预算、停止条件、上次结果、下一步、
待审失败记忆与相关 Wiki 行。跨会话、进程重启和压缩后使用同一入口，不依赖 executor 回忆。
Pi 的 `before_agent_start` 每次读取磁盘并注入恢复包；超过 12,000 字符会明确要求读取指定对象，
不会静默丢弃约束。完整请求、支线内容和日志只按指针读取。

未完成的 Run 独占主线。`next` 的 step ID 必须等于下一项已批准步骤；任意改写问题、范围、
预算、请求、代码/config 输入都会阻止继续。Pi 同时保护运行输出和尚在执行的请求/输入，
禁止直接用 write/edit 改写它们。另一个终端的普通文件修改会被下次校验发现。
要修订执行方案，先 `flow stop "准确原因"`，按既有权限审阅后使用新 Run；不自动创建 H-task。

正常暂停在步骤之间，可用 resume。执行期间取消会保留已预留预算并停止；进程崩溃时保留
running 状态和锁，`flow recover` 仅在宿主进程和命令进程组都不存在后标记 interrupted，
不重新执行；若崩溃发生在结算后，只清除死进程留下的锁并保留已结算状态。
遇到仍活跃或属于其他主机的进程，先由宿主检查和停止。

这能防止已声明动作被替换、丢失状态或无依据扩大计算；不能仅凭文本证明某个科学前提正确，
也不能消除模型对“这条支线是否必要”的所有误判。新语义、新阈值、科学边界和阶段变化仍交由
研究者决定。CLI 运行的是已批准的本地程序，`shell: false` 不等于 OS 沙箱；文件、网络、硬件
权限须由实际宿主限制，禁止把任意命令伪装为 local-compute 绕过机器人授权。

## 4. 按需交给网页 AI 审阅

你可以在需要时说“生成一份当前方案的审阅包”。Codex 从现有 Active 科学 TASK 和明确引用
的材料导出 Markdown，并直接给出文件链接。不会自动上传、调用外部模型，也不会因为没审阅
而阻止正常实验。未定义科学任务时不编造主线，不为了使用这个工具新建 H-task 或 Run；
普通维护仍按会话授权处理。

文件位置固定：

```text
scratch/<task-id>/review/01/                  # 运行前
runs/<task-id>/<run-id>/review/01/            # 已有当前 Run
  REVIEW_PACKET.md                          # 交给网页 AI 的文件
  REVIEW_FEEDBACK.md                        # 保留网页 AI 的完整原文
  review.json                               # 程序使用的版本与登记信息，不需要交给网页 AI
```

每次生成使用新的 `01、02…` 目录，保留旧版。CLI 返回两个 Markdown 的相对路径和可点击的
绝对文件链接。`review list` 列出当前任务在运行前及各 Run 中的审阅包，并给出最新入口。

```bash
node tools/research-workflow.mjs review export
node tools/research-workflow.mjs review list
node tools/research-workflow.mjs review check scratch/H-101/review/01
```

`REVIEW_PACKET.md` 开头先说明问题、行动、边界和希望得到的意见，随后附编号原文。
表述尽量控制在一页主线说明，长约束引用后面的完整章节。它直接提取 TASK、CURRENT、
Context 引用、执行请求、请求引用的当前输入、运行中的全部已登记结果和失败记录；
已有 Run 时另附启动时输入副本，当前版本不同则明确提醒。
当前语义与执行快照不同、运行还没结束时会明确提醒。没有用户原话时，不把 TASK 问题冒充
原话；补充的 executor 解释单独标记为未经独立核实。

缺失、非文本或超过篇幅的材料明确列为“未附”，不会偷偷截断。单个源文件最多读取 1 MiB，
每份正文节选上限 16,000 字符，全部来源正文累计上限 60,000 字符；正文较大的控制/结果记录
不能通过手选几行来隐藏，必要时缩小本轮审阅对象。外部网页不会自动抓取；论文/PDF 中需要的
定义和限制应准备准确节选或明确附件。`ready: true` 只表示所有列明材料已附，不代表引用完整
覆盖了科学问题。缺材料仍会生成可读包，但 `export/check` 返回退出码 2 提示先核对缺口。

如果需要用户原话、明确关注点或特定参考章节，Codex 可以准备一个可选 brief 文件：

```json
{
  "mode": "scope-check",
  "userGoal": "用户明确提供的原始目标，不擅自改写",
  "executorNote": "本轮动作怎样服务当前问题；这段只是执行者的解释",
  "questions": ["本轮检查是否覆盖了所声明的适用边界？"],
  "sources": [
    {
      "path": "docs/ideas/I-001_path_error.md",
      "reason": "本轮采用的定义和限制条件",
      "startLine": 12,
      "endLine": 35
    }
  ]
}
```

```bash
node tools/research-workflow.mjs review export scratch/H-101/review-brief.json
```

brief 由 Agent 根据用户意图填写，不要求研究者手填。省略行号就附全文；节选必须显式标明
位置和使用理由，不能覆盖或删减必须提供的控制状态和结果。`plain-language` 模式用于解释
定义、规则与限制；`scope-check` 用于主线和引用范围建议，两者都不产生科学裁决或 Evidence。

发送前执行 `review check`。工具对交接源文件和原文使用 SHA-256 检查版本与意外改写；这些
hash 只服务手动交接的完整性，不提升 EXPLORE/VALIDATE 的证据等级，也不改变实验闸门。
任务、输入、结果或代码基线变化后，旧包标记为 stale；重新导出，保留旧包。

你把 `REVIEW_PACKET.md` 上传或粘贴给网页 AI 即可；审阅要求和回答格式已经在文件里。
包内要求先复述主线，再用 `### R1`、`### R2` 等编号给出“引用材料 → 疑点 → 如何检查”。
关闭不需要的工具/连接器，reviewer 只处理明确交付的上下文。

网页反馈可以完整粘贴进 `REVIEW_FEEDBACK.md` 后登记：

```bash
node tools/research-workflow.mjs review feedback scratch/H-101/review/01 "ChatGPT 网页" "网页未显示"
```

也可以把原文另存成文件，再让程序逐字节导入：

```bash
node tools/research-workflow.mjs review feedback scratch/H-101/review/01 "DeepSeek 网页" "网页实际显示的型号" scratch/H-101/raw-feedback.md
```

provider/model 必须显式记录，型号不可猜测。登记保留 UTF-8 原文的换行和全部内容，不套用
注入式 API reviewer 的 1600 字符上限。已经粘贴的不同文本不能被覆盖，已登记反馈不能再次
替换；新的反馈使用下一轮目录。即使任务或材料后来变了，仍能登记针对旧包的原文，同时报告
stale。没有标准编号时原文照样保留，Codex 用原话和段落定位逐条回应。

Codex 的解释、修复与不同意理由写在对话或单独的响应文件中，不写回反馈原文。外部意见不能
直接更改科学状态、放行实验或升级阶段。保留网页会话作为你直接可见的原始来源：本地登记
核验的是你提供的文本，不能证明其真实作者；同目录 hash 也不构成针对任意写权限的防篡改隔离。

## Pi 启用和验证

Pi 通常发现项目 `.pi/extensions/`。显式加载时同时传入：

```bash
pi -e .pi/extensions/research-gates.ts -e .pi/extensions/research-workflow.ts
```

新工具 `research_workflow` 提供 start、next、status、park、memory、propose_memory。
Active TASK 的任意 bash 仍被拦截，实验通过此受控入口执行。Codex 使用相同 CLI 和
`state/CONTEXT.md` 恢复约定；它不会自动执行 Pi hook。

```bash
node --test .pi/harness/research-workflow.test.mjs .pi/harness/research-gates.test.mjs .pi/harness/manual-review.test.mjs
python3 -B tools/validate_harness.py --self-test
```

测试使用临时 Git 仓库、实际本地子进程、跨进程 CLI 和已安装 Pi 的实际扩展加载器，
不调用模型、网络或机器人。修改 AGENTS 后须重启 Codex/Pi，再用实际获批研究问题验证行为；
自动测试不能代替这一步，也没有产生新的科学证据。
