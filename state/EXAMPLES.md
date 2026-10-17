# 最小完整示例

本文件只在准备或审查 TASK、Closeout、Experiment、`run.yaml` 时读取。以下 ID 和内容都是
示例，不代表真实项目状态，也不要把示例对象预建到项目中。

## 1. Inactive TASK

```markdown
# Current Task

Status: Inactive

下一步：根据 CURRENT 进入 THINK；用户确认计划后再替换本文件。
```

## 2. EXPLORE 阶段的 TASK

```markdown
# Current Task

Status: Active

## Identity

- Task ID: H-101
- Stage: EXPLORE
- Progress: Ready — waiting to run the first counterexample sweep.
- Research objects: I-001:C-01

## Question

在曲率接近零时，候选误差公式是否仍保持符号连续？

## Context

- Idea: `docs/ideas/I-001_path_error.md#c-01`

## Allowed Changes

- `scratch/H-101/`
- `state/TASK.md`（仅覆盖 Progress）

## Non-goals

- 不修改正式实现；不形成论文证据。

## Budget

- Files: `scratch/H-101/` 内最多 3 个文件
- Time / Compute: CPU 10 分钟；不使用网络或 GPU
- Robot runs: 0

## Done When

- 给出零曲率两侧各一个数值例子，并说明是否存在符号跳变。

## Stop and Ask

- 需要改变 C-01 的定义或引入新的阈值。

## Outputs

- `scratch/H-101/summary.md`
- `scratch/H-101/check.py`

## Information Gain

- Scientific: Pending.
- Engineering: None expected.
```

## 3. 同一 H-task 进入 VALIDATE

```markdown
# Current Task

Status: Active

## Identity

- Task ID: H-101
- Stage: VALIDATE
- Progress: Ready — comparison fixed; branch `task/H-101` created from accepted commit `abc1234`.
- Research objects: I-001:C-01

## Question

在固定样本、seed 和 endpoint 下，候选有符号横向误差实现是否比旧实现更连续？

## Context

- Idea: `docs/ideas/I-001_path_error.md#c-01`
- Accepted baseline: `abc1234`

## Allowed Changes

- `src/path_error.py`
- `tests/test_path_error.py`
- `state/TASK.md`（仅覆盖 Progress）

## Non-goals

- 不加入 ROS 适配、阈值或降级策略；不把 VALIDATE 结果标为 claim-level evidence。

## Budget

- Files: 2 个正式文件
- Time / Compute: CPU 测试 2 分钟
- Robot runs: 0

## Done When

- 固定 seed 下完成候选/基线受控比较；单位、符号和零曲率边界测试通过。

## Stop and Ask

- 理论没有定义退化切向量时的行为。

## Outputs

- `src/path_error.py`
- `tests/test_path_error.py`

## Validation Contract

| Hypothesis / Claim | Treatment | Baseline | Endpoint | Population / Inferential Unit | Seed Schedule | Code / Config Revision | Failure / Stop Rule |
|---|---|---|---|---|---|---|---|
| I-001:C-01 | candidate implementation | accepted implementation at `abc1234` | zero-curvature sign discontinuities | frozen 20-case set / case | 11, 23, 47 | candidate `fed1234`; baseline `abc1234`; no external config | any unit/sign failure stops |

## Execution Contract

| Requirement | Owner / Interface | Inputs / Outputs / Scientific Invariants | Verification |
|---|---|---|---|
| Signed lateral error | `src/path_error.py::signed_lateral_error` | meters; right-handed coordinates; zero-curvature continuity | unit, boundary, property tests |

## Information Gain

- Scientific: Pending controlled-comparison result.
- Engineering: Pending implementation and interface verification.
```

## 4. 同一 H-task 进入 FORMAL

```markdown
# Current Task

Status: Active

## Identity

- Task ID: H-101
- Stage: FORMAL
- Progress: Ready — protocol and commit frozen; no run started.
- Research objects: I-001:C-01, P-001, E-001

## Question

冻结数据集上的横向误差是否满足 P-001 的精度判据？

## Context

- Idea: `docs/ideas/I-001_path_error.md#c-01`
- Protocol: `docs/protocols/P-001_accuracy.md` (`Frozen`)
- Frozen commit: `def5678`

## Allowed Changes

- `runs/H-101/`
- `experiments/E-001/`
- `state/TASK.md`（仅覆盖 Progress）

## Non-goals

- 不改实现、协议、指标、排除条件或数据范围。

## Budget

- Files: 1 个 Experiment 摘要；runs 不计入提交文件
- Time / Compute: CPU 2 小时；不使用网络
- Robot runs: 0

## Done When

- 所有冻结样本运行一次；run.yaml 和校验值完整；E-001 记录所有结果。

## Stop and Ask

- 冻结数据不可用、实现错误或需要排除新样本。

## Outputs

- `runs/H-101/R-20260816T073015Z-a7f2/`
- `experiments/E-001/README.md`

## Experiment Contract

| Claim | Frozen Protocol | Holdout / Data | Metric / Decision Rule | Immutable Request / Environment Binding | Evidence Location |
|---|---|---|---|---|---|
| I-001:C-01 | P-001 | frozen holdout manifest | MAE ≤ 0.05 m 且无预定义失败样本 | `request-R-001.yaml` / frozen commit `def5678` | `experiments/E-001/` |

## Information Gain

- Scientific: Pending FORMAL outcome and researcher acceptance.
- Engineering: None expected.
```

## 5. 默认内容级 Closeout

### Result

- Scientific information gain: P-001 的冻结判定结果为 Supported；研究者接受仍为 Pending。
- Engineering information gain: 参考实现的单位、边界和性质测试共 12 项通过。

### Evidence

- Frozen Protocol: P-001；immutable request: `request-R-001.yaml`；Run: `R-20260816T073015Z-a7f2`。
- 拟更新 `docs/ideas/I-001_path_error.md` 为 Patch A，`state/CURRENT.md` 为 Patch B。
- 拟接受代码文件：`src/path_error.py`、`tests/test_path_error.py`；决定性 diff 为新增纯函数、单位/符号/零曲率测试。
- 拟删除的准确路径：`scratch/H-101/`。
- 拟提交文件：上述 Idea、CURRENT、代码、测试和 `state/TASK.md`；commit message:
  `research: accept signed lateral error evidence`。

Patch A：Idea 的拟批准内容：

```diff
 ## Current Conclusion
-- Untested.
+- Accepted: 在 P-001 定义域内，公式满足横向误差精度判据。
+- Scope: 不覆盖退化切向量；理由和限制见 E-001。
```

Patch B：CURRENT 的拟批准内容：

```diff
 ## 一句话接受状态
-- I-001:C-01 尚未验证。
+- I-001:C-01 已在 P-001 定义域内接受；完整结论见 Idea。

 ## 下一项需要决定的事情
-- 是否执行 P-001。
+- 是否为退化切向量建立新 Claim。
```

### Remaining uncertainty

- 退化切向量不在 P-001 范围内。

### Next scientific action

- 研究者决定是否接受 Patch A/B、精确文件集、删除路径和 commit message；实际 diff 如有实质差异必须重新请求批准。

## 6. Experiment 记录

```markdown
# E-001 横向误差精度

- ExperimentID: E-001
- Task: H-101
- Claim: I-001:C-01
- Protocol: P-001
- FrozenCommit: def5678
- ExperimentStatus: Completed
- ProtocolOutcome: Supported
- ResearcherVerification: Pending
- ResearcherNote: Pending review.

## Runs

- `R-20260816T073015Z-a7f2`

## Result

冻结判定规则得到 Supported。该值是协议结果，不代表研究者已经接受科学结论。

## Deviations

None.
```

研究者确认 Closeout 后，将 `ResearcherVerification` 改为 `Accepted` 或 `Disputed`，并写短
`ResearcherNote`。若 `ExperimentStatus: Invalid`，则 `ProtocolOutcome` 必须是
`NotEvaluated`，并在 Deviations 中说明原因。

## 7. FORMAL run.yaml

```yaml
run_id: R-20260816T073015Z-a7f2
task_id: H-101
experiment_id: E-001
protocol_id: P-001
claim_ids:
  - I-001:C-01
frozen_commit: def5678
command:
  - python3
  - tools/evaluate.py
  - --config
  - config/formal.yaml
output_dir: runs/H-101/R-20260816T073015Z-a7f2
environment:
  os: ubuntu-24.04
  python: 3.12.4
config: config/formal.yaml
data:
  uri: s3://example-research/path-error/v1/data.parquet
  sha256: 0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef
started_at_utc: 2026-08-16T07:30:15Z
```

真实记录还应按 Protocol 补齐容器、ROS、硬件或随机种子信息；不要把密钥写入 run.yaml。
