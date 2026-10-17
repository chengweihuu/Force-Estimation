# Docs 目录准入

## 用途

这里保存长期研究知识，不保存 Codex 当前任务状态和执行流水账。

| 目录 | 允许内容 | 禁止内容 |
|---|---|---|
| `ideas/` | 想法、假设、理论推导、当前结论 | 每轮讨论的副本 |
| `protocols/` | 冻结验证协议、数据合同 | 临时实验计划 |
| `references/` | 实际使用的资料索引和阅读结论 | 无目的的资料堆积 |
| `paper/` | 论文直接使用的文本和材料 | 普通研究笔记 |
| `RESEARCH_WIKI.md` | 指向 Idea、Protocol、Experiment 的可检索研究记忆索引 | 第二份科学结论、Evidence 或运行日志 |

资料、环境和协议的稳定 ID 是跨对象检索入口；正文只加载当前任务需要的章节，不把完整目录或运行原始输出作为默认上下文。

`attachment/workflow_architecture.svg` 是可编辑的工作流架构源图，
`attachment/workflow_architecture.png` 是直接预览版本；它们只随工作流结构变化而更新，
不记录执行过程。`attachment/` 集中保存文档引用的架构图附件。

`RESEARCH_WIKI.md` 为失败和未闭合路径保留短索引：每行只记录权威对象指针、状态、证据指针、
适用边界、可复用教训与重启条件；不复制 Idea 或 Experiment 的完整结论。

[`research_workflow.md`](research_workflow.md) 定义本地实验阶梯、失败记忆和主线恢复的实际
CLI / Pi 入口与请求格式；它是使用说明，不保存当前 TASK、运行记录或科学判断。

## 准入

新文档必须从本 AGENTS 或对应子目录 AGENTS 建立入口。找不到明确归属时，不创建文件。

## 禁止

- 不在 `docs/` 复制 `state/` 的当前控制状态。
- 不把 EXPLORE 日志、运行输出或 Codex 对话流水账当作长期知识。

## 生命周期

Idea、Protocol 和 Paper 材料由各自 AGENTS 定义准入和状态；引用关系必须指向唯一权威记录，
不要按日期追加副本。
