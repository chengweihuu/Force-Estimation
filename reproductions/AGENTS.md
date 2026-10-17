# Reproductions 目录准入

## 用途

这里隔离其他论文的方法和官方实现，避免来源、环境与本项目算法混合。大型上游项目可以
保持为独立仓库或固定版本依赖；不要求复制进本目录。

## 准入

每个实际使用的方法按需创建：

```text
reproductions/<paper-or-method>/
├── AGENTS.md
├── adapter/        # 需要时创建；只做输入输出转换
├── patches/        # 确实修改上游时创建
└── environment/    # 独立 venv 或 Docker 说明
```

方法 AGENTS 必须记录：

- 论文题目、作者和稳定引用；
- 官方仓库或代码来源；
- 固定 commit、tag 或版本；
- 许可证；
- 本地修改和修改理由；
- 输入输出适配与单位、坐标系；
- 使用它的 Protocol 和 Experiment；
- 已知的复现差异。

自己根据论文维护的重实现可以进入独立 baseline package，但必须保留来源说明、行为测试
和与实验 adapter 的边界。

## 禁止

- 官方代码不得直接混入 `src/<project>/core/`。
- 不把无来源的复制代码、未使用的论文收藏或项目运行输出放入这里。
- 不在 reproduction 内悄悄改变上游语义；适配与补丁必须可识别。

## 生命周期

先固定来源和版本，再记录本地适配；Protocol/Experiment 使用它时保留对应版本引用。停止
使用时可移除本地副本，但正式 Experiment 仍必须能追溯到来源和适配边界。
