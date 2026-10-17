# Docker 目录准入

## 用途

这里只保存可复现环境所需的 Dockerfile、compose 文件和入口脚本。它描述环境，不承载
研究代码或结果。

## 准入

- 只有确实需要固定系统依赖、ROS/驱动环境或跨机器复现时才创建文件。
- 镜像构建应引用正式代码、固定依赖和明确版本；运行参数由任务或正式协议提供。
- 环境变更应说明影响的当前 H-task 和 Stage，并保持入口可重放；不因环境变化创建新
  H-task 或 Scientific Protocol version。
- `EXPLORE` 只在当前 probe 确有需要时使用 Docker，记录理解输出所需的最小版本信息，不要求
  image digest 或 lockfile hash。
- `VALIDATE` 在观察 endpoint 前由当前 TASK 的 Execution Contract 固定运行模式、基础镜像/主机版本、
  依赖参照、硬件/驱动和资源上限；Dockerfile 只是合同实现，不是需求发现的替代品。
  未确认系统依赖时不得凭空创建项目镜像。
- `FORMAL` 在 immutable request / Environment Binding 中记录不可变 image digest、Dockerfile/lockfile
  hash、挂载点、入口命令和资源限制。新环境必须创建新 binding/request 并保留旧绑定，
  但不升级 Scientific Protocol。

## 禁止

- 不放业务代码、实验配置、数据、日志、bag 或 `runs/`。
- 不把临时调试命令固化成默认入口。

## 生命周期

候选环境先服务于当前 H-task 的 `VALIDATE`；被 `FORMAL` 使用后保留精确 binding/request
引用。未被任务或执行绑定使用的环境文件不进入目录。
