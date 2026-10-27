# UR5 Docker 环境

**镜像构建与安装检查已通过；控制器激活及UR5运动尚未验收。** 唯一入口为本目录的 `compose.yaml`。

```text
docker/
├── Dockerfile             # 系统、ROS和Python安装步骤
├── compose.yaml           # 镜像、挂载、资源和运行网络
├── entrypoint.sh          # 加载ROS/Python，设置日志与缓存
├── requirements/          # 三组Python依赖
├── checks/                # 默认环境与刚体环境的安装检查
├── .dockerignore          # 限制构建上下文
├── README.md              # 使用说明
└── AGENTS.md              # 目录维护规则
```

## 环境组成

| 组件 | 配置 |
|---|---|
| 系统 | Ubuntu24.04、ROS2 Jazzy、Python3.12，linux/amd64 |
| MuJoCo | ROS vendor与两套Python均要求3.12.0 |
| 控制桥 | mujoco_ros2_control、demos、msgs、plugins 0.1.2；vendor0.1.0 |
| ROS控制组件 | controller_manager、joint_trajectory_controller、joint_state_broadcaster、position_controllers、control_toolbox |
| 默认Python | NumPy1.26.4、SciPy1.14.1、PyTorch2.9.1+cpu等，见requirements/requirements-main.txt及requirements-torch.txt |
| 刚体Python | Pinocchio3.8.0、NumPy2.3.5、CVXPY1.6.5、Clarabel0.11.1等，见requirements/requirements-rigid.txt |

控制桥相关完整APT版本以Dockerfile为准，来自2026-10-26核对的
[官方Noble/amd64仓库](http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-amd64/Packages.gz)。
包版本不可获得时停止，不自动替换版本。
[官方控制桥说明](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html)。

默认venv可访问系统ROS包；刚体venv独立，隔离NumPy与原生依赖，不向ROS全局注入其库路径。
控制桥的MuJoCo库来自vendor，Python库来自PyPI；检查脚本分别核对实际加载版本。
直接依赖和关键原生依赖已固定，但不构成完整传递依赖锁，也不是冻结实验环境。

## 使用

在项目根目录执行：

```bash
# 静态检查
docker compose -f docker/compose.yaml config --quiet

# 构建（研究任务按已批准预算及新请求执行）
docker compose -f docker/compose.yaml build

# 构建成功后进入容器
PIAKF_UID=$(id -u) PIAKF_GID=$(id -g) \
  docker compose -f docker/compose.yaml run --rm --no-deps piakf
```

容器内默认`python`是`/opt/piakf-venv/bin/python`；刚体环境用`$REGRESSION_PYTHON`。
重放安装检查：

```bash
python /opt/piakf-check-environment.py
"$REGRESSION_PYTHON" /opt/piakf-check-rigid-environment.py
```

## 运行与验收边界

- 默认只启动Shell。项目根目录挂载为`/workspace`，容器内修改会写回宿主机。
- 容器限2 CPU、4 GiB内存、256 MiB共享内存；不等于构建器资源限制。
- 运行禁用外部网络，ROS使用本地回环和domain97；不连接真机，未配置GUI/GPU。构建下载仍需网络。
- 临时日志/缓存位于容器`/tmp`；需要保留的结果写入`/workspace`下指定Run。
- 构建检查包版本、动态链接、ROS消息导入、CPU自动微分及既有小型刚体回归/优化问题；不调用mj_step，不启动ROS节点，不读取项目轨迹。
- 安装检查不验证渲染、插件激活、控制时序或UR5无载/带载跟踪；这些需要后续独立验收。

成功构建后，实际依赖表位于`/opt/piakf-system-packages.txt`、
`/opt/piakf-python-packages.txt`、`/opt/piakf-rigid-python-packages.txt`；
检查报告为`/opt/piakf-installation-check.json`。

## 旧环境

新镜像标签为`piakf:jazzy-mujoco3.12.0-control0.1.2`，不覆盖旧3.3.7及h022-physical镜像。
旧版本源码备份和失败记录留在任务目录及Run中，不放入本目录。
旧构建请求不再适用，下一次构建应绑定当前配置；旧实验按原镜像和请求重放。

最近构建：`runs/H-097/R-20261026T140845Z-installfinish/report.md`。新镜像已导出，容器内安装检查通过；控制桥0.1.2、vendor0.1.0、MuJoCo3.12.0、PyTorch及Pinocchio检查通过。旧镜像和历史失败记录保留。
