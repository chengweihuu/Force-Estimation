# UR5模型入口

本目录只保存模型、运行配置、官方来源和许可证。当前有效性、官方参数一致性和验证范围见[模型与接口测试报告](../../../docs/conclusion/UR5_Simulation/模型与接口测试报告.md)；机制与历史边界见[仿真接口与控制基础](../../../docs/conclusion/UR5_Simulation/接口与控制基础.md)。

## 选择入口

| 用途 | MuJoCo模型 | ROS配置 | 说明 |
|---|---|---|---|
| 当前六轴小齿隙与只读电流 | `scene_backlash.xml` | `control_backlash/` | 6个驱动＋6个被动间隙坐标；需显式选择 |
| 无齿隙控制与对照 | `scene_equivalent_current.xml` | `control/` | 位置检查脚本的默认配置；与六轴齿隙配置使用相同PID增益 |
| 既有采集与可视化 | `scene.xml`包含`ur5.xml` | 不使用上述ROS配置 | 仍被采集、运动及可视化脚本引用，不能当作冗余删除 |

两个ROS配置目录的文件分工相同。`control`已采用改善后的J6比例增益2.052846875 N·m/rad；原0.052846875配置只保留在历史运行记录中，不再维护独立目录。
| 文件 | 内容 |
|---|---|
| `robot.urdf` | 关节树、命令/状态接口、MJCF及PID路径 |
| `controllers.yaml` | 轨迹控制器、状态发布及更新频率 |
| `pids.yaml` | 实际使用的六轴伺服增益 |
| `equivalent_current.json` | 关节顺序、电流换算系数、驱动/间隙映射 |
| `design.json` | 保持姿态、增益设计来源、时间与坐标定义 |

## 电流与齿隙

- 上层发送六轴位置轨迹，底层PID向motor写入力矩；电流只读。
- `/joint_states`的`effort`单位为N·m；`/ur5/measurement`输出位置、速度、等效电流A和换算驱动力矩，共用源消息时间戳。它不是原生电气电流模型。
- 六轴齿隙模型中，命令与PID反馈均在驱动侧；测量包的连杆角/速度是驱动坐标与间隙坐标之和，另行保留驱动侧及间隙状态。电流只由驱动侧力矩换算，不能使用间隙约束力矩。
- 六轴全间隙各1角分，即0.000290888209 rad；半宽±0.000145444104 rad。采用MuJoCo双同轴hinge及软限位；每轴驱动armature为0.0001 kg·m²。数值均为仿真假设，不是UR5实测标定。软限位仍可能穿透。
- 新模型状态为12维，驱动与间隙交错排列；必须按关节名称寻址，禁止用`qpos[:6]`代表六轴。
- 配置为16 kHz物理积分、2 kHz控制、125 Hz状态发布；真实间隔以时间戳为准。旧`scene.xml`的基础步长为0.5 ms，调用方可能另设步长，不能混用配置说明。

## 来源文件

`metadata/source.json`记录官方仓库、固定版本、许可证和23个原始文件清单。`vendor/ur_description/`保留UR5的URDF/Xacro、配置及网格；视觉网格仍由官方Xacro引用，不因MJCF只使用碰撞网格而删除。

`metadata/adaptation.json`说明`ur5.xml`/`scene.xml`的刚体适配。`metadata/backlash_six_axis.json`记录六轴间隙参数、坐标定义和厂商资料来源。原生dcmotor与单J6过渡模型已清出工作目录，历史结论和已有Run快照保留。

## 使用

容器内，在`/workspace`执行静态配置检查（不推进物理时间）：

```bash
python scripts/simulation/check_backlash_configuration.py --output-dir /tmp/ur5-backlash-check
```

启动当前六轴齿隙ROS配置（会推进仿真，不连接真机）：

```bash
ros2 launch scripts/simulation/position_control.launch.py config_dir:=/workspace/scripts/models/ur5/control_backlash
```

该入口为headless；当前Docker没有GUI挂载。六轴配置静态检查通过，但首次ROS启动检查在发送轨迹前触发J1力矩限幅，当前不能作为已验证的运动/采集配置；见[测试报告](../../../docs/conclusion/UR5_Simulation/模型与接口测试报告.md)。旧刚体运动检查器不能直接用于齿隙模型；无齿隙配置的通过结果不能移用到六轴齿隙配置。
