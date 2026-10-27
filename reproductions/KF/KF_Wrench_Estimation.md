# 给定动力学模型下，利用 Kalman Filter 递推估计末端外力与力矩

记录日期：2026-10-20。本文整理本次讨论中的理论推导，采用固定基座、刚性关节机械臂作为基础模型。

这是一条基于动力学模型的外力观测路线，常见名称包括 *Kalman disturbance observer*、*momentum-based Kalman filter*。核心是：**把未知末端 wrench 加入状态向量，利用它对机器人运动的影响，通过“模型预测—运动测量校正”递推估计。** Wahrburg 等人的 [Cartesian Contact Force Estimation for Robotic Manipulators using Kalman Filters and the Generalized Momentum](https://www.researchgate.net/publication/282808269_Cartesian_Contact_Force_Estimation_for_Robotic_Manipulators_using_Kalman_Filters_and_the_Generalized_Momentum) 采用了“广义动量 + wrench”的增广状态。以下使用统一的符号和正负号约定，推导一个基础版本。

先明确动力学和估计目标。设环境作用于末端的六维 wrench 为

$$
w=
\begin{bmatrix}
f\\
m
\end{bmatrix}
\in\mathbb R^6,
$$

其中 $f$ 是三维力，$m$ 是关于指定末端参考点的三维力矩。对具有 $n$ 个关节自由度的机器人，动力学写为

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)+\tau_f
=
\tau_a+J(q)^\top w.
$$

各符号的含义如下。

| 符号 | 含义 |
|---|---|
| $q,\dot q,\ddot q\in\mathbb R^n$ | 关节位置、速度、加速度 |
| $M(q)\in\mathbb R^{n\times n}$ | 惯性矩阵 |
| $C(q,\dot q)\dot q\in\mathbb R^n$ | 科氏力和离心力项 |
| $g(q)\in\mathbb R^n$ | 重力项 |
| $\tau_f\in\mathbb R^n$ | 摩擦力矩 |
| $\tau_a\in\mathbb R^n$ | 与该动力学模型对应的实际驱动力矩 |
| $J(q)\in\mathbb R^{6\times n}$ | 末端几何雅可比矩阵，速度分量顺序为线速度、角速度 |
| $J(q)^\top w\in\mathbb R^n$ | 末端 wrench 对应的关节外力矩 |

$J$ 必须与 wrench 使用相同的参考点、坐标系和分量顺序。本文把“环境作用于机器人”的 wrench 放在动力学右侧并取正号；若文献采用机器人作用于环境的 wrench，符号会相反。

通常可用的信号是 $q,\dot q,\tau_a$。其中 $\tau_a$ 可以来自关节力矩传感器，或由电机电流经过力矩常数、传动比等换算得到。必须确认力矩信号与模型处于同一侧，并正确计入传动损失和摩擦。**仅有动力学模型和位置指令，一般不足以完成估计；还需要实际运动和驱动力信息。** 使用力矩指令代替实际力矩时，执行误差也会进入外力估计。

从动力学看，外力信息就在下面的残差里：

$$
J^\top w
=
M\ddot q+C\dot q+g+\tau_f-\tau_a.
$$

若右侧各量都准确，且 $J$ 满行秩，就可以通过 $J^\top$ 反求 wrench。实际困难是：$\ddot q$ 往往需要数值微分，噪声较大；动力学、摩擦和驱动力模型也存在误差。因此需要构造递推观测器。

**广义动量形式的价值，是把加速度从需要测量或计算的信号中消掉。**

定义广义动量

$$
p=M(q)\dot q.
$$

这里 $p\in\mathbb R^n$ 是关节空间的广义动量，不是末端位置，也不是六维空间动量。对时间求导：

$$
\dot p=\dot M\dot q+M\ddot q.
$$

采用满足 $\dot M=C+C^\top$ 的标准科氏矩阵构造，代入动力学后得到

$$
\begin{aligned}
\dot p
&=\dot M\dot q+\tau_a+J^\top w-C\dot q-g-\tau_f\\
&=\tau_a+C^\top\dot q-g-\tau_f+J^\top w.
\end{aligned}
$$

这一恒等式依赖于 $C$ 的一致构造，不能把任意满足相同 $C\dot q$ 的矩阵直接代入。将可计算部分记为

$$
b(q,\dot q,\tau_a)
=
\tau_a+C^\top\dot q-g-\hat\tau_f,
$$

于是观测器所用的模型为

$$
\boxed{\dot p=b+J^\top w+\eta_p.}
$$

$\eta_p$ 表示驱动力误差、摩擦残差及其他过程扰动。另一方面，动量可以由运动测量构造：

$$
y_p=M(q_{\mathrm{meas}})\dot q_{\mathrm{meas}}.
$$

**只计算 $p$，不对测得的 $p$ 再求导；$\dot p$ 方程用于向前预测。**

接下来，要让 KF 估计 wrench，必须给 wrench 本身安排一个时间演化模型。最简单的是随机游走：

$$
\dot w=\eta_w,
$$

或离散形式

$$
w_{k+1}=w_k+\epsilon_{w,k}.
$$

连续式中的 $\eta_w$ 是白噪声驱动的形式化写法。其含义是：没有新证据时，预测下一时刻 wrench 与当前相同；允许它通过过程噪声发生变化。它不要求真实接触力恒定，也不要求预先知道环境刚度。随机游走假设针对的是所选坐标系和参考点下的 wrench 分量，换用随末端旋转的坐标系时，需要重新考虑这个时间先验是否合适。

现在取增广状态

$$
x=
\begin{bmatrix}
p\\
w
\end{bmatrix}
\in\mathbb R^{n+6}.
$$

连续模型变成

$$
\dot x=
\underbrace{
\begin{bmatrix}
0&J^\top\\
0&0
\end{bmatrix}
}_{A(t)}
x+
\begin{bmatrix}
b\\
0
\end{bmatrix}
+
\begin{bmatrix}
\eta_p\\
\eta_w
\end{bmatrix},
$$

$$
y_p=
\underbrace{\begin{bmatrix}I_n&0\end{bmatrix}}_{H}
x+v.
$$

**非线性机械臂能够在这个构造下采用普通 KF，是因为 $q,\dot q$ 被当作已测信号，用于逐时计算 $J$ 和 $b$。** 对待估状态 $(p,w)$ 而言，上式是线性的，只是系数随时间变化。

这是一个条件化的建模近似：实际测量噪声也会进入 $J,b,y_p$，并可能引入过程与测量误差之间的相关性。下面先采用标准 KF 的加性、零均值、白噪声及相互独立假设；实际实现需要评估这些近似。

设采样周期为 $T$，采用一阶离散化：

$$
x_{k+1}=F_kx_k+d_k+\epsilon_k,
$$

$$
F_k=
\begin{bmatrix}
I_n&TJ_k^\top\\
0&I_6
\end{bmatrix},
\qquad
d_k=
\begin{bmatrix}
Tb_k\\
0
\end{bmatrix},
\qquad
y_k=Hx_k+v_k.
$$

设离散过程噪声和测量噪声协方差分别为

$$
Q_k=\operatorname{Cov}(\epsilon_k),
\qquad
R_k=\operatorname{Cov}(v_k).
$$

$Q_k$ 应与采样周期及所采用的离散化方式一致；连续时间的噪声谱密度不能不经换算就直接当作离散协方差使用。

**KF 递推通过预测和校正两步完成。** 记 $\hat x_k^-$ 为第 $k$ 步测量更新前的预测，$\hat x_k^+$ 为更新后的估计，$P_k^-,P_k^+$ 为相应误差协方差。

先预测状态和协方差：

$$
\hat x_k^-
=
F_{k-1}\hat x_{k-1}^++d_{k-1},
$$

$$
P_k^-
=
F_{k-1}P_{k-1}^+F_{k-1}^\top+Q_{k-1}.
$$

再根据测量计算新息、新息协方差和增益：

$$
r_k=y_k-H\hat x_k^-,
$$

$$
S_k=HP_k^-H^\top+R_k,
\qquad
K_k=P_k^-H^\top S_k^{-1}.
$$

最后校正状态：

$$
\hat x_k^+=\hat x_k^-+K_kr_k.
$$

协方差可采用数值上更稳健的 Joseph 形式更新：

$$
P_k^+
=
(I-K_kH)P_k^-(I-K_kH)^\top
+
K_kR_kK_k^\top.
$$

在精确算术和标准 KF 增益下，它与 $P_k^+=(I-K_kH)P_k^-$ 等价。

输出状态的最后六维，即为 wrench 估计：

$$
\boxed{
\hat w_k=
\begin{bmatrix}
0&I_6
\end{bmatrix}
\hat x_k^+.
}
$$

虽然测量方程没有直接测量 wrench，动力学中的 $TJ^\top$ 会建立动量与 wrench 的交叉协方差。因此，动量预测出现偏差时，KF 可以同时修正 wrench。将增益按状态分块，外力更新就是

$$
\hat w_k^+
=
\hat w_k^-+
K_{w,k}\left(y_k-\hat p_k^-\right).
$$

直观上，它在不断判断：**在已知驱动力作用下，为了解释实际观测到的动量变化，需要多大的外部 wrench？**

初始化时，可令 $\hat p_0=M(q_0)\dot q_0$。如果已知机器人处于无接触状态，可以令 $\hat w_0=0$；若初始接触未知，则需要给 wrench 合理的先验及较大的初始不确定性，不能把零外力当成已知事实。

**可观测性决定了哪些 wrench 分量能够从关节信号中恢复。**

对上述随机游走模型，在固定构型下有 $A^2=0$，连续系统可观测矩阵的有效部分为

$$
\mathcal O=
\begin{bmatrix}
H\\
HA
\end{bmatrix}
=
\begin{bmatrix}
I_n&0\\
0&J^\top
\end{bmatrix}.
$$

因此

$$
\operatorname{rank}(\mathcal O)
=
n+\operatorname{rank}(J).
$$

完整六维 wrench 可观测要求 $\operatorname{rank}(J)=6$。若存在非零 $\delta w$ 满足

$$
J^\top\delta w=0,
$$

该构型下的关节信号就无法区分 $w$ 与 $w+\delta w$。接近奇异构型时，某些 wrench 方向对关节信号的影响很弱，相应估计容易受噪声和误差影响。构型随时间变化时，应进一步检查整个时间区间的可观测性；固定构型结论不能直接替代时变系统分析。

**模型误差与真实外力的可区分性，往往是精度的主要限制。**

实际残差可以写为

$$
\dot p-b
=
J^\top w+\delta\tau_{\mathrm{model}}.
$$

如果某部分模型误差也能写成 $J^\top\delta w$，单凭这些信号就无法将它与真实外力分开。例如，负载重力补偿错误可能表现为持续接触力。调节 $Q,R$ 可以改变估计的平滑程度和响应速度，但不能凭空解决这种不可辨识性。

还需要注意接触位置假设。上述模型假设未知外部作用可以表示为指定末端点的 wrench。若机械臂中间连杆也发生接触，真实关节外力矩可能为

$$
\tau_{\mathrm{ext}}
=
\sum_i J_i^\top w_i.
$$

此时将全部残差归到末端，可能得到错误的末端 wrench，甚至不存在能准确解释全部关节残差的末端 wrench。不能仅凭估计输出了六个数，就认为识别出了真实末端接触。

调参时，各类不确定性的作用可以这样理解：

| 设置 | 主要含义与影响 |
|---|---|
| 增大 wrench 演化的过程噪声 | 更允许 wrench 快速变化，通常响应更快，但估计也更容易波动 |
| 减小 wrench 演化的过程噪声 | 更强地相信 wrench 在短时间内保持，通常更平滑，但突变接触的跟踪可能变慢 |
| 增大动量过程噪声 | 表达对动力学预测更低的信任，允许更多残差由模型扰动解释 |
| 增大动量测量噪声 $R$ | 表达对由关节信号构造的动量测量更低的信任 |

力与力矩的单位不同，各关节信号的噪声水平也不同，协方差应体现各分量的实际尺度。KF 的最优性依赖模型与噪声假设成立；系统偏差不能自动被零均值噪声假设消除，滤波器报告的协方差也不一定覆盖未建模偏差。

**另一种状态选择是直接联合估计关节运动与 wrench。**

取

$$
x=
\begin{bmatrix}
q\\
v\\
w
\end{bmatrix},
\qquad v=\dot q,
$$

则状态模型为

$$
\begin{cases}
\dot q=v,\\
\dot v=M(q)^{-1}
\left[
\tau_a-C(q,v)v-g(q)-\tau_f+J(q)^\top w
\right],\\
\dot w=\eta_w.
\end{cases}
$$

这时 $q,v$ 也属于待估状态，系统通常是非线性的，需要 EKF、UKF 等方法。测量可以是关节位置，也可以联合关节速度或其他传感器信息。

类似的“位姿、速度、wrench 联合估计”被用于六维虚拟力传感器。Roveda 等人的 [6D Virtual Sensor for Wrench Estimation in Robotized Interaction Tasks Exploiting Extended Kalman Filter](https://ipg.idsia.ch/preprints/Roveda2020b.pdf) 基于笛卡尔阻抗控制下的交互动力学构造 EKF；其具体模型依赖相应控制律，不能直接视为上面通用关节状态模型的同一实现。

对于“给定动力学模型，用 KF 递推估计末端 wrench”，基础构造是 $x=[p^\top,w^\top]^\top$：动力学提供力与运动之间的约束，wrench 演化模型提供时间先验，KF 用连续到来的运动测量修正外力估计。
