# Physics-informed adaptive Kalman filter for contact force estimation in industrial robots considering model uncertainty

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/4e8b4aa69746200faf48dbe2ce3348033729661f6c3c56f5a9a438e7bf14112e.jpg)


Hongbo Hu <sup>a</sup>, Jia Ma <sup>b</sup>, Yuhan Lu <sup>a</sup>, Chungang Zhuang <sup>a,*</sup> 

<sup>a</sup> School of Mechanical Engineering, Shanghai Jiao Tong University, Shanghai, 200240, PR China <sup>b</sup> Shanghai Institute of Aerospace Systems Engineering, Shanghai, 201108, PR China 

## A R T I C L E I N F O

Keywords: Industrial robots Contact force estimation Physics-informed neural networks Adaptive Kalman filter Model uncertainty learning 

## A B S T R A C T

Accurate external contact force estimation is essential for robot interaction tasks that lack torque sensors. Existing force observation methods accounting for model uncertainties often fail to effectively address dynamic uncertainties caused by joint backlash. This article presents a hybrid physics-data modeling approach for robot dynamics and friction, leveraging physics-informed neural networks (PINNs), and applies it to observer design. First, a dynamic error model associated with backlash is formulated using Gaussian basis functions, enabling real-time compensation for model uncertainties induced by joint backlash. Building on this model, a physics informed adaptive Kalman filter (PIAKF) is proposed to address model uncertainty. Furthermore, the propagation of disturbances during observation is analyzed, and effective suppression of disturbance noise is achieved by applying a variable-time-period mode-switching moving average filter. Finally, numerical simulations and physical experiments demonstrate the performance of the proposed PIAKF, showing significant improvement over existing advanced observation techniques. 

## 1. Introduction

Robotic manipulators operating under position/velocity control modes perform a wide range of tasks, such as palletizing, welding, and painting. However, due to the lack of consideration for interaction with the external environment, they are limited to performing target position movement or trajectory tracking in structured scenarios. The introduc tion of external force information can significantly enhance the manipulation performance and safety of robots. Through impedance control or hybrid force/position control [1,2], more complex interactive environment perception and compliant control tasks can be achieved, such as cooperative transportation [3], precision assembly [4], or other applications requiring interaction with the environment or humans. 

Force/torque sensors represent the most prevalent solution for acquiring external force information. However, constrained by manufacturing costs and robot structural rigidity requirements, most industrial robots are not equipped with joint torque sensors and gener ally avoid installing force/torque sensors on end-effectors. Moreover, improper mechanical integration of such sensors may induce instability in the robotic control system [5]. These limitations have propelled sensorless contact force estimation methods to the forefront of recent research. Fig. 1 illustrates the roadmap of previous sensorless contact force estimation approaches. 

The fundamental principle of sensorless force estimation lies in constructing a state observer based on the robot’s inverse dynamics model (IDM) to approximate external forces. To circumvent issues of differential noise from joint acceleration and computational overhead caused by inertia matrix inversion, generalized momentum observers (GMOs) have been widely implemented in applications including collision detection [6] and payload identification [7]. Nevertheless, the inherent first-order filtering characteristic of GMO-based external force observers inevitably attenuates high-frequency components, limiting their accuracy in contact force signal estimation [8]. 

Another critical limitation of IDM-based sensorless observers arises from their heavy reliance on model fidelity [9,10]. While robot IDM is typically developed using physics-based approaches like the Newton-Euler or Lagrangian methods to establish standard dynamics models, followed by parameter minimization and identification [11], several practical factors compromise model accuracy. Unmodeled nonlinearities including joint friction, structural elasticity, and transmission backlash invariably distort the dynamics model [12]. Additionally, non-negligible noise during the estimation process can also induce significant model uncertainties [13]. 

Effective handling of model uncertainties has emerged as a crucial factor in improving contact force estimation performance. The extended state observer (ESO) within the active disturbance rejection control (ADRC) framework treats both dynamics model uncertainties and external disturbances as aggregated disturbances, extending them as additional state variables. This approach grants ESO-based observers greater robustness, particularly for nonlinear systems, making them well-suited for force estimation under significant disturbances. The effectiveness of ESO-based observers depends heavily on their state space dimensionality. Higher-order ESOs exhibit superior observation performance, enabling effective estimation of high-frequency timevarying contact forces [14] and accelerations [15], and demonstrating enhanced capability in handling complex, nonlinear, and time-varying disturbances. ESOs can also be combined with GMOs to achieve order reduction and improve computational efficiency [16]. However, ESO-based observers are highly sensitive to gain parameters, resulting in more complex designs. Additionally, a trade-off between computational efficiency and observation delay must be considered, which hinders their application in more rigorous tasks. 

The introduction of prior knowledge regarding external forces can further improve contact force estimation accuracy [17]. The disturbance Kalman filter (DKF), incorporating a prior process model error covariance matrix (ECM), effectively reduces measurement and estimation noise in real time. Despite inherent inaccuracies in ECM, the DKF maintains robust performance, validated in continuum robot control [18], vehicle state estimation [19], and ball screw disturbance estimation [20]. Under the assumption that system noise and measurement noise follow Gaussian distributions, the DKF provides statistically optimal linear estimation. Consequently, the integration of DKF with generalized momentum has become a key research focus in robot force estimation. 

Wahrburg et al. are among the first to propose the use of DKF for robot contact force estimation [21,22]. Building on this foundation, adaptive Kalman filters (AKF) incorporating mode-switching filtering with variable time periods [23] and sliding-window variational techniques [24] have been developed to address the time-varying characteristics of external contact forces during robot-environment interactions. Furthermore, Liang et al. designed the Sage-Husa adaptive strong tracking Kalman filter (SHASTKF) specifically for lower-limb rehabilitation robots, which recursively estimates noise characteristics and achieves rapid tracking of abrupt contact force changes [25]. 

Although both ESO-based and KF-based methods can compensate for model uncertainties to some extent, the multi-source heterogeneous nature of these uncertainties in real-world contact force estimation tasks still poses challenges for high-precision estimation [10]. Data-driven methods, benefiting from their superior learning capabilities for nonlinear errors, have emerged as a potential solution. To address un certainties in robot dynamics models, semi-parametric dynamics models based on neural networks have been applied to the development of DKF [26,27]. Lim et al. further integrated model uncertainties and external disturbances by constructing a long short-term memory (LSTM) module to improve GMO [28]. 

Gaussian process regression (GPR) offers another approach for model-data hybrid-driven modeling. Wei et al. derived a virtual measurement equation for GPR-based contact force estimation and developed the Gaussian process adaptive disturbance Kalman filter (GPADKF) [10]. Furthermore, Evangelisti et al. derived physically consistent Lagrangian-Gaussian processes (L-GP) and developed a momentum observer with uncertainty quantification [29,30]. 

The incorporation of data-driven techniques offers improved solutions for modeling nonlinear effects. However, the resulting nonparametric models are inherently nonlinear and often lack clear dynamics descriptions, making it challenging to design a force observer that explicitly incorporates the physical information of the robot. Physics-informed neural networks (PINNs), which combine the strengths of physical modeling and deep learning, have demonstrated promise in robot dynamics [31,32], yet remain understudied for contact force estimation. 

To address the aforementioned issues in existing observers, this article develops a physics-informed adaptive Kalman filter (PIAKF) based on PINN, which accounts for model uncertainties and enables higher-precision estimation of external contact forces in robots, while achieving uncertainty compensation with explicit physical information. First, the robot dynamics and friction modeling based on PINN are completed, and the derivative of the inertia matrix is derived, constructing the physics-informed Kalman filter (PIKF). Subsequently, a disturbance compensation module incorporating Gaussian basis functions is designed and embedded into the proposed PIKF to achieve compensation for internal uncertainties in the robot system. Finally, the issue of disturbance propagation during the observation process is effectively mitigated through online moving average filtering. Simulations and real-world experiments are conducted, and the proposed PIAKF is compared with other advanced methods, demonstrating its superior performance in observation and handling model uncertainties. Additionally, the computational time cost of the observer is compared and evaluated. The main contributions of this work are as follows: 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/1459b5844f64c3353f9936fd46c8a583a6eb38b887b1a60192cb329622669207.jpg)



Fig. 1. Previous sensorless contact force estimation approaches.


1) The PIAKF observer incorporating PINN-based robot dynamics and friction modeling is proposed. The derivative of the inertia matrix is solved through chain computation, resulting in reduced observation time cost. 

2) A general backlash uncertainty compensation module integrating Gaussian basis functions is designed and embedded into a unified PINN framework, effectively addressing internal model uncertainties in the robot system. 

3) The backlash uncertainty is decoupled from disturbance noise during the observation process by constructing an auxiliary observation variable, while a mode-switching moving average filter with variable time periods is applied to effectively suppress the noise. 

The remainder of this article is organized as follows. Section 2 outlines the necessary theoretical background and related work. Section 3 provides a detailed description of the proposed PIAKF for robot contact force estimation, including the construction of the observer and uncertainty modeling. Section 4 validates the effectiveness of PIAKF through simulations and real-world experiments. Finally, Section 5 provides the conclusion. 

## 2. Preliminary knowledge

## 2.1. Model-based external torque estimation

Benefiting from prior knowledge of dynamics and friction behaviors, model-based external force estimation demonstrates superior performance compared to model-free methods. The rigid-body dynamics model of an n-degree-of-freedom (DOF) serial robot can be derived in the same form using either the Newton-Euler method or the Lagrangian method: 

$$
\underbrace {\boldsymbol {M} (\boldsymbol {q}) \ddot {\boldsymbol {q}} + \boldsymbol {C} (\boldsymbol {q} , \dot {\boldsymbol {q}}) \dot {\boldsymbol {q}} + \boldsymbol {G} (\boldsymbol {q})} _ {:= \boldsymbol {\tau} _ {d y n}} + \boldsymbol {\tau} _ {f r i} = \boldsymbol {\tau} _ {d r i} + \boldsymbol {\tau} _ {e x t}\tag{1}
$$

where $\pmb q , \dot { \pmb q } , \ddot { \pmb q } \in \mathfrak { R } ^ { n \times 1 }$ represent the vectors of robot joint angles, angular velocities, and angular accelerations, respectively. The inertia matrix is represented as $\pmb { M } ( \pmb { q } ) \in \Re ^ { n \times n }$ , which is symmetric and positive-definite. $C ( \pmb { q } , \ \pmb { \dot { q } } ) \in \mathfrak { R } ^ { n \times n }$ is the Coriolis and centrifugal matrix and $\pmb { G } ( \pmb q ) \in \Re ^ { n \times 1 }$ denotes the gravity vector. $\pmb { \tau } _ { f r i } \in \Re ^ { n \times 1 }$ and $\pmb { \tau } _ { d r i } \in \Re ^ { n \times 1 }$ represent the friction torque and driving torque at each joint of the robot, respectively. 

The force $\left[ F _ { x } \quad F _ { y } \quad F _ { z } \right] ^ { T }$ and the corresponding torque $\begin{array} { r l } { [ \tau _ { x }  } & { { } \tau _ { y } \quad \tau _ { z } ] ^ { T } } \end{array}$ acting on the robot end-effector in Cartesian space together constitute the generalized external force $\pmb { \omega } \in \Re ^ { m \times 1 }$ applied to the robot. The external torque components $\pmb { \tau } _ { e x t } \in \Re ^ { n \times 1 }$ generated in the robot joint space can be calculated using the velocity Jacobian matrix $\pmb { J } ( \pmb q ) \in \Re ^ { m \times n }$ as follows: 

$$
\pmb {\tau} _ {e x t} = \pmb {J} (\pmb {q}) ^ {T} \pmb {\omega}\tag{2}
$$

For industrial robots typically without joint torque sensors, the joint driving torque can be estimated by converting the joint current $\pmb { I _ { m } } \in \Re ^ { n \times 1 }$ : 

$$
\pmb {\tau} _ {d r i} = \pmb {K} _ {m} \odot \pmb {I} _ {m}\tag{3}
$$

where $\pmb { K } _ { m } \in \Re ^ { n \times 1 }$ is the joint drive gain used for estimating the driving torque from the joint current, and ⊙ represents the Hadamard product. 

While $\pmb { \tau } _ { e x t }$ can theoretically be derived from Eq. (1), practical implementation requires online joint acceleration estimation through numerical velocity differentiation. This process amplifies measurement noise, potentially masking external torque components. The observer based on generalized momentum addresses this issue by avoiding the acceleration terms. Its fundamental idea is to construct the observer using the difference between the model-based generalized momentum and the measured generalized momentum. The generalized momentum is defined as $\pmb { p } = \pmb { M } ( \pmb { q } ) \dot { \pmb { q } } \in \Re ^ { n \times 1 }$ , and the estimated value of the external torque components is given by: 

$$
\widehat {\boldsymbol {\tau}} _ {e x t} = \boldsymbol {K} _ {p} (\widehat {\boldsymbol {p}} - \boldsymbol {p})\tag{4}
$$

where $\pmb { K } _ { p } \in \Re ^ { n \times n }$ is the observer gain matrix, and hat $\widehat { \cdot } \widehat { \cdot }$ indicates estimated values. Taking the derivative with respect to time t, the derivative of the generalized momentum is obtained as: 

$$
\dot {\boldsymbol {p}} = \dot {\boldsymbol {M}} (\boldsymbol {q}) \dot {\boldsymbol {q}} + \boldsymbol {M} (\boldsymbol {q}) \ddot {\boldsymbol {q}}\tag{5}
$$

To avoid the direct computation of the inertia matrix differentiation $\dot { \pmb { M } } ( \pmb { q } )$ , the passivity property is utilized to optimize the computational process, since $\dot { \pmb { M } } ( \pmb { q } ) - 2 \pmb { C } ( \pmb { q } , \ \dot { \pmb { q } } )$ is skew-symmetric [8]: 

$$
\dot {\boldsymbol {M}} (\boldsymbol {q}) = \boldsymbol {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) + \boldsymbol {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) ^ {T}\tag{6}
$$

where $C ( \pmb { q } , \dot { \pmb { q } } )$ is a Coriolis and centrifugal matrix satisfying the passivity property and $C ( \pmb { q } , \dot { \pmb { q } } ) ^ { T }$ denotes its transpose. 

Therefore, Eq. (5) can be rewritten as: 

$$
\begin{array}{l} \dot {\boldsymbol {p}} = \boldsymbol {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) ^ {T} \dot {\boldsymbol {q}} + \boldsymbol {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) \dot {\boldsymbol {q}} + \boldsymbol {M} (\boldsymbol {q}) \ddot {\boldsymbol {q}} \\ = \boldsymbol {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) ^ {T} \dot {\boldsymbol {q}} - \boldsymbol {G} (\boldsymbol {q}) + \boldsymbol {\tau} _ {d r i} + \boldsymbol {\tau} _ {e x t} - \boldsymbol {\tau} _ {f r i} \end{array}\tag{7}
$$

The estimated generalized momentum $\widehat { \pmb { p } }$ can be computed through an integration operation: 

$$
\widehat {\boldsymbol {p}} = \int \widehat {\dot {\boldsymbol {p}}} d t = \int \left[ \widehat {\boldsymbol {C}} (\boldsymbol {q}, \dot {\boldsymbol {q}}) ^ {T} \dot {\boldsymbol {q}} - \widehat {\boldsymbol {G}} (\boldsymbol {q}) + \boldsymbol {\tau} _ {d r i} + \widehat {\boldsymbol {\tau}} _ {e x t} - \widehat {\boldsymbol {\tau}} _ {f r i} \right] d t\tag{8}
$$

By substituting $\operatorname { E q . } \ ( 8 )$ into $\mathrm { E q . } \ ( 4 ) .$ , external force estimation for the robot based on the generalized momentum observer can be achieved 

## 2.2. Robot dynamics and friction modeling with PINNs

Since the robot inertia matrix possesses the properties of being symmetric and positive-definite, it can be decomposed as follows [31]: 

$$
\boldsymbol {M} (\boldsymbol {q}) = \boldsymbol {L} (\boldsymbol {q}) \boldsymbol {L} (\boldsymbol {q}) ^ {\boldsymbol {T}}\tag{9}
$$

where $\pmb { L } ( \pmb { q } ) \in \Re ^ { n \times n }$ is a lower triangular matrix. This decomposition strictly ensures the physical constraint of the positive definiteness of the inertia matrix ${ \pmb M } ( { \pmb q } )$ . For brevity, the dependence on q is omitted in the following derivations, with M and L denoting ${ \pmb M } ( { \pmb q } )$ and ${ \pmb L } ( { \pmb q } )$ , respectively. 

Furthermore, the time derivative of the inertia matrix can be obtained as: 

$$
\dot {\boldsymbol {M}} = \frac {d}{d t} \left(\boldsymbol {L} \boldsymbol {L} ^ {T}\right) = \boldsymbol {L} \frac {d \boldsymbol {L} ^ {T}}{d t} + \frac {d \boldsymbol {L}}{d t} \boldsymbol {L} ^ {T}\tag{10}
$$

However, since time t is not an input to the robot dynamics model, directly computing Eq. (10) is challenging. Therefore, chain computation is employed to avoid direct differentiation: 

$$
\dot {\boldsymbol {M}} = \boldsymbol {L} \frac {\partial \boldsymbol {L} ^ {T}}{\partial \boldsymbol {q}} \frac {\partial \boldsymbol {q}}{\partial t} + \frac {\partial \boldsymbol {L}}{\partial \boldsymbol {q}} \frac {\partial \boldsymbol {q}}{\partial t} \boldsymbol {L} ^ {T} = \boldsymbol {L} \frac {\partial \boldsymbol {L} ^ {T}}{\partial \boldsymbol {q}} \dot {\boldsymbol {q}} + \frac {\partial \boldsymbol {L}}{\partial \boldsymbol {q}} \dot {\boldsymbol {q}} \boldsymbol {L} ^ {T}\tag{11}
$$

Benefiting from the modular composition and differentiability of the network, the derivatives of L and $L ^ { T }$ with respect to the input q can be computed through recursive application of chain rules: 

$$
\frac {\partial \boldsymbol {L}}{\partial \boldsymbol {q}} = \frac {\partial \boldsymbol {L}}{\partial \boldsymbol {h} _ {N - 1}} \frac {\partial \boldsymbol {h} _ {N - 1}}{\partial \boldsymbol {h} _ {N - 2}} \dots \frac {\partial \boldsymbol {h} _ {1}}{\partial \boldsymbol {q}}\tag{12}
$$

where $\pmb { h } _ { i }$ denotes the output of the i-th network layer, derived from an affine transformation succeeded by a nonlinear activation function g (⋅): 

$$
\boldsymbol {h} _ {i} = \boldsymbol {g} _ {i} \left(\boldsymbol {W} _ {i} ^ {T} \boldsymbol {h} _ {i - 1} + \boldsymbol {b} _ {i}\right)\tag{13}
$$

where ${ \pmb w } _ { i }$ and $\pmb { b } _ { i }$ represent the network parameters of the i-th layer. $\pmb { h } _ { i _ { - } }$ 1 is the output of the (i–1)-th network layer and also serves as the input to the i-th network layer, thus yielding: 

$$
\frac {\partial \boldsymbol {h} _ {i}}{\partial \boldsymbol {h} _ {i - 1}} = \operatorname{diag} \left[ \boldsymbol {g} ^ {\prime} \left(\boldsymbol {W} _ {i} ^ {T} \boldsymbol {h} _ {i - 1} + \boldsymbol {b} _ {i}\right) \right] \boldsymbol {W} _ {i}\tag{14}
$$

By repeatedly applying Eq. (14) and substituting it into Eq. (12), the derivatives of L and $L ^ { T }$ with respect to the dynamics input q can be obtained. Subsequently, the time derivative of the inertia matrix M ˙ can be derived through Eq. (10). 

The other part of the Coriolis and centrifugal terms in the robot dynamics model can also be derived in a similar manner: 

$$
\frac {\partial}{\partial \boldsymbol {q}} \left(\dot {\boldsymbol {q}} ^ {T} \boldsymbol {L} \boldsymbol {L} ^ {T} \dot {\boldsymbol {q}}\right) = \dot {\boldsymbol {q}} ^ {T} \left(\frac {\partial \boldsymbol {L}}{\partial \boldsymbol {q}} \boldsymbol {L} ^ {T} + \boldsymbol {L} \frac {\partial \boldsymbol {L} ^ {T}}{\partial \boldsymbol {q}}\right) \dot {\boldsymbol {q}}\tag{15}
$$

Therefore, the robot dynamics model can be determined, and each term can be inferred: 

$$
\boldsymbol {\Theta} (\boldsymbol {q}, \dot {\boldsymbol {q}}, \ddot {\boldsymbol {q}}) = \boldsymbol {L L} ^ {T} \ddot {\boldsymbol {q}} + \underbrace {\frac {d}{d t} \left(\boldsymbol {L L} ^ {T}\right) \dot {\boldsymbol {q}} - \frac {1}{2} \left\{\frac {\partial}{\partial \boldsymbol {q}} \left[ \dot {\boldsymbol {q}} ^ {T} \boldsymbol {L L} ^ {T} \dot {\boldsymbol {q}} \right] \right\} ^ {T}} _ {:= C (\boldsymbol {q}, \dot {\boldsymbol {q}}) \dot {\boldsymbol {q}}} + \boldsymbol {G}\tag{16}
$$

where L and G are computed through a Lagrangian layer with rectified linear unit (ReLU) networks. 

The joint friction behavior exhibits a significant Stribeck effect at low velocities, where friction torque decreases nonlinearly with increasing velocity. The modeling of joint friction considering the Stribeck effect can also be embedded into the PINN-based dynamics framework to further enhance the accuracy of joint torque estimation [32]. The friction torque of the k-th joint can be expressed as: 

$$
\begin{array}{l} \tau_ {f r i} ^ {k} = \mathbf {W} _ {f} \big (\dot {q} _ {k}, v _ {s} ^ {k}, \delta_ {s} ^ {k}, \alpha^ {k} \big) \mathbf {K} _ {f} \\ \mathbf {K} _ {f} = \left[ \begin{array}{c c c} f _ {c} ^ {k} & f _ {\nu} ^ {k} & f _ {b} ^ {k} \end{array} \right] ^ {T} \end{array}\tag{17}
$$

where $f _ { c } ^ { k } , f _ { \nu } ^ { k }$ and $f _ { b } ^ { k }$ represent the Coulomb coefficient, viscous coefficient, and friction offset of the k-th joint, respectively. ${ \bf W } _ { f }$ represents the friction model matrix, which can be expanded as: 

$$
\begin{array}{l} \mathbf {W} _ {f} = \operatorname{sign} (\dot {q} _ {k}) \cdot \left[ \lambda_ {k} (1 - \lambda_ {k}) | \dot {q} _ {k} | ^ {\alpha^ {k}} \right] \\ \lambda_ {k} = e x p \left(- | \dot {q} _ {k} / v _ {s} ^ {k} | ^ {\delta_ {s} ^ {k}}\right) \end{array}\tag{18}
$$

The sign function is denoted as sign(⋅). The empirical parameters $\nu _ { s } ^ { k } ,$ $\delta _ { s } ^ { k } , \alpha ^ { k }$ and friction coefficients $\mathbf { K } _ { f }$ are independent of the inputs to the friction model; instead, they serve as internal trainable parameters of the PINN and are updated during backpropagation. 

## 3. Proposed physics-informed adaptive Kalman filter

This section details the design methodology of the proposed PIAKF for robot contact force estimation, as illustrated in Fig. 2. First, the Kalman filter (KF) incorporating PINN-based dynamics modeling is derived to achieve better approximation of nonlinear dynamics and friction behaviors. Simultaneously, a faster inference method for the derivative of the inertia matrix is developed to enhance the efficiency of the observer. Subsequently, a disturbance compensation module based on Gaussian basis functions is designed to address uncertainties caused by internal disturbances in the robot system. Finally, the issue of disturbance propagation during the observation process is analyzed. An auxiliary observation variable is constructed, and the influence of observation noise is effectively mitigated through online weighted moving average (WMA) filtering. 

## 3.1. PINN-based improved Kalman filter

The generalized momentum p of the robot system is also utilized to construct the PINN-based improved KF. Based on the PINN modeling approach for robot dynamics and friction introduced in Section 2.2, the inverse dynamics and friction inference models of the robot can be obtained through excitation experiments and offline training, thereby capturing the robot’s dynamics behaviors, namely $\pmb { \tau } _ { d y n }$ and $\tau _ { f r i } .$ . Specifically, the inertia matrix $\pmb { M } ( \pmb q )$ , the Coriolis and centrifugal matrix $C ( { \pmb q } ,$ ${ \dot { \mathbf { q } } } ) ,$ and the gravity vector ${ \pmb G } ( { \pmb q } )$ can all be solved through the structured PINN network. 

For the design of the PINN-based improved KF, since the inertia matrix differentiation M ˙ can be computed using Eq. (11), there is no need to further reformulate the observer structure using Eq. (6). Consequently, the time derivative of the generalized momentum $\dot { p }$ can be calculated as follows: 

$$
\dot {\boldsymbol {p}} = \left(\boldsymbol {L} \frac {\partial \boldsymbol {L} ^ {T}}{\partial \boldsymbol {q}} + \frac {\partial \boldsymbol {L}}{\partial \boldsymbol {q}} \boldsymbol {L} ^ {T}\right) \dot {\boldsymbol {q}} ^ {2} + \boldsymbol {M} \ddot {\boldsymbol {q}}\tag{19}
$$

By substituting into Eq. (1), the following can be obtained: 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/8b37bfce1bb5f304fa2678779cdff85e4bff87e6dc503880c55a17c7b0ae3ba8.jpg)



Fig. 2. Illustration of the proposed PIAKF architecture for contact force estimation.


$$
\dot {\pmb {p}} = \left(\pmb {L} \frac {\partial \pmb {L} ^ {T}}{\partial \pmb {q}} + \frac {\partial \pmb {L}}{\partial \pmb {q}} \pmb {L} ^ {T}\right) \dot {\pmb {q}} ^ {2} + \pmb {\tau} _ {d r i} + \pmb {\tau} _ {e x t} - \pmb {\tau} _ {f r i} - \pmb {\tau} _ {d y n} (\pmb {q}, \dot {\pmb {q}}, 0)\tag{20}
$$

where $\pmb { \tau } _ { d y n } ( \pmb { q } , \pmb { \dot { q } } , 0 )$ represents the result obtained by setting the acceleration q ¨ in the dynamics term to zero. To simplify the expression, an auxiliary observation variable H can be constructed as follows: 

$$
\pmb {H} = \left(\pmb {L} \frac {\partial \pmb {L} ^ {T}}{\partial \pmb {q}} + \frac {\partial \pmb {L}}{\partial \pmb {q}} \pmb {L} ^ {T}\right) \dot {\pmb {q}} ^ {2} + \pmb {\tau} _ {d r i} - \pmb {\tau} _ {f r i} - \pmb {\tau} _ {d y n} (\pmb {q}, \dot {\pmb {q}}, 0)\tag{21}
$$

Consequently, the following can be derived: 

$$
\dot {\boldsymbol {p}} = \boldsymbol {H} + \boldsymbol {\tau} _ {e x t}\tag{22}
$$

By substituting into $\operatorname { E q . } \left( 2 \right)$ and incorporating considerations of noise in the auxiliary variable $H ,$ the following is obtained: 

$$
\dot {\boldsymbol {p}} = \boldsymbol {H} + \boldsymbol {J} (\boldsymbol {q}) ^ {T} \boldsymbol {\omega} + \boldsymbol {e} _ {p}\tag{23}
$$

where $\pmb { e } _ { p } \sim N \Big ( 0 , \sigma _ { p } ^ { 2 } \Big )$ represents the noise in H, which includes both model uncertainties and noise in the measured torque $\tau _ { d r i } .$ The matrix ${ \mathfrak {sigma } } _ { p } ^ { 2 } \in \mathfrak { R } ^ { n \times n }$ can be assumed to be a diagonal matrix reflecting independent errors across each joint. 

Since the external contact force ω varies dynamically and is difficult to model precisely, it can be approximated using a constant disturbance or a higher-order polynomial disturbance form. For the component ω<sub>j</sub> of the external contact force in the j-th direction, its γ-th order polynomial disturbance can be expressed as: 

$$
\left[ \begin{array}{c c} \dot {\omega} _ {j} \ddot {\omega} _ {j} & \vdots \\ \omega_ {j} ^ {(\gamma)} \end{array} \right] = \left[ \begin{array}{c c c c} 0 & 1 & \dots & 0 \\ 0 & 0 & \ddots & 0 \\ \vdots & \vdots & \ddots & 1 \\ 0 & 0 & \dots & 0 \end{array} \right] \left[ \begin{array}{c} \omega_ {j} \\ \vdots \\ \dot {\omega} _ {j} \\ \omega_ {j} ^ {(\gamma - 1)} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \vdots \\ 1 \end{array} \right] e _ {\omega j}\tag{24}
$$

Here, $\omega _ { j } ^ { ( \gamma ) }$ represents the γ-th derivative of $\omega _ { j } ,$ and $e _ { \omega j }$ denotes unpredictable model errors, which are typically assumed to follow a Gaussian distribution. When γ = 1, it can be approximated as a constant or piecewise constant disturbance. Therefore, the external contact force ω can be dynamically modeled as a first-order Markov process in the following form: 

$$
\dot {\boldsymbol {\omega}} = \boldsymbol {S} \boldsymbol {\omega} + \boldsymbol {e} _ {\omega}\tag{25}
$$

where $\pmb { S } \in \Re ^ { m \times m }$ is the dynamic matrix of external disturbances, and the uncertainty $\pmb { e } _ { \omega } \sim N \big ( 0 , \pmb { \sigma } _ { \omega } ^ { 2 } \big )$ follows a normal distribution. The matrix $\pmb { \sigma } _ { \omega } ^ { 2 } \in \Re ^ { m \times m }$ is assumed to be a diagonal matrix reflecting independent uncertainties in each direction of the external force. 

The estimation of the contact force is determined by both the robot’s internal state and the external force state. Eqs. (23) and (25) provide models for these two components, respectively. Therefore, the obser vation system model can be expressed as follows: 

$$
\left[ \begin{array}{c} \dot {\boldsymbol {\omega}} \\ \boldsymbol {\omega} \end{array} \right] = \left[ \begin{array}{c c} 0 _ {n \times n} & \boldsymbol {J} ^ {T} \\ 0 _ {m \times n} & \boldsymbol {S} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {p} \\ \boldsymbol {\omega} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {I} _ {n} \\ 0 _ {m \times n} \end{array} \right] \boldsymbol {H} + \left[ \begin{array}{c} \boldsymbol {e} _ {p} \\ \boldsymbol {e} _ {\omega} \end{array} \right]\tag{26}
$$

where $I _ { n }$ is the n-th order identity matrix, and 0 denotes the zero matrix. The observed value of the generalized momentum for the robot system is: 

$$
\widehat {\boldsymbol {p}} = \left[ \begin{array}{c c} \boldsymbol {I} _ {n} & 0 _ {n \times m} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {p} \\ \boldsymbol {\omega} \end{array} \right] + \boldsymbol {\nu}\tag{27}
$$

where $\pmb { \nu } \sim N ( 0 , \pmb { R } _ { c } )$ represents the measurement noise of the generalized momentum, and $\pmb { R } _ { c } \in \Re ^ { n \times n }$ is the corresponding noise covariance matrix. 

By defining the system state variable as $\pmb { x } = \left[ \pmb { p } ^ { T } \pmb { \omega } ^ { T } \right] ^ { T } \in \Re ^ { n + m }$ , the observation system state equation can be simplified as: 

$$
\left\{ \begin{array}{c} \dot {\boldsymbol {x}} = \boldsymbol {A} _ {c} \boldsymbol {x} + \boldsymbol {B} _ {c} \boldsymbol {u} + \boldsymbol {w} \\ y = \boldsymbol {C} _ {c} \boldsymbol {x} + \nu \\ \boldsymbol {A} _ {c} = \left[ \begin{array}{c c} 0 _ {n \times n} & \boldsymbol {J} ^ {T} \\ 0 _ {m \times n} & \boldsymbol {S} \end{array} \right], \boldsymbol {B} _ {c} = \left[ \begin{array}{c} \boldsymbol {I} _ {n} \\ 0 _ {m \times n} \end{array} \right], \boldsymbol {C} _ {c} = [ \boldsymbol {I} _ {n} 0 _ {n \times m} ] \end{array} \right.\tag{28}
$$

where $\pmb { A } _ { c } , \pmb { B } _ { c } ,$ and $c _ { c }$ are the state-space matrices. $\pmb { w } = \left[ \pmb { e } _ { p } ^ { \tau } \pmb { e } _ { \omega } ^ { \tau } \right] ^ { T } \sim$ $N ( 0 , \pmb { Q } _ { c } )$ represents the predicted state noise, and $\pmb { Q } _ { c } = d i a g \bigg \{ \pmb { \sigma } _ { p } ^ { 2 } , \pmb { \sigma } _ { \omega } ^ { 2 } \bigg \}$ is the corresponding noise covariance matrix. 

To implement the observation state update of the KF, the continuous system (28) is discretized, yielding the discrete state equation for the k-th time step as follows: 

$$
\left\{ \begin{array}{c} \boldsymbol {x} _ {k} = \boldsymbol {A} _ {k} \boldsymbol {x} _ {k - 1} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {w} _ {k} \\ \boldsymbol {y} _ {k} = \boldsymbol {C} _ {k} \boldsymbol {x} _ {k} + \boldsymbol {v} _ {k} \end{array} \right.\tag{29}
$$

Given the sampling time $T _ { s } ,$ the discrete state-space matrices $\pmb { A } _ { k } , \pmb { B } _ { k }$ and $c _ { k }$ for the discrete system can be computed as follows: 

$$
\begin{array}{l} \left[ \begin{array}{c c} \boldsymbol {A} _ {k} & \boldsymbol {B} _ {k} \\ 0 _ {n \times (n + m)} & \boldsymbol {I} _ {n} \end{array} \right] = e x p \left(\left[ \begin{array}{c c} \boldsymbol {A} _ {c c} & \boldsymbol {B} _ {c c} \\ 0 _ {n \times (n + m)} & 0 _ {n \times n} \end{array} \right] T _ {s}\right) \\ \boldsymbol {C} _ {k} = \boldsymbol {C} _ {c} \end{array}\tag{30}
$$

For contact force estimation with KF, w<sub>k</sub> and $\nu _ { k }$ can typically be treated as mutually uncorrelated Gaussian white noise. Their covariance matrices $\pmb { Q } _ { k }$ and $\pmb { R } _ { k }$ are calculated as follows: 

$$
\begin{array}{l} \left[ \begin{array}{c c} \boldsymbol {M} _ {1} & \boldsymbol {M} _ {2} \\ 0 _ {(n + m) \times (n + m)} & \boldsymbol {M} _ {3} \end{array} \right] = e x p \left(\left[ \begin{array}{c c} \boldsymbol {A} _ {c c} & \boldsymbol {Q} _ {c c} \\ 0 _ {(n + m) \times (n + m)} & - \boldsymbol {A} _ {c c} ^ {T} \end{array} \right] T _ {s}\right) \\ \boldsymbol {Q} _ {k} = \boldsymbol {M} _ {2} \boldsymbol {M} _ {1} ^ {T}, \boldsymbol {R} _ {k} = \boldsymbol {R} _ {c} \end{array}\tag{31}
$$

where $M _ { 1 } , M _ { 2 } ,$ , and ${ \pmb M } _ { 3 }$ are intermediate matrices obtained from the Van Loan block matrix exponential used for stochastic discretization. 

The discrete system (29) satisfies the state observability requirement. The selection of the process noise covariance matrix $\pmb { Q } _ { k }$ and the measurement noise covariance matrix $\pmb { R } _ { k }$ is crucial to the performance of the KF, and both matrices typically require calibration. Here, $\sigma _ { p } ^ { 2 }$ and $\pmb { R } _ { k }$ can be estimated based on prior information or experimental testing, which is relatively straightforward. However, $\mathfrak { \sigma } _ { \omega } ^ { 2 }$ , which reflects the rate of change of external forces, is often difficult to obtain a priori in most cases. In practical engineering applications, repeated experiments are typically required to identify satisfactory values for the noise covariance matrices, achieving a balance between response speed and noise sup pression effectiveness [33]. 

After completing the state modeling of the observation system, the KF algorithm can be executed for online estimation of the contact force. At the k-th time step, the prior estimate covariance matrix of the state vector from the previous time step is denoted as $P _ { k - 1 } ,$ representing the uncertainty of the predicted state. Based on the system model and the state estimate from the previous time step, the state vector and its covariance matrix at the current time step can be predicted as follows: 

$$
\left\{ \begin{array}{l} \overline {{\boldsymbol {x} _ {k}}} = \boldsymbol {A} _ {k} \boldsymbol {x} _ {k - 1} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} \\ \overline {{\boldsymbol {P} _ {k}}} = \boldsymbol {A} _ {k} \boldsymbol {P} _ {k - 1} \boldsymbol {A} _ {k} ^ {T} + \boldsymbol {Q} _ {k} \end{array} \right.\tag{32}
$$

where $\overline { { \boldsymbol { x } _ { k } } }$ and $\overline { { \pmb { P } _ { k } } }$ represent the predicted values of the state vector and its covariance matrix at the current time step, respectively. Subsequently, the predicted values are corrected using the observation data at the current time step to obtain a more accurate posterior state estimate: 

$$
\begin{array}{l} \boldsymbol {K} _ {k} = \frac {\overline {{\boldsymbol {P} _ {k}}} \boldsymbol {C} _ {k} ^ {T}}{\boldsymbol {C} _ {k} \overline {{\boldsymbol {P} _ {k}}} \boldsymbol {C} _ {k} ^ {T} + \boldsymbol {R} _ {k}} \\ \boldsymbol {x} _ {k} = \overline {{\boldsymbol {x} _ {k}}} + \boldsymbol {K} _ {k} (\boldsymbol {y} _ {k} - \boldsymbol {C} _ {k} \overline {{\boldsymbol {x} _ {k}}}) \\ \boldsymbol {P} _ {k} = \overline {{\boldsymbol {P} _ {k}}} - \boldsymbol {K} _ {k} \boldsymbol {C} _ {k} \overline {{\boldsymbol {P} _ {k}}} \end{array}\tag{33}
$$

(35) 

where $\pmb { K } _ { k }$ is the gain matrix adjusted based on the system state. $\pmb { x } _ { k }$ represents the posterior estimate of the state vector, which is the optimal result combining prediction and observation. $\scriptstyle P _ { k }$ denotes the posterior estimate covariance matrix, characterizing the updated state uncertainty. Since the pair $( \pmb { A } _ { k } , \pmb { C } _ { k } )$ is observable and $\mathbf { \psi } _ { Q _ { k } } , \mathbf { \psi } _ { R _ { k } }$ are positive definite, the Kalman filter recursion is mean-square stable and convergent according to standard KF theory. 

Finally, the external contact force can be extracted from the state vector of the observation system: 

$$
\boldsymbol {\omega} _ {k} = \left[ \begin{array}{c c} 0 _ {m \times n} & \boldsymbol {I} _ {m} \end{array} \right] \boldsymbol {x} _ {k}\tag{34}
$$

where $\pmb { \tau } _ { e r r }$ represents the modeling error torque. 

In the process of contact force observation based on generalized momentum, the computation of the time derivative of the inertia matrix M ˙ is a primary factor determining the real-time performance of the observation. The method based on the passivity property reformulates the observer structure from Eq. (6) to Eq. (7). Here, the calculation of $C ( \pmb q , \dot { \pmb q } ) ^ { T }$ requires the use of an auxiliary variable to separate and compute the quadratic terms related to q ˙ in the Newton-Euler formu lation, which is computationally complex [34]. Additionally, this method is only applicable to standard dynamics models, where the inertia matrix of the links is expressed at the centroid of the links rather than the origin of the link coordinate frame. To overcome this limitation, studies have employed automatic differentiation techniques to directly compute M ˙ [35]. Automatic differentiation allows for branches, loops, and recursive structures, making it suitable for automated imple mentation via programming. Compared to methods based on the passivity property, automatic differentiation offers advantages in computational speed. Unlike existing approaches, the proposed PIKF calculates M ˙ in a structured manner, achieving lower inference time costs. Experimental validation is presented in Section 4.1. 

$$
\widehat {\pmb {\tau}} _ {d y n} + \widehat {\pmb {\tau}} _ {f r i} + \pmb {\tau} _ {e r r} = \pmb {\tau} _ {d r i} + \pmb {\tau} _ {e x t}
$$

## 3.2. PINN-based model uncertainty compensation

The dynamics and friction behaviors of the robot can be obtained using the method described in Section 2.2. Subsequently, the predicted values of the dynamics and friction terms $\widehat { \pmb { \tau } } _ { d y n }$ and $\widehat { \pmb { \tau } } _ { f r i }$ can be calculated based on the robot’s motion state. However, due to noise in the joint currents and other nonlinear factors that are difficult to model, the robot’s dynamics and friction models inherently exhibit modeling biases and learning errors, which introduces unavoidable model uncertainties in the estimation of external contact forces. Therefore, Eq. (1) needs to be modified as follows: 

This model uncertainty is notably multi-source and heterogeneous, posing significant challenges for designing observers that account for such uncertainties. However, for the majority of contact force estimation tasks, such as collision detection, preventing sudden and large fluctuations in the observed contact force is a core requirement to ensure that safety thresholds are not inadvertently triggered by non-collision behaviors. Existing research indicates that during robot joint reversals, joint backlash caused by manufacturing tolerances, wear, and assembly errors can lead to substantial deviations between the actual and esti mated joint torques [36]. Fig. 3 illustrates the joint velocity and torque error curves during a segment of the robot’s motion, where the torque error denotes the deviation between the torque computed from the robot dynamics model and the joint torque estimated from motor current feedback, revealing a clear correlation between the two variables. During the transition of velocity direction, large amplitude backlash-induced dynamic errors will occur, which not only affect the real-time control performance but also lead to misjudgments of external forces, thereby compromising task reliability and safety. 

Therefore, this article predicts the occurrence of robot reversal behaviors in advance based on the robot’s historical motion states. Subsequently, a generalized error model is designed using Gaussian basis functions, and a PINN is constructed to achieve real-time compensation for backlash-induced errors in the contact force estimation task. 

The characteristics of backlash-induced dynamic errors exhibit a strong temporal correlation with the joint motion state. However, directly predicting whether backlash-induced dynamic errors will occur at the next time step based solely on the current robot state is unreliable, as the same joint state at a given time may belong to different motions. Additionally, it is necessary to detect the onset of backlash errors in their initial stage, when significant error peaks are typically absent. Therefore, this article incorporates historical motion information of the joints to enhance the stability of the prediction model. A temporal convolutional network (TCN) implemented using stacked one-dimensional convolutions is selected as the backbone of the designed temporal prediction module. 

Define the backlash error occurrence label $\kappa \in [ 0 , 1 ]$ as the target output of the temporal prediction module, where $\kappa = 1$ indicates that the robot undergoes a reversal and generates backlash-induced dynamic errors at that time step. A sliding window of length l is utilized to process the kinematic quantities q, q˙, q¨, thereby obtaining the input for the temporal prediction module: 

$$
\begin{array}{l} \boldsymbol {X} _ {t} = [ \boldsymbol {Q} _ {t - l + 1}, \dots , \boldsymbol {Q} _ {t} ] \\ \boldsymbol {Q} _ {t} = \left[ q _ {t}, \dot {q} _ {t}, \ddot {q} _ {t} \right] ^ {T} \end{array}\tag{36}
$$

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/8855714450234a4e8b1bfaf7165c895c513b3f75ddf13ce25a12605be3c95764.jpg)



Fig. 3. Backlash-induced dynamic errors and joint velocity during a segment of robot motion.


where $\mathbf { Q } _ { t }$ denotes the robot state vector at time t and the input $\pmb { X } _ { t } \in \Re ^ { 3 \times l }$ is the combination of continuous state vectors over a past duration of length l. 

The historical information within the input is captured using a stack structure of one-dimensional convolutions combined with dilated con volutions, expressed as follows: 

$$
\boldsymbol {y} (t) = \left(\boldsymbol {x} _ {d} ^ {*} \boldsymbol {f}\right) (t) = \sum_ {i = 0} ^ {k - 1} x (t - d i) \cdot f (i)\tag{37}
$$

where $^ * { d }$ denotes the dilated convolution operation, $\pmb { x } \in \Re ^ { n \times 1 }$ is the input data, $\pmb { f } \in \Re ^ { k \times 1 }$ signifies the dilated convolutional filter, k is the filter size, and d is the dilation factor. 

Thus, the backlash error occurrence label can be predicted as: 

$$
\widehat {\boldsymbol {\kappa}} = \operatorname{TCN} (\boldsymbol {q}, \dot {\boldsymbol {q}}, \ddot {\boldsymbol {q}})\tag{38}
$$

When the predicted label at a certain time step changes from 0 to a non-zero value, it indicates the occurrence of backlash-induced dynamic errors. This enables both the early temporal prediction and the categorical prediction of joint errors. Next, Gaussian basis functions are employed to fit the backlash-induced dynamic errors, completing the modeling of individual error instances. Subsequently, a PINN is utilized to establish a mapping relationship between the robot joint motion and the backlash-induced dynamic errors, enabling online error prediction and compensation. 

First, the Gaussian basis function is defined as follows: 

$$
\Phi (t) = a \cdot \exp \left(- \frac {(t - b) ^ {2}}{c ^ {2}}\right)\tag{39}
$$

where $a , b , c$ represents the parameters that determine the shape of the Gaussian basis function. 

By learning and updating these parameters, the Gaussian basis function can accurately fit the errors. Since the length of an individual error sample in the time domain is finite, the Gaussian basis function can be modified as: 

$$
\Phi_ {j} (S _ {j}, \delta) = \Phi_ {j} (t), t \in B (S _ {j}, \delta)\tag{40}
$$

where $B ( S _ { j } , \delta )$ denotes the j-th error persistence interval, specifically the j-th continuous interval where the error label κ is non-zero. δ serves as a scaling factor for interval length. 

The mapping relationship between the robot motion states and the shape parameters of the Gaussian basis functions is learned through a multilayer perceptron (MLP) module: 

$$
(\widehat {a}, \widehat {b}, \widehat {c}) = \operatorname{MLP} (q, \ddot {q}; \vartheta)\tag{41}
$$

where $\widehat { \boldsymbol { a } } , \widehat { \boldsymbol { b } } , \widehat { \boldsymbol { c } }$ represent the predicted values of the parameters of the Gaussian basis function. The generalized joint displacement $q ,$ which corresponds to the joint angular position, together with the angular acceleration $\ddot { q } ,$ serve as the inputs to the module. The parameter ϑ denotes the network weight updated during training. The joint position primarily determines the temporal shift of the backlash error, whereas the acceleration governs the impact intensity and thus the peak amplitude of the error. The joint velocity q ˙ is intentionally excluded because backlash errors occur at motion reversal instants, where the velocity approaches zero and therefore provides limited informative value for predicting the error shape, as illustrated in Fig. 3. 

Therefore, the predicted value of the j-th individual error sample can be expressed as: 

$$
\widehat {\Phi} _ {j} (t) = \Phi_ {j} \big (t, \widehat {a} _ {j}, \widehat {b} _ {j}, \widehat {c} _ {j} \big), t \in B \big (S _ {j}, \delta \big)\tag{42}
$$

Finally, the predicted values of backlash-induced dynamic error over a period of motion can be obtained as follows: 

$$
\widehat {\pmb {\tau}} _ {e r r} = \Sigma_ {j = i} ^ {m} \widehat {\Phi} _ {j} (t)\tag{43}
$$

Additionally, since backlash-induced dynamic errors typically exhibit higher peak values and a sparser temporal distribution compared to dynamic errors caused by other factors, the loss function of the PINNbased model is defined as: 

$$
L o s s = \frac {1}{N} \sum_ {i = 1} ^ {N} \omega_ {i} (\boldsymbol {\tau} _ {e r r} - \widehat {\boldsymbol {\tau}} _ {e r r}) ^ {2}\tag{44}
$$

$$
\omega_ {i} = \left(\pmb {\tau} _ {e r r} - \widehat {\pmb {\tau}} _ {e r r}\right) ^ {2} + \sigma
$$

where $\omega _ { i }$ represents the weighting coefficients of the loss function, which emphasize the influence of errors with larger amplitudes, thereby achieving a better approximation of backlash-induced dynamic errors. σ denotes a small positive constant close to zero, introduced to prevent computational errors when the error value is zero. 

The proposed method predicts the parameters of the error model through the PINN rather than directly predicting the error values. This approach effectively reduces the complexity of the mapping relationship, making the proposed error compensation method more robust and easier to learn. 

By incorporating the predicted backlash error values into the observation formula as Eq. (35) and correcting the computation of generalized momentum as Eqs. (19) and (20), the corrected predicted value of generalized momentum $\widehat { \pmb { p } }$ is obtained as: 

$$
\widehat {\boldsymbol {p}} = \left(\widehat {\boldsymbol {L}} \frac {\partial \widehat {\boldsymbol {L}} ^ {T}}{\partial \boldsymbol {q}} + \frac {\partial \widehat {\boldsymbol {L}}}{\partial \boldsymbol {q}} \widehat {\boldsymbol {L}} ^ {T}\right) \dot {\boldsymbol {q}} ^ {2} + \boldsymbol {\tau} _ {d r i} + \boldsymbol {\tau} _ {e x t} - \widehat {\boldsymbol {\tau}} _ {f r i} - \widehat {\boldsymbol {\tau}} _ {e r r} - \widehat {\boldsymbol {\tau}} _ {d y n} (\boldsymbol {q}, \dot {\boldsymbol {q}}, 0)\tag{45}
$$

In fact, the aforementioned modeling and compensation of backlash errors represent a further refinement of robot dynamics. Unlike most existing semi-parametric modeling and compensation methods [37], the designed PINN-based model uncertainty compensation does not target the residuals of the robot dynamics model but rather the observation residuals caused by the propagation of dynamics model residuals during the observation process. The proposed method for handling model uncertainties is implemented within the observation process. When observing external contact forces on the robot system, the torque errors caused by model uncertainties are treated as internal disturbances and incorporated into the observer update iterations as Eqs. (32) and (33). This approach avoids the inevitable loss associated with the propagation of offline model learning into online force estimation, thereby enhancing the accuracy and robustness of external force disturbance estimation. 

## 3.3. Noise propagation analysis and AKF design

Due to the lack of joint force sensing in industrial robots, the join driving torque $\pmb { \tau } _ { d r i }$ is typically estimated using Eq. (3). However, the presence of significant noise in the current signal necessitates setting $\mathfrak { \sigma } _ { \omega } ^ { 2 }$ to a large value to ensure satisfactory tracking performance of the KF in contact force observation. This inevitably results in noisy estimates when no external force is applied and requires a higher contact force detection threshold, thereby compromising practical application effectiveness. 

Therefore, real-time online filtering of the robot joint current signals is a potential solution to suppress current noise. The mode-switching moving average based on a variable time period has been introduced into the KF to address this issue, resulting in the AKF [23]. However, unfortunately, this method is no longer applicable in cases with significant model uncertainties. 

When online filtering is applied to the robot joint current signals, both the noise in the current and a portion of the model uncertainty components, such as the backlash-induced dynamic errors as described in Section 3.2, are filtered out. This makes it impossible to isolate the internal disturbance terms in the observer caused by model uncertainties, thereby complicating the detection of phenomena like backlash. Therefore, it is necessary to analyze the error propagation during the robot contact force observation process and construct an auxiliary observation variable to further filter out noise without compromising the compensation for model uncertainties. 

The detailed introduction of the WMA algorithm can be found in the literature [38]. The application of WMA to a noisy measurement signal s can be expressed as: 

$$
s _ {d} ^ {\prime} = W (s, d, z) = \frac {2 \left[ s _ {d} z + s _ {d - 1} (z - 1) + \cdots + s _ {d - z + 1} \right]}{z (z + 1)}\tag{46}
$$

where $s _ { d }$ and $s _ { d } ^ { ' }$ represent the values of the signal s before and after filtering at time step $d ,$ respectively. z is the number of samples averaged in each step. Letting the length of each time step be $\Delta t ,$ the length of the variable time domain is zΔt. 

If the signal undergoes significant changes, it is necessary to adjust the time domain of the WMA to minimize the loss caused by filtering high-frequency signals. Assuming the time difference between any two sudden changes is greater than zΔt, when a signal abrupt change is detected at time r, i.e., $| s _ { d } - s _ { d - 1 } ^ { ' } | > \varepsilon ,$ the WMA is adjusted as follows for the next z time steps: 

$$
s _ {d} ^ {\prime} = W (s, d, d - r + 1)\tag{47}
$$

where ε is an experimentally determined threshold parameter for time period adjustment. 

Considering the presence of model uncertainties, this article opts not to apply WMA filtering directly to the joint current signals. Instead, an auxiliary observation variable $H _ { e }$ is constructed during the observation process to separate model uncertainties from current noise before applying WMA filtering. The auxiliary variable $H _ { e }$ is obtained by isolating uncertainties from the auxiliary variable as Eq. (21), and its predicted value during the observation process can be expressed as: 

$$
\widehat {\boldsymbol {H}} _ {e} = \left(\widehat {\boldsymbol {L}} \frac {\partial \widehat {\boldsymbol {L}} ^ {T}}{\partial \boldsymbol {q}} + \frac {\partial \widehat {\boldsymbol {L}}}{\partial \boldsymbol {q}} \widehat {\boldsymbol {L}} ^ {T}\right) \dot {\boldsymbol {q}} ^ {2} + \boldsymbol {\tau} _ {d r i} - \widehat {\boldsymbol {\tau}} _ {f r i} - \widehat {\boldsymbol {\tau}} _ {e r r} - \widehat {\boldsymbol {\tau}} _ {d y n} (\boldsymbol {q}, \dot {\boldsymbol {q}}, 0)\tag{48}
$$

This auxiliary variable can be interpreted as the residual part of the joint momentum derivative after removing the joint torque caused by external forces. In essence, it represents the comprehensive quantity involved in the observation process excluding the observed variable. The result of applying WMA filtering to this comprehensive quantity at time step d can be expressed as: 

$$
\widehat {H} _ {e d} ^ {\prime} = W (\widehat {\pmb {H}} _ {e}, \boldsymbol {d}, \pmb {z})\tag{49}
$$

Substituting into Eq. (45), the corrected prediction of generalized momentum $\widehat { \pmb { p } }$ is obtained as: 

$$
\widehat {\dot {\pmb {p}}} = \widehat {\pmb {H}} _ {e} ^ {'} + \pmb {\tau} _ {e x t}\tag{50}
$$

By incorporating this predicted value into the PIKF described in Section $3 . 1 _ { \cdot }$ , the final adaptive observer PIAKF, which considers model uncertainties and noise suppression, can be realized. The specific algorithm flow of the proposed PIAKF is outlined in Algorithm 1. 

## 4. Performance evaluation and analysis

In this section, the advantages of the proposed PIAKF in terms of observation performance and inference cost are validated through numerical simulations. Subsequently, the effectiveness of the proposed uncertainty compensation and noise suppression methods is verified in real contact force estimation tasks on the SIASUN SN7B 6-DOF serial industrial robot. Furthermore, the proposed PIAKF is compared against other advanced methods with the corresponding evaluation and analysis. 

## Algorithm 1

The proposed PIAKF algorithm.

Input: Joint drive gain: $K_{m}$ Joint motion and current data: q, $\dot{q}$ , $\ddot{q}$ , $I_{m}$ The constructed temporal data←Eq. (36)

Output: Contact force estimation: $\hat{\omega}$ Joint external torque estimation: $\hat{\tau}_{ext}$ 1 START contact force observation

2 Compute $\tau_{dri} \leftarrow Eq. (3)$ 3 Compute $\tau_{dyn}$ based on PINNs←Eq. (16)

4 Compute $\tau_{fri}$ based on PINNs←Eq. (17)

5 Initialize state estimation←Eq. (23)-(28)

6 Repeat

7 Compute $\dot{M}$ based on PINNs←Eq. (11)

8 Compute $\dot{p}$ based on PINNs←Eq. (20)

9 Predict the backlash-induced uncertainty←Eq. (38)

10 If backlash-induced uncertainty occurs: $\hat{\kappa} = 1$ 11 Compute backlash-induced errors $\hat{\tau}_{err} \leftarrow Eq. (43)$ 12 Revise $\hat{p}$ by considering uncertainty←Eq. (45)

13 End

14 Revise the auxiliary term $\hat{H}_{e} \leftarrow Eq. (48)$ 15 Perform WMA filtering operation on $\hat{H}_{e} \leftarrow Eq. (49)$ 16 Compute the discrete-time state←Eq. (29)-(31)

17 Compute state and covariance estimation←Eq. (32)

18 Compute optimal Kalman gain←Eq. (33)

19 Update the posterior state estimation←Eq. (33)

20 Implement external contact force estimation←Eq. (34)

21 Until observation is terminated

22 END 

## 4.1. Numerical simulations

The numerical simulation is implemented using MATLAB Simscape with a sampling and control frequency of 1 kHz. In the simulation, only the velocity-dependent viscous term is considered for joint friction. The performance of the PIAKF is compared with several advanced techniques, as detailed in Table 1. Among these, MESO is a variant combining ESO and GMO, SOSM is a sliding mode momentum observer, while HODO and HOFTO are high-order disturbance observers based on ESO. The parameter selection for AKF, SWVAKF and PIAKF refers to the existing literature of KF [23]. The covariance matrix of the predicted state noise is set to ${ \sigma } _ { p } ^ { 2 } = 3 0 0 I _ { 6 }$ and $\sigma _ { \omega } ^ { 2 } = 0 \sb { 6 \times 6 } ,$ respectively. The measurement noise covariance matrix is set to $\pmb { R _ { k } } = \mu \pmb { M } \pmb { M } ^ { T }$ , where M is the inertia matrix, and $\mu = 1 e - 9$ . The parameters of the ESO are selected based on the literature [39], with the parameters set to $\beta =$ $\left[ \begin{array} { l l l } { 3 \omega _ { 0 } } & { 3 \omega _ { 0 } ^ { 2 } } & { \omega _ { 0 } ^ { 3 } } \end{array} \right]$ and ω chosen as 40. The gain parameters of the other observers are determined by ensuring that the observers have comparable bandwidths. 

First, the estimation performance of the robot for single-dimensional contact forces under static conditions is tested. A sinusoidal force with a period of 0.5 s and an amplitude of 20 N is applied in the X-direction. The external contact force estimation results and estimation residuals for each method are shown in Fig. 4(a) and Fig. 4(b), respectively, where the curve labeled $\mathbf { \tilde { \Gamma } } ^ { \mathrm { s } } \mathbf { \tilde { R } } \mathbf { e } \mathbf { f } ^ { \mathrm { s } }$ represents the reference values. The results indicate that the AKF and SWVAKF method, which reasonably accounts 

## Table 1


Comparison of contact force estimation methods


<table><tr><td>Methods</td><td>Publication</td><td>Technique</td></tr><tr><td>GMO</td><td>De Luca et al. [40]</td><td>Generalized momentum observer</td></tr><tr><td>ESO</td><td>Han et al. [41]</td><td>Extended state observer</td></tr><tr><td>MESO</td><td>Ren et al. [16]</td><td>Modified second-order ESO</td></tr><tr><td>SOSM</td><td>Garofalo et al. [8]</td><td>Sliding mode momentum observer</td></tr><tr><td>HODO</td><td>Zhang et al. [42]</td><td>High-order disturbance observer</td></tr><tr><td>HOFTO</td><td>Han et al. [14]</td><td>High-order finite-time observer</td></tr><tr><td>AKF</td><td>Cao et al. [23]</td><td>Adaptive Kalman filter</td></tr><tr><td>SWVAKF</td><td>Dong et al. [24]</td><td>Sliding window variational adaptive Kalman filter</td></tr><tr><td>PIAKF</td><td>Proposed</td><td>Physics-informed adaptive Kalman filter</td></tr></table>

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/6eaca2ba3414acfc018462935cded2947cd8d21069421304d61161f60b28cc6d.jpg)



(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/f7ac26459544fed1b9ec7030c3c3ea2a33742effff24abdfd625445f2ebf94c2.jpg)



(b)



Fig. 4. Contact force estimation results for sinusoidal signals. (a) Comparison of observation results. (b) Comparison of estimation residuals.


for disturbances, outperforms other observers with less phase lag. The proposed PIAKF, which constructs the observer based on PINN, benefits from the nonlinear learning capability of the network model and further reduces contact force estimation errors. 

When the robot is in motion, accurate observation of contact forces becomes more challenging. Additionally, the impact of multidimensional generalized external contact forces requires further investigation. A six-dimensional generalized force (20 N in force and 2 N⋅m in torque) is applied to the robot end-effector. The external torque components on each joint can be derived from the external contact forces. Taking joints 1–3 as examples, the external torque components estimated by various methods and the estimation residuals are shown in Fig. 5(a) and Fig. 5(b), respectively. 

Furthermore, the proposed method is also tested in a general material-handling task. The robot executed the trajectory shown in Fig. 6. The observed external torque components obtained by the proposed method and other comparative methods are illustrated in Fig. 7. The average observation performance across all joints is quantified using the root mean squared error (RMSE) and the mean absolute error (MAE), as illustrated in Fig. 8. In non-static conditions, the proposed PIAKF still achieves more accurate observations than other methods. While HODO and SOSM achieve more effective observations compared to GMO and ESO, their results contain high-frequency minor disturbances, which are caused by the complexity of the higher-order parameter settings. In addition, the performance of SWVAKF is also superior to traditional methods, but it is limited due to its dependence on friction priors. 

In KF-based observers for robot contact force estimation combined with generalized momentum, the derivative operation of the robot inertia matrix is the primary factor determining the real-time performance of the observer. The proposed PINN-based improved KF calcu lates M ˙ in a structured manner through PINN. Its computational cost is compared with methods based on passivity property [34] and automatic differentiation [35], as detailed in Table 2. The results show that although other AKF methods can also achieve excellent observation performance, the proposed PIAKF significantly reduces observation cost while maintaining observation accuracy. On the same computer equipped with a single NVIDIA RTX 4060 GPU (16 GB), the proposed method achieves an inference time of 2.50e−06 s per sample, corresponding to only 3.24 % and 10.29 % of the time required by the recursive Newton-Euler numerical algorithm and automatic differentiation tech nique, respectively. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/0e490c6c0a9ef223ba72bb203fcfface9b1fa685104105be4e2499dd91206b02.jpg)



(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/e653e4fd1cbd6d4e02a9ed815b91cb5aeca1459caa03200c2118e1532692ebeb.jpg)



(b)



Fig. 5. Joint external torque component estimation results under multi-dimensional external forces during robot motion. (a) Comparison of observation results. (b) Comparison of estimation residuals.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/63b498e1ffd26f2273abec7f99e47e73d46fc7ad077e4b4b4ef7928cc5dfb021.jpg)



Fig. 6. Robot material handling task.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/3d270e4c8ea2fcdf1d4e72170ac237c476b7d41ffa2011fe8c61b3af6b6eb791.jpg)



Fig. 7. Joint external torque component estimation results in robot material handling tasks.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/3098d2ff84ae6ecd33908e49e7636fe4a00995df2d76b0aca1e7fefeedd65d40.jpg)



Fig. 8. Estimation performance comparison with other methods evaluated by RMSE and MAE.



Table 2



Comparison of computation time for inertia matrix derivatives.


<table><tr><td>Methods</td><td>Techniques</td><td>Computation time (s)</td></tr><tr><td>De Luca et al. [34]</td><td>Recursive Newton-Euler numerical algorithm</td><td>7.72e-05</td></tr><tr><td>Baydin et al. [35]</td><td>Automatic differentiation technique</td><td>2.43e-05</td></tr><tr><td>Proposed</td><td>PINN-based modular computational technique</td><td>2.50e-06</td></tr></table>

## 4.2. Experimental evaluation

The experimental setup of the robotic system is displayed in Fig. 9. The SIASUN SN7B 6-DOF serial industrial robot is used for experimental testing. All joints provide feedback on joint position, velocity, and motor current. A Beckhoff C6930 industrial-grade computing platform is employed as the core control unit, enabling real-time control and data acquisition at 1 kHz through EtherCAT. The observer algorithm and the robot dynamics model obtained from PINN training are compiled in MATLAB Simulink and imported into the TwinCAT environment, then deployed in the actual robot control scenario using the Beckhoff TE1400 module. A six-axis force/torque sensor (SRI M4313M4E) is installed at the robot’s end-effector to measure the reference information of external contact forces. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/ef7fc1385887b93a85b220595e03b5b220284ec5e2577760e708b844f596cdf0.jpg)



Fig. 9. Diagram of the robotic experimental platform.


To ensure the effective generalization of the IDM and inertia matrix inference, the collected robot motion data must cover the entire workspace and exhibit comprehensive motion characteristics. Therefore, the fifth-order Fourier series is selected as the excitation trajectory for dy namics modeling [43], specifically as follows: 

$$
q _ {i} (t) = q _ {i, 0} + \sum_ {k = 1} ^ {5} \left(\frac {a _ {i , k}}{k \omega_ {f}} \sin (k \omega_ {f} t) - \frac {b _ {i , k}}{k \omega_ {f}} \cos (k \omega_ {f} t)\right)\tag{51}
$$

where t is a certain moment $, \omega _ { f }$ is the fundamental frequency, $a _ { i , k }$ and $b _ { i , k }$ are the k-order trajectory coefficients of the i-th joint, and $q _ { i , 0 }$ denotes the initial position. Typically, $\omega _ { f }$ is set to 0.1 Hz. The trajectory coefficients are determined through an iterative optimization technique [44]. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/2a2c3fb4631f3b698ae1a7d3fdfafaf0af9e741064846d3af2cf46ba20c759de.jpg)


The real motion dataset collects joint positions, velocities, and motor currents from 12 excitation trajectories, each lasting 10 s. Raw data are filtered using a bidirectional fifth-order Butterworth low-pass filter with a cutoff frequency of 50 Hz. Joint accelerations are obtained through numerical differentiation of velocities. Joint driving torques are estimated via Eq. (3). The dataset comprises a total of 120,000 samples, which are randomly split into training, validation, and test subsets with proportions of 60 %, 20 %, and 20 %, respectively. The constructed PINN-based dynamics modeling network consists of three hidden layers, with the number of nodes set to [128 128 128]. Adaptive moment estimation (Adam) is used as the optimizer, and the ReLU serves as the activation function. Regarding the network hyperparameters, the training batch size is set to 50, the learning rate is set to 5e−4, and the sparsity probability is set to 0.2. After training, cross-validation experiments are conducted on the test trajectories, which are outside the training set. Without loss of generality, the torque modeling results for the robot joints predicted by the PINN-based method are shown in Fig. 10. The results demonstrate the effectiveness of the PINN-based dynamics modeling approach, as well as its generalization capability across different trajectories. 


(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/3d7b3d42edce08f77fad35a7d61fc4368324aa7a10f44ceee101a7fd32f84e37.jpg)



(b)



Fig. 10. Joint torque prediction results of the SIASUN SN7B utilizing PINN-based robot dynamics modeling. (a) Joint torque prediction. (b) Joint torque predic tion residuals.


The real contact estimation experiments evaluate two scenarios: (1) static tests with manual random disturbances comparing PIAKF against other advanced methods listed in Table 1; and (2) dynamic tests with payloads during robot motion, assessing PIAKF’s handling of joint backlash-induced uncertainties and disturbance propagation effects on noise processing. 

In the first scenario of experiments, the robot is maintained in a stationary state. Random external forces, including both pushing and collision, are manually applied to the robot, as shown in Fig. 11(a). The proposed PIAKF’s performance in estimating external forces along the X-$Y _ { \overline { { \mathbf { \Gamma } } } }$ , and Z-axes at the robot end-effector is compared with other methods. The force signals acquired by the force/torque sensor are calibrated and served as reference values for the external forces. Fig. 12 presents the external force estimation results when the robot is subjected to pushing forces, and displays the corresponding magnified views. The experimental results demonstrate the reliability of the proposed method for practical contact force estimation tasks. 

On the other hand, unlike the case of applied pushing forces, the external force signals exhibit sudden and significant changes when an instantaneous collision occurs at the robot end-effector. This scenario is used to evaluate the robustness of the proposed PIAKF under extreme conditions. The external force estimation results during robot collisions are presented in Fig. 13. During multiple time intervals (e.g., 5.2–5.4 s), the external force signals exhibited sudden increases due to collisions. Compared with other methods, the proposed PIAKF demonstrates faster tracking of external force variations, making it particularly suitable for demanding collision detection applications. All methods are quantitatively compared using RMSE, as summarized in Table 3. The RMSE is calculated as: 

$$
R M S E = \sqrt {\sum_ {i = 1} ^ {N} \left(\widehat {\boldsymbol {F}} _ {i} - \boldsymbol {F} _ {i} ^ {r e f}\right) ^ {2} / N}\tag{52}
$$

where $\widehat { \pmb F } = \left[ \widehat { \pmb F } _ { x } \quad \widehat { \pmb F } _ { y } \quad \widehat { \pmb F } _ { z } \right] ^ { T } \in \mathfrak { R } ^ { 3 \times N }$ denotes the observed external contact forces of the robot along the X-, Y-, and Z-axes. $\pmb { F } ^ { r e f } =$ $\left\lceil F _ { x } ^ { r e f } \quad F _ { y } ^ { r e f } \quad F _ { z } ^ { r e f } \right\rceil ^ { T } \in \Re ^ { 3 \times N }$ denotes the reference values for the external forces acquired by the six-axis force/torque sensor along the three channels. 

Under static conditions, joint backlash-induced model uncertainty remains negligible, allowing other methods to obtain acceptable accu racy, though the proposed PIAKF achieves superior performance. To demonstrate the effectiveness of the PIAKF in handling model uncertainties, the observation performance during robot motion is further tested as shown in Fig. 11(b). The robot performs reciprocating motion along a straight line between points $P _ { 1 }$ and $P _ { 2 } .$ . Different payloads, including 3 kg and 6 kg, are attached to the end-effector for generalization testing, while the rated load of the robot is 7 kg. Owing to the robot’s kinematic configuration and payload mounting arrangement, joints 2, 3, and 5 function as the primary torque-bearing components, exhibiting significant torque variations from the external force applied at the end-effector. Notably, model uncertainty caused by joint backlash becomes significantly pronounced during joint motion reversal. 

Four different cases including PIAK, PIAK-C, PIAKF-CW1 and PIAKF-CW2 are implemented to estimate the external torque components of joints 2, 3 and 5 with detailed explanations provided in Table 4. Among them, PIAK and PIAK-C serve as comparison baselines in the ablation experiments, while PIAKF-CW2 represents the proposed method. PIAKF-CW1 performs the model uncertainty compensation while applying WMA to the joint currents, which is consistent with the filtering approach used in existing force estimation methods. In contrast, the proposed PIAKF-CW2 applies WMA to the auxiliary variable $\widehat { \pmb { H } } _ { e }$ constructed in Eq. (48). Both methods employ identical online filtering parameter configurations. 

Fig. 14 presents the external torque component observation results $\widehat { \pmb { \tau } } _ { e x t }$ for joints 2, 3, and 5 obtained using different cases. The curve labeled “Ref” represents the reference values, which are obtained by converting measurements of the six-axis force/torque sensor through Eq. (2). It is noteworthy that during the time interval from 2.4 to 2.8 s, the joints undergo motion reversal, where the model uncertainty induced by 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/29e0eb98befe48ab996cccbbb23172f21c7f4d1b863b8ec9a8403f82e05c173f.jpg)



(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/f2fe7657f6a4a318ef9756c59058204c5bb66505d6ea7062a536a32e0011dc3b.jpg)



(b)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/eaafbd4464962ecbee3fbc5c946792456299e6d414fab4ab67f15bd708d32198.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/13e01cf5e3bbd24dce70deb8a50cf16d0e87f92d9a97e0b920cd2d91217fdfdc.jpg)



(c)



Fig. 11. Hardware setup of the robotic platform for (a) random disturbance estimation, (b) constant disturbance estimation and (c) general material-handling task.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/6c3f7fe3b8c0f31d1ba2cf882a41f63434163e239b56d98c8aa428043fe04fd3.jpg)



Time (s)



Fig. 12. Comparison of external force estimation results under pushing with other methods.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/eecbb335233f2d0a279c012de2b8ac1a4a76540281f1d7cc367c5855e28f815b.jpg)



Time (s)



Fig. 13. Comparison of external force estimation results under collision with other methods.


joint clearances causes rapid torque variations, making accurate force estimation particularly challenging for other methods. Benefiting from compensation for backlash-induced dynamic errors, the PIAK-C still performs excellent external force observation and maintains generalization across different payloads. 

Furthermore, the propagation process of noise disturbances during observation and its impact on force estimation performance are investigated. Due to model uncertainties, the reliability of directly applying noise filtering to joint currents cannot be guaranteed. Unlike the approach in [38], the proposed method first constructs the auxiliary observation variable in Eq. (48) and then applies WMA filtering to it. 

This treats model uncertainty as an internal disturbance, separating it from current noise to ensure effective suppression. Without loss of generality, Fig. 15 compares force observation performance between the proposed method (PIAKF-CW2) and direct current filtering approach (PIAKF-CW1) for joint 2 with 3 kg payload. Results indicate that while both methods filter torque signal noise effectively, direct suppression of current noise fails to enhance force observation performance under significant model uncertainty and even cause estimation failure with increased errors. In contrast, the proposed method accounts for the interaction between model uncertainty and noise propagation, ulti mately achieving superior force observation accuracy. 


Table 3



External force estimation performance comparison evaluated by RMSE.


<table><tr><td rowspan="2">Method</td><td colspan="3">Pushing</td><td colspan="3">Collision</td><td rowspan="2">Average (N)</td></tr><tr><td>X-axis (N)</td><td>Y-axis (N)</td><td>Z-axis (N)</td><td>X-axis (N)</td><td>Y-axis (N)</td><td>Z-axis (N)</td></tr><tr><td>ESO</td><td>0.49</td><td>1.14</td><td>5.68</td><td>0.45</td><td>0.87</td><td>4.75</td><td>2.23</td></tr><tr><td>HOFTO</td><td>0.44</td><td>0.92</td><td>4.16</td><td>0.42</td><td>0.81</td><td>3.92</td><td>1.78</td></tr><tr><td>MESO</td><td>0.39</td><td>0.84</td><td>3.91</td><td>0.39</td><td>0.71</td><td>3.69</td><td>1.65</td></tr><tr><td>GMO</td><td>0.24</td><td>0.49</td><td>2.01</td><td>0.27</td><td>0.45</td><td>2.46</td><td>0.99</td></tr><tr><td>SOSM</td><td>0.13</td><td>0.16</td><td>7.78</td><td>0.22</td><td>0.31</td><td>4.52</td><td>2.19</td></tr><tr><td>HODO</td><td>0.08</td><td>0.11</td><td>3.42</td><td>0.21</td><td>0.30</td><td>3.33</td><td>1.24</td></tr><tr><td>AKF</td><td>0.14</td><td>0.25</td><td>0.76</td><td>0.21</td><td>0.27</td><td>2.16</td><td>0.63</td></tr><tr><td>SWVAKF</td><td>0.18</td><td>0.33</td><td>1.02</td><td>0.25</td><td>0.33</td><td>2.37</td><td>0.75</td></tr><tr><td>Proposed</td><td>0.08</td><td>0.14</td><td>0.41</td><td>0.15</td><td>0.17</td><td>1.62</td><td>0.43</td></tr></table>


Table 4



Comparison of four different test cases.


<table><tr><td>Case</td><td>Description</td></tr><tr><td>PIKF (Comparison)</td><td>Without model uncertainty compensation</td></tr><tr><td>PIKF-C (Comparison)</td><td>With model uncertainty compensation</td></tr><tr><td>PIAKF-CW1 (Comparison)</td><td>With model uncertainty compensation and WMA filtering on joint currents</td></tr><tr><td>PIAKF-CW2 (Proposed)</td><td>With model uncertainty compensation and WMA filtering on auxiliary variable <eq>\widehat{H}_{e}</eq></td></tr></table>

The observation performance across four cases is quantitatively evaluated using both RMSE and standard deviation of second-order differences (SD2), as shown in Table 5, where RMSE measures the average tracking error and SD2 evaluates the signal smoothness to characterize noise suppression effectiveness. The RMSE and SD2 metric are calculated as follows respectively: 

$$
R M S E = \sqrt {\sum_ {i = 1} ^ {N} \left(\widehat {\boldsymbol {\tau}} _ {e x t , i} - \boldsymbol {\tau} _ {e x t , i} ^ {r e f}\right) ^ {2} / N}\tag{53}
$$

$$
S D 2 = \operatorname{Std} \left(\nabla^ {2} \widehat {\tau} _ {e x t} ^ {k}\right) = \sqrt {\frac {1}{N - 1} \sum_ {i = 1} ^ {N} \left(\widehat {\tau} _ {e x t , i + 1} ^ {k} - 2 \widehat {\tau} _ {e x t , i} ^ {k} + \widehat {\tau} _ {e x t , i - 1} ^ {k}\right) ^ {2}}\tag{54}
$$

where $\widehat { \pmb { \tau } } _ { e x t } \in \Re ^ { 6 \times N }$ represents the observed external torque component at each joint and $\widehat { \tau } _ { e x t } ^ { k }$ is the torque component at the kth joint. $\pmb { \tau } _ { e x t } ^ { r e f } \in \Re ^ { 6 \times N }$ denotes the reference values of the joint external torque components, which are obtained by transforming the six-channel signals ω acquired from the six-axis force/torque sensor through the Jacobian transformation as $\operatorname { E q } .$ . (2). Std(⋅) represents the standard deviation for unbiased estimation, $\nabla ^ { 2 } ( \cdot )$ denotes the second-order difference operation and N is the total number of samples in the torque signal. 

Quantitative analysis indicates that when model uncertainties such as backlash cannot be ignored, the direct current filtering approach (PIAKF-CW1: 4.14 N⋅m and 4.35 N⋅m) exhibits a significant degradation in estimation accuracy, performing worse than both the proposed method (PIAKF-CW2: 2.48 N⋅m and 2.88 N⋅m) and the unfiltered approach (PIKF-C: 2.31 N⋅m and 2.72 N⋅m). 

Regarding noise suppression performance, the evaluation is conducted based on the SD2 metric. The performance of PIAKF-CW1 (1.21 N⋅m and 1.24 N⋅m) is significantly worse than that of PIKF (0.26 N⋅m and 0.26 N⋅m) and PIKF-C (0.25 N⋅m and 0.26 N⋅m), indicating that PIAKF-CW1 not only failed to suppress noise, but also caused more significant fluctuations in the observed signal due to the failure of compensating for model uncertainty. In contrast, the proposed PIAKF-CW2 constructs an auxiliary observation variable $\widehat { \pmb { H } } _ { e }$ to decouple the backlash uncertainty from the observation noise. By treating the backlash uncertainty as an internal disturbance within the observer, PIAKF-CW2 achieves effective compensation of backlash uncertainty while simultaneously ensuring superior noise suppression (0.10 N⋅m and 0.14 N⋅m). 

In addition to the above observer performance validation experi ments, the material-handling task shown in $\mathrm { F i g . }$ 6 is also reproduced on the real robot platform to examine the behavior of the proposed method in an application-level scenario. The experimental setup is illustrated in Fig. 11(c). During the material-handling process, joints 4 and 6 are subjected to negligible external forces. Fig. 16(a) presents the estimated external joint torques produced by the proposed observer during the task. Pronounced backlash-induced uncertainty is observed in the time intervals 2.2–2.6 s and 4.6–5.0 s, corresponding to joint reversal behavior. Fig. 16(b) and (c) show the estimation residuals within these intervals. The proposed method effectively suppresses the residual errors caused by backlash uncertainty. These results confirm that the proposed framework not only maintains robustness in controlled validation experiments but also achieves effective separation of joint-level backlash uncertainty from external torques and compensates estimation errors during practical material-handling tasks. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/feec70d6b796f899067ea36cf8f7406194bd436e0764652ddb9f03c68252a3eb.jpg)



(a)



(b)



Fig. 14. Comparison of external torque component estimation results for joints 2, 3, and 5 (a) with 3 kg payload and (b) with 6 kg payload.



Table 5


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/e427cb5019af2c61fd567d1904d113eeac3657ffc5bcddcde4f97afe0baeca7b.jpg)



Time (s)



Fig. 15. Comparison of external torque component estimation performance between PIAKF-CW1 and PIAKF-CW2 for joint 2 with 3 kg payload



Joint external torque component estimation performance comparison with different payloads evaluated by RMSE and SD2.


<table><tr><td rowspan="2">Payload mass</td><td rowspan="2">Case</td><td colspan="4">RMSE (N·m)</td><td colspan="4">SD2 (N·m)</td></tr><tr><td>Joint 2</td><td>Joint 3</td><td>Joint 5</td><td>Average</td><td>Joint 2</td><td>Joint 3</td><td>Joint 5</td><td>Average</td></tr><tr><td rowspan="4">3 kg</td><td>PIKF</td><td>10.22</td><td>4.68</td><td>0.74</td><td>5.21</td><td>0.25</td><td>0.29</td><td>0.24</td><td>0.26</td></tr><tr><td>PIKF-C</td><td>4.98</td><td>1.53</td><td>0.42</td><td>2.31</td><td>0.24</td><td>0.28</td><td>0.24</td><td>0.25</td></tr><tr><td>PIAKF-CW1</td><td>9.30</td><td>2.57</td><td>0.56</td><td>4.14</td><td>2.92</td><td>0.58</td><td>0.13</td><td>1.21</td></tr><tr><td>PIAKF-CW2</td><td>5.43</td><td>1.67</td><td>0.34</td><td>2.48</td><td>0.10</td><td>0.11</td><td>0.10</td><td>0.10</td></tr><tr><td rowspan="4">6 kg</td><td>PIKF</td><td>11.28</td><td>6.41</td><td>0.99</td><td>6.23</td><td>0.25</td><td>0.29</td><td>0.24</td><td>0.26</td></tr><tr><td>PIKF-C</td><td>5.20</td><td>2.47</td><td>0.47</td><td>2.72</td><td>0.24</td><td>0.29</td><td>0.24</td><td>0.26</td></tr><tr><td>PIAKF-CW1</td><td>8.59</td><td>3.73</td><td>0.75</td><td>4.35</td><td>2.71</td><td>0.74</td><td>0.26</td><td>1.24</td></tr><tr><td>PIAKF-CW2</td><td>5.63</td><td>2.66</td><td>0.35</td><td>2.88</td><td>0.13</td><td>0.14</td><td>0.14</td><td>0.14</td></tr></table>

![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/c0bf55473d112179242217104ccb4e170e420761c98cbab8ff2130b1b5a95c88.jpg)



(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/e76ec2e9b4cada4e7622fc3b5cf768f8f7609ba77cd88320dcc7a930f3ef4c51.jpg)



(b)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-09-30/d5d3e7aa-28fa-4a48-9bed-7dd60d4c3daa/76c1a72781ce06fb62d126674fce3a01c315584dcc2ccdb4811fc44cc2b1dfb1.jpg)



(c)



Fig. 16. Experimental verification results in material handling task. (a) External torque component estimation results. (b) Estimation residuals during 2.2–2.6 s. (c) Estimation residuals during 4.6–5.0 s.


## 4.3. Result analysis and discussion

Due to the specific physical structure of robot joint clearance, the resulting model uncertainty is strongly correlated with the robot’s joint motion. When the robot’s joints reverse, the rate of change in joint velocity determines the intensity of the reversal impact, which in turn determines the magnitude of dynamic errors caused by backlash. Under fixed robot motion conditions, the external force or payloads applied at the end-effector has a relatively small impact on the degree of backlash uncertainty. Therefore, this article treats the backlash uncertainty as an internal disturbance in the robot observation process, distinguishing it from the observed external forces. This approach also ensures the proposed PIAKF exhibits good generalization across different payload scenarios. 

On the other hand, due to the complexity of the physical contact process in joint backlash and the difficulty in collecting relevant information through sensors available in industrial robots, directly constructing a comprehensive physical model is challenging. This article adopts a general model for backlash errors based on Gaussian basis functions to replace the complex modeling of joint backlash. Addition ally, the potential relationship between backlash and joint motion is learned through PINN. Ultimately, based on a time-series prediction approach, effective compensation for model uncertainties caused by clearance is achieved during contact force observation. Experimental results demonstrate the effectiveness of this method in handling model uncertainties introduced by clearance. 

In practical external force estimation tasks, observation disturbances are inevitable and originate from multiple sources. Unlike existing methods that handle uncertainty and observation noise directly, the proposed method constructs an auxiliary observation variable to decouple backlash uncertainty from observation noise. This approach avoids the loss of model uncertainty components that would occur if joint currents are filtered directly, which would otherwise prevent effective separation and modeling of the model uncertainty. 

## 5. Conclusion

This work addresses the problem of high-precision sensorless contact force estimation for industrial robots in the presence of significant model uncertainty, particularly backlash-induced dynamics. A PIAKF framework is proposed that integrates structured PINN-based dynamics modeling, uncertainty-inclusive observer design, and noise propagation analysis into a unified estimation architecture. The proposed method explicitly models internal disturbances rather than treating model mismatch as unstructured noise, enabling more reliable force estimation while preserving physical interpretability and computational efficiency. To the best of the authors’ knowledge, this work represents one of the first attempts to combine PINN-based robot dynamics modeling with adaptive Kalman filtering for contact force estimation. Simulation and real-world contact force estimation experiments demonstrate that the proposed PIAKF consistently outperforms advanced observers in both static and dynamic scenarios. The method achieves improved force tracking accuracy, stronger robustness against backlash-induced errors, and superior noise suppression. Experimental results further confirm its generalization capability under different payload conditions. These results indicate that combining physics-informed learning with observer structure design provides an effective pathway for robust force estimation in industrial robots lacking torque sensors. Future work will investigate force observation under high-speed motions with joint flex ibility and pursue system-level validation through comprehensive interaction experiments. 

## CRediT authorship contribution statement

Hongbo Hu: Methodology, Investigation. Jia Ma: Formal analysis. Yuhan Lu: Validation. Chungang Zhuang: Resources, Methodology, 

Investigation. 

## Declaration of competing interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper. 

## Acknowledgments

This work was sponsored by the National Natural Science Foundation of China (Grant No. 52275500) and the Explorers Program of Shanghai (Basic Research Funding) (Grant No. 24TS1414700). 

## Data availability

Data will be made available on request. 

## References



[1] Z. Wang, L. Zou, X. Su, G. Luo, R. Li, Y. Huang, Hybrid force/position control in workspace of robotic manipulator in uncertain environments based on adaptive fuzzy control, Robot. Auton. Syst. 145 (2021) 103870, https://doi.org/10.1016/j. robot.2021.103870. 





[2] T. Sun, J. Sun, B. Lian, Q. Li, Sensorless admittance control of 6-DoF parallel robot in human-robot collaborative assembly, Robot. Comput.-Integr. Manuf. 88 (2024) 102742, https://doi.org/10.1016/j.rcim.2024.102742 





[3] D. Sirintuna, A. Giammarino, A. Ajoudani, An object deformation-agnostic framework for human-robot collaborative transportation, IEEE Trans. Autom. Sci. Eng. 21 (2) (2023) 1986–1999, https://doi.org/10.1109/TASE.2023.3259162. 





[4] B. Shi, F. Wang, Z. Huo, Y. Tian, R. Cong, D. Zhang, Contact force sensing and control for inserting operation during precise assembly using a micromanipulator integrated with force sensors, IEEE Trans. Autom. Sci. Eng. 20 (3) (2022) 2147 2155, https://doi.org/10.1109/TASE.2022.3195951. 





[5] S. Eppinger, W. Seering, On dynamic models of robot force control, in: 1986 IEEE International Conference on Robotics and Automation (ICRA), 1986, pp. 29 34, https://doi.org/10.1109/ROBOT.1986.1087723. 





[6] S. Long, X. Dang, S. Sun, Y. Wang, M. Gui, A novel sliding mode momentum observer for collaborative robot collision detection, Machines 10 (9) (2022) 818 https://doi.org/10.3390/machines10090818. 





[7] A. Kurdas, M. Hamad, J. Vorndamme, N. Mansfeld, S. Abdolshah, S. Haddadin, Online payload identification for tactile robots using the momentum observer, in: 2022 IEEE International Conference on Robotics and Automation (ICRA), 2022, pp. 5953–5959. ICRA46639.2022.9811691. 





[8] G. Garofalo, N. Mansfeld, J. Jankowski, C. Ott, Sliding mode momentum observers for estimation of external torques and joint acceleration, in: 2019 IEEE International Conference on Robotics and Automation (ICRA), 2019, pp. 6117–6123, https://doi.org/10.1109/ICRA.2019.8793529 





[9] L. Guo, S. Cao, Anti-disturbance Control For Systems With Multiple Disturbances, 





[10] Y. Wei, S. Lyu, W. Li, X. Yu, Z. Wang, L. Guo, Contact force estimation of robot manipulators with imperfect dynamic model: on gaussian process adaptive disturbance Kalman filter, IEEE Trans. Autom. Sci. Eng. 21 (2023) 3524–3537, https://doi.org/10.1109/TASE.2023.3280750 





[11] Y. Han, J. Wu, C. Liu, Z. Xiong, An iterative approach for accurate dynamic model identification of industrial robots, IEEE Trans. Robot 36 (5) (2020) 1577 1594, https://doi.org/10.1109/TRO.2020.2990368. 





[12] T. Shen, X. Qiao, Y. Dong, Y. Wang, W. Zhang, Y. Yuan, Deep adaptive control with online identification for industrial robots, Sci. China 65 (11) (2022) 2593–2604, https://doi.org/10.1007/s11431-022-2183-7. 





[13] G. Liao, Y. Sheng, X. Zeng, Spacecraft hovering around asteroid via disturbance observer based exponential time-varying sliding mode controller, in: 2017 IEEE International Conference on Control and Automation (ICCA), 2017, pp. 313–318, https://doi.org/10.1109/ICCA.2017.8003079. 





[14] L. Han, J. Mao, P. Cao, Y. Gan, S. Li, Toward sensorless interaction force estimation for industrial robots using high-order finite-time observers, IEEE Trans. Ind. Electron. 69 (7) (2021) 7275–7284, https://doi.org/10.1109/TIE.2021.3095820. 





[15] J. Zhang, P. Nie, B. Zhang, A variable structure passivity control method for elastic joint robots based on cascaded high-order state estimation, Sci. China 67 (2) (2024) 395–407, https://doi.org/10.1007/s11431-023-2479-5. 





[16] T. Ren, Y. Dong, D. Wu, K. Chen, Collision detection and identification for robot manipulators based on extended state observer, Control Eng. Pract. 79 (2018) 144 153, https://doi.org/10.1016/j.conengprac.2018.07.004. 





[17] A. Stolt, A. Robertsson, R. Johansson, Robotic force estimation using dithering to decrease the low velocity friction uncertainties, in: 2015 IEEE International Conference on Robotics and Automation (ICRA), 2015, pp. 3896–3902, https:// doi 0rg/10.1109/ICRA.2015.7139742 





[18] M. Li, R. Kang, D.T. Branson, J.S. Dai, Model-free control for continuum robots based on an adaptive Kalman filter, IEEE-ASME Trans. Mechatron. 23 (1) (2017) 286 297, https://doi.org/10.1109/TMECH.2017.2775663. 





[19] Y. Liu, X. Fan, C. Lv, J. Wu, L. Li, D. Ding, An innovative information fusion method with adaptive Kalman filter for integrated INS/GPS navigation of autonomous vehicles, Mech. Syst. Signal Proc. 100 (2018) 605–616, https://doi.org/10.1016/j. ymssp.2017.07.051. 





[20] Y. Cheng, Y. Li, K. Li, X. Liu, C. Liu, X. Hao, Fusing LSTM neural network and expanded disturbance Kalman filter for estimating external disturbing forces of ball screw drives, Robot. Comput.-Integr. Manuf. 89 (2024) 102776, https://doi.org/ 10.1016/j.rcim.2024.102776. 





[21] A. Wahrburg, E. Morara, G. Cesari, B. Matthias, H. Ding, Cartesian contact force estimation for robotic manipulators using Kalman filters and the generalized momentum, in: 2015 IEEE International Conference on Automation Science and Engineering (CASE), 2015, pp. 1230–1235, https://doi.org/10.1109/ CoASE.2015.7294266. 





[22] A. Wahrburg, J. Bos, ¨ K.D. Listmann, F. Dai, B. Matthias, H. Ding, Motor-current based estimation of cartesian contact forces and torques for robotic manipulators and its application to force control, IEEE Trans. Autom. Sci. Eng. 15 (2) (2017) 879–886, https://doi.org/10.1109/TASE.2017.2691136. 





[23] F. Cao, P.D. Docherty, S. Ni, X. Chen, Contact force and torque sensing for serial manipulator based on an adaptive Kalman filter with variable time period, Robot. Comput.-Integr. Manuf. 72 (2021) 102210, https://doi.org/10.1016/j. rcim.2021.102210. 





[24] J. Dong, J. Xu, L. Wang, A. Liu, L. Yu, External force estimation of the industrial robot based on the error probability model and SWVAKF, IEEE Trans. Instrum. Meas. 71 (2022) 1–11, https://doi.org/10.1109/TIM.2022.3209742. 





[25] X. Liang, Y. Yan, W. Wang, T. Su, G. He, G. Li, Z. Hou, Adaptive human-robot interaction torque estimation with high accuracy and strong tracking ability for a lower limb rehabilitation robot, IEEE-ASME Trans. Mechatron. (2024), https://doi. org/10.1109/TMECH.2024.3394491. 





[26] J. Hu, R. Xiong, Contact force estimation for robot manipulator using semiparametric model and disturbance Kalman filter, IEEE Trans. Ind. Electron. 65 (4) (2017) 3365–3375, https://doi.org/10.1109/TIE.2017.2748056. 





[27] S. Liu, L. Wang, X.V. Wang, Sensorless force estimation for industrial robots using disturbance observer and neural learning of friction approximation, Robot. Comput.-Integr. Manuf. 71 (2021) 102168, https://doi.org/10.1016/j. rcim.2021.102168. 





[28] D. Lim, D. Kim, J. Park, Momentum observer-based collision detection using LSTM for model uncertainty learning, in: 2021 IEEE International Conference on Robotics and Automation (ICRA), 2021, pp. 4516–4522, https://doi.org/10.1109/ ICRA48506.2021.9561667. 





[29] G. Evangelisti, S. Hirche, Physically consistent learning of conservative lagrangian systems with gaussian processes, in: 2022 IEEE Conference on Decision and Control (CDC), 2022, pp. 4078 4085, https://doi.org/10.1109/CDC51059.2022.9993123. 





[30] G. Evangelisti, S. Hirche, Data-driven momentum observers with physically consistent Gaussian processes, IEEE Trans. Robot 40 (2024) 1938 1951, https:// doi org/10.1109/TRO.2024.3366818 





[31] M. Lutter, C. Ritter, J. Peters, Deep lagrangian networks: using physics as model prior for deep learning, in: 2019 International Conference on Learning Representations (ICLR), 2019, https://doi.org/10.48550/arXiv.1907.04490 





[32] H. Hu, Z. Shen, C. Zhuang, A PINN-based friction-inclusive dynamics modeling method for industrial robots, IEEE Trans. Ind. Electron. 72 (2024) 5136 5144, https://doi.org/10.1109/TIE.2024.3476977. 





[33] S. Yousefizadeh, T. Bak, Unknown external force estimation and collision detection for a cooperative robot, Robotica 38 (9) (2020) 1665 1681, https://doi.org 10.1017/S0263574719001681. 





[34] A. De Luca, L. Ferrajoli, A modified Newton-Euler method for dynamic computations in robot fault detection and control, in: 2009 IEEE International Conference on Robotics and Automation (ICRA), 2009, pp. 3359–3364, https:// doi.org/10.1109/ROBOT.2009.5152618 





[35] A.G. Baydin, B.A. Pearlmutter, A.A. Radul, J.M. Siskind, Automatic differentiation in machine learning: a survey, J. Mach. Learn. Res. 18 (153) (2018) 1–43, https:/ doi.org/10.48550/arXiv.1502.05767. 





[36] J. Han, H. Cheng, X. Shan, H. Liu, J. Xiao, T. Huang, A novel multi-pulse friction compensation strategy for hybrid robots, Mech. Mach. Theory 201 (2024) 105726, https://doi.org/10.1016/j.mechmachtheory.2024.105726. 





[37] C. Zhuang, Y. Yao, Y. Shen, Z. Xiong, A convolution neural network based semiparametric dynamic model for industrial robot, J. Mech. Eng. Sci. 236 (7) (2022) 3683–3700, https://doi.org/10.1177/09544062211039875. 





[38] F. Cao, P. Docherty, X. Chen, Contact force estimation for serial manipulator based on weighted moving average with variable span and standard Kalman filter with automatic tuning, Int. J. Adv. Manuf. Technol. 118 (9) (2022) 3443–3456, https:/ doi.org/10.1007/s00170-021-08036-9. 





[39] Z. Gao, Scaling and bandwidth-parameterization based controller tuning, in: 2003 American Control Conference (ACC), 2003, pp. 984–989, https://doi.org/10.1109 ACC.2003.1242516. 





[40] A. De Luca, R. Mattone, Actuator failure detection and isolation using generalized momenta, in: 2003 IEEE International Conference on Robotics and Automation (ICRA), 2003, pp. 634–639, https://doi.org/10.1109/ROBOT.2003.1241665. 





[41] J. Han, From PID to active disturbance rejection control, IEEE Trans. Ind. Electron. 56 (3) (2009) 900–906, https://doi.org/10.1109/TIE.2008.2011621. 





[42] J. Zhang, P. Nie, Y. Chen, B. Zhang, A joint acceleration estimation method based on a high-order disturbance observer, IEEE Robot. Autom. Lett. 7 (4) (2022) 12615 12622, https://doi.org/10.1109/LRA.2022.3220501. 





[43] K.J. Park, Fourier-based optimal excitation trajectories for the dynamic identification of robots, Robotica 24 (5) (2006) 625–633, https://doi.org/ 10.1017/S0263574706002712. 





[44] V. Bonnet, P. Fraisse, A. Crosnier, M. Gautier, A. Gonz´alez, G. Venture, Optima exciting dance for identifying inertial parameters of an anthropomorphic structure, IEEE Trans. Robot 32 (4) (2016) 823 836, https://doi.org/10.1109 TRO.2016.2583062. 

