# A Joint Acceleration Estimation Method Based on a High-Order Disturbance Observer

Jiexin Zhang , Graduate Student Member, IEEE, Pingyun Nie , Yuhang Chen , and Bo Zhang gD 

Abstract—Joint acceleration feedback is widely used in the design of controllers and observers since joint accelerations reflect the joint dynamics of robots, especially in physical human-robot interaction. However, joint acceleration acquisition is a technical difficulty for robots. The dynamics-based methods can achieve joint acceleration estimation using only a nominal model. Still, the performance of these methods is limited by the fast time-varying disturbances in the system. This letter proposes a joint acceleration estimation method based on a high-order disturbance observer. This method can observe and compensate for fast time-varying lumped disturbances in the observer while maintaining joint acceleration estimation performance at low frequencies. The finite-time stability of the proposed estimation method is proved using the Lyapunov theory. Simulations and experiments with a lower limb rehabilitation robot are implemented to verify the performance of the proposed method. 

Index Terms—High-order disturbance observer, joint acceleration estimation, motion control, physical human-robot interaction. 

## I. INTRODUCTION

R <sup>ECENTLY,</sup> <sup>physical</sup> <sup>human-robot</sup> <sup>interaction</sup> <sup>(pHRI)</sup> <sup>has</sup>become a research focus in robotics [1], [2]. In most pHRI become a research focus in robotics [1], [2]. In most pHRI scenarios, the robot must act in an unknown environment. Robot dynamics will change due to unforeseen contact, which significantly affects the robot’s control accuracy. Thus, the control systems are required to observe and respond fast to the changes in the robot dynamics to ensure the robot’s performance. 

Joint acceleration was widely used in feedback control since it can intuitively reflect the changes in the robot dynamics [3], [4]. A general control framework is shown in Fig. 1. For industrial robots, arm vibration problems are caused by resonance vibration depending on actuator velocity and transient oscillations caused by acceleration change of actuators. Reference [3] proposed an industrial robot’s control method based on joint acceleration feedback to absorb arm vibrations. Meanwhile, flexible robots have been widely used in human-robot interaction. They are more compliant than rigid robots, yet the dynamics are vulnerable to external disturbances, which further motivates the application of acceleration feedback. A dynamic compensation controller based on joint acceleration was derived in [5]. A generic control architecture based on joint acceleration feedback was proposed in [6], and it achieved good robustness even under significant environmental disturbances. However, joint acceleration acquisition is a technical difficulty for robots and has been extensively studied over the last few years. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/1b43e73620c1ce600ada3cfe7a6571d7f746d67789f43648dc39e17ea364a9fb.jpg)



Fig. 1. Applications of joint acceleration estimation in pHRI.


The existing joint acceleration acquisition methods can be divided into two categories: direct measurement with the accelerometers attached to the robot’s links and indirect methods based on robot dynamics or joint positions [4], [7], [8], [9]. The accelerometer performs well for joint acceleration acquisition, yet its performance at low frequencies is limited by bandwidth [10]. Furthermore, the configuration of the accelerometers on the robot requires independent kinematics and signal calibration procedures. The rough calibrations can distort the data quality [11], [12]. Joint accelerations can also be obtained indirectly via numerical differentiation of the joint position and filtering, but the performance is limited by noise amplification and signal lag [8]. Another indirect method is based on the dynamics model [9]. This method avoids measurement noise amplification and signal lag, with which joint acceleration can be estimated. Importantly, it excels at low frequencies, which makes up for the shortcomings of the above methods. However, system uncertainties, including model disturbances and unforeseen external torques, will distort the estimated accelerations. Therefore, compensation for system uncertainties is essential in this class of joint acceleration estimation methods. 

In recent studies, uniform observation and compensation for system uncertainties have been proven effective in improving state estimation performance in state observers [13], [14], [15], [16], [17]. The target of this kind of observer is to design an auxiliary lumped disturbance variable to approximate the system uncertainties [18]. A joint torque observer to estimate the interaction time-varying forces was proposed in [13], and the authors highlighted the importance of the dynamics model’s quality. Ren et al. [14] combined an extended state observer (ESO) and a generalized momentum observer to estimate the external torques. A super-twisting algorithm was designed to estimate the external torques directly and achieved great success [19]. Combining with the linear algorithm, an improved version (SOSML) [20] was also used to enhance the system’s convergence rate. However, these methods only focus on constant or slow lumped disturbance. 

The high-order disturbance observers (HODOs) can be used to deal with fast time-varying disturbances [15], [17], [21]. Not only disturbances but also their time derivatives can be observed in HODOs. However, all system uncertainties are included in the lumped disturbance variable. Hence, the observers’ performance must rely on large gain parameters when system uncertainties play a significant role in the observers. It is usually not a good choice because the observer’s high gain will amplify measurement noise [16], [22]. As a result, migration of the above method to the joint acceleration estimations in pHRI has performance limitations. The system uncertainties of the joint acceleration estimation system include the terms caused by model disturbances and the external torques. The latter is usually fast time-varying, and complex [15], [23]. 

This letter proposes a joint acceleration estimation method based on a high-order disturbance observer (HODO-based) to deal with fast time-varying lumped disturbances containing significant system uncertainties in pHRI. Specifically, a HODO is designed to accurately observe the fast time-varying lumped disturbances in the desired frequency range for pHRI. External torques are estimated based on first-order generalized momentum (GM) and compensated to the observer for suppressing the main system uncertainties. The finite-time stability of the proposed method is proved using the Lyapunov theory. Comparative experiments are implemented with a lower limb rehabilitation robot. The experimental results show that the proposed method performs well even with unforeseen contact forces and highfrequency disturbances. The main contributions of this letter include the following. 

- A finite-timejoint acceleration estimation method based on HODO with rigorous stability analysis is derived, which is better than direct measurement and momentum observerbased method [19]. 

- A HODO is designed by combining external torque compensation to suppress the main system uncertainties. 

- Derivatives of lumped disturbances are observed by combining linear and nonlinear algorithms, which ensures convergence of lumped disturbance’s derivatives near and far from the origin’s equilibrium point. 

The rest of this letter is organized as follows. Section II derives the robot’s dynamics model and the dynamics-based joint acceleration estimation equation. Section III first demonstrates thejoint acceleration estimation method utilizing existing algorithms and analyzes their shortcomings. Then, our work is derived to cope with these issues. Section IV designs the simulation experiments of a single-degree-of-freedom robot. Section V conducts verification experiments with a lower limb rehabilitation robot. Conclusions and discussions are presented in Section VI. 

## II. DYNAMICS MODEL

This section derives the robot’s dynamics model and the dynamics-based acceleration estimation equation. 

## A. Robot’s Dynamics Model

The Euler–Lagrange equation is employed to represent the dynamics of robots in this letter, shown as the following differential equation [24]. 

$$
\mathrm{M} _ {r} (\mathrm{q}) \ddot {\mathrm{q}} + \mathrm{C} _ {r} (\mathrm{q}, \dot {\mathrm{q}}) \dot {\mathrm{q}} + \mathrm{g} _ {r} (\mathrm{q}) = \mathcal {T} + \mathcal {T} _ {e x t}\tag{1}
$$

Assuming n represents the degree of freedom of the robot. $\mathrm { M } _ { r } ( \mathbf { q } ) \in \mathrm { R } ^ { \bar { \mathbf { n } } \times \mathbf { n } }$ is a matrix representing the robot inertia. $\mathrm { C } _ { r } ( \tilde { \mathbf { q } } , \tilde { \mathbf { q } } ) \in \mathrm { R } ^ { \mathrm { n } \times \mathrm { n } }$ is the Coriolis and centripetal matrix of the robot. $\bar { \mathrm { g } _ { r } } ( \mathsf { q } ) \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ is the gravity vector ofthe robot. $\mathcal { T } \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ and $\mathcal { T } _ { e x t } ^ { - } \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ are thejoint torque and external torque vectors, respectively. $\mathrm { q } \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ is the joint position vector. $\dot { \mathbf { q } } \in \mathbf { R } ^ { \mathrm { n } \times 1 }$ is the joint velocity vector. $\ddot { \mathrm { q } } \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ is the joint acceleration vector. The inertia matrix $\mathrm { M } _ { r } ( \mathbf { q } )$ and the Coriolis and centripetal matrix $\mathrm { C } _ { r } ( \mathrm { q } , \dot { \mathrm { q } } )$ satisfy the passivity property [24] 

$$
\dot {\mathrm{M}} _ {r} (\mathrm{q}, \dot {\mathrm{q}}) = \mathrm{C} _ {r} (\mathrm{q}, \dot {\mathrm{q}}) + \mathrm{C} _ {r} (\mathrm{q}, \dot {\mathrm{q}}) ^ {\mathrm{T}}\tag{2}
$$

## B. Dynamics-Based Joint Acceleration Estimation

The joint acceleration can be estimated by (1), and the explicit representation is derived in (3) by reshaping (1). To keep the brief mathematical description in the subsequent development, we omit joint variable dependencies and time dependence of time-varying model variables. 

$$
\ddot {\mathrm{q}} = \mathrm{M} _ {r} ^ {- 1} (\mathcal {T} + \mathcal {T} _ {e x t} - \mathrm{C} _ {r} \dot {\mathrm{q}} - \mathrm{g} _ {r})\tag{3}
$$

However, the external torque $\tau _ { e x t }$ is unmeasurable, and model disturbances exist between actual and nominal models. Therefore, the equation for estimation of $\ddot { \mathrm { ~ \textq ~ } }$ can be derived as 

$$
\ddot {\hat {\mathrm{q}}} = \hat {\mathrm{M}} _ {r} ^ {- 1} (\mathcal {T} - \hat {\mathrm{C}} _ {r} \dot {\mathrm{q}} - \hat {\mathrm{g}} _ {r})\tag{4}
$$

where joint position q can be measured by the sensors. The velocity q˙ is obtained by thejoint position’s first-order numerical differentiation and filter. $\ddot { \hat { \mathrm { q } } } \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ represents the estimation value of the joint acceleration vector. $\hat { \mathrm { M } } _ { r } , \hat { \mathrm { C } } _ { r } , \hat { \mathrm { g } } _ { r }$ are the nominal values of $\bar { \mathrm { M } } _ { r } , ~ \mathrm { C } _ { r } , ~ \mathrm { g } _ { r }$ . The relationship between actual and nominal models is introduced as: 

$$
\mathrm{M} _ {r} ^ {- 1} = \hat {\mathrm{M}} _ {r} ^ {- 1} + \Delta \mathrm{M} _ {r} ^ {- 1}, \mathrm{C} _ {r} = \hat {\mathrm{C}} _ {r} + \Delta \mathrm{C} _ {r}, \mathrm{g} _ {r} = \hat {\mathrm{g}} _ {r} + \Delta \mathrm{g} _ {r}\tag{5}
$$

where $\Delta \mathrm { M } _ { r } ^ { - 1 } , \Delta \mathrm { C } _ { r } , \Delta \mathrm { g } _ { \imath }$ are the disturbances of $\mathrm { M } _ { r } ^ { - 1 } , \mathrm { ~ C } _ { r } ,$ $\mathrm { g } _ { r } ,$ respectively. Hence, the lumped disturbance term in (4) is represented as $\mathrm { w } = \ddot { \mathrm { q } } - \ddot { \hat { \mathrm { q } } }$ , which is represented as 

$$
\begin{array}{r l} & {\mathrm{w} = \Delta \hat {\mathrm{M}} _ {r} ^ {- 1} \mathcal {T} - \Delta \hat {\mathrm{M}} _ {r} ^ {- 1} ((\hat {\mathrm{C}} _ {r} + \Delta \mathrm{C} _ {r}) \dot {\mathrm{q}} + (\hat {\mathrm{g}} _ {r} + \Delta \mathrm{g} _ {r}))} \\ & {\quad - \hat {\mathrm{M}} _ {r} ^ {- 1} (\Delta \mathrm{C} _ {r} \dot {\mathrm{q}} + \Delta \mathrm{g} _ {r}) + \mathrm{M} _ {r} ^ {- 1} \mathcal {T} _ {e x t}.} \end{array}\tag{6}
$$

w contains the model disturbances and external torques on the system. Note that the dynamics-based joint acceleration estimation is sensitive to lumped disturbances. Thus, it is necessary to observe and compensate for lumped disturbances in the system using the disturbance observation algorithms. 

## III. THE JOINT ACCELERATION ESTIMATION METHOD BASED ON DISTURBANCE OBSERVER

This section discusses the joint acceleration estimation method utilizing SOSML and analyzes its shortcomings. Then the proposed HODO-based method is derived by further improvements to cope with these issues. 

## A. Joint Acceleration Estimation Based on SOSML

SOSML [20] combined linear and nonlinear disturbance observation algorithms and performed well in observing and compensating for lumped disturbances. In this subsection, the estimation method based on SOSML is described in detail. The explicit calculated function of joint acceleration (4) can be modified in the form of the state equation. 

$$
\dot {\mathrm{x}} _ {1} = \mathrm{f+w}\tag{7}
$$

where $\mathrm { x } _ { 1 } = \dot { \mathrm { q } } , \dot { \mathrm { x } } _ { 1 } = \ddot { \mathrm { q } } .$ , and $\mathrm { f } \in \mathrm { R } ^ { \mathrm { n \times 1 } }$ is a nominal model related to time and state variables, which is derived as 

$$
\mathrm{f} = \hat {\mathrm{M}} _ {r} ^ {- 1} (\mathcal {T} - \hat {\mathrm{C}} _ {r} \dot {\mathrm{q}} - \hat {\mathrm{g}} _ {r})\tag{8}
$$

As shown in (4), w is set as the lumped disturbance vector. Considering w as an auxiliary state variable $\mathbf { X } _ { 2 } .$ , (7) is derived as follows [25]. 

$$
\left\{ \begin{array}{l} \dot {\mathrm{x}} _ {1} = \mathrm{f} + \mathrm{x} _ {2} \\ \dot {\mathrm{x}} _ {2} = \mathrm{h} \end{array} \right.\tag{9}
$$

where $\mathrm { h } = \dot { \mathrm { w } }$ . Following the same observer structure in [19], the observer is introduced as 

$$
\left\{ \begin{array}{l} \dot {\hat {x}} _ {1} = f + \hat {x} _ {2} + u _ {1} \\ \dot {\hat {x}} _ {2} = u _ {2} \end{array} \right.\tag{10}
$$

where $\mathrm { u } _ { 1 } \in \mathrm { R } ^ { \mathrm n \times 1 }$ and $\mathrm { u } _ { 2 } \in \mathrm { R } ^ { \mathrm { n } \times 1 }$ are the time-dependent observer input vectors. It is worth mentioning that (10) is the general form of ESO, which has been applied in [25], [26]. $\hat { \mathrm { X } } _ { 2 }$ is regarded as the finite-time observation of w. Further, <sup>˙</sup>xˆ is the joint acceleration estimations. With setting $\mathbf { e } _ { 1 } = \mathbf { x } _ { 1 } - \hat { \mathbf { x } } _ { 1 }$ and $\mathrm { e _ { 2 } = x _ { 2 } - \hat { x } _ { 2 } }$ , the input vectors is designed as follows [27]. 

$$
\left\{ \begin{array}{l} \mathrm {u_ {1}} = \mathrm {K_ {1}} | \mathrm {e_ {1}} | ^ {1 / 2} \mathrm{sign} (\mathrm {e_ {1}}) + \mathrm {K_ {2}} \mathrm {e_ {1}} \\ \mathrm {u_ {2}} = \mathrm {K_ {3}} \mathrm{sign} (\mathrm {e_ {1}}) + \mathrm {K_ {4}} \mathrm {e_ {1}} \end{array} \right.\tag{11}
$$

where $\mathrm { K } _ { 1 }$ and $\mathrm { K _ { 2 } }$ are $\left( \mathrm { n } \times \mathrm { n } \right)$ diagonal matrices whose diagonal elements are the observer gains of the corresponding channels [18]. sign(-) is an operator used to take the sign of an element. Note that all operations are performed on elements of vectors and matrices. Then, it is easy to derive the error dynamics of the observer by (9) minus (10). 

$$
\left\{ \begin{array}{l} \dot {\mathrm{e}} _ {1} = \mathrm{e} _ {2} - \mathrm{K} _ {1} | \mathrm{e} _ {1} | ^ {1 / 2} \text {sign} (\mathrm{e} _ {1}) - \mathrm{K} _ {2} \mathrm{e} _ {1} \\ \dot {\mathrm{e}} _ {2} = \mathrm{h} - \mathrm{K} _ {3} \text {sign} (\mathrm{e} _ {1}) - \mathrm{K} _ {4} \mathrm{e} _ {1} \end{array} \right.\tag{12}
$$

The error dynamics (12) is an n-order independent SOSML. Since the change of the external torque and the model disturbance is limited, w, w˙ are considered to be bounded. Thus, the finite-time convergence of $( \mathrm { e } _ { 1 } , \mathrm { e } _ { 2 } ) = ( 0 , 0 )$ can be guaranteed if the gain matrices $\mathrm { K _ { 1 } , K _ { 2 } , K _ { 3 } , K _ { 4 } }$ are positive definite diagonal matrices [20]. 

The joint acceleration estimation method based on SOSML ensures the responsiveness of the observer’s error dynamics near and far from the origin’s equilibrium point. However, SOSML can only accurately observe disturbances with the first-order derivative bounded. In pHRI, the human motion may have large amplitude and time-varying frequency with a maximum of 8 Hz [23]. Additionally, model disturbances also have complex effects on joint acceleration estimation. 

The proposed method requires the estimation of the joint velocity <sup>˙</sup>qˆ to be available. The estimatedjoint velocity is calculated 

by integrating $\dot { \hat { \mathbf { x } } } _ { 1 }$ . 

$$
\dot {\hat {\mathrm{q}}} = \int \dot {\hat {\mathrm{x}}} _ {1} \mathrm{dt}\tag{13}
$$

## B. Joint Acceleration Estimation Based on HODO

This subsection proposes our method by further improving the above method. Two main improvements are considered in the proposed method to cope with the issues mentioned in the previous section: 

- External torque compensation is introduced to suppress the main system uncertainties in the observer system. 

- The derivatives of lumped disturbances are observed by combining the linear and nonlinear algorithms. 

Moreover, the finite-time stability of the proposed method is proved using the Lyapunov theory. 

1) External Torques Compensation in Observer: The most well-established external torque estimation method is presented in [28], called GM observer. The GM observer is based on the derivation of joint momentum, and the robot’s generalized momentum $p \in \bar { \mathrm { R } } ^ { \mathrm { n } \times 1 }$ can be derived as 

$$
\mathrm{p} = \mathrm{M} _ {r} \dot {\mathrm{q}}\tag{14}
$$

Rewriting (1), the derivative version of generalized momentum is derived as 

$$
\dot {\mathrm{p}} = \mathcal {T} + \mathcal {T} _ {e x t} - \mathrm{C} _ {r} \dot {\mathrm{q}} - \mathrm{g} _ {r} + \dot {\mathrm{M}} _ {r} \dot {\mathrm{q}}.\tag{15}
$$

We apply (2) into (15) to avoid calculating $\dot { \mathrm { M } } _ { r }$ . Hence, 

$$
\dot {\mathrm{p}} = \mathcal {T} + \mathcal {T} _ {e x t} + \mathrm{C} _ {r} ^ {T} \dot {\mathrm{q}} - \mathrm{g} _ {r}\tag{16}
$$

The GM observer is derived as follows. 

$$
\mathrm{r} = \mathrm{K} _ {\mathrm{m}} \left(\mathrm{p} - \int_ {0} ^ {\mathrm{t}} \left(\mathcal {T} + \mathrm{C} _ {\mathrm{r}} ^ {\mathrm{T}} \dot {\mathrm{q}} - \mathrm{g} _ {\mathrm{r}} + \mathrm{r}\right) \mathrm{ds} - \mathrm{p} (0)\right)\tag{17}
$$

where $\mathrm { K } _ { \mathrm { m } } \in \mathrm { R } ^ { \mathrm { n } \times \mathrm { n } }$ is a positive definite diagonal matrix. $r \in$ $\mathrm { R } ^ { \mathrm { n \times 1 } }$ is a residual vector. Assuming that the initial time is zero, then $\mathrm { p } ( 0 )$ is the momentum at the initial time. Differentiating (17), we get [28] 

$$
\dot {r} = \mathrm{K} _ {\mathrm{m}} (r - \mathcal {T} _ {e x t})\tag{18}
$$

The estimated external torque vector is set as $\hat { \mathcal { T } } _ { e x t } = r$ , meaning each element of r is a first-order lowpass filtered version of the corresponding joint external torque in $\mathcal { T } _ { e x t }$ . Its cutoff frequency depends on $\mathrm { K } _ { \mathrm { m } }$ . Then, the estimated external torques can be compensated in the observer, and the nominal model (8) is reshaped as 

$$
\mathrm{f} ^ {*} = \hat {\mathrm{M}} _ {r} ^ {- 1} (\mathcal {T} + \hat {\mathcal {T}} _ {e x t} - \hat {\mathrm{C}} _ {r} \dot {\mathrm{q}} - \hat {\mathrm{g}} _ {r})\tag{19}
$$

The system lumped disturbances after external torque compensation are modified to 

$$
\begin{array}{c} \mathrm{w} ^ {*} = \Delta \hat {\mathrm{M}} _ {r} ^ {- 1} \mathcal {T} - \Delta \hat {\mathrm{M}} _ {r} ^ {- 1} ((\hat {\mathrm{C}} _ {r} + \Delta \mathrm{C} _ {r}) \dot {\mathrm{q}} + (\hat {\mathrm{g}} _ {r} + \Delta \mathrm{g} _ {r})) \\ - \hat {\mathrm{M}} _ {r} ^ {- 1} (\Delta \mathrm{C} _ {r} \dot {\mathrm{q}} + \Delta \mathrm{g} _ {r}) + \mathrm{M} _ {r} ^ {- 1} \mathcal {T} _ {e x t} - \hat {\mathrm{M}} _ {r} ^ {- 1} \hat {\mathcal {T}} _ {e x t} \end{array}\tag{20}
$$

It is worth noting that the estimated external torques cannot be completely equivalent to the actual external torques since only the first-order lowpass filtered version is provided. Therefore $\mathrm { w } ^ { * }$ includes the model disturbance and the residual high-frequency components of actual external torques, which need further observation and compensation by the HODO. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/520f95212eeb5a0dc6d70c5d86284786721fbd342ec45336477e3ecf406f0a26.jpg)



Fig. 2. The structure diagram of the HODO-based method.


2) HODO-Based Joint Acceleration Estimation: For accurate observations of the fast time-varying lumped disturbances, it is necessary to estimate and compensate for the derivative of the lumped disturbances. Since external torque compensation is implemented, the lumped disturbance term in HODO is reset as $\mathrm { x } _ { 2 } = \mathrm { w } ^ { \ast }$ . The derivative of lumped disturbances is also designed as an auxiliary state variable. We assume that $\mathrm { x _ { 3 } = h ^ { * } = \dot { w } ^ { * } }$ and $\mathrm { e _ { 3 } = x _ { 3 } - \hat { x } _ { 3 } }$ . By combining (19), the HODO is derived as 

$$
\left\{ \begin{array}{l} \dot {\hat {x}} _ {1} = f ^ {*} + \hat {x} _ {2} + u _ {1} \\ \dot {\hat {x}} _ {2} = \hat {x} _ {3} + u _ {2} \\ \dot {\hat {x}} _ {3} = u _ {3} \end{array} \right.\tag{21}
$$

${ \hat { \mathrm { X } } } _ { 3 }$ represents the estimation of the lumped disturbances’ first derivative. $\mathrm { u _ { 3 } }$ combines linear and nonlinear algorithms,just like $\mathrm { u } _ { 1 }$ and u , to guarantee the convergence of $\mathrm { \hat { x } _ { 3 } }$ near and far from the origin’s equilibrium point. 

$$
\left\{ \begin{array}{l} u _ {1} = K _ {1} | e _ {1} | ^ {1 / 2} \text { sign } (e _ {1}) + K _ {2} e _ {1} \\ u _ {2} = K _ {3} \text { sign } (e _ {1}) + K _ {4} e _ {1} \\ u _ {3} = K _ {5} \text { sign } (e _ {1}) + K _ {6} e _ {1} \end{array} \right.\tag{22}
$$

where $\mathrm { K } _ { j } , j = 1 , 2 , \dotsc , 6$ are the positive definite (n×n) diagonal parameter matrices. The diagonal element $\operatorname { K } _ { j - i } , i =$ $1 , 2 , \ldots ,$ n represents the gain coefficient ofthe ithjoint ofmatrices $\mathrm { K } _ { i } , j = \bar { 1 } , 2 , \ldots , 6$ , respectively. Then, the error dynamics of the HODO is written as follows. 

$$
\left\{ \begin{array}{l} \dot {\mathrm{e}} _ {1} = \mathrm{e} _ {2} - \mathrm{K} _ {1} | \mathrm{e} _ {1} | ^ {1 / 2} \mathrm{sign} (\mathrm{e} _ {1}) - \mathrm{K} _ {2} \mathrm{e} _ {1} \\ \dot {\mathrm{e}} _ {2} = \mathrm{e} _ {3} - \mathrm{K} _ {3} \mathrm{sign} (\mathrm{e} _ {1}) - \mathrm{K} _ {4} \mathrm{e} _ {1} \\ \dot {\mathrm{e}} _ {3} = \dot {\mathrm{h}} - \mathrm{K} _ {5} \mathrm{sign} (\mathrm{e} _ {1}) - \mathrm{K} _ {6} \mathrm{e} _ {1} \end{array} \right.\tag{23}
$$

where $\dot { \mathrm { h } } = \ddot { \mathrm { w } }$ . The finite-time stability of the error dynamics of $( \mathrm { e _ { 1 } } , \mathrm { e _ { 2 } } , \mathrm { e _ { 3 } } ) = ( 0 , 0 , 0 )$ is proved using the Lyapunov theory, which is described detailly in the Appendix. Then, $\dot { \hat { \mathbf { x } } } _ { 1 }$ represents the estimation of the joint acceleration ${ \ddot { \mathrm { { q } } } } .$ Fig. 2 shows the structure of the HODO-based method. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/b33452de438b94d6f49c72b772f2a66e20ee1ff91d564617dca46db232ffd86c.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/b490b1ee7b8f522e9e3f1abecbe4bd30c0fad97a2ce8a38440d95bd3b4b265dd.jpg)



Fig. 3. The observation results of the lumped disturbance at different frequencies.


## IV. SIMULATION EXPERIMENT FOR HODO PROPERTIES AND NOISE ANALYSIS

In theory, HODO can accurately observe the fast time-varying lumped disturbances, which benefits from observing the derivative of the lumped disturbances. We set up the simulation of a single-joint robot in MATLAB Simulink with lumped disturbances at different frequencies to verify this conclusion. Meanwhile, a theoretical discussion and a simulation experiment about the effect of measurement noise on acceleration estimation performance are also conducted. 

The detailed settings of the simulations are as follows: the joint inertia M is assumed to be 5 kg·m<sup>2</sup>, the link is a rod with 1 m in length and 10 kg in weight, and its mass is evenly distributed on the rod. The initial configuration of the joint is parallel to the direction of gravity. In the simulation, the joint torque and external torque compensation are set to 0. The sampling frequency of the data is designed to be 1000 Hz. 

## A. Simulations for Observing Fast Time-Varying Lumped Disturbances

In pHRI, the influence of external torques on the robot has more high-frequency components than model disturbances. Therefore, the accurate observation of fast time-varying disturbances is no longer a problem as long as HODO has the bandwidth up to the frequency range of human motions, which is below 8Hz [23]. The lumped disturbance $\mathrm { x _ { 2 } }$ is designated as a variable frequency sinusoid whose amplitude is $2 \mathrm { r a d } \bar { \mathsf { I s } } ^ { 2 }$ (shown in Fig. 3(a)). Its frequency varies from 6 Hz to 8 Hz in 8 seconds to simulate high-frequency disturbances. 

We set up two groups in the simulation to clarify the properties ofHODO. The first group observes lumped disturbance using the HODO-based method, as described in (21). In the second group, we set $\mathrm { { u } _ { 3 } = 0 }$ , and other settings are consistent with the first group, which equivalent to the SOSML-based method described in (10) when there are no external torques. In the case of a single degree of freedom, the gain matrices $\mathrm { K } _ { j } , j = 1 , 2 , \ldots , 6$ degenerate into real positive parameters $k _ { j } , j = 1 , 2 , \ldots , 6$ . The criterion of the gain parameter selection is to find a trade-off between monitoring bandwidth and noise amplification. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/626617727851a39fdac4dbbccf96ddd5f41e9dbf999d48d0861fc0b956836ad9.jpg)



Fig. 4. Influence of observer parameters on noise amplification.


Fig. 3 shows the observations of lumped disturbances in the simulation. In the case of SOSML, the increase of the lumped disturbance frequency leads to the phase lag and the observation error growth. Compared with the SOSML-based method, the HODO-based method is superior at different frequencies, proving that the HODO can achieve accurate observation for fast time-varying lumped disturbance in pHRI. This conclusion can be extended to the multi-degree-of-freedom case since independent parameters control the observers for each joint. 

## B. Noise Analysis

The HODO can achieve the accurate observation of fast timevarying disturbances, but the uncertainty of input (22) will affect the estimation performance of joint acceleration. The equation for the estimated joint acceleration with the system inputs can be derived by reshaping (21). 

$$
\dot {\hat {x}} _ {1} = u _ {1} + f ^ {*} + \int \left(u _ {2} + \int u _ {3}\right)\tag{24}
$$

In practice, only measurements of actual joint velocities are available, obtained by first-order numerical differentiation in joint positions. Therefore, the main uncertainty of input is due to measurement noise of joint velocity. 

$$
\dot {\mathrm{q}} _ {m e s} = \dot {\mathrm{q}} + \kappa\tag{25}
$$

where κ is the noise term caused by numerical differentiation in joint positions. The effects of κ on nominal models and numerical integral can be negligible. Applying (22) to (24), the uncertainty of estimated joint accelerations due to the noise κ can be described as 

$$
\dot {\hat {\mathrm{x}}} _ {1 \kappa} = \mathrm{K} _ {1} | \mathrm{e} _ {1} + \kappa | ^ {1 / 2} \mathrm{sign} (\mathrm{e} _ {1} + \kappa) + \mathrm{K} _ {2} \kappa\tag{26}
$$

where the values of $\mathrm { K } _ { 1 }$ and $\mathrm { K _ { 2 } }$ play an important role in noise amplification. A simulation experiment is implemented on the single-joint robot to discuss the effect of $\mathrm { K } _ { 1 }$ , and $\mathrm { K _ { 2 } }$ . The other parameters of the observer are set to be consistent in the simulation. The measurement noise of the joint position is set up as a gaussian random signal, whose variance is 0.5 and the amplitude is $2 . 6 1 8 \mathrm { e } ^ { - 5 }$ rad. Related parameters design refers to the product manuals of the position sensor Netzer VLX-60. The joint acceleration is required to keep at zero. 

The simulation results show that the effect of $\mathrm { K } _ { 1 }$ is more significant than $\mathrm { K _ { 2 } }$ on noise amplification, shown in Fig. 4. This phenomenon can be discussed theoretically by (26). When $\mathrm { e _ { 1 } }$ approaches $0 ,$ κ will significantly affect the switching term $\mathrm { s i g n } ( \mathrm { e } _ { 1 } + \kappa )$ . Hence, $\dot { \hat { \mathbf { x } } } _ { 1 \kappa }$ is more sensitive to the value of $\mathrm { K } _ { 1 }$ A practical noise suppression method is choosing the proper $\mathrm { K } _ { 1 }$ while ensuring the amplitude estimation performance of the joint acceleration. In addition, the superior actual joint velocity acquisition method can intuitively suppress joint velocities measurement noise, such as in literature [29]. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/b67fec9101e9d4522abc3e703921c1b9b6fe78b1cfb2b030c19937fc863d4680.jpg)



Fig. 5. The sitting lower limb rehabilitation robot and the desired periodic lifting motion.


## V. VALIDATION EXPERIMENTS

Validation experiments are carried out with a sitting lower limb rehabilitation robot. The robot comprises three joints, as shown in Fig. 5. Joint 1 is fixed with the pedestal, and the three joints are placed in parallel, so the robot can be considered as a three-link planar robot. Two encoders are placed on the joints’ motor side and link side to form the joint torque sensing system. The encoders are Netzer VLX-60. Assuming that the rehabilitation robot’s joints satisfy Hooke’s law, the joint torque can be obtained by multiplying the joint stiffness and the difference between the encoder values on both sides [30]. Joint stiffnesses and other nominal model parameters in (19) are obtained from the Computer-Aided Design (CAD) model and static structural analysis in the finite element simulation. For comparison with the proposed method, the acceleration sensors (model 356A01) are placed on the joints’ links to collect the corresponding joints’ acceleration signals. The communication method between the sensors and the host computer is CANBus, and the communication frequency is 1000 Hz. The calibration procedure for the accelerometer follows the product manual. The robot tracks a simple trajectory in the experiment, i.e., joint 2 performs a periodic lifting motion (refer to Fig. 5), and joints 1 and 3 both hold at the initial position. The host computer prints the collected data in real-time. Then, the comparative experiments are conducted to discuss the advantages of the proposed method. 

## A. Data Processing

The inherent vibration of the robot will affect the measurement accuracy of the accelerometers, and the noise will overwhelm the data collected by the accelerometers in a severe case. Therefore, filtering is necessary to obtain reliable acceleration data from accelerometers. In this letter, the fifth-order Butterworth low-pass filter is used to filter the raw data measured by the accelerometer to suppress noise, and the filter’s cutoff frequency is set to 50 Hz. Further, to ensure the same signal lag, the same filter is used to filter the joint accelerations estimated by the other methods in the experiments. Moreover, Zero-Phase filtering can achieve zero phase distortion while eliminating the high-frequency noise of the signal, so it is used to deal with the second-order numerical differentiation of joint positions to obtain the actual accelerations. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/9c7d1d733798852f6bb4a8d0e63fcce7d53058d0f1d77ed95449efb656f119fa.jpg)



(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/adca4e57e0f2223184548ce63c3249cae33c87aea88a735930fae48b300a258f.jpg)



(b)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/a678acdc625b48d3f40211b29c23ba97364567c1b17134957047614f9c3ad01d.jpg)



(c)



Fig. 6. The real-time system states and estimations of joint 2 acceleration in case1 using three different methods, where no external torques are applied to the robot. (a) Measured system states. (b) Estimation of joint acceleration. (c) Amplitude spectrum (0-8Hz) of joint accelerations using different methods.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/4822d39451c0cbbf07b5b882306a8ab26657c40530a3562059c18efff09cf40b.jpg)



(a)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/dbef77b334f7c3d47fc99459e01416dead6d4c32b9e69746be8c85cc925b6fd3.jpg)



(b)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/3e9cd3e3a604b23cd366e3d2e09597fcbbcafbf9340f22785ae238abf45dad45.jpg)



(c)



Fig. 7. The real-time system states and estimations of joint 2 acceleration in case2, using three different methods, where random external torques are applied to the robot. (a) Measured system states. (b) Estimation of joint acceleration. (c) Amplitude spectrum (0–8 Hz) of joint accelerations using different methods.


## B. Comparative Experiments

A recently proposed joint acceleration estimation method based on momentum observer enabled finite-time estimation of joint acceleration [19]. Comparative experiments are carried out to verify the proposed HODO-based method’s effectiveness. In the experiment, Two cases are recorded independently. Case 1 is that the robot moves without any external torque. In Case 2, the operator manually adds the random external torque to the robot in motion. Three methods, the HODO-based estimation method, the estimation method based on momentum observer [19], and the direct measurement by accelerometers, are used in each case to acquire the acceleration, respectively. Moreover, the selection criterion of gain coefficients is that the estimated joint accelerations under the two estimation methods have the same noise level. 

Figs. 6 and 7 show the measured system states and real-time estimations of joint 2 acceleration during one motion period in Case 1 and Case 2, using three different methods, respectively. As shown in Figs. 6(b) and 7(b), the HODO-based method can more precisely observe acceleration in both cases compared to the other two methods. The estimation method based on momentum observer performs poorly at some sampling points since it requires perfectly known robot dynamics and is more sensitive to model disturbances [19]. Moreover, as shown in Figs. 6(c) and 7(c), when the signal frequency is higher than 1 Hz, the acquired joint accelerations by the accelerometer and HODO-based method both have good tracking accuracy for the actual value. However, the HODO-based method is superior in the signal frequency range of 0–1 Hz, while the accelerometer has poor performance limited by its bandwidth. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/f4f10ce985dbb184a798211658a8aaf573772bd467da9cfb55fc196887841f6f.jpg)



(a) Estimation of joint acceleration


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/8ddf5a9d6b9d56359112746aeba0dcc39bceef5469affc82344fb1ad124dc6d3.jpg)



(b) Amplitude spectrum of joint accelerations



Fig. 8. An experiment about the effect of external torque compensation (ETC) for acceleration estimation of joint 2.


## C. External Torque Compensation

In Case 2, the HODO-based methods with and without external torque compensation are used for acceleration estimation in the experiment. Fig. 8 shows the acceleration estimation results in joint 2. The results show that the method without external torque compensation has a more significant joint acceleration estimation error than the method with external torque compensation, especially in the low-frequency range (0–5 Hz), which is consistent with the description in (18). Therefore, external torque compensation is essential for suppressing the lumped disturbances caused by contact with large amplitude in pHRI. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/058f578746d24dca8d50cd34e60e6366416dd22d796dc52ef36cccadd505ff25.jpg)



(a) Estimation of joint acceleratior


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/b702ceb5-7ec0-479a-b339-8805f8376a90/9ea803ae8634d92bd1b24bde7d4f816b6934eb5ecd7970ec89d550e4f0c170d0.jpg)



(b) Amplitude spectrum of joint accelerations and assumed disturbances



Fig. 9. The joint 2 acceleration estimations in case2 with disturbances.


## D. The Validation With Fast Time-Varying Disturbances

The ability of the proposed method to observe the fast timevarying lumped disturbances in pHRI is verified in this experiment. The assumed sinusoidal disturbances at frequencies of 6 Hz, 7 Hz, and 8 Hz with an amplitude of 1 rad/s<sup>2</sup> are added directly to the lumped disturbances. Fig. 9 shows joint 2 acceleration estimation performance in Case2 with assumed disturbances. It shows that the assumed disturbances are significantly suppressed in the corresponding frequency range. It can be concluded that the proposed method can effectively observe the fast time-varying lumped disturbances in pHRI. 

## VI. CONCLUSION AND DISCUSSION

This letter proposes a HODO-based acceleration estimation method, which can achieve finite-time acceleration estimation. The derivative of lumped disturbances can be observed by combining the advantages of linear and nonlinear disturbances observation algorithms. The external torque estimated by the GM observer is compensated to suppress the main system uncertainties. Simulations reveal that the proposed HODO can observe effectively fast time-varying disturbances in pHRI. The effect of noise on the proposed method is also theoretically discussed. Comparative experiments are carried out with a sitting lower limb rehabilitation robot. The experimental results show that the HODO-based method performs better than direct measurement by accelerometers and the estimation method based on momentum observer, both with or without interaction forces. Finally, the proposed method’s external torque compensation necessity and disturbance rejection capability are validated. 

The acceleration observed with the HODO-based method is expected to use in feedback control to achieve pHRI tasks next. 

## APPENDIX

The finite-time stability of HODO is proved in Appendix. The estimated error of joint velocity is considered to be bounded as $- L _ { 1 }  { \mathrm { I } _ { n \times n } } \leq  { \mathrm { e } } _ { 1 } \leq L _ { 1 }  { \mathrm { I } _ { n \times n } }$ , where $L _ { 1 }$ is a positive real number. Define a vector: 

$$
\boldsymbol {\zeta} ^ {\mathrm{T}} = [ \boldsymbol {\zeta} _ {1}, \boldsymbol {\zeta} _ {2}, \boldsymbol {\zeta} _ {3} ] = \left[ | \mathrm{e} _ {1} | ^ {1 / 2} \mathrm{sign} (\mathrm{e} _ {1}), \mathrm{e} _ {2}, \mathrm{e} _ {3} \right]\tag{27}
$$

The derivative of ζ is given by 

$$
\dot {\zeta} = (1 / | \zeta_ {1} |) A \zeta\tag{28}
$$

where $| \zeta _ { 1 } | = | \mathrm { e } _ { 1 } | ^ { 1 / 2 }$ , and 

$$
\mathrm{A} = \left[ \begin{array}{c c c} - 0. 5 \left(\mathrm{K} _ {1} + \mathrm{K} _ {2} | \mathrm{e} _ {1} | ^ {1 / 2}\right) & 0. 5   \mathrm{I} _ {\mathrm{n} \times \mathrm{n}} & 0 \\ - (\mathrm{K} _ {3} + | \mathrm{e} _ {1} | \mathrm{K} _ {4}) & 0 & | \mathrm{e} _ {1} | ^ {1 / 2} \mathrm{I} _ {\mathrm{n} \times \mathrm{n}} \\ - (\mathrm{K} _ {5} + | \mathrm{e} _ {1} | \mathrm{K} _ {6}) & 0 & 0 \end{array} \right]
$$

The following quadratic function is considered as a Lyapunov equation. 

$$
V (e) = \boldsymbol {\zeta} ^ {\mathrm{T}} P \boldsymbol {\zeta}\tag{29}
$$

where $P$ is a positive definite matrix. Differentiating (29), we get 

$$
\dot {V} (e) = - | \mathrm{e} _ {1} | ^ {- 1 / 2} \boldsymbol {\zeta} ^ {\mathrm{T}} Q \boldsymbol {\zeta}\tag{30}
$$

where $P$ and $Q$ are related by the Algebraic Lyapunov Equation. Satisfying 

$$
A ^ {\mathrm{T}} P + P A = - Q\tag{31}
$$

If Assumption 1 holds, then $0 \leq \left| \mathbf { e } _ { 1 } \right| ^ { 1 / 2 } \leq \sqrt { L _ { 1 } } \mathrm { I } _ { n \times n }$ . It will be divided into two cases to discuss the stability of (23). If $\mathrm { e } _ { 1 } \neq 0$ , A can be Hurwitz by choosing the suitable positive real diagonal matrix $\mathrm { K } _ { j } , j = 1 , 2 , \dotsc , 6$ . In this case, there is a unique solution of (31) for each positive definite matrix $Q = Q ^ { \mathrm { T } }$ , so it can be proved that $V ( e )$ is a strict Lyapunov Function. When $\mathrm { { e } } _ { 1 } = 0$ , the aforementioned conclusion does not hold. 

Next, we will discuss the stability of the equilibrium $\mathrm { { e } _ { 1 } = 0 }$ Equation (29) can be expanded as 

$$
\begin{array}{r} V (e) = \boldsymbol {\zeta} ^ {\mathrm{T}} P \boldsymbol {\zeta} = p _ {1 1} | e _ {1} | + p _ {2 2} e _ {2} ^ {2} + p _ {3 3} e _ {3} ^ {2} + 2 p _ {2 3} e _ {2} e _ {3} + \dots \\ \dots + 2 p _ {1 2} | e _ {1} | ^ {1 / 2} e _ {2} \mathrm{sign} (e _ {1}) + 2 p _ {1 3} | e _ {1} | ^ {1 / 2} e _ {3} \mathrm{sign} (e _ {1}) \end{array}\tag{32}
$$

$V ( e )$ is an absolutely continuous (AC) function of $e [ 3 1 ] . p _ { 1 1 }$ 5 $p _ { 1 2 } , p _ { 1 3 } , p _ { 2 3 } , p _ { 2 2 } , p _ { 3 3 }$ are the sub-blocks of the matrix P at the corresponding positions. 

The finite-time stability of the observer can be derived utilizing the Lyapunov theory when the Lyapunov function is continuously differentiable. However, $V ( e )$ does not satisfy this condition on the set $S = \{ ( e _ { 1 } , e _ { 2 } , e _ { 3 } ) \in \mathbb { R } ^ { 3 } | e _ { 1 } = 0 \}$ . Another solution was proposed in [32], which required us to prove that the Lyapunov function $V ( \varphi ( t , e _ { 0 } ) )$ is an AC function, where $\varphi ( t , e _ { 0 } )$ is a solution to (23) from an arbitrary starting point e<sub>0</sub>. Then, $V ( e )$ still can work as a Lyapunov function. 

Substituting $\varphi ( t , e _ { 0 } )$ for e, (32) can be rewritten as 

$$
\begin{array}{l} V (\varphi (t, e _ {0})) = p _ {1 1}   | \varphi_ {1} (t, e _ {0}) | + p _ {2 2} \varphi_ {2} (t, e _ {0}) ^ {2} \cdot \cdot \\ \quad + \cdot \cdot \cdot + p _ {3 3} \varphi_ {3} (t, e _ {0}) ^ {2} + 2 p _ {2 3} \varphi_ {2} (t, e _ {0}) \varphi_ {3} (t, e _ {0}) \cdot \cdot \\ \quad + \cdot \cdot \cdot + 2 p _ {1 2} | \varphi_ {1} (t, e _ {0}) | ^ {1 / 2} \varphi_ {2} (t, e _ {0}) \text {sign} (\varphi_ {1} (t, e _ {0})) \cdot \cdot \\ \quad + \cdot \cdot \cdot + 2 p _ {1 3} | \varphi_ {1} (t, e _ {0}) | ^ {1 / 2} \varphi_ {3} (t, e _ {0}) \text {sign} (\varphi_ {1} (t, e _ {0})) \end{array}\tag{33}
$$

Note that $V ( \varphi ( t , e _ { 0 } ) ) = V \circ \varphi ( t , e _ { 0 } )$ is the combination of V and $\varphi ( t , e _ { 0 } )$ . They all are AC functions. However, a combination of two ACs is not necessarily an AC function unless either V and $\varphi ( t , e _ { 0 } )$ is Lipschitz or monotonic [31]. It is obvious that V is not Lipschitz at $e _ { 1 } = 0$ due to the term ${ \left| { \varphi _ { 1 } ( t , e _ { 0 } ) } \right| ^ { 1 / 2 } } \mathrm { s i g n } ( \varphi _ { 1 } ( t , e _ { 0 } ) )$ . In order to prove the stability of $( 2 3 ) , \varphi _ { 1 } ( t , e _ { 0 } )$ must be proved to be monotonous at $e _ { 1 } = 0$ 

Setting that $\varphi _ { 1 } ( t , e _ { 0 } )$ crosses zero at the instant $t = t _ { k }$ , and $\varphi _ { 2 } ( t _ { k } , e _ { 0 } ) \neq 0$ . According to (23), $\dot { \varphi } _ { 1 } ( t , e _ { 0 } ) \neq 0$ at the instant $t = t _ { k }$ . Therefore, $\varphi _ { 1 } ( t , e _ { 0 } )$ is shown to be monotonically increasing or decreasing during an interval containing $t _ { k }$ . Setting $\varphi _ { 2 } ( t , e _ { 0 } ) = 0$ at the instant $t = t _ { k }$ . Then, $\dot { \varphi } _ { 1 } ( t , e _ { 0 } ) = 0$ , and $\varphi _ { 1 } ( t , e _ { 0 } )$ will maintain as zero. Therefore, $V ( \varphi ( t , e _ { 0 } ) )$ is an absolutely continuous function, and then the stability of (23) is proved at $e _ { 1 } = 0 \quad$ 

Further, the boundary of $V ( e )$ can be determined by the eigenvalues of the positive definite matrix $P _ { - }$ 

$$
\lambda_ {\mathrm{min}} (P) \left\| \boldsymbol {\zeta} \right\| _ {2} ^ {2} \leq V (e) \leq \lambda_ {\mathrm{max}} (P) \left\| \boldsymbol {\zeta} \right\| _ {2} ^ {2}\tag{34}
$$

where $\lambda _ { \mathrm { m i n } } ( P )$ and $\lambda _ { \operatorname* { m a x } } ( P )$ are the maximum and minimum eigenvalues of the matrix P, respectively. So, $\left| \mathrm { e } _ { 1 } \right| ^ { 1 / 2 } \leq \| \zeta \| _ { 2 } \leq$ $\lambda _ { \mathrm { m i n } } ( P ) ^ { - 1 / 2 } V ( e ) ^ { 1 / 2 }$ . Equation (30) can be modified as follows. 

$$
\dot {V} (e) \leq - \sigma V (e) ^ {1 / 2}\tag{35}
$$

where $\sigma = \lambda _ { \operatorname* { m i n } } ( P ) ^ { 1 / 2 } \lambda _ { \operatorname* { m i n } } ( Q ) / \lambda _ { \operatorname* { m a x } } ( P ) > 0$ . Since (35) holds, the error dynamics of the observer (21) will converge to the origin in a finite time [15], [33]. 

## REFERENCES



[1] A. B. Zoss, H. Kazerooni, and A. Chu, “Biomechanical design of the Berkeley lower extremity exoskeleton (BLEEX),” IEEE/ASME Trans. Mechatron., vol. 11, no. 2, pp. 128–138, Apr. 2006. 





[2] H. Yu, S. Huang, G. Chen, Y. Pan, and Z. Guo, “Human–robot interaction control of rehabilitation robots with series elastic actuators,” IEEE Trans. Robot., vol. 31, no. 5, pp. 1089–1100, Oct. 2015. 





[3] S. Futami, N. Kyura, and S. Hara, “Vibration absorption control of industrial robots by acceleration feedback,” IEEE Trans. Ind. Electron., vol. IE-30, no. 3, pp. 299–305, Aug. 1983. 





[4] A. D. Luca, D. Schroder, and M. Thummel, “An acceleration-based state observer for robot manipulators with elastic joints,” in Proc. IEEE Int. Conf. Robot. Automat., 2007, pp. 3817–3823. 





[5] G. A. Pratt and M. M. Williamson, “Series elastic actuators,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. Hum. Robot Interaction Cooperative Robots, 1995, vol. 1, pp. 399–406. 





[6] A. Calanca and P. Fiorini, “A rationale for acceleration feedback in force control of series elastic actuators,” IEEE Trans. Robot., vol. 34, no. 1, pp. 48–61, Feb. 2018. 





[7] S. A. B. Birjandi, J. Kühn, and S. Haddadin, “Observer-extended direct method for collision monitoring in robot manipulators using proprioception and IMU sensing,” IEEE Robot. Automat. Lett., vol. 5, no. 2, pp. 954–961, Apr. 2020. 





[8] T. C. S. Hsia, T. A. Lasky, and Z. Guo, “Robust independent joint controller design for industrial robot manipulators,” IEEE Trans. Ind. Electron., vol. 38, no. 1, pp. 21–25, Feb. 1991. 





[9] M. Keppler, D. Lakatos, C. Ott, and A. Albu-Schäffer, “Elastic structure preserving (ESP) control for compliantly actuated robots,” IEEE Trans. Robot., vol. 34, no. 2, pp. 317–335, Apr. 2018. 





[10] M. Schiefer, R. Bono, R. D. Sill, P. Piezotronics, and N. Depew, “Improved low frequency accelerometer calibration,” in Proc. XIX IMEKO World Congr. Fundam. Appl. Metrology, 2009, pp. 2521–2525. 





[11] P. Zhao and Y. Zhou, “Active vibration control of flexible-joint manipulators using accelerometers,” Ind. Robot: Int. J. Robot. Res. Appl., vol. 47, no. 1, pp. 33–44, 2020, doi: 10.1108/IR-07-2019-0144. 





[12] J. G. Garcia, A. Robertsson, J. G. Ortega, and R. Johansson, “Automatic calibration procedure for a robotic manipulator force observer,” in Proc. IEEE Int. Conf. Robot. Automat., 2005, pp. 2703–2708. 





[13] G. Sebastian, Z. Li, V. Crocher, D. Kremers, Y. Tan, and D. Oetomo, “Interaction force estimation using extended state observers: An application to impedance-based assistive and rehabilitation robotics,” IEEE Robot. Automat. Lett., vol. 4, no. 2, pp. 1156–1161, Apr. 2019. 





[14] T. Ren, Y. Dong, D. Wu, and K. Chen, “Collision detection and identification for robot manipulators based on extended state observer,” Control Eng. Pract., vol. 79, pp. 144–153, Oct. 2018. 





[15] L. Han, J. Mao, P. Cao, Y. Gan, and S. Li, “Towards sensorless interaction force estimation for industrial robots using high-order finite-time observers,” IEEE Trans. Ind. Electron., vol. 69, no. 7, pp. 7275–7284, Jul. 2022. 





[16] K. Łakomy and R. Madonski, “Cascade extended state observer for active disturbance rejection control applications under measurement noise,” ISA Trans., vol. 109, pp. 1–10, Mar. 2021. 





[17] A. Izadbakhsh and N. Nikdel, “Chaos synchronization using differential equations as extended state observer,” Chaos, Solitons Fractals, vol. 153, Dec. 2021, Art. no. 111433. 





[18] W. Chen, J. Yang, L. Guo, and S. Li, “Disturbance-observer-based contro and related methods—An overview,” IEEE Trans. Ind. Electron., vol. 63, no. 2, pp. 1083–1095, Feb. 2016. 





[19] G. Garofalo, N. Mansfeld, J. Jankowski, and C. Ott, “Sliding mode momentum observers for estimation of external torques and joint acceleration,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 6117–6123. 





[20] A. Levant, “Sliding order and sliding accuracy in sliding mode control,” Int. J. Control, vol. 58, no. 6, pp. 1247–1263, 1993. 





[21] E. Sariyildiz, R. Oboe, and K. Ohnishi, “Disturbance observer-based robust control and its applications: 35th anniversary overview,” IEEE Trans. Ind. Electron., vol. 67, no. 3, pp. 2042–2053, Mar. 2020. 





[22] E. Sariyildiz, R. Mutlu, and C. Zhang, “Active disturbance rejection based robust trajectory tracking controller design in state space,” J. Dyn. Syst., Meas., Control, vol. 141, no. 6, 2019, Art. no. 061013, doi: 10.1115/1.4042878. 





[23] K. Kong, J. Bae, and M. Tomizuka, “Control of rotary series elastic actuator for ideal force-mode actuation in human–robot interaction applications,” IEEE/ASME Trans. Mechatron., vol. 14, no. 1, pp. 105–118, Feb. 2009. 





[24] K. M. Lynch and F. C. Park, Modern Robotics. Cambridge, U.K.: Cambridge Univ. Press, 2017. 





[25] S. E. Talole, J. P. Kolhe, and S. B. Phadke, “Extended-State-observer-based control of flexible-joint system with experimental validation,” IEEE Trans. Ind. Electron., vol. 57, no. 4, pp. 1411–1419, Apr. 2010. 





[26] J. Han, “From PID to active disturbance rejection control,” IEEE Trans. Ind. Electron., vol. 56, no. 3, pp. 900–906, Mar. 2009. 





[27] J. A. Moreno and M. Osorio, “A Lyapunov approach to second-order sliding mode controllers and observers,” in Proc. IEEE 47th Conf. Decis. Control, 2008, pp. 2856–2861. 





[28] A. D. Luca and R. Mattone, “Actuator failure detection and isolation using generalized momenta,” in Proc. IEEE Int. Conf. Robot. Automat., 2003, vol. 1, pp. 634–639. 





[29] P. R. Belanger, “Estimation of angular velocity and acceleration from shaft encoder measurements,” in Proc. IEEE Int. Conf. Robot. Automat., 1992, vol. 1, pp. 585–592. 





[30] M. W. Spong, “Modeling and control of elastic joint robots,” J. Dyn. Syst., Meas., Control, vol. 109, no. 4, pp. 310–318, 1987. 





[31] J. A. Moreno and M. Osorio, “Strict Lyapunov functions for the super-twisting algorithm,” IEEE Trans. Autom. Control, vol. 57, no. 4, pp. 1035–1040, Apr. 2012. 





[32] A. Bacciotti and L. Rosier, “Monotonicity and generalized derivatives,” in Liapunov Functions and Stability in Control Theory. Berlin, Germany: Springer, 2005, pp. 203–217. 





[33] H. Du, C. Qian, S. Yang, and S. Li, “Recursive design of finite-time convergent observers for a class of time-varying nonlinear systems,” Automatica, vol. 49, no. 2, pp. 601–609, 2013. 

