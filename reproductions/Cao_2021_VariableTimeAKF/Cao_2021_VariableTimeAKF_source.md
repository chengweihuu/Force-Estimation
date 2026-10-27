# Contact force and torque sensing for serial manipulator based on an adaptive Kalman filter with variable time period

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/8b273c77a06b54437ad7d57c11695b122448992648b7bc7755fe7d93e4d5b826.jpg)


Feng Cao <sup>a</sup>, Paul D. Docherty <sup>a,b,*</sup>, Siliang Ni <sup>a</sup>, XiaoQi Chen <sup>c</sup> 

<sup>a</sup> Mechanical Engineering Department, University of Canterbury, Christchurch, New Zealand 

<sup>b</sup> Institute for Technical Medicine (ITeM), Furtwangen University, Villingen-Schwenningen, Germany 

<sup>c</sup> Faculty of Science, Engineering and Technology, Swinburne University of Technology, Victoria, Australia 

## A R T I C L E I N F O

Keywords: Contact force and torque sensing Adaptive Kalman filter Mode-switching moving average Variable time period 

## A B S T R A C T

Contact force and torque sensing approaches enable manipulators to cooperate with humans and to interact appropriately with unexpected collisions. In this paper, a mode-switching moving average with variable time period is proposed to reduce the effects of measured motor current noise and thus provide improved confidence in joint output torque estimation. The time period of the filter adapts continuously to achieve optimal tradeoff between response time and precision of estimation in real-time. An adaptive Kalman filter that consists of the proposed moving average and the classical Kalman filter is proposed. Calibration routines for the adaptive Kalman filter take the measured motor current noise and errors in the speed data from the individual joints into account. The combination of the proposed adaptive Kalman filter with variable time period and its calibration method facilitates force and torque estimation without force/torque sensors. Contact force/torque sensing and response time assessments from the proposed approach were performed on the Universal Robot 5 manipulator with differing unexpected end effector loads. The combined force and torque sensing method led to a reduction of the estimation errors and response time in comparison with the pioneering method, and the effect is further improved as the payload rises. The proposed method can be applied to any robotic manipulators as long as the motor information (current, joint position, and joint velocities) are available and consequently the cost will be reduced dramatically from methods that require load cells 

## 1. Introduction

Robotic manipulators play an important role in various manufacturing tasks. However, robotic control strategies can fail to take environmental and operator interaction into account and can result in hazardous or unexpected situations. Force-control techniques are widely applied in cooperating manipulators [1–3], assembly [4], haptics [5], grinding [6–13], physical human-robot interaction [13–15], exoskeleton robots [16,17], and collision detection [18]. The force-control techniques require information regarding force exerted and torque on the robot in order to regulate the robotic motion within appropriate bounds. Historically, the most common method to measure the contact force and torque is to utilize the 6-axis force/torque sensors which are mounted on the end of the robotic manipulator, or torque sensors that are mounted between the link side and the motor side of the robot. With these sensitive sensors, hybrid force/position control [19] and impedance control [20] are well developed and have been introduced into various areas. However, the extension of these sensors is limited because of their high price. Furthermore, the sensors may become worn or damaged as the end effector usually conducts rigid contact with the environment. Finally, both the varying environment such as temperature and humidity and the narrow bandwidth caused by the sensors inevitable susceptibility to measurement noise will affect the performance of these sensors, and resultant motion control. 

To enable sensorless force/torque contact estimation, various observers for estimating the external wrench have been proposed. An early disturbance observer developed by Kiyoshi et al. [21] has been widely used as a basis for force/torque sensing. In sensorless force/torque contact estimation, when the manipulator interacts with the environment, the external wrench is regarded as the disturbance and estimated by comparing the actual output to the output of a nominal model. Further disturbance observer force/torque sensing research is presented in [22,23]. More recently, Chen et al. [24,25] improved the disturbance observer and applied it to nonlinear robotic systems. The nonlinear disturbance observer is also extensively used [26,27]. In order to utilize the disturbance observer, a real-time inverse inertia matrix of the robot must be continuously calculated leading to notable computational burden. 

Due to the development of compliance elements in robotics, various flexible robotic systems, such as rehabilitation robots, have been pro posed and thus the safety and backdrivability of physical Human-Robot Interaction systems have been improved. In particular, the exerted force/torque on the manipulator can be estimated with kinematic measurement. For instance, sensorless torque estimation of the stiff and sensitive joint is implemented due to the structural elasticity of the ro botic joints with harmonic drive transmission in [28–30] and only motor-side and link-side position measurements are required. However, this method is difficult to conduct since a prior model of the harmonic drive is required and it is very complicated to build. Furthermore, compliant elements ultimately lead to high costs that lead to preference for direct force measurement. Other sensorless external wrench sensing approaches, such as time delay estimation and virtual springs, can be found in [31] and [4] respectively. 

De Luca et al. [32–34] present a novel method, in which the robot dynamics are expressed in the format of the generalized momentum. This approach is originally proposed for collision detection and hybrid force/position control, but recent researchers have exploited its further application to lead-through programming [35]. With the motor current and Moore-Penrose generalized inverse matrix, the exerted force/torque on the end of the manipulator is estimated and applied to a combined control mode by Yuan et al. [36]. Similarly, Yen et al. [37] monitor the motor current and transform it to joint torques, which is then used to estimate the external wrench. Another collision detection method that depends on virtual power indices is proposed in [38]. At a fundamental level, [36–40] all have characteristics of De Luca’s generalized momentum dynamic model due to the model’s independence from joint acceleration measurements and inversion of the manipulator inertia matrix. However, the research arising from De Luca’s model is limited to collision detection or low-precision force/torque control since the noise in the measured current can trigger false positive collision recognition or insufficient force estimation precision. 

The Kalman filter is an effective method to realize real-time reduction of the measurement and estimation noise, of which the adaptive version is the most popular in recent years. For example, the variational Bayesian based adaptive Kalman filter (AKF) and suboptimal AKF are proposed in [41,42] and [43]. These methods perform well in spite of inaccuracies intrinsic within the measurement covariance matrices. Paolo et al. present a model-based AKF, which involves a structure-switching strategy, to realize the sensorless control of a piezo actuator [44] and monitoring of the health information with no physical sensors [45], respectively. The cascaded extended AKF [46] was utilized to estimate the immeasurable states of the piezo actuator. In order to estimate the real-time state of the power system, an error-prediction AKF with step-varying processes [47] and a two-stage AKF [48] were utilized, respectively. The two-stage AKF can also be employed [49] as a disturbance estimator in automotive applications. In order to enhance filter accuracy, the Sage-Husa AKF can be employed to estimate the statistical characteristics of the noise [50]. The constrained adaptive robust integration Kalman filter is used to realize a reliable and precise navigation solution [51]. In this navigation solution, the raw observations from all heterogeneous sensors are corrected by the pseudo ob servations derived from state equality constraint. A computationally efficient version of the variational adaptive Kalman filter is proposed to solve the filtering problem of transfer alignment with an inaccurate measurement noise covariance matrix [52]. An unscented AKF is presented in [53] to track sudden changes in stiffness of structural systems exposed to earthquake induced excitations. Variations of the AKF have been applied to vehicle navigation [54,55], UAV control [56], robot operation [57–59] and battery state detection [60]. 

Recently, the Kalman disturbance observer has been applied to sensorless contact force/torque estimation of the robots. Using the extended Kalman filter and a Lyapunov-based adaption law, Jung et al. designed an estimator and applied it to estimate the external wrench of a three-link manipulator [61]. Their proposed estimator was tolerant to model error and sensor noise. The estimator has been further improved and certain assumptions such as a-priori determination of underlying patterns that govern subject behavior can be omitted [62]. In [63], a semiparametric dynamic model that combines the parametric rigid-body dynamics and the nonparametric multilayer perception dynamics is proposed. With the composite model, the exerted force/torque is observed as a disturbance variable and estimated by the disturbance Kalman filter. Subsequently, the uncertainty in the dynamics is neglected as the unmodelled effects are accounted for by the nonparametric model. Consequently, empirical parameters are needed when calibrating the Kalman filter. With an automatic covariance matrix tuning scheme presented in [64], the problem of the neglected dynamic uncertainty in [63] was overcome. The scheme abstracts uncertainty in the system dynamics to improve the estimation quality. However, the covariance matrix in [64] is limited by the errors of the friction model that are generally assumed constant. Furthermore, the scheme still depends on joint torque sensors when modeling the friction and the current measurement noise is ignored. 

In this paper, an adaptive Kalman filter, which composes of a modeswitching moving average (MSMA) based on variable time period and the classical Kalman filter (CKF), is proposed and applied to contact force/torque sensing (CFTS) with no embedded force/torque sensors. In order to apply the CKF, the covariance matrix of the predicted state noise has to be calibrated. It was found that smaller diagonal elements in the covariance matrix led to faster CKF responses to the external wrench. Hence, reducing the magnitude of the diagonal elements of the covari ance matrix is hypothesized to reduce the response time. 

The main contributions of this paper are the nature of the AKF and the validation of its capacity in CFTS. A proposed MSMA that switches mode between the Hull moving average and the weighted moving average was designed to reduce the effect of noise in the measured motor current and reduce the magnitude of the diagonal elements of the covariance matrix. Due to the automatically-varying time period, the trends in the smoothed motor current signal are preserved when the motor output changes. Consequently, faster response and improved accuracy are realized despite of the time lag resulting from the moving average. A tuning strategy for the Kalman filter was provided and an approach for determining the time period of the MSMA is presented. 

In this paper, Section 2 presents the generalized-momentum-based model and its discretization methods. The proposed AKF and its implementation is also described in this section. Section 3 presents the sensorless identification method of the dynamic model, including the motor torque constant and the friction model. The MSMA based on variable time period and calibration approach of the AKF are proposed. In Section 4, experiments are conducted on the Universal Robot 5 (UR5) manipulator with the proposed CFTS method and the experimental results are shown. The experimental results and the effectiveness of the proposed approach are discussed in Section 5. Section 6 gives conclusion. 

## 2. Model development

2.1. Dynamic model of UR5 expressed in the format of the generalized momentum 

The rigid body dynamics of UR5 is expressed as 

$$
\boldsymbol {M} (\boldsymbol {q}) \ddot {\boldsymbol {q}} + \boldsymbol {C} _ {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) \dot {\boldsymbol {q}} + \boldsymbol {G} (\boldsymbol {q}) + \boldsymbol {\tau} _ {\mathrm{fri}} + \boldsymbol {\tau} _ {\mathrm{ext}} + \boldsymbol {\tau} _ {\mathrm{dri}} = 0\tag{1}
$$

where the inertia matrix is represented by $\pmb { M } ( \pmb q ) \in \mathcal { R } ^ { n \times n }$ and n is the degrees of freedom of the serial manipulator. The Coriolis and centrifugal matrix is captured by $C _ { C } ( \pmb q , \dot { \pmb q } )$ . The effect of the robot gravity is expressed as $\pmb { G } ( \pmb { q } ) \in \mathcal { R } ^ { n \times n } . \ \pmb { \tau } _ { \mathrm { f r i } } \in \mathcal { R } ^ { n }$ denotes the joint friction torques. $\pmb { \tau } _ { \mathrm { d r i } } \in \mathcal { R } ^ { n }$ is the joint driving torques. Joint position, velocity, and acceleration are described as $\ b q \in \mathcal { R } ^ { n } , \dot { \pmb q } \in \mathcal { R } ^ { n }$ , and $\ddot { \pmb q } \in \mathcal { R } ^ { n }$ respectively. The joint torques, which result from external contact forces and torques, are expressed as $\pmb { \tau _ { \mathrm { e x t } } } \in \mathcal { R } ^ { n }$ The external wrench $f =$ $\left[ \begin{array} { l l l l l } { f _ { x } } & { f _ { y } } & { f _ { z } } & { \tau _ { x } } & { \tau _ { y } } & { \tau _ { z } } \end{array} \right] ^ { T } \in \mathcal { R } ^ { m }$ is related to $\pmb { \tau } _ { \mathrm { e x t } }$ by the manipulator Jacobian $\pmb { J } ( \pmb q ) \in \mathcal { R } ^ { m \times n }$ (where m is the dimension of the external wrench) via the equation: $\begin{array} { r } { \pmb { \tau } _ { \mathrm { e x t } } = \pmb { J } ( \pmb q ) ^ { T } \pmb { f } , } \end{array}$ , where $f _ { x } , f _ { y } ,$ and $f _ { z }$ are the contact forces that occur at the tool center point (TCP) in $x , y ,$ and z directions of the base frame and $\tau _ { x } , \tau _ { y } ,$ , and $\tau _ { z }$ are the corresponding interaction torques. 

In the general case, calculation of f exerted on the manipulator relies on the Euler-Lagrange formulation of the manipulator dynamics described in Eq. (1). However, this method fails when real-time esti mation of the external wrench is conducted, since joint acceleration q ¨ is obtained online by numerical differentiation of the measurable joint velocity ${ \dot { \pmb q } } ,$ which will lead to amplification of the measurement noise. The dynamic model expressed in the format of the generalized momentum in [32–34] avoids the difficulty and the generalized momentum p of the manipulator is defined as $\pmb { p } = \pmb { M } ( \pmb { q } ) \dot { \pmb { q } }$ 

The derivate of the generalized momentum with respect to time is obtained as 

$$
\dot {\boldsymbol {p}} = \dot {\boldsymbol {M}} (\boldsymbol {q}) \dot {\boldsymbol {q}} + \boldsymbol {M} (\boldsymbol {q}) \ddot {\boldsymbol {q}}\tag{2}
$$

Following $\pmb { M } ( \pmb { q } ) - 2 C _ { C } ( \pmb { q } , \dot { \pmb { q } } ) = - ( \dot { \pmb { M } } ( \pmb { q } ) - 2 C _ { C } ( \pmb { q } , \dot { \pmb { q } } ) ) ^ { T }$ in [65], Eq. (1) can be converted into 

$$
\dot {\boldsymbol {p}} = \boldsymbol {C} _ {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) ^ {T} \dot {\boldsymbol {q}} - \boldsymbol {G} (\boldsymbol {q}) - \boldsymbol {\tau} _ {\mathrm{fri}} - \boldsymbol {\tau} _ {\mathrm{ext}} - \boldsymbol {\tau} _ {\mathrm{dri}}\tag{3}
$$

Defining the term ${ \pmb \tau } _ { \mathrm { i n p } } = C _ { C } ( \pmb q , \dot { \pmb q } ) ^ { T } \dot { \pmb q } - { \pmb G } ( \pmb q ) - { \pmb \tau } _ { \mathrm { f r i } } - { \pmb \tau } _ { \mathrm { d r i } }$ and substitut ing it into Eq. (3), the dynamics of the manipulator can be written as 

$$
\dot {\boldsymbol {p}} = \boldsymbol {\tau} _ {\mathrm{inp}} - \boldsymbol {J} (\boldsymbol {q}) ^ {T} \boldsymbol {f} + \boldsymbol {e} _ {p}\tag{4}
$$

where $\pmb { e } _ { p } \sim N ( 0 , \pmb { \sigma } _ { \mathrm { D y n } } ^ { 2 } )$ denotes the noise resulting from $\tau _ { \mathrm { i n p } } . ~ \sigma _ { \mathrm { D y n } } ^ { 2 }$ is assumed as a diagonal matrix reflecting that errors in the individual joints are independent and $\pmb { \sigma } _ { \mathrm { D y n } } ^ { 2 } \in \mathcal { R } ^ { n \times n }$ . The term $\pmb { \tau } _ { \mathrm { d r i } }$ included in Eq. (4) is obtained based on the noisy current. It is reasonable to consider the uncertainty in the measured current as the dominant noise in Eq. (4). Details of how the uncertainty is obtained can be found in Section 3. 

## 2.2. Discretization of the UR5 dynamics

The dynamics of the external wrench is defined as 

$$
\dot {\boldsymbol {f}} = \boldsymbol {S} \boldsymbol {f} + \boldsymbol {e} _ {f}\tag{5}
$$

where $\pmb { S } \in \mathcal { R } ^ { m \times m }$ is the dynamic matrix of the external wrench. The uncertainty of the external wrench is assumed to be normally distributed as $e _ { f } \sim N ( 0 , \sigma _ { f } ^ { 2 } ) , \sigma _ { f } ^ { 2 }$ is assumed as a diagonal matrix reflecting that un certainty of the external wrench in each directions are independent and $\sigma _ { f } ^ { 2 } \in \mathcal { R } ^ { m \times m }$ . In this research, S is defaulted to 0 and thus $\dot { \pmb f } = \pmb e _ { f }$ . Details of how S and $\sigma _ { f } ^ { 2 }$ are determined can be found in Section 3. 

With Eq. (4) and Eq. (5), the state vector can be defined as $x =$ $\left[ \boldsymbol { p } ^ { T } \boldsymbol { f } ^ { T } \right] ^ { T } \in \mathcal { R } ^ { n + m }$ . The generalized momentum p is available since both q and q ˙ can be measured and thus the output y of the system is obtained. The state model of the manipulator is expressed as 

$$
\begin{array}{l} \boldsymbol {x} = \boldsymbol {A x} + \boldsymbol {B u} + \boldsymbol {w} \\ \boldsymbol {y} = \boldsymbol {C x} + \boldsymbol {Z} \end{array}\tag{6}
$$

where $A \ = \ \left[ \begin{array} { c c } { { 0 _ { n \times n } } } & { { - { \pmb J } ( { \pmb q } ) ^ { T } } } \\ { { 0 _ { m \times n } } } & { { \hphantom { - } S } } \end{array} \right] , B \ = \ \left[ \begin{array} { c c } { { \pmb I _ { n } } } \\ { { 0 _ { m \times n } } } \end{array} \right]$ , and $\pmb { C } = \left[ \pmb { I _ { n } } \quad \mathbf { 0 } _ { n \times m } \right]$ are the state matrices. $I _ { n } \in \mathcal { R } ^ { n \times n }$ is the identify matrix. $\pmb { w } = \left[ \pmb { e } _ { p } ^ { T } \quad \pmb { e } _ { f } ^ { T } \right] ^ { T } \in \mathcal { R } ^ { n + m }$ denotes the covariance matrix of the predicted state noise. $u = \pmb { \tau } _ { \mathrm { i n p } }$ is the input of the state model and the measurement noise is $\mathbf { Z } \sim N ( 0 , \sigma _ { \mathrm { m e a } } ^ { 2 } )$ with ${ \pmb { \sigma } } _ { \mathrm { m e a } } ^ { 2 } \in \mathcal { R } ^ { n \times n }$ 

In order to apply the AKF, the state space dynamics in Eq. (6) are discretized as 

$$
\begin{array}{l} \boldsymbol {x} _ {k + 1} = \boldsymbol {A} _ {k + 1} \boldsymbol {x} _ {k} + \boldsymbol {B} _ {k + 1} \boldsymbol {u} _ {k + 1} + \boldsymbol {w} _ {k + 1} \\ \boldsymbol {y} _ {k + 1} = \boldsymbol {C} _ {k + 1} \boldsymbol {x} _ {k + 1} + \boldsymbol {z} _ {k + 1} \end{array}\tag{7}
$$

where k and $k + 1$ are the time steps of the discrete system. With the method proposed in [66], the discretized matrices $A _ { k + 1 } , B _ { k + 1 }$ <sub>1</sub> and $C _ { k + 1 }$ are obtained by 

$$
\begin{array}{l} \left[ \begin{array}{c c} \boldsymbol {A} _ {k + 1} & \boldsymbol {B} _ {k + 1} \\ 0 _ {n \times (n + m)} & \boldsymbol {I} _ {n} \end{array} \right] = e x p \Bigg (\left[ \begin{array}{c c} \boldsymbol {A} & \boldsymbol {B} \\ 0 _ {n \times (n + m)} & 0 _ {n \times n} \end{array} \right] t _ {s} \Bigg) \\ \boldsymbol {C} _ {k + 1} = \boldsymbol {C} \end{array}\tag{8}
$$

The measurement noise covariance matrix is calculated by 

$$
\sigma_ {k + 1} ^ {2} = \sigma_ {\text { mea }} ^ {2} / t _ {s}\tag{9}
$$

where $t _ { s }$ is the sampling time. Following [67], the discretized predicted state noise matrix $\boldsymbol { Q } _ { k + 1 }$ is given by 

$$
\begin{array}{l} \left[ \begin{array}{c c} \boldsymbol {M} _ {k + 1} ^ {1 1} & \boldsymbol {M} _ {k + 1} ^ {1 2} \\ 0 _ {(n + m) \times (n + m)} & \boldsymbol {M} _ {k + 1} ^ {2 2} \end{array} \right] = e x p \bigg (\left[ \begin{array}{c c} \boldsymbol {A} & \boldsymbol {B} \\ 0 _ {n \times (n + m)} & 0 _ {n \times n} \end{array} \right] t _ {s} \bigg) \\ \boldsymbol {Q} _ {k + 1} = \left(\boldsymbol {M} _ {k + 1} ^ {2 2}\right) ^ {T} \boldsymbol {M} _ {k + 1} ^ {1 2} \end{array}\tag{10}
$$

where the covariance matrix of the predicted state noise is expressed as 

$$
\pmb {\sigma} _ {\mathrm{pro}} ^ {2} = \left[ \begin{array}{c c} \pmb {\sigma} _ {\mathrm{Dyn}} ^ {2} & 0 _ {n \times m} \\ 0 _ {m \times n} & \pmb {\sigma} _ {f} ^ {2} \end{array} \right].
$$

## 2.3. Adaptive Kalman filter

In order to achieve faster estimation response as well as improved estimation accuracy, an adaptive Kalman filter is proposed (as shown in Fig. 1). In contrast to the classical filter that directly utilizes the measured current, the measured signal is regulated by the modeswitching moving average based on variable time period first and then used to enable estimation of the contact force and torque (CFT) with the classical Kalman filter. 

The schematic of the mode-switching moving average based on variable time period is displayed in Fig. 2. where t denotes the time period, d is the step time, $i _ { d } ^ { o p t }$ stands for the filtered current at step time d, δ is a experimentally determined parameter that is used to decide whether the time period should be varied. $W ( d , t )$ is the Weighted Moving Average and is calculated by 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/7a630ee75f422743a10f92592a53c2842df4410f784cbec6aa3fae792dd02798.jpg)



Fig. 1. Adaptive Kalman filter.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/562dda1b6cfd9d088b002b638af76a7f6a9fecc2c6cf4519e24686ba2f4ae156.jpg)



Fig. 2. Mode-switching filter based on variable time period.


$$
W (d, t) = 2 \left[ i _ {d} t + i _ {d - 1} (t - 1) + \dots + i _ {d - t + 1} \right] / [ t (t + 1) ]\tag{11}
$$

H(d, t) is the Hull Moving Average, and the closing current is obtained by 

$$
H (d, t) = W \left\{\{2 W [ d, \operatorname{int} (t / 2) ] - W (d, t) \}, \operatorname{int} (\sqrt {t}) \right\}\tag{12}
$$

## 2.4. Implementation of the adaptive Kalman filter

With Eqs. (7)–(12), the AKF is implemented as follows: 

Step 1. Initialize the state vector and covariance matrix $P _ { 0 }$ 

In the general case, the initial state vector and the initial covariance matrix are defined as ${ \pmb x } _ { 0 } = { \pmb 0 } _ { ( m + n ) } \ \times \ { \pmb 1 }$ and $P _ { 0 } = I _ { m + n }$ respectively. How ever, a rough estimation can be considered as the initial value if known a priori. 

Step 2. Filter the measured current. 

The motor current is measured online and regulated by the MSMA in each circulatory computation following Eq. (11) and Eq. (12) (as shown in Fig. 2). 

Step 3. Discretize the system matrices. 

A is a time-varying matrix since it contains the joint-position-based manipulator Jacobian $\pmb { J } ( \pmb q )$ . As a result, the system matrices have to be discretized in each computation following Eqs. (8)–(10). 

Step 4. Predict the state vector and the covariance matrix 

$$
\begin{array}{l} \overline {{\boldsymbol {x} _ {k + 1}}} = \boldsymbol {A} _ {k + 1} \boldsymbol {x} _ {k} + \boldsymbol {B} _ {k + 1} \boldsymbol {u} _ {k + 1} \\ \overline {{\boldsymbol {P} _ {k + 1}}} = \boldsymbol {A} _ {k + 1} \boldsymbol {P} _ {k} \boldsymbol {A} _ {k + 1} ^ {T} + \boldsymbol {Q} _ {k + 1} \end{array}\tag{13}
$$

Step 5. Update the Kalman gain 

$$
\boldsymbol {K} _ {k + 1} = \overline {{\boldsymbol {P} _ {k + 1}}} \boldsymbol {C} _ {k + 1} ^ {T} / \left(\boldsymbol {C} _ {k + 1} \overline {{\boldsymbol {P} _ {k + 1}}} \boldsymbol {C} _ {k + 1} ^ {T} + \boldsymbol {\sigma} _ {k + 1} ^ {2}\right)\tag{14}
$$

Step 6. Update the state vector and covariance matrix with the measurement 

$$
\begin{array}{l} \boldsymbol {x} _ {k + 1} = \overline {{\boldsymbol {x} _ {k + 1}}} + \boldsymbol {K} _ {k + 1} (\boldsymbol {y} _ {k + 1} - \boldsymbol {C} _ {k + 1} \overline {{\boldsymbol {x} _ {k + 1}}}) \\ \boldsymbol {P} _ {k + 1} = (\boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {C} _ {k + 1}) \overline {{\boldsymbol {P} _ {k + 1}}} \end{array}\tag{15}
$$

Step 7. Compute the external wrench and then repeat Step 2 

$$
\boldsymbol {f} _ {k + 1} = \left[ \begin{array}{c c} 0 _ {m + n} & \boldsymbol {I} _ {m} \end{array} \right] \boldsymbol {x} _ {k + 1}\tag{16}
$$

## 3. Identification of the dynamic parameters and calibration of the AKF

In order to implement the AKF, the dynamic model of the manipulator must be defined and identification of the unknown dynamic parameters must be performed. Furthermore, the predicted state noise matrix $\sigma _ { \mathrm { D y n } } ^ { 2 } , \sigma _ { f } ^ { 2 }$ , and the measurement noise matrix $\pmb { \sigma } _ { \mathrm { m e a } } ^ { 2 }$ must be calibrated. In [35] and [63], the motor constant and gearbox ratio are given and therefore the dynamic model of $\pmb { \tau } _ { \mathrm { d r i } }$ is obtained. Furthermore, in [14] and [63], joint torque sensors are used to calibrate the friction model. However, the proposed approach aims to avoid using joint torque sensors and an alternative is proposed. An analysis of UR5 joints (Fig. 3) was conducted to determine friction characteristics. 

## 3.1. Identification of the motor torque constant and friction

It is to be noticed that, in this brief, only the motor current $i ,$ joint position q, and joint velocity q ˙ are measurable. The joint driving torque is calculated by 

$$
\widehat {\boldsymbol {\tau} _ {\mathrm{dri}}} = \boldsymbol {k} \circ \boldsymbol {r} \circ \boldsymbol {i}\tag{17}
$$

where k and r denote the motor torque constant and gearbox ratio, respectively. A composite term is subsequently defined as 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/6eea7188e6bd9b7e33e7cc9a817ba0e506f343552d99cf62618b0ed85f8be92d.jpg)



Fig. 3. Sketch of the UR5 [68].


$$
\boldsymbol {D} = \boldsymbol {k} \circ \boldsymbol {r}\tag{18}
$$

The term D is determined experimentally by moving the joints at a constant speed clockwise and counterclockwise while the other joints stay stationary (as shown in Fig. 4). 

When the elbow joint rotates clockwise and counterclockwise without external wrench, the manipulator dynamics described in terms of current can be expressed as 

$$
\begin{array}{l} \boldsymbol {i} _ {\text {gra}} + \boldsymbol {i} _ {\text {fri}} + \boldsymbol {i} _ {\text {dri}} ^ {+} = 0 \\ \boldsymbol {i} _ {\text {gra}} - \boldsymbol {i} _ {\text {fri}} + \boldsymbol {i} _ {\text {dri}} ^ {-} = 0 \end{array}\tag{19}
$$

where $i _ { \mathrm { g r a } }$ is the current related to the elbow joint torque resulting from the gravity of Link 3, Link $^ { 4 , }$ Link 5, Link 6, Wrist 1, Wrist 2, and Wrist 3. $i _ { \mathrm { f r i } }$ denotes the current corresponding to the elbow joint friction. $i _ { \mathrm { d r i } } ^ { + }$ and $i _ { \mathrm { d r i } } ^ { - }$ indicate the motor currents with the elbow joint moving clockwise and counterclockwise respectively. With Eq. (19), $i _ { \mathrm { g r a } }$ is calculated by 

$$
\boldsymbol {i} _ {\text { gra }} = \left(- \boldsymbol {i} _ {\text { dri }} ^ {+} - \boldsymbol {i} _ {\text { dri }} ^ {-}\right) / 2\tag{20}
$$

An accurate dynamic model is assumed and thus the term D can be obtained by 

$$
\boldsymbol {D} = \boldsymbol {G} (\boldsymbol {q}) / \boldsymbol {i} _ {\text { gra }}\tag{21}
$$

Generally, the friction increases discontinuously as the relative velocity of the contact surfaces increases. Furthermore, the friction is affected by the loading and temperature. However, to simplify the model, only Coulomb friction and viscous friction are considered in this approach. The friction model is defined: 

$$
\boldsymbol {\tau} _ {\mathrm{fri}} (\dot {\boldsymbol {q}}) = \boldsymbol {f} _ {C} + \boldsymbol {f} _ {\dot {q}} \dot {\boldsymbol {q}}\tag{22}
$$

where $f _ { C }$ is the Column friction and $f _ { \dot { q } }$ is the viscous friction coefficient. Details of more friction models can be found in [69]. With Eq. (19) and Eq. (21), the friction of the elbow joint moving at different speeds is obtained 

$$
\boldsymbol {\tau} _ {\mathrm{fri}} (\dot {\boldsymbol {q}}) = \boldsymbol {D} \left(- \boldsymbol {i} _ {\mathrm{dri}} ^ {+} + \boldsymbol {i} _ {\mathrm{dri}} ^ {-}\right) / 2\tag{23}
$$

Consequently, the unknown parameters $f _ { C }$ and $f _ { \dot { q } }$ in Eq. (22) can be identified a-priori. 

## 3.2. Covariance matrix of predicted state noise

In order to implement the AKF, the predicted state noise w and the measurement noise Z must be calculated. Note that the term $\pmb { \tau } _ { \mathrm { i n p } }$ con tains the noise caused by the measurement of q, q˙, and i. In contrast to [64], where the noise resulting from q and i were ignored, the current approach compiles and analyses the kinematic measurement noise. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/5ef9756686125fb76c5359002e7ed70e09a28c70c2d25e8e9d2050f63c92a53e.jpg)



Fig. 4. Identification of the motor torque constant and friction.


Initially, experiments without external wrench are performed. Since the joint acceleration is not directly measurable, the individual joints were set to move at a constant speed in the experiments and the ideal joint output torques were calculated by 

$$
\overline {{\boldsymbol {\tau} _ {\mathrm{dri}}}} = - \boldsymbol {C} _ {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) \dot {\boldsymbol {q}} - \boldsymbol {G} (\boldsymbol {q}) - \boldsymbol {\tau} _ {\mathrm{fri}}\tag{24}
$$

In order to obtain the model uncertainty, Anderson-Darling test was conducted to analyze the errors between the target driving torques $\overline { { \pmb { \tau } _ { \mathrm { d r i } } } }$ and the driving torques calculated from Eq. (17). For example, the elbow joint moves at constant speed with no payload on the manipulator while the motor current is filtered by the MSMA with a time period of t. In Fig. 5, the errors show a normal distribution that is verified by Anderson-Darling test. Consequently, the variance of the elbow joint model errors with time period t can be calculated by the empirical standard deviation of the errors. 

The covariance matrix $\sigma _ { \mathrm { D y n } } ^ { 2 }$ of the predicted state noise $e _ { p }$ is further considered. For the individual joints, the variance of the model errors changes as the time period t varies. Therefore, identification experiments regarding the model errors variance of six joints must be conducted to obtain the optimal t. Each joint is commanded to move at constant speed while the motor current is filtered by the MSMA with different time periods. 

The covariance matrix $\sigma _ { \mathrm { D y n } } ^ { 2 }$ of the predicted state noise $e _ { p }$ over changes in t was considered. For the individual joints, the variance of the model errors changed as the time period t varies. Thus the optimal t values were identified directly. Each joint was commanded to move at constant speed while the motor current was filtered by the MSMA with different time periods. Fig. 6 shows the dependence of the variance in the elbow joint driving torque errors as a function of t. The variance drops with increasing t up to approximately t = 50, after which there are minimal changes. Thus, a t value of 50 was selected as the benefit of extending that value beyond 50 is minimal. For the other joints, the similar phenomena appeared thus confirming convergence of the proposed algorithm. 

It is assumed that larger t values would lead to less noisy filtered current signals. Consequently, the response of the external force/torque estimation will be more precise, and thus quicker convergence is potentially achieved. However, excessive samples in each averaging step increase the time lag, and weaken the trends in the smoothed motor current signal. Hence, the variance initially decreases as t increases. After that, the variance in the driving torque errors plateaus as the smoothing benefits are canceled by the latency of the system. Furthermore, at high values of $t ,$ missed dynamics contribute to the errors observed by the estimator. In contrast, at low values, Gaussian errors resulting from the sensor contribute more significantly to the errors observed by the estimator. Hence, while low values of t yield Gaussian error signals, larger values of t tend to lead to non-Gaussian distributions. The optimal span of t = 50 for the elbow joint was within the domain of values that yielded Gaussian error distributions remained. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/182548023351aaa0ec06209147d95a5b742858d0406a437358e7a0db44e35916.jpg)



Joint output torque errors τdri − τdri [Nm]



Fig. 5. Distribution of joint output errors. The elbow joint is employed as an example and moves without payload. The errors are computed with $\overline { { \pmb { \tau } _ { \mathrm { d r i } } } }$ obtained from Eq. (24) and $\widehat { \pmb { \tau } _ { \mathrm { d r i } } }$ obtained from Eq. (17). The red line is an added fitted line for the normal distribution, the variance of which is computed from the empirical standard deviation of the errors. (For interpretation of the references to color in this figure legend, the reader is referred to the web version of this article.)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/05708d8f475e3df216d90c67991a04f23e284bed08bc154c3cd696d1f765e72e.jpg)



Fig. 6. The variance of the predicted state noise $e _ { p }$ with different time periods.


## 3.3. Measurement noise matrix

In this research, an accurate dynamic model of the manipulator was assumed, and thus the measurement noise in M(q)q was caused by only the measured terms q and $\dot { \mathbf { \nabla } } \cdot \dot { \mathbf { q } } .$ The errors in q ˙ are the dominant part of the measurement noise since q ˙ is calculated from measurable q by numerical differentiation, which amplifies the errors. In order to extract the errors in ${ \dot { \pmb q } } ,$ each joint of the UR5 manipulator is commanded to move at constant speed, the empirical distribution of the errors between the target velocity $\overline { { \dot { q } } }$ and the measured speed $\widehat { \dot { \pmb q } }$ is shown in Fig. 7. The Anderson-Darling test allows an assumption of a Gaussian distribution in velocity error $\dot { \pmb q } \sim N ( \overline { { \dot { \pmb q } } } , \pmb \sigma _ { \dot { q } } ^ { 2 } )$ . The measurement noise in Eq. (6) can thus be expressed by 

$$
\mathbf {Z} \sim N (0, \pmb {\sigma} _ {\mathrm{mea}} ^ {2})\tag{25}
$$

where $\pmb { \sigma } _ { \mathrm { m e a } } ^ { 2 } = \pmb { M } ( \pmb { q } ) \pmb { \sigma } _ { \dot { q } } ^ { 2 } \pmb { M } ( \pmb { q } )$ 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/e475375b1470809133db22865a99b3ce38fd4cc5e1fd3a119bbbe76ca9ece85b.jpg)



Fig. 7. Distribution of the measured joint speed errors. The elbow joint is employed as an example and moves at constant speed. The frequency of the errors is described as a histogram. The errors are computed with the target speed $\cdot \bar { q }$ and the measured mean speed $\widehat { \dot { \pmb q } } .$ The red line is an added fitted line for the normal distribution, the variance of which is computed from the empirical standard deviation of the errors. (For interpretation of the references to color in this figure legend, the reader is referred to the web version of this article.)


## 3.4. External wrench model

Normally, the term S is defined according to the nature of the contact force/torque exerted on the manipulator. However, a-priori information of the external force is not available in most cases. Consequently, S is defaulted to 0 and $\dot { \boldsymbol f } = \boldsymbol e _ { f }$ . The common choice for $e _ { f }$ are positive gains. Despite the sudden and unexpected contact at 0.56 s of the experiment, larger diagonal elements of $\sigma _ { f } ^ { 2 }$ are preferred in order to decrease the effect of that and reduce response time. However, the diagonal elements must be limited to a certain magnitude as large diagonal elements can increase the effect of the noise. The problem is mitigated in the proposed approach with the introduction of the MSMA based on variable time period. In other words, during the same response period (with the same $\sigma _ { f } ^ { 2 } )$ , the effect of noise is lower than that caused by the CKF. 

## 4. Experiments and result

## 4.1. Experiments

In this section, the effectiveness of the proposed CFTS method is demonstrated and the UR5 manipulator is used to conduct the following experiments. External contact is exerted on the UR5, which includes three steps: 

Step 1. The robot is moved to an initial position with a gripper attached to the end effector (Fig. 8(a)). After initialization, the TCP is programmed to move up vertically at constant speed of 0.06 m/s. The payload (lead filled blue cup in Fig. 8) is initially not engaged by the gripper. The effect of gripper on the TCP can be deemed as a constant force of 17.09 N in the –Z0 direction and a constant torque in the X0 direction. 

Step 2. The payload is engaged by the moving gripper, as shown in Fig. 8(b). The model has no a priori information to anticipate coupling with the payload. 

Step 3. The payload and gripper are coupled and raised together (Fig. 8(c)). The effect of the payload and the gripper can be regarded as a constant force in the –Z0 direction and a constant torque in the X0 direction. 

During the experiment, the joint kinematic information and motor current are recorded with a sampling frequency of 125 Hz. Counterpart experiments undergo the same motion with payloads of 100 g to 1000 g in increments of 100 $g .$ Furthermore, in order to validate the generalization of the proposed method in terms of different speeds, another two experiments undergo the same payload but different speeds, which are 0.04 m/s and 0.08 m/s respectively. 

## 4.2. Results

Fig. 9(a) and (b) show the performance of the varying contact force and torque (with 1000 g lead in the cup) estimation based on the peer method of the CKF [64], the approach of the AKF without the WMA, the proposed way of the AKF. Fig. 9(c) and (d) show the estimation results of constant force and torque. 

The variance of the estimation errors in Fig. 9(c) and (d) are summarized in Tables 1 and 2. Results of the comparison experiments with the same payload but different motions are displayed in Tables 3–6. 

With different payloads, the standard deviation (SD) and root mean squared error (RMSE) of the force and torque estimation based on the CKF and the AKF are displayed in Fig. 10(a) and (b) respectively. 

The response time of force and torque sensing based on different methods varies as the payload changes and is shown in Fig. 11. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/6aaa38a451e42c03a1293735e202c5acfedb8e80b538fadce60bb146a88510c3.jpg)



(a) Initial pose. The gripper in the red square weighs 1.742 kg. The TCP goes up at constant velocity of 0.06m/s and the cup stands still


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/d2495f366e77b371b2d36e2f4993d9a17d3432969b514429f7f177ff7071687e.jpg)



(b) The cup and the lead inside it start to move up vertically due to the movement of the gripper.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/56b3c1c7822df79f19905da28c627b123a1ed4fe1bb7953a9250aa09e246135a.jpg)



(c) The maniputator moves up at constant speed as the TCP does.



Fig. 8. Demonstration scenario for the experiment. The UR5 manipulator is initialized as (a). The TCP is commanded to move up vertically and the effect of the cup filled with lead can be regarded as constant external wrench exerted on the TCP.


## 5. Discussion

## 5.1. Effect of the AKF

The proposed contact force/torque estimation method based on the AKF performs better in the physical experiments compared to the peer [64] method of the CKF. The SD and the RMSE of the force estimation based on two methods are shown in Fig. 10(a). With the exception of the RMSE of the estimation at 100 g, the errors of the estimation based on the AKF are all smaller than those resulting from the CKF. Hence, in cases similar to the experimental process, more accurate estimation of contact force is likely with the proposed AKF. The same conclusion can be made for torque estimation errors with the SD and the RMSE of the AKF being lower than those of the CKF. However, some model uncertainty is evident as the SD is larger than the RMSE (Fig. 10). This is likely to be caused by unmodelled effects, such as the weight and the damping of the hose that powers the gripper (Fig. 8). 

It can be seen from Fig. 11(a) that the AKF conducts apparently faster force estimation than the CKF when the payload increases above 400 g. Below this threshold, the time consumed by both filters is very similar. The response time of torque estimation is similar across the two methods during all experiments. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/2e167f2025957935b7ba11fd474f3f63580c84f9daa74006018f52ee64265c9c.jpg)



(a) Estimation of the varying force with the lead of 1kg.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/a04e061f46804ea7610461d0cef1da4810c788364600f39cf02c66d47e4d9b01.jpg)



(b) Estimation of the varying torque with the lead of 1kg.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/42093d97febcfacc199f818dc4de53d1372ce3338eb808b58ab2af44715c79ca.jpg)



(c) Estimation of the constant force.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/4c7354453fe8bbfadd784a4583d0b5e0a7b8a834d11654d08dca90ad829143b9.jpg)



(d) Estimation of the constant torque.



(caption on next column)



Fig. 9. Estimation of the force in Z0 direction and torque in X0 direction. The actual force/torque (black curve) is compared to estimation with the CKF (green curve), the AKF without the WMA (blue curve), and the AKF (red curve). The dashed green, blue, and red lines show the response time of three esti mation methods. Under some circumstances, the estimation results of the AKF and the AKF (no WMA) are very similar. Therefore the red line and the blue line overlap each other. (For interpretation of the references to color in this figure legend, the reader is referred to the web version of this article.)


The magnitude of the exerted torque is relatively small, because of which the advantage of torque-estimating response time is not apparent. As the varying torque gets larger, the response time advantages of the proposed AKF are likely to emerge in concert with the benefits in force estimation. However, a validation experiment is required to validate this claim. 

## 5.2. Effect of the MSMA

The mechanism of mode switching, the involved moving averages, and the variable time period all contribute to the improved response time and estimation accuracy. 

## 5.2.1. Effect of the switching mode, the HMAand the WMA

The smoothing effect of the HMA is better than that of the WMA when the time period is fixed and large. Conversely, the performance of the WMA is better in terms of reducing estimation errors and response time. Advantages of both moving average approaches are adopted in this research, the performance of which can be seen in Fig. 9. As shown in Fig. 9(a), the estimation errors resulting from the WMA were smaller than that resulting from the HMA when the time period was varied (from 0 to 0.45 s and from 0.56 to 1 s). Furthermore, the WMA also needed less time to finish the estimation. A similar outcome occurred for the torque estimation (as shown in Fig. 9(b)), although the gap of response time was not pronounced. These benefits occurred across all payloads considered. Further, both the WMA and the HMA produced faster response times than the CKF. 

In particular, the smaller diagonal elements in the covariance matrix $\pmb { \sigma } _ { \mathrm { D y n } } ^ { 2 }$ led to a reduced response time for the system. The effect of the noise in the measured current was reduced by the mode-switching filter (both the HMA and the WMA) and thus the diagonal elements of $\pmb { \sigma } _ { \mathrm { D y n } } ^ { 2 }$ were reduced. 

Compared to the CKF (Table 1), the variance is reduced from 0.885 to 0.358 in the initial period (0–0.45 s) of force estimation as the WMA was used in our method. When the time period was fixed (0.45–1.5 s), the variance was decreased from 1.423 to 0.78 due to the activation of the HMA. Consequently, the estimation precision was improved across the experiment (from 1.256 to 0.711). Furthermore, the WMA (0.358) facilitates better estimation accuracy compared to the HMA (0.841) when the time period changed (0–0.45 s). The variance of the torque estimation errors showed the similar results (Table 2). In the comparison experiments with the same payload but different motions, the above conclusion was further validated from Tables 3–6. 

## 5.2.2. Effect of the variable time period

The WMA and the HMA facilitate the application of the AKF and the estimation result was improved in terms of response time and estimation accuracy. Furthermore, the variable time period further reduces the response time when interacting with the unexpected change in external force, (Fig. 11(b)). The only difference between WMAWVT and WMAWFT is whether the time period is varying or not. In particular, when the external wrench was exerted on the manipulator, the proposed approach was able to omit the previous current samples in each averaging step due to the variable time period. Consequently, the proposed approach leads to faster response (with the exceptions of estimation at low weights of 100 g and 200 g (Fig 11(b)). 


Table 3



Variance [N<sup>2</sup>] of the force estimation errors in different periods (0.06 m/s). (For Tables $^ { 1 - 6 , }$ *denotes periods of WMA averaging, and ** denotes periods of HMA averaging – note that AKF automatically switches between WMA and HMA if permitted).


<table><tr><td>Time Method</td><td>0–0.45 s</td><td>0.45–1.5 s</td><td>0–1.5 s</td></tr><tr><td>CKF</td><td>0.885</td><td>1.423</td><td>1.256</td></tr><tr><td>AKF(no WMA)</td><td>0.841**</td><td>0.78**</td><td>0.933</td></tr><tr><td>AKF</td><td>0.358*</td><td>0.78**</td><td>0.711</td></tr></table>


Variance $\left[ ( \mathrm { N m } ) ^ { 2 } \right]$ of the Torque estimation errors in different periods (0.06 m/ s).


<table><tr><td>Time Method</td><td>0–0.45 s</td><td>0.45–1.5 s</td><td>0–1.5 s</td></tr><tr><td>CKF</td><td>0.1083</td><td>0.0666</td><td>0.0802</td></tr><tr><td>AKF(no WMA)</td><td>0.0925**</td><td>0.0115**</td><td>0.0386</td></tr><tr><td>AKF</td><td>0.0399*</td><td>0.0115**</td><td>0.0208</td></tr></table>


Variance $[ \mathsf { N } ^ { 2 } ]$ of the force estimation errors in different periods (0.08 m/s).


<table><tr><td>Time Method</td><td>0–0.45 s</td><td>0.45–1.5 s</td><td>0–1.5 s</td></tr><tr><td>CKF</td><td>1.1228</td><td>0.8547</td><td>0.9579</td></tr><tr><td>AKF(no WMA)</td><td>2.2484**</td><td>0.561**</td><td>1.1519</td></tr><tr><td>AKF</td><td>0.9255*</td><td>0.561**</td><td>0.8066</td></tr></table>


Variance [(Nm)<sup>2</sup>] of the Torque estimation errors in different periods (0.08 m/ s).


<table><tr><td>Time Method</td><td>0–0.45 s</td><td>0.45–1.5 s</td><td>0–1.5 s</td></tr><tr><td>CKF</td><td>0.07439</td><td>0.0802</td><td>0.07839</td></tr><tr><td>AKF(no WMA)</td><td>0.07067**</td><td>0.0167**</td><td>0.037</td></tr><tr><td>AKF</td><td>0.02946*</td><td>0.0167**</td><td>0.02466</td></tr></table>


Variance $[ \mathsf { N } ^ { 2 } ]$ of the force estimation errors in different periods (0.04 m/s).


<table><tr><td>Time Method</td><td>0–0.45 s</td><td>0.45–1.5 s</td><td>0–1.5 s</td></tr><tr><td>CKF</td><td>1.5289</td><td>2.4825</td><td>2.2218</td></tr><tr><td>AKF(no WMA)</td><td>2.5284**</td><td>2.26**</td><td>2.3318</td></tr><tr><td>AKF</td><td>1.1967*</td><td>2.26**</td><td>1.9897</td></tr></table>


Variance [(Nm)<sup>2</sup>] of the Torque estimation errors in different periods (0.04 m/ s).


<table><tr><td>Time Method</td><td>0–0.45 s</td><td>0.45–1.5 s</td><td>0–1.5 s</td></tr><tr><td>CKF</td><td>0.07778</td><td>0.11523</td><td>0.10586</td></tr><tr><td>AKF(no WMA)</td><td>0.10863**</td><td>0.068**</td><td>0.07987</td></tr><tr><td>AKF</td><td>0.03413*</td><td>0.068**</td><td>0.06191</td></tr></table>

## 5.3. Effect of tuning strategy of the proposed AKF

In contrast to peer adaptive Kalman filters that update process and measurement covariance matrices using the estimated information, the proposed approach uses measured systematic uncertainty by offline experiments combined with the real-time dynamic information to update measurement covariance matrices in each iterative step. Hence, better performance was achieved by the proposed AKF as it was able to utilize measured values over a shorter timeframe upon recognition of an external wrench and could thus, determine the noise characteristics more precisely. Furthermore, tuning routines for the mode-switching moving averages must be considered in our research, and determining the optimal time period provides an ability to optimize estimation accuracy and response time when dealing unexpected loads. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/a83efcb0612a48615a25b26f591d134182b3b006c3f33011bcb8dad2c6c33e0a.jpg)



(a) Errors of force estimation.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/ae6fbe237c278ad6374196810de1e014811b80adeebe701d08d9f7b8c4c25868.jpg)



(b) Errors of torque estimation.



Fig. 10. The SD and RMSE of the external wrench estimation based on the CKF and the AKF with different payloads.


## 5.4. Effect of the generalized-momentum-based model

Since the dynamic model is based on the generalized momentum format, the proposed contact force estimation method works independently from inversion of the manipulator inertia matrix, and consequently the computational burden is low. Furthermore, joint acceleration is not required with the proposed method, and therefore amplification of the measurement noise can be avoided. Uncertainty of the manipulator dynamics is overcome in the proposed approach as the noise in the measured current is taken into account and ultimately allows for definition of a more accurate Kalman filter gain. In addition, the physical implementation cost of the approach is lower than the similar Kalman filter based approaches [63] as it does not require additional force/torque sensors. It can potentially be applied to any robotic manipulators where the kinematic and dynamic information is accessible. However, the efficacy of the methodology in robotic manipulators of different orientations and sizes has not been established. 

## 5.5. Future work

It must be noted that in order to apply the proposed estimation method, a time gap between every two occurrences of the external force during the experiment is assumed to be larger than $t _ { s } { \cdot } t .$ The value of $t _ { s } { \cdot } t$ is typically in the order of 200 ms - 400 ms, and thus the assumption is valid in many manufacturing applications. However, if the robotic manipulator is used in an environment with high frequency vibration, such as a turning or milling processes, the time gaps between contact may become smaller than $t _ { s } { \cdot } t ,$ and the approach may not yield benefit. Future research should be conducted to evaluate the performance of the approach in such turning or milling (or similar) applications and if necessary consider additions to the AKF calculation that retain the benefits of the proposed approaches. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/700e876e96ddcab5927ec462492e3ba20a172112a94172bb67824330cc6671e4.jpg)



(a) The green curve and the red one stand for the force estimation based on the CKF and the AKF respectively. The dashed green and red ones denote the torque estimation based on the CKF and the AKF


![image](https://cdn-mineru.openxlab.org.cn/result/2026-10-22/85e97a48-5ae3-45a1-ba5a-cb5f117b4049/1d4a43c5d516f4ccab6c931a5ec0381f8b950f86b23cbb4232dfa61320e3672f.jpg)



(b) The blue curve and dashed blue one stand for the force and torque estimation time by the proposed AKF with only the WMA and varying time perioć (WMAWVT). The purple ones represent the estimation based on the proposed AKF with only the WMA and fixed time period(WMAWFT).



Fig. 11. Comparison of the response time with different methods.


The proposed approach was validated in a physical system imple mentation with differing loading scenarios. The validation used key parameters that are critical for the effective implementation of robotic manipulators – response time and force/force estimation precision. The control system was not aware of the force/torque contact before occurrence nor the weight of the end effector payload. Use of a series of loading scenarios shows that parameters of the approach are not specifically tuned to succeed in limited range of applications. However, the estimation quality will depend on the manipulator used. This concern is somewhat mitigated by the general ability of control system architecture to remain reasonably consistent across geometry. Furthermore, the AKF will be considered in practice and possibly adapted to some industrial scenarios such as polishing, assembly, and lead-through programming. 

While there are a number of potential moving average functions that could be implemented in the proposed approach, this paper presents a comparison of the WMA and HMA filters. These filters were chosen for their popularity and comparative ease of implementation. Future research may consider other filters. 

A simplified friction model identification was conducted in this research. Although validation of the proposed contact force/torque sensing method was realized based on it, a more advanced friction model identification will strengthen the approach. In the future, modern machine learning techniques will be used to determine a phenomenological friction model. 

Theoretically, the proposed method can be applied to any robotic manipulators as long as the information of manipulator kinematics, manipulator dynamics, and motor feedbacks is available. However, this conclusion needs to be verified and validation of the generalization of the proposed method will be conducted on more serial manipulators. 

## 6. Conclusion

This paper proposes a sensorless contact force estimation method. The serial manipulator was modelled in the format of generalized momentum and the dynamic parameters were identified without additional force/torque sensors. The noisy motor current measurements were optimized by the mode-switching moving average based on variable time period and the covariance matrix of the predicted state noise was determined with an optimal time period t. The measurement noise matrix was obtained by taking the uncertainty in the joint speeds into account. The adaptive Kalman filter was tuned and then used to conduct contact force and torque estimation with the optimized current. Exper iments were then performed on the UR5 manipulator. The proposed estimation approach was compared to some popular modern techniques and led to faster response times and estimation accuracy. Furthermore, the effect of the proposed method became more apparent as the magnitude of the exerted external wrench increased. 

## Declaration of Competing Interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper. 

## Appendix

a $\pmb q \in \mathcal { R } ^ { 6 } , \dot { \pmb q } \in \mathcal { R } ^ { 6 }$ , and $\ddot { q } \in \mathcal { R } ^ { 6 }$ are the position, velocity, and acceleration of UR5 and described as: 

$$
\boldsymbol {q} = \left[ \begin{array}{c c c c c c} q _ {1} & q _ {2} & q _ {3} & q _ {4} & q _ {5} & q _ {6} \end{array} \right] ^ {T}
$$

$$
\dot {\boldsymbol {q}} = \left[ \begin{array}{c c c c c c} \dot {q} _ {1} & \dot {q} _ {2} & \dot {q} _ {3} & \dot {q} _ {4} & \dot {q} _ {5} & \dot {q} _ {6} \end{array} \right] ^ {T}
$$

$$
\ddot {\boldsymbol {q}} = \left[ \begin{array}{c c c c c c} \ddot {q} _ {1} & \ddot {q} _ {2} & \ddot {q} _ {3} & \ddot {q} _ {4} & \ddot {q} _ {5} & \ddot {q} _ {6} \end{array} \right] ^ {T}
$$

where $q _ { a } , \dot { q } _ { a } ,$ and $\ddot { q } _ { a }$ are the position, velocity, and acceleration of the ath joint. 

b $M ( q ) \in \mathcal { R } ^ { 6 \mathrm { ~ \times ~ } 6 }$ is the inertia matrix of UR5 and described as: 

$$
\boldsymbol {M} (\boldsymbol {q}) = \left[ \sum_ {i = 1} ^ {6} \left\{m _ {i} \boldsymbol {J} _ {v _ {i}} (\boldsymbol {q}) ^ {T} \boldsymbol {J} _ {v _ {i}} (\boldsymbol {q}) + \boldsymbol {J} _ {\omega_ {i}} (\boldsymbol {q}) ^ {T} \boldsymbol {R} _ {i} (\boldsymbol {q}) \boldsymbol {I} _ {i} \boldsymbol {R} _ {i} (\boldsymbol {q}) ^ {T} \boldsymbol {J} _ {\omega_ {i}} (\boldsymbol {q}) \right\} \right]
$$

where $m _ { i }$ is the mass of the i-th link. $\pmb { R _ { i } }$ is the orientation transformation between the body attached frame of the i-th link and the inertia frame. $\pmb { I } _ { i } \in \mathcal { R } ^ { 3 } \times \mathbb { 3 }$ is the inertia tensor of the i-th link. $J _ { i } ( q ) \in \mathcal { R } ^ { 6 \mathrm { ~ \times ~ } 6 }$ and $J _ { i } ( { \pmb q } ) = \left[ J _ { \nu _ { i } } ( { \pmb q } ) \right]$ is the Jacobian matrix of the i-th link. 

$$
\boldsymbol {C} _ {C} (\boldsymbol {q}, \dot {\boldsymbol {q}}) \in \mathscr {R} ^ {6 \times 6}
$$

$$
k, j
$$

$$
c _ {k j} = \sum_ {i = 1} ^ {n} c _ {i j k} (\boldsymbol {q}) \dot {q} _ {i} = \sum_ {i = 1} ^ {n} \frac {1}{2} \left\{\frac {\partial m _ {k j}}{\partial q _ {i}} + \frac {\partial m _ {k i}}{\partial q _ {j}} - \frac {\partial m _ {i j}}{\partial q _ {k}} \right\} \dot {q} _ {i}
$$

where $m _ { k j }$ is the $k , j \cdot$ -th element of $M ( q )$ 

d $G ( q ) \in \mathcal { R } ^ { 6 }$ is the gravity of UR5 and expressed as: 

$$
\boldsymbol {G} (\boldsymbol {q}) = \boldsymbol {g r a} \cdot \boldsymbol {g} (\boldsymbol {q})
$$

where $g ( \pmb { q } ) = \left[ g _ { 1 } ( \pmb { q } ) \quad g _ { 2 } ( \pmb { q } ) \quad g _ { 3 } ( \pmb { q } ) \quad g _ { 4 } ( \pmb { q } ) \quad g _ { 5 } ( \pmb { q } ) \quad g _ { 6 } ( \pmb { q } ) \right] ^ { T }$ . gra is vector giving the direction of gravity in the inertial frame. $g _ { i } ( \pmb q ) = \partial P / \partial q _ { i }$ . The function $\begin{array} { r } { P = \sum _ { i = 1 } ^ { 6 } m _ { i } g r a ^ { T } r _ { c i } } \end{array}$ provides the potential energy of $\mathrm { U R } 5 . r _ { c i }$ is the coordinates of the center of mass of link i. $\mathrm { ~ e ~ } \tau _ { \mathrm { { f r i } } } \in \mathcal { R } ^ { 6 }$ is the joint friction of UR5 

$\mathrm { ~ f ~ } \tau _ { \mathrm { d r i } } \in \mathcal { R } ^ { 6 }$ is the joint driving torques of UR5. 

$\textit { g } \tau _ { \mathrm { e x t } } \in \mathcal { R } ^ { 6 }$ is the joint torques caused by the external wrench f and related to $\pmb { \tau } _ { \mathrm { e x t } }$ by the manipulator Jacobian matrix: 

$$
\boldsymbol {\tau} _ {e x t} = \boldsymbol {J} (\boldsymbol {q}) ^ {T} \boldsymbol {f}
$$

## References



[1] R. Tinos, M.H. Terra, J.Y. Ishihara, Motion and force control of cooperative robotic manipulators with passive joints, IEEE Trans. Control Syst. Technol. 14 (4) (2006) 725–734. Jul. 





[2] D. Kruse, J.T. Wen, R.J. Radke, A sensor-based dual-arm tele-robotic system, IEEE Trans. Autom. Sci. Eng. 12 (1) (2015) 4–18. Jan. 





[3] A. Lopes, F. Almeida, A force-impedance controlled industrial robot using an active robotic auxiliary device, Robot. Comput.-Integr. Manuf. 24 (3) (2008) 299–309. 





[4] A. Stolt, M. Linderoth, A. Robertsson, R. Johansson, Detection of contact force transients in robotic assembly, in: Proc. IEEE Int. Conf. Robot. Autom., 2015, pp. 962 968. 





[5] J. Podobnik, M. Munih, Haptic interaction stability with respect to grasp force, IEEE Trans. Syst. Man, Cybern. C, Appl. Rev. 37 (6) (Nov. 2007) 1214 1222. 





[6] X. Zhu, B. Lin, L. Liu, Y. Luan, Power transfer performance and cutting force effects of contactless energy transfer system for rotary ultrasonic grinding, IEEE Trans. Ind. Electron. 63 (5) (May 2016) 2785 2795. 





[7] X. Zhang, H. Chen, J. Xu, X. Song, J. Wang, X. Chen, A novel sound-based belt condition monitoring method for robotic grinding using optimally pruned extreme learning machine, J. Mater. Process. Technol. 260 (Oct. 2018) 9–19. 





[8] F. Chen, H. Zhao, D. Li, L. Chen, C. Tan, H. Ding, Contact force control and vibration suppression in robotic polishing with a smart end effector, Robot. Comput.-Integr. Manuf. 57 (Jun. 2019) 391–403. 





[9] J.E. Solanes, L. Gracia, P. Munoz-Benavent, ˜ A. Esparza, J.V. Miro, J. Tornero, Adaptive robust control and admittance control for contactdriven robotic surface conditioning, Robot. Comput.-Integr. Manuf. 54 (Dec. 2018) 115–132. 





[10] T. Olsson, M. Haage, H. Kihlman, R. Johansson, K. Nilsson, A. Robertsson, M. Bjorkman, ¨ R. Isaksson, G. Ossbahr, T. Brogårdh, Costefficient drilling using industrial robots with high-bandwidth force feedback, Robot. Comput.-Integr. Manuf. 26 (1) (Feb. 2010) 24–38. 





[11] X. Xu, W. Chen, D. Zhu, S. Yan, H. Ding, Hybrid active/passive force control strategy for grinding marks suppression and profile accuracy enhancement in robotic belt grinding of turbine blade, Robot. Comput.-Integr. Manuf. 67 (Feb. 2021). 





[12] D. Zhu, X. Feng, X. Xu, Z. Yang, W. Li, S. Yan, H. Ding, Robotic grinding of complex components: a step towards efficient and intelligent machining–challenges, solutions, and applications, Robot. Comput.-Integr. Manuf. 65 (Oct. 2020). 





[13] C. Gaz, E. Magrini, A. De Luca, A model-based residual approach for human-robot collaboration during manual polishing operations, Mechatronics 55 (Nov. 2018) 234 247. 





[14] B. Yao, Z. Zhou, L. Wang, W. Xu, Q. Liu, A. Liu, Sensorless and adaptive admittance control of industrial robot in physical human-robot interaction, Robot. Comput.- Integr. Manuf. 51 (Jun. 2018) 158–168. 





[15] A. Cherubini, R. Passama, A. Crosnier, A. Lasnier, P. Fraisse, Collaborative manufacturing with physical human-robot interaction, Robot. Comput-Integr. Manuf. 31 (Aug. 2016) 1–13. 





[16] H. Lo, S. Xie, Exoskeleton robots for upper-limb rehabilitation: state of the art and future prospects, Med. Eng. Phys. 34 (3) (Apr. 2012) 261 268. 





[17] W. Meng, Q. Liu, Z. Zhou, Q. Ai, B. Sheng, S. Xie, Recent development of mechanisms and control strategies for robot-assisted lower limb rehabilitation, Mechatronics 31 (2015) 132 145 





[18] L. Han, W. Xu, B. Li, P. Kang, Collision detection and coordinated compliance control for a dual-arm robot without force/torque sensing based on momentum observer, IEEE/ASME Trans. Mechatronics 24 (5) (2019) 2261–2272. 





[19] M.H. Raibert, J.J. Craig, Hybrid position/force control of manipulators, J. Dyn. Syst. Meas. Control 103 (2) (1981) 126–133. 





[20] N. Hogan, Impedance control: an approach to manipulation-part I: theory; part II: implementation; part III: applications, Trans. ASME J. Dyn. Syst. Meas. Control 107 (1) (1985) 1–24. 





[21] K. Ohnishi, M. Shibata, T. Murakami, Motion control for advanced mechatronics, IEEE/ASME Trans. Mechatronics 1 (1) (Mar. 1996) 56–67. 





[22] T.T. Phuong, K. Ohishi, Y. Yokokura, ‘Fine sensorless force control realization based on dither periodic component elimination Kalman filter and wide band disturbance observer, IEEE Trans. Ind. Electron. 67 (1) (Jan. 2020) 757–767. 





[23] L. Chan, F. Naghdy, D. Stirling, Extended active observer for force estimation and disturbance rejection of robotic manipulators, Robot. Auton. Syst. 61 (12) (2013) 1277 1287. 





[24] W.-H. Chen, D.J. Ballance, P.J. Gawthrop, J. O’Reilly, A nonlinear disturbance observer for robotic manipulators, IEEE Trans. Ind. Electron. 47 (4) (Aug. 2000) 932 938. 





[25] W.-H. Chen, Disturbance observer based control for nonlinear systems, IEEE/ASME Trans. Mechatronics 9 (4) (Dec. 2004) 706–710. 





[26] A. Mohammadi, M. Tavakoli, H. Marquez, F. Hashemzadeh, Nonlinear disturbance observer design for robotic manipulators, Control Eng. Pract. 21 (3) (2013) 253–267. 





[27] A. Nikoobin, R. Haghighi, Lyapunov-based nonlinear disturbance observer for serial n-link robot manipulators, J. Intell. Robot. Syst. 55 (2/3) (Jul. 2009) 135 153. 





[28] H. Zhang, S. Ahmad, G. Liu, Torque estimation for robotic joint with harmonic drive transmission based on position measurements, IEEE Trans. Robot. 31 (2) (Apr. 2015) 322–330. 





[29] F. Zhou, Y. Li, G. Liu, Robust decentralized force/position fault-tolerant control for constrained reconfigurable manipulators without torque sensing, Nonlinear Dyn 89 (3) (2017). 





[30] B. Dong, F. Zhou, K. Liu, Y. Li, Torque sensorless decentralized neuro-optimal control for modular and reconfigurable robots with uncertain environments, Neurocomputing 282 (2018) 60–73. 





[31] J.W. Jeong, P.H. Chang, K.B. Park, Sensorless and modeless estimation of external force using time delay estimation: application to impedance control, J. Mech. Sci. Technol. 25 (8) (Sep. 2011) 2051–2059. 





[32] A.D. Luca, R. Mattone, Sensorless robot collision detection and hybrid force motion control, in, in: Proc. IEEE Int. Conf. Robot. Autom., 2005, pp. 999–1004 





[33] A.D. Luca, A. Albu-Sch¨affer, S. Haddadin, G. Hirzinger, Collision detection and safe reaction with the DLR-III lightweight manipulator arm, in: Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2006, pp. 1623–1630. 





[34] A.D. Luca, R. Mattone, Actuator failure detection and isolation using generalized momenta, in, Proc. IEEE Int. Conf. Robot. Autom. 1 (Sep. 2003) 634 639. 





[35] M. Ragaglia, A.M. Zanchettin, L. Bascetta, P. Rocco, Accurate sensorless leadthrough programming for lightweight robots in structured environments, Robot Comput.-Integr. Manuf 39 (June 2016) 9–21. 





[36] J. Yuan, Y. Qian, Z. Yuan, L. Gao, W. Wan, Position based impedance forc controller with sensorless force estimation, Assembly Autom. 39 (3) (2019) 489–496. 





[37] S.H. Yen, P.C. Tang, Y.C. Lin, C.Y. Lin, Development of a virtual force sensor for a low-cost collaborative robot and applications to safety control, Sensors 19 (11) (2019) 2603. 





[38] Z. Qiu, R. Ozawa, S. Ma, A force-sensorless approach to collision detection based on virtual powers, Adv. Robot. 33 (23) (2019) 1209 1224. 





[39] Y. Tian, Z. Chen, T. Jia, A. Wang, L. Li, Sensorless collision detection and contact force estimation for collaborative robots based on torque observer, in: Proc. IEEE Int. Conf. Robot. Biom, Dec 2016, pp. 946–951. 





[40] Z. Liu, F. Yu, L. Zhang, T. Li, Real-time estimation of sensorless planar robo contact information, J. Robot. Mechatron. 29 (3) (2017) 557–565. 





[41] Y. Huang, et al., A novel adaptive Kalman filter with inaccurate process and measurement noise covariance matrices, IEEE Trans. Automat. Contr. 63 (2) (2017) 594–601. 





[42] H. Zhu, et al., An adaptive Kalman filter with inaccurate noise covariances in the presence of outliers, IEEE Trans. Automat. Contr. (2021). 





[43] J. Wang, et al., Suboptimal adaptive Kalman filtering based on the proportiona control of prior error covariance, ISA Trans. 100 (2020) 145–154. 





[44] P. Mercorelli, A switching kalman filter for sensorless control of a hybrid hydraulic piezo actuator using mpc for camless internal combustion engines, in: 2012 IEEE International Conference on Control Applications, IEEE, 2012. 





[45] M. Schimmack, B. Haus, P. Mercorelli, An extended Kalman filter as an observer in a control structure for health monitoring of a metal–polymer hybrid soft actuator, IEEE/ASME Trans. Mechatronics 23 (3) (2018) 1477 1487. 





[46] B. Haus, H. Aschemann, P. Mercorelli, Tracking control of a piezo-hydraulic actuator using input–output linearization and a cascaded extended kalman filter structure, J. Franklin Inst. 355 (18) (2018) 9298 9320. 





[47] L. Zanni, et al., A prediction-error covariance estimator for adaptive Kalman filtering in step-varying processes: application to power-system state estimation IEEE Trans. Control Syst. Technol. 25 (5) (2016) 1683 1697. 





[48] J. Zhang, et al., A two-stage Kalman filter approach for robust and real-time power system state estimation, IEEE Trans. Sustain. Energy 5 (2) (2013) 629 636. 





[49] L. Chen, P. Mercorelli, S. Liu, A Kalman estimator for detecting repetitive disturbances, in: Proceedings of the 2005, American Control Conference, 2005, IEEE, 2005. 





[50] X. Gao, D. You, S. Katayama, Seam tracking monitoring based on adaptive Kalman filter embedded Elman neural network during high-power fiber laser welding, IEEE Trans. Ind. Electron. 59 (11) (2012) 4315 4325. 





[51] Z. Zhou, et al., Equality constrained robust measurement fusion for adaptive kalman-filter-based heterogeneous multi-sensor navigation, IEEE Trans. Aerosp. Electron. Syst. 49 (4) (2013) 2146 2157. 





[52] G. Xu, et al., A computationally efficient variational adaptive Kalman filter fo transfer alignment, IEEE Sens. J. 20 (22) (2020) 13682–13693. 





[53] S.S. Bisht, M.P. Singh, An adaptive unscented Kalman filter for tracking sudden stiffness changes, Mech. Syst. Signal Process. 49 (1–2) (2014) 181–195. 





[54] Y. Liu, et al., An innovative information fusion method with adaptive Kalman filter for integrated INS/GPS navigation of autonomous vehicles, Mech. Syst. Signal Process. 100 (2018) 605–616. 





[55] F.A. Ghaleb, et al., Improved vehicle positioning algorithm using enhanced innovation-based adaptive Kalman filter, Pervasive Mob. Comput. 40 (2017) 139–155. 





[56] C. Hajiyev, H.E. Soken, Robust adaptive Kalman filter for estimation of UAV dynamics in the presence of sensor/actuator faults, Aerosp. Sci. Technol. 28 (1) (2013) 376–383. 





[57] A. ´ Odry, et al., Kalman filter for mobile-robot attitude estimation: novel optimized and adaptive solutions, Mech. Syst. Signal Process. 110 (2018) 569–589. 





[58] Y. Huang, et al., A new adaptive extended Kalman filter for cooperative localization, IEEE Trans. Aerosp. Electron. Syst. 54 (2017) 353–368, 1. 





[59] M. Li, et al., Model-free control for continuum robots based on an adaptive Kalman filter, IEEE/ASME Trans. Mechatronics 23 (2017) 286–297, 1. 





[60] B. Haus, P. Mercorelli, Polynomial augmented extended kalman filter to estimate the state of charge of lithium-ion batteries, IEEE Trans. Veh. Technol. 69 (2019) 1452–1463, 2. 





[61] J. Jung, J. Lee, K. Huh, Robust contact force estimation for robot manipulators in three-dimensional space, in, Proc. Inst. Mech. Eng. C J. Mech. Eng. Sci. 220 (9) (2006) 1317–1327. 





[62] A.U. Pehlivan, D.P. Losey, M.K. O’Malley, Minimal assist-as-needed controller for upper limb robotic rehabilitation, IEEE Trans. Robot. 32 (1) (Feb. 2016) 113 124. 





[63] H. Jin, X. Rong, Contact force estimation for robot manipulator using semiparametric model and disturbance kalman filter, IEEE Trans. Ind. Electron. 65 (4) (Apr. 2018) 3365–3375. 





[64] A. Wahrburg, J. Bos, ¨ K.D. Listmann, F. Dai, B. Matthias, H. Ding, Motor-current based estimation of cartesian contact forces and torques for robotic manipulators and its application to force control, IEEE Trans. Autom. Sci. Eng. (2017) 1–8. 





[65] M.W. Spong, S. Hutchinson, M. Vidyasagar, Robot Modeling and Control, Wiley, New York, NY, USA, 2006. 





[66] P. Axelsson, F. Gustafsson, Discrete-time solutions to the continuous-time differential Lyapunov equation with applications to Kalman filtering, IEEE Trans Autom. Control 60 (3) (Mar. 2015) 632–643. 





[67] C. Van Loan, Computing integrals involving the matrix exponential, IEEE Trans. Autom. Control 23 (3) (Jun. 1978) 395–404. 





[68] J.T. Gravdahl, Force Estimation in Robotic Manipulators: Modeling, Sim-Ulation and Experiments, M.S thesis, Dept. Eng. Cybern., NTNU., Norway, 2014. 





[69] H. Olsson, K. Åstrom, ¨ C. Canudas de Wit, M. G¨afvert, P. Lischinsky, Friction models and friction compensation, Eur. J. Control 4 (3) (Dec. 1998) 176–195. 

