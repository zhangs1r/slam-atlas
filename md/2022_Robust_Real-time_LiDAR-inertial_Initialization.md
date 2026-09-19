# Robust Real-time LiDAR-inertial Initialization

Fangcheng Zhu<sup>∗</sup>, Yunfan Ren<sup>∗</sup>, Fu Zhang

Abstract— For most LiDAR-inertial odometry, accurate initial states, including temporal offset and extrinsic transformation between LiDAR and 6-axis IMUs, play a significant role and are often considered as prerequisites. However, such information may not be always available in customized LiDARinertial systems. In this paper, we propose LI-Init: a full and real-time LiDAR-inertial system initialization process that calibrates the temporal offset and extrinsic parameter between LiDARs and IMUs, and also the gravity vector and IMU bias by aligning the state estimated from LiDAR measurements with that measured by IMU. We implement the proposed method as an initialization module, which can automatically detects the degree of excitation of the collected data and calibrate, onthe-fly, the temporal offset, extrinsic, gravity vector, and IMU bias, which are then used as high-quality initial state values for real-time LiDAR-inertial odometry systems. Experiments conducted with different types of LiDARs and LiDAR-inertial combinations show the robustness, adaptability and efficiency of our initialization method. The implementation of our LiDARinertial initialization procedure LI-Init and test data are opensourced on Github<sup>1</sup> and also integrated into a state-of-the-art LiDAR-inertial odometry system FAST-LIO2.

## I. INTRODUCTION

Sensors are called the eyes of robots, which endow them with capability of exploring surroundings and performing self-localization and navigation. Camera is a commonly used sensor due to the ability to provide rich RGB information with low cost and light weight, but it is vulnerable to inadequate illumination and is lack of direct depth measurement, leading to high computation complexity when reconstructing 3D environments. Compared with cameras, light detection and ranging (LiDAR) sensors can offer direct, accurate 3D measurements and is robust to illumination changes, making it a preferred choice for robot localization [1, 2] and mapping [3] applications.

To answer up the emergency such as sensor failure and to keep the whole system robust, multi-sensor fusion is becoming the main trend in recent years. Inertial Measurement Unit (IMU) is an excellent complementary sensor to fuse with camera or LiDAR since it can provide short-term egomotion estimations without any external references. IMU is an ideal option to mitigate short-term odometry failure caused by degeneration, such as dim light scenes for cameras and structure-less environments for LiDARs. Moreover, high-frequency kinematic measurements from IMU help to compensate motion distortion of LiDAR scans especially when the robot is in high-speed motion [4]. More and more multi-sensor based simultaneous localization and mapping (SLAM) methods show up, including visual-inertial system [5, 6], LiDAR-inertial system [1, 7, 8], and LiDAR inertial visual system [9, 10].

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/dbe8978806755a845826002641b7c8fb7b072c1bfdab2903d3f5d6ffaccca952.jpg)  
Fig. 1. Experiment platform including multiple LiDARs (Non-repetitive scanning Livox Mid360, mechanical spinning Hesai PandarXT, small FoV Livox Avia) and built-in IMUs of LiDARs and Pixhawk flight controller. RealSense L515 camera is only used for recording videos in first personal view.

Owing to the strong non-linearity, the performance of sensor fusion system is heavily dependent on accurate initial states provided by efficient initialization module. Initialization of visual-inertial system has been widely studied [11, 12, 13], but few researches have focused on initial ization for LiDAR-inertial system, which is necessary due to reasons below: 1) For self-assembled devices, the LiDAR and IMU are often not time-synchronized and with unknown extrinsic, necessitating extra, laborious temporal and spatial calibration in advance. 2) Points of a LiDAR scan are sampled at different instants, leading to inevitable motion distortion. In case the temporal offset is unknown, IMU aided motion distortion compensation methods adopted by [1, 14, 15] are no longer viable. 3) IMU raw measurements suffer from significant noises and the true values of linear accelerations and angular velocity are coupled with unknown bias. All these challenges drive us to find a well rounded LiDAR-inertial initialization method, capable of providing high-quality initial states including extrinsic transformation, gravity vector, IMU bias, and synchronizing the two sensors without any dedicated hardware setup.

Motivated by this, we propose a fast, robust LiDARinertial initialization method, which can automatically and accurately calibrate temporal offset and provide acceptable initial states without requiring any target or extra sensor, enabling a LiDAR-inertial odometry to run on a customized sensor setup without any dedicated prior calibration or hardware setup. Our contributions are highlighted as follows:

• We propose an efficient, accurate, hardware-free temporal calibration method based on cross-correlation and a unified temporal-spatial optimization, to estimate unknown but constant LiDAR-inertial temporal offset.

• We propose a novel optimization formulation to perform spatial initialization and a method to assess the degree of excitation in data. By further aligning states estimated from LiDAR with noise-mitigated IMU measurements, our initialization can automatically extract initialization data and estimate extrinsic transformation, gravity vector, gyroscope bias and accelerometer bias on the fly.

• We conduct experiments on multiple types of LiDARs and LiDAR-inertial combinations (see Fig. 1) to validate the efficiency and accuracy of our initialization procedure. As far as we know, the proposed method is the first open-sourced temporal and spatial initialization algorithm for 3D LiDAR-inertial system, supporting both mechanical spinning LiDARs and non-repetitive scanning LiDARs.

## II. RELATED WORKS

There is a wide variety of initialization methods for visualinertial systems. For example, an efficient IMU initialization method named VI-ORB-SLAM was introduced by Mur-Artal et al. [11]. The initialization problem is divided into three simple sub-problems and achieves high accuracy in a short time. Authors of [12] propose a robust initialization framework to recover the metric scale of monocular camera and to estimate extrinsic transformation and IMU bias. Huang et al. [13] propose a coarse-to-fine method to calibrate scale factor, gravity, and extrinsic transformation online. For the calibration of temporal offset between camera and inertial sensors, Mair et al. [16] propose a method based on cross-correlation and phase congruency analysis, and calibrate extrinsic rotation following standard hand-eye calibration. Qin et al. [17] propose an online method to calibrate temporal offset by jointly optimizing time offset, camera and IMU states.

In contrast to visual-inertial initialization, the initialization of LiDAR-inertial system is much less studied. Some of the existing initialization methods of LiDAR-inertial system require the sensors to be priorly synchronized, or rely on extra sensor, or ignore some initial states. Specifically, Wang et al. [18] present an online initialization method to estimate the temporal and spatial offset between LiDAR and IMU, but camera is needed as an extra auxiliary sensor. Similarly, GPS/GNSS are used in [19] for acquiring accurate position and attitude of the inertial sensor. Some LiDAR-inertial odometry systems have built-in initialization process, but these initialization modules are usually simple and incomplete. For example, [1] initializes the gyroscope bias, gravity vector, and temporal offset. But the initialization is fairly rough. For instance, the temporal offset is calibrated by assuming the sensor data receiving time as the sampling time, while data transmission and processing delay are totally neglected, leading to imprecise time offset estimation. The gyroscope bias in [1] is calibrated by keeping the sensor still for a few seconds in operation. Since gravity and accelerometer bias are coupled when the device stays still, accelerometer bias is not calibrated in its initialization. Although [1] calibrates the extrinsic online, the extrinsic initialization is not taken into account. So, good initial guess is required otherwise the convergence and robustness of the subsequent LiDAR-inertial odometry will be severely impacted. Similar to [1], initialization of [20] requires the device to stay still for a while. The accelerometer bias and LiDAR-inertial extrinsic parameters are obtained by prior offline calibration while the temporal offset is assumed to be priorly known. Compared to [18, 19], our proposed method does not require any extra sensor. Compared to [1, 20, 21], our work is more complete by initializing all the temporal offset, extrinsic, IMU bias, and gravity vector without any special requirements on the initial motion (e.g., keeping still) or any dedicated time synchronization or pre-calibration.

One of the main goals of our LiDAR-inertial initialization is to calibrate the extrinsic between LiDAR and IMU without any initial estimate. Some existing extrinsic calibration methods are based on batch optimization with tight data association, causing large time consumption. For example, Lv et al. [14] propose a continuous-time batch optimization based calibration. The usage of B-splines leads to more parameters to be estimated and would result in large computation cost. [15] uses an extended Kalman filter to estimate the extrinsic transformation with complicated motion compensation, which has limited convergence speed. Compared with these methods, our method is more lightweight, being able to run on the fly, while still achieving accurate extrinsic calibration sufficient for subsequent online estimation (e.g., by [1]). Our methods also calibrates the temporal offset that are not considered in [14, 15]. Besides, NDT based scan-to-scan matching adopted by [14, 15] usually does not work well for LiDARs with non-repetitive scanning pattern. In contrast, our method adopt scan-to-map matching strategy, which can be easily applied to both repetitive and non-repetitive scanning LiDARs.

## III. METHODOLOGY

## A. Framework Overview

Since IMU is only excited when it is in motion [15], our initialization procedure is a motion-based approach, which means sufficient excitation is necessary. The overview of our workflow is shown in Fig. 2 and some important notations are shown in Table I. The LiDAR odometry (see Section III-B) we propose is modified from FAST-LIO2 [1], by adopting a constant (both angular and linear) velocity (CV) model to predict the LiDAR motion and compensate the point distortion in a scan. To mitigate the mismatch between the constant velocity model and the actual sensor motion, the LiDAR odometry rate is increased by splitting an input frame into several sub-frames. If the LiDAR odometry does not fail (e.g., due to degeneration) and the estimated LiDAR angular and linear velocity satisfy our proposed assessment criterion (see Section III-C.5), the excitation is considered to be sufficient and both LiDAR odometry output and the corresponding IMU data are fed to the initialization module (see Section. III-C). In the initialization, the time offset is first calibrated by shifting IMU measurements to align with the LiDAR odometry, and then followed by an optimization process further refining the time offset, calibrating extrinsic transformation, and estimating IMU bias and gravity vector. The initialized states can be fed to a tightly-coupled LiDARinertial odometry (e.g., [1]) for online state estimation by fusing subsequent LiDAR and IMU data.

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/b24b815db954c5af2966955d67463ae6c1aa7bcdf225d3cdd9297a637a1c4c02.jpg)  
Fig. 2. Framework of our LiDAR-inertial initialization procedure.

SOME IMPORTANT NOTATIONS
<table><tr><td>Notation</td><td>Explanation</td></tr><tr><td>田/日</td><td>The encapsulated “boxplus&quot; and “boxminus&quot; operations on the state manifold.</td></tr><tr><td> $t _ { k }$ </td><td>Timestamp of the k-th LiDAR scan.</td></tr><tr><td> $\rho _ { j }$ </td><td>Timestamp of the j-th point in a LiDAR scan.</td></tr><tr><td> $\tau _ { i }$ </td><td>Timestamp of the i-th IMU measurement.</td></tr><tr><td> $L _ { j } , L _ { k }$ </td><td>The LiDAR body frame at the time  $\rho _ { j }$  and  $t _ { k } .$ </td></tr><tr><td> $\mathbf { x } , \widehat { \mathbf { x } } , \bar { \mathbf { x } }$   $\breve { \mathbf { x } }$ </td><td>The ground-true, predicted, and updated state value.</td></tr><tr><td> ${ } ^ { I } \mathbf { R } _ { L } , { } ^ { I } \mathbf { p } _ { L }$ </td><td>Estimation of  $\mathbf { x } _ { j }$  relative to  $\mathbf { x } _ { k }$  1i in backward propagation. The extrinsic rotation and translation from LiDAR to IMU.</td></tr><tr><td> ${ { I } _ { { { t } _ { L } } } }$ </td><td>The total time offset between LiDAR and IMU.</td></tr><tr><td> $\mathbf { b } _ { \omega } , \mathbf { b } _ { \mathbf { a } }$ </td><td>The bias of gyroscope and accelerometer.</td></tr><tr><td> $G _ { \mathbf { g } }$ </td><td>The gravity vector in global frame.</td></tr><tr><td> $\mathcal { T } _ { i } , \mathcal { T } _ { k }$ </td><td>IMU data sequence used in initialization step with times-</td></tr><tr><td> $\bar { \mathcal { T } } _ { k }$ </td><td>tamp  $\tau _ { i } , t _ { k }$  respectively. IMU data sequence after compensating the initialized time</td></tr><tr><td> $\mathcal { L } _ { k }$ </td><td>offset, with synchronized timestamp  $t _ { k } .$  LiDAR data used in initialization step with timestamp  $t _ { k } .$ </td></tr></table>

## B. LiDAR Odometry

Our LiDAR-only odometry and mapping is built on a constant velocity (CV) motion model, which assumes the angular and linear velocity are constant between two consecutive scans received at $t _ { k }$ and $t _ { k + 1 }$ respectively, $i . e . .$

$$
\mathbf { x } _ { k + 1 } = \mathbf { x } _ { k } \boxplus \left( \Delta t \mathbf { f } \left( \mathbf { x } _ { k } , \mathbf { w } _ { k } \right) \right)\tag{1}
$$

where $\Delta t$ is the time interval between the two scans, the state vector x, noise w, and discrete state transition function f are defined as:

$$
\mathbf { x } = \left[ \begin{array} { l } { \mathbf { \boldsymbol { G } } \mathbf { R } _ { L } } \\ { \mathbf { \boldsymbol { G } } \mathbf { p } _ { L } } \\ { \mathbf { \boldsymbol { G } } \mathbf { v } _ { L } } \\ { \omega _ { L } } \end{array} \right] , \mathbf { w } = \left[ \mathbf { \mathbf { n } } _ { \mathbf { v } } \right] , \mathbf { f } ( \mathbf { x } , \mathbf { w } ) = \left[ \begin{array} { l } { \omega _ { L } } \\ { \mathbf { \boldsymbol { G } } _ { \mathbf { V } _ { L } } } \\ { \mathbf { \mathbf { n } } _ { \mathbf { v } } } \\ { \mathbf { \mathbf { n } } _ { \omega } } \end{array} \right]\tag{2}
$$

where ${ } ^ { G } \mathbf { R } _ { L } \in S O ( 3 ) , { } ^ { G } \mathbf { p } _ { L }$ are the attitude and position of LiDAR in the global frame (here is the first LiDAR body frame $L _ { 0 } ) , \mathbf { \Sigma } ^ { G } \mathbf { v } _ { L }$ is LiDAR’s linear velocity described in global frame, and $\omega _ { L }$ is LiDAR’s angular velocity in LiDAR body frame, which are modelled as a random walk process driven by Gaussian noises $\mathbf { n _ { v } }$ and $\mathbf { n } _ { \omega } ,$ , respectively. In (1), we used the notation ⊞/⊟ defined in [22] to compactly represent the $\mathrm { \Delta ^ { 6 6 } p l u s ^ { , 3 } }$ on the state manifold. Specifically, for the state manifold $S O ( 3 ) \times \mathbb { R } ^ { n }$ in (2), the ⊞ operation and its inverse ⊟ are defined as

$$
\mathbf { \Big [ R \Big ] } \boxplus \left[ \mathbf { r } \right] = \left[ \mathbf { R E x p } ( \mathbf { r } ) \right] ; \left[ \mathbf { R } _ { 1 } \right] \boxplus \left[ \mathbf { R } _ { 2 } \right] = \left[ \mathbf { L o g } ( \mathbf { R } _ { 2 } ^ { T } \mathbf { R } _ { 1 } ) \right]
$$

where R, $\mathbf { R } _ { 1 } , \mathbf { R } _ { 2 } \in S O ( 3 ) , \mathbf { r } , \mathbf { a } , \mathbf { b } \in \mathbb { R } ^ { n } , \mathbf { \delta E x p } ( \cdot ) : \mathbb { R } ^ { 3 } \mapsto \qquad $ $S O ( 3 )$ is the exponential map on SO(3) [22] and $\mathrm { L o g ( \cdot ) }$ $S O ( 3 ) \mapsto \mathbb { R } ^ { 3 }$ is its inverse logarithmic map.

In practice, the sensor motion may not have a constant velocity. To mitigate the effect of this model error, we can split an input LiDAR scan into multiple sub-frames of smaller duration, over which the sensor motion agrees more with the CV model.

1) Error State Iterated Kalman Filter: Based on the onmanifold system representation (1), we use an Error State Iterated Kalman Filter (ESIKF) [23] to estimate its states. The prediction step of the ESIKF consists of state prediction and covariance propagation as follows:

$$
\widehat { \mathbf { x } } _ { k + 1 } = \bar { \mathbf { x } } _ { k } \boxplus ( \Delta t \mathbf { f } ( \bar { \mathbf { x } } _ { k } , \mathbf { 0 } ) )\tag{3}
$$

$$
\widehat { \mathbf { P } } _ { k + 1 } = \mathbf { F } _ { \tilde { \mathbf { x } } } \bar { \mathbf { P } } _ { k } \mathbf { F } _ { \tilde { \mathbf { x } } } ^ { T } + \mathbf { F } _ { \mathbf { w } } \mathbf { Q } \mathbf { F } _ { \mathbf { w } } ^ { \ T } \mathbf { \Lambda } ^ { T }\tag{4}
$$

where P, Q are covariance matrix of state estimation and process noise w respectively. $\mathbf { F } _ { \tilde { \mathbf { x } } }$ and $\mathbf { F _ { w } }$ are as follows:

$$
\begin{array} { r l } & { \mathbf { F } _ { \widetilde { x } } = \frac { \partial ( \overline { { \mathbf { x } } } _ { k } \boxplus \mathbf { \hat { \mathbf { \alpha } } } \mathbf { x } _ { k } \boxplus \mathbf { \hat { \alpha } } \mathbf { ( \Delta } \mathbf { t } \mathbf { f } ( \widetilde { \mathbf { x } } _ { k } \boxplus \delta \mathbf { x } _ { k } , 0 ) \boxplus \overline { { \mathbf { x } } } _ { k } \boxplus \Delta t ( \overline { { \mathbf { x } } } _ { k } , 0 ) ) } { \partial \mathbf { x } _ { k } } } \\ & { \qquad = [ \begin{array} { r r r r } { \mathrm { E x p } ( - \widehat { \omega } _ { I _ { k } } \Delta t ) } & { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { I } _ { 3 \times 3 } \Delta t } \\ { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { 1 } _ { 3 \times 3 } } & { \mathbf { I } _ { 3 \times 3 } \Delta t } & { \mathbf { 0 } _ { 3 \times 3 } } \\ { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { 1 } _ { 3 \times 3 } } & { \mathbf { 0 } _ { 3 \times 3 } } \\ { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { 0 } _ { 3 \times 3 } } & { \mathbf { I } _ { 3 \times 3 } } \end{array} ] } \\ & { \mathbf { F } _ { \mathbf { w } } = \frac { \partial ( \overline { { \mathbf { x } } } _ { k } \boxplus ( \Delta t \mathbf { f } ( \overline { { \mathbf { x } } } _ { k } , \mathbf { w } _ { k } ) ) \in ( \overline { { \mathbf { x } } } _ { k } \boxplus \Delta t \mathbf { f } ( \overline { { \mathbf { x } } } _ { k } , \mathbf { 0 } ) ) } { \partial \mathbf { y } _ { k } } } \\ { \mathbf { 0 } _ { 3 \times 3 } } &  \mathbf { 0 } _  3  \end{array}
$$

where $\delta \mathbf { x }$ represents the error state.

(5)

2) Motion Compensation: In our considered problem, IMU and LiDAR are unsynchronized, hence IMU-aided motion compensation methods adopted by [14, 15] are not viable. After receiving a new LiDAR scan at timestamp $t _ { k + 1 }$ , to compensate the motion distortion, we project each contained point ${ \cal L } _ { j } { \bf \sigma _ { p } } _ { j }$ sampled at timestamp $\rho _ { j } \in \left( t _ { k } , t _ { k + 1 } \right)$ into the scan-end LiDAR frame $L _ { k + 1 }$ as follows. With the constant velocity model, we have $G _ { \widehat { \mathbf { V } } _ { L _ { k + 1 } } } \ = \ G _ { \bar { \mathbf { V } } _ { L _ { k } } }$ $\widehat { \omega } _ { L _ { k + 1 } } ~ = ~ \bar { \omega } _ { L _ { k } }$ , which leads to a relative transformation $\begin{array} { r } { L _ { k + 1 } \ddot { \bf T } _ { L _ { j } } = \left( \overset { . . . } { \underset { . } { \bf R } _ { k + 1 } } \check { \bf R } _ { L _ { j } } , \overset { . . . } { \bf R } _ { k + 1 } \check { \bf p } _ { L _ { j } } \right) } \end{array}$ from time $\rho _ { j }$ to $t _ { k + 1 }$ as:

$$
\begin{array} { r l } & { L _ { k + 1 } \check { \mathbf { R } } _ { L _ { j } } = \mathrm { E x p } ( - \widehat { \omega } _ { L _ { k + 1 } } \Delta t _ { j } ) } \\ & { L _ { k + 1 } \check { \mathbf { p } } _ { L _ { j } } = - ^ { G } \widehat { \mathbf { R } } _ { L _ { k + 1 } } ^ { T } G _ { \widehat { \mathbf { V } } _ { L _ { k + 1 } } } \Delta t _ { j } } \\ & { ~ \Delta t _ { j } = t _ { k + 1 } - \rho _ { j } . } \end{array}\tag{6}
$$

Then the local measurement ${ L _ { j } } _ { \mathbf { p } _ { j } }$ can be projected to scan-end LiDAR frame as

$$
\begin{array} { r } { L _ { k + 1 } \mathbf p _ { j } = { } ^ { L _ { k + 1 } } \mathbf { \check { T } } _ { L _ { j } } { } ^ { L _ { j } } \mathbf p _ { j } } \end{array}\tag{7}
$$

Then the distortion-compensated scan $\left\{ { { \begin{array} { l } { L _ { k + 1 } } \\ { \qquad P _ { j } } \end{array} } } \right\}$ provides an implicit measurement of the unknown state $\mathbf { \bar { \Pi } } ^ { \mathbf { \Lambda } _ { G } } \mathbf { T } _ { L _ { k + 1 } }$ expressed as the point-to-plane distance residual, based on which the full state $\mathbf { x } _ { k + 1 }$ is iteratively estimated in an iterated Kalan filter framework until convergence. The converged state estimate, denoted as $\bar { \mathbf { x } } _ { k + 1 }$ , will then be used to propagate the subsequent IMU measurements as in Section III-B.1. Details of this iterative estimation can be referred to FAST-LIO2 [1] or [23] for a more general treatment of manifold constraints. The mapping comparison of our LiDAR odometry using scans with and without motion compensation is shown in Fig. 3.

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/49b188aab4032006e7db6d269da9cb571c54d946b42e3438a3d247c44dfb39f6.jpg)  
Fig. 3. The mapping result comparison using Livox Mid360 LiDAR. (a) Map without motion compensation. (b) Point cloud map using distortion compensated scans following (6) and (7). (c, d) Mapping details of (b)

## C. LiDAR-inertial Initialization

The LiDAR odometry in Section III-B outputs the Li-DAR’s angular velocity $\omega _ { L _ { k } }$ and linear velocity $\bar { G } _ { \mathbf { V } _ { L _ { k } } }$ at each scan-end time with timestamp $t _ { k }$ . Meanwhile, IMU provides raw measurements, which are body angular velocity $\omega _ { m _ { i } }$ and linear acceleration $\mathbf { a } _ { m } .$ with timestamp $\tau _ { i } .$ . These data are accumulated and repeatedly assessed by the excitation criterion shown in Section. III-C.5. Once data of sufficient excitation is collected, the initialization module is called, which eventually outputs time offset $\boldsymbol { \mathit { \tau } } ^ { I } { } _ { t _ { L } } \in \mathbb { \mathbb { R } }$ , extrinsic ${ ^ I } \mathbf { T } _ { L } = ( ^ { I } \mathbf { R } _ { L } , ^ { I } \mathbf { \bar { p } } _ { L } ) \in S E ( 3 )$ , IMU bias $\mathbf { b } _ { \omega } , \mathbf { b _ { a } } \in \mathbb { R } ^ { 3 }$ , and gravity vector $G _ { \mathbf { g } } \in \mathbb { R } ^ { 3 }$ in the global frame.

1) Data Preprocess: The IMU raw measurements suffer from noises $\mathbf { n } _ { \omega _ { i } }$ and $\mathbf { n } _ { \mathbf { a } _ { i } }$ . The IMU measurement model is:

$$
\omega _ { m _ { i } } = \omega _ { i } ^ { \mathrm { g t } } + \mathbf { b } _ { \omega } + \mathbf { n } _ { \omega _ { i } } , ~ \mathbf { a } _ { m _ { i } } = \mathbf { a } _ { i } ^ { \mathrm { g t } } + \mathbf { b } _ { \mathbf { a } } + \mathbf { n } _ { \mathbf { a } _ { i } }\tag{8}
$$

where $\omega _ { i } ^ { \mathrm { g t } } , \mathbf { a } _ { i } ^ { \mathrm { g t } }$ are the ground-truth of IMU angular velocity and linear acceleration. Similarly, the estimations $\omega _ { L _ { k } } , \mathrm { { } } ^ { G } { \bf v } _ { L _ { k } }$ from the LiDAR odometry contain noise as well.

To mitigate these noises, which are usually of high frequency, a non-causal zero phase low-pass filter [24] is used to filter the noise without introducing any filter delay. The zero phase filter is implemented by running a Butterworth low-pass filter forward and backward [24], producing noiseattenuated IMU measurements $\omega _ { I _ { i } } { \ } = \ \omega _ { i } ^ { \mathrm { g t } } + { \bf b } _ { \omega } , { \bf a } _ { I _ { i } }$ ${ \bf a } _ { i } ^ { \mathrm { g t } } + { \bf b } _ { \bf a }$ . The noise-attenuated LiDAR estimations are still denoted as $\omega _ { L _ { k } } , \mathrm { { } } ^ { G } { \bf v } _ { L _ { k } }$ for notation simplicity.

From the LiDAR odometry $\omega _ { L _ { k } } , \mathrm { { } } ^ { G } { \bf v } _ { L _ { k } }$ , we obtain the LiDAR angular and linear accelerations $\Omega _ { L _ { k } } , { } ^ { G } \mathbf { a } _ { L _ { k } }$ by noncausal central difference [25]. The resultant LiDAR odometry data can be collectively denoted as

$$
\mathcal { L } _ { k } = \left\{ \omega _ { L _ { k } } , { } ^ { G } \mathbf { v } _ { L _ { k } } , \Omega _ { L _ { k } } , { } ^ { G } \mathbf { a } _ { L _ { k } } \right\}\tag{9}
$$

Similarly, we obtain the IMU angular acceleration $\Omega _ { I _ { i } }$ from noise-attenuated gyroscope measurements $\omega _ { I _ { i } }$ , leading to:

$$
{ \mathcal { T } } _ { i } = \{ \omega _ { I _ { i } } , \mathbf { a } _ { I _ { i } } , \Omega _ { I _ { i } } \}\tag{10}
$$

Since IMU frequency is usually higher than that of LiDAR odometry, the two sequence $\mathcal { T } _ { i }$ and $\mathcal { L } _ { k }$ are not of the same size. To fix this, we extract the LiDAR and IMU data received within the same time period, and down-sample $\mathcal { T } _ { i }$ by linearly interpolating it at each LiDAR odometry time $t _ { k }$ (see Fig. 4). The down-sampled IMU data is denoted as $\mathcal { T } _ { k }$ :

$$
{ \mathcal { T } } _ { k } = \{ \omega _ { I _ { k } } , \mathbf { a } _ { I _ { k } } , \Omega _ { I _ { k } } \}\tag{11}
$$

which has the same timestamp $t _ { k }$ with $\mathcal { L } _ { k }$ (but the data is really delayed by the known temporal constant ${ \mathit { \Pi } } ^ { I } t _ { L } )$

<table><tr><td rowspan="2"> $\mathcal { T } _ { i }$  T1 Raw IMU data</td><td rowspan="2"></td><td colspan="3">T2 T3</td></tr><tr><td>Linear</td><td></td><td></td></tr><tr><td>Interpolated &amp; downsampled</td><td> $\mathcal { T } _ { k }$ </td><td>Interpolation</td><td></td><td></td></tr><tr><td>IMU data</td><td></td><td>t1</td><td>t2</td><td>t3</td></tr><tr><td>Raw LiDAR data</td><td> $\mathcal { L } _ { k }$ </td><td>x</td><td>x t2</td><td></td></tr></table>

Fig. 4. Down sample IMU data by interpolating it at each LiDAR odometry timestamp.

2) Temporal Initialization by Cross-Correlation: In most cases, due to inevitable transmission and processing delay prior to its reception by LiDAR-inertial odometry module, an unknown but constant offset ${ { I } _ { { { t } _ { L } } } }$ between the LiDAR $\mathcal { L } _ { k }$ and IMU $\mathcal { T } _ { k }$ will exist, such that the IMU measurement $\mathcal { T } _ { k }$ , if advanced by ${ { I } _ { { { t } _ { L } } } }$ , will be aligned with the LiDAR odometry $\mathcal { L } _ { k }$ . Since the LiDAR data (9) and IMU data (11) are at discrete times $t _ { k } .$ , advancing the IMU data is essentially made in discrete steps $d = \bar { { t } _ { L } } / \Delta t$ , where $\Delta t$ is the time interval between two LiDAR scans. Specifically, for the angular velocity, we have

$$
\boldsymbol { \omega } _ { I _ { k + d } } = { } ^ { I } \mathbf { R } _ { L } \boldsymbol { \omega } _ { L _ { k } } + \mathbf { b } _ { \omega }\tag{12}
$$

Ignoring the gyroscope bias $\mathbf { b } _ { \omega } .$ , which is usually small, we find that the magnitude of $\omega _ { I _ { k + d } }$ and $\omega _ { L _ { k } }$ should be the same, regardless of the extrinsic ${ \bf \dot { \cal I } } _ { \bf R _ { \cal L } }$ . Inspired by [16], we use the zero-centered cross-correlation to quantify the similarity between their magnitude. Then, the offset d can be solved from the following optimization problem

$$
d ^ { * } = \arg \operatorname* { m a x } _ { \boldsymbol { d } } \sum \| \omega _ { I _ { k + d } } \| \cdot \| \omega _ { L _ { k } } \|\tag{13}
$$

by enumerating the offset $d$ in the index range of $\mathcal { L } _ { k }$

3) Unified Extrinsic Rotation and Temporal Calibration: The cross-correlation method is robust against noise and small-scale gyroscope bias. But one obvious defect of (13) is that the calibration resolution of the temporal offset can only be made up to one sampling interval $\Delta t$ of the LiDAR odometry, any residual offset $\bar { \delta t }$ smaller than $\Delta t$ cannot be identified. Let ${ { I } _ { { { t } _ { L } } } }$ be the total offset between LiDAR odometry $\omega _ { L }$ and IMU data $\omega _ { I }$ , then ${ { ^ I t } _ { L } } \mathrm { ~ = ~ } { { d } ^ { * } } \Delta t \mathrm { ~ + ~ } \delta t$ Similar to (12), the IMU measurement $\omega _ { I } .$ , if advanced by time ${ { I } _ { { { t } _ { L } } } }$ , will be aligned with the LiDAR odometry $\omega _ { L } \colon$

$$
\omega _ { I } ( t + { } ^ { I } t _ { L } ) = { } ^ { I } { \bf R } _ { L } \omega _ { L } ( t ) + { \bf b } _ { \omega }\tag{14}
$$

Since the actual LiDAR odometry $\omega _ { L }$ in (9) is only available at timestamps $t _ { k }$ , substituting $t = t _ { k }$ and $^ { I } t _ { L } \ =$ $d ^ { * } \Delta t + \delta t$ into (14) and noticing $\omega _ { L } ( t _ { k } ) = \omega _ { L _ { k } }$ , we have

$$
\omega _ { I } ( t _ { k } + d ^ { * } \Delta t + \delta t ) = { } ^ { I } \mathbf { R } _ { L } \omega _ { L _ { k } } + \mathbf { b } _ { \omega } .\tag{15}
$$

Notice that $\omega _ { I } ( t _ { k } + d ^ { * } \Delta t + \delta t )$ is the IMU angular velocity right after time $t _ { k } + d ^ { \ast } \Delta t$ , where the angular velocity and acceleration are $\omega _ { I } ( t _ { k } + d ^ { * } \Delta t ) = \omega _ { I _ { k } }$ and $\Omega _ { I } ( t _ { k } + d ^ { * } \dot { \Delta } t ) =$ $\Omega _ { I _ { k ^ { \prime } } }$ , respectively, where $k ^ { \prime } = k + \ddot { d } ^ { * }$ . We can interpolate the value of $\omega _ { I } ( t _ { k } + d ^ { * } \Delta t + \delta t )$ by assuming the angular acceleration is constant over the small δt (see Fig. 5):

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/7686253a56908652990a3cc16efe46a56f64a471b58ef2d222431a8571226fd7.jpg)  
Fig. 5. Illustration of time offset calibration and the first order approximation shown in Equation (16).

$$
\omega _ { I } ( t _ { k } + d ^ { * } \Delta t + \delta t ) \approx \omega _ { I _ { k ^ { \prime } } } + \delta t \Omega _ { I _ { k ^ { \prime } } }\tag{16}
$$

which can be substituted into (15) to obtain

$$
\boldsymbol { \omega } _ { I _ { k ^ { \prime } } } + \delta t \pmb { \Omega } _ { I _ { k ^ { \prime } } } = { } ^ { I } \mathbf { R } _ { L } \boldsymbol { \omega } _ { L _ { k } } + \mathbf { b } _ { \omega }\tag{17}
$$

Finally, based on the constraint in (17), the unified temporal-spatial optimization problem can be stated as:

$$
\underset { ^ { I } \mathbf { R } _ { L } , \mathbf { b } _ { \omega } , \delta t } { \arg \operatorname* { m i n } } \sum \| ^ { I } \mathbf { R } _ { L } \omega _ { L _ { k } } + \mathbf { b } _ { \omega } - \omega _ { I _ { k ^ { \prime } } } - \delta t \cdot \pmb { \Omega } _ { I _ { k ^ { \prime } } } \| ^ { 2 }\tag{18}
$$

which is solved iteratively (due to the nonlinear constraint ${ \mathbf { \Gamma } } ^ { I } { \mathbf { R } } _ { L } \in \mathrm { ~ } S O ( 3 ) )$ by Ceres Solver<sup>2</sup> from an initial value of $( ^ { I } { \bf R } _ { L } , { \bf b } _ { \omega } , \delta t ) = ( { \bf I } _ { 3 \times 3 } , { \bf 0 } _ { 3 \times 1 } , 0 )$

4) Extrinsic Translation and Gravity Initialization: In Section III-C.3, we obtained the extrinsic rotation ${ } ^ { I } { \bf R } _ { L }$ gyroscope bias $\mathbf { b } _ { \omega }$ and the temporal offset ${ { I } _ { { { t } _ { L } } } }$ . In this section, we fix these values and proceed to the calibration of extrinsic translation, gravity vector, and acceleromter bias.

First, we align the IMU data $\mathcal { T } _ { k }$ with that of LiDAR $\mathcal { L } _ { k }$ using the offset $d ^ { * }$ and residual δt previously calibrated. The aligned IMU data is denoted as $\bar { \mathcal { T } } _ { k }$ , which is now assumed to be perfectly aligned with $\mathcal { L } _ { k }$ without time offset. Specifically, the IMU angular velocity $\bar { \omega } _ { I _ { k } }$ corresponding to LiDAR angular velocity $\omega _ { L _ { k } }$ at time $t _ { k }$ is (see (15))

$$
\begin{array} { r } { \bar { \omega } _ { I _ { k } } = \omega _ { I } ( t _ { k } + d ^ { * } \Delta t + \delta t ) \approx \omega _ { I _ { k + d } } + \delta t \Omega _ { I _ { k + d } } , } \end{array}\tag{19}
$$

Similarly, the IMU acceleration $\bar { \mathbf { a } } _ { I _ { k } }$ corresponding to LiDAR acceleration ${ { G } _ { { \bf { a } } _ { L _ { k } } } }$ at time $t _ { k }$ is

$$
\begin{array} { l } { { { \bar { \mathbf { a } } } _ { I _ { k } } } = { \mathbf { a } } _ { I } ( t _ { k } + { d ^ { * } } \Delta t + \delta t ) } \\ { \approx { \mathbf { a } } _ { I _ { k + d } } + \displaystyle \frac { \delta t } { \Delta t } ( { \mathbf { a } } _ { I _ { k + d + 1 } } - { \mathbf { a } } _ { I _ { k + d } } ) , } \end{array}\tag{20}
$$

Then, similar to (14), we can find the acceleration constraint between IMU and LiDAR. As marked in [26], the accelerations of two frames A, B with fixed extrinsic have the following relationship:

$$
{ } ^ { A } { \bf R } _ { B } { \bf a } _ { B } = { \bf a } _ { A } + \lfloor \omega _ { A } \rfloor _ { \wedge } ^ { 2 A } { \bf p } _ { B } + \lfloor \Omega _ { A } \rfloor _ { \wedge } ^ { A } { \bf p } _ { B }\tag{21}
$$

where ${ } ^ { A } \mathbf { R } _ { B } , { } ^ { A } \mathbf { p } _ { B }$ represent the extrinsic transformation from frame $B$ to frame A. Both $\mathbf { a } _ { A } , \mathbf { a } _ { B }$ are described in their own body frame.

For LiDAR-inertial system, we have two choices: A for IMU and B for LiDAR, or the opposite situation. Noticing that in the first case the accuracy of $\omega _ { A } = \bar { \omega } _ { I _ { k } } - \mathbf { b } _ { \omega }$ is influenced by gyroscope bias estimation, and the error of $\Omega _ { A }$ would be amplified due to the noise in angular velocity measurements. To avoid this problem and increase the robustness of extrinsic translation calibration, we set LiDAR as A and IMU as B. Since LiDAR’s acceleration ${ { G } _ { { \bf { a } } _ { L _ { k } } } }$ is described in the global frame, we need to calculate LiDAR’s instant acceleration described in body frame, denoted as $\mathbf { a } _ { L _ { k } } \colon$

$$
\mathbf { a } _ { L _ { k } } = { ^ { G } \mathbf { R } _ { L } ^ { T } } ( { ^ { G } \mathbf { a } _ { L _ { k } } } - { ^ { G } \mathbf { g } } )\tag{22}
$$

where ${ \cal G } _ { \mathbf { R } _ { L } }$ is the LiDAR’s attitude in the global frame and is obtained from the LiDAR odometry in Section III-B.

Finally, the extrinsic translation, accelerometer bias, and gravity vector can be jointly estimated from the following optimization problem:

$$
\underset { ^ { I } \mathbf { p } _ { L } , \mathbf { b } _ { \mathbf { a } } , \sigma \mathbf { g } } { \arg \operatorname* { m i n } } \sum \| ^ { I } \mathbf { R } _ { L } ^ { T } ( \bar { \mathbf { a } } _ { I _ { k } } - \mathbf { b } _ { \mathbf { a } } ) - \mathbf { a } _ { L _ { k } } - ( \vert \omega _ { L _ { k } } \vert _ { \Lambda } ^ { 2 } + \vert \Omega _ { L _ { k } } \vert _ { \Lambda } ) ^ { L } \mathbf { p } _ { I } \| ^ { 2 }\tag{23}
$$

which can be solved iteratively (due to the constraint $G _ { \mathbf { g } } \in$ $\mathbb { S } _ { 2 } )$ by Ceres Solver from the initial value $( ^ { I } \mathbf { p } _ { L } , \mathbf { b _ { a } } , ^ { G } \mathbf { g } ) =$ $( \mathbf { 0 } _ { 3 \times 1 } , \mathbf { 0 } _ { 3 \times 1 } , 9 . 8 1 \mathbf { e } _ { 3 } )$ . After ${ \boldsymbol { \mathbf { \mathit { L } } } } _ { \mathbf { \mathbf { \mathit { p } } } _ { I } }$ is estimated, the translation from LiDAR to IMU can be computed as ${ \displaystyle { } ^ { I } { \bf p } _ { L } = - { } ^ { I } { \bf R } _ { L } { } ^ { L } { \bf p } _ { I } }$

5) Data Accumulation Assessment: The proposed initialization method relies on sufficient excitation (adequate motion) of LiDAR-inertial device. Thus, the system should be capable of assessing whether the excitation is sufficient to perform initialization all by itself. Ideally, the excitation can be assessed by the rank of the full Jacobian matrix of (18) for $( ^ { I } \mathbf { R } _ { L } , \mathbf { b } _ { \omega } , \delta t )$ and (23) for $( ^ { I } \mathbf { p } _ { L } , \mathbf { b _ { a } } , ^ { G } \mathbf { g } )$ . In practice, we found that it is sufficient to assess the Jacobian w.r.t. the extrinsic rotation ${ \boldsymbol { I } } _ { \mathbf { R } _ { L } }$ and extrinsic translation ${ \boldsymbol { I } } _ { \mathbf { p } _ { L } }$ only, since excitation on the extrinsic usually require complicated motion that excite the other states as well. Denote $\mathbf { J } _ { r }$ the Jacobian of (18) w.r.t. ${ } ^ { I } { \bf R } _ { L }$ and $\mathbf { J } _ { t }$ the Jacobian of (23) w.r.t. ${ \boldsymbol { I } } _ { \mathbf { p } _ { L } }$

$$
\mathbf { J } _ { r } = \left[ - \mathop { \mathbf { \mu } } _ { \mathbf { \mu } _ { L } } ^ { \vdots } \biggr | \underset { \mathbf { \mu } _ { L } } { \vdots } \biggr ] _ { \mathbf { \mu } _ { L } } \right] , \mathbf { J } _ { t }  &  = \left[ \underset { \mathbf { \mu } _ { L _ { k } } } { \vdots } \biggr | \underset { \mathbf { \mu } _ { L _ { k } } } { \vdots } \biggr ] _ { \mathbf { \mu } _ { L _ { k } } } \biggr ] _ { \mathbf { \kappa } } \right] .\tag{24}
$$

Then the excitation can be assessed from the rank of $\begin{array} { r } { \mathbf { J } _ { r } ^ { T } \mathbf { J } _ { r } ~ = ~ \sum \left\lfloor \omega _ { L _ { k } } \right\rfloor _ { \wedge } ^ { T } \left\lfloor \omega _ { L _ { k } } \right\rfloor _ { \wedge } } \end{array}$ and $\begin{array} { r } { { \bf J } _ { t } ^ { T } { \bf J } _ { t } \ = \ \sum ( \lfloor \omega _ { L _ { k } } \rfloor _ { \wedge } ^ { 2 } } \end{array}$ + $\lvert \Omega _ { L _ { k } } \rvert _ { \wedge } ) ^ { T } ( \lvert \omega _ { L _ { k } } \rvert _ { \wedge } ^ { 2 } + \lvert \Omega _ { L _ { k } } \rvert _ { \wedge } )$ . More quantitatively, the extent of excitation is indicated by the singular values of $\mathbf { J } _ { r } ^ { T } \mathbf { J } _ { r }$ and $\mathbf { J } _ { t } ^ { T } \mathbf { J } _ { t }$ . Based on this principle, we developed an assessment program that can instruct the users how to move their devices to obtain sufficient excitation. We quantify the excitation based on the singular values of the Jacobian matrix, and set a threshold to assess if the excitation is sufficient.

## IV. EXPERIMENTS

We evaluate our initialization method mainly on datasets collected by our self-assembled LiDAR-inertial handheld setup (Fig. 1). We test our initialization algorithm with multiple types of LiDAR (Livox<sup>3</sup> Avia/Mid360 and Hesai

TABLE II  
TEMPORAL INITIALIZATION RESULTS
<table><tr><td rowspan=1 colspan=1>LiDAR</td><td rowspan=1 colspan=1> ${ { \mathbf { \mathit { \Pi } } } ^ { I } } t _ { L } \ \left( \mathbf { \mathbf { \mathit { s } } } \right)$ </td><td rowspan=1 colspan=1>Mean[s]</td><td rowspan=1 colspan=1>RMSE[s]</td><td rowspan=1 colspan=1>NEES</td></tr><tr><td rowspan=3 colspan=1>Avia</td><td rowspan=1 colspan=1>0.05</td><td rowspan=1 colspan=1>0.0490</td><td rowspan=1 colspan=1>0.0016</td><td rowspan=1 colspan=1>3.2%</td></tr><tr><td rowspan=1 colspan=1>0.1</td><td rowspan=1 colspan=1>0.0988</td><td rowspan=1 colspan=1>0.0017</td><td rowspan=1 colspan=1>1.7%</td></tr><tr><td rowspan=1 colspan=1>0.5</td><td rowspan=1 colspan=1>0.4989</td><td rowspan=1 colspan=1>0.0018</td><td rowspan=1 colspan=1>0.36%</td></tr><tr><td rowspan=3 colspan=1>Mid360</td><td rowspan=1 colspan=1>0.05</td><td rowspan=1 colspan=1>0.0479</td><td rowspan=1 colspan=1>0.0034</td><td rowspan=1 colspan=1>6.8%</td></tr><tr><td rowspan=1 colspan=1>0.1</td><td rowspan=1 colspan=1>0.0982</td><td rowspan=1 colspan=1>0.0028</td><td rowspan=1 colspan=1>2.8%</td></tr><tr><td rowspan=1 colspan=1>0.5</td><td rowspan=1 colspan=1>0.4984</td><td rowspan=1 colspan=1>0.0029</td><td rowspan=1 colspan=1>0.58%</td></tr></table>

Temporal initialization results of Livox LiDARs and their built-in IMUs. RMSE means root mean square error and NEES denotes normalized estimation error squared of calibrated time offset [17], computed as $\mathrm { N E E S } = \mathrm { R M S E } / ^ { \bar { I } } t _ { L } .$

PandarXT<sup>4</sup>) and 6-axis IMUs (Bosch BMI088 inside both Pixhawk flight controller<sup>5</sup> and Livox LiDARs). The original data frequency of LiDAR is set as 10 $H z \ ( 1 0$ scans per second), the frequency of IMU raw data is 200 Hz. All the experiments are conducted on a desktop computer with Intel i7-10700 @2.90 GHz with 32 GB RAM. In all experiments, the initial states are set to $( ^ { I } { \bf R } _ { L } , { \bf b } _ { \omega } , \delta t ) = ( { \bf I } _ { 3 \times 3 } , { \bf 0 } _ { 3 \times 1 } , 0 )$ in (18) and $( ^ { I } { \bf p } _ { L } , { \bf b } _ { \bf a } , ^ { G } { \bf g } ) = ( { \bf 0 } _ { 3 \times 1 } , { \bf 0 } _ { 3 \times 1 } , 9 . 8 1 { \bf e } _ { 3 } )$ in (23). The initialization of other states does not need any initial values.

## A. Temporal Initialization Evaluation

In order to show the effectiveness and accuracy of our temporal initialization (calibration) method, we test it on data collected by Livox LiDARs (Livox Avia and Livox Mid360) and their built-in IMUs. Since each Livox LiDAR and its built-in IMU are hardware synchronized in factory, the ground-true time offset is around 0 seconds, with microsecond accuracy level. To demonstrate the capability of our temporal calibration, we manually shift the input IMU timestamps to construct an artificial time offset [17], by adding a fixed value ${ { I } _ { { { t } _ { L } } } }$ to IMU timestamps. We collect 5 data sequences in a laboratory scene using each Livox LiDAR and its built-in IMU. The calibration results are shown in Table II. As can be seen, the temporal calibration error is in microsecond level which suffices the requirements of most LiDAR-inertial sensor fusion algorithms. The results show our temporal calibration approach has great accuracy and consistency.

For unsynchronized self-assembled LiDAR-inertial system, the ground-true time offset is hard to obtain. An indirect way that can validate the effectiveness of our approach is to examine the performance of tightly-coupled LiDAR-Inertial-Odometry (LIO) with the calibrated time offset. The LIO we adopt is FAST-LIO2 [1], a state-of-the-art LiDAR inertial odometry that provides a temporal synchronization and state initialization internally. We test FAST-LIO2 with temporal offset calibrated by itself and by our approach, while keeping other state initialization unchanged with extrinsic from CAD reference, on data sequences collected by unsynchronized Hesai PandarXT LiDAR and Pixhawk IMU (Fig. 1). The localization and mapping results are shown in Fig. 6. It can be seen that the overall LIO performance with our temporal calibration is better than that with the $\mathrm { L O } ^ { \circ } \mathrm { s }$ internal time synchronization. The odometry end-to-end drift is only 0.0102 m over a 11.3627 m trajectory for our method while 0.246 m for the LIO’s internal time synchronization, the map with our approach also preserves more fine structural details.

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/f17991db67ef0f196cb5539bb7f2b4e2dcf60933cf7f2b14e3cbf68bfcc4719a.jpg)  
Fig. 6. Mapping result comparison (bird view) using FAST-LIO2 on the same dataset collected in laboratory. The device is waved back and forth to obtain sufficient excitation, and return back to the origin. The entire sequence lasts 55.8 seconds, where the first 20 seconds are used to calibrate temporal offset. We rerun FAST-LIO2 on the entire sequence with the calibrated offset. (a) Mapping result with time synchronization in FAST-LIO2, the map accuracy is limited by the inaccurate internal synchronization.

## B. Spatial Initialization Evaluation

1) Multiple LiDAR Types Test: We test our spatial (extrinsic) initialization method on data sequences acquired by platform shown in Fig. 1. As the LiDARs are covered with shell and the IMU is built in Pixhawk flight controller, their precise extrinsic are hard to know. However, we can validate our extrinsic initialization approach in an indirect way. We fix the Pixhawk at two poses $\mathcal { T } _ { 1 }$ and $\mathcal { T } _ { 2 }$ shown in Fig. 1, their ground-true relative pose is designed in CAD and ensured in actual assembly. The ground-true relative pose is (0, 0, 0) degrees in Euler angle for rotation and (0.25, 0, 0) meters for translation, the manufacturing accuracy level of CAD can achieve 0.01 degrees and millimeters. In each pose, the extrinsic of LiDAR and IMU is calibrated, denoted as ${ \cal I } _ { 1 } { \bf \Delta T } _ { L }$ , ${ \cal I } _ { { } ^ { 2 } \mathbf { T } _ { L } , }$ , respectively. The relative pose is then computed as $\bar { I _ { 1 } } \mathbf { T } _ { I _ { 2 } } ^ { - } = \bar { I _ { 1 } } \mathbf { T } _ { L } ^ { I _ { 2 } } \bar { \mathbf { T } } _ { L } ^ { - 1 }$ , which can be compared with groundtruth provided by CAD reference. Aiming to prove that our calibration method supports multiple LiDARs with different scanning patterns, we collect 5 data sequences for each LiDAR, including Livox Mid360, Livox Avia, and Hesai PandarXT, with the Pixhawk IMU at both two poses.

In all experiments, we set initial extrinsic as $( ^ { I } { \bf R } _ { L } , { } ^ { I } { \bf p } _ { L } ) =$ $\left( \mathbf { I } _ { 3 \times 3 } , \mathbf { 0 } _ { 3 \times 1 } \right)$ , even though the ground-truth is far from the initial values (e.g., for Mid360, IMU is at $\mathcal { T } _ { 1 }$ , the actual extrinsic is about (0, −2, 178) degrees for rotation and (0.12, 0, 0.11) meters for translation). We calculate the absolute values of the relative IMU pose errors, in average and standard deviation. The results are shown in Table III. As can be seen, the translation errors are in centimeter level, the rotational errors are less than 1<sup>◦</sup>. The overall small errors indicates the extrinsic initialization results are close across different datasets with the same sensor setup, and can be used as high-quality initial values for the subsequent LiDARinertial odometry.

Another interesting phenomenon from Table III is that, the extrinsic calibration errors of Livox Avia and Livox Mid360 are smaller than that of Hesai PandarXT. This mainly benefits

TABLE III  
EXTRINSIC INITIALIZATION RESULTS
<table><tr><td rowspan=1 colspan=2>LiDAR</td><td rowspan=1 colspan=1>Livox Mid360</td><td rowspan=1 colspan=1>Livox Avia</td><td rowspan=1 colspan=1>PandarXT</td></tr><tr><td rowspan=2 colspan=1>RelativeError</td><td rowspan=1 colspan=1>Rot(°)</td><td rowspan=1 colspan=1>0.2472±0.2043</td><td rowspan=1 colspan=1>0.4019±0.1708</td><td rowspan=1 colspan=1>0.7244±0.5076</td></tr><tr><td rowspan=1 colspan=1>Trans(m)</td><td rowspan=1 colspan=1>0.0081±0.0075</td><td rowspan=1 colspan=1>0.0064±0.0069</td><td rowspan=1 colspan=1>0.0133±0.0102</td></tr></table>

The mean value and SD (standard deviation) of extrinsic initialization.

TABLE IV  
EXTRINSIC CALIBRATION COMPARISON
<table><tr><td rowspan=1 colspan=1>Method</td><td rowspan=1 colspan=1>Rotation(°)</td><td rowspan=1 colspan=1>Translation(m)</td><td rowspan=1 colspan=1>Time(s)</td></tr><tr><td rowspan=1 colspan=1>Proposed</td><td rowspan=1 colspan=1>0.6208</td><td rowspan=1 colspan=1>0.0162</td><td rowspan=1 colspan=1>10.2</td></tr><tr><td rowspan=1 colspan=1>LI-Calib [14]</td><td rowspan=1 colspan=1>1.0375</td><td rowspan=1 colspan=1>X</td><td rowspan=1 colspan=1>332.6</td></tr><tr><td rowspan=1 colspan=1>Target-Free [15]</td><td rowspan=1 colspan=1>0.8483</td><td rowspan=1 colspan=1>0.0187</td><td rowspan=1 colspan=1>115.7</td></tr></table>

The first 40 seconds of the dataset is used for calibration. × denotes the refinement of LI-Calib [14] fails and calibration result diverges due to ignorance of gravity vector initialization. So we use its rotation calibrated in the coarse calibration.

from the non-repetitive scanning of Livox LiDARs. Since our LiDAR odometry is based on constant velocity (CV) model, the input LiDAR frame is splitted into several subframes according to sampling time, to increase the odometry frequency and hence mitigate the CV model mismatch. For LiDARs with non-repetitive scanning, the frame splitting would not change the FOV of subframes. Also, the point map would get denser when the LiDAR stays still, which benefits our scan-to-map strategy. In contrast, for mechanical spinning LiDARs like Hesai PandarXT, the frame splitting would decrease the FOV of subframes and the robustness of LiDAR odometry. Thus, we can not move quickly when collecting data, which lowers the IMU excitation and the signal-to-noise ratio (SNR).

2) Accuracy and Robustness Comparison: To further verify the accuracy of our extrinsic initialization, we select the state-of-the-art LiDAR-inertial extrinsic calibration methods for comparison, which are LI-Calib [14] and Target-Free [15]. Since these two methods only support mechanical spinning LiDAR, we select two sequences collected by Hesai PandarXT and Pixhawk in Section IV-B.1, and use the same relative IMU pose error to assess the calibration accuracy. Fig. 7 (e) shows the scene of the two sequences. We use the first 40 seconds (including 400 LiDAR scans and corresponding IMU measurements) to run all calibration methods. The temporal offset is compensated in advance when testing LI-Calib and Target-Free since they are unable to do temporal calibration. The default parameters of the two methods are used on both sequences. The time-consumption and average relative IMU pose errors are shown in Table IV. As can be seen, our method is more accurate while consuming much less time. Fig. 7 shows some qualitative results. LI-Calib [14] adopts a coarse-to-refine calibration. In the coarse calibration, it registers LiDAR scans by NDT matching and calibrates the extrinsic rotation; in the refine stage, it performs batch optimization of all the extrinsic parameters. The map constructed in the coarse calibration is shown in Fig. 7 (a). The refinement of LI-Calib fails due to the ignorance of gravity initialization, leading to a messy surfel map shown in Fig. 7 (b). In contrast, the LiDAR-only odometry in our calibration method is robust and accurate, the point map is shown in Fig. 7 (c,d). Target-Free [15] does not provide visualization module to show its map. These results show our method has better robustness than [14], and similar accuracy compared with [15], and much less computation cost than both [14, 15].

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/4feb011fc7e34b8fd09f378b6d9e280ac2a4cf92886b84b8cde559248d5dd01f.jpg)  
Fig. 7. (a) Surfel map constructed during the coarse calibration of LI-Calib [14]. (b) Refinement of LI-Calib fails, surfel map in a mess. (c,d) Accurate point cloud map constructed during our calibration, see Section III-B. (e) The calibration scene.

## C. Time Consumption Evaluation

Compared with [14, 15], two extrinsic calibration methods with high computation load, which cannot be processed in real-time, our approach is fast and can be implemented in real-time. Different from minimizing point-to-plane with batch optimization, our LiDAR odometry is very efficient, the average processing time of a subframe is about 8 ms. Once sufficient data is collected, the total time consumption of initialization solver, including data pre-processing, temporal initialization, extrinsic and gravity initialization is less than 500 ms, which is far smaller than the actual data collection time.

To evaluate the efficiency, we test [14], [15] and our method on the same dataset collected in an apartment hallway, and compare the time consumption of motion compensation for each scan. Also, we compare the total calibration time when the data length, measured by LiDAR input size, is different. All the results are shown in Fig. 9, which suggests that our method has high computation efficiency. Moreover, as the input data amount increases, the processing time of our approach grows slowly.

## D. Gravity and Bias Initialization Evaluation

Our method is able to calibrate gyroscope bias, accelerometer bias and gravity vector, which can be used as highquality initial states for real-time LiDAR-inertial odometry system. To examine the accuracy of our method, we integrate it into FAST-LIO2 [1], which further refines all the states, including bias and gravity vector by tightly fusing the subsequent LiDAR data with IMU. We plot in Fig. 8 the difference between the states estimated online by FAST-LIO2 and their initial values supplied by our initialization. As can be seen, initial gyroscope bias, accelerometer bias and gravity vector are already very accurate and the subsequent refinement are small.

## V. CONCLUSION

This paper proposes a fast, robust, temporal and spatial initialization method for LiDAR-inertial system. An accurate and efficient coarse-to-fine temporal calibration method is proposed for unsynchronized LiDAR and IMU which is independent of any hardware synchronization setup. Also, we propose a fast, novel data association function to initialize LiDAR-inertial extrinsic transformation, gravity vector, as well as the bias of gyroscope and accelerometor. Various experiments show the consistency, robustness and high quality of our initialization method. Moreover, experiments using multiple types of LiDAR demonstrate the applicability to LiDARs with different scanning patterns.

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/c79ad39e60a6be50da2de7aea3a50894a43ca93d7e55a1846aed36c461938caf.jpg)  
Fig. 8. Error of gyroscope bias, accelerometer bias and gravity vector refined by FAST-LIO2 and their initial values supplied by our initialization. Test dataset is collected by Livox Mid360 LiDAR and IMU inside Pixhawk flight controller.

![](images/2022_Robust_Real-time_LiDAR-inertial_Initialization/58c5941203d60d4c2db7e641b9c452a50ffa01e7ee5c2d34e9d2f0bc08f8d7e2.jpg)  
Fig. 9. Time consumption comparison. All test data are collected by Hesai PandarXT LiDAR with 10 Hz output. The blue area is where the initialization time is below the data collection time, so it can run in realtime.

## VI. ACKNOWLEDGEMENT

This work is supported in part by the University Grants Committee of Hong Kong General Research Fund under Project 17206421 and in part by DJI under the grant 200009538. The authors gratefully acknowledge Livox Technology for the equipment support during the whole work. The authors would like to thank Wei Xu, Yixi Cai, Jiajun Lv, Meng Li for the helpful discussions and supports.

## REFERENCES

[1] Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, and Fu Zhang. Fast-lio2: Fast direct lidar-inertial odometry. IEEE Transactions on Robotics, 2022.

[2] Jiarong Lin and Fu Zhang. Loam livox: A fast, robust, high-precision lidar odometry and mapping package for lidars of small fov. In 2020 IEEE International Conference on Robotics and Automation (ICRA), pages 3126–3131. IEEE, 2020.

[3] Zheng Liu and Fu Zhang. Balm: Bundle adjustment for lidar mapping. IEEE Robotics and Automation Letters, 6(2):3184–3191, 2021.

[4] Yunfan Ren, Fangcheng Zhu, Wenyi Liu, Zhepei Wang, Yi Lin, Fei Gao, and Fu Zhang. Bubble planner: Planning high-speed smooth quadrotor trajectories using receding corridors. arXiv preprint arXiv:2202.12177, 2022.

[5] Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza. On-manifold preintegration for real-time visual–inertial odometry. IEEE Transactions on Robotics, 33(1):1–21, 2016.

[6] Michael Bloesch, Michael Burri, Sammy Omari, Marco Hutter, and Roland Siegwart. Iterated extended kalman filter based visual-inertial odometry using direct photometric feedback. The International Journal of Robotics Research, 36(10):1053–1072, 2017.

[7] Tixiao Shan, Brendan Englot, Drew Meyers, Wei Wang, Carlo Ratti, and Daniela Rus. Lio-sam: Tightly-coupled lidar inertial odometry via smoothing and mapping. In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 5135–5142. IEEE, 2020.

[8] Kailai Li, Meng Li, and Uwe D Hanebeck. Towards high-performance solid-state-lidar-inertial odometry and mapping. IEEE Robotics and Automation Letters, 6(3):5167–5174, 2021.

[9] Jiarong Lin, Chunran Zheng, Wei Xu, and Fu Zhang. R<sup>2</sup>live: A robust, real-time, lidar-inertial-visual tightly-coupled state estimator and mapping. IEEE Robotics and Automation Letters, 6(4):7469–7476, 2021.

[10] Jiarong Lin and Fu Zhang. R<sup>3</sup>live: A robust, real-time, rgb-colored, lidar-inertial-visual tightly-coupled state estimation and mapping package. In 2022 International Conference on Robotics and Automation (ICRA), pages 10672–10678. IEEE, 2022.

[11] Raul Mur-Artal and Juan D Tard ´ os. Visual-inertial monocular slam´ with map reuse. IEEE Robotics and Automation Letters, 2(2):796–803, 2017.

[12] Tong Qin and Shaojie Shen. Robust initialization of monocular visualinertial estimation on aerial robots. In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 4225– 4232. IEEE, 2017.

[13] Weibo Huang and Hong Liu. Online initialization and automatic camera-imu extrinsic calibration for monocular visual-inertial slam. In 2018 IEEE International Conference on Robotics and Automation (ICRA), pages 5182–5189. IEEE, 2018.

[14] Jiajun Lv, Jinhong Xu, Kewei Hu, Yong Liu, and Xingxing Zuo. Targetless calibration of lidar-imu system based on continuous-time batch estimation. In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 9968–9975. IEEE, 2020.

[15] Subodh Mishra, Gaurav Pandey, and Srikanth Saripalli. Targetfree extrinsic calibration of a 3d-lidar and an imu. In 2021 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI), pages 1–7. IEEE, 2021.

[16] Elmar Mair, Michael Fleps, Michael Suppa, and Darius Burschka. Spatio-temporal initialization for imu to camera registration. In 2011 IEEE International Conference on Robotics and Biomimetics, pages 557–564. IEEE, 2011.

[17] Tong Qin and Shaojie Shen. Online temporal calibration for monocular visual-inertial systems. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 3662–3669. IEEE, 2018.

[18] Yan Wang and Hongwei Ma. Online spatial and temporal initialization for a monocular visual-inertial-lidar system. IEEE Sensors Journal, 2021.

[19] Zachary Taylor and Juan Nieto. Motion-based calibration of multimodal sensor arrays. In 2015 IEEE International Conference on Robotics and Automation (ICRA), pages 4843–4850. IEEE, 2015.

[20] Chao Qin, Haoyang Ye, Christian E Pranata, Jun Han, Shuyang Zhang, and Ming Liu. Lins: A lidar-inertial state estimator for robust and efficient navigation. In 2020 IEEE International Conference on Robotics and Automation (ICRA), pages 8899–8906. IEEE, 2020.

[21] Wei Xu and Fu Zhang. Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter. IEEE Robotics and Automation Letters, 6(2):3317–3324, 2021.

[22] C. Hertzberg, R. Wagner, U. Frese, and L. Schroder. Hertzberg, ¨ christoph and wagner, rene and frese, udo and schr ´ oder, lutz. ¨ Information Fusion, 14(1):57–77, 2013.

[23] Dongjiao He, Wei Xu, and Fu Zhang. Embedding manifold structures into kalman filters. arXiv preprint arXiv:2010.05957, 2021.

[24] Fredrik Gustafsson. Determining the initial states in forward-backward filtering. IEEE Transactions on signal processing, 44(4):988–992, 1996.

[25] Dumitru Baleanu, Ozlem Defterli, and Om P. Agrawal. A central difference numerical scheme for fractional optimal control problems. Journal of Vibration and Control, 15(4):583–597, 2009.

[26] Wei Xu, Dongjiao He, Yixi Cai, and Fu Zhang. Robots’ state estimation and observability analysis based on statistical motion models. IEEE Transactions on Control Systems Technology, 2022.