# FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter

Wei Xu and Fu Zhang

Abstract—This letter presents a computationally efficient and robust LiDAR-inertial odometry framework. We fuse LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. To lower the computation load in the presence of a large number of measurements, we present a new formula to compute the Kalman gain. The new formula has computation load depending on the state dimension instead of the measurement dimension. The proposed method and its implementation are tested in various indoor and outdoor environments. In all tests, our method produces reliable navigation results in real-time: running on a quadrotor onboard computer, it fuses more than 1200 effective feature points in a scan and completes all iterations of an iEKF step within 25 ms. Our codes are open-sourced on Github.<sup>1</sup>

Index Terms—Aerial systems, localization, perception and autonomy, sensor fusion.

## I. INTRODUCTION

IMULTANEOUS localization and mapping (SLAM) is a fundamental prerequisite of mobile robots, such as unmanned aerial vehicles (UAVs). Visual (-inertial) odometry (VO), such as Stereo VO [1], [2] and Monocular VO [3], [4] are commonly used on mobile robots due to their lightweight and low-cost. Although providing rich RGB information, visual solutions lack direct depth measurements and require much computation resources to reconstruct the 3D environment for trajectory planning. Moreover, they are very sensitive to lighting conditions. Light detection and ranging (LiDAR) sensors could overcome all these difficulties but have been too costly (and bulky) for small-scale mobile robots.

Solid-state LiDARs recently emerge as main trends in LiDAR developments, such as those based on microelectro-mechanical-system (MEMS) scanning [5] and rotating prisms [6]. These LiDARs are very cost-effective (in a cost range similar to global shutter cameras), lightweight (can be carried by a small-scale UAV), and of high performance (producing active and direct 3D measurements of long-range and high-accuracy).

These features make such LiDARs viable for UAVs, especially industrial UAVs, which need to acquire accurate 3D maps of the environments (e.g., aerial mapping) or may operate in cluttered environments with severe illumination variations (e.g., post-disaster search and inspection).

Despite the great potentiality, solid-state LiDARs bring new challenges to SLAM: 1) the feature points in LiDAR measurements are usually the geometrical structures (e.g., edges and planes) in the environments. When the UAV is operating in cluttered environments where no strong features are present, the LiDAR-based solution easily degenerates. This problem is more obvious when the LiDAR has a small FoV. 2) Due to the high-resolution along the scanning direction, a LiDAR scan usually contains many feature points (e.g., a few thousand). While these feature points are not adequate to reliably determine the pose in case of degeneration, tightly fusing such a large number of feature points to IMU measurements requires tremendous computational resources that are not affordable by the UAV onboard computer. 3) Since the LiDAR samples point sequentially with a few laser/receiver pairs, laser points in a scan are always sampled at different times, resulting in motion distortion that will significantly degrade a scan registration [7]. The constant rotations of UAV propellers and motors also introduce significant noises to the IMU measurements.

To make the LiDAR navigation viable for small-scale mobile robots such as UAVs, we propose the FAST-LIO, a computationally efficient and robust LiDAR-inertial odometry package. More specifically, our contributions are as follows: 1) To cope with fast-motion, noisy or cluttered environments where degeneration occurs, we adopt a tightly-coupled iterated Kalman filter to fuse LiDAR feature points with IMU measurements. We propose a formal back-propagation process to compensate for the motion distortion; 2) To lower the computation load caused by a large number of LiDAR feature points, we propose a new formula for computing the Kalman gain and prove its equivalence to the conventional Kalman gain formula. The new formula has a computation complexity depending on the state dimension instead of the measurement dimension. 3) We implement our formulations into a fast and robust LiDAR-inertial odometry software package. The system is able to run on a small-scale quadrotor onboard computer. 4) We conduct experiments in various indoor and outdoor environments and with actual UAV flight tests (Fig. 1) to validate the system’s robustness when fast motion or intense vibration noise exists.

The remaining letter is organized as follows: In Section II, we discuss relevant research works. We give an overview of the complete system pipeline and the details of each key component in Section III. The experiments are presented in Section IV, followed by conclusions in Section V.

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/ee6b8c3cef8686c39e2ba923cfdbaea13f330337ea471eebd0c9d0606b309a19.jpg)  
Fig. 1. Our LiDAR-inertial navigation system runs on a Livox AVIA LiDAR<sup>2</sup> and a DJI Manifold 2-C onboard computer<sup>3</sup>, all on a customized small-scale quadrotor UAV (280 mm wheelbase). The RGB camera is not used in our algorithm, but only for visualization. Video is available at https://youtu.be/iYCY6T79oNU.

## II. RELATED WORKS

Existing works on LiDAR SLAM are extensive. Here we limit our review to the most relevant works: LiDAR-only odometry and mapping, loosely-coupled and tightly-coupled LiDAR-Inertial fusion methods.

## A. LiDAR Odometry and Mapping

Besl et al. [7] propose an iterated closest points (ICP) method for scan registration, which builds the basis for LiDAR odometry. ICP performs well for dense 3D scans. However, for sparse point clouds of LiDAR measurements, the exact point matching required by ICP rarely exists. To cope with this problem, Segal et al. [8] propose a generalized-ICP based on the point-to-plane distance. Then Zhang et al. [9] combine this ICP method with a point-to-edge distance and developed a LiDAR odometry and mapping (LOAM) framework. Thereafter, many variants of LOAM have been developed, such as LeGO-LOAM [10] and LOAM-Livox [11]. While these methods work well for structured environments and LiDARs of large FoV, they are very vulnerable to featureless environments or small FoV Li-DARs [11].

## B. Loosely-Coupled LiDAR-Inertial Odometry

IMU measurements are commonly used to mitigate the problem of LiDAR degeneration in featureless environments. Loosely-coupled LiDAR-inertial odometry (LIO) methods typically process the LiDAR and IMU measurements separately and fuse their results later. For example, IMU-aided LOAM [9] takes the pose integrated from IMU measurements as the initial estimate for LiDAR scan registration. Zhen et al. [12] fuse the IMU measurements and the Gaussian Particle Filter output of

LiDAR measurements using the error-state EKF. Balazadegan et al. [13] add the IMU-gravity model to estimate the 6-DOF ego-motion to aid the LiDAR scan registration. Zuo et al. [14] use a Multi-State Constraint Kalman Filter (MSCKF) to fuse the scan registration results with IMU and visual measurements. A common procedure of the loosely-coupled approach is obtaining a pose measurement by registering a new scan and then fusing the pose measurement with IMU measurements. The separation between scan registration and data fusion reduces the computation load. However, it ignores the correlation between the system’s other states (e.g., velocity) and the pose of the new scan. Moreover, in the case of featureless environments, the scan registration could degenerate in certain directions and causes unreliable fusion in later stages.

## C. Tightly-Coupled LiDAR-Inertial Odometry

Unlike the loosely-coupled methods, tightly-coupled LiDARinertial odometry methods typically fuse the raw feature points (instead of scan registration results) of LiDAR with IMU data. There are two main approaches to tightly-coupled LIO: optimization-based and filter-based. Geneva et al. [15] use a graph optimization with IMU pre-integration constrains [16] and plane constraints [17] from LiDAR feature points. Recently, Ye et al. [18] propose the LIOM package which uses a similar graph optimization but is based on edge and plane features. For filter-based methods, Bry [19] uses a Gaussian Particle Filter (GPF) to fuse the data of IMU and a planar 2D LiDAR. This method has also been used in the Boston Dynamics Atlas humanoid robot. Since the computation complexity of particle filter grows quickly with the number of feature points and the system dimension, Kalman filter and its variants are usually more preferred, such as extended Kalman filter [20], unscented Kalman filter [21], and iterated Kalman filter [22].

Our method falls into the tightly-coupled approach. We adopt an iterated extended Kalman filter similar to [22] to mitigate linearization errors. Kalman filter (and its variants) has a time complexity $\mathcal { O } ( m ^ { 2 } )$ where m is the measurement dimension [23], this may lead to remarkably high computation load when dealing with a large number of LiDAR measurements. Naive downsampling would reduce the number of measurements but at the cost of information loss. [22] reduces the number of measurements by extracting and fitting ground planes similar to [10]. This, however, does not apply to aerial applications where the ground plane may not always present.

## III. METHODOLOGY

## A. Framework Overview

This letter will use the notations shown in Table I. The overview of our workflow is shown in Fig. 2. The LiDAR inputs are fed into the feature extraction module to obtain planar and edge features. Then the extracted features and IMU measurements are fed into our state estimation module for state estimation at 10 Hz − 50 Hz. The estimated pose then registers the feature points to the global frame and merges them with the feature points map built so far. The updated map is finally used to register further new points in the next step.

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/70e890ad81d3c5790b832f6461b576dc551babdbc6327128a98aea1677f820ae.jpg)  
Fig. 2. System overview of FAST-LIO. (a): the overall pipeline; (b): the forward and backward propagation.

## B. System Description

1) - /  Operator: Let M be the manifold of dimension n in consideration $( \mathbf { e . g . } , \mathcal { M } = S O ( 3 ) )$ . Since manifolds are locally homeomorphic to $\mathbb { R } ^ { n }$ , we can establish a bijective mapping from a local neighborhood on M to its tangent space $\mathbb { R } ^ { n }$ via two encapsulation operators - and  [24]:

$$
\begin{array} { r l } & { \mathrel { \phantom { = } } \boxplus \colon \mathcal { M } \times \mathbb { R } ^ { n } { \to } \mathcal { M } ; \quad \ \stackrel { \ominus \ d \dag , \mathcal { M } \times \mathcal { M } \to \mathbb { R } ^ { n } } { \sqcup \dag , \mathcal { M } } } \\ & { \mathcal { M } = S O ( 3 ) : { \mathbf { R } } \boxplus \mathbf { r } { = } \mathbf { R } \mathrm { E x p } ( \mathbf { r } ) ; \quad \mathbf { R } _ { 1 } \boxplus \mathbf { R } _ { 2 } { = } \mathrm { L o g } ( \mathbf { R } _ { 2 } ^ { \mathrm { T } } \mathbf { R } _ { 1 } ) } \\ & { \mathcal { M } { = } \mathbb { R } ^ { n } : \quad \mathbf { a } \boxplus \mathbf { b } { = } \mathbf { a } { + } \mathbf { b } ; \quad \quad \mathbf { a } \boxplus \mathbf { b } { = } \mathbf { a } { - } \mathbf { b } } \end{array}
$$

where $\begin{array} { r } { \mathrm { E x p } ( \mathbf { r } ) { = } \mathbf { I } + \frac { \mathbf { r } } { \Vert \mathbf { r } \Vert } \sin ( \Vert \mathbf { r } \Vert ) { + } \frac { \mathbf { r } ^ { 2 } } { \Vert \mathbf { r } \Vert ^ { 2 } } ( 1 - \cos ( \Vert \mathbf { r } \Vert ) ) } \end{array}$ is the exponential map [24] and $\mathrm { L o g ( \cdot ) }$ is its inverse map. For a compound manifold $\mathcal { M } = S O ( 3 ) \times \mathbb { R } ^ { n }$ we have:

$$
{ \bf \Pi } _ { \mathbf { a } } ^ { \mathbf { \tilde { A } } } \equiv \left[ \mathbf { r } _ { \mathbf { b } } ^ { \mathbf { \tilde { r } } } \right] = \left[ \mathbf { R } \oplus \mathbf { r } _ { \mathbf { \tilde { c } } } \right] ; \left[ \mathbf { R } _ { 1 } \right] \bigm \equiv \left[ \mathbf { R } _ { 2 } \right] = \left[ \mathbf { R } _ { 1 } \ominus \mathbf { R } _ { 2 } \right]
$$

From the above definition, it is easy to verify that

$$
( \mathbf { x } \boxplus \mathbf { u } ) \boxplus \mathbf { x } = \mathbf { u } ; ~ \mathbf { x } \boxplus ( \mathbf { y } \boxplus \mathbf { x } ) = \mathbf { y } ; ~ \forall \mathbf { x } , \mathbf { y } \in \mathcal { M } , ~ \forall \mathbf { u } \in \mathbb { R } ^ { n } .
$$

2) Continuous Model: Assuming an IMU is rigidly attached to the LiDAR with a known extrinsic ${ ^ I } { \bf T } _ { L } = \bar { ( } ^ { I } \bar { \bf R } _ { L } , { ^ I } { \bf p } _ { L } )$ Taking the IMU frame (denoted as I) as the body frame of reference leads to a kinematic model:

$$
\begin{array} { r l } & { { { \bf \Lambda } ^ { G } } { \dot { \bf p } } _ { I } = { { \bf \Lambda } ^ { G } } { { \bf v } _ { I } } , { { \bf \Lambda } ^ { G } } { \dot { \bf v } } _ { I } = { { \bf \Lambda } ^ { G } } { \bf R } _ { I } \left( { { \bf a } _ { m } } - { { \bf b } _ { \bf a } } - { { \bf n } _ { \bf a } } \right) + { { \bf \Lambda } ^ { G } } { { \bf g } } , { { \bf \Lambda } ^ { G } } { \dot { \bf g } } = { \bf 0 } } \\ & { { { \bf \Lambda } ^ { G } } { \dot { \bf R } } _ { I } = { { \bf \Lambda } ^ { G } } { \bf R } _ { I } \left. { { \bf \omega } \omega } - { { \bf b } _ { \omega } } - { { \bf n } _ { \omega } } \right. _ { \Lambda } , { \dot { \bf b } _ { \omega } } = { \bf n } _ { \bf b \omega } , { \dot { \bf b } _ { \bf a } } = { \bf n } _ { \bf b a } } \end{array}\tag{1}
$$

where ${ { \bf \Lambda } } ^ { G } { \bf p } _ { I } , { { \bf \Lambda } } ^ { G } { \bf R } _ { I }$ are the position and attitude of IMU in the global frame $( \mathrm { i . e . }$ , the first IMU frame, denoted as $G ) , { \mathbf { \omega } } ^ { G } \mathbf { g }$ is the unknown gravity vector in the global frame, $\mathbf { a } _ { m }$ and $\omega _ { m }$ are IMU measurements, $\mathbf { n _ { a } }$ and $\mathbf { n } _ { \omega }$ are the white noise of IMU measurements, $\mathbf { b _ { a } }$ and $\mathbf { b } _ { \omega }$ are the IMU bias modelled as the random walk process with Gaussian noises $\mathbf { n _ { b a } }$ and $\mathbf { n } _ { \mathbf { b } \omega } ,$ and the notation $\lfloor \mathbf { a } \rfloor _ { \wedge }$ denotes the skew-symmetric matrix of vector $\mathbf { a } \in \mathbb { R } ^ { 3 }$ that maps the cross product operation.

3) Discrete Model: Based on the - operation defined above, we can discretize the continuous model in (1) at the IMU sampling period $\Delta t$ using a zero-order holder. The resultant discrete model is

$$
\mathbf { x } _ { i + 1 } = \mathbf { x } _ { i } \boxplus \left( \Delta t \mathbf { f } ( \mathbf { x } _ { i } , \mathbf { u } _ { i } , \mathbf { w } _ { i } ) \right)\tag{2}
$$

where i is the index of IMU measurements, the function $\mathbf { f } ,$ state $\mathbf { x } ,$ input u and noise w are defined as below:

$$
\begin{array} { r l } & { M = S O ( 3 ) \times { \mathbf B } ^ { 1 5 } , \ \mathrm { d } \mathrm { m } (  { \mathcal { M } } ) = 1 8 } \\ & { \quad \times = [ { \mathbf R } _ { T } ^ { T } \quad G _ { { \mathbf B } } ^ { T } \quad { \mathbf G } _ { T } ^ { T } \quad { \mathbf b } _ { T } ^ { T } \quad { \mathbf b } _ { \omega } ^ { T } \quad { \mathbf b } _ { R } ^ { T } \quad G _ { { \mathbf B } } ^ { T } \quad { \mathbf G } _ { { \mathbf B } } ^ { T } ] ^ { T } \in \mathcal { M } } \\ & { \quad = [ \omega _ { m } ^ { T } \quad { \mathbf a } _ { m } ^ { T } ] ^ { T } , \ w \stackrel { \cdot } { = } [ { \mathbf n } _ { \omega } ^ { T } \quad \mathbf { n } _ { \omega } ^ { T } \quad \mathbf { n } _ { \mathbf { b } \omega } ^ { T } \quad \mathbf { n } _ { \mathbf { b } \omega } ^ { T } \quad \mathbf { n } _ { \mathbf { b } \omega } ^ { T } ] ^ { T } } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad } \\ &  \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \mathrm { h } _ { \mathbf { b } \omega _ { i } } \quad \mathbf { n } _ { \mathbf { b } \omega } ^ { T } ) = (  \mathbf { G } _ { { \mathbf B } _ { i } } \quad ( \mathbf { n } _ { m _ { i } } - \mathbf { b } _ { \mathbf { a } _ { m } _ { i } } ) \quad + { \mathbf G } _ { { \mathbf B } _ { i } } | \quad \quad \quad \quad \quad \quad \end{array}\tag{3}
$$

4) Preprocessing of LiDAR Measurements: LiDAR measurements are point coordinates in its local body frame. Since the raw LiDAR points are sampled at a very high rate (e.g., 200 kHz), it is usually not possible to process each new point once being received. A more practical approach is to accumulate these points for a certain time and process them all at once. In FAST-LIO, the minimum accumulation interval is set to 20 ms, leading to up to 50 $H z$ full state estimation (i.e., odometry output) and map update as shown in Fig. 2 (a). Such an accumulated set of points is called a scan, and the time for processing it is denoted as $t _ { k }$ (see Fig. 2 (b)). From the raw points, we extract planar points with high local smoothness [9] and edge points with low local smoothness as in [11]. Assume the number of feature points is $m .$ , each is sampled at time $\rho _ { j } \in ( t _ { k - 1 } , t _ { k } ]$ and is denoted as ${ \cal L } _ { j } { \bf \sigma _ { p } } _ { f _ { j } }$ , where $L _ { j }$ is the LiDAR local frame at the time $\rho _ { j }$ . During a LiDAR scan, there are also multiple IMU measurements, each sampled at time $\tau _ { i } \in [ t _ { k - 1 } , t _ { k } ]$ with the respective state $\mathbf { x } _ { i }$ as in (2). Notice that the last LiDAR feature point is the end of a scan, i.e., $\rho _ { m } = t _ { k }$ , while the IMU measurements may not necessarily be aligned with the start or end of the scan.

## C. State Estimation

To estimate the states in the state formulation (2), we use an iterated extended Kalman filter. Moreover, we characterize the estimation covariance in the tangent space of the state estimate as in [24], [25]. Assume the optimal state estimate of the last LiDAR scan at $t _ { k - 1 } \mathrm { i s } \bar { \bf x } _ { k - 1 }$ with covariance matrix $\bar { \mathbf { P } } _ { k - 1 }$ . Then $\bar { \mathbf { P } } _ { k - 1 }$ represents the covariance of the random error state vector defined below:

$$
\widetilde { \mathbf { x } } _ { k - 1 } \doteq \mathbf { x } _ { k - 1 } \boxdot { \mathbf { u } } \bar { \mathbf { x } } _ { k - 1 } = \left[ \delta \pmb { \theta } ^ { T } { \mathbf { \Sigma } } ^ { G } \widetilde { \mathbf { p } } _ { I } ^ { T } { \mathbf { \Sigma } } ^ { G } \widetilde { \mathbf { v } } _ { I } ^ { T } \quad \widetilde { \mathbf { b } } _ { \omega } ^ { T } \widetilde { \mathbf { b } } _ { \mathbf { a } } ^ { T } { \mathbf { \Sigma } } ^ { G } \widetilde { \mathbf { g } } ^ { T } \right] ^ { T }
$$

where $\delta { \pmb \theta } = \mathrm { L o g } ( { \bf \mathrm { \bf { G } } } \bar { \bf R } _ { I } ^ { T G } { \bf R } _ { I } )$ is the attitude error and the rests are standard additive errors (i.e., the error in the estimate x¯ of a quantity x is $\widetilde { \mathbf { x } } = \mathbf { x } - \bar { \mathbf { x } } )$ . Intuitively, the attitude error $\delta \pmb { \theta }$ describes the (small) deviation between the true and the estimated attitude. The main advantage of this error definition is that it allows us to represent the attitude uncertainty by the $3 \times 3$ covariance matrix $\mathbb { E } \{ \delta \pmb { \theta } \delta \pmb { \theta } ^ { T } \}$ . Since the attitude has $3 ^ { \circ }$ of freedom (DOF), this is a minimal representation.

1) Forward Propagation: The forward propagation is performed once receiving an IMU input (see Fig. 2). More specifically, the state is propagated following (2) by setting the process noise $\mathbf { w } _ { i }$ to zero:

$$
\widehat { \mathbf { x } } _ { i + 1 } = \widehat { \mathbf { x } } _ { i } \boxplus \left( \Delta t \mathbf { f } ( \widehat { \mathbf { x } } _ { i } , \mathbf { u } _ { i } , \mathbf { 0 } ) \right) ; \widehat { \mathbf { x } } _ { 0 } = \bar { \mathbf { x } } _ { k - 1 } .\tag{4}
$$

To propagate the covariance, we use the error state dynamic model obtained below:

$$
\begin{array} { r l } & { \widetilde { \mathbf { x } } _ { i + 1 } = \mathbf { x } _ { i + 1 } \boxplus \widehat { \mathbf { x } } _ { i + 1 } } \\ & { \qquad = ( \mathbf { x } _ { i } \boxplus \Delta t \mathbf { f } \left( \mathbf { x } _ { i } , \mathbf { u } _ { i } , \mathbf { w } _ { i } \right) ) \boxplus ( \widehat { \mathbf { x } } _ { i } \boxplus \Delta t \mathbf { f } \left( \widehat { \mathbf { x } } _ { i } , \mathbf { u } _ { i } , \mathbf { 0 } \right) ) } \\ & { \qquad \stackrel { ( 2 3 ) } { \simeq } \mathbf { F } _ { \widetilde { \mathbf { x } } } \widetilde { \mathbf { x } } _ { i } + \mathbf { F } _ { \mathbf { w } } \mathbf { w } _ { i } . } \end{array}\tag{5}
$$

The matrix $\mathbf { F } _ { \widetilde { \mathbf { x } } }$ and $\mathbf { F _ { w } }$ in (5) is computed following the Appendix. V-A. The result is shown in (7) shown at the bottom of this page, where $\widehat { \pmb { \omega } } _ { i } = \pmb { \omega } _ { m _ { i } } - \widehat { \mathbf { b } } _ { \omega _ { i } } , \widehat { \mathbf { a } } _ { i } = \mathbf { a } _ { m _ { i } } - \widehat { \mathbf { b } } _ { \mathbf { a } }$ and ${ \mathbf { A } ( \mathbf { u } ) ^ { - 1 } }$ follows the same definition in [26] as below:

$$
\begin{array} { r l } & { \mathbf { A ( u ) } ^ { - 1 } = \mathbf { I } - \frac { 1 } { 2 } \lfloor \mathbf { u } \rfloor _ { \wedge } + \left( 1 - \alpha \left( \left\| \mathbf { u } \right\| \right) \right) \frac { \lfloor \mathbf { u } \rfloor _ { \wedge } ^ { 2 } } { \| \mathbf { u } \| ^ { 2 } } } \\ & { \quad \alpha \left( \mathrm { m } \right) = \frac { \mathrm { m } } { 2 } \mathrm { c o t } \left( \frac { \mathrm { m } } { 2 } \right) = \frac { \mathrm { m } } { 2 } \frac { \mathrm { c o s } \left( \mathrm { m } / 2 \right) } { \mathrm { s i n } \left( \mathrm { m } / 2 \right) } } \end{array}\tag{6}
$$

Denoting the covariance of white noises w as Q, then the propagated covariance $\widehat { \mathbf { P } } _ { i }$ can be computed iteratively following the below equation.

$$
\begin{array} { r } { \widehat { { \bf P } } _ { i + 1 } = { \bf F } _ { \tilde { \bf x } } \widehat { { \bf P } } _ { i } { \bf F } _ { \tilde { \bf x } } ^ { T } + { \bf F } _ { \bf w } { \bf Q } { \bf F } _ { \bf w } ^ { T } ; \ \widehat { { \bf P } } _ { 0 } = \bar { { \bf P } } _ { k - 1 } . } \end{array}\tag{8}
$$

The propagation continues until reaching the end time of a new scan at $t _ { k }$ where the propagated state and covariance are denoted as $\widehat { \mathbf { x } } _ { k } , \widehat { \mathbf { P } } _ { k }$ . Then $\hat { \mathbf { P } } _ { k }$ represents the covariance of the error between the ground-truth state $\mathbf { x } _ { k }$ and the state propagation $\widehat { \mathbf { x } } _ { k } \left( \mathrm { i . e . , } \mathbf { x } _ { k } \bigm \in \widehat { \mathbf { x } } _ { k } \right)$

2) Backward Propagation and Motion Compensation: When the points accumulation time interval is reached at time $t _ { k }$ , the new scan of feature points should be fused with the propagated state $\widehat { \mathbf { x } } _ { k }$ and covariance $\widehat { \mathbf { P } } _ { k }$ to produce an optimal state update. However, although the new scan is at time $t _ { k }$ , the feature points are measured at their respective sampling time $\rho _ { j } \leq t _ { k }$ (see Section. III-B4 and Fig. 2 (b)), causing a mismatch in the body frame of reference.

To compensate the relative motion (i.e., motion distortion) between time $\rho _ { j }$ and time $t _ { k } ,$ , we propagate (2) backward as $\check { \mathbf { x } } _ { j - 1 } = \check { \mathbf { x } } _ { j } \boxplus ( - \Delta t \mathbf { f } ( \check { \mathbf { x } } _ { j } , \mathbf { u } _ { j } , \mathbf { 0 } ) )$ ), starting from zero pose and rest states (e.g., velocity and bias) from $\widehat { \mathbf { x } } _ { k }$ . The backward propagation is performed at the frequency of feature point, which is usually much higher than the IMU rate. For all the feature points sampled between two IMU measurements, we use the left IMU measurement as the input in the back propagation. Furthermore, noticing that the last three block elements (corresponding to the gyro bias, accelerometer bias, and gravity) of $\mathbf { f } ( \mathbf { x } _ { j } , \mathbf { u } _ { j } , \mathbf { 0 } )$ (see (3)) are zeros, the back propagation can be reduced to:

$$
\begin{array} { r l } & { I _ { k } \check { \mathbf { p } } _ { I _ { j - 1 } } = { } ^ { I _ { k } } \check { \mathbf { p } } _ { I _ { j } } - { } ^ { I _ { k } } \check { \mathbf { v } } _ { I _ { j } } \Delta t , \qquad \mathrm { s . f . ~ } ^ { I _ { k } } \check { \mathbf { p } } _ { I _ { m } } = \mathbf { 0 } ; } \\ & { I _ { k } \check { \mathbf { v } } _ { I _ { j - 1 } } = { } ^ { I _ { k } } \check { \mathbf { v } } _ { I _ { j } } - { } ^ { I _ { k } } \check { \mathbf { R } } _ { I _ { j } } \big ( \mathbf { a } _ { m _ { i - 1 } } - \widehat { \mathbf { b } } _ { \mathbf { a } _ { k } } \big ) \Delta t - { } ^ { I _ { k } } \widehat { \mathbf { g } } _ { k } \Delta t , } \\ & { \qquad \mathrm { s . f . ~ } ^ { I _ { k } } \check { \mathbf { v } } _ { I _ { m } } = { } ^ { G } \widehat { \mathbf { R } } _ { I _ { k } } ^ { T } { } ^ { G } \widehat { \mathbf { v } } _ { I _ { k } } , { } ^ { I _ { k } } \widehat { \mathbf { g } } _ { k } = { } ^ { G } \widehat { \mathbf { R } } _ { I _ { k } } ^ { T } { } \widehat { \mathbf { g } } _ { k } ; } \\ & { I _ { k } \check { \mathbf { R } } _ { I _ { j - 1 } } = { } ^ { I _ { k } } \check { \mathbf { R } } _ { I _ { j } } \mathrm { E x p } ( ( \widehat { \mathbf { b } } _ { \omega _ { k } } - \omega _ { m _ { i - 1 } } ) \Delta t ) , \mathrm { ~ s . f . ~ } ^ { I _ { k } } \mathbf { R } _ { I _ { m } } = \mathbf { I } , } \end{array}\tag{9}
$$

where $\rho _ { j - 1 } \in [ \tau _ { i - 1 } , \tau _ { i } )$ and s.f. means “starting from”.

The backward propagation will produce a relative pose between time $\rho _ { j }$ and the scan-end time $\begin{array} { r l } { t _ { k } \colon } & { { } ^ { I _ { k } } \check { \mathbf { T } } _ { I _ { j } } = } \end{array}$ $( { ^ { I _ { k } } \check { \mathbf { R } } _ { I _ { j } } } , { ^ { I _ { k } } \check { \mathbf { p } } _ { I _ { j } } } )$ . This relative pose enables us to project the local measurement ${ L _ { j } } _ { \mathbf { p } _ { f _ { j } } }$ to scan-end measurement $\bar { L _ { k } } _ { \bar { \mathbf { p } _ { f _ { j } } } }$ as follows (see Fig. 2):

$$
{ } ^ { L _ { k } } { \bf p } _ { f _ { j } } = { } ^ { I } { \bf T } _ { L } ^ { - 1 I _ { k } } { \bf \check { T } } _ { I _ { j } } { } ^ { I } { \bf T } _ { L } { } ^ { L _ { j } } { \bf p } _ { f _ { j } } ,\tag{10}
$$

where ${ { I } _ { { { \mathbf { T } } _ { L } } } }$ is the known extrinsic (see Section. III-B2). Then the projected point $\boldsymbol { L } _ { k }  _ { \mathbf { p } _ { f _ { j } } }$ is used to construct a residual in the following section.

3) Residual Computation: With the motion compensation in (10), we can view the scan of feature points $\{ { ^ { L _ { k } } } _ { \mathbf { p } _ { f _ { j } } } \}$ all sampled at the same time $t _ { k }$ and use it to construct the residual. Assume the current iteration of the iterated Kalman filter is $\kappa ,$ and the corresponding state estimate is $\widehat { \mathbf { x } } _ { k } ^ { \kappa }$ . When $\boldsymbol { \kappa } = 0 , \widehat { \mathbf { x } } _ { k } ^ { \kappa } = \widehat { \mathbf { x } } _ { k }$ , the

$$
\mathbf { F } _ { \widetilde { \mathbf { x } } } = [ \begin{array} { c c c c c c c } { \mathrm { E x p } ( - \widetilde { \omega } _ { i } \Delta t ) } & { \mathbf { 0 } } & { \mathbf { 0 } } & { - \mathbf { A } ( \widetilde { \omega } _ { i } \Delta t ) ^ { T } \Delta t } & { \mathbf { 0 } } & { \mathbf { 0 } } \\ { \mathbf { 0 } } & { \mathbf { I } \ \mathbf { L } \Delta t } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } \\ { - ^ { G } \widehat { \mathbf { R } } _ { I _ { i } }  \widehat { \mathbf { a } } _ { i }  _ { , N } \Delta t } & { \mathbf { 0 } } & { \mathbf { I } } & { \mathbf { 0 } } & { - ^ { G } \widehat { \mathbf { R } } _ { I _ { i } } \Delta t \mathbf { \Delta } \mathbf { I } \Delta t } \\ { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { I } } & { \mathbf { 0 } } & { \mathbf { 0 } } \\ { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { I } } & { \mathbf { 0 } } \\ { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { I } } \end{array} ] , \mathbf { F } _ { \mathbf { w } } = [ \begin{array} { c c c c c c } { - \mathbf { A } ( \widetilde { \omega } _ { i } \Delta t ) ^ { T } \Delta t } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } \\ { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { 0 } } \\ { \mathbf { 0 } } & { - ^ { G } \widehat { \mathbf { R } } _ { I _ { i } } \Delta t } & { \mathbf { 0 } } & { \mathbf { 0 } } \\ { \mathbf { 0 } } & { \mathbf { 0 } } & { \mathbf { L } \Delta t } &  \mathbf { 0 }  \end{array}\tag{7}
$$

predicted state from the propagation in (4). Then, the feature points $\{ { ^ { L _ { k } } } _ { \mathbf { p } _ { f _ { j } } } \}$ can be transformed to the global frame as below:

$$
{ } ^ { G } \widehat { \mathbf { p } } _ { f _ { j } } ^ { \kappa } = { } ^ { G } \widehat { \mathbf { T } } _ { I _ { k } } ^ { \kappa } { } ^ { I } \mathbf { T } _ { L } { } ^ { L _ { k } } \mathbf { p } _ { f _ { j } } ; \ j = 1 , \ldots , m .\tag{11}
$$

For each LiDAR feature point, the closest plane or edge defined by its nearby feature points in the map is assumed to be where the point truly belongs to. That is, the residual is defined as the distance between the feature point’s estimated global frame coordinate $G _ { \widehat { \mathbf { p } } _ { f _ { \ j } } } \kappa _ { }$ and the nearest plane (or edge) in the map. Denoting $\mathbf { u } _ { j }$ the normal vector (or edge orientation) of the corresponding plane (or edge), on which lying a point ${ } ^ { G } { \bf q } _ { j }$ then the residual $\mathbf { z } _ { j } ^ { \kappa }$ is computed as:

$$
\mathbf { z } _ { j } ^ { \kappa } = \mathbf { G } _ { j } \left( { \cal G } _ { \widehat { \mathbf { p } } _ { f _ { j } } ^ { \kappa } } - { \cal G } _ { \mathbf { q } _ { j } } \right)\tag{12}
$$

where $\mathbf { G } _ { j } = \mathbf { u } _ { j } ^ { T }$ for planar features and $\mathbf { G } _ { j } = \lfloor \mathbf { u } _ { j } \rfloor _ { \wedge }$ for edge features. The computation of the $\mathbf { u } _ { j }$ and the search of nearby points in the map, which define the corresponding plane or edge, is achieved by building a KD-tree of the points in the most recent map [11]. Moreover, we only consider residuals whose norm is below certain threshold (e.g., 0.5m). Residuals exceeding this threshold are either outliers or newly observed points.

4) Iterated State Update: To fuse the residual $\mathbf { z } _ { j } ^ { \kappa }$ computed in (12) with the state prediction $\widehat { \mathbf { x } } _ { k }$ and covariance $\widehat { \mathbf { P } } _ { k }$ propagated from the IMU data, we need to linearize the measurement model that relates the residual $\mathbf { z } _ { j } ^ { \kappa }$ to the ground-truth state $\mathbf { x } _ { k }$ and measurement noise. The measurement noise originates from the LiDAR ranging and beam-directing noise ${ L } _ { { ^ j } { \bf n } _ { f _ { j } } }$ when measuring the point $\bar { L } _ { j } \bar { \bf p } _ { f _ { j } }$ . Removing this noise from the point measurement ${ L } _ { j }  { \bf { \sigma } } _ { \mathbf { p } _ { f _ { j } } }$ leads to the true point location

$$
{ \bf \sp { L } } _ { j } { \bf p } _ { f _ { j } } ^ { \mathrm { g t } } = { \bf \sp { L } } _ { j } { \bf p } _ { f _ { j } } - { \bf \sp { L } } _ { j } { \bf n } _ { f _ { j } } .\tag{13}
$$

This true point, after projecting to the frame $L _ { k }$ via (10) and then to the global frame with the ground-truth state $\mathbf { x } _ { k }$ (i.e, pose), should lie exactly on the plane (or edge) in the map. That is, plugging (13) into (10), then into (11), and further into (12) should result in zero. i.e.,

$$
\mathbf { 0 } = \mathbf { h } _ { j } \left( \mathbf { x } _ { k } , \mathbf { \Lambda } _ { } ^ { L _ { j } } \mathbf { n } _ { f _ { j } } \right) = \mathbf { G } _ { j } \bigg ( { } ^ { G } \mathbf { T } _ { I _ { k } } { } ^ { I _ { k } } \widehat { \mathbf { T } } _ { I _ { j } } { } ^ { I } \mathbf { T } _ { L } \big ( { } ^ { L _ { j } } \mathbf { p } _ { f _ { j } } - { } ^ { L _ { j } } \mathbf { n } _ { f _ { j } } \big ) - { } ^ { G } \mathbf { q } _ { j } \bigg )
$$

Approximating the above equation by its first order approximation made at $\widehat { \mathbf { x } } _ { k } ^ { \kappa }$ leads to

$$
\begin{array} { r l } & { \mathbf { 0 } = \mathbf { h } _ { j } \left( \mathbf { x } _ { k } , { \mathbf { \xi } } ^ { L _ { j } } \mathbf { n } _ { f _ { j } } \right) \simeq \mathbf { h } _ { j } \left( \widehat { \mathbf { x } } _ { k } ^ { \kappa } , \mathbf { 0 } \right) + \mathbf { H } _ { j } ^ { \kappa } \widetilde { \mathbf { x } } _ { k } ^ { \kappa } + \mathbf { v } _ { j } } \\ & { \quad = \mathbf { z } _ { j } ^ { \kappa } + \mathbf { H } _ { j } ^ { \kappa } \widetilde { \mathbf { x } } _ { k } ^ { \kappa } + \mathbf { v } _ { j } } \end{array}\tag{14}
$$

where $\widetilde { \mathbf { x } } _ { k } ^ { \kappa } = \mathbf { x } _ { k } \boxminus \widehat { \mathbf { x } } _ { k } ^ { \kappa }$ (or equivalently $\mathbf { x } _ { k } = \widehat { \mathbf { x } } _ { k } ^ { \kappa } \mathbb { E } \widetilde { \mathbf { x } } _ { k } ^ { \kappa } )$ , H<sup>κ</sup> is the Jacobin matrix of $\mathbf { h } _ { j } ( \widehat { \mathbf { x } } _ { k } ^ { \kappa } \boxed { \mathbb { H } } \widetilde { \mathbf { x } } _ { k } ^ { \kappa } , { \cal L } _ { j } \mathbf { n } _ { f _ { j } } )$ with respect to $\widetilde { \mathbf { x } } _ { k } ^ { \kappa }$ evaluated at zero, and $\mathbf { v } _ { j } \in \mathcal { N } ( \mathbf { 0 } , \mathbf { R } _ { j } )$ comes from the raw measurement noise $L _ { j } { \bf \Pi } _ { \mathbf { n } _ { f _ { i } } }$

Notice that the prior distribution of $\mathbf { x } _ { k }$ obtained from the forward propagation in Section. III-C1 is for

$$
\begin{array} { r } { { \bf x } _ { k } \boxminus \widehat { \bf x } _ { k } = ( \widehat { \bf x } _ { k } ^ { \kappa } \boxplus \widetilde { \bf x } _ { k } ^ { \kappa } ) \boxplus \widehat { \bf x } _ { k } = \widehat { \bf x } _ { k } ^ { \kappa } \boxplus \widehat { \bf x } _ { k } + { \bf J } ^ { \kappa } \widetilde { \bf x } _ { k } ^ { \kappa } } \end{array}\tag{15}
$$

where $\mathbf { J } ^ { \kappa }$ is the partial differentiation of $\left( \widehat { \mathbf { x } } _ { k } ^ { \kappa } \boxplus \widetilde { \mathbf { x } } _ { k } ^ { \kappa } \right) \boxminus \widehat { \mathbf { x } } _ { k }$ with respect to $\widetilde { \mathbf { x } } _ { k } ^ { \kappa }$ evaluated at zero:

$$
\mathbf { J } ^ { \kappa } = \left[ \begin{array} { c c } { \mathbf { A } \left( ^ { G } \widehat { \mathbf { R } } _ { I _ { k } } ^ { \kappa } \boxplus ^ { G } \widehat { \mathbf { R } } _ { I _ { k } } \right) ^ { - T } } & { \mathbf { 0 } _ { 3 \times 1 5 } } \\ { \mathbf { 0 } _ { 1 5 \times 3 } } & { \mathbf { I } _ { 1 5 \times 1 5 } } \end{array} \right]\tag{16}
$$

where ${ \mathbf { A } ( \cdot ) ^ { - 1 } }$ is defined in (6). For the first iteration (i.e., the case of extended Kalman filter), $\widehat { \mathbf { x } } _ { k } ^ { \kappa } = \widehat { \mathbf { x } } _ { k }$ , then $\mathbf { J } ^ { \kappa } = \mathbf { I }$

Combining the prior in (15) with the posteriori distribution from (14) yields the maximum a-posteriori estimate (MAP):

$$
\operatorname* { m i n } _ { \widetilde { \mathbf { x } } _ { k } ^ { \kappa } } \left( \| \mathbf { x } _ { k } \boxplus \widehat { \mathbf { x } } _ { k } \| _ { \widehat { \mathbf { P } } _ { k } ^ { - 1 } } ^ { 2 } + \sum _ { j = 1 } ^ { m } \| \mathbf { z } _ { j } ^ { \kappa } + \mathbf { H } _ { j } ^ { \kappa } \widetilde { \mathbf { x } } _ { k } ^ { \kappa } \| _ { \mathbf { R } _ { j } ^ { - 1 } } ^ { 2 } \right)\tag{17}
$$

where $\| \mathbf { x } \| _ { \mathbf { M } } ^ { 2 } = \mathbf { x } ^ { T } \mathbf { M } \mathbf { x }$ . Substituting the linearization of the prior in (15) into (17) and optimizing the resultant quadratic cost leads to the standard iterated Kalman filter [22], which can be computed below (to simplify the notation, let $\mathbf { H } { = } [ \mathbf { H } _ { 1 } ^ { \kappa ^ { T } } , \ldots , \mathbf { H } _ { m } ^ { \bar { \kappa } ^ { T } } ] ^ { T }$ $\mathbf { R } { = } \mathrm { d i a g } ( \mathbf { R } _ { 1 } , \cdot \cdot \cdot \mathbf { R } _ { m } )$ , P<sub>=</sub> $( \mathbf { J } ^ { \kappa } ) ^ { - 1 } \widehat { \mathbf { P } } _ { k } ( \mathbf { J } ^ { \kappa } ) ^ { - T }$ , and $\mathbf { z } _ { k } ^ { \kappa } = [ \mathbf { z } _ { 1 } ^ { \kappa ^ { T } } , \ldots , \mathbf { z } _ { m } ^ { \kappa ^ { T } } ] ^ { T } )$

$$
\begin{array} { r } { \mathbf { K } = \mathbf { P H } ^ { T } ( \mathbf { H } \mathbf { P H } ^ { T } + \mathbf { R } ) ^ { - 1 } , \phantom { x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x } } \\ { \widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 } = \widehat { \mathbf { x } } _ { k } ^ { \kappa } \mathbb { H } \bigl ( - \mathbf { K } \mathbf { z } _ { k } ^ { \kappa } - ( \mathbf { I } - \mathbf { K H } ) ( \mathbf { J } ^ { \kappa } ) ^ { - 1 } ( \widehat { \mathbf { x } } _ { k } ^ { \kappa } \boxplus \widehat { \mathbf { x } } _ { k } ) \bigr ) . } \end{array}\tag{18}
$$

The updated estimate $\widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 }$ is then used to compute the residual in Section. III-C3 and repeat the process until convergence (i.e., $\| \widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 } \ominus \widehat { \mathbf { x } } _ { k } ^ { \kappa } \| < \epsilon )$ . After convergence, the optimal state estimation and covariance is:

$$
\bar { \mathbf { x } } _ { k } = \widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 } , \bar { \mathbf { P } } _ { k } = \left( \mathbf { I } - \mathbf { K } \mathbf { H } \right) \mathbf { P }\tag{19}
$$

A problem with the commonly used Kalman gain form in (18) is that it requires to invert the matrix ${ \bf H P H } ^ { T } \bar { + }$ R which is in the dimension of the measurements. In practice, the number of LiDAR feature points are very large in number, inverting a matrix of this size is prohibitive. As such, existing works [22], [27] only use a small number of measurements. In this letter, we show that this limitation can be avoided. The intuition originates from (17) where the cost function is over the state, hence the solution should be calculated with complexity depending on the state dimension. In fact, if directly solving (17), we can obtain the same solution in (18) but with a new form of Kalman gain shown below:

$$
{ \bf K } { = } \big ( { \bf H } ^ { T } { \bf R } ^ { - 1 } { \bf H } { + } { \bf P } ^ { - 1 } \big ) ^ { - 1 } { \bf H } ^ { T } { \bf R } ^ { - 1 } .\tag{20}
$$

We prove in Appendix B that the two forms of Kalman gains are indeed equivalent based on the matrix inverse lemma [28]. Since the LiDAR measurements are independent, the covariance matrix R is (block) diagonal and hence the new formula only requires to invert two matrices both in the dimension of state instead of measurements. The new formula greatly saves the computation as the state dimension is usually much lower than measurements in LIO (e.g., more than 1000 effective feature points in a scan for 10 Hz scan rate while the state dimension is only 18).

Algorithm 1: State Estimation.   
Input:Last optimal estimation $\bar { \bf x } _ { k - 1 }$ and $\bar { \mathbf { P } } _ { k - 1 } ,$   
IMU inputs $( \mathbf { a } _ { m } , \omega _ { m } )$ in current scan;   
LiDAR feature points ${ L _ { j } } _ { \mathbf { p } _ { f _ { j } } }$ in current scan. Forward   
propagation to obtain state prediction $\widehat { \mathbf { x } } _ { k }$ via (4) and   
covariance prediction $\widehat { \mathbf { P } } _ { k }$ via (8);   
Backward propagation to obtain ${ L } _ { k }  _ { \mathbf { p } _ { f _ { j } } }$ via (9), (10);   
$\kappa = - 1 , \widehat { \mathbf { x } } _ { k } ^ { \kappa = 0 } = \widehat { \mathbf { x } } _ { k } ;$   
repeat $\| \widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 } \boxdot { \mathbf { x } } \widehat { \mathbf { x } } _ { k } ^ { \kappa } \| < \epsilon \kappa = \kappa + 1 ;$   
Compute $\mathbf { J } ^ { \kappa }$ via (16) and $\mathbf { P } = ( \mathbf { J } ^ { \kappa } ) ^ { - 1 } \widehat { \mathbf { P } } _ { k } ( \mathbf { J } ^ { \kappa } ) ^ { - T } ;$   
Compute residual $\mathbf { z } _ { j } ^ { \kappa }$ (12) and Jocobin H<sup>κ</sup> (14);   
Compute the state update $\widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 }$ via (18) with the Kalman   
gain K from $( 2 0 ) ;$   
$\bar { \mathbf { x } } _ { k } = \widehat { \mathbf { x } } _ { k } ^ { \kappa + 1 } ; \bar { \mathbf { P } } _ { k } = ( \mathbf { I } - \mathbf { K H } ) \mathbf { P } .$   
Output:Current optimal estimation $\bar { \bf x } _ { k }$ and $\bar { \mathbf { P } } _ { k } .$

5) The Algorithm: Our state estimation is summarized in Algorithm 1.

## D. Map Update

With the state update $\bar { \mathbf { x } } _ { k }$ (hence ${ } ^ { G } \bar { \mathbf { T } } _ { I _ { k } } = ( { } ^ { G } \bar { \mathbf { R } } _ { I _ { k } } , { } ^ { G } \bar { \mathbf { p } } _ { I _ { k } } ) )$ each feature point $( ^ { L _ { k } } \mathbf { p } _ { f _ { j } } )$ projected to the body frame $L _ { k }$ (see (10)) is then transformed to the global frame via:

$$
{ } ^ { G } \bar { \mathbf { p } } _ { f _ { j } } = { } ^ { G } \bar { \mathbf { T } } _ { I _ { k } } { } ^ { I } \mathbf { T } _ { L } { } ^ { L _ { k } } \mathbf { p } _ { f _ { j } } ; \ j = 1 , \ldots , m .\tag{21}
$$

These features points are finally appended to the existing map containing feature points from all previous steps.

## E. Initialization

To obtain a good initial estimate of the system state (e.g., gravity vector $\breve { G } _ { \mathbf { g } }$ , bias, and noise covariance) so to speedup the state estimator, initialization is required. In FAST-LIO, the initialization is simple: keeping the LiDAR static for several seconds (2 seconds for all the experiments in this letter), the collected data is then used to initialize the IMU bias and the gravity vector. If non-repetitive scanning is supported by the LiDAR (e.g., Livox AVIA), keeping static also allows the LiDAR to capture an initial high-resolution map that is beneficial for the subsequent navigation.

## IV. EXPERIMENT RESULTS

## A. Computational Complexity Experiments

In order to validate the computational efficiency of the proposed new formula for computing Kalman gains. We intentionally replace the computation of Kalman gains with the old formula in our system and compare their computation time under the same system pipeline and number of feature points. The results are shown in Table. II. It is obvious that the complexity of the new formula is much lower than the old one.

TABLE I  
SOME IMPORTANT NOTATIONS
<table><tr><td>Symbols</td><td>Meaning</td></tr><tr><td> $t _ { k }$ </td><td>The scan-end time of the k-th LiDAR scan.</td></tr><tr><td> $\tau _ { i }$ </td><td>The i-th IMU sample time in a LiDAR scan.</td></tr><tr><td> $\rho _ { j }$ </td><td>The j-th feature point&#x27;s sample time in a LiDAR scan.</td></tr><tr><td> $\dot { I _ { i } } , I _ { j } , I _ { k }$ </td><td>The IMU body frame at the time  $\tau _ { i } , \rho _ { j }$  and  $t _ { k } .$ </td></tr><tr><td> $L _ { j } , \dot { L } _ { k }$ </td><td>The LiDAR body frame at the time  $\rho _ { j }$  and  $t _ { k } .$ </td></tr><tr><td> $\mathbf { x } , \widehat { \mathbf { x } } , \bar { \mathbf { x } }$ </td><td>The ground-true, propagated, and updated value of x.  $\textstyle { \overline { { \mathbf { x } } } } .$ </td></tr><tr><td> $\widetilde { \mathbf { x } }$ </td><td>The error between ground-true x and its estimation</td></tr><tr><td> $\widehat { \mathbf { x } } ^ { \kappa }$ </td><td>The κ-th update of x in the iterated Kalman filter.</td></tr><tr><td> $\mathbf { x } _ { i } , \mathbf { x } _ { j } , \mathbf { x } _ { k }$ </td><td>The vector (e.g.,state) x at time  $\tau _ { i } , \rho _ { j }$  and  $t _ { k } .$ </td></tr><tr><td> $\check { \mathbf { x } } _ { j }$ </td><td>Estimate of  $\mathbf { x } _ { j }$  relative to  $\mathbf { x } _ { k }$  in the back propagation.</td></tr></table>

TABLE II

THE RUNNING TIMES OF TWO KALMAN GAIN FORMULAS
<table><tr><td>Feature Num.</td><td>307</td><td>717</td><td>998</td><td>1243</td><td>1453</td><td>1802</td></tr><tr><td>Old Formula (ms)</td><td>7.1</td><td>23.4</td><td>109.3</td><td>251</td><td>1219</td><td>1621</td></tr><tr><td>New Formula (ms)</td><td>0.07</td><td>0.11</td><td>0.25</td><td>0.37</td><td>0.59</td><td>1.16</td></tr></table>

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/db4ca6734ae3159899b13245f9d9b64c77e88e0da72599a7f5af4b79296c1393.jpg)  
Fig. 3. During the flight experiment, the UAV is automatically flying in a circle path with 1.8 m radius and 1.4 m height. The circle path is conducted repeatedly for 4 times with different periods (6-10 s). The yaw command of the UAV maintains constant during the flight. In the end, the UAV is manually controlled to land at the take-off point, which enables us to measure the drift.

## B. UAV Flight Experiments

In order to validate the robustness and computational efficiency of FAST-LIO in actual mobile robots, we build a smallscale quadrotor that can carry a Livox Avia LiDAR with 70<sup>◦</sup> FoV and a DJI Manifold 2-C onboard computer with a 1.8 GHz quad-core Intel i7-8550 U CPU and 8 GB RAM, as shown in Fig. 1. The UAV has only 280 mm wheelbase, and the LiDAR is directly installed on the airframe. The LiDAR-inertial odometry is sent to the flight controller tracking a circle trajectory (Fig. 3). The actual flight experiments show that FAST-LIO can achieve real-time and stable odometry output and mapping in a maximum of 50 Hz for the indoor environment. The flight trajectory and mapping results of 50 $H z$ frame rate indoor experiment are shown in Fig. 3. The average number of effective feature points and running time is 270 and 6.7 ms, respectively, the drift is smaller than 0.3% (0.08 m drift over 32 m trajectory). The flight video can be found at https://youtu.be/iYCY6T79oNU.

## C. Indoor Experiments

Then we test FAST-LIO in a challenging indoor environment with large rotation speeds. In order to generate large rotation, the sensor suite is held on hands and undergoes quickly shaking. Fig. 4 shows the angular velocity and acceleration during the experiment. It is seen that the angular velocity often exceeds 100 deg/s. A state of the art implementation of LOAM on Livox LiDARs<sup>4</sup> [11] and LOAM with IMU<sup>5</sup> [9] are also tested as comparisons when the feature extraction are replaced with the one of FAST-LIO. The results show that FAST-LIO can output odometry faster and more stable than others, as shown in Fig. 5 and Table. III . It should be noted that the LOAM+IMU is a loosely-coupled method, hence results in inconsistent mapping. To further verify the mapping result, we perform a second experiment in the same environment but with a much slower motion. The map built by FAST-LIO is shown in the lower-right figure of Fig. 4. Since the two experiments have non-identical movements, it leads to slight visual differences at places occlusions occur. The rest mapping results are very close.

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/3cda5721e148c4a25fc7bb3bf5c74e716dd1d4bbbdcf4c96bc25941d1ad07c7b.jpg)

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/cb655b87d9fb5d0d52400f60fe2182bd0d65b48a1e6b785ae6103f0db080c5d8.jpg)  
Fig. 4. The angular velocity and acceleration in the indoor experiments.

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/15bae7af697d41cb7c31f8a8f002c3ec9193ebf90c6445a2a9838ee31cd3819e.jpg)

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/caed278d39ba45be2a44e8b7792e90a6483342dca5fa21faa9b6eb1b85053196.jpg)  
Fig. 5. The Mapping results of different LIO packages in an indoor environment with large rotation speed.

TABLE III  
COMPARISON OF PROCESSING TIME FOR A LIDAR SCAN AT 10 HZ
<table><tr><td>Packages</td><td>Num. of effective features</td><td>Running time</td></tr><tr><td>LOAM</td><td>1107</td><td>59 ms</td></tr><tr><td>LOAM+IMU</td><td>1107</td><td>44 ms</td></tr><tr><td>FAST-LIO</td><td>1430</td><td>23 ms</td></tr></table>

![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/a862fe2b015b682c91b9115ea5f1885077ce12d6916bf3b489a4d7976867ea03.jpg)

Fig. 6. Mapping results of the Main Building, University of Hong Kong. The LiDAR platform, the same one in Fig. 1, is handheld randomly walking to scan the building. In order to show the drift, the experiment are started and ended at the same place.  
![](images/2021_FAST-LIO__A_Fast__Robust_LiDAR-Inertial_Odometry_Package/7df8c9a1e3417d27243d9f5411e4aaaa14f12769204060e2baf0652aba281839.jpg)  
Fig. 7. Comparison between LINS [22] and FAST-LIO. The data is from [22] and is collected by a Velodyne VLP-16 LiDAR and an Xsens MTiG-710 IMU, the green straight line in the center is the odometry output.

## D. Outdoor Experiments

Here we show the performance of FAST-LIO in outdoor environments. Fig. 6 shows the mapping results (displaying all raw points) of the Main Building in the University of Hong Kong. The sensor suite is handheld during the data collection and returned to the starting position after traveling around 140m. The drift in this experiment is smaller than 0.05% (0.07 m drift over 140 m trajectory). The scan rate is set to 10 Hz in this experiment, and the average processing time of a scan is 25 ms with average 1497 effective feature points.

Further, we compare FAST-LIO with LINS<sup>6</sup> [22]. To make a fair comparison, we use the dataset from LINS [22], which is a seaport area data collected by a Velodyne VLP-16 and an Xsens MTiG-710 IMU.<sup>6</sup> The results show that the FAST-LIO can achieve better mapping accuracy (see Fig. 7) and only consumes 7.3 ms processing time in average while LINS takes 34.5 ms in average, both running at 10 Hz. It should be noted that since the EKF formula in LINS package has high computational complexity (see Section. III-C4)), it down samples the feature points to average 147 points in a scan (while 784 in a scan for FAST-LIO). This leads to degraded mapping accuracy for

LINS. The result in Fig. 7 shows all the feature points (before down-sample) of FAST-LIO and LINS. All the experiments are conducted on the DJI Manifold2 onboard computer.

## V. CONCLUSION

This letter proposed FAST-LIO, a computationally efficient and robust LiDAR-inertial odometry framework by a tightly-coupled iterated Kalman filter. We used the forward and backward propagation to predict the states and compensate for the motion in a LiDAR scan. Besides, we proved and implemented an equivalent formula that can achieve much lower complexity for the Kalman gain computation. FAST-LIO was tested in the UAV flight experiment, challenging indoor environment with large rotation speed and outdoor environment. In all tests, our method produced precise, real-time, and reliable navigation results.

## APPENDIX

## A. Computation of $\mathbf { F } _ { \widetilde { \mathbf { x } } }$ and $\mathbf { F _ { w } }$

Recall $\begin{array} { r } { \mathbf { x } _ { i } = \widehat { \mathbf { x } } _ { i } \boxplus \widetilde { \mathbf { x } } _ { i } . } \end{array}$ , denote $\mathbf { g } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { w } _ { i } ) = \mathbf { f } ( \mathbf { x } _ { i } , \mathbf { u } _ { i } , \mathbf { w } _ { i } ) \Delta t =$ $\mathbf { f } \left( \widehat { \mathbf { x } } _ { i } \right)$ - $\widetilde { \mathbf { x } } _ { i } , \mathbf { u } _ { i } , \mathbf { w } _ { i } ) \Delta t$ . Then the error state model (5) is rewriten as:

$$
\widetilde { \mathbf { x } } _ { i + 1 } = \underset { \mathbf { G } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { H } \widetilde { \mathbf { x } } _ { i } ) \mathbf { H } \mathbf { g } \left( \widetilde { \mathbf { x } } _ { i } , \mathbf { w } _ { i } \right) ) \big \boxtimes \left( \widehat { \mathbf { x } } _ { i } \boxplus \mathbf { g } \left( \mathbf { 0 } , \mathbf { 0 } \right) \right) } { \mathbf { G } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { g } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { w } _ { i } ) ) }\tag{22}
$$

Following the chain rule of partial differention, the matrix $\mathbf { F } _ { \widetilde { \mathbf { x } } }$ and $\mathbf { F _ { w } }$ in (5) are computed as below.

$$
\begin{array} { r } { \mathbf { F } _ { \widetilde { \mathbf { x } } } = \left. \left( \frac { \partial \mathbf { G } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { g } ( \mathbf { 0 } , \mathbf { 0 } ) ) } { \partial \widetilde { \mathbf { x } } _ { i } } + \frac { \partial \mathbf { G } ( \mathbf { 0 } , \mathbf { g } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { 0 } ) ) } { \partial \mathbf { g } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { 0 } ) } \frac { \partial \mathbf { g } ( \widetilde { \mathbf { x } } _ { i } , \mathbf { 0 } ) } { \partial \widetilde { \mathbf { x } } _ { i } } \right) \right| _ { \widetilde { \mathbf { x } } _ { i } = \mathbf { 0 } } } \end{array}
$$

$$
\begin{array} { r } { \mathbf { F _ { w } } = \left. \left( \frac { \partial \mathbf { G } ( \mathbf { 0 } , \mathbf { g } ( \mathbf { 0 } , \mathbf { w } _ { i } ) ) } { \partial \mathbf { g } ( \mathbf { 0 } , \mathbf { w } _ { i } ) } \frac { \partial \mathbf { g } ( \mathbf { 0 } , \mathbf { w } _ { i } ) } { \partial \mathbf { w } _ { i } } \right) \right| _ { \mathbf { w } _ { i } = \mathbf { 0 } } } \end{array}\tag{23}
$$

## B. Equivalent Kalman Gain Formula

Based on the matrix inverse lemma [28], we can get:

$$
\left( \mathbf { P } ^ { - 1 } + \mathbf { H } ^ { T } \mathbf { R } ^ { - 1 } \mathbf { H } \right) ^ { - 1 } = \mathbf { P } - \mathbf { P } \mathbf { H } ^ { T } \left( \mathbf { H } \mathbf { P } \mathbf { H } ^ { T } + \mathbf { R } \right) ^ { - 1 } \mathbf { H } \mathbf { P }
$$

Substituting above into (20), we can get:

$$
\begin{array} { r l } & { { \bf K } = \left( { { \bf H } ^ { T } { \bf R } ^ { - 1 } { \bf H } + { \bf P } ^ { - 1 } } \right) ^ { - 1 } { \bf H } ^ { T } { \bf R } ^ { - 1 } } \\ & { \quad = { \bf P } { \bf H } ^ { T } { \bf R } ^ { - 1 } - { \bf P } { \bf H } ^ { T } \left( { \bf H } { \bf P } { \bf H } ^ { T } + { \bf R } \right) ^ { - 1 } { \bf H } { \bf P } { \bf H } ^ { T } { \bf R } ^ { - 1 } } \end{array}
$$

Now note that HPH $\mathbf { R } ^ { - 1 } = ( \mathbf { H } \mathbf { P } \mathbf { H } ^ { T } + \mathbf { R } ) \mathbf { R } ^ { - 1 } - \mathbf { I }$ . Substituting it into above, we can get the standard Kalman gain formula in (18), as shown below.

$$
\begin{array} { r l } & { { \bf K } = { \bf P } { \bf H } ^ { T } { \bf R } ^ { - 1 } - { \bf P } { \bf H } ^ { T } { \bf R } ^ { - 1 } + { \bf P } { \bf H } ^ { T } \left( { \bf H } { \bf P } { \bf H } ^ { T } + { \bf R } \right) ^ { - 1 } } \\ & { \quad = { \bf P } { \bf H } ^ { T } \left( { \bf H } { \bf P } { \bf H } ^ { T } + { \bf R } \right) ^ { - 1 } . } \end{array}
$$

## REFERENCES

[1] K. Sun et al., “Robust stereo visual inertial odometry for fast autonomous flight,” IEEE Robot. Automat. Lett., vol. 3, no. 2, pp. 965–972, Apr. 2018.

[2] A. Howard, “Real-time stereo visual odometry for autonomous ground vehicles,” in Proc. IEEE/RSJInt. Conf. Intell. Robots Syst., 2008, pp. 3946– 3952.

[3] T. Qin, P. Li, and S. Shen, “VINS-MONO: A robust and versatile monocular visual-inertial state estimator,” IEEE Trans. Robot., vol. 34, no. 4, pp. 1004–1020, Aug. 2018.

[4] C. Forster, M. Pizzoli, and D. Scaramuzza, “SVO: Fast semi-direct monocular visual odometry,” in Proc. IEEE Int. Conf. Robot. Automat. , 2014, pp. 15–22.

[5] D. Wang, C. Watkins, and H. Xie, “Mems mirrors for lidar: A review,” Micromachines, vol. 11, no. 5, pp. 456–479, 2020.

[6] Z. Liu, F. Zhang, and X. Hong, “Low-cost retina-like robotic lidars based on incommensurable scanning,” IEEE/ASME Trans. Mechatronics, pp. 1–1, 2021, early access, doi: 10.1109/TMECH.2021.3058173.

[7] P. J. Besl and N. D. McKay, “Method for registration of 3D shapes,” in Sensor fusion IV: control paradigms and data structures. Int. Soc. Opt. Photonics, vol. 1611, 1992, pp. 586–606.

[8] A. Segal, D. Haehnel, and S. Thrun, “Generalized-ICP,” in Proc. Robot.: Sci. Syst., Seattle, WA, vol. 2, no. 4, p. 435, 2009.

[9] J. Zhang and S. Singh, “LOAM: Lidar odometry and mapping in real-time,” in Robot.: Sci. Syst., vol. 2, no. 9, 2014.

[10] T. Shan and B. Englot, “LEGO-LOAM: Lightweight and groundoptimized lidar odometry and mapping on variable terrain,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 4758–4765.

[11] J. Lin and F. Zhang, “Loam livox: A fast, robust, high-precision lidar odometry and mapping package for LiDARs of small FoV,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 3126–3131.

[12] W. Zhen, S. Zeng, and S. Soberer, “Robust localization and localizability estimation with a rotating laser scanner,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 6240–6245.

[13] Y. Balazadegan Sarvrood, S. Hosseinyalamdary, and Y. Gao, “Visual-lidar odometry aided by reduced IMU,” ISPRS Int. J. Geo-inf., vol. 5, no. 1, p. 3, 2016.

[14] X. Zuo, P. Geneva, W. Lee, Y. Liu, and G. Huang, “Lic-fusion: Lidarinertial-camera odometry,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 5848–5854.

[15] P. Geneva, K. Eckenhoff, Y. Yang, and G. Huang, “Lips: Lidar-inertial 3 d plane slam,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 123–130.

[16] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “On-manifold preintegration for real-time visual-inertial odometry,” IEEE Trans. Robot., vol. 33, no. 1, pp. 1–21, Feb. 2017.

[17] M. Hsiao, E. Westman, and M. Kaess, “Dense planar-inertial SLAM with structural constraints,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 6521–6528.

[18] H. Ye, Y. Chen, and M. Liu, “Tightly coupled 3 d lidar inertial odometry and mapping,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 3144–3150.

[19] A. Bry, A. Bachrach, and N. Roy, “State estimation for aggressive flight in gps-denied environments using onboard sensing,” in Proc. IEEE Int. Conf. Robot. Automat., 2012, pp. 1–8.

[20] J. A. Hesch, F. M. Mirzaei, G. L. Mariottini, and S. I. Roumeliotis, “A laser-aided inertial navigation system (l-ins) for human localization in unknown indoor environments,” in Proc. IEEE Int. Conf. Robot. Automat., 2010, pp. 5376–5382.

[21] Z. Cheng et al., “Practical phase unwrapping of interferometric fringes based on unscented Kalman filter technique,” Opt. Exp., vol. 23, no. 25, pp. 32337–32349, 2015.

[22] C. Qin, H. Ye, C. E. Pranata, J. Han, S. Zhang, and M. Liu, “Lins: A lidar-inertial state estimator for robust and efficient navigation,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 8899–8906.

[23] M. Raitoharju and R. Piché, “On computational complexity reduction methods for kalman filter extensions,” IEEE Aerosp. Electron. Syst. Mag., vol. 34, no. 10, pp. 2–19, Oct. 2019.

[24] C. Hertzberg, R. Wagner, U. Frese, and L. Schröder, “Integrating generic sensor fusion algorithms with sound state representations through encapsulation of manifolds,” Inf. Fusion, vol. 14, no. 1, pp. 57–77, 2013.

[25] W. Xu, D. He, Y. Cai, and F. Zhang, “Robots state estimation and observability analysis based on statistical motion models,” 2020, arXiv:2010.05957.

[26] F. Bullo and R. M. Murray, “Proportional derivative (PD) control on the euclidean group,” in Proc. Eur. Control Conf., vol. 2, 1995, pp. 1091–1097.

[27] M. Bloesch, M. Burri, S. Omari, M. Hutter, and R. Siegwart, “Iterated extended kalman filter based visual-inertial odometry using direct photometric feedback,” Int. J. Robot. Res., vol. 36, no. 10, pp. 1053–1072, 2017.

[28] N. J. Higham, Accuracy and Stability ofNumerical Algorithms. Philadelphia, PA, USA: SIAM, 2002.

Authorized licensed use limited to: Jiangnan University. Downloaded on September 16,2026 at 03:27:52 UTC from IEEE Xplore. Restrictions apply.