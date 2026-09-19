# DM-VIO: Delayed Marginalization Visual-Inertial Odometry

Lukas von Stumberg and Daniel Cremers

Abstract—We present DM-VIO, a monocular visual-inertial odometry system based on two novel techniques called delayed marginalization and pose graph bundle adjustment. DM-VIO performs photometric bundle adjustment with a dynamic weight for visual residuals. We adopt marginalization, which is a popular strategy to keep the update time constrained, but it cannot easily be reversed, and linearization points of connected variables have to be fixed. To overcome this we propose delayed marginalization: The idea is to maintain a second factor graph, where marginalization is delayed. This allows us to later readvance this delayed graph, yielding an updated marginalization prior with new and consistent linearization points. In addition, delayed marginalization enables us to inject IMU information into already marginalized states. This is the foundation of the proposed pose graph bundle adjustment, which we use for IMU initialization. In contrast to prior works on IMU initialization, it is able to capture the full photometric uncertainty, improving the scale estimation. In order to cope with initially unobservable scale, we continue to optimize scale and gravity direction in the main system after IMU initialization is complete. We evaluate our system on the EuRoC, TUM-VI, and 4Seasons datasets, which comprise flying drone, large-scale handheld, and automotive scenarios. Thanks to the proposed IMU initialization, our system exceeds the state of the art in visual-inertial odometry, even outperforming stereo-inertial methods while using only a single camera and IMU. The code will be published at http://vision.in.tum.de/dm-vio

Index Terms—Visual-Inertial SLAM, SLAM.

## I. INTRODUCTION

V <sup>ISUAL-(INERTIAL)</sup> <sup>odometry</sup> <sup>is</sup> <sup>an</sup> <sup>increasingly</sup> <sup>relevant</sup> task with applications in robotics, autonomous driving, and augmented reality. A combination of cameras and inertial measurement units (IMUs) for this task is a popular and sensible choice, as they are complementary sensors, resulting in a highly accurate and robust system [1]. In the minimal configuration of a single camera, the IMU can also be used to recover the metric scale. However, the scale is not always observable, the most common degenerate case being movement with a constant velocity [2]. Hence, initialization of such system can take arbitrarily long, depending on the trajectory. Even worse, when initialized prematurely the IMU can in fact worsen the performance. The difficulty of IMU initialization is why stereo-inertial methods have outperformed mono-inertial ones in the past.

![](images/2022_DM-VIO/b6d73378fea1dd5e751451ce354e02ed24fd82b01a68f2da48c4d55d2ac14bcc.jpg)  
Fig. 1. In this paper, we propose a novel method for monocular visualinertial odometry. It provides state-of-the-art performance on three different benchmarks. Here we show pointclouds and trajectories (red) for magistrale5, V203\_difficult, and neighbor\_2020-03-26\_13-32-55\_0.

Most prior systems [3]–[5] initially run visual-only odometry and an IMU initialization in parallel. Once finished, the visual-inertial system is started. This introduces a trade-off for the duration of the initialization period: It should be as short as possible, as no IMU information is used in the main system in the meantime. But when too short, the scale estimate will be inaccurate, leading to bad performance.

VI-DSO [6] instead initializes immediately with an arbitrary scale, and explicitly optimizes the scale in the main system. This yields highly accurate scale estimates, but it can significantly increase the time until the scale is correctly estimated. Also, it can fail in cases where the initial scale error is very high, like in large-scale outdoor environments.

We propose a combination of the two strategies: Similar to the former, we start with a visual-only system and run an IMU initializer in parallel. But after IMU initialization we still estimate scale and gravity direction as explicit optimization variables in the main system. This results in a quickly converging and highly accurate system.

This initialization strategy can lead to three questions:

1) How can the visual uncertainty be properly captured in the IMU initializer.

2) How can information about scale and IMU variables be transferred from the IMU initializer to the main system?

3) If the scale estimate changes, how can a consistent marginalization prior be maintained?

VI-DSO [6] tried to address 3 by introducing dynamic marginalization, which does keep the marginalization factor consistent, but loses too much information in the process.

In this work we propose delayed marginalization, which provides a meaningful answer to all three of these questions. The idea is to maintain a second, delayed marginalization prior, which has very little overhead, but enables three techniques:

1) We can populate the delayed factor graph with new IMU factors to perform the proposed pose graph bundle adjustment (PGBA). This is the basis of an IMU initialization which captures the full photometric uncertainty, leading to increased accuracy.

2) The graph used for IMU initialization can be re-advanced, providing a marginalization prior with IMU information for the main system.

3) When the scale changes significantly in the main system we can trigger marginalization replacement.

The combination of these techniques makes for a highly accurate initializer, which is robust even to long periods of unobservability. Based on it we implement a visual-inertial odometry (VIO) system featuring a photometric front-end integrated with a new dynamic photometric weight.

We evaluate our method on three challenging datasets (Fig. 1), capturing three domains: The EuRoC dataset [7] recorded by a flying drone, the TUM-VI dataset [8] captured with a handheld device, and the automotive 4Seasons dataset [9]. The latter features long stretches of constant velocity, posing a particular challenge for mono-inertial odometry.

We show that our system exceeds the state of the art in visualinertial odometry, even outperforming stereo-inertial methods. In summary our contributions are:

\- Delayed marginalization compensates drawbacks of marginalization while retaining the advantages.

\- Pose graph bundle adjustment (PGBA) combines the efficiency of pose graph optimization with the full uncertainty of bundle adjustment.

\- A state-of-the-art visual-inertial odometry system with a novel multi-stage IMU initializer and dynamically weighted photometric factors.

The full source code for our approach will be released.

## II. RELATED WORK

Initially, most visual odometry and SLAM systems have been feature-based [10], either using filtering [11] or nonlinear optimization [12], [13]. More recently, direct methods have been proposed, which optimize a photometric error function and can operate on dense [14], [15], semi-dense [16], or sparse point clouds [17].

Mourikis and Roumeliotis [1] have shown that a tight integration of visual and inertial measurements can greatly increase accuracy and robustness of odometry. Afterwards, many tightly-coupled visual-inertial odometry [18], [19] and SLAM systems [3], [5], [20], [21] have been proposed.

Initialization of monocular visual-inertial systems is not trivial, as sufficient motion is necessary for the scale to become observable [2], [22]. Most systems [3]–[5] start with a visualonly system and use its output for a separate IMU initialization. In contrast to these systems, we continue optimizing the scale explicitly in the main system. We note that ORB-SLAM3 [5] also continues to refine the scale after initialization, but this is a separate optimization fixing all poses and only performed until 75 seconds after initialization. [23] also continues to optimize the scale in the main system, but in contrast to us they do not have a way to transfer covariances between the main system and the initializer, thus they do not achieve the same level of accuracy. Different to all these systems, the proposed delayed marginalization allows our IMU initializer to capture the full visual uncertainty and continuously optimize the scale in the main system.

VI-DSO [6] initializes immediately with an arbitrary scale and explicitly optimizes the scale in the main system. It also introduced dynamic marginalization to handle the consequential large scale changes in the main system. Compared to it we propose a separate IMU initializer, delayed marginalization as a better alternative to dynamic marginalization, a dynamic photometric error weight, and more improvements, resulting in greatly improved accuracy and robustness.

## III. METHOD

## A. Notation

We denote vectors as bold lowercase letters x, matrices as bold upper-case letter H, scalars as lowercase letters $\lambda ,$ , and functions as uppercase letters E. $\mathbf { T } _ { \mathrm { w \_ c a m } _ { i } } ^ { V } \in \mathbf { S E } ( 3 )$ represents the transformation from camera i to world in the visual coordinate frame $V ,$ , and $\mathbf { R } _ { \mathrm { w } _ { - } \mathrm { c a m } _ { i } } ^ { V } \in \mathbf { S O } ( 3 )$ is the respective rotation. Poses are represented either in visual frame $\begin{array} { r } { \mathbf { P } _ { i } ^ { V } : = \mathbf { T } _ { \mathrm { c a m } _ { i - } \mathrm { w } } ^ { V } , } \end{array}$ or in inertial frame $\mathbf { P } _ { i } ^ { I } : = \mathbf { T } _ { \mathrm { w \_ i m u } _ { i } } ^ { I }$ . If not mentioned otherwise we use poses in visual frame $\bar { \mathbf { P } } _ { i } : = \mathbf { P } _ { i } ^ { V }$ . We also use states ${ \mathbf { s } } ,$ which can contain transformations, rotations, and vectors. For states we define the subtraction operator $\mathbf { s } _ { i } \boxminus \mathbf { s } _ { j }$ , which applies log $( \mathbf { R } _ { i } \mathbf { R } _ { i } ^ { - 1 } )$ for rotations and other Lie group elements, and a regular subtraction for vector values.

## B. Direct Visual-Inertial Bundle Adjustment

The core of DM-VIO is the visual-inertial bundle adjustment performed for all keyframes. As commonly done, we jointly optimize visual and IMU variables in a combined energy function. For the visual part we choose a direct formulation based on DSO [17], as it is a very accurate and robust system. For integrating IMU data into the bundle adjustment we perform preintegration [24] between keyframes.

We optimize the following energy function using the Levenberg-Marquardt algorithm:

$$
E ( \mathbf { s } ) = W ( e _ { \mathrm { p h o t o } } ) \cdot E _ { \mathrm { p h o t o } } + E _ { \mathrm { i m u } } + E _ { \mathrm { p r i o r } }\tag{1}
$$

$E _ { \mathrm { p r i o r } }$ contains added priors on the first pose and the gravity direction, as well as the marginalization priors explained in section III-C. In the following we describe the individual energy terms and the optimized state.

Photometric error: The photometric energy is based on [17]. We optimize a set of active keyframes ${ \mathcal F } ,$ each of which hosts a set of points $\mathcal { P } _ { i }$ . Every point p is projected into all keyframes obs(p) where it is visible, and the photometric energy is computed:

$$
E _ { \mathrm { p h o t o } } = \sum _ { i \in \mathcal { F } } \sum _ { \mathbf { p } \in \mathcal { P } _ { i } } \sum _ { j \in \mathrm { o b s } ( \mathbf { p } ) } E _ { \mathbf { p } j }\tag{2}
$$

$$
E _ { \mathbf { p } j } = \sum _ { \mathbf { p } \in \mathcal { N } _ { \mathbf { p } } } \omega _ { \mathbf { p } } \bigg \Vert \big ( I _ { j } [ \mathbf { p ^ { \prime } } ] - b _ { j } \big ) - \frac { t _ { j } e ^ { a _ { j } } } { t _ { i } e ^ { a _ { i } } } ( I _ { i } [ \mathbf { p } ] - b _ { i } ) \bigg \Vert _ { \gamma }\tag{3}
$$

For details regarding the variables we refer the reader to [17].

Dynamic photometric weight: In cases of bad image quality, the system should rely mostly on the inertial data. However due to the photometric cost function used, bad image quality will often lead to very large photometric residuals, effectively increasing the photometric weight compared to the IMU. To counteract this we propose a dynamic photometric weight $W ( e _ { \mathrm { p h o t o } } )$ We compute it using the root mean squared photometric error $e _ { \mathrm { p h o t o } } = \sqrt { E _ { \mathrm { p h o t o } } / n _ { \mathrm { r e s i d u a l s } } } .$

$$
W ( e _ { \mathrm { p h o t o } } ) = \lambda \cdot \left\{ \begin{array} { l l } { ( \theta / e _ { \mathrm { p h o t o } } ) ^ { 2 } , } & { \mathrm { i f } \ e _ { \mathrm { p h o t o } } \geq \theta } \\ { 1 , } & { \mathrm { o t h e r w i s e } } \end{array} \right.\tag{4}
$$

where λ is a static weight component, and θ is the threshold from which the error-dependent weight is activated. This effectively normalizes the root mean squared photometric error to be $\sqrt { \lambda } \theta$ at maximum, similar to a threshold robust cost function [25]. In contrast to the Huber norm in Equation (3), which downweights individual points that violate the photometric assumption, this weight addresses cases where the overall image quality is bad and increases the relative weight of the IMU. In our experiments we choose $\theta = 8$

Optimized variables: We optimize scale and gravity direction as explicit variables. While bundle adjustment can in principle also change the scale and global orientation, convergence is improved when optimizing them explicitly instead [6]. To facilitate this, we represent poses for the visual factors in visual frame V and poses for the IMU factors in IMU frame I. Whereas the IMU frame has a metric scale and a z-axis aligned with gravity direction, the visual frame can have an arbitrary scale and rotation, which is defined during initialization of the visual system. To model this we optimize the scale s and the rotation $\dot { \mathbf { R } } _ { V _ { - } I }$ . As yaw is not observable using an IMU, we fix the last coordinate of $\mathbf { R } _ { V \_ I }$ . We convert between the coordinate frames using:

$$
\begin{array} { r l } & { \mathbf { P } _ { i } ^ { I } : = \mathbf { T } _ { \mathrm { w } _ { - } \mathrm { i m u } _ { i } } ^ { I } = \Omega ( \mathbf { P } _ { i } ^ { V } , \mathbf { S } , \mathbf { R } _ { V _ { - } I } ) } \\ & { \quad \quad = \mathbf { R } _ { V _ { - } I } ^ { - 1 } \mathbf { S } _ { I _ { - } V } ( \mathbf { P } _ { i } ^ { V } ) ^ { - 1 } \mathbf { S } _ { I _ { - } V } ^ { - 1 } \mathbf { T } _ { \mathrm { c a m } _ { - } \mathrm { i m u } } } \end{array}\tag{5}
$$

where $\mathbf { S } _ { I _ { - } V }$ is the Sim(3) element with identity rotation and translation, and scale s. The other variables are converted to Sim(3), but note that the result has scale 1 and is in $\mathbf { S E } ( 3 )$ .

The full state optimized is

$$
\mathbf { s } = \{ s , \mathbf { R } _ { V _ { - } I } \} \cup \bigcup _ { i \in \mathcal { F } } \mathbf { s } _ { i }\tag{6}
$$

with $\mathbf { s } _ { i }$ being the states for all active keyframes defined as:

$$
\mathbf { s } _ { i } = \{ \mathbf { P } _ { i } ^ { V } , \mathbf { v } _ { i } , \mathbf { b } _ { i } , a _ { i } , b _ { i } , d _ { i } ^ { 0 } , d _ { i } ^ { 2 } , . . . d _ { i } ^ { j } \}\tag{7}
$$

where $\mathbf { v } _ { i }$ is the velocity, $\mathbf { b } _ { i }$ the bias, $a _ { i }$ and $b _ { i }$ are affine brightness parameters, and $d _ { i } ^ { j }$ are the inverse depths of active points hosted in the keyframe. Optimization is performed with a custom integration of the SIMD-accelerated code from [17] for photometric residuals and GTSAM for other factors.

IMU Error: We apply the well-known IMU preintegration first proposed in [26], implemented as smart factors in [27], and further improved in [24]. For this energy we use the IMU state $\mathbf { s } _ { i } ^ { I } : = \{ \mathbf { P } _ { i } ^ { \tilde { I } } , \mathbf { v } _ { i } , \mathbf { b } _ { i } \}$ , which contains poses in IMU frame and is computed from the optimized state $\mathbf { s } _ { i }$ using Equation (5). Given the previous state $\mathbf { s } _ { i } ^ { I } .$ , the preintegration data provides us with a prediction $\widehat { \mathbf { s } } _ { j } ^ { I }$ for the following state $\mathbf { s } _ { j } ^ { I }$ as well as a covariance matrix $\widehat { \Sigma } _ { j }$ . The resulting inertial error function penalizes deviations of the current state estimate from the predicted state.

$$
E _ { \mathrm { i m u } } ( \mathbf { s } _ { i } ^ { I } , \mathbf { s } _ { j } ^ { I } ) : = \left( \widehat { \mathbf { s } } _ { j } ^ { I } \boxtimes \mathbf { s } _ { j } ^ { I } \right) ^ { T } \widehat { \mathbf { \xi } } \widehat { \mathbf { \xi } } _ { j } ^ { - 1 } \left( \widehat { \mathbf { s } } _ { j } ^ { I } \boxtimes \mathbf { s } _ { j } ^ { I } \right)\tag{8}
$$

## C. Partial Marginalization Using the Schur Complement

We marginalize old variables using the Schur complement. When marginalizing a set $\beta$ of variables, we gather all factors dependent on them as well as the connected variables $\alpha ,$ which form the Markov blanket. These factors are linearized at the current state estimate, yielding the linear system:

$$
\begin{array} { r } { [ \mathbf { H } _ { \alpha \alpha } \quad \mathbf { H } _ { \alpha \beta } ] [ \mathbf { s } _ { \alpha } ] = [ \mathbf { b } _ { \alpha } ] } \\ { \mathbf { H } _ { \beta \alpha } \quad \mathbf { H } _ { \beta \beta } ] [ \mathbf { s } _ { \beta } ] = [ \mathbf { b } _ { \beta } ] } \end{array}\tag{9}
$$

We apply the Schur-complement, which results in the new linear system $\widehat { { \bf H } _ { \alpha \alpha } } \pmb { s } _ { \alpha } = \widehat { b _ { \alpha } }$ with

$$
\widehat { \mathbf { H } _ { \alpha \alpha } } = \mathbf { H } _ { \alpha \alpha } - \mathbf { H } _ { \alpha \beta } \mathbf { H } _ { \beta \beta } ^ { - 1 } \mathbf { H } _ { \beta \alpha }\tag{10}
$$

$$
\widehat { \pmb { b } _ { \alpha } } = \pmb { b } _ { \alpha } - \mathbf { H } _ { \alpha \beta } \mathbf { H } _ { \beta \beta } ^ { - 1 } \pmb { b } _ { \beta }\tag{11}
$$

This linear system forms a marginalization prior connecting all variables in α (Fig. 2 a).

We keep a maximum of $N _ { f } = 8$ keyframes during the bundle adjustment<sup>1</sup>. The marginalization strategy is taken over from [17]: This means that different from a fixed-lag smoother, we do not always marginalize the oldest pose, but instead keep a combination of newer and older poses, as long as they do not leave the field of view. As shown in [17] this is superior to a fixed-lag smoother for visual odometry. When marginalizing a pose, first all remaining points hosted in the frame are marginalized and residuals with remaining active points are dropped. This retains sparsity of the Hessian while preserving enough information.

## D. Delayed Marginalization

The concept of marginalization explained in the previous section has the advantage of capturing the full probability distribution. In fact, solving the resulting smaller system is equivalent to solving the much larger original system, as long as the marginalized factors are not relinearized.

However, it also comes with severe drawbacks: Reverting the marginalization of a set of variables is not possible without redoing the whole marginalization procedure. Also, to keep the marginalization prior consistent, First-Estimates Jacobians (FEJ) [28] have to be applied. This means that the linearization point of all connected variables has to be fixed as soon as they are connected to a marginalization prior. This is especially problematic for visual-inertial odometry, where the scale is connected to the marginalization prior as soon as the first keyframe is marginalized, but might change significantly. In [6] dynamic marginalization was introduced to combat this, but it is limited in its application to a single one-dimensional variable, namely the scale, and loses most prior inertial information when the scale changes quickly.

![](images/2022_DM-VIO/d207be35f57d4bfa60161e92ecd0d7828f4d84e7fed52244c8782b7aaca7ec7c.jpg)  
Fig. 2. Delayed Marginalization and PGBA: a) Normal marginalization in the visual graph. Note that not always the oldest pose is marginalized. b) Delayed marginalization: We marginalize all variables in the same order as the main graph, but with a delay d (in practice $d = 1 0 0 )$ . Marginalization in this graph is equally fast as marginalizing in the main graph. c) For the pose graph bundle adjustment (PGBA) we populate the delayed graph with IMU factors. This optimization leverages the full photometric uncertainty. d) We readvance the marginalization in the graph used for PGBA to obtain an updated marginalization prior for the main system. This transfers inertial information from the initializer to the main system.

Here we introduce delayed marginalization which circumvents the drawbacks of marginalization while retaining the advantages. It enables us to:

Effectively undo part of the marginalization to capture the full photometric probability distribution for the pose graph bundle adjustment (Section III-E).

\- Update the initially visual-only marginalization prior with IMU information after the IMU initialization.

\- Relinearize variables in the Markov blanket while keeping all visual and most inertial information.

The idea of delayed marginalization is that marginalization cannot be undone, but it can be delayed: In addition to the normal marginalization prior we also maintain a second, delayed marginalization prior and corresponding factor graph. In this delayed graph, marginalization of frames is performed with a delay of d. Points are still marginalized at the same time in the delayed graph, resulting in linearized photometric factors. We note that the same marginalization order as in the original graph is preserved. Switching to a fixed-lag smoother for this graph would immediately lead to a much larger Markov blanket jeopardizing the runtime of the system. E.g. in Fig. 2 b we depict the delayed marginalization of ${ \bf P } _ { 1 }$ . The Markov blanket only contains $\mathbf { P } _ { 0 } , \mathbf { P } _ { 2 }$ , and $\mathbf { P } _ { 3 }$ . If we instead marginalized the oldest frame $\mathbf { P } _ { 0 } .$ the Markov blanket would contain $\mathbf { P } _ { 1 } - \mathbf { P } _ { 7 }$ , leading to higher runtime.

Marginalization in the delayed graph has the same runtime as marginalization in the original graph: The delayed graph contains the same photometric factors as the original graph, and points are marginalized at the same time. This means that each linearized photometric factor in the delayed graph is connected to exactly the $N _ { f } = 8$ keyframes which were active when the respective factor was generated. By keeping the marginalization order, the Markov blanket in the delayed graph always has the same size as the one in the original graph. Thus, the runtime of the Schur complement is the same. This means that the overhead of Delayed Marginalization is very small even for arbitrarily large delays, as it only amounts to an additional marginalization procedure per delayed graph.

## E. Pose Graph Bundle Adjustmentfor IMU Initialization

PGBA utilizes delayed marginalization for IMU initialization. The idea is to populate the delayed graph with IMU factors and optimize all variables (Fig. 2 c).

Populating the graph: Let a frame $\mathbf { P } _ { i }$ be directly connected to the newest pose $\mathbf { P } _ { k }$ iff all poses $\mathbf { P } _ { j } , i < j < k$ have not been marginalized yet. We determine the first frame $\mathbf { P } _ { \mathrm { c o n n } }$ in the delayed graph which is still directly connected to the newest frame. In Fig. 2 c this is $\mathbf { P } _ { 2 }$ . From there, we insert IMU factors and bias factors to all successive frames.

We cannot start before $\mathbf { P } _ { \mathrm { c o n n } }$ because we do not want to insert IMU factors between non-successive keyframes. As the marginalization order is not fixed-lag, this means that we have to optimize poses without corresponding IMU variables.

It can be shown that there can be at most $N _ { f } - 2$ poses without IMU variables: The reason is that all non-connected poses were at some point active at the same time. This means that in practice we have at least $d - N _ { f } + 2$ poses for which we can add IMU data. In practice, we choose $N _ { f } = 8$ and delay $d = 1 0 0$ , meaning that even in the worst case there will be 93 IMU factors in the optimization. As explained previously, fixed-lag smoothing would either result in a dense Hessian or in suboptimal performance of the visual system, so this is a very good trade-off.

Optimization: We optimize the graph with the GTSAM [29] library using the Levenberg-Marquardt optimizer with the provided Ceres-default settings. In this optimization all points are marginalized. We call it pose graph bundle adjustment because it is a combination of regular pose graph optimization (PGO) and bundle adjustment (BA). In contrast to BA, we do not update our estimates for point depths and do not relinearize photometric error terms. Different from PGO we do not use binary constraints between poses, but instead use “octonary” constraints, which connect $N _ { f }$ frames and capture the full probability distribution of BA. Compared to PGO our solution is thus more accurate while being much faster than full BA. By using a fixed delay it is also constrained in runtime even though it can be performed at any time without losing any prior visual information.

Readvancing: Another advantage of delayed marginalization and our PGBA is that we can obtain a marginalization prior for the main system, capturing all visual and inertial information. For this, we readvance the graph used for the PGBA. This works by successively marginalizing all the variables which have been marginalized in the main graph. Again, this is done preserving the marginalization order, resulting in a fixed size Markov blanket in each marginalization step. Hence, marginalizing step by step is significantly faster than marginalizing all variables at once, which would involve a much larger matrix inversion. Fig. 2 d shows the result of readvancing.

## F. Robust Multi-Stage IMU Initialization

Our initialization strategy is based on three insights:

1) When some variables are unknown (in our case scale, gravity direction, and biases) and others are close to the optimum, it is most efficient to first optimize only the unknown variables and fix the others.

2) The most accurate result can be obtained by optimizing all variables jointly, capturing the full covariance.

3) When marginalizing, connected variables have to be close to the optimum, otherwise the marginalization prior becomes inconsistent.

These observations inspire 1) the Coarse IMU Initialization, 2) the PGBA, and 3) the Marginalization Replacement (Fig. 3). Note that after “Initialize main VIO,” the main VIO system III-B (green box) is already running in parallel.

For this initializer we use a single delayed graph with a delay of d <sub>=</sub> 100. This delayed graph will always contain only visual factors and no IMU factors, even after the first initialization, to facilitate the marginalization replacement.

Coarse IMU Initialization: For this we only consider the last $d = 1 0 0$ keyframes and connect them with IMU factors. Similar to the inertial only optimization used for initialization in ORB-SLAM3 [5], in this optimization we fix the poses and use a single bias. We only optimize velocities, bias, the gravity direction and the scale. Gravity direction is initialized by averaging the accelerometer measurements between the first two keyframes, scale is initialized with 1, and bias and velocity with 0. This optimization is less accurate than PGBA but serves as an initialization for it. After optimizing, we compute the marginal covariance for the scale $\operatorname { c o v } ( s )$ and continue to the PGBA if it is smaller than a threshold $\theta _ { \mathrm { i n i t } }$ . As shown in [31], taking into account IMU noise parameters is crucial for good IMU initialization, which our coarse IMU initialization satisfies. But for our method it is just an initialization for the PGBA, which in addition models photometric noise properties.

![](images/2022_DM-VIO/e7bc4659a15d3c861c2e3ddf8c6c245963152707233ce00526ef8ec1761463cc.jpg)  
Fig. 3. Our multi-stage IMU initialization. First we perform a coarse IMU initialization, which provides initial values for the PGBA. The PGBA captures the full visual covariances, achieving very accurate initial estimates for scale, gravity direction, and biases. It also provides an updated marginalization prior for the main graph. By also optimizing the scale in the main VIO system (green box), we can initialize early (purple box) and later reinitialize or perform marginalization replacement, if new information about the scale becomes available. The proposed delayed marginalization is what enables both, the PGBA, and the marginalization replacement.

PGBA IMU Init.: We perform PGBA as explained in section III-E. Afterwards, we again threshold on the marginal covariance for the scale to find out if the optimization was successful. When a tighter threshold $\theta _ { \mathrm { r e i n i t } }$ is not also met, we initialize with the result, but will perform another PGBA afterwards to reinitialize with more accurate values. This reinitialization enables us to set $\theta _ { \mathrm { i n i t } }$ to a relatively large value, allowing to use IMU data in the main system earlier.

Marginalization Replacement: After IMU initialization, we monitor how much the scale s changes compared to the First-Estimates scale $s _ { \mathrm { f e j } }$ used in the marginalization prior. If this change exceeds a threshold $\theta _ { \mathrm { { m a r g } } } ,$ i.e. $\delta _ { s } : = $ ma $\mathopen { } \mathclose \bgroup \left[ s , s _ { \mathrm { f e j } } \aftergroup \egroup \right) / \operatorname* { m i n } ( s , s _ { \mathrm { f e j } } ) > \theta _ { s } ,$ we trigger a marginalization replacement. For the marginalization replacement we rebuild the PGBA graph by populating the delayed graph with IMU factors, Fig. 2 c). Different from the PGBA, we do not optimize in this graph but insteadjust readvance it to obtain an updated marginalization prior. This new prior still contains all visual factors and at least the last $d - \bar { N _ { f } { + } 1 } = 9 3 \bar { \mathrm { I M U } }$ factors. We disable the marginalization replacement if more than $\theta _ { \mathrm { l o s t } } = 5 0 \%$ of the IMU factors contained in the previous prior would be lost. This procedure shows how delayed marginalization can be used to update FEJ values, overcoming one of the main problems of marginalization.

TABLE I  
EVALUATION OF VARIOUS MONO (M) AND STEREO (S) VISUAL-INERTIAL ODOMETRY SYSTEMS ON EUROC. OUR SYSTEM PROVIDES A NOTABLE IMPROVEMENT OVER THE STATE-OF-THE-ART. PLEASE NOTE THAT A FULL SLAM SYSTEM UTILIZING LOOP CLOSURES CAN ACHIEVE EVEN MORE ACCURATE RESULTS, E.G. ORB-SLAM-VI HAS A MEAN ERROR OF 0.075, AND ORB-SLAM3 HAS A MEAN ERROR OF 0.043
<table><tr><td>Sequence</td><td></td><td>MH1</td><td>MH2</td><td>MH3</td><td>MH4</td><td>MH5</td><td>V11</td><td>V12</td><td>V13</td><td>V21</td><td>V22</td><td>V23</td><td>Avg</td></tr><tr><td>MCSKF2 [1] (M)</td><td>RMSE</td><td>0.42</td><td>0.45</td><td>0.23</td><td>0.37</td><td>0.48</td><td>0.34</td><td>0.20</td><td>0.67</td><td>0.10</td><td>0.16</td><td>1.13</td><td>0.414</td></tr><tr><td>OKVIS1 [19] (M)</td><td>RMSE</td><td>0.33</td><td>0.37</td><td>0.25</td><td>0.27</td><td>0.39</td><td>0.094</td><td>0.14</td><td>0.21</td><td>0.090</td><td>0.17</td><td>0.23</td><td>0.231</td></tr><tr><td>ROVIO2 [18] (M)</td><td>RMSE</td><td>0.21</td><td>0.25</td><td>0.25</td><td>0.49</td><td>0.52</td><td>0.10</td><td>0.10</td><td>0.14</td><td>0.12</td><td>0.14</td><td>0.14</td><td>0.224</td></tr><tr><td>VINS-Mono [3] (M)</td><td>RMSE</td><td>0.15</td><td>0.15</td><td>0.22</td><td>0.32</td><td>0.30</td><td>0.079</td><td>0.11</td><td>0.18</td><td>0.080</td><td>0.16</td><td>0.27</td><td>0.184</td></tr><tr><td>Kimera [21] (S)</td><td>RMSE</td><td>0.11</td><td>0.10</td><td>0.16</td><td>0.24</td><td>0.35</td><td>0.05</td><td>0.08</td><td>0.07</td><td>0.08</td><td>0.10</td><td>0.21</td><td>0.141</td></tr><tr><td>Online VIO [23] (M)</td><td>RMSE</td><td>0.14</td><td>0.13</td><td>0.20</td><td>0.22</td><td>0.20</td><td>0.05</td><td>0.07</td><td>0.16</td><td>0.04</td><td>0.11</td><td>0.17</td><td>0.135</td></tr><tr><td>VI-DSO [6] (M)</td><td>RMSE Scale Error (%)</td><td>0.062 1.1</td><td>0.044 0.5</td><td>0.117 0.4</td><td>0.132 0.2</td><td>0.121 0.8</td><td>0.059 1.1</td><td>0.067 1.1</td><td>0.096 0.8</td><td>0.040 1.2</td><td>0.062 0.3</td><td>0.174 0.4</td><td>0.089 0.7</td></tr><tr><td>BASALT [20] (S)</td><td>RMSE</td><td>0.07</td><td>0.06</td><td>0.07</td><td>0.13</td><td>0.11</td><td>0.04</td><td>0.05</td><td>0.10</td><td>0.04</td><td>0.05</td><td>-</td><td>0.072</td></tr><tr><td>DM-VIO (M)</td><td>RMSE Scale Error (%)</td><td>0.065 1.3</td><td>0.044 0.9</td><td>0.097 0.4</td><td>0.102 0.2</td><td>0.096 0.4</td><td>0.048 0.4</td><td>0.045 1.0</td><td>0.069 0.3</td><td>0.029 0.02</td><td>0.050 0.6</td><td>0.114 0.8</td><td>0.069 0.6</td></tr></table>

<sup>1</sup> results taken from [3].  
<sup>2</sup> results taken from [30], these are Sim(3)-aligned.  
All other results are taken from the respective paper.

In realtime mode we perform the coarse IMU initialization and the PGBA in a separate thread. Note how important the proposed delayed marginalization is for this IMU initialization. It allows the PGBA to capture the full covariance from the photometric bundle adjustment. By readvancing, this also enables us to generate a marginalization prior for the main system, containing all IMU information from the initializer. Lastly, it is used for updating the marginalization prior when the scale changes after the initialization.

## IV. RESULTS

We evaluate our method on the EuRoC dataset [7], the TUM-VI dataset [8], and the 4Seasons dataset [9], covering flying drones, handheld sequences, and autonomous driving respectively. We encourage the reader to watch the supplementary video which shows qualitative realtime results on 4Seasons and TUM-VI slides1. We also provide ablation studies and runtime evaluations in the supplementary available at http://vision.in. tum.de/dm-vio.

Unless otherwise stated all experiments are performed in realtime mode on the same MacBook Pro 2013 (i7 at 2.3 GHz) which was used for generating the results in [6], without utilizing the GPU. As ORB-SLAM3 is not officially supported on MacOS, we show results for it on a slightly stronger desktop with an Intel Core i7-7700 K at 4.2 GHz, which is very similar to the PC used in their paper.

All methods are evaluated 10 times for EuRoC and 5 times for the other datasets on each sequence. Following [17], results are presented in cumulative error plots, which show how many sequences (y-axis) have been tracked with an accuracy better than the threshold on the x-axis. We perform SE(3) alignment of the trajectory with the provided ground-truth and report the root mean squared error (RMSE), also called absolute trajectory error (ATE). On TUM-VI and 4Seasons, trajectory lengths can vary greatly so we report the drift in %, which we compute with drift <sub>=</sub> <sup>rmse·100</sup> . We also show tables to compare to numbers length from other papers and report the median result for each sequence for our method.

## A. EuRoC Dataset

The EuRoC dataset [7] is the most popular visual-inertial dataset to date, and many powerful methods have been evaluated on it. In Table I we compare to the state-of-the art in visual-inertial odometry, all results are without loop-closure. Our method outperforms all other methods clearly in terms of RMSE. The closest competitor is Basalt [20], a stereo-inertial method which achieves a smaller error on 2 sequences. We also observe the lowest average scale error reported on the dataset so far, confirming that our contributions in IMU initialization have a positive impact on performance. In the supplementary we provide runtime evaluations, showing that tracking takes 10.34 ms on average, and keyframe processing takes 53.67 ms. The delayed marginalization is responsible for an overhead of 0.44 ms or 0.8% in the keyframe thread.

## B. TUM-VI Dataset

The TUM-VI dataset [8] is a very challenging handheld dataset, featuring large-scale indoor and outdoor scenes, and even sequences sliding down a tube, where almost the full image is covered. With long periods of walking in straight lines, stereo methods have an advantage here as they still can observe the scale with constant motion. We compare to the state-of-the-art visual-inertial odometry methods evaluated in [8] in Table II. Our method clearly outperforms the other monocular method in VINS-Mono [3] on most sequences, and even compared to the stereo methods it shows the best result on 16 sequences and a mean drift of 0.472. The closest competitor is again Basalt, which achieves the best result on 8 sequences and a mean drift of 0.939.

On this dataset, we also evaluate against ORB-SLAM3 [5], which is the state-of-the art visual-inertial SLAM system. This is not entirely fair as ORB-SLAM3 uses loop closures (which cannot be disabled), constituting an advantage over the other methods. We find the comparison still helpful as it allows to make conclusions regarding the underlying odometry. We have evaluated ORB-SLAM3 5 times on each sequence and reproduced their results with code and settings provided by the authors. For this comparison we have also evaluated VI-DSO [6], and the results are shown in Fig. 4. We observe that ORB-SLAM3 is more accurate on some sequences thanks to its very strong loop closure system. However, our method is more robust overall.

TABLE II  
RMSE ATE IN M ON THE TUM-VI DATASET [8]. BEST RESULTS IN BOLD, UNDERLINE IS THE BEST RESULT AMONG MONOCULAR METHODS. DM-VIO OUTPERFORMS EVEN STATE-OF-THE-ART STEREO-INERTIAL METHODS BY A LARGE MARGIN
<table><tr><td>Sequence</td><td>ROVIO stereo</td><td>VINS mono</td><td>OKVIS stereo</td><td>BASALT stereo</td><td>DM-VIO mono</td><td>length</td></tr><tr><td></td><td></td><td>0.63</td><td>0.33</td><td>0.34</td><td>0.19</td><td>[m]</td></tr><tr><td>corridor1 corridor2</td><td>0.47 0.75</td><td>0.95</td><td>0.47</td><td>0.42</td><td>0.47</td><td>305 322</td></tr><tr><td>corridor3</td><td>0.85</td><td>1.56</td><td>0.57</td><td>0.35</td><td>0.24</td><td>300</td></tr><tr><td>corridor4</td><td>0.13</td><td>0.25</td><td>0.26</td><td>0.21</td><td>0.13</td><td>114</td></tr><tr><td>corridor5</td><td>2.09</td><td>0.77</td><td>0.39</td><td>0.37</td><td>0.16</td><td>270</td></tr><tr><td>magistrale1</td><td>4.52</td><td>2.19</td><td>3.49</td><td>1.20</td><td>2.35</td><td>918</td></tr><tr><td>magistrale2</td><td>13.43</td><td>3.11</td><td>2.73</td><td>1.11</td><td>2.24</td><td>561</td></tr><tr><td>magistrale3</td><td>14.80</td><td>0.40</td><td>1.22</td><td>0.74</td><td>1.69</td><td>566</td></tr><tr><td>magistrale4</td><td>39.73</td><td>5.12</td><td>0.77</td><td>1.58</td><td>1.02</td><td>688</td></tr><tr><td>magistrale5</td><td>3.47</td><td>0.85</td><td>1.62</td><td>0.60</td><td>0.73</td><td>458</td></tr><tr><td>magistrale6</td><td>X</td><td>2.29</td><td>3.91</td><td>3.23</td><td>1.19</td><td>771</td></tr><tr><td>outdoors1</td><td>101.95</td><td>74.96</td><td>X</td><td>255.04</td><td>123.24</td><td>2656</td></tr><tr><td>outdoors2</td><td>21.67</td><td>133.46</td><td>73.86</td><td>64.61</td><td>12.76</td><td>1601</td></tr><tr><td>outdoors3</td><td>26.10</td><td>36.99</td><td>32.38</td><td>38.26</td><td>8.92</td><td>1531</td></tr><tr><td>outdoors4</td><td>X</td><td>16.46</td><td>19.51</td><td>17.53</td><td>15.25</td><td>928</td></tr><tr><td>outdoors5</td><td>54.32</td><td>130.63</td><td>13.12</td><td>7.89</td><td>7.16</td><td>1168</td></tr><tr><td>outdoors6</td><td>149.14</td><td>133.60</td><td>96.51</td><td>65.50</td><td>34.86</td><td>2045</td></tr><tr><td>outdoors7</td><td>49.01</td><td>21.90</td><td>13.61</td><td>4.07</td><td>5.00</td><td>1748</td></tr><tr><td>outdoors8</td><td>36.03</td><td>83.36</td><td>16.31</td><td>13.53</td><td>2.11</td><td>986</td></tr><tr><td>room1</td><td>0.16</td><td>0.07</td><td>0.06</td><td>0.09</td><td>0.03</td><td>146</td></tr><tr><td>room2</td><td>0.33</td><td>0.07</td><td>0.11</td><td>0.07</td><td>0.13</td><td>142</td></tr><tr><td>room3</td><td>0.15</td><td>0.11</td><td>0.07</td><td>0.13</td><td>0.09</td><td>135</td></tr><tr><td>room4</td><td>0.09</td><td>0.04</td><td>0.03</td><td>0.05</td><td>0.04</td><td>68</td></tr><tr><td>room5</td><td>0.12</td><td>0.20</td><td>0.07</td><td>0.13</td><td>0.06</td><td>131</td></tr><tr><td>room6</td><td>0.05</td><td>0.08</td><td>0.04</td><td>0.02</td><td>0.02</td><td>67</td></tr><tr><td>slides1</td><td>13.73</td><td>0.68</td><td>0.86</td><td>0.32</td><td>0.31</td><td>289</td></tr><tr><td>slides2</td><td>0.81</td><td>0.84</td><td>2.15</td><td>0.32</td><td>0.87</td><td>299</td></tr><tr><td>slides3</td><td>4.68</td><td>0.69</td><td>2.58</td><td>0.89</td><td>0.60</td><td>383</td></tr><tr><td>avg drift%</td><td>16.83*</td><td>1.700</td><td>0.815*</td><td>0.939</td><td>0.472</td><td></td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td>normalized</td></tr></table>

![](images/2022_DM-VIO/26dc55e362e6ef5ab8d14a9382566334faaa8b18ac02a287c4d65df3f97df8b1.jpg)  
Fig. 4. Cumulative error plot for the TUM-VI dataset (drift in %). Our method clearly outperforms both VI-DSO and ORB-SLAM3 in terms of robustness. Thanks to its powerful loop closure system, ORB-SLAM3 has an advantage in terms of accuracy on some sequences.

![](images/2022_DM-VIO/74a4b547e2ef198e06aac22467d950f70f8e4c3a35fd1c60735d486645be3b96.jpg)  
Fig. 5. Cumulative error plot for the 4Seasons dataset (drift in %). With lots of stretches with constant velocity, this dataset is extremely challenging for monocular visual-inertial methods. Thanks to our novel IMU initializer powered by delayed marginalization and PGBA, DM-VIO is able to cope with it and even outperforms stereo-inertial methods.

This indicates that an integration of loop closure and map reuse into our system would be an interesting future research direction.

## C. 4Seasons Dataset

The 4Seasons dataset [9] is a very recent automotive dataset, which, in contrast to most other car datasets, features a well time-synchronized visual-inertial sensor. The lower part of the images is obstructed by the car hood, hence we crop off the bottom 96 pixels, which we do for all methods. As this is the first odometry method to evaluate on the 4Seasons dataset, we make sure to determine IMU noise parameters for all methods the same way to ensure a fair comparison: We have manually read off the accelerometer and gyroscope noise density and bias random walk from the Allan variance plot provided in the data sheet of the IMU. To handle unmodeled effects we follow [8] and inflate noise values by different amounts to determine the best setting for all methods. For each method we tried noise models inflated by 1, 10, 100, 1000 respectively and chose the configuration which gave best results. For VI-DSO and for our method we slightly modified the visual initializer by adding a zero-prior to the translation on the x and y axis, and also added a threshold to stop keyframe creation for translations smaller than 0.01 m (the latter was not activated for VI-DSO as it did not improve the results for it). Otherwise, parameters are the same as for the other experiments. For Basalt we tried all three provided default configurations with the optimal noise values to find the best settings. After choosing the configuration for each method, we perform one final evaluation, running all 30 sequences 5 times each.

The results are shown in Fig. 5. It is clear that the automotive scenario is very challenging for monocular methods. This is expected as it naturally features many stretches with constant motion, where scale is not observable, constituting a challenge for IMU initialization. Thanks to our novel IMU initialization, DM-VIO not only works well on the dataset but even outperforms stereo-inertial ORB-SLAM3 and Basalt, while using monocular images and no loop closures.

## V. CONCLUSION AND FUTURE WORK

We have presented a monocular visual-inertial odometry system which outperforms the state of the art, even stereo-inertial methods. Thanks to a novel IMU initializer, it works well in flying, handheld, and automotive scenarios, extending the applicability of monocular methods. The foundation of our IMU initialization is delayed marginalization, which also enables the pose graph bundle adjustment.

We anticipate that this method will spark further research in this direction. The idea of delayed marginalization could be applied to more use cases, e.g. for reactivating old keyframes in a marginalization setting to enable map reuse. The pose graph bundle adjustment can also be applied to long-term loop closures. Lastly, our open-source system is easily extendible, as all optimizations are integrated with GTSAM, allowing to quickly add new factors. This could be used for GPS integration, wheel odometry, and more.

## REFERENCES

[1] A. I. Mourikis and S. I. Roumeliotis, “A multi-state constraint Kalman filter for vision-aided inertial navigation,” in Proc. IEEE Int. Conf. Robot. Automat., 2007, pp. 3565–3572.

[2] J. Kaiser, A. Martinelli, F. Fontana, and D. Scaramuzza, “Simultaneous state initialization and gyroscope bias calibration in visual inertial aided navigation,” RA-L, vol. 2, no. 1, pp. 18–25, 2017.

[3] T. Qin, P. Li, and S. Shen, “VINS-Mono: A robust and versatile monocular visual-inertial state estimator,” T-RO, vol. 34, no. 4, pp. 1004–1020, 2018.

[4] R. Mur-Artal and J. D. Tardós, “Visual-inertial monocular slam with map reuse,” RA-L, vol. 2, no. 2, pp. 796–803, 2017.

[5] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel, and J. D. Tardós, “ORB-SLAM3: An accurate open-source library for visual, visual-inertial, and multimap slam,” IEEE Trans. Robot., vol. 37, no. 6, pp. 1874–1890, 2021, doi: 10.1109/TRO.2021.3075644.

[6] L. von Stumberg, V. Usenko, and D. Cremers, “Direct sparse visual-inertial odometry using dynamic marginalization,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 2510–2517.

[7] M. Burri et al., “The EuRoC micro aerial vehicle datasets,” The Int. J. Robot. Res., vol. 35, no. 10, pp. 157–1163, 2016.

[8] D. Schubert, T. Goll, N. Demmel, V. Usenko, J. Stueckler, and D. Cremers, “The TUM VI benchmark for evaluating visual-inertial odometry,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 1680–1687.

[9] P. Wenzel et al., “4Seasons: A cross-season dataset for multi-weather SLAM in autonomous driving,” in Proc. DAGM German Conf. Pattern Recognit., 2020, pp. 404–417.

[10] D. Nister, O. Naroditsky, and J. Bergen, “Visual odometry,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2004, vol. 1, pp. 652–659.

[11] A. Davison, I. Reid, N. Molton, and O. Stasse, “MonoSLAM: Real-time single camera SLAM,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 29, no. 6, pp. 1052–1067, Jun. 2007.

[12] G. Klein and D. Murray, “Parallel tracking and mapping for small AR workspaces,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2007, pp. 225–234.

[13] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardós, “ORB-SLAM: A versatile and accurate monocular slam system,” T-RO, vol. 31, no. 5, pp. 1147–1163, Oct. 2015.

[14] C. Kerl, J. Sturm, and D. Cremers, “Robust odometry estimation for RGB-D cameras,” in Proc. IEEE Int. Conf. Robot. Automat., 2013, pp. 3748–3754.

[15] R. Newcombe, S. Lovegrove, and A. Davison, “DTAM: Dense tracking and mapping in real-time,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2011, pp. 2320–2327.

[16] J. Engel, T. Schöps, and D. Cremers, “LSD-SLAM: Large-scale direct monocular SLAM,” in Proc. IEEE Eur. Conf. Comput. Vis., 2014, pp. 834–849.

[17] J. Engel, V. Koltun, and D. Cremers, “Direct sparse odometry,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 3, pp. 611–625, Mar. 2018.

[18] M. Bloesch, S. Omari, M. Hutter, and R. Siegwart, “Robust visual inertial odometry using a direct EKF-based approach,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2015, pp. 298–304.

[19] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, “Keyframe-based visual-inertial odometry using nonlinear optimization,” The Int. J. Robot. Res., vol. 34, no. 3, pp. 314–334, 2014.

[20] V. Usenko, N. Demmel, D. Schubert, J. Stueckler, and D. Cremers, “Visualinertial mapping with non-linear factor recovery,” RA-L, vol. 5, no. 2, pp. 422–429, 2020.

[21] A. Rosinol, M. Abate, Y. Chang, and L. Carlone, “Kimera: An open-source library for real-time metric-semantic localization and mapping,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 1689–1696.

[22] A. Martinelli, “Closed-form solution of visual-inertial structure from motion,” Int. J. Comput. Vis., vol. 106, no. 2, pp. 138–152, 2014.

[23] E. Hong and J. Lim, “Visual-inertial odometry with robust initialization and online scale estimation,” Sensors, vol. 18, p. 4287, 2018.

[24] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation,” in Proc. Robot.: Sci. Syst., 2015. [Online]. Available: http: //www.roboticsproceedings.org/rss11/p06.html

[25] K. MacTavish and T. D. Barfoot, “At all costs: A comparison of robust cost functions for camera correspondence outliers,” in Proc. 12th Conf. Comput. Robot Vis., 2015, pp. 62–69.

[26] T. Lupton and S. Sukkarieh, “Visual-inertial-aided navigation for highdynamic motion in built environments without initial conditions,” T-RO, vol. 28, no. 1, pp. 61–76, 2012.

[27] L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert, “Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors,” in Proc. IEEE Int. Conf. Robot. Automat., 2014, pp. 4290–4297.

[28] G. Huang, A. I. Mourikis, and S. Roumeliotis, “A first-estimates jacobian EKF for improving slam consistency,” in Proc. Exp. Robot., 2008, pp. 373– 372.

[29] F. Daellert, “Factor graphs and GTSAM: A hands-on introduction,” Georgia Inst. Technol., Tech. Rep. GT-RIM-CP&R-2012-002, Sep. 2012. [Online]. Available: https://gtsam.org

[30] J. Delmerico and D. Scaramuzza, “A benchmark comparison of monocular visual-inertial odometry algorithms for flying robots,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 2502–2509.

[31] C. Campos, J. Montiel, and J. D. Tardós, “Inertial-only optimization for visual-inertial initialization,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 51–57.