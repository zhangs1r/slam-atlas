# Visual-Inertial Mapping With Non-Linear Factor Recovery

Vladyslav Usenko , Nikolaus Demmel , David Schubert , Jörg Stückler , and Daniel Cremers

Abstract—Cameras and inertial measurement units are complementary sensors for ego-motion estimation and environment mapping. Their combination makes visual-inertial odometry (VIO) systems more accurate and robust. For globally consistent mapping, however, combining visual and inertial information is not straightforward. To estimate the motion and geometry with a set of images large baselines are required. Because of that, most systems operate on keyframes that have large time intervals between each other. Inertial data on the other hand quickly degrades with the duration of the intervals and after several seconds of integration, it typically contains only little useful information. In this letter, we propose to extract relevant information for visual-inertial mapping from visual-inertial odometry using non-linear factor recovery. We reconstruct a set of non-linear factors that make an optimal approximation of the information on the trajectory accumulated by VIO. To obtain a globally consistent map we combine these factors with loop-closing constraints using bundle adjustment. The VIO factors make the roll and pitch angles ofthe global map observable, and improve the robustness and the accuracy of the mapping. In experiments on a public benchmark, we demonstrate superior performance of our method over the state-of-the-art approaches.

Index Terms—Simultaneous localization and mapping, sensor fusion.

## I. INTRODUCTION

V <sup>ISUAL-INERTIAL</sup> <sup>odometry</sup> <sup>(VIO)</sup> <sup>is</sup> <sup>a</sup> <sup>popular</sup> <sup>approach</sup> for tracking the motion of a camera in application domains such as robotics or augmented reality. By combining visual and IMU measurements, one can exploit the complementary strengths of both sensors and thereby increase accuracy and robustness. Commonly, the optimization of camera trajectory and map is performed locally on a small window of recent camera frames and IMU measurements. This approach, however, is inevitably prone to drift in the estimates.

Globally consistent optimization for visual-inertial mapping is less explored in the computer vision community. While in principle the optimization could be formulated as bundle adjustment with additional IMU measurements, this approach would quickly become computationally infeasible due to the high number of frames which would lead to a large number of optimization parameters in a naive formulation. To keep the computational burden in bounds, bundle adjustment subsamples the high-frame rate images of the camera to a smaller set of keyframes. The common choice in VIO is to preintegrate IMU measurements between consecutive frames. If we select keyframes temporally far apart to make the optimization efficient, the preintegrated IMU measurements provide only little information to constrain the trajectory due to the accumulated sensor noise. The small frame rate also affects the quality of the estimated velocities and biases from visual and inertial cues which are required for pose prediction using preintegrated IMU measurements.

![](images/2020_Basalt/fd4a003ca190545385f47a25c411d2038f3fbdcbdcbc3c99cbc85b52eab7fa28.jpg)  
Fig. 1. Orthographic top-down projection of the map (MH\_05 sequence of the EuRoC dataset [5]) rendered using the estimated gravity direction. To obtain a gravity-aligned globally consistent map, non-linear factors are recovered from the marginalization prior of the VIO and combined with keypoint-based bundle adjustment. Green lines visualize keyframe connections resulting from bundle adjustment factors and red lines connections from the recovered relative pose factors. Additionally each keyframe has a recovered factor that penalizes deviation from the gravity direction observed in VIO.

We propose a novel approach that formulates visual-inertial mapping as bundle adjustment on a high-frame-rate set of visual and inertial measurements. Instead of directly optimizing the camera trajectory for all frames, we propose a hierarchical approach which first recovers a local VIO estimate at the frame rate of the camera. Once keyframes are removed and marginalized from the current local VIO optimization window, we extract non-linear factors [15] that approximate the accumulated visual-inertial information about the camera motion between keyframes. The keyframes and non-linear factors are subsequently used on the global bundle-adjustment layer.

For the VIO layer, our method uses image features designed for fast and accurate tracking, while for the mapping layer we employ distinctive but lighting and viewpoint invariant keypoints that are suitable for loop closing. With this, our approach can leverage information from the IMU and short-term visual tracking at high frame rates together with keypoint matching and loop-closing at low frame rates for globally consistent mapping (Fig. 1). The factors also help to keep the map gravity-aligned, bridge between frames that do not have enough visual information. Our approach also makes the optimization problem smaller, since we do not have to estimate velocities and biases.

In summary, our contributions are:

\- We propose a novel two-layered visual-inertial mapping approach that integrates keypoint-based bundleadjustment with inertial and short-term visual tracking through non-linear factor recovery.

\- As the first layer of our mapping approach we propose a VIO system which outperforms the state-of-the-art methods in terms of trajectory accuracy on the majority of the evaluated sequences. This is achieved by carefully combining appropriate components (patch tracking, landmark representation, first-estimate Jacobians, marginalization scheme) as detailed in Section IV.

\- Unlike other state-of-the-art systems that use preintegrated IMU measurements also for mapping, we subsume highframe rate visual-inertial information in non-linear factors extracted from the marginalization prior of the VIO layer. This results not only in a smaller optimization problem but also in better pose estimates in the resulting gravity aligned map.

We encourage the reader to watch the demonstration video and inspect the open-source implementation of the system, which is available at:

https://vision.in.tum.de/research/vslam/basalt

## II. RELATED WORK

Visual-inertial odometry: Early methods for visual-inertial odometry are primarily filter-based [11], [18]. In tightly integrated filters, the prediction step typically propagates the current camera state estimate using the IMU measurements. The state is recursively corrected based on the camera images. A significant drawback of filters is that the linearization point for the non-linear measurement and state transition models cannot be changed, once a measurement is integrated. Fixed-lag smoothers (a.k.a. optimization-based approaches) such as [13], [27] relinearize at the current states in a local optimization window of recent frames. The visual-inertial state estimation is formulated as a full bundle adjustment (BA) over keyframes and IMU measurements. The problem is reduced to a computationally manageable size by marginalization of old frames up to the recent set in the optimization window. The continuous relinearization, windowed optimization and maintenance of the marginalization prior increase the accuracy of the methods. The above methods need to discard keypoints and observations that are observed in marginalized keyframes in order to maintain the sparse structure of the marginalization prior. Hsiung et al. [9] apply non-linear factor recovery to achieve a sparse marginalization prior without discarding information about observed keypoints. This way, the approach can further refine the keypoints and achieve higher accuracy, but in contrast to our work it is limited to local BA.

Visual-inertial mapping: Only few works have tackled globally consistent mapping from visual and inertial measurements. Kasyanov et al. [12] add a pose-graph optimization layer with loop-closing on top of a keyframe-based visual-inertial odometry method [13]. The pose graph is built from the keyframes of the VIO and their relative pose estimates. In [19], the authors add inertial measurements to a keyframe-based SLAM system through IMU preintegration. The IMU measurements are preintegrated into a set of pseudo-measurements between keyframes. They notice that the accuracy of preintegrated measurements degrades over time and restrict the time between keyframes to 0.5 seconds in local BA and 3 seconds in global BA. A further shortcoming of the method is its requirement of estimating the camera velocity and IMU biases at each keyframe which is less well constrained through visual measurements than in our approach due to the strong temporal subsampling into keyframes. Schneider et al. [24] follow a similar approach in which preintegrated IMU measurements are inserted into the optimization. The approach in [20] proposes a combination of VIO and 4 degree-of-freedom (DoF) pose optimization for visual-inertial mapping. They fix 2 DoF (roll and pitch) and optimize only for the others. We also constrain roll and pitch from visual-inertial measurements. However, we extract non-linear factors in a probabilistic formulation which account for uncertainties in those values and are traded off with other information in the global probabilistic optimization.

## III. PRELIMINARIES

In this letter, we write matrices as bold capital letters $( \mathrm { e . g . R } )$ and vectors as bold lowercase letters $( \mathrm { e . g . } \xi )$ . Rigid-body poses are represented as $( \mathbf { R } , \mathbf { p } ) \in \mathrm { S O } ( 3 ) \times \mathbb { R } ^ { 3 }$ or as transformation matrices $\mathbf { T } \in \mathrm { S E } ( 3 )$ when needed. Incrementing a rotation R by an increment $\pmb { \xi } \in \mathbb { R } ^ { 3 }$ is defined as $\mathbf { R } \oplus \pmb { \xi } = \mathrm { E x p } ( \pmb { \xi } ) \mathbf { R }$ . The difference between two rotations ${ \bf R } _ { 1 }$ and ${ \bf R } _ { 2 }$ is calculated as ${ \bf R } _ { 1 } \ominus { \bf R } _ { 2 } = \mathrm { L o g } ( { \bf R } _ { 1 } { \bf R } _ { 2 } ^ { - 1 } )$ such that $( \mathbf { R } \oplus \pmb { \xi } ) \ominus \mathbf { R } = \pmb { \xi }$ . Here we use $\mathrm { E x p } : \mathbb { R } ^ { 3 }  \mathrm { S O ( 3 ) }$ ), which is a composition of the hat operator $( \mathbb { R } ^ { 3 } \to \mathfrak { s o } ( 3 ) )$ and the matrix exponential $( \mathfrak { s o } ( 3 ) \to $ $\mathrm { S O ( 3 ) } )$ and maps rotation vectors to their corresponding rotation matrices, and its inverse $\mathrm { L o g : S O ( 3 ) \to \mathbb { R } ^ { 3 } }$ . For all other variables, such as translation, velocity and biases, we define ⊕ and  as regular addition and subtraction.

In the following we will use a state s that is defined as a tuple of several rotation and vector variables, and a function $\mathbf { r } ( \mathbf { s } )$ that depends on it and can also produce rotations and vectors as the result. An increment $\pmb { \xi } \in \mathbb { R } ^ { n }$ is a stacked vector with all the increments of the variables in s. Then, the Jacobian of the function with respect to the increment is defined as

$$
\mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } = \operatorname* { l i m } _ { \boldsymbol { \xi } \to \mathbf { 0 } } \frac { \mathbf { r } ( \mathbf { s } \oplus \boldsymbol { \xi } ) \ominus \mathbf { r } ( \mathbf { s } ) } { \boldsymbol { \xi } } .\tag{1}
$$

Here, $\mathbf { s } \oplus \pmb { \xi }$ denotes that each component in s is incremented with the corresponding segment in $\boldsymbol { \xi }$ using the appropriate definition of the $\oplus$ operator, and similarly $\operatorname { f o r } \ominus$ . The limit is done component-wise, such that the Jacobian is a matrix. For

Euclidean quantities, this definition is just a normal derivative, with an extension for rotations, both as function value and as function argument. For more details and possible alternative formulations we refer the reader to [2], [4], [7].

In non-linear least squares problems, we minimize functions of the form

$$
E ( \mathbf { s } ) = \frac { 1 } { 2 } \mathbf { r } ( \mathbf { s } ) ^ { \top } \mathbf { W } \mathbf { r } ( \mathbf { s } ) ,\tag{2}
$$

which is a squared norm of the sum of residuals with blockdiagonal weight matrix W. In this case, r(s) is purely vectorvalued. Near the current state s we can use a linear approximation of the residual, which leads to

$$
E ( \mathbf { s } \oplus \pmb { \xi } ) = E ( \mathbf { s } ) + \xi ^ { \top } \mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } ^ { \top } \mathbf { W } \mathbf { r } ( \mathbf { s } ) + \frac { 1 } { 2 } \xi ^ { \top } \mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } ^ { \top } \mathbf { W } \mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } \xi .\tag{3}
$$

The optimum of this approximated energy can be attained using the Gauss-Newton increment

$$
\begin{array} { r } { \pmb { \xi } ^ { * } = - ( \mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } ^ { \top } \mathbf { W } \mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } ) ^ { - 1 } \mathbf { J } _ { \mathbf { r } ( \mathbf { s } ) } ^ { \top } \mathbf { W } \mathbf { r } ( \mathbf { s } ) . } \end{array}\tag{4}
$$

With this, we can iteratively update the state $\mathbf { s } _ { i + 1 } = \mathbf { s } _ { i } \oplus \pmb { \xi } ^ { * }$ until convergence.

## IV. VISUAL-INERTIAL ODOMETRY

We formulate the incremental motion tracking of the camera-IMU setup over time as fixed-lag smoothing. First, we use patch-based optical flow to track a sparse set of points in the 2D image plane between consecutive frames. This information is then used in a bundle-adjustment framework which for every frame minimizes an error that consists of point reprojection and IMU propagation terms. To maintain a fixed parameter size of the optimization problem we marginalize out old states. In the remainder of this section we will discuss these stages in more detail.

## A. KLT Tracking

As a first step of our algorithm we detect a sparse set of keypoints in the frame using the FAST [22] corner detector. To track the motion of these points over a series of consecutive frames we use sparse optical flow based on KLT [14]. To achieve fast, accurate and robust tracking we combine the inverse-compositional approach as described in [1] with a patch dissimilarity norm that is invariant to intensity scaling. Several authors suggested zero-normalized cross-correlation (ZNCC) for illumination-invariant optical flow [17], [25], but we use locally-scaled sum ofsquared differences (LSSD) defined in [21] which is computationally less expensive than alternatives.

We formulate the patch tracking problem as estimating the transform $\mathbf { T } \in \mathrm { S E } ( 2 )$ between two corresponding patches in two consecutive frames that minimizes the differences between the patches according to the selected norm. Essentially, we minimize a sum of squared residuals, where every residual is defined as

$$
r _ { i } ( \pmb { \xi } ) = \frac { I _ { t + 1 } ( \mathbf T \mathbf x _ { i } ) } { I _ { t + 1 } } - \frac { I _ { t } ( \mathbf x _ { i } ) } { I _ { t } } \quad \forall \mathbf x _ { i } \in \Omega .\tag{5}
$$

![](images/2020_Basalt/eb84c7a4dabe0708590adc8cbef8a69aef8dc09f13449c750d0b4ac5867ceb04.jpg)  
Fig. 2. Example of KLT tracks estimated by our system. Despite changes in exposure time the proposed method is able to estimate the warp in SE(2) between the patches in the images.

Here, $I _ { t } ( \mathbf { x } )$ is the intensity of image t at pixel location x. The set of image coordinates that defines the patch is denoted Ω and the mean intensity of the patch in image t is $\overline { { I _ { t } } } .$ A visualization of the patch and tracking results is shown in Fig. 2.

To achieve robustness to large displacements in the image we use a pyramidal approach, where the patch is first tracked on the coarsest level and then on increasingly finer levels. For outlier filtering, instead of an absolute threshold on the error, we track the patches from the current frame to the target frame and back to check consistency. Points that do not return to the initial location with the second tracking are considered as outliers and discarded.

## B. Visual-Inertial Bundle Adjustment

To estimate the motion of the camera we combine error terms based on tracked feature locations from KLT tracking with IMU error terms based on preintegrated IMU measurements [8].

We use the following coordinate frames throughout the letter: W is the world frame, I is the IMU frame and $\mathrm { C } _ { i }$ is the frame of camera i, where i is the index of the camera in a stereo setup. We estimate transformations $\mathbf { T } _ { \mathrm { W I } } \in \mathrm { S E } ( 3 )$ from IMU to world coordinate frame. The transformations $\mathbf { T } _ { \mathrm { I C } }$ from camera frame i to IMU frame and the projection functions $\pi _ { i }$ are assumed to be static and known from calibration. For the formulation of reprojection errors we denote the transformations from camera i to world by $\mathbf { T } _ { \mathrm { W C } _ { i } }$ . Those do not constitute additional optimization variables and are calculated using $\mathbf { T } _ { \mathbb { W } \mathrm { I } }$ and $\mathbf { T } _ { \mathrm { I C } _ { i } }$ in practice.

At different points in time, we optimize a state

$$
{ \bf s } = \left\{ { \bf s } _ { \mathrm { k } } , { \bf s } _ { \mathrm { f } } , { \bf s } _ { \mathrm { l } } \right\} ,\tag{6}
$$

where $\mathbf { s } _ { \mathrm { k } }$ contains IMU poses for n older keyframes, s<sub>f</sub> contains IMU poses, velocities and biases of the m most recent frames, which possibly are also keyframes if they host landmarks, and s<sub>l</sub> contains landmarks. A graphical representation of the problem is shown in Fig. 5(a). Landmarks are stored relative to the keyframe where they were observed for the first time [16] and defined by a unit-length direction vector in the coordinate frame of the camera and an inverse distance to the landmark [6]. In the proposed system only keyframes host landmarks, which distinguishes them from regular frames.

![](images/2020_Basalt/3d00aab73b8d114cf8696e74326b488536c728ab73b58a7cfda92af3b20a7f3c.jpg)  
Fig. 3. Geometric interpretation of stereographic projection used to represent unit vectors. The two parameters define a point in the XY-plane ofthe coordinate system shown in blue. To obtain the corresponding 3D unit vector we cast a ray from $\left( 0 0 { - } 1 \right) ^ { - }$  and find an intersection with the unit sphere shown in black. Three example points are visualized in red, green and yellow, with dashed lines representing the rays intersecting with the sphere and arrows showing the resulting unit vectors.

1) Representation of Unit Vectors in 3D: In order to avoid the necessity of additional constraints for the optimization and to keep the number of optimiziation variables small, we parametrize the bearing vector in 3D space using a minimal representation, which is two-dimensional. In [3] the authors provide an extensive review of possible parametrizations and suggest a new parametrization based on SO(3) rotations that yields simple derivatives with respect to 2D increments.

In this work we use a parametrization based on stereographic projection that given 2D coordinates $( u , v ) ^ { \top }$ generates a unitlength bearing vector

$$
{ \binom { x } { y } } = { \binom { \eta u } { \eta v } } , \eta = { \frac { 2 } { 1 + u ^ { 2 } + v ^ { 2 } } } .\tag{7}
$$

This parametrization is efficient as it only uses simple operations such as multiplication and division (compared to trigonometric operations needed in [6]) and is defined for all u and v. A geometric interpretation is shown in Fig. 3. The only direction vector that cannot be represented with finite u, v is the negative Z-direction $( 0 ~ \mathrm { ~  ~ 0 ~ } - \bar { 1 } ) ^ { \top }$ . However, this is not a drawback in practice, as cameras usually have a limited field of view and cannot see points behind them.

2) Reprojection Error: The first cue we can use for motion estimation is the reprojection error. When point i that is hosted in frame $h ( i )$ is detected in target frame t at image coordinates $\mathbf { z } _ { i t }$ , the residual is defined as

$$
\mathbf { r } _ { i t } = \mathbf { z } _ { i t } - \pi _ { c ( t ) } ( \mathbf { T } _ { t } ^ { - 1 } \mathbf { T } _ { h ( i ) } \mathbf { q } _ { i } ( u , v , d ) ) ,\tag{8}
$$

$$
\mathbf { q } _ { i } ( u , v , d ) = \Big ( x ( u , v ) \quad y ( u , v ) \quad z ( u , v ) \quad d \Big ) ^ { \top } ,\tag{9}
$$

where $c ( t )$ is the index of the camera used to take frame t. The pose $\mathbf { T } _ { t }$ denotes $\mathbf { T } _ { \mathrm { W C } _ { c ( t ) } }$ at the time when frame t has been taken, and similarly for $\mathbf { T } _ { h ( i ) }$ . The first three entries of the homogeneous point coordinates $\mathbf { q } _ { i } ( u , v , d )$ are computed from the minimal representation $( u , v )$ as described in Section IV-B1, with an additional fourth entry $d ,$ the inverse distance. Since the projection function is independent of scale we do not have to normalize $\mathbf { q } _ { i } ,$ which makes this formulation numerically stable even when d is close or equal to zero.

3) IMU Error: The second cue for motion estimation is the IMU data. To deal with high frequency of the IMU measurements we preintegrate several consecutive IMU measurements into a pseudo-measurement. When adding an IMU factor between frame i and frame $j ,$ we compute pseudo-measurement $\Delta \mathbf { s } = ( \Delta \mathbf { R } , \Delta \mathbf { v } , \Delta \mathbf { p } )$ similar to [8], which we can use to formulate the residuals as

$$
\mathbf { r } _ { \Delta \mathbf { R } } = \mathrm { L o g } \left( \Delta \tilde { \mathbf { R } } \mathbf { R } _ { j } ^ { \top } \mathbf { R } _ { i } \right) ,\tag{10}
$$

$$
\mathbf { r } _ { \Delta \mathbf { v } } = \mathbf { R } _ { i } ^ { \top } ( \mathbf { v } _ { j } - \mathbf { v } _ { i } - \mathbf { g } \Delta t ) - \Delta \tilde { \mathbf { v } } ,\tag{11}
$$

$$
{ \bf r } _ { \Delta \mathbf { p } } = { \bf R } _ { i } ^ { \top } ( { \bf p } _ { j } - { \bf p } _ { i } - \frac { 1 } { 2 } { \bf g } \Delta t ^ { 2 } ) - \Delta \tilde { \bf p } ,\tag{12}
$$

where g is the gravity vector and R and p denote the rotation and translation components of T<sub>WI</sub>, respectively. These residuals have to be weighted with appropriate covariance matrix $\Sigma _ { i j }$ which can be calculated recursively. For more detailed information about the underlying physical model of the IMU and preintegration theory we refer the reader to the supplementary material.

4) Optimization and Partial Marginalization: For each new frame we minimize a non-linear energy that consists of reprojection terms, IMU terms and a marginalization prior $E _ { \mathrm { m } }$

$$
E = \sum _ { { i \in \mathcal { P } } \atop { t \in \mathrm { o b s } ( i ) } } { \bf r } _ { i t } ^ { \top } { \Sigma } _ { i t } ^ { - 1 } { \bf r } _ { i t } + \sum _ { ( i , j ) \in \mathcal { C } } { \bf r } _ { i j } ^ { \top } { \Sigma } _ { i j } ^ { - 1 } { \bf r } _ { i j } + E _ { \mathrm { m } } .\tag{13}
$$

The reprojection errors are summed over the set of points P and for each point i over the set obs(i) of frames where the point is observed, including its host frame. The set C contains pairs of frames which are connected by IMU factors.

The energy E is optimized using the Gauss-Newton algorithm. To constrain the problem size we fix the number of keyframe poses and consecutive states that we optimize at every iteration. When a new frame is added, there are n pose-only keyframes in $\mathbf { s } _ { \mathrm { k } }$ and the m newest frames including the newly added one in s . After optimizing, we perform a partial marginalization of the state to prevent the problem size from growing.

Two possible scenarios for marginalization are shown in Fig. 5. In the first one we marginalize out the oldest nonkeyframe. In this case we drop the landmark factors that have this frame as a target to maintain the sparsity of the problem. In the second case we have a new keyframe, so we marginalize out velocity and biases for this frame and one old keyframe with corresponding landmarks.

In both cases the marginalization is done on the linearized Markov blanket of the variables we want to remove, where the Markov blanket is a collection of incident states to those variables. The linearization H and b represent a distribution of the estimated state in the vector space of the increment ξ. If we split the increment $\pmb { \xi } = [ \pmb { \xi } _ { \alpha } ^ { \top } , \pmb { \xi } _ { \beta } ^ { \top } ] ^ { \top }$ into variables $\pmb { \xi } _ { c }$ to stay in the system and variables $\xi _ { \beta }$ to be marginalized, we can compute the parameters of the new distribution using the Schur complement,

![](images/2020_Basalt/0eb832b1d1d23dfe84a69c986e02341a27865c4cf44072fb78c34edf3ded669f.jpg)  
Fig. 4. Visual-inertial odometry subsystem proposed in Section IV. Projections of the landmarks with color-coded inverse distance used for estimating the position of the current frame are shown on the left. The results of local visual-inertial bundle adjustment are shown on the right. Keyframe poses with the associated landmarks are visualized in blue, current states and the estimated trajectory are visualized in red. Information about the keyframe poses in the local window is approximated using a set of non-linear factors as described in Section V and reused for global mapping.

$$
\mathbf { H } _ { \alpha \alpha } ^ { \mathrm { m } } = \mathbf { H } _ { \alpha \alpha } - \mathbf { H } _ { \alpha \beta } \mathbf { H } _ { \beta \beta } ^ { - 1 } \mathbf { H } _ { \beta \alpha } ,\tag{14}
$$

$$
{ \displaystyle { \bf b } _ { \alpha } ^ { \mathrm { m } } = { \bf b } _ { \alpha } - { \bf H } _ { \alpha \beta } { \bf H } _ { \beta \beta } ^ { - 1 } { \bf b } _ { \beta } } ,\tag{15}
$$

where we have split the original H and b into

$$
\mathbf { H } = \left[ \mathbf { H } _ { \alpha \alpha } \quad \mathbf { H } _ { \alpha \beta } \right] , \mathbf { b } = \left[ \mathbf { b } _ { \alpha } \right] .\tag{16}
$$

$\mathbf { H } _ { \alpha \alpha } ^ { \mathrm { m } }$ and ${ \bf b } _ { \alpha } ^ { \mathrm { m } }$ now define an energy term that only depends on $\xi _ { \alpha }$ and can be added to the total energy at the next iteration.

We use first-estimate Jacobians [10] to maintain the nullspace properties of the linearized marginalization prior. As soon as a variable becomes a part of the marginalization prior, its linearization point is fixed, and the Jacobian used to calculate H and b is evaluated at this linearization point, while the residuals are calculated at the current state estimate. Residuals already in the marginalization term have to be linearly approximated, thus not ${ \bf b } _ { \alpha } ^ { \mathrm { m } }$ , but ${ \bf b } _ { \alpha } ^ { \mathrm { m } } + { \bf H } _ { \alpha \alpha } ^ { \mathrm { m } } \delta _ { o }$ is added to the Gauss-Newton optimization once $\xi _ { \alpha }$ deviates by $\delta _ { \alpha }$ from the state used to calculate the residuals in ${ \bf b } _ { \alpha } ^ { \mathrm { m } }$

## V. VISUAL-INERTIAL MAPPING

The fixed-lag smoothing method for visual-inertial odometry (Fig. 4) presented in the previous section accumulates drift in the estimate due to the fixed linearization points outside the optimization window. A typical approach to eliminate such drift is to detect loop closures and incorporate loop-closing constraints into the optimization. We propose a two-layered approach which runs our visual-inertial odometry on the lower layer and bundle-adjustment on the visual-inertial mapping layer, where we additionally use non-linear factors that summarize the keyframe pose information from the odometry layer. BA optimizes the camera poses of keyframes and positions of keypoints. We implicitly detect loop closures using keypoint matching and achieve globally consistent mapping.

## A. Global Map Optimization

To get statistically independent observations we detect and match ORB [23] features (distinct from VIO points) between the keyframes in the global map optimization. This allows us to use the reprojection error function as defined in Eq. (8). Combining this reprojection error with the error terms from the recovered non-linear factors yields the objective function:

$$
E ^ { \mathrm { G } } ( \mathbf { s } ) = \sum _ { \stackrel { i \in \mathcal { P } } { t \in \mathrm { o b s } ( i ) } } \mathbf { r } _ { i t } ^ { \top } \Sigma _ { i t } ^ { - 1 } \mathbf { r } _ { i t } + E _ { \mathrm { n f r } } ( \mathbf { s } ) ,\tag{17}
$$

where $E _ { \mathrm { n f r } } ( \mathbf { s } )$ collects the error terms by the recovered nonlinear factors. These factors and their recovery are detailed in the following. The state s that we optimize on this global optimization layer includes the keyframe poses and the positions of the new landmarks (parametrized as in Section IV-B1).

We interface the global map optimization with the VIO layer at the keyframe poses. When a keyframe is marginalized out from the VIO we save the linearization of the Markov blanket (Fig. 5(c)) and marginalize all other variables except of keyframe poses. From this marginalization prior, we recover a set of non-linear factors on the keyframe poses that approximate the distribution stored in it.

## B. Non-Linear Factor Recovery

Non-linear factor recovery (NFR [15]) approximates a dense distribution stored in the linearized Markov blanket of the original factor graph with a different set of non-linear factors that yield a sparse factor graph topology. While the initial aim of NFR is to keep the computational complexity of SLAM optimization bounded, we use it to transfer information accumulated during VIO to our globally consistent visual-inertial map optimization.

By linearization of the residual function of a non-linear least squares problem Eq. (2), we obtain a multivariate Gaussian distribution $p ( \mathbf { s } ) \sim N ( \mu _ { \mathrm { o } } , \mathbf { H } _ { \mathrm { o } } ^ { - 1 } )$ in which the mean $\pmb { \mu } _ { 0 }$ equals the state estimate. We want to construct another distribution $p _ { \mathrm { a } } ( \mathbf { s } ) \sim N ( \mu _ { \mathrm { a } } , \mathbf { H } _ { \mathrm { a } } ^ { - 1 } )$ that well approximates the original distribution with a sparser factor graph topology.

We follow NFR [15] and minimize the Kullback-Leibler divergence (KLD) between the recovered distribution and the original distribution. More formally, we minimize

$$
\begin{array} { l } { { \displaystyle D _ { \mathrm { K L } } ( p ( \mathbf { s } ) | | p _ { \mathrm { a } } ( \mathbf { s } ) ) } } \\ { { \displaystyle \quad = \frac { 1 } { 2 } \left( \langle \mathbf { H } _ { \mathrm { a } } , \Sigma _ { \mathrm { o } } \rangle - \log \operatorname* { d e t } ( \mathbf { H } _ { \mathrm { a } } \Sigma _ { \mathrm { o } } ) + | | \mathbf { H } _ { \mathrm { a } } ^ { \frac { 1 } { 2 } } ( \mu _ { \mathrm { a } } - \mu _ { \mathrm { o } } ) | | ^ { 2 } - d \right) , } } \end{array}\tag{18}
$$

where $\pmb { \Sigma } _ { 0 } = \mathbf { H } _ { 0 } ^ { - 1 }$ and d is constant.

![](images/2020_Basalt/ae3d4282bbbc6034c1a5dcde131ddaf93ebf547403e2b5ee13068050a2408bec.jpg)  
Fig. 5. Factor graphs. (a) After marginalizing a frame, the system consists of n older keyframes $K _ { 1 } \ldots K _ { n }$ and the m − 1 most recent frames $F _ { 1 }$ and $F _ { 2 }$ (which could potentially also host landmarks and hence be keyframes). After a new frame has been added, the oldest velocity v and the oldest bias b are marginalized. If they do not belong to a keyframe (b), the whole frame including its pose T is marginalized. If they belong to a keyframe (c), another keyframe is selected for marginalization, including the landmarks hosted in it and its pose. In both cases, reprojection factors where the target frame is the marginalized frame are dropped In the latter case, reprojection factors from the marginalized frame to $F _ { 2 }$ are dropped to allow relinearization. Note that not all possible combinations of host and target frames for reprojection factors are shown.

For the ith non-linear factor that we want to recover, we need to define a residual function such that $\mathbf { r } _ { i } ( \mathbf { s } , \mathbf { z } _ { i } ) = \epsilon$ with $\epsilon \sim$ ${ \cal N } ( { \bf 0 } , { \bf H } _ { i } ^ { - 1 } )$ . NFR estimates the pseudo measurements $\mathbf { z } _ { i }$ and information matrices $\mathbf { H } _ { i }$ for the factors. Choosing $\mathbf { z } _ { i }$ such that $\mathbf { r } _ { i } ( \mu _ { \mathrm { o } } , \mathbf { z } _ { i } ) = \mathbf { 0 }$ induces $\mu _ { \mathrm { a } } = \mu _ { \mathrm { o } }$ which makes the third term of (18) vanish. To estimate $\mathbf { H } _ { i }$ we define

$$
\mathbf { J } _ { \mathrm { r } } = \left[ \begin{array} { l } { \vdots } \\ { \mathbf { J } _ { i } } \\ { \vdots } \end{array} \right] \mathbf { H } _ { \mathrm { r } } = \left[ \begin{array} { l l l } { \ddots } & & { } \\ & { \mathbf { H } _ { i } } & \\ & & { \ddots } \\ { 0 } & & & { \ddots } \end{array} \right] ,\tag{19}
$$

where $\mathbf { J } _ { \mathrm { r } }$ stacks the Jacobians of the defined residual functions with respect to the state, and H is a block diagonal matrix that consists of the $\mathbf { H } _ { i }$ <sub>i</sub> for the corresponding residual functions. This allows us to write $\mathbf { H } _ { \mathrm { a } } = \mathbf { J } _ { \mathrm { r } } ^ { \top } \mathbf { H } _ { \mathrm { r } } \mathbf { J } _ { \mathrm { r } }$ , and consequently, we can recover the information matrices $\mathbf { H } _ { i }$ by minimizing

$$
\begin{array} { r } { D _ { \mathrm { K L } } ( \mathbf { H } _ { \mathrm { r } } ) = \langle \mathbf { J } _ { \mathrm { r } } ^ { \top } \mathbf { H } _ { \mathrm { r } } \mathbf { J } _ { \mathrm { r } } , \boldsymbol { \Sigma } _ { 0 } \rangle - \log \operatorname* { d e t } ( \mathbf { J } _ { \mathrm { r } } ^ { \top } \mathbf { H } _ { \mathrm { r } } \mathbf { J } _ { \mathrm { r } } ) . } \end{array}\tag{20}
$$

For full-rank and invertible $\mathbf { J } _ { \mathrm { r } } ,$ [9], [15] showed that the following closed-form solution exists,

$$
\mathbf { H } _ { i } = ( \{ \mathbf { J } _ { \mathrm { r } } \mathbf { \Sigma } \mathbf { \Sigma } \mathbf { \Sigma } _ { \mathrm { o } } \mathbf { J } _ { \mathrm { r } } ^ { \top } \} _ { i } ) ^ { - 1 } ,\tag{21}
$$

where $\{ \}$ <sub>i</sub> denotes the corresponding diagonal block.

## C. Non-Linear Factors for Distribution Approximation

When we need to marginalize out a keyframe as shown in Fig. 5(c), we save the current linearization and marginalize out everything except the keyframe poses. This gives us a factor that densely connects all keyframe poses in the optimization window. We use it to recover non-linear factors between the marginalized keyframe and all other keyframes as shown in Fig. 6. We define the following residual functions:

$$
\begin{array} { r } { \mathbf { r } _ { \mathrm { r e l } } ( \mathbf { s } , \mathbf { z } _ { \mathrm { r e l } } ) = \mathrm { L o g } ( \mathbf { z } _ { \mathrm { r e l } } \mathbf { T } _ { j } ^ { - 1 } \mathbf { T } _ { i } ) , } \end{array}\tag{22}
$$

$$
\begin{array} { r } { { \bf r } _ { \mathrm { r p } } ( { \bf s } , { \bf z } _ { \mathrm { r p } } ) = \lfloor { \bf z } _ { \mathrm { r p } } { \bf R } _ { i } ^ { - 1 } ( 0 , 0 , - 1 ) ^ { \top } \rfloor _ { x y } , } \end{array}\tag{23}
$$

$$
\begin{array} { r } { { \bf r } _ { \mathrm { p o s } } ( { \bf s } , { \bf z } _ { \mathrm { p o s } } ) = { \bf z } _ { \mathrm { p o s } } - { \bf p } _ { i } , } \end{array}\tag{24}
$$

$$
\begin{array} { r } { { \bf r } _ { \mathrm { y a w } } ( { \bf s } , { \bf z } _ { \mathrm { y a w } } ) = \lfloor { \bf R } _ { i } { \bf z } _ { \mathrm { y a w } } \rfloor _ { y } , } \end{array}\tag{25}
$$

![](images/2020_Basalt/efead69137f66086d07902426e580f49758803776c7b4bc9f2a134d6b4531b1d.jpg)  
Fig. 6. Visualization of non-linear factor recovery. Left: Densely connected factor from marginalization saved from the VIO before removing a keyframe pose. Right: Extracted non-linear factors that approximate the distribution stored in the original factor.

where with $\left\lfloor \right\rfloor _ { x y }$ we denote x and y components of the vector and with z we denote the recovered measurements from the estimated state at the time of linearization. In our case ${ \bf z } _ { \mathrm { r e l } } = { \bf T } _ { i } ^ { - 1 } { \bf T } _ { j } \in \mathrm { S E } ( 3 ) , ~ { \bf z } _ { \mathrm { r p } } = { \bf R } _ { i } \in \mathrm { S O } ( 3 ) , ~ { \bf z } _ { \mathrm { p o s } } = { \bf p } _ { i } \in$ $\mathbb { R } ^ { 3 }$ and $\mathbf { z } _ { \mathrm { y a w } } = \mathbf { R } _ { i } ^ { - 1 } ( 1 0 0 ) ^ { \top } \in \mathbb { R } ^ { 3 }$

We recover pairwise relative-pose factors between the keyframe that we will remove and all other current VIO keyframes. For that keyframe we also recover roll-pitch, absolute position and yaw factors (Fig. 6). This gives us a full-rank invertible Jacobian $\mathbf { J } _ { \mathrm { r } }$ which means that we can use Eq. (21) for recovering information matrices for the factors.

Since yaw and absolute position are 4 unobservable states of the VIO, the only information we have there comes from the initial prior on the start pose. As we do not need this information for the global map we drop yaw and absolute position factors, and only take relative pose and roll-pitch factors for the map optimization. With these factors, the energy terms $E _ { \mathrm { n f r } } ^ { \mathrm { G } }$ become

$$
E _ { \mathrm { n f r } } ^ { \mathrm { G } } ( \mathbf { s } ) = \sum _ { ( i , j ) \in { \mathcal { R } } } \mathbf { r } _ { i j } ^ { \top } \mathbf { H } _ { i j } \mathbf { r } _ { i j } + \sum _ { i \in { \mathcal { P } } } \mathbf { r } _ { i } ^ { \top } \mathbf { H } _ { i } \mathbf { r } _ { i } ,\tag{26}
$$

where $\mathcal { R }$ is a set of all relative pose factors and $\mathcal { P }$ is the set of all roll-pitch factors.

$$
\mathrm { V I . ~ \ E V A L U A T I O N }
$$

To evaluate the presented approach we conduct evaluation on the EuRoC dataset [5] and compare it to other state-of-the-art systems. We present the evaluation for both our VIO subsystem and our full visual-inertial mapping approach. Our VIO runs the optimization in a local window of frames and provides a pose for every tracked frame, while the mapping system performs global map optimization for keyframes that were selected by the VIO. To measure the accuracy of the evaluated systems, we use the root mean square (RMS) of the absolute trajectory error (ATE) after aligning the estimates with ground truth.

TABLE I  
RMS ATE OF THE ESTIMATED TRAJECTORY IN METERS ON THE EUROC DATASET FOR SEVERAL DIFFERENT METHODS. IN THE UPPER PART WE SUMMARIZE THE RESULTS FOR THE VIO METHODS THAT RUN OPTIMIZATION IN A LOCAL WINDOW AND ESTIMATE THE POSE OF EVERY CAMERA FRAME. IN THE LOWER PART WE EVALUATE MAPPING METHODS THAT OPERATE ON ALL KEYFRAMES AND PERFORM GLOBAL MAP OPTIMIZATION. IN BOTH EVALUATIONS THE PROPOSED SYSTEM SHOWS THE LOWEST ERROR ON THE MAJORITY OF THE SEQUENCES AND OUTPERFORMS THE COMPETITORS. NOTE: THE V2\_03 SEQUENCE IS EXCLUDED FROM THE COMPARISON BECAUSE IT HAS MORE THAN 400 MISSING FRAMES FOR ONE OF THE CAMERAS
<table><tr><td>Sequence</td><td> $\mathrm { M H \_ 0 1 }$ </td><td> $\mathrm { M H } \_ { 0 2 }$ </td><td> $\mathrm { M H } \_ { 0 3 }$ </td><td> $\mathrm { M H \_ 0 4 }$ </td><td> $\mathrm { M H } \_ { 0 5 }$ </td><td> $\mathrm { V } 1 \_ 0 1$ </td><td> ${ \mathrm { V } } 1 \_ { 0 2 }$ </td><td> ${ \mathrm { V } } 1 \_ { 0 3 }$ </td><td> ${ \mathrm { V } } 2 \_ { 0 1 }$ </td><td> ${ \mathrm { V } } 2 \_ { 0 2 }$ </td></tr><tr><td>VI DSO [26], mono</td><td>0.06</td><td>0.04</td><td>0.12</td><td>0.13</td><td>0.12</td><td>0.06</td><td>0.07</td><td>0.10</td><td>0.04</td><td>0.06</td></tr><tr><td>OKVIS [13] mono</td><td>0.34</td><td>0.36</td><td>0.30</td><td>0.48</td><td>0.47</td><td>0.12</td><td>0.16</td><td>0.24</td><td>0.12</td><td>0.22</td></tr><tr><td>OKVIS [13] stereo</td><td>0.23</td><td>0.15</td><td>0.23</td><td>0.32</td><td>0.36</td><td>0.04</td><td>0.08</td><td>0.13</td><td>0.10</td><td>0.17</td></tr><tr><td>VINS FUSION [20] mono</td><td>0.18</td><td>0.09</td><td>0.17</td><td>0.21</td><td>0.25</td><td>0.06</td><td>0.09</td><td>0.18</td><td>0.06</td><td>0.11</td></tr><tr><td>VINS FUSION [20] stereo</td><td>0.24</td><td>0.18</td><td>0.23</td><td>0.39</td><td>0.19</td><td>0.10</td><td>0.10</td><td>0.11</td><td>0.12</td><td>0.10</td></tr><tr><td>IS VIO [9] stereo</td><td>0.06</td><td>0.06</td><td>0.10</td><td>0.24</td><td>0.19</td><td>0.06</td><td>0.10</td><td>0.26</td><td>0.08</td><td>0.21</td></tr><tr><td>Proposed  $\mathbf { V I O } ,$  stereo</td><td>0.07</td><td>0.06</td><td>0.07</td><td>0.13</td><td>0.11</td><td>0.04</td><td>0.05</td><td>0.10</td><td>0.04</td><td>0.05</td></tr><tr><td>VI SLAM [12] mono, KF</td><td>0.25</td><td>0.18</td><td>0.21</td><td>0.30</td><td>0.35</td><td>0.11</td><td>0.13</td><td>0.20</td><td>0.12</td><td>0.20</td></tr><tr><td>VI SLAM [12] stereo, KF</td><td>0.11</td><td>0.09</td><td>0.19</td><td>0.27</td><td>0.23</td><td>0.04</td><td>0.05</td><td>0.11</td><td>0.10</td><td>0.18</td></tr><tr><td>VI ORB-SLAM [19], mono, KF</td><td>0.07</td><td>0.08</td><td>0.09</td><td>0.22</td><td>0.08</td><td>0.03</td><td>0.03</td><td>X</td><td>0.03</td><td>0.04</td></tr><tr><td>Pure BA, stereo, KF</td><td>0.09</td><td>0.08</td><td>0.05</td><td>0.27</td><td>0.16</td><td>0.04</td><td>0.03</td><td>X</td><td>0.04</td><td>0.04</td></tr><tr><td>BA + Identity Factors, stereo, KF</td><td>0.08</td><td>0.07</td><td>X</td><td>0.34</td><td>0.15</td><td>0.04</td><td>0.03</td><td>0.56</td><td>0.05</td><td>0.04</td></tr><tr><td>Proposed VI Mapping, stereo, KF</td><td>0.08</td><td>0.06</td><td>0.05</td><td>0.10</td><td>0.08</td><td>0.04</td><td>0.02</td><td>0.03</td><td>0.03</td><td>0.02</td></tr></table>

## A. System Parameters

At the KLT tracking stage the image is divided into a regular grid with the cell size of 50 pixels. For each cell that has no point tracked from the previous frame, one feature point with the best FAST response is extracted (if it exceeds the threshold). With the resolution of the EuRoC dataset it results in 80–120 features tracked by the system at every point in time. At the VIO level we use a window of 7 old keyframes (poses) and 3 latest temporal states (poses, velocities and biases). The newest temporal state is selected as a keyframe if less than 70% of the KLT features are connected to the currently tracked points in the local map.

## B. Accuracy

The results of the evaluation are summarized in Table I. When considering visual-inertial odometry methods our system shows the best performance on eight out of ten sequences while the closest competitor (VI DSO [26]) shows the best results on five.

To evaluate the mapping part we compare it to the visualinertial version of ORB-SLAM [19], where the vision subsystem is very similar to the one proposed in our mapping layer (ORB keypoints). The main difference lies in the inertial part where ORB-SLAM uses preintegrated measurements between keyframes, while we use recovered non-linear factors that summarize IMU and visual tracking on the VIO layer.

The proposed system clearly outperform ORB-SLAM on the “machine hall” sequences where the large scale of the environment results in large time intervals between keyframes. On the “Vicon room” sequences the difference is smaller, since the rapid motion of the MAV that carries the camera in a small room results in many keyframes with small time intervals between them.

Qualitative results of reconstructed maps are shown in Fig. 1. With the proposed system we are able to reconstruct globally consistent gravity-aligned maps and recover keyframe poses even for segments where no matches between detected ORB features can be estimated.

## C. Factor Weighting

To evaluate the importance of the extracted factors and their proper weighting in the final mapping results we consider two alternative implementations. In the first one we do not use any factors and rely purely on the BA with ORB features. In the second one we extract the factors, but use identity weights (i.e. $\mathbf { H } _ { i j } = \mathbf { H } _ { i } = \mathbf { I }$ in Eq. (26)) for all of them, which is a typical approach for pose graph optimization [19], [20]. The evaluation results presented in Table I show that the system with the factor weights recovered according to Section V results in better accuracy and robustness when compared to those alternatives.

## D. Timing

The main source of timing improvement for the mapping stage is the fact that for a global optimization requires a 2.5 smaller state (no velocity or biases) compared to the naive IMU integration. In absolute numbers we test our system on an Intel E5-1620 CPU (4 cores, 8 virtual cores). Our implementation is highly parallel and utilizes all available CPU resources. For the VIO the average time per frame on the EuRoC sequences is 7.83 ms (largest: 9.4 ms on MH\_02; smallest: 5.5 ms in V1\_03). On average 11.5% of the frames are selected as keyframes and proceed to the mapping stage.

The timing of the mapping stage is provided in Table II. In particular, for the MH\_05 sequence (see Fig. 1, 2273 stereo frames, 114 seconds) the processing takes 19.2 seconds for VIO and 9.7 seconds for mapping for the entire sequence (around 4x faster than real-time playback).

TABLE II  
MEAN PROCESSING TIME IN MILLISECONDS OF THE MAPPING SUBSYSTEM ON EUROC SEQUENCES NORMALIZED (DIVIDED) BY THE NUMBER OF KEYFRAMES IN THE MAP
<table><tr><td>Total</td><td>Factor Extraction</td><td>Keypoint detection</td><td>Matching and Triangulation</td><td>Optimization (10 iterations)</td></tr><tr><td>52.8</td><td>3.6</td><td>6.4</td><td>23.1</td><td>19.7</td></tr></table>

## VII. CONCLUSIONS

In this letter we present a novel approach for visual-inertial mapping that combines the strengths of highly accurate visualinertial odometry with globally consistent keyframe-based bundle adjustment. We achieve this in a hierarchical framework that successively recovers non-linear factors from the VIO estimate that summarize the accumulated inertial and visual information between keyframes. VIO is formulated as fixed-lag smoothing which optimizes a set of active recent frames in a sliding window and keeps past information in marginalization priors. The accumulated VIO information between keyframes is extracted and retained for the visual-inertial mapping when a keyframe falls outside the window and is marginalized.

Compared to alternative approaches that use preintegrated IMU measurements between keyframes our system shows better trajectory estimates on a public benchmark. This formulation has the potential to reduce the computational cost of optimization by reducing the dimensionality of the state space and enable large-scale visual-inertial mapping. Integrating information from other sensor modalities or extending the system for multicamera settings are interesting directions for future research.

## REFERENCES

[1] S. Baker and I. Matthews, “Equivalence and efficiency of image alignment algorithms,” in Proc. IEEE Comput. Soc. Conf. Comput. Vision Pattern Recognit, 2001, p. I.

[2] T. Barfoot, State Estimation for Robotics. Cambridge, U.K.: Cambridge Univ. Press, 2017.

[3] M. Bloesch, M. Burri, S. Omari, M. Hutter, and R. Siegwart, “Iterated extended Kalman filter based visual-inertial odometry using direct photometric feedback,” Int. J. Robot. Res., vol. 36, no. 10, pp. 1053–1072, Sep. 2017.

[4] M. Bloesch et al., “A primer on the differential calculus of3D orientations,” Jun. 2016. [Online]. Available: https://arxiv.org/abs/1606.05285

[5] M. Burri et al., “The EuRoC micro aerial vehicle datasets,” Int. J. Robot. Res., vol. 35, no. 10, pp. 1157–1163, Jan. 2016.

[6] J. Civera, A. Davison, and J. Montiel, “Inverse depth parametrization for monocular SLAM,” IEEE Trans. Robot., vol. 24, no. 5, pp. 932–945, Oct. 2008.

[7] E. Eade, “Lie groups for computer vision,” Cambridge Univ., Cambridge, U.K., Tech. Rep., 2014. [Online]. Available: http://ethaneade.com/ lie\_groups.pdf

[8] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation,” in Proc. Robot., Sci. Syst, Jul. 2015. [Online]. Available: http://www.roboticsproceedings.org/rss11/p06.html

[9] J. Hsiung, M. Hsiao, E. Westman, R. Valencia, and M. Kaess, “Information sparsification in visual-inertial odometry,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2018, pp. 1146–1153.

[10] G. P. Huang, A. I. Mourikis, and S. I. Roumeliotis, “A first-estimates Jacobian EKF for improving SLAM consistency,” in Experimental Robotics. Berlin, Germany: Springer, 2009, pp. 373–382.

[11] E. S. Jones and S. Soatto, “Visual-inertial navigation, mapping and localization: A scalable real-time causal approach,” Int. J. Robot. Res., vol. 30, no. 4, pp. 407–430, Jan. 2011.

[12] A. Kasyanov, F. Engelmann, J. Stückler, and B. Leibe, “Keyframe-based visual-inertial online SLAM with relocalization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2017, pp. 6662–6669.

[13] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, “Keyframe-based visual–inertial odometry using nonlinear optimization,” Int. J. Robot. Res., vol. 34, no. 3, pp. 314–334, Dec. 2014.

[14] B. D. Lucas and T. Kanade, “An iterative image registration technique with an application to stereo vision,” in Proc. 7th Int. Joint Conf. Artif. Intell., 1981, pp. 674–679.

[15] M. Mazuran, W. Burgard, and G. D. Tipaldi, “Nonlinear factor recovery for long-term SLAM,” Int. J. Robot. Res., vol. 35, nos. 1–3, pp. 50–72, Jun. 2015.

[16] C. Mei, G. Sibley, M. Cummins, P. Newman, and I. Reid, “RSLAM: A system for large-scale mapping in constant-time using stereo,” Int. J. Comput. Vision, vol. 94, no. 2, pp. 198–214, Jun. 2010.

[17] J. Molnár, D. Chetverikov, and S. Fazekas, “Illumination-robust variational optical flow using cross-correlation,” Comput. Vision Image Understanding, vol. 114, no. 10, pp. 1104–1114, Oct. 2010.

[18] A. I. Mourikis and S. I. Roumeliotis, “A multi-state constraint Kalman filter for vision-aided inertial navigation,” in Proc. IEEE Int. Conf. Robot. Autom., Apr. 2007, pp. 3565–3572.

[19] R. Mur-Artal and J. D. Tardós, “Visual-inertial monocular SLAM with map reuse,” IEEE Robot. Autom. Lett., vol. 2, no. 2, pp. 796–803, Apr. 2017.

[20] T. Qin, J. Pan, S. Cao, and S. Shen, “A general optimization-based framework for local odometry estimation with multiple sensors,” Jan. 2019. [Online]. Available: https://arxiv.org/abs/1901.03638

[21] N. Roma, J. Santos-Victor, and J. Tomé, “A comparative analysis of crosscorrelation matching algorithms using a pyramidal resolution approach,” in Series in Machine Perception and Artificial Intelligence. Singapore: World Scientific, May 2002, pp. 117–142.

[22] E. Rosten, R. Porter, and T. Drummond, “Faster and better: A machine learning approach to corner detection,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 32, no. 1, pp. 105–119, Jan. 2010.

[23] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: An efficient alternative to sift or surf,” in Proc. Int. Conf. Comput. Vision, Washington, DC, USA, 2011, pp. 2564–2571.

[24] T. Schneider et al., “Maplab: An open framework for research in visualinertial mapping and localization,” IEEE Robot. Autom. Lett., vol. 3, no. 3, pp. 1418–1425, Jul. 2018.

[25] F. Steinbrücker, T. Pock, and D. Cremers, “Advanced data terms for variational optic flow estimation,” in Proc. Vision, Model., Visualization Workshop, Braunschweig, Germany, 2009, pp. 155–164.

[26] L. V. Stumberg, V. Usenko, and D. Cremers, “Direct sparse visual-inertial odometry using dynamic marginalization,” in Proc. IEEE Int. Conf. Robot. Autom., May 2018, pp. 2510–2517.

[27] V. Usenko, J. Engel, J. Stückler, and D. Cremers, “Direct visual-inertial odometry with stereo cameras,” in Proc. IEEE Int. Conf. Robot. Autom., May 2016, pp. 1885–1892.