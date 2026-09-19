# RD-VIO: Robust Visual-Inertial Odometry for Mobile Augmented Reality in Dynamic Environments

Jinyu Li , Xiaokun Pan , Gan Huang , Ziyang Zhang , Nan Wang , Hujun Bao , Member, IEEE, and Guofeng Zhang , Member, IEEE

Abstract—It is typically challenging for visual or visual-inertial odometry systems to handle the problems of dynamic scenes and pure rotation. In this work, we design a novel visual-inertial odometry (VIO) system called RD-VIO to handle both of these two problems. First, we propose an IMU-PARSAC algorithm which can robustly detect and match keypoints in a two-stage process. In the first state, landmarks are matched with new keypoints using visual and IMU measurements. We collect statistical information from the matching and then guide the intra-keypoint matching in the second stage. Second, to handle the problem of pure rotation, we detect the motion type and adapt the deferred-triangulation technique during the data-association process. We make the pure-rotational frames into the special subframes. When solving the visual-inertial bundle adjustment, they provide additional constraints to the purerotational motion. We evaluate the proposed VIO system on public datasets and online comparison. Experiments show the proposed RD-VIO has obvious advantages over other methods in dynamic environments.

Index Terms—Degenerate motion, dynamic environment, RANSAC, SLAM, VIO.

## I. INTRODUCTION

ISUAL-INERTIAL odometry (VIO) systems are crucial to many applications from virtual reality (VR), augmented reality (AR), drones, autonomous driving, and robotics. Most of the visual-inertial SLAM systems rely on VIO technique for sensor fusion and state estimation. Especially, a robust and lightweight VIO is very crucial for performing augmented reality on a mobile device. A bunch of algorithms have been developed to improve the accuracy and speed of VIO.

Compared to visual odometry, VIO systems can fuse visual and inertial information to achieve better robustness in complex environments, such as textureless scenes with dynamic objects. However, if there are large moving objects and degenerated motion (like long-time stopping or pure-rotation), traditional VIO systems still easily encounter robustness problems. Although a few methods [3] [2][52] [26] have been proposed to use semantic information to improve the tracking robustness in dynamic environments, the computational complexity is still a big problem to achieve real-time performance on a mobile device.

In this paper, we focus on robustifying a VIO system from two aspects: better moving keypoint removal and robust purerotation handling, while still keeping the system lightweight. To recognize moving keypoints, we propose a novel algorithm IMU-PARSAC which detects and matches keypoints in a twostage process. In the first stage, known landmarks are matched with new keypoints using both visual and IMU measurements. We collect error-statistics from the matching results, which then guide the intra-keypoint matching in the second stage. To handle pure-rotation, we detect the motion type for the incoming image frames. We adapt the deferred-triangulation technique during the data-association process, where we postpone the triangulation for landmarks under pure-rotational situations. The pure-rotational motion information is honored in our modified design of sliding-window in a way that it always keeps keyframes with sufficient translations. We make pure-rotational frames into special subframes. When solving the visual-inertial bundle adjustment, they provide additional constraints to the pure-rotational motion.

As shown in Fig. 1, the proposed VIO system RD-VIO can accommodate pure-rotational motions and large moving objects, which would easily lead to divergence on many other VIO/VI-SLAM systems, such as VINS-Mobile [25]. We test the proposed system and compared it with many state-of-the-art VIO systems in public datasets. The experimental results show that our proposed system not only produces accurate tracking results but also does so in a more robust manner.

The major contributions of this paper are as follows:

\- A novel IMU-PARSAC algorithm is proposed to detect and remove moving ourliers in dynamic scenes, which can obviously improve the tracking robustness.

\- A novel subframes strategy in the sliding window is proposed to efficiently reduce drift under pure rotational motion.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/1b14c604eff578f15814692c28a05854828c0c4e5168d2a572b23a896c27d4e3.jpg)  
Fig. 1. Proposed RD-VIO can robustly work in dynamic scenes with pure rotation motions, and outperforms some other SOTA VIO/VI-SLAM systems such as VINS-Mobile.

\- The source code of the whole system is released to benefit the community, including core VIO algorithm and an iOS project for mobile AR application.

## II. RELATED WORKS

For precise and dependable tracking in mobile augmented reality (AR) applications, especially within dynamic environments, it is imperative to devise robust visual-inertial odometry systems. Yet, conventional methods like VIO and SLAM sometimes falter in the face of intricate motions and everchanging scenarios, leading to tracking discrepancies. Consequently, scholars are delving into innovative strategies, including deep learning-infused VIO and SLAM, along with adaptive SLAM techniques, to better navigate these hurdles and yield more exact tracking outcomes.

## A. VIO and SLAM

Visual odometry and SLAM systems are popular in recent vision studies. [51] provides a useful review for visual-only SLAM systems. However, vision-only systems cannot recover the metric scale of the scene. In visual-inertial odometry (VIO), the inertial measurements are fused with visual measurements to recover the metric scale. According to their fusion technique, VIO systems can be roughly divided into filter-based and optimization-based.

Filter-based VIO MSCKF [27] [24] are early VIO systems based on Kalman filtering. The state vector of its filtering consists of a fixed number of frame poses. Landmarks and their observations are processed and marginalized for each update phase. So the amount of computation is bounded. ROVIO [4] is another filtering-based system that used photometric errors for visual observation. So the data association is integrated with the filter estimation process. It also demonstrates the use of bearing vectors in the parametrization of landmarks. R-VIO [17] is a novel robocentric VIO algorithm. Different from the standard world-centric algorithms, R-VIO provides more accurate estimates of relative motion in relation to a moving local frame, and gradually updates the global pose.

OpenVINS [14] is a newly open-source platform using MSCKF filter. The modular design makes it flexible to use and easy to expand. Open source datasets evaluation show its high precision and robustness.

In order to further improve the performance of VIO, some newly VIO systems like [1] use pre-built high-precision maps to improve accuracy greatly. And some others like RNIN-VIO [7] take advantage of the neural network of IMU navigation to improve robustness.

Optimization-based VIO OKVIS [21] is an optimizationbased system. It works in a sliding window fashion by adding new keyframes into the optimization and marginalizing old keyframes. The marginalization of the old frames linearizes old observations into priors terms and adds the prior into the optimization. VINS-Mono [34] and VINS-Fusion [35] [33] [34] are recent VI-SLAM systems. The frontend also uses keyframe-based bundle adjustment with a sliding window. The loop-closure in the backend can help cancel accumulated errors, thus achieve better precision. VI-ORB-SLAM [29] is a looselycoupled VI-SLAM system, meaning that its inertial measurements are not fused immediately with visual observations. For each new frame, visual observations are processed first, using the traditional VSLAM approach. Then the result is aligned with inertial measurements to get the metric result. In its recent evolution, ORB-SLAM3 [6] demonstrated a tightly-coupled system that produces astonishingly accurate results. However ORB-SLAM3 solves the full SLAM problem, meaning that early poses still get optimized by using later observations. This is simply impossible for real-time applications. VI-DSO [46] and DM-VIO [45] are VIO extensions of original DSO, which increase robustness and have true scale.

Deep learning based SLAM Recently, deep learning based methods are widely adopted into SLAM systems, and achieved amazing results. DROID-SLAM [43] uses a dense and accurate optical flow RAFT [42] as measurement, and builds an end-to-end network to perform bundle adjustment of pose and structure. PVO [50] has a further performance improvement by coupling panoramic semantic segmentation. NICE-SLAM [53], Vox-Surf [22], Vox-Fusion [49] and the latest ESLAM [19],

Co-SLAM [47] are typical neural implicit SLAM methods. With the powerful representation ability of neural implicit expression, these methods also show good performances both of tracking and mapping. But all of these methods depend on a large computation cost of CPU or GPU, which limits the application for AR.

## B. SLAM in Dynamic Environments

Robustness in dynamic environments is also a research hotspot in the SLAM field. Both traditional methods, such as RDSLAM [41] based on the assumption of dynamic object distribution, and deep learning-based methods like DynaSLAM [3], DynaSLAM II [2], and DynaFusion [31], have conducted in-depth research on this issue. In VIO/VI-SLAM systems, due to the independence of IMU measurement from the external environment, they have a certain level of robustness in dynamic scenes compared to pure visual SLAM systems, resulting in relatively less research. Despite many successful VIO/VI-SLAM systems, bad visual cues can still damage the tracking quality. Since VIO systems are using static landmarks, keypoints from moving objects can have adverse effects. Hence in highly dynamic extreme environments, even using an IMU cannot prevent the system from being affected by dynamic objects.

Typically, VIO systems rely on methods like the traditional robust estimator RANSAC [11]. However, when moving keypoints dominate the view, these RANSAC systems usually do not work well. RDSLAM [41] proposed an alternative method of evaluating model hypothesis. In their PARSAC algorithm, they exploits the locality of moving objects. Keypoints on these objects usually get clustered in a region so the model with mostly scattered keypoints will be elected as the background model. PARSAC works well for small-sized moving objects. However if the moving object is big enough to hijack the scene, the locality heruisic will fail. To get robust background model, some approaches rely on structure regularities like planes, nevertheless these approaches are primarily limited to particular scenarios. DynaVINS [40] introduced a robust bundle adjustment capable of discarding dynamic outliers by utilizing pose priors derived from IMU preintegration. We also use the strong prior of IMU measurement to efficiently estimate dynamic observation in a different way. Thanks to the rapid development of deep learning, some systems such as Mask-SLAM [20], DS-SLAM [52], Dynamic-VINS [31], SOF-SLAM [10], Mask-Fusion [38] use semantic segmentations to aid moving object handling. However, these methods are still very heavy-weight, which imposes another limitation on VIO applications.

## C. SLAM Under Degenerated Conditions

Degenerated movements can also be problematic, and people have noticed this problem in prior works. When the camera undergoes only negligible translation, there are few observations of the landmark depths, making it difficult to accurately estimate the camera’s position and orientation. To address this issue, some previous works have proposed different approaches. DT-SLAM [16] attempted to avoid triangulating new landmarks in order to prevent initialization with erroneous depths. Instead, the system relied on existing landmarks and their known positions to estimate camera motion. Another approach, described in [32], involves tracking rotation-only cameras with panoramas, resulting in a system with mixed geometry models. This allows the system to handle cases where the camera is not translating, as well as cases where it is. For the inertial measurements, stopped cases can be exploitable.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/6060be4d18be90882f25f3e458ebba8c2a4c35db4b128b9a7c37c761c39b03a2.jpg)  
Fig. 2. Pipeline of RD-VIO.

In IMU-only tasks, such as pedestrian navigation, one can detect full stops in the foot movement and use it for suppressing accumulation error. This technique is commonly known as the zero-velocity update (ZUPT) [30]. The idea is to constrain the integration because we know the total translation should be zero when stopped. LARVIO [48], a recent VIO system, employed the idea of ZUPT to improve performance. By modeling ZUPT as an elegant closed-form measurement update, it has successfully achieved a trade-off between computational efficiency and localization precision

There are also works that aim to address the degradation problem by utilizing geometric features in the environment. When there are a large number of similar point features in the scene, point features are prone to degradation, leading to increased uncertainty in estimation or even complete failure of estimation. PL-VINS [13] and PL-VIO [15] leverage a visual-inertial bundle adjustment to minimize separate reprojection errors for point features and line features in the visual measurement residuals. In addition, some works rely on structure of planes like RP-VIO [37] and PVIO [23], that uses planar features to increase robustness and accuracy in dynamic environments.

## III. APPROACH

We begin with a baseline VIO system, which is based on PVIO [23] but without using planar prior. The baseline VIO will also be used for comparison. The pipeline of our system is as shown in Fig. 2. On the top of this system, we made our modifications. We detect pure-rotations and triangulate landmarks properly, and then organize pure-rotational frames into subframes and optimize the poses accordingly.

## A. Sliding-Window VIO

Our system followed a sliding-window approach. So we first introduce a baseline VIO system with sliding window optimization (Baseline-VIO), and define most of the notations. The Baseline-VIO system works by keeping a recent number of keyframes in a window, running bundle adjustment to fuse visual and inertial measurements, and marginalizing out stale frames, as if a multi-frame window sliding along the time.

1) Sliding-Window Optimization: During the tracking, we keep a fixed number of recent keyframes and the landmarks observed in these frames. The states of these keyframes and landmarks will be refined with a visual-inertial bundle adjustment. The state vector of a frame is parametrized as $s _ { i } =$ $[ p _ { i } , q _ { i } , v _ { i } , b _ { g _ { i } } , b _ { a _ { i } } ]$ — the position, orientation, velocity, gyroscope bias and accelerometer bias correspondingly. Quaternion representation is used for $q _ { i }$ . We use inverse-depth parametrization [8] for landmarks. The state of a landmark $x _ { k }$ consists only a 1-dimensional inverse depth $d _ { k }$ . For the sake of simplicity, we assume a constant camera intrinsic matrix K, and an ideal configuration between camera and IMU where their relative rotation and translation can be ignored (the extrinsic parameter between them is equal to identity). For a landmark $x _ { k } ,$ , we denote its position on frame i as $u _ { i k }$ . This landmark is associated with a reference frame $r _ { k }$ and a reference keypoint position $u _ { r _ { k } k } .$ . So the 3D point corresponding to this landmark can be represented by $\begin{array} { r } { x _ { k } = \frac { 1 } { d _ { k } } C ( q _ { r _ { k } } ) K ^ { - 1 } \bar { u } _ { r _ { k } k } / \| K ^ { - 1 } \bar { u } _ { r _ { k } k } \| + p _ { r _ { k } } } \end{array}$ . Here $C ( q )$ denotes the rotation matrix of a quaternion q, and u¯ denotes the homogeneous vector ${ \bar { u } } = { \binom { u } { 1 } }$ . When $x _ { k }$ is observed in another frame $i \neq r _ { k } .$ , let Π be the homogeneous projection, we get the following reprojection error:

$$
E _ { \mathrm { r e p r o j } ( i , k ) } = \| \Pi [ K C ^ { \top } ( q _ { i } ) ( x _ { k } - p _ { i } ) ] - u _ { i k } \| ^ { 2 } .\tag{1}
$$

To process IMU measurements, we employ the method used in [23] to add IMU cost term:

$$
\begin{array} { l } { E _ { \mathrm { m o t i o n } ( i , j ) } = D ^ { 2 } ( q _ { j } , \hat { q _ { j } } ) } \\ { \qquad + \lVert v _ { j } - \hat { v _ { j } } \rVert ^ { 2 } + \lVert p _ { j } - \hat { p _ { j } } \rVert ^ { 2 } } \\ { \qquad + \lVert b _ { g _ { j } } - \hat { b _ { g _ { j } } } \rVert ^ { 2 } + \lVert b _ { a _ { j } } - \hat { b _ { a _ { j } } } \rVert ^ { 2 } . } \end{array}\tag{2}
$$

Here $\hat { q _ { j } } , \hat { v _ { j } } , \hat { p _ { j } } , \hat { b _ { g _ { j } } } , \hat { b _ { a _ { j } } }$ are the measurements of motion state at IMU frame j by IMU pre-integration. $D ( q _ { 1 } , q _ { 2 } ) = \| \mathrm { L o g } ( q _ { 2 } ^ { - 1 }$ $q _ { 1 } ) \parallel$ is the distance between two nearby quaternions $q _ { 1 }$ and $q _ { 2 }$ by recognizing their difference as a small purturbation in the underlying Lie-algebra. All the norms here must be derived using Mahalanobis distances, with the corresponding covariances. We skip the maths for computing covariances here, but a reader can refer to [12] for details.

The final bundle adjustment will solve for the states that minimize the total of all the reprojection errors as well as the motion measurement errors:

$$
\underset { \{ s _ { i } \} , \{ d _ { k } \} } { \arg \operatorname* { m i n } } \sum _ { i } \sum _ { k } E _ { \mathrm { r e p r o j } ( i , k ) } + \sum _ { i } E _ { \mathrm { m o t i o n } ( i , i + 1 ) } + E _ { \mathrm { m a r g } } .\tag{3}
$$

$E _ { \mathrm { r e p r o j } }$ and $E _ { \mathrm { m o t i o n } }$ are the cost terms of vision and IMU, respectively. $E _ { \mathrm { m a r g } }$ is a prior term which comes from marginalization.

Similar to [23], we marginalize out the states of an old keyframe as soon as this keyframe is out of sliding window to bound the computational complexity.

2) Initialization: The initialization of the VIO includes the pursuit of the gravity vector, the solution to the global scale, and the determination for the initial states. First, a sequence of initial frames are selected, and we do a visual-only SfM with these frames. The result gives the relative pose of these frames up to some arbitrary scale. Then the IMU measurements are aligned with the SfM results. From the alignment, we can solve for gravity vector and the initial scale using the method introduced in [36]. Finally, a complete bundle adjustment (3) is used for finding the best initial states.

3) Keypoint Tracking: We detect and track keypoints using KLT as described in [39]. If a keypoint already has an associated landmark, we predict its landing position on the next frame by projecting the landmark onto this new frame. And we use this position as the initial position for KLT tracking. The pose for the new frame, which has not been solved yet, is extrapolated by integrating IMU measurements since the last solved frame. To get rid of outlier matches, we estimate an essential matrix and a homography matrix using RANSAC [11]. Instead of choosing between two results, like in ORB-SLAM [28], we estimate the homography matrix after estimating the essential matrix. So the second (homography) RANSAC can take advantage of the matches obtained from the previous (essential) one. The essential matrix RANSAC is using a tight error threshold, which aimed to enforce binocular geometry relations. Meanwhile, the homography RANSAC uses a much larger error threshold, in the sense that for relatively small movements, movements of keypoints can be loosely described by a homography. This two-pass RANSAC solely relies on visual information, and can suffer from motion ambiguities. Therefore, we propose an IMU-PARSAC algorithm to make improvements, which will be introduced in the next section.

After keypoint tracking, a new frame will be registered with the sliding window. Let $s _ { i }$ be the new frame’s states, $\{ x _ { k } \}$ be the landmarks tracked in this frame. We solve the following visual-inertial PnP to get an intial estimation to $s _ { i } \colon$

$$
\underset { s _ { i } } { \arg \operatorname* { m i n } } \sum _ { k } E _ { \mathrm { r e p r o j } ( i , k ) } + E _ { \mathrm { m o t i o n } ( i - 1 , i ) } .\tag{4}
$$

A typical sliding-window based system, like OKVIS [21] or VINS-Mono [34], will conditionally mark the new frame as a keyframe. If it is a keyframe, it will be optimized with a bundle adjustment like (3). It will be appended to the back of the sliding-window, and the oldest frame will be marginalized out. Otherwise, it will be quickly purged to save computation cost. In the following sections, we introduce how we modify the keypoint tracking to reduce moving object matches and how we handle the new frame and the sliding-window to fight the low-translation problem.

## B. Outliers Detection and Removal

We introduce the IMU-PARSAC algorithm, leveraging IMU information to differentiate between moving elements and static backgrounds. This differentiation enhances the robustness of VIO tracking. Our dynamic outlier removal approach unfolds in two phases: an essential 3D-2D matching phase (IMU-PARSAC) and an optional 2D-2D matching phase as shown in Fig. 3. In the initial phase, we align static 3D landmarks from the map to the 2D keypoints of the newly captured image. IMU preintegration predicts the current pose, guiding the 3D-2D matching process. If landmarks are scant, new ones are derived from 2D-2D matches. After this, we collect error statistics from the 3D-2D phase, formulating dynamic thresholds for 2D-2D PARSAC. This strategy counters the variable errors stemming from moving objects. At its heart, our methodology seeks to weave IMU measurements into a robust parameter estimation algorithm framework, and synthesis capitalizes on the synergistic benefits of both the camera and IMU.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/76b1fca0dce92f65b25e2768924bbc8a657311058f24268f54b8fa622ee6efda.jpg)  
Fig. 3. Moving outlier detection and removal strategy: In the mandatory 3D-2D stages, the current frame obtains initial matches of 2D observations and 3D points based on optical flow tracking with the last frame. After the IMU-PARSAC algorithm, most outliers are filtered out. In the optional 2D-2D stage, the current frame and the key frames in the sliding window are matched frame by frame using the original PARSAC algorithm. The remaining dynamic outliers are removed through this multi-view cross-validation approach.

1) 3D-2D Matching Stage: Upon the arrival of a new frame i, we apply (4) for the initial $s _ { i } .$ So we need to match the 3D landmarks with the 2D keypoints on frame i. We are assuming that these landmarks were static at the time of triangulation. And outlier matches are due to false correspondences or objects start moving. Vision-only RANSAC algorithms can easily overlook the unwanted correspondences from the moving objects. Therefore we propose to use the pose prediction from integrating the IMU measurements. Our IMU-PARSAC algorithm followed the iterative scheme of the typical RANSAC [11] algorithm. For each iteration, after sampling a hypothesis it solves the model and creates the consensus correspondence set. Let q<sub>VIS</sub>, p<sub>VIS</sub> be the model fit from visual correspondences. At the same time, we have q<sub>IMU</sub>, p<sub>IMU</sub> predicted by applying pre-integration from frame i − 1 to frame i. We can obtain the visual-consensus set S<sub>VIS</sub> and motion-consensus set S<sub>IMU</sub> as:

$$
\begin{array} { r l } & { S _ { \mathrm { V I S } } = \{ \left( x _ { k } , u _ { i k } \right) \vert E _ { \mathrm { r e p r o j } ( i , k ) } ( q _ { \mathrm { V I S } } , p _ { \mathrm { V I S } } ) \le \epsilon _ { \mathrm { V I S } } ^ { 2 } \} , } \\ & { S _ { \mathrm { I M U } } = \{ \left( x _ { k } , u _ { i k } \right) \vert E _ { \mathrm { r e p r o j } ( i , k ) } ( q _ { \mathrm { I M U } } , p _ { \mathrm { I M U } } ) \le \epsilon _ { \mathrm { I M U } } ^ { 2 } \} . } \end{array}\tag{5}
$$

We then take their intersection $S _ { \mathrm { V I } } = S _ { \mathrm { V I S } } \cap S _ { \mathrm { I M U } }$ as the final consensus set for this iteration. Note that for each frame i, S<sub>IMU</sub> is not changing between IMU-PARSAC iterations. Therefore we can pre-compute $S _ { \mathrm { I M U } }$ and look for its “best” subset.

Now, in terms of the “best”, we no-longer use the number of correspondences $| S _ { V I } |$ or the total re-projection error $\sum E _ { \mathrm { r e p r o j } ( i , k ) }$ . Following the prior-based adaptive RANSAC algorithm proposed by [41], we evaluate the distribution of the inlier keypoints with the weighted covariance:

$$
\begin{array} { l } { \displaystyle \mathsf { C o v } ( S _ { \mathrm { V I } } ) } \\ { = \frac { \sum \lambda _ { k } } { ( \sum \lambda _ { k } ) ^ { 2 } - \sum \lambda _ { k } ^ { 2 } } \sum _ { k } \lambda _ { k } ( u _ { k } - \mathrm { B } [ u _ { k } ] ) ( u _ { k } - \mathrm { B } [ u _ { k } ] ) ^ { T } . } \end{array}\tag{6}
$$

$\mathrm { B } [ u _ { k } ]$ represent the center coordinate of the bin which 2D observation $u _ { k }$ located in, and $\lambda _ { k }$ is the inlier ratio of each bin as the confidence weight, but it is different from the PARSAC that the time prior is considered in our method.

In some highly dynamic scenes, it may not be sufficient to identify static landmarks based solely on the information from a single frame. In such cases, historical information is needed. This is because static landmarks in the scene can be triangulated and tracked stably for a longer period of time. To account for this, we take into consideration the observation time prior to the hypothesis evaluation step. Specifically, we collect the continuous observation time of each landmark and use the average time of all observations in each bin to measure the motion state of that bin. This is similar to the original PARSAC algorithm, which uses the proportion of inliers in each bin as the confidence weight. These weights determine the influence of each bin on the current model evaluation. To improve the accuracy of the weights, we add a prior time factor to the redefinition of the weight.

$$
w _ { i } ^ { t } = 1 - p ^ { 0 . 1 t } ,\tag{7}
$$

$$
\lambda _ { i } ^ { \prime } = w _ { i } ^ { t } \cdot \lambda _ { i } .\tag{8}
$$

In $( 8 ) , t$ represents the average observation time of all landmarks in $b _ { i }$ . We define the observation time as the number ofcontinuous frames that can observe the landmark. To measure the dynamic degree of the scene, we introduce a dynamic coefficient $p \in$ [0, 1]. This coefficient can be adjusted to fit a particular scene. A larger value of t indicates that the landmark is more likely to be static and should be given more weight than newly generated landmarks when evaluating hypotheses.

Then we can define the quality of the consensus set as:

$$
a ( S _ { \mathrm { V I } } ) = \sum \lambda _ { k } \cdot \sqrt { \operatorname* { d e t } ( \operatorname { C o v } ( S _ { \mathrm { V I } } ) ) } .\tag{9}
$$

The $S _ { \mathrm { V I } }$ with highest $a ( S _ { \mathrm { V I } } )$ will be selected as our final inlier set. Geometrically, it corresponds to the inlier set whose keypoints are most scattered on the image.

This combination of $S _ { \mathrm { V I } }$ and $a ( S _ { \mathrm { V I } } )$ has several implications: a) The difference between the vision-based pose estimation and IMU-based prediction impacts the number of correspondences in $S _ { \mathrm { V I } }$ . b) Even when there is a tie in correspondence count, the quality criteria selects the most spreaded subset of keypoints. But when a huge rigid body is moving in the view, the use of IMU prediction will help avoid creating correspondence on this body. Otherwise, the system could fixate on this moving object, thinking it is the static background, and following adrift. c) In case when visual estimation and IMU prediction are having a large difference, it suggests that either there are a lot of moving outliers moving around or the IMU prediction is inaccurate. There will be insufficient correspondences in $S _ { \mathrm { V I } }$ , leading us to the 2D-2D stage.

2) 2D-2D Matching Stage: In our sliding-window strategy, failing to track a landmark will lead to this landmark being marginalized. So we need to populate new landmarks to keep them at a sufficient number. The 2D-2D stage is used for this purpose. In this stage, we used the original PARSAC algorithm from [41]. However, near-degraded cases can still be challenging. Inside the PARSAC algorithm, we rely on the epipolar geometry of the matches. Since outliers are determined based on the epipolar distances, when keypoints are moving along the direction of the epipolar lines, these moving-object keypoints will be indistinguishable from the static background. Luckily, in the real world, noises in the motion will lead to errors in the epipolar distance. With the help of IMU data, we can identify moving keypoints in the 3D-2D stage without worrying about the motion characteristics. So we can train an epipolar distance threshold from the 3D-2D matches. And use the trained value for the 2D-2D stage thresholding.

Let $N _ { I }$ and $N _ { O }$ be the number of inliers and outliers from a 3D-2D stage. We sort the result matches according to the epipolar distances. Let $\{ d _ { 0 } ^ { + } , d _ { 1 } ^ { + } , \ldots , d _ { N _ { I } } ^ { + } \}$ be the epipolar distances of the inlier matches in non-descreasing order, $\{ d _ { 0 } ^ { - } , d _ { 1 } ^ { - } , \ldots , d _ { N _ { O } } ^ { - } \}$ be the distances of the outlier matches correspondingly, and $\lambda ^ { + } , \lambda ^ { - } \in [ 0 , 1 0 0 ]$ be the prechosen percentile value. We update the 2D-2D stage threshold $\epsilon _ { \mathrm { 2 D } }$ as:

$$
\epsilon _ { \mathrm { 2 D } } = \frac { d _ { \lfloor \lambda ^ { + } \cdot N _ { I } \rfloor } ^ { + } + d _ { \lfloor \lambda ^ { - } \cdot N _ { O } \rfloor } ^ { - } } { 2 } .\tag{10}
$$

-<sub>2D</sub> will be updated whenever $N _ { I } + N _ { O } \ge N _ { P } , N _ { P }$ is a predefined total number.

We do not triangulate the 2D-2D inlier matches immediately. Instead, we trace the historical matches. If a keypoint is observed L times previously, and be marked as an inlier for $L ^ { + }$ times whereas $L ^ { + } \geq \delta L$ , we regard it as a static keypoint and label it for triangulation. However, its triangulation can still be postponed, as we will introduce next.

## C. Pure-Rotation Detection and Delayed Triangulation

Due to the large noise ofthe IMU sensors from consumer-level phones, we design a vision-based method to detect pure-rotation. We add a third RANSAC pass in feature tracking. The 3 rd one is solving for a rotation matrix from the matches. Suppose there is a translation t between the latest two frames. The two frames are observing a common landmark p. Let D be the distance from p to the line where t belongs. From the geometry relation, as shown in Fig. 4, the angle between the two observations must satisfy

$$
\theta \leq 2 \arctan { \frac { \| t \| } { 2 D } } .\tag{11}
$$

Therefore, when $\| t \| \ll D ,$ , i.e. the translation is insignificant comparinig the the depth, the two observations will be nearly in-parallel. Conversely, if all the keypoints’ motion can be well described by a common rotation matrix, then the frame is likely performing a pure rotation. Based on this observation, we compare the angles between the bearing vectors of the matching keypoints. Let $R _ { i j }$ be the rotation matrix from frame j to frame i, $\left\{ \left( { { u } _ { i k } } , { { u } _ { j k } } \right) \right\}$ be the positions of the matching keypoint-pairs, the maximum angle of observations is computed as:

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/13f1d2cac211134b8f2f04de9eeceef468f3646cf53ed54762e544257139cfec.jpg)  
Fig. 4. Geometry illustration of our angle based pure-rotation detection. The maximum θ is realized when two rays-of-observation and the translation vector t forms an isosceles triangle.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/b6595065dfe7b965c95e68df8a78eb0ea8c910dff8a432cb6cfda86d03734c85.jpg)  
Fig. 5. Example for point cloud from tracking when camera is stopped. Blue points are DT landmarks. They are casted into points with a fake 1 m depth for visualization. And we can see normal landmarks (in red) are scarse in Baseline-VIO, because their depths are diverging. With DT, more keypoints can be tracked. SF-VIO on the otherhand, can keep the depths stable.

$$
\theta _ { \operatorname* { m a x } } = \operatorname* { m a x } \{ \langle \bar { u } _ { i k } , R _ { i j } \bar { u } _ { j k } \rangle \} .\tag{12}
$$

If $\theta _ { \mathrm { { m a x } } } \leq \theta _ { \mathrm { { r o t } } }$ , a pre-defined threshold, we tag the latest frame as “pure-rotational-frame”, or R-frame for short, otherwise, it is a “normal-frame” or an N-frame.

If one frame is an R-frame, it lacks depth observation of new landmarks. When new keypoints are detected from this frame, we choose to triangulate them into landmarks partially. We only record the originating frame $r _ { k }$ and the location $u _ { r _ { k } k }$ of a new landmark k, delaying the estimation of its depth $d _ { k }$ . Upon getting sufficient observations of the depths, we re-estimate and update these landmarks. This delayed-triangulation (DT) strategy is inspired by [16]. Fig. 5 shows how DT can help tracking more keypoints even for Baseline-VIO. But since Baseline-VIO cannot fully leverage the DT information in its bundle adjustment, in experiments, Baseline-VIO is not using any DT.

It worth mentioning that an R-frame is visually incapable of providing depth observations. We can extract some early position estimation, for example, through a VI-PnP. However, in some situations, like when the translation is small or the surrounding space is huge, depth observations can still be insufficient, leading to significant triangulation error. Therefore, our vision-based method fits well in this visual-inertial fusion context.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/71e448c0fa514c3ba4f392dcb896d3023f5291e9e378c1e4d73c06b5179ce65e.jpg)  
Fig. 6. Frame management rules for adding new frames at the tail of the sliding window. (a,d,g,i) are the initial cases while $^ { ( \mathrm { b , e , f , h , j } ) }$ are the results after new frames added. (c) demostrates R-frame compression introduced in Section III-D2.

## D. Sliding Window With Subframes

Like we introduced before, we cannot afford to fill the sliding window with R-frames, nor can we discard R-frames as they must be kept for continuous estimation of IMU biases. We introduce a subframe mechanism in our system, which allows a keyframe to carry a set of subframes, as illustrated in the lower branch of each case in Fig. 6 (the N-frame which they attached to known as keyframe). The system utilizes this subframe strategy to handle long sequences of R-frames. The design rationale behind the subframe mechanism will be gradually explained in this section.

1) Frame Management: The strategy of frame management is the key to our sliding window structure. We follow several rules to add the new frame into the sliding window. The following two properties are kept when adding a new frame:

\- The last keyframe in the sliding window is always an Nframe,

\- There will never be N-frames and R-frames mixed in the same subframe window.

Based on the type of frames in the latest subframe window and the type of the new frame, there will be $2 \times 2 = 4$ cases. Fig. 6 illustrates all the frame management rules. In the following text, if the subframes are R-frames and the new frame is an N-frame, this is called case RN for simplicity.

Assume there are already subframes in the latest keyframe. For the RR case, we will append the new frame to the end of the subframes. When there are consecutive new R-frame comes, a series of RR-case will happen so that the last subframe will be extended with R-frames. This is important since we can have a chain of IMU preintegrations between these R-frames. The positions of these R-frames will be constrained in a ZUPT-like manner, while the orientations can be aligned with either 3D landmarks or delay-triangulated bearing vectors. The detailed optimization for these R-subframes will be introduced later. And we will introduce our remedy to avoid excessively long R-subframes when the device is not translating for a while. In a word, the subframe of R-frames will allow us better modeling the transitioning IMU biases during a pure-rotational motion.

For case NN, since we do not lack any depth observations, the new N-frame can always be used as a keyframe. Yet, we want to save some computations, so we followed the idea used in VINS-Mono. When the number of N-subframes exceeds a predefined size $N _ { s } ,$ the new N-frame will be added as a keyframe, as shown in Fig. 6(f). Otherwise, it will be added as another subframe, as shown in Fig. 6(e).

For case RN, we first make the last R-subframe into a keyframe, then add the new N-frame as a keyframe. For case NR, we make the last N-subframe a keyframe, then add the new R-frame as a subframe to this keyframe. In this way, the two properties of the sliding window will always be ensured.

Besides the above cases, when the last keyframe has no subframes, the new frame will be added as a subframe regardless of its type. Also, when the number of tracked keypoints is below some threshold $N _ { t }$ , the new frame will be added as an N-keyframe anyway.

2) Bundle Adjustment: With the modified sliding window, the bundle adjustment will make use of the two properties mentioned above. When there is no new keyframe, we don’t run full BA. Instead, we optimize the states in the last subframe window for a quick update. If the last subframe window contains N-frames, we have sufficient translation, hence sufficient depth observations. The BA we used here is the same as (3), but with keyframes and landmarks observed in these keyframes fixed. In this case, only new landmarks observed in the last subframe window, as well as the states of these subframes, are refined. The result of this BA registers the new landmarks with the old landmarks, but we don’t have to solve the full sliding window because there will be at most $N _ { s }$ subframes in an N-type subframe window.

On the other hand, if the last subframe is filled with R-frames, we will deal with the chain of preintegrations for better IMU bias estimation. We give up estimating depths since R-frames means insufficient translation. Hence, we regularize the orientation of the subframes with the bearing vectors from delayed triangulation. The orientation error of subframe i when observing a delay-triangulated landmark k can be written as:

$$
E _ { \mathrm { r o t } ( i , k ) } = \left\| C ^ { \top } ( q _ { i } ) C ( q _ { r _ { k } } ) \frac { K ^ { - 1 } \bar { u } _ { r _ { k } k } } { \| K ^ { - 1 } \bar { u } _ { r _ { k } k } \| } - \frac { K ^ { - 1 } \bar { u } _ { i k } } { \| K ^ { - 1 } \bar { u } _ { i k } \| } \right\| ^ { 2 } .\tag{13}
$$

We also fix the keyframe pose and old landmarks in the optimization. So they can be used to constrain the poses of these subframes. Since the relative translation should be tiny, a ZUPT-like regularizer can also be used. For example $E _ { \mathrm { Z U P T } ( i ) } =$ $\| p _ { i } - p _ { i - 1 } \| ^ { 2 }$ . This is trivial but can be helpful when the number of landmarks is small.

The overall optimization for this R-type subframe window will be:

$$
\begin{array} { l } { \displaystyle \underset { \{ s _ { i } \} } { \arg \operatorname* { m i n } } \sum _ { i } \sum _ { k , d _ { k } \neq 0 } E _ { \mathrm { r e p r o j } ( i , k ) } + \sum _ { i } \sum _ { k , d _ { k } = 0 } E _ { \mathrm { r o t } ( i , k ) } } \\ { \displaystyle \qquad + \sum _ { i } E _ { \mathrm { m o t i o n } ( i , i + 1 ) } + \sum _ { i } E _ { \mathrm { Z U P T } ( i ) } . } \end{array}\tag{14}
$$

(c) PARSAC

Here, $d _ { k } = 0$ designates the landmarks that are delaytriangulated $( \mathrm { i } . \mathrm { e } . \ d _ { k }$ hasn’t been computed yet). Since all the keyframes are fixed, $E _ { \mathrm { m a r g } }$ will be constant, hence ignored in (14). $E _ { \mathrm { r e p r o j } } , E _ { \mathrm { r o t } }$ and $E _ { \mathrm { Z U P T } }$ all contributes to the stabilization of the subframe poses. Allowing the IMU-biases be better optimized through minimizing $E _ { \mathrm { m o t i o n } }$ . The regularization from $E _ { \mathrm { r o t } }$ and $E _ { \mathrm { Z U P T } }$ may temporarily lead to larger localization error in favor of robustness. However, experiments show that the compromise on accuracy is small. But we gain better stability during degenerate movements. In the example shown in Fig. 5, more landmarks are preserved with the help of R-frames.

Unlike the N-type subframe window, there is no number limit for R-frames in a subframe window. If there are too many of them, solving (14) will be slow. As a remedy, the subframe window is compressed when the total number of R-frames exceeds $N _ { r }$ . We evenly choose 1/3 of the R-frames and concatenate the preintegrations in between. Fig. 6(c) illustrates this operation when $N _ { r } = 3$ . Since (14) can produce good estimation to intermediate poses and IMU-biases. We won’t lose too much accuracy in this compression. Otherwise, the estimation of the IMU-biases will be more easily trapped into a local minimum in later optimizations.

When there are new keyframes added into the sliding window, we perform a full BA on all the keyframes. This is similar to (3). Except that for keyframes carrying R-type subframes, the chain of preintegrations is used instead. After BA, if the sliding window has more than $N _ { w }$ keyframes, we marginalize the keyframes from the oldest to the newest until there are $N _ { w }$ keyframe left. Since the subframes are not participating in this keyframe BA, they are removed when we marginalize a keyframe.

For the convenience of writing and ablation study, we name SF-VIO as the version with only subframe strategy based on our baseline VIO.

## IV. EXPERIMENTS

To evaluate the effectiveness of our proposed method and the robustness of the VIO system, we conducted a series of experiments. Regarding the dynamic outlier removal strategy, we qualitatively compared and analyzed IMU-PARSAC algorithm with other algorithms. To address the problem of system state estimation degradation caused by pure rotation, we investigated the detection performance of pure rotation and the system stability when the camera is stationary. Finally, we quantitatively compared our method with current state-of-the-art VIO/VI-SLAM algorithms on publicly available datasets.

We evaluated our method and other SOTA systems on two public datasets.

The EuRoC [5] dataset is a benchmark dataset for VIO and SLAM algorithms. It includes high-quality data captured by a micro aerial vehicle (MAV) equipped with a stereo camera and a synchronized IMU, covering various indoor scenarios. The dataset provides ground truth poses obtained from a motion-capture system, which enables the evaluation of the accuracy of the estimated trajectories.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/dc70ead96d8b0cb311cce026bf9a6c19f95ce5106a17ca9b09ed81a2db45fbd2.jpg)  
(a) IMU  
(b) RANSAC  
Fig. 7. Qualitatively comparison between several outlier removal methods: (a) using IMU pre-integration predicted pose to identify outliers (b) traditional robust etimator RANSAC (c) dynamic object distribution prior estimator PARSAC (d) our proposed IMU-PARSAC algorithm. We visualize the 2D observations and color them as green for inliers and red for outliers based on the inlier mask. Additionally, PARSAC and IMU-PARSAC are visualized their corresponding bins based on the confidence.

\- ADVIO [9] (Advanced Visual-Inertial Odometry) dataset is an open benchmark dataset designed for evaluating visual-inertia algorithms. It contains diverse real-world scenes, including different indoor/outdoor environments, lighting conditions, and dynamic object interferences, which can be used to evaluate the robustness, accuracy, and real-time performance of various algorithms.

## A. Outliers Removal

We conducted qualitative and quantitative evaluations of IMU-PARSAC on both handcrafted scenes and public datasets ADVIO. Our handcrafted scenes consisted ofstatic backgrounds and moving objects in the foreground, with some objects occasionally occluding a significant portion of the field of view to test the ability of IMU-PARSAC. Fig. 7 compares several outlier removal schemes across two different scenarios, including traditional robust estimator RANSAC, dynamic object distribution prior estimator PARSAC, and our proposed IMU-PARSAC. We also compared the ability to eliminate outliers in visual observations using IMU pre-integration predicted poses. We performed PnP geometric estimation on the 2D points visible in the current frame and the 3D points in the map to identify whether the observed 2D points correspond to moving objects.

The top row of Fig. 7 is from Sequence 01 of ADVIO [9], where the operator walks in a shopping center and rides an escalator. When on the escalator, the operator is not walking and the device maintains a gaze on the steps of the escalator. To accurately estimate the current state of the device, the 2D visual observations should all come from static structures in the environment. We visualized the results ofseveral outlier removal methods and found that neither IMU pre-integration predicted poses nor RANSAC could completely detect out the dynamic

Time (s)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/358ede98542d0a678fd2b08e9463403a1ace37f720ca4263cccea587d48a6a6f.jpg)  
Fig. 8. Heatmap of R and N-frames.Red for R-Frames and blue for N-Frames.

outlier (escalator). The bottom row shows a dynamic scene captured with an iPhone, where a chair in the middle ofthe image is moved forward, and all 2D observation points falling on the chair should be considered outliers. In this scenario, the method of IMU pre-integration can only remove a portion of the outliers, while RANSAC completely estimated an incorrect geometric model which led to a “hijacking” situation, as RANSAC can only consider the geometric model with the greatest number of fitted points as the final result. At the same time PARSAC algorithm makes assumptions about the inliers distribution, it is difficult to apply to more general scenarios, and it is challenging to completely remove dynamic points. Our IMU-PARSAC algorithm, utilizing the motion-consensus property of IMU, can find the correct inliers even in highly dynamic scenes.

The quantitative evaluation of the dynamic object removal strategy is in Section IV-D2.

## B. Detection ofPure Rotation

To carefully examine the pure-rotation detection and stabilization effect, we rely on the high-quality ground-truths from the EuRoC datasets. We computed the motion speed from the ground-truth data and plot the speed curve. For each R-frame detected, we add a red line indicating its time point. For all the sequences, there are long stopped periods. And our method was able to mark almost all the frames in these periods as R-frames. In fact, only a few frames in MH\_01\_easy, MH\_02\_easy, and MH\_05\_difficult are the outliers. They are due to moving objects appeared in the background. Besides stopped periods, we can see many speed local-minimums successfully detected as R-frames.

The scene appeared in MH sequences is large, and the overall motion speed in V1\_01\_easy and V2\_01\_easy is slow. Therefore we can see sparsely marked R-frames in a lot of the local-minimum points. To further examine the speed range for our pure rotation detection method, we draw the heatmaps of R-frames and N-frames for each sequence in Fig. 8. R-frames are distributed in the lower part of the speed range. Due to the large scale ofthe scene in MH sequences, the hot zone ofR-frames reaches slightly higher speed ranges for these sequences.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/48159e9ecc7da6100f8c1cfddc39f27285841edb05460c2bcfb09f0e0101967d.jpg)

MH\_04\_difficult (full)  
![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/cb02cd6d98e341a490c4a883939e347831f6d4c6971e136d4d38a7de606ff6f4.jpg)  
Fig. 9. Positioning error curves for the first 20 seconds of the sequence MH\_05\_difficult.

## C. Stabilization Effect

The main goal of the subframes is the better handling of lowtranslation scenarios. In the EuRoC dataset, all the sequences have a long period of stopping in the beginning. By running an algorithm and examine the behavior at these stopped cases, we can see if it can handle them well. To compare the results, we align the result trajectories at the starting point, then minimize their overall RMSEs by registering them with the best global rotation. This is done using the Umeyama algorithm [44]. Then we can compare the poses with the ground-truth and plot the curve of positioning error. In Fig. 9, we compare the error curves of SF-VIO and Baseline-VIO on sequence MH\_05\_difficult.

The error curve for Baseline-VIO is jittering when the drone is stopped: its position slowly drifts, then suddenly being “dragged” back to a better location, then drifts again. This drift is the consequence of missing depth observation – the positions of the landmarks are becoming more and more inaccurate, and the error in IMU integration is accumulating. Whenever there is a small purturbation in the translation, the keyframe is added into the sliding window. The bundle adjustment will quickly pick up the new depth observation and correct the pose of the new keyframe, resulting in the “jittering” result in its error curve. Sometimes, the lack of proper depth observation causes Baseline-VIO produces jumping results, as can be seen around 10th to 11th seconds in sequence MH\_05\_difficult. If the stopping continues, Baseline-VIO will be starved of reliable landmarks eventually.

On the other hand, SF-VIO was able to accommodate the stopping situation. As shown in in Fig. 10, We have visualized the speed curves of the groundtruth trajectories for each sequence, along with the detection results of the R-Frames. We can tell that the frames during the stopped time are identified as R-frames, hence are handled in the subframe window. The compression strategy was able to keep the subframe window short while keeping the IMU bias estimation stable. In both sequences, the algorithm can keep the tracked position “locked” to its place. Therefore, we can see a smooth and flat error curve during these periods.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/47356c8b5c2b804e805b8691aabb5de2f559f2ddccaabecfc0a2545c0ecd423e.jpg)  
Fig. 10. Speed plot for all the sequences with R-frames highlighted. Red lines denote the detected R-Frames and the speed curves are expressed in blue.

## D. Qualitative Comparison

Besides the above evaluations, we also compute the RMSE of the localization error for quantitative comparison. We compute the RMSE of each result using the tool evo.<sup>1</sup> It registers the trajectory result with the ground-truth by seeking the RMSEminimizing global rigid transform. For each sequence, we run an algorithm 10 times, and take the average RMSE as the final result. We compared SF-VIO with Baseline-VIO, VINS-Fusion, ORB-SLAM3, OKVIS, ROVIO, MSCKF and PVIO [23], etc. Except for VINS-Fusion and ORB-SLAM3, all other systems are VIO systems based on optimization or Kalman filter. VINS-Fusion is a complete VI-SLAM system which uses a sliding window based VIO frontend for tracking. PVIO is a VIO system using plane (structural) heuristics for regularizing the sliding window. Our system, on the other hand, uses motion heuristics. In addition, we use the third-party implementation of VIO based on MSCKF<sup>2</sup> for comparison.

Although there has been a lot of work focus on the SLAM systems in dynamic scenes, most of the work is based on pure visual methods, such as RDSLAM [41], DynaSLAM [3], DynaSLAMII [2], or based on RGBD input, such as DynaFusion [31]. This is quite different from our VIO system, so we do not consider VSLAM solutions in the experiments. At the same time, as we emphasize the robustness of a lightweight VIO system on mobile devices in dynamic scenes, recent approaches based on dynamic object segmentation, such as Dynamic-VINS [26], have high requirements for GPU computation and are difficult to achieve real-time performance even on PCs. Therefore, we do not compare our method with such approaches. DynaVINS [40] is the newest VIO system designed for handling dynamic scenes. We compared the robustness against our method in dynamic scenarios with it.

We evaluate two different versions of VINS-Fusion and ORB-SLAM3 with/without using loop closure and global bundle adjustment. To ensure a fair comparison in terms of accuracy and real-time performance, we first report the online pose estimation results of VINS-Fusion and ORB-SLAM3 without loop closure optimization or final global optimization. For this assessment, we disabled the loop closure and global bundle adjustment modules in backend. We denote this version with “VIO<sup>‡</sup>” which represents the real-time estimated pose derived from the frontend. Notably, the loop-closure component of ORB-SLAM3 is deeply coupled with the entire system, making it unfeasible to fully disable its loop-closure. We also report the results of full versions, which indicate the comprehensive use of all available information for pose optimization. We still use the real-time output camera poses of VINS-Fusion (full) for tracking accuracy evaluation, and found that the overall accuracy is slightly improved because of the loop closure optimization. It should be noted that ORB-SLAM3 (full) actually refines the whole camera trajectory by a post-processing, which indeed dramatically improves the RMSE but is meaningless for online applications (e.g. mobile AR applications).

1) EuRoC Datasets: Table I lists all the EuRoC RM-SEs we gathered on these algorithms. Comparing with Baseline-VIO, SF-VIO showed significant improvements on many sequences. Especially for sequence MH\_01\_easy and MH\_04\_difficult, thanks to the additional stabilization effect, the significant drifts are canceled. We did carefully tuned Baseline-VIO. And it turns out that even this baseline version can perform well on some of the sequences like V1\_02\_medium and V2\_02\_medium. However, the margin between SF-VIO and Baseline-VIO on these sequences are small. Even for V1\_02\_medium, SF-VIO is only falling 1.9cm behind. Among all the VIO systems tested, we can see that SF-VIO has top-tier accuracy with 6 of 11 sequences ranked within the top 3. The subframe window came as a regularizer for small translation, and it did not compromises accuracy too much.

As there are almost no moving objects in the EuRoC, our proposed dynamic object removal strategy theoretically cannot improve the system’s accuracy. On the other hand, any false detection of moving keypoints will lower the number of observations, resulting in a slight worse in RM-SEs, such as in MH\_03\_medium, MH\_04\_difficult, MH\_05\_difficult. However, we can see that RD-VIO can achieve higher RMSEs compared to on some sequences, such as MH\_02\_easy and V2\_01\_easy. The reason is that although the proposed dynamic object removal strategy is not designed specifically for EuRoC, it still can remove the matches with relatively large errors, thereby improving the system’s accuracy.

TABLE I  
TRACKING ACCURACY (RMSE IN METERS) ON THE EUROC DATASET
<table><tr><td>Algorithm</td><td>MH-01</td><td>MH-02</td><td>MH-03</td><td>MH-04</td><td>MH-05</td><td>V1-01</td><td>V1-02</td><td>V1-03</td><td>V2-01</td><td>V2-02</td><td>V2-03</td><td>AVG</td></tr><tr><td>RD-VIO</td><td>0.109</td><td>0.115</td><td>0.141</td><td>0.247</td><td>0.267</td><td>0.060</td><td>0.091</td><td>0.133</td><td>0.058</td><td>0.100</td><td>0.147</td><td>0.133</td></tr><tr><td>SF-VIO</td><td>0.109</td><td>0.147</td><td>0.131</td><td>0.190</td><td>0.240</td><td>0.056</td><td>0.101</td><td>0.134</td><td>0.066</td><td>0.089</td><td>0.162</td><td>0.130</td></tr><tr><td>Baseline-VIO</td><td>1.622</td><td>0.218</td><td>0.154</td><td>4.313</td><td>0.256</td><td>0.080</td><td>0.082</td><td>0.248</td><td>0.074</td><td>0.073</td><td>0.289</td><td>0.674</td></tr><tr><td>LARVIO</td><td>0.132</td><td>0.137</td><td>0.168</td><td>0.237</td><td>0.314</td><td>0.083</td><td>0.064</td><td>0.086</td><td>0.148</td><td>0.077</td><td>0.168</td><td>0.147</td></tr><tr><td>Open-VINS</td><td>0.111</td><td>0.287</td><td>0.181</td><td>0.182</td><td>0.365</td><td>0.059</td><td>0.084</td><td>0.075</td><td>0.086</td><td>0.074</td><td>0.145</td><td>0.150</td></tr><tr><td>VI-DSO</td><td>0.125</td><td>0.072</td><td>0.285</td><td>0.343</td><td>0.202</td><td>0.197</td><td>0.135</td><td>4.073</td><td>0.242</td><td>0.202</td><td>0.212</td><td>0.553</td></tr><tr><td>OKVIS</td><td>0.342</td><td>0.361</td><td>0.319</td><td>0.318</td><td>0.448</td><td>0.139</td><td>0.232</td><td>0.262</td><td>0.163</td><td>0.211</td><td>0.291</td><td>0.281</td></tr><tr><td>MSCKF</td><td>0.734</td><td>0.909</td><td>0.376</td><td>1.676</td><td>0.995</td><td>0.520</td><td>0.567</td><td></td><td>0.236</td><td></td><td></td><td>0.752</td></tr><tr><td>PVIO</td><td>0.129</td><td>0.210</td><td>0.162</td><td>0.286</td><td>0.341</td><td>0.079</td><td>0.093</td><td>0.155</td><td>0.054</td><td>0.202</td><td>0.290</td><td>0.182</td></tr><tr><td>DynaVINS</td><td>0.308</td><td>0.152</td><td>1.789</td><td>2.264</td><td></td><td></td><td>0.365</td><td></td><td></td><td></td><td></td><td>0.976</td></tr><tr><td>VINS-Fusion (VIO‡)</td><td>0.149</td><td>0.110</td><td>0.168</td><td>0.221</td><td>0.310</td><td>0.071</td><td>0.282</td><td>0.170</td><td>0.166</td><td>0.386</td><td>0.190</td><td>0.202</td></tr><tr><td>ORB-SLAM3 (VIO‡)</td><td>0.543</td><td>0.700</td><td>1.874</td><td>0.999</td><td>0.964</td><td>0.709</td><td>0.545</td><td>2.649</td><td>0.514</td><td>0.451</td><td>1.655</td><td>1.055</td></tr><tr><td>VINS-Fusion (Full)</td><td>0.178</td><td>0.105</td><td>0.143</td><td>0.216</td><td>0.362</td><td>0.066</td><td>0.287</td><td>0.169</td><td>0.131</td><td>0.226</td><td>0.173</td><td>0.187</td></tr><tr><td>ORB-SLAM3 (Full)</td><td>0.025</td><td>0.048</td><td>0.035</td><td>0.088</td><td>0.058</td><td>0.043</td><td>0.020</td><td>0.031</td><td>0.050</td><td>0.017</td><td>0.027</td><td>0.040</td></tr></table>

We highlight the top 3 results of each column in gold, silver, and bronze

Furthermore, the complete VI-SLAM systems (e.g., VINS-Fusion and ORB-SLAM3) generally combine VIO with loop closure and global bundle adjustment to eliminate the accumulated error. Nevertheless, SF-VIO can provide a decent VIO frontend for these complete SLAM systems. Also, comparing with PVIO, which uses multi-plane heuristics, the compromise of our pure-rotation regularization is relatively small.

It is worth noting that for some of the compared methods, due to their poor initialization performance, a longer time was required for system initialization, which resulted in relatively shorter evaluated trajectory lengths. While this may have benefited the final RMSEs comparison, it is a disadvantage for the stability of a VIO/SLAM system. For example, OpenVINS requires 40 seconds for initialization on MH\_01\_easy and 35 seconds on MH\_02\_easy.

Meanwhile, to evaluate the system’s operational efficiency, we also compared the running time on V1\_01\_easy with VINS-Mono. VINS-Mono is a sliding window optimizationbased SLAM system that has been open-sourced. Our system has a similar structure to it, and after aligning the parameters, it is easy to compare the computational time of each part with VINS-Mono. We measure the running time for each module of the system. We configured VINS-Mono with a sliding window size of 8 frames and deactivated its backend, ensuring a fair comparison between the two systems. Both VINS-Mono and RD-VIO were executed on a computer equipped with an Intel i7-7700 CPU @3.6 GHz and 16 GB of memory. The results for different modules are presented in Table II.

For Non-Keyframe PnP, the full subframe window bundle adjustment (BA) is not executed. Instead, only the latest subframe’s observations are used to quickly update the state. To maintain consistency, keyframes’ states and observed landmarks remain fixed during subframe window BA. VINS-Mono conducts sliding window BA for every frame. In contrast, our method conducts subframe sliding window BA for non-keyframes, which significant reduces the time of non-keyframe PnP. As for Keyframe BA, when a new keyframe enters the sliding window, the entire main sliding window, including all its subframes, undergoes optimization. This process requires a longer computation time compared to VINS-Mono. Implementing the improved marginalization strategy outlined in PVIO [23] allows for a more efficient approach, which allows us to effectively reduce the running time. Overall, the average computational time for all frames indicates that our system is more time-efficient than VINS-Mono.

TABLE II  
RUNNING TIME (MS) OF DIFFERENT MODULES IN VINS-MONO (FRONT-END) AND RD-VIO
<table><tr><td>Module</td><td>VINS-Mono</td><td>RD-VIO</td></tr><tr><td>Keypoint Tracking</td><td>8.34</td><td>7.47</td></tr><tr><td>Pre-Integration</td><td>0.44</td><td>0.04</td></tr><tr><td>Non-Keyframe PnP</td><td>17.78</td><td>1.27</td></tr><tr><td>Non-Keyframe Marg</td><td>0.68</td><td></td></tr><tr><td>IMU-PARSAC</td><td></td><td>1.07</td></tr><tr><td>Keyframe BA</td><td>19.18</td><td>30.9</td></tr><tr><td>Keyframe Marg</td><td>32.91</td><td>3.99</td></tr><tr><td>Keyframe Average</td><td>60.87</td><td>42.4</td></tr><tr><td>All frame Average</td><td>44.72</td><td>18.38</td></tr></table>

2) ADVIO Datasets: As a challenging dataset in real-world settings, ADVIO offers 23 diverse scenarios, encompassing indoor and outdoor environments, varying lighting conditions, and dynamic elements such as pedestrians and vehicles. We found our proposed algorithms performed well on the AD-VIO [9] datasets while most of the aforementioned algorithms didn’t survive including current SOTA full VI-SLAM system ORB-SLAM3 and recent DynaVINS specialized in handling dynamic environment. Beside our system, only VINS-Fusion and LARVIO were able to produce meaningful results. In addition to accuracy comparison, we measured the trajectory completeness. For each input image frame, if the visual tracking was able to give a meaningful pose output, it contributes to our completeness evaluation. When a system is initializing, lost, or drift away by a large distance, the output is discarded, making the trajectory incomplete.

TABLE III  
ACCURACY & COMPLETENESS ON THE ADVIO DATASET
<table><tr><td rowspan="2">sequence</td><td colspan="5">RMSE</td><td rowspan="2"></td><td colspan="3">Comp.(%)</td></tr><tr><td>SF-VIO</td><td>RD-VIOs1</td><td>RD-VIO</td><td>VINS-Fusion</td><td>LARVIO</td><td>RD-VIO</td><td>VINS-Fusion</td><td>LARVIO</td></tr><tr><td>01</td><td>2.177</td><td>1.956</td><td>1.788</td><td>2.339</td><td>5.049</td><td>97.9</td><td></td><td>59.6</td><td>80.8</td></tr><tr><td>02</td><td>1.679</td><td>2.090</td><td>1.695</td><td>1.914</td><td>4.242</td><td>96.8</td><td></td><td>68.2</td><td>62.9</td></tr><tr><td>03</td><td>2.913</td><td>2.270</td><td>2.690</td><td>2.290</td><td>4.295</td><td>98.7</td><td></td><td>70.1</td><td>57.3</td></tr><tr><td>04</td><td></td><td></td><td>2.860</td><td>3.350</td><td></td><td>89.9</td><td></td><td>71.2</td><td></td></tr><tr><td>05</td><td>1.385</td><td>1.366</td><td>1.263</td><td>0.938</td><td>2.034</td><td>95.1</td><td></td><td>67.7</td><td>61.6</td></tr><tr><td>06</td><td>2.837</td><td>3.107</td><td>2.497</td><td>11.005</td><td>8.201</td><td>97.7</td><td></td><td>70.0</td><td>56.8</td></tr><tr><td>07</td><td>0.559</td><td>0.567</td><td>0.548</td><td>0.912</td><td>2.369</td><td>87.7</td><td></td><td>82.2</td><td>44.5</td></tr><tr><td>08</td><td>2.075</td><td>2.009</td><td>2.151</td><td>1.136</td><td>2.078</td><td>90.6</td><td></td><td>64.5</td><td>69.5</td></tr><tr><td>09</td><td>0.332</td><td>2.488</td><td>2.281</td><td>1.063</td><td>3.168</td><td></td><td>94.7</td><td>69.1</td><td>94.6</td></tr><tr><td>10</td><td>1.997</td><td>1.700</td><td>2.128</td><td>1.847</td><td>4.742</td><td>99.2</td><td></td><td>73.9</td><td>97.6</td></tr><tr><td>11</td><td>4.103</td><td>4.496</td><td>3.986</td><td>18.760</td><td>5.298</td><td>99.6</td><td></td><td>70.4</td><td>82.0</td></tr><tr><td>12</td><td>2.084</td><td>2.032</td><td>1.951</td><td></td><td>1.191</td><td>99.4</td><td></td><td></td><td>64.6</td></tr><tr><td>13</td><td>3.227</td><td></td><td>2.899</td><td></td><td>1.324</td><td>97.5</td><td></td><td></td><td>97.6</td></tr><tr><td>14</td><td>1.524</td><td></td><td>1.532</td><td></td><td></td><td>94.4</td><td></td><td></td><td></td></tr><tr><td>15</td><td>0.779</td><td>0.772</td><td>0.780</td><td>0.944</td><td>0.851</td><td>94.2</td><td></td><td>68.9</td><td>96.4</td></tr><tr><td>16</td><td>0.986</td><td>0.954</td><td>0.991</td><td>1.289</td><td>2.346</td><td>98.0</td><td></td><td>68.1</td><td>92.7</td></tr><tr><td>17</td><td>1.734</td><td>1.862</td><td>1.657</td><td>1.235</td><td>1.569</td><td>99.9</td><td></td><td>70.0</td><td>98.3</td></tr><tr><td>18</td><td>1.171</td><td>1.057</td><td>1.164</td><td></td><td>3.436</td><td>99.1</td><td></td><td></td><td>98.7</td></tr><tr><td>19</td><td>3.256</td><td>2.740</td><td>3.154</td><td></td><td>2.010</td><td></td><td>59.4</td><td></td><td>98.6</td></tr><tr><td>20</td><td></td><td>6.960</td><td>7.013</td><td>10.433</td><td>16.441</td><td>99.6</td><td></td><td>68.6</td><td>98.2</td></tr><tr><td>21</td><td>8.962</td><td>8.432</td><td>8.534</td><td>11.004</td><td>13.142</td><td></td><td>99.3</td><td>73.1</td><td>97.9</td></tr><tr><td>22</td><td>4.686</td><td>4.498</td><td>4.548</td><td></td><td>8.104</td><td></td><td>99.8</td><td></td><td>98.7</td></tr><tr><td>23</td><td>6.631</td><td>5.085</td><td>6.486</td><td>4.668</td><td>9.389</td><td></td><td>99.6</td><td>70.4</td><td>98.0</td></tr></table>

We highlight the top 2 results of each row in gold and silver.

As the ADVIO dataset contains common dynamic scenes in some sequences, such as escalator, pedestrians, and trains, the results on this dataset can reflect the system’s robustness to dynamic scenarios. In addition to reporting the results of full RD-VIO system, we evaluated the effectiveness of our proposed two-stage dynamic object removal strategy by comparing it with SF-VIO, which completely disables dynamic object removal strategy, and RD-VIO<sup>s1</sup>, which only retains the IMU-PARSAC algorithm in the first stage. Since the completeness of trajectories depends on the system’s initialization and termination time, and above three VIO systems have no difference in the initialization stage, most of the completeness of trajectories are the same. So we only report the trajectory completeness of the full RD-VIO system.

Table III lists the accuracy and completeness results from the ADVIO datasets. Compared to SF-VIO without dynamic object removal strategies, RD-VIO showed significantly better RMSEs on ADVIO dataset, and achieved the best accuracy on most ofthe sequences in both RD-VIO<sup>s1</sup> and RD-VIO. Although RD-VIO<sup>s1</sup> outperformed RD-VIO on some sequences, stage 2, achieved higher accuracy in some more complex dynamic scene sequences such as seq04, seq07, and seq09. And there is no significant difference on other sequences, even though the accuracy was slightly lower than stage 1.

While VINS-Fusion ranks as the second-most accurate algorithm, its result completeness significantly trails that of the two other VIO algorithms. A lower completeness, which indicates late initialization or tracking loss, results in shorter trajectories. These abbreviated trajectories can inherently lead to a higher RMSE. Hence, relying exclusively on RMSE as a measure might not provide a holistic view and could be deemed misleading. Also, it failed on more sequences than LARVIO and RD-VIO. LARVIO could track most of the sequences and had a relatively good trajectory completeness thanks to its ZUPT scheme. However, dynamic objects still negatively affect their tracking quality. There are pedestrians and escalators in sequences 02-08. In sequence 04, the operator rides multiple escalators consecutively. LARVIO cannot handle these moving objects, therefore having a bad trajectory completeness or even failing to track. The same situation happened for sequences 11 and 12 where subway trains are moving in the picture. In comparison, RD-VIO was able to recognize these moving bodies and robustify its tracking. We can see RD-VIO producing much more complete trajectories comparing with the other two algorithms.

3) Online Comparison: In the previous section, various open-source algorithms were compared using pre-recorded data. However, since the two mainstream AR commercial software, ARKit<sup>3</sup> and ARCore,<sup>4</sup> can only directly access data from the device’s camera and IMU, and do not support pre-recorded data, it is impossible to directly evaluate our system using the evaluation data from the previous section. To facilitate a comparison between our work and ARKit/ARCore, we designed an online comparative experiment. The evaluation hardware configuration comprised two iPhone Xs and one Xiaomi Mi 8 smartphone. Three phones were tied together as close as possible on the same plane, facing the same direction. ARCore ran on the Xiaomi 8, ARKit ran on an iPhone X, and another iPhone X was used to record data for the execution of RD-VIO. Before the comparison, we performed an extrinsic calibration between the cameras of three smartphones and the VICON markers. We obtained the ground-truth camera trajectories with VICON. We simulated these scenarios using five cases (A0-A4). Case A0 features slow movement in a static scene. Cases A1 and A2 simulate rapid rotation, translation, and oscillation to test the tracking robustness under fast motion. Case A3 depicts pedestrians entering and exiting a room, creating a dynamic scene. Lastly, case A4 entailed manually occluding the camera for a certain period, which made tracking significantly more challenging. The experiment utilized the tracking accuracy and robustness [18] to evaluate the tracking quality of AR-based odometry and SLAM systems.

TABLE IV  
COMPARISON WITH ARKIT & ARCORE
<table><tr><td>Metrics</td><td>Sequence</td><td>ARKit</td><td>ARCore</td><td>RD-VIO</td></tr><tr><td rowspan="4">APE (mm)</td><td>A0</td><td>15.643</td><td>16.625</td><td>22.385</td></tr><tr><td>A1</td><td>28.361</td><td>23.973</td><td>28.902</td></tr><tr><td>A2</td><td>22.576</td><td>31.576</td><td>20.931</td></tr><tr><td>A3 A4</td><td>33.339 42.615</td><td>40.387 34.492</td><td>19.583</td></tr><tr><td rowspan="5">Robustness</td><td>A0</td><td>0.078</td><td>0.083</td><td>23.049 0.112</td></tr><tr><td>A1</td><td>0.142</td><td>0.120</td><td>0.145</td></tr><tr><td>A2</td><td>0.113</td><td>0.158</td><td>0.105</td></tr><tr><td>A3</td><td>0.167</td><td>0.202</td><td>0.098</td></tr><tr><td>A4</td><td>0.213</td><td>0.172</td><td>0.115</td></tr></table>

The best results of each row are highlighted in bold.

Table IV shows the absolute position error (APE) of 3 algorithms in millimeters and their corresponding robustness values, where smaller values indicate better performance. Compared to ARKit and ARCore, our system registers slightly larger APE in static scenes with typical camera motion (e.g. A0). However, it performs on par with ARKit and ARCore in fast-paced scenes (e.g. cases A1 & A2). In scenarios like case A3, which features moving people and foreground occlusions, RD-VIO boasts a substantially lower APE than ARKit and ARCore. This improved performance is attributed to the newly introduced subframes strategy and the IMU-PARSAC algorithm, which effectively reduce interference from dynamic objects and bolster pose robustness in degenerate motion. In situations where the camera experiences significant occlusion (e.g., case A4), RD-VIO outshines both ARKit and ARCore. Fig. 11 shows the trajectories generated by the mentioned algorithms and the trajectories recorded by VICON. It can be clearly observed that RD-VIO achieves more stable and robust tracking in such challenging scenarios. It is crucial to acknowledge that both ARKit and ARCore are comprehensive VI-SLAM systems. They’ve benefited from extensive engineering optimizations, spanning hardware, software, and chip-level enhancements. In contrast, our research centers on crafting a compact visual-inertial odometry system that strikes a balance between being lightweight and robustness.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/d2a1ab1110831846cdf0b28780f9b55852d340e4d45ddb0496e4e64d5d1756f5.jpg)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/9962411f5c2075877cd307a014a91073b6ddd2522e77826d6af3402427e8dcdb.jpg)  
Fig. 11. Trajectories of: VICON, ARCore, ARKit, RD-VIO on A3 (left) and A4 (right). To ensure clear visualization, the overlapping area is manually faded.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/aecd14863d32d3b096d23739f8adf168c4f3cf5f1582eca914f352ce7b4c0a79.jpg)  
(a)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/a9aa71b4ab655fb0f00a70e715d8fcc9929f04324f54779965b6d7b6e3b59648.jpg)  
(b)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/43c29b19c49fe9de3cab67a9cba09b47c7ba0e1361e0a9b089608d81c82b1efc.jpg)  
(c)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/1fc334c2535516ffc6df311c37cdc47b4625efec56b6ad0a4bfd3da986632beb.jpg)  
(d)  
Fig. 12. AR effect on a mobile phone: (a)-(b) kangaroo in the wild. (c)-(d) try the product before purchase.

4) Mobile AR Application: We deploy RD-VIO to iOS platform and develop a simple AR demo to show its accuracy and robustness. We use 30 Hz images data with resolution 640 × 480 and IMU data which consists of angular velocity and acceleration with 100 Hz captured by the iPhone X. RD-VIO can run in real-time on mobile devices. A virtual cube and some other virtual objects are inserted into the real scenes. Fig. 12 shows two AR examples. We also compare it against with VINS-Mobile, which is one of the best open source mobile AR system. Both of them run on iPhone X. The experimental results show the superiority of RD-VIO both on pure rotation conditions and dynamic scenes 1. Please refer to the supplementary video for the complete results with comparisons.

## V. CONCLUSION

In this article, we propose a robust and novel VIO system which can efficiently handle dynamic scenes and pure rotational motion. By using IMU-PARSAC algorithm, dynamic feature points are removed in a two-state process. This method enables our system to effectively respond to drastic scenarios changes. For pure rotation motion, we design a subframe structure and use deferred-triangulation technique. Both of which bring us a significant improvement in degenerate motion scenes.

We have achieved obvious better results than the baseline on EuRoc and ADVIO datasets, which proves the effectiveness of our system. Our algorithm also has a good performance on computation cost and can run on a mobile device in real-time. The AR demo on the iPhone X further demonstrates the robustness of the algorithm in challenging scenarios. It also illustrates the ability of the algorithm in the field of applications for mobile AR.

Our system still has some limitations. It could not work well when devices are in extremely challenging scenes for a long time. Especially, when there are no valid visual observations as input, our system will lose tracking inevitably. In this condition, combining some other algorithms maybe helpful, such as pure inertial odometry or wireless tracking.

## ACKNOWLEDGMENTS

The authors would like to thank Xinyang Liu for his kind help in data collection. Thanks to Danpeng Chen, Weijian Xie and Shangjin Zhai for their kind help in algorithm fine-tuning and evaluation.

## REFERENCES

[1] H. Bao et al., “Robust tightly-coupled visual-inertial odometry with prebuilt maps in high latency situations,” IEEE Trans. Vis. Comput. Graph., vol. 28, no. 5, pp. 2212–2222, May 2022.

[2] B. Bescos, C. Campos, J. D. Tardós, and J. Neira, “DynaSLAM II: Tightlycoupled multi-object tracking and SLAM,” IEEE Trans. Robot. Autom., vol. 6, no. 3, pp. 5191–5198, Jul. 2021.

[3] B. Bescos, J. M. Fácil, J. Civera, and J. Neira, “DynaSLAM: Tracking, mapping, and inpainting in dynamic scenes,” IEEE Trans. Robot. Autom., vol. 3, no. 4, pp. 4076–4083, Oct. 2018.

[4] M. Bloesch, M. Burri, S. Omari, M. Hutter, and R. Siegwart, “Iterated extended Kalman filter based visual-inertial odometry using direct photometric feedback,” Int. J. Robot. Res., vol. 36, no. 10, pp. 1053–1072, 2017.

[5] M. Burri et al., “The EuRoC micro aerial vehicle datasets,” Int. J. Robot. Res., vol. 35, no. 10, pp. 1157–1163, 2016.

[6] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. Montiel, and J. D. Tardós, “ORB-SLAM3: An accurate open-source library for visual, visual–inertial, and multimap SLAM,” IEEE Trans. Robot., vol. 37, no. 6, pp. 1874–1890, Dec. 2021.

[7] D. Chen, N. Wang, R. Xu, W. Xie, H. Bao, and G. Zhang, “RNIN-VIO: Robust neural inertial navigation aided visual-inertial odometry in challenging scenes,” in Proc. IEEE Int. Symp. MixedAugmented Reality, 2021, pp. 275–283.

[8] J. Civera, A. J. Davison, and J. M. M. Montiel, “Inverse depth parametrization for monocular SLAM,” IEEE Trans. Robot., vol. 24, no. 5, pp. 932–945, Oct. 2008.

[9] S. Cortés, A. Solin, E. Rahtu, and J. Kannala, “ADVIO: An authentic dataset for visual-inertial odometry,” in Proc. Eur. Conf. Comput. Vis., 2018, pp. 419–434.

[10] L. Cui and C. Ma, “SOF-SLAM: A semantic visual SLAM for dynamic environments,” IEEE Access, vol. 7, pp. 166528–166539, 2019.

[11] M. A. Fischler and R. C. Bolles, “Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography,” Commun. ACM, vol. 24, no. 6, pp. 381–395, Jun. 1981.

[12] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “On-manifold preintegration for real-time visual–inertial odometry,” IEEE Trans. Robot., vol. 33, no. 1, pp. 1–21, Feb. 2017.

[13] Q. Fu et al., “PL-VINS: Real-time monocular visual-inertial SLAM with point and line features,” 2020, arXiv:2009.07462.

[14] P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, and G. Huang, “OpenVINS: A research platform for visual-inertial estimation,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 4666–4672.

[15] Y. He, J. Zhao, Y. Guo, W. He, and K. Yuan, “PL-VIO: Tightly-coupled monocular visual–inertial odometry using point and line features,” Sensors, vol. 18, no. 4, 2018, Art. no. 1159.

[16] D. C. Herrera, K. Kim, J. Kannala, K. Pulli, and J. Heikkilä, “DT-SLAM: Deferred triangulation for robust SLAM,” in Proc. 2nd Int. Conf. 3D Vis., 2014, pp. 609–616.

[17] Z. Huai and G. Huang, “Robocentric visual-inertial odometry,” Int. J. Robot. Res., vol. 41, no. 7, pp. 667–689, 2022.

[18] L. Jinyu, Y. Bangbang, C. Danpeng, W. Nan, Z. Guofeng, and B. Hujun, “Survey and evaluation of monocular visual-inertial SLAM algorithms for augmented reality,” Virtual Reality Intell. Hardware, vol. 1, no. 4, pp. 386–410, 2019.

[19] M. M. Johari, C. Carta, and F. Fleuret, “ESLAM: Efficient dense SLAM system based on hybrid representation of signed distance fields,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 17408–17419.

[20] M. Kaneko, K. Iwami, T. Ogawa, T. Yamasaki, and K. Aizawa, “Mask-SLAM: Robust feature-based monocular slam by masking using semantic segmentation,” in Proc. IEEE Int. Conf. Comput. Vis. Pattern Recognit. Workshops, 2018, pp. 371–3718.

[21] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, “Keyframe-based visual–inertial odometry using nonlinear optimization,” Int. J. Robot. Res., vol. 34, no. 3, pp. 314–334, 2015.

[22] H. Li, X. Yang, H. Zhai, Y. Liu, H. Bao, and G. Zhang, “Vox-Surf: Voxelbased implicit surface representation,” IEEE Trans. Vis. Comput. Graph., to be published, doi: 10.1109/TVCG.2022.3225844.

[23] J. Li, B. Yang, K. Huang, G. Zhang, and H. Bao, “Robust and efficient visual-inertial odometry with multi-plane priors,” in Pattern Recognit. Comput. Vis., Springer International Publishing, Cham, 2019, pp. 283–295.

[24] M. Li and A. I. Mourikis, “Improving the accuracy of EKF-based visualinertial odometry,” in Proc. IEEE Int. Conf. Robot. Automat., 2012, pp. 828–835.

[25] P. Li, T. Qin, B. Hu, F. Zhu, and S. Shen, “Monocular visual-inertial state estimation for mobile augmented reality,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2017, pp. 11–21.

[26] J. Liu, X. Li, Y. Liu, and H. Chen, “RGB-D inertial odometry for a resourcerestricted robot in dynamic environments,” IEEE Trans. Robot. Autom., vol. 7, no. 4, pp. 9573–9580, Oct. 2022.

[27] A. I. Mourikis and S. I. Roumeliotis, “A multi-state constraint Kalman filter for vision-aided inertial navigation,” in Proc. Proc. IEEE Int. Conf. Robot. Automat., 2007, pp. 3565–3572.

[28] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardós, “ORB-SLAM: A versatile and accurate monocular SLAM system,” IEEE Trans. Robot., vol. 31, no. 5, pp. 1147–1163, Oct. 2015.

[29] R. Mur-Artal and J. D. Tardós, “Visual-inertial monocular SLAM with map reuse,” IEEE Trans. Robot. Autom., vol. 2, no. 2, pp. 796–803, Apr. 2017.

[30] J.-O. Nilsson, A. K. Gupta, and P. Händel, “Foot-mounted inertial navigation made easy,” in Proc. Int. Conf. Indoor Positioning Indoor Navigation, 2014, pp. 24–29.

[31] S. Owada and J. Fujiki, “Dynafusion: A modeling system for interactive impossible objects,” in Proc. 6th Int. Symp. Non-Photorealistic Animation Rendering, 2008, pp. 65–68.

[32] C. Pirchheim, D. Schmalstieg, and G. Reitmayr, “Handling pure camera rotation in keyframe-based SLAM,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2013, pp. 229–238.

[33] T. Qin, S. Cao, J. Pan, and S. Shen, “A general optimizationbased framework for global pose estimation with multiple sensors,” 2019, arXiv:1901.03642.

[34] T. Qin, P. Li, and S. Shen, “VINS-Mono: A robust and versatile monocular visual-inertial state estimator,” IEEE Trans. Robot., vol. 34, no. 4, pp. 1004–1020, Aug. 2018.

[35] T. Qin, J. Pan, S. Cao, and S. Shen, “A general optimizationbased framework for local odometry estimation with multiple sensors,” 2019, arXiv:1901.03638.

[36] T. Qin and S. Sheen, “Robust initialization of monocular visual-inertial estimation on aerial robots,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2017, pp. 4225–4232.

[37] K. Ram, C. Kharyal, S. S. Harithas, and K. M. Krishna, “RP-VIO: Robust plane-based visual-inertial odometry for dynamic environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 9198–9205.

[38] M. Runz, M. Buffier, and L. Agapito, “MaskFusion: Real-time recognition, tracking and reconstruction of multiple moving objects,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2018, pp. 10–20.

[39] J. Shi and Tomasi, “Good features to track,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 1994, pp. 593–600.

[40] S. Song, H. Lim, A. J. Lee, and H. Myung, “DynaVINS: A visual-inertial SLAM for dynamic environments,” IEEE Trans. Robot. Autom., vol. 7, no. 4, pp. 11523–11530, Oct. 2022.

[41] W. Tan, H. Liu, Z. Dong, G. Zhang, and H. Bao, “Robust monocular SLAM in dynamic environments,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2013, pp. 209–218.

[42] Z. Teed and J. Deng, “Raft: Recurrent all-pairs field transforms for optical flow,” in Proc. Eur. Conf. Comput. Vis., Springer, 2020, pp. 402–419.

[43] Z. Teed and J. Deng, “DROID-SLAM: Deep visual slam for monocular, stereo, and RGB-D cameras,” in Proc. Adv. Neural Inf. Process. Syst., Curran Associates, Inc., 2021, pp. 16558–16569.

[44] S. Umeyama, “Least-squares estimation of transformation parameters between two point patterns,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, pp. 376–380, Apr. 1991.

[45] L. von Stumberg and D. Cremers, “DM-VIO: Delayed marginalization visual-inertial odometry,” IEEE Robot. Automat. Lett., vol. 7, no. 2, pp. 1408–1415, Apr. 2022.

[46] L. Von Stumberg, V. Usenko, and D. Cremers, “Direct sparse visual-inertial odometry using dynamic marginalization,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 2510–2517.

[47] H. Wang, J. Wang, and L. Agapito, “Co-SLAM: Joint coordinate and sparse parametric encodings for neural real-time SLAM,” in Proc. IEEE Int. Conf. Comput. Vis. Pattern Recognit., 2023, pp. 13293–13302.

[48] Q. Xiaochen, H. Zhang, and F. Wenxing, “Lightweight hybrid visualinertial odometry with closed-form zero velocity update,” Chin. J. Aeronaut., vol. 33, no. 12, pp. 3344–3359, 2020.

[49] X. Yang, H. Li, H. Zhai, Y. Ming, Y. Liu, and G. Zhang, “Vox-Fusion: Dense tracking and mapping with voxel-based neural implicit representation,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2022, pp. 499–507.

[50] W. Ye et al., “PVO: Panoptic visual odometry,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 9579–9589.

[51] K. Yousif, A. Bab-Hadiashar, and R. Hoseinnezhad, “An overview to visual odometry and visual SLAM: Applications to mobile robotics,” Intell. Ind. Syst., vol. 1, no. 4, pp. 289–311, 2015.

[52] C. Yu et al., “DS-SLAM: A semantic visual SLAM towards dynamic environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 1168–1174.

[53] Z. Zhu et al., “NICE-SLAM: Neural implicit scalable encoding for SLAM,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 12786–12796.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/3623166b823fa5a8cf0493b1e985e275a1e0e813bc71f6269c6e630d33c7e5a1.jpg)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/b90a25ef91de1a27a74f535dabc200945ab2992ea6bc5992fe8763cb1309809f.jpg)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/3c5c3c60fdb15a5671a1544a0568499f07651a430c2114122402f49d3ab7b60c.jpg)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/3a07f31d1285ed41b1ed9ee06e186a7bb94f1d45a820141feb8ed0ec2ed99c4d.jpg)  
Jinyu Li received the PhD degree in computer science and technology from the Zhejiang University, in 2022. His research interests include 3D reconstruction, SLAM and Augmented Reality.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/748464978dd46137c5eb2119fb29f48137043e15fc3f1eb9cc136dfa1bf9d536.jpg)

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/3593c42dc7c0e514aaa8517b668fc4e0a6ea52cf971fe738df6387f7e5117c2c.jpg)

Hujun Bao (Member, IEEE) is currently a professor with the Computer Science Department of Zhejiang University, and the former director of the State Key Lab ofCAD&CG. His research interests include computer graphics, computer vision and mixed reality. He leads the mixed reality group in the lab to do a wide range of research on 3D Reconstruction and Modeling, Real-time Rendering and Virtual Reality, Real-time 3D Fusion, and Augmented Reality. Some of these algorithms have been successfully integrated into the mixed reality system SenseMARS.

Xiaokun Pan received the BS degree in electrical engineering from Wuhan University, in 2019. He is currently working toward the PhD degree with Zhejiang University. His research interests include 3D reconstruction, SLAM and augmented reality.

Nan Wang received the master’s degree from the State Key Lab of CAD&CG, Zhejiang University, in 2015, advised by Prof. Guofeng Zhang. He is currently working as research director with SenseTime. Before that, he was a senior RD with Baidu Inc. His research interests include SLAM, 3D reconstruction, and augmented reality.

Ziyang Zhang received the BS degree in computer science and technology from Heilongjiang University, in 2020. He is currently working toward the PhD degree with Zhejiang University. His research interests include HCI, SLAM, and augmented reality.

Guofeng Zhang (Member, IEEE) received the BS and PhD degrees in computer science and technology from Zhejiang University, in 2003 and 2009, respectively. He is currently a professor with Zhejiang University. He received the National Excellent Doctoral Dissertation Award, the Excellent Doctoral Dissertation Award of China Computer Federation and the ISMAR 2020 Best Paper Award. His research interests include SLAM, 3D reconstruction, and augmented reality.

Gan Huang received the BS and MS degrees in electronic information engineering from China Jiliang University, in 2018 and 2022, respectively. He is curretnly working toward the PhD degree with Zhejiang University. His research interests include SLAM, 3D reconstruction and visual knowledge learning.

![](images/2024_RD-VIO__Robust_Visual-Inertial_Odometry_for_Mobile_Augme/2c51daf9c9dd686d2e671b4f3de08f90690ed9e3274362cc62494e2cfe971e9d.jpg)