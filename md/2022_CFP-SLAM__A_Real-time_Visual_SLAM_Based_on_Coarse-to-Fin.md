# CFP-SLAM: A Real-time Visual SLAM Based on Coarse-to-Fine Probability in Dynamic Environments

Xinggang Hu<sup>1</sup>, Yunzhou Zhang<sup>1∗</sup>, Zhenzhong Cao<sup>1</sup>, Rong Ma<sup>2</sup>, Yanmin Wu<sup>3</sup>, Zhiqiang Deng<sup>1</sup>, Wenkai Sun<sup>1</sup>

Abstract— The dynamic factors in the environment will lead to the decline of camera localization accuracy due to the violation of the static environment assumption of SLAM algorithm. Recently, some related works generally use the combination of semantic constraints and geometric constraints to deal with dynamic objects, but problems can still be raised, such as poor real-time performance, easy to treat people as rigid bodies, and poor performance in low dynamic scenes. In this paper, a dynamic scene-oriented visual SLAM algorithm based on object detection and coarse-to-fine static probability named CFP-SLAM is proposed. The algorithm combines semantic constraints and geometric constraints to calculate the static probability of objects, keypoints and map points, and takes them as weights to participate in camera pose estimation. Extensive evaluations show that our approach can achieve almost the best results in high dynamic and low dynamic scenarios compared to the state-of-the-art dynamic SLAM methods, and shows quite high real-time ability.

## I. INTRODUCTION

Simultaneous localization and mapping (SLAM) is the key technology for autonomous navigation of mobile robots, and it is widely applied in the fields of autopilot, UAV and augmented reality (AR). SLAM system is based on environmental static assumption [1], and dynamic factors will bring wrong observation data to the system, making it difficult to establish various geometric constraints on which SLAM system works, and reducing the accuracy and robustness of SLAM system. The abnormal point processing mechanism of RANSAC (Random Sample Consensus) algorithm can solve the influence of certain abnormal points in static or slightly dynamic environment. However, when dynamic objects occupy most of the camera view, RANSAC algorithm has little effect.

With the development of deep learning technology, some advanced researchers have used semantic constraints to solve the visual SLAM problem in dynamic environment recent years. The general approach is to take the semantic information obtained from object detection [2], [3] or semantic segmentation [4]–[12] as a priori and eliminate the dynamic objects in the environment combined with geometric constraints. Semantic segmentation can provide a fine pixel level object mask, but its real-time performance is poor. The improvement of segmentation accuracy and robustness often comes at the cost of huge computational cost. Even so, the segmentation boundary of the object can not be extremely accurate and can not completely cover the moving object [12]. Object detection can circumvent the problems above, but there are a large amount of background point clouds in the box of objects, and some complex cases will be missed easily [3]. In addition, there are two common problems with current schemes: 1) All dynamic objects are treated as high dynamic attributes, which leads to poor performance in low dynamic scene. 2) As non-rigid objects, human bodies often perform partial movement. Directly eliminating the human body as a whole object will reduce the constraint of keypoints and introduce a negative effect on accuracy of localization.

For the above problems, we propose CFP-SLAM, which is a high-performance high-efficiency visual SLAM system based on object detection and static probability in indoor dynamic environments. On the basis of ORB-SLAM2 [13], CFP-SLAM uses YOLOv5 to obtain semantic information, uses extended Kalman filter (EKF) and Hungarian algorithm to compensate missed detection, calculates the static probability of objects to distinguish high dynamic objects from low dynamic objects, and distinguishes foreground points and background points of object detection results based on DBSCAN (Density-Based Spatial Clustering of Applications with Noise) algorithm. Established on a variety of constraints, a two-stage calculation method of the static probability of keypoints from coarse to fine is designed. The static probability of keypoints is used as a weight to participate in the camera pose optimization. Considering the needs of different scenarios, we provide a lower-performance version to improve the real-time performance without calculating the static probability of objects.

Extensive experiments are conducted on public datasets. Compared with state-of-the-art dynamic SLAM methods, our approach achieves the highest localization accuracy in almost all low dynamics and high dynamic scenarios. The main contributions of this paper are as follows:

• Compensating missed detection based on EKF and Hungarian algorithm, while using DBSCAN clustering algorithm to distinguish the foreground points and background points of box.

• The distinction of object dynamic attributes. Based on the YOLOv5 object detection and geometric constraints, the object motion attributes are divided into high dynamics and low dynamics, which are provided to the subsequent methods as a priori information for processing with different strategies, so as to improve the robustness and adaptability of SLAM system.

• The static probability of keypoints from coarse to fine. A two-stage static probability of keypoints calculation method based on the static probability of object, the DBSCAN clustering algorithm, the epipolar constraints and the projection constraints is proposed to solve the problem of false deletion of static keypoints caused by non-rigid body local motion.

## II. RELATED WORK

## A. Dynamic SLAM without Priori Semantic Information

When there is no semantic information as the priori, using reliable constraints to find the correct feature matching relationship is the basic method to deal with dynamic SLAM problem. Li et al. [14] propose a static weighting method of keyframe edge points, and integrated into the IAICP method to reduce tracking error. Sun et al. [15] roughly detect the motion of moving objects based on self motion compensation image difference, and enhance the motion detection by tracking the motion using particle filter. Then, they [16] propose a novel RGB-D data-based on-line motion removal approach, and build and update the foreground model incrementally. StaticFusion [17] simultaneously estimates the camera motion as well as a probabilistic static/dynamic segmentation of the current RGB-D image pair. DMS-SLAM [18] uses GMS [19] to eliminate mismatched points. Kim et al. [20] propose a dense visual mileage calculation method based on background model to estimate the nonparametric background model from depth scene. Dai et al. [21] distinguishe dynamic and static map points based on feature correlation. Flowfusion [22] uses optical flow residuals to highlight dynamic regions in rgbd point clouds. Because there is no need for deep learning networks to provide semantic priors, the above methods are usually fast in dealing with dynamic factors. However, these methods ignore the potential motion of the object, resulting in missed detection of moving objects, which is relatively lack of accuracy.

## B. Dynamic SLAM Based on Semantic Constraints

Semantic segmentation or object detection can provide a steady and reliable priority constraint for dynamic SLAM. Detect-SLAM [2] detects objects in keyframes and propagates the motion probability of keypoints in real time to eliminate the influence of dynamic objects in SLAM. DS-SLAM [4] uses SegNet [23] to obtain semantic information, combines sparse optical flow and motion consistency detection to judge people’s dynamic and static attributes. Dyna-SLAM [5] combines mask R-CNN [24] and multi view geometry to process moving objects. Brasch et al. [6] present monocular SLAM approach for highly dynamic environments which models dynamic outliers with a joint probabilistic model based on semantic prior information predicted by a CNN.

With the help of the initial segmentation results, Wang et al. [7] extract the accurate pose from the rough pose by identifying and processing the moving object and possible moving object respectively, and further help to make up for the error and boundary inaccuracy of the segmentation area. Dynamic-SLAM [3] compensates SSD for missed detection based on the speed invariance of adjacent frames, and eliminates dynamic objects combined with selective tracking algorithm. SaD-SLAM [8] extracts static feature points from objects judged as dynamic based on semantic by verifying whether the inter frame feature points meet the epipolar constraints. Vincent et al. [9] perform semantic segmentation of object instances in the image, and use EKF to identify, track and remove dynamic objects from the scene. DP-SLAM [10] combines the results of geometric constraints and semantic segmentation, the dynamic keypoints are tracked in the Bayesian probability estimation framework. Ji et al. [11] only perform semantic segmentation on keyframes, cluster the depth map and identifies moving objects combined with re-projection error to remove known and unknown dynamic objects. Blitz-SLAM [12] repairs the mask of BlitzNet [25] based on depth information, and classifies static and dynamic matching points in potential dynamic areas using epipolar constraints. Generally, the above methods can accurately eliminate dynamic objects in the environment, but it is difficult to give consideration to both localization accuracy and real-time, and the performance is generally poor in low dynamic scenes.

## III. SYSTEM OVERVIEW

## A. Definition of Variables

In this paper, common variables are defined as follows:

$$
F _ { k } \mathrm { ~ - ~ } \mathrm { F r a m e ~ K } .
$$

• $K$ - The intrinsic matrix of a pinhole camera model.

$T _ { k , w } \in R ^ { 4 \times 4 }$ - The transformation from world frame to camera frame K, which is composed of a rotation $R _ { k , w } \in R ^ { 3 \times 3 }$ and a translation $t _ { k , w } \in R ^ { 3 \times 1 }$

P<sup>k</sup> - The keypoint with ID i in $F _ { k }$ . Its pixel coordinate is $\begin{array} { r } { P _ { i _ { u v } } ^ { k } \ = \ \left[ u _ { i } ^ { k } , v _ { i } ^ { k } \right] ^ { T } } \end{array}$ , camera coordinate is $P _ { i _ { k } } ^ { k } \ = \ \left[ X _ { i _ { k } } ^ { k } , \bar { Y _ { i _ { k } } ^ { k } } , Z _ { i _ { k } } ^ { k } \right] ^ { T }$ , world coordinate is $P _ { i _ { w } } ^ { k } \ =$ $\left[ X _ { i _ { w } } ^ { k } , Y _ { i _ { w } } ^ { k } , Z _ { i _ { w } } ^ { k } \right] ^ { \prime } . ~ ( \tilde { \cdot } )$ is the form of homogeneous coordinates in each coordinate system.

$P _ { i ^ { * } } ^ { k - 1 }$ - The keypoint with ID $i ^ { * }$ in $F _ { k - 1 }$ which forms a matching relationship with $P _ { i } ^ { k }$

$O _ { i ^ { + } } ^ { k }$ - The static probability of potential moving object with ID $i ^ { + } . \ : P _ { i } ^ { k }$ is the extracted keypoint on the object.

$O _ { T h }$ - The threshold to distinguish whether the object motion attribute is high dynamic or low dynamic.

$K _ { i } ^ { k }$ - The static probability of $P _ { i } ^ { k }$ , which is in the update state and participates in camera pose optimization.

$K _ { i } ^ { D k } , K _ { i } ^ { T \dot { k } } , K _ { i } ^ { F \dot { k } }$ - The static probability of $P _ { i } ^ { k }$ obtained by the DBSCAN clustering algorithm, the projection constraints and the epipolar constraints respectively.

$M _ { i ^ { - } } ^ { k }$ - The static probability of the map point forming a matching relationship with $P _ { i } ^ { k }$

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/f6d673d5800f5737ea4d43d332bcffd51e51812f981ccbf5820dfee4610ef182.jpg)  
Fig. 1. The overview of CFP-SLAM. The green portion and the purple portion are the input and output modules of the system respectively. The yellow portion is the semantic module, including object detection, missed detection compensation, and data association. The orange portion and the blue portion are static probability calculation modules for two stages of keypoints, respectively. In the first stage, the rough static probability of keypoints is calculated based on the static probability of objects and the results of DBSCAN clustering. In the second stage, based on the epipolar constraint and projection constraint, and considering the static probability of the object and the data association result of the box, the accurate static probability of feature points is calculated. During the whole process, the static probability of the map points is maintained and updated, and together with the static probability of the keypoints will be used as weight to participate in pose optimization.

## B. System Architecture

The overview of CFP-SLAM is demonstrated in Fig.1. Based on ORB-SLAM2 [13], we design a complete static probability calculation and update framework of keypoints based on multiple constraints to deal with the influence of moving objects in dynamic environment. The system obtains semantic information based on YOLOv5, compensates for missed detection based on EKF and Hungarian algorithm, and then the box between adjacent frames is associated. In $F _ { k }$ , only calculate and update the static probability of the keypoints inside the potential moving object box. Firstly, the static probability of potential moving object $O _ { i ^ { + } } ^ { k }$ is obtained by using the optical flow and the epipolar constraints, and the object is divided into high dynamic object and low dynamic object. Initialize $K _ { i } ^ { k }$ as the static probability of the object to which the keypoint belongs. Then, foreground points and background points is distinguished and the $\bar { K } _ { i } ^ { D k }$ is calculated by using the DBSCAN clustering results, and the $K _ { i } ^ { k }$ is updated to estimate the camera pose in the first stage to obtain $T _ { k , w }$ . Next, $K _ { i } ^ { T k } , K _ { i } ^ { F k }$ are obtained by using the projection constraints and the epipolar constraints, $K _ { i } ^ { k }$ and $M _ { i ^ { - } } ^ { k }$ are updated to participate in camera pose optimization as weights to obtain a more accurate $T _ { k , w }$

## IV. SPECIFIC IMPLEMENTATION

## A. Missed Detection Compensation Algorithm

When processing dynamic objects, if the semantic information as a priori is suddenly missing in some frames, on the one hand, the subsequent methods based on semantic priors will not be able to process dynamic objects. On the other hand, the sudden emergence of dynamic objects in high dynamic scenes will lead to a sharp increase in the number of keypoints incorrectly matched between adjacent frames, which leads to the loss of tracking in SLAM system in high dynamic scenario. Therefore, stable and accurate semantic information is critical.

In order to solve the missed detection problem of YOLOv5, we introduce EKF and Hungarian algorithm to compensate the missed detection of potential moving objects. EKF is used to predict the boxes of potential moving objects in $F _ { k }$ , while the Hungarian algorithm is used to correlate the predicted boxes with the boxes detected by YOLOv5. If the predicted box does not find a matching detected box, it could be considered that $F _ { k }$ has missed detection, and the prediction result of EKF is adopted to compensate the missed detection result. After missed detection compensation, EKF and Hungarian algorithm are used again for inter frame data association of boxes.

## B. Static Probability of Objects

We extract and track the optical flow point pairs of adjacent images, and use the optical flow point pairs outside the box of potential moving objects to solve the fundamental matrix. Then the polar error of the optical flow point pair is calculated, and the static probability of the object $O _ { i ^ { + } } ^ { k }$ is calculated with chi-square distribution. According to the calculation result of the static probability of the object and the real motion of the object, we set $O _ { T h } = 0 . 9$ , the object motion attributes are divided into high dynamic and low dynamic, which are provided to the subsequent methods as a priori information for processing with different strategies.

The static probability of all keypoints in the box of the potential moving object is initialized to $O _ { i ^ { + } } ^ { k }$ , and the static probability of other keypoints is initialized to 1.0.

## C. Static Probability of Keypoints in the First Stage

1) DBSCAN Density Clustering Algorithm: Object detection can not provide accurate object mask. Especially when the non-rigid body occupies a large proportion in the camera field of view, there are often a large number of background point clouds in its box. We noticed that people as the foreground as a non-rigid body, his depth has a good continuity, and usually has a large fault with the background depth. Therefore, when box of a person accounts for a large proportion in the camera field of view, we use the DBSCAN density clustering algorithm to distinguish between the foreground and background points of the box. We adaptively determine the parameters of DBSCAN clustering based on the number of clustering sample points and the distribution of clustering criteria (here, depth). According to the number of clusters, we combine several groups of points with small depth as the foreground. This strategy makes DBSCAN clustering algorithm have strong robustness, and can deal with some special cases, such as people being blocked by objects.

After getting the DBSCAN clustering results, we adopt a soft strategy to further estimate the static probability of background points in the box of a potential moving object. Obviously, the static probability of background points must be greater than that of the object, and it is positively correlated with the static probability of the object. Specifies that the static probability of background points derived from the DBSCAN cluster is:

$$
K _ { i } ^ { D k } = \left\{ \begin{array} { c } { \frac { 1 - O _ { T h } } { ( O _ { T h } ) ^ { 4 } } \left( K _ { i } ^ { k } \right) ^ { 3 } + 1 , O _ { i ^ { + } } ^ { k } \leq O _ { T h } } \\ { \frac { 1 } { K _ { i } ^ { k } } , \quad O _ { i ^ { + } } ^ { k } > O _ { T h } } \end{array} \right.\tag{1}
$$

Considering that the static probability estimation of keypoints has not been strictly calculated at each point, in other words, the static probability of the keypoints is coarse at present, and the camera pose estimation is vulnerable to dynamic points, we set the static probability of all foreground points in the box of high dynamic objects to 0.

2) First Stage Pose Optimization: Update the static probability of keypoints:

$$
\boldsymbol { K } _ { i } ^ { k } = \boldsymbol { K } _ { i } ^ { k } \times \boldsymbol { K } _ { i } ^ { D k }\tag{2}
$$

When initializing the SLAM system, map points will be created. At this time, the static probability of map point $M _ { i ^ { - } } ^ { k }$ will be initialized to the static probability of corresponding keypoint $K _ { i } ^ { k }$ . In the frame after initialization, $K _ { i } ^ { k }$ and $M _ { i ^ { - } } ^ { k }$ are used as weights to optimize the camera pose, and the camera pose estimation value $T _ { k , w }$ in the first stage is obtained. Then, the static probability of $P _ { i } ^ { k }$ , which has a matching relation with the keypoints in $F _ { k - 1 }$ , is calculated precisely based on the projection constraints and the epipolar constraints.

## D. Static Probability of Keypoints in the Second Stage

1) Static Probability Based on the Projection Constraints: Convert the $\dot { P _ { i ^ { * } } ^ { k - 1 } }$ from the pixel coordinate to the camera coordinate:

$$
P _ { i _ { k - 1 } ^ { * } } ^ { k - 1 } = \frac { 1 } { K } Z _ { i _ { k - 1 } ^ { * } } ^ { k - 1 } \widetilde { P _ { i _ { u \nu } ^ { * } } ^ { k - 1 } }\tag{3}
$$

Transform and project $P _ { i _ { k - 1 } ^ { * } } ^ { k - 1 }$ to $F _ { k }$ , and the Euclidean distance between the projection point and $P _ { i } ^ { k }$ is:

$$
d _ { i } ^ { T } = \left\| P _ { i _ { u v } } ^ { k } - \left| \frac { 1 } { \left| T _ { k , k - 1 } \widetilde { P _ { i _ { k - 1 } ^ { k - 1 } } ^ { k - 1 } } \right| _ { Z } } K \left| T _ { k , k - 1 } \widetilde { P _ { i _ { k - 1 } ^ { k - 1 } } ^ { k - 1 } } \right| _ { X Y Z } \right| _ { u \nu } \right\| _ { 2 }\tag{4}
$$

Where function $| P | _ { Z }$ represents the z-axis coordinate of point P, and $| P | _ { X Y Z }$ represents the non-homogeneous coordinate form of point P. On the premise that the camera pose $T _ { k , w }$ is relatively accurate, the greater $d _ { i } ^ { T }$ , the greater the possibility that $P _ { i } ^ { k }$ and $P _ { i ^ { * } } ^ { k - 1 }$ are mismatched. Based on this principle, we design a static probability model based on the projection constraints. After sorting the $d _ { i } ^ { T }$ of all keypoints outside the box of the dynamic object in $F _ { k }$ from small to large, take $d _ { i } ^ { T }$ at the truncated position of 0.8 as the adaptive threshold $D _ { T h } ^ { \check { T } }$ of the projection error, and obtain the minimum value $d _ { m i n } ^ { T }$ of $d _ { i } ^ { T }$ . We use the Sigmoid function form to measure the static probability of keypoints of the matching relationship in the box:

$$
K _ { i } ^ { T k } = \frac { 1 } { 1 + e ^ { \left( d _ { i } ^ { T } - D _ { T h } ^ { T } \right) \times } \frac { m } { D _ { T h } ^ { T } - d _ { \operatorname* { m i n } } ^ { T } } }\tag{5}
$$

The parameter m represents the mapping of the interval $\left( D _ { T h } ^ { T } - \dot { d } _ { \operatorname* { m i n } } ^ { T } , D _ { T h } ^ { T } + \dot { d } _ { \operatorname* { m i n } } ^ { T } \right)$ to the $( - m , m )$ interval of sigmod original function, and take $m = 5$ in the experiment. This is because when $m = 5$ , the value result of sigmod original function in the interval $( - m , m )$ is very close to its value range. The $d _ { \operatorname* { m i n } } ^ { T }$ instead of 0 is used to prevent the $d _ { i } ^ { T }$ from being generally too large due to the inaccurate camera pose.

For a pair of matching points, the satisfaction of the projection constraints is not only related to whether the corresponding spatial points strictly meet the static environment assumption, but also directly related to the number of constraints when solving the pose matrix and whether the pose matrix itself is correctly solved. Therefore, the statistical confidence $C _ { s } ^ { T k }$ and calculation confidence $C _ { c } ^ { T k }$ of the pose matrix are introduced:

$$
C _ { S } ^ { T k } = \frac { 1 } { 1 + e ^ { - N _ { B A } + 0 . 5 T h _ { B A } } }\tag{6}
$$

$$
C _ { C } ^ { T k } = 1 - \frac { \sum d _ { i } ^ { T } } { N _ { T } \times D _ { T h } ^ { T } }\tag{7}
$$

Where $N _ { B A }$ is the number of interior points obtained by participating in the last camera pose solution, and threshold $T h _ { B A }$ is the minimum number of interior points required to participate in the camera pose solution, $N _ { T }$ and $\sum d _ { i } ^ { T }$ respectively represent the number of all sample points and the sum of $d _ { i } ^ { T }$ satisfying $d _ { i } ^ { T } < D _ { T h } ^ { T }$

(a)

2) Static Probability Based on the Epipolar Constraints: Based on the camera pose estimation $T _ { k , w }$ in the first stage, a more accurate fundamental matrix can be calculated:

$$
F _ { k , k - 1 } = \mathrm { K } ^ { - \mathrm { T } } \left( t _ { k , k - 1 } \right) ^ { \wedge } R _ { k , k - 1 } \mathrm { K } ^ { - 1 }\tag{8}
$$

The pole line ${ l } _ { i } ^ { k } = \left[ { A } _ { i } ^ { k } , { B } _ { i } ^ { k } , { C } _ { i } ^ { k } \right] ^ { T }$ corresponding to $P _ { i } ^ { k }$ is:

$$
l _ { i } ^ { k } = F _ { k , k - 1 } \widetilde { P _ { i _ { u v } ^ { k } } ^ { k - 1 } }\tag{9}
$$

Then the polar error $d _ { i } ^ { F }$ is:

$$
d _ { i } ^ { F } = \frac { \left| \left( \widetilde { P _ { i _ { \mathrm { u v } } } ^ { k } } \right) ^ { T } l _ { i } ^ { k } \right| } { \sqrt { \left( A _ { i } ^ { k } \right) ^ { 2 } + \left( B _ { i } ^ { k } \right) ^ { 2 } } }\tag{10}
$$

Similar to the projection constraints, we calculate static probability and confidence based on the epipolar constraints to obtain $K _ { i } ^ { F k }$ , the statistical confidence $\bar { C _ { s } ^ { F \bar { k } } }$ and calculation confidence $\dot { C } _ { c } ^ { F k }$ of the fundamental matrix.

It should be noted that, as Eq.8 mentioned, the fundamental matrix can not be obtained when the camera translation is not large enough. Therefore, when the camera translation is less than the set threshold $t _ { T h }$ , skip the calculation of static probability and confidence based on the epipolar constraints, that is:

$$
K _ { i } ^ { F k } = 0 , C _ { S } ^ { F k } = C _ { C } ^ { F k } = 0 \quad \mathrm { s . t . } \| t _ { k , k - 1 } \| _ { 2 } \leq t _ { T h }\tag{11}
$$

3) Second Stage Pose Optimization: After calculating the static probability of the keypoints based on the projection constraints and the epipolar constraints, we update the static probability of $P _ { i } ^ { k }$ which matches the keypoints in $F _ { k - 1 }$ for the second time. When the object is in high dynamics, the negative impact of dynamic points on camera pose estimation is generally greater than the positive impact of the increase in the number of static point constraints, which is just the opposite when the object is in low dynamics. This is because ORB-SLAM2 has certain outlier suppression strategies, which can suppress dynamic disturbances in low dynamics, but does not work in high dynamics. So, when $O _ { i ^ { + } } ^ { k } \leq O _ { T h }$

$$
K _ { i } ^ { k } = \left\{ \begin{array} { l } { K _ { i } ^ { T k } \times K _ { i } ^ { F k } , \left\| t _ { k , k - 1 } \right\| _ { 2 } > t _ { T h } } \\ { K _ { i } ^ { T k } , \quad \left\| t _ { k , k - 1 } \right\| _ { 2 } \leq t _ { T h } } \end{array} \right.\tag{12}
$$

when $O _ { i ^ { + } } ^ { k } > O _ { T h }$

$$
K _ { i } ^ { k } = \frac { K _ { i } ^ { T k } \times C _ { s } ^ { T k } C _ { c } ^ { T k } } { C _ { s } ^ { T k } C _ { c } ^ { T k } + C _ { s } ^ { F k } C _ { c } ^ { F k } } + \frac { K _ { i } ^ { F k } \times C _ { s } ^ { F k } C _ { c } ^ { F k } } { C _ { s } ^ { T k } C _ { c } ^ { T k } + C _ { s } ^ { F k } C _ { c } ^ { F k } }\tag{13}
$$

After missed detection compensation, we use EKF and Hungarian algorithm to correlate the boxes of potential moving objects between adjacent frames. It is easy to know that if the association result of a box in $F _ { k }$ is not found in $F _ { k - 1 }$ , even if there is a matching relationship between the foreground points in the box, it is generally a false matching, so let $K _ { i } ^ { k } \ = \ 0$ in this case. For $P _ { i } ^ { k }$ that does not match the keypoints in $F _ { k - 1 }$ , according to the results of DBSCAN clustering, if $P _ { i } ^ { k }$ belongs to the foreground points, let $K _ { i } ^ { k } = 0 ,$ else let ${ \dot { K } } _ { i } ^ { k } = M _ { i ^ { - } } ^ { k }$ . After the second estimation result of $K _ { i } ^ { k }$ is obtained, $M _ { i ^ { - } } ^ { k }$ is updated. When $M _ { i ^ { - } } ^ { k } ~ < ~ 0 . 3$ , delete the map point. Then $K _ { i } ^ { k }$ and $M _ { i ^ { - } } ^ { k }$ are used as weights to participate in the second stage of camera pose optimization. When there is a big difference between $K _ { i } ^ { k }$ and $M _ { i ^ { - } } ^ { k }$ , it can be considered that $K _ { i } ^ { k }$ and $M _ { i ^ { - } } ^ { k }$ are mismatched and do not participate in optimization.

## V. EXPERIMENTS AND RESULTS

In this section, we test the performance of the proposed algorithm in 8 dynamic sequences of the TUM RGB-D dataset [26], including 4 low dynamic sequences (fr3/s for short) and 4 high dynamic sequences (fr3/w for short), and the camera includes 4 kinds of motion: static, xyz, halfsphere and rpy. The indicators used to evaluate the accuracy are the Absolute Trajectory Error (ATE) and the Relative Pose Error (RPE). ATE represents the global consistency of trajectory. RPE includes translation drift and rotation drift. The Root-Mean-Square-Error (RMSE) and Standard Deviation (S.D.) of both are used to represent the robustness and stability of the system [12]. Firstly, we show the effect of missed detection compensation and DBSCAN clustering, then compare our method with some of the most advanced methods, then design a series of ablation experiments to test the impact of each module, and finally carry out real-time analysis. All the experiments are performed on a computer with Intel i7 CPU, 3060 GPU, and 16GB memory.

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/8747e55ecadfbe923d93f9c496c8c01061dfd3e56a4fc8fe73d3a6753440d297.jpg)  
(b)  
(c)  
(d)  
Fig. 2. Missed detection and the results of missed detection compensation in the following cases: (a) The rapid motion of the object. (b) The incomplete appearance of the object to be detected in the camera field of view. (c) The blurred image. (d) The singular angle of view caused by camera rotation. (e) Continuous frame miss detection.  
(e)

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/27c01e9af6749165b642796358467dacd3f3bbc7bc627881c60e3c5ee56a8de6.jpg)  
Fig. 3. Effect of DBSCAN density clustering algorithm in two consecutive frames. The top set of images is taken every 8 frames, and the bottom set of images is taken every 4 frames. The images contain three common states of movement: sitting in a chair, slow motion and fast motion. After clustering, the foreground and background points are shown in red and green respectively.

## A. Missed Detection Compensation and DBSCAN Clustering

In the dynamic SLAM scene, the motion of the object, the incomplete appearance of the object to be detected in the camera field of view, the blurred image and the singular angle of view caused by camera rotation all bring severe challenges to the object detection, very easy to cause miss detection, even will lead to continuous frame miss detection. Fig.2(a)-(d) and Fig.2(e) show the results of missed detection compensation of object detection in the above four cases and six consecutive frames, respectively. Fig.3 shows the DBSCAN clustering results after missed detection compensation. We select two consecutive frames to show the clustering effect. The foreground points are marked with red and the background points are marked with green. The upper image group contains two people sitting on the chair and moving slowly respectively, and the people in the lower image group are in the fast walking state. It is worth noting from Fig.3 that many keypoints are extracted from the edge of the person, which is generally the part with the highest dynamic attributes. However, semantic segmentation is difficult to accurately judge the boundary of objects [12], which leads to the misjudgment of dynamic and static attributes of keypoints. We use DBSCAN algorithm to cluster keypoints based on depth information, which can well avoid this problem. The experimental results fully show the effectiveness and robustness of the missed detection compensation algorithm and clustering algorithm.

## B. Comparison with State-of-the-arts

We contrast with ORB-SLAM2 [13] and forth most advanced dynamic SLAM methods, including DS-SLAM [4], Dyna-SLAM [5], Blitz-SLAM [12] and TRS [11]. Like our method, these algorithms are all improved based on ORB-SLAM2. Without calculating the static probability of the object, we provide a lower performance version of the algorithm in this paper with higher real-time performance, which is called CFP-SLAM<sup>−</sup>. The quantitative comparison results are shown in Tables I, II and III, in which the best results are highlighted in bold and the second-best are underlined. The data of DS-SLAM, Dyna-SLAM, Blitz-SLAM and TRS comes from the source literature, / indicates that the corresponding data is not provided in the source literature. The experimental results show that, unlike other dynamic SLAM algorithms, which only have advantages over ORB-SLAM2 in high dynamic scenarios, this algorithm can achieve almost the best results in high dynamic and low dynamic scenarios. Even the low-performance version

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/bff3957b8d6d50ea7497736e8402ec5f9e1cdc30477a1de35c28792735982cd6.jpg)  
(1) s/xyz

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/2b1bdd4752ac787d909aaa7625660eec2b20ac91fc7e92c03d12f62f7de44316.jpg)  
(2) s/hs

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/1e7a55acbe9fbebd5c0cddfa3f10de5b484f8ed8f076288e4fc4d5c7c59976b3.jpg)  
(3) s/static

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/8a9824e2b781f2c02ac08b1eac015109e7b78cf99db0f27dd8db67aef1c4b6de.jpg)  
(4) s/rpy

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/299d1df3cd54056a1bf3676aa17f8df123a61ef4013df4f19e18c77acb60858e.jpg)  
(5) w/xyz

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/20cdb58c72f79a1ebae4e7cd3ccf042d3ae569e6cceb586fc4e16bb5186cacb6.jpg)  
(6) w/hs

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/ec7659102332a1ac619493e8c7f09bc904fb51fefe3ec471dd9acf91eb602b5f.jpg)  
(7) w/static

![](images/2022_CFP-SLAM__A_Real-time_Visual_SLAM_Based_on_Coarse-to-Fin/e2d4f28a82810410a51aedd9edc8aaa43d8109267046602a2839d85b255c6785.jpg)  
(8) w/rpy

Fig. 4. ATE and RPE from CFP-SLAM.  
TABLE I  
RESULTS OF METRICS ABSOLUTE TRAJECTORY ERROR (ATE)
<table><tr><td rowspan="2">Sequences</td><td colspan="2">ORB-SLAM2</td><td colspan="2">Dyna-SLAM</td><td colspan="2">DS-SLAM</td><td colspan="2">Blitz-SLAM</td><td colspan="2">TRS</td><td colspan="2">CFP-SLAM⁻</td><td colspan="2">CFP-SLAM</td></tr><tr><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td></tr><tr><td>fr3/s/xyz</td><td>0.0092</td><td>0.0047</td><td>0.0127</td><td>0.0060</td><td>1</td><td>1</td><td>0.0148</td><td>0.0069</td><td>0.0117</td><td>1</td><td>0.0129</td><td>0.0068</td><td>0.0090</td><td>0.0042</td></tr><tr><td>fr3/s/half</td><td>0.0192</td><td>0.0110</td><td>0.0186</td><td>0.0086</td><td>1</td><td>1</td><td>0.0160</td><td>0.0076</td><td>0.0172</td><td></td><td>0.0159</td><td>0.0072</td><td>0.0147</td><td>0.0069</td></tr><tr><td>fr3/s/static</td><td>0.0087</td><td>0.0042</td><td>1</td><td>1</td><td>0.0065</td><td>0.0033</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0.0061</td><td>0.0029</td><td>0.0053</td><td>0.0027</td></tr><tr><td>fr3/s/rpy</td><td>0.0195</td><td>0.0124</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td></td><td>0.0244</td><td>0.0175</td><td>0.0253</td><td>0.0154</td></tr><tr><td>fr3/w/xyz</td><td>0.7214</td><td>0.2560</td><td>0.0164</td><td>0.0086</td><td>0.0247</td><td>0.0161</td><td>0.0153</td><td>0.0078</td><td>0.0194</td><td></td><td>0.0149</td><td>0.0077</td><td>0.0141</td><td>0.0072</td></tr><tr><td>fr3/w/half</td><td>0.4667</td><td>0.2601</td><td>0.0296</td><td>0.0157</td><td>0.0303</td><td>0.0159</td><td>0.0256</td><td>0.0126</td><td>0.0290</td><td></td><td>0.0235</td><td>0.0114</td><td>0.0237</td><td>0.0114</td></tr><tr><td>fr3/w/static</td><td>0.3872</td><td>0.1636</td><td>0.0068</td><td>0.0032</td><td>0.0081</td><td>0.0036</td><td>0.0102</td><td>0.0052</td><td>0.0111</td><td>1</td><td>0.0069</td><td>0.0032</td><td>0.0066</td><td>0.0030</td></tr><tr><td>fr3/w/rpy</td><td>0.7842</td><td>0.4005</td><td>0.0354</td><td>0.0190</td><td>0.4442</td><td>0.2350</td><td>0.0356</td><td>0.0220</td><td>0.0371</td><td>1</td><td>0.0411</td><td>0.0257</td><td>0.0368</td><td>0.0230</td></tr></table>

TABLE II  
RESULTS OF METRIC TRANSLATIONAL DRIFT (RPE)
<table><tr><td rowspan="2">Sequences</td><td colspan="2">ORB-SLAM2</td><td colspan="2">Dyna-SLAM</td><td colspan="2">DS-SLAM</td><td colspan="2">Blitz-SLAM</td><td colspan="2">TRS</td><td colspan="2">CFP-SLAM-</td><td colspan="2">CFP-SLAM</td></tr><tr><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td></tr><tr><td>fr3/s/xyz</td><td>0.0117</td><td>0.0060</td><td>0.0142</td><td>0.0073</td><td>1</td><td>1</td><td>0.0144</td><td>0.0071</td><td>0.0166</td><td>1</td><td>0.0149</td><td>0.0081</td><td>0.0114</td><td>0.0055</td></tr><tr><td>fr3/s/half</td><td>0.0231</td><td>0.0163</td><td>0.0239</td><td>0.0120</td><td>1</td><td>1</td><td>0.0165</td><td>0.0073</td><td>0.0259</td><td>/</td><td>0.0214</td><td>0.0099</td><td>0.0162</td><td>0.0079</td></tr><tr><td>fr3/s/static</td><td>0.0090</td><td>0.0043</td><td>1</td><td>1</td><td>0.0078</td><td>0.0038</td><td>1</td><td>1</td><td>1</td><td>/</td><td>0.0078</td><td>0.0034</td><td>0.0072</td><td>0.0035</td></tr><tr><td>fr3/s/rpy</td><td>0.0245</td><td>0.0144</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0.0322</td><td>0.0217</td><td>0.0316</td><td>0.0186</td></tr><tr><td>fr3/w/xyz</td><td>0.3944</td><td>0.2964</td><td>0.0217</td><td>0.0119</td><td>0.0333</td><td>0.0229</td><td>0.0197</td><td>0.0096</td><td>0.0234</td><td></td><td>0.0196</td><td>0.0099</td><td>0.0190</td><td>0.0097</td></tr><tr><td>fr3/w/half</td><td>0.3480</td><td>0.2859</td><td>0.0284</td><td>0.0149</td><td>0.0297</td><td>0.0152</td><td>0.0253</td><td>0.0123</td><td>0.0423</td><td></td><td>0.0274</td><td>0.0130</td><td>0.0259</td><td>0.0128</td></tr><tr><td>fr3/w/static</td><td>0.2349</td><td>0.2151</td><td>0.0089</td><td>0.0044</td><td>0.0102</td><td>0.0048</td><td>0.0129</td><td>0.0069</td><td>0.0117</td><td>1</td><td>0.0092</td><td>0.0043</td><td>0.0089</td><td>0.0040</td></tr><tr><td>fr3/w/rpy</td><td>0.4582</td><td>0.3447</td><td>0.0448</td><td>0.0262</td><td>0.1503</td><td>0.1168</td><td>0.0473</td><td>0.0283</td><td>0.0471</td><td>1</td><td>0.0540</td><td>0.0350</td><td>0.0500</td><td>0.0306</td></tr></table>

TABLE III

RESULTS OF METRIC ROTATIONAL DRIFT (RPE)
<table><tr><td rowspan="2">Sequences</td><td colspan="2">ORB-SLAM2</td><td colspan="2">Dyna-SLAM</td><td colspan="2">DS-SLAM</td><td colspan="2">Blitz-SLAM</td><td colspan="2">TRS</td><td colspan="2">CFP-SLAM-</td><td colspan="2">CFP-SLAM</td></tr><tr><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td></tr><tr><td>fr3/s/xyz</td><td>0.4890</td><td>0.2713</td><td>0.5042</td><td>0.2651</td><td>1</td><td>1</td><td>0.5024</td><td>0.2634</td><td>0.5968</td><td>1</td><td>0.5126</td><td>0.2793</td><td>0.4875</td><td>0.2640</td></tr><tr><td>fr3/s/half</td><td>0.6015</td><td>0.2924</td><td>0.7045</td><td>0.3488</td><td>1</td><td>1</td><td>0.5981</td><td>0.2739</td><td>0.7891</td><td></td><td>0.7697</td><td>0.3718</td><td>0.5917</td><td>0.2834</td></tr><tr><td>fr3/s/static</td><td>0.2850</td><td>0.1241</td><td>1</td><td>1</td><td>0.2735</td><td>0.1215</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0.2749</td><td>0.1192</td><td>0.2654</td><td>0.1183</td></tr><tr><td>fr3/s/rpy</td><td>0.7772</td><td>0.3999</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td></td><td>0.8303</td><td>0.4653</td><td>0.7410</td><td>0.3665</td></tr><tr><td>fr3/w/xyz</td><td>7.7846</td><td>5.8335</td><td>0.6284</td><td>0.3848</td><td>0.8266</td><td>0.5826</td><td>0.6132</td><td>0.3348</td><td>0.6368</td><td></td><td>0.6204</td><td>0.3850</td><td>0.6023</td><td>0.3719</td></tr><tr><td>fr3/w/half</td><td>7.2138</td><td>5.8299</td><td>0.7842</td><td>0.4012</td><td>0.8142</td><td>0.4101</td><td>0.7879</td><td>0.3751</td><td>0.9650</td><td></td><td>0.7853</td><td>0.3821</td><td>0.7575</td><td>0.3743</td></tr><tr><td>fr3/w/static</td><td>4.1856</td><td>3.8077</td><td>0.2612</td><td>0.1259</td><td>0.2690</td><td>0.1182</td><td>0.3038</td><td>0.1437</td><td>0.2872</td><td>1</td><td>0.2535</td><td>0.1130</td><td>0.2527</td><td>0.1051</td></tr><tr><td>fr3/w/rpy</td><td>8.8923</td><td>6.6658</td><td>0.9894</td><td>0.5701</td><td>3.0042</td><td>2.3065</td><td>1.0841</td><td>0.6668</td><td>1.0587</td><td>1</td><td>1.0521</td><td>0.5577</td><td>1.1084</td><td>0.6722</td></tr></table>

TABLE IV

RESULTS OF METRICS ABSOLUTE TRAJECTORY ERROR (ATE) WITH DIFFERENT CONFIGURATIONS
<table><tr><td rowspan="2">Sequences</td><td colspan="2">CFP-SLAM</td><td colspan="2">CFP-SLAM-</td><td colspan="2">W/O-MDC</td><td colspan="2">W/O-DBS</td><td colspan="2">W/O-KSP</td><td colspan="2">Only-YOLO</td></tr><tr><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td><td>RMSE</td><td>S.D.</td></tr><tr><td>fr3/s/xyz</td><td>0.0090</td><td>0.0042</td><td>0.0129</td><td>0.0068</td><td>0.0123</td><td>0.0066</td><td>0.0130</td><td>0.0060</td><td>0.0142</td><td>0.0063</td><td>0.0174</td><td>0.0079</td></tr><tr><td>fr3/s/half</td><td>0.0147</td><td>0.0069</td><td>0.0159</td><td>0.0072</td><td>0.0150</td><td>0.0074</td><td>0.0305</td><td>0.0179</td><td>0.0201</td><td>0.0089</td><td>0.0281</td><td>0.0158</td></tr><tr><td>fr3/s/static</td><td>0.0053</td><td>0.0027</td><td>0.0061</td><td>0.0029</td><td>0.0055</td><td>0.0025</td><td>0.0064</td><td>0.0030</td><td>0.0062</td><td>0.0030</td><td>0.0064</td><td>0.0027</td></tr><tr><td>fr3/s/rpy</td><td>0.0253</td><td>0.0154</td><td>0.0244</td><td>0.0175</td><td>0.0237</td><td>0.0149</td><td>0.0297</td><td>0.0205</td><td>0.0287</td><td>0.0195</td><td>0.0460</td><td>0.0332</td></tr><tr><td>fr3/w/xyz</td><td>0.0141</td><td>0.0072</td><td>0.0149</td><td>0.0077</td><td>0.0158</td><td>0.0079</td><td>0.0159</td><td>0.0081</td><td>0.0154</td><td>0.0076</td><td>0.0165</td><td>0.0082</td></tr><tr><td>fr3/w/half</td><td>0.0237</td><td>0.0114</td><td>0.0235</td><td>0.0114</td><td>0.0258</td><td>0.0134</td><td>0.0274</td><td>0.0137</td><td>0.0307</td><td>0.0151</td><td>0.0310</td><td>0.0165</td></tr><tr><td>fr3/w/static</td><td>0.0066</td><td>0.0030</td><td>0.0069</td><td>0.0032</td><td>0.0070</td><td>0.0031</td><td>0.0078</td><td>0.0033</td><td>0.0076</td><td>0.0033</td><td>0.0073</td><td>0.0032</td></tr><tr><td>fr3/w/rpy</td><td>0.0368</td><td>0.0230</td><td>0.0411</td><td>0.0257</td><td>0.1910</td><td>0.1594</td><td>0.0749</td><td>0.0536</td><td>0.0405</td><td>0.0211</td><td>0.0456</td><td>0.0312</td></tr></table>

we provide shows better performance than other algorithms. In rpy sequences, on the one hand, the epipolar constraints cannot be used, on the other hand, the large change of camera angle leads to insufficient feature matching, so our method performs slightly worse. The ATE and RPE plots of our algorithm on 8 sequences are shown in Fig.4.

## C. Ablation Experiment

In order to prove the function of each module of our algorithm, We design a series of ablation experiments, and the experimental results are shown in Table IV. Among them, CFP-SLAM: The algorithm of this paper; CFP-SLAM<sup>−</sup>: Do not use static probability of objects; W/O-MDC: Without missed detection compensation; W/O-DBS: Without DB-SCAN clustering; W/O-KSP: Without the static probability of keypoints, that is, all the foreground points after missed detection compensation and DBSCAN clustering are directly eliminated; Only-YOLO: Directly eliminate all keypoints in the box with human category.

The experimental results show that CFP-SLAM<sup>−</sup> shows worse performance in low dynamic scenes, because we cannot distinguish between high dynamic objects and low dynamic objects, so all objects are processed according to high dynamic. W/O-MDC is almost unaffected in low dynamic scenes, but the performance is very poor in high dynamic scenes, especially in w/rpy, when the camera and objects are moving violently. In fact, the tracking is often lost in w/xyz, w/half and w/rpy because of missed detection. W/O-DBS and W/O-KSP show general performance in all sequences, which illustrates the effectiveness of DBSCAN clustering and the limitation of dealing with non-rigid bodies with partial motion as a whole, respectively. Only-YOLO encounters difficulties in initialization due to insufficient features in almost all sequences, and tracking is lost in some sequences.

## D. Real-time Analysis

Real-time performance is one of the important evaluation indexes of SLAM system. We test the average running time of each module, as shown in Table V. EKF represents the missed detection compensation and data association of boxes module, OSP represents the static probability calculation module of objects, and KSP represents the static probability calculation module of keypoints based on the epipolar constraints and the projection constraints. Semantic threads based on YOLOv5s run in parallel with ORB feature extraction. The results show that the average processing time per frame for the main threads of CFP-SLAM and CFP-SLAM<sup>−</sup> is 42.7 ms and 24.77 ms, that is, the running speed reaches 23 Fps and 40 Fps respectively. Compared with the SLAM system based on semantic segmentation, it can better meet the real-time requirements while ensure the accuracy.

TABLE V  
THE AVERAGE RUNNING TIME OF EACH MODULE.
<table><tr><td rowspan=1 colspan=1>Methods</td><td rowspan=1 colspan=1>YOLO</td><td rowspan=1 colspan=1>EKF</td><td rowspan=1 colspan=1>OSP</td><td rowspan=1 colspan=1>DBSCAN</td><td rowspan=1 colspan=1>KSP</td><td rowspan=1 colspan=1>Tracking</td></tr><tr><td rowspan=1 colspan=1>CFP-SLAM</td><td rowspan=1 colspan=1>12.44</td><td rowspan=1 colspan=1>0.07</td><td rowspan=1 colspan=1>17.93</td><td rowspan=1 colspan=1>1.76</td><td rowspan=1 colspan=1>3.66</td><td rowspan=1 colspan=1>42.7</td></tr><tr><td rowspan=1 colspan=1>CFP-SLAM-</td><td rowspan=1 colspan=1>12.44</td><td rowspan=1 colspan=1>0.07</td><td rowspan=1 colspan=1>1</td><td rowspan=1 colspan=1>1.76</td><td rowspan=1 colspan=1>3.66</td><td rowspan=1 colspan=1>24.77</td></tr></table>

## VI. CONCLUSION

In this paper, we propose a dynamic scene-oriented visual SLAM algorithm based on YOLOv5s and coarse-to-fine static probability. After missed detection compensation and keypoints clustering, the static probabilities of objects, keypoints and map points are calculated and updated as weights to participate in pose optimization. Extensive evaluation shows that our algorithm achieves the highest accuracy of localization in almost all low dynamic and high dynamic scenes, and has quite high real-time performance. In the future, we intend to build a lightweight plane and object map containing only static environment for robot navigation and augmented reality.

## REFERENCES

[1] M. R. U. Saputra, A. Markham, and N. Trigoni, “Visual slam and structure from motion in dynamic environments: A survey,” ACM Computing Surveys (CSUR), vol. 51, no. 2, pp. 1–36, 2018.

[2] F. Zhong, S. Wang, Z. Zhang, and Y. Wang, “Detect-slam: Making object detection and slam mutually beneficial,” in 2018 IEEE Winter Conference on Applications ofComputer Vision (WACV). IEEE, 2018, pp. 1001–1010.

[3] L. Xiao, J. Wang, X. Qiu, Z. Rong, and X. Zou, “Dynamic-slam: Semantic monocular visual localization and mapping based on deep learning in dynamic environment,” Robotics and Autonomous Systems, vol. 117, pp. 1–16, 2019.

[4] C. Yu, Z. Liu, X.-J. Liu, F. Xie, Y. Yang, Q. Wei, and Q. Fei, “Dsslam: A semantic visual slam towards dynamic environments,” in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018, pp. 1168–1174.

[5] B. Bescos, J. M. Facil, J. Civera, and J. Neira, “Dynaslam: Tracking,´ mapping, and inpainting in dynamic scenes,” IEEE Robotics and Automation Letters, vol. 3, no. 4, pp. 4076–4083, 2018.

[6] N. Brasch, A. Bozic, J. Lallemand, and F. Tombari, “Semantic monocular slam for highly dynamic environments,” in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018, pp. 393–400.

[7] K. Wang, Y. Lin, L. Wang, L. Han, M. Hua, X. Wang, S. Lian, and B. Huang, “A unified framework for mutual improvement of slam and semantic segmentation,” in 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019, pp. 5224–5230.

[8] X. Yuan and S. Chen, “Sad-slam: A visual slam based on semantic and depth information,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 4930– 4935.

[9] J. Vincent, M. Labbe, J.-S. Lauzon, F. Grondin, P.-M. Comtois-Rivet,´ and F. Michaud, “Dynamic object tracking and masking for visual slam,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 4974–4979.

[10] A. Li, J. Wang, M. Xu, and Z. Chen, “Dp-slam: A visual slam with moving probability towards dynamic environments,” Information Sciences, vol. 556, pp. 128–142, 2021.

[11] T. Ji, C. Wang, and L. Xie, “Towards real-time semantic rgb-d slam in dynamic environments,” in 2021 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2021, pp. 11 175–11 181.

[12] Y. Fan, Q. Zhang, Y. Tang, S. Liu, and H. Han, “Blitz-slam: A semantic slam in dynamic environments,” Pattern Recognition, vol. 121, p. 108225, 2022.

[13] R. Mur-Artal and J. D. Tardos, “Orb-slam2: An open-source slam´ system for monocular, stereo, and rgb-d cameras,” IEEE transactions on robotics, vol. 33, no. 5, pp. 1255–1262, 2017.

[14] S. Li and D. Lee, “Rgb-d slam in dynamic environments using static point weighting,” IEEE Robotics and Automation Letters, vol. 2, no. 4, pp. 2263–2270, 2017.

[15] Y. Sun, M. Liu, and M. Q.-H. Meng, “Improving rgb-d slam in dynamic environments: A motion removal approach,” Robotics and Autonomous Systems, vol. 89, pp. 110–122, 2017.

[16] ——, “Motion removal for reliable rgb-d slam in dynamic environments,” Robotics and Autonomous Systems, vol. 108, pp. 115–128, 2018.

[17] R. Scona, M. Jaimez, Y. R. Petillot, M. Fallon, and D. Cremers, “Staticfusion: Background reconstruction for dense rgb-d slam in dynamic environments,” in 2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2018, pp. 3849–3856.

[18] G. Liu, W. Zeng, B. Feng, and F. Xu, “Dms-slam: A general visual slam system for dynamic scenes with multiple sensors,” Sensors, vol. 19, no. 17, p. 3714, 2019.

[19] J. Bian, W.-Y. Lin, Y. Matsushita, S.-K. Yeung, T.-D. Nguyen, and M.- M. Cheng, “Gms: Grid-based motion statistics for fast, ultra-robust feature correspondence,” in Proceedings of the IEEE conference on computer vision and pattern recognition, 2017, pp. 4181–4190.

[20] D.-H. Kim and J.-H. Kim, “Effective background model-based rgb-d dense visual odometry in a dynamic environment,” IEEE Transactions on Robotics, vol. 32, no. 6, pp. 1565–1573, 2016.

[21] W. Dai, Y. Zhang, P. Li, Z. Fang, and S. Scherer, “Rgb-d slam in dynamic environments using point correlations,” IEEE Transactions on Pattern Analysis and Machine Intelligence, 2020.

[22] T. Zhang, H. Zhang, Y. Li, Y. Nakamura, and L. Zhang, “Flowfusion: Dynamic dense rgb-d slam based on optical flow,” in 2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2020, pp. 7322–7328.

[23] V. Badrinarayanan, A. Kendall, and R. Cipolla, “Segnet: A deep convolutional encoder-decoder architecture for image segmentation,” IEEE transactions on pattern analysis and machine intelligence, vol. 39, no. 12, pp. 2481–2495, 2017.

[24] K. He, G. Gkioxari, P. Dollar, and R. Girshick, “Mask r-cnn,” in´ Proceedings of the IEEE international conference on computer vision, 2017, pp. 2961–2969.

[25] N. Dvornik, K. Shmelkov, J. Mairal, and C. Schmid, “Blitznet: A realtime deep network for scene understanding,” in Proceedings of the IEEE international conference on computer vision, 2017, pp. 4154– 4162.

[26] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of rgb-d slam systems,” in 2012 IEEE/RSJ international conference on intelligent robots and systems. IEEE, 2012, pp. 573–580.