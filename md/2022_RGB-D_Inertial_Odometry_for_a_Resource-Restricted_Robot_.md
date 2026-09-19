# RGB-D Inertial Odometry for a Resource-Restricted Robot in Dynamic Environments

Jianheng Liu , Xuanfu Li, Yueqian Liu , and Haoyao Chen , Member, IEEE

Abstract—Current simultaneous localization and mapping (SLAM) algorithms perform well in static environments but easily fail in dynamic environments. Recent works introduce deep learning-based semantic information to SLAM systems to reduce the influence of dynamic objects. However, it is still challenging to apply a robust localization in dynamic environments for resource-restricted robots. This paper proposes a real-time RGB-D inertial odometry system for resource-restricted robots in dynamic environments named Dynamic-VINS. Three main threads run in parallel: object detection, feature tracking, and state optimization. The proposed Dynamic-VINS combines object detection and depth information for dynamic feature recognition and achieves performance comparable to semantic segmentation. Dynamic-VINS adopts grid-based feature detection and proposes a fast and efficient method to extract high-quality FAST feature points. IMU is applied to predict motion for feature tracking and moving consistency check. The proposed method is evaluated on both public datasets and real-world applications and shows competitive localization accuracy and robustness in dynamic environments. Yet, to the best of our knowledge, it is the best-performance real-time RGB-D inertial odometry for resource-restricted platforms in dynamic environments for now. The proposed system is open source at: https://github.com/HITSZ-NRSL/Dynamic-VINS.git

Index Terms—Localization, visual-inertial SLAM.

## I. INTRODUCTION

IMULTANEOUS localization and mapping (SLAM) is a foundational capability for many emerging applications, such as autonomous mobile robots and augmented reality. Cameras as portable sensors are commonly equipped on mobile robots and devices. Therefore, visual SLAM (vSLAM) has received tremendous attention over the past decades. Lots of works [1]–[4] are proposed to improve visual SLAM systems performance. Most of the existing vSLAM systems depend on a static world assumption. Stable features in the environment are used to form a solid constraint for Bundle Adjustment [5]. However, in real-world scenarios like shopping malls and subways, dynamic objects such as moving people, vehicles, and unknown objects, have an adverse impact on pose optimization. Although some approaches like RANSAC [6] can suppress the influence of dynamic features to a certain extent, it will become overwhelmed when a vast number of dynamic objects appear in the scene.

Therefore, it is necessary for the system to reduce dynamic objects’ influence on the estimation results consciously. The pure geometric methods [7]–[9] are widely used to handle dynamic objects, but it is unable to cope with latent or slightly moving objects. With the development of deep learning, many researchers have tried combining multi-view geometric methods with semantic information [10]–[13] to implement a robust SLAM system in dynamic environments. To avoid the accidental deletion of stable features through object detection [14], recent dynamic SLAM systems [15], [16] exploit the advantages of pixel-wise semantic segmentation for a better recognition of dynamic features. Due to the expensive computing resource consumption of semantic segmentation, it is difficult for a semantic-segmentation-based SLAM system to run in real-time. Therefore, some researchers have tried to perform semantic segmentation only on keyframes and track moving objects via moving probability propagation [17], [18] or direct method [19] on each frame. In the cases of missed detections or object tracking failures, the pose optimization is imprecise. Moreover, since semantic segmentation is performed after keyframe selection, real-time precise pose estimation is inaccessible, and unstable dynamic features in the original frame may also cause redundant keyframe creation and unnecessary computational burdens.

The above systems still require too many computing resources to perform robust real-time localization in dynamic environments for Size, Weight, and Power (SWaP) restricted mobile robots or devices. Some researchers [20]–[22] try to run visual odometry in real-time on embedded computing devices, yet the keyframe-based visual odometry is not performed [23], which makes their accuracy unsatisfactory. At the same time, increasingly embedded computing platforms are equipped with NPU/GPU computing units, such as HUAWEI Atlas200, NVIDIA Jetson, etc. It enables lightweight deep learning networks to run on the embedded computing platform in real-time. Some studies [14], [24] implemented a keyframebased dynamic SLAM system running on embedded computing platforms. However, these works are still difficult to balance efficiency and accuracy for mobile robot applications.

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/17ab127b21987edcea1daf55bb7645648f80c869dae7b5b25cd55d4399a8183a.jpg)  
Fig. 1. The framework of Dynamic-VINS. The contributing modules are highlighted and surrounded by dash lines with different colors. Three main threads run in parallel in Dynamic-VINS. Features are tracked and detected in the feature tracking thread. The object detection thread detects dynamic objects in each frame in real-time. The state optimization thread summarizes the features information, object detection results, and depth image to recognize the dynamic features. Finally stable features and IMU preintegration results are used for pose estimation.

To address all these issues, this paper proposes a real-time RGB-D inertial odometry for resource-restricted robots in dynamic environments named Dynamic-VINS. It enables edge computing devices to provide instant robust state feedback for mobile platforms with little computation burden. An efficient dynamic feature recognition module that does not require a high-precision depth camera can be used in mobile devices equipped with depth-measure modules. The main contributions of this paper are as follows:

1) An efficient optimization-based RGB-D inertial odometry is proposed to provide real-time state estimation results for resource-restricted robots in dynamic and complex environments.

2) Lightweight feature detection and tracking are proposed to cut the computing burden. In addition, dynamic feature recognition modules combining object detection and depth information are proposed to provide robust dynamic feature recognition in complex and outdoor environments.

3) Validation experiments are performed to show the proposed system’s competitive accuracy, robustness, and efficiency on resource-restricted platforms in dynamic environments.

## II. SYSTEM OVERVIEW

The proposed SLAM system in this paper is extended based on VINS-Mono [2] and VINS-RGBD [25]; our framework is shown in Fig. 1, and the contributing modules are highlighted with different colors. For efficiency, three main threads (surrounded by dash lines) run parallel in Dynamic-VINS: object detection, feature tracking, and state optimization. Color images are passed to both the object detection thread and the feature tracking thread. IMU measurements between two consecutive frames are preintegrated [26] for feature tracking, moving consistency check, and state optimization.

In the feature tracking thread, features are tracked with the help of IMU preintegration and detected by grid-based feature detection. The object detection thread detects dynamic objects in each frame in real-time. Then, the state optimization thread will summarize the features information, object detection results, and depth image to recognize the dynamic features. A missed detection compensation module is conducted in case of missed detection. The moving consistency check procedure combines the IMU preintegration and historical pose estimation results to identify potential dynamic features. Finally, stable features and IMU preintegration results are used for the pose estimation. And the propagation of the IMU is responsible for an IMU-rate pose estimation result. Loop closure is also supported in this system, but this paper pays more attention to the localization independent of loop closure.

## III. METHODOLOGY

This study proposes lightweight, high-quality feature tracking and detection methods to accelerate the system. Semantic and geometry information from the input RGB-D images and IMU preintegration are applied for dynamic feature recognition and moving consistency check. The missed detection compensation module plays a subsidiary role to object detection in case of missed detection. Dynamic features on unknown objects are further identified by moving consistency check. The proposed methods are divided into five parts for a detailed description.

## A. Feature Matching

For each incoming image, the feature points are tracked using the KLT sparse optical flow method [27]. In this paper, the IMU measurements between frames are used to predict the motion of features. Better initial position estimation of features is provided to improve the efficiency of feature tracking by reducing optical flow pyramid layers. It can effectively discard unstable features such as noise and dynamic features with inconsistent motion. The basic idea is illustrated in Fig. 2.

In the previous frame, stable features are colored red, and newly detected features are colored blue. When the current frame arrives, the IMU measurements between the current and previous frames are used to predict the feature position (green) in the current frame. Optical flow uses the predicted feature position as the initial position to look for a match feature in the current frame. The successfully tracked features are turned red, while those that failed to be tracked are marked as unstable features (purple). In order to avoid the repetition and aggregation of feature detection, an orange circular mask centered on the stable feature is set; the region where the unstable features are located is considered an unstable feature detection region and masked with a purple circular to avoid unstable feature detection. According to the mask, new features are detected from unmasked areas in the current frame and colored blue.

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/9104cee1a370176811d41dbce5448f894bf51948d4a92ce3039da6f3d5a01f62.jpg)  
Fig. 2. Illustration of feature tracking and detection. Stable features and new features are colored red and blue, respectively. The green circles denote the prediction for optical flow. The successfully tracked features turn red; otherwise, the features turn purple. The orange and purple dash-line circles as masks are set for a uniform feature distribution and reliable feature detection. New feature points are detected from unmasked areas in the current frame.

The above means can obtain uniformly distributed features to capture comprehensive constraints and avoid repeatedly extracting unstable features on the area with blurs or weak textures. Long-term feature tracking can reduce the time consumption with the help of grid-based feature detection in the following.

## B. Grid-Based Feature Detection

The system maintains a minimum number of features for stability. Therefore, feature points need to be extracted from the frame constantly. This study adopts grid-based feature detection. Image is divided into grids, and the boundary of each grid is padded to prevent the features at the edge of the grid from being ignored; the padding enables the current grid to obtain adjacent pixel information for feature detection. Unlike traversing the whole image to detect features, only the grid with insufficient matched features will conduct feature detection. The grid cell that fails to detect features due to weak texture or is covered by the mask will be skipped in the next detection frame to avoid repeated useless detection. The thread pool technique is used to exploit the parallel performance of grid-based feature detection. Thus, the time consumption of feature detection is significantly reduced without loss.

The FAST feature detector [28] can efficiently extract feature points but easily treats noise as features and extracts similar clustered features. Therefore, the ideas of mask in Section III-A and Non-Maximum-Suppression are combined to select highquality and uniformly distributed FAST features.

## C. Dynamic Feature Recognition

Most feature points can be stably tracked through the above improvement. However, long-term tracking features on dynamic objects always come with abnormal motion and introduce wrong constraints to the system. For the sake of efficiency and computational cost, a real-time single-stage object detection method, YOLOv3 [11], is used to detect many kinds of dynamic scene elements like people and vehicles. If a detected bounding box covers a large region of the image, blindly deleting feature points in the bounding box might result in no available features to provide constraints. Therefore, semantic-segmentation-like masks are helpful to maintain the system’s running by tracking features not occluded by dynamic objects.

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/ea6bbe2c16b0d44ff662a7a30e4944603c4d7c228bcbf4b30aa6722f47633105.jpg)  
Fig. 3. Illustration of semantic mask setting for dynamic feature recognition when all pixel’s depth is available $( d > 0 )$ . The left scene represents when an objected bounding box’s farthest corner’s depth is bigger than the center to a threshold - and a semantic mask with weighted depth is set between them to separate features on dynamic objects from the background. Otherwise, the semantic mask is set behind the bounding box’s center with the distance of -, shown on the right.

This paper combines object detection and depth information for highly efficient dynamic feature recognition to achieve performance comparable to semantic segmentation. As the farther the depth camera measures, the worse the accuracy is. This problem makes some methods, such as Seed Filling, DBSCAN, and K-Means, which make full use of the depth information, exhibit poor performance with a low accuracy depth camera, as shown in Fig. 5(a). Therefore, a set of points in the detected bounding box and depth information are integrated to obtain comparable performance to the semantic segmentation, as illustrated in Fig. 3.

A pixel’s depth d is available, if $d > 0$ , otherwise, $d = 0 ,$ Considering that the bounding box corners of most dynamic objects correspond to the background points, and the dynamic objects commonly have a relatively large depth gap with the background. The K-th dynamic object’s largest background depth $\kappa _ { d _ { \mathrm { m a x } } }$ is obtained as follow

$$
\begin{array} { r } { { ^ K { d _ { \operatorname* { m a x } } } } = \operatorname* { m a x } \left( { ^ K { d _ { t l } } } + { ^ K { d _ { t r } } } ^ { K } + { ^ K { d _ { b l } } } + { ^ K { d _ { b r } } } \right) , } \end{array}\tag{1}
$$

where $^ K d _ { t l } , ^ { K } d _ { t r } , ^ { K } d _ { b l } , ^ { K } d _ { b r }$ are the depth values of the Kth object detection bounding box’s corners, respectively. Next, the Kth bounding box’s depth threshold $\kappa _ { \bar { d } }$ is defined as

$$
\begin{array} { r } { { \kappa } \bar { d } = \left\{ \begin{array} { l l } { \frac { 1 } { 2 } \left( { \displaystyle { ^ { K } d _ { \operatorname* { m a x } } } } + { ^ { K } d _ { c } } \right) , } & { \mathrm { i f ~ } { ^ { K } d _ { \operatorname* { m a x } } } - { ^ { K } d _ { c } } > \epsilon , { ^ { K } d _ { c } } > 0 , } \\ { { ^ { K } d _ { c } } + \epsilon , } & { \mathrm { i f ~ } { ^ { K } d _ { \operatorname* { m a x } } } - { ^ { K } d _ { c } } < \epsilon , { ^ { K } d _ { c } } > 0 , } \\ { { ^ { K } d _ { \operatorname* { m a x } } } , } & { \mathrm { i f ~ } { ^ { K } d _ { \operatorname* { m a x } } } > 0 , { ^ { ~ K } d _ { c } } = 0 , } \\ { + \infty , } & { \mathrm { o t h e r w i s e } \ , } \end{array} \right. } \end{array}\tag{2}
$$

where $\kappa _ { d _ { c } }$ is the depth value of the bounding box’s center; $\epsilon > 0$ is a predefined distance according to the most common dynamic objects’ size in scenes. The depth threshold $\kappa _ { \bar { d } }$ is defined in the middle of the center’s depth $\dot { \kappa } _ { d _ { c } }$ and the deepest background depth $\kappa _ { d _ { \mathrm { m a x } } }$ . When the dynamic object has a close connection with the background or is behind an object $\begin{array} { r } { ^ { K } d _ { \mathrm { m a x } } - { ^ { K } } d _ { c } < \epsilon , } \end{array}$ the depth threshold is defined at - distance from the dynamic object. If the depth is unavailable, a conservative strategy is adopted to choose an infinite depth as the threshold.

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/5e579fe5d9d053578770f1add716f780da1939624a7d035497d3f98231720ca1.jpg)  
(a)

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/f10517576ca9a1a4e7f6b18f1767750aa5b4642faf7394de8bed5b82cea8c755.jpg)  
(b)

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/15a88bf6ddf2fb59847cb0f0b3e5e197c6800c55c0da134ca5e488ff9dbb5716.jpg)  
(c)

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/919bc89085109711f031c5dfa083a9b7071cce955f721cea993f80f35e640912.jpg)  
(d)

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/219f58f69252bcd6566a14c4a4e710f864db69010009a6135599fbefa9e8e9e4.jpg)  
(e)

Fig. 4. Results of missed detection compensation. The dynamic feature recognition results are shown in the first row. The green box shows the dynamic object’s position from the object detection results. The second row shows the generated semantic mask. With the help of missed detection compensation, even if object detection failed in (b) and (d), a semantic mask including all dynamic objects could be built.  
![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/5ec7c76a7d881960117a8a5852b095904cbb5fe6bbc8b6debd64ab1380ec1014.jpg)

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/a1c58df736e0d8e5e229639319258f003407e2096160b2ed1fe6255a2732f05b.jpg)  
(a) Seed Filling  
(b) Proposed Method  
Fig. 5. Results of dynamic feature recognition. The stable features are circled by yellow. The dynamic feature recognition results generated by Seed Filling and the proposed method are shown in (a) and (b), respectively. The weighted depth d<sup>¯</sup>is colored gray; the brighter means a bigger value. The feature point on the white area will be marked as a dynamic feature.

On the semantic mask, the area covered by the K-th dynamic object bounding box is set to the weighted depth $^ { K } \bar { d } ;$ the area without dynamic objects is set to 0. Each incoming feature’s depth d is compared with the corresponding pixel’s depth threshold <sup>¯</sup>d on the semantic mask. If $d < { \bar { d } } ,$ , the feature is considered as a dynamic one. Otherwise, the feature is considered as a stable one. Therefore, the region where the depth value is smaller than the weighted depth <sup>¯</sup>d constitutes the generalized semantic mask, as shown in Figs. 4 and 5(b).

Considering that dynamic objects may exist in the field of view for a long time, the dynamic features are tracked but not used for pose estimation, different from directly deleting dynamic features. According to its recorded information, each incoming feature point from the feature tracking thread will be judged whether it is a historical dynamic feature or not. The above methods can avoid blindly deleting feature points while ensuring efficiency. It can save time from detecting features on dynamic objects, has the robustness to the missed detection of object detection, and recycle false-positive dynamic features, as illustrated in Section III-E.

## D. Missed Detection Compensation

Since object detection might sometimes fail, the proposed Dynamic-VINS utilizes the previous detection results to predict the following detection result to compensate for missed detections. It is assumed that the dynamic objects in adjacent frames have a consistent motion. Once a dynamic object is detected, its pixel velocity and bounding box will be updated. Assumed that j is the current detected frame and $j - 1$ is the previous detected frame, the pixel velocity $K _ { \mathbf { V } } c _ { j }$ (pixel/frame) of the Kth dynamic object between frames is defined as

$$
\mathbf { \Delta } ^ { K } \mathbf { v } ^ { c _ { j } } = \mathbf { \Delta } ^ { K } \mathbf { u } _ { c } ^ { c _ { j } } - \mathbf { \Delta } ^ { K } \mathbf { u } _ { c } ^ { c _ { j - 1 } } ,\tag{3}
$$

where $^ K \mathbf { u } _ { c } ^ { c _ { j } } , \kappa \mathbf { u } _ { c } ^ { c _ { j - 1 } }$ represent the pixel location of the Kth object detection bounding box’s center in jth frame and $j - 1 \mathrm { t h }$ frame, respectively. A weighted predicted velocity $K _ { \hat { \mathbf { v } } }$ is defined as

$$
\boldsymbol { \mathsf { { K } } } \hat { \mathbf { v } } ^ { c _ { j + 1 } } = \frac { 1 } { 2 } \left( \boldsymbol { \mathsf { { K } } } \mathbf { v } ^ { c _ { j } } + \boldsymbol { \mathsf { { K } } } \hat { \mathbf { v } } ^ { c _ { j } } \right) ,\tag{4}
$$

With the update going on, the velocities of older frames will have a lower weight in $K _ { \hat { \mathbf { v } } }$ . If the object fail to be detected in the next frame, the bounding box <sup>K</sup>Box containing the corners’ pixel locations $^ K { \mathbf { u } } _ { t l } , ^ { K } { \mathbf { u } } _ { t r } , ^ { K } { \mathbf { u } } _ { b l }$ and $\kappa _ { { \bf u } _ { b r } }$ , will be updated based on the predicted velocity $K _ { \hat { \mathbf { v } } }$ as follow

$$
\mathbf { \ } ^ { K } \mathbf { B } \mathbf { \hat { o } } \mathbf { x } ^ { c _ { j + 1 } } = \mathbf { \alpha } ^ { K } \mathbf { B } \mathbf { o x } ^ { c _ { j } } + \mathbf { \alpha } ^ { K } \hat { \mathbf { v } } ^ { c _ { j + 1 } } ,\tag{5}
$$

When the missed detection time is over a threshold, this dynamic object’s compensation will be abandoned. The result is shown in Fig. 4. It improves the recall rate of object detection and is helpful for a more consistent dynamic feature recognition.

## E. Moving Consistency Check

Since object detection can only recognize artificially defined dynamic objects and has a missed detection problem, the state optimization will still be affected by unknown moving objects like books moved by people. Dynamic-VINS combines the pose predicted by IMU and the optimized pose in the sliding windows to recognize dynamic features.

Consider the kth feature is first observed in the ith image and is observed by other m images in sliding windows. The average reprojection residual $r _ { k }$ of the feature observation in the sliding windows is defined as

$$
r _ { k } = \frac { 1 } { m } \sum _ { j \neq i } \Big \| \mathbf { u } _ { k } ^ { c _ { i } } - \boldsymbol { \pi } \left( \mathbf { T } _ { b } ^ { c } \mathbf { T } _ { w } ^ { b _ { i } } \mathbf { T } _ { b _ { j } } ^ { w } \mathbf { T } _ { c } ^ { b } \mathbf { P } _ { k } ^ { c _ { j } } \right) \Big \| ,\tag{6}
$$

where ${ \bf u } _ { k } ^ { c _ { i } }$ is the observation of kth feature in the ith frame; $\mathbf { P } _ { k } ^ { c _ { j } }$ k is the 3D location of kth feature in the jth frame; $\mathbf { T } _ { c } ^ { b }$ and $\mathbf { T } _ { b _ { i } } ^ { w }$ are the transforms from camera frame to body frame and from jth body frame to world frame, respecvtively; π represents the camera projection model. When the $r _ { k }$ is over a preset threshold, the kth feature is considered as a dynamic feature.

As shown in Fig. 7, the moving consistency check (MCC) module can find out unstable features. However, some stable features are misidentified (top left image), and features on standing people are not recognized (bottom right image). A low threshold holds a high recall rate of unstable features. Further, a misidentified unstable feature with more observations will be recycled if its reprojection error is lower than the threshold.

## IV. EXPERIMENTAL RESULTS

Quantitative experiments<sup>1</sup> are performed to evaluate the proposed system’s accuracy, robustness, and efficiency. Public SLAM evaluation datasets, OpenLORIS-Scene [29] and TUM RGB-D [30], provide sensor data and ground truth to evaluate SLAM system in complex dynamic environments. Since our system is built on VINS-Mono [2] and VINS-RGBD [25], they are used as the baselines to demonstrate our improvement. VINS-Mono [2] provides robust and accurate visual-inertial odometry by fusing IMU preintegration and feature observations. VINS-RGBD [25] integrates RGB-D camera based on VINS-Mono for better performance. Furthermore, DS-SLAM [15] and Ji et al.[24], state-of-the-art semantic algorithms based on ORB-SLAM2 [4], are also included for comparison.

The accuracy is evaluated by Root-Mean-Square-Error (RMSE) of Absolute Trajectory Error (ATE), Translational Relative Pose Error (T.RPE), and Rotational Relative Pose Error (R.RPE). Correct Rate (CR) [29] measuring the correct rate over the whole period of data is used to evaluate the robustness. The RMSE of an algorithm is calculated only for its successful tracking outputs. Therefore, the longer an algorithm tracks successfully, the more error is likely to accumulate. It implies that evaluating algorithms purely by ATE could be misleading. On the other hand, considering only CR could also be misleading.

In order to demonstrate the efficiency of the proposed system, all experiments of Dynamic-VINS are performed on the embedded edge computing devices, HUAWEI Atlas200 DK and NVIDIA Jetson AGX Xavier. And the compared algorithms results are included from their original papers. Atlas200 DK has an 8-core A55 Arm CPU (1.6 GHz), 8 GB of RAM, and a 2-core HUAWEI DaVinci NPU. Jetson AGX Xavier has an 8-core ARMv8.2 64-bit CPU (2.25 GHz), 16 GB of RAM, and a 512-core Nvidia Volta GPU. And the results tested on both devices are named Dynamic-VINS-Atlas and Dynamic-VINS-Jetson, respectively. Yet, to the best of our knowledge, the proposed method is the best-performance real-time RGB-D inertial odometry for dynamic environments on resource-restricted embedded platforms.

## A. OpenLORIS-Scene Dataset

OpenLORIS-Scene [3] is a real-world indoor dataset with a large variety of challenging scenarios like dynamic scenes, featureless frames, and dim illumination. The results on the OpenLORIS-Scene dataset are shown in Fig. 6, including the results of VINS-Mono, ORB-SLAM2, and DS-SLAM from [3] as baselines.

The OpenLORIS dataset includes five scenes and 22 sequences in total. The proposed Dynamic-VINS shows the best robustness among the tested algorithms. In office scenes that are primarily static environments, all the algorithms can track successfully and achieve a decent accuracy. It is challenging for the pure visual SLAM systems to track stable features in home and corridor scenes that contain a large area of textureless walls and dim lighting. Thanks to the IMU sensor, the VINS systems show robustness superiority when the camera is unreliable. The scenarios of home and cafe contain a number of sitting people with a bit of motion, and market exists lots of moving pedestrians and objects with unpredictable motion. And the market scenes cover the largest area and contain highly dynamic objects, as shown in Fig. 5. Although DS-SLAM is able to filter out some dynamic features, its performance is still unsatisfactory. VINS-RGBD has a similar performance with Dynamic-VINS in relative static scenes, while VINS-RGBD’s accuracy drops in highly dynamic market scenes. The proposed Dynamic-VINS can effectively deal with complex dynamic environments and improve robustness and accuracy.

## B. TUM RGB-D Dataset

The TUM RGB-D dataset [30] offers several sequences containing dynamic objects in indoor environments. The highly dynamic fr3\_walking sequences are chosen for evaluation where two people walk around a desk and change chairs positions while the camera moves in different motions. As the VINS system does not support VO mode and the TUM RGB-D dataset does not provide IMU measurements, a VO mode is implemented by simply disabling modules relevant to IMU in Dynamic-VINS for experiments. The results are shown in Table I. The compared methods’ results are included from their original published papers. The algorithms based on ORB-SLAM2 and semantic segmentation perform better. Although

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/ff6f3d79188d690204ba33f2d7a2cbe3163f4b964a170965891a03d85304f677.jpg)  
Fig. 6. Per-sequence testing results with the OpenLORIS-Scene datasets. Each black dot on the top line represents the start of one data sequence. For each algorithm, blue dots indicate successful initialization moments, and blue lines indicate successful tracking span. The percentage value on the top left of each scene is the average correct rate; the higher the correct rate of an algorithm, the more robust it is. The float value on the first line below is average ATE RMSE and the values on the second line below are T.RPE and R.RPE from left to right, and smaller means more accurate.

TABLE I  
RESULTS OF RMSE OF ATE [m], T.RPE [m/s], AND R.RPE [◦/s] ON TUM RGB-D fr3\_walking DATASETS
<table><tr><td rowspan="2">Sequence</td><td colspan="3">ORB-SLAM2 [4]</td><td colspan="3">DS-SLAM [15]</td><td colspan="3">Ji et al. [24]</td><td colspan="3">Dynamic-VINS</td></tr><tr><td>ATE</td><td>T.RPE</td><td>R.RPE</td><td>ATE</td><td>T.RPE</td><td>R.RPE</td><td>ATE</td><td>T.RPE</td><td>R.RPE</td><td>ATE</td><td>T.RPE</td><td>R.RPE</td></tr><tr><td>fr3_walking_xyz</td><td>0.7521</td><td>0.4124</td><td>7.7432</td><td>0.0247</td><td>0.0333</td><td>0.8266</td><td>0.0194</td><td>0.0234</td><td>0.6368</td><td>0.0486</td><td>0.0578</td><td>1.6932</td></tr><tr><td>fr3_walking_static</td><td>0.3900</td><td>0.2162</td><td>3.8958</td><td>0.0081</td><td>0.0102</td><td>0.2690</td><td>0.0111</td><td>0.0117</td><td>0.2872</td><td>0.0077</td><td>0.0095</td><td>0.4581</td></tr><tr><td> $f r 3 \_ w a l k i n g \_ r p y$ </td><td>0.8705</td><td>0.4249</td><td>8.0802</td><td>0.4442</td><td>0.1503</td><td>3.0042</td><td>0.0371</td><td>0.0471</td><td>1.0587</td><td>0.0629</td><td>0.0595</td><td>5.0839</td></tr><tr><td>fr3_walking_half</td><td>0.4863</td><td>0.3550</td><td>7.3744</td><td>0.0303</td><td>0.0297</td><td>0.8142</td><td>0.0290</td><td>0.0423</td><td>0.9650</td><td>0.0608</td><td>0.0665</td><td>5.2116</td></tr></table>

TABLE II

ABLATION EXPERIMENT RESULTS OF RMSE OF ATE [m], T.RPE [m/s], AND R.RPE [◦/s] ON TUM RGB-D fr3\_walking DATASETS
<table><tr><td rowspan="2">Sequence</td><td colspan="3">W/O CIRCULAR MASK</td><td colspan="3">W/O OBJECT DETECTION</td><td colspan="3">W/O SEG-LIKE MASK</td><td colspan="3">W/O MCC</td></tr><tr><td>ATE</td><td>T.RPE</td><td>R.RPE</td><td>ATE</td><td>T.RPE</td><td>R.RPE</td><td>ATE</td><td>T.RPE</td><td>R.RPE</td><td>ATE</td><td>T.RPE</td><td>R.RPE</td></tr><tr><td>fr3_walking_xyz</td><td>0.9795</td><td>0.6156</td><td>6.2692</td><td>0.0592</td><td>0.0575</td><td>1.7181</td><td>0.0523</td><td>0.0608</td><td>1.7474</td><td>0.0676</td><td>0.0604</td><td>1.8020</td></tr><tr><td>fr3_walking_static</td><td>0.4111</td><td>0.4052</td><td>9.8985</td><td>0.3458</td><td>0.3136</td><td>9.2520</td><td>0.0305</td><td>0.0194</td><td>0.5463</td><td>0.0454</td><td>0.0229</td><td>0.5676</td></tr><tr><td>fr3_walking_rpy</td><td>0.4111</td><td>0.4052</td><td>9.8985</td><td>0.2138</td><td>0.1191</td><td>5.4847</td><td>0.1174</td><td>0.0729</td><td>5.5470</td><td>0.1236</td><td>0.0996</td><td>5.4196</td></tr><tr><td>fr3_walking_half</td><td>1.1218</td><td>0.6779</td><td>11.521</td><td>0.0988</td><td>0.0651</td><td>5.1839</td><td>0.0754</td><td>0.0672</td><td>5.1952</td><td>0.1748</td><td>0.1169</td><td>5.8525</td></tr></table>

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/641781d59dec37862d781e0c1f0a703e9a893027f5a08d7ab339cbf3555cc60a.jpg)  
Fig. 7. Results of Moving Consistency Check. Features without yellow circular are the outliers marked by the Moving Consistency Check module.

Dynamic-VINS is not designed for pure visual odometry, it still shows competitive performance and has a significant improvement over ORB-SLAM2.

To validate the effectiveness of each module in Dynamic-VINS, ablation experiments are conducted as shown in Table II. The system without applying circular masks (W/O CIRCU-LAR MASK) from the Section III-A and Section III-B fails to extract evenly distributed stable features, which seriously degrades the accuracy performance. Without the object detection (W/O OBJECT DETECTION), dynamic features introduce wrong constraints to impair the system’s accuracy. Dynamic-VINS-W/O-SEG-LIKE-MASK shows the results that mask all features in the bounding boxes. The background features help the system maintain as many stable features as possible to provide more visual constraints. The moving consistency check plays an important role when object detection fails, as shown in the column W/O-MCC.

## C. Runtime Analysis

This part compares VINS-Mono, VINS-RGBD, and Dynamic-VINS for runtime analysis. These methods are expected to track and detect 130 feature points, and the frames in Dynamic-VINS are divided into 7x8 grids. The object detection runs on the NPU/GPU parallel to the CPU. The average computation times of each module and thread are calculated on OpenLORIS market scenes; the results run on both embedded platforms are shown in Table III. It should be noted that the average computation time is only to be updated when the module is used. Specifically, in VINS architecture, the feature detection is executed at a consistent frequency with the state optimization

TABLE III  
AVERAGE COMPUTATION TIME [ms] OF EACH MODULE AND THREAD ON OPENLORIS market SCENES
<table><tr><td>Platforms</td><td>Mehods</td><td>Feature Tracking</td><td>Feature Detection</td><td>Tracking Thread*</td><td>Dynamic Feature Recognition Modules†</td><td>State Optimization</td><td>Optimization Thread*</td><td>Object Detection*</td></tr><tr><td rowspan="3">HUAWEI Atlas200 DK</td><td>VINS-Mono [2]</td><td>18.6226</td><td>58.2712</td><td>57.6301</td><td></td><td>76.5047</td><td>85.0247</td><td></td></tr><tr><td>VINS-RGBD [25]</td><td>20.6066</td><td>58.9413</td><td>81.4598</td><td></td><td>75.2211</td><td>83.3476</td><td></td></tr><tr><td>Dynamic-VINS</td><td>15.5350</td><td>1.7645</td><td>19.8980</td><td>1.3424</td><td>74.9509</td><td>82.4916</td><td>17.5850</td></tr><tr><td rowspan="3">NVIDIA Jetson AGX Xavier</td><td>VINS-Mono [2]</td><td>4.4990</td><td>14.3691</td><td>10.9123</td><td></td><td>49.5326</td><td>52.4842</td><td></td></tr><tr><td>VINS-RGBD [25]</td><td>4.1099</td><td>15.4521</td><td>11.9251</td><td></td><td>49.0472</td><td>52.3388</td><td></td></tr><tr><td>Dynamic-VINS</td><td>3.3649</td><td>0.9396</td><td>5.5416</td><td>0.4707</td><td>43.0424</td><td>47.5377</td><td>21.9211</td></tr></table>

\* Tracking Thread, Optimization Thread and Object Detection correspond to the three different threads shown in Fig. 1, respectively.  
Dynamic Feature Recognition Modules sum up the Dynamic Feature Recognition, Missed Detection Compensation, and Moving Consistency Check modules.

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/2da41250b84fb5a16f5941f6f5c1c5d6b227b5f363cb9df7cdfa097acfaceb82.jpg)  
Fig. 8. A compact aerial robot equipped with an RGB-D camera, an autopilot with IMUs, an onboard computer, and an embedded edge computing device. The whole size is about 255 165 mm.

![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/d6fefb41766fe59ac435fa176ccdf8217966d94e9590c22cf8a20dc6824cf8f2.jpg)  
(a) HITSZ campus

A compact aerial robot is shown in Fig. 8. An RGB-D camera (Intel Realsense D455) provides 30 Hz color and aligned depth images. An autopilot (CUAV X7pro) with an onboard IMU (ADIS16470, 200 Hz) is used to provide IMU measurements. The aerial robot is equipped with an onboard computer (Intel NUC, i7-5557 U CPU) and an embedded edge computing device (HUAWEI Atlas200 DK). These two computation resource providers play different roles in the aerial robot. The onboard computer charges for peripheral management and other core functions requiring more CPU resources, such as planning and mapping. The edge computing device as auxiliary equipment offers instant state feedback and object detection results to the onboard computer.

## D. Real-World Experiments

thread, which means the frequency of feature detection is lower than that of Feature Tracking Thread.

Large-scale outdoor datasets with moving people and vehicles on the HITSZ and THUSZ campus are recorded by the

(b) THUSZ campus  
![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/db871565cc48decbdd1ecc3e3b2a449c6a4f5e243b089c70e2b5f4e06e864087.jpg)

On edge computing devices with AI accelerator modules, the single-stage object detection method is computed by an NPU or GPU without costing the CPU resources and can output inference results in real-time. With the same parameters, Dynamic-VINS shows significant improvement in feature detection efficiency in both embedded platforms and is the one able to achieve instant feature tracking and detection in HUAWEI Atlas200 DK. The dynamic feature recognition modules (Dynamic Feature Recognition, Missed Detection Compensation, Moving Consistency Check) to recognize dynamic features only take a tiny part of the consuming time. For real-time application, the system is able to output a faster frame-to-frame pose and a higher-frequency imu-propagated pose rather than waiting for the complete optimization result.

Fig. 9. The estimated trajectories in the outdoor environment aligned with the Google map. The green line is the estimated trajectory from Dynamic-VINS, the red line is from VINS-RGBD, and the yellow line represents the loop closure that happened at the end of the dataset.  
![](images/2022_RGB-D_Inertial_Odometry_for_a_Resource-Restricted_Robot_/443f686e210cedd63ab59f940226cde6557627bfd70a8bcdbb6eac5a55915390.jpg)  
Fig. 10. Results of dynamic feature recognition in outdoor environments. The dynamic feature recognition modules are still able to segment dynamic objects but with a larger mask region.

handheld aerial robot above for safety. The total path lengths are approximately 800 m and 1220 m, respectively. The dataset has a similar scene at the beginning and the end for loop closure, while loop closure fails in the THUSZ campus dataset. VINS-RGBD and Dynamic-VINS run the dataset on NVIDIA Jetson AGX Xavier. The estimated trajectories and loop closure trajectory aligned with the Google map are shown in Fig. 9. In outdoor environments, the depth camera is limited in range and affected by the sunlight. The dynamic feature recognition modules can still segment dynamic objects but with a larger mask region, as shown in Fig. 10. Compared with loop closure results, Dynamic-VINS could provide a robust and stable pose estimation with little drift.

## V. CONCLUSION

This paper presents a real-time RGB-D inertial odometry for resource-restricted robots in dynamic environments. Costefficient feature tracking and detection methods are proposed to cut down the computing burden. A lightweight object-detectionbased method is introduced to deal with dynamic features in real-time. Validation experiments show the proposed system’s competitive accuracy, robustness, and efficiency in dynamic environments. Furthermore, Dynamic-VINS is able to run on resource-restricted platforms to output an instant pose estimation. In the future, the proposed approaches are expected to be validated on the existing popular SLAM frameworks. The missed detection compensation module is expected to develop into a moving object tracking module, and semantic information will be further introduced for high-level guidance on mobile robots or mobile devices in complex dynamic environments.

## REFERENCES

[1] J. Engel, V. Koltun, and D. Cremers, “Direct sparse odometry,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 3, pp. 611–625, Mar. 2018.

[2] T. Qin, P. Li, and S. Shen, “VINS-Mono: A robust and versatile monocular visual-inertial state estimator,” IEEE Trans. Robot., vol. 34, no. 4, pp. 1004–1020, Aug. 2018.

[3] P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, and G. Huang, “OpenVINS: A research platform for visual-inertial estimation,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 4666–4672.

[4] R. Mur-Artal and J. D. Tardos, “ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1255–1262, Oct. 2017.

[5] B. Triggs, P. F. McLauchlan, R. I. Hartley, and A. W. Fitzgibbon, “Bundle adjustment—a modern synthesis,” in Proc. Int. Workshop Vis. Algorithms, 1999, pp. 298–372.

[6] M. A. Fischler and R. C. Bolles, “Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography,” Commun. ACM, vol. 24, no. 6, pp. 381–395, 1981.

[7] Y. Sun, M. Liu, and M.Q.-H. Meng, “Improving RGB-D SLAM in dynamic environments: A motion removal approach,” Robot. Auton. Syst., vol. 89, pp. 110–122, 2017.

[8] E. Palazzolo,, J. Behley, P. Lottes, P. Gigu, and C. Stachniss, “ReFusion: 3D reconstruction in dynamic environments for RGB-D cameras exploiting residuals,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 7855–7862.

[9] W. Dai et al., “RGB-D SLAM in dynamic environments using point correlations,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 44, no. 1, pp. 373–389, Jan. 2022.

[10] W. Liu et al., “SSD: Single shot MultiBox detector,” in Eur. Conf. Comp. Vis., 2016, pp. 21–37.

[11] J. Redmon and A. Farhadi, “YOLOv3: An incremental improvement,” 2018, arXiv:1804.02767.

[12] V. Badrinarayanan, A. Kendall, and R. Cipolla, “SegNet: A deep convolutional encoder-decoder architecture for image segmentation,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 39, no. 12, pp. 2481–2495, Dec. 2017.

[13] K. He, G. Gkioxari, P. Dollar, and R. Girshick, “Mask R-CNN,” in Proc. IEEE Int. Conf. Comput. Vis., 2017, pp. 2961–2969.

[14] L. Xiao et al., “Dynamic-SLAM: Semantic monocular visual localization and mapping based on deep learning in dynamic environment,” Robot. Auton. Syst., vol. 117, pp. 1–16, 2019.

[15] C. Yu et al., “DS-SLAM: A semantic visual SLAM towards dynamic environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 1168–1174.

[16] B. Bescos,, J. M. Facil, J. Civera, and J. Neira, “DynaSLAM: Tracking, mapping, and inpainting in dynamic scenes,” IEEE Robot. Automat. Lett., vol. 3, no. 4, pp. 4076–4083, Oct. 2018.

[17] F. Zhong, S. Wang, Z. Zhang, C. Chen, and Y. Wang, “Detect-SLAM: Making object detection and SLAM mutually beneficial,” in Proc. IEEE Winter Conf. Appl. Comput. Vis., 2018, pp. 1001–1010.

[18] Y. Liu and J. Miura, “RDS-SLAM: Real-time dynamic SLAM using semantic segmentation methods,” IEEE Access, vol. 9, pp. 23 772–23 785, 2021.

[19] I. Ballester, A. Fontán, J. Civera, K. H. Strobl, and R. Triebel, “DOT: Dynamic object tracking for visual SLAM,” in Proc. IEEEInt. Conf. Robot. Automat., 2021, pp. 11 705–11 711.

[20] K. Schauwecker, N. R. Ke, S. A. Scherer, and A. Zell, “Markerless visual control of a quad-rotor micro aerial vehicle by means of on-board stereo processing,” in Proc. Auton. Mobile Syst., 2012, pp. 11–20.

[21] Z. Z. Nejad and A. Hosseininaveh Ahmadabadian, “ARM-VO: An efficient monocular visual odometry for ground vehicles on ARM CPUs,” Mach. Vis. Appl., vol. 30, no. 6, pp. 1061–1070, 2019.

[22] S. Bahnam, S. Pfeiffer, and G. C. H. E. de Croon, “Stereo visual inertial odometry for robots with limited computational resources,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 9154–9159.

[23] G. Younes et al., “Keyframe-based monocular SLAM: Design, survey, and future directions,” Robot. Auton. Syst., vol. 98, pp. 67–88, 2017.

[24] T. Ji, C. Wang, and L. Xie, “Towards real-time semantic RGB-D SLAM in dynamic environments,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 11 175–11 181.

[25] Z. Shan, R. Li, and S. Schwertfeger, “RGBD-inertial trajectory estimation and mapping for ground robots,” Sensors, vol. 19, no. 10, 2019, Art. no. 2251.

[26] C. Forster et al., “IMU preintegration on manifold for efficient visualinertial maximum-a-posteriori estimation,” in Proc. Robot.: Sci. Syst., 2015.

[27] B. D. Lucas et al., “An iterative image registration technique with an application to stereo vision,” in Proc. DARPA Image Understanding Workshop, 1981, pp. 121–130.

[28] E. Rosten and T. Drummond, “Machine learning for high-speed corner detection,” in Proc. Eur. Conf. Comput. Vis., 2006, pp. 430–443.

[29] X. Shi et al., “Are we ready for service robots? The OpenLORIS-Scene datasets for lifelong SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 3139–3145.

[30] J. Sturm,, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2012, pp. 573–580.