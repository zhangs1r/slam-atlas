# RGB-D SLAM in Dynamic Environments Using Point Correlations

Weichen Dai , Yu Zhang , Ping Li, Zheng Fang , and Sebastian Scherer

Abstract—In this paper, a simultaneous localization and mapping (SLAM) method that eliminates the influence of moving objects in dynamic environments is proposed. This method utilizes the correlation between map points to separate points that are part of the static scene and points that are part of different moving objects into different groups. A sparse graph is first created using Delaunay triangulation from all map points. In this graph, the vertices represent map points, and each edge represents the correlation between adjacent points. If the relative position between two points remains consistent over time, there is correlation between them, and they are considered to be moving together rigidly. If not, they are considered to have no correlation and to be in separate groups. After the edges between the uncorrelated points are removed during point-correlation optimization, the remaining graph separates the map points of the moving objects from the map points of the static scene. The largest group is assumed to be the group of reliable static map points. Finally, motion estimation is performed using only these points. The proposed method was implemented for RGB-D sensors, evaluated with a public RGB-D benchmark, and tested in several additional challenging environments. The experimental results demonstrate that robust and accurate performance can be achieved by the proposed SLAM method in both slightly and highly dynamic environments. Compared with other state-of-the-art methods, the proposed method can provide competitive accuracy with good real-time performance.

Index Terms—SLAM, motion estimation, dynamic environments

## 1 INTRODUCTION

recent years, vision-based motion estimation methods, Iincluding visual odometry (VO) [1] and visual simultaneous localization and mapping (vSLAM) [2], [3], have played an important role in robotic navigation thanks to the low cost and weight of cameras. These methods can provide six degree-of-freedom motion estimation using only input images. However, the scenarios for which they are suitable are strictly limited by their assumptions of a static world. In reality, the methods based on the static world assumption are influenced by or even fail because of moving objects appearing in the field of view (FOV). A scene containing moving objects is referred to as a dynamic environment. According to the area of the FOV that is occupied by moving objects, dynamic environments can be categorized as slightly or highly dynamic environments. Because only a small part of the FOV is covered by moving objects in slightly dynamic environments, traditional robust estimation methods such as random sample consensus (RANSAC) [4] methods and robust weighting functions [5], [6] can eliminate most of the influence of moving objects. In contrast, if a large part of the FOV is covered by moving objects, there are more observations of moving objects than observations of the static scene, which causes robust estimation methods to fail. Therefore, traditional VO and vSLAM, which assume a static world, have limited applications in practice, and eliminating the influence of these moving objects has become an important topic.

For solving the problem of motion estimation failure caused by moving objects in dynamic environments, researchers utilize multiple types of prior information such as motion consistency and semantic information. For motion consistency, the map points on the same rigid moving object have a motion consistency that is independent of the static scene. Thus, some methods [7], [8] utilize motion consistency to determine which points are on a moving object. However, the real world also includes objects that are nonrigid owing to their dynamics and inherent deformability. Therefore, this type of method cannot determine all of the map points on the moving objects. In addition to motion consistency, deeplearning-based methods [9], [10] learn semantic information from a training set as prior information to separate suspected moving objects from the static scene directly on the image. However, these methods cannot recognize unknown objects that were not present in the training set. Moreover, their computational requirements mean that they can often struggle to run in real time in an embedded context. Hence, it still is challenging for vSLAM and VO to provide robust and accurate navigation information in dynamic environments.

In contrast to the above two types of methods, in this study, the correlation between points is used to address the interference caused by moving objects. For simplicity, the map points on moving objects and in the static scene are called dynamic and static points, respectively. A correlation exists between static points, but there is no correlation between dynamic and static points. A segmentation method based on point correlations is proposed to exploit the connectedness of map points and separate moving objects from the static scene. The core idea of the proposed segmentation method consists of two parts. First, only the correlations between adjacent points are considered to reduce the computational requirements. In addition, through optimization of the point correlations, the edges connecting points with no correlation are removed to separate the graph. Finally, as shown in Fig. 1, a simultaneous localization and mapping (SLAM) method that uses this segmentation method is proposed to eliminate the influence of moving objects in dynamic environments. The proposed SLAM method can accurately estimate the camera pose while eliminating the influence of slowly moving objects using the history information in previous images. The proposed SLAM method is called dynamic SLAM (DSLAM).

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/ccb5945f69478fad2d81bdc807eca91b076d65c9db0036b90d8cb6d1aff12b32.jpg)  
Fig. 1. Results of static-point determination obtained by the proposed method. The edges between static points are shown as green lines. The features on moving objects and in the static scene are shown in pink and green, respectively, and the features in the static scene are correctly determined. Because the visual descriptor’s performance is limited, a few dynamic points are inevitably matched to the features in the static scene.

The main contributions of this paper are as follows:

A segmentation method using point correlations is proposed to separate static and dynamic points. It can exploit temporal information from multiple frames to extend the captured view and is not limited to RGB-D sensors as long as the sensor can provide point-correlation measurements.

A SLAM method using RGB-D sensors is proposed to improve the robustness and accuracy of motion estimation in dynamic environments.

The rest of the paper is structured as follows. The related work is discussed in Section 2. Section 3 explains why dynamic points should be excluded from motion estimation. Section 4 explains how to use the point correlations in detail. Section 5 details the SLAM system. Section 6 presents and discusses the experimental results, and Section 7 presents the conclusions and plans for future work. Our experimental results can also be seen in a publicly available video.<sup>1</sup>

## 2 RELATED WORK

Vision-based motion estimation methods can be categorized as filter-based methods [11], [12] and factor graph optimization-based methods [13], [14], [15]. Because factor graph optimization is more accurate and efficient than the original approaches for SLAM based on nonlinear filtering, most advanced visual systems are based on it [16].

Factor-graph-optimization-based methods can be further divided into two categories: indirect (feature-based) methods [17], [18], [19] and direct methods [20], [21], [22]. The difference between these two methods is that indirect methods use the reprojection error of the feature points, whereas direct methods directly utilize the photometric error of the raw images. Both methods have their advantages and disadvantages. Indirect methods can make full use of the geometric information in the correspondences between feature points and are robust to geometric noise. Direct methods can skip the precomputation step to save computational resources. Moreover, direct methods have the ability to reconstruct a dense map.

Although the methods described above can provide excellent performance in static environments, current visionbased motion estimation methods often fail when the environment is too challenging (e.g., in highly dynamic environments) [23], [24]. Existing robust estimation methods can only deal with part of the interference of moving objects in slightly dynamic environments. Therefore, many methods have been proposed to address these problems and can be categorized into two main types: methods based on motion consistency and those based on learning.

(1) Methods based on motion consistency: Methods based on motion consistency find dynamic points on the same moving object using points with motion consistency.

Most methods treat moving objects as noise, which they filter out as soon as possible. Alcantarilla et al. [25] use scene flow to distinguish moving objects from the static scene, but the calculation of the scene flow is based on a result estimated by VO. Therefore, the camera pose must be estimated twice for each frame, which increases the computation time. Azartash et al. [26] first partition RGB-D images. Then, the motion of each region is individually estimated to determine the regions belonging to the moving object. Their experimental results show that the accuracy slightly improves in dynamic environments. Zhang et al. [27] estimate optical flow using PWC-net and apply it to dynamic segmentation. Stuckler and Behnke [28] achieved good performance when€ segmenting RGB-D images into pixel regions, but this segmentation is still time-consuming. Sun et al. [29] use the image difference and depth segmentation to filter the RGB-D data that are associated with moving objects. However, part of the depth data in the static background that is close to moving objects is also removed. Li and Lee [30] proposed a real-time depth edge-based RGB-D SLAM system. In the point cloud registration, each edge point has a static weight indicating the likelihood it is a part of the static background.

Some methods focus on enforcing spatial or temporal coherence among the detected dynamic points in consecutive frames. Kim and Kim [31] obtain static regions in images by computing the depth differences between consecutive frames. However, only some of the regions belonging to frames. However, only some of the regions belonging to moving objects have depth changes. Therefore, not all regions belonging to moving objects can be obtained. Similarly, Jaimez et al. [32] introduced a joint VO and scene flow estimation method. Furthermore, Scona et al. [33] designed a segmentation by coupling camera motion residuals, depth inconsistency, and a regularization term.

In other approaches, external sensors such as an inertial measurement unit (IMU) are leveraged to solve this problem. Kim et al. [34] combined an RGB-D camera with an IMU to estimate the camera pose. They regard the IMU information, which is relatively accurate over a short time interval, as a prior for filtering incorrect visual information from moving objects. However, the reliance on the extra IMU limits this method to scenarios with IMU sensors.

Detecting moving objects using only RGB cameras is also a related research topic [35]. Most methods relying on geometric constraints leverage the properties of epipolar geometry to segment static and dynamic features. The constraints can be derived from the equation of triangulation [36] or fundamental matrix estimation [37]. Notably, Tan et al. [38] proposed a prior-based adaptive RANSAC algorithm to categorize points according to reprojection error. Then, the hypothesis that the texture of the static scene is evenly distributed is used to determine static points. However, this hypothesis may fail in situations where most of the FOV is covered by moving objects. Another approach also leverages the reprojection error. Zou et al. [39] use multiple cameras that can capture a broader view to avoid occlusion. To benefit from this additional information, they employ intercamera pose estimation and intercamera mapping to deal with dynamic objects and enhance the system’s robustness. In the proposed method, point correlation is employed to segment the static and dynamic points. In contrast to the reprojection error, which only considers the correlation between frame and map points, the point correlation further determines the correlation between map points. Therefore, in contrast to the expansion of the captured view using multiple cameras, the method proposed in this paper expands the captured view using multiple temporal frames.

The methods described above provide good performance in dynamic environments. However, it is difficult for them to maintain robust motion estimation when a slowly moving object enters the FOV. Moreover, most methods can only be applied to a specific sensor.

(2) Methods based on learning: Other methods, such as [40], [41], [42], use pretrained learning-based methods to detect potential moving objects. Most methods of this type treat dynamic region features as outliers or exclude those regions from the image.

Kitt et al. [43] classify feature points to distinguish dynamic and static points. However, the classifier has to be trained in advance, which prevents this method from being used to explore unknown environments. Riazuelo et al. [44] used semantic information to partition the image regions of people to eliminate the influence of walking people. B^arsan et al. [45] also used an instance-aware semantic segmentation algorithm to recognize moving objects from a single image. Bescos et al. [9] proposed DynaSLAM, which combines a Mask R-CNN [46] prior with multi-view geometry to partition an image and determine which regions in the image belong to moving objects. Moreover, some other methods additionally track moving objects to provide a complete three-dimensional (3D) map and the trajectory of these moving objects. Qiu et al. [47] eliminate the interference of moving objects using visual-inertial sensing and use Re-3 [48] to track moving objects. Yang et al. [49] utilize moving objects and motion model constraints to improve the camera pose estimation. Strecke et al. [50] initially detected and segmented new movable instances using Mask R-CNN. Runz€ and Agapito [51] partition an image using semantic cues to improve the accuracy of segmentation. However, only a few 3D models of different objects are maintained, and the method only works at a low frame rate. To robustly estimate motion, Xu [52] proposed a system to generate an object-level dynamic volumetric map from a single RGB-D camera.

These approaches can work well in environments that only contain the types of objects on which the classifier was trained, but they can fail in the presence of unknown objects that are not present in the training set. Furthermore, stateof-the-art segmentation methods such as Mask R-CNN are still computationally intensive and often have to be run in a background thread in practice to amortize their cost over several frames [10].

In contrast to the above methods, we propose a method based on point correlations. Instead of preprocessing the image as in image segmentation methods, the proposed method relies on the fact that there are correlations between two static points but not between dynamic and static points. Because the point correlations can be obtained irrespective of the type of sensors used, the proposed method has the potential to be applied to various types of sensors.

## 3 PROBLEM STATEMENT

In this section, we briefly introduce motion estimation based on the static world assumption and explain why estimation is influenced by moving objects.

Bundle adjustment [53] under the static world assumption is used in most VO and vSLAM methods. As shown in Fig. 2, the states ${ \bf T } _ { k }$ and $\boldsymbol { \mathsf { p } } _ { i }$ are estimated using bundle adjustment. Here, $\mathbf { T } _ { k }$ is the transformation matrix representing the pose of the sensor at time k, and $\mathbf { p } _ { i }$ is the ith coordinate representing the position of the ith map point, where $k = 1 , \ldots , K$ and $i = 1 , \dots , M$ . All states are represented as

$$
\begin{array} { r } { { \mathbf { x } } = \{ { \mathbf { T } } _ { 1 } , . . . , { \mathbf { T } } _ { K } , { \mathbf { p } } _ { 1 } , . . . , { \mathbf { p } } _ { M } \} , } \end{array}\tag{1}
$$

where $\mathbf { x } _ { i k } = \{ \mathbf { T } _ { k } , \mathbf { p } _ { i } \}$ is the subset of states, including the kth pose and ith map point. Regardless of the type of visual sensors used, the measurement ${ \bf y } _ { i k }$ corresponding to the observation of point i from pose k can be expressed as

$$
\begin{array} { r } { { \bf y } _ { i k } = { \bf g } ( { \bf x } _ { i k } ) + { \bf n } _ { i k } , } \end{array}\tag{2}
$$

where $\mathbf { g } ( \mathbf { x } _ { i k } ) = \mathbf { s } ( \mathbf { r } ( \mathbf { x } _ { i k } ) )$ is the measurement model, $\mathbf { r } ( \mathbf { x } _ { i k } ) =$ $\mathbf { T } _ { k } \cdot \overline { { \mathbf { p } } } _ { i } , \mathbf { s } ( \cdot )$ is the nonlinear sensor model producing observations from the point in the frame (e.g., the camera model), the notation p makes an augmented vector from a 3D point $ { \mathbf { p } } \in \mathbb { R } ^ { 3 }$ to homogeneous coordinates, and $\mathbf { n } _ { i k } \sim \mathcal N ( \pmb { \mu } _ { i k } , \mathbf { C } _ { i k } )$ is additive Gaussian noise, where $\mu _ { i k } = ( 0 )$ and $\mathbf { C } _ { i k }$ denotes the Gaussian noise covariance associated with ${ \bf y } _ { i k }$

Typically, a maximum likelihood approach is used. This approach finds the optimal states x that maximize the probability of obtaining the actual measurements, ${ \bf x } ^ { * } =$ ptember 16,2026 at 03:33:56 UTC from IEEE Xplore. Restrictions apply.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/eca9f71492ed582ba9dd9e196d4c39998b49230f9ccaad0b54db49553660a3dd.jpg)  
Fig. 2. Illustration of the negative effects caused by dynamic points. The gray circles are the correct results from pose estimation. The white circles are static map points, and the pink circles are points on a moving object. The movement of dynamic points destroys the consistency of motion estimation, as shown in the figure, where the unrecognized dynamic points influence the estimation of $\mathbf { T } _ { k } ,$ so it moves from the correct estimation result (gray circle) to the incorrect result (blue circle). Besides, the movement of the dynamic points also destroys the point correlations between points $\mathbf { p } _ { i }$ and $\boldsymbol { \mathsf { p } } _ { j }$

argmax $P ( \mathbf { y } | \mathbf { x } ) .$ , where y represents all measurements. Traditional methods neglect the correlations between points, so that they can perform the estimation in real time, and the objective function is hence written as

$$
J _ { b a } ( { \bf x } ) = \frac { 1 } { 2 } \sum _ { i , k } { \bf e } _ { y , i k } ( { \bf x } ) ^ { T } { \bf C } _ { i k } ^ { - 1 } { \bf e } _ { y , i k } ( { \bf x } ) ,\tag{3}
$$

where x is the full state that we wish to estimate and

$$
\mathbf { e } _ { y , i k } ( \mathbf { x } ) = \mathbf { y } _ { i k } - \mathbf { g } ( \mathbf { x } _ { i k } ) .\tag{4}
$$

The usual approach to this estimation problem is to apply the Gauss–Newton method. In the Gauss–Newton method, the Hessian structure of the objective function in Eq. (3) is

$$
\mathbf { H } = \left[ \begin{array} { c c } { \mathbf { H } _ { p p } } & { \mathbf { H } _ { p g } } \\ { \mathbf { H } _ { g p } } & { \mathbf { H } _ { g g } } \end{array} \right] ,\tag{5}
$$

where $\mathbf { H } _ { p p } , \mathbf { H } _ { p g } ,$ and $\mathbf { H } _ { g g }$ denote the pose–pose, pose–geometry, geometry–geometry blocks, respectively. Here, “pose” refers to the pose parameters of all of the camera poses in $\mathbf { x , }$ and “geometry” refers to the geometry parameters of all of the map points in x. Because $\mathbf { H } _ { g g }$ is diagonal, methods such as Cholesky factorization and the Schur complement method [54], [55] can be used to solve it efficiently.

The above descriptions all assume a static world. For dynamic environments, the dynamic points do not satisfy the measurement model in Eq. (2). The measurement model of a dynamic point $\mathbf { p } _ { d }$ should be

$$
{ \bf y } _ { d k } = { \bf g } ( { \bf x } _ { d k } + { \bf v } _ { d k } T ) + { \bf n } _ { d k } ,\tag{6}
$$

where ${ \bf v } _ { d k }$ is the average velocity of the moving object over the interval $T$ between time stamps $k - 1$ and k. If the measurement model in Eq. (2) is applied to the dynamic points, ${ \mathbf v } _ { d k } T$ cannot be modeled and will generate extra noise, thereby jeopardizing the estimation result. Some robust methods, such as RANSAC, can only be used to reduce the influence of moving objects in slightly dynamic environments. However, robust estimation methods will also fail if there are moving objects with rich texture occupying the majority of the FOV. Therefore, errors will be introduced into the result if methods based on the static world assumption are used for dynamic environments. The estimation could even fail.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/19105a2db765510883c56b1dc907ecf7f17745b3eb85e43690aa0f84cb610637.jpg)  
Fig. 3. Overview of the proposed segmentation method, which comprises three steps to divide the point cloud into different components with different motion patterns: graph initialization, inconsistent edge culling, and point graph segmentation.

In dynamic environments, motion estimation methods should distinguish between static and dynamic points and then apply the measurement models in Eqs. (2) and (6) separately. In reality, the velocities of moving objects cannot be obtained without external sensors. Because the velocities are unknown, only static points using the model in Eq. (2) should be used to acquire the result. Therefore, the static points must be accurately determined so that the map points used for motion estimation do not contain dynamic points. A segmentation method for determining the static points is proposed in the next section.

## 4 SEGMENTATION USING POINT CORRELATIONS

The proposed segmentation method is based on point correlations. Because the relative positions between static points do not change with time whereas the relative positions between static and dynamic points change with time.

The proposed segmentation method can be divided into three steps, as shown in Fig. 3. In the first step, we build a graph to represent the correlations between adjacent map points. The second step is to optimize the geometric parameters of all map points and remove inconsistent edges to partition the graph. In the third step, a depth-first search (DFS) is used to determine connected components. The points in the same component have consistent motion, and the points in different components have independent motion patterns. Fig. 4 shows a 2D example of two frames in which the static and dynamic point groups are separated.

## 4.1 Step 1: Graph Initialization

In this step, a graph is built using map points to represent point correlations. Each edge of the graph $\mathcal { G }$ represents a possible correlation between two connected points. The edge between points $\boldsymbol { \mathsf { p } } _ { i }$ and ${ \bf p } _ { j }$ is indexed as a vector $\mathbf { l } _ { i j } ,$ defined as follows:

$$
\mathbf { l } _ { i j } = \mathbf { p } _ { i } - \mathbf { p } _ { j } .\tag{7}
$$

Next, Delaunay triangulation [56] is applied to reduce the complexity of the construction of the 3D point graph. In Delaunay triangulation, for a given set of discrete points, triangulation is carried out such that no point is inside the ptember 16,2026 at 03:33:56 UTC from IEEE Xplore. Restrictions apply.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/550079472fe8fc2a25aadc0db955774211a6052b2252257a436c03b06f42935e.jpg)  
(a)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/7b12d8c555886b77b28e3e1f60e321453257fa5e07f9709e03433bc4340b0233.jpg)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/3bd3d8e146831f73357e07bb9b0551ab981d67947415eb7b384cc67c2a2f330e.jpg)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/57b6c6c98223f94af4687f967f423214b896d3b2dbe7c1a51fa1564d95599a8d.jpg)

(b)  
(d)  
![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/eacde7d1bc429835cafc694e551bee8f189f498d5eade39cdbf3b80652355b29.jpg)  
(e)

(c)  
![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/a51b97c09562acb2b6ce6c2ef16a2cb2996b92ea3339045d1f4c0b514676bff5.jpg)  
(f)  
Fig. 4. Two-dimensional example of the segmentation method using point correlations: (a) current frame; (b) reference frame; (c) structure graph of the matched feature points in the reference frame created by Delaunay triangulation; (d) graph after removing inconsistent edges; (e) extracted largest region, which belongs to the static scene; and (f) moving object region separated from the static scene.

circumcircle of any triangle. It maximizes the minimum angle of all of the angles of the triangles. Therefore, only adjacent feature points are connected in the graph. The generated graph structure is similar to a sparse mesh, as shown in Fig. 4c. In other words, because of its sparsity, only the correlations between adjacent connected points are verified in the next step.

## 4.2 Step 2: Inconsistent Edge Culling

The initial graph G obtained from Step 1 represents the correlation between adjacent map points. At time stamp k, the point-correlation measurement $\mathbf { z } _ { i j k }$ of $\mathbf { l } _ { i j k }$ has the form

$$
{ \bf z } _ { i j k } = { \bf y } _ { i k } - { \bf y } _ { j k } = { \bf h } ( { \bf l } _ { i j k } ) + { \bf n } _ { i j k } ,\tag{8}
$$

where $\mathbf { l } _ { i j k }$ is the edge between p and p at time stamp k, ${ \bf n } _ { i j k }$ is additive Gaussian noise, and hðÞ is the observation model of the edge, derived as

$$
\mathbf { h } ( \mathbf { l } _ { i j k } ) = \mathbf { s } ( \mathbf { r } ( \mathbf { x } _ { i k } ) ) - \mathbf { s } ( \mathbf { r } ( \mathbf { x } _ { j k } ) ) .\tag{9}
$$

The set of measurements collected up to time k is

$$
\mathbf { z } _ { k } = \{ \mathbf { z } _ { i j k } \} _ { ( i j ) \in \mathcal { G } } .\tag{10}
$$

Note that we use the simpler notation

$$
\mathbf { z } = \{ \mathbf { z } _ { 0 } , \ldots , \mathbf { z } _ { k } \}\tag{11}
$$

to express all of the measurements that are available. The set of all the geometric parameters of the map points is denoted as $\mathbf { \boldsymbol { x } } _ { g } ,$ which is a subset of x. Similar to bundle adjustment, we also set up the estimation of $\mathbf { \boldsymbol { x } } _ { g }$ using the maximum likelihood framework. The objective function of point-correlation optimization is defined as

$$
J _ { p } ( \pmb { x } _ { g } ) = \frac { 1 } { 2 } \sum _ { i j , k } \mathbf { e } _ { z , i j k } ( \mathbf { 1 } _ { i j k } ) ^ { T } \mathbf { C } _ { i j k } ^ { - 1 } \mathbf { e } _ { z , i j k } ( \mathbf { 1 } _ { i j k } ) ,\tag{12}
$$

where $\mathbf { C } _ { i j k }$ is the covariance matrix associated with the ijkth measurement and $\mathbf { e } _ { z , i j k }$ is the error term and is defined as

$$
\begin{array} { r } { \mathbf { e } _ { z , i j k } ( \mathbf { x } _ { g } ) = \mathbf { z } _ { i j k } - \mathbf { h } ( \mathbf { l } _ { i j k } ) . } \end{array}\tag{13}
$$

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/40c0063919651968b7be44a45b173825f16bb057391447b23c5e3d35a65ad658.jpg)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/f8595076eac2aa9eefb77d7ffcd97fd11a2822183fd04e3e2ee35dd7d7a439d8.jpg)  
(b)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/0f98b07bb9475db0c645ec6b4e2cb561d7db09bb41aa32d6425de2f843d0dbd6.jpg)  
(c)  
Fig. 5. Example of a Hessian structure. (a) Hessian structure of bundle adjustment in Eq. (3). Because the geometry–geometry block is diagonal, the Hessian can be efficiently solved using the Schur complement. (b) Hessian structure of the objective function in Eq. (14). The geometry– geometry block of the Hessian structure is the green block with the pink diagonal. (c) The result after the inconsistent edge observations are removed from the Hessian structure. The green blocks indicate the point correlations, and the blue blocks indicate the geometry–pose correlations. The red blocks indicate moving objects in the camera image. Therefore, if inconsistent measurements in the point correlations can be determined, the two point clusters with different motion consistencies are separated.

Combining the objective function of the bundle adjustment problem in Eq. (3), the final objective function is derived as

$$
J ( { \bf x } ) = J _ { p } ( { \bf x } _ { g } ) + J _ { b a } ( { \bf x } ) .\tag{14}
$$

As shown in Fig. 2, the estimated pose at time k is disturbed by interference from a moving object. Because the relative positions between static and dynamic points have changed, the edges between these points no longer have consistent observations in multiple frames. These edges are called inconsistent edges, meaning that there are no correlations between connected map points. In the process of optimizing $\scriptstyle { \boldsymbol { x } } _ { p } ,$ the squared Mahalanobis length of the error is used to determine whether a measurement is an outlier, and a chi-squared value is chosen as the threshold according to the P-value. the measurements of inconsistent edges cannot fit the point-correlation model and are removed as outliers in the iterations. Therefore, if all observations of an edge are outliers after optimization, this edge is identified as an inconsistent edge and removed from G. Because there are no correlations between any dynamic and static points, the remaining graph G corresponds to the segmentation result.

The process of culling the inconsistent edges can also be shown as changes in the Hessian structure of the objective function in Eq. (14). As shown in Fig. 5, the geometry– geometry block on the Hessian side is divided into multiple independent blocks after the outlier observations corresponding to the inconsistent edges have been removed. The points in the blocks do not correlate with points in the other blocks. From the example shown in Fig. 5c, two point groups that have their own motion patterns have been separated. However, objective function JðxÞ in Eq. (14) destroys the specific sparsity pattern of the Hessian [54], [55] of the objective function in Eq. (3). The introduction of correlations between the geometric parameters makes joint optimization real-time infeasible [57], which depends on the blockdiagonal geometry–geometry block. For retaining the efficiency of solving the bundle adjustment problem, the optimization of the objective function in Eq. (12) must be performed separately from bundle adjustment. In addition, the Huber norm, as used in bundle adjustment, is also applied to the edge residuals to improve robustness.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/95485879d2bf0f4766fa8d5f1f4f5414ac1fdc897e6987867f803559c4a17693.jpg)  
Fig. 6. System overview. Static point determination is performed in both the front and back ends. Before tracking the local map. the dynamic points are removed from the estimation data. In the back end. the determination of static points and local mapping simultaneously occur. The shaded rec tangles indicate components specifically modified for dynamic environments.

The 2D example shown in Fig. 4 illustrates this process. After the camera position changes, the distances between static points do not significantly change, whereas the distances between static and dynamic points change by large amounts. Therefore, the edges between static and dynamic points are removed. The remaining connected components of the graph describe the separated moving objects and static scene. As a result, the person shown in Fig. 4f is successfully separated from the static background.

## 4.3 Step 3: Determination of Static Points

After removing the inconsistent edges, search algorithms such as the DFS algorithm are used to check whether G is divided into several isolated subgraphs (connected components). If there are different connected components, these components represent different point groups with different motion patterns.

For determining which points are reliable static points, the connected component with the largest volume is assumed to consist of reliable static points for two reasons:

In a map built over a period of time, the spatial volume of the static points, whose volumes grow the most over time, is generally the largest.

Static points are generally evenly distributed in 3D space, whereas dynamic points are generally only distributed on a surface because the camera can only observe one side of the surface of a moving object. Therefore, the volume of the dynamic points will be much smaller than that of the static points.

For these reasons, the points of the connected component with the largest volume are identified as the reliable static points. In the subsequent calculations, only reliable static points are used for motion estimation.

## 5 DSLAM IN DYNAMIC ENVIRONMENTS USING RGB-D SENSORS

A SLAM method that integrates the segmentation method proposed in Section 4 called dynamic SLAM (DSLAM) is proposed for dynamic environments. This SLAM method is implemented on RGB-D sensors, which provide color images with a synchronized depth image. DSLAM is built on ORB-SLAM2 [19] and consists of front and back ends.

The entire pipeline is illustrated in Fig. 6, where each component modified for dynamic environments is shown in gray. As shown in Fig. 6, the static point determination module implementing the proposed segmentation method is added to eliminate the influence of moving objects. Moreover, the map points are divided into marked and unmarked points. The marked points indicate that a map point is a reliable static label. The unmarked points indicate that a map point cannot be reliably identified as a static point. Note that all new map points are initialized as marked points and are checked by the static point determination module later.

The front end is responsible for estimating the motion between consecutive frames and determining when to insert a new keyframe. Therefore, the front-end tasks consist of motion estimation, determination of the static points in the current frame, and determination of new keyframes. Because RGB-D sensors can capture 3D information, the initialization of the system is finished when the first frame has created a sufficient number of available map points.

After initialization is complete, the specific steps are as follows. For each new frame, initial feature matching is performed with the marked points that were successfully tracked in the previous frame. Then, in the initial pose estimation step, a pose is estimated using motion-only bundle adjustment with all matches. This step excludes the incorrect feature matching results. In the initial pose estimation step, note that the features of fast-moving objects are rarely matched with the existing map points. Meanwhile, this step can also exclude some of the correspondences on fast-moving objects with significant reprojection error. Therefore, even if the initial pose is obtained before the static point determination, only a few features related to moving objects may jeopardize motion-only bundle adjustment. If the tracking is not lost, the static point determination step is performed for the tracked map points. Afterward, the matches between the untracked features and the local map points are searched for by reprojection and the pose is optimized again in the track local map step. Finally, the tracking thread determines whether a new keyframe needs to be inserted in the new keyframe decision step.

The back-end tasks consist of optimal static scene reconstruction, determination of the static points using sliding ptember 16,2026 at 03:33:56 UTC from IEEE Xplore. Restrictions apply.

windows, and pose graph optimization. These tasks are performed by three modules running in three independent threads: the local mapping, static point determination, and loop closing modules. The local mapping module processes each new keyframe and uses the local bundle adjustment to optimize all marked points. The static point determination module uses point-correlation optimization to determine reliable static points and remove the marked status from the other points to eliminate the influence of moving objects. The loop closing module searches for loop closures using the bag-of-words method. If a new loop closure is found, pose graph optimization is performed to eliminate the accumulated global drift and achieve global consistency.

In the next sections, changes made to three components in DSLAM for dynamic environments are introduced: the determination of static points, feature matching, and map management.

## 5.1 Determination of Static Points for RGB-D Sensors

This section introduces the measurement models related to RGB-D sensors. Then, the static point determination modules in the front and back ends are described in detail. Both implementations utilize graph optimization to optimize the objective function Eq. (12) using nonlinear least-squares techniques [54].

## 5.1.1 Formulation ofthe Point-Correlation Measurement Model forRGB-D Sensors

The measurement models related to RGB-D sensors should be described before describing the proposed segmentation method. Hence, the map-point and point-correlation measurement models are described in this section.

Map-Point Measurement. For RGB-D sensors, ${ \bf y } _ { i k }$ can be specified as

$$
\mathbf { y } _ { i k } = \left[ \begin{array} { c } { \left( u - c _ { u } \right) \times d / f _ { u } } \\ { \left( v - c _ { v } \right) \times d / f _ { v } } \\ { d } \end{array} \right] = \mathbf { g } ( \mathbf { x } _ { i k } ) + \mathbf { n } _ { i k } ,\tag{15}
$$

where $( c _ { u } , c _ { v } )$ is the camera’s principal point and $\left( f _ { u } , f _ { v } \right)$ is the focal length. It is assumed that the measurement of map point $\mathbf { p } _ { i }$ is $( u , v )$ in the RGB image and its corresponding depth measurement is d. Moreover, ${ \bf g } ( { \bf x } _ { i k } )$ is the RGB-D sensor model, expressed as

$$
\begin{array} { r } { { \bf g } ( { \bf x } _ { i k } ) = { \bf s } _ { R G B - D } ( { \bf T } _ { k } \cdot \overline { { \bf p } } _ { i } ) , } \end{array}\tag{16}
$$

where $\mathbf { \boldsymbol { \mathsf { s } } } _ { R G B - D } ( \mathbf { \boldsymbol { \mathsf { p } } } ) = \left[ \begin{array} { l l l l } { 1 } & { 0 } & { 0 } & { 0 } \\ { 0 } & { 1 } & { 0 } & { 0 } \\ { 0 } & { 0 } & { 1 } & { 0 } \end{array} \right] \mathbf { \boldsymbol { \mathsf { p } } } ,$ and r is a 4D vector. For convenience, $[ a , b , c ] ^ { T }$ is used to represent ${ \bf y } _ { i k ^ { \prime } }$ i.e.,

$$
\mathbf { y } _ { i k } = { \left[ \begin{array} { l } { a } \\ { b } \\ { c } \end{array} \right] } .\tag{17}
$$

In addition to $\mathbf { g } ( \cdot ) ,$ the uncertainty of the noise should also be determined. Given the particular characteristics of the structured light technology used by RGB-D sensors, the depth measurement error dramatically increases with the sensing depth. Therefore, the uncertainty in the depth measurement should be modeled. Khoshelham [58] modeled the depth measurement of Kinect-style devices and concluded that the uncertainty in each depth measurement is proportional to the square of its depth, i.e.,

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/70342e029306656b0ca315a8a54d5c97d0481446dfa5af536bd24475942a45dc.jpg)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/b96c3c069fe5c2587c457de1edc763a48accc04c0e0106b6e3b14c185c28f1ad.jpg)  
(a) RGB  
(b) Depth

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/964904ebe486d85945a585d215743cd11feacc4bf3cdec2d806a7d179ea0a0cf.jpg)  
(c) Uncertainty  
Fig. 7. Images captured by an ASUS Xtion Pro camera and the uncertainty results for depth: (a) color image, (b) depth image registered with the color image, and (c) the uncertainty of the depth image. Blue pixels indicate no data because the depth exceeds the range of the depth camera. Green pixels indicate that the uncertainty of the measured depth is too large to be trusted. Colors from black to red denote the uncertainties of the pixels with valid depths (black = low uncertainty, red = high uncertainty).

$$
\boldsymbol { \sigma } _ { d } = \frac { 1 } { f _ { v } } \boldsymbol { \sigma } d ^ { 2 } ,\tag{18}
$$

where s and $\sigma _ { d }$ are respectively the standard deviation of the measured normalized disparity and the standard deviation of the calculated depth. In this model, $\sigma _ { d }$ increases as d increases. However, this model does not consider the problem of more significant uncertainty at object edges. For establishing a more accurate uncertainty model, a Gaussian mixture model is used [59]. In this model, u and v are assumed to be independent random variables distributed according to the normal distributions $\mathcal { N } ( u , \sigma _ { u } )$ and $\mathcal { N } ( v , \sigma _ { v } ) .$ respectively. The random variable c is defined as a mixture of the d variables in a local window $i \in [ u - 1 , u + 1 ]$ $j \in$ $[ v - 1 , v + 1 ]$ . The mean and variance of the resulting Gaussian mixture are

$$
\begin{array} { r l } & { \hat { \boldsymbol { \mu } } _ { c } = \displaystyle \sum _ { \mathbf { q } \in \mathcal { N } _ { u v } } w _ { \mathbf { q } } ( d _ { \mathbf { q } } ) , } \\ & { \hat { \boldsymbol { \sigma } } _ { c } = \displaystyle \sum _ { \mathbf { q } \in \mathcal { N } _ { u v } } w _ { \mathbf { q } } ( \sigma _ { d _ { \mathbf { q } } } ^ { 2 } + d _ { \mathbf { q } } ^ { 2 } ) - \hat { \boldsymbol { \mu } } _ { c } ^ { 2 } , } \end{array}\tag{19}
$$

where $\mathcal { N } _ { u v }$ is the position set located in the window with $( u , v )$ as the center, $d _ { \mathbf { q } }$ is the depth measurement at position ${ \mathfrak { q } } ,$ and the weight $w _ { \mathbf { q } }$ is chosen according to kernel W, which is expressed as follows:

$$
W = \frac { 1 } { 1 6 } \left[ \begin{array} { l l l } { 1 } & { 2 } & { 1 } \\ { 2 } & { 4 } & { 2 } \\ { 1 } & { 2 } & { 1 } \end{array} \right] .\tag{20}
$$

An example is shown in Fig. 7c.

Therefore, based on Eq. (19), the 3D covariance $\pmb { \Sigma } _ { i k }$ of the measurement of the ith map point at time k is assigned using

$$
\mathbf { C } _ { i k } = \left[ \begin{array} { c c c } { \sigma _ { a } ^ { 2 } } & { \sigma _ { a b } } & { \sigma _ { a c } } \\ { \sigma _ { b a } } & { \sigma _ { b } ^ { 2 } } & { \sigma _ { b c } } \\ { \sigma _ { c a } } & { \sigma _ { c b } } & { \sigma _ { c } ^ { 2 } } \end{array} \right] ,\tag{21}
$$

where

$$
\begin{array} { r l } & { \sigma _ { \alpha } ^ { 2 } = \frac { \hat { \sigma } _ { c } ^ { 2 } ( u - c _ { u } ) ( v - c _ { v } ) + \sigma _ { u } ^ { 2 } ( \hat { \mu } _ { c } ^ { 2 } + \hat { \sigma } _ { c } ^ { 2 } ) } { f _ { x } ^ { 2 } } , } \\ & { \sigma _ { b } ^ { 2 } = \frac { \hat { \sigma } _ { c } ^ { 2 } ( u - c _ { u } ) ( v - c _ { v } ) + \sigma _ { v } ^ { 2 } ( \hat { \mu } _ { c } ^ { 2 } + \hat { \sigma } _ { c } ^ { 2 } ) } { f _ { y } ^ { 2 } } , } \\ & { \sigma _ { \alpha } = \sigma _ { \alpha } = \hat { \sigma } _ { c } ^ { 2 } \frac { u - c _ { u } } { f _ { u } } , } \\ & { \sigma _ { b c } = \sigma _ { c b } \hat { \sigma } _ { c } ^ { 2 } \frac { v - c _ { v } } { f _ { c } } , } \\ & { \sigma _ { u b } = \sigma _ { b a } + \hat { \sigma } _ { c } ^ { 2 } \frac { ( u - c _ { u } ) ( v - c _ { v } ) } { f _ { u } f _ { v } } , } \\ & { \sigma _ { \alpha } ^ { 2 } = \hat { \sigma } _ { c } ^ { 2 } . } \end{array}\tag{22}
$$

Point-Correlation Measurement Model. On the basis of the point measurement model in Eq. (8), the point-correlation measurement model can be expressed as

$$
\mathbf { z } _ { i j k } = \mathbf { y } _ { i k } - \mathbf { y } _ { j k } = \mathbf { s } _ { R G B - D } ( \mathbf { T } _ { k } \cdot ( \overline { { \mathbf { p } } } _ { i } - \overline { { \mathbf { p } } } _ { j } ) ) + \mathbf { n } _ { i j k } ,\tag{23}
$$

where $\mathbf { z } _ { i j k }$ is the relative position between $\boldsymbol { \mathsf { p } } _ { i }$ and ${ \bf p } _ { j }$ for RGB-D sensors. If we assume that ${ \bf y } _ { i k }$ and ${ \bf y } _ { j k }$ are independent, the covariance of $\mathbf { z } _ { i j k }$ between the two points can be computed as

$$
\mathbf { C } _ { i j k } = \mathbf { C } _ { i k } + \mathbf { C } _ { j k } .\tag{24}
$$

## 5.1.2 Determination ofStatic Points bythe Front End

The static points are determined by the front end to eliminate the influence of fast-moving objects on its motion estimation. Because incorrect correspondences affect the optimization of the point correlations, and the pose estimation step can remove false correspondences, the static point determination module is added after initial pose estimation. As introduced in Section 4.1, the 3D graph $| \mathcal { G } = \{ \mathbf { l } _ { i j } \}$ is constructed using Delaunay triangulation based on the tracked map points of the previous frame. Only the measurements of the two frames are used to optimize the graph. Then, the squared Mahalanobis length of the error is computed for each measurement of each edge $\mathbf { l } _ { i j } \in \mathcal { G } .$ . If the error in a measurement is larger than the given threshold, this measurement is removed. If all measurements of an edge are removed, the edge is determined to be an inconsistent edge and is removed from G. After removing all inconsistent edges, the remaining graph $\mathcal { G } _ { n e w }$ is separated into multiple connected components. Afterward, all connected components are found by checking the connectedness of $\mathcal { G } _ { n e w } ,$ and the result is denoted by fC g. Finally, the points of the largest connected component of fC g are determined to be reliable static points, and the marks of every map point in $\{ \mathcal { G } _ { n e w } \setminus \overline { { c _ { m a x } } } \}$ are removed.

## 5.1.3 Static Point Determination in the BackEnd

This module verifies the marked points after new keyframes have been added. The calculation is shown in Algorithm 1. In contrast to the determination module in the front end, this module attempts to use all of the information in the sliding window to determine the static points.

Algorithm 1. Back-End Segmentation   
Input:- Static local map-point set $\mathcal { P }$   
- Maximum number of iterations n allowed in the algorithm   
- Threshold value t   
Output:- Graph $\mathcal { G } _ { o u t p u t }$   
1: Triangulate map points $\mathcal { P }$ to obtain the 3D edge set   
$\mathcal { G } = \{ \bar { \bf l } _ { i j } \}$   
2: for $\mathbf { l } _ { i j } \in \mathcal { G }$ do   
3: $\mathbf { y } _ { i }$ the observation set $\{ { \bf y } _ { i k } \}$ of $\mathbf { p } _ { i }$   
4: $\mathbf { y } _ { j }$ the observation set $\{ \mathbf { y } _ { j k } \}$ of ${ \bf p } _ { j }$   
5: if $\mathbf { y } _ { i }$ and $\mathbf { y } _ { j }$ have the same observation in frame k then   
6: Compute the observation $\mathbf { z } _ { i j k }$ of ${ \mathbf { l } _ { i j } }$ and $\mathbf { C } _ { i j k }$   
7: Add $\mathbf { z } _ { i j k }$ to z   
8: end if   
9: end for   
10: ${ \bf z } _ { i n l i e r } \gets { \bf z }$   
11: iterations $ 0$   
12: for iterations $< n$ do   
13: Optimize the objective function in Eq. (12) with $\mathbf { z } _ { i n l i e r }$   
14: for ${ \bf z } _ { i j k } \in { \bf z }$ do   
15: if ${ \bf e } _ { z , i j k } ( { \bf { l } } _ { i j k } ) ^ { T } { \bf C } _ { i j k } ^ { - 1 } { \bf e } _ { z , i j k } ( { \bf { l } } _ { i j k } ) > t$ then   
16: Remove $\mathbf { z } _ { i j k }$ from $\mathbf { z } _ { i n l i e r }$   
17: else   
18: Add $\mathbf { z } _ { i j k }$ to $\mathbf { z } _ { i n l i e r }$   
19: end if   
20: end for   
21: iterations iterations þ 1   
22: end for   
23: Compute a new graph $\mathcal { G } _ { n e w }$ based on the remaining $\mathbf { z } _ { i n l i e r }$   
24: Determine whether the graph $\mathcal { G } _ { n e w }$ is separated into several   
connected components {C } using DFS   
25: Compute the volume of each connected component $\mathcal { C } _ { i }$ and   
determine the largest one $\mathcal { C } _ { m a x }$   
26: Erase the static marks of each map point in $\{ \mathcal { G } _ { n e w } \setminus \mathcal { C } _ { m a x } \}$   
27: $\mathcal { G } _ { o u t p u t }  \mathcal { C } _ { m a x }$

This module searches for the keyframes that have covisibility with the currently added keyframe. Then, a 3D graph structure is created using the marked points of these obtained keyframes. The available measurements of all edges are used to build the Hessian matrix of the objective function in Eq. (12). Afterwards, the outliers are removed in the iterations. After optimization and the removal of inconsistent edges from the graph, the remaining graph is divided into multiple connected components if there are moving objects in the FOV. Finally, the points of the component with the largest volume are retained as marked points, but the points in the smaller connected components are changed to unmarked points.

## 5.2 Map-Point Matching

Map-point matching is used to obtain the matching between map points and features. As shown in Fig. 8, the matching of marked points can determine the features that are reliable in the static scene in the FOV of the frame. In contrast, the matching of unmarked points can include features that are not reliable and should not be used for motion estimation. In addition, the determination of unreliable features reduces the risk of failure when identifying static points and the probability of incorrect matching.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/49d238c6b63e24d6625a919ab41d185b222218202b70724f02572bbf7a98e989.jpg)  
Fig. 8. Example of map-point matching. If the unmarked points are removed immediately after the static point determination module, some unreliable features on the image will incorrectly match the marked points.

## 5.3 Map Management

Map management is divided into two parts: point management and keyframe management.

## 5.3.1 Point Management

Point management consists of three tasks: creation, updating, and culling.

Creation. There are two methods for creating map points. In the first method, map points are created by triangulation from two keyframes that have covisibility. In the second method, map points are immediately created from a new feature that has a depth measurement. For the first method, it is difficult to create a point on a moving object because of the huge triangulation error. For the second method, a point on a moving object will be created because the corresponding new feature cannot be recognized as a feature on the moving object from a single image. As described in Section 5.2, dynamic points are prevented from being created by the second method because the features are matched with the unmarked points.

Update. After creation, updating the positions of the marked points improves the accuracy of motion estimation by the front end and can avoid mismatches. Therefore, the marked and unmarked points are separately updated using new information. The marked points are updated by bundle adjustment. The unmarked points are updated by static point determination using the newly available measurements from the new frame.

Culling. In addition to creation, the map points must meet a criterion to ensure that no point is incorrectly triangulated. Each type of map point has its own criterion. Marked points must have a certain number of successful observations, which ensures that they are trackable. Otherwise, they should be removed. The unmarked points are removed when they are not observed in the FOV of the sliding window.

## 5.3.2 Keyframe Management

Keyframe culling is used to maintain a compact reconstruction. The policy is that the keyframes in which most points have been observed in other keyframes are discarded. Keyframe culling confers two benefits. The preservation of valid data: Frames without new static information are often mistakenly identified as keyframes owing to the presence of moving objects. Therefore, these incorrect keyframes are more likely to be discarded after the static points have been correctly determined. A longer sliding-window time span: Discarding redundant keyframes lengthens the time spans of the sliding windows without increasing the number of keyframes. A longer time span increases the ability of the static point determination module to eliminate the influence of slow-moving objects.

## 6 EXPERIMENTS

In this section, we present an experimental evaluation with indoor sequences from the Technical University of Munich (TUM) RGB-D benchmark [60] to evaluate the accuracy of DSLAM as well as three sequences from especially challenging environments to evaluate the robustness of DSLAM. We also extracted the statistics of the time spent in each step of the static point determination module to evaluate the efficiency of the proposed segmentation method.

The TUM benchmark includes ground-truth trajectories obtained from a high-accuracy motion capture system and contains both the static and dynamic scenarios in indoor environments. We divided the environments of the TUM benchmark into three categories: static, slightly dynamic, and highly dynamic. When there are no moving objects in the scene, we call it a static environment. If only a small part of the FOV is covered by moving objects, e.g., someone in the office makes a gesture, it is defined as a slightly dynamic environment. If the majority of the FOV is occupied by moving objects, we call it a highly dynamic environment. The performance of most state-of-the-art RGB-D methods are evaluated on the TUM RGB-D dataset, and these methods have achieved good results. However, the sequences containing moving objects are not often used for evaluation. Therefore, the slightly and highly dynamic types of sequences were used to evaluate the performance of the proposed method in dynamic environments.

All experiments were performed on a desktop computer equipped with an Intel Core i5-3470 (3.2 GHz) CPU and 8 GB of RAM. For comparison, we also obtained results for the following state-of-the-art methods: dense VO (DVO) [5], ORB-SLAM2 [29], model-based dense VO (BAMVO) [31],and DVO SLAM [61]. In addition, the reported results of FlowFusion [27], Motion Removal DVO SLAM [29], StaticFusion [33], RGBD SLAM with static-point weighting (SPWSLAM) [30], Co-Fusion [51], MaskFusion [10], MID-Fusion [52], EM-Fusion [50], and DynaSLAM [9] are included in the comparison. The results for the improvements to the original SLAM system without the proposed segmentation are also reported to determine whether the improvements in the results are due to the segmentation method or the SLAM core systems. DVO, DVO SLAM, and ORB-SLAM2 represent the most advanced methods based on the static world assumption. Motion Removal DVO SLAM, BAMVO, FlowFusion, StaticFusion, and SPWSLAM are recent methods that consider the influence of moving objects. Finally, Co-Fusion, MaskFusion, MID-Fusion, EM-Fusion, and DynaSLAM are methods that leverage learning techniques.

## 6.1 Comparison of the Accuracy

In the TUM benchmark, the slightly dynamic environment sequences are the sitting and desk-person sequences, and the highly dynamic environment sequences are the walking eptember16,2026 at 03:33:56 UTC from IEEE Xpfore. Restrictions apply.

TABLE 1  
Comparison of the Rotational Root Mean-Squared Error (RMSE) of the Relative Pose Error (RPE) on the TUM Benchmark
<table><tr><td rowspan="2" colspan="2">Sequences</td><td colspan="7">Rot. RMSE of trajectory alignment [° /s]</td></tr><tr><td>BAMVO</td><td>StaticFusion</td><td>SPWSLAM</td><td>DVO</td><td>ORB-SLAM2 w/o SPD</td><td>Our w/ SPD</td><td>Improvement w/ SPD</td></tr><tr><td rowspan="5">slightly dynamic</td><td>fr2/desk-person</td><td>1.2159</td><td></td><td>0.8213</td><td>1.5368</td><td>1.3717</td><td>1.3951</td><td>-1.70%</td></tr><tr><td>fr3/sitting-static</td><td>0.6997</td><td>0.43</td><td>0.7228</td><td>0.6084</td><td>0.3630</td><td>0.3786</td><td>-4.30%</td></tr><tr><td>fr3/sitting-xyz</td><td>1.3885</td><td>0.92</td><td>0.8466</td><td>1.4980</td><td>0.5817</td><td>0.5792</td><td>0.43%</td></tr><tr><td>fr3/sitting-rpy</td><td>5.9834</td><td></td><td>5.6258</td><td>6.0164</td><td>0.9361</td><td>0.9047</td><td>3.35%</td></tr><tr><td>fr3/sitting-halfsphere</td><td>2.8804</td><td>2.11</td><td>1.8836</td><td>4.6490</td><td>0.9101</td><td>0.8699</td><td>4.41%</td></tr><tr><td rowspan="4">highly dynamic</td><td>fr3/walking-static</td><td>2.0833</td><td>0.38</td><td>0.8085</td><td>6.3502</td><td>10.5764</td><td>0.3293</td><td>96.89%</td></tr><tr><td>fr3/walking-xyz</td><td>4.3911</td><td>2.66</td><td>1.6442</td><td>7.6669</td><td>19.7299</td><td>2.7413</td><td>86.11%</td></tr><tr><td>fr3/walking-rpy</td><td>6.3398</td><td></td><td>5.6902</td><td>7.0662</td><td>22.2934</td><td>4.6327</td><td>79.22%</td></tr><tr><td>fr3/walking-halfsphere</td><td>4.2863</td><td>5.04</td><td>2.4048</td><td>5.2179</td><td>24.6634</td><td>0.9854</td><td>96.00%</td></tr></table>

The best results are shown in bold. Not all papers provide results for all sequences. We report the improvement with respect to the original SLAM system (ORB-SLAM2) without static point determination (SPD).

TABLE 2  
Comparison of the Translational RMSE of the RPE on the TUM Benchmark
<table><tr><td rowspan="2" colspan="2">Sequences</td><td colspan="8">Trans. RMSE of trajectory alignment [m/s]</td></tr><tr><td>BAMVO</td><td>StaticFusion</td><td>FlowFusion</td><td>SPWSLAM</td><td>DVO</td><td>ORB-SLAM2 w/o SPD</td><td>Our w/ SPD</td><td>Improvement w/ SPD</td></tr><tr><td rowspan="5">slightly dynamic</td><td>fr2/desk-person</td><td>0.0352</td><td></td><td></td><td>0.0173</td><td>0.0354</td><td>0.0377</td><td>0.0362</td><td>3.98 %</td></tr><tr><td>fr3/sitting-static</td><td>0.0248</td><td>0.011</td><td></td><td>0.0231</td><td>0.0157</td><td>0.0122</td><td>0.0138</td><td>-13.11%</td></tr><tr><td>fr3/sitting-xyz</td><td>0.0482</td><td>0.028</td><td></td><td>0.0219</td><td>0.0453</td><td>0.0137</td><td>0.0134</td><td>2.19%</td></tr><tr><td>fr3/sitting-rpy</td><td>0.1872</td><td></td><td></td><td>0.0843</td><td>0.1735</td><td>0.0380</td><td>0.0320</td><td>15.79%</td></tr><tr><td>fr3/sitting-halfsphere</td><td>0.0589</td><td>0.030</td><td></td><td>0.0389</td><td>0.1005</td><td>0.0365</td><td>0.0354</td><td>3.01%</td></tr><tr><td rowspan="4">highly dynamic</td><td>fr3/walking-static</td><td>0.1339</td><td>0.013</td><td>0.030</td><td>0.0327</td><td>0.3818</td><td>0.5826</td><td>0.0141</td><td>97.58%</td></tr><tr><td>fr3/walking-xyz</td><td>0.2326</td><td>0.121</td><td>0.21</td><td>0.0651</td><td>0.4360</td><td>1.0484</td><td>0.1266</td><td>87.92%</td></tr><tr><td>fr3/walking-rpy</td><td>0.3584</td><td></td><td></td><td>0.2252</td><td>0.4038</td><td>1.1843</td><td>0.2299</td><td>80.59%</td></tr><tr><td>fr3/walking-halfsphere</td><td>0.1738</td><td>0.207</td><td></td><td>0.0527</td><td>0.2628</td><td>1.0790</td><td>0.0517</td><td>95.21%</td></tr></table>

The best results are shown in bold. Not all papers provide results for all sequences. We report the improvement with respect to the original SLAM system (ORB-SLAM2) without static point determination (SPD).

sequences. The walking sequences are challenging because the moving objects cover a large part of the FOV. In the slightly dynamic environment, there is a person sitting in front of a desk and moving their arms, sometimes in an organized office. In the highly dynamic environment, two people are walking around a desk. In the sequences for both types of environments, there are four types of camera motion, which are indicated in the sequence names. Here, halfsphere indicates that the camera follows the trajectory of a 1-m diameter half sphere, xyz indicates that the camera almost moves along the x, y, and z axes, rpy indicates that the camera rotates in the roll, pitch, and yaw directions, and static indicates that the camera only moves around a position in the environment.

The translational RMSE of the RPE in meters per second and the rotational RMSE of the RPE in degrees per second were calculated for the evaluation. The RMSE of the RPE is much more easily influenced by large occasional errors in the estimate; thus, it is more suitable for an evaluation in dynamic environments. The results for the RPE are listed in Tables 1 and 2. We also evaluated the full trajectory performance of the proposed method using the translational RMSE of the ATE, as listed in Table 3. Moreover, a comparison with methods that use learning techniques is shown in Table 4. The estimated trajectories were compared with the ground truth, and some results obtained by the proposed method are shown in Figs. 10 and 11. It can be seen that the proposed method is able to process all sequences well, including those in both slightly and highly dynamic environments.

Slightly Dynamic Environments. Compared to the methods with the static world assumption, DSLAM is slightly more accurate. In fr3/desk-person, the methods using the static world assumption provide slightly better results because most of the influence of moving objects can be eliminated. However, as the proportion of moving objects occupying the FOV increases, the robust estimation methods are not able to discard all dynamic points. Therefore, DSLAM provides better results in the sitting sequences.

In addition, DSLAM is able to outperform the methods that consider moving objects. The performances of methods that consider moving objects are even worse than those obtained using ORB-SLAM2. The reason is that a part of the information of the static scene is mistakenly removed by aggressive thresholds or image segmentation methods in these methods. Therefore, there is less available static information for motion estimation, decreasing accuracy. The poor performance in slightly dynamic environments limits their application in practice. DSLAM, which makes use of static points, provides the best results in most sequences.

Moreover, when compared with the learning-based methods, the proposed method performs better, as shown in Table 4. In the slightly dynamic environment with people sitting in front of the desk, most of the body is static, and only their hands move. Because the entire image region of static movable objects is detected and ignored, most static eptember 16,2026 at 03:33:56 UTC from IEEE Xplore. Restrictions apply.

TABLE 3  
Comparison of the Absolute Trajectory Error (ATE) on the TUM Benchmark
<table><tr><td rowspan="2" colspan="2">Sequences</td><td colspan="8">Trans. RMSE of trajectory alignment [m]</td></tr><tr><td>DVO SLAM</td><td>DVO SLAM Motion Removal</td><td>StaticFusion</td><td>FlowFusion</td><td>SPWSLAM</td><td>ORB-SLAM2 w/o SPD</td><td>Our w/SPD</td><td>Improvement w/ SPD</td></tr><tr><td rowspan="5">slightly dynamic</td><td>fr2/desk-person</td><td>0.1037</td><td>0.0596</td><td></td><td></td><td>0.0484</td><td>0.0064</td><td>0.0075</td><td>-17.18%</td></tr><tr><td>fr3/sitting-static</td><td>0.0119</td><td></td><td>0.013</td><td></td><td></td><td>0.0077</td><td>0.0096</td><td>-24.68%</td></tr><tr><td>fr3/sitting-xyz</td><td>0.2420</td><td>0.0482</td><td>0.040</td><td></td><td>0.0397</td><td>0.0094</td><td>0.0091</td><td>3.19%</td></tr><tr><td>fr3/sitting-rpy</td><td>0.1756</td><td></td><td></td><td></td><td></td><td>0.0250</td><td>0.0225</td><td>10.0%</td></tr><tr><td>fr3/sitting-halfsphere</td><td>0.2198</td><td>0.1252</td><td>0.040</td><td></td><td>0.0432</td><td>0.0250</td><td>0.0235</td><td>6.00%</td></tr><tr><td rowspan="5">highly</td><td>fr3/walking-static</td><td>0.7515</td><td>0.0656</td><td>0.014</td><td>0.028</td><td>0.0261</td><td>0.4080</td><td>0.0108</td><td>97.35%</td></tr><tr><td>fr3/walking-xyz</td><td>1.3830</td><td>0.0932</td><td>0.127</td><td>0.12</td><td>0.0601</td><td>0.7215</td><td>0.0874</td><td>87.88%</td></tr><tr><td>fr3/walking-rpy</td><td>1.2922</td><td>0.1333</td><td></td><td></td><td>0.1791</td><td>0.8054</td><td>0.1608</td><td>80.03%</td></tr><tr><td>fr3/walking-</td><td>1.0136</td><td>0.470</td><td>0.391</td><td></td><td>0.0489</td><td>0.7225</td><td>0.0354</td><td>95.10%</td></tr><tr><td>halfsphere</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr></table>

The best results are shown in bold. Not all papers provide results for all sequences. We report the improvement with respect to the original SLAM system (ORB-SLAM2) without static point determination (ŚPD).

TABLE 4  
Comparison of the ATE on the TUM Benchmark
<table><tr><td rowspan="2">Sequences</td><td rowspan="2"></td><td colspan="6">Trans. RMSE of trajectory alignment [m]</td></tr><tr><td>Co-Fusion*</td><td>MaskFusion</td><td>MID-Fusion</td><td>EM-Fusion</td><td>DynaSLAM*</td><td>Our</td></tr><tr><td rowspan="5">slightly dynamic</td><td>fr2/desk-person</td><td></td><td></td><td></td><td></td><td></td><td>0.0075</td></tr><tr><td>fr3/sitting-static</td><td>0.011</td><td>0.021</td><td>0.010</td><td>0.09</td><td></td><td>0.0096</td></tr><tr><td>fr3/sitting-xyz</td><td>0.027</td><td>0.031</td><td>0.062</td><td>0.37</td><td>0.015</td><td>0.0091</td></tr><tr><td>fr3/sitting-rpy</td><td></td><td></td><td></td><td></td><td></td><td>0.0225</td></tr><tr><td>fr3/sitting-halfsphere</td><td>0.036</td><td>0.052</td><td>0.031</td><td>0.032</td><td>0.017</td><td>0.0235</td></tr><tr><td rowspan="4">highly dynamic</td><td>fr3/walking-static</td><td>0.551</td><td>0.035</td><td>0.023</td><td>0.014</td><td>0.006</td><td>0.0108</td></tr><tr><td>fr3/walking-xyz</td><td>0.696</td><td>0.104</td><td>0.068</td><td>0.066</td><td>0.015</td><td>0.0874</td></tr><tr><td>fr3/walking-rpy</td><td></td><td></td><td></td><td></td><td>0.035</td><td>0.1608</td></tr><tr><td>fr3/walking-halfsphere</td><td>0.803</td><td>0.106</td><td>0.038</td><td>0.051</td><td>0.025</td><td>0.0354</td></tr></table>

The best results are shown in bold. Not all papers provide results for all sequences. <sup></sup> indicates the methods that segment the scene into different objects using both motion and semantic cues (deep learning).

information in the person is discarded. As a result, less static information is used by learning-based methods to estimate the camera pose.

Highly Dynamic Environments. Because the static world assumption is not true, DSLAM outperforms the methods that assume a static world. In these sequences, most of the features on the moving objects are incorrectly tracked as inliers by these methods, which are not designed for dynamic environments. Therefore, both DVO and ORB-SLAM2 yield unacceptable errors. The results for ORB-SLAM2 show that loop-closure detection does not correctly reduce drift because the moving objects dramatically influence the loop closing module.

When compared with the methods that consider moving objects, DSLAM also provides better results for most sequences. DSLAM performs worse than SPWSLAM in only one case, namely the walking-xyz sequence. In the beginning part of the walking-xyz sequence, the people walk away from the camera. Therefore, the position of the person in the RGB image only slightly changes, while the depth value of the person in the depth images substantially changes. Meanwhile, because depth measurements are not very accurate, the segmentation relies more on the information of the RGB image in our implementation. Therefore, it is difficult to separate dynamic points when segmentation is performed with an insufficient amount of information from RGB images.

This problem can also be seen in the results shown in Fig. 9, where a significant error is introduced in the beginning part of the sequence. This problem can be solved by setting the threshold value aggressively. However, more aggressive parameters will lead to more false negatives, which reduces the accuracy of the results obtained on other sequences.

In this highly dynamic environment, the features on the bodies of people are all moving dynamically and should be excluded. Because the entire image region of the person is masked using learning techniques, the learning-based methods only utilizing static information should perform better. However, the proposed method performs better than most learning-based methods. Only DynaSLAM can perform better than DSLAM. Therefore, the motion estimation core system and geometry cues are also crucial for increasing accuracy.

In the comparison of several methods that use learning techniques (Co-Fusion, MaskFusion, MID-Fusion, EM-Fusion, and DynaSLAM), the results differ even when using the same neural network, as shown in Table 4. DynaSLAM, the learning-based method that combines geometry cues, performs the best at present. The reason is that though only people appear in the benchmark environment as moving objects, the learning-based techniques still cannot obtain 100 percent accuracy because of the nature of the algorithms. In addition, there are objects that cannot be detected by learning-based techniques eptember 16,2026 at 03:33:56 UTC from IEEE Xplore. Restrictions apply.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/4321ae4757fc22ead710fcc2bc4e216bdc6ad938880dd4fd40e211d7153c897b.jpg)  
(a) fr3/walking-rpy (without)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/c4c1140f6d50a1873260f8f6da4d43078e9d71a76cca45cb24e73c30ad29e294.jpg)  
(b) fr3/walking-static (without)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/df876cee488084c26e2504821098ab9d609e440f587ff8ba036f98ef04102456.jpg)  
(c) fr3/walking-xyz (without)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/a6ae26ae742563d257ada1770c8141a6738af8bb09f2cad0f26aeb2982cdacc9.jpg)  
(d) fr3/walking-rpy (with)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/09c7134c5f6690b57d1986d9dc97eb0fb04dbdfc1a07ff1294461789dd6fd1ee.jpg)  
(e) fr3/walking-static (with)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/8f11f68dacde0026bd715e4bf7cf4185a79ff0f327a547fadc5d1bd1e88caf3f.jpg)  
(f) fr3/walking-xyz (with)

Fig. 9. Comparison of example estimated trajectories. (a), (b), and (c) Trajectories estimated using ORB-SLAM2, which is not designed for dynamic environments. (d), (e), and (f) Trajectories estimated using the proposed method.  
![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/0aa8315be35403343dd1473fd3a7b4eac31383d3a1505b3b5163e2d6029d9ef7.jpg)  
(a)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/422f0e497f70a0b2a2aab5818deaa44b1bebd2932b3b16bbee47c245a1c8b9b3.jpg)  
(b)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/3e636ec53f391937848c22188e890d5f3ba46de1464992e694bbc480948a9560.jpg)  
(c)

Fig. 10. Example taken from the fr3/sitting-halfsphere sequence. (a) Result showing the determination of static points by the front end. (b) Result of 3D edge culling during the determination of feature static points by the back end. (c) Estimated trajectory compared with the ground truth.  
![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/f8f1d6b8fb687975bccee4dc4c477c261274b21b4a6df718f3fd1e7809db99f5.jpg)  
(a)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/c6c83b9c19296aa50aad40d3fdb0ff6a424a13458d2da5c9e8445d2a8c0e9970.jpg)  
(b)

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/074b46f9f6b98350ec46b75cb641d120f39343b60bd59e10a2b1f3b0e5a992cc.jpg)  
(c)  
Fig. 11. Example taken from the fr3/waling-halfsphere sequence. (a) Result showing the determination of static points by the front end. (b) Result of 3D edge culling during the determination of feature static points by the back end. (c) Estimated trajectory compared with the ground truth.

because they are not a priori dynamic, but they are movable. Therefore, the methods that only depend on learning techniques do not offer excellent performance, even in environments that only contain pretrained objects. Furthermore, in the presence of unknown objects that were not present in the training set, these learning-based methods may fail. This poor generalization limits their use in unknown environments. Moreover, most learning methods require pre-training in ptember 16,2026 at 03:33:56 UTC from IEEE Xplore. Restrictions apply.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/8f6ce484f9fb9272e6ac3cf58b12e6e4413e9835c412953f358b6fe7db0def74.jpg)  
Fig. 12. Tracking and static point determination results for each of the three challenging environments: (a) Environment with an object moving at a slow speed, (b) environment with an object moving with unstructured motion, and (c) environment with a person walking and covering most of the FOV.

advance for better performance, which makes them even more inconvenient to use in practice.

## 6.2 Comparison of the Robustness

The robustness of DSLAM was further evaluated in three challenging environments with three types of moving objects (as shown in Fig. 12): a slowly moving object, an object moving with unstructured motion, and a person walking and covering most of the background. In each environment, the ASUS Xtion Pro Live, which was used to record the data, was stationary. Therefore, the trajectory estimated by the methods should be a point. In other words, the distance between the origin and the trajectory is the error. Because DVO SLAM and SPWSLAM do not have open-source code and DVO cannot run in real time on our platform, only ORB-SLAM2 and BAMVO were used for the comparison.

In the environment with a slowly moving object, the reprojected position of the object in the images also moves slowly. Therefore, the projection of the moving object slightly changes in the images of the current and previous frames. The small difference between consecutive frames is a challenge. Therefore, both ORB-SLAM2 and BAMVO are influenced by the moving object. For DSLAM, the front end also cannot determine which points are on the moving object all of the time. However, the back end can determine static points with data over a longer time span. Because DSLAM is almost uninfluenced by the moving object, the trajectory result remains close to the origin, as shown in Fig. 13.

In the environment with an object moving with unstructured motion, the object is freely distorted. Distortion can be seen as another challenge because the dynamic points on this moving object have inconsistent and independent motion. Note that the object first enters the FOV. Therefore, ORB-SLAM2 and BAMVO are affected in the first part of the sequence because they cannot remove information disturbed by the appearance of the moving object. DSLAM can resist the disturbance of the moving object, as shown in Fig. 14. After the object becomes distorted, the performance of DSLAM is still the best. Although the moving object remains in almost the same image region, ORB-SLAM2 and BAMVO still cannot perform well. In particular, the image region for the moving object cannot be completely excluded by BAMVO. Therefore, the defective exclusion result influences the estimation output by BAMVO.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/610a86e8f18a47e8d35f572d166e4c9c95fb843995c5ed1f097e26447f7d504b.jpg)  
Fig. 13. XYZ trajectory results for the environment with an object moving at a slow speed.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/1822a14bbf52ba6e4c78aeaec5ded354000b54a3af5c4c99710f1fa0abcd4714.jpg)  
Fig. 14. XYZ trajectory results for the environment with an object moving with unstructured motion.

The last experiment was carried out in an environment with a person walking and covering most of the FOV. In this sequence, most of the image is covered by the person and a paper because the person holding the paper is close to the camera. Only a small amount of the information about the static scene can be sensed in most of the sequences. As shown in Fig. 15, ORB-SLAM2 and BAMVO cannot compensate for the influence of the walking person. Therefore, the trajectories of ORB-SLAM2 and BAMVO are far from the origin. In the results obtained by DSLAM, the dynamic points on both the walking person and handheld paper are discarded from motion estimation. Therefore, the large moving object does not significantly influence DSLAM, and the trajectory obtained by DSLAM remains close to the origin.

## 6.3 Analysis of the Efficiency

In this section, the statistics of the runtimes are presented to evaluate the efficiency of the determination of static points by the front and back ends. We evaluated the real-time performance of DSLAM on the walking-static sequence of the TUM RGB-D benchmark. In the walking-static sequence, the viewpoint of the camera does not change in the sequences since the camera remains almost static at one position. Therefore, only walking people influence the runtime of each method, as summarized in Table 5. Compared to ORB-SLAM2, DSLAM performs more efficiently because the dynamic points have been discarded, thereby reducing the computational cost of motion estimation. Meanwhile, the standard deviation of ORB-SLAM2 is larger because its estimation is influenced by the moving objects, as shown in Table 6. For DVO, the time consumption is unacceptable because the estimation is difficult to complete when people pass by the FOV of the camera. For BAMVO and StaticFusion, their dense operation leads to a poor result even though they already use a downsampled image (320240 resolution). Besides, among the feature-based methods (SPWSLAM, ORB-SLAM2, and DSLAM), the methods based on sparse features (SPWSLAM and DSLAM) have better real-time performance because fewer data need to be processed. With regard to the runtime of the learning-based techniques, most such methods need additional hardware (e.g., GPUs) and are not designed for real-time applications.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/29bef72afd639850ff16c11e6ea33660bea7184306d5889acc6ce005b23c7294.jpg)  
Fig. 15. XYZ trajectory results for the environment with a person walking and covering most of the FOV.

TABLE 5  
Comparison of the Real-Time Performance of the Motion Estimation Output in Dynamic Environments
<table><tr><td></td><td>Mean [ms]</td></tr><tr><td>DVO</td><td>717.48</td></tr><tr><td>ORB-SLAM2</td><td>31.34</td></tr><tr><td>Co-Fusion*</td><td>83.3 (with GPU)</td></tr><tr><td>MaskFusion*</td><td>200 (with 2 GPUs)</td></tr><tr><td>MID-Fusion*</td><td>400 (with pre-computed data on GPU)</td></tr><tr><td>DynaSLAM*</td><td>738.46 (with GPU)</td></tr><tr><td>StaticFusion*</td><td>30 (with GPU)</td></tr><tr><td>SPWSLAM*</td><td>22</td></tr><tr><td>BAMVO</td><td>67.78</td></tr><tr><td>Our</td><td>30.65</td></tr></table>

Only the results for BAMVO and StaticFusion are reported for an image resolution of 320  240 pixels. Here,  denotes the running time evaluated in the original papers with more powerful hardware.

The real-time performance for each major step of the static point determination module was measured, and the results are listed in Table 7. In the walking-halfsphere sequence used for the evaluation, there are moving objects, and the viewpoint of the camera changes over time. Therefore, this sequence is more challenging and suitable for evaluating the efficiency of each step in dynamic environments. The results in Table 7 show that the determination of static points by the front end only needs 2.0216 ms on average. This implies that our method has the potential for online applications, and further indicates that DSLAM has the potential for real-time applications.

We evaluated the efficiency of the determination of static points in the back end by measuring the time cost with respect to the number of map points. As shown in Fig. 16, the relationship between the number of map points and the processing time is nearly linear initially. As the number of map points continues to increase, the runtime does not increase because the number of local map points used for the computation remains stable. Moreover, the number of static points changes when the number of local points is almost constant because there are dynamic points that can be ignored in the calculation. Therefore, the time cost is not smooth.

TABLE 6  
Real-Time Performance Comparison With Standard Deviation
<table><tr><td></td><td>DVO</td><td>ORBSLAM2</td><td>BAMVO</td><td>Our</td></tr><tr><td>Medium [ms]</td><td>109.419</td><td>31.37</td><td>66.1960</td><td>30.4144</td></tr><tr><td>Mean [ms]</td><td>717.4791</td><td>31.34</td><td>67.7775</td><td>30.6515</td></tr><tr><td>Std. [ms]</td><td>1467.3502</td><td>4.8466</td><td>8.3373</td><td>3.3191</td></tr></table>

The results of BAMVO were evaluated at an image resolution of $3 2 0 \times 2 4 0$ pixels.

TABLE 7  
Real-Time Performance of the Major Steps of the Determination of Static Points
<table><tr><td></td><td></td><td>Module</td><td>Medium [ms]</td><td>Mean [ms]</td><td>Std [ms]</td></tr><tr><td rowspan="5">Static Point Determination</td><td rowspan="5">Front End</td><td>Build the graph Remove the edges</td><td>1.079 0.4665</td><td>1.1172 0.4864</td><td>0.3643 0.2554</td></tr><tr><td></td><td></td><td></td><td></td></tr><tr><td>Separate dynamic group</td><td>0.307</td><td>0.4180</td><td>0.3895</td></tr><tr><td>Total</td><td>1.930</td><td>2.0216</td><td>0.7609</td></tr><tr><td>Build the graph</td><td>14.6465</td><td>14.6582</td><td>4.6123</td></tr><tr><td rowspan="5"></td><td rowspan="5">Back End</td><td>Remove the edges</td><td>95.263</td><td>91.4375</td><td>30.5377</td></tr><tr><td>Separate dynamic group</td><td>1.328</td><td>1.3051</td><td></td></tr><tr><td></td><td></td><td></td><td>0.3327</td></tr><tr><td>Total</td><td>111.1730</td><td>107.401</td><td>35.0098</td></tr><tr><td></td><td></td><td></td><td></td></tr></table>

We note that the static point determination module needs almost 180 ms to optimize 2,500 local map points thanks to the sparse graph construction. The bundle adjustment module requires nearly 200 ms to optimize poses and points and runs on another thread, so the real-time performance of both modules is close. However, because bundle adjustment optimization is needed to determine dynamic points in the local map, it is necessary to complete segmentation as soon as possible before bundle adjustment optimization. Therefore, the efficiency of the static point determination module running on another thread still needs to be improved.

## 6.4 Summary of the Results

The comparison of the accuracy shows that DSLAM can provide much more accurate results in slightly dynamic environments than the methods that account for dynamic environments. Moreover, DSLAM obtains more competitive results than those obtained with the methods that assume a static world. In highly dynamic environments, DSLAM obtains more accurate results than most methods considering dynamic environments. The exception is DynaSLAM, which is time consuming to run and needs pretraining.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/eb2d7e28bc537fa251514b33d781b15e454339dfa8d68262fd33e7d41df9dbce.jpg)  
Fig. 16. Efficiency of the determination of static points by the back end. The relationship between the number of map points and the time spent by the operation is nearly linear at first. Because the local map points are from a keyframe within the sliding window, the runtime for the determination of the local map points does not grow indefinitely. Moreover, the second half of the graph indicated by the green shading is not smooth because the local map points contain dynamic points that are ignored during optimization.

Moreover, the results shown in the improvement columns in Tables 1, 2, and 3 prove that the robustness of motion estimation is improved significantly in highly dynamic environments because of the proposed segmentation. The comparison with the learning-based methods in Table 4 shows that if a method is only based on learning techniques to filter out moving objects and improve accuracy, it cannot provide the highest performance. Therefore, the proposed method performs better than most learning-based methods.

The comparison of the robustness demonstrates that DSLAM does not drift away from the ground truth, as do other methods, despite the presence of an unknown moving object with challenging motion in the FOV.

In the analysis of efficiency, although the learning-based methods with geometry cues can provide more accurate results, the computational resources they consume are also enormous. In contrast, the results show that DSLAM has the potential for real-time applications.

## 7 CONCLUSION AND FUTURE WORK

In this paper, a segmentation method using point correlations was proposed to divide map points into different components according to their own motion pattern. If there are moving objects, the map points are divided into multiple connected components in which every point has a correlation with the others in that component. Moreover, the proposed segmentation method is not limited to the type of sensor as long as the sensor can provide information related to the point correlations.

Integrating the proposed segmentation method, a SLAM method called DSLAM implemented on RGB-D sensors was proposed to eliminate the influence of moving objects in dynamic environments. DSLAM can provide accurate and robust results in dynamic environments. In the implementation, the sparse construction of a graph with the correlations of adjacent points reduces the computational complexity. An experimental comparison using a benchmark demonstrates that DSLAM can outperform state-of-the-art methods in most dynamic environments. Though it does not outperform DynaSLAM in accuracy, it does not require a GPU and needs less computation time for dynamic object segmentation. Moreover, the proposed method also could be combined with semantic cues to improve accuracy. We further evaluated DSLAM in three challenging environments. The results demonstrate that DSLAM can provide robust performance in challenging dynamic environments.

In future work, the proposed segmentation method will be extended to sensors such as those in monocular, stereo, and light detection and ranging systems. The main challenge is that different sensors have different noise models because an accurate noise model plays a crucial role in retaining the available information from the sensor. Moreover, the performance of the proposed method could be further improved. For example, graph construction can be incrementally implemented to avoid repeated graph creation and reduce complexity. Besides, the special sparsity of the Hessian structure of the point correlation formulation should be used to obtain more efficient solvers. Finally, random sample techniques will be considered to improve the robustness of the estimation.

## ACKNOWLEDGMENTS

This work was supported by the National Natural Science Foundation of China (Grant Nos. 61673341, and 61573091), National Key R&D Program of China (2016YFD0200701-3), China’s Double First-class Initiative, the Project of State Key Laboratory of Industrial Control Technology, Zhejiang University, China (No. ICT1913) and the Open Research Project of the State Key Laboratory of Industrial Control Technology, Zhejiang University, China (No. ICT1900312, No. ICT20037). Weichen Dai and Yu Zhang contributed equally to this work.

## REFERENCES

[1] D. Scaramuzza and F. Fraundorfer, “Visual odometry [tutorial],” IEEE Robot. Autom. Magazine, vol. 18, no. 4, pp. 80–92, Dec. 2011.

[2] J. Fuentes-Pacheco, J. Ruiz-Ascencio, and J. M. Rendon-Mancha,- “Visual simultaneous localization and mapping: A survey,” Artif. Intell. Rev., vol. 43, no. 1, pp. 55–81, 2015.

[3] C. Stachniss, J. J. Leonard, and S. Thrun, “Simultaneous localization and mapping,” in Springer Handbook of Robotics, Berlin, Germany: Springer, 2016, pp. 1153–1176.

[4] M. A. Fischler and R. C. Bolles, “Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography,” Commun. ACM, vol. 24, no. 6, pp. 381–395, 1981.

[5] C. Kerl, J. Sturm, and D. Cremers, “Robust odometry estimation for RGB-D cameras,” in Proc. IEEE Int. Conf. Robot. Autom., 2013, pp. 3748–3754.

[6] K. MacTavish and T. D. Barfoot, “At all costs: A comparison of robust cost functions for camera correspondence outliers,” in Proc. 12th Conf. Comput. Robot Vis., 2015, pp. 62–69.

[7] S. Lee, C. Y. Son, and H. J. Kim, “Robust real-time RGB-D visual odometry in dynamic environments via rigid motion model,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 6891–6898.

[8] E. Palazzolo, J. Behley, P. Lottes, P. Giguere, and C. Stachniss, “Refusion: 3D reconstruction in dynamic environments for RGB-D cameras exploiting residuals,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 7855–7862.

[9] B. Bescos, J. M. F-acil, J. Civera, and J. Neira, “DynaSLAM: Tracking, mapping, and inpainting in dynamic scenes,” IEEE Robot. Autom. Lett., vol. 3, no. 4, pp. 4076–4083, Oct. 2018.

[10] M. Runz, M. Buffier, and L. Agapito, “Maskfusion: Real-time recognition, tracking and reconstruction of multiple moving objects,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2018, pp. 10–20.

[11] A. Chiuso, P. Favaro, H. Jin, and S. Soatto, "Structure from motion causally integrated over time,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 24, no. 4, pp. 523–535, Apr. 2002.

[12] A. J. Davison, I. D. Reid, N. D. Molton, and O. Stasse, “Monoslam: Real-time single camera SLAM,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 29, no. 6, pp. 1052–1067, Jun. 2007.

[13] J.-S. Gutmann and K. Konolige, “Incremental mapping of large cyclic environments,” in Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom., 1999, pp. 318–325.

[14] E. Mouragnon, M. Lhuillier, M. Dhome, F. Dekeyser, and P. Sayd, “Real time localization and 3D reconstruction,” in Proc. IEEE Comput. Soc. Conf. Comput. Vis. Pattern Recognit., 2006, pp. 363–370.

[15] G. Klein and D. Murray, “Parallel tracking and mapping for small ar workspaces,” in Proc. 6th IEEE ACM Int. Symp. Mixed Augmented Reality, 2007, pp. 225–234.

[16] H. Strasdat, J. M. Montiel, and A. J. Davison, “Visual slam: Why filter?” Image Vis. Comput., vol. 30, no. 2, pp. 65–77, 2012.

[17] F. Endres, J. Hess, J. Sturm, D. Cremers, and W. Burgard, “3-D mapping with an RGB-D camera,” IEEE Trans. Robot., vol. 30, no. 1, pp. 177–187, Feb. 2013.

[18] C. Forster, Z. Zhang, M. Gassner, M. Werlberger, and D. Scaramuzza, “SVO: Semidirect visual odometry for monocular and multicamera systems,” IEEE Trans. Robot., vol. 33, no. 2, pp. 249–265, Apr. 2017.

[19] R. Mur-Artal and J. D. Tardos, “ORB-SIAM2: An open-sourceslam system for monocular, stereo, and RGB-D cameras,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1255–1262, Oct. 2017.

[20] J. Stuhmer, S. Gumhold, and D. Cremers, “Real-time dense geom-€ etry from a handheld camera,” in Joint Pattern Recognition Symposium. Berlin, Germany: Springer, 2010, pp. 11–20.

[21] R. A. Newcombe, S. J. Lovegrove, and A. J. Davison, “DTAM: Dense tracking and mapping in real-time,” in Proc. IEEE Int. Conf. Comput. Vis., 2011, pp. 2320–2327.

[22] F. Steinbrucker, J. Sturm, and D. Cremers, “Real-time visual€ odometry from dense RGB-D images,” in Proc. IEEE Int. Conf. Comput. Vis. Workshops, 2011, pp. 719–722.

[23] C. Cadena et al., “Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1309–1332, Dec. 2016.

[24] M. R. U. Saputra, A. Markham, and N. Trigoni, “Visual slam and structure from motion in dynamic environments: A survey,” ACM Comput. Surv., vol. 51, no. 2, pp. 1–36, 2018.

[25] P. F. Alcantarilla, J. J. Yebes, J. Almaz-an, and L. M. Bergasa, “On combining visual slam and dense scene flow to increase the robustness of localization and mapping in dynamic environments,” in Proc. IEEE Int. Conf. Robot. Autom., 2012, pp. 1290–1297.

[26] H. Azartash, K.-R. Lee, and T. Q. Nguyen, “Visual odometry for RGB-D cameras for dynamic scenes,” in Proc. IEEE Int. Conf. Acoust. Speech Signal Process., 2014, pp. 1280–1284.

[27] T. Zhang, H. Zhang, Y. Li, Y. Nakamura, and L. Zhang, “Flowfusion: Dynamic dense RGB-D slam based on optical flow,” in Proc. IEEE Int. Con. Robot. Autom., 2020.

[28] J. Stuckler and S. Behnke, “Efficient dense 3D rigid-body motion€ segmentation in RGB-D video,” in Proc. 24th British Mach. Vis. Conf., 2013, pp. 1–13.

[29] Y. Sun, M. Liu, and M. Q.-H. Meng, “Improving RGB-D slam in dynamic environments: A motion removal approach,” Robot. Auton. Syst., vol. 89, pp. 110–122, 2017.

[30] S. Li and D. Lee, “RGB-D slam in dynamic environments using static point weighting,” IEEE Robot. Autom. Lett., vol. 2, no. 4, pp. 2263–2270, Oct. 2017.

[31] D.-H. Kim and J.-H. Kim, “Effective background model-based RGB-D dense visual odometry in a dynamic environment,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1565–1573, Dec. 2016.

[32] M. Jaimez, C. Kerl, J. Gonzalez-Jimenez, and D. Cremers, “Fast odometry and scene flow from RGB-D cameras based on geometric clustering,” in Proc. IEEE Int. Conf. Robot. Autom., 2017, pp. 3992–3999.

[33] R. Scona, M. Jaimez, Y. R. Petillot, M. Fallon, and D. Cremers, “Staticfusion: Background reconstruction for dense RGB-D slam in dynamic environments,” in Proc. IEEE Int. Conf. Robot. Autom., 2018, pp. 1–9.

[34] D.-H. Kim, S.-B. Han, and J.-H. Kim, “Visual odometry algorithm using an RGB-D sensor and IMU in a highly dynamic environment,” in Proc. Int. Conf. Robot. Intell. Technol. Appl., 2015, pp. 11–26.

[35] J. Huang, S. Yang, T.-J. Mu, and S.-M. Hu, “Clustervo: Clustering moving instances and estimating visual odometry for self and surroundings,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2020, pp. 2168–2177.

[36] D. Migliore, R. Rigamonti, D. Marzorati, M. Matteucci, and D. G. Sorrenti, “Use a single camera for simultaneous localization and mapping with mobile object tracking in dynamic environments,” in Proc. ICRA Workshop Safe Navigation Open Dyn. Environ., Appl. Auton. Vehicles, 2009, pp. 12–17.

[37] A. Kundu, K. M. Krishna, and J. Sivaswamy, “Moving object detection by multi-view geometric techniques from a single camera mounted robot,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2009, pp. 4306–4312.

[38] W. Tan, H. Liu, Z. Dong, G. Zhang, and H. Bao, “Robust monocular slam in dynamic environments,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2013, pp. 209–218.

[39] D. Zou and P. Tan, "CoSLAM: Collaborative visual slam in dynamic environments,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 35, no. 2, pp. 354–366, Feb. 2012.

[40] J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, “You only look once: Unified, real-time object detection,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2016, pp. 779–788.

[41] K. He, G. Gkioxari, P. Dollar, and R. Girshick, “Mask R-CNN,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 42, no. 2, pp. 386–397, Feb. 2020.

[42] J. Dai, K. He, and J. Sun, “Instance-aware semantic segmentation via multi-task network cascades,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2016, pp. 3150–3158.

[43] B. Kitt, F. Moosmann, and C. Stiller, “Moving on to dynamic environments: Visual odometry using feature classification,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2010, pp. 5551–5556.

[44] L. Riazuelo, L. Montano, and J. Montiel, “Semantic visual slam in populated environments,” in Proc. Eur. Conf. Mobile Robots, 2017, pp. 1–7.

[45] I. A. B^arsan, P. Liu, M. Pollefeys, and A. Geiger, “Robust dense mapping for large-scale dynamic environments,” in Proc. IEEE Int. Conf. Robot. Autom., 2018, pp. 7510–7517.

[46] K. He, G. Gkioxari, P. Doll-ar, and R. Girshick, “Mask R-CNN,” in Proc. IEEE Int. Conf. Comput. Vis., 2017, pp. 2961–2969.

[47] K. Qiu, T. Qin, W. Gao, and S. Shen, “Tracking 3-D motion of dynamic objects using monocular visual-inertial sensing,” IEEE Trans. Robot., vol. 35, no. 4, pp. 799–816, Aug. 2019.

[48] D. Gordon, A. Farhadi, and D. Fox, "Re3: Re al-time recurrent regression networks for visual tracking of generic objects,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 788–795, Apr. 2018.

[49] S. Yang and S. Scherer, “Cubeslam: Monocular 3-D object slam,” IEEE Trans. Robot., vol. 35, no. 4, pp. 925–938, Aug. 2019.

[50] M. Strecke and J. Stuckler, “EM-fusion: Dynamic object-level SLAM with probabilistic data association,” in Proc. IEEE Int. Conf. Comput. Vis., 2019, pp. 5865–5874.

[51] M. Runz and L. Agapito, “Co-fusion: Real-time segmentation,€ tracking and fusion of multiple objects,” in Proc. IEEE Int. Conf. Robot. Autom., 2017, pp. 4471–4478.

[52] B. Xu, W. Li, D. Tzoumanikas, M. Bloesch, A. Davison, and S. Leutenegger, “MID-fusion: Octree-based object-level multiinstance dynamic SLAM,” in Proc. Int. Conf. Robot. Autom., 2019, pp. 5231–5237.

[53] T. D. Barfoot, State Estimation for Robotics. Cambridge, U.K.: Cambridge Univ. Press, 2017.

[54] R. Kummerle, G. Grisetti, H. Strasdat, K. Konolige, and W. Burgard,€ “g2o: A general framework for graph optimization,” in Proc. Int. Conf. Robot. Autom., 2011, pp. 3607–3613.

[55] L. Polok, V. Ila, M. <sup></sup>Solony, and P. Smrz, “Incremental block cholesky factorization for nonlinear least squares in robotics,” in Proc. Robot.: Sci. Syst., 2013, pp. 328–336.

[56] C. B. Barber, D. P. Dobkin, and H. Huhdanpaa, “The quickhull algorithm for convex hulls,” ACM Trans. Math. Softw., vol. 22, no. 4, pp. 469–483, 1996.

[57] J. Engel, V. Koltun, and D. Cremers, “Direct sparse odometry,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 3, pp. 611–625, Mar. 2018.

[58] K. Khoshelham and S. O. Elberink, “Accuracy and resolution of kinect depth data for indoor mapping applications,” Sensors, vol. 12, no. 2, pp. 1437–1454, 2012.

[59] I. Dryanovski, R. G. Valenti, and J. Xiao, “Fast visual odometry and mapping from RGB-D data,” in Proc. IEEE Int. Conf. Robot. Autom., 2013, pp. 2305–2310.

[60] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D slam systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots and Syst., 2012, pp. 573–580.

[61] C. Kerl, J. Sturm, and D. Cremers, “Dense visual slam for RGB-d cameras,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2013, pp. 2100–2106.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/e7bff722b5ff4b5ce762704a68568f899c23ff88b39c7a4c7143fcfed7edd4fa.jpg)

Weichen Dai received the BS degree in information engineering from the Zhejiang University of Technology, Hangzhou, China, in 2015. He is currently working toward the PhD degree in the College of Control Science and Engineering, Zhejiang University. His research interests include visual navigation, perception, and intelligent autonomous systems.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/1db2c64fd098a75fd1dcf062845995d4909ef87c31ea31a4f4b4ba6a3999e8b5.jpg)

Yu Zhang received the BS degree in information engineering from Xi’an Jiaotong University, Xi’an, China, in 2003, and the MS and PhD degrees in computer science from Tsinghua University, Beijing, China, in 2009. He was a postdoctoral fellow with Tsinghua University from 2009 to 2011 and a visiting scholar with Carnegie Mellon University from 2013 to 2014. He is currently an associate professor with the College of Control Science and Engineering, Zhejiang University, China. His research interests include visual navigation, intelligent control, computer vision, and intelligent autonomous systems.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/bf8a2160386dd002749ee79412374fbf931c4310ae2a3afcf465f804c5bf1f78.jpg)

Ping Li received the BS degree in chemical engineering automation, in 1982, and the MS and PhD degrees in industrial automation from Zhejiang University, Hangzhou, China, in 1985 and 1988, respectively. He is now a professor with the College of Control Science and Engineering, Zhejiang University. His research interests include unmanned aerial vehicle navigation and control, industrial process control and optimization, and intelligent transportation systems.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/5603c0b8585de029c1484a7d4b03ae23ed814618786173f39e88760dcdd6a1a4.jpg)

Zheng Fang received the BS degree in automation and PhD degree in pattern recognition and intelligent systems from Northeastern University, China, in 2002 and 2006, respectively. He was a postdoctoral research fellow with Carnegie Mellon University from 2013 to 2015. He is now an associate professor with the Faculty of Robot Science and Engineering, Northeastern University, China. His research interests include visual/laser SLAM, perception, and autonomous navigation of various mobile robots.

![](images/2022_RGB-D_SLAM_in_Dynamic_Environments_Using_Point_Correlati/8dbd95d314c8c7d838e0df0f0bc11c298b4344bee94f97662381bcbd6f87a992.jpg)

Sebastian Scherer received the BS degree in computer science and the MS and PhD degrees in robotics from Carnegie Mellon University (CMU), Pittsburgh, PA, in 2004, 2007, and 2010, respectively. He is currently an associate research professor with the Robotics Institute, Carnegie Mellon University. He and his team have demonstrated the fastest and most tested obstacle avoidance on a YamahaRMax (2006), the first obstacle avoidance for microaerial vehicles in natural environments (2008), and the first (2010) and fastest (2014) automatic landing zone detection and landing on a full-size helicopter. His research interest includes enabling autonomy for unmanned rotorcraft to operate at low altitude in cluttered environments.

" For more information on this or any other computing topic, please visit our Digital Library at www.computer.org/csdl.