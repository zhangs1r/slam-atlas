# LOG-LIO: A LiDAR-Inertial Odometry With Efficient Local Geometric Information Estimation

Kai Huang , Junqiao Zhao , Member, IEEE, Zhongyang Zhu , Chen Ye , Member, IEEE, and Tiantian Feng

Abstract—Local geometric information, i.e., normal and distribution of points, is crucial for LiDAR-based simultaneous localization and mapping (SLAM) because it provides constraints for data association, which further determines the direction of optimization and ultimately affects the accuracy of localization. However, estimating normal and distribution of points are time-consuming tasks even with the assistance of kdtree or volumetric maps. To achieve fast normal estimation, we look into the structure ofLiDAR scan and propose a ring-based fast approximate least squares (Ring FALS) method. With the Ring structural information, estimating the normal requires only the range information of the points when a new scan arrives. To efficiently estimate the distribution of points, we extend the ikd-tree to manage the map in voxels and update the distribution of points in each voxel incrementally while maintaining its consistency with the normal estimation. We further fix the distribution after its convergence to balance the time consumption and the correctness of representation. Based on the extracted and maintained local geometric information, we devise a robust and accurate hierarchical data association scheme where point-to-surfel association is prioritized over point-to-plane. Extensive experiments on diverse public datasets demonstrate the advantages of our system compared to other state-of-the-art methods.

Index Terms—Lidar-inertial odometry, SLAM, sensor fusion.

## I. INTRODUCTION

ily on the registration between the LiDAR scan and the map, i.e., finding the correspondence between them based on the similarity of the local geometric information and then minimizing their distance. Local geometric information includes attributes that can represent the position, shape, and other characteristics of the local surface where a point is located.

The conventional method for estimating local geometric information is to evaluate the smoothness of the input scan and to locally approximate the map with geometric primitives [1]. However, accurate estimation of local geometric information requires the retrieval of neighborhood information in a dense point cloud, but for LiDAR-inertial odometry (LIO) systems, this results in a huge computational burden even with the help of kdtree or volumetric maps.

Accurate and fast estimation of local geometric information has gained increasing attention in recent studies [2], [3], [4], [5], [6]. Among them, the normal and the distribution of points are two representative attributes, since the former indicates the tangent plane of the local surface, and the latter implies the average position and shape of the point cloud sampled from the local surface. However, current LIO systems seldom incorporate the real-time estimation of the normal and the distribution of points, which hampers their pose estimation performance.

This letter presents LOG-LIO, a robust and accurate LIO system focusing on the real-time estimation ofthe normal ofLiDAR scan points and the distribution of map points, and their rational utilization. Inspired by [7] and [8], we look into the structure of a LiDAR scan and propose a Ring-based fast approximate least squares method, namely Ring FALS. We project point cloud onto the range image to pre-build a lookup table, which represents the structural information of the specific LiDAR. With the arrival of a new scan, only the range information of the points is needed to estimate the normal. We incrementally update the distribution of points for each voxel in the map while maintaining its consistency with the normals. To balance time consumption and correctness of representation, we manage the map on the extended ikd-tree and further fix the distribution after it converges.

Similar to the FAST-LIO series [9], [10], we directly associate scan points with voxels on the map after distortion correction. For scan points that satisfy visibility and consistency checks based on normals, we devise a robust and accurate hierarchical data association scheme considering the distribution. The poses are optimized by integrating the IMU measurements as initial estimates and then using an error-state iterative Extended Kalman filter (iEKF) [9] to minimize the multi-scale point-to-surfel and point-to-plane distances.

The main contributions of this work are as follows:

\- Ring FALS, modified from FALS, a normal estimator that utilizes the structural information of LiDAR scan can meet the real-time requirements of the LIO system.

\- A robust and accurate hierarchical data association scheme considering the distribution of points within map voxels where point-to-surfel is prioritized over point-to-plane and large-scale surfel over small-scale surfel.

\- Extensive experiments on public datasets demonstrate the advantages of our LIO system compared to other stateof-the-art methods. To benefit the community, our implementation of this work is open-source at https://github. com/tiev-tongji/LOG-LIO, and we also open-source Ring FALS as an independent normal estimation tool at https: //github.com/tiev-tongji/RingFalsNormal.

## II. RELATED WORKS

## A. Point Cloud Normal Estimation

The most commonly used method to obtain surface normals from point cloud is the least square estimation based on the neighborhood search due to its ease of implementation [11]. However, the least squares-based method is computationally expensive for LIO systems.

[7] compares the complexity of least squares approaches and proposes FALS, which simplifies the least squares loss function to completely avoid the computation of the covariance matrix for each point. [7] also reformulates the traditional least squares solution to estimate the normal by calculating the derivatives of the surface from a spherical range image (SRI). However, the number of multiplications for normals computation of SRI is greater than FALS.

Rather than least square-based solution, 3F2N [8] performs three filtering operations on the inverse depth image to estimate the normals, which has the comparative performance as FALS, but its efficiency and accuracy are strongly influenced by the filter selection.

Inspired by [7] and [8], we propose Ring FALS. We pre-build a lookup table which represent the structural information for the specific LiDAR. Compared to FALS, Ring FALS further simplifies the projection of each LiDAR scan with the assistance of ring index, while preserving accuracy.

## B. Distribution of Points Estimation

The distribution of a point is represented through its 3D coordinates and the covariance matrix computed by its neighboring points. [12] proposes the generalized ICP (GICP) algorithm, which takes into account the locally planar structure of points in a probabilistic model and then minimizes the distance between distributions. But searching neighboring points to compute the covariance matrix is too time-consuming for LIO systems.

LOAM [1] does not estimate the distribution of points, but performs Eigen analysis on the associated map points to determine whether its local geometry is a line or a plane. However, the coordinates of the sparse map points cannot accurately represent the local geometric information, which leads to inaccurate constraints for the registration.

![](images/2024_LOG-LIO/c6132056351d655d3aea097db46784a929f164c3873357f067e040cbe103e7f1.jpg)  
(a) LiDAR observation model

![](images/2024_LOG-LIO/e1100e11c102250a4bc6197fe4d7a15a3159bada8beb1ee576fe7ef6e2856366.jpg)  
(b) multi-scale surfel association  
Fig. 1. Illustration of the LiDAR observation model and multi-scale surfel association. (a) The magenta line indicates the ray of the red point. The eigh points are the neighborhoods that Ring FALS uses to estimate the normal of the red point. (b) The orange ellipses represent the large-scale surfel merged by the five blue small-scale surfels.

DLO [2] registers point cloud using GICP to minimize the plane-to-plane distance, which is derived from the covariance matrix of each point. It assumes that the covariance of submap can be approximated by concatenating the normals from keyframes, and the covariance of points is only computed once when the scan is acquired. However, such a normals stitching method cannot accurately reflect the local geometric information of the point cloud, which ultimately affects accuracy. LOCUS 2.0 [4] extends the work of LOCUS [5], which constructs covariance matrices for GICP-based registration based on the pre-computed normals, but how to pre-compute normals is not elaborated in their letter.

Wildcat [6] fits ellipsoids based on the coordinates and timestamps ofthe clustered points. The ellipsoids representing the distribution of points are further used to generate surfels. SLICT [3] further proposes an octree-based global map and updates the distribution of points within each voxel incrementally. It obtains large-scale distributions by merging multiple voxels to generate surfels in multi-resolution.

Inspired by the above methods, we extend ikd-tree to maintain the distribution of points in each map node incrementally, and fix the distribution after its convergences.

## C. LiDAR (-Inertial) Odometry

LOAM [1] has inspired many LiDAR SLAM systems due to the low coupling of system modules and the rational use of point cloud geometry attributes. However, the lack of effective map management and the high time consumption required for optimization can degrade the performance of the system.

LIO-SAM [13] proposes a framework based on keyframes and local maps, which optimizes poses in a factor graph. However, LIO-SAM builds sub-maps for input scans by simply merging point cloud of surrounding keyframes, which is a timeconsuming process when the number of points is large compared to incrementally maintaining maps.

FAST-LIO [9] employs point-to-plane correspondence and the iEKF to directly register the LiDAR scan and the map. It presents a new formula to compute the Kalman gain, and the computation load only depends on the dimension of state dimension. FAST-LIO2 [10] maintains the map by an incremental kdtree data structure, namely ikd-tree, to further improve efficiency.

In this letter, we adopt the iEKF to optimize poses by directly registering the LiDAR scan and the map, but with a different data association scheme. By incorporating real-time normal and distribution of points estimation, we can efficiently construct surfels in multi-scale, which represent the local surface geometry more accurately than other geometric primitives, e.g., plane. We prioritize associating large-scale surfels over small-scale surfels since large-scale surfels are modeled with more points and are insensitive to noise.

## III. PRELIMINARY

## A. Notation

We now define notations and frames that we used throughout the letter. We consider as the world frame and $\mathcal { T } _ { k } , \mathcal { L } _ { k }$ as the IMU and LiDAR frames, related to the k-th LiDAR scan at time $t _ { k } .$ , respectively. ${ } ^ { a } \mathbf { T } _ { b } \in S E ( 3 )$ to be Euclidean transformation take 3D points from frame b to frame a, which is consisted of rotation ${ } ^ { a } \mathbf { R } _ { b } \in S O ( 3 )$ and translation ${ } ^ { a } \mathbf { t } _ { b } \in \mathbb { R } ^ { 3 }$ $\mathbf { \nabla } _ { \mathbf { \eta } _ { n _ { \mathrm { ~ r ~ } } } }$ denotes the normal from Ring FALS and $e _ { d }$ is the eigenvector corresponding to the smallest eigenvalue of a distribution (see Section III-E).

## B. LiDAR Observation Model

In practice, LiDAR obtains the 3D coordinates of a point by combining bearing and range measurements of the target surface [14], [15], as shown in Fig. 1(a). The LiDAR observation model is as follows:

$$
\begin{array} { r } { \pmb { p _ { i } } = r _ { i } \pmb { v _ { i } } = r _ { i } \left[ \begin{array} { c } { \cos \theta _ { i } \cos \varphi _ { i } } \\ { \sin \theta _ { i } \cos \varphi _ { i } } \\ { \sin \varphi _ { i } } \end{array} \right] } \end{array}\tag{1}
$$

where $r _ { i }$ is the range, $\theta _ { i }$ the azimuth and $\varphi _ { i }$ the vertical angle of the target point. ${ \mathbf { } } v _ { i }$ represents the horizontal and vertical structural information of the point relative to the LiDAR.

For a spinning LiDAR, we denote the horizontal resolution as $H _ { r e s } = 2 \pi / m$ , where m is the constant number of points within each ring. Denoting the structural information $s _ { i } = [ \cos \theta _ { i }$ sin $\theta _ { i }$ cos $\varphi _ { i }$ sin $\varphi _ { i } \mathbf { \bar { \Pi } } ^ { - }$ and then $s _ { i }$ can be arranged into a lookup table based on the ring index and azimuth relative to the LiDAR as follows:

$$
\mathbf { \boldsymbol { s } } _ { i } = \mathcal { T } ( r o w _ { i } , \ c o l _ { i } )\tag{2}
$$

where row<sub>i</sub> represents the ring index of $\mathbf { \nabla } p _ { i }$ and $c o l _ { i } =$ round $\left( \theta _ { i } / H _ { r e s } \right)$

## C. Least Squares Normal Estimation

Given a subset ofn 3D points $p _ { i } , i = 1 , 2 , . . . , r$ ofthe surface, least squares finds the normal vector $\pmb { n } = ( n _ { x } , n _ { y } , n _ { z } )$ and the scalar d that minimizes (3).

$$
e = \sum _ { i = 1 } ^ { n } \left( { \pmb { p } } _ { i } ^ { T } { \pmb { n } } - d \right) ^ { 2 }\tag{3}
$$

The closed form solution of the normal n is the eigenvector corresponding to the smallest eigenvalue of the covariance matrix

in (4).

$$
M = \sum _ { i = 1 } ^ { n } \left( { \pmb { p } } _ { i } - { \overline { { \pmb { p } } } } \right) \left( { \pmb { p } } _ { i } - { \overline { { \pmb { p } } } } \right) ^ { T }\tag{4}
$$

with $\textstyle { \overline { { p } } } = 1 / n \sum _ { i = 1 } ^ { n } p _ { i }$

## D. Distribution ofPoints

The distribution of points within a voxel can be represented by its mean position p and the covariance matrix M, as shown in (4). And M can be further simplified as follows:

$$
M = S _ { n } - \frac { 1 } { n } \mathcal { P } _ { n } \mathcal { P } _ { n } ^ { T }\tag{5}
$$

where $S _ { n }$ denotes $\scriptstyle \sum _ { i = 1 } ^ { n } p _ { i } p _ { i } ^ { T }$ and $\mathcal { P } _ { n }$ denotes $\scriptstyle \sum _ { i = 1 } ^ { n } p _ { i }$ . Due to the symmetric nature of M, it is only necessary to record the six elements in its upper right corner.

The accurate representation of the distribution of points requires a large number of points. Due to limited resolution and occlusion, point cloud from multiple locations must be accumulated incrementally to obtain high quality maps. For newly incorporated m points in a voxel, their $S _ { m } , \mathcal { P } _ { m }$ need to be calculated. Subsequently, the distribution of points within this voxel can be updated by [16]:

$$
\begin{array} { c } { \overline { { p } } = ( \mathcal { P } _ { n } + \mathcal { P } _ { m } ) / ( n + m ) } \\ { M = S _ { n } + S _ { m } - \displaystyle \frac { 1 } { n + m } ( \mathcal { P } _ { n } + \mathcal { P } _ { m } ) ( \mathcal { P } _ { n } + \mathcal { P } _ { m } ) ^ { T } } \end{array}\tag{6}
$$

## E. Surfel

We define the planarity $\rho$ within a voxel similar to SLICT [3], and further introduce $\gamma$ as following:

$$
\begin{array} { r l } & { \rho _ { i } = 2 ( \lambda _ { 2 } - \lambda _ { 1 } ) / ( \lambda _ { 1 } + \lambda _ { 2 } + \lambda _ { 3 } ) } \\ & { \gamma _ { i } = \lambda _ { 2 } / \lambda _ { 1 } } \end{array}\tag{7}
$$

where $\lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 }$ are the eigenvalues of covariance matrix M with $\lambda _ { 1 } < \lambda _ { 2 } < \lambda _ { 3 }$ . We define a surfel has $\rho _ { i }$ greater than 1.0 and $\gamma _ { i }$ greater than 100. A larger $\rho _ { i }$ implies that the distribution of the sampled points is flatter on the surface, and a larger $\gamma _ { i }$ indicates that the distribution is less close to a linear geometry. If the above criteria are satisfied, the surfel is represented by the mean position of points $\overline { { p } }$ and the normal $e _ { d } .$ , where $e _ { d }$ is the eigenvector corresponding to $\lambda _ { 1 }$ . Multiple small-scale surfels can be merged into a large-scale surfel by merging the distributions following (6). And the merged distribution still needs to satisfy the criteria in the above to be considered as a large-scale surfel.

## IV. RING FALS NORMAL ESTIMATOR

We first revisit FALS [7]. In FALS, (3) is reformulated to obtain:

$$
\widetilde e = \sum _ { i = 1 } ^ { n } \left( \pmb { p } _ { i } ^ { T } \widetilde { \pmb { n } } - 1 \right) ^ { 2 }\tag{8}
$$

![](images/2024_LOG-LIO/dc8901cb453625ed04800f2fb17f402f77d37285d522bd4bfd4d7b5976c47cab.jpg)  
Fig. 2. System overview of LOG-LIO.

where $\widetilde { n }$ is defined up to a scale factor. Substituting (1) gives:

$$
\widetilde { e } = \sum _ { i = 1 } ^ { n } r _ { i } ^ { 2 } \left( \pmb { v } _ { i } ^ { T } \widetilde { \pmb { n } } - \boldsymbol { r } _ { i } ^ { - 1 } \right) ^ { 2 }\tag{9}
$$

where $r _ { i }$ is the range and ${ \mathbf { } } v _ { i }$ implies the bearing information of the target points related to the LiDAR.

It can be assumed that the range of points within a small region are similar, thanks to the high-resolution LiDAR. Therefore, $r _ { i } ^ { 2 }$ can be removed from (9) to obtain an approximation:

$$
\widehat { e } = \sum _ { i = 1 } ^ { n } \left( { \pmb v } _ { i } ^ { T } \widehat { { \pmb n } } - { \boldsymbol r } _ { i } ^ { - 1 } \right) ^ { 2 }\tag{10}
$$

where $\widehat { \mathbfcal { n } }$ is the approximate normal, and it has the closed form solution $\widehat { \pmb { n } } = \widehat { \pmb { M } } ^ { - 1 } \widehat { \pmb { b } }$ where $\begin{array} { r } { \widehat { M } = \sum _ { i = 1 } ^ { n } { \pmb { v } _ { i } \pmb { v } _ { i } ^ { T } } } \end{array}$ and $\widehat { \pmb { b } } =$ $\scriptstyle \sum _ { i = 1 } ^ { n } v _ { i } / r _ { i }$ . The matrix $\widehat { M } ^ { - 1 }$ depends only on the constant structural information v, independent of the range r. Hence, the matrix $\widehat { M } ^ { - 1 }$ can be pre-computed as a lookup table.

To obtain the neighborhood of ${ \mathbf { } } v _ { i }$ for computing $\widehat { M } ^ { - 1 }$ and ${ \widehat { \boldsymbol { b } } } ,$ FALS projects the LiDAR scan points onto an SRI following (1). The computation of $\widehat { M } ^ { - 1 }$ and $\widehat { b }$ requires that each pixel in the SRI has a corresponding measurement, hence an interpolation is needed since the LiDAR scan points only occupy sparse pixels. This is in turn a time-consuming process.

Different from FALS, Ring FALS establishes a fast mapping following (2) based on the structural information of the LiDAR. To avoid the costly vacant pixel interpolation, we create a table with the number of rows corresponding to the number of rings and the number ofcolumns matching the number ofpoints within each ring. The table is created solely based on the provided measurements, eliminating the necessity for interpolation. Thus Ring FALS speeds up the projection step in FALS and circumvents the time-consuming neighborhood search in many LIO systems, facilitating dense normal estimation for LiDAR scans.

Note that there are instances where the assumption of Ring FALS may not hold. Such cases include scenarios like wall edges, occlusions where the range of points varies significantly in a small area, and situations with missing range measurements. To address this, we flip all the backfaced normals and then apply image median blurring to smooth the normals and enhance their consistency. For points whose normal direction still differs significantly from the associated map points, we identify them as outliers in the optimization process through visibility and consistency checks, as elaborated in Section V-B2.

## V. SYSTEM DESCRIPTION

The pipeline of LOG-LIO is shown in Fig. 2. For a new input scan, we first estimate the normal ofthe points. The association is then performed between the undistorted point cloud and the map according to their local geometric information. We incorporate the measurements of IMU and optimize the poses of the body via iEKF. After optimization, new points are added to the map managed by the extended ikd-tree, and the distribution within a voxel is incrementally maintained.

Our system takes the IMU frame as the body frame, where the system state x can be written as:

$$
\mathbf { x } = \left[ ^ { \mathcal { W } } \mathbf { R } _ { \mathcal { T } } ^ { T } \mathbf { \Sigma } ^ { \mathcal { W } } \mathbf { p } _ { \mathcal { T } } ^ { T } \mathbf { \Sigma } ^ { \mathcal { W } } \mathbf { v } _ { \mathcal { T } } ^ { T } \mathbf { \Phi } \mathbf { b } _ { \omega } ^ { T } \mathbf { \Phi } \mathbf { b } _ { a } ^ { T } \mathbf { \Sigma } ^ { \mathcal { W } } \mathbf { g } ^ { T } \right]\tag{11}
$$

whereWR<sup>T</sup>, ${ } ^ { w } { \bf p } _ { \mathbb { Z } } ^ { T }$ and $w _ { \mathbf { v } _ { \tau } ^ { T } }$ are the orientation, position and velocity of IMU in the world frame (i.e., the first IMU frame), $\mathbf { b } _ { \omega } ^ { T }$ and ${ \mathbf b } _ { a } ^ { T }$ are gyroscope and accelerometer bias respectively, $\mathcal { W } _ { \mathbf { g } ^ { T } }$ is the known gravity vector in the world frame.

## A. Data Pre-Processing

LOG-LIO uses Ring FALS (Section IV) to to estimate the normal for each input point, denoted as $\mathbf { \delta } _ { \mathbf { \eta } ^ { n _ { r } } }$ . Subsequently, voxel grid downsampling and backward propagation based on IMU measurements are used for point reduction and distortion correction, respectively.

## B. Data Association

At the beginning of data association, the IMU measurements are integrated from the previous frame to predict the pose $\widehat { \mathbf { x } } _ { k }$ Using this prediction, each new input point ${ \mathcal { L } } _ { p _ { i } }$ <sub>is</sub> <sub>transformed</sub> <sub>to</sub> the world frame $^ { \dag } p _ { i } = ^ { \dag } \hat { T } _ { \mathcal { T } } { ^ { \mathcal { I } } T _ { \mathcal { L } } } ^ { \mathcal { L } } p _ { i }$ . Then, data association is performed in three consecutive steps:

1) Initial Correspondence: For a query point ${ \boldsymbol { \ w } } _ { \pmb { p } _ { i } }$ , we first search for its k nearest map points corresponding to k nearest map voxels.

2) Visibility and Consistency Checks: The candidate associated map points may not be visible to the LiDAR if the angle between the normal of the map point and the ray (vector from the query scan point to the LiDAR center) is greater than 90 degrees. Such a case usually occurs indoors, where the two planes of an object (e.g., a wall) are close to each other, which is referred to as the double-side issue [17]. This incorrect correspondence is eliminated directly.

The consistency of the associated map points is evaluated by computing the average angle between the normal of the query point and the normals of the associated map points. If the average angle is larger than a threshold α $( \alpha = 6 0 ^ { \circ } )$ , we consider it an inconsistent association and discard it.

3) Hierarchical Association: For query points satisfying visibility and consistency checks, a hierarchical association is performed, where point-to-surfel is prioritized over point-to-plane, and large-scale surfels are prioritized over small-scale surfels.

Surfels offer a more precise and flexible representation of a local surface compared to a plane fitted with sparse map points since they are modeled with the distribution of points, which not only indicates the location but also captures the shape ofthe local surface. Large-scale surfel can be approximated by merging multiple small-scale distributions following (6). Moreover, they exhibit a high tolerance to noise, thereby providing more robust constraints when contrasted with small-scale surfels. As illustrated in Fig. 1(b), the orange ellipse depicts the large-scale surfel merged by the five blue small-scale surfels. The green query point is initially associated with the merged large-scale surfel if the merged large-scale surfel satisfies the criteria outlined in Section III-E, and the distance from the mean position of each small-scale surfel to the large-scale surfel is below a predefined threshold. Otherwise, the association with small-scale surfels is preferred.

For constraints with small-scale surfels, we associate the query point with the surfel of the voxel where the point is located, which must already be fixed. If the voxel cannot meet the criteria of a surfel (Section III-E), we resort to using the point-to-plane association as LOAM [1].

## C. Pose Optimization

We adopt the iEKF from FAST-LIO2 to optimize the pose. The prediction step is implemented by the integration of IMU measurements from the latest optimized state $\overline { { \mathbf { x } } } _ { k - 1 }$ along with the covariance matrix $\overline { { \mathbf { P } } } _ { k - 1 }$

For the residual computation, given a point ${ } ^ { \mathcal { W } } p _ { i }$ in the world frame, the residual $z _ { i }$ is calculated as:

$$
{ \bf z } _ { i } = { { \pmb n } _ { j } } \left( ^ { \mathcal { W } } { { p } _ { i } } - ^ { \mathcal { W } } { { \pmb q } _ { j } } \right)\tag{12}
$$

where $\boldsymbol { n } _ { j }$ is the normalized normal of the associated surfel or plane for $\mathbf { \nabla } _ { p _ { i } }$ , and ${ } ^ { \mathcal { W } } { \bf q } _ { j }$ is a point lying on the associated element.

Then, we denote the propagated state and covariance by $\widehat { \mathbf { x } } _ { k }$ and $\widehat { \mathbf { P } } _ { k }$ respectively. They represent the prior Gaussian distribution for the state. By incorporating the prior distribution and the measurement models for point-to-surfel and point-to-plane associations from (12), we obtain the maximum a-posterior estimate (MAP) as follows:

$$
\begin{array} { l } { { \displaystyle { m i n \left( \| { \bf { x } } _ { k } \boxed { \hat { \bf { x } } _ { k } } \| _ { \hat { \bf { p } } _ { k } } ^ { 2 } + \sum _ { i \in s u r f e l } \| { \bf { z } } _ { i } ^ { \kappa } + { \bf { H } } _ { i } ^ { \kappa } \widetilde { \bf { x } } _ { k } ^ { \kappa } \| _ { { \bf { R } } _ { i } } ^ { 2 } \right. } } } \\ { { \displaystyle { \left. + \sum _ { j \in p l a n e } \| { \bf { z } } _ { j } ^ { \kappa } + { \bf { H } } _ { j } ^ { \kappa } \widetilde { \bf { x } } _ { k } ^ { \kappa } \| _ { { \bf { Q } } _ { j } } ^ { 2 } \right) } } } \end{array}\tag{13}
$$

where - computes the difference between $\mathbf { x } _ { k }$ and $\widehat { \mathbf { x } } _ { k }$ in the local tangent space of $\mathbf { x } _ { k } , \widetilde { \mathbf { x } } _ { k } ^ { \kappa }$ is the error of the κ-th iterate update at time $k , \mathbf { H } _ { i } ^ { \kappa }$ and $\mathbf { H } _ { j } ^ { \kappa }$ are Jacobian matrices with respect to $\widetilde { \mathbf { x } } _ { k } ^ { \kappa } .$ $\mathbf { R } _ { i }$ and $\mathbf { Q } _ { j }$ come from the raw measurement noise. Compared with FAST-LIO2, we augment the MAP with point-to-surfel associations, which are the middle term of (13). The Kalman gain can be computed efficiently, with the computation load depending on the state dimension instead of the measurement dimension [9], [10].

## D. Map Management

LOG-LIO uses an extended ikd-tree to manage the map. The ikd-tree originally stores map points in both leaf nodes and internal nodes [10]. In our extension, we additionally store a distribution in each node, and this distribution is maintained through voxels. Upon the first scan, we initialize the tree-structured map with a predetermined voxel resolution and associate the distribution of points within each voxel with the corresponding tree node. For subsequent points, if they fall within the same voxel as the nearest associated map point, we incrementally update the distribution within the voxel (Section III-D). In cases where the points belong to a different voxel, we initialize a new tree node encompassing those points and the voxel and add it to the map.

To balance computational efficiency and accuracy, we limit the number of points added to each voxel by tuning the downsampling rate in pre-processing. Additionally, we consider the distribution stabilizes once the directions of ${ \mathbf { } } n _ { \eta }$ <sub>r</sub> and $e _ { d }$ converge. The distribution is then fixed in the map.

## VI. EXPERIMENTAL RESULTS

## A. Implementation Details

In data pre-processing, we set the downsampling grid to match the map’s voxel size, ensuring that each voxel contains at most one point per frame. And we normalize the normal after downsamping. Within the extended ikd-tree nodes, we maintain the point distribution, updating it once the voxel accumulates $\eta = 2 5$ points. When the angle between $\mathbf { n } _ { r }$ and $e _ { d }$ falls below 20 degrees, we consider the distribution stabilized, and both $\mathbf { } _ { \mathbf { } ^ { n _ { \gamma } } }$ and $e _ { d }$ are fixed and replaced by the average of their values. For map voxels that accumulate 2η points without stabilization, we conclude that their distribution no longer requires updates and fix it.

## B. Experimental Settings

The experiment focuses on the following two research questions:

\- Can Ring FALS estimates the normal of LiDAR points in real-time and accurately represents environmental information?

\- Can LOG-LIO improve the accuracy of pose estimation by incorporating normal and distribution of points estimation?

We conduct extensive experiments on the M2DGR [18] and NTU VIRAL [19] datasets, both of which include 9-axis IMU measurements and ground truth trajectories. The M2DGR dataset collects data on a ground platform equipped with Velodyne-32 LiDAR and captured in diverse indoor and outdoor scenarios with ground truth trajectories obtained from laser 3D tracking, motion capture, and RTK receivers. The NTU VIRAL dataset collects data on an Unmanned Aerial Vehicle (UAV) platform with ground truth obtained by a laser-tracker total station with centimeter-level accuracy. The horizontal Ouster 16-channel OS1 LiDAR and VectorNav VN100 IMU are used. Compared with M2DGR, the LiDAR used by VIRAL has a sparser point cloud, making it more challenging to estimate poses in open areas. In the experiments, the resolution of maps and new scan downsampling size are set to 0.4 m for M2DGR and 0.5 m for NTU VIRAL respectively. Our workstation runs with Ubuntu 18.04, equipped with an Intel Core Xeon(R) Gold 6248R 3.00 GHz processor and 32 GB RAM.

![](images/2024_LOG-LIO/5548692a9da9cde8870b8687fb703c5444dce76e597f893d5b8c1cb5a7cb898e.jpg)  
Fig. 3. Starting position of the sequence gate03 of M2DGR dataset. The white lines represent normalized normals from Ring FALS estimation.

TABLE I  
MEAN RUNNING TIME (MS) OF NORMAL ESTIMATION FOR A SINGLE SCAN TO CERTAIN LIDARS
<table><tr><td rowspan="2"></td><td rowspan="2">points</td><td colspan="4">Ring FALS</td><td colspan="2">PCL</td></tr><tr><td>projection</td><td>box-filtering</td><td>smoothing</td><td>total</td><td>single thread</td><td>OMP 10 threads</td></tr><tr><td>Velodyne-32</td><td>57600</td><td>2.045</td><td>2.540</td><td>3.199</td><td>7.784</td><td>79.811</td><td>26.355</td></tr><tr><td>Ouster-16</td><td>16384</td><td>0.560</td><td>1.221</td><td>0.815</td><td>2.597</td><td>155.972</td><td>39.664</td></tr></table>

The minimum mean running time is marked in bold

## C. Evaluation ofNormal Estimation

We conduct a comparative analysis of Ring FALS and PCL [11] normal estimation tools. The implementation of PCL normal estimation is based on traditional least squares with the assistance of kdtree. PCL also provides a parallel implementation using OpenMP to speed up the computation. Note that normal smoothing is a time-consuming process for PCL, so only Ring FALS smoothness the normals.

To provide an intuitive evaluation, we visualize the normals at the starting position of sequence gate03 of the M2DGR dataset. As shown in Fig. 3, the white lines represent the normalized normals estimated by Ring FALS. Notably, almost all ground point normals exhibit a vertical upward orientation. With respect to the pillar in the yellow box, the normals at the corners transit smoothly with the normals on the adjacent sides. Due to occlusion, points near fake edges within the green box fail to meet the neighborhood range similarity assumption, leading to inaccurate Ring FALS estimation. However, these points with misestimated normals are a minority within the scan and are filtered out during visibility and consistency checks (Section V-B2).

Table I shows the average processing times of normal estimation for a single LiDAR scan from the M2DGR and NTU VIRAL datasets, respectively. The M2DGR dataset contains about 57,600 points per scan. Ring FALS demonstrates significantly reduced processing time compared to PCL, taking only one-tenth of the time, and it’s four times faster than the OpenMP version. For NTU VIRAL, Ring FALS also achieves significantly shorter processing times compared to PCL, with or without OpenMP.

It is noteworthy that despite the Ouster-16 LiDAR having fewer points than Velodyne-32 in one scan, the consumption time increases. This is due to the time-consuming kdtree neighborhood search in PCL’s normal estimation. And it is influenced by the spatial structure of the kdtree, which, in turn, reflects the complexity of the environment.

## D. Evaluation of Odometry

We compare LOG-LIO with two state-of-the-art LIO methods, FAST-LIO2 [10] and LIO-SAM [20] without enabling loop closure. Accuracy assessment is based on the root-mean-square error (RMSE) of absolute trajectory error (ATE). We employ LOG-C, which only performs point-to-plane association, for ablation experiments.

1) M2DGR Datasets: Due to the instability of the RTK signal, the first 100 seconds and the last 100 seconds of street07 and street10 are discarded in the experiment.

Table II reports the quantitative results. Notably, LOG-LIO, LOG-C, and FAST-LIO2 show comparable accuracy in indoor scenes, such as doors and halls, outperforming LIO-SAM in most cases. This is due to the abundance of planar features in indoor scenes, which results in more map points forming true planes. Consequently, the point-to-plane data association provides effective constraints for pose estimation. Conversely, the point-to-line data association of LIO-SAM may become less reliable, especially when errors accumulate.

![](images/2024_LOG-LIO/9a26b6711c834b38890d341c1963fb503ce5b096be6c1ec3325122d19e7c3488.jpg)

![](images/2024_LOG-LIO/e63405bf773b56644ab0c40dc29d2f790851bcd78257ed89927eb77208fb533a.jpg)  
Fig. 4. Localization estimates in sequence street10 of the M2DGR dataset. The zoomed in image of the colored boxes corresponds to the boxes of the same color in the trajectory.

TABLE II  
TRANSLATION RMSE (M) RESULTS OF POSE ESTIMATION COMPARISON ON THE M2DGR DATASET
<table><tr><td rowspan=1 colspan=1>Seq</td><td rowspan=1 colspan=1>duration(s)</td><td rowspan=1 colspan=5>LOG-LIO  LOG-C  FAST-LIO2  LIO-SAM</td></tr><tr><td rowspan=1 colspan=1>gate01</td><td rowspan=1 colspan=1>172</td><td rowspan=1 colspan=2>0.097</td><td rowspan=1 colspan=1>0.085</td><td rowspan=1 colspan=1>0.091</td><td rowspan=1 colspan=1>0.122</td></tr><tr><td rowspan=1 colspan=1>gate02</td><td rowspan=1 colspan=1>327</td><td rowspan=1 colspan=2>0.270</td><td rowspan=1 colspan=1>0.277</td><td rowspan=1 colspan=1>0.279</td><td rowspan=1 colspan=1>0.288</td></tr><tr><td rowspan=1 colspan=1>gate03</td><td rowspan=1 colspan=1>283</td><td rowspan=1 colspan=2>0.085</td><td rowspan=1 colspan=1>0.085</td><td rowspan=1 colspan=1>0.109</td><td rowspan=1 colspan=1>0.095</td></tr><tr><td rowspan=1 colspan=1>walk01</td><td rowspan=1 colspan=1>291</td><td rowspan=1 colspan=2>0.078</td><td rowspan=1 colspan=1>0.077</td><td rowspan=1 colspan=1>0.112</td><td rowspan=1 colspan=1>0.080</td></tr><tr><td rowspan=1 colspan=1>door01</td><td rowspan=1 colspan=1>461</td><td rowspan=1 colspan=2>0.251</td><td rowspan=1 colspan=1>0.249</td><td rowspan=1 colspan=1>0.271</td><td rowspan=1 colspan=1>0.269</td></tr><tr><td rowspan=1 colspan=1>door02</td><td rowspan=1 colspan=1>127</td><td rowspan=1 colspan=2>0.172</td><td rowspan=1 colspan=1>0.177</td><td rowspan=1 colspan=1>0.200</td><td rowspan=1 colspan=1>0.180</td></tr><tr><td rowspan=1 colspan=1>street01</td><td rowspan=1 colspan=1>1028</td><td rowspan=1 colspan=2>0.246</td><td rowspan=1 colspan=1>0.294</td><td rowspan=1 colspan=1>0.329</td><td rowspan=1 colspan=1>0.559</td></tr><tr><td rowspan=1 colspan=1>street02</td><td rowspan=1 colspan=1>1227</td><td rowspan=1 colspan=2>2.448</td><td rowspan=1 colspan=1>2.228</td><td rowspan=1 colspan=1>2.754</td><td rowspan=1 colspan=1>3.320</td></tr><tr><td rowspan=1 colspan=1>street03</td><td rowspan=1 colspan=1>354</td><td rowspan=1 colspan=2>0.097</td><td rowspan=1 colspan=1>0.103</td><td rowspan=1 colspan=1>0.106</td><td rowspan=1 colspan=1>0.102</td></tr><tr><td rowspan=1 colspan=1>street04</td><td rowspan=1 colspan=1>858</td><td rowspan=1 colspan=2>0.485</td><td rowspan=1 colspan=1>0.565</td><td rowspan=1 colspan=1>0.552</td><td rowspan=1 colspan=1>1.009</td></tr><tr><td rowspan=1 colspan=1>street05</td><td rowspan=1 colspan=1>469</td><td rowspan=1 colspan=2>0.331</td><td rowspan=1 colspan=1>0.287</td><td rowspan=1 colspan=1>0.377</td><td rowspan=1 colspan=1>0.407</td></tr><tr><td rowspan=2 colspan=1>street06street07</td><td rowspan=1 colspan=1>494</td><td rowspan=1 colspan=2>0.342</td><td rowspan=1 colspan=1>0.391</td><td rowspan=1 colspan=1>0.434</td><td rowspan=2 colspan=1>0.3321.614</td></tr><tr><td rowspan=1 colspan=1>829</td><td rowspan=1 colspan=2>2.916</td><td rowspan=1 colspan=1>3.646</td><td rowspan=1 colspan=1>3.512</td></tr><tr><td rowspan=1 colspan=1>street08</td><td rowspan=1 colspan=1>491</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>0.130</td><td rowspan=1 colspan=1>0.146</td><td rowspan=1 colspan=1>0.170</td><td rowspan=1 colspan=1>0.161</td></tr><tr><td rowspan=3 colspan=1>street09street10hall01</td><td rowspan=1 colspan=1>907</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>3.164</td><td rowspan=1 colspan=1>3.754</td><td rowspan=1 colspan=1>3.648</td><td rowspan=2 colspan=1>2.6578.560</td></tr><tr><td rowspan=1 colspan=1>810</td><td rowspan=1 colspan=2>0.388</td><td rowspan=1 colspan=1>0.977</td><td rowspan=1 colspan=1>0.956</td></tr><tr><td rowspan=1 colspan=1>351</td><td rowspan=1 colspan=2>0.256</td><td rowspan=1 colspan=1>0.256</td><td rowspan=1 colspan=1>0.258</td><td rowspan=1 colspan=1>0.281</td></tr><tr><td rowspan=1 colspan=1>hall02</td><td rowspan=1 colspan=1>128</td><td rowspan=1 colspan=2>0.274</td><td rowspan=1 colspan=1>0.272</td><td rowspan=2 colspan=1>0.2740.343</td><td rowspan=4 colspan=1>0.2850.5791.0761.015</td></tr><tr><td rowspan=3 colspan=1>hall03hall04hall05</td><td rowspan=1 colspan=1>164</td><td rowspan=1 colspan=2>0.345</td><td rowspan=1 colspan=1>0.359</td></tr><tr><td rowspan=1 colspan=1>181</td><td rowspan=1 colspan=2>0.944</td><td rowspan=1 colspan=1>0.944</td><td rowspan=2 colspan=1>0.9521.049</td></tr><tr><td rowspan=1 colspan=1>402</td><td rowspan=1 colspan=2>1.045</td><td rowspan=1 colspan=1>1.046</td></tr><tr><td rowspan=1 colspan=1>mean</td><td rowspan=1 colspan=1></td><td rowspan=1 colspan=2>0.684</td><td rowspan=1 colspan=1>0.772</td><td rowspan=1 colspan=1>0.799</td><td rowspan=1 colspan=1>1.095</td></tr></table>

The best and second-best results are bolded and underlined respectively

In outdoor sequences, i.e., gate, street, map points are relatively sparse compared to indoor scenes. This is especially evident in the street sequences, where the robot moves on wide campus roads at night. Fig. 4 shows the trajectories of sequence street10 for qualitative comparison. The competitive results of

TABLE III  
TRANSLATION RMSE (M) RESULTS OF POSE ESTIMATION COMPARISON ON THE NTU VIRAL DATASET
<table><tr><td>Seq</td><td>duration(s)</td><td>LOG-LIO</td><td>LOG-C</td><td>FAST-LIO2</td><td>LIO-SAM</td></tr><tr><td>eee_01</td><td>399</td><td>0.084</td><td>0.082</td><td>0.084</td><td>0.049</td></tr><tr><td>eee_02</td><td>321</td><td>0.072</td><td>0.072</td><td>0.073</td><td>0.051</td></tr><tr><td>eee_03</td><td>181</td><td>0.112</td><td>0.114</td><td>0.113</td><td>0.081</td></tr><tr><td>nya_01</td><td>396</td><td>0.081</td><td>0.080</td><td>0.075</td><td>0.174</td></tr><tr><td>nya_02</td><td>428</td><td>0.111</td><td>0.111</td><td>0.109</td><td>0.085</td></tr><tr><td>nya_03</td><td>411</td><td>0.120</td><td>0.121</td><td>0.121</td><td>0.249</td></tr><tr><td>sbs_01</td><td>354</td><td>0.094</td><td>0.095</td><td>0.097</td><td>X</td></tr><tr><td>sbs_02</td><td>373</td><td>0.085</td><td>0.086</td><td>0.080</td><td>0.080</td></tr><tr><td>sbs_03</td><td>389</td><td>0.085</td><td>0.083</td><td>0.083</td><td>X</td></tr><tr><td>rtp_01</td><td>482</td><td>0.191</td><td>0.230</td><td>0.209</td><td>X</td></tr><tr><td>rtp_02</td><td>453</td><td>0.147</td><td>0.153</td><td>0.163</td><td>0.117</td></tr><tr><td>rtp_03</td><td>355</td><td>0.180</td><td>0.181</td><td>0.170</td><td>0.113</td></tr><tr><td>tnp_01</td><td>583</td><td>0.093</td><td>0.095</td><td>0.094</td><td>X</td></tr><tr><td>tnp_02</td><td>457</td><td>0.075</td><td>0.058</td><td>0.071</td><td>X</td></tr><tr><td>tnp_03</td><td>407</td><td>0.084</td><td>0.079</td><td>0.080</td><td>X</td></tr><tr><td>spms_01</td><td>446</td><td>1.450</td><td>1.670</td><td>1.818</td><td>X</td></tr><tr><td>spms_02</td><td>398</td><td>2.196</td><td>2.748</td><td>3.378</td><td>X</td></tr><tr><td>spms_03</td><td>386</td><td>0.685</td><td>0.766</td><td>0.793</td><td>X</td></tr><tr><td>mean</td><td></td><td>0.330</td><td>0.379</td><td>0.423</td><td>X</td></tr></table>

The best and second-best results are bolded and underlined respectively

LOG-LIO suggest that efficient and accurate estimation of local geometric information exhibits great potential in reducing the error of LIO system.

Overall, when employing only point-to-plane association, LOG-C demonstrates a slight improvement in average accuracy when compared to FAST-LIO2. However, with the implementation of our proposed hierarchical data association and map management scheme, LOG-LIO consistently achieves the lowest mean error and gets the best results in 10 out of the 21 sequences.

2) NTU VIRAL Datasets: As depicted in Table III, LOG-LIO demonstrates the best performance in most sequences, closely followed by LOG-C. LIO-SAM achieves the best results on several sequences but fails in half of the dataset.

TABLE IV  
AVERAGE TIME CONSUMPTION (MS) OF EACH SEQUENCE IN THE EXPERIMENTS
<table><tr><td rowspan="2"></td><td colspan="5">M2DGR</td><td colspan="7">NTU VIRAL</td></tr><tr><td>gate</td><td>walk</td><td>door</td><td>street</td><td>hall</td><td>eee</td><td>nya</td><td>sbs</td><td>rtp</td><td>tnp</td><td>spms</td><td>mean</td></tr><tr><td>LOG-LIO</td><td>46.563</td><td>45.964</td><td>24.083</td><td>42.480</td><td>24.903</td><td>20.991</td><td>18.027</td><td>17.997</td><td>25.388</td><td>19.412</td><td>23.348</td><td>28.101</td></tr><tr><td>FAST-LIO2</td><td>31.378</td><td>32.276</td><td>14.754</td><td>28.970</td><td>15.509</td><td>15.705</td><td>12.476</td><td>12.785</td><td>21.006</td><td>13.340</td><td>17.106</td><td>20.523</td></tr></table>

The minimum average time consumption is marked in bold.

In the sequences nya and tnp, where the drone traverses indoors, LOG-LIO, LOG-C and FAST-LIO2 exhibit similar errors. This behavior is consistent with what we observed in M2DGR, where the presence of numerous planes in confined spaces can effectively constrain the point-to-plane association. In the eee, sbs, and rtp outdoor scenes amidst buildings, LOG-LIO, LOG-C, and FAST-LIO2 yield highly similar trajectories due to effective constraints imposed by the plane structure. However, in the spms sequences, where the drone departs from an area surrounded by buildings and ascends to higher altitudes, the sparse LiDAR points lead to limited map overlap. This can potentially result in registration errors when performing pointto-plane data association. LOG-LIO addresses this challenge by performing point association with corresponding voxels. The accurate local geometric information within these voxels help mitigate registration errors, ultimately resulting in more precise trajectories.

It is worth noting that in practice, trajectory error in the LIO system should consider various factors such as map resolution, IMU noise, and etc. And we focus on factors closely tied to our contributions while keeping the other parameters fixed in this letter.

3) Processing Time Evaluation: We perform statistical analysis on the time consumption of LOG-LIO and FAST-LIO2 in each sequence, as shown in Table IV. It is observed that the average processing time per scan of LOG-LIO is slightly longer than that of FAST-LIO2, which is mainly due to the Ring FALS normal estimation and incremental point distribution maintenance within map voxels. Despite LOG-LIO exhibiting an additional average time consumption of 8 ms than FAST-LIO2, it still meets real-time requirements. This performance difference should be considered in light of the number of points and the complexity of the environment.

## VII. CONCLUSION AND FUTURE WORK

This letter introduces LOG-LIO, an online LiDAR-inertial odometry method that estimates normal and distribution of points for local geometric information in real time. To improve normal estimation efficiency for LiDAR scans, we introduce Ring FALS, an efficient normal estimator that pre-records structural information and uses range data for normal estimation. In LOG-LIO, we manage the map using an extended ikd-tree, incrementally maintaining normal and point distribution within map voxels. We employ a hierarchical data association scheme for accurate constraints, resulting in precise pose estimation. Experimental results show that LOG-LIO is competitive with state-of-the-art LIO systems in various environments.

For future research, we intend to incorporate dynamic noise removal and loop closure to enhance stability in dynamic environments and ensure long-term operation.

## REFERENCES

[1] J. Zhang and S. Singh, “LOAM: Lidar odometry and mapping in real-time,” in Proc. Robot.: Sci. Syst., 2014, vol. 2, pp. 1–9.

[2] K. Chen, B. T. Lopez, A.-A. Agha-mohammadi, and A. Mehta, “Direct LiDAR odometry: Fast localization with dense point clouds,” IEEE Robot. Automat. Lett., vol. 7, no. 2, pp. 2000–2007, Apr. 2022.

[3] T.-M. Nguyen, D. Duberg, P. Jensfelt, S. Yuan, and L. Xie, “SLICT: Multiinput multi-scale surfel-based LiDAR-inertial continuous-time odometry and mapping,” IEEE Robot. Automat. Lett., vol. 8, no. 4, pp. 2102–2109, Apr. 2023.

[4] A. Reinke et al., “LOCUS 2.0: Robust and computationally efficient LiDAR odometry for real-time 3D mapping,” IEEE Robot. Automat. Lett., vol. 7, no. 4, pp. 9043–9050, Oct. 2022.

[5] M. Palieri et al., “LOCUS: A multi-sensor LiDAR-centric solution for high-precision odometry and 3D mapping in real-time,” IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 421–428, Apr. 2021.

[6] M. Ramezani et al., “Wildcat: Online continuous-time 3D LiDAR-inertial SLAM,” 2022, arXiv:2205.12595.

[7] H. Badino, D. Huber, Y. Park, and T. Kanade, “Fast and accurate computation of surface normals from range images,” in Proc. IEEE Int. Conf. Robot. Automat., 2011, pp. 3084–3091.

[8] R. Fan et al., “Three-filters-to-normal: An accurate and ultrafast surface normal estimator,” IEEE Robot. Automat. Lett., vol. 6, no. 3, pp. 5405–5412, 2021.

[9] W. Xu and F. Zhang, “FAST-LIO: A fast, robust LiDAR-inertial odometry package by tightly-coupled iterated Kalman filter,” IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 3317–3324, Jul. 2021.

[10] W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, “FAST-LIO2: Fast direct LiDAR-inertial odometry,” IEEE Trans. Robot., vol. 38, no. 4, pp. 2053–2073, Aug. 2022.

[11] R. B. Rusu and S. Cousins, “3D is here: Point cloud library (PCL),” in Proc. IEEE Int. Conf. Robot. Automat., 2011, pp. 1–4.

[12] A. Segal, D. Haehnel, and S. Thrun, “Generalized-ICP,” in Proc. Robot.: Sci. Syst., 2009, vol. 2, p. 435.

[13] M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. J. Leonard, and F. Dellaert, “iSAM2: Incremental smoothing and mapping using the Bayes tree,” Int. J. Robot. Res., vol. 31, no. 2, pp. 216–235, 2012.

[14] C. Yuan, X. Liu, X. Hong, and F. Zhang, “Pixel-level extrinsic self calibration of high resolution LiDAR and camera in targetless environments,” IEEE Robot. Automat. Lett., vol. 6, no. 4, pp. 7517–7524, Oct. 2021.

[15] C. Yuan, W. Xu, X. Liu, X. Hong, and F. Zhang, “Efficient and probabilistic adaptive voxel mapping for accurate online LiDAR odometry,” IEEE Robot. Automat. Lett., vol. 7, no. 3, pp. 8518–8525, Jul. 2022.

[16] S. Thrun, “Probabilistic robotics,” Commun. ACM, vol. 45, no. 3, pp. 52–57, 2002.

[17] L. Zhou, D. Koppel, and M. Kaess, “LiDAR SLAM with plane adjustment for indoor environment,” IEEE Robot. Automat. Lett., vol. 6, no. 4, pp. 7073–7080, Oct. 2021.

[18] J. Yin, A. Li, T. Li, W. Yu, and D. Zou, “M2DGR: A multi-sensor and multi-scenario SLAM dataset for ground robots,” IEEE Robot. Automat. Lett., vol. 7, no. 2, pp. 2266–2273, Apr. 2022.

[19] T.-M. Nguyen, S. Yuan, M. Cao, Y. Lyu, T. H. Nguyen, and L. Xie, “NTU viral: A visual-inertial-ranging-LiDAR dataset, from an aerial vehicle viewpoint,” Int. J. Robot. Res., vol. 41, no. 3, pp. 270–280, 2022.

[20] T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus, “LIO-SAM: Tightly-coupled LiDAR inertial odometry via smoothing and mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5135–5142.