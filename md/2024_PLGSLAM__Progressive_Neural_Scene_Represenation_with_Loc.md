# PLGSLAM: Progressive Neural Scene Represenation with Local to Global Bundle Adjustment

Tianchen Deng<sup>1</sup>, Guole Shen<sup>1</sup>, Tong Qin<sup>1</sup>, Jianyu Wang<sup>1</sup>, Wentao Zhao<sup>1</sup>, Jingchuan Wang<sup>1</sup>, Danwei Wang<sup>2</sup>, Weidong Chen<sup>1</sup> \* <sup>1</sup> Shanghai Jiao Tong University <sup>2</sup> Nanyang Technological University

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/3af3d58dd0e9b97fb4cac02c4e6ac240b6c1949d04e21b0e32518ee4f4d966c4.jpg)  
Figure 1. Large-scale indoor scene 3D Reconstruction with different methods. We depict the final mesh and camera tracking trajectory error (Absolute Trajectory Error) of different methods. The color bar on the right shows the relative scaling of color. PLGSLAM outperforms others in both scene reconstruction and pose estimation.

## Abstract

Neural implicit scene representations have recently shown encouraging results in dense visual SLAM. However, existing methods produce low-quality scene reconstruction and low-accuracy localization performance when scaling up to large indoor scenes and long sequences. These limitations are mainly due to their single, global radiance field with finite capacity, which does not adapt to large scenarios. Their end-to-end pose networks are also not robust enough with the growth of cumulative errors in large scenes. To this end, we introduce PLGSLAM, a neural visual SLAM system capable of high-fidelity surface reconstruction and robust camera tracking in real-time. To handle large-scale indoor scenes, PLGSLAM proposes a progressive scene representation method which dynamically allocates new local scene representation trained with frames within a local sliding window. This allows us to scale up to larger indoor scenes and improves robustness (even under pose drifts). In local scene representation, PLGSLAM utilizes tri-planesfor local high-frequencyfeatures with multilayerperceptron (MLP) networksfor the low-frequencyfeature, achieving smoothness and scene completion in unobserved areas. Moreover, we propose local-to-global bundle adjustment method with a global keyframe database to address the increased pose drifts on long sequences. Experi-

mental results demonstrate that PLGSLAM achieves stateof-the-art scene reconstruction results and tracking performance across various datasets and scenarios (both in small and large-scale indoor environments).

## 1. Introduction

Visual Simultaneous Localization and Mapping (SLAM) has been a fundamental computer vision problem with wide applications such as autonomous driving, remote sensing [6], and virtual/augmented reality. Many traditional methods have been introduced in the past years, such as ORB-SLAM [19, 20], VINS [24], and so on. They can estimate the camera pose and construct sparse point cloud maps in real-time with accurate localization performance. However, the sparse point cloud maps cannot meet the further perception needs of the robot. Recent attention has turned to learning-based methods for dense scene reconstruction. Kinectfusion [11], BAD-SLAM[25] reconstruct meaningful global 3D maps and show reasonable but limited reconstruction accuracy with deep learning networks.

Nowadays, with the proposal of Neural Radiance Fields (NeRF), there are many following works on different areas [8]. iMAP [27] is the first work to use a single multilayer perceptron (MLP) to represent the entire scene in SLAM system. NICE-SLAM [41] improves the scene representation method with feature grids. ESLAM [12] and Co-SLAM [31] further improve the scene representation methods. ESLAM uses tri-planes for better real-time performance and reconstruction accuracy. Co-SLAM uses joint coordinate and sparse parametric scene for accurate scene representation. They can achieve promising reconstruction quality in a small indoor room.

Although ESLAM and Co-SLAM perform well in smaller indoor scenes, they face challenges in representing large-scale indoor scenes (e.g., multi-room apartments). We outline the key challenges for real-time incremental NeRF-SLAM: a) insufficient scene representation capability: Existing methods employ a fixed-capacity, global model, limiting scalability to larger scenes and longer video sequences. b) accumulation of errors and pose drift: Existing works struggle with accuracy and robustness in largescale indoor scenes due to accumulating errors.

To this end, we design our neural SLAM system for accurate scene reconstruction and robust pose estimation in large indoor scenes and long sequences. We propose a progressive scene representation method which dynamically initialize new scene representation when the camera moves to the bound of the local scene representation. The entire scene is divided into multiple local scene presentations, which can significantly improve the scene representation capacity of large indoor scenes. The robustness of our system is also increased because the mis-estimation is locally bounded.

In local scene representation, We propose a parametriccoordinate joint encoding method for accuracy, speed, and completion of unseen region. Parametric encoding is the triplane encoding, and the coordinate encoding is the one-blob encoding with MLP. We use tri-planes to encode the local high-frequency feature of the scene and use MLP to represent global low-frequency features with the coherence priors inherent. We bring together the benefits of both methods for accuracy, smoothness, and hole-filling in areas without observation.

Furthermore, we combine the traditional SLAM systems with end-to-end pose networks to improve pose estimation performance. We propose a local-to-global bundle adjustment (BA) method to eliminate the cumulative error which becomes significantly evident in large indoor scenes and long video sequences. So far, all neural SLAM systems only use end-to-end network and perform BA with rays sampled from a local subset of selected keyframes, resulting in inaccurate, non-robust pose estimation and significant cumulative errors in camera tracking. PLGSLAM maintains a global keyframe database and performs local-to-global neural warpping and reprojection Bundle Adjustment. The proposed Local-to-global BA method can eliminate the cumulative error with all the historical observations. In practice, PLGSLAM achieves SOTA performance in camera tracking and 3D reconstruction while maintaining real-time performance. Overall, our contributions are shown as follows:

• A progressive scene representation method is proposed which dynamically initiate local scene representation trained with frames within a local window. This enables scalability to extensive indoor scenes and long videos sequences, substantially improving robustness.

• In local scene representation, We design a joint parametric-coordinate encoding method. We combine the tri-planes with the one-blob encoding encoding method for accurate and smooth surface reconstruction. It can not only enhance the ability of scene representation, but also substantially reduce the memory growth from cubic to square.

• We integrate the traditional SLAM system with an endto-end pose estimation network. A local-to-global bundle adjustment algorithm is proposed, which can mitigate cumulative error in large-scale indoor scenes. Our system maintain a global keyframe database with the system operation, enabling bundle adjustment across all past observations, from local to global.

## 2. Related Work

Dense Visual SLAM. SLAM [15] and localization [16, 23, 37, 38] has been an active field for the past two decades. Traditional visual SLAM algorithms [9, 19, 24, 34, 35] estimate accurate camera poses and use sparse point clouds as the map representation. They use manipulated key points for tracking, mapping, relocalization, and loop closing. Dense visual SLAM approaches focus on reconstructing a dense map of a scene. DTAM [21] is one of the pioneer works that use the dense map and view-centric scene representation. KinectFusion [11] performs camera tracking via projective iterative-closest-point (ICP) and explicitly represents the surface of the environment via TSDF-Fusion. Some works [5, 25, 29] propose bundle adjustment(BA) method to optimize keyframe poses and construct the dense 3D structure jointly. In contrast to previous SLAM approaches, we adopt implicit scene representation of the geometry and directly optimize them during mapping.

Implicit Scene Representation. With the proposal of Neural radiance fields (NeRF) [18], many researchers explore taking the advantages of the implicit method into 3D reconstruction. NeRF is a ground-breaking method for novel view synthesis using differentiable rendering. However, the representation of volume densities can not commit the geometric consistency. In order to deal with it, UNISURF [22] and NeuS [32] are proposed, combining neural radiance fields with Signed Distance Field (SDF) values. Other methods [1–3, 28] use various scene geometry representation methods, such as truncated signed distance function, voxel grid. For large-scale representation, Mega-NeRF and LocalRF [17, 30] use multiple local scene representations for the entire scene.

NeRF-based SLAM. iMAP [27] and NICE-SLAM [41] are successively proposed to combine neural implicit mapping with SLAM. iMAP uses a single multi-layer perceptron (MLP) to represent the scene, and NICE-SLAM uses a learnable hierarchical feature grid. [13, 14, 40] use semantic feature embedding to improve scene representation. Some methods [7, 39] also use 3D gaussian to improve the scene representation. Co-SLAM [31] and ESLAM [12] are the most relative work of our method. However, all of them have difficult in large-scale indoor environments and long sequences. With the proposed progressive scene representation method, we can successfully scale up to larger indoor scenarios. The fusion of tri-planes and one-blob encoding leads to high-fidelity and smooth surface reconstruction in local scene. A local-to-global bundle adjustment method is also proposed. This method can effectively eliminate growing cumulative errors in existing methods in large indoor scenes.

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/59d7c9eec6aaa0eff1064f5fd62ffe375d7f40866588acb8a943c9372e60edff.jpg)  
Figure 2. The isometric view of the proposed PLGSLAM system. Our system has two parallel threads: the mapping thread and the tracking thread. In the mapping thread, we propose the progressive scene representation method for the entire scene. In local scene representation, we combine the tri-planes with the multi-layer perceptron to improve the accuracy and smoothness. Both of them are online updated by minimizing our carefully designed loss through differentiable rendering with the system operating. As for the tracking thread, we propose a local-to-global bundle adjustment for accurate and robust pose estimation. Those two threads are running with an alternating optimization.

## 3. Method

The pipeline of our system is shown in Fig. 2. We use a set of sequential RGB-D frames $\{ I _ { i } , D _ { i } \} _ { i = 1 } ^ { M }$ with known camera intrinsic $K \in R _ { 3 \times 3 }$ as our input. Our model predicts camera poses $\{ R _ { i } | t _ { i } \} _ { i = 1 } ^ { M }$ , color c, and an implicit truncated signed distance function (TSDF) representation $\phi _ { g }$ that can be used in marching cubes algorithm to extract 3D meshes. For the implicit mapping thread, a progressive scene representation method (Sec. 3.1) is designed to represent large-scale indoor environments. Then, in the local radiance fields, we improve the scene representation methods and combine the tri-planes with multi-layer perceptron (MLP) by our designed architecture. Sec. 3.2 walks through the rendering process, which converts raw representations into pixel depths, colors, and SDF values. For the camera tracking thread, a local-to-global bundle adjustment method (Sec. 3.3) is designed for robust and accurate pose estimation. Several carefully designed loss functions are proposed to jointly optimize the scene implicit representation and camera pose estimation. The network is incrementally updated with the system operation.

## 3.1. Progressive Scene Representation

All the existing NeRF-based SLAM systems have difficulties in large-scale indoor scenes. They use a single, global representation of the entire environment, which limits their scene representation capacity. There are two key limitations when modeling large-scale indoor scenes: a) the incapacity of a single, fixed-capacity model to represent videos of arbitrary length. b) the single scene representation tends to overfit to the early data in the sequence, leading to poorer performance in learning from the later data. c) any misestimation (e.g. outlier pose) has a global impact and might cause thefalse reconstruction.

Mega-NeRF and Bungee-NeRF [30, 33] pre-partition the space for radiance fields. However, this approach is not applicable in our setting, as the camera poses in our system are concurrently optimized alongside the mapping thread.

In our method, we dynamically create local scene representation. Whenever the estimated camera pose trajectory leaves the space of the current scene representation, we dynamically allocate new local scene representation trained with a small set of frames. and from there, we progressively introduce subsequent local frames to the optimization. So, the entire scene can be represented as multiple local scene representations:

$$
\{ I _ { i } , D _ { i } \} _ { i = 1 } ^ { M } \mapsto \{ \mathrm { S R } _ { \theta _ { 1 } } ^ { 1 } , \mathrm { S R } _ { \theta _ { 2 } } ^ { 2 } , \dots , \mathrm { S R } _ { \theta _ { n } } ^ { n } \} \mapsto \{ \mathbf { c } , \sigma \}\tag{1}
$$

where $S R _ { \theta _ { n } } ^ { n }$ denotes the local scene representation, σ denotes the volume density. Each local scene representation is centered at the position of the last estimated camera pose. We train each scene representation with a local subset frames. Each subset contains some overlap frames, which is important for achieving consistent reconstructions in the local scene representation. Whenever the estimated camera pose leaves the bound of the current scene representation, we stop optimizing previous ones (freeze the network parameters). At this point, we can reduce memory requirements by removing unnecessary supervisory frames. We also stop updating the mapping parameters in the tracking thread to reduce errors. If the estimated camera pose is outside the current bounds, but within a previous local scene representation, we activate the previous one and proceed with the optimization process. We further increase the global consistency by inverse distance weight (IDW) fusion for all overlapping scene representations at any supervising frame.

Local Scene Representation. Voxel grid-based architectures [10, 31, 41] are the mainstream in NeRF-based SLAM system. However, they struggle with cubical memory growing and real-time performance. Inspired by [12], we design a parametric-coordinate joint encoding method. Parametric encoding is tri-plane encoding, and the coordinate encoding is the one-blob encoding with MLP. We store and optimize high-frequency features(e.g. texture) on perpendicular axisaligned planes. The one-blob encoding with MLPs are used to encode and store low-frequency features for the coherence and smoothness priors. This joint scene representation architecture achieve high-fidelity and smoothness scene reconstruction with the ability of hole filling.

Specifically, the tri-planes are at two scales, i.e., coarse and fine. The tri-planes feature $\pmb { T } ( \boldsymbol { x } )$ can be formulated as:

$$
\begin{array} { l } { { t ^ { c } ( x ) = T _ { x y } ^ { c } ( x ) + T _ { x z } ^ { c } ( x ) + T _ { y z } ^ { c } ( x ) } } \\ { { \qquad t ^ { f } ( x ) = T _ { x y } ^ { f } ( x ) + T _ { x z } ^ { f } ( x ) + T _ { y z } ^ { f } ( x ) } } \\ { { \qquad T ( x ) = C o n c a t \left( t ^ { c } ( x ) ; t ^ { f } ( x ) \right) } } \end{array}\tag{2}
$$

where $t ^ { c } ( x ) , t ^ { f } ( x )$ denote the coarse and fine feature form tri-planes. x is the world coordinate. $\{ T _ { x y } ^ { c } , T _ { x z } ^ { c } , T _ { y z } ^ { c } \}$ represent the three coarse geometry feature planes, and $\{ \bar { T } _ { x y } ^ { f } , T _ { x z } ^ { f } , T _ { y z } ^ { f } \}$ represent the three fine geometry feature planes.

For a sample point $x ,$ we use bilinearly interpolating the nearest neighbors on each feature plane. Then, we sum the interpolated coarse features and the fine, respectively, into the coarse output and fine output. At last, we concatenate the outputs together as the tri-plane features. The geometry decoder outputs the predicted SDF value $\phi _ { g } ( x )$ and a feature vector z:

$$
f _ { g } ( \gamma ( x ) , \pmb { T } ( x ) )  ( \mathbf { z } , \phi _ { \pmb { g } } ( x ) )\tag{3}
$$

where $\gamma ( \mathbf { x } )$ represents coordinate position encoding. z is the latent code. We use one-blob encoding [31] instead

of embedding spatial coordinates into multiple frequency bands. Finally, the color decoder predicts the RGB value:

$$
f _ { c } ( \gamma ( x ) , \mathbf { z } , \pmb { a } ( x ) ) \mapsto \phi _ { \pmb { a } } ( x )\tag{4}
$$

$\phi _ { a } ( x )$ represents the color of the sample points. $\pmb { a } ( x )$ is the is the appearence feature from tri-planes. Combining the MLP with the tri-planes scene representation, our architecture achieve accurate and smooth surface reconstruction, efficient memory use, and hole filling performance.

## 3.2. Differentiable Rendering

Inspired by the recent success of volume rendering in NeRF [18], we also propose to use a differentiable rendering process to integrate the predicted density and colors from our scene representation. We determine a ray $r ( t ) = \mathbf { o } + t \mathbf { d }$ whose origin is at the camera center of projection $^ { O , }$ ray direction r. We uniformly sample K points. The sample bound is within the near and far planes $t _ { k } \in [ t _ { n } , t _ { f } ]$ $k \in \{ 1 , \ldots , K \}$ with depth values $\{ \mathbf { d _ { 1 } } , \dotsc , \mathbf { d _ { K } } \}$ and predicted colors $\{ \mathbf { c _ { 1 } } , . . . , \mathbf { c _ { K } } \}$ . For all sample points along rays, we query TSDF $\phi _ { g } ( p _ { k } )$ and raw color $\phi _ { a } ( p _ { k } )$ from our networks and use the SDF-Based rendering approach to convert SDF values to volume densities:

$$
{ \pmb { \sigma } } \left( x _ { k } \right) = \frac { 1 } { \beta } \cdot \operatorname { S i g m o i d } \left( \frac { - \phi _ { g } \left( x _ { k } \right) } { \beta } \right)\tag{5}
$$

where $\beta \ \in \ \mathbb { R }$ is a learnable parameter that controls the sharpness of the surface boundary. Then we define the termination probability $w _ { k } ,$ , depth $\hat { \boldsymbol { d } } ,$ and color cˆ as:

$$
\begin{array} { c } { { w _ { k } = \displaystyle \exp \left( - \sum _ { m = 1 } ^ { n - 1 } \sigma \left( x _ { m } \right) \right) \left( 1 - \exp \left( - \sigma \left( x _ { k } \right) \right) \right) } } \\ { { \hat { c } = \displaystyle \sum _ { k = 1 } ^ { N } w _ { k } \phi _ { a } \left( x _ { k } \right) \quad \mathrm { ~ a n d ~ } \quad \hat { d } = \displaystyle \sum _ { k = 1 } ^ { N } w _ { k } t _ { k } } } \end{array}\tag{6}
$$

## 3.3. Local-to-global Bundle Adjustment

Currently, existing NeRF-based SLAM methods exhibit poor accuracy in large-scale indoor scene localization. Their tracking networks are performed via minimizing rgb loss functions with respect to learnable parameters θ to estimate the relative pose matrix $\{ R _ { i } | t _ { i } \} \in \mathbb { S E } ( 3 )$ . With the growing cumulative error ε of pose estimation, those methods result in failure in large-scale indoor scenes and long videos. To this end, we design a local-to-global bundle adjustment method to solve this problem, which performs well with our progressive scene representation. We design our method by drawing inspiration from traditional keyframebased SLAM systems for improving the robustness and accuracy of pose estimation. We propose neural warpping error and reprojection error for local-to-global bundle adjustment. The neural warpping loss is formulated as:

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/4c77096e20dbdcd06388bc56506173dc8d7797632f520b68c5da5e3d2ed54c72.jpg)  
Figure 3. This figure illustrates the designed neural warping loss. We calculate the neural warpping loss between keyframe I and keyframe $I ^ { \prime } .$

$$
\begin{array} { l } { { \mathcal { L } _ { n w c } = \displaystyle \sum _ { i } ^ { N } ( \mathcal { F } ( o _ { i } , d _ { i } , \{ R _ { i  i ^ { \prime } } , t _ { i  i ^ { \prime } } \} ) - C _ { i ^ { \prime } } ) } } \\ { { \ } } \\ { { \mathcal { L } _ { n w d } = \displaystyle \sum _ { i } ^ { N } ( \mathcal { F } ( o _ { i } , d _ { i } , \{ R _ { i  i ^ { \prime } } , t _ { i  i ^ { \prime } } \} ) - D _ { i ^ { \prime } } ) } } \end{array}\tag{7}
$$

Here, $L _ { n w c }$ and $L _ { n w d }$ are the neural warpping color and depth loss. $o _ { i } , d _ { i }$ denotes the rays from image $I _ { i } .$ $\{ R _ { i  i ^ { \prime } } , t _ { i  i ^ { \prime } } \}$ denotes the relative pose from image $I _ { i }$ to $I _ { i ^ { \prime } } . \quad F ( )$ denotes our scene representation network. We present the illustration of neural warpping loss in Fig. 3. We formulate reprojection errors with SIFT features:

$$
\mathcal { L } _ { r e } = \sum _ { i = 1 } ^ { n } \Vert ( u _ { i ^ { \prime } } , v _ { i ^ { \prime } } ) - \Pi ( R _ { i  i ^ { \prime } } P _ { i } + t _ { i  i ^ { \prime } } ) \Vert\tag{8}
$$

where $\Pi ( R _ { i  i ^ { \prime } } P _ { i } + t _ { i  i ^ { \prime } } )$ represents the reprojection of 3D point $P _ { i }$ to the corresponding pixel $( u _ { i ^ { \prime } } , v _ { i ^ { \prime } } )$ in image $i ^ { \prime } .$

Whenever a keyframe arrives, we perform local bundle adjustment in our tracking and mapping thread. A keyframe is selected for every K frames. When the camera moves to the bound of the current scene representation, we also initialize it as the keyframe. In local bundle adjustment, we only select keyframes from the local keyframe database that visually overlap with the current frame when optimizing the scene geometry to ensure the geometry outside the current view remains static and fast convergence. Meanwhile, we also maintain a global keyframe list with the operation of our system. After accumulating a specific number of local keyframes or the camera moves to the local bound, a global bundle adjustment is performed. In global BA, we randomly select keyframes and rays from the global keyframe database, which leverages all historical observations of the scene. This approach effectively integrates local and global information which greatly improves the accuracy of camera pose optimization in large-scale indoor scenes.

## 3.4. Objective Functions

Our mapping and tracking thread are performed via minimizing our objective functions with respect to network parameters θ and camera parameters $\{ R _ { i } | t _ { i } \}$ . The color and depth rendering losses are used in our mapping and tracking

thread:

$$
\mathcal { L } _ { c } = \frac { 1 } { N } \sum _ { i = 1 } ^ { N } \left( \hat { \mathbf { c } } _ { i } - \mathbf { C _ { i } } \right) ^ { 2 } , \quad \mathcal { L } _ { d } = \frac { 1 } { \lvert R _ { i } \rvert } \sum _ { i \in R _ { i } } \left( \hat { \mathbf { d _ { i } } } - \mathbf { D _ { i } } \right) ^ { 2 }\tag{9}
$$

where $R _ { i }$ is the set of rays that have a valid depth observation. In addition, we design SDF loss, free space loss, and feature smoothness losses for our mapping thread. Specifically, for samples within the truncation region, we leverage the depth sensor measurement to approximate the signed distance field:

$$
\mathcal { L } _ { s d f } = \frac { 1 } { | R _ { i } | } \sum _ { r \in R _ { i } } \frac { 1 } { | X _ { r } ^ { t r } | } \sum _ { x \in X _ { r } ^ { t r } } \left( \phi _ { g } ( x ) \cdot T - ( \mathbf { D _ { i } } - \mathbf { d } ) \right) ^ { 2 }\tag{10}
$$

where $X _ { r } ^ { t r }$ is a set of points on the ray r that lie in the truncation region, $| \mathbf { D _ { i } } - \mathbf { d } | \leq t r$ . We differentiate the weights of points that are closer to the surface $X _ { r } ^ { t m } \ = \ \{ x | x \in$ $| \mathbf { D _ { i } } - \mathbf { d } | \leq 0 . 4 t r \}$ from those that are at the tail of the truncation region $X _ { r } ^ { t t }$ in our SDF loss.

$$
\begin{array} { r } { \mathcal { L } _ { s d f _ { m } } = \mathcal { L } _ { s d f } \left( X _ { r } ^ { t m } \right) , \quad \mathcal { L } _ { s d f _ { t } } = \mathcal { L } _ { s d f } \left( X _ { r } ^ { t t } \right) } \end{array}\tag{11}
$$

For sample points that are far from the surface $| D _ { i } - d | \geq T \colon$

$$
\mathcal { L } _ { f s } = \frac { 1 } { | R _ { i } | } \sum _ { r \in R _ { i } } \frac { 1 } { \Big | X _ { r } ^ { f s } \Big | } \sum _ { x \in X _ { r } ^ { f s } } \big ( \phi _ { g } ( x ) - 1 \big ) ^ { 2 }\tag{12}
$$

This loss can force the SDF prediction value to be the truncated distance tr. In addition, we propose feature smoothness losses to prevent the noisy reconstructions caused by tri-planes in unobserved free-space regions:

$$
\mathcal { L } _ { \mathrm { s m o o t h } } = \frac { 1 } { | \mathcal { M } | } \sum _ { { \bf x } \in \mathcal { M } } \Delta _ { x y } ^ { 2 } + \Delta _ { x z } ^ { 2 } + \Delta _ { y z } ^ { 2 }\tag{13}
$$

where $\Delta _ { x y } = \mathbf { T } \left( \mathbf { x } + \epsilon _ { x , y } \right) - \mathbf { T } ( \mathbf { x } ) , \Delta _ { x z } = \mathbf { T } \left( \mathbf { x } + \epsilon _ { x , z } \right) -$ $\mathbf { T } ( \mathbf { x } ) , \boldsymbol { \Delta } _ { y z } = \mathbf { T } \left( \mathbf { x } + \boldsymbol { \epsilon } _ { y , z } \right) - \mathbf { T } ( \mathbf { x } )$ denotes the featuremetric difference between adjacent sampled vertices on the three feature planes. M denotes a small random region form tri-planes. This loss can enhance the smoothness of our surface reconstruction results and it is only used in mapping thread.

## 4. Experiments

We validate that our method outperforms existing implicit representation-based methods in surface reconstruction, pose estimation, and real-time performance.

## 4.1. Datasets and Metrics

Datasets. We evaluate PLGSLAM on a variety of scenes from different datasets. We quantitatively evaluate the reconstruction quality on 8 small room scenes from Replica [26] (nearly $6 . 5 m \times 4 . 2 m \times 2 . 7 m$ with 2000 images). We evaluate on real-world scenes from ScanNet [4] for long sequences (more than 5000 images) and large-scale indoor

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/7f369c9b82b492019820c5960e08e3a65a136e8aa91538cea1ffa1a47971a6b6.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/46f13bd16b6db8d1860ebd45aa290fa5a5533a3d018da57c29d935003fa45596.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/bf72e46cb7e8e0c227175850002dff2a1dd6e1af52c047c16d1062611d31093d.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/542c6a347c3a8880a79fcda309f079f4f6240ccdb68ec1dc8767a4beb69b632e.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/99923fd353646a1e04a1d3c9e2f00dfeb5622c72c915fa118748e9a2b2096949.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/22ce6002435e2d4c01506a6c17974df6bfe0048ac53ec922dced558cbddff053.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/dd24dccfce2598075a082be7034d43a3073a19718765f89f0c148866761c0782.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/e1c17b4d3246a17d7cc6466c1e29e614ac07c524aa0d2f430d184e2c034c4e90.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/050e58077fce5a4dd1847a6eae022da3c948d48205ea31c08c0578d73a451709.jpg)  
Co-SLAM

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/fdedac6b8313fa65db662cea9b4f2c06c9cfdb76a15bfc89da4451be77cb29be.jpg)  
ESLAM

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/dc150299ea304dac3dd13ee3179506ca946d0d8692706d3a06859bbeeef53a1a.jpg)  
Ours

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/190ffc8e01be337caa0544325f903c4861f46fdd46ef7f1c7ebf53a0c85a1e52.jpg)  
GT

Figure 4. Reconstruction results (without cull) on Replica [26] apartment dataset. In comparison to our baselines, our methods achieve accurate and high-quality scene reconstruction and completion on various scenes.The region outlined on the image is marked in red to signify lower predictive accuracy, in green to signify higher accuracy, and in yellow to represent the ground truth results. The number in the bottom right corner of the image represents the completion ratio metric.
<table><tr><td rowspan="2">Methods</td><td colspan="4">Reconstruction</td><td colspan="2">Localization</td></tr><tr><td>Depth L1[cm] ↓ Acc.[cm] ↓ Comp.[cm] ↓ Comp.Ratio(%) ↑</td><td></td><td></td><td></td><td>ATE Mean[cm] ↓ ATE RMSE[cm] ↓</td><td></td></tr><tr><td>iMAP [27]</td><td>4.645</td><td>3.624</td><td>4.934</td><td>80.515</td><td>3.118</td><td>4.153</td></tr><tr><td>NICE-SLAM [41]</td><td>1.903</td><td>2.373</td><td>2.645</td><td>91.137</td><td>1.795</td><td>2.503</td></tr><tr><td>Vox-Fusion [36]</td><td>2.913</td><td>1.882</td><td>2.563</td><td>90.936</td><td>1.067</td><td>1.453</td></tr><tr><td>ESLAM [12]</td><td>0.945</td><td>2.082</td><td>1.754</td><td>96.427</td><td>0.565</td><td>0.707</td></tr><tr><td>Co-SLAM [31]</td><td>1.513</td><td>2.104</td><td>2.082</td><td>93.435</td><td>0.935</td><td>1.059</td></tr><tr><td>Ours</td><td>0.771</td><td>1.793</td><td>1.543</td><td>97.877</td><td>0.525</td><td>0.635</td></tr></table>

Table 1. Quantitative results of our proposed PLGSLAM with existing NeRF-based SLAM system on the Replica dataset [26]. We evaluate reconstruction and localization performance in small room scenes. The results are the average on the scenes of the Replica dataset. Our method outperforms the existing method in surface reconstruction and pose estimation.
<table><tr><td rowspan="2">Methods</td><td colspan="2">Reconstruction[cm]</td><td colspan="2">Localization[cm]</td></tr><tr><td>Acc.</td><td>Comp. Comp.Ratio(%)</td><td>Mean</td><td>RMSE</td></tr><tr><td>NICE-SLAM[41]</td><td>29.17</td><td>4.45</td><td>67.97 8.78</td><td>9.63</td></tr><tr><td>ESLAM[12]</td><td>26.22</td><td>4.53 71.43</td><td>7.89</td><td>8.95</td></tr><tr><td>Co-SLAM[31]</td><td>26.55</td><td>4.67</td><td>70.34 7.67</td><td>8.75</td></tr><tr><td>Ours</td><td>19.42</td><td>4.21</td><td>74.48 6.12</td><td>6.77</td></tr></table>

Table 2. Camera tracking results on the Scannet datasets [4]. We evaluate our camera tracking performance on the Scannet dataset to verify the effectiveness of our method. Our method achieves high-fidelity surface reconstructions and superior camera tracking. scenarios (nearly $7 . 5 m \times 6 . 6 m \times 3 . 5 m )$ . We also evaluate on Apartment dataset of the multi-rooms scene (nearly 14.5m × 7.5m × 3.8m with more than 12000 images) from NICE-SLAM [41].

Metrics. We use Depth L1 (cm), Accuracy (cm), Completion (cm), and Completion ratio (%) to evaluate the reconstruction quality. Following NICE-SLAM and ESLAM[12,

41], we perform frustum and occlusion mesh culling that removes unobserved regions outside frustum and the noisy points within the camera frustum but outside the target scene. However, this simple strategy removes too many meshes, leading to excessive holes and ineffective assessment of the reconstruction results. For the evaluation of camera tracking, we adopt ATE RMSE and Mean(cm).

Implementation We run PLGSLAM on a desktop PC with NVIDIA RTX 3090ti GPU. We employ feature planes with a resolution of 24 cm for coarse tri-planes. We use 6 cm resolution for fine tri-planes. All feature planes have 32 channels, resulting in a 64-channel concatenated feature input for the decoders. The decoders are two-layer MLPs with 32 channels in the hidden layer. For Replica [26], we sample N = 32 points for stratified sampling and $N _ { s u r f a c e } = 8$ points for importance sampling on each ray. And for Scan-

<table><tr><td rowspan="2">Methods</td><td colspan="4">Reconstruction</td><td rowspan="2">Localization [ATE Mean[cm] ↓ ATE RMSE[cm] ↓</td></tr><tr><td>Depth L1[cm] ↓ Acc.[cm] ↓ Comp.[cm] ↓ Comp.Ratio(%) ↑]</td><td></td><td></td><td></td></tr><tr><td>iMAP [27]</td><td>24.558</td><td>14.296</td><td>7.476</td><td>44.422</td><td>9.963 10.612</td></tr><tr><td>NICE-SLAM [41]</td><td>37.052</td><td>6.064</td><td>5.576</td><td>71.792</td><td>4.776 5.394</td></tr><tr><td>Vox-Fusion [36]</td><td>43.077</td><td>26.375</td><td>9.454</td><td>49.554</td><td>11.473 12.754</td></tr><tr><td>ESLAM [12]</td><td>16.355</td><td>17.546</td><td>4.301</td><td>71.626</td><td>6.637 7.283</td></tr><tr><td>Co-SLAM [31]</td><td>6.702</td><td>13.355</td><td>3.666</td><td>80.486</td><td>6.182 6.891</td></tr><tr><td>Ours</td><td>6.033</td><td>11.086</td><td>3.261</td><td>85.357</td><td>5.574 6.228</td></tr></table>

Table 3. Quantitative results of our proposed PLGSLAM with existing NeRF-based SLAM system on the Apartment dataset [41]. We evaluate reconstruction and localization performance in large-scale multi-room scenes. The results are the average of three runs. Our method outperforms the existing method in surface reconstruction and pose estimation.

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/663d97707e84cb8d88158561354906f6fc78c07cdf93770c709f527c762ce876.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/33b19d59758464cdf53da72891a5b79f2cdc78564168ca40f581aaeb4fe51de0.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/04f5a70e6c4e2c94ed6f53bd6ecab5089571ae5e8f9b3eeb3a3fb34c6bf5f281.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/0d7e9ec07317725336a165e85019b745017a32a20d7ae6250585911d0c161f6b.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/c3accca58be6c263dec39812f364ebefb5fda16f804212584297dde2c7af5be0.jpg)  
NICE-SLAM

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/3144ff3883e15a2505703361ecfd3b70ebb569a1997d91fb99fc2e3fbebdae0e.jpg)  
Co-SLAM

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/c5493db12b40f97d81bbe24a778c4bb86f531f78f3ba0a44c0c363da88d30bc8.jpg)  
ESLAM

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/3503e9f6df43975ec67f0a036decca8aa2352dff689ae4571525c00ce3a87633.jpg)  
Ours

Figure 5. Qualitative comparison of our proposed PLGSLAM method’s surface reconstruction and localization accuracy with existing NeRF-based dense visual SLAM methods, NICE-SLAM [41], Co-SLAM [31], and ESLAM [12] on the ScanNet dataset [4]. The ground truth camera trajectory is shown in blue, and the estimated trajectory is shown in red. Our method predicts more accurate camera trajectories and does not suffer from drifting issues. We also visualize the Absolute Trajectory Error ATE (bottom color bar) of different methods. The color bar on the right shows the relative scaling of color. It should also be noted that our method runs faster on this dataset.  
![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/e0028c645876f479485fb047f9fc8de80183c563d378019b3a145f84a467db38.jpg)

![](images/2024_PLGSLAM__Progressive_Neural_Scene_Represenation_with_Loc/32a624e31efb02e7b096c83f867ff212ec20a13fb5aa2579008f182b4211fb5a.jpg)  
Figure 6. Completion ratio vs. model size and average time for PLGSLAM with other methods. Each model corresponds to a different hash-table or tri-planes size.

Net [4], we set $N = 4 8$ and $N _ { s u r f a c e } = 8$ . For further details of our implementation, refer to the supplementary.

## 4.2. Experimental Results

Replica dataset. We evaluate on the same RGB-D sequences as ESLAM [12] and Co-SLAM [31]. We use this dataset to test our system performance in small room scenes (nearly $6 . 5 m \times 4 . 2 m \times 2 . 7 m )$ . As shown in Tab. 1, our method achieves higher reconstruction and pose estimation accuracy. We show the qualitative results in Fig. 4. We can see that ESLAM maintains more reconstruction details, but the results contain some artifacts. Co-SLAM achieves smooth completion in unobserved areas, but the accuracy of the reconstruction and pose estimation is relatively low. Our method successfully achieves consistent completion as well as high-fidelity reconstruction results.

Scannet dataset. We evaluate the camera tracking and reconstruction results of PLGSLAM on real-world large room sequences (nearly $7 . 5 m \times 6 . 6 m \times 2 . 7 m )$ from ScanNet [4] . We use the absolute trajectory error (ATE) as our metric. Tab. 2 shows that our method achieves better pose estimation and surface reconstruction results in comparison to NICE-SLAM [41], ESLAM [12], and Co-SLAM [31]. PLGSLAM exhibits superior scene representation capabilities and more accurate and robust tracking performance in large-scale indoor scenes. Fig. [4] also shows PLGSLAM achieves better reconstruction quality with smoother results and finer details.

<table><tr><td rowspan=1 colspan=1></td><td rowspan=1 colspan=1>|Method</td><td rowspan=1 colspan=1>|Speed FPT(s)</td><td rowspan=1 colspan=1>|Memory Grow.R.</td></tr><tr><td rowspan=2 colspan=1>Replica[26]</td><td rowspan=2 colspan=1>|NICE-SLAM[41]ESLAM[12]Co-SLAM[31]Ours</td><td rowspan=1 colspan=1>2.10</td><td rowspan=2 colspan=1>O(L3)O(L2)O(L3)O(L2)</td></tr><tr><td rowspan=1 colspan=1>0.180.160.14</td></tr><tr><td rowspan=2 colspan=1>Scannet[4]</td><td rowspan=2 colspan=1>|NICE-SLAM[41]ESLAM[12]Co-SLAM[31]Ours</td><td rowspan=1 colspan=1>3.35</td><td rowspan=2 colspan=1>O(L3)O(L2)O(L3)O(L2)</td></tr><tr><td rowspan=1 colspan=1>0.550.380.37</td></tr></table>

Table 4. Runtime analysis of our method in comparison with existing ones in terms of average frame processing time (AFPT), and model size growth rate w.r.t. scene side length L. We evaluate these method on replica dataset [26] and Scannet dataset [4]. Our method is greatly faster and the model size grow is significantly reduced from cubic to square.

<table><tr><td rowspan="2">Methods</td><td colspan="3">Reconstruction[cm]</td><td rowspan="2">Localization[cm] RMSE</td></tr><tr><td>Acc.</td><td>Comp. Comp.Ratio(%)</td><td>Mean</td></tr><tr><td rowspan="3">w/o joint enc. w/o prog. w/o lg BA</td><td>13.314</td><td>4.687</td><td>81.347 5.935</td><td>6.787</td></tr><tr><td>12.754</td><td>4.231 83.156</td><td>5.875</td><td>6.693</td></tr><tr><td>12.435 4.181</td><td>83.473</td><td>5.874</td><td>6.591</td></tr><tr><td>Ours</td><td>11.0863.261</td><td>85.357</td><td>5.574</td><td>6.228</td></tr></table>

Table 5. Ablation study. We conduct experiments on Apartment dataset [41] to verify the effectiveness of our method. Our full model achieves better completion reconstructions and more accurate pose estimation results.

Apartment dataset. We evaluate the surface reconstruction and camera tracking accuracy of PLGSLAM on Apartment dataset (nearly 14 $\hphantom { - } 5 m \times 7 . 5 m \times 3 . 2 m )$ . Tab. 3 shows that quantitatively, our method achieves SOTA tracking results in comparison to Co-SLAM and ESLAM. These algorithms typically exhibit significant cumulative errors in large-scale indoor dataset scenarios. Fig. 1 also shows PLGSLAM achieves better reconstruction quality with smoother results and finer details.

## 4.3. Runtime analysis

In this section, we analysis the speed and memory usage of our method compared with other SOTA methods in Replica datasets [26] and ScanNet datasets [4]. We report the average frame pocessing time (FPT) and the memory growth rate in Tab. 4. The results indicate that our method is faster than previous methods and the model size does not grow cubically with the scene length. In Fig. 6, we present the completion ratio under different model size and memory usage. We visualized the variation curve of the completion ratio by altering the size of the hash table/triplanes.

## 4.4. Ablation Study

In this section, we conduct various experiments to verify the effectiveness of our method. Tab. 5 illustrates a quantitative evaluation with different settings.

Joint scene representation. It is obvious that the joint scene representation (tri-planes with MLP) significantly improves our surface reconstruction accuracy.

Progressive scene representation. We replace our progressive scene representation and use a single network for the entire scene. We can observe that this method has a great influence on pose estimation and reconstruction metrics. This network significantly improves the capacity of scene geometry representation and enhances the robustness for local misestimation.

Local-to-global bundle adjustment. We remove our localto-global bundle adjustment in this experiment. Our full model leads to higher accuracy and better completion. The local-to-global BA can significantly reduce the growing cumulative error and improve the robustness and accuracy of the camera tracking.

## 5. Conclusion

In this paper, we propose a novel dense SLAM system, PLGSLAM, which achieve accurate surface reconstruction and pose estimation in large indoor scenes. Our progressive scene representation method enables our system to represent large-scale indoor scenes and long videos. The joint encoding method with the tri-planes and multi-layer perceptron further improves the accuracy of local scene representation. The local-to-global bundle adjustment method combines the traditional SLAM method with end-to-end pose estimation, which achieves robust and accurate camera tracking and mitigate the influence of cumulative error and pose drift. Our extensive experiments demonstrate the effectiveness and accuracy of our system in both scene reconstruction, depth estimation, and pose estimation.

Acknowledgement This work is supported by the National Key R&D Program of China (Grant 2020YFC2007500), the National Natural Science Foundation of China (Grant U1813206), and the Science and Technology Commission of Shanghai Municipality (Grant 20DZ2220400). Authors gratefully appreciate the contribution of Yanbo Wang from Shanghai Jiao Tong University, Hengyi Wang from University College London.

## References

[1] Dejan Azinovic, Ricardo Martin-Brualla, Dan B Goldman,´ Matthias Nießner, and Justus Thies. Neural rgb-d surface reconstruction. In Proceedings ofthe IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pages 6290–6301, June 2022. 2

[2] Aljaz Bozic, Pablo Palafox, Justus Thies, Angela Dai, and Matthias Nießner. Transformerfusion: Monocular rgb scene reconstruction using transformers. Advances in Neural Information Processing Systems, 34:1403–1414, 2021.

[3] Jaesung Choe, Sunghoon Im, Francois Rameau, Minjun Kang, and In So Kweon. Volumefusion: Deep depth fusion for 3d scene reconstruction. In Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), pages 16086–16095, October 2021. 2

[4] Angela Dai, Angel X. Chang, Manolis Savva, Maciej Halber, Thomas Funkhouser, and Matthias Niessner. Scannet: Richly-annotated 3d reconstructions of indoor scenes. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), July 2017. 5, 6, 7, 8

[5] Angela Dai, Matthias Nießner, Michael Zollhöfer, Shahram Izadi, and Christian Theobalt. Bundlefusion: Real-time globally consistent 3d reconstruction using on-the-fly surface reintegration. ACM Trans. Graph., 36(4), jul 2017. 2

[6] Yang Tang Debao Huang and Rongjun Qin. An evaluation of planetscope images for 3d reconstruction and change detection – experimental validations with case studies. GIScience & Remote Sensing, 59(1):744–761, 2022. 1

[7] Tianchen Deng, Yaohui Chen, Leyan Zhang, Jianfei Yang, Shenghai Yuan, Danwei Wang, and Weidong Chen. Compact 3d gaussian splatting for dense visual slam. arXiv preprint arXiv:2403.11247, 2024. 3

[8] Tianchen Deng, Siyang Liu, Xuan Wang, Yejia Liu, Danwei Wang, and Weidong Chen. Prosgnerf: Progressive dynamic neural scene graph with frequency modulated auto-encoder in urban scenes. arXiv preprint arXiv:2312.09076, 2023. 1

[9] Tianchen Deng, Hongle Xie, Jingchuan Wang, and Weidong Chen. Long-term visual simultaneous localization and mapping: Using a bayesian persistence filter-based global map prediction. IEEE Robotics & Automation Magazine, 30(1):36–49, 2023. 2

[10] Sara Fridovich-Keil, Alex Yu, Matthew Tancik, Qinhong Chen, Benjamin Recht, and Angjoo Kanazawa. Plenoxels: Radiance fields without neural networks. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, pages 5501–5510, 2022. 4

[11] Shahram Izadi, David Kim, Otmar Hilliges, David Molyneaux, Richard Newcombe, Pushmeet Kohli, Jamie Shotton, Steve Hodges, Dustin Freeman, Andrew Davison, et al. Kinectfusion: real-time 3d reconstruction and interaction using a moving depth camera. In Proceedings of the 24th annual ACM symposium on User interface software and technology, pages 559–568, 2011. 1, 2

[12] Mohammad Mahdi Johari, Camilla Carta, and François Fleuret. Eslam: Efficient dense slam system based on hybrid representation of signed distance fields. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, pages 17408–17419, 2023. 1, 3, 4, 6, 7,

8

[13] Mingrui Li, Jiaming He, Guangan Jiang, and Hongyu Wang. Ddn-slam: Real-time dense dynamic neural implicit slam with joint semantic encoding. arXiv preprint arXiv:2401.01545, 2024. 3

[14] Mingrui Li, Shuhong Liu, and Heng Zhou. Sgs-slam: Semantic gaussian splatting for neural dense slam. arXiv preprint arXiv:2402.03246, 2024. 3

[15] Jiuming Liu, Guangming Wang, Chaokang Jiang, Zhe Liu, and Hesheng Wang. Translo: A window-based masked point transformer framework for large-scale lidar odometry. In Proceedings of the AAAI Conference on Artificial Intelligence, volume 37, pages 1683–1691, 2023. 2

[16] Jiuming Liu, Guangming Wang, Zhe Liu, Chaokang Jiang, Marc Pollefeys, and Hesheng Wang. Regformer: An efficient projection-aware transformer network for large-scale point cloud registration. In Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), pages 8451–8460, October 2023. 2

[17] Andreas Meuleman, Yu-Lun Liu, Chen Gao, Jia-Bin Huang, Changil Kim, Min H Kim, and Johannes Kopf. Progressively optimized local radiance fields for robust view synthesis. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, pages 16539–16548, 2023. 2

[18] Ben Mildenhall, Pratul P. Srinivasan, Matthew Tancik, Jonathan T. Barron, Ravi Ramamoorthi, and Ren Ng. Nerf: Representing scenes as neural radiance fields for view synthesis. In ECCV, 2020. 2, 4

[19] Raúl Mur-Artal, J. M. M. Montiel, and Juan D. Tardós. Orbslam: A versatile and accurate monocular slam system. IEEE Transactions on Robotics, 31(5):1147–1163, 2015. 1, 2

[20] Raúl Mur-Artal and Juan D. Tardós. Orb-slam2: An opensource slam system for monocular, stereo, and rgb-d cameras. IEEE Transactions on Robotics, 33(5):1255–1262, 2017. 1

[21] Richard A Newcombe, Steven J Lovegrove, and Andrew J Davison. Dtam: Dense tracking and mapping in real-time. In 2011 international conference on computer vision, pages 2320–2327. IEEE, 2011. 2

[22] Michael Oechsle, Songyou Peng, and Andreas Geiger. Unisurf: Unifying neural implicit surfaces and radiance fields for multi-view reconstruction. In Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), pages 5589–5599, October 2021. 2

[23] Guohao Peng, Jun Zhang, Heshan Li, and Danwei Wang. Attentional pyramid pooling of salient visual residuals for place recognition. In 2021 IEEE/CVF International Conference on Computer Vision (ICCV), pages 865–874, 2021. 2

[24] Tong Qin, Peiliang Li, and Shaojie Shen. Vins-mono: A robust and versatile monocular visual-inertial state estimator. IEEE Transactions on Robotics, 34(4):1004–1020, 2018. 1, 2

[25] Thomas Schops, Torsten Sattler, and Marc Pollefeys. Bad slam: Bundle adjusted direct rgb-d slam. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), June 2019. 1, 2

[26] Julian Straub, Thomas Whelan, Lingni Ma, Yufan Chen, Erik Wijmans, Simon Green, Jakob J Engel, Raul Mur-Artal, Carl Ren, Shobhit Verma, et al. The replica dataset: A digital

replica of indoor spaces. arXiv preprint arXiv:1906.05797, 2019. 5, 6, 8

[27] Edgar Sucar, Shikun Liu, Joseph Ortiz, and Andrew J. Davison. imap: Implicit mapping and positioning in real-time. In ICCV, pages 6229–6238, October 2021. 1, 2, 6, 7

[28] Jiaming Sun, Yiming Xie, Linghao Chen, Xiaowei Zhou, and Hujun Bao. Neuralrecon: Real-time coherent 3d reconstruction from monocular video. In Proceedings ofthe IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pages 15598–15607, June 2021. 2

[29] Chengzhou Tang and Ping Tan. Ba-net: Dense bundle adjustment network. ICLR, 2018. 2

[30] Haithem Turki, Deva Ramanan, and Mahadev Satyanarayanan. Mega-nerf: Scalable construction of largescale nerfs for virtual fly-throughs. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, pages 12922–12931, 2022. 2, 3

[31] Hengyi Wang, Jingwen Wang, and Lourdes Agapito. Coslam: Joint coordinate and sparse parametric encodings for neural real-time slam. In Proceedings ofthe IEEE/CVF Conference on Computer Vision and Pattern Recognition, pages 13293–13302, 2023. 1, 3, 4, 6, 7, 8

[32] Peng Wang, Lingjie Liu, Yuan Liu, Christian Theobalt, Taku Komura, and Wenping Wang. Neus: Learning neural implicit surfaces by volume rendering for multi-view reconstruction. In M. Ranzato, A. Beygelzimer, Y. Dauphin, P.S. Liang, and J. Wortman Vaughan, editors, Advances in Neural Information Processing Systems, volume 34, pages 27171–27183. Curran Associates, Inc., 2021. 2

[33] Yuanbo Xiangli, Linning Xu, Xingang Pan, Nanxuan Zhao, Anyi Rao, Christian Theobalt, Bo Dai, and Dahua Lin. Bungeenerf: Progressive neural radiance field for extreme multi-scale scene rendering. In European conference on computer vision, pages 106–122. Springer, 2022. 3

[34] Hongle Xie, Tianchen Deng, Jingchuan Wang, and Weidong Chen. Robust incremental long-term visual topological localization in changing environments. IEEE Transactions on Instrumentation and Measurement, 72:1–14, 2022. 2

[35] Hongle Xie, Tianchen Deng, Jingchuan Wang, and Weidong Chen. Angular tracking consistency guided fast feature association for visual-inertial slam. IEEE Transactions on Instrumentation and Measurement, 2024. 2

[36] Xingrui Yang, Hai Li, Hongjia Zhai, Yuhang Ming, Yuqian Liu, and Guofeng Zhang. Vox-fusion: Dense tracking and mapping with voxel-based neural implicit representation. In 2022 IEEE International Symposium on Mixed and Augmented Reality (ISMAR), pages 499–507, 2022. 6, 7

[37] Jun Zhang et al. Ntu4dradlm: 4d radar-centric multi-modal dataset for localization and mapping. In 2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC), pages 4291–4296. IEEE, 2023. 2

[38] Jun Zhang and Danwei others. 4dradarslam: A 4d imaging radar slam system for large-scale environments based on pose graph optimization. In 2023 IEEE International Conference on Robotics and Automation (ICRA), pages 8333– 8340. IEEE, 2023. 2

[39] Siting Zhu, Renjie Qin, Guangming Wang, Jiuming Liu, and Hesheng Wang. Semgauss-slam: Dense semantic gaussian splatting slam. arXiv preprint arXiv:2403.07494, 2024. 3

[40] Siting Zhu, Guangming Wang, Hermann Blum, Jiuming Liu, Liang Song, Marc Pollefeys, and Hesheng Wang. Sni-slam: Semantic neural implicit slam. arXiv preprint arXiv:2311.11016, 2023. 3

[41] Zihan Zhu, Songyou Peng, Viktor Larsson, Weiwei Xu, Hujun Bao, Zhaopeng Cui, Martin R. Oswald, and Marc Pollefeys. Nice-slam: Neural implicit scalable encoding for slam. In CVPR, pages 12786–12796, June 2022. 1, 2, 4, 6, 7, 8