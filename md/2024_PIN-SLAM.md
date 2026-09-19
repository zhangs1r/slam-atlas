# PIN-SLAM: LiDAR SLAM Using a Point-Based Implicit Neural Representation for Achieving Global Map Consistency

Yue Pan , Xingguang Zhong, Louis Wiesmann , Graduate Student Member, IEEE, Thorbjörn Posewsky, Jens Behley , Member, IEEE, and Cyrill Stachniss , Member, IEEE

Abstract—Accurate and robust localization and mapping are essential components for most autonomous robots. In this paper, we propose a simultaneous localization and mapping (SLAM) system for building globally consistent maps, called point-based implicit neural (PIN)-SLAM, that is based on an elastic and compact PIN map representation. Taking range measurements as input, our approach alternates between incremental learning of the local implicit signed distance field and the pose estimation using a correspondence-free, point-to-implicit model registration to the current local map. Our implicit map is based on sparse optimizable neural points, which are inherently elastic and deformable with the global pose adjustment when closing a loop. Loops are also detected using the neural point features. Extensive experiments validate that PIN-SLAM is robust to various environments and versatile to different range sensors, such as light detection and ranging (LiDAR) and RGB color and depth (RGB-D) cameras. PIN-SLAM achieves pose estimation accuracy better or on par with the state-of-the-art LiDAR odometry or SLAM systems and outperforms the recent neural implicit SLAM approaches while maintaining a more consistent, and highly compact implicit map that can be reconstructed as accurate and complete meshes. Finally, thanks to the voxel hashing for efficient neural points indexing and the fast implicit map-based registration without closest point association, PIN-SLAM can run at the sensor frame rate on a moderate graphics processing unit (GPU).

Index Terms—Deep learning, localization, mapping, simultaneous localization and mapping (SLAM).

## I. INTRODUCTION

IMULTANEOUS localization and mapping (SLAM) based on range sensors, such as light detection and ranging (Li-DAR) sensors or RGB color and depth (RGB-D) cameras, is a fundamental building block for autonomous mobile robots [15], [48], [78], [97]. Various map representations [4], [12], [76] and scan registration algorithms [5], [52], [64] have been proposed over the last decades for efficient and high-fidelity representations of the environment as well as robust and accurate localization of the robot within the mapped environment.

The recent advance in neural implicit representation has shown several advantages over the classical explicit map representation widely used nowadays. Instead of explicitly storing properties in a grid map, one can train a neural network to fit the observations in a scene. This allows for querying properties, such as signed distances [53], occupancy probability [43], colors [44], intensities [22], and semantics [99], at an arbitrary location within the scene implicitly using the neural network. As a memory-efficient continuous representation, these kinds of implicit neural maps support efficient rendering [47], surface reconstruction [100], collision avoidance [51] and path planning [91]. To enable the model to have a high representation capacity, current approaches often learn the neural implicit representation using optimizable local latent features instead of using a single globally shared neural network [51], [71]. The local latent features can be structured by a regular 3-D grid [47], an axis-aligned tri-plane [55] with 2-D grids, or a set of irregular points [89]. Consequently, several SLAM systems based on neural implicit representation have been proposed, mainly for RGB-D cameras operating indoor [24], [65], [71], [92], [101] but also for LiDAR sensors operating outdoor [11]. However, these SLAM approaches lack support for direct loop closure corrections and are not able to build globally consistent maps of larger scenes. This is mainly due to the usage of regular grid-based local feature embeddings, which are not elastic and resilient to loop corrections.

In this article, we investigate the problem of realizing a SLAM system using an implicit neural map representation that supports globally consistent mapping. We opt to use a neural point-based implicit representation, which has two main advantages over grid-based representations: the flexibility of spatial distribution and the elasticity for transformations. Recent work [65] only makes use of the first advantage for neural RGB-D SLAM at the cost of scalability and inefficient neighborhood querying. Instead, we exploit the second and more important advantage to build a globally consistent map that can be corrected while online mapping after closing a loop, which is essential for large-scale LiDAR-based SLAM. Our approach alternates between mapping, i.e., online learning of the local implicit signed distance map given poses, and odometry, i.e., tracking the next scan’s pose given the current local map, additionally with the ability to correct the drift and keep a globally consistent map after closing a detected loop. For the mapping part, we adapt the training data sampling strategy and loss function used in our previous work [100] but replace the octree-based multiresolution feature grid with a set of neural feature points. The signed distance function (SDF) and additional properties, such as color or semantics, at a query position are predicted by interpolating among the surrounding neural points. For each neural point, we concatenate its optimizable latent feature with the query position’s coordinates in the neural point’s coordinate system. This concatenated feature is then decoded by a globally shared multilayer perceptron (MLP) as the prediction for that neural point. For efficient and scalable neighborhood querying for the interpolation, we use a voxel hashing data structure to index the neural points by keeping no more than one active neural point in each voxel. This enables our system to run at the sensor frame rate (10 Hz) regardless of the scale of the mapped scene. To tackle the problem of “catastrophic forgetting” in incremental learning, instead of using the less stable regularization-based strategy [100], we use a local data pool replay and local map update strategy in a sliding window manner.

![](images/2024_PIN-SLAM/904ca3e9c751e99fb43869d67f50839ad965bea9e1126963b2cc787078d97090.jpg)  
Fig. 1. We present PIN-SLAM, a novel LiDAR SLAM system using an elastic PIN map representation. Depicted in the middle, we show a large-scale globally consistent neural point map built with our approach using about 20 000 LiDAR scans recorded with a car without using any information from a GNSS, IMU, or wheel odometry. We can query the SDF value at an arbitrary position from the neural point map and reconstruct surface meshes. The point colors represent the neural point feature after online optimization. On the left, we show the consistent neural points (top) and mesh (bottom) of a region traversed by the car multiple times indicated by the dashed orange box. The colors of the neural points (top) represent timesteps when the point was added to the map. On the right, we show the high-fidelity mesh (bottom) of a building reconstructed from the neural point map (top) of the region indicated by a dashed blue box.

For the odometry part, we extend our previous work Loc-NDF [86] to accomplish scan-to-implicit map registration efficiently without the effort of point correspondence association using a second-order optimization to estimate the ego-motion incrementally. We propose an additional robust kernel based on the regularity of the predicted SDF to further improve the registration accuracy and robustness.

To correct the drift accumulated by the odometry, we use the optimized neural point local map for global loop closure detection and correction. Pose graph optimization (PGO) is conducted after a loop verification to correct the drift and the inherently elastic neural points are transformed along with the poses of their associated frames to form a globally consistent map, from which a consistent SDF and mesh can be generated.

The main contribution of this article is a novel neural SLAM system, called point-based implicit neural (PIN)-SLAM, based on a PIN map representation that supports building largescale globally consistent maps online, as shown in Fig. 1. To the best of the authors’ knowledge, PIN-SLAM is the first full-fledged implicit neural SLAM system including odometry, loop closure detection, and globally consistent implicit mapping.

In sum, we make four key claims, which will be backed up in the experiments as follows:

1) Our SLAM system achieves localization accuracy better or on par with state-of-the-art LiDAR odometry/SLAM approaches and is more accurate than recent implicit neural SLAM methods on various datasets using different range sensors.

2) Our method can conduct large-scale globally consistent mapping with loop closure thanks to the elastic neural point representation.

3) Our map representation is more compact than the previous counterparts and can be used to reconstruct accurate and complete meshes at an arbitrary resolution.

4) Our correspondence-free scan-to-implicit map registration and the efficient neural point indexing by voxel hashing enable our algorithm to run at the sensor frame rate on a single NVIDIA A4000 graphics processing unit (GPU).

## II. RELATED WORK

## A. LiDAR Odometry and SLAM

SLAM using range sensors, such as LiDAR sensors, has been an active research topic for the last decades. A LiDAR SLAM system often consists of odometry, mapping, and optionally the loop closure correction. At the core of such a system are the map representation and the point cloud registration algorithm.

The earlier works on 2-D LiDAR SLAM [15], [18], [29] adopt a probabilistic occupancy grid map and use 2-D scan matching based on iterative closest point (ICP) algorithm [5] or particle filters [15], [46] for ego-motion estimation.

For 3-D LiDAR odometry and mapping, similar to the featurematching-based methods popularized in visual SLAM, the seminal work LiDAR odometry and mapping (LOAM) [97] proposes to extract sparse planar or edge feature points from the scan point cloud and registers them to the last frame or the feature point map using ICP. LOAM inspired multiple follow-up works [36], [52], [56], [67], [68], [82] based on more sophisticated feature point extraction [68], [82], classification and registration optimization schemes [36], [52] or the fusing with inertial measurements [56], [67], leading to faster, more accurate and more robust odometry estimation.

Recently, CT-ICP [10] and KISS-ICP [78] achieved strong LiDAR odometry performance without feature point extraction. They register and append voxel-downsampled point cloud to a local point cloud map supporting efficient neighborhood search using either point-to-plane or point-to-point metrics. Combined with a motion model providing initial poses, these feature-free “direct” methods [10], [13], [31], [78] are comparably robust in structure-less scenarios and can take advantage of denser observations for optimization. The same idea has been applied to LiDAR-inertial odometry [88], [90] at a high operating frequency. Our method belongs to the “direct” algorithms but avoids the time-consuming and nondifferentiable correspondence association procedure of ICP.

Except for the point-based map used by aforementioned methods, some other works estimate the ego-motion by conducting scan registration to other explicit map representations, such as 3-D surfels [4], [7], triangle meshes [62], [76], and voxelized Gaussian distributions [30], [93], as well as the implicit map representations, such as moving least square model [12], Gaussian process [63], and grid-based implicit neural SDF [11]. In contrast, our method uses a PIN SDF map representation, which combines the best of both worlds: it retains the flexibility and elasticity characteristic of point-based methods while offering continuity and compactness simultaneously thanks to the neural representation.

There exists also learning-based LiDAR odometry methods [33], [80], [81], which realize registration between adjacent frames using end-to-end pose supervision. However, they typically cannot generalize well to unseen test sets and do not make use of the map for odometry. Similar to recent work neural radiance field (NeRf)-LOAM [11], our method exploits neural networks to fit the SDF online using the input point clouds, thus requiring no pretraining and generalizing well in various scenarios.

Besides incremental pose estimation, loop closure detection and correction are necessary for building a globally consistent, long-term SLAM system. Various approaches have been proposed to first conduct LiDAR place recognition and then estimate the transformation between the queried and retrieved scan using geometry-based [26], [27] or learning-based [41], [42], [75] scan-wise global descriptor matching as well as local feature matching and verification [40]. However, previous methods often rely on the raw point cloud from a sparse single scan for descriptor generation and matching. In contrast, our method makes use of the neural point features in the local maps for loop closure detection. The usage of the local map makes our method robust to occlusions and sensors with a narrow field of view. Besides, reusing the online-optimized neural point features can avoid the generalizability concern of offline-trained feature extractors and save additional computation for local feature extraction.

## B. Implicit Neural Map Representation

Over the past decades, explicit map representations have been widely used in the robotics community for localization [74], planning, and exploration [69]. These methods explicitly represent the scene using point cloud [97], surfels [4], triangle meshes [76], or voxel grids storing either the occupancy probabilities [15] or a truncated signed distance function (TSDF) [48]. Various approaches have been proposed to improve the scalability and efficiency of the map data structure [19], [49], [77] and to realize the incremental integration from sensor measurements [50] for the dense volumetric mapping. These explicit representations often discretize the scene with a fixed spatial resolution. Another line of work adopts a continuous implicit representation, such as Gaussian processes [87] or reproducing kernel Hilbert maps [58]. However, these approaches currently do not scale well to 3-D data.

Recently, implicit neural representations have been proven effective in modeling radiance fields [44] and geometric (occupancy or distance) fields [43], [53] using fairly simple neural networks. These representations have demonstrated notable success in various applications, including novel view synthesis, 3-D surface reconstruction, and shape completion. Following the seminal works on NeRF [44], DeepSDF [53], and occupancy networks [43], numerous works target improving the scalability of the implicit neural representation while boosting the time and memory efficiency. Instead of representing the whole scene with a single MLP, more recent methods exploit a hybrid representation by jointly optimizing explicitly stored local latent features and a shallow MLP. These works propose to store the optimizable local latent features in various data structures such as multi-resolution dense voxel grids [55], sparse voxel hashing grids [35], [47], octree nodes [72], permutohedral lattices [61], axis-aligned tri-plane 2-D grids [6], or as an unordered point sets [89].

The advances in efficient training of scalable implicit neural representations open up an avenue to implicit neural mapping and SLAM systems. Compared to traditional methods, implicit neural map representations have several attractive properties, such as more compact storage, better noise smoothing capabilities, and stronger inpainting and hole-filling ability for sparse or occluded observations. In the realm of mapping and SLAM from a stream of RGB-D data, several works propose to use a single MLP [2], [51], [71] and a hybrid representation combining grid-based local latent features and a shallow MLP [21], [24], [83], [92], [101] to model the radiance or geometric field of the scene and simultaneously track the camera pose. These approaches have shown comparable tracking accuracy compared to the classic visual odometry methods and can generate a more complete and compact map, which can be reconstructed as a 3-D mesh.

Although with fewer works, a related trend can be seen in LiDAR mapping and localization research. IR-MCL [32] and LocNDF [86] propose to localize the robot within an implicit neural distance map built by laser scans. To scale up the implicit neural representation to large-scale outdoor LiDAR data, SHINE-Mapping [100] stores local features in octree-based sparse voxels. Toward a SLAM system, LONER [23] integrates the incremental neural mapping into a LiDAR odometry front-end using ICP. Recently, NeRF-LOAM [11] proposed an implicit neural LiDAR odometry and mapping system using an online optimizable octree-based feature grid. Besides grid-based representations, the PIN representation stores latent features in a set of neural points. It has been applied to efficient NeRF fitting [89], dynamic NeRF [1], adaptive surface reconstruction [34], and RGB-D SLAM [65]. Although also using neural points, the recent work point-SLAM [65] targets only indoor RGB-D SLAM and does not support loop closure and globally consistent mapping. Point-SLAM takes advantage of the flexibility of point-based representation for the adaptive scene encoding, i.e., allocating more features to structures that require more details. We, instead, make use of the elasticity property of neural points for building a globally consistent map online by transforming the neural points on loop closures.

The aforementioned neural implicit RGB-D and LiDAR SLAM systems still have two major limitations. First, most of them [11], [65], [71], [101] cannot run at the sensor frame rate, limiting their applications for online robotics applications. This is mainly due to the map data structure, computationally demanding differentiable rendering-based optimization for mapping, or an inefficient gradient descent optimization for pose estimation. Second, the map representations of previous neural implicit SLAM approaches do not support loop closure correction, and thus cannot build globally consistent maps, especially in outdoor long-term robotics missions. The reason for this is the usage of the regular latent feature grids that are not elastic to pose corrections introduced by loop closures. Once a loop is corrected and the poses are updated, previous methods need to re-allocate the feature grids and retrain the entire map, which is computationally demanding and clearly prevents online applications. To solve the first challenge, we realize an efficient SLAM system by adopting direct point-wise SDF supervision in a local map during mapping and a second-order on-manifold optimization for odometry estimation. To deal with the second limitation, our approach exploits the elastic and deformable point-based implicit representation to avoid grids and thus the remapping after loop correction.

## C. Global Consistency in SLAM

Global consistency is a desired property for maps. The solutions to globally consistent mapping can be divided into remapping-based, submap-based, and point-based methods.

The remapping-based method, such as Bundle Fusion [8], accomplishes online globally consistent 3-D reconstruction using on-the-fly surface reintegration in a bundle adjustment (BA) manner at the cost of substantial computational efforts.

The submap-based methods accumulate the observations as a submap and assume the submap is a locally defined rigid body. The pseudoglobal consistency is maintained by optimizing a graph linking submaps and their associated poses through submap-to-submap registrations. The submap can use different representations, such as feature points [37], [52] and TSDF [59], [84]. The usage of submaps decreases the number of nodes and edges in PGO, thus saving computational effort. However, these methods have issues of ambiguity in the overlapping region of the adjacent submaps and determining appropriate criteria for submap division to balance the rigidity and efficiency.

The point-based methods represent the map as a fully deformable point set or surfels, each associated with a frame pose. Whelan et al. [85] proposed elastic fusion for globally consistent RGB-D SLAM using surfel map with deformation graph, which is further scaled for outdoor surfel-based LiDAR SLAM [4], [7]. These methods can handle globally consistent mapping without the need for submap division. However, in contrast to volumetric mapping, point-based methods are less suitable for online 3-D reconstruction and path planning. This is attributed to their sparsity and the absence of a direct representation of free or unknown space.

Only a few recent works are trying to solve the loop closure correction and globally consistent mapping using implicit neural map representation without storing the raw observation data and conducting remapping after the pose update. IMT-Mapping [95] uses the SE(3)-equivalent grid-based feature for implicit mapping. When a loop is corrected, IMT-Mapping can directly get the updated grid features by interpolating the transformed SE(3)-equivalent features. Following the submap-based explicit mapping systems [52], [59], [84], MIPS-Fusion [73] and NF-Atlas [94] employ an implicit submap with MLP or octree-based feature grid and adjust the submaps when a loop is corrected to keep the consistency.

In contrast to previous methods, our PIN map belongs to the point-based globally consistent mapping approach. It avoids the nontrivial submap division and overlapping disambiguation while mapping a continuous SDF that enables efficient 3-D reconstruction.

## III. OUR APPROACH TO GLOBALLY CONSISTENT IMPLICIT NEURAL SLAM

Our approach PIN-SLAM primarily addresses large-scale LiDAR SLAM for global map consistency, with applicability to other range sensors, such as RGB-D cameras.

Notations: In the following, we denote the transformation of a point $\mathbf { \nabla } _ { \pmb { p } _ { A } }$ in coordinate frame A to a point ${ \pmb p } _ { B }$ in coordinate frame B by $\mathsf { \bar { T } } _ { B A } \in \mathrm { S E } ( 3 )$ , such that $\pmb { p } _ { B } = \mathsf { T } _ { B A } \pmb { p } _ { A }$ . Note that we use the homogeneous coordinate conversion of points before and after the affine transformation, but will not include this operation explicitly in the following derivations. We let $R _ { B A } \in \bar { \bf S O } ( 3 )$ and $\pmb { t } _ { B A } \in \dot { \mathbb { R } } ^ { 3 }$ denote the corresponding rotational and translational part of transformation $\mathsf { T } _ { B A }$

We denote the sensor coordinate frame at timestep t as $C _ { t }$ Frame $C _ { t }$ is associated to the world coordinate frame W by a pose $\mathsf { T } _ { W C _ { t } } \in \mathrm { S E } ( 3 )$ . We assume the transformation ${ \mathsf { T } } _ { W C _ { 0 } }$ from the first frame $C _ { 0 }$ to the world frame W is a constant, as either the identity matrix or the extrinsic calibration matrix. We denote the position of frame $C _ { t }$ as t<sub>WC</sub> and the accumulated travel distance up to frame $C _ { t }$ as $D ( t )$ , which is given by

$$
D ( t ) = \sum _ { n = 1 } ^ { t } \| \pmb { t } _ { C _ { n - 1 } C _ { n } } \| _ { 2 } .\tag{1}
$$

Overview: Before the detailed explanation of our methodology, we provide a brief overview of PIN-SLAM’s pipeline. For each point cloud $\mathcal { P } = \{ p \in \mathbb { R } ^ { 3 } \}$ measured at timestep t, we simultaneously estimate the pose ${ \mathsf { T } } _ { W C _ { t } }$ of the sensor frame and update the PIN map M. For every timestep, as shown in Fig. 2, PIN-SLAM performes the following steps.

![](images/2024_PIN-SLAM/0e490e58494929a96b9faa76d320370d3a48cb2fd8e29e9746850411923391de.jpg)  
Fig. 2. Pipeline overview of PIN-SLAM. Starting from a point cloud P scanned at timestep t, (1) a point cloud for registration $\mathcal { P } _ { r }$ and a point cloud for mapping $\mathcal { P } _ { m }$ are voxel-downsampled. (2) We align $\mathcal { P } _ { r }$ to the implicit SDF of current local map $\mathcal { M } _ { l }$ to estimate the global pose of current frame $\mathsf { T } _ { W C _ { t } } . \left( 3 \right) \mathsf { T } _ { W C _ { t } }$ is then used to transform $\mathcal { P } _ { m }$ into the map coordinate system. With the transformed ${ \mathcal { P } } _ { m } ,$ we update the PIN map M and optimize the neural point features in the local map $\mathcal { M } _ { l }$ by online incremental learning. (4) We generate polar context descriptor $\mathbf { U } _ { t }$ using the current local map $\mathcal { M } _ { l }$ and search for loop closures by comparing $\mathbf { U } _ { t }$ to descriptors generated for previous frames. Once a loop between frame $\hat { C } _ { t }$ and $C _ { k }$ is detected, we add the transformation $\mathsf { T } _ { C _ { k } C _ { t } }$ as a loop edge of the pose graph and then (5) conduct the PGO. The position and orientation of the neural points in $\mathcal { M }$ are transformed along with their associated frames after the PGO, leading to a globally consistent map. With the PIN map, we can query the SDF value at an arbitrary position during or after the SLAM task for path planning and mesh reconstruction.

1) Preprocessing: We voxel-downsample the input point cloud P into the point cloud for registration $\mathcal { P } _ { r }$ and the point cloud for mapping $\mathcal { P } _ { m }$ (Section III-B).

2) Odometry: We estimate the global pose T<sub>WC</sub> by registering the point cloud P<sub>r</sub> to the implicit SDF of the local map $\mathcal { M } _ { l }$ . The odometry transformation $\mathsf { T } _ { C _ { t - 1 } C _ { t } }$ is added as an edge in the pose graph G (Section III-C).

3) Mapping: We filter the dynamic points in $\mathcal { P } _ { m }$ based on the map M. Then, we sample along the ray from the sensor to each point in $\mathcal { P } _ { m }$ to get the training samples D and transform them to the world frame using T<sub>W</sub> $\scriptstyle \mathbf { C } _ { t }$ . We use the close-to-surface samples $\mathcal { D } _ { s } \subset \mathcal { D }$ to initialize new neural points, append them to the map M and reset the local map $\mathcal { M } _ { l }$ centered at the current position $\mathbf { \Delta } _ { t _ { W C _ { t } } }$ . We update the training sample pool $\mathcal { D } _ { p }$ with the training samples D of the current frame. Then we optimize the neural point features in the local map $\mathcal { M } _ { l }$ using the samples in the pool $\mathcal { D } _ { p }$ with direct SDF supervision by gradient descent. After that, we allocate the updated local map $\mathcal { M } _ { l }$ back to the global map M (Section III-D).

4) Loop closure detection: We generate a local polar context descriptor $\mathbf { U } _ { t }$ using the local map $\mathcal { M } _ { l }$ . Then, we search for a potential loop closure by comparing the feature distances between the descriptors of the current frame and candidate frames. Once a loop closure candidate between frame $C _ { t }$ and $C _ { k }$ is detected, we verify it by registering the point cloud $\mathcal { P } _ { r }$ ofthe current frame to the local map $\mathcal { M } _ { l }$ centered at the sensor position $\scriptstyle \mathbf { \mathfrak { t } } _ { W C _ { k } }$ of frame $C _ { k }$ . We add the loop transformation $\mathsf { T } _ { C _ { k } C _ { t } }$ resulting from the registration as an edge in the pose graph $\mathcal { G }$ if the registration succeeds (Section III-E).

5) Pose graph optimization: Once a loop closure edge is added, we optimize the pose graph G. The position and orientation of each neural point in the global map $\mathcal { M }$ are transformed along with its associated frames after the optimization to keep the global consistency. We then transform the training sample pool $\mathcal { D } _ { p }$ and reset the local map $\mathcal { M } _ { l }$ accordingly after loop correction (Section III-F).

For the first timestep, we initialize the map using only the first scan as done in the mapping step. During or after the SLAM, we can query the SDF value at an arbitrary position for mesh reconstruction via the marching cubes algorithm [39].

Next, we will explain the basics, data structure, and training process of the proposed PIN map representation (Section III-A) and then explain each step of the pipeline in more detail.

## A. Neural Point-Based Map Representation

1) Map Representation: We define the proposed PIN map as a set of neural points

$$
\mathcal { M } = \{ \pmb { m } _ { i } = ( \pmb { x } _ { i } , \pmb { q } _ { i } , \pmb { f } _ { i } ^ { g } , \pmb { t } _ { i } ^ { c } , \pmb { t } _ { i } ^ { u } , \mu _ { i } ) \ | \ i = 1 , \dots , N \}\tag{2}
$$

where each neural point $m _ { i }$ is defined in the world frame $W$ by a position $\pmb { x } _ { i } \in \mathbb { R } ^ { 3 }$ and a quaternion $\ b q _ { i } \in \mathbb { R } ^ { 4 }$ representing the orientation of its own coordinate frame. Each neural point stores the optimizable latent feature vectors $\pmb { f } _ { i } ^ { g } \in \mathbb { R } ^ { F _ { g } }$ representing the local geometry. In addition, we keep track of the creation timestep $t _ { i } ^ { c }$ , the last update timestep $t _ { i } ^ { u }$ and the stability $\mu _ { i }$ for each neural point to determine whether a neural point is active or inactive, stable or unstable. We link each neural point $\mathbf { m } _ { i }$ to the sensor pose $\mathsf { T } _ { W C _ { t } m }$ at the mean of the neural point’s creation and last timestep $t _ { i } ^ { \bar { m } } = \lfloor ( t _ { i } ^ { c } + t _ { i } ^ { u } ) / 2 \rfloor$ to directly manipulate the map by updating the sensor poses.

As shown in Fig. 3, similar to the autodecoder architecture of DeepSDF [53], we predict the SDF value s at a query position p conditioned on K nearby neural points. For each neural point $m _ { j }$ in the K-neighborhood $\mathcal { N } _ { p }$ of the query position $^ { p , }$ we first concatenate the latent feature vector $f _ { j } ^ { g }$ and the relative coordinate $d _ { j }$ . Here, $d _ { j }$ represents the query position in the coordinate system of the neural point $m _ { j }$ . Then, we feed the concatenated vector to a globally shared geometric decoder $D _ { \theta } ^ { g }$ , a shallow MLP containing $M _ { \mathrm { m l p } }$ hidden layers with $N _ { \mathrm { m l p } }$ neurons, to predict the SDF value $s _ { j }$ as

![](images/2024_PIN-SLAM/0dae9ae537148c3d6fef2068a6bfcc3f10ec249880e808fff7b6c59cdeae6d97.jpg)  
Fig. 3. Diagram of SDF querying in our PIN map simplified in 2-D. (a) Point in gray is the query position p while the other points are the neighboring neural points. Each neural point predicts the SDF value $s _ { i }$ at the query position by feeding the neural point feature $\pmb { f } _ { i } ^ { g }$ and the query point’s position $\mathbf { \Delta } d _ { i }$ under the neural point’s coordinate system through a globally shared decoder $D _ { \theta } ^ { g } .$ . Then, the predictions are weighted as the final prediction s according to the distances from the neural points to the query position. (b) Orientation of each neural point defines its local coordinate system, ensuring the relative coordinate $\mathbf { } d _ { i }$ and thus the SDF querying invariant to rigid-body transformation.

$$
s _ { j } = D _ { \theta } ^ { g } ( f _ { j } ^ { g } , d _ { j } )\tag{3}
$$

where the relative coordinate $d _ { j }$ is given by

$$
\begin{array} { r } { \pmb { d } _ { j } = \pmb { q } _ { j } ( \pmb { p } - \pmb { x } _ { j } ) \pmb { q } _ { j } ^ { - 1 } . } \end{array}\tag{4}
$$

As shown in Fig. 3(b), the concatenation of relative coordinate $d _ { j }$ makes the prediction invariant to the local translation and rotation, thus enabling the elasticity of the map after loop closure correction, which will be further explained in Section III-F.

The predicted SDF values $s _ { j }$ of the K nearest neural points at the query position $_ p$ are then interpolated as the final prediction $s = S ( \pmb { p } )$ by inverse distance weighting, given by

$$
S ( { \pmb p } ) = \sum _ { j \in \mathcal { N } _ { p } } \frac { w _ { j } } { \sum _ { k \in \mathcal { N } _ { p } } w _ { k } } s _ { j }\tag{5}
$$

where the weights $w _ { j }$ are defined as

$$
w _ { j } = \| \pmb { p } - \pmb { x } _ { j } \| ^ { - 2 } .\tag{6}
$$

Likewise, we define the stability $\mu = H ( p )$ at the query position $_ p$ as a distance-weighted mean of the stability $\mu _ { j }$ of the K nearby neural points by switching $s _ { j }$ with $\mu _ { j }$ in $( 5 )$ .

Intuitively, the final prediction can be regarded as a voting of the individual prediction from each neighboring neural point $m _ { j }$ or an ensemble of multiple locally defined DeepSDF-like autodecoder models. In contrast with the relatively deep decoder used in DeepSDF [53], which needed to be pretrained to represent the object globally, we represent the local geometries mainly in the locally defined latent feature space of neural points and use a shallow decoder as a general interpreter to map from the feature space to SDF values.

2) Map Data Structure: For fast neural point indexing and neighborhood search, we maintain a voxel hashing data structure with a fixed voxel resolution $v _ { p }$ and a hash table size T. This structure serves to organize the neural points, ensuring that each voxel contains no more than one active neural point. In line with prior work [47], [78], we use a spatial hashing function $\kappa = h ( \pmb { p } _ { W } )$ to map from a position $\pmb { p } _ { W } \in \mathbb { R } ^ { 3 }$ to an entry $\kappa \in \mathbb { Z }$ in the hash table. This entry stores either the index i of a neural point $m _ { i }$ in the map M or the default value −1, indicating that the entry is not yet occupied.

In contrast to point-SLAM [65], for efficient proximity search during the SDF prediction at a query position $^ { p , }$ we use the voxel structure to find the neural points in the voxel-defined neighborhood $\mathcal { N } _ { p } ^ { v }$ containing $N _ { n } \times N _ { n } \times N _ { n }$ voxels centered at the corresponding voxel of p with constant time access. We sort the distances and take the K nearest neural points in $\mathcal { N } _ { p } ^ { v }$ to get the K-neighborhood $\mathcal { N } _ { p }$ for SDF prediction. The larger $N _ { n }$ is, the larger the receptive field would be while the computation time for neighborhood search would increase. If the number of neighboring neural points $K _ { n }$ is $1 \leq K _ { n } < K$ , we conduct SDF interpolation among the $K _ { n }$ points.

3) Map Initialization and Update: We initialize or update neural points at each timestep using the measured point cloud. For a point $p _ { W }$ in the world frame W measured at timestep t, we will initialize a new neural point $m _ { k } .$ , append it to the map M and set the hashing entry’s value as the new neural point’s index, i.e., $, \kappa  k$ only under the following three circumstances. 1) The hashing entry is not yet occupied, i.e., 1). The hashing entry is not vet ccupied i e $\kappa = - 1$

2) There is a hash collision, i.e., the position $\scriptstyle { \mathbf { \mathscr { x } } } _ { \kappa }$ of currently stored neural point $\mathbf { \nabla } m _ { \kappa }$ in the voxel of the added point $p _ { W }$ is far away from $p _ { W }$ due to hash collision.

3) The stored neural point $\mathbf { \nabla } m _ { \kappa }$ is no longer active, i.e., $D ( t ) - D ( t _ { \kappa } ^ { u } ) > d _ { l }$ , meaning the travel distance from the last updated timestep $t _ { \kappa } ^ { u }$ of the neural point to the current timestep t is larger than a travel distance threshold $d _ { l }$ . This is used to differentiate between the historical observation and the present observation at the same position in the world frame upon revisiting.

In such cases, we initialize a new neural point $m _ { k }$ to have a position $\mathbf { \boldsymbol { x } } = \mathbf { \boldsymbol { p } } _ { W }$ , an identity quaternion $\pmb { q } = ( 1 , 0 , 0 , 0 )$ , a zero feature vector $\pmb { f } ^ { g } = \mathbf { 0 }$ , creation and last update timestep as $t ^ { c } = t ^ { u } = t .$ , and the stability as $\mu = 0$ . In the latter two cases, the originally stored neural point is replaced by the new one and no longer indexed by the voxel hashing map. However, they are kept in the global neural point map for the sake of loop closure correction (cf., Section III-E) and globally consistent map adjustment (cf., Section III-F).

4) Local Map: Our method distinguishes between an active local map $\mathcal { M } _ { l }$ and a global map M. The registration described in Sections III-C and III-E are performed using the local map $\mathcal { M } _ { l }$ to avoid the alignment to the inconsistent historical observations caused by odometry drift.

In previous works, a local map is often defined using either a spatial window [52], [78] or a temporal window [4], [85]. We propose to use both of them. In practice, the speed of the robot may vary a lot during the operation. For instance, the robot may stop at a position for a long time. Since the drift of the odometry is often proportional to the accumulated travel distance $D ( \dot { t } )$ instead of the time span, we propose to use a travel distance threshold instead of the timestep threshold [4], [85] to determine the local temporal window. Consequently, we define a local map $\mathcal { M } _ { l }$ centered at the sensor position $\mathbf { \Delta } _ { t _ { W C _ { t } } }$ at timestep t. This map includes all neural points within both the spatial window $\| \pmb { x } _ { i } - \pmb { t } _ { W C _ { t } } \| _ { 2 } < r _ { l }$ and the temporal window $\hat { D ( t ) } - D ( t _ { i } ^ { c } ) <$ $d _ { l }$ . Here, $r _ { l }$ represents the radius of the local spatial window, and $d _ { l }$ denotes the previously mentioned travel distance threshold.

![](images/2024_PIN-SLAM/a87f0a2d1fb29556df66f315779ee75d4c2f632009b010717a01b68319f70981.jpg)  
Fig. 4. Example during the operation of PIN-SLAM at a timestep: (a) Point cloud for mapping $\mathcal { P } _ { m }$ . A moving bus is highlighted in the green circle. (b) Sparser point cloud for registration ${ \mathcal { P } } _ { r } .$ , colorized according to the point-wise registration weight from black to red. The points in blue are filtered, mainly lying at dynamic objects, rough vegetation, and newly observed regions. (c) Points from the training sample pool $\mathcal { D } _ { p }$ for map optimization, colorized according to their SDF target values from blue to red.

Next, we will explain how we incrementally optimize the neural point features in the local map $\mathcal { M } _ { l }$ using a sliding window-like training sample pool $\mathcal { D } _ { p }$

5) Map Training Samples: At each timestep $t ,$ we take samples along the rays from the voxel-downsampled egocentric scan $\mathcal { P } _ { m } \overset { \cdot } { = } \{ p \in \mathrm { \mathbb { R } } ^ { 3 } \}$ to collect training data. On each ray ${ \pmb r } = { \pmb p } / \| { \pmb p } \| _ { 2 }$ from the sensor to the measured point $^ { p , }$ we can represent a sampled point u in the sensor frame $\dot { C } _ { t }$ uniquely with its depth d along the ray, given by: ${ \pmb u } = d { \pmb r }$ . We take the endpoint $( d = { \bar { \| } } p \| _ { 2 } )$ and $N _ { s }$ points close to the surface with the depth sampled from a Gaussian distribution $d _ { s } \sim \mathcal N ( \| p \| _ { 2 } , \sigma _ { s } ^ { 2 } )$ with the endpoint as the mean and $\sigma _ { s }$ as the standard deviation. In addition, we sample $N _ { f }$ points uniformly in the free space in front of the surface with the depth $d _ { f } \sim \dot { U } ( \zeta _ { \operatorname* { m i n } } \| \pmb { p } \| _ { 2 } , \| \pmb { p } \| _ { 2 } - 2 \sigma _ { s } )$ and $N _ { b }$ points in the truncated free space behind the surface with the depth $d _ { b } \sim U ( \| p \| _ { 2 } + 2 \sigma _ { s } , \| \hat { \pmb { p } } \| _ { 2 } + d _ { b } )$ , where $\zeta _ { \mathrm { m i n } }$ is the minimum sample depth ratio and $d _ { b }$ represents the maximum sample range behind surfaces. For each sample point $^ { \mathbf { \delta u } , }$ we take the projective signed distance along the ray as its SDF target value ${ \hat { s } } = \| { \pmb { p } } \| _ { 2 } - d$ . Although the projective distance would always be an overestimation, it is computed quickly and is a good approximation when the sample point is close to the surface.

We define the training samples D at timestep t as $N _ { t }$ sample positions and their target values from all the rays, given by

$$
\mathcal { D } = \{ ( { \boldsymbol { u } } _ { j } , \hat { \boldsymbol { s } _ { j } } , t ) ~ | ~ j = 1 , \ldots , N _ { t } \}\tag{7}
$$

where $N _ { t } = M _ { t } ( N _ { s } + N _ { f } + N _ { b } + 1 )$ and $M _ { t }$ is the number of points (rays) in point cloud $\mathcal { P } _ { m }$ . We associate the training samples with the corresponding sensor frame $C _ { t }$ by recording the timestep t of each sample so that we can later transform each sample point into the world frame.

To tackle the “catastrophic forgetting” problem in incremental mapping, we maintain a training sample pool $\mathcal { D } _ { p }$ by appending the samples D from each timestep for replaying the historical samples, as shown in Fig. 4(c). We filter $\mathcal { D } _ { p }$ at each timestep t by removing the samples lying outside a local sliding window with a radius $\boldsymbol { r } _ { p } .$ , centered at the current sensor position $\pmb { t } _ { W C _ { t } }$ given by

$$
\left\| \mathbf { u } _ { W } ^ { i } - \pmb { t } _ { W C _ { t } } \right\| _ { 2 } > r _ { p }\tag{8}
$$

where $\pmb { u } _ { W } ^ { i } = \mathsf { T } _ { W C _ { t _ { i } } } \pmb { u } _ { i }$ is the sample position in world frame and $\begin{array} { r } { r _ { p } = r _ { l } - \frac { \sqrt { 3 } } { 2 } N _ { n } v _ { p } } \end{array}$ is set to enforce that the training samples would only affect the neural points in the local map, considering the local map radius $r _ { l } ,$ neighborhood voxel counts $N _ { n } ,$ and the voxel size $v _ { p } .$ . In addition, to deal with the case when the robot stops, if the number of samples in $\mathcal { D } _ { p }$ exceeds $N _ { p } { } _ { : }$ , we randomly keep $N _ { p }$ samples.

6) Map Training Losses: We want to train our neural points to predict SDF value on arbitrary locations and these neural point features need to be trained ideally during the incremental mapping process. During incremental mapping, we randomly sample from the training sample pool $\mathcal { D } _ { p }$ in batch for training. For the training of the SDF, we use a similar loss function as our previous work [100], which combines the binary cross entropy (BCE) loss and the Ekional regularization loss.

At a sample position ${ \pmb u } _ { W } ^ { i } = \top _ { W C _ { t _ { i } } } { \pmb u } _ { i }$ in the world frame, we map both the SDF prediction $s _ { i } = S ( { \pmb u } _ { W } ^ { i } )$ from the nearby neural points and target value $\hat { s } _ { i }$ to the range of [0, 1] by a scaled sigmoid function $\Phi _ { s } ( s ) = 1 / ( 1 + e ^ { s / \sigma _ { t } } )$ as

$$
o _ { i } = \Phi _ { s } ( s _ { i } ) , \hat { o } _ { i } = \Phi _ { s } ( \hat { s } _ { i } )\tag{9}
$$

and then the BCE loss is calculated for $N$ samples as

$$
\mathcal { L } _ { \mathrm { B C E } } = \frac { 1 } { N } \sum _ { i = 1 } ^ { N } \hat { o } _ { i } \log ( o _ { i } ) + ( 1 - \hat { o } _ { i } ) \log ( 1 - o _ { i } ) .\tag{10}
$$

The BCE loss enables fast convergence of the SDF [100]. It accomplishes the logistic regression of the projective SDF value and realizes the soft truncation of signed distance close to the surface. We can adjust the truncation smoothness using the scale factor $\sigma _ { t }$ in the scaled sigmoid function $\Phi _ { s }$

One necessary property of SDF is the Ekional equation [16], $\mathrm { i . e . , } \| \nabla S ( \pmb { x } ) \| _ { 2 } = 1$ . Therefore, we use the Ekional loss $\mathcal { L } _ { \mathrm { e i k } }$ to enforce the regularity and validity of the fitted SDF

$$
\mathcal { L } _ { \mathrm { e i k } } = \frac { 1 } { N } \sum _ { i = 1 } ^ { N } \left( \left\| \nabla S \left( \pmb { u } _ { W } ^ { i } \right) \right\| _ { 2 } - 1 \right) ^ { 2 } .\tag{11}
$$

In line with Neuralangelo [35], we calculate the numerical gradient by manually adding perturbations instead of using the analytical gradient based on automatic differentiation. This enables the backpropagation updates beyond the local hash grid that stores neural points, resulting in a smoother SDF gradient for the calculation of $\mathcal { L } _ { \mathrm { e i k } }$ . The gradient component at a position x on the x-axis is given by

$$
\nabla _ { x } S ( { \pmb x } ) = \frac { S ( { \pmb x } + { \pmb \epsilon } _ { x } ) - S ( { \pmb x } - { \pmb \epsilon } _ { x } ) } { 2 \epsilon }\tag{12}
$$

where $\epsilon _ { x } = ( \epsilon , 0 , 0 ) ^ { \top }$ is the perturbation vector on the x-axis and  is the perturbation step size used on every axis. The gradient on the $y -$ and z-axes can be calculated likewise.

The final loss is then formulated as a weighted sum of BCE and Ekional term, given by

$$
\mathcal { L } = \mathcal { L } _ { \mathrm { b c e } } + \lambda _ { e } \mathcal { L } _ { \mathrm { e i k } }\tag{13}
$$

where $\lambda _ { e }$ is the weight for Ekional loss. During training, we minimize the loss by optimizing latent features of the involved neural points in the local map. For each sample point $\mathbf { \Delta } \mathbf { u } _ { i }$ , we also update the last update timestep $t _ { j } ^ { u }$ and stability $\mu _ { j }$ of each neural point $m _ { j }$ in the neighborhood $\mathcal { N } _ { p }$ of $\mathbf { \Delta } \mathbf { u } _ { i }$ , given by

$$
t _ { j } ^ { u } \gets \operatorname* { m a x } ( t _ { j } ^ { u } , t _ { i } ) , \mu _ { j } \gets \mu _ { j } + \frac { w _ { j } } { \sum _ { j \in \mathcal { N } _ { p } } w _ { j } }\tag{14}
$$

where $t _ { i }$ is the timestep of this training sample, and the weight $w _ { j }$ can be calculated as (6).

Next, we will explain how we use the proposed map representation, namely, the PIN map, to realize a LiDAR SLAM system toward globally consistent mapping.

## B. Preprocessing

For each egocentric point cloud $\mathcal { P }$ measured at timestep $t ,$ we voxel-downsample $\bar { \mathcal P }$ into the point cloud for mapping $\mathcal { P } _ { m }$ with a smaller voxel size $v _ { m }$ , and the point cloud for registration $\mathcal { P } _ { r }$ with a larger voxel size $v _ { r } .$ . During the downsampling, we keep exactly one point, namely, the one whose coordinate is the closest to the voxel center.

Typically for LiDAR sensors, the point cloud of one sweep (frame) consists of the continuously scanned points measured across the acquisition time. Therefore, for each scan frame, we first need to revert the distortions of the point cloud caused by the sensor motion. In line with Vizzo et al. [78], we use a constant velocity model for motion prediction. Based on this model, we first deskew the points $\mathcal { P } _ { r }$ for registration before the odometry estimation by interpolating the motion prediction with the pointwise timestamp. We deskew $\mathcal { P } _ { m }$ with the more accurate ego-motion from odometry afterward.

## C. Odometry Estimation

To achieve efficient and robust odometry, we propose a correspondence-free, scan-to-implicit map registration method based on second-order optimization under multiple weighting strategies, which is based on the approach proposed by Wiesmann et al. [86] targeting localization in known maps.

Our goal is to align the source point cloud $\mathcal { P } _ { r }$ at timestep t to the neural SDF of the local PIN map $\mathcal { M } _ { l }$ by finding the transformation ${ \sf T } ^ { * } \in \mathrm { S E } ( 3 )$ minimizing the least square error of the SDF prediction at the transformed points, given by

$$
\mathsf { T } ^ { * } = \mathop { \mathrm { a r g m i n } } _ { \mathsf { T } } \sum _ { p \in \mathcal { P } _ { r } } S \left( \mathsf { T } p \right) ^ { 2 }\tag{15}
$$

which can be solved by a Levenberg–Marquardt optimization taking the initial guess predicted by a constant velocity model.

We denote the 6DOF transformation parameters as the Lie algerba $\xi = [ t , \Theta ] = \log ( \mathsf { T } )$ , where t is the translation vector and $\Theta = \log \bar { ( } R )$ is the axis-angle representation of the rotation matrix $R .$ The Jacobian of the transformed point $p _ { i } ^ { \prime } = \mathsf { T } p _ { i }$ with regards to $\boldsymbol { \xi }$ is given by

$$
J _ { i } = \left[ \frac { \partial S \left( \pmb { p } _ { i } ^ { \prime } \right) } { \partial t } , \frac { \partial S \left( \pmb { p } _ { i } ^ { \prime } \right) } { \partial \pmb { \Theta } } \right] = \left[ \pmb { g } _ { i } ^ { \top } , ( \pmb { p } _ { i } ^ { \prime } \times \pmb { g } _ { i } ) ^ { \top } \right]\tag{16}
$$

where $\mathbf { \nabla } \mathbf { { g } } _ { i } = \nabla S ( \mathbf { \nabla } \mathbf { { p } } _ { i } ^ { \prime } )$ is the distance gradient at $\mathbf { \Delta } _ { p _ { i } ^ { \prime } } ,$ which can be queried from the implicit neural distance field of PIN map by automatic differentiation. In our experiments, we discovered that using this analytical gradient for odometry estimation enhances robustness compared to utilizing the smoother but less precise numerical gradient employed for the Ekional regularization in mapping, see (12).

Intuitively, the registration can be solved by knowing, in which direction, given by g and how much, given by $\bar { S ( p ^ { \prime } ) }$ we have to go. This is conducted without knowing the explicit point-to-point correspondences. The de facto optimization target in previous neural implicit SLAM approaches [65], [71], [92], [101] is the depth rendering loss, which can be seen as related to the point-to-point metric with projection-based data association. Our method optimizes the point-to-model SDF loss, which is similar to the point-to-plane metric in ICP with the closest surface-based data association. The latter method has a faster convergence rate and more robust optimization according to the study by Rusinkiewicz, and Levoy [64].

We then approximate the Hessian matrix H as $H = J ^ { \Gamma } P J$ and calculate the gradient of the target function as $\begin{array} { r } { \mathbf { { \boldsymbol { g } } } = \bar { \mathcal { I } } ^ { \top } \bar { P } \mathbf { { \boldsymbol { b } } } . } \end{array}$ , where $J \in \mathbb { R } ^ { N _ { r } \times 6 }$ is the Jacobian, $\boldsymbol { P } \in \mathbb { R } ^ { N _ { r } \times N _ { \tau } }$ is the weight matrix, and $\pmb { b } \in \mathbb { R } ^ { N _ { r } }$ is the residual vector. We filter points that have fewer than $K$ neural points in their $N _ { p } ^ { v }$ neighborhood and set each row of $\boldsymbol { J }$ using the $N _ { r }$ remaining points as (16). For each iteration, the increment of the transformation parameters $\delta \xi$ is given by

$$
\delta \pmb { \xi } = ( H + \lambda _ { d } \operatorname { d i a g } \left( H \right) ) ^ { - 1 } \pmb { g }\tag{17}
$$

where $\lambda _ { d }$ is the damping parameter for Levenberg–Marquardt optimization. The termination criterion is determined by a threshold $\gamma _ { c }$ for the applied correction $\delta \xi$ and a maximum number of iterations $\tau _ { c }$

To robustify the optimization under noisy measurements and underfitted map, we apply the Geman-McClure (GM) robust kernel for both the SDF residual $S ( \pmb { p } _ { i } ^ { \prime } )$ and the SDF gradient anomaly $\epsilon _ { i }$ to down-weight the potential measurement noise, dynamic objects, and underfitted details of the implicit map. The pointwise robust kernel weights are given by

$$
w _ { i } ^ { r } = \left( \frac { \kappa _ { r } } { \kappa _ { r } ^ { 2 } + S ( { p } _ { i } ^ { \prime } ) ^ { 2 } } \right) ^ { 2 }\tag{18}
$$

$$
w _ { i } ^ { g } = \left( \frac { \kappa _ { g } } { \kappa _ { g } ^ { 2 } + \varepsilon _ { i } ^ { 2 } } \right) ^ { 2 }\tag{19}
$$

where $\kappa _ { r }$ and $\kappa _ { g }$ are the scale parameters of the robust kernel. The gradient anomaly $\varepsilon _ { i }$ is defined as the deviation of the distance gradient from the Ekional equation, given by

$$
\varepsilon _ { i } = \big | \big | \nabla S ( { \pmb p } _ { i } ^ { \prime } ) \big | \big | _ { 2 } - 1 \big | .\tag{20}
$$

To apply the robust kernel to the optimization, we scale each diagonal element of the weight matrix $P$ with the point-wise robust kernel weights $w _ { i } ^ { r } w _ { i } ^ { g }$ . An example of the filtering and point-wise weight of ${ \mathcal P _ { r } }$ is shown in Fig. 4(b).

After convergence, we additionally calculate the Eigenvalues λ of the Hessian matrix H for a degeneracy check [96]. The registration will be regarded as a success if the average residual $\bar { \pmb { b } } > b _ { s }$ , the ratio of valid points $\alpha > \alpha _ { s }$ , and the minimum Eigenvalue min $( \lambda ) > \lambda _ { s }$ for the last iteration, where $b _ { s } , \alpha _ { s }$ , and $\lambda _ { s }$ are the thresholds. Ifthe optimization succeeds, $\mathrm { i . e . }$ , all checks are fulfilled, we take $\mathsf { T } _ { W C _ { t } } = \exp ( \xi )$ as the estimated pose of the current frame $C _ { t }$ for mapping (Section III-D). Otherwise, we directly use the initial guess, i.e., the constant velocity estimate, but skip the mapping step to avoid introducing wrong measurements.

For a potential PGO, we add the odometry transformation $\mathsf { T } _ { C _ { t - 1 } C _ { t } }$ as an odometry edge connecting pose node $C _ { t - 1 }$ and $C _ { t }$ in the pose graph G.

## D. Mapping and Local BA

With the sensor poses $\mathsf { T } _ { W C _ { t } }$ estimated by the odometry at each timestep, we can update and optimize the PIN map M using the point cloud for mapping $\mathcal { P } _ { m }$

First, we filter the dynamic objects from $\mathcal { P } _ { m }$ and use only the static part for mapping. Inspired by Dynablox [66], we make use of the assumption that a measured point lying in the stable free space can be regarded as a dynamic point. Thereby, we filter points fulfilling $( S ( p _ { W } ) > \gamma _ { d } ) \wedge ( H ( p _ { W } ) > \gamma _ { \mu } )$ , where $S ( \pmb { p } _ { W } )$ and $H ( \pmb { p } _ { W } )$ are the SDF and stability prediction at the measured point $\pmb { p } _ { W } = \pmb { \top } _ { W C _ { t } } \pmb { p }$ in the world frame. $\gamma _ { d }$ and $\gamma _ { \mu }$ are the free space distance and stability threshold, respectively. Only the static points remain for mapping.

As we explained in Section III-A, we get training samples $\mathcal { D }$ by sampling along the rays from the sensor to the static points in $\mathcal { P } _ { m }$ . We initialize new neural points in the map M using the close-to-surface sample points $\mathcal { D } _ { s }$ transformed by current pose ${ \mathsf { T } } _ { W C _ { t } }$ . We then reset the local map $\mathcal { M } _ { l }$ centered at the current position $\mathbf { \Delta } _ { t _ { W C _ { t } } }$ , and append current training samples $\mathcal { D }$ to the training sample pool $\mathcal { D } _ { p }$ . Next, we conduct PIN map optimization by gradient descent using the training samples from $\mathcal { D } _ { p }$ in batches. When the SLAM starts, we jointly optimize the neural point feature $f ^ { g }$ in the local map $\bar { \mathcal { M } _ { l } }$ and the weights of the MLP decoder $\dot { D } _ { \theta } ^ { g }$ for the first $F _ { \mathrm { m l p } }$ timesteps. After $F _ { \mathrm { m l p } }$ timesteps, we freeze $D _ { \theta } ^ { g }$ and optimize only the neural point features to avoid catastrophic forgetting due to the ever-changing decoder while incremental mapping.

The odometry estimation relies on pairwise scan-to-map registration and always optimizes only the latest pose. It ignores the multiview consistency of multiple scans in the local sliding window, thus pruning to rapidly accumulated drift. Inspired by the BA technique widely employed in photogrammetry and computer vision, Liu et al. [38] introduced a LiDAR BA algorithm. This algorithm aims to refine odometry by simultaneously optimizing the scene geometry and a collection of sensor poses. The target of this optimization is to minimize inconsistencies among explicit geometric features, such as edges and planes.

We propose an implicit local BA approach making use of the implicit neural map without the tedious extraction of explicit geometric features. For every $F _ { \mathrm { b a } }$ timesteps, we jointly optimize the sensor poses at the past $N _ { \mathrm { b a } }$ timesteps, and the neural point features in the local map $\mathcal { M } _ { l } .$ , taking the current state as the initial guess. We optimize the L2 SDF regression loss by gradient descent using only the endpoint samples from the training sample pool $\mathcal { D } _ { p }$ to achieve a more consistent local map and poses. The loss function is given by

$$
L _ { \mathrm { b a } } = \sum _ { \tau \in \mathcal { T } _ { \mathrm { b a } } } \sum _ { p _ { i } \in \mathcal { P } _ { m } ^ { \tau } } S \left( \mathsf { T } _ { W C _ { \tau } } \pmb { p } _ { i } \right) ^ { 2 }\tag{21}
$$

where $T _ { \mathrm { b a } } = \{ t - N _ { \mathrm { b a } } + 1 , \ldots , t - 1 , t \}$ are the timesteps used for local BA. t is the current timestep and $\mathcal { P } _ { m } ^ { \tau }$ is the point cloud for mapping at timestep τ .

After the map optimization and local BA, we assign the updated local map $\mathcal { M } _ { l }$ back to the global map $\mathcal { M } .$

## E. Loop Closure Detection

Detecting loop closures is essential to correct the accumulated drift of the odometry for globally consistent mapping.

First, we use a distance-based criterion for identifying local loop closures. Specifically, we assess whether the positions $\mathbf { } t _ { t }$

and $\mathbf { \Delta } _ { t _ { i } }$ of the current scan at $C _ { t }$ and a historical scan at $C _ { t _ { i } }$ (where $t _ { i } < t$ and $D ( t ) - D ( t _ { i } ) > d _ { l } )$ meet the condition

$$
\| t _ { t } - t _ { t _ { i } } \| _ { 2 } < d _ { \mathrm { l o o p } } .\tag{22}
$$

If there is no local loop candidate, we search for global loop closures by comparing frame-wise descriptors. We propose a neural point feature enhanced local map descriptor for typical driving scenarios. This descriptor is used for global loop detection and provides a semimetric localization as the initial guess for the relative loop transformation. To address the place recognition challenges arising from diverse scan patterns and occlusions caused by varied viewpoints and dynamic objects, we use the local map as the processing unit for loop detection, instead of relying on a single scan. Therefore, at each timestep $t ,$ with a latency of $F _ { \mathrm { l a t } }$ timesteps waiting for sufficient map training using the most recent observations, we generate a local context descriptor $\mathbf { U } _ { t }$ using neural points in the local map $\mathcal { M } _ { l }$

Similar to Scan Context [27], the neural points in the local map are mapped into discretized 2-D polar-coordinate bins by a bird-eye-view projection with the ring resolution $H _ { r }$ and sector resolution $H _ { s }$ . Instead of encoding the maximum point height in each bin to create the descriptor [27], we encode the average neural point features optimized by the self-supervised online mapping and keep a $F _ { g }$ dimensional vector in each bin, thus resulting in a descriptor $\mathbf { U } _ { t } \in \mathbb { R } ^ { H _ { r } \times H _ { s } \times F _ { g } }$ . We average the sector dimensions of $\mathbf { U } _ { t }$ to get a retrieval key $\mathbf { R } _ { t } \in \mathbb { R } ^ { \breve { H } _ { r } \times F _ { g } }$ as a rotation-invariant descriptor of the local map for fast global descriptor retrieval. We employ the mean of ring-wise feature cosine distance as the comparative metric and select the closest historical frame $C _ { k }$ as a loop candidate. Then, the rough relative rotation between the local map centered at the current and candidate loop frame can be estimated by shifting along the sector dimension of $\mathbf { U } _ { t }$ and taking the shift number $n ^ { * }$ with the minimum bin-wise feature cosine distance $d _ { c } ^ { * } .$ , which is calculated as the mean of $H _ { r } \cdot H _ { s }$ bins. If $d _ { c } ^ { * } < d _ { \mathrm { m c } }$ , where $d _ { \mathrm { m c } }$ is the cosine distance threshold, we accept the loop candidate and use the relative rotation around z-axis by $\frac { 2 n ^ { * } \pi } { H _ { s } }$ as the rotation part of the semimetric localization.

Furthermore, to deal with the deficiency in translational invariance, we follow Scan Context++ [26] to use a polar context augmentation to enable the loop with large relative translation at the revisit. Specifically, for each scan position $\mathbf { \Delta } _ { t _ { W C _ { t } } }$ , we generate $2 V _ { a }$ local map context descriptors at virtual positions $\eta _ { t } ^ { j }$ shifted from $\pmb { t _ { W C } }$ by increments of equal length $d _ { v }$ along a direction perpendicular to the motion path $\delta \pmb { t } = \pmb { t } _ { W C _ { t } } - \pmb { t } _ { W C _ { t - 1 } }$ , results in virtual scan positions

$$
\eta _ { t } ^ { j } = t _ { W C _ { t } } + \frac { j d _ { v } } { \left. \left. \delta t \right. \right. _ { 2 } } R _ { z } \left( 9 0 ^ { \circ } \right) \delta t , j = - V _ { a } , . . . , V _ { a }\tag{23}
$$

where $R _ { z } ( 9 0 ^ { \circ } )$ denotes the rotation around z-axis by $9 0 ^ { \circ }$ . We use these descriptors to query the most similar historical descriptors. We use the one resulting in the minimum cosine distance and take $\eta _ { t } - t _ { W C _ { 1 } }$ as the translation part of semimetric localization results. We refer to Kim et al. [26] for more details regarding retrieval and augmentation.

When a candidate loop is detected, we register the current scan $\mathcal { P } _ { r }$ to the local map $\mathcal { M } _ { l }$ centered at the sensor position $\scriptstyle t _ { W C _ { k } }$ of the loop frame $C _ { k }$ as described in Section III-C. For scan-to-map registration of local loops, we initialize the transformation with the identity matrix. For global loops, we use the results obtained from semimetric localization as the initial guess for the transformation. If the registration succeeds, we take the transformation $\mathsf { T } _ { C _ { k } C _ { t } }$ resulting from the registration as the loop closure edge connecting node $C _ { k }$ and $C _ { t }$ in the pose graph G. Otherwise, the candidate loop will be rejected.

## F. Globally Consistent Implicit Neural Map Adjustment

Once a loop is detected and verified by scan-to-map registration, we conduct PGO taking the current poses as the initial guess and using constant information matrices. To avoid the almost redundant optimization, which would take up significant time, we disable the loop detection for $F _ { \mathrm { l o o p } }$ timesteps after conducting a PGO. After performing PGO, the neural points in the global map M will move along with their associated frames. We apply a transformation to both the position and orientation of each neural point, as detailed in Section III-A. The transformation is determined according to the associated sensor frame $C _ { t _ { i } ^ { m } }$ of the neural point as the mean of its creation and last update timestep $t _ { i } ^ { m } = \bar { \lfloor ( t _ { i } ^ { c } + t _ { i } ^ { u } ) / 2 \rfloor }$ . We denote the pose difference of frame $C _ { t }$ before and after the PGO as $\delta \mathsf { T } _ { t } = \mathsf { T } _ { W C _ { t } ^ { \prime } } ^ { - 1 } \mathsf { T } _ { W C _ { t } }$ and the quaternion for the rotation part of $\delta \mathsf { T } _ { t }$ as $\delta \pmb q _ { t }$ . We can update the position $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ and orientation $\pmb q _ { i }$ of the neural point as

$$
\pmb { x } _ { i } \gets \delta \mathsf { T } _ { t _ { i } ^ { m } } \pmb { x } _ { i } , \ \pmb { q } _ { i } \gets \delta \pmb { q } _ { t _ { i } ^ { m } } \pmb { q } _ { i } .\tag{24}
$$

After transforming the neural points, we recreate the voxel hash map and reset the local map to ensure correct neural point indexing. If there are multiple neural points in one voxel, we keep the one whose stability $\mu$ is higher to avoid redundant map memory consumption in the revisiting region. To enable incremental mapping after PGO, we also transform the samples in the training sample pool $\mathcal { D } _ { p }$ according to the optimized poses of their associated frames.

## IV. EXTENSION TO RGB-D OR METRIC-SEMANTIC SLAM

As a proof of concept, we show how we can extend our approach to RGB-D and metric-semantic SLAM by extending the environment model with color or semantic information.

If the input point cloud contains point-wise RGB values or semantic classification probabilities predicted by any off-theshelf segmentation models [45], we can additionally predict the RGB value $c \in \mathbb { R } ^ { 3 }$ and the L-class semantic probability $\pmb { v } \in \mathbb { R } ^ { L }$ at $_ p$ by each nearby neural point $m _ { j }$ in the same way as the SDF value prediction

$$
\pmb { c } _ { j } = D _ { \theta } ^ { c } \left( \pmb { f } _ { j } ^ { c } , \pmb { d } _ { j } \right) , \pmb { v } _ { j } = D _ { \theta } ^ { s } \left( \pmb { f } _ { j } ^ { s } , \pmb { d } _ { j } \right)\tag{25}
$$

where $D _ { \theta } ^ { c }$ and $D _ { \theta } ^ { s }$ are the globally shared color and semantic decoder, and $f _ { j } ^ { c }$ and $f _ { j } ^ { s }$ are the latent feature of neural point $m _ { j }$ assigned for color and semantics. We can interpolate the final RGB prediction $c = C ( p )$ and semantic prediction $\pmb { v } = V ( \pmb { p } )$ in the same way as (5). To train the color and semantic field, we set the color target values $\hat { \ b { c } } _ { j }$ and semantic pseudolabels $\hat { \mathbf { } v } _ { j }$ for the $N _ { s } + 1$ close-to-surface samples as the same value at the endpoint. We then add the L1 loss $\mathcal { L } _ { \mathrm { c o l } }$ for color regression with a weight $\lambda _ { c }$ or the L-class cross-entropy loss $\mathcal { L } _ { \mathrm { s e m } }$ for semantic segmentation with a weight $\lambda _ { s }$ to (13) using the close-to-surface training samples

$$
\mathcal { L } _ { \mathrm { c o l } } = \frac { 1 } { N _ { c } } \sum _ { i = 1 } ^ { N _ { c } } \left. \hat { \mathbf { c } } _ { i } - \mathbf { c } _ { i } \right. _ { 1 }\tag{26}
$$

$$
\mathcal { L } _ { \mathrm { s e m } } = - \frac { 1 } { N _ { c } } \sum _ { \substack { i = 1 } } ^ { N _ { c } } \sum _ { j = 1 } ^ { L } p _ { i } ^ { j } \log { \hat { p } _ { i } ^ { j } }\tag{27}
$$

where $\hat { \ b { c } } _ { i }$ and $\pmb { c } _ { i } = C ( \pmb { u } _ { W } ^ { i } )$ are the color label and prediction while $p _ { i } ^ { j }$ is the semantic prediction probability for class $j$ and $\hat { p } _ { i } ^ { j }$ is the one-hot probability for the semantic pseudolabel.

For odometry estimation explained in Section III-C, with the point-wise color available, we add the photometric term into the optimization with a weight of $w _ { c }$ to provide more constraints. In this case, we additionally minimize the photometric difference at the transformed position $\pmb { p } _ { i } ^ { \prime }$ between the observed value $c _ { i }$ and the predicted value $C ( \pmb { p } _ { i } ^ { \prime } )$ from the PIN map as

$$
\pmb { \xi } ^ { * } = \underset { \pmb { \xi } } { \mathrm { a r g m i n } } \sum _ { \pmb { p _ { i } \in \mathcal { P } _ { r } } } S \left( \pmb { p _ { i } ^ { \prime } } \right) ^ { 2 } + w _ { c } \left. C \left( \pmb { p _ { i } ^ { \prime } } \right) - \pmb { c _ { i } } \right. _ { 2 } ^ { 2 } .\tag{28}
$$

The Jacobian for the photometric term $J _ { i } ^ { c }$ can be calculated likewise the geometric term as (16), by replacing the SDF gradient g with the color gradient defined by $\check { \nabla } C ( \pmb { p } _ { i } ^ { \prime } )$

$$
\begin{array} { r } { \pmb { J _ { i } ^ { c } } = \left[ \nabla C \left( \pmb { p } _ { i } ^ { \prime } \right) ^ { \top } , ( \pmb { p } _ { i } ^ { \prime } \times \nabla C \left( \pmb { p } _ { i } ^ { \prime } \right) ) ^ { \top } \right] . } \end{array}\tag{29}
$$

We then append $J _ { i } ^ { c }$ to Jacobian matrix J for the iterative optimization as (17).

During or after the mapping, we can colorize the mesh reconstructed from the SDF by querying the color or semantic value at the position of the mesh vertices.

## V. EXPERIMENTAL EVALUATION

The main focus of this work is a LiDAR SLAM system for building globally consistent maps using a PIN map representation.

We present our experiments to show the capabilities of our method. The results of our experiments also support our key claims, which are as follows:

1) Our SLAM system achieves localization accuracy better or on par with state-of-the-art LiDAR odometry/SLAM approaches and is more accurate than recent implicit neural SLAM methods on various datasets using different range sensors.

2) Our method can conduct large-scale globally consistent mapping with loop closure thanks to the elastic neural point representation.

3) Our map representation is more compact than the previous counterparts and can be used to reconstruct accurate and complete meshes at an arbitrary resolution.

4) Our correspondence-free scan-to-implicit map registration and the efficient neural point indexing by voxel hashing enable our algorithm to run at the sensor frame rate on a single NVIDIA A4000 GPU.

## A. Experimental Setup

1) Datasets: We extensively test our method on various datasets summarized in Table I. For driving scenes, we test on KITTI, MulRAN, and our self-collected dataset called IPB-car. KITTI odometry dataset [14] comprises 22 sequences of LiDAR scans acquired by a Velodyne HDL64 LiDAR mounted on a car driving through Karlsruhe, Germany. Reference poses are available on sequences 00-10. MulRAN dataset [25] is collected using an Ouster OS1-64 LiDAR mounted on a car traversing the roads of Daejeon, South Korea. Note that for the MulRAN dataset, the LiDAR is blocked by the Radar sensor, resulting in a smaller FoV than KITTI. For KITTI and MulRAN, the poses measured by GNSS-INS are regarded as the reference for evaluation. We additionally collected another challenging robot car dataset in Bonn, Germany with an OS1-64 LiDAR in 2020 and an OS1-128 LiDAR in 2023. We generate the reference poses by fusing GNSS-INS, LiDAR odometry [78], loop closure constraints, and scan-to-map constraints between OS1-128 and a global map obtained by a geo-referenced terrestrial laser scanner (TLS) in a factor graph.

TABLE I  
CHARACTERISTIC OF THE DATASETS USED FOR EVALUATION
<table><tr><td>dataset</td><td>sensor</td><td>scenario</td><td> $\# \ \mathrm { s e q s . }$ </td><td># frames</td></tr><tr><td>KITTI [14]</td><td>64-beam LiDAR</td><td>outdoor, car</td><td>22</td><td>43k</td></tr><tr><td>MulRAN [25]</td><td>64-beam LiDAR</td><td>outdoor, car</td><td>9</td><td>64k</td></tr><tr><td>IPB-Car</td><td>64/128-beam LiDAR</td><td>outdoor, car</td><td>4</td><td>43k</td></tr><tr><td>Newer College [57]</td><td>64/128-beam LiDAR</td><td>outdoor/indoor, handheld</td><td>7</td><td>53k</td></tr><tr><td>Hilti-21 [17]</td><td>64-beam LiDAR</td><td>outdoor/indoor, handheld</td><td>6</td><td>15k</td></tr><tr><td>Replica [70]</td><td>synthetic RGB-D</td><td>indoor, handheld</td><td>8</td><td>16k</td></tr></table>

To further test our approach on handheld LiDARs, which have a less constant motion profile, we adopt the Newer College and Hilti-21 dataset. Newer College dataset [57] contains two longer sequences acquired by a handheld OS1-64 LiDAR and a couple of shorter sequences collected by an OS0-128 LiDAR on the campus of Oxford University, U.K. The reference poses are obtained by aligning each scan to the survey grade point cloud map measured by TLS. We also take the survey grade map as the reference model for mapping quality evaluation. The Hilti 2021 SLAM challenge dataset [17] comprises indoor sequences of offices, labs, and basements, as well as outdoor sequences of construction sites. These sequences were captured using a handheld OS0-64 LiDAR. The reference trajectories are measured by either a total station tracking system or a motion capture system.

Aside from LiDAR datasets, we adopt the Replica synthetic RGB-D dataset [70], which is a popular benchmark for recent neural RGB-D SLAM methods to show that our approach can also achieve precise pose estimation using RGB-D images.

2) Parameters and Implementation Details: We list the parameter setting of our approach in Table II. All the length-based parameters used in our method are set adaptively according to the maximum used measurement range $r _ { \mathrm { m a x } }$ of the sensor. The maximum range is $r _ { \mathrm { m a x } } = 8 0 \mathrm { m }$ for KITTI, MulRAN, and self-recorded IPB-Car datasets and $r _ { \operatorname* { m a x } } = 6 0 \mathrm { { m } }$ for Newer College, and Hilti-21 datasets. For RGB-D datasets, we have $r _ { \mathrm { m a x } } = 8 \mathrm { m }$ . Our model is implemented mainly in PyTorch [54]. For the online training of PIN map, we use Adam optimizer [28] with a learning rate of 0.01 and a batch size of 16384 for 15 iterations. Note that we train for 600 iterations at the first frame for the map initialization. To improve training efficiency, we only use 1/10th of all training samples to calculate the numerical gradient for the Ekional loss $\mathcal { L } _ { \mathrm { e i k } }$ . For local BA, the poses are represented as Lie algebra tensors with PyPose [79]. We use a learning rate of 0.01 for the neural point features, 0.0001 for the poses, and a batch size of 16 384 for 80 iterations. For PGO, we employ GTSAM [9] and optimize the pose graph using Levenberg–Marquardt optimization with a maximum of 50 iterations.

TABLE II  
HYPERPARAMETERS OF OUR APPROACH
<table><tr><td>type</td><td>symbol</td><td>value</td><td>description</td></tr><tr><td rowspan="9">PIN map</td><td rowspan="9">νp Fg  $M _ { \mathrm { m l p } } , N _ { \mathrm { m l p } }$  Nn  $K$ </td><td> $0 . 0 0 5 r _ { \mathrm { m a x } }$ </td><td>voxel hashing map resolution dimensions of neural point latent features</td></tr><tr><td>8</td><td></td></tr><tr><td> $2 , 6 4$ </td><td>MLP level and neuron count</td></tr><tr><td>56</td><td>neighborhood voxel count on each axis</td></tr><tr><td></td><td>neighborhood neural point number</td></tr><tr><td></td></tr><tr><td> $0 . 0 0 1 r _ { \operatorname* { m a x } }$  sigmoid function scale factor</td></tr><tr><td> $0 . 0 0 2 r _ { \mathrm { m a x } }$  perturbation step for numerical gradient</td></tr><tr><td></td></tr><tr><td rowspan="9"> $r _ { \mathit { l } }$   $d _ { l }$ </td><td>€  $\lambda _ { e } , \lambda _ { c } , \lambda _ { s }$ </td><td> $0 . 5 , 0 . 5 , 1 . 0$ </td></tr><tr><td></td><td> $1 . 0 5 r _ { \mathrm { m a x } }$ </td></tr><tr><td></td><td>radius of local map</td></tr><tr><td> $4 r _ { l }$ </td><td>travel distance threshold of local map</td></tr><tr><td> $0 . \dot { 0 } 0 8 r _ { \mathrm { m a x } } , 4 . 0$ </td><td>dynamic filtering threshold for SDF and stability</td></tr><tr><td> $0 . 0 0 1 r _ { \mathrm { m a x } }$ </td><td>downsample voxel size for mapping</td></tr><tr><td> $0 . 0 0 3 r _ { \mathrm { m a x } }$ </td><td>close-to-surface sampling standard deviation</td></tr><tr><td>0.3</td><td>minimum sample depth ratio</td></tr><tr><td></td><td>maximum sample range behind the surface</td></tr><tr><td rowspan="9"></td><td> $d _ { b }$ </td><td> $4 \sigma _ { s }$ </td><td></td></tr><tr><td> $N _ { s } , N _ { f } , N _ { b }$ </td><td> $4 , 2 , 1$ </td><td>surface, front, and behind free space sample count</td></tr><tr><td> $N _ { p }$ </td><td> $2 \times 1 0 ^ { 7 }$ </td><td>maximum sample count in training sample pool</td></tr><tr><td></td><td></td><td></td></tr><tr><td> $v _ { r }$   $w _ { c }$ </td><td> $0 . 0 0 7 5 r _ { \mathrm { m a x } }$   $0 . 0 1$ </td><td>downsample voxel size for registration photometric tracking weight</td></tr><tr><td> $\kappa _ { r } , \kappa _ { g }$ </td><td> $0 . 0 0 5 r _ { \mathrm { m a x } } , 0 . 1$ </td><td>GM kernel scale of residual and gradient anomaly</td></tr><tr><td></td><td></td><td></td></tr><tr><td> $F _ { \mathrm { b a } }$ </td><td>20</td><td>implicit BA frequency</td></tr><tr><td> $N _ { \mathrm { b a } }$ </td><td>50</td><td>count of poses optimized during BA</td></tr><tr><td rowspan="7">loop</td><td> $F _ { \mathrm { l o o p } }$ </td><td></td><td></td></tr><tr><td> $d _ { \mathrm { l o o p } }$ </td><td>20</td><td>PGO frequency</td></tr><tr><td></td><td> $0 . 0 2 5 r _ { \mathrm { m a x } }$ </td><td>distance threshold for local loop</td></tr><tr><td> ${ H _ { r } } , H _ { s }$ </td><td> $2 0 , 6 0$ </td><td>ring and sector resolution of polar descriptor</td></tr><tr><td> $d _ { \mathrm { m c } }$ </td><td> $0 . { \dot { 3 } }$ </td><td>cosine distance threshold for global loop</td></tr><tr><td> $V _ { a }$ </td><td>6</td><td>polar descriptor augmentation count</td></tr><tr><td> $d _ { v }$ </td><td> $0 . 0 2 r _ { \mathrm { m a x } }$ </td><td>polar descriptor augmentation translation step</td></tr></table>

## B. Localization Accuracy Evaluation

In this section, we compare the pose estimation performance of PIN-SLAM with state-of-the-art odometry/SLAM systems.

1) LiDAR Odometry Evaluation on KITTI: We first evaluate the pure odometry accuracy of our approach on the competitive KITTI odometry benchmark [14]. We denote the LiDAR odometry version of our approach as PIN-LO, which disables the loop closure detection correction module of PIN-SLAM. Since the LiDAR scans in KITTI odometry dataset are already deskewed, we disable the motion compensation for our method. We compare PIN LiDAR odometry against various LiDAR odometry systems using different map representations, such as feature points [52], [68], [82], denser voxel-downsampling points [10], [31], [78], normal distribution transformation [93], surfels [4], IMLS model [12], and triangle meshes [62], [76]. Since our approach is based on online learning of a neural implicit map, we additionally compare our method against the learning-based LiDAR odometry systems, including those using supervised learning [33], [80], [81] and Nerf-LOAM [11], which also utilizes a neural implicit map. Note that those systems based on supervised learning are trained on sequences 00-,06 and tested on sequences 07–10 while our method and Nerf-LOAM do not rely on any pretraining. We use the average relative translational error (ARTE) [14] as the metric for odometry drift evaluation. Due to the nondeterministic nature of online training in our method, the results may have slight variations with different random seeds. Therefore, we opt to report the mean and standard deviation of the metrics calculated from ten different runs with varying random seeds on the KITTI dataset. This allows a fair comparison with other methods and provides a better understanding of the stochastic nature of PIN-SLAM. As shown in Table III, PIN LiDAR odometry outperforms all the learning-based methods and most of the nonlearning-based methods. It attains an average translation error of 0.51% with a standard deviation of 0.02%, which is on par with the two open-source state-of-the-art LiDAR odometry KISS-ICP [78] and CT-ICP [10] using a voxel-downsampled point cloud map.

TABLE III  
LIDAR ODOMETRY PERFORMANCE COMPARISON ON KITTI LIDAR DATASET WITH MOTION COMPENSATED POINT CLOUD WITH THE AVERAGE RELATIVE TRANSLATIONAL DRIFTING ERROR (%)
<table><tr><td>Method</td><td>Type</td><td>00</td><td>01</td><td>02</td><td>03</td><td>04</td><td>05</td><td>06</td><td>07</td><td>08</td><td>09</td><td>10</td><td>Avg.</td><td>11-21</td></tr><tr><td>LeGO-LOAM [68]</td><td>feature points</td><td>2.17</td><td>13.4</td><td>2.17</td><td>2.34</td><td>1.27</td><td>1.28</td><td>1.06</td><td>1.12</td><td>1.99</td><td>1.97</td><td>2.21</td><td>2.49</td><td></td></tr><tr><td>F-LOAM [82]</td><td>feature points</td><td>0.78</td><td>1.43</td><td>0.92</td><td>0.86</td><td>0.71</td><td>0.57</td><td>0.65</td><td>0.63</td><td>1.12</td><td>0.77</td><td>0.79</td><td>0.84</td><td>0.72</td></tr><tr><td>MULLS [52]</td><td>feature points</td><td>0.56</td><td>0.64</td><td>0.55</td><td>0.71</td><td>0.41</td><td>0.30</td><td>0.30</td><td>0.38</td><td>0.78</td><td>0.48</td><td>0.59</td><td>0.52</td><td>0.65</td></tr><tr><td>VG-ICP [31]</td><td>dense points</td><td>2.16</td><td>2.38</td><td>0.99</td><td>0.67</td><td>0.55</td><td>0.45</td><td>0.24</td><td>0.99</td><td>1.74</td><td>0.95</td><td>0.95</td><td>1.10</td><td></td></tr><tr><td>CT-ICP [10]</td><td>dense points</td><td>0.49</td><td>0.76</td><td>0.52</td><td>0.72</td><td>0.39</td><td>0.25</td><td>0.27</td><td>0.31</td><td>0.81</td><td>0.49</td><td>0.48</td><td>0.50</td><td>0.59</td></tr><tr><td>KISS-ICP [78]</td><td>dense points</td><td>0.52</td><td>0.63</td><td>0.51</td><td>0.66</td><td>0.36</td><td>0.31</td><td>0.26</td><td>0.33</td><td>0.82</td><td>0.51</td><td>0.56</td><td>0.50</td><td>0.61</td></tr><tr><td>SuMa-LO [4]</td><td>surfels</td><td>0.73</td><td>1.71</td><td>1.06</td><td>0.66</td><td>0.38</td><td>0.50</td><td>0.42</td><td>0.39</td><td>1.02</td><td>0.48</td><td>0.71</td><td>0.73</td><td>1.39</td></tr><tr><td>Litamin-LO [93]</td><td>normal distribution</td><td>0.78</td><td>2.10</td><td>0.95</td><td>0.96</td><td>1.05</td><td>0.55</td><td>0.55</td><td>0.48</td><td>1.01</td><td>0.69</td><td>0.80</td><td>0.88</td><td></td></tr><tr><td>IMLS-SLAM [12]</td><td>implicit model</td><td>0.50</td><td>0.82</td><td>0.53</td><td>0.68</td><td>0.33</td><td>0.32</td><td>0.33</td><td>0.33</td><td>0.80</td><td>0.55</td><td>0.53</td><td>0.52</td><td>0.69</td></tr><tr><td>Puma [76] SLAMesh [62]</td><td>mesh mesh</td><td>1.46</td><td>3.38</td><td>1.86</td><td>1.60</td><td>1.63</td><td>1.20</td><td>0.88</td><td>0.72</td><td>1.44</td><td>1.51</td><td>1.38</td><td>1.55</td><td></td></tr><tr><td></td><td></td><td>0.77</td><td>1.25</td><td>0.77</td><td>0.64</td><td>0.50</td><td>0.52</td><td>0.53</td><td>0.36</td><td>0.87</td><td>0.57</td><td>0.65</td><td>0.68</td><td></td></tr><tr><td>LONet [33]</td><td>supervised</td><td>1.47†</td><td>1.36†</td><td>1.52†</td><td>1.03†</td><td>0.51†</td><td>1.04†</td><td>0.71†</td><td>1.70</td><td>2.12</td><td>1.37</td><td>1.80</td><td>1.33</td><td></td></tr><tr><td>PWCLONet [81]</td><td>supervised</td><td>0.78†</td><td>0.67†</td><td>0.86†</td><td>0.76†</td><td>0.37†</td><td>0.45†</td><td>0.27†</td><td>0.60</td><td>1.26</td><td>0.79</td><td>1.69</td><td>0.77</td><td></td></tr><tr><td>ELONet [80]</td><td>supervised</td><td>0.83†</td><td>0.55†</td><td>0.71†</td><td>0.49†</td><td>0.22†</td><td>0.34†</td><td>0.36†</td><td>0.46</td><td>1.14</td><td>0.78</td><td>0.80</td><td>0.61</td><td>1.92</td></tr><tr><td>Nerf-LOAM [11]</td><td>neural implicit</td><td>1.34</td><td>2.07</td><td></td><td>2.22</td><td>1.74</td><td>1.40</td><td></td><td>1.00</td><td></td><td>1.63</td><td>2.08</td><td>1.69</td><td></td></tr><tr><td>PIN-LO</td><td>neural implicit</td><td>0.55 (±0.02)</td><td>0.68 (±0.13)</td><td>0.54 (±0.02)</td><td>0.76 (±0.02)</td><td>0.22 (±0.02)</td><td>0.30 (±0.01)</td><td>0.35 (±0.01)</td><td>0.34 (±0.02)</td><td>0.80 (±0.02)</td><td>0.54 (±0.06)</td><td>0.50 (±0.05)</td><td>0.51 (±0.02)</td><td>0.64</td></tr></table>

The numbers with † are the results of the training set for learning-based methods and are not included in the ranking. - indicates the number is not reported. We highlight the best results in bold and the second best in underscored. For the results of our method PIN-LO. we report the mean and standard deviation (in brackets) calculated from ten runs with different random seeds.

TABLE IV  
SLAM PERFORMANCE COMPARISON (ATE RMSE [M]) ON KITTI LIDAR DATASET WITH MOTION COMPENSATED POINT CLOUD
<table><tr><td>Method</td><td>00*</td><td>01</td><td>02*</td><td>03</td><td>04</td><td>05*</td><td>06*</td><td>07*</td><td>08*</td><td>09*</td><td>10</td><td>Avg.*</td><td>Avg.</td></tr><tr><td>SuMa [4]</td><td>1.1</td><td>14.6</td><td>8.0</td><td>1.0</td><td>0.3</td><td>0.7</td><td>0.6</td><td>1.1</td><td>3.7</td><td>1.2</td><td>1.4</td><td>2.3</td><td>3.1</td></tr><tr><td>MULLS [52]</td><td>1.1</td><td>1.9</td><td>5.4</td><td>0.7</td><td>0.9</td><td>1.0</td><td>0.3</td><td>0.4</td><td>2.9</td><td>2.1</td><td>1.1</td><td>1.9</td><td>1.6</td></tr><tr><td>Litamin2 [93]</td><td>1.3</td><td>15.9</td><td>3.2</td><td>0.8</td><td>0.7</td><td>0.6</td><td>0.8</td><td>0.5</td><td>2.1</td><td>2.1</td><td>1.0</td><td>1.5</td><td>2.4</td></tr><tr><td>SC-LeGO-LOAM [27], [68]</td><td>2.3</td><td>19.7</td><td>5.3</td><td>1.6</td><td>0.4</td><td>1.2</td><td>1.0</td><td>1.5</td><td>5.9</td><td>2.0</td><td>1.7</td><td>2.7</td><td>5.3</td></tr><tr><td>HLBA [37]‡</td><td>0.8</td><td>1.9</td><td>5.1</td><td>0.6</td><td>0.8</td><td>0.4</td><td>0.2</td><td>0.3</td><td>2.7</td><td>1.3</td><td>1.1</td><td>1.5</td><td>1.4</td></tr><tr><td>SC-F-LOAM [27], [82]‡</td><td>1.3</td><td>4.7</td><td>3.3</td><td>0.7</td><td>0.3</td><td>1.2</td><td>0.4</td><td>0.5</td><td>3.0</td><td>1.3</td><td>1.6</td><td>1.6</td><td>1.7</td></tr><tr><td>SC-KISS-ICP [27], [78]‡</td><td>1.0</td><td>3.7</td><td>1.9</td><td>0.4</td><td>0.3</td><td>0.3</td><td>0.3</td><td>0.3</td><td>2.2</td><td>1.0</td><td>0.8</td><td>1.0</td><td>1.1</td></tr><tr><td>PIN-LO</td><td>5.6</td><td>4.3</td><td>9.3</td><td>0.7</td><td>0.1</td><td>1.7</td><td>0.5</td><td>0.5</td><td>3.0</td><td>1.8</td><td>0.8</td><td>3.2</td><td>2.6</td></tr><tr><td rowspan="2"></td><td>(±0.3)</td><td>(±1.7)</td><td>(±0.7)</td><td>(±0.1)</td><td>(±0.0)</td><td>(±0.1)</td><td>(±0.0)</td><td>(±0.0)</td><td>(±0.2)</td><td>(±0.4)</td><td>(±0.1)</td><td>(±0.3)</td><td>(±0.2)</td></tr><tr><td>0.8</td><td>4.3</td><td>2.1</td><td>0.7</td><td>0.1</td><td>0.3</td><td>0.4</td><td>0.3</td><td>2.1</td><td>1.2</td><td>0.8</td><td>1.0</td><td>1.2</td></tr><tr><td>PIN-SLAM</td><td>(±0.1)</td><td>(±1.7)</td><td>(±0.4)</td><td>(±0.1)</td><td>(±0.0)</td><td>(±0.0)</td><td>(±0.0)</td><td>(±0.0)</td><td>(±0.4)</td><td>(±0.1)</td><td>(±0.1)</td><td>(±0.1)</td><td>(±0.2)</td></tr></table>

\*Indicates the sequence is with loops, Avg.\* denotes the average metric on sequences with loops. We highlight the best results in bold and the second best in underscored. ‡ indicates the method conducts offline PGO. For the results of our method PIN-LO and PIN-SLAM, we report the mean and standard deviation (in brackets) calculated from ten runs with different random seeds.

Our method demonstrates superior performance compared to Nerf-LOAM [11], the sole baseline using also an implicit neural map representation. This achievement is attributed to our better SDF training in the close-to-surface free space and the more robust point-to-SDF registration using Levenberg–Marquardt optimization combined with robust kernels instead of the gradient descent used by Nerf-LOAM.

In addition, as shown in the last column of Table III, our method performs among the best LiDAR odometry/SLAM systems on the hidden sequences 11–21 of the KITTI odometry leaderboard and is the best-ranked learning-based method.

2) LiDAR SLAM Evaluation: We proceed to evaluate the full system of PIN-SLAM, including the loop closure correction module. We test PIN-SLAM quantitatively and compare it against state-of-the-art LiDAR SLAM/odometry approaches on four public datasets and one self-recorded dataset collected by different robot platforms in various scenarios. Instead of using the ARTE, we adopt the root-mean-square error (RMSE) of the absolute trajectory error (ATE) [98] with Umeyama trajectory alignment as the localization accuracy metric since it can better reflect the global consistency of the estimated pose. For comparison, we focus on methods that either disclose their achieved ATE RMSE on our utilized datasets in their papers or release open-sourced code that supports our utilized datasets.

First, on the KITTI sequence 00–10, we compare our approach with four state-of-the-art LiDAR SLAM systems enabling loop closure correction, and HLBA [37], a postprocessing method using LiDAR BA with the pose initial guess of MULLS [52]. In line with previous work [37], [93], we report the accuracy at decimeter precision. As shown in Table IV, PIN-SLAM achieves the smallest average RMSE of 1.0 m on the sequences with loops and 1.2 m on all the eleven sequences with a standard deviation of 0.2 m calculated from ten runs with different random seeds. Notably, PIN-SLAM even outperforms the post-processing approach [37] while PIN-SLAM can run online. In addition to SC-LeGO-LOAM, an open-source SLAM system combining LeGO-LOAM [68] with scan context [27], we additionally implement SC-F-LOAM and SC-KISS-ICP, which perform offline PGO using the pose of more recent LiDAR odometry systems F-LOAM and KISS-ICP with the loop closures detected by scan context. We conduct a fine ICP registration between the query and retrieved point cloud to refine the loop transformation initial guess. PIN-SLAM outperforms SC-F-LOAM and achieves comparable localization accuracy to SC-KISS-ICP. Meanwhile, PIN-SLAM can maintain a continuous SDF map online for downstream tasks such as mesh reconstruction or path planning whereas the compared methods typically construct only a sparse point cloud map. We also report the result of PIN LiDAR odometry to show the significant improvement of PIN-SLAM on trajectory global consistency, with the RMSE decreasing from 3.2 to 1.0 m on sequences with loop closures. As shown in Fig. 5, PIN-SLAM manages to correct the drift of PIN LiDAR odometry and build a globally consistent map without duplicated structures.

![](images/2024_PIN-SLAM/45cf5877f68f7a1d72c5a1bb69f2d5377cc2f58858d7dfd4614e4fe2ee79e113.jpg)  
(a)  
(b)  
Fig. 5. Comparison of (a) locally consistent mesh with duplicated structures reconstructed by PIN LiDAR odometry—PIN-LO, and (b) globally consistent mesh reconstructed by PIN-SLAM built on KITTI sequence 00 after loop closure corrections—PIN-SLAM. The estimated trajectories are overlaid on the map and colorized according to the timestamp. Details of two revisited regions are highlighted in the boxes.

TABLE V  
SLAM PERFORMANCE COMPARISON (ATE RMSE [M]) OF OUR METHOD VERSUS THE STATE-OF-THE-ART LIDAR ODOMETRY (ABOVE) AND LIDAR SLAM METHODS (BELOW) ON MULRAN LIDAR DATASET
<table><tr><td>Method</td><td>KA1</td><td>KA2</td><td>KA3</td><td>DC1</td><td>DC2</td><td>DC3</td><td>RS1</td><td>RS2</td><td>RS3</td><td>Avg.</td></tr><tr><td>F-LOAM [82] KISS-ICP [78]</td><td>49.16 20.81</td><td>35.93 13.52</td><td>38.36 14.85</td><td>36.89 20.26</td><td>26.72 12.79</td><td>31.97 13.17</td><td>91.85 14.35</td><td>104.25 32.74</td><td>119.79 75.14</td><td>59.44 24.18</td></tr><tr><td>SuMa [4] MULLS [52]</td><td>10.45 19.72</td><td>9.33 15.26</td><td>10.33 6.93</td><td>13.00 14.95</td><td>86.04 11.31</td><td>12.03 6.00</td><td>114.89 41.25</td><td>818.74 45.05</td><td>367.41 44.18</td><td>160.25 22.74</td></tr><tr><td>SC-LeGO-LOAM [27], [68] HLBA [37]‡</td><td>5.45</td><td>5.49</td><td>5.70</td><td>6.95</td><td>5.49</td><td>6.29</td><td>19.05</td><td>16.04</td><td>30.91</td><td>11.26</td></tr><tr><td></td><td>3.36</td><td>3.75</td><td>3.53</td><td>5.20</td><td>3.22</td><td>2.54</td><td>8.92</td><td></td><td></td><td>5.71</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>7.94</td><td>10.26</td><td></td></tr><tr><td>SC-F-LOAM [27], [82]‡</td><td>4.74</td><td>4.70</td><td>4.32</td><td>9.67</td><td>5.57</td><td>3.98</td><td>17.72</td><td>22.42</td><td>24.07</td><td>10.80</td></tr><tr><td>SC-KISS-ICP [27], [78]‡</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td></td><td>3.33</td><td>2.80</td><td>2.65</td><td>6.41</td><td>3.42</td><td>2.13</td><td>6.59</td><td>9.45</td><td>8.97</td><td>5.08</td></tr><tr><td>PIN-LO</td><td>29.07</td><td></td><td></td><td>22.07</td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td></td><td></td><td>24.74</td><td>23.05</td><td></td><td>12.94</td><td>23.42</td><td>45.81</td><td>54.41</td><td>44.41</td><td>31.10</td></tr><tr><td>PIN-SLAM</td><td>2.25</td><td>2.86</td><td>2.15</td><td>5.25</td><td>2.83</td><td>2.65</td><td>8.70</td><td>7.94</td><td>6.00</td><td>4.51</td></tr></table>

All sequences are with loops. The best result is in bold and the second best is underscored. † indicates the method conducts offline PGO.

TABLE VI  
SLAM PERFORMANCE COMPARISON (ATE RMSE [M]) OF THE PROPOSED METHOD VERSUS THE STATE-OF-THE-ART LIDAR ODOMETRY (ABOVE) AND LIDAR SLAM METHODS (BELOW) ON OUR SELF-COLLECTED IPB-CAR LIDAR DATASET
<table><tr><td>Method</td><td>2020-0*</td><td>2020-1*</td><td>2023-0</td><td>2023-1*</td><td>Avg.</td></tr><tr><td>F-LOAM [82]</td><td>11.75</td><td>39.92</td><td>92.48</td><td>52.70</td><td>49.21</td></tr><tr><td>KISS-ICP [78]</td><td>6.13</td><td>15.66</td><td>93.70</td><td>22.91</td><td>34.60</td></tr><tr><td>SuMa [4] MULLS [52]</td><td>7.15 10.79</td><td>117.91</td><td>100.12</td><td>57.66</td><td>70.71</td></tr><tr><td></td><td></td><td>47.07</td><td>78.10</td><td>68.63</td><td>51.15</td></tr><tr><td>PIN-LO</td><td>5.70</td><td>17.42</td><td>87.59</td><td>21.44</td><td>33.04</td></tr><tr><td>PIN-SLAM</td><td>3.51</td><td>6.12</td><td>87.59</td><td>19.27</td><td>29.12</td></tr></table>

\* means the sequence is with loops. The best result is in bold and the second best is underscored

Next, we test PIN-SLAM on longer and more challenging driving scenes, i.e., nine sequences of the MulRan dataset and four sequences of our self-recorded IPB-Car dataset. As shown in Tables V and VI, compared with the state-of-the-art LiDAR odometry, SLAM or offline optimized approaches, PIN-SLAM is able to correct the drift of PIN LiDAR odometry and achieves the best overall localization accuracy.

To further test our method on other motion profiles, we run experiments on Newer College and Hilti-21 datasets, which are mainly collected by a handheld LiDAR in both indoor and outdoor environments. We evaluate PIN-SLAM against existing LiDAR odometry/SLAM systems supporting handheld

TABLE VII  
SLAM PERFORMANCE COMPARISON (ATE RMSE [M]) OF OUR METHOD VERSUS THE STATE-OF-THE-ART LIDAR ODOMETRY (ABOVE) AND LIDAR SLAM METHODS (BELOW) ON NEWER COLLEGE HANDHELD LIDAR DATASET
<table><tr><td>Method</td><td>01</td><td>02</td><td>quad_e</td><td>math_e</td><td>ug_e</td><td>cloister</td><td>stairs</td><td>Avg.</td></tr><tr><td>F-LOAM [82]</td><td>6.74</td><td>x</td><td>0.40</td><td>0.26</td><td>0.09</td><td>7.69</td><td>x</td><td>3.04</td></tr><tr><td>KISS-ICP [78]</td><td>0.62</td><td>1.88</td><td>0.10</td><td>0.07</td><td>0.33</td><td>0.30</td><td>x</td><td>0.55</td></tr><tr><td>SuMa [4]</td><td>2.03</td><td>3.65</td><td>0.28</td><td>0.16</td><td>0.09</td><td>0.20</td><td>1.85</td><td>1.18</td></tr><tr><td>MULLS [52]</td><td>2.51</td><td>8.39</td><td>0.12</td><td>0.35</td><td>0.86</td><td>0.41</td><td>x</td><td>2.11</td></tr><tr><td>MD-SLAM [13]</td><td>-</td><td>1.74</td><td>0.25</td><td>-</td><td></td><td>0.36</td><td>0.34</td><td></td></tr><tr><td>SC-LeGO-LOAM [27], [68]</td><td>=</td><td>1.30</td><td>0.09</td><td>=</td><td>=</td><td>0.20</td><td>3.20</td><td>-</td></tr><tr><td>PIN-LO</td><td>2.08</td><td>5.32</td><td>0.09</td><td>0.09</td><td>0.07</td><td>0.19</td><td>0.07</td><td>1.13</td></tr><tr><td>PIN-SLAM</td><td>0.43</td><td>0.31</td><td>0.09</td><td>0.09</td><td>0.07</td><td>0.15</td><td>0.06</td><td>0.17</td></tr></table>

All sequences are with loops. The best result is bold and the second best is underscored. denotes failure. - indicates the number is not reported and unavailable using the open-source code.

TABLE VIII  
SLAM PERFORMANCE COMPARISON (ATE RMSE [M]) OF THE PROPOSED METHOD VERSUS THE STATE-OF-THE-ART METHODS ON HILTI-21 LIDAR DATASET
<table><tr><td>Method</td><td>rpg</td><td>lab</td><td>basel</td><td>base4</td><td>cons2</td><td>camp2</td><td>Avg.</td></tr><tr><td>F-LOAM [82]</td><td>2.78</td><td>0.18</td><td>0.91</td><td>0.29</td><td>11.52</td><td>8.95</td><td>4.10</td></tr><tr><td>KISS-ICP [78]</td><td>0.22</td><td>0.07</td><td>0.32</td><td>0.11</td><td>0.84</td><td>1.98</td><td>0.58</td></tr><tr><td>HDLGraph-SLAM [30]</td><td>0.35</td><td>0.05</td><td>0.28</td><td>0.37</td><td>0.74</td><td>0.35</td><td>0.36</td></tr><tr><td>PIN-SLAM</td><td>0.21</td><td>0.04</td><td>0.30</td><td>0.08</td><td>0.41</td><td>0.11</td><td>0.19</td></tr></table>

Best result is bold and the second best is underscored

LiDARs. As given in Tables VII and VIII, PIN-SLAM demonstrates superior performance, yielding the smallest localization error on both datasets and surpassing the compared approaches by a large margin. For the Newer College dataset, half of the compared methods fail on the challenging stairs sequence while our approach achieves the smallest RMSE of 6 cm. Note that the sequences in Hilti-21 dataset are relatively short and the LiDAR is mainly moving in a confined area without explicit loops. Therefore, we do not distinguish an odometry and a SLAM method on Hilti-21.

TABLE IX  
SLAM PERFORMANCE COMPARISON (ATE RMSE [CM]) OF THE PROPOSED METHOD VS. THE STATE-OF-THE-ART METHODS ON REPLICA RGB-D DATASET
<table><tr><td>Method</td><td>r0</td><td>rl</td><td>r2</td><td>o0</td><td>01</td><td>o2</td><td>03</td><td>04</td><td>Avg.</td></tr><tr><td>iMAP [71]</td><td>3.12</td><td>2.54</td><td>2.31</td><td>1.69</td><td>1.03</td><td>3.99</td><td>4.05</td><td>1.93</td><td>2.58</td></tr><tr><td>NICE-SLAM [101]</td><td>1.69</td><td>2.04</td><td>1.55</td><td>0.99</td><td>0.90</td><td>1.39</td><td>3.97</td><td>3.08</td><td>1.95</td></tr><tr><td>Vox-Fusion [92]</td><td>0.40</td><td>0.54</td><td>0.54</td><td>0.50</td><td>0.46</td><td>0.75</td><td>0.50</td><td>0.60</td><td>0.54</td></tr><tr><td>Co-SLAM [83]</td><td>0.60</td><td>1.13</td><td>1.43</td><td>0.55</td><td>0.50</td><td>0.46</td><td>1.40</td><td>0.77</td><td>0.86</td></tr><tr><td>ESLAM [24]</td><td>0.71</td><td>0.70</td><td>0.52</td><td>0.57</td><td>0.55</td><td>0.58</td><td>0.72</td><td>0.63</td><td>0.63</td></tr><tr><td>Point-SLAM [65]</td><td>0.61</td><td>0.41</td><td>0.37</td><td>0.38</td><td>0.48</td><td>0.54</td><td>0.72</td><td>0.63</td><td>0.52</td></tr><tr><td>PIN-SLAM</td><td>0.27</td><td>0.31</td><td>0.13</td><td>0.22</td><td>0.30</td><td>0.28</td><td>0.16</td><td>0.28</td><td>0.24</td></tr><tr><td>PIN-SLAM w/o BA</td><td>0.41</td><td>0.36</td><td>0.25</td><td>0.36</td><td>0.33</td><td>0.34</td><td>0.38</td><td>0.73</td><td>0.40</td></tr><tr><td>PIN-SLAM w/o color</td><td>0.38</td><td>0.40</td><td>0.16</td><td>0.39</td><td>0.27</td><td>0.30</td><td>0.52</td><td>0.40</td><td>0.35</td></tr></table>

Best result is bold and the second best is underscored

![](images/2024_PIN-SLAM/32c640b5c2029e72e16f2c74581bf2edc85953351053b9646ffabc79c819ae22.jpg)  
Fig. 6. Qualitative results of PIN-SLAM on Room0 sequence ofReplica RGB-D dataset. We show the mesh colorized in the normal direction and the mesh colorized by RGB built by PIN-SLAM. The estimated trajectory is shown in red.

3) RGB-D SLAM Evaluation: We also conducted experiments to show that PIN-SLAM can also work well taking as input the RGB-D images. We compare the localization (camera pose tracking) accuracy of PIN-SLAM against six state-of-the-art neural implicit RGB-D SLAM approaches on all eight sequences of the popular Replica dataset. Since the camera is moving in a single room that is always covered by the local map, we disable the loop closure detection and correction. As given in Table IX, PIN-SLAM achieves the best camera tracking accuracy on average with an RMSE of only 2.4 mm. The superior performance demonstrates that, with precise depth measurements, our point-to-SDF registration achieves greater accuracy compared to the rendering-based camera tracking utilized in most of the neural implicit SLAM approaches under comparison. As an ablation study, we show that the localization RMSE of PIN-SLAM decreases by 40% and 31% by conducting local BA and using the color information during camera tracking. Moreover, as shown in Fig. 6, PIN-SLAM can reconstruct high-fidelity colorized mesh from the neural point map built by the RGB-D SLAM.

4) Additional Challenging Scenarios: We show qualitative results on various challenging scenarios, such as a cave tunnel from Nebula dataset [60], as shown in Fig. 7. PIN-SLAM manages to build a globally consistent map using the data collected by a quadruped robot moving in the cave. PIN-SLAM is also robust to highly dynamic scenes in ETH DOALS dataset [66] as shown in Fig. 8. PIN-SLAM manages to filter the moving pedestrians and reconstruct a static mesh.

## C. Evaluation of the Loop Closure Detection Performance

We follow BEVPlace [41] to evaluate loop closure detection performance on the KITTI dataset. We use the same partition of database and query frames as used in BEVPlace. We show the loop detection recall at Top-1 in Table X and compare our approach with several nonlearning and learning-based methods. Even without any offline pretraining, our approach achieves the second-best average recall after BEVPlace, outperforming the other compared methods. We show that using the polar context descriptor of neural points in the local map can improve the loop detection recall over the original scan context [27]. Moreover, leveraging online optimized neural point features can enhance the distinctiveness of the context descriptor. Though our approach does not achieve the best performance in LiDAR place recognition, it proves to be adequate for a highly accurate SLAM system, without the need for pre-training or external map representations.

![](images/2024_PIN-SLAM/6df1958b6f74715b6b5dfa3306def9b9fabc8cd86daf66c404ddd7bce82c02d7.jpg)  
Fig. 7. Qualitative results on the Nebula dataset collected by a Spot1 robot with a 32-beam LiDAR moving back-and-forth in the Valentine Cave. We show the estimated trajectory and mesh built by PIN-SLAM on the top. We show the mapping error compared to the survey-grade map measured by a TLS on the bottom.

![](images/2024_PIN-SLAM/4fed3c83aa84e09a7c1de785d08c7a0104151efdab3452551bac9a1a5aee07dc.jpg)  
Fig. 8. Qualitative results on the hauptgebaeude and station sequences of ETH DOALS dataset. We show the LiDAR scan overlaid on the static mesh built by PIN-SLAM. The dynamic points (in red) are filtered from a LiDAR scan (in gray) online using the PIN map.

TABLE X  
LOOP CLOSURE DETECTION RECALL AT TOP-1 OF THE PROPOSED METHOD VERSUS THE STATE-OF-THE-ART NON-LEARNING (ABOVE) AND LEARNING-BASED (BELOW) METHODS ON FOUR SEQUENCES FROM KITTI DATASET
<table><tr><td>Method</td><td>00</td><td>02</td><td>05</td><td>06</td><td>Avg.</td></tr><tr><td>Scan Context [27]</td><td>89.7</td><td>73.9</td><td>77.0</td><td>86.7</td><td>81.8</td></tr><tr><td>BVMatch [40]</td><td>93.8</td><td>78.2</td><td>90.2</td><td>93.8</td><td>89.0</td></tr><tr><td>PointNetVLAD [75]</td><td>91.6</td><td>62.3</td><td>76.9</td><td>77.8</td><td>77.2</td></tr><tr><td>OverlapTransformer [42]</td><td>96.7</td><td>80.1</td><td>91.9</td><td>95.6</td><td>91.1</td></tr><tr><td>BEVPlace [41]</td><td>99.7</td><td>98.1</td><td>99.3</td><td>100.0</td><td>99.3</td></tr><tr><td>Ours (local map context)</td><td>96.0</td><td>73.0</td><td>98.3</td><td>100.0</td><td>91.8</td></tr><tr><td>Ours (full)</td><td>99.4</td><td>78.4</td><td>100.0</td><td>100.0</td><td>94.5</td></tr></table>

The results of the compared methods are reported by BEVPlace [41]. The best result is in bold and the second best is underscored.

TABLE XI  
3-D RECONSTRUCTION QUALITY OF DIFFERENT METHODS ON QUAD AND MATH INSTITUTE SEQUENCE OF THE NEWER COLLEGE DATASET [57]
<table><tr><td rowspan="2">Method</td><td rowspan="2">Pose</td><td colspan="4">Quad</td><td colspan="4">Math Institute</td></tr><tr><td>Map. Acc. ↓</td><td>Map. Comp. ↓</td><td>C-11. ↓</td><td>F-score ↑</td><td>Map. Acc. ↓</td><td>Map. Comp. ↓</td><td>C-11. ↓</td><td>F-score ↑</td></tr><tr><td>VDB-Fusion [77]</td><td></td><td>14.03</td><td>25.46</td><td>19.75</td><td>69.50</td><td>15.21</td><td>28.66</td><td>21.94</td><td>63.35</td></tr><tr><td>SHINE [100]</td><td>KISS-ICP [78]</td><td>14.87</td><td>20.02</td><td>17.45</td><td>68.85</td><td>14.46</td><td>34.03</td><td>24.24</td><td>64.38</td></tr><tr><td>NKSR [20]</td><td></td><td>15.67</td><td>36.87</td><td>26.27</td><td>58.57</td><td>15.11</td><td>27.10</td><td>21.11</td><td>65.08</td></tr><tr><td>Puma [76]</td><td></td><td>15.30</td><td>71.91</td><td>43.60</td><td>57.27</td><td>15.81</td><td>46.00</td><td>30.91</td><td>54.95</td></tr><tr><td>SLAMesh [62]</td><td rowspan="4">Own Odometry</td><td>19.21</td><td>48.83</td><td>34.02</td><td>45.24</td><td>12.80</td><td>23.50</td><td>18.16</td><td>75.17</td></tr><tr><td></td><td>12.89</td><td>22.21</td><td>17.55</td><td>74.37</td><td></td><td></td><td></td><td></td></tr><tr><td>Nerf-LOAM [11]</td><td>11.55</td><td>15.25</td><td>13.40</td><td>82.08</td><td>13.70</td><td>21.91</td><td>17.80</td><td>75.49</td></tr><tr><td>PIN-SLAM</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr></table>

The reference model is measured by terrestrial laser scanning with mm-level accuracy. We report completion, accuracy, and chamfer-L1 in cm as well as F-Score in % calculated with a 20 cm error threshold. We highlight the best results in bold and the second best in underscored. - indicates the number is not reported and unavailable using the open-source code.

![](images/2024_PIN-SLAM/4b6654df3147db6b63625082b6bb7e22fa7edfda887ce18550675efb2c02e801.jpg)  
Fig. 9. Globally consistent mesh reconstructed by PIN-SLAM on Newer College dataset with multiple loops. The estimated trajectories are overlaid on the map and colorized according to the timestamp. Details of several revisited regions are highlighted in the dashed boxes.

## D. Evaluation of the Mapping Performance

1) 3-D Reconstruction Quality: In this section, we show the mapping quality of PIN-SLAM in terms of the 3-D reconstruction quality of the resulting mesh in Table XI. The mesh is reconstructed from the SDF queried at the fixed-size grid using the marching cubes algorithm [39]. For the quantitative evaluation, we use the commonly used 3-D reconstruction metrics [43] calculated between the reconstructed and reference 3-D model, namely, accuracy, completeness, Chamfer-L1 distance, and F-score. We select two scenes from the Newer College dataset [57], namely, Quad from the 02\_long sequence and Math Institute from the math\_easy sequence. Both scenes have the survey-grade point cloud map measured by TLS available, which is taken as the reference model for evaluation. We compare PIN-SLAM against three state-of-the-art LiDAR SLAM approaches that can reconstruct dense 3-D meshes, namely, Puma [76] based on Poisson surface reconstruction, SLAMesh [62] utilizing Gaussian process reconstruction, and Nerf-LOAM [11] employing implicit neural representation like us. We additionally include three state-of-the-art “mapping with known poses” methods tailored for LiDAR data, namely, VDB-Fusion [77] based on TSDF integration, SHINE-Mapping [100] based on implicit neural scene reconstruction, and the datadriven method NKSR [20] based on neural kernel field. We use the pose estimation of KISS-ICP [78] as the pose input to these three mapping methods. Note that KISS-ICP and PIN-SLAM achieve similar localization RMSE of about 10 cm in both scenes. We use the same voxel size of 20 cm for mesh reconstruction for the compared methods. Table XI lists the obtained results. As can be seen, PIN-SLAM achieves the best mapping quality in terms of completeness, Chamfer distance and F-score in both scenes, indicating that PIN-SLAM can achieve more accurate and more complete reconstruction of the environment than the compared methods. The superior performance of PIN-SLAM owes to the continuous implicit neural representation and the more consistent pose estimation.

2) Map Consistency: Since the global localization accuracy can reflect the mapping consistency, we qualitatively depict the globally consistent mesh reconstructed from PIN map built by PIN-SLAM on Newer College in Fig. 9, KITTI in Fig. 10, MulRAN in Fig. 11, and IPB-Car dataset in Fig. 12. Several regions with multiple loop closures are highlighted.

## E. Ablation Studies

In this section, we conduct ablation studies to validate our design choices. We use the average relative translational drifting error (ARTE) on KITTI dataset as the metrics and the same random seed to carry out the studies. We show ARTE obtained on KITTI sequences 00–10 with different design choices for mapping and odometry in Tables XII and XIII, respectively. For the mapping losses, we find out that the BCE loss used by our method is more suitable for SDF training than L1 or L2 loss. Besides, Ekional loss is necessary for the odometry and a weight λ<sub>e</sub> of 0.5 is preferable. To calculate the Ekional loss, using the numerical gradient rather than the analytical gradient leads to a smooth and regular SDF in the free space, thus improving localization accuracy. A perturbation step  of8 cm for numerical gradient calculation is preferable. For odometry estimation, we verify that the usage of GM robust kernel for SDF residual and SDF gradient anomaly are both beneficial to localization accuracy. The odometry loses track when using the numerical gradient because it is often less accurate than the analytical one once the SDF has been fitted. Besides, as shown in Table XIV, we find out that a too-large or too-small neural point resolution $v _ { p }$ would lead to suboptimal odometry accuracy. Setting $v _ { p }$ as 0.4 m, corresponding to our adaptive setting $0 . 0 0 5 r _ { \mathrm { m a x } }$ , leads to the smallest ARTE on all three tested sequences.

![](images/2024_PIN-SLAM/2e6d00a14fc71c52518a8f1f384cce3955808fe6ef2b7a3a9f39e02daac7bb8b.jpg)  
Fig. 10. Globally consistent mesh reconstructed by PIN-SLAM on KITTI dataset with multiple loops. The estimated trajectories are overlaid on the map and colorized according to the timestamp. Details of several revisited regions are highlighted in the dashed boxes.

![](images/2024_PIN-SLAM/bc74cbc4b660ebd9bcc7311cb4aca76a6192cbc6aecc8a889104a50053f7f71c.jpg)  
Fig. 11. Globally consistent mesh reconstructed by PIN-SLAM on MulRAN dataset with multiple loops. The estimated trajectories are overlaid on the map and colorized according to the timestamp. Details of several revisited regions are highlighted in the dashed boxes.

TABLE XII  
ABLATION STUDY: DESIGN CHOICES OF MAP TRAINING LOSSES
<table><tr><td>SDF loss</td><td>Ekional loss λe</td><td>num. grad.</td><td>€ [m]</td><td>ARTE [%] ↓</td></tr><tr><td>BCE</td><td>0.5</td><td></td><td>1</td><td>0.57</td></tr><tr><td>BCE</td><td>1.0</td><td>√</td><td>0.08</td><td>0.59</td></tr><tr><td>BCE</td><td>0.2</td><td>√</td><td>0.08</td><td>0.55</td></tr><tr><td>BCE</td><td>0.0</td><td>-</td><td></td><td>x</td></tr><tr><td>L1</td><td>0.5</td><td>√</td><td>0.08</td><td>1.15</td></tr><tr><td>L2</td><td>0.5</td><td>√</td><td>0.08</td><td>x</td></tr><tr><td>BCE</td><td>0.5</td><td>√</td><td>0.04</td><td>0.54</td></tr><tr><td>BCE</td><td>0.5</td><td>√</td><td>0.16</td><td>1.48</td></tr><tr><td>BCE</td><td>0.5</td><td>√</td><td>0.08</td><td>0.50</td></tr></table>

ARTE represents the average relative translational drifting errors on KITTI sequence 00–10. X denotes failure.  
The best results are highlighted in bold.

TABLE XIII  
ABLATION STUDY: DESIGN CHOICES OF ODOMETRY ESTIMATION
<table><tr><td>GM kernel  $w _ { r }$ </td><td>GM kernel  $w _ { g }$ </td><td>anl. grad. |</td><td>ARTE [%] ↓</td></tr><tr><td>x</td><td>x</td><td>√</td><td>0.56</td></tr><tr><td>√</td><td>x</td><td>√</td><td>0.52</td></tr><tr><td>x</td><td>√</td><td>√</td><td>0.55</td></tr><tr><td>√</td><td>√</td><td>x</td><td>x</td></tr><tr><td>√</td><td>√</td><td>√</td><td>一 0.50</td></tr></table>

ARTE represents the average relative translational drifting errors on KITTI sequence 00–10.  denotes failure.  
The best results are highlighted in bold.

![](images/2024_PIN-SLAM/0c9b38a88f4f34725dcc948e67a4df41226c0f1acc732d8e80cfcebf8599975a.jpg)  
Fig. 12. Qualitative results on the IPB-Car dataset. On the left, we show the globally consistent mesh reconstructed by PIN-SLAM. On the right, we show two examples of the colorized mesh reconstructed using our calibrated LiDAR-camera multisensor platform.

TABLE XIV  
ABLATION STUDY: AVERAGE RELATIVE TRANSLATIONAL DRIFTING ERROR [%] WITH REGARDS TO DIFFERENT NEURAL POINT RESOLUTIONS $v _ { p }$
<table><tr><td rowspan="2">Sequence</td><td colspan="5">Neural point resolution  $v _ { p }$ </td></tr><tr><td>1.0m</td><td>0.8 m</td><td>0.6m</td><td>0.4m</td><td>0.2 m</td></tr><tr><td>KITTI 00</td><td>0.98</td><td>0.75</td><td>0.85</td><td>0.55</td><td>1.06</td></tr><tr><td>KITTI 05</td><td>0.36</td><td>0.56</td><td>0.45</td><td>0.29</td><td>0.50</td></tr><tr><td>KITTI 08</td><td>0.96</td><td>0.88</td><td>0.85</td><td>0.83</td><td>1.05</td></tr></table>

The best results are highlighted in bold

TABLE XV  
MEMORY CONSUMPTION IN MB AND THE COMPRESSION RATIO WITH REGARDS TO THE RAW POINT CLOUD OF DIFFERENT MAP REPRESENTATIONS
<table><tr><td>Representation</td><td>KITTI 00</td><td>KITTI 05</td><td>KITTI 08</td><td>NCD 02</td></tr><tr><td>Raw point cloud</td><td>13624.2</td><td>8284.7</td><td>12214.1</td><td>26559.0</td></tr><tr><td>Surfel map</td><td>887.7</td><td>512.6</td><td>835.7</td><td>79.0</td></tr><tr><td>Mesh map</td><td>2032.9</td><td>1317.4</td><td>1894.1</td><td>1503.7</td></tr><tr><td>VDB TSDF map</td><td>748.1</td><td>434.6</td><td>958.6</td><td>462.5</td></tr><tr><td>SHINE map</td><td>160.6</td><td>114.2</td><td>189.9</td><td>117.1</td></tr><tr><td>PIN-LO map</td><td>137.3</td><td>89.6</td><td>162.4</td><td>110.3</td></tr><tr><td>PIN-SLAM map</td><td>102.1 (0.7%)</td><td>66.3 (0.8%)</td><td>138.8 (1.1%)</td><td>76.8 (0.3%)</td></tr></table>

The best results are highlighted in bold

## F. Memory and Computational Resources

1) Memory Requirement: We report the map memory consumption of PIN-SLAM on four representative sequences from KITTI and Newer College dataset in Table XV. We compare the proposed PIN map with several common map representations: surfel map used by SuMa [4], mesh map used by Puma [76], VDB TSDF map used by VDB Fusion [77], and the gridbased sparse hierarchical implicit neural (SHINE) map used by SHINE-Mapping [100] and Nerf-LOAM [11]. We also provide the raw point cloud memory consumption as a reference. For the VDB TSDF map, we use a voxel size of 20 cm. For SHINE map and PIN map, we use the same resolution of 40 cm for the local latent feature grid and neural points. With the continuous neural SDF queried from SHINE or PIN map, we can reconstruct the mesh with a voxel size of 20 cm with a similar quality as the mesh reconstructed from the discrete VDB TSDF map. Besides, for VDB TSDF map and SHINE map, we take the odometry estimation of KISS-ICP [78] as their pose. As shown in Table XV, our PIN map is the most compact representation among the compared methods with a compression ratio of about 0.7% on average. Compared to SHINE map, PIN map using the same resolution and feature dimensions is about 15% more memory efficient even with the additional storage of neural point coordinates and orientations. This is because PIN map does not have a hierarchical tree structure like SHINE map and the neural points are only allocated close to the surface. Notably, compared with PIN LiDAR odometry, the memory consumption of PIN map decreases by about 20% due to the loop correction and map update of PIN-SLAM. The elastic PIN map can deform with the corrected pose and eliminate duplicated neural points representing the same place. This is not possible for the grid-based VDB or SHINE map without the submap scheme or offline remapping.

2) Computational Requirement: In Table XVI, we report the average runtime of PIN-SLAM running on a single NVIDIA

TABLE XVI  
COMPARISON OF THE AVERAGE OPERATION SPEED AND THE LOCALIZATIONERROR ON KITTI SEQUENCE 00–10 USING A SINGLE NVIDIA A4000 GPU
<table><tr><td>Method</td><td>Time per frame [s] ↓</td><td>FPS [Hz] ↑</td><td>ARTE [%] ↓</td></tr><tr><td>PIN-SLAM (full)</td><td>0.14</td><td>7.1</td><td>0.50</td></tr><tr><td>PIN-SLAM (light)</td><td>0.09</td><td>11.3</td><td>0.56</td></tr><tr><td>Nerf-LOAM [11]</td><td>4.43</td><td>0.2</td><td>1.69</td></tr></table>

ARTE represents the average relative translational error.  
The best results are highlighted in bold

![](images/2024_PIN-SLAM/49b6fa82447bc6140a5dc26a221d875634f099c813cf4e479c3437bb3575eb46.jpg)  
Fig. 13. Processing time for each submodule of the light version of PIN-SLAM on KITTI sequence 00.

![](images/2024_PIN-SLAM/22971d1e9b02b080050b9adf1075c5a941fbb3d464ba21bb08e28420a3629163.jpg)  
Fig. 14. Example of the metric-semantic map built by PIN-SLAM on KITTI sequence 00 using the semantic labels of SemanticKITTI.

Quadro A4000 GPU over the whole 11 sequences of the KITTI dataset. We show the results of two versions of PIN-SLAM, the full version as used in the localization evaluation above and the light version which takes a fewer number of iterations during mapping and odometry. The light version of PIN-SLAM works 38% faster at the cost of a minor degradation (10%) of the localization accuracy. It can operate efficiently at approximately 11 Hz, aligning with the typical sensor frame rate. In Fig. 13, we report the average processing time for each submodule of PIN-SLAM (light). Odometry and map optimization are the two most time-consuming parts, each taking up about 40% of the total run time. When loops are detected and PGO is conducted, the run time per frame occasionally exceeds 200 ms (as the spikes in the figure). Notably, the processing time remains consistent as the frame count increases. In contrast, the only implicit neural LiDAR odometry baseline, Nerf-LOAM [11] takes more than 4 s per frame, which is 30× slower than PIN-SLAM. Moreover, we observe that Nerf-LOAM’s processing time increases drastically with an increasing number of scans.

## G. Extension on Semantic Mapping

As a proof of concept, we illustrate briefly that our approach is capable of conducting metric-semantic SLAM with global consistency. As shown in Fig. 14, PIN-SLAM manages to build a metric-semantic map using the SemanticKITTI [3] dataset by integrating the semantic labels as described in Section IV. Note that we only need to add another shallow MLP for the semantic querying. Currently, the semantics do not play a role in odometry estimation and loop closure detection. Considering this is outside the scope of this work but a possible future extension.

## VI. CONCLUSION

In this article, we presented PIN-SLAM, a novel LiDAR SLAM approach to perform globally consistent mapping using a PIN map representation. Our approach alternates between the online incremental learning of the local implicit map and the odometry estimation using a correspondence-free point-toimplicit map registration. We exploit sparse neural points as local feature embeddings, which are inherently elastic and deformable throughout the global pose adjustment when correcting a loop closure. This enables us to effectively maintain the global consistency of both the neural points and the underlying implicit map. We implemented and evaluated our approach on various datasets, provided comparisons to other existing techniques and supported all claims made in this article. Extensive experiments suggest PIN-SLAM achieves better or on-par localization accuracy than previous methods. In addition, it builds a more consistent and compact implicit map, which can be reconstructed into more accurate and complete meshes. Besides, PIN-SLAM can run at the sensor frame rate using a moderate GPU.

Limitations andfuture work: Future work may use additionally an IMU for a more robust, accurate and efficient SLAM system. Besides, we are currently using a fixed resolution of neural points. We can enhance the reconstruction quality by adaptively distributing and dynamically moving neural points throughout the scene like previous works [34], [65], and use a data structure, such as i-Kdtree [90], enabling efficient neighbor search without using voxel structures. Another direction can be using semantic information to further enhance odometry estimation and loop closure detection.

## ACKNOWLEDGMENT

The authors would like to thanks Ignacio Vizzo, Benedikt Mersch, Yibin Wu, Tiziano Guadagnino, and Haofei Kuang for the fruitful discussions and Rodrigo Marcuzzi for the colorized point cloud.

## REFERENCES

[1] J. Abou-Chakra, F. Dayoub, and N. Sünderhauf, “ParticleNeRF: A particle-based encoding for online neural radiance fields,” in Proc. IEEE Winter Conf. Appl. Comput. Vis., 2024, pp. 5963–5972.

[2] D. Azinovi´c, R. Martin-Brualla, D. B. Goldman, M. Nießner, and J. Thies, “Neural RGB-D surface reconstruction,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 6280–6291.

[3] J. Behley et al., “SemanticKITTI: A dataset for semantic scene understanding of LiDAR sequences,” in Proc. IEEE/CVF Intl. Conf. Comput. Vis., 2019, pp. 9296–9306.

[4] J. Behley and C. Stachniss, “Efficient surfel-based SLAM using 3D laser range data in urban environments,” in Proc. Robotics: Sci. Syst., 2018, pp. 59–68.

[5] P. Besl and N. McKay, “A method for registration of 3D shapes,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 14, no. 2, pp. 239–256, Feb. 1992.

[6] A. Chen, Z. Xu, A. Geiger, J. Yu, and H. Su, “TensoRF: Tensorial radiance fields,” in Proc. Eur. Conf. Comput. Vis., 2022, pp. 333–350.

[7] X. Chen, A. Milioto, E. Palazzolo, P. Giguère, J. Behley, and C. Stachniss, “SuMa : Efficient LiDAR-based semantic SLAM,” in Proc. IEEE/RSJInt Conf. Intell. Robots Syst., 2019, pp. 4530–4537.

[8] A. Dai, M. Nießner, M. Zollhöfer, S. Izadi, and C. Theobalt, “Bundle-Fusion: Real-time globally consistent 3D reconstruction using online surface re-integration,” ACM Trans. Graph., vol. 36, no. 3, pp. 1–18, 2017.

[9] F. Dellaert, “Factor graphs and GTSAM: A hands-on introduction,” Georgia Inst. Technol., Atlanta, GA, USA, Tech. Rep. GT-RIM-CP&R-2012-002, 2012.

[10] P. Dellenbach, J. Deschaud, B. Jacquet, and F. Goulette, “CT-ICP realtime elastic LiDAR odometry with loop closure,” in Proc. IEEE Int. Conf. Robot. Autom., 2022, pp. 5580–5586.

[11] J. Deng et al., “NeRF-LOAM: Neural implicit representation for largescale incremental lidar odometry and mapping,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2023, pp. 8184–8193.

[12] J. Deschaud, “IMLS-SLAM: Scan-to-model matching based on 3D data,” in Proc. IEEE Int. Conf. Robot. Autom., 2018, pp. 2480–2485.

[13] L. Di Giammarino, L. Brizi, T. Guadagnino, C. Stachniss, and G. Grisetti, “MD-SLAM: Multi-cue direct slam,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2022, pp. 11047–1105.

[14] A. Geiger, P. Lenz, and R. Urtasun, “Are we ready for Autonomous Driving? The KITTI vision benchmark suite,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2012, pp. 3354–3361.

[15] G. Grisetti, C. Stachniss, and W. Burgard, “Improved techniques for grid mapping with Rao-Blackwellized particle filters,” IEEE Trans. Robot., vol. 23, no. 1, pp. 34–46, Feb. 2007.

[16] A. Gropp, L. Yariv, N. Haim, M. Atzmon, and Y. Lipman, “Implicit geometric regularization for learning shapes,” in Proc. Int. Conf. Mach. Learn., 2020, Art. no. 355.

[17] M. Helmberger, K. Morin, B. Berner, N. Kumar, G. Cioffi, and D. Scaramuzza, “The Hilti SLAM challenge dataset,” IEEE Robot. Automat. Lett., vol. 7, no. 3, pp. 7518–7525, Jul. 2022.

[18] W. Hess, D. Kohler, H. Rapp, and D. Andor, “Real-time loop closure in 2D LIDAR SLAM,” in Proc. IEEE Int. Conf. Robot. Autom., 2016, pp. 1271–1278.

[19] A. Hornung, K. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, “OctoMap: An efficient probabilistic 3D mapping framework based on octrees,” Auton. Robots, vol. 34, no. 3, pp. 189–206, 2013.

[20] J. Huang, Z. Gojcic, M. Atzmon, O. Litany, S. Fidler, and F. Williams, “Neural kernel surface reconstruction,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 4369–4379.

[21] J. Huang, S. S. Huang, H. Song, and S. M. Hu, “Di-fusion: Online implicit 3D reconstruction with deep priors,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 8928–8937.

[22] S. Huang et al., “Neural LiDAR fields for novel view synthesis,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2023, pp. 18236–18246.

[23] S. Isaacson, P. C. Kung, M. Ramanagopal, R. Vasudevan, and K. A. Skinner, “LONER: LiDAR only neural representations for real-time slam,” IEEE Robot. Autom. Lett., vol. 8, no. 12, pp. 8042–8049, Dec. 2023.

[24] M. M. Johari, C. Carta, and F. Fleuret, “ESLAM: Efficient dense slam system based on hybrid representation of signed distance fields,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 17408– 17419.

[25] G. Kim, Y. Park, Y. Cho, J. Jeong, and A. Kim, “MulRan: Multimodal range dataset for urban place recognition,” in Proc. IEEE Intl. Conf. Robot. Autom., 2020, pp. 6246–6253.

[26] G. Kim, S. Choi, and A. Kim, “Scan context : Structural place recognition robust to rotation and lateral variations in urban environments,” IEEE Trans. Robot., vol. 38, no. 3, pp. 1856–1874, Jun. 2022.

[27] G. Kim and A. Kim, “Scan context: Egocentric spatial descriptor for place recognition within 3D point cloud map,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 4802–4809.

[28] D. Kingma and J. Ba, “Adam: A method for stochastic optimization,” in Proc. Int. Conf. Learn. Representations, 2015, pp. 1–13.

[29] S. Kohlbrecher, S. O. Von, J. Meyer, and U. Klingauf, “A flexible and scalable SLAM system with full 3D motion estimation,” in Proc. IEEE Int. Symp. Saf. Secur. Rescue Robot., 2011, pp. 155–160.

[30] K. Koide, J. Miura, and E. Menegatti, “A portable three-dimensional LiDAR-based system for long-term and wide-area people behavior measurement,” Intl. J. Adv. Robotic Syst., vol. 16, no. 2, pp. 1–20, 2019.

[31] K. Koide, M. Yokozuka, S. Oishi, and A. Banno, “Voxelized GICP for fast and accurate 3D point cloud registration,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 11054–11059.

[32] H. Kuang, X. Chen, T. Guadagnino, N. Zimmerman, J. Behley, and C. Stachniss, “IR-MCL: Implicit representation-based online global localization,” IEEE Robot. Autom. Lett., vol. 8, no. 3, pp. 1627–1634, Mar. 2023.

[33] Q. Li et al., “LO-Net: Deep real-time LiDAR odometry,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 8473–8482.

[34] T. Li, X. Wen, Y. S. Liu, H. Su, and Z. Han, “Learning deep implicit functions for 3D shapes with dynamic code clouds,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 12840–12850.

[35] Z. Li et al., “Neuralangelo: High-fidelity neural surface reconstruction,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 8456–8465.

[36] J. Lin, F. Zhang, and A. Loam\_livox, “Robust LiDAR odemetry and mapping LOAM package for livox LiDAR,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 3126–3131.

[37] X. Liu, Z. Liu, F. Kong, and F. Zhang, “Large-scale LiDAR consistent mapping using hierarchical LiDAR bundle adjustment,” IEEE Robot. Autom. Lett., vol. 8, no. 3, pp. 1523–1530, Mar. 2023.

[38] Z. Liu, X. Liu, and F. Zhang, “Efficient and consistent bundle adjustment on LiFAR point clouds,” IEEE Trans. Robot., vol. 39, no. 6, pp. 4366–4386, Dec. 2023.

[39] W. Lorensen and H. Cline, “Marching cubes: A high resolution 3D surface construction algorithm,” in Proc. Int. Conf. Comput. Graph. Interactive Techn., 1987, pp. 163–169.

[40] L. Luo, S. Y. Cao, B. Han, H. L. Shen, and J. Li, “BVMatch: LiDAR-based place recognition using birds-eye view images,” IEEE Robot. Autom. Lett., vol. 6, no. 3, pp. 6076–6083, Jul. 2021.

[41] L. Luo et al., “BEVPlace: Learning LiDAR-based place recognition using bird’s eye view images,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2023, pp. 8666–8675.

[42] J. Ma, J. Zhang, J. Xu, R. Ai, W. Gu, and X. Chen, “OverlapTransformer: An efficient and yaw-angle-invariant transformer network for LiDAR-based place recognition,” IEEE Robot. Autom. Lett., vol. 7, no. 3, pp. 6958–6965, Jul. 2022.

[43] L. Mescheder, M. Oechsle, M. Niemeyer, S. Nowozin, and A. Geiger, “Occupancy networks: Learning 3D reconstruction in function space,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 4460–4470.

[44] B. Mildenhall, P. Srinivasan, M. Tancik, J. Barron, R. Ramamoorthi, and R. Ng, “NeRF: Representing scenes as neural radiance fields for view synthesis,” in Proc. Europ. Conf. Comput. Vis., 2020, pp. 405–421.

[45] A. Milioto, I. Vizzo, J. Behley, and C. Stachniss, “RangeNet : Fast and accurate LiDAR semantic segmentation,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 4213–4220.

[46] M. Montemerlo, S. Thrun, D. Koller, and B. Wegbreit, “FastSLAM: A. factored solution to the simultaneous localization and mapping problem,” in Proc. Conf. Advancements Artif. Intell., 2002, pp. 593–598.

[47] T. Müller, A. Evans, C. Schied, and A. Keller, “Instant neural graphics primitives with a multiresolution hash encoding,” ACM Trans. Graph., vol. 41, no. 4, pp. 102:1–102:15, 2022.

[48] R. A. Newcombe et al., “KinectFusion: Real-time dense surface mapping and tracking,” in Proc. Int. Symp. Mixed Augmented Reality, 2011, pp. 127–136.

[49] M. Nießner, M. Zollhöfer, S. Izadi, and M. Stamminger, “Real-time 3D reconstruction at scale using voxel hashing,” ACM Trans. Graph., vol. 32, no. 6, pp. 1–11, 2013.

[50] H. Oleynikova, Z. Taylor, M. Fehr, R. Siegwar, and J. Nieto, “Voxblox: Incremental 3D Euclidean signed distance fields for on-board MAV planning,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2017, pp. 1366–1373.

[51] J. Ortiz et al., “iSDF: Real-time neural signed distance fields for robot perception,” in Proc. Robotics: Sci. Syst., 2022, pp. 30–39.

[52] Y. Pan, P. Xiao, Y. He, Z. Shao, and Z. Li, “MULLS: Versatile LiDAR SLAM via multi-metric linear least square,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 11633–1164.

[53] J. J. Park, P. Florence, J. Straub, R. Newcombe, and S. Lovegrove, “DeepSDF: Learning continuous signed distance functions for shape representation,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 165–174.

[54] A. Paszke et al., “PyTorch: An imperative style, high-performance deep learning library,” in Proc. Conf. Neural Inf. Process. Syst., 2019, pp. 8026–8037.

[55] S. Peng, M. Niemeyer, L. Mescheder, M. Pollefeys, and A. Geiger, “Convolutional occupancy networks,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 523–540.

[56] C. Qin, H. Ye, C. E. Pranata, J. Han, S. Zhang, and M. Liu, “LINS: A LiDAR-inertial state estimator for robust and efficient navigation,” in Proc. IEEE Int. Conf. Robot. Autom., 2020, pp. 8899–8906.

[57] M. Ramezani, Y. Wang, M. Camurri, D. Wisth, M. Mattamala, and M. Fallon, “The newer college dataset: Handheld LiDAR, inertial and vision with ground truth,” in Proc. IEEE/RSJInt. Conf. Intell. Robots Syst., 2020, pp. 4353–4360.

[58] F. Ramos and L. Ott, “Hilbert maps: Scalable continuous occupancy mapping with stochastic gradient descent,” Int. J. Robot. Res., vol. 35, no. 14, pp. 1717–1730, 2016.

[59] V. Reijgwart, A. Millane, H. Oleynikova, R. Siegwart, C. Cadena, and J. Nieto, “Voxgraph: Globally consistent, volumetric mapping using signed distance function submaps,” IEEE Robot. Autom. Lett., vol. 5, no. 1, pp. 227–234, Jan. 2020.

[60] A. Reinke et al., “LOCUS 2.0: Robust and computationally efficient LiDAR odometry for real-time 3D mapping,” IEEE Robot. Autom. Lett., vol. 7, no. 4, pp. 9043–9050, Oct. 2022.

[61] R. A. Rosu and S. Behnke, “PermutoSDF: Fast multi-view reconstruction with implicit surfaces using permutohedral lattices,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 8466–8475.

[62] J. Ruan, B. Li, Y. Wang, and Y. Sun, “Slamesh: Real-time LiDAR simultaneous localization and meshing,” in Proc. IEEE Int. Conf. Robot. Autom., 2023, pp. 3546–3552.

[63] J. Ruan, B. Li, Y. Wang, and Z. Fang, “GP-SLAM : Real-time 3D LiDAR SLAM based on improved regionalized Gaussian process map reconstruction,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5171–5178.

[64] S. Rusinkiewicz and M. Levoy, “Efficient variants of the ICP algorithm,” in Proc. Int. Conf. 3-D Digit. Imag. Model., 2001, pp. 145–152.

[65] E. Sandström, Y. Li, L. Van Gool, and M. R. Oswald, “Point-SLAM: Dense neural point cloud-based SLAM,” in Proc. IEEE/CVF Intl. Conf. Comput. Vis., 2023, pp. 18433–18444.

[66] L. Schmid, O. Andersson, A. Sulser, P. Pfreundschuh, and R. Siegwart, “Dynablox: Real-time detection of diverse dynamic objects in complex environments,” IEEE Robot. Autom. Lett., vol. 8, no. 10, pp. 6259–6266, Oct. 2023.

[67] T. Shan et al., “LIO-SAM: Tightly-coupled LiDAR inertial odometry via smoothing and mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5135–5142.

[68] T. Shan and B. Englot, “LeGO-LOAM: Lightweight and ground-optimized LiDAR odometry and mapping on variable terrain,” in Proc. IEEE/RSJ Intl. Conf. Intell. Robots Syst., 2018, pp. 4758–4765.

[69] C. Stachniss, G. Grisetti, and W. Burgard, “Information gain-based exploration using Rao-Blackwellized particle filters,” in Proc. Robotics: Sci. Syst., 2005, pp. 65–72.

[70] J. Straub et al., “The replica dataset: A digital replica of indoor spaces,” 2019, arXiv:1906.05797.

[71] E. Sucar, S. Liu, J. Ortiz, and A. J. Davison, “iMAP: Implicit mapping and positioning in real-time,” in Proc. IEEE/CVF Intl. Conf. Comput. Vis., 2021, pp. 6229–6238.

[72] T. Takikawa et al., “Neural geometric level of detail: Real-time rendering with implicit 3D shapes,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 11358–11367.

[73] Y. Tang, J. Zhang, Z. Yu, H. Wang, and K. Xu, “MIPS-fusion: Multi-implicit-submaps for scalable and robust online neural RGB-D reconstruction,” ACM Trans. Graph., vol. 42, no. 6, 2023, Art. no. 246.

[74] S. Thrun, D. Fox, W. Burgard, and F. Dellaert, “Robust Monte Carlo localization for mobile robots,”Artif. Intell., vol. 128, no. 1/2, pp. 99–141, 2001.

[75] A. Uy and G. Lee, “PointNetVLAD: Deep point cloud based retrieval for large-scale place recognition,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2018, pp. 4470–4479.

[76] I. Vizzo, X. Chen, N. Chebrolu, J. Behley, and C. Stachniss, “Poisson surface reconstruction for LiDAR odometry and mapping,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 5624–5630.

[77] I. Vizzo, T. Guadagnino, J. Behley, and C. Stachniss, “VDBFusion: Flexible and efficient TSDF integration of range sensor data,” Sensors, vol. 22, no. 3, pp 1296, 2022.

[78] I. Vizzo, T. Guadagnino, B. Mersch, L. Wiesmann, J. Behley, and C. Stachniss, “KISS-ICP: In defense of point-to-point ICP–simple, accurate, and robust registration if done the right way,” IEEE Robot. Autom. Lett., vol. 8, no. 2, pp. 1029–1036, Feb. 2023.

[79] C. Wang et al., “PyPose: A library for robot learning with physics-based optimization,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 22024–22034.

[80] G. Wang, X. Wu, S. Jiang, Z. Liu, and H. Wang, “Efficient 3D deep LiDAR odometry,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 45, no. 5, pp. 5749–5765, May 2023.

[81] G. Wang, X. Wu, Z. Liu, and H. Wang, “PWCLO-Net: Deep LiDAR odometry in 3D point clouds using hierarchical embedding mask optimization,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 15910–15919.

[82] H. Wang, C. Wang, C. Chen, and L. Xie, “F-LOAM: Fast LiDAR odometry and mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 4390–4396.

[83] H. Wang, J. Wang, and L. Agapito, “Co-SLAM: Joint coordinate and sparse parametric encodings for neural real-time slam,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2023, pp. 13293–13302.

[84] Y. Wang et al., “Elastic and efficient LiDAR reconstruction for largescale exploration tasks,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 5035–5041.

[85] T. Whelan, S. Leutenegger, R. S. Moreno, B. Glocker, and A. Davison, “ElasticFusion: Dense SLAM without a pose graph,” in Proc. Robotics: Sci. Syst., 2015, pp. 3–11.

[86] L. Wiesmann et al., “LocNDF: Neural distance field mapping for robot localization,” IEEE Robot. Autom. Lett., vol. 8, no. 8, pp. 4999–5006, Aug. 2023.

[87] L. Wu, K. M. B. Lee, C. Le Gentil, and T. Vidal-Calleja, “Log-GPIS-MOP: A unified representation for mapping, odometry, and planning,” IEEE Trans. Robot., vol. 39, no. 5, pp. 4078–4094, Oct. 2023.

[88] Y. Wu, T. Guadagnino, L. Wiesmann, L. Klingbeil, C. Stachniss, and H. Kuhlmann, “LIO-EKF: High frequency LiDAR-inertial odometry using extended kalman filters,” in Proc. IEEE Int. Conf. Robot. Autom., 2024, pp. 13741–13747.

[89] Q. Xu et al., “Point-neRF: Point-based neural radiance fields,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 5428–5438.

[90] W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, “FAST-LIO2: Fast direct LiDAR-inertial odometry,” IEEE Trans. Robot., vol. 38, no. 4, pp. 2053–2073, Aug. 2022.

[91] Z. Yan, H. Yang, and H. Zha, “Active neural mapping,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2023, pp. 10947–10958.

[92] X. Yang, H. Li, H. Zhai, Y. Ming, Y. Liu, and G. Zhang, “Voxfusion: Dense tracking and mapping with voxel-based neural implicit representation,” in Proc. Int. Symp. Mixed Augmented Reality, 2022.

[93] M. Yokozuka, K. Koide, S. Oishi, and A. Banno, “LiTAMIN2: Ultra light LiDAR-based SLAM using geometric approximation applied with kl-divergence,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 11619–11625.

[94] X. Yu et al., “Nf-atlas: Multi-volume neural feature fields for large scale LiDAR mapping,” IEEE Robot. Autom. Lett., vol. 8, no. 9, pp. 5870–5877, Sep. 2023.

[95] Y. Yuan and A. Nüchter, “An algorithm for the SE(3)-transformation on neural implicit maps for remapping functions,” IEEE Robot. Automat. Lett., vol. 7, no. 3, pp. 7763–7770, Jul. 2022.

[96] J. Zhang, M. Kaess, and S. Singh, “On degeneracy of optimization-based state estimation problems,” in Proc. IEEE Int. Conf. Robot. Autom., 2016, pp. 809–816.

[97] J. Zhang and S. Singh, “LOAM: LiDAR odometry and mapping in realtime,” in Proc. Robotics: Sci. Syst., 2014, pp. 1–9.

[98] Z. Zhang and D. Scaramuzza, “A tutorial on quantitative trajectory evaluation for visual(-inertial) odometry,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 7244–7251.

[99] S. Zhi, T. Laidlow, S. Leutenegger, and A. Davison, “In-place scene labelling and understanding with implicit scene representation,” in Proc. IEEE/CVF Intl. Conf. Comput. Vis., 2021, pp. 15818–15827.

[100] X. Zhong, Y. Pan, J. Behley, and C. Stachniss, “SHINE-Mapping: Large-scale 3D mapping using sparse hierarchical implicit neural representations,” in Proc. IEEE Int. Conf. Robot. Autom., 2023, pp. 8371–8377.

[101] Z. Zhu et al., “NICE-SLAM: Neural implicit scalable encoding for SLAM,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 12776–12786.

![](images/2024_PIN-SLAM/e1239e37a6e3665f39add56dd71ccc2ec8f66dd7d1135a7fde8ce2c29b6f4509.jpg)

Yue Pan received the B.Sc. degree in geomatics engineering from Wuhan University, Wuhan, China, in 2019, and the M.Sc. degree in geomatics engineering from ETH Zurich, Zurich, Switzerland, in 2022. He is currently working toward the Ph.D. degree in engineering with the Photogrammetry & Robotics Lab headed by Prof. Cyrill Stachniss, University of Bonn, Bonn, Germany.

His research interets include SLAM, 3-D reconstruction, and navigation.

![](images/2024_PIN-SLAM/4aafe5cd37adffc94ec2f3472c60d01b3eaea07af8a6c9c82783b66e0adc9050.jpg)

Xingguang Zhong received the B.Sc. degree in mechanical engineering and the M.Sc. degree in mechatronic engineering from the Harbin Institute of Technology, Harbin, China, in 2017 and 2019, respectively. He is currently working toward the Ph.D. degree in engineering with the Photogrammetry & Robotics Lab, University of Bonn, Bonn, Germany, headed by Prof. Cyrill Stachniss.

His research interests include large-scale 3-D reconstruction and autonomous navigation.

![](images/2024_PIN-SLAM/1e7c800a3cf4a27c8305f96f9f5498ae1ab91e6b310945bdf5b4669546ccd5bd.jpg)

Louis Wiesmann (Graduate Student Member, IEEE) received the B.Sc. degree and M.Sc. degree in geodetic engineering in 2017 and 2019, respectively, from the University of Bonn, Bonn, Germany, where he is currently working toward the Ph.D. degree with the Photogrammetry & Robotics Lab headed by Prof. Cyrill Stachniss

His research focuses on localization and mapping in large-scale environments.

![](images/2024_PIN-SLAM/9979da7567685e5cedca56143ab2c30134ffa40bedb2b673511f8372588dd004.jpg)

Thorbjörn Posewsky received the B.Sc. and M.Sc. degrees in computer science from the University of Paderborn, Paderborn, Germany, in 2012 and 2015, respectively. He is currently working toward the Ph.D. degree in engineering with the Photogrammetry & Robotics Lab headed by Prof. Cyrill Stachniss, University of Bonn, Bonn, Germany.

He is currently a Software Engineer with Micro-Vision GmbH, Hamburg, Germany. His research focuses on Mapping and SLAM.

![](images/2024_PIN-SLAM/9bb96b9510f3091a01483544f806b387fac4de1f8f7b0212e1bab175beda71ba.jpg)

Jens Behley (Member, IEEE) received the Dipl.- Inform. in computer science and the Ph.D. in computer science, both from the Department of Computer Science, University ofBonn, Bonn, Germany, in 2009 and 2014, respectively.

Since 2016, he is a Postdoctoral Researcher with the Photogrammetry & Robotics Lab, University of Bonn. He finished his habilitation with the University of Bonn in 2023. His research interests include in the area of perception for autonomous vehicles, deep learning for semantic interpretation, and LiDAR-based SLAM.

![](images/2024_PIN-SLAM/864139884e70b6134222b1d545688d3981715896384079a18a84dc32ce09c074.jpg)

Cyrill Stachniss (Member, IEEE) received the Habilitation degree and the Ph.D. degree in computer science from the University of Freiburg, Freiburg im Breisgau, Germany, in 2009 and 2016, respectively.

He is currently a Full Professor with the University of Bonn, Bonn, Germany, University of Oxford, Oxford, U.K., as well as Lamarr Institute for Machine Learning and AI, Dortmund, Germany.

He is the Spokesperson of the DFG Cluster of Excellence PhenoRob with the University of Bonn. His research interets include probabilistic techniques and learning approaches for mobile robotics, perception, and navigation, agricultural robotics, service robotics, and self-driving cars.