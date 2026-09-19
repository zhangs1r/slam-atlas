# Vox-Fusion: Dense Tracking and Mapping with Voxel-based Neural Implicit Representation

Xingrui Yang1\* Hai Li1\* Hongjia Zhai Yuhang Ming² Yuqian Liu³ Guofeng Zhang¹†

1State Key Lab of CAD&CG, Zhejiang University 2Visual Information Laboratory, University of Bristol 3Autonomous Driving Group, SenseTime xingruiy@gmail.com, {garyli, zhj1999, zhangguofeng}@zju.edu.cn, yuhang.ming@bristol.ac.uk,liuyuqian@senseauto.com

![](images/2022_Vox-Fusion/2487e4704f4c32310b4571a09ec60091c1e543aef8bb161c97bd4833e6745c5f.jpg)  
Figure 1: We present Vox-Fusion, a SLAM system that incrementally reconstructs scenes from RGB-D frames using neural implicit networks. The current camera frame is in red and the camera trajectory is in green. Our method divides the scene into an explicit voxel grid representation (depicted as black bounding boxes) which is gradually built on-the-fly. Inside each voxel, we fuse depth and color observations into local feature embeddings through volume rendering.

## ABSTRACT

In this work, we present a dense tracking and mapping system named Vox-Fusion, which seamlessly fuses neural implicit representations with traditional volumetric fusion methods. Our approach is inspired by the recently developed implicit mapping and positioning system and further extends the idea so that it can be freely applied to practical scenarios. Specifically, we leverage a voxel-based neural implicit surface representation to encode and optimize the scene inside each voxel. Furthermore, we adopt an octree-based structure to divide the scene and support dynamic expansion, enabling our system to track and map arbitrary scenes without knowing the environment like in previous works. Moreover, we proposed a high-performance multi-process framework to speed up the method, thus supporting some applications that require realtime performance. The evaluation results show that our methods can achieve better accuracy and completeness than previous methods. We also show that our Vox-Fusion can be used in augmented reality and virtual reality applications. Our source code is publicly available at https://github.com/zju3dv/Vox-Fusion.

Index Terms: Dense SLAM—Implicit Networks—Voxelization— Surface Rendering

## 1 INTRODUCTION

Dense simultaneous localization and mapping (SLAM) aims to track the 6 degrees of freedom (DoF) poses of a moving RGB-D camera whilst constructing a dense map of the surrounding environment in real-time. It is an essential part of augmented reality (AR) and virtual reality (VR). With high tracking accuracy and the ability to recover complete surfaces, it can support real-time occlusion effects and collision detection during virtual interaction.

Traditional SLAM methods using either feature matching [3, 16–18], nonlinear energy minimization [20], or a combination of both [7, 39] to solve the camera poses. These poses are then coupled with their corresponding input point clouds to update a global map represented by geometric primitives such as cost volumes [20,37], surfels [29, 35, 38] or voxels [12, 19,21]. Although these methods have been well studied and have shown good reconstruction results, they are incapable of rendering novel views as they cannot hallucinate the unseen parts of the scene. Storing and distributing the maps can be challenging as well due to the requirement of large video memory (VRAM). Moreover, modifying the map on-the-fly is also difficult because of the large number of elements in the map and weaker data associations compared to feature-based methods [7, 26].

Focusing on reducing memory usage and improving efficiency, recent works such as CodeSLAM [2] and the follow-up works [5,9] have demonstrated that neural networks have the ability to encode depth maps using fixed-length optimizable latent embeddings. These latent codes can be updated with multi-view constraints. This method provides a good trade-off between scene quality and memory usage. However, the pre-trained networks used in these systems generalize poorly to different types of scenes, making them less useful in practical scenarios. Also, a consistent global representation is difficult to obtain due to the use of local latent codes.

To address these issues, recent works take advantage of the success of NeRF [15] and train a neural implicit network on-thefly to represent 3D scenes continuously in dense SLAM applications [31,40]. Specifically, iMap [31] directly uses a single multilayer perceptron (MLP) to approximate a global scene map and jointly optimizes the map and the camera poses. However, the use of a single MLP makes it difficult to represent geometric details of the scene as well as scale to larger environments without significantly increasing the network capacity.

![](images/2022_Vox-Fusion/396dc0c0c4d57f415dd4c2d54799bb09d32cd00bada1e4d2db181d591c84ebb8.jpg)  
Figure 2: Overview of our SLAM system. Our system consists of three parts: 1) Volume Renderer, which encodes the scene in a MLP and embedding vectors, and outputs the rendered color and SDF value for a given pixel; 2) Tracking Process, which takes as input RGB-D frames and optimizes the camera poses via differentiable rendering and 3) Mapping Process, which reconstructs the geometry of the scene.

In this paper, we are interested in mapping unknown scenes with neural implicit networks. Inspired by traditional volumetric SLAM systems [34] and the successful application of neural implicit representation in parallel tracking and mapping [31], we propose a more efficient hybrid data structure that combines a sparse voxel representation with neural implicit embeddings. More specifically, we use a sparse octree with Morton coding for fast allocation and retrieval of voxels, which have been proven to be real-time capable for dense mapping [34]. We model the scene geometry within local voxels as a continuous signed distance function (SDF) [4], which is encoded by a neural implicit decoder and shared feature embeddings. The shared embedding vectors allow us to use a more lightweight decoder because they contain knowledge of local geometry and appearance. The tracking and mapping process is achieved with differentiable volumetric rendering. We show that our explicit voxel representation is beneficial to AR applications and our mapping method creates more detailed reconstructions compared to current state-of-the-art (SOTA) systems, as we show in the experiments section. To summarize, our contributions are:

1. We propose a novel fusion system for real-time implicit tracking and mapping. Our Vox-Fusion combines voxel embeddings indexed by an explicit octree and a neural implicit network to achieve scalable implicit scene reconstruction with sufficient details.

2. We show that by directly rendering signed distance volumes, our system provides better tracking accuracy and reconstruction quality compared to current SOTA systems with no performance overhead.

3. We propose to use a fast and efficient keyframe selection strategy based on ratio test and measuring information gain, which is more suitable to maintain large-sized maps.

4. We perform extensive experiments on synthetic and real-world scenes to demonstrate the proposed method is capable of producing high-quality 3D reconstructions, which can directly benefit many AR applications.

This paper is organized as follows: Sect. 2 gives a review of related works. Sect. 3 presents an overview of our proposed Vox-Fusion system. We explain our reconstruction pipeline in Sect. 4. Finally, we evaluate the proposed system on various synthetic and real-world tasks in Sect. 5. We then conclude our paper by introducing potential applications in Sect. 6 and discussing limitations in Sect. 7.

## 2 RELATED WORK

## 2.1 Dense SLAM

Traditional SLAM Methods. DTAM [20] introduced the first dense SLAM system that uses every pixel photometric consistency to track a handheld camera. They employ multi-view stereo constraints to update a dense scene model, represented as a cost volume. However, their method is only applicable to small workshop-like spaces. Taking advantage of RGB-D cameras, KinectFusion [19] proposes a novel reconstruction pipeline, which exploits the accurate depth acquisition from commodity depth sensors and the parallel processing power of modern graphics units (GPUs). They track input depth maps with iterative closest point (ICP) and progressively update a voxel grid with aligned depth maps. They also proposed a novel frame-to-model tracking method that greatly reduces short-term drifts with circular camera motion. Following this basic design, many systems gain improvements by introducing different 3D structures [38], exploring space subdivision [21, 24] and performing global map optimization [11, 13,26]. Another interesting research direction is to combine features and dense maps [7,39], which greatly increases the robustness of iterative methods. These systems provide good results on scene reconstruction at the expense of having a large memory footprint.

Learning-based Methods. Exploiting the power of learned geometric priors, DI-Fusion [10] proposes to encode points in a low dimensional latent space, which can be decoded to generate SDF values. However, the learned geometric prior is inaccurate in complex areas, which leads to poor reconstruction quality. CodeSLAM [2] proposes to use an encoder-decoder structure to embed depth maps as low dimensional codes. These codes, combined with the pretrained neural decoder, can be used to jointly optimize a collection of key-frames and camera poses. However, similar to other learning-based methods, their approach is not robust to scene variations. Another successful design is to learn feature matching and scene reconstruction separately [33]. Recently there have been successful experiments on representing scenes with a single implicit networks [31]. They formulate the dense SLAM problem as a continuous learning paradigm. To bound optimization time, they use heuristic sampling strategies and keyframe selection based on information gain. This method provides a good trade-off between compactness and accuracy. Our method is directly inspired by their design.

![](images/2022_Vox-Fusion/aa89b7d160dd2d85937b269a33c3ffbf07f884aaa092a4816d439d21cec9a2a2.jpg)  
Figure 3: Example of Morton coding on a two-level quad-tree.

A recent concurrent work NICE-SLAM [40] proposes to tackle the scalability problem by subdividing the world coordinate system into uniform grids. Their system is similar to ours in that we both use voxel features instead of encoding 3D coordinates. However, our approach differs from theirs in the following aspects: (1) NICE-SLAM pre-allocates a dense hierarchical voxel grid for an entire scene, which is not suitable for a practical scenario where the scene bound is unknown, while ours dynamically allocate sparse voxels on the fly, as shown in Fig. 1, which not only improves usability but also drastically reduces memory consumption; (2) NICE-SLAM uses a pre-trained geometry decoder which could reduce generalization ability while our parameters are all learned on-the-fly, thus our reconstructed surface is not affected by prior information; (3) We propose a keyframe strategy suitable for sparse voxels which is simple and efficient, while NICE-SLAM adopts the strategy from iMap which does not take advantage of the voxel representation.

## 2.2 Neural Implicit Networks

NeRF [15] proposes a method to render the scene as volumes that have density, which is good for representing transparent objects. Their method is only interested in rendering photo-realistic images, a good surface reconstruction is not guaranteed. However, for most AR tasks it is important to know where the surface lies. To solve the surface reconstruction problem, many new methods assume that color is only contributed by points near or on the surface. They propose to identify the surface via iterative root-finding [22], weighting the rendered color with the associated SDF values [36], forcing the network to learn more details near the surface within a pre-defined truncation distance [1]. We adopt the rendering method from [1] but instead of regressing absolute coordinates, we work on interpolated voxel embeddings.

![](images/2022_Vox-Fusion/f6820be69b8a1ab7bc1afbff8c0e5e3f638c8a5d9b49af339e84708d3e72badb.jpg)  
Figure 4: Example of our hierarchical octree structure. For simplicity we only show a four-level octree here.

As a single network often has limited capacity and cannot be scaled to larger scenes without dramatically increasing the number of learnable parameters, NSVF [14] proposes to embed local information in a separate voxel grid of features. They are able to generate similar if not better results with fewer parameters. Plenoxels [25] introduced spherical harmonic functions as voxel embeddings, which completely removed the need for a neural network. The other benefit of an explicit feature grid is the faster rendering speed, since the implicit network can be much smaller compared to the original NeRF network. This uniform grid design can be found in other neural reconstruction methods. E.g. NGLOD [32] leverages a hierarchical data structure by concatenating features from each level to achieve scene representation with different levels of details.

Our hybrid scene representation is inspired by recent works that use voxel-based neural implicit representations [8, 14]. More specifically, we share a similar structure with Vox-Surf [8] which encodes 3D scenes with neural networks and local embeddings. However, Vox-Surf is an offline system that requires posed images, while our system is a SLAM method that consumes consecutive RGB-D frames for pose estimation and scene reconstruction simultaneously. Vox-surf also needs to allocate all voxels in advance, while our system can manage voxels on-the-fly to allow dynamic growth of the map. Moreover, we use a different rendering function that reconstructs scenes more efficiently.

## 3 SYSTEM OVERVIEW

An overview of our system is shown in Fig. 2. The input to our system is continuous RGB-D frames which consist of RGB images $\boldsymbol { I _ { i } } \in \mathbb { R } ^ { 3 }$ and depth maps $D _ { i } \in \mathbb { R }$ We use the pinhole model as the default camera model and assume that the intrinsic matrix K ∈ $\mathbb { R } ^ { 3 \times 3 }$ of the camera is known. Similar to other well-known SLAM architectures, our systems also maintain two separate processes: a tracking process to estimate the current camera pose as the frontend and a mapping process to optimize the global map as the backend.

When the system starts, we initialize the global map by running a few mapping iterations for the first frame. For subsequent frames, the tracking process first estimate a 6-DoF pose $T _ { i } \in S E ( 3 )$ w.r.t. the fixed implicit scene network $F _ { \theta }$ via our differentiable volume rendering method. Then, each tracked frame is sent to the mapping process for constructing the global map. The mapping process first takes the estimated camera poses $T _ { i }$ from frontend and allocates new voxels from the back-projected and appropriately transformed 3D point cloud $P _ { i } \in \mathbb { R } ^ { 3 }$ from the input depth map $D _ { i } .$ Then it fuses the new voxel-based scene into the global map and applies the joint optimization. In order to reduce the complexity of optimization, we only maintain a small number of keyframes, which are selected by measuring the ratio of observed voxels. And the long-term map consistency is maintained by constantly optimizing a fixed window of keyframes. These individual components will be explained in detail in the following subsections.

## 4 METHOD

## 4.1 Volume Renderer

Voxel-based sampling. We represent our scene as an implicit SDF decoder $F _ { \theta }$ with optimizable parameters θ, and a collection of Ndimensional sparse voxel embeddings. The voxel embeddings are attached to the vertices of each voxel and are shared by neighboring voxels. The shared embeddings alleviate voxel border artifacts as commonly seen in non-sharing structures [10].

However, sampling points inside the sparse voxel representation is not straightforward. Naive sampling methods such as stratified random sampling [15] waste computational power on sampling spaces that are not covered by valid voxels. Therefore, we adopt the method used in [14] to perform efficient point sampling. For each sampled pixel, we first check if it has hit any voxel along the visual ray by performing a fast ray-voxel intersection test. Pixels without any hit are masked out since they do not contribute to rendering. As the scene could be unbounded in complex scenes, we enforce a limit $M _ { h }$ on how many voxels a single pixel is able to see. Unlike prior works where the limit is heuristically specified [8, 14], we dynamically change it according to the specified maximum sampling distance $D _ { m a x }$

Implicit surface rendering. Unlike NeRF [15] where an MLP is used to predict occupancy for 3D points, we directly regress SDF values which is a more useful geometric representation that can support tasks such as ray tracing. The key to our approach is to use voxel embeddings instead of 3D coordinates as opposed to previous works. To render color and depth from the map, we adopt the volume rendering method proposed in [1]. However, we modify it to apply to feature embeddings instead of global coordinates. We sample N points to render the color for each ray. More specifically, we use the following rendering function to obtain the color C and depth D for each ray:

$$
( \mathbf { c } _ { i } , s _ { i } ) = F _ { \pmb { \theta } } ( \mathrm { T r i L e r p } ( \hat { \xi } _ { i } T _ { i } \mathbf { p } _ { i } , \mathbf { e } ) ) ,\tag{1}
$$

$$
w _ { i } = \sigma ( \frac { s _ { i } } { t r } ) \cdot \sigma ( - \frac { s _ { i } } { t r } ) ,\tag{2}
$$

$$
\mathbf { C } = \frac { 1 } { \sum _ { j = 0 } ^ { N - 1 } w _ { j } } \sum _ { i = 0 } ^ { N - 1 } w _ { j } \cdot \mathbf { c } _ { j } ,\tag{3}
$$

$$
D = \frac { 1 } { \sum _ { j = 0 } ^ { N - 1 } w _ { i } } \sum _ { j = 0 } ^ { N - 1 } w _ { j } \cdot d _ { j } ,\tag{4}
$$

where $T _ { i }$ is the current camera pose, TriLerp(·, ·) is the trilinear interpolation function, $F _ { \theta }$ is the implicit network with trainable parameters θ, and $\xi \in \mathfrak { s e } ( 3 )$ is the frame pose update. $\mathbf { c } _ { j }$ is the predicted color for each 3D point from the network, by trilinearly interpolating voxel embeddings e. Likewise, s is the predicted SDF value, and $d _ { j }$ the j-th depth sample along the ray. $\sigma ( \cdot )$ is the sigmoid function and tr is a pre-defined truncation distance. The depth map is similarly rendered from the map by weighting sampled distance instead of colors.

Optimization. To supervise the network, we apply four different loss functions: RGB loss, depth loss, free-space loss and SDF loss on the sampled points P. The RGB and depth losses are simply absolute differences between render and ground-truth images:

$$
\begin{array} { r } { \mathcal { L } _ { \boldsymbol { r } g b } = \displaystyle \frac { 1 } { | P | } \sum _ { i = 0 } ^ { | P | } \| \mathbf { C } _ { i } - \mathbf { C } _ { i } ^ { g t } \| , } \\ { \mathcal { L } _ { d e p t h } = \displaystyle \frac { 1 } { | P | } \sum _ { i = 0 } ^ { | P | } \| D _ { i } - D _ { i } ^ { g t } \| , } \end{array}\tag{5}
$$

where $D _ { i } , C _ { i }$ are the rendered depth and color of the i-th pixel in a batch, respectively. $D _ { i } ^ { g t } , C _ { i } ^ { g t }$ are the corresponding ground truth values. The free-space loss works with a truncation distance tr within which the surface is defined. The MLP is forced to learn a truncation value tr for any points lie within the camera center and the positive truncation region of the surface:

![](images/2022_Vox-Fusion/51ea7d4aaf57e3f45edc310a87667f587be0569b82dc025b153ceccdfd1a5c40.jpg)  
Figure 5: Network architecture.

$$
\mathcal { L } _ { f s } = \frac { 1 } { | P | } \sum _ { p \in P } \frac { 1 } { S _ { p } ^ { f s } } \sum _ { s \in S _ { p } ^ { f s } } ( D _ { s } - t r ) ^ { 2 }\tag{6}
$$

Finally, we apply SDF loss to force the MLP to learn accurate surface representations within the surface truncation area:

$$
\mathcal { L } _ { s d f } = \frac { 1 } { | P | } \sum _ { p \in P } \frac { 1 } { S _ { p } ^ { t r } } \sum _ { s \in S _ { p } ^ { t r } } ( D _ { s } - D _ { s } ^ { g t } ) ^ { 2 }\tag{7}
$$

Unlike methods such as [23] that force the network to learn a negative truncation value —tr for points behind the truncation region, we simply mask out these points during rendering to avoid solving surface intersection ambiguities [36] as proposed in [1]. This simple formulation allows us to obtain accurate surface reconstructions with a much faster processing speed.

## 4.2 Tracking

During tracking, we keep our voxel embeddings and the parameters of the implicit network fixed, i.e., we only optimize a 6-DoF pose $T \in S E ( 3 )$ for the current camera frame. Similar to previous methods where pose estimates are iteratively updated by solving an incremental update, in each update step, we measure the pose update in the tangent space of SE(3), represented as the lie algebra $\xi \in \mathfrak { s e } ( 3 )$ . We assume a zero motion model where the new frame is sufficiently close to the last tracked frame, therefore we initialize the pose of the new frame to be identical to that of the last tracked frame. Although other motion models such as constant motion are also applicable here. For each frame, we sample a sparse set of $N _ { t }$ pixels from the input images for tracking. We follow the procedure described in Sect. 4.1 to sample candidate points and perform volume rendering, respectively. The frame pose is updated in each iteration via back-propagation. Similar to [31], we keep a copy of our SDF decoder and voxel embeddings for the tracking process. This map copy is directly obtained from the mapping process and updated each time a new frame has been fused into the map.

## 4.3 Mapping

Key-frame selection. In online continuous learning, keyframe selection is the key to ensuring long-term map consistency and preventing catastrophic forgetting [31]. Unlike previous works where keyframes are only inserted based on heuristically chosen metrics [31] or at a fixed interval [40], our explicit voxel structure allows us to determine when to insert key-frames by performing an intersection test. Specifically, each successfully tracked frame is tested against the existing map to find the number of voxels $N _ { c }$ that would be allocated if we are to choose it as a new keyframe. We insert a new keyframe if the ratio $p _ { k f } = N _ { c } / N _ { o }$ is larger than a threshold, where $N _ { o }$ is the number of currently observed voxels.

![](images/2022_Vox-Fusion/6be75cfcf3874ac12b26c06be8824b59a838409eeb9c38cabf7e3604ce6db5c9.jpg)  
(a) Ground truth

![](images/2022_Vox-Fusion/8472fc706ef4bd3296037c06380aec8504a1ce30ce0d4607822a056a1ff481d2.jpg)  
(b) Ours (voxel length=0.2)

![](images/2022_Vox-Fusion/d27bc3ca5503c40593e9ee4657a4700928d9518289a4f46a7a51f22bdd85a68f.jpg)  
(c) NICE-SLAM (voxel 1ength=0.16)  
Figure 6: Surface reconstruction details comparison. Our system is able to recover thin structures, such as chair legs and flowers, from the raw scans, albeit using a slightly larger voxel.

This simple strategy is adequate for exploratory movements as new voxels are constantly being allocated. However, for loopy camera motions, especially trajectories with long-term loops, there is a risk that we may never allocate new keyframes as we keep looking at an existing scene model. This will result in part of the model being completely missing or not having enough multi-view constraints. To solve this problem, we also enforce a maximum interval between adjacent frames, i.e., we will create a new keyframe if have not done so for the past N frames. This key-frame selection strategy is simple yet effective at creating consistent scene maps.

Joint mapping and pose update. Our mapping subroutine accepts tracked RGB-D frames and fuses them into the existing scene map by jointly optimizing the scene geometry and camera poses. It is noted by [31] that online incremental learning is prone to network forgetting. Therefore we use a similar method to jointly optimize the scene network and feature embeddings. For each frame, we randomly select $N _ { k f }$ keyframes. These keyframes, including the recently tracked frame, can be seen as an optimization window akin to the sliding window approach employed in traditional SLAM systems [27].

Similar to our tracking process, for each frame in the actively sampled optimization window, we randomly sample $N _ { m }$ rays. These rays are transformed into the world coordinate system with estimated frame poses. Then we sample points within our sparse voxels and then render a set of pixels from the sample points and calculate related loss functions, using the method described in Sect. 4.1.

## 4.4 Dynamic Voxel Management

Contrary to existing approaches where the full extent of the scene is encoded [31], we are only interested in reconstructing surfaces that have observations. Therefore, on-the-fly voxel allocation and searching are of most importance to us. For this reason, we adopt an octree structure to divide the whole scene into mutually exclusive axis-aligned voxels where we consider the voxel as the basic scene unit and the leaf node of the scene octree. An example octree is shown in Fig. 4. We make our system usable in unexplored areas by dynamically allocating new voxels when new observations are made.

Specifically, we initially set the leaf nodes corresponding to the unobserved scene area to empty, when a new frame is successfully tracked, we back-project its depth map into 3D points, these points are then transformed by the estimated camera pose. We then allocate new voxels for any point that does not fall into an existing voxel. Since this process needs to be applied for every point, which could have tens of thousands based on the resolution of the input images, we use an octree structure to store voxels and feature embeddings to enable fast voxel allocation and retrieval.

Morton coding Inspired by traditional volumetric SLAM systems [34], we choose to encode voxel coordinates as Morton codes. Morton codes are generated by interleaving the bits from each coordinate into a unique number. A 2D example of Morton coding is given in Fig. 3. Given the 3D coordinate $( x , y , z )$ of a voxel, we can quickly find its position in the octree by traversing through its Morton code. It is also possible to recover the encoded coordinates by applying a decoding operation. The neighbors are also identifiable by shifting the appropriate bits of the code, which is beneficial for quickly finding shared embedding vectors.

## 5 EXPERIMENTS

## 5.1 Experimental Setup

Datasets: In our experiments we use 3 different datasets: (1) Replica dataset [28] which contains 18 different scenes captured by a camera rig. Based on Replica dataset, [31] further synthesizes the RGB-D sequences, which are used in our experiments. (2) ScanNet dataset [6] which contains more than 1000 captured RGB-D sequences and ground truth poses estimated from a SLAM system [7]. (3) Several indoor and outdoor RGB-D sequences that were captured by iOS devices equipped with range sensors such as iPhone 13 Pro and iPad Pro. These datasets cover a wide range of applications and scenarios therefore are well suited to study our proposed system.

Evaluation Metrics: We use several different metrics to measure the performance of our system as well as other competing methods. For reconstruction quality, we measure accuracy and completion, this is in line with previous works [31,40]. Mesh accuracy (Acc.) is defined as the un-directional Chamfer distance from the reconstructed mesh to the ground-truth. Completion (Comp.) is similarly defined as the distance the other way around. We also measure the completion ratio, which is the percentage of the reconstructed points whose distance to the ground truth mesh is smaller than 5cm. We show the formulation of Chamfer distance in Equation 8, where $p \in P$ and $q \in { \cal Q }$ are two point sets sampled from the reconstructed

Table 1: Trajectory estimation results on the Replica dataset. Our method obtained better results on all sequences.
<table><tr><td>Methods</td><td>Metric</td><td>Room-0</td><td>Room-1</td><td>Room-2</td><td>Office-0</td><td>Office-1</td><td>Office-2</td><td>Office-3</td><td>Office-4</td><td>Avg.</td></tr><tr><td rowspan="3">iMap* [31]</td><td>RMSE[m]↓</td><td>0.7005</td><td>0.0453</td><td>0.0220</td><td>0.0232</td><td>0.0174</td><td>0.0487</td><td>0.5840</td><td>0.0262</td><td>0.1834</td></tr><tr><td>mean[m]↓</td><td>0.5891</td><td>0.0395</td><td>0.0195</td><td>0.01652</td><td>0.0155</td><td>0.0319</td><td>0.5488</td><td>0.0215</td><td>0.1603</td></tr><tr><td>median[m]↓</td><td>0.4478</td><td>0.0335</td><td>0.0173</td><td>0.0135</td><td>0.0137</td><td>0.0235</td><td>0.4756</td><td>0.0186</td><td>0.1304</td></tr><tr><td rowspan="3">NICE-SLAM [40]</td><td>RMSE[m]↓</td><td>0.0169</td><td>0.0204</td><td>0.01554</td><td>0.0099</td><td>0.0090</td><td>0.0139</td><td>0.0397</td><td>0.0308</td><td>0.0195</td></tr><tr><td>mean[m]↓</td><td>0.0150</td><td>0.0180</td><td>0.0118</td><td>0.0086</td><td>0.0081</td><td>0.0120</td><td>0.0205</td><td>0.0209</td><td>0.0144</td></tr><tr><td>median[m]↓</td><td>0.0138</td><td>0.0167</td><td>0.0098</td><td>0.0076</td><td>0.0074</td><td>0.0109</td><td>0.0128</td><td>0.0153</td><td>0.0118</td></tr><tr><td rowspan="3">Ours</td><td>RMSE[m]↓</td><td>0.0027</td><td>0.0133</td><td>0.0047</td><td>0.0070</td><td>0.0111</td><td>0.0046</td><td>0.0026</td><td>0.0058</td><td>0.0065</td></tr><tr><td>mean[m]↓</td><td>0.0022</td><td>0.0107</td><td>0.0030</td><td>0.0049</td><td>0.0073</td><td>0.0037</td><td>0.0022</td><td>0.0040</td><td>0.0048</td></tr><tr><td>median[m]↓</td><td>0.0020</td><td>0.0078</td><td>0.0027</td><td>0.0029</td><td>0.0045</td><td>0.0031</td><td>0.0019</td><td>0.0028</td><td>0.0035</td></tr></table>

![](images/2022_Vox-Fusion/c4827170030d541f086fa58b13933955a8cb6c47305620f27a4bef0cbe29dac7.jpg)  
(a) iMap\*

![](images/2022_Vox-Fusion/6490190f3da895d95861b68d63caad9f3371e2abe40acc504058006a96e0a37d.jpg)  
(b) NICE-SLAM

![](images/2022_Vox-Fusion/4d2501eca50edc58e7cdee1b19f5d6f9d802b3da10e9eed6adc8c202cd275a1b.jpg)  
(c) Ours

![](images/2022_Vox-Fusion/0704890c96a1532376052e5dce06d72299882f35f19847c99eb5bb65e4c775a2.jpg)  
(d) Ground truth  
Figure 7: Qualitative reconstruction results on the Replica dataset. From left to right, we show the results of scene reconstruction of different methods (iMAP\*, NICE-SLAM, our method, and ground truth). It can be clearly seen that our reconstruction results are much better than iMAP\*. To better show the difference in reconstruction between NICE-SLAM and our method, we use red boxes in the figures to indicate the improvements over NICE-SLAM.

Table 2: Reconstruction Results of 8 Scenes in the Replica dataset. Compared with iMAP and NICE-SLAM, our approach yields better results consistently.
<table><tr><td>Methods</td><td>Metric</td><td>Room-0</td><td>Room-1</td><td>Room-2</td><td>Office-0</td><td>Office-1</td><td>Office-2</td><td>Office-3</td><td>Office-4</td><td>Avg.</td></tr><tr><td rowspan="3">iMap [31]</td><td>Acc.[cm]↓</td><td>3.58</td><td>3.69</td><td>4.68</td><td>5.87</td><td>3.71</td><td>4.81</td><td>4.27</td><td>4.83</td><td>4.43</td></tr><tr><td>Comp.[cm].↓</td><td>5.06</td><td>4.87</td><td>5.51</td><td>6.11</td><td>5.26</td><td>5.65</td><td>5.45</td><td>6.59</td><td>5.56</td></tr><tr><td>Comp. Ratio[&lt; 5cm %]↑</td><td>83.91</td><td>83.45</td><td>75.53</td><td>77.71</td><td>79.64</td><td>77.22</td><td>77.34</td><td>77.63</td><td>79.06</td></tr><tr><td rowspan="3">NICE-SLAM [40]</td><td>Acc.[cm]↓</td><td>3.53</td><td>3.60</td><td>3.03</td><td>5.56</td><td>3.35</td><td>4.71</td><td>3.84</td><td>3.35</td><td>3.87</td></tr><tr><td>Comp.[cm]↓</td><td>3.40</td><td>3.62</td><td>3.27</td><td>4.55</td><td>4.03</td><td>3.94</td><td>3.99</td><td>4.15</td><td>3.87</td></tr><tr><td>Comp. Ratio[&lt; 5cm %]↑</td><td>86.05</td><td>80.75</td><td>87.23</td><td>79.34</td><td>82.13</td><td>80.35</td><td>80.55</td><td>82.88</td><td>82.41</td></tr><tr><td rowspan="3">Ours</td><td>Acc.[cm]↓</td><td>2.55</td><td>2.25</td><td>4.26</td><td>2.49</td><td>3.68</td><td>4.78</td><td>4.15</td><td>3.35</td><td>3.44</td></tr><tr><td>Comp.[cm]↓</td><td>2.82</td><td>2.36</td><td>2.67</td><td>1.64</td><td>1.78</td><td>3.02</td><td>3.11</td><td>3.36</td><td>2.60</td></tr><tr><td>Comp. Ratio[&lt; 5cm %]↑</td><td>90.97</td><td>92.72</td><td>89.88</td><td>95.40</td><td>93.91</td><td>89.06</td><td>87.59</td><td>86.12</td><td>90.71</td></tr></table>

![](images/2022_Vox-Fusion/dde8587c2a3386dcfca9a907131cf5c12b8f4830bdfa3f27552d85797f180d0b.jpg)  
Figure 8: Reconstruction of real-world RGB-D sequences captured with a handheld device. The images were taken by an iPhone 13 Pro. We get depth images directly from the onboard lidar sensor. We show (top) reconstructions and (bottom) an example of the original input image.

and ground truth meshes:

$$
\begin{array} { c } { { d _ { C h a m f e r } = | P | ^ { - 1 } \displaystyle \sum _ { ( p , q ) \in \Lambda _ { P , Q } } | | p - q | | ^ { 2 } } } \\ { { \Lambda _ { Q , P } ^ { * } = \{ ( p , a r g m i n _ { q } \| p - q \| ) \} } } \end{array}\tag{8}
$$

To benchmark pose estimation, we adopt the commonly used absolute trajectory error (ATE) using the scripts provided by [30]. ATE is calculated as the absolute translational difference between the estimated and ground truth poses.

Implementation Details: We illustrate our network architecture in Fig. 5. Our decoder is implemented as an MLP consists of several fully-connected layers (FC) and skip connections. The input to our network is a 16-D feature embedding. The features are processed by 4 FC layers that each has 256 hidden units. The SDF head outputs a scalar SDF value s and a 128-D hidden vector. The color head has two FC layers with 256 hidden units each. We apply sigmoid to generate RGB values in the range [0, 1]. To sample points, We use a step size ratio of 0.1 for sampling voxel points. For all scenes, we use a voxel size of 0.2m.

## 5.2 Reconstruct Synthetic Scenes

To test our system on reconstructing synthetic scenes, we use the Replica dataset. The RGB-D sequences we use are released by [31] and subsequently used in NICE-SLAM [40]. We compare our system qualitatively with iMap and NICE-SLAM. The results of iMap are directly obtained from their paper, while the results for NICE-SLAM is generated from their official code release. We show the qualitative evaluation results in Fig. 7. It can be seen that our method produces better maps than iMap, and performs on par with NICE-SLAM.

It is worth noting that both NICE-SLAM and iMap assume densely populated surfaces, therefore they will create surfaces even in places that is not observed. Surfaces created in this way will be realistic when the gap is small, but deviates from the ground truth by a large margin when there is a huge gap. Our use of an explicit voxel map prevents this from happening, i.e., our system only hallucinates surfaces inside the visible sparse voxels, therefore we can still produce plausible hole fill-in effects, while leaving large unobserved spaces empty. By doing so, we effectively combine the best of both worlds. Although it might seem like a disadvantage at first, we argue that for real-world tasks it is often more important to know where has been observed and where has not.

Table 3: Trajectory estimation results on the ScanNet dataset (RMSE).
<table><tr><td>Scene ID</td><td>0000</td><td>0106</td><td>0169</td><td>0181</td><td>0207</td></tr><tr><td>DI-Fusion [10]</td><td>0.6299</td><td>0.1850</td><td>0.7580</td><td>0.8788</td><td>1.0019</td></tr><tr><td>iMap* [31]</td><td>0.5595</td><td>0.1750</td><td>0.7051</td><td>0.3210</td><td>0.1191</td></tr><tr><td>NICE-SLAM [40]</td><td>0.0864</td><td>0.0809</td><td>0.1028</td><td>0.1293</td><td>0.0559</td></tr><tr><td>Ours</td><td>0.0747</td><td>0.0323</td><td>0.0841</td><td>0.0879</td><td>0.0289</td></tr></table>

Owing to the expressiveness of the signed distance representation, Our method is also able to reconstruct more detailed surfaces. This is shown in Fig. 6. In our results, the table legs and flowers are clearly seen but missing in the results of NICE-SLAM, despite that we use slightly larger voxels and only a single voxel grid level as opposed to three levels in NICE-SLAM.

We also quantitatively compared our system on reconstruction quality and trajectory estimation with iMap and NICE-SLAM. Please note that the results of reconstruction quality for iMap are directly obtained from its paper [31], while the trajectory estimation experiment was performed with the iMap implementation of [40] since the original authors do not open source their code (denoted as iMap\*). The results for NICE-SLAM are taken from the supplementary material of the published paper. Please note that we use the results computed without mesh culling for a fair comparison. The results on reconstruction accuracy are listed in Table 2. The results for trajectory estimation accuracy are listed in Table 1. It is clear that we surpass both systems on all metrics. We also obtained much better results on camera pose estimation with a large margin. These results further confirm our observation that our system is able to produce state-of-the-art results on synthetic datasets.

## 5.3 Reconstruct Real Scans

Unlike synthetic datasets, real scans are noisier and contain erroneous measurements. Reconstructing real scans is considered a challenging task that has not yet been solved. We benchmarked our system on selected sequences of ScanNet [6]. The selection of sequences is in line with [40], and the results from DI-Fusion [10], iMap\* and NICE-SLAM are directly taken from [40]. The quantitative results are shown in Table 3. As can be seen, despite our simple design, we are able to get better performance with NICE-SLAM while surpassing iMap by a large margin. It can be seen that despite the simplicity of our design, we are outperforming iMap and NICE-SLAM by a large margin. We also provide geometric reconstruction with sufficient details, which provides better results in scene accuracy and completion.

![](images/2022_Vox-Fusion/089ddfd10358bce19d61e41cd31958333f5acbe147f323c021fee7de480662e6.jpg)  
(a) Ground Truth

![](images/2022_Vox-Fusion/8690858e342f73047307c70947c24bae03fdf4c89b45bcca502e322f2a0feaef.jpg)  
(b) Rendered Image

![](images/2022_Vox-Fusion/53ba1d69d508fb0de361a050cf5c48b604028c58bdb342646385c300e4818b1d.jpg)  
(c) AR View1

![](images/2022_Vox-Fusion/023421f5601e24c5834cfd60e1302894b84150dadbefb6cbd1256db57e377c3e.jpg)  
(d) AR View2  
Figure 9: The ground truth image (a) in the Replica office3 dataset and its rendered image (b) with our reconstructed scene. And we place some pre-defined objects in office3, which is shown by (c) and (d) in different viewpoints. We show that we can achieve a good occlusion relationship between real and virtual objects.

Table 4: Average time spent on each component.
<table><tr><td>Components</td><td>Measured time</td></tr><tr><td>Tracking</td><td>12 ms</td></tr><tr><td>Mapping</td><td>55 ms</td></tr><tr><td>Voxel allocation</td><td>0.1 ms</td></tr><tr><td>Ray-voxel intersection test</td><td>0.9 ms</td></tr><tr><td>Point sampling</td><td>1 ms</td></tr><tr><td>Volume rendering</td><td>4ms</td></tr><tr><td>Back-propagation</td><td>6 ms</td></tr></table>

We also tested our system on reconstructing outdoor RGB-D sequences captured with a handheld device. In our case, we take these images with an iPhone 13 Pro and an iPad Pro (2020). Our system is able to reconstruct the scene with reasonable accuracy without prior knowledge of the scene. We demonstrate the results in Fig. 8. Please note that iOS devices can only provide depth maps with very low resolution, which limits the reconstruction quality. As can be seen from the figure, our system can reconstruct the scene in various scenarios, and we believe this is a big step towards truly useful neural implicit SLAM systems.

## 5.4 Time and Memory Efficiency

We use a highly efficient multi-process implementation for the parallel tracking and mapping. Since we make a local copy of shared resources (e.g. voxels, features, and the implicit decoder, etc.) for the tracking process each time the map is updated, the probability of resource contention between the two processes is low. The performance hit of accessing the same resource can be further minimized using better engineered lock-free structures, which are not covered in our work.

To study the impact on running time for our sparse voxel-based sampling and rendering method, we profile our system on the synthetic Replica dataset. We measure the average time spent on important components, including voxel allocation, ray-voxel intersection test and volume rendering, etc. The experiment is performed on a single NVidia RTX 3090 video card. The results are listed in Table 4. Please note that tracking and mapping are profiled on a per-iteration basis. As can be seen, our voxel manipulation functions have no significant impact on the running time of the reconstruction pipeline. Depending on the scene complexity, our method can take around 150-200 ms to track a new frame and 450-550 ms for the joint frame and map optimization. In a typical setting, our system can run 5hz for tracking, and 2hz for optimization. However, for more challenging scenarios, the system might run slower.

As explained before, our sparse voxel structure allows us to only allocate voxels occupied by objects and surfaces, which is often only a fraction of the entire environment. We profile our system as well as NICE-SLAM for memory consumption of implicit decoders and voxel embeddings on the Replica office-0 scene. the results are listed in Table 5. Please note that NICE-SLAM uses 4 layers of densely populated voxel grids while we only use one. It can be clearly seen that our method can achieve better reconstruction accuracy while using significantly less memory.

Table 5: Memory consumption for implicit features.
<table><tr><td>Method</td><td>Decoder</td><td>Embedding</td></tr><tr><td>Ours</td><td>1.04MB</td><td>0.149MB</td></tr><tr><td>NICE-SLAM</td><td>0.22MB</td><td>238.88MB</td></tr></table>

## 6 APPLICATIONS

Our Vox-Fusion can not only estimate accurate camera poses in cluttered scenes, but also obtain dense depth maps with fine geometric details and render realistic images through our differentiable rendering. Therefore, it can be applied to many AR and VR applications. For AR applications, our method allows us to place arbitrary virtual objects into reconstructed scenes, and accurately represent the occlusion relationship between real and virtual contents using the rendered depth maps. We show examples of rendered images and AR demo images in Fig. 9. As can be seen, our dense scene representation allows us to handle occlusion between different objects very well in the AR demo.

For VR applications, apart from providing accurate camera tracking result, the realistically rendered color images the realistic rendering ability can be used in free-view virtual scene traveling. Our voxel-based method also makes scene editing much easier since we can remove part of the scene by simply deleting its supporting voxels and the associated feature embeddings. The explicit voxel representation can also be used to perform fast collision detection. We can also control the level of details by splitting and refining feature embeddings as introduced in [14].

## 7 CONCLUSION

We propose Vox-Fusion, a novel dense tracking and mapping system built on voxel-based implicit surface representation. Our Vox-Fusion system supports dynamic voxel creation, which is more suitable for practical scenes. Moreover, we design a multi-process architecture and corresponding strategies for better performance. Experiments show that our method achieves higher accuracy while using smaller memory and faster speed. Currently, our Vox-Fusion method cannot robustly handle dynamic objects and drift in long-time tracking. We consider these as potential future works.

## ACKNOWLEDGMENTS

This work was partially supported by the National Key Research and Development Program of China under Grant 2020AAA0105900.

## REFERENCES

[1] D. Azinović, R. Martin-Brualla, D. B. Goldman, M. Nießner, and J. Thies. Neural RGB-D surface reconstruction. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 6290–6301, 2022.

[2] M. Bloesch, J. Czarnowski, R. Clark, S. Leutenegger, and A. J. Davison. CodeSLAM - learning a compact, optimisable representation for dense visual SLAM. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 2560–2568, 2018.

[3] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel, and J. D. Tardós. ORB-SLAM3: an accurate open-source library for visual, visual-inertial, and multimap SLAM. IEEE Trans. Robotics, 37(6):1874–1890, 2021.

[4] B. Curless and M. Levoy. A volumetric method for building complex models from range images. Annual Conference on Computer Graphics and Interactive Techniques, pp. 303–312, 1996.

[5] J. Czarnowski, T. Laidlow, R. Clark, and A. Davison. DeepFactors: Real-time probabilistic dense monocular slam. IEEE Robotics and Automation Letters, 5:721–728, 2020.

[6] A. Dai, A. X. Chang, M. Savva, M. Halber, T. Funkhouser, and M. Nießner. ScanNet: Richly-annotated 3D reconstructions of indoor scenes. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 2432–2443, 2017.

[7] A. Dai, M. Nießner, M. Zollhöfer, S. Izadi, and C. Theobalt. BundleFusion: Real-time globally consistent 3D reconstruction using on-the-fly surface reintegration. ACM Trans. Graph., 36(3):24:1–24:18, 2017.

[8] H. Li and X. Yang, H. Zhai, Y. Liu, H. Bao, and G. Zhang. Vox-Surf: Voxel-based implicit surface representation. In arXiv preprint arXiv:2208.10925, 2022.

[9] M. Hidenobu, S. Raluca, C. Jan, and J. D. Andrew. CodeMapping: Real-time dense mapping for sparse slam using compact scene representations. IEEE Robotics and Automation Letters, 2021.

[10] J. Huang, S. Huang, H. Song, and S. Hu. DI-Fusion: Online implicit 3d reconstruction with deep priors. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 8932–8941, 2021.

[11] O. Kähler, V. A. Prisacariu, and D. W. Murray. Real-time large-scale dense 3D reconstruction with loop closure. In 14th European Conference on Computer Vision, vol. 9912, pp. 500–516, 2016.

[12] O. Kähler, V. A. Prisacariu, C. Y. Ren, X. Sun, P. H. S. Torr, and D. W. Murray. Very high frame rate volumetric integration of depth images on mobile devices. IEEE Trans. Vis. Comput. Graph., 21(11):1241–1250, 2015.

[13] C. Kerl, J. Sturm, and D. Cremers. Dense visual SLAM for RGB-D cameras. In IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 2100–2106, 2013.

[14] L. Liu, J. Gu, K. Z. Lin, T. Chua, and C. Theobalt. Neural sparse voxel fields. In Annual Conference on Neural Information Processing Systems, 2020.

[15] B. Mildenhall, P. P. Srinivasan, M. Tancik, J. T. Barron, R. Ramamoorthi, and R. Ng. NeRF: representing scenes as neural radiance fields for view synthesis. Commun. ACM, 65(1):99–106, 2022.

[16] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardós. ORB-SLAM: A versatile and accurate monocular SLAM system. IEEE Trans. Robotics, 31(5):1147–1163,2015.

[17] R. Mur-Artal and J. D. Tardós. ORB-SLAM2: an open-source SLAM system for monocular, stereo, and RGB-D cameras. IEEE Trans. Robotics, 33(5):1255–1262, 2017.

[18] R. A. Newcombe and A. J. Davison. Live dense reconstruction with a single moving camera. In IEEEConference on Computer Vision and Pattern Recognition, pp. 1498–1505, 2010.

[19] R. A. Newcombe, S. Izadi, O. Hilliges, D. Molyneaux, D. Kim, A. J. Davison, P. Kohli, J. Shotton, S. Hodges, and A. Fitzgibbon. KinectFusion: Real-time dense surface mapping and tracking. In IEEE International Symposium on Mixed and Augmented Reality, pp. 127–136, 2011.

[20] R. A. Newcombe, S. J. Lovegrove, and A. J. Davison. DTAM: Dense tracking and mapping in real-time. In IEEE International Conference on Computer Vision, pp. 2320–2327, 2011.

[21] M. Nießner, M. Zollhöfer, S. Izadi, and M. Stamminger. Real-time 3D reconstruction at scale using voxel hashing. ACM Transactions on

Graphics, 32(6), 2013.

[22] M. Oechsle, S. Peng, and A. Geiger. UNISURF: unifying neural implicit surfaces and radiance fields for multi-view reconstruction. In IEEE/CVF International Conference on Computer Vision, pp. 5569– 5579, 2021.

[23] K. Rematas, A. Liu, P. P. Srinivasan, J. T. Barron, A. Tagliasacchi, T. Funkhouser, and V. Ferrari. Urban radiance fields. In IEEE Conference on Computer Vision and Pattern Recognition, 2022.

[24] H. Roth and M. Vona. Moving volume kinectfusion. In British Machine Vision Conference, BMVC, pp. 1–11, 2012.

[25] Sara Fridovich-Keil and Alex Yu, M. Tancik, Q. Chen, B. Recht, and A. Kanazawa. Plenoxels: Radiance fields without neural networks. In IEEE Conference on Computer Vision and Pattern Recognition, 2022.

[26] T. Schöps, T. Sattler, and M. Pollefeys. BAD SLAM: bundle adjusted direct RGB-D SLAM. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 134–144, 2019.

[27] H. Strasdat, A. J. Davison, J. M. Montiel, and K. Konolige. Double window optimisation for constant time visual SLAM. IEEE International Conference on Computer Vision, pp. 2352–2359, 2011.

[28] J. Straub, T. Whelan, L. Ma, Y. Chen, E. Wijmans, S. Green, J. J. Engel, R. Mur-Artal, C. Ren, S. Verma, A. Clarkson, M. Yan, B. Budge, Y. Yan, X. Pan, J. Yon, Y. Zou, K. Leon, N. Carter, J. Briales, T. Gillingham, E. Mueggler, L. Pesqueira, M. Savva, D. Batra, H. M. Strasdat, R. D. Nardi, M. Goesele, S. Lovegrove, and R. Newcombe. The Replica dataset: A digital replica of indoor spaces. arXiv preprint arXiv:1906.05797, 2019.

[29] J. Stückler and S. Behnke. Multi-resolution surfel maps for efficient dense 3D modeling and tracking. Journal of Visual Communication and Image Representation, 25:137–147, 2014.

[30] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers. A benchmark for the evaluation of RGB-D SLAM systems. In IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 573– 580, 2012.

[31] E. Sucar, S. Liu, J. Ortiz, and A. J. Davison. iMAP: Implicit mapping and positioning in real-time. In IEEE/CVF International Conference on Computer Vision, pp. 6209–6218. IEEE, 2021.

[32] T. Takikawa, J. Litalien, K. Yin, K. Kreis, C. T. Loop, D. Nowrouzezahrai, A. Jacobson, M. McGuire, and S. Fidler. Neural geometric level of detail: Real-time rendering with implicit 3d shapes. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 11358-11367,2021.

[33] Z. Teed and J. Deng. DROID-SLAM: deep visual SLAM for monocular, stereo, and RGB-D cameras. In Annual Conference on Neural Information Processing Systems, pp. 16558–16569, 2021.

[34] E. Vespa, N. Nikolov, M. Grimm, L. Nardi, P. H. J. Kelly, and S. Leutenegger. Efficient octree-based volumetric SLAM supporting signed-distance and occupancy mapping. IEEE Robotics Autom. Lett., 3(2):1144–1151, 2018.

[35] K. Wang, F. Gao, and S. Shen. Real-time scalable dense surfel mapping. In IEEE International Conference on Robotics and Automation, pp. 6919–6925, 2019.

[36] P. Wang, L. Liu, Y. Liu, C. Theobalt, T. Komura, and W. Wang. Neus: Learning neural implicit surfaces by volume rendering for multi-view reconstruction. In Annual Conference on Neural Information Processing Systems 2021, pp. 27171–27183, 2021.

[37] C. S. Weerasekera, R. Garg, Y. Latif, and I. Reid. Learning Deeply Supervised Good Features to Match for Dense Monocular Reconstruction. Lecture Notes in Computer Science, 11365 LNCS:609–624, 2019.

[38] T. Whelan, S. Leutenegger, R. F. Salas-Moreno, B. Glocker, and A. J. Davison. ElasticFusion: Dense SLAM without a pose graph. In Robotics: Science and Systems, vol. 11, 2015.

[39] X. Yang, Y. Ming, Z. Cui, and A. Calway. FD-SLAM: 3-D reconstruction using features and dense matching. In International Conference on Robotics and Automation, pp. 8040–8046, 2022.

[40] Z. Zhu, S. Peng, V. Larsson, W. Xu, H. Bao, Z. Cui, M. R. Oswald, and M. Pollefeys. NICE-SLAM: neural implicit scalable encoding for SLAM. In IEEE Conference on Computer Vision and Pattern Recognition, pp. 12786–12796, 2022.