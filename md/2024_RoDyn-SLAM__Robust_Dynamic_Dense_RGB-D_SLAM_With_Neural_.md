# RoDyn-SLAM: Robust Dynamic Dense RGB-D SLAM With Neural Radiance Fields

Haochen Jiang , Graduate Student Member, IEEE, Yueming Xu , Graduate Student Member, IEEE, Kejie Li , Jianfeng Feng , and Li Zhang

Abstract—Leveraging neural implicit representation to conduct dense RGB-D SLAM has been studied in recent years. However, this approach relies on a static environment assumption and does not work robustly within a dynamic environment due to the inconsistent observation of geometry and photometry. To address the challenges presented in dynamic environments, we propose a novel dynamic SLAM framework with neural radiance field. Specifically, we introduce a motion mask generation method to filter out the invalid sampled rays. This design effectively fuses the optical flow mask and semantic mask to enhance the precision of motion mask. To further improve the accuracy of pose estimation, we have designed a divide-and-conquer pose optimization algorithm that distinguishes between keyframes and non-keyframes. The proposed edge warp loss can effectively enhance the geometry constraints between adjacent frames. Extensive experiments are conducted on the two challenging datasets, and the results show that RoDyn-SLAM achieves state-of-the-art performance among recent neural RGB-D methods in both accuracy and robustness. Our implementation of the Rodyn-SLAM will be open-sourced to benefit the community.

Index Terms—Deep learning methods, dynamic scene, NeRF, pose estimation, RGB-D SLAM.

## I. INTRODUCTION

D <sup>ENSE</sup> <sup>visual</sup> <sup>simultaneous</sup> <sup>localization</sup> <sup>and</sup> <sup>mapping</sup>(SLAM) is a fundamental task in 3D computer vision and robotics, which has been widely used in various forms in fields such as service robotics, autonomous driving, and augmented/virtual reality (AR/VR). It is defined as reconstructing a dense 3D map in an unknown environment while simultaneously estimating the camera pose, which is regarded as the key to achieving autonomous navigation for robots [1]. However, the majority of methods assume a static environment, limiting the applicability of this technology to more practical scenarios. Thus, it becomes a challenging problem that how the SLAM system can mitigate the interference caused by dynamic objects.

Traditional visual SLAM methods using semantic segmentation prior [2], [3], [4], [5], optical flow motion [6], [7], [8] or re-sampling and residual optimization strategies [9], [10], [11] to remove the outliers under dynamic environments, which can improve the accuracy and robustness of pose estimation. However, re-sampling and optimization methods can only handle small-scale motions and often fail when encountering largescale continuous object movements. Moreover, semantic priors are specific to particular categories and can not represent the real motion state of the observation object. The above learning-based methods often exhibit a domain gap when applied in real-world environments, leading to the introduction of prediction errors.

Recently, dense visual SLAM with neural implicit representation has gained more attention and popularity. This novel map representation is more compact, continuous, efficient, and able to be optimized with differentiable rendering, which has the potential to benefit applications like navigation, planning, and reconstruction. Moreover, the neural scene representations have attractive properties for mapping, including improving noise and outlier handling, geometry estimation capabilities for unobserved scene parts, high-fidelity reconstructions with reduced memory usage, and the ability to generate high-quality static background images from novel views. Existing methods like iMap [12] and NICE-SLAM [13] respectively leverage single MLP and hierarchical feature grids to achieve a consistent geometry representation. However, these methods have limited capacity to capture intricate geometric details. Recent works such as Co-SLAM [14] and ESLAM [15] explore hash encoding or tri-plane representation strategy to enhance the capability of scene representation and the system’s execution efficiency. However, all these above-mentioned methods do not perform well in dynamic scenes. The robustness of these systems significantly decreases, even leading to tracking failures when dynamic objects appear in the environment.

To tackle these problems, we propose a novel NeRF-based RGB-D SLAM that can reliably track camera motion in indoor dynamic environments. One of the key elements to improve the robustness of pose estimation is the motion mask generation algorithm that filters out the sampled rays located in invalid regions. By incrementally fusing the optical flow mask [16], the semantic segmentation mask [17] can become more precise to reflect the true motion state of objects. To further improve the accuracy of pose estimation, we design a divide-and-conquer pose optimization algorithm for keyframes and non-keyframes. While an efficient edge warp loss is used to track camera motions for all keyframes and non-keyframes w.r.t. adjacent frames, only keyframes are further jointly optimized via rendering loss in the global bundle adjustment (GBA).

In summary, our contributions are summarized as follows:

1) To the best of our knowledge, this is the first dynamic neural RGB-D SLAM with joint robust pose estimation and dense reconstruction.

2) In response to the issue of inaccurate semantic priors, we propose a motion mask generation strategy fusing spatial-temporal consistent optical flow masks to improve the robustness of camera pose estimation and quality of static scene reconstruction.

3) Instead of a single frame tracking method, we design a novel mixture pose optimization algorithm utilizing an edge warp loss to enhance the geometry consistency in the non-keyframe tracking stage.

4) We evaluate our method on two challenging dynamic datasets to demonstrate the state-of-the-art performance of our method in comparison to existing NeRF-based RGB-D SLAM approaches.

## II. RELATED WORK

## A. Conventional Visual SLAM With Dynamic Objects Filter

Dynamic object filtering aims to reconstruct the static scene and enhance the robustness of pose estimation. Prior methods can be categorized into two groups: the first one utilizes the re-sampling and residual optimization strategies to remove the outliers [9], [10], [11]. However, these methods can only handle small-scale motions and often fail when encountering largescale continuous object movements. The second group employs the additional prior knowledge, such as semantic segmentation prior [2], [3], [4], [5], [18] or optical flow motion [6], [7], [8] to remove the dynamic objects. However, all these methods often exhibit a domain gap when applied in real-world environments, leading to the introduction of prediction errors. In this letter, we propose a motion mask generation strategy that complements the semantic segmentation mask with warping optical flow masks [16], [19], which is beneficial for reconstructing more accurate static scene maps and reducing observation error.

## B. RGB-D SLAM With Neural Implicit Representation

Neural implicit scene representations, also known as neural fields [20], have garnered significant interest in RGB-D SLAM due to their expressive capacity and minimal memory requirements. iMap [12] firstly adopts a single MLP representation to jointly optimize camera pose and implicit map throughout the tracking and mapping stages. However, it suffers from representation forgetting problems and fails to produce detailed scene geometry. DI-Fusion [21] encodes the scene prior in a latent space and optimizes a feature grid, but it leads to poor reconstruction quality replete with holes. NICE-SLAM [13] leverages a multi-level feature grid enhancing scene representation fidelity and utilizes a local feature update strategy to reduce network forgetting. However, it remains memory-intensive and lacks real-time capability. More recently, existing methods like Vox-Fusion [22], Co-SLAM [14], and ESLAM [15] explore sparse encoding or tri-plane representation strategy to improve the quality of scene reconstruction and the system’s execution efficiency. All these methods have demonstrated impressive results based on the strong assumptions of static scene conditions. The robustness of these systems significantly decreases when dynamic objects appear in the environment. Our SLAM system aims to enhance the accuracy and robustness of pose estimation under dynamic environments, which can expand the application range for the NeRF-based RGB-D SLAM system.

## C. Dynamic Objects Decomposition in NeRFs

As the field of NeRF continues to advance, some researchers are attempting to address the problem of novel view synthesis in the presence of dynamic objects. One kind of solution is to decompose the static background and dynamic objects with different neural radiance fields like [23], [24], [25], [26], [27], [28], [29]. The time dimension will be encoded in latent space, and novel view synthesis is conducted in canonical space. Although these space-time synthesis results are impressive, these techniques rely on precise camera pose input. Robust-Dynrf[30] jointly estimate the static and dynamic radiance fields along with the camera parameters (poses and focal length), which can achieve the unknown camera pose training. However, it can not directly apply to RGB-D SLAM system for large-scale tracking and mapping. Another kind of solution is to ignore the dynamic objects’ influence by utilizing robust loss and optical flow like [28], [31]. Compared to the dynamic NeRF problem, we often focus on the accuracy of pose estimation and the quality of static reconstruction without a long training period. Thus, we also ignore modeling dynamic objects and propose a robust loss function with a novel optimization strategy to recover the static scene map.

## III. METHOD

Given a sequence of RGB-D frames $\{ I _ { i } , D _ { i } \} _ { i = 1 } ^ { N } , I _ { i } \in$ $\mathbb { R } ^ { 3 } , D _ { i } \in \mathbb { R }$ , our method (Fig. 1) aims to simultaneously recover camera poses $\{ \xi _ { i } \} _ { i = 1 } ^ { N } , \xi _ { t } \in \mathbf { \bar { S } } \mathbb { E } ( 3 )$ and reconstruct the static 3D scene map represented by neural radiance fields in dynamic environments. Similar to most modern SLAM systems [32], [33], our system comprises two distinct processes: the tracking process as the frontend and the mapping process as the backend, combined with keyframe management $\{ F _ { k } \} _ { k = 1 } ^ { M }$ and neural implicit map $f _ { \theta } .$ . Invalid sampling rays within dynamic objects are filtered out using a motion mask generation approach. Contrary to the conventional constant-speed motion model in most systems, we introduce an edge warp loss for optimization in non-keyframes to enhance the robustness of pose estimation. Furthermore, keyframe poses and the implicit map representations are iteratively optimized using differentiable rendering.

## A. Implicit Map Representation

We introduce two components of our implicit map representation: an efficient multi-resolution hash encoding $\bar { \mathbb { V } _ { \alpha } }$ to encode the geometric information of the scene, and individual tiny MLP decoders $f _ { \phi }$ to render the color and depth information with truncated signed distance (TSDF) prediction.

a) Multi-resolution hash encoding: We use a multi resolution hash-based feature grid $\mathbb { V } _ { \alpha } = \{ V _ { \alpha } ^ { l } \} _ { l = 1 } ^ { L }$ and individual shallow MLPs to represent the implicit map following Instant-NGP [34]. The spatial resolution of each level is progressively set between the coarsest resolution, denoted as $R _ { \mathrm { m i n } } .$ , and the finest resolution, represented as $R _ { \mathrm { m a x } }$ . Given a sampled point x in 3D space, we compute the interpolate feature $V _ { \alpha } ^ { i } ( \mathbf { x } )$ from each level via trilinear interpolation. To obtain more complementary geometric information, we concat the encoding features from all levels as the input of the MLPs decoder. While simple MLPs can lead to the issue of catastrophic forgetting [12], [13], this mechanism of forgetfulness can be leveraged to eliminate historical dynamic objects.

![](images/2024_RoDyn-SLAM__Robust_Dynamic_Dense_RGB-D_SLAM_With_Neural_/51827f29cbbc309b0d765c852b097aef1a21b5499564ffc69f0db5b92080b3f2.jpg)  
Fig. 1. Schematic illustration of the proposed method. Given a series of RGB-D frames, we simultaneously construct the implicit map and camera pose via multi-resolution hash gird with the geometric loss $\mathcal { L } _ { s d f - m } , \mathcal { L } _ { s d f - t } , \mathcal { L } _ { f s } , \mathcal { L } _ { d e p t h }$ , color loss $\mathcal { L } _ { c o l o r }$ , and edge warp loss $\mathcal { L } _ { e d g e }$

b) Color and depth rendering: To obtain the final formulation of implicit map representation, we adopt a two-layer shallow MLP to predict the geometric and appearance information, respectively. The geometry decoder outputs the predicted SDF value s and a feature vector h at the point x. The appearance decoder outputs the predicted RGB value c. Similar to Co-SLAM [14], we joint encode the coordinate encoding $\gamma ( \mathbf { x } )$ and parametric encoding $V _ { \alpha }$ as:

$$
f _ { \beta } \left( \gamma ( \mathbf { x } ) , V _ { \alpha } ( \mathbf { x } ) \right) \mapsto ( \mathbf { h } , s ) , \quad f _ { \phi } \left( \gamma ( \mathbf { x } ) , \mathbf { h } \right) \mapsto \mathbf { c } ,\tag{1}
$$

where $\{ \alpha , \beta , \phi \}$ are the learnable parameters. Following the volume rendering method in NeRF [20], we accumulate the predicted values along the viewing ray r at the current estimation pose $\xi _ { i }$ to render the color and depth value as:

$$
\hat { C } ( { \bf r } ) = \frac { 1 } { \sum _ { i = 1 } ^ { M } w _ { i } } \sum _ { i = 1 } ^ { M } w _ { i } { \bf c } _ { i } , \hat { D } ( { \bf r } ) = \frac { 1 } { \sum _ { i = 1 } ^ { M } w _ { i } } \sum _ { i = 1 } ^ { M } w _ { i } z _ { i } ,\tag{2}
$$

where $w _ { i }$ is the computed weight along the ray, $\mathbf { c } _ { i }$ and $z _ { i }$ are the color and depth value of the sampling point $\mathbf { x } _ { i }$ . Since we do not directly predict voxel density σ like NeRF, here we need to convert the SDF values $s _ { i }$ into weights w<sub>i</sub>. Thus, we employ a straightforward bell-shaped function [35], formulated as the product of two sigmoid functions $\sigma ( \cdot )$

$$
w _ { i } = \sigma \left( { \frac { s _ { i } } { t r } } \right) \sigma \left( - { \frac { s _ { i } } { t r } } \right) ,
$$

$$
\hat { D } _ { v a r } ( \mathbf { r } ) = \frac { 1 } { \sum _ { i = 1 } ^ { M } w _ { i } } \sum _ { i = 1 } ^ { M } w _ { i } ( \hat { D } - z _ { i } ) ^ { 2 } ,\tag{3}
$$

where tr denotes the truncation distance with TSDF prediction, $\hat { D } _ { v a r }$ is the depth variance along this ray. When possessing GT depth values, we opt for uniform point sampling near the surface rather than employing importance sampling, with the aim of enhancing the efficiency of point sampling.

## B. Motion Mask Generation

For each input keyframe, we select its associated keyframes within a sliding window to compute the dense optical flow warping set S. Note that optical flow estimation is conducted solely on keyframes, thereby optimizing system efficiency. To separate the ego-motion from dynamic objects, we additionally estimate the fundamental matrix F with inliers sampled from the matching set S. Given any matching points $o _ { j i } , o _ { k i }$ within S, we utilize matrix F to compute the Sampson distance between corresponding points and their epipolar lines. By setting a suitable threshold $e _ { t h }$ , we derive the warp mask $\widehat { \mathcal { M } } _ { j , k } ^ { w \bar { f } }$ corresponding to dynamic objects as:

$$
\begin{array} { r } { \widehat { \mathcal { M } } _ { j , k } ^ { w f } : \{ \displaystyle \bigcap _ { i = 1 } ^ { M } \mathbf { 1 } ( \frac { o _ { j i } ^ { T } F o _ { k i } } { \sqrt { A ^ { 2 } + B ^ { 2 } } } < e _ { t h } )  } \\ { \qquad \otimes \mathbf { I } _ { m \times n } | \forall ( o _ { j i } , o _ { k i } ) \in \mathcal { S } \} } \end{array}\tag{4}
$$

where A, B denotes the coefficients of the epipolar line, and m, n represents the size of the warp mask, aligning with the current frame image’s dimensions. Additionally, j and k stand for the keyframe ID, illustrating the optical flow mask warping process from the k-th to the j-th keyframe. As illustrated in Fig. 1, to derive a more precise motion mask, we consider the spatial coherence of dynamic object motions within a sliding window of length N and iteratively optimize the current motion mask. Subsequently, we integrate the warp mask and segment mask to derive the final motion mask $\widehat { \mathcal { M } } _ { j }$ as:

$$
\widehat { \mathcal { M } } _ { j } = \widehat { \mathcal { M } } _ { j , k } ^ { w f } \otimes \widehat { \mathcal { M } } _ { j , k - 1 } ^ { w f } \otimes \widehat { \mathcal { M } } _ { j , k - 2 } ^ { w f } \cdots \otimes \widehat { \mathcal { M } } _ { j , k - N } ^ { w f } \cup \widehat { \mathcal { M } } _ { j } ^ { s g } ,\tag{5}
$$

where $\otimes$ represents the mask fusion operation, which is applied when pixels corresponding to a specific motion mask have been continuously observed for a duration exceeding a certain threshold $o _ { t h }$ within a sliding window. Note that we do not focus on the specific structure of the segment or optical flow network. Instead, we aim to introduce a general motion mask fusing method for application in NeRF-based SLAMs. We believe that there is potential for integrating this approach into any visual SLAM system.

## C. Joint Optimization

We introduce the details on optimizing the implicit scene representation and camera pose. Given a set of frames ${ \mathcal F } ,$ we only predict the current camera pose represented with lie algebra $\xi _ { i }$ in tracking process. Moreover, we utilize the global bundle adjustment (GBA) [36], [37], [38] to jointly optimize the sampled camera pose and the implicit mapping.

1) Photometric Rendering Loss: To jointly optimize the scene representation and camera pose, we render depth and color in independent view as (6) comparing with the proposed ground truth map:

$$
\mathcal { L } _ { r g b } = \frac { 1 } { M } \sum _ { i = 1 } ^ { M } \Big \Vert \left( \hat { C } ( \mathbf { r } ) - C ( \mathbf { r } ) \right) \cdot \widehat { \mathcal { M } } _ { i } ( \mathbf { r } ) \Big \Vert _ { 2 } ^ { 2 } ,
$$

$$
\mathcal { L } _ { d e p t h } = \frac { 1 } { N _ { d } } \sum _ { \mathbf { r } \in N _ { d } } \left\| \left( \frac { \hat { D } ( \mathbf { r } ) - D ( \mathbf { r } ) } { \sqrt { \hat { D } _ { v a r } ( \mathbf { r } ) } } \right) \cdot \widehat { \mathcal { M } } _ { i } ( \mathbf { r } ) \right\| _ { 2 } ^ { 2 }\tag{6}
$$

where $C ( \mathbf { r } )$ and $D ( \mathbf { r } )$ denote the ground truth color and depth map corresponding with the given pose,s respectively. M represents the number of sampled pixels in the current image. Note that only rays with valid depth value $N _ { d }$ are considered in $\mathcal { L } _ { d e p t h }$ . In contrast to existing methods, we introduce the motion mask $\widehat { \mathcal { M } } _ { j }$ to remove sampled pixels within the dynamic object region effectively. Moreover, to improve the robustness of pose estimation, we add the depth variance $\hat { D } _ { v a r }$ to reduce the weight of depth outliers.

2) Geometric Constraints: Following the practice [35], assuming a batch of rays M within valid motion mask regions are sampled, we directly leverage the free space loss with truncation tr to restrict the SDF values $s ( \bf { x } _ { i } )$ as:

$$
\mathcal { L } _ { f s } = \frac { 1 } { M } \sum _ { i = 1 } ^ { M } \frac { 1 } { | \mathscr { R } _ { f s } | } \sum _ { i \in \mathscr { R } _ { f s } } ( s ( \mathbf { x } _ { i } ) - t r ) ^ { 2 } , \lceil u _ { i } , v _ { i } \rceil \subseteq ( \widehat { \mathscr { M } } _ { i } = 1 ) .\tag{7}
$$

It is unreasonable to employ a fixed truncation value to optimize camera pose and SDF values in dynamic environments simultaneously. To reduce the artifacts in occluded areas and enhance the accuracy of reconstruction, we further divide the entire truncation region near the surface into middle and tail truncation regions inspired by ESLAM [15] as:

$$
\mathcal { L } _ { s d f } = \frac { 1 } { M } \sum _ { i = 1 } ^ { M } \frac { 1 } { | \mathscr { R } _ { t r } | } \sum _ { i \in \mathcal { R } _ { t r } } \left( s ( \mathbf { x } _ { i } ) - ( D [ u _ { i } , v _ { i } ] - T \cdot t r ) \right) ^ { 2 } ,\tag{8}
$$

where T denotes the ratio ofthe entire truncation length occupied by the middle truncation, $[ u _ { i } , v _ { i } ] \subseteq ( \widehat { { \mathcal { M } } } _ { i } = 1 )$ . Note that we use the different weights to adjust the importance of middle and tail truncation in camera tracking and mapping process. The overall loss function is finally formulated as the following minimization,

$$
\begin{array} { r l } & { \mathcal { P } ^ { * } = \underset { \mathcal { P } } { \arg \operatorname* { m i n } } ~ \lambda _ { 1 } \mathcal { L } _ { r g b } + \lambda _ { 2 } \mathcal { L } _ { d e p t h } + \lambda _ { 3 } \mathcal { L } _ { f s } } \\ & { ~ + \lambda _ { 4 } \mathcal { L } _ { s d f - m } + \lambda _ { 5 } \mathcal { L } _ { s d f - t } , } \end{array}\tag{9}
$$

where $\mathcal { P } = \{ \theta , \phi , \alpha , \beta , \gamma , \xi _ { i } \}$ is the list of parameters being optimized, including fields feature, decoders, and camera pose.

3) Camera Tracking Process: The construction of implicit maps within dynamic scenes often encounters substantial noise and frequently exhibits a lack of global consistency. Existing methods [13], [14], [15], [39] rely solely on rendering loss for camera pose optimization, which makes the system vulnerable and prone to tracking failures. To solve this problem, we introduce edge warp loss to enhance geometry consistency in data association between adjacent frames.

Edge reprojection loss: For a 2D pixel p in frame i, we first define the warp operation in a similar spirit as DIM-SLAM [39] to reproject it onto frame j as follows:

$$
p _ { i  j } = f _ { w a r p } ( \xi _ { j i } , p _ { i } , D ( p _ { i } ) ) = K T _ { j i } ( K ^ { - 1 } D ( p _ { i } ) p _ { i } ^ { h o m o } ) ,\tag{(10}
$$

where K and $\pmb { T } _ { j i }$ represent the intrinsic matrix and the transformation matrix between frame i and frame j, respectively. $\pmb { p } _ { i } ^ { h o m o } = ( u , v , 1 )$ is the homogeneous coordinate of $\mathbf { \nabla } p _ { i }$ . Since the edge are detected once and do not change forwards, we can precompute the distance map (DT) [40] to describe the projection error with the closest edge. For a edge set $\mathcal { E } _ { i }$ in frame i, we define the edge loss $\mathcal { L } _ { e d g e }$ as

$$
\mathcal { L } _ { e d g e } = \sum _ { \pmb { p _ { i } } \in \mathcal { E } _ { i } } \rho ( \mathcal { D } _ { j } ( f _ { w a r p } ( \xi _ { j i } , \pmb { p _ { i } } , D ( \pmb { p _ { i } } ) ) ) \cdot \widehat { \mathcal { M } } _ { j } ) ,\tag{11}
$$

where $\mathcal { D } _ { j }$ denotes the DT map in frame $j ,$ and the $\rho$ is a Huber weight function to reduce the influence of large residuals. Moreover, we drop a potential outlier if the projection distance error is greater than $\delta _ { e }$ . The pose optimization problem is finally formulated as the following minimization,

$$
\xi _ { j i } ^ { * } = \underset { \xi _ { j i } } { \mathrm { a r g m i n } } \ \lambda \mathcal { L } _ { e d g e } , \ \mathrm { i f } \ j \notin \mathcal { K }\tag{12}
$$

To further improve the accuracy and stability of pose estimation, we employ distinct methods for tracking keyframes and non-keyframes in dynamic scenes. Keyframe pose estimation utilizes the edge loss to establish the initial pose, followed by optimization (9). For non-keyframe pose estimation, we optimize the current frame’s pose related to the nearest keyframe (12).

## IV. EXPERIMENTS

Datasets: We evaluate our method on two real-world public datasets: TUM RGB-D dataset [41] and BONNRGB-D Dynamic dataset [11]. Both datasets capture indoor scenes using a handheld camera and provide the ground-truth trajectory.

![](images/2024_RoDyn-SLAM__Robust_Dynamic_Dense_RGB-D_SLAM_With_Neural_/e9c3ed10a9e19a93227d9047b492b0e7ce80216472db6428a30817999028f993.jpg)  
Fig. 2. Qualitative results of the generation motion mask. By iteratively optimizing the optical flow mask, the fused optical mask can be more precise without noises. The semantic mask can only identify dynamic objects within predefined categories. The best results are obtained with our method.

Metrics: For evaluating pose estimation, we adopt the RMSE and STD ofAbsolute Trajectory Error (ATE) [41]. The estimated trajectory is oriented to align with the ground truth trajectory using the unit quaternions algorithm [42] before evaluation. We also use three metrics which are widely used for scene reconstruction evaluation following [13], [39]: i) Accuracy (cm), ii) Completion (cm), iii) Completion Ratio (< 5 cm %). Since the BONN-RGBD only provided the ground truth point cloud, we randomly sampled the 200,000 points from both the ground truth point cloud and the reconstructed mesh surface to compute the metrics. We remove unobserved regions that are outside of any camera’s viewing frustum and conduct extra mesh culling to remove the noisy points external to the target scene [14].

Implementation details: We adopt Co-SLAM [14] as the baseline in our experiments and run our RoDyn-SLAM on an high-performance workstation with a 3.4 GHz Intel Core i7-13700 K CPU and RTX 3090Ti GPU at 10 FPS (without optical flow mask) on the Tum datasets, which takes roughly 4 GB of memory in total. Specific to implementation details, we sample $N _ { t } = 1 0 2 4$ rays and $N _ { p } = 8 5$ points along each camera ray with 20 iterations for tracking and 2048 pixels from every 5 th frames for global bundle adjustment. We set loss weight $\lambda _ { 1 } = 1 . 0 , \lambda _ { 2 } = 0 . 1 , \lambda _ { 3 } = 1 0 , \lambda _ { 4 } = 2 0 0 0 , \lambda _ { 5 } = 5 0 0$ to train our model with Adam [43] optimizer. In the motion mask generation method, we utilize Oneformer [17] for semantic segmentation prior generation and RAFT-GMA [16] for optical flow prediction. In the edge extraction process, we utilize the Canny [44] edge detection algorithm with double-threshold. For the sake of comparison fairness, we employ the same keyframe insertion strategy as Co-SLAM [14].

## A. Evaluation ofGenerating Motion Mask

Fig. 2 shows the qualitative results of the generated motion mask. We evaluated our method on the balloon and move\_no\_box2 sequence of the BONN dataset. In these sequences, in addition to the movement of the person, there are also other dynamic objects associated with the person, such as balloons and boxes. As shown in Fig. 2 final mask part, our methods can significantly improve the accuracy of motion mask segmentation and effectively mitigate both false positives and false negatives issues in motion segmentation.

![](images/2024_RoDyn-SLAM__Robust_Dynamic_Dense_RGB-D_SLAM_With_Neural_/d5b761228b62851619c4a4877afdc19658ede04a84d01a7c48ccc57b4dd2fd5f.jpg)  
Fig. 3. Visual comparison of the reconstructed meshes on the BONN and TUM RGB-D datasets. Our results are more complete and accurate without the dynamic object floaters.

## B. Evaluation of Mapping and Tracking

a) Mapping:To better demonstrate the performance of our proposed system in dynamic scenes, we evaluate the mapping results from both qualitative and quantitative perspectives. Since the majority of dynamic scene datasets do not provide ground truth for static scene reconstruction, we adopt the BONN dataset to conduct quantitative analysis experiments. We compare our RoDyn-SLAM method against traditional dynamic SLAM method like ReFusion [11] and current state-of-the-art NeRF-based methods with RGB-D sensors, including NICE-SLAM [13], iMap [12], Vox-Fusion [22], ESLAM [15], and Co-SLAM [14], which are open source. The evaluation metrics have been mentioned above at the beginning of Section IV.

As shown in Table I, our method outperforms most of the neural RGB-D slam systems on accuracy and completion. To improve the accuracy of pose estimation, we filter the invalid depth, which may reduce the accuracy metric on mapping evaluation. The visual comparison ofreconstructed meshes with other methods [14], [15] is provided in Fig. 3. Note that the TUM dataset does not provide ground truth meshes for evaluating mapping quality. Our methods can generate a more accurate static mesh than other compared methods. Since the baseline methods [14] adopt the hash encoding to represent the implicit map, it may exacerbate the issue of the hash collisions in dynamic scenes and generate the hole in the reconstruction map.

TABLE I  
QUANTITATIVE RESULTS ON SEVERAL DYNAMIC SCENE SEQUENCES IN THE BONN-RGBD DATASET
<table><tr><td></td><td></td><td>ball</td><td>ball2</td><td> $_ { \mathrm { p } S \_ \mathrm { t r k } }$ </td><td> $\mathtt { p s \_ t r k } 2$ </td><td> $\mathtt { m v \_ b o x 2 }$ </td><td>Avg.</td></tr><tr><td rowspan="3">ReFusion [11]</td><td>Acc.[cm]↓</td><td>8.20</td><td>7.85</td><td>46.89</td><td>78.47</td><td>9.07</td><td>30.10</td></tr><tr><td>Comp.[cm]↓</td><td>12.58</td><td>11.69</td><td>104.04</td><td>166.63</td><td>13.09</td><td>61.61</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>31.57</td><td>32.18</td><td>13.93</td><td>10.55</td><td>35.51</td><td>24.75</td></tr><tr><td rowspan="3">iMAP* [12]</td><td>Acc.[cm]↓</td><td>16.68</td><td>31.20</td><td>35.38</td><td>54.16</td><td>17.01</td><td>30.89</td></tr><tr><td>Comp.[cm]↓</td><td>27.32</td><td>30.14</td><td>201.38</td><td>107.28</td><td>20.499</td><td>77.32</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>25.68</td><td>21.91</td><td>11.54</td><td>12.63</td><td>24.86</td><td>19.32</td></tr><tr><td rowspan="3">NICE-SLAM [13]</td><td>Acc.[cm]↓</td><td>X</td><td>24.30</td><td>43.11</td><td>74.92</td><td>17.56</td><td>39.97</td></tr><tr><td>Comp.[cm]↓</td><td>X</td><td>16.65</td><td>117.95</td><td>172.20</td><td>18.19</td><td>81.25</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>X</td><td>29.68</td><td>15.89</td><td>13.96</td><td>32.18</td><td>22.93</td></tr><tr><td rowspan="3">Vox-Fusion [22]</td><td>Acc.[cm]↓</td><td>85.70</td><td>89.27</td><td>208.03</td><td>162.61</td><td>40.64</td><td>117.25</td></tr><tr><td>Comp.[cm]↓</td><td>55.01</td><td>29.78</td><td>279.42</td><td>229.79</td><td>28.40</td><td>124.48</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>3.88</td><td>11.76</td><td>2.17</td><td>4.55</td><td>14.69</td><td>7.41</td></tr><tr><td rowspan="3">Co-SLAM [14]</td><td>Acc.[cm]↓</td><td>10.61</td><td>14.49</td><td>26.46</td><td>26.00</td><td>12.73</td><td>18.06</td></tr><tr><td>Comp.[cm]↓</td><td>10.65</td><td>40.23</td><td>124.86</td><td>118.35</td><td>10.22</td><td>60.86</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>34.10</td><td>3.21</td><td>2.05</td><td>2.90</td><td>39.10</td><td>16.27</td></tr><tr><td rowspan="3">ESLAM [15]</td><td>Acc.[cm]↓</td><td>17.17</td><td>26.82</td><td>59.18</td><td>89.22</td><td>12.32</td><td>40.94</td></tr><tr><td>Comp.[cm]↓</td><td>9.11</td><td>13.58</td><td>145.78</td><td>186.65</td><td>10.03</td><td>73.03</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>47.44</td><td>47.94</td><td>20.53</td><td>17.33</td><td>41.41</td><td>34.93</td></tr><tr><td rowspan="3">Ours(RoDyn-SLAM)</td><td>Acc.[cm]↓</td><td>10.60</td><td>13.36</td><td>10.21</td><td>13.77</td><td>11.34</td><td>11.86</td></tr><tr><td>Comp.[cm]↓</td><td>7.15</td><td>7.87</td><td>27.70</td><td>18.97</td><td>6.86</td><td>13.71</td></tr><tr><td>Comp. Ratio[≤ 5cm%]↑</td><td>47.58</td><td>40.91</td><td>34.13</td><td>32.59</td><td>45.37</td><td>40.12</td></tr></table>

“X" denotes the tracking failures. The best results are bolded, and the second best results are indicated with an underline

TABLE II  
CAMERA TRACKING RESULTS ON SEVERAL DYNAMIC AND STATIC SCENE SEQUENCES IN THE TUM RGB-D DATASET
<table><tr><td rowspan="2">Methods</td><td rowspan="2">Dense</td><td colspan="7">Dynamic</td><td rowspan="2"></td><td colspan="4">Static</td><td rowspan="2"></td><td rowspan="2">Avg.</td></tr><tr><td> $\pounds _ { 3 } / \mathrm { w k \_ x y z }$ </td><td></td><td> $\pm 3 / \mathrm { w k \_ h f }$ </td><td></td><td></td><td> $\mathbb { f } 3 / { \mathbb { w } } { \mathbb { k } } _ { - } { \mathrm { s t } }$ </td><td> $\mathbb { f } 3 / \mathrm { s t \_ h f }$ </td><td>f1/xyz</td><td></td><td>f1/rpy</td><td></td></tr><tr><td>Traditional SLAM methods</td><td>T/F</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td></tr><tr><td>ORB-SLAM3 [10]</td><td>x</td><td>28.1</td><td>12.2</td><td>30.5</td><td>9.0</td><td>2.0</td><td>1.1</td><td>2.6</td><td>1.6</td><td>1.1</td><td>0.6</td><td>2.2</td><td>1.3</td><td>11.1</td><td>4.3</td></tr><tr><td>DVO-SLAM [46]</td><td>√</td><td>59.7</td><td></td><td>52.9</td><td></td><td>21.2</td><td>一</td><td>6.2</td><td></td><td>1.1</td><td></td><td>2.0</td><td></td><td>22.9</td><td></td></tr><tr><td>DynaSLAM [3]</td><td>× &gt;</td><td>1.7</td><td>=</td><td>2.6</td><td>1</td><td>0.7</td><td>–</td><td>2.8</td><td>-</td><td>=</td><td></td><td>=</td><td></td><td>2.0</td><td></td></tr><tr><td>ReFusion [11]</td><td></td><td>9.9</td><td></td><td>10.4</td><td></td><td>1.7</td><td>=</td><td>11.0</td><td></td><td></td><td></td><td></td><td></td><td>8.3</td><td></td></tr><tr><td>NeRF based SLAM methods</td><td>T/F</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td></tr><tr><td>iMAP* [12]</td><td></td><td>111.5</td><td>43.9</td><td>X</td><td>X</td><td>137.3</td><td>21.7</td><td>93.0</td><td>35.3</td><td>7.9</td><td>7.3</td><td>16.0</td><td>13.8</td><td>73.2</td><td>24.4</td></tr><tr><td>NICE-SLAM [13]</td><td></td><td>113.8</td><td>42.9</td><td>X</td><td>X</td><td>88.2</td><td>27.8</td><td>45.0</td><td>14.4</td><td>4.6</td><td>3.8</td><td>3.4</td><td>2.5</td><td>51</td><td>18.3</td></tr><tr><td>Vox-Fusion [22]</td><td></td><td>146.6</td><td>32.1</td><td>X</td><td>X</td><td>109.9</td><td>25.5</td><td>89.1</td><td>28.5</td><td>1.8</td><td>0.9</td><td>4.3</td><td>3.0</td><td>70.4</td><td>18</td></tr><tr><td>Co-SLAM [14]</td><td></td><td>51.8</td><td>25.3</td><td>105.1</td><td>42.0</td><td>49.5</td><td>10.8</td><td>4.7</td><td>2.2</td><td>2.3</td><td>1.2</td><td>3.9</td><td>2.8</td><td>36.3</td><td>14.1</td></tr><tr><td>ESLAM [15]</td><td></td><td>45.7</td><td>28.5</td><td>60.8</td><td>27.9</td><td>93.6</td><td>20.7</td><td>3.6</td><td>1.6</td><td>1.1</td><td>0.6</td><td>2.2</td><td>1.2</td><td>34.5</td><td>13.5</td></tr><tr><td>RoDyn-SLAM(Ours)</td><td></td><td>8.3</td><td>5.5</td><td>5.6</td><td>2.8</td><td>1.7</td><td>0.9</td><td>4.4</td><td>2.2</td><td>1.5</td><td>0.8</td><td>2.8</td><td>1.5</td><td>4.1</td><td>2.3</td></tr></table>

“\*" denotes the version reproduced by NICE-SLAM. “-" denote the absence of mention. The metric unit is [cm]. The best results are in bold.

b) Tracking:To evaluate the accuracy of camera tracking in dynamic scenes, we compare our methods with the recent neural RGB-D SLAM methods and traditional SLAM methods like ORB-SLAM3 [10], DVO-SLAM [45], Droid-SLAM [46], and traditional dynamic SLAM like DynaSLAM [3], and ReFusion [11].

As shown in Table II, we report the results on three highly dynamic sequences, one slightly dynamic sequence, and two static sequences from TUM RGB-D dataset. Our system achieves advanced tracking performance owing to the motion mask filter and edge-based optimization algorithm under dynamic environment. Compared with our baseline methods Co-SLAM [14], our method does not compromise the performance of the original SLAM methods in terms of tracking and mapping in static scenes. In fact, it achieves competitive results. Notably, our proposed optimization algorithm is not restricted to a specific slam system. Thus, it can also be applied to other neural rgb-d slam methods to improve the data association between the inter-frame. We have also evaluated the tracking performance on the more complex and challenging BONN RGB-D dataset, as illustrated in Table III. In more complex and challenging scenarios, our method has achieved superior results. While there is still some gap compared to the more mature and robust traditional dynamic SLAM methods, our systems can drive the dense and textural reconstruction map to finish the more complex robotic navigation tasks.

## C. Ablation Study

To demonstrate the effectiveness of the proposed methods in our system, we perform the ablation studies on seven representative sequences of the BONN dataset, including person\_tracking, balloon, balloon\_track, move\_no\_box. As the semantic prior in the TUM dataset already covers most of the motion categories, we did not conduct ablation studies on this dataset. We compute the average ATE and

TABLE III  
CAMERA TRACKING RESULTS ON SEVERAL DYNAMIC SCENE SEQUENCES IN THE BONN RGB-D DATASET
<table><tr><td>Methods</td><td>Dense</td><td colspan="2">balloon</td><td colspan="2">balloon2</td><td colspan="2"> $_ { \mathrm { p s \_ t r a c k } }$ </td><td colspan="2"> $\mathtt { p s \_ t r a c k } 2$ </td><td colspan="2"> $_ { \textrm b a l 1 \_ t r a c k }$ </td><td colspan="2"> $\operatorname* { m v } _ { - } \mathrm { b o x } 2$ </td><td colspan="2">Avg.</td></tr><tr><td>Traditional SLAM methods</td><td>T/F</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td>S.D.</td></tr><tr><td>ORB-SLAM3 [10]</td><td>x</td><td>5.8</td><td>2.8</td><td>17.7</td><td>8.6</td><td>70.7</td><td>32.6</td><td>77.9</td><td>43.8</td><td>3.1</td><td>1.6</td><td>3.5</td><td>1.5</td><td>29.8</td><td>15.2</td></tr><tr><td>Droid-VO [47]</td><td>√</td><td>5.4</td><td></td><td>4.6</td><td></td><td>21.34</td><td></td><td>46.0</td><td></td><td>8.9</td><td></td><td>5.9</td><td></td><td>15.4</td><td></td></tr><tr><td>DynaSLAM [3]</td><td>x</td><td>3.0</td><td></td><td>2.9</td><td></td><td>6.1</td><td></td><td>7.8</td><td></td><td>4.9</td><td></td><td>3.9</td><td></td><td>4.8</td><td></td></tr><tr><td>ReFusion [11]</td><td>√</td><td>17.5</td><td></td><td>25.4</td><td></td><td>28.9</td><td></td><td>46.3</td><td></td><td>30.2</td><td></td><td>17.9</td><td></td><td>27.7</td><td></td></tr><tr><td>NeRF based SLAM methods</td><td>T/F</td><td>ATE</td><td>S.D.</td><td></td><td>S.D.</td><td>ATE</td><td>S.D.</td><td>ATE</td><td></td><td>ATE</td><td></td><td></td><td>S.D.</td><td></td><td>S.D.</td></tr><tr><td>iMAP* [12]</td><td>√</td><td>14.9</td><td>5.4</td><td>ATE 67.0</td><td>19.2</td><td>28.3</td><td>12.9</td><td>52.8</td><td>S.D. 20.9</td><td>24.8</td><td>S.D. 11.2</td><td>ATE 28.3</td><td>35.3</td><td>ATE 36.1</td><td>17.5</td></tr><tr><td>NICE-SLAM [13]</td><td>√</td><td>X</td><td>X</td><td>66.8</td><td>20.0</td><td>54.9</td><td>27.5</td><td>45.3</td><td>17.5</td><td>21.2</td><td>13.1</td><td>31.9</td><td>13.6</td><td>44.1</td><td>18.4</td></tr><tr><td></td><td>√</td><td>65.7</td><td>30.9</td><td>82.1</td><td>52.0</td><td>128.6</td><td>52.5</td><td>162.2</td><td>46.2</td><td>43.9</td><td>16.5</td><td>47.5</td><td>19.5</td><td>88.4</td><td>36.3</td></tr><tr><td>Vox-Fusion [22]</td><td>√</td><td>28.8</td><td>9.6</td><td>20.6</td><td>8.1</td><td>61.0</td><td>22.2</td><td>59.1</td><td>24.0</td><td>38.3</td><td>17.4</td><td>70.0</td><td>25.5</td><td>46.3</td><td>17.8</td></tr><tr><td>Co-SLAM [14]</td><td>√</td><td>22.6</td><td>12.2</td><td>36.2</td><td>19.9</td><td>48.0</td><td>18.7</td><td>51.4</td><td>23.2</td><td>12.4</td><td>6.6</td><td>17.7</td><td>7.5</td><td>31.4</td><td>14.7</td></tr><tr><td>ESLAM [15] RoDyn-SLAM(Ours)</td><td>S</td><td>7.9</td><td>2.7</td><td>11.5</td><td>6.1</td><td>14.5</td><td>4.6</td><td>13.8</td><td>3.5</td><td>13.3</td><td>4.7</td><td>12.6</td><td>4.7</td><td>12.3</td><td>4.4</td></tr></table>

“\*” denotes the version reproduced by NICE-SLAM. “-" denote the absence of mention. The metric unit is [cm]. The best results are in bold.

TABLE IV  
ABLATION STUDY OF THE PROPOSED METHOD IN OUR SYSTEMS
<table><tr><td></td><td>w/o Seg</td><td>w/o Flow</td><td>w/o Edge</td><td>RoDyn-SLAM</td></tr><tr><td>ATE RMSE (m) ↓</td><td>0.3089</td><td>0.1793</td><td>0.2056</td><td>0.1354</td></tr><tr><td>STD (m) ↓</td><td>0.1160</td><td>0.0739</td><td>0.0829</td><td>0.0543</td></tr></table>

The best results are in bold.

TABLE V  
TIME COMPARISON OF DIFFERENT METHODS IN OUR SYSTEMS
<table><tr><td></td><td>NICE-SLAM</td><td>ESLAM</td><td>Co-SLAM</td><td>RoDyn-SLAM</td></tr><tr><td>Tracking (ms) ↓</td><td>3535.67</td><td>1002.52</td><td>174.47</td><td>159.06</td></tr><tr><td>Mapping (ms) ↓</td><td>3055.58</td><td>703.69</td><td>565.50</td><td>675.08</td></tr></table>

The best results are in bold.

STD results to show how different methods affect the overall system performance. The results presented in Table IV demonstrate that all the proposed methods are effective in camera tracking. This suggests that fusing the optical flow mask and semantic motion mask can promote better pose estimation. At the same time, leveraging a divide-and-conquer pose optimization can effectively improve the robustness and accuracy of camera tracking.

## D. Time Consumption Analysis

As shown in Table V, we report time consumption (per frame) of the tracking and mapping without computing semantic segmentation and optical flow. Note that we pay more attention to evaluating the impact of our proposed methods on the baseline SLAM system’s runtime. All the results were obtained using an experimental configuration of sampled 1024 pixels and 20 iterations for tracking and 2048 pixels and 40 iterations for mapping, with an RTX 3090 GPU in our laboratory server. Despite incorporating additional methods for handling dynamic objects, our system maintains a comparable level of computational cost to that of Co-SLAM. We also evaluate the time efficiency of our used optical flow and semantic segmentation network in our laboratory server, which required 97 ms and 163 ms respectively to process a single frame. Since semantic segmentation results can be pre-generated, the overall execution time of our optical flow fusion module is approximately 247 ms. Note that Rodyn-SLAM is not optimized for real-time operation. With ongoing advancements in these research fields and improvements in computing power, the processing speeds for optical flow and semantic segmentation are expected to increase, ensuring they do not become bottlenecks for our method.

![](images/2024_RoDyn-SLAM__Robust_Dynamic_Dense_RGB-D_SLAM_With_Neural_/26573d7fb210d338ef940fba94a0ceaa60a032b9f3f3dd775463e31b47396c56.jpg)  
Fig. 4. Visual comparison of the rendering image on the TUM and BONN datasets.

## E. Visualization of Rendering Static Implicit Map

To further demonstrate the performance of static scene reconstruction, we compared the rendered image with the ground truth pose obtained from the generated static implicit map. We selected two challenging sequences, person\_track from the BONN dataset and f3\_walk\_xyzfrom the TUM RGB-D dataset. As shown in Fig. 4, our method achieves a favorable rendering performance while enjoying the benefits of the proposed methods. Meanwhile, our methods can fill the hole which can not be captured in the original depth image. It can make the scene representation smoother and more complementing. We observed variations in rendering capabilities among different methods, which resulted in differences in the presentation quality. Note that our methods can be incrementally implemented in any existing baseline methods. Therefore, we don’t focus on the actual performance ofthe code base Co-SLAM [14] but solely on the proposed methods’s ability and effectiveness in addressing dynamic scene challenges.

## V. CONCLUSION

We present RoDyn-SLAM, a novel dense RGB-D SLAM with neural implicit representation for dynamic environments. The proposed system is able to estimate camera poses and recover 3D geometry in this challenging setup thanks to the motion mask generation that successfully filters out dynamic regions. To further improve the stability and robustness of pose optimization, a divide-and-conquer pose optimization algorithm is designed to enhance the geometry consistency between keyframe and non-keyframe with the edge warp loss. The experiment results demonstrate that RoDyn-SLAM achieves state-of-the-art performance among recent neural RGB-D methods in both accuracy and robustness. In future work, a more robust keyframe management method is a promising direction to improve the system further.

## REFERENCES

[1] J. J. Leonard and H. F. Durrant-Whyte, “Simultaneous map building and localization for an autonomous mobile robot,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 1991, pp. 1442–1447.

[2] C. Yu et al., “DS-SLAM: A semantic visual SLAM towards dynamic environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 1168–1174.

[3] B. Bescos, J. M. Fácil, J. Civera, and J. Neira, “DynaSLAM: Tracking, mapping, and inpainting in dynamic scenes,” IEEE Robot. Automat. Lett., vol. 3, no. 4, pp. 4076–4083, Oct. 2018.

[4] L. Xiao, J. Wang, X. Qiu, Z. Rong, and X. Zou, “Dynamic-SLAM: Semantic monocular visual localization and mapping based on deep learning in dynamic environment,” Robot. Auton. Syst., vol. 117, pp. 1–16, 2019.

[5] J. Zhang, M. Henein, R. Mahony, and V. Ila, “VDO-SLAM: A visual dynamic object-aware SLAM system,” 2020, arXiv:2005.11052.

[6] Y. Sun, M. Liu, and M. Q.-H. Meng, “Motion removal for reliable RGB-D SLAM in dynamic environments,” Robot. Auton. Syst., vol. 108, pp. 115–128, 2018.

[7] J. Cheng, Y. Sun, and M. Q.-H. Meng, “Improving monocular visual slam in dynamic environments: An optical-flow-based approach,” Adv. Robot., vol. 33, pp. 576–589, 2019.

[8] T. Zhang, H. Zhang, Y. Li, Y. Nakamura, and L. Zhang, “FlowFusion: Dynamic dense RGB-D slam based on optical flow,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 7322–7328.

[9] R. Mur-Artal and J. D. Tardós, “ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1255–1262, Oct. 2017.

[10] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. Montiel, and J. D. Tardós, “ORB-SLAM3: An accurate open-source library for visual, visual–inertial, and multimap SLAM,” IEEE Trans. Robot., vol. 37, no. 6, pp. 1874–1890, Dec. 2021.

[11] E. Palazzolo, J. Behley, P. Lottes, P. Giguere, and C. Stachniss, “ReFusion: 3D reconstruction in dynamic environments for RGB-D cameras exploiting residuals,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 7855–7862.

[12] E. Sucar, S. Liu, J. Ortiz, and A. Davison, “iMAP: Implicit mapping and positioning in real-time,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 6209–6218.

[13] Z. Zhu et al., “NICE-SLAM: Neural implicit scalable encoding for SLAM,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2022, pp. 12776–12786.

[14] H. Wang, J. Wang, and L. Agapito, “Co-SLAM: Joint coordinate and sparse parametric encodings for neural real-time SLAM,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2023, pp. 13293–13302.

[15] M. M. Johari, C. Carta, and F. Fleuret, “ESLAM: Efficient dense SLAM system based on hybrid representation of signed distance fields,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2023, pp. 17408–17419.

[16] S. Jiang, D. Campbell, Y. Lu, H. Li, and R. Hartley, “Learning to estimate hidden motions with global motion aggregation,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 9752–9761.

[17] J. Jain, J. Li, M. Chiu, A. Hassani, N. Orlov, and H. Shi, “OneFormer: One transformer to rule universal image segmentation,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2023, pp. 2989–2998.

[18] B. Bescos, C. Campos, J. D. Tardós, and J. Neira, “DynaSLAM II: Tightlycoupled multi-object tracking and SLAM,” IEEE Robot. Automat. Lett., vol. 6, no. 3, pp. 5191–5198, Jul. 2021.

[19] Z. Teed and J. Deng, “RAFT: Recurrent all-pairs field transforms for optical flow,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 402–419.

[20] B. Mildenhall, P. P. Srinivasan, M. Tancik, J. T. Barron, R. Ramamoorthi, and R. Ng, “NeRF: Representing scenes as neural radiance fields for view synthesis,” Commun. ACM, vol. 65, pp. 99–106, 2021.

[21] J. Huang, S.-S. Huang, H. Song, and S.-M. Hu, “DI-Fusion: Online implicit 3D reconstruction with deep priors,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2021, pp. 8928–8937.

[22] X. Yang, H. Li, H. Zhai, Y. Ming, Y. Liu, and G. Zhang, “Vox-fusion: Dense tracking and mapping with Voxel-based neural implicit representation,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2022, pp. 499–507.

[23] R. Martin-Brualla, N. Radwan, M. S. Sajjadi, J. T. Barron, A. Dosovitskiy, and D. Duckworth, “NeRF in the wild: Neural radiance fields for unconstrained photo collections,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2021, pp. 7206–7215.

[24] K. Park et al., “Nerfies: Deformable neural radiance fields,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 5845–5854.

[25] A. Pumarola, E. Corona, G. Pons-Moll, and F. Moreno-Noguer, “D-NeRF: Neural radiance fields for dynamic scenes,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2021, pp. 10313–10322.

[26] K. Park et al., “HyperNeRF: A higher-dimensional representation for topologically varying neural radiance fields,” ACM Trans. Graph., vol. 40, pp. 1–12, 2021.

[27] C. Gao, A. Saraf, J. Kopf, and J.-B. Huang, “Dynamic view synthesis from dynamic monocular video,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 5692–5701.

[28] Q.-A. Chen and A. Tsukada, “Flow supervised neural radiance fields for static-dynamic decomposition,” in Proc. IEEE Int. Conf. Robot. Automat., 2022, pp. 10641–10647.

[29] T. Wu, F. Zhong, A. Tagliasacchi, F. Cole, and C. Oztireli, “D <sup>2</sup> NeRF: Selfsupervised decoupling of dynamic and static objects from a monocular video,” in Proc. Adv. Neural Inf. Process. Syst., 2022, pp. 32653–32666.

[30] Y.-L. Liu et al., “Robust dynamic radiance fields,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2023, pp. 13–23.

[31] S. Sabour, S. Vora, D. Duckworth, I. Krasin, D. J. Fleet, and A. Tagliasacchi, “RobustNeRF: Ignoring distractors with robust losses,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2023, pp. 20626–20636.

[32] G. Klein and D. Murray, “Parallel tracking and mapping for small AR workspaces,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2007, pp. 225–234.

[33] R. A. Newcombe, S. J. Lovegrove, and A. J. Davison, “DTAM: Dense tracking and mapping in real-time,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2011, pp. 2320–2327.

[34] T. Müller, A. Evans, C. Schied, and A. Keller, “Instant neural graphics primitives with a multiresolution hash encoding,” ACM Trans. Graph., vol. 41, pp. 1–15, 2022.

[35] D. Azinovi´c, R. Martin-Brualla, D. B. Goldman, M. Nießner, and J. Thies, “Neural RGB-D surface reconstruction,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2022, pp. 6280–6291.

[36] Z. Wang, S. Wu, W. Xie, M. Chen, and V. A. Prisacariu, “NeRF−−: Neural radiance fields without known camera parameters,” 2021, arXiv:2102.07064.

[37] C.-H. Lin, W.-C. Ma, A. Torralba, and S. Lucey, “BARF: Bundle-adjusting neural radiance fields,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 5721–5731.

[38] W. Bian, Z. Wang, K. Li, J.-W. Bian, and V. A. Prisacariu, “NoPe-NeRF: Optimising neural radiance field with no pose prior,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2023, pp. 4160–4169.

[39] H. Li, X. Gu, W. Yuan, L. Yang, Z. Dong, and P. Tan, “Dense RGB SLAM with neural implicit maps,” in Proc. Int. Conf. Learn. Representations, 2023.

[40] P. F. Felzenszwalb and D. P. Huttenlocher, “Distance transforms ofsampled functions,” Theory Comput., vol. 8, pp. 415–428, 2012.

[41] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2012, pp. 573–580.

[42] B. K. Horn, “Closed-form solution of absolute orientation using unit quaternions,” JOSA A, vol. 4, pp. 629–642, 1987.

[43] D. P. Kingma and J. Ba, “Adam: A method for stochastic optimization,” in Proc. Int. Conf. Learn. Representations, 2015.

[44] J. Canny, “A computational approach to edge detection,” IEEE Trans. Pattern Anal. Mach. Intell., vol. PAMI-8, no. 6, pp. 679–698, Nov. 1986.

[45] C. Kerl, J. Sturm, and D. Cremers, “Dense visual SLAM for RGB-D cameras,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2013, pp. 2100– 2106.

[46] Z. Teed and J. Deng, “DROID-SLAM: Deep visual SLAM for monocular, stereo, and RGB-D cameras,” in Proc. Adv. Neural Inf. Process. Syst., 2021, pp. 16558–16569.