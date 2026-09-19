# DEEPV2D: VIDEO TO DEPTH WITH DIFFERENTIABLE STRUCTURE FROM MOTION

Zachary Teed   
Princeton University   
zteed@cs.princeton.edu   
Jia Deng   
Princeton University   
jiadeng@cs.princeton.edu

## ABSTRACT

We propose DeepV2D, an end-to-end deep learning architecture for predicting depth from video. DeepV2D combines the representation ability of neural networks with the geometric principles governing image formation. We compose a collection of classical geometric algorithms, which are converted into trainable modules and combined into an end-to-end differentiable architecture. DeepV2D interleaves two stages: motion estimation and depth estimation. During inference, motion and depth estimation are alternated and converge to accurate depth. Code is available https://github.com/princeton-vl/DeepV2D.

## 1 INTRODUCTION

In video to depth, the task is to estimate depth from a video sequence. The problem has traditionally been approached using Structure from Motion (SfM), which takes a collection of images as input, and jointly optimizes over 3D structure and camera motion (Schonberger & Frahm, 2016b). The resulting camera parameter estimates can be used as input to Multi-View Stereo in order to build a more complete 3D representation such as surface meshes and depth maps (Furukawa et al., 2015; Furukawa & Ponce, 2010).

In parallel, deep learning has been highly successful in a number of 3D reconstruction tasks. In particular, given ground truth depth, a network can learn to predict depth from a single image (Eigen et al., 2014; Eigen & Fergus, 2015; Laina et al., 2016), stereo images (Kendall et al., 2017; Mayer et al., 2016a), or collections of frames (Zhou et al., 2018; Kar et al., 2017; Tang & Tan, 2018; Yao et al., 2018). One advantage of deep networks is that they can use single-image cues such as tex ture gradients and shading as shown by their strong performance on depth estimation from a single image (Eigen et al., 2014; Eigen & Fergus, 2015; Laina et al., 2016). Furthermore, differentiable network modules can be composed so that entire pipelines (i.e. feature extraction, feature matching, regularization) can be learned directly from training data. On the other hand, as recent work has shown, it is often hard to train generic network layers to directly utilize multiview geometry (e.g. using interframe correspondence to recover depth), and it is often advantageous to embed knowledge of multiview geometry through specially designed layers or losses (Ummenhofer et al., 2017; Kendall & Cipolla, 2017; Zhou et al., 2017; Vijayanarasimhan et al., 2017; Zhou et al., 2018).

In this work, we continue the direction set forth by recent works (Ummenhofer et al., 2017; Kendall et al., 2017; Tang & Tan, 2018; Zhou et al., 2018; Kar et al., 2017; Wang et al., 2018) that combine the representation ability of neural networks with the geometric principles underlying image formation. We propose DeepV2D, a composition of classical geometrical algorithms which we turn into differentiable network modules and combine into an end-to-end trainable architecture. DeepV2D interleaves two stages: camera motion estimation and depth estimation (Figure 1). The motion module takes depth as input, and outputs an incremental update to camera motion. The depth module takes camera motion as input, and performs stereo reconstruction to predict depth. At test time, DeepV2D acts as block coordinate descent, alternating between updating depth and camera motion.

![](images/2020_DeepV2D/81cc21635e6c079ba3c31bdb4e221a882ddf1a9710c75ca9775f053f1b677ba6.jpg)  
Figure 1: DeepV2D predicts depth from video. It is the composition of classical geometric algorithms, made differentiable, and combined into an end-to-end trainable network architecture. Video to depth is broken down into the subproblems of motion estimation and depth estimation, which are solved by the Motion Module and Depth Module respectively.

To estimate camera motion we introduce Flow-SE3, a new motion estimation architecture, which outputs an incremental update to camera motion. Flow-SE3 takes depth as input, and estimates dense 2D correspondence between pairs of frames. We unroll a single iteration of Perspectiven-Point (PnP) (Lepetit et al., 2009; Li et al., 2012) performing Gauss-Newton updates over SE3 perturbations to minimize geometric reprojection error. The new estimate of camera motion can then be fed back into Flow-SE3, which re-estimates correspondence for a finer grain pose update.

Our Depth Module builds upon prior work (Kendall et al., 2017; Yao et al., 2018) and formulates multiview-stereo (MVS) reconstruction as a single feed-forward network. Like classical MVS, we leverage geometry to build a cost volume over video frames, but use trainable network for both feature extraction and matching.

Our work shares similarities with prior works (Ummenhofer et al., 2017; Kendall et al., 2017; Tang & Tan, 2018; Zhou et al., 2018; Kar et al., 2017; Wang et al., 2018) that also combine deep learning and multiview geometry, but is novel and unique in that it essentially “differentializes” a classical SfM pipeline that alternates between stereopsis, dense 2D feature matching, and PnP. As a comparison, DeMon (Ummenhofer et al., 2017) and DeepTAM (Zhou et al., 2018) differentialize stereopsis and feature matching, but not PnP because they use a generic network to predict camera motion.

Another comparison is with BA-Net (Tang & Tan, 2018), whose classical analogue is performing bundle adjustment from scratch to optimize feature alignment over camera motion and the coefficients of a limited set of depth maps (depth basis). In other words, BA-Net performs one joint nonlinear optimization over all variables, whereas we decompose the joint optimization into more tractable subproblems and do block coordinate descent. Our decomposition is more expressive in terms of reconstruction since we can optimize directly over per-pixel depth and are not constrained by a depth basis, which can potentially limit the accuracy of the final depth.

In our experiments, we demonstrate the effectiveness of DeepV2D across a variety of datasets and tasks, and outperform strong methods such as DeepTAM (Zhou et al., 2018), DeMoN (Ummenhofer et al., 2017), BANet (Tang & Tan, 2018), and MVSNet (Yao et al., 2018). As we show, alternating depth and motion estimation quickly converges to good solutions. On all datasets we outperform all existing single-view and multi-view approaches. We also show superior cross-dataset generalizabil ity, and can outperform existing methods even when training on entirely different datasets.

## 2 RELATED WORK

Structure from Motion: Beginning with early systems designed for small image collections (Longuet-Higgins, 1981; Mohr et al., 1995), Structure from Motion (SfM) has improved dramatically in regards to robustness, accuracy, and scalability. Advances have come from improved features (Lowe, 2004; Han et al., 2015), optimization techniques (Snavely, 2009), and more scalable data structures and representations (Schonberger & Frahm, 2016a; Gherardi et al., 2010), culminat ing in a number of robust systems capable of large-scale reconstruction task (Schonberger & Frahm, 2016a; Snavely, 2011; Wu et al., 2011). Ranftl et al. (2016) showed that SfM could be extended to reconstruct scenes containing many dynamically moving objects. However, SfM is limited by the accuracy and availability of correspondence. In low texture regions, occlusions, or lighting changes SfM can produce noisy or missing reconstructions.

Simultaneous Localization and Mapping (SLAM) jointly estimates camera motion and 3D structure from a video sequence (Engel et al., 2014; Mur-Artal et al., 2015; Mur-Artal & Tardos, 2017; New-´ combe et al., 2011; Engel et al., 2018). LSD-SLAM (Engel et al., 2014) is unique in that it relies on a featureless approach to 3D reconstruction, directly estimating depth maps and camera pose by minimizing photometric error. Our Motion Network behaves similarly to the tracking component in LSD-SLAM, but we use a network which predicts misalignment directly instead of using intensity gradients. We end up with an easier optimization problem characteristic of indirect methods (Mur

Artal et al., 2015), while retaining the flexibility of direct methods in modeling edges and smooth intensity changes (Engel et al., 2018).

Geometry and Deep Learning: Geometric principles has motivated the design of many deep learning architectures. In video to depth, we need to solve two subproblems: depth estimation and motion estimation.

Depth: End-to-end networks can be trained to predict accurate depth from a rectified pair of stereo images (Han et al., 2015; Mayer et al., 2016a; Kendall et al., 2017; Chang & Chen, 2018). Kendall et al. (2017) and Chang & Chen (2018) design network architectures specifically for stereo matching. First, they apply a 2D convolutional network to extract learned features, then build a cost volume over the learned features. They then apply 3-D convolutions to the cost volume to perform feature matching and regularization. A similar idea has been extended to estimate 3D structure from multiple views (Kar et al., 2017; Yao et al., 2018). In particular, MVSNet (Yao et al., 2018) estimates depth from multiple images. However, these works require known camera poses as input, while our method estimates depth from a video where the motion of the camera is unknown and estimated during inference.

Motion: Several works have used deep networks to predict camera pose. Kendall et al. (2015) focus on the problem of camera localization, while other work (Zhou et al., 2017; Vijayanarasimhan et al., 2017; Wang et al., 2017) propose methods which estimate camera motion between a pairs of frames in a video. Networks for motion estimation have typically relied on generic network components whereas we formulate motion estimation as a least-squares optimization problem. Whereas prior work has focused on estimating relative motion between pairs of frames, we can jointly update the pose of a variable number of frames.

Depth and Motion: Geometric information has served as a self-supervisory signal for many recent works (Vijayanarasimhan et al., 2017; Zhou et al., 2017; Wang et al., 2018; Yin & Shi, 2018; Yang et al., 2018; Godard et al., 2017; Mahjourian et al., 2018). In particular, Zhou et al. (2017) and Vijayanarasimhan et al. (2017) trained a single-image depth network and a pose network while supervising on photometric consistency. However, while these works use geometric principles for training, they do not use multiple frames to predict depth at inference.

DeMoN (Ummenhofer et al., 2017) and DeepTAM (Zhou et al., 2018) where among the first works to combine motion estimation and multi-view reconstruction into a trainable pipeline. DeMoN (Ummenhofer et al., 2017) operates on two frames and estimates depth and motion in separate network branches, while DeepTAM (Zhou et al., 2018) can be used on variable number of frames. Like our work and other classical SLAM framesworks (Engel et al., 2014; Newcombe et al., 2011), Deep-TAM separates depth and motion estimation, however we maintain end-to-end differentiablity between our modules. A major innovation of DeepTAM was to formulate camera motion estimation in the form of incremental updates. In each iteration, DeepTAM renders the keyframe from a synthetic viewpoint, and predicts the residual motion from the rendered viewpoint and the target frame.

Estimating depth and camera motion can be naturally modeled as a non-linear least squares problem, which has motivated several works to include an differentiable optimization layer within network architectures (Tang & Tan, 2018; Wang et al., 2018; Clark et al., 2018; Bloesch et al., 2018). We follow this line of work, and propose the Flow-SE3 module which introduces a direct mapping from 2D correspondence to a 6-dof camera motion update. Our Flow-SE3 module is different from prior works such as DeMon (Ummenhofer et al., 2017) and DeepTAM (Zhou et al., 2018) which do not impose geometric constraints on camera motion and use generic layers. BA-Net (Tang & Tan, 2018) and LS-Net (Clark et al., 2018) include optimization layers, but instead optimize over photometric error (either pixel alignment (Clark et al., 2018) or feature alignment (Tang & Tan, 2018)). Our Flow-SE3 module still imposes geometric constraints on camera motion like BA-Net (Tang & Tan, 2018), but we show that in minimizing geometric reprojection error ( difference of pixel locations), we end up with a well-behaved optimization problem, well-suited for end-to-end training.

An important difference between our approach and BA-Net is that BA-Net performs one joint optimization problem by formulating Bundle-Adjustment as a differentiable network layer, whereas we separate motion and depth estimation. With this separation, we avoid the need for a depth basis. Our final reconstructed depth is the product of a cost volume, which can adapt the reconstruction as camera motion updates improve, while the output of BA-Net is restricted by the initial quality of the depth basis produced by a single-image network.

![](images/2020_DeepV2D/494e7c8bd96d70372a914eff05e746409e3a6f102dbc7bfa82df4915c7f41bc2.jpg)  
Figure 2: The Depth Module performs stereo matching over multiple frames to estimate depth. First each image is fed through a network to extract a dense feature map. The 2D features are backprojected into a set of cost volumes. The cost volumes are processed by a set of 3D hourglass networks to perform feature matching. The final cost volume is processed by the differentiable arg-max operator to produce a pixelwise depth estimate.

## 3 APPROACH

DeepV2D predicts depth from a calibrated video sequence. We take a video as input and output dense depth. We consider two subproblems: depth estimation and motion estimation. Both subproblems are formulated as trainable neural network modules, which we refer to as the Depth Module and the Motion Module. Our depth module takes camera motion as input and outputs an updated depth prediction. Our motion module takes depth as input, and outputs a motion correction term. In the forward pass, we alternate between the depth and motion modules as we show in Figure 1.

Notation and Camera Geometry: As a preliminary, we define some of the operations used within the depth and motion modules. We define π to be the camera projection operator which maps a 3D point $\mathbf { X } = ( X , Y , Z , 1 ) ^ { T }$ to image coordinates $\mathbf { x } = ( u , v )$ . Likewise, $\pi ^ { \frac { \bullet } { - 1 } }$ is defined to be the backprojection operator, which maps a pixel x and depth z to a 3D point. Using the pinhole camera model with intrinsics $( f _ { x } , f _ { y } , c _ { x } , c _ { y } )$ we have

$$
\pi ( { \bf X } ) = ( f _ { x } \frac { X } { Z } + c _ { x } , f _ { y } \frac { Y } { Z } + c _ { y } ) , \qquad \pi ^ { - 1 } ( { \bf x } , z ) = ( z \frac { u - c _ { x } } { f _ { x } } , z \frac { v - c _ { y } } { f _ { y } } , z , 1 ) ^ { T }\tag{1}
$$

The camera pose is represented using rigid body transform $\mathbf { G } \in S E ( 3 )$ . To find the image coordinates of point X in camera $i ,$ we chain the projection and transformation: $( u , v ) ^ { T } = \bar { \pi ( \mathbf { G } _ { i } \mathbf { X } ) }$ where $\mathbf { G } _ { i }$ is the pose of camera i.

Now, given two cameras $\mathbf { G } _ { i }$ and $\mathbf { G } _ { j } .$ If we know the depth of a point $\mathbf { x } ^ { i } = ( u ^ { i } , v ^ { i } )$ in camera $i ,$ we can find its reprojected coordinates in camera $j \colon$

$$
\binom { u ^ { j } } { v ^ { j } } = \pi ( \mathbf { G } _ { j } \mathbf { G } _ { i } ^ { - 1 } \pi ^ { - 1 } ( \mathbf { x } , z ) ) = \pi ( \mathbf { G } _ { i j } \pi ^ { - 1 } ( \mathbf { x } , z ) )\tag{2}
$$

using the notation $\mathbf { G } _ { i j } = \mathbf { G } _ { j } \mathbf { G } _ { i } ^ { - 1 }$ for the relative pose between cameras i and $j .$

## 3.1 DEPTH MODULE

The depth module takes a collection of frames, $\mathbf { I } = \{ I _ { 1 } , I _ { 2 } , . . . , I _ { N } \}$ , along with their respective pose estimates, $\mathbf { G } = \{ G _ { 1 } , G _ { 2 } , . . . , G _ { N } \}$ , and predicts a dense depth map $D ^ { * }$ for the keyframe (Figure 2). The depth module works by building a cost volume over learned features. Information is aggregated over multiple viewpoints by applying a global pooling layer which pools across viewpoints.

The depth module can be viewed as the composition of 3 building blocks: 2D feature extractor, cost volume backprojection, and 3D stereo matching.

2D Feature Extraction: The Depth Module begins by extracting learned features from the input images. The 2D encoder consists of 2 stacked hourglass modules (Newell et al., 2016) which maps each image to a dense feature map $I _ { i } \to F _ { i }$ . More information regarding network architectures is provided in the appendix.

Cost Volume Backprojection: Take $I _ { 1 }$ to be the keyframe, a cost volume is constructed for each of the remaining N-1 frames. The cost volume for frame $j , { \bf C } ^ { j }$ , is constructed by backprojecting 2D features into the coordinate system defined by the keyframe image. To build the cost volume, we enumerate over a range of depths $z _ { 1 } , z _ { 2 } , . . . , z _ { D }$ which is chosen to span the ranges observed in the dataset (0.2m - 10m for indoor scenes). For every depth $z _ { k }$ , we use Equation 2 to find the reprojected coordinates on frame $j ,$ , and then use differentiable bilinear sampling of the feature map $F _ { j }$

More formally, given a pixel $\mathbf { x } = ( u , v ) \in \mathbb { N } ^ { 2 }$ in frame $I _ { 1 }$ and depth $z _ { k } \mathrm { : }$

$$
C _ { u v k } ^ { j } = F _ { j } ( \pi ( \mathbf { G } _ { j } \mathbf { G } _ { 1 } ^ { - 1 } \pi ^ { - 1 } ( \mathbf { x } , z _ { k } ) ) ) \in \mathbb { R } ^ { H \times W \times D \times C }\tag{3}
$$

where $F ( \cdot )$ is the differentiable bilinear sampling operator (Jaderberg et al., 2015). Since the bilinear sampling is differentiable, $\mathbf { C } ^ { j }$ is differentiable w.r.t all inputs, including the camera pose.

Applying this operation to each frame, gives us a set of N-1 cost volumes each with dimension $\mathbf { H } { \times } \mathbf { \bar { W } } { \times } \mathbf { \bar { D } } { \times } \mathbf { C }$ . As a final step, we concatenate each cost volume with the keyframe image features increasing the dimension to $\mathrm { H } { \times } \mathrm { W } { \times } \mathrm { D } { \times } 2 \mathrm { C }$ . By concatenating features, we give the network the necessary information to perform feature matching between the keyframe/image pairs without decimating the feature dimension.

3D Matching Network: The set of N-1 cost volumes are first processed by a series of 3D convolutional layers to perform stereo matching. We then perform view pooling by averaging over the N-1 volumes to aggregate information across frames. View pooling leaves us with a single volume of dimension $\mathbf { H } { \times } \mathbf { \bar { W } } { \times } \mathbf { \bar { D } } { \times } \mathbf { C }$ . The aggregated volume is then processed by a series of 3D hourglass modules, each outputs an intermediate depth.

Each 3D hourglass module predicts an intermediate depth estimate. We produce an intermediate depth representation by first applying a 1x1x1 convolution to a produce H×W×D volume. We then apply the softmax operator over the depth dimension, so that for each pixel, we get a probability distribution over depths. We map the probability volume into a single depth estimate using the differentiable argmax function (Kendall et al., 2017) which computes the expected depth.

## 3.2 MOTION MODULE

The objective of the motion module is to update the camera motion estimates given depth as input. Given the input poses, $\mathbf { G } = \{ G _ { 1 } , G _ { 2 } , . . . , \bar { G } _ { N } \}$ , the motion module outputs a set of local perturbations $\pmb { \xi } = \{ \xi _ { 1 } , \xi _ { 2 } , . . . , \xi _ { N } \} , \xi _ { i } \in s e ( 3 )$ used to update the poses. The updates are found by setting up a least squares optimization problem which is solved using a differentiable in-network optimization layer.

Initialization: We use a generic network architecture to predict the initial pose estimates similiar to prior work Zhou et al. (2017). We choose one frame to be the keyframe. The poses are initialized by setting the keyframe pose to be the identity matrix, and then predicting the relative motion between the keyframe and each of the other frames in the video.

Feature Extraction: Our motion module operates over learned features. The feature extractor maps every frame to a dense feature map, $I _ { i } \to F _ { i }$ . The weights of the feature extractor are shared across all frames. Network architecture details are provided in the appendix.

Error Term: Take two frames, $( I _ { i } , I _ { j } )$ , with respective poses $( \mathbf { G } _ { i } , \mathbf { G } _ { j } )$ and feature maps $( F _ { i } , F _ { j } )$ Given depth $Z _ { i }$ we can use Equation 2 we can warp $F _ { j }$ onto camera i to generate the warped feature map $\tilde { F } _ { j }$ . If the relative pose $\mathbf { G } _ { i j } = \mathbf { G } _ { j } \mathbf { G } _ { i } ^ { - 1 }$ is correct, then the feature maps $F _ { i }$ and $\tilde { F } _ { j }$ should align. However, if the relative pose is noisy, then there will be misalignment between the feature images which should be corrected by the pose update.

We concatenate $F _ { i }$ and $\tilde { F } _ { j }$ , and send the concatenated feature map through an hourglass network to predict the dense residual flow between the feature maps, which we denote R, and corresponding confidence map W. Using the residual flow, we define the following error term:

$$
\mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) = \mathbf { r } _ { k } - [ \pi ( ( e ^ { \xi _ { j } } \mathbf { G } _ { j } ) ( e ^ { \xi _ { i } } \mathbf { G } _ { i } ) ^ { - 1 } \mathbf { X } _ { k } ^ { i } ) - \pi ( \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) ] , \qquad \mathbf { X } _ { k } ^ { i } = \pi ^ { - 1 } ( \mathbf { x } _ { k } , z _ { k } )\tag{4}
$$

![](images/2020_DeepV2D/f63d8beae03795b0a7588eccd91236c8cf3588ea8cbc0376524296c770fc6601.jpg)  
Figure 3: The Motion Module updates the input pose estimates by solving a least squares optimization problem. The motion module predicts the residual flow between pairs of frames, and uses the residual terms to define the optimization objective. Pose increments $\bar { \pmb { \xi } }$ are found by performing a single differentiable Gauss-Newton optimization step.

where $\mathbf { r } _ { k }$ is the residual flow at pixel $\mathbf { x } _ { k }$ predicted by the network, and $z _ { k }$ is the predicted depth. The weighting map W is mapped to $( 0 , 1 )$ using the sigmoid activation, and is used to determine how the individual error terms are weighted in the final objective.

Optimization Objective: The previous section showed how two frames $( i , j )$ can be used to define a collection of error terms $\mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } )$ for each pixel $\mathbf { x } _ { k }$ in image $I _ { i }$ . The final optimization objective is a weighted combination of error terms:

$$
E ( \pmb { \xi } ) = \sum _ { ( i , j ) \in \mathcal { C } } \sum _ { k } \mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) ^ { T } d i a g ( \mathbf { w } _ { k } ) \mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) , \qquad d i a g ( \mathbf { w } _ { k } ) = \left( \begin{array} { l l } { w _ { k } ^ { u } } & { 0 } \\ { 0 } & { w _ { k } ^ { v } } \end{array} \right)\tag{5}
$$

This leaves us with the question of which frames pairs $( i , j ) \in \mathcal { C }$ to use when defining the optimization objective. In this paper, we consider two different approaches which we refer to as Global pose optimization and Keyframe pose optimization.

Global Pose Optimization: Our global pose optimization uses all pairs of frames ${ \mathcal { C } } = ( i , j ) , i \neq j$ to define the objective function (Equation 5) and the pose increment ξ is solved for jointly over all poses. Therefore, given N frames, dense pose optimization uses $\mathbf { N } { \times } \mathbf { N } { - } 1$ frame pairs. Since every pair of frames is compared, this means that the global pose optimization requires the predicted depth maps for all frames as input. Although each pair $( i , j )$ only gives us information about the relative pose $\mathbf { G } _ { i j }$ , considering all pairs allows us to converge to a globally consistent pose graph.

Keyframe Pose Optimization: Our keyframe pose optimization selects a given frame to be the keyframe (i.e select $I _ { 1 }$ as the keyframe), and only computes the error terms between the keyframe and each of the other frames: $ { \mathcal { C } } = ( 1 , j )$ for $j = \hat { 2 } , . . . , \hat { N }$

Fixing the pose of the keyframe, we can remove $\xi _ { 1 }$ from the optimization objective. This means that each error $\mathbf { e } _ { k } ^ { i j } ( \mathbf { 0 } , \boldsymbol { \xi } _ { j } )$ term is only a function of a single pose increment $\xi _ { j }$ . Therefore, we can solve for each of the $N - 1$ pose increments independently. Additionally, since $i = 1$ for all pairs $( i , j ) \in \mathcal { C }$ , we only need the depth of the keyframe as input when using keyframe pose optimization.

LS-Optimization Layer: Using the optimization objective in Equation $^ { 5 , }$ , we solve for the pose increments ξ by applying a Gauss-Newton update. We backpropogate through the Gauss-Newton update so that the weights of the motion module (both feature extractor and flow network) can be trained on the final objective function. In the appendix, we provide additional information for how the update is derived and the expression for the Jacobian of Equation 4.

## 3.3 FULL SYSTEM

During inference, we alternate the depth and motion modules for a selected number of iterations. The motion module uses depth to predict camera pose. As the depth estimates converge, the camera poses become more accurate. Likewise, as camera poses converge, the depth module can estimate more accurate depth.

![](images/2020_DeepV2D/b4e395686d6e63139078cc4eba72169dd0a6bf691dca5de59b242540c96c65e7.jpg)  
Figure 4: Visualization of predicted depth maps on NYU, ScanNet, and SUN3D. On ScanNet and SUN3D (marked with \*) we show the results of the model trained only on NYU data.

Initialization: We try two different strategies for initialization in our experiments: (1) self initial ization initializes DeepV2D with a constant depth map and (2) single image initialization uses the output of a single-image depth network for initialization. Both methods give good performance.

## 3.4 SUPERVISION

Depth Supervision: We supervise on the L1 distance between the ground truth and predicted depth. We additionally apply a small L1 smoothness penalty to the predicted depth map. Given predicted depth Z and ground truth depth $Z ^ { \ast }$ , the depth loss is defined as:

$$
\mathcal { L } _ { d e p t h } ( Z ) = \sum _ { \mathbf { x } _ { i } } \left| Z ( \mathbf { x } _ { i } ) - Z ^ { * } ( \mathbf { x _ { i } } ) \right| + w _ { s } \sum _ { \mathbf { x } _ { i } } \left| \partial _ { x } Z ( \mathbf { x } _ { i } ) \right| + \left| \partial _ { y } Z ( \mathbf { x } _ { i } ) \right|\tag{6}
$$

Motion Supervision: We supervise pose using the geometric reprojection error. Given predicted pose G and ground truth pose $\mathbf { \hat { G } } ^ { * }$ , the pose loss is defined

$$
\mathcal { L } _ { m o t i o n } ( \mathbf { G } ) = \sum _ { \mathbf { x } _ { i } } | | \pi ( \mathbf { G } \pi ^ { - 1 } ( \mathbf { x } _ { i } , Z ( \mathbf { x } _ { i } ) ) ) - \pi ( \mathbf { G } ^ { * } \pi ^ { - 1 } ( \mathbf { x } _ { i } , Z ( \mathbf { x } _ { i } ) ) ) | | _ { \delta }\tag{7}
$$

where $| | \cdot | | _ { \delta }$ is the robust Huber loss; we set $\delta = 1$

Total Loss: The total loss is taken as a weighted combination of the depth and motion loss terms: $\mathcal { L } = \mathcal { L } _ { d e p t h } + \lambda \mathcal { L } _ { m o t i o n }$ , where we set $\lambda = \overline { { 1 } } . 0$ in our experiments.

## 4 EXPERIMENTS

We test DeepV2D across a wide range of benchmarks to provide a thorough comparison to other methods. While the primary focus of these experiments is to compare to other works which estimate depth from multiple frames, often single-view networks still outperform multiview depth estimation. To put our results in proper context, we include both multiview and state-of-the-art single-image comparisons. Since it is not possible to recover the absolute scale of the scene through SfM, we report all results (both ours and all other approaches) using scale matched depth (Tang & Tan, 2018).

Our primary experiments are on NYU, ScanNet, SUN3D, and KITTI, and we report strong results across all datasets. We show visualization of our predicted depth maps in Figure 4. The figure shows that DeepV2D can recover accurate and sharp depth even when applied to unseen datasets. One aspect of particular interest is cross-dataset generalizability. Our results show that DeepV2D generalizes very well—we achieve the highest accuracy on ScanNet and SUN3D even without training on either dataset.

## 4.1 DEPTH EXPERIMENTS

We evaluate depth on NYU (Silberman et al., 2012), ScanNet (Dai et al., 2017), SUN3D (Xiao et al., 2013), and KITTI (Geiger et al., 2013). On all datasets, DeepV2D is given a video clip with unknown camera poses and alternates depth and pose updates and is evaluated after 8 iterations.

NYU: NYU depth (Silberman et al., 2012) is a dataset composed of videos taken in indoor settings including offices, bedrooms, and libraries. We experiment on NYU using the standard train/test split (Eigen et al., 2014) and report results in Table 1 using scaled depth (Zhou et al., 2017; Tang & Tan, 2018). We evaluate two different initialization methods of our approach. Self-init uses a constant depth map for initialization, while fcrn-init uses the output of a FCRN (Laina et al., 2016)—a singleview network for initialization. Using a single-image depth network for initialization gives a slight improvement in performance.

<table><tr><td>NYUv2</td><td></td><td>δ &lt; 1.25 ↑</td><td> $\delta < 1 . 2 5 ^ { 2 } \uparrow$ </td><td> $\delta < 1 . 2 5 ^ { 3 } \uparrow$ </td><td>Abs Rel ↓</td><td>Sc Inv ↓</td><td>RMSE↓</td><td>log10 ↓</td></tr><tr><td rowspan="3">se</td><td>FCRN (Laina et al., 2016)</td><td>0.853</td><td>0.965</td><td>0.991</td><td>0.121</td><td>0.151</td><td>0.592</td><td>0.052</td></tr><tr><td>DORN (Fu et al., 2018)</td><td>0.875</td><td>0.966</td><td>0.989</td><td>0.109</td><td></td><td>0.464</td><td>0.047</td></tr><tr><td>Alhashim &amp; Wonka (2018)</td><td>0.895</td><td>0.980</td><td>0.996</td><td>0.103</td><td></td><td>0.390</td><td>0.043</td></tr><tr><td rowspan="7">mul-view</td><td>COLMAP DfUSMC</td><td>0.619</td><td>0.760</td><td>0.829</td><td>0.312</td><td>1.512</td><td>1.381</td><td>0.153</td></tr><tr><td>MVSNet + OpenMVG</td><td>0.487</td><td>0.697</td><td>0.814</td><td>0.447</td><td>0.456</td><td>1.793</td><td>0.169</td></tr><tr><td>DeMoN</td><td>0.766</td><td>0.913</td><td>0.965</td><td>0.181</td><td>0.212</td><td>0.917</td><td>0.072</td></tr><tr><td>DeMoN ↑</td><td>0.776</td><td>0.933</td><td>0.979</td><td>0.160</td><td>0.196</td><td>0.775</td><td>0.067</td></tr><tr><td></td><td>0.805</td><td>0.951</td><td>0.985</td><td>0.144</td><td>0.179</td><td>0.717</td><td>0.061</td></tr><tr><td>Ours (self-init) - Keyframe</td><td>0.940</td><td>0.985</td><td>0.995</td><td>0.072</td><td>0.105</td><td>0.459</td><td>0.031</td></tr><tr><td>Ours (fcrn-init) - Keyframe Ours (self-init) - Global</td><td>0.955</td><td>0.990</td><td>0.996</td><td>0.062</td><td>0.095</td><td>0.405</td><td>0.027</td></tr><tr><td></td><td>0.942</td><td>0.986</td><td>0.995</td><td>0.070</td><td>0.104</td><td>0.454</td><td>0.030</td></tr><tr><td>Ours (fcrn-init) - Global</td><td>0.956</td><td>0.989</td><td>0.996</td><td>0.061</td><td>0.094</td><td>0.403</td><td></td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>0.026</td></tr></table>

Table 1: Results on the NYU dataset. Our approach outperforms existing single-view and multiview depth estimation methods. Ours (self-init) uses a constant depth map for initialization while ours(fcrn-init) uses a single-image depth network for initialization.

We compare to state-of-the-art single-image depth networks DORN (Fu et al., 2018) and DenseDepth (Alhashim & Wonka, 2018) which are built on top of a pretrained ResNet (DORN) or DenseNet-201 (DenseDepth). The results show that we can do much better than single-view depth by using multiple views. We also include classical multiview approaches such as COLMAP (Schonberger & Frahm, 2016a) and DfUSMC (Ha et al., 2016) which estimate poses with bundle adjustment, followed by dense stereo matching. While COLMAP uses SIFT features, DfUSMC is built on local-feature tracking and is designed for small baseline videos.

Table 1 also includes results using multi-view deep learning approaches. MVSNet (Yao et al., 2018) is trained to estimate depth from multiple viewpoints. Unlike our approach which estimates camera pose during inference, MVSNet requires ground truth poses as input. We train MVSNet on NYU and use poses estimated from OpenMVG (Moulon et al.) during inference. Finally, we also evaluate DeMoN (Ummenhofer et al., 2017) on NYU. DeMoN is not originally trained on NYU, but instead trained on a combination of 5 other datasets. We also try a version of DeMoN which we retrain on NYU using the code provided by the authors (denoted †).

In Appendix C, we include additional results on NYU where we test different versions of our model, along with parameter counts, timing information, peak memory usage, and depth accuracy. A shallower version of DeepV2D (replacing the stacked hourglass networks with a single hourglass network) and lower resolution inference still outperform existing work on NYU. However, using a 3D network for stereo matching turns out to be very important for depth accuracy. When the 3D stereo network is replaced with a correlation layer (Dosovitskiy et al., 2015) and 2d encoder-decoder, depth accuracy is worse increasing Abs-Rel from 0.062 to 0.135.

Figure 5 shows the impact of the number of iterations and views on the scale-invariant (sc-inv) validation set accuracy. Figure 5 (left) shows that DeepV2D requires very few iterations to con verge, suggesting that block coordinate descent is effective for estimate depth from small video clips. In Figure 5 (right) we test accuracy as a function of the number of input frames used. Although DeepV2D is trained using a fixed number (4) frames as input, accuracy continues to improve a more frames are added.

ScanNet: ScanNet is a large indoor dataset consisting of 1513 RGB-D videos in distinct scenes. We use the train/test split proposed by Tang & Tan (2018) and evaluate depth and pose accuracy in Table 2. While our primary focus is on depth, DeepV2D accurately predicts camera motion.

We use ScanNet to test cross-dataset generalization and report results from two versions of our approach: ours (nyu) is our method trained only on nyu, ours (scannet) is our method trained on ScanNet. As expected, when we train on the ScanNet training set we do better than if we train only on NYU. But the performance of our NYU model is still good and outperforms BA-Net on all metrics. The design of our approach is motivated by generalizability. Our network only needs to learn feature matching and correspondence; this experiment indicates that by learning these low level tasks, we can generalize well to new data.

![](images/2020_DeepV2D/c3be6bf0570f346691897fd2eba4f588c1677a2db42609ca051da2098057e8a3.jpg)

![](images/2020_DeepV2D/d2c1a5b08a1ee3642699263a55aa4ba434e2b47820a3655b650a5bfd7dc3f93c.jpg)  
Figure 5: Impact of the number of iterations (left) and frames (right) on sc-inv validation accuracy. (left) shows that DeepV2D quickly converges within a small number of iterations. In (right) we see that accuracy consistently improves as more views are added. DeepV2D can be applied to variable numbers of views for a variable number of iterations without retraining.

<table><tr><td>ScanNet</td><td>Abs Rel ↓</td><td>Sq Rel ↓</td><td>RMSE↓</td><td>RMSE log ↓</td><td>sc inv ↓</td><td>rot.(deg) ↓</td><td>tr. (deg) ↓</td><td>tr. (cm) ↓</td></tr><tr><td>DeMoN</td><td>0.231</td><td>0.520</td><td>0.761</td><td>0.289</td><td>0.284</td><td>3.791</td><td>31.626</td><td>15.50</td></tr><tr><td>BA-Net (orig.)</td><td>0.161</td><td>0.092</td><td>0.346</td><td>0.214</td><td>0.184</td><td>1.018</td><td>20.577</td><td>3.390</td></tr><tr><td>BA-Net (5-view)</td><td>0.091</td><td>0.058</td><td>0.223</td><td>0.147</td><td>0.137</td><td>1.009</td><td>14.626</td><td>2.365</td></tr><tr><td>DSO (Engel et al., 2018)</td><td></td><td></td><td></td><td></td><td></td><td>0.925</td><td>19.728</td><td>2.174</td></tr><tr><td>DSO (fcrn-init)</td><td></td><td></td><td></td><td></td><td></td><td>0.946</td><td>19.238</td><td>2.165</td></tr><tr><td>Ours (nyu) Ours (scannet)</td><td>0.080 0.057</td><td>0.018 0.010</td><td>0.223 0.168</td><td>0.109 0.080</td><td>0.105 0.077</td><td>0.714 0.628</td><td>12.205 10.800</td><td>1.514 1.373</td></tr></table>

Table 2: ScanNet experiments evaluating depth and pose accuracy and cross-dataset generalization. Our approach trained on NYU (ours nyu) outperforms BA-Net despite BA-Net being trained on ScanNet data; training on ScanNet (ours scannet) gives even better performance.

Pose accuracy from DSO Engel et al. (2018) is also included in Table 2. We test DSO using both the default initialization and single-image depth initialization using the output of FCRN (Laina et al., 2016). DSO fails to initialize or loses tracking on some of the test sequences so we only evaluate on sequences where DSO is successful. DSO fails on 335 of the 2000 test sequences while DSO (fcrn-init) fails on only 271.

SUN3D: SUN3D (Xiao et al., 2013) is another indoor scenes dataset which we use for comparison with DeepTAM. DeepTAM only evaluates their depth module in isolation using the poses provided by dataset, while our approach is designed to estimate poses during inference. We provide results from our SUN3D experiments in Table 3.
<table><tr><td>SUN3D</td><td>Training Data</td><td>L1-Inv ↓</td><td>L1-Rel ↓</td><td>Sc-Inv ↓</td></tr><tr><td>SGM DTAM</td><td></td><td>0.197 0.210</td><td>0.412 0.423</td><td>0.340 0.374</td></tr><tr><td>DeMoN</td><td>S11+RGBD+MVS+SUN3D</td><td></td><td></td><td>0.146</td></tr><tr><td>DeepTAM</td><td>MVS+SUNCG+SUN3D</td><td>0.054</td><td>0.101</td><td>0.128</td></tr><tr><td>Ours</td><td>NYU</td><td>0.056</td><td>0.106</td><td>0.134</td></tr><tr><td>Ours</td><td>NYU + ScanNet</td><td>0.041</td><td>0.077</td><td>0.104</td></tr></table>

Table 3: Results on SUN3D dataset and comparison to DeepTAM. DeepTAM only evaluates depth in isolation and uses the poses from the dataset during inference, while our approach jointly estimates camera poses during inference. We outperform DeepTAM and DeMoN on SUN3D even when we do not use SUN3D data for training.

We cannot train using the same data as DeepTAM since DeepTAM is trained using a combination of SUN3D, SUNCG, and MVS, and, at this time, neither MVS nor SUNCG are publicly available. Instead we train on alternate data and test on SUN3D. We test two different versions of our model; one where we train only on NYU, and another where we train on a combination of NYU and ScanNet data. Our NYU model performs similiar to DeepTAM; When we combine with ScanNet data, we outperform DeepTAM even though DeepTAM is trained on SUN3D and is evaluated with ground truth pose as input.

KITTI: The KITTI dataset (Geiger et al., 2013) is captured from a moving vehicle and has been widely used to evaluate depth estimation and odometry. We follow the Eigen train/test split (Eigen et al., 2014), and report results in Table 4. We evaluate using the official ground truth depth maps. We compare to the state-of-the-art single-view methods and also multiview approaches such as BA Net (Tang & Tan, 2018), and outperform previous methods on the KITTI dataset across all metrics.
<table><tr><td>KITTI</td><td>Multi</td><td>δ &lt; 1.25 ↑</td><td>δ &lt; 1.252 ↑</td><td>δ &lt; 1.253 ↑</td><td>Abs Rel ↓</td><td>Sq Rel ↓</td><td>Sq Rel † ↓</td><td>RMSE↓</td><td>RMSE log ↓</td></tr><tr><td>DORN</td><td>N</td><td>0.945</td><td>0.988</td><td>0.996</td><td>0.069</td><td>0.300</td><td>-</td><td>2.857</td><td>0.112</td></tr><tr><td>DfUSMC</td><td>Y</td><td>0.617</td><td>0.796</td><td>0.874</td><td>0.346</td><td>5.984</td><td></td><td>8.879</td><td>0.454</td></tr><tr><td>BA-Net</td><td>Y</td><td></td><td></td><td></td><td>0.083</td><td></td><td>0.025</td><td>3.640</td><td>0.134</td></tr><tr><td>Ours</td><td>Y</td><td>0.977</td><td>0.993</td><td>0.997</td><td>0.037</td><td>0.174</td><td>0.013</td><td>2.005</td><td>0.074</td></tr></table>

Table 4: Results on the KITTI dataset. We compare to state-of-the-art single-image depth network DORN (Fu et al., 2018) and multiview BA-Net (Tang & Tan, 2018). BA-Net reports results using a different form of the Sq-Rel metric which we denote by †.

Overall, the depth experiments demonstrates that imposing geometric constraints on the model architecture leads to higher accuracy and better cross-dataset generalization. By providing a differentiable mapping from optical flow to camera motion, the motion network only needs to learn to estimate interframe correspondence. Likewise, the 3D cost volume means the the depth network only needs to learn to perform stereo matching. These tasks are easy for the network to learn, which leads to strong results on all datasets, and can generalize to new datasets.

## 4.2 TRACKING EXPERIMENTS

DeepV2D can be turned into a basic SLAM system. Using NYU and ScanNet for training, we test tracking performance on the TUM-RGBD tracking benchmark (Table 5) using sensor depth as input. We achieve a lower translational rmse [m/s] than DeepTAM on most of the sequences. DeepTAM uses optical flow supervision to improve performance, but since our network directly maps optical flow to camera motion, we do not need supervision on optical flow.

We use our global pose optimization in our tracking experiments. We maintain a fixed window of 8 frames during tracking. At each timestep, the pose of the first 3 frames in the window are fixed and the remaining 5 are updated using the motion module. After the update, the start of the tracking window is incremented by 1 frame. We believe our ability to jointly update the pose of multiple frames is a key reason for our strong performance on the RGB-D benchmark.

<table><tr><td></td><td>360</td><td>desk</td><td>desk2</td><td>plant</td><td>room</td><td>rpy</td><td>xyz</td><td>mean</td></tr><tr><td>DVO (Kerl et al., 2013)</td><td>0.125</td><td>0.037</td><td>0.020</td><td>0.062</td><td>0.042</td><td>0.082</td><td>0.051</td><td>0.060</td></tr><tr><td>DeepTAM (Zhou et al., 2018)</td><td>0.054</td><td>0.027</td><td>0.017</td><td>0.057</td><td>0.039</td><td>0.065</td><td>0.019</td><td>0.040</td></tr><tr><td>DeepTAM (w/o flow) (Zhou et al., 2018)</td><td>0.069</td><td>0.042</td><td>0.025</td><td>0.063</td><td>0.051</td><td>0.070</td><td>0.030</td><td>0.050</td></tr><tr><td>Ours</td><td>0.046</td><td>0.034</td><td>0.017</td><td>0.052</td><td>0.032</td><td>0.037</td><td>0.014</td><td>0.033</td></tr></table>

Table 5: Tracking results in the RGB-D benchmark (translational rmse [m/s]).

## 5 CONCLUSION

We propose DeepV2D, a deep learning architecture which is built by composing classical geometric algorithms into a fully differentiable pipeline. DeepV2D is flexible and performs well across a variety of tasks and datasets.

Acknowledgements We would like to thank Zhaoheng Zheng for helping with baseline experiments. This work was partially funded by the Toyota Research Institute, the King Abdullah University of Science and Technology (KAUST) Office of Sponsored Research (OSR) under Award No. OSR-2015-CRG4-2639, and the National Science Foundation under Grant No. 1617767.

## A APPENDIX

## A.1 LS-OPTIMIZATION LAYER:

In Equation 4 we defined the residual error to be:

$$
\mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) = \mathbf { r } _ { k } - [ \pi ( ( e ^ { \xi _ { j } } \mathbf { G } _ { j } ) ( e ^ { \xi _ { i } } \mathbf { G } _ { i } ) ^ { - 1 } \mathbf { X } _ { k } ^ { i } ) - \pi ( \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) ] , \qquad \mathbf { X } _ { k } ^ { i } = \pi ^ { - 1 } ( \mathbf { x } _ { k } , z _ { k } )\tag{8}
$$

and the objective function as the weighted sum of error terms:

$$
E ( \pmb { \xi } ) = \sum _ { ( i , j ) \in \mathcal { C } } \sum _ { k } \mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) ^ { T } d i a g ( \mathbf { w } _ { k } ) \mathbf { e } _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) , \qquad d i a g ( \mathbf { w } _ { k } ) = \left( \begin{array} { l l } { w _ { k } ^ { u } } & { 0 } \\ { 0 } & { w _ { k } ^ { v } } \end{array} \right)\tag{9}
$$

We apply a Gauss-Newton update to Equation 9. The Gauss-Newton update is computed by solving for the minimum of the second order approximation of the objective function:

$$
\boldsymbol { \xi } ^ { * } = - ( \mathbf { J } ^ { T } \mathbf { W } \mathbf { J } ) ^ { - 1 } \mathbf { J } ^ { T } \mathbf { W } \mathbf { r } ( \xi _ { 1 } , . . . , \xi _ { N } ) , \qquad \mathbf { J } _ { p } = \frac { \partial r _ { p } ( \epsilon ) } { \partial \epsilon } | _ { \epsilon = 0 }\tag{10}
$$

where $\mathbf { r } ( \xi _ { 1 } , . . . , \xi _ { N } )$ is the stack of residuals and J is the Jacobian matrix. Each row $\mathbf { J } _ { i }$ is the Jacobian of the $\mathrm { i } ^ { t h }$ error term w.r.t to each of the parameters. Each ξ is 6-dimensional, so optimizing over N poses means we are updating 6N variables.

Let $r _ { p } = e _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } )$ be the $\mathrm { p } ^ { t h }$ residual, then

$$
\begin{array} { r l r } & { } & { \displaystyle { \frac { \partial e _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) } { \partial \xi _ { j } } } \vert _ { \xi _ { i } = 0 , \xi _ { j } = 0 } = \frac { \partial } { \partial \xi _ { j } } [ { \bf r } _ { k } - [ \pi ( ( e ^ { \xi _ { j } } { \bf G } _ { j } ) ( e ^ { \xi _ { i } } { \bf G } _ { i } ) ^ { - 1 } { \bf X } _ { k } ^ { i } ) - \pi ( { \bf G } _ { i j } { \bf X } _ { k } ^ { i } ) ] ] = } \\ & { } & { \displaystyle { \frac { \partial } { \partial \xi _ { j } } \pi ( ( e ^ { \xi _ { j } } { \bf G } _ { j } ) ( e ^ { \xi _ { i } } { \bf G } _ { i } ) ^ { - 1 } { \bf X } _ { k } ^ { i } ) } = \frac { \partial } { \partial \xi _ { j } } \pi ( e ^ { \xi _ { j } } ( { \bf G } _ { i j } { \bf X } _ { k } ^ { i } ) ) } \\ & { } & { \displaystyle { \frac { \partial e _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) } { \partial \xi _ { j } } \vert _ { \xi _ { i } = 0 , \xi _ { j } = 0 } = \frac { \partial } { \partial ( { \bf G } _ { i j } { \bf X } _ { k } ^ { i } ) } \pi ( { \bf G } _ { i j } { \bf X } _ { k } ^ { i } ) \cdot \frac { \partial } { \partial \xi _ { j } } e ^ { \xi _ { i } } ( { \bf G } _ { i j } { \bf X } _ { k } ^ { i } ) } } \end{array}\tag{11}
$$

Likewise, the Jacobian for $\xi _ { j }$ is

$$
\begin{array} { r l } & { \displaystyle \frac { \partial e _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) } { \partial \xi _ { j } } \vert _ { \xi _ { i } = 0 , \xi _ { j } = 0 } = \frac { \partial } { \partial \xi _ { i } } [ \mathbf { r } _ { k } - [ \pi ( ( e ^ { \xi _ { j } } \mathbf { G } _ { j } ) ( e ^ { \xi _ { i } } \mathbf { G } _ { i } ) ^ { - 1 } \mathbf { X } _ { k } ^ { i } ) - \pi ( \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) ] ] = } \\ & { \displaystyle \frac { \partial } { \partial \xi _ { i } } \pi ( ( e ^ { \xi _ { j } } \mathbf { G } _ { j } ) ( e ^ { \xi _ { i } } \mathbf { G } _ { i } ) ^ { - 1 } \mathbf { X } _ { k } ^ { i } ) = \frac { \partial } { \partial \xi _ { i } } \pi ( \mathbf { G } _ { j } \mathbf { G } _ { i } ^ { - 1 } e ^ { - \xi _ { i } } \mathbf { X } _ { k } ^ { i } ) = \frac { \partial } { \partial \xi _ { i } } \pi ( \mathbf { G } _ { i j } e ^ { - \xi _ { i } } \mathbf { X } _ { k } ^ { i } ) } \end{array}\tag{12}
$$

using the adjoint to move the increment to the left of the transformation

$$
\begin{array} { r } { \displaystyle = \frac { \partial } { \partial \xi _ { i } } \pi ( e ^ { w } \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) \qquad \mathrm { w h e r e ~ } w = - A d j _ { \mathbf { G } _ { i j } } \cdot \boldsymbol { \xi } } \\ { \displaystyle \frac { \partial e _ { k } ^ { i j } ( \xi _ { i } , \xi _ { j } ) } { \partial \xi _ { j } } \vert _ { \xi _ { i } = 0 , \xi _ { j } = 0 } = - \frac { \partial } { \partial ( \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) } \pi ( \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) \cdot \frac { \partial } { \partial w } e ^ { w } ( \mathbf { G } _ { i j } \mathbf { X } _ { k } ^ { i } ) \cdot A d j _ { \mathbf { G } _ { i j } } } \end{array}\tag{13}
$$

where the Jacobian of the action of a SE(3) element on a 3D point is computed

$$
\frac { \partial e ^ { \xi } \mathbf { X } } { \partial \xi } | _ { \xi = 0 } = [ \begin{array} { c c c } { 1 } & { 0 } & { 0 } \\ { 0 } & { 1 } & { 0 } \\ { 0 } & { 0 } & { 1 } \end{array} ] \begin{array} { c c c } { 0 } & { - Z } & { Y } \\ { Z } & { 0 } & { X } \\ { - Y } & { X } & { 0 } \end{array} ]\tag{14}
$$

During training, we propagate through the Gauss-Newton update. The update is found by solving the linear system

$$
\mathbf { H } \xi = - \mathbf { b } , \qquad \mathbf { H } = \mathbf { J } ^ { T } \mathbf { W } \mathbf { J } , \ \mathbf { \ b } = \mathbf { J } ^ { T } \mathbf { W } \mathbf { r } ( \xi _ { 1 } , . . . , \xi _ { N } )\tag{15}
$$

Since H is positive definite, we solve Equation 15 using Cholesky decomposition. In the backward pass, the gradients can be found by solving another linear system.

$$
\frac { \partial \mathcal { L } } { \partial \mathbf { H } } = - \boldsymbol { \xi } \cdot ( \mathbf { H } ^ { - T } \frac { \partial \mathcal { L } } { \partial \boldsymbol { \xi } } ) ^ { T } , \qquad \frac { \partial \mathcal { L } } { \partial \mathbf { b } } = \mathbf { H } ^ { - T } \frac { \partial \mathcal { L } ^ { T } } { \partial \boldsymbol { \xi } }\tag{16}
$$

## B TRAINING DETAILS

DeepV2D is implemented in Tensorflow (Abadi et al., 2016). All components of the network are trained from scratch without using any pretrained weights. We use gradient checkpointing (Chen et al., 2016) to reduce memory usage and increase batch size.

When training on NYU and ScanNet, we train with 4 frame video clips. On KITTI, we use 5 frame video clips. The video clips are created by first selecting a keyframe. The other frames are randomly sampled from the set of frames within a specified time window of the keyframe. For example, on NYU, we create the training video by sampling from frames within 1 second of the keyframe.

Training occurs in the following two stages:

Stage I: We train the Motion Module using the $L _ { m o t i o n }$ loss with RMSProp (Tieleman & Hinton, 2012) and a learning rate of 0.0001. For the input depth, we use the ground truth depth with missing values interpolated. We train Stage I for 20k iterations on NYU, 16k iterations on KITTI, and 30k iterations on ScanNet.

Stage II: In stage II, we jointly train the motion and depth modules end-to-end on the combined loss with RMSProp. The initial learning rate is set to .001 and decayed to .0002 after 100k training steps. During the second stage we store depth predictions to be used during the next training epoch. We train Stage II for a total of 120k iterations with a batch size of 2. In our ScanNet experiments, we train for an additional 60k iterations.

Data Augmentation: We perform data augmentation by adjusting brightness, gamma, and performing random scaling of the image channels. We also randomly perturb the input camera pose to the Motion Module by sampling small perturbations.

## C TIMING AND MEMORY USAGE

In the below table we provide timing and peak memory usage for different versions of our method. All results are obtained using 8 frame video sequences as input with the exception of the basline single-image network FCRN Laina et al. (2016) which uses a single frame as input.

<table><tr><td></td><td>Abs-Rel ↓</td><td>Parameters</td><td>Peak GPU Memory</td><td>Iteration Time</td></tr><tr><td>FCRN (Laina et al., 2016)</td><td>0.121</td><td>64M</td><td>0.1G</td><td>0.05s</td></tr><tr><td>Ours (1/2 res)</td><td>0.083</td><td>32M</td><td>0.7G</td><td>0.22s</td></tr><tr><td>Ours (1-HG)</td><td>0.071</td><td>16M</td><td>2.8G</td><td>0.61s</td></tr><tr><td>Ours (corr)</td><td>0.135</td><td>25M</td><td>1.8G</td><td>0.32s</td></tr><tr><td>Ours</td><td>0.062</td><td>32M</td><td>2.8G</td><td>0.69s</td></tr></table>

Table 6: Timing and memory details for different versions of our approach.

In ours(1-HG) we replace the feature extractor with a single 2D-hourglass network, and replace the stereo network with a single 3D-hourglass network. The shallower network still performs well, but causes Abs-Rel to increase from 0.065 to 0.071, showing that stacking hourglass networks is beneficial for performance. In ours (1/2 res) we test the performance of DeepV2D when images are downsampled to 1/2 resolution for training and inference. Using lower resolution images decreases memory usage and inference time but slightly decreases accuracy.

We also test a version where we replace the 3d stereo network with a correlation layer and 2d encoder-decoder. In ours(corr), we take the correlation between features over the same depth range as we use to build the 3D cost volume, then concatenate the correlation response with features from the keyframe image, similar to DispNet (Mayer et al., 2016b). The correlation version performs worse, increasing Abs-Rel from 0.065 to 0.135. This is consistent with prior work which has demonstrated that 3D cost volumes give better performance than direct correlation (Kendall et al., 2017; Chang & Chen, 2018).

## D ADDITIONAL TRACKING INFORMATION

In Table 7 we report tracking results for all sequences in the Freiburg 1 dataset.

<table><tr><td rowspan=1 colspan=1>Sequence</td><td rowspan=1 colspan=1>RGB-D SLAM</td><td rowspan=1 colspan=1>DeepTAM</td><td rowspan=1 colspan=1>Ours</td></tr><tr><td rowspan=1 colspan=1>360</td><td rowspan=1 colspan=1>0.119</td><td rowspan=1 colspan=1>0.063</td><td rowspan=1 colspan=1>0.056</td></tr><tr><td rowspan=1 colspan=1>360(v)</td><td rowspan=1 colspan=1>0.125</td><td rowspan=1 colspan=1>0.054</td><td rowspan=1 colspan=1>0.046</td></tr><tr><td rowspan=2 colspan=1>deskdesk(v)</td><td rowspan=1 colspan=1>0.030</td><td rowspan=1 colspan=1>0.033</td><td rowspan=14 colspan=1>0.0290.0340.0410.0170.0640.0190.0520.0470.0320.0390.0370.0430.0250.016</td></tr><tr><td rowspan=1 colspan=1>0.037</td><td rowspan=1 colspan=1>0.027</td></tr><tr><td rowspan=3 colspan=1>desk2desk2(v)floor</td><td rowspan=1 colspan=1>0.055</td><td rowspan=1 colspan=1>0.046</td></tr><tr><td rowspan=1 colspan=1>0.020</td><td rowspan=1 colspan=1>0.017</td></tr><tr><td rowspan=1 colspan=1>0.090</td><td rowspan=1 colspan=1>0.081</td></tr><tr><td rowspan=1 colspan=1>plant</td><td rowspan=1 colspan=1>0.036</td><td rowspan=1 colspan=1>0.027</td></tr><tr><td rowspan=1 colspan=1>plant(v)</td><td rowspan=1 colspan=1>0.062</td><td rowspan=1 colspan=1>0.057</td></tr><tr><td rowspan=1 colspan=1>room</td><td rowspan=1 colspan=1>0.048</td><td rowspan=1 colspan=1>0.040</td></tr><tr><td rowspan=6 colspan=1>room(v)rpyrpy(v)teddyxyzxyz(v)</td><td rowspan=1 colspan=1>0.042</td><td rowspan=1 colspan=1>0.039</td></tr><tr><td rowspan=1 colspan=1>0.043</td><td rowspan=1 colspan=1>0.046</td></tr><tr><td rowspan=1 colspan=1>0.082</td><td rowspan=1 colspan=1>0.065</td></tr><tr><td rowspan=1 colspan=1>0.067</td><td rowspan=1 colspan=1>0.059</td></tr><tr><td rowspan=1 colspan=1>0.051</td><td rowspan=1 colspan=1>0.019</td></tr><tr><td rowspan=1 colspan=1>0.024</td><td rowspan=1 colspan=1>0.017</td></tr><tr><td rowspan=1 colspan=1>Average</td><td rowspan=1 colspan=1>0.058</td><td rowspan=1 colspan=1>0.043</td><td rowspan=1 colspan=1>0.037</td></tr></table>

Table 7: Per-Sequence tracking results on the RGB-D benchmark evaluated using translational RMSE [m/s]. We outperform DeepTAM and DVO on 12 of the 16 sequences and achieve a lower translational RMSE averaged over all sequences. While DeepTAM requires optical flow supervision to achieve good performance, we do not require supervision on optical flow since the relation between camera motion and optical flow is embedded into our network architecture.

## E CAMERA POSE ABLATIONS

The focus of this work on depth estimation, but we are interested in how different methods for estimating camera pose impact the final performance. In Table 8, we test different methods for estimating camera pose on NYU. In each experiment, we replace the motion module of our trained network with the given alternative, and test the final results. We also report results from MVSNet (trained on NYU) using each SfM implementation.

COLMAP (Schonberger & Frahm, 2016a) and OpenMVG (Moulon et al.) are publicly available SfM implementations. They do not return results on all input sequences, so we only evaluate sequences were they converge without an error. PWCNet+Ceres takes the output of an optical flow network, PWCNet (Sun et al., 2018), and performs joint optimization of depth and pose using the Ceres solver (Agarwal et al., 2012). Finally, we evaluate MVSNet (Yao et al., 2018) when the pose predicted by DeepV2D is given as input. Note that not all SfM implementations converge on all sequences (success rate is reported in parenthesis) and we only evaluate the method on the frames in which it converges.

<table><tr><td>Depth</td><td>Motion</td><td>Abs-Rel ↓  $\delta _ { 1 } \uparrow$ </td><td> $\delta _ { 2 }$  ↑</td><td> $\delta _ { 3 } \uparrow$ </td></tr><tr><td>MVSNet DeepV2D</td><td>Identity Identity COLMAP (274/654)</td><td>0.419 0.382 0.362 0.460 0.724</td><td>0.681 0.756</td><td>0.859 0.901</td></tr><tr><td>MVSNet DeepV2D MVSÑet</td><td>COLMAP OpenMVG (422/654)</td><td>0.244 0.199 0.741 0.181 0.766</td><td>0.857 0.878 0.913</td><td>0.925 0.940</td></tr><tr><td>DeepV2D</td><td>OpenMVG PWC+Ceres (654/654)</td><td>0.173 0.774</td><td>0.913</td><td>0.965 0.963</td></tr><tr><td>MVSNet DeepV2D MVSNet</td><td>PWC+Ceres DeepV2D (654/654)</td><td>0.279 0.651 0.274 0.664 0.101 0.885</td><td>0.845 0.846 0.970</td><td>0.925 0.925 0.990</td></tr></table>

Table 8: Impact of pose estimation method on depth accuracy. Replacing our motion module with SfM degrades performance for both MVSNet and our approach.

We also show results of our method when the motion module is replaced with other methods for estimation motion. In all cases, using SfM results in worse performance. We observe that classical SfM is not robust enough to consistently produce accurate poses, which leads to large errors on the test set. MVSNet performs better using the poses estimated by our network, but still underperforms our full system, showing the importance of differentiable alternation between pose and stereo.

## F ADDITIONAL RESULTS

![](images/2020_DeepV2D/89cd0af44a84ebef429fee19bf46dbb6945260cacd9743fe67e2ba0172092177.jpg)  
Figure 6: Visualizations of depth predictions on KITTI dataset.

Image  
GT  
FCRN  
DeMoN  
Ours  
![](images/2020_DeepV2D/c3b483909a99f1a29db453d2b0da7d1456d3bb01a82d76bc92a7133a798f6856.jpg)  
Figure 7: Additional results on the NYU depth dataset Silberman et al. (2012) using 7-frame video clips. We show results compared with Laina et al. (2016) and Ummenhofer et al. (2017).

## G NETWORK ARCHITECTURES

![](images/2020_DeepV2D/15d2a9fef51d6cfa218a20c0e389d2d8384bb2ee688e8fb50f961679d4380ca2.jpg)  
Figure 8: Motion Module Architecture: The Encoder(left) extracts a dense 1/4 resolution feature map for each of the input images. The Residual Flow Network (right) takes in a pair of feature maps and estimates the residual flow and corresponding weights. This residual flow is estimated with an encoder-decoder network, with skip connections formed by concatenating feature maps. Numbers in parenthesis correspond to the number of output channels for each layer.

![](images/2020_DeepV2D/07159c3764b395e7e9b73184aa092e0eb6a9fc35b95d3165068bb3d1b63d0de3.jpg)  
Figure 9: Depth Module Architecture: The 2D encoder (top) is applied to each image in the video sequence. The 2D Encoder consists of a series of residual convolutions and 2 Hourglass Networks. The hourglass networks process the incoming features maps as multiple scales. The hourglass network is defined recursively (i.e. HG(n) contains lower resolution hourglass HG(n-1)). We use 4 nested hourglass modules with feature dimension 64-128-192-256. The resulting feature maps from the 2D encoder are used to construct the cost volumes. The 3D matching network (bottom) takes a collection of cost volumes as input. After a 1x1x1 convolutional layer and a 3x3x3 residual convo lution, we perform view pooling, which aggregates information over all the frames in the video. The aggregated volume is then processed by a series of 3D hourglass networks, each of which outputs an intermediate depth estimate. The widths of the 3D hourglass is 32-80-128-176.

## REFERENCES

Mart´ın Abadi, Paul Barham, Jianmin Chen, Zhifeng Chen, Andy Davis, Jeffrey Dean, Matthieu Devin, Sanjay Ghemawat, Geoffrey Irving, Michael Isard, et al. Tensorflow: A system for largescale machine learning. In OSDI, volume 16, pp. 265–283, 2016.

Sameer Agarwal, Keir Mierle, et al. Ceres solver. 2012.

Ibraheem Alhashim and Peter Wonka. High quality monocular depth estimation via transfer learning. arXiv preprint arXiv:1812.11941, 2018.

Michael Bloesch, Jan Czarnowski, Ronald Clark, Stefan Leutenegger, and Andrew J Davison. Codeslamlearning a compact, optimisable representation for dense visual slam. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 2560–2568, 2018.

Jia-Ren Chang and Yong-Sheng Chen. Pyramid stereo matching network. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 5410–5418, 2018.

Tianqi Chen, Bing Xu, Chiyuan Zhang, and Carlos Guestrin. Training deep nets with sublinear memory cost. arXiv preprint arXiv:1604.06174, 2016.

Ronald Clark, Michael Bloesch, Jan Czarnowski, Stefan Leutenegger, and Andrew J Davison. Learning to solve nonlinear least squares for monocular stereo. In Proceedings of the European Conference on Computer Vision (ECCV), pp. 284–299, 2018.

Angela Dai, Angel X Chang, Manolis Savva, Maciej Halber, Thomas Funkhouser, and Matthias Nießner. Scannet: Richly-annotated 3d reconstructions of indoor scenes. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 5828–5839, 2017.

Alexey Dosovitskiy, Philipp Fischer, Eddy Ilg, Philip Hausser, Caner Hazirbas, Vladimir Golkov, Patrick Van Der Smagt, Daniel Cremers, and Thomas Brox. Flownet: Learning optical flow with convolutional networks. In Proceedings ofthe IEEE International Conference on Computer Vision, pp. 2758–2766, 2015.

David Eigen and Rob Fergus. Predicting depth, surface normals and semantic labels with a common multi-scale convolutional architecture. In Proceedings of the IEEE International Conference on Computer Vision, pp. 2650–2658, 2015.

David Eigen, Christian Puhrsch, and Rob Fergus. Depth map prediction from a single image using a multi-scale deep network. In Advances in neural information processing systems, pp. 2366–2374, 2014.

Jakob Engel, Thomas Schops, and Daniel Cremers. Lsd-slam: Large-scale direct monocular slam.¨ In European Conference on Computer Vision, pp. 834–849. Springer, 2014.

Jakob Engel, Vladlen Koltun, and Daniel Cremers. Direct sparse odometry. IEEE transactions on pattern analysis and machine intelligence, 40(3):611–625, 2018.

Huan Fu, Mingming Gong, Chaohui Wang, Kayhan Batmanghelich, and Dacheng Tao. Deep ordinal regression network for monocular depth estimation. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 2002–2011, 2018.

Yasutaka Furukawa and Jean Ponce. Accurate, dense, and robust multiview stereopsis. IEEE transactions on pattern analysis and machine intelligence, 32(8):1362–1376, 2010.

Yasutaka Furukawa, Carlos Hernandez, et al. Multi-view stereo: A tutorial.´ Foundations and Trends R in Computer Graphics and Vision, 9(1-2):1–148, 2015.

Andreas Geiger, Philip Lenz, Christoph Stiller, and Raquel Urtasun. Vision meets robotics: The kitti dataset. The International Journal ofRobotics Research, 32(11):1231–1237, 2013.

Riccardo Gherardi, Michela Farenzena, and Andrea Fusiello. Improving the efficiency of hierarchical structure-and-motion. In Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on, pp. 1594–1600. IEEE, 2010.

Clement Godard, Oisin Mac Aodha, and Gabriel J Brostow. Unsupervised monocular depth estima-´ tion with left-right consistency. In CVPR, volume 2, pp. 7, 2017.

Hyowon Ha, Sunghoon Im, Jaesik Park, Hae-Gon Jeon, and In So Kweon. High-quality depth from uncalibrated small motion clip. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 5413–5421, 2016.

Xufeng Han, Thomas Leung, Yangqing Jia, Rahul Sukthankar, and Alexander C Berg. Matchnet: Unifying feature and metric learning for patch-based matching. In Computer Vision and Pattern Recognition (CVPR), 2015 IEEE Conference on, pp. 3279–3286. IEEE, 2015.

Max Jaderberg, Karen Simonyan, Andrew Zisserman, et al. Spatial transformer networks. In Advances in neural information processing systems, pp. 2017–2025, 2015.

Abhishek Kar, Jitendra Malik, and Christian Hane. Learning a multi-view stereo machine. In¨ Advances in Neural Information Processing Systems, pp. 364–375, 2017.

Alex Kendall and Roberto Cipolla. Geometric loss functions for camera pose regression with deep learning. In Proc. CVPR, volume 3, pp. 8, 2017.

Alex Kendall, Matthew Grimes, and Roberto Cipolla. Posenet: A convolutional network for realtime 6-dof camera relocalization. In Computer Vision (ICCV), 2015 IEEE International Conference on, pp. 2938–2946. IEEE, 2015.

Alex Kendall, Hayk Martirosyan, Saumitro Dasgupta, Peter Henry, Ryan Kennedy, Abraham Bachrach, and Adam Bry. End-to-end learning of geometry and context for deep stereo regression. In Proceedings of the IEEE International Conference on Computer Vision, pp. 66–75, 2017.

Christian Kerl, Jurgen Sturm, and Daniel Cremers. Dense visual slam for rgb-d cameras. In Intelligent Robots and Systems (IROS), 2013 IEEE/RSJ International Conference on, pp. 2100–2106. IEEE, 2013.

Iro Laina, Christian Rupprecht, Vasileios Belagiannis, Federico Tombari, and Nassir Navab. Deeper depth prediction with fully convolutional residual networks. In 3D Vision (3DV), 2016 Fourth International Conference on, pp. 239–248. IEEE, 2016.

Vincent Lepetit, Francesc Moreno-Noguer, and Pascal Fua. Epnp: An accurate o (n) solution to the pnp problem. International journal ofcomputer vision, 81(2):155, 2009.

Shiqi Li, Chi Xu, and Ming Xie. A robust o (n) solution to the perspective-n-point problem. IEEE transactions on pattern analysis and machine intelligence, 34(7):1444–1450, 2012.

H Christopher Longuet-Higgins. A computer algorithm for reconstructing a scene from two projections. Nature, 293(5828):133–135, 1981.

David G Lowe. Distinctive image features from scale-invariant keypoints. International journal of computer vision, 60(2):91–110, 2004.

Reza Mahjourian, Martin Wicke, and Anelia Angelova. Unsupervised learning of depth and egomotion from monocular video using 3d geometric constraints. In Proceedings of the IEEE Con ference on Computer Vision and Pattern Recognition, pp. 5667–5675, 2018.

Nikolaus Mayer, Eddy Ilg, Philip Hausser, Philipp Fischer, Daniel Cremers, Alexey Dosovitskiy, and Thomas Brox. A large dataset to train convolutional networks for disparity, optical flow, and scene flow estimation. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 4040–4048, 2016a.

Nikolaus Mayer, Eddy Ilg, Philip Hausser, Philipp Fischer, Daniel Cremers, Alexey Dosovitskiy, and Thomas Brox. A large dataset to train convolutional networks for disparity, optical flow, and scene flow estimation. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 4040–4048, 2016b.

Roger Mohr, Long Quan, and Franc¸oise Veillon. Relative 3d reconstruction using multiple uncalibrated images. The International Journal ofRobotics Research, 14(6):619–632, 1995.

Pierre Moulon, Pascal Monasse, Renaud Marlet, and Others. Openmvg. an open multiple view geometry library. https://github.com/openMVG/openMVG.

Raul Mur-Artal and Juan D Tardos. Orb-slam2: An open-source slam system for monocular, stereo,´ and rgb-d cameras. IEEE Transactions on Robotics, 33(5):1255–1262, 2017.

Raul Mur-Artal, Jose Maria Martinez Montiel, and Juan D Tardos. Orb-slam: a versatile and accurate monocular slam system. IEEE Transactions on Robotics, 31(5):1147–1163, 2015.

Richard A Newcombe, Steven J Lovegrove, and Andrew J Davison. Dtam: Dense tracking and mapping in real-time. In 2011 international conference on computer vision, pp. 2320–2327. IEEE, 2011.

Alejandro Newell, Kaiyu Yang, and Jia Deng. Stacked hourglass networks for human pose estimation. In European Conference on Computer Vision, pp. 483–499. Springer, 2016.

Rene Ranftl, Vibhav Vineet, Qifeng Chen, and Vladlen Koltun. Dense monocular depth estimation´ in complex dynamic scenes. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 4058–4066, 2016.

Johannes L Schonberger and Jan-Michael Frahm. Structure-from-motion revisited. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 4104–4113, 2016a.

Johannes L Schonberger and Jan-Michael Frahm. Structure-from-motion revisited. In Proceedings ofthe IEEE Conference on Computer Vision and Pattern Recognition, pp. 4104–4113, 2016b.

Nathan Silberman, Derek Hoiem, Pushmeet Kohli, and Rob Fergus. Indoor segmentation and support inference from rgbd images. Computer Vision–ECCV 2012, pp. 746–760, 2012.

Keith N Snavely. Scene reconstruction and visualization from internet photo collections. 2009.

Noah Snavely. Scene reconstruction and visualization from internet photo collections: A survey. IPSJ Transactions on Computer Vision and Applications, 3:44–66, 2011.

Deqing Sun, Xiaodong Yang, Ming-Yu Liu, and Jan Kautz. Pwc-net: Cnns for optical flow using pyramid, warping, and cost volume. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 8934–8943, 2018.

Chengzhou Tang and Ping Tan. Ba-net: Dense bundle adjustment network. arXiv preprint arXiv:1806.04807, 2018.

Tijmen Tieleman and Geoffrey Hinton. Lecture 6.5-rmsprop: Divide the gradient by a running average of its recent magnitude. COURSERA: Neural networks for machine learning, 4(2):26– 31, 2012.

Benjamin Ummenhofer, Huizhong Zhou, Jonas Uhrig, Nikolaus Mayer, Eddy Ilg, Alexey Dosovitskiy, and Thomas Brox. Demon: Depth and motion network for learning monocular stereo. In Proceedings ofthe IEEE Conference on Computer Vision and Pattern Recognition, pp. 5038– 5047, 2017.

Sudheendra Vijayanarasimhan, Susanna Ricco, Cordelia Schmid, Rahul Sukthankar, and Katerina Fragkiadaki. Sfm-net: Learning of structure and motion from video. arXiv preprint arXiv:1704.07804, 2017.

Chaoyang Wang, Jose Miguel Buenaposada, Rui Zhu, and Simon Lucey. Learning depth from ´ monocular videos using direct methods. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 2022–2030, 2018.

Sen Wang, Ronald Clark, Hongkai Wen, and Niki Trigoni. Deepvo: Towards end-to-end visual odometry with deep recurrent convolutional neural networks. In Robotics and Automation (ICRA), 2017 IEEE International Conference on, pp. 2043–2050. IEEE, 2017.

Changchang Wu et al. Visualsfm: A visual structure from motion system. 2011.

Jianxiong Xiao, Andrew Owens, and Antonio Torralba. Sun3d: A database of big spaces reconstructed using sfm and object labels. In Proceedings of the IEEE International Conference on Computer Vision, pp. 1625–1632, 2013.

Nan Yang, Rui Wang, Jorg Stuckler, and Daniel Cremers. Deep virtual stereo odometry: Leveraging deep depth prediction for monocular direct sparse odometry. In Proceedings of the European Conference on Computer Vision (ECCV), pp. 817–833, 2018.

Yao Yao, Zixin Luo, Shiwei Li, Tian Fang, and Long Quan. Mvsnet: Depth inference for unstructured multi-view stereo. In Proceedings ofthe European Conference on Computer Vision (ECCV), pp. 767–783, 2018.

Zhichao Yin and Jianping Shi. Geonet: Unsupervised learning of dense depth, optical flow and camera pose. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 1983–1992, 2018.

Huizhong Zhou, Benjamin Ummenhofer, and Thomas Brox. Deeptam: Deep tracking and mapping. In Proceedings ofthe European Conference on Computer Vision (ECCV), pp. 822–838, 2018.

Tinghui Zhou, Matthew Brown, Noah Snavely, and David G Lowe. Unsupervised learning of depth and ego-motion from video. In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 1851–1858, 2017.