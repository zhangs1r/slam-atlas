# NeurAR: Neural Uncertainty for Autonomous 3D Reconstruction With Implicit Neural Representations

Yunlong Ran , Jing Zeng , Shibo He , Senior Member, IEEE, Jiming Chen , Fellow, IEEE, Lincheng Li , Yingfeng Chen , Gimhee Lee, and Qi Ye

Abstract—Implicit neural representations have shown compelling results in offline 3D reconstruction and also recently demonstrated the potential for online SLAM systems. However, applying them to autonomous 3D reconstruction, where a robot is required to explore a scene and plan a view path for the reconstruction, has not been studied. In this paper, we explore for the first time the possibility of using implicit neural representations for autonomous 3D scene reconstruction by addressing two key challenges: 1) seeking a criterion to measure the quality of the candidate viewpoints for the view planning based on the new representations, and 2) learning the criterion from data that can generalize to different scenes instead of a hand-crafting one. To solve the challenges, firstly, a proxy of Peak Signal-to-Noise Ratio (PSNR) is proposed to quantify a viewpoint quality; secondly, the proxy is optimized jointly with the parameters of an implicit neural network for the scene. With the proposed view quality criterion from neural networks (termed as Neural Uncertainty), we can then apply implicit representations to autonomous 3D reconstruction. Our method demonstrates significant improvements on various metrics for the rendered image quality and the geometry quality of the reconstructed 3D models when compared with variants using TSDF or reconstruction without view planning.

Index Terms—Computer vision for automation, motion and path planning, planning under uncertainty.

## I. INTRODUCTION

UTONOMOUS 3D reconstruction has a wide range of applications, e.g. augmented/virtual reality, autonomous driving, filming, gaming, medicine, architecture. The problem requires a robot to make decisions about moving towards which viewpoint in each step to get the best reconstruction quality of an unknown scene with the lowest cost, i.e. view planning. In this work, we assume a robot can localize itself and at each viewpoint, the information of a scene is captured by an RGB image (with an optional depth image).

Implicit neural representations for 3D objects have shown their potential to be precise in geometry encoding, efficient in memory consumption (adaptive to scene size and complexity), predictive in filling unseen regions and flexible in the amount of training data. The reconstruction with offline images [1] or online images with cameras held by human [2] has achieved compelling results recently with implicit neural representations. However, leveraging these advancements to achieve highquality autonomous 3D reconstruction has not been studied.

Previous 3D representations for autonomous 3D reconstruction include point cloud, volume, and surface. To plan the view without global information of a scene, previous work resorts to a greedy strategy: given the current position of a robot and the reconstruction status, they quantify the quality of the candidate viewpoints via information gain to plan the next best view (NBV). In these works, the information gain relies on hand-crafted criteria, each designed ad-hoc for a particular combination of a 3D representation and a reconstruction algorithm. For example, Mendez et al. [3] define view cost by triangulated uncertainty given by the algorithm inferring depth from stereo RGB images, Isler et al. [4] quantify the information gain for a view using entropy in voxels seen in this viewpoint, Wu et al. [5] identify the quality of the view by a Poisson field from point clouds and Song et al. [6] leverage mesh holes and boundaries to guide the view planning.

To use implicit neural representations for autonomous 3D reconstruction, a key capability is to quantify the quality of the candidate viewpoints. For implicit neural representations, how to define the viewpoint quality? Is it possible for the neural network to learn a measurement of the quality from data instead of defining it heuristically? In this paper, we make efforts to answer both questions.

The quality of reconstructed 3D models can be measured by the quality of images rendered from different viewpoints and the measurement is adopted in many offline 3D reconstruction works. PSNR is one popular measurement and it is defined according to the difference between the images rendered from the reconstructed model and the ground truth model.

The ground truth images, however, are unavailable for unvisited viewpoints to calculate PSNR during autonomous reconstruction. Is it possible to learn a proxy for PSNR? In [7], [8], the authors point out that if the target variable to regress is under a Gaussian noise model and the target distribution conditioned on the input is optimized by maximum likelihood, the optimum value of the noise variance is given by the residual variance of the target values and the regressed ones. Inspired by this, we assume the color to regress for a spatial point in a scene as a random variable modeled by a Gaussian distribution. The Gaussian distribution models the uncertainty of the reconstruction and the variance quantifies the uncertainty. When the regression network converges, the variance of the distribution is given by the squared error of the predicted color and the ground truth color; the integral of the uncertainty of points in the frustum of a viewpoint can be taken as a proxy of PSNR to measure the quality of candidate viewpoints.

With the key questions solved, we are able to build an autonomous 3D reconstruction system (NeurAR) using an implicit neural network. In summary, the contributions of the paper are:

\- We propose the first autonomous 3D reconstruction system using an implicit neural representation.

\- We propose a novel view quality criterion that learns online from continuously added input images per target scene instead of hand-engineering or learning from a large corpus of 3D scenes.

\- Our proposed method significantly improves on various metrics from alternatives using voxel-based representations or using man-designed paths for the reconstruction.

## II. RELATED WORK

View Planning: Most view planning methods focus on the NBV problem which uses feedback from current partial reconstruction to determine NBV. According to the representations for the 3D models, these methods can be divided into voxel-based method [3], [4] and surface-based method [5], [9].

The voxel-based methods are most commonly used due to their simplicity in representing space. Vasquez-Gomez et al. [10] analyze a set of boundary voxels and determine NBV for dense 3D modeling ofsmall-scale objects. Stefan Isler et al. [4] provide several metrics to quantify the volume information contained in the voxels. Mendez et al. [3] define the information gain by the triangulation uncertainty of stereo images. Despite the simplicity, these methods suffer from memory consumption with growing scene complexity and higher spatial resolutions.

A complete volumetric map does not necessarily guarantee a perfect 3D surface. Therefore, researchers propose to analyze the shape and quality of the reconstructed surface for NBV [5], [9]. Wu et al. [5] estimate a confidence map representing the completeness and smoothness of the constructed Poisson isosurface and the confidence map is used to guide the calculation of NBV. Schmid et al. [9] propose information gain to evaluate the quality of observed surfaces and unknown voxels near the observed surfaces, then plan a path by RRT.

In contrast, some methods explore function learning to solve the NBV problem [11], [12], [13]. Supervised learning-based methods [13] learn information gain of a viewpoint given a partial occupancy map and [12] learns an informed distribution of high-utility viewpoints based on a partial occupancy map. For the reinforcement learning-based methods [11], no hand-crafted heuristics are required and an agent explores the viewpoints with high overall coverage. These learning-based methods require a large-scale dataset for training and may be hard to generalize to different scenes.

Different from the hand-crafted heuristics, the learned NBV policy, and the existing learning-based information gain, our proposed neural uncertainty is learned per target scene during the reconstruction, requiring no manual definition, no large training set, and being able to work in any new scene.

Online Dense Reconstruction: For online 3D reconstruction from RGB images, most methods use Multi-View-Stereo (MVS) [14] to reconstruct dense models by first getting a sparse set of initial matches, iteratively expanding matches to nearby locations and performing surface reconstruction. With the release of commodity RGB-D sensors, the fusion of point clouds reprojected from depth images gains popularity. KinectFusion [15] achieves real-time 3D reconstruction with a moving depth camera by integrating points from depth images with Truncated Signed Distance Functions (TSDFs). OctoMap [16] builds a probabilistic occupancy volume based on octree. Recently, implicit representations have shown compelling results in 3D reconstruction, either for the radiance field approximation from RGB images [17] or the shape approximation from point clouds [18]. The novel representations are also studied for online dense reconstruction. iMAP [2] adopts MLPs as the scene representation and reconstructs the scene from RGBD images. In addition to using MLPs to represent a scene implicitly, NeRFusion [19] further combines a feature volume to fuse information from different views as a latent scene representation. Similarly, we use the implicit neural function to represent 3D models but we focus on how to leverage this representation for autonomous view planning.

## III. METHOD

## A. Problem Description and System Overview

The problem considered is to generate a trajectory for a robot that yields high-quality 3D models of a bounded priori unknown scene and fulfills robot constraints. The trajectory is defined as a sequence of viewpoints $V = ( v _ { 1 } , . . . , v _ { n } )$ where $v _ { i } \in \mathcal { R } ^ { 3 } \times$ (2) (usually roll is not considered). Let S be the set of all the sequences. The problem can therefore be expressed as

$$
V ^ { * } = \underset { V \in S } { \arg \operatorname* { m a x } } I ( V ) \mathrm { s . t . } L ( V ) \leq L _ { m a x } ,\tag{1}
$$

where $I ( V )$ equals $\Sigma I ( v _ { i } )$ , an objective function measuring the contributions from different viewpoints to the reconstruction quality and L(V ) is the length of the trajectory.

One difficulty of the problem is defining the objective function itself as there is no ground truth data to measure the quality of the reconstruction. Hence, one major line of work in this field is seeking a surrogate quantity that can be used for the view sequence optimization, which is also the focus of this paper.

Finding the best sequence for (1) is prohibitively expensive and most work resolves this problem by a greedy strategy. At each step of the reconstruction process, a number of candidate viewpoints are sampled according to some constraints, the contributions of these viewpoints to the reconstruction quality are evaluated based on the current reconstructed scene and the viewpoint with the most contribution is chosen as NBV. The process is repeated until a maximum length is reached. To move a robot to NBV, various path planning methods can be applied to navigate the robot.

In this work, we follow the greedy strategy and choose an existing RRT\* based view planner. For each viewpoint, an RGB image (with an optional depth image) is captured. To achieve an autonomous 3D reconstruction system depending only on the input images, joint estimation of camera poses and the target scene is required. To focus on view planning with implicit scene representations, we assume the ground truth camera poses are given and leave the localization of the camera as future work.

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/265d7f287d9492e790d14352d9eb1bb5165e21a525a0c51f79bc0e9216c2c1f2.jpg)  
Fig. 1. View paths planned by NeurAR and the uncertainty maps used to guide the planner. Given current pose 5 and 15, paths are planned toward viewpoints having higher uncertainties, i.e. pose 6 and 16. Uncertainties of the viewpoints are shown in red text. Notice that darker regions in uncertainty maps relate to worse quality regions of rendered images.

Accordingly, our proposed autonomous 3D reconstruction system can be divided into three modules (shown in the top of Fig. 1): a 3D reconstruction module, a view planner module, and a simulation module. At each step, the 3D reconstruction module reconstructs a scene represented by an implicit neural network with new images from the simulation module and provides a quantity (neural uncertainty) measuring the reconstructed quality of each position (Section III-B). To adapt the implicit neural representation to online reconstruction, we further propose several strategies for online training and acceleration in Section III-C. The view planner module (Section III-D) samples viewpoints from empty space, measures the contributions of these sampled viewpoints by composing the neural uncertainty, chooses NBV and plans a view path to NBV. The simulation module, built upon Unity Engine, renders RGBD images from new viewpoints and provides both the images and the camera poses to the 3D reconstruction module.

## B. 3D Reconstruction with Neural Uncertainty

In the section, we first formulate Neural Uncertainty when a target scene is approximated by an implicit neural network. Then we show how to optimize the uncertainty together with the network parameters. With the neural uncertainty formulation, a strong linear relationship is established between the uncertainty and the image quality metric, i.e. PSNR and therefore a proxy for PSNR is proposed.

1) Problem Formulation: NeRF [1] represents a continuous scene with an implicit neural function by taking the location of a point x on a ray of direction d as inputs and its color value $c ,$ density $\rho$ as outputs, i.e. $\mathbf { F } _ { \theta } ( \mathbf { x } , \mathbf { d } ) = \bar { ( } c , \rho )$ . The function $\mathbf { F } _ { \theta }$ is represented by an MLP.

To learn the neural uncertainty for view planning, we treat the color as a random variable under a Gaussian distribution instead of taking it as a deterministic one. The color distribution can be represented as $p ( C ) = \mathcal { N } ( \mu , \sigma ^ { 2 } )$ , where the RGB channels of $\dot { C }$ share the same $\sigma .$ . To estimate the distribution, we map the inputs $( \mathbf { x } , \mathbf { d } )$ to its parameters $( \mu , \sigma )$ of the distribution for the color variable $C ;$ in other words, the outputs of the MLP is $\mu , \sigma$ rather than $c ,$ so we have $\mathbf { F } _ { \theta } ( \mathbf { x } , \mathbf { d } ) = ( \bar { \mu } , \sigma , \rho )$ . Having defined the color distribution for a point x on a ray d, we now deduce the rendered color distribution for a camera ray. Following NeRF, we define the rendered color as

$$
C _ { r } = \sum _ { i = 1 } ^ { N } \omega _ { i } C _ { i } ,\tag{2}
$$

where $\omega _ { i }$ is $o _ { i } \prod _ { j = 1 } ^ { i - 1 } ( 1 - o _ { j } ) , o _ { i } = ( 1 - \exp ( - \rho _ { i } \delta _ { i } ) )$ and $\delta _ { i } =$ $d _ { i + 1 } - d _ { i }$ which represents the inter-sample distance. $p ( C _ { i } ) =$ $\dot { \mathcal { N } ( \mu _ { i } , \sigma _ { i } ^ { 2 } ) }$ is the color distribution for a point on the ray. Assuming $\{ C _ { i } \}$ for points on the ray are independent from each other, the distribution for $C _ { r }$ is a Gaussian distribution. The probability of the rendered color value being $c _ { r }$ is $p ( C _ { r } = c _ { r } ) = \mathcal { N } ( c _ { r } | \mu _ { r } , \sigma _ { r } )$ , where the mean and variance are $\begin{array} { r } { \mu _ { r } = \sum _ { i = 1 } ^ { N } \omega _ { i } \mu _ { i } } \end{array}$ and $\begin{array} { r } { \sigma _ { r } ^ { 2 } = \sum _ { i = 1 } ^ { N } \omega _ { i } \sigma _ { i } ^ { 2 } } \end{array}$

Similarly, assuming $\big \{ C _ { r } \big \}$ for different rays are independent, the random variable $C _ { I }$ for the mean rendered color of rays sampled from an image pool is also under a Gaussian distribution, whose mean and variance are

$$
\mu _ { I } = \frac { 1 } { R } \sum _ { r = 1 } ^ { R } \mu _ { r } = \frac { 1 } { R } \sum _ { r = 1 } ^ { R } \sum _ { i = 1 } ^ { N } \omega _ { r i } \mu _ { r i } ,\tag{3}
$$

$$
\sigma _ { I } ^ { 2 } = \frac { 1 } { R } \sum _ { r = 1 } ^ { R } \sigma _ { r } ^ { 2 } = \frac { 1 } { R } \sum _ { r = 1 } ^ { R } \sum _ { i = 1 } ^ { N } \omega _ { r i } \sigma _ { r i } ^ { 2 } ,\tag{4}
$$

where $r$ represents a camera ray tracing through a pixel from an image pool and R the number of rays sampled for the image pool.

2) Optimization: We now consider how to optimize the network parameters $\theta$ and determine $\mu , \sigma , \rho$ for each point on a ray (we ignore the subscript ri for brevity). Consider a pool of images, the likelihood for the mean color value $c _ { I }$ of a set of rays shooting from the pixels sampled from the image pool are obtained as $\bar { p } ( C _ { I } = c _ { I } \bar { ) } = \mathcal { N } ( c _ { I } | \bar { \mu } _ { I } , \sigma _ { I } )$ . Taking the negative logarithm of the likelihood and ignoring the constant, we have

$$
L _ { c o l o r } = \log \sigma _ { I } + \frac { L _ { m e a n } } { 2 \sigma _ { I } ^ { 2 } } \leq \log \sigma _ { I } + \frac { L _ { I } } { 2 \sigma _ { I } ^ { 2 } } ,\tag{5}
$$

$$
L _ { m e a n } = \| c _ { I } - \mu _ { I } \| _ { 2 } ^ { 2 } \leq L _ { I } = { \frac { 1 } { R } } \sum _ { r = 1 } ^ { R } \| c _ { r } - \mu _ { r } \| _ { 2 } ^ { 2 } .\tag{6}
$$

For $L _ { m e a n }$ of $( 6 )$ , the constraint for each pixel is too weak and the network is not able to converge a meaningful result (PSNR about 10). As we have supervision for the color of each pixel and $L _ { m e a n }$ is smaller than or equal to $L _ { I }$ , we choose to minimize $L _ { I }$ instead. The loss function above is differentiable w.r.t θ and $\mu , \sigma , \rho$ are the outputs ofMLPs $\left( \mathbf { F } _ { \theta } \right)$ ; we can use gradient descent to determine the network parameters $\theta$ and $\mu , \sigma , \rho .$ . To make σ positive, we let the network estimate $\sigma ^ { 2 }$ and the output of MLPs s is activated by $e ^ { s }$ to get $\sigma ^ { 2 }$

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/11d79fc5260bf0941bbbe6b28c4797b5f90e091ca83b199fe2b8e45d6370c6fc.jpg)

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/41aadaa8feb3c57e2bbc776317f51037f75fc3f2ce0c7c6c249a6131399f0d88.jpg)

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/fba698e7d0587a95bbe82cd65da27a337c54e83091539c61b09c130149d24162.jpg)

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/d203accf25d272393d92a7cbf83117707aca4d0e5e78c244029db6beb733bede.jpg)  
Fig. 2. The pipeline of our method

Consider the minimization with respect to $\theta .$ Given $\sigma _ { I } .$ , we can see the minimization of the maximum likelihood under a conditional Gaussian distribution for each point is equivalent to minimizing a mean-of-squares error function given by $L _ { I }$ in (6). Apply $\mathbf { F } _ { \theta }$ on a point on a ray and $\mu , \rho , \sigma$ can be obtained.

3) Neural Uncertainty and PSNR: For the variance $\sigma _ { I } ^ { 2 } .$ , or Neural Uncertainty, the optimum value can be achieved by setting the derivative of $L _ { c o l o r }$ with respect to $\sigma _ { I }$ to zero, giving

$$
\sigma _ { I } ^ { 2 } = L _ { I } .\tag{7}
$$

The equation above indicates that the optimal solution for $\sigma _ { I } ^ { 2 }$ is the squared errors between the predicted image and the ground truth image.

On the other hand, PSNR is defined as 10 log $_ { 1 0 } \ \frac { M A X _ { I } ^ { 2 } } { M S E }$ , where $M A X _ { I }$ is the maximum possible pixel value ofthe image. When a pixel is represented using 8 bits, it is 255 and MSE is the mean squared error between two images, the same as $L _ { I }$ . Then we establish a linear relationship between the logarithm of Neural Uncertainty and PSNR, i.e.

$$
P S N R = A \log \sigma _ { I } ^ { 2 } ,\tag{8}
$$

where A is a constant coefficient.

To verify the linear relationship, we scatter data pairs of $( P S N R , \deg \sigma _ { I } ^ { 2 } )$ for images in the testing set evaluated at different iterations when optimizing $\mathbf { F } _ { \theta }$ for a cabin scene (the scene is shown in Section IV). Two different training strategies are conducted: online training with images captured along a planned trajectory added sequentially and offline batch training using all the images precaptured from the trajectory. As can be seen Fig. 3(c-d), a strong correlation exists between PSNR and log $\sigma ^ { 2 }$ , whose Pearson Correlation Coefficient (PCC) is 0.96 for online and 0.92 for offline. The two variables are almost perfectly negatively linearly related. During the training, $\sigma _ { I }$ and $L _ { I }$ are jointly optimized. The loss curves of the uncertainty part log σ and the ratio part $\frac { L _ { I } } { \sigma _ { I } ^ { 2 } }$ in (5) for online are shown in Fig. 3(b). Notice that the loss curve for the ratio part stays almost constant during training, which also verifies the effectiveness using uncertainty as a proxy of PSNR.

For the verification and the usage of the linear relationship for autonomous reconstruction, we adopt NeRF [1] as our implicit representation for a scene. From the formulation and the derivation of the relation between neural uncertainty and PSNR, our neural uncertainty is agnostic to the underlying function $\mathbf { F } _ { \theta }$ which can be MLPs like NeRF or networks based on trainable feature vectors with MLP Decoder [19]

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/ef9bd793b6a7b6fac88e87487f4f941c55d6289aa16b507e0ba7e9f32fc9ee76.jpg)  
Fig. 3. Loss curves and linear relationship between log $\sigma _ { I } ^ { 2 }$ and PSNR. PCC value of -1 signifies strong negative correlation. (a) Training loss curves for online training and offline training. (b) The loss curves for the uncertainty part log $\sigma _ { I }$ and the ratio part $\begin{array} { r } { \frac { L _ { I } } { \sigma _ { { \phantom { } } _ { I } } ^ { 2 } } } \end{array}$ in (5). (c) Linearity when the scene is optimized using offline images. (d) Linearity when the scene is optimized using online images.

## C. Online Training and Acceleration

Though online reconstruction with Neural Uncertainty supervised by images can achieve similar accuracy with NeRF, the convergence is too slow for view planning. We accelerate training by introducing particle filter, depth supervision and a keyframe strategy.

Particlefilter: keeps particles (rays) active in high loss region, which helps the network optimize details faster. At each step, when a new image is added to an image pool for the training, a set of particles are randomly sampled from the image. After an iteration of training, a quarter of particles are resampled according to the weight of particles, which is defined according to the loss of a ray, and the other particles are uniformly sampled from the image.

Particle filter is applied after coarse learning of the whole scene is done. At the early stage of training, the model knows little information about the scene and tends to have a higher loss for rays shooting at objects than that for empty space. This results in particles always staying at the surfaces ofobjects and therefore the network learns the whole space slower. Considering this, at the beginning iterations of the training, we use random sampling.

Depth supervision: can greatly speed up training. Depth images for NeRF are rendered similar to color images in [1]. We define depth loss and our final loss as

$$
L _ { d e p t h } = \frac { 1 } { R } \sum _ { r = 1 } ^ { R } \| \hat { \boldsymbol { z } } _ { r } - \boldsymbol { z } _ { r } \| _ { 2 } ^ { 2 } , L = L _ { c o l o r } + \lambda _ { d } L _ { d e p t h } ,\tag{9}
$$

where $\hat { z } _ { r }$ and $z _ { r }$ represent the rendered depth from reconstructed 3D model $\mathbf { F } _ { \theta }$ and the ground truth depth for a pixel. As depth captured from real sensors typically has noise, we find using depth supervision may make the model not able to converge well due to the conflict between noisy depth and the depth inferred from multiview RGB images. A balance needs to be made between the two cases. We strengthen depth supervision at the early stage of training to accelerate training and decrease it after getting a coarse 3D structure. The weight of depth loss is decreased from 1 to 1/10 after $N _ { d }$ iterations to emphasize structure from multiview images.

Keyframe pool: We follow iMAP [2] to maintain a keyframe pool containing 4 images for continual training. The pool is initialized with the first four views and during training, the image with minimum image loss in the pool is replaced with a new image or an image from a seen images set.

## D. View Planning with Neural Uncertainty

As explained in Section III-A, we follow the greedy strategy. For the view path planning, we exploit the scenic path planner based on $\mathrm { R R T s ^ { \ast } }$ [3] to evaluate the efficacy of the proposed Neural Uncertainty in guiding the view planning. In the scenic path planner, the $\mathrm { R R } \bar { \mathrm { T s } } ^ { \ast }$ algorithm samples from a prior distribution for the view quality cost-space approximated by Sequential Monte-Carlo (SMC) instead from $S E ( 3 )$ , which baises the growth of the tree towards areas with good NBV cost while also aims at a shortest Euclidean path. Our view planner replaces their view cost with

$$
C _ { v i e w } = \sigma _ { I } ^ { 2 } ,\tag{10}
$$

where $\sigma _ { I } ^ { 2 }$ is defined in (4) and here the image pool contains only one image.

For the view planning, we introduce a prior of objects in the center of the space for the candidate viewpoint sampling: the camera moves in a band with the nearest distance to the object $D _ { n e a r }$ meters and the farthest distance $D _ { f a r }$ meters; the camera looks at directions pointing roughly to the center of the object; candidate viewpoints are sampled in a sphere with the center at the current camera position and the radius $R _ { s a m p l \epsilon }$ meters.

## IV. RESULTS

Data: The experiments are conducted on five 3D models, drums from NeRF synthetic dataset, Alexander,<sup>1</sup> cabin, tank and monsters we collect online. The scenes used in the paper are mostly of the size $5 m \times 5 m \times 5 m$ , and the larger scene Alexander is about $8 0 m \times 8 0 m \times 8 0 m . D _ { n e a r } ,$ $D _ { f a r } , \ R _ { s a m p l e }$ are 80 m, 90 m, 80 m for Alexander and 3 m, 4 m, 3 m for other scenes. RGBD images are rendered by Unity Engine and we assume their corresponding camera poses are known. To simulate depth noise, all rendered depth images are added with noise scaling approximately quadratically with depth z [20]. The depth noise model is $\bar { \epsilon } = N ( \mu ( z ) , \sigma ( z ) )$ where $\mu ( z ) = 0 . 0 0 0 1 \bar { 1 } 2 5 z ^ { 2 } + 0 . 0 0 4 8 8 7 5 .$ $\sigma ( z ) = \mathrm { 0 . 0 0 2 9 2 5 } z ^ { 2 } + 0 . 0 0 3 3 2 5$ and the constant parameters are acquired by fitting the model to the noise reported for Intel Realsense L515.<sup>2</sup> The depth noise model for Alexander of large size, $\mu ( z ) = 0 . 0 0 0 0 \bar { 1 } 2 3 5 z ^ { 2 } + 0 . 0 0 0 0 4 6 5 1$ $\sigma ( z ) =$ $0 . 0 0 0 0 1 2 2 8 z ^ { 2 } + 0 . 0 0 0 0 1 5 7 1$ , the constant parameters acquired by fitting the model to the noise reported for Lidar $\mathrm { V L P l } \bar { 6 } ^ { 3 }$

Implementation details: The networks and the hyperparameters of all the experiments for different scenes below are set to the same value. Most hyper parameters use the default setting in NeRF, including parameters of Adam optimizer (with hyperparameters $\beta _ { 1 } = \bar { 0 . 9 } , \beta _ { 2 } = 0 . 9 9 9 , \epsilon = 1 0 ^ { - 7 } )$ , 64/128 sampling points on a ray for coarse/fine sampling, a batch size of 1024 etc.

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/351e26a718a8e68cdae14e27ec2091f840e2192af6715a39b7841f3f85a1e58d.jpg)  
Fig. 4. The network architecture of the 3D reconstruction module of NeurAR. Similar to NeRF. The only difference is an additional branch of a 128 fully connected layer for the estimation of uncertainty

NeurAR reconstructs scenes and plans the view path simultaneously, running parallel on two RTX3090 GPUs: the 3D reconstruction module, i.e. optimization of NeRF on a GPU; the view planner module and the renderer module on the other one. After a step of the view planner (also 700 iterations for the NeRF optimization), the optimization receives one or two images collected on the view path from cameras. We set the maximum views for planning to be 28 views, i.e. 11 k training iterations and about 15 steps for the optimization during the view planning. In each step, 1 or 2 views are selected along the planned path. The network architecture of our NeurAR is shown in Fig. 4.

Algorithmic Variants: To evaluate the efficacy of the proposed method, we designed baselines and variants of our method. For the implicit scene representation, we adopt TSDF as the baseline as it is oen of the most used representation for SLAM and autonomous reconstruction [4], [9], [21]. For view path planning based on the proposed Neural Uncertainty, we construct two variants: one using a pre-defined circular trajectory with which existing work [21], [22] usually compares and the other one randomly sampling a viewpoint instead of using the view cost to choose NBV at each step.

The variants are 1) TSDF FT: RGBD images and corresponding poses are collected from a Fixed circular Trajectory and are fed into the system sequentially. The voxel resolution of TSDF is 1cm. 2) TSDF RS: replace the fixed trajectory in TSDF FT with Randomly Sampled NBVs. 3) TSDF RRT: As for autonomous scene reconstruction most work is not open source, we re-implement the view cost defined in [9] which adopts TSDF representation. The online trajectory is planned with RRT and the view cost is defined according to the quality of a reconstructed voxel, measured by the number ofrays traversing through it. Images and corresponding poses for the fusion are collected from the planned views and are fed into the system sequentially. 4) Offline FT: The variant is NeurAR with views from the fixed trajectory and trained offline. 5) Online FT: The variant is NeurAR with views from the fixed trajectory. 6) Online RS: NeurAR with randomly sampled NBVs. 7) Online even cover.: NeurAR with views evenly distributed in a dome to cover the whole space. 8) Ours offline: NeurAR trained offline, i.e. trained from scratch with images precaptured from all the planned views from our proposed method.

Metrics: The quality of the reconstructed models can be measured in two aspects: the quality of the rendered images (measure both the geometry and the texture) and the quality of the geometry of the constructed surface. For the former, we evenly distribute about 200 testing views 80m for Alexander, 3m and 3.4m from the center for other scenes, render images for the reconstructed models and evaluate PSNR, SSIM and LPIPS for these images. For the latter, we adopt metrics from iMAP [2]: Accuracy (cm), Completion (cm), CompletionRatio (the percentage of points in the reconstructed mesh with Completion under a threshold, 30 cm for Alexander and 1cm for others). For the geometry metrics, about 300 k points are sampled from the surfaces.

TABLE I  
EVALUATIONS ON THE RECONSTRUCTED 3D MODELS USING DIFFERENT METHODS
<table><tr><td rowspan="2">Method</td><td rowspan="2">PSNR↑</td><td colspan="2">cabin</td><td colspan="2">drums</td><td colspan="2"></td><td colspan="2">alexander</td><td colspan="2">tank</td><td colspan="2"></td><td colspan="2">monsters</td></tr><tr><td>SSIM↑</td><td>LPIPS↓</td><td>PSNR↑</td><td>SSIM↑</td><td>LPIPS↓</td><td>PSNR↑</td><td>SSIM↑</td><td>LPIPS↓</td><td>PSNR↑</td><td>SSIM↑</td><td>LPIPS↓</td><td>PSNR↑</td><td>SSIM↑</td><td>LPIPS↓</td></tr><tr><td>TSDF FT</td><td>21.17</td><td>0.768</td><td>0.140</td><td>18.67</td><td>0.746</td><td>0.150</td><td>19.30</td><td>0.662</td><td>0.190</td><td>20.39</td><td>0.782</td><td>0.151</td><td>20.42</td><td>0.786</td><td>0.126</td></tr><tr><td>TSDF RS</td><td>20.87</td><td>0.75</td><td>0.151</td><td>18.68</td><td>0.738</td><td>0.160</td><td>19.15</td><td>0.649</td><td>0.195</td><td>18.64</td><td>0.749</td><td>0.171</td><td>20.19</td><td>0.777</td><td>0.131</td></tr><tr><td>TSDF RRT [9]</td><td>20.34</td><td>0.739</td><td>0.171</td><td>17.95</td><td>0.716</td><td>0.181</td><td>18.86</td><td>0.644</td><td>0.199</td><td>21.36</td><td>0.772</td><td>0.172</td><td>20.14</td><td>0.773</td><td>0.146</td></tr><tr><td>Offline FT</td><td>24.31</td><td>0.842</td><td>0.108</td><td>21.17</td><td>0.831</td><td>0.117</td><td>8.40</td><td>0.431</td><td>0.606</td><td>21.71</td><td>0.819</td><td>0.142</td><td>25.01</td><td>0.880</td><td>0.076</td></tr><tr><td>Online FT</td><td>23.42</td><td>0.833</td><td>0.114</td><td>21.49</td><td>0.831</td><td>0.117</td><td>8.71</td><td>0.419</td><td>0.597</td><td>23.03</td><td>0.839</td><td>0.124</td><td>24.42</td><td>0.878</td><td>0.077</td></tr><tr><td>Online RS</td><td>26.74</td><td>0.864</td><td>0.082</td><td>23.66</td><td>0.859</td><td>0.0838</td><td>13.08</td><td>0.533</td><td>0.373</td><td>21.97</td><td>0.789</td><td>0.162</td><td>24.68</td><td>0.871</td><td>0.063</td></tr><tr><td>Online even cover.</td><td>26.56</td><td>0.868</td><td>0.106</td><td>24.82</td><td>0.913</td><td>0.049</td><td>24.30</td><td>0.767</td><td>0.182</td><td>24.81</td><td>0.875</td><td>0.108</td><td>25.90</td><td>0.909</td><td>0.065</td></tr><tr><td>Ours Offline</td><td>28.86</td><td>0.917</td><td>0.048</td><td>26.45</td><td>0.916</td><td>0.051</td><td>23.98</td><td>0.758</td><td>0.188</td><td>27.54</td><td>0.909</td><td>0.064</td><td>27.57</td><td>0.927</td><td>0.039</td></tr><tr><td>Ours</td><td>28.35</td><td>0.902</td><td>0.062</td><td>25.73</td><td>0.905</td><td>0.058</td><td>24.07</td><td>0.757</td><td>0.199</td><td>25.83</td><td>0.874</td><td>0.097</td><td>26.57</td><td>0.908</td><td>0.054</td></tr><tr><td>Method</td><td>Acc↓</td><td>Comp↓</td><td>C.R.↑</td><td>Acc↓</td><td>Comp↓</td><td>C.R.↑</td><td>Acc↓</td><td>Comp↓</td><td>C.R.↑</td><td>Acc↓</td><td>Comp↓</td><td>C.R.↑</td><td>Acc.↓</td><td>Comp↓</td><td>C.R.↑</td></tr><tr><td>TSDF FT</td><td>2.47</td><td>2.68</td><td>0.39</td><td>2.51</td><td>1.64</td><td>0.21</td><td>87.16</td><td>164.30</td><td>0.44</td><td>2.46</td><td>3.14</td><td>0.40</td><td>2.44</td><td>1.21</td><td>0.43</td></tr><tr><td>TSDF RS</td><td>2.83</td><td>2.81</td><td>0.42</td><td>3.09</td><td>1.56</td><td>0.22</td><td>107.85</td><td>159.78</td><td>0.44</td><td>3.04</td><td>3.53</td><td>0.40</td><td>2.93</td><td>1.19</td><td>0.48</td></tr><tr><td>TSDF RRT [9]</td><td>2.05</td><td>2.53</td><td>0.44</td><td>2.90</td><td>1.33</td><td>0.25</td><td>100.85</td><td>169.56</td><td>0.43</td><td>2.01</td><td>1.62</td><td>0.48</td><td>2.97</td><td>1.14</td><td>0.48</td></tr><tr><td>Offline FT</td><td>1.09</td><td>1.02</td><td>0.70</td><td>1.77</td><td>1.21</td><td>0.66</td><td>1582.74</td><td>193.77</td><td>0.04</td><td>1.38</td><td>1.28</td><td>0.63</td><td>1.10</td><td>1.10</td><td>0.71</td></tr><tr><td>Online FT</td><td>1.19</td><td>1.06</td><td>0.67</td><td>1.79</td><td>1.22</td><td>0.65</td><td>1506.59</td><td>202.52</td><td>0.05</td><td>1.37</td><td>1.30</td><td>0.62</td><td>1.19</td><td>1.09</td><td>0.71</td></tr><tr><td>Online RS</td><td>1.88</td><td>1.20</td><td>0.57</td><td>1.53</td><td>1.10</td><td>0.70</td><td>652.66</td><td>211.00</td><td>0.27</td><td>2.62</td><td>2.25</td><td>0.37</td><td>1.89</td><td>1.11</td><td>0.66</td></tr><tr><td>Online even cover.</td><td>1.21</td><td>1.01</td><td>0.74</td><td>1.15</td><td>1.08</td><td>0.77</td><td>54.24</td><td>35.69</td><td>0.48</td><td>1.50</td><td>1.29</td><td>0.64</td><td>1.17</td><td>1.06</td><td>0.76</td></tr><tr><td>Ours Offline</td><td>0.96</td><td>0.93</td><td>0.76</td><td>1.31</td><td>1.07</td><td>0.74</td><td>31.66</td><td>21.90</td><td>0.73</td><td>1.51</td><td>1.19</td><td>0.68</td><td>1.27</td><td>1.01</td><td>0.76</td></tr><tr><td>Ours</td><td>1.07</td><td>0.95</td><td>0.74</td><td>1.29</td><td>1.04</td><td>0.75</td><td>48.48</td><td>34.16</td><td>0.60</td><td>1.37</td><td>1.20</td><td>0.65</td><td>1.43</td><td>1.02</td><td>0.74</td></tr></table>

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/28b1e96df73533785024ecb088e9199499b8dd6998db32e88367cc80e46dd241.jpg)  
Fig. 5. Uncertainty maps from different viewpoints and training iterations. Top: Uncertainty maps for seen viewpoints; Bottom: uncertainty maps for an unseen viewpoint. The overall uncertainty decreases with training while the uncertainty for the unseen viewpoint stays high even when the network converges.

## A. Neural Uncertainty

Fig. 3 has quantitatively verified the correlation between Neural Uncertainty and the image quality. Here, we further demonstrate the correlation with examples. In this experiment, the reconstruction network is optimized by the loss defined in (5) with 73 images collected from cameras placed roughly at one hemisphere. Fig. 5 shows the images rendered from different viewpoints for the reconstructed 3D models and their corresponding uncertainty map during training. The uncertainty map is rendered from an uncertainty field where the value of each point in the field is $\scriptstyle { \frac { 1 } { l o g \sigma } }$ . Viewpoints of Row 1 are in the hemisphere seen in the training and the viewpoint of the last row is in the other hemisphere. At the beginning, for all viewpoints, the object region and the empty space both have high uncertainty. For the seen viewpoints, with training continuing, the uncertainty of the whole space decreases and the quality of the rendered images improve. When the network converges, the uncertainty only remains high in the local areas having complicated geometries. For the unseen viewpoint, though the uncertainty for the empty space decreases dramatically with training, the uncertainty is still very high on the whole surface of the object even when the network converges.

Fig. 1 shows the Neural Uncertainty can successfully guide the planner to planning view paths for cameras to look at regions that are not well reconstructed. For example, given current pose 15 and $\mathbf { F } _ { \theta } .$ , a path is planned toward viewpoint 16 having a higher uncertainty map. The rendered image from viewpoint 16 from $\mathbf { F } _ { \theta }$ exhibits poor quality (zoom in the image and the uncertainty map for details under the roof).

## B. View Planning With Neural Uncertainty

To show the efficacy of our proposed method, we compare metrics in Table I and the rendered images of reconstructed 3D models using different methods in Fig. 6. Table I demonstrates except for the even coverage, our method outperforms all variants on all metrics significantly. Our proposed NeurAR achieves better reconstruction results with shorter view paths compared with other variants and existing work.

Compared with methods using the implicit representation without path planning (Offline FT, Online FT, Online RS), our method demonstrates significant improvements in the image quality and geometry accuracy. Methods without the planned views 1) are even unable to converge to a reasonable result in the large scene (NeRF will fail due to overfitting without carefully planned input viewpoint coverage), 2) have many holes in the objects as these regions are unseen in the images and 3) inferior image details on the surface of objects as these regions have a little overlap of different views, making inferring the 3D geometry hard (check red box in Fig. 6 for visual comparison). In addition, these variants tend to have ghost effects in the empty space. This is largely because the viewpoints are designed to make the camera look at the objects and the empty space is not considered. Notice that placing viewpoints covering the whole space without the aid of visualization tools as feedback is non-trivial for humans.

Assuming we have the 3D shape of a scene with the target object in the center (which is our goal), to cover the whole space, we distribute viewpoints evenly at a dome centered at the scene center with a radius of 80 meters for Alexander and 3 meters for others. All viewpoints look at the center. An even coverage path can provide a good reference as even coverage for many scenes of simple shape and texture is an optimum solution. Metrics for Online even cover. and Ours in Table I demonstrate that our planned views can achieve similar or even better reconstruction results and our method has no prior or only a minor knowledge about the scene.

![](images/2023_NeurAR__Neural_Uncertainty_for_Autonomous_3D_Reconstruct/9b2be74a1da086c747d69c3105e874f478e54d17f43407d62e51963dd30569f9.jpg)  
Fig. 6. Comparison of the reconstruction models with different methods. Refer to the supplementary video for higher resolution, more comparison and more viewpoints.

Ours offline: in most scenes gives better results than NeurAR trained online as using all views enforces multiview constraints at the start of the training.

In addition to the lower mean PSNR shown in the Table I, our method achieves much smaller variance regarding the PSNR of the rendered images from different viewpoints. For example, the PSNR variance of the rendered images for reconstructed cabins using Online FT, Online RS and our method is 32.55dB, 15.44dB to 4.78dB, indicating our method more even. Further, for the average path length in the smaller scenes, our NeurAR traverses 43.27m while Online RS about 70.24m; in the larger scene, our NeurAR traverses 907.60m while Online RS about 1329.75m.

For models using the reconstruction with TSDF (TSDF FT, TSDF RS, TSDF RRT), we render images via volume rendering. From the images in the last three columns in Fig. 6, the surfaces exhibit many holes; in addition, the surfaces of the reconstructed models are rugged due to the noise in the depth images. Though NeurAR uses depth too, it depends on the depth images to accelerate convergence only at the early stage of training and decreases its effect after coarse structures have been learned. The finer geometry is acquired by multiview image supervision. NeurAR fills holes in TSDF RRT and provide finer details. It outperforms TSDF RRT in the image quality, geometry quality and path length (43.27m vs 57.39m). Though post-processing can be applied to extra finer meshes for the reconstructions using TSDF and get smoother images than our rendering images from TSDF directly, denoising and filling the holes are non-trivial tasks, particularly in scenes having complex structures.

## C. Ablation Study

Training Iterations between Steps: The number of iterations for NeRF optimization iter allowed for planning a view step affects final results. We choose different iterations to run our NeurAR system in the scene cabin and compare PSNR of the rendered images of the reconstructed models. PSNR for the models optimized using 300, 700, 1400 iterations is 26.16, 28.58, 26.91. This is because too few iterations may lead to uncertainty not being optimized well while too many steps may lead to overfitting to images added.

TABLE II  
INFLUENCE OF DIFFERENT NOISE MAGNITUDE
<table><tr><td>Method</td><td>PSNR↑</td><td>SSIM↑</td><td>LPIPS↓</td><td>Acc↓</td><td>Comp↓</td><td>C.R.↑</td></tr><tr><td>Ours (no noise)</td><td>28.71</td><td>0.926</td><td>0.043</td><td>0.92</td><td>0.90</td><td>0.75</td></tr><tr><td>Ours (with noise)</td><td>28.35</td><td>0.902</td><td>0.062</td><td>1.07</td><td>0.95</td><td>0.74</td></tr><tr><td>Ours (noise x2)</td><td>28.19</td><td>0.903</td><td>0.062</td><td>0.99</td><td>0.97</td><td>0.74</td></tr><tr><td>Ours (noise x3)</td><td>27.65</td><td>0.900</td><td>0.063</td><td>1.00</td><td>0.94</td><td>0.77</td></tr></table>

TABLE III

EVALUATIONS ON THE RECONSTRUCTED 3D MODELS USING DIFFERENT NUMBER OF VIEWS
<table><tr><td>Views</td><td>PSNR↑</td><td>SSIM↑</td><td>LPIPS↓</td><td>Acc↓</td><td>Comp↓</td><td>C.R.↑</td></tr><tr><td>18</td><td>22.47</td><td>0.802</td><td>0.155</td><td>2.18</td><td>1.31</td><td>0.59</td></tr><tr><td>28</td><td>28.35</td><td>0.902</td><td>0.062</td><td>1.07</td><td>0.95</td><td>0.74</td></tr><tr><td>38</td><td>30.03</td><td>0.919</td><td>0.053</td><td>0.90</td><td>0.86</td><td>0.77</td></tr><tr><td>58</td><td>29.39</td><td>0.926</td><td>0.050</td><td>0.82</td><td>0.90</td><td>0.79</td></tr></table>

Depth noise: We construct variants of NeurAR using different noise magnitude from no noise, the noise magnitude of L515, to the double $( 2 \times \mu ( z ) , 2 \times \sigma ( z ) )$ ) and the triple $( 3 \times \mu ( z )$ $3 \times \sigma ( z ) ,$ ) noise magnitude of L515. Table II shows the metrics measuring the cabin models reconstructed by these variants: NeurAR can be accelerated with very noisy depth at a cost of only a minor drop in the reconstruction quality. The robustness verifies the effect of dampening the depth supervision during the training in Section III-C.

Number of views: To further study the influence, we set the maximum allowed views from 18 to 58 for the cabin scene and provide the PSNR for the reconstructed models under different view settings.

## V. CONCLUSION

In this letter, we have presented an autonomous 3D reconstruction method based on an implicit neural representation, with the view path planned according to a proxy of PSNR. The proxy is acquired by learning uncertainty for the estimation of the color of a spatial point on a ray during the reconstruction. A strong correlation is discovered between the uncertainty and PSNR, which is verified by extensive experiments. Compared with variants with a pre-defined trajectory and variants using TSDF, our method demonstrates significant improvements. Our work shows a promising potential of implicit neural representations for high-quality autonomous 3D reconstruction.

One limitation of NeurAR is the optimization of implicit neural representation is slow and the computation consumption is high. The optimization of the model between view steps takes about 50-120 seconds depending on the number of iterations, which is much slower than existing unknown area explorations. However, our method is agnostic to the underlying NeRF realization and therefore, we can easily swap the module for more advanced NeRF variants emerging lately [23]. Future work includes achieving acceleration of the training and rendering for NeurAR. The other limitation is that we assume known camera poses, which require extra location systems to make NeurAR work. Future work includes joint optimization of the camera poses, reconstruction, and view planning. Also, extending NeurAR to multiple agents for large scene reconstructions with distributed learning [24] and adverse environments with the mmWave radar [25] are interesting directions.

## REFERENCES

[1] B. Mildenhall, P. P. Srinivasan, M. Tancik, J. T. Barron, R. Ramamoorthi, and R. Ng, “NeRF: Representing scenes as neural radiance fields for view synthesis,” in Proc. Eur. Conf. Comput. Vis., Springer, 2020, pp. 405–421.

[2] E. Sucar, S. Liu, J. Ortiz, and A. J. Davison, “iMAP: Implicit mapping and positioning in real-time,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 6229–6238.

[3] O. Mendez, S. Hadfield, N. Pugeault, and R. Bowden, “Taking the scenic route to 3D: Optimising reconstruction from moving cameras,” in Proc. IEEE Int. Conf. Comput. Vis., 2017, pp. 4677–4685.

[4] S. Isler, R. Sabzevari, J. Delmerico, and D. Scaramuzza, “An information gain formulation for active volumetric 3D reconstruction,” in Proc. IEEE Int. Conf. Robot. Automat., 2016, pp. 3477–3484.

[5] S. Wu, “Quality-driven poisson-guided autoscanning,” ACM Trans. Graph., vol. 33, no. 6, pp. 1–12, 2014.

[6] S. Song and S. Jo, “Surface-based exploration for autonomous 3D modeling,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 4319–4326.

[7] C. M. Bishop, Pattern Recognition and Machine Learning. New York, NY, USA: Springer-Verlag, 2006.

[8] Q. Ye and T.-K. Kim, “Occlusion-aware hand pose estimation using hierarchical mixture density network,” in Proc. Eur. Conf. Comput. Vis., 2018, pp. 801–817.

[9] L. Schmid, M. Pantic, R. Khanna, L. Ott, R. Siegwart, and J. Nieto, “An efficient sampling-based method for online informative path planning in unknown environments,” IEEE Robot. Automat. Lett., vol. 5, no. 2, pp. 1500–1507, Apr. 2020.

[10] J. I. Vasquez-Gomez, L. E. Sucar, and R. Murrieta-Cid, “View/state planning for three-dimensional object reconstruction under uncertainty,” Auton. Robots, vol. 41, no. 1, pp. 89–109, 2017.

[11] M. D. Kaba, M. G. Uzunbas, and S. N. Lim, “A reinforcement learning approach to the view planning problem,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2017, pp. 6933–6941.

[12] L. Schmid, C. Ni, Y. Zhong, R. Siegwart, and O. Andersson, “Fast and compute-efficient sampling-based local exploration planning via distribution learning,” IEEE Robot. Autom. Lett., vol. 7, no. 3, pp. 7810–7817, 2022.

[13] B. Hepp, D. Dey, S. N. Sinha, A. Kapoor, N. Joshi, and O. Hilliges, “Learnto-score: Efficient 3D scene exploration by predicting view utility,” in Proc. Eur. Conf. Comput. Vis., 2018, pp. 437–452.

[14] J. L. Schönberger, E. Zheng, J.-M. Frahm, and M. Pollefeys, “Pixelwise view selection for unstructured multi-view stereo,” in Proc. Eur. Conf. Comput. Vis., Cham, Switzerland„ 2016, pp. 501–518.

[15] S. Izadi et al., “Kinectfusion: Real-time 3D reconstruction and interaction using a moving depth camera,” in Proc. 24th Annu. ACM Symp. User Interface Softw. Technol., 2011, pp. 559–568.

[16] A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard, “Octomap: An efficient probabilistic 3D mapping framework based on octrees,” Auton. Robots, vol. 34, no. 3, pp. 189–206, 2013.

[17] “Nerf project page,” 2020. [Online].Available:https://www. matthewtancik.com/nerf

[18] A. Gropp, L. Yariv, N. Haim, M. Atzmon, and Y. Lipman, “Implicit geometric regularization for learning shapes,” in Proc. 37th Int. Conf. Mach. Learn., 2020, pp. 3789–3799.

[19] X. Zhang, S. Bi, K. Sunkavalli, H. Su, and Z. Xu, “Nerfusion: Fusing radiance fields for large-scale scene reconstruction,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 5449–5458.

[20] C. V. Nguyen, S. Izadi, and D. Lovell, “Modeling kinect sensor noise for improved 3d reconstruction and tracking,” in Proc. IEEE 2nd Int. Conf. 3D Imag., Model., Process., Visual. Transmiss., 2012, pp. 524–530.

[21] S. Song, D. Kim, and S. Choi, “View path planning via online multiview stereo for 3-D modeling of large-scale structures,” IEEE Trans. Robot., vol. 38, no. 1, pp. 372–390, Feb. 2022.

[22] D. Peralta, J. Casimiro, A. M. Nilles, J. A. Aguilar, R. Atienza, and R. Cajote, “Next-best view policy for 3D reconstruction,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 558–573.

[23] T. Müller, A. Evans, C. Schied, and A. Keller, “Instant neural graphics primitives with a multiresolution hash encoding,” ACM Trans. Graph., vol. 41, no. 4, pp. 1–15, 2022.

[24] Y. Huang, Y. Sun, Z. Zhu, C. Yan, and J. Xu, “Tackling data heterogeneity: A new unified framework for decentralized SGD with sample-induced topology,” in Proc. Int. Conf. Mach. Learn., 2022, pp. 9310–9345.

[25] A. Chen, X. Wang, S. Zhu, Y. Li, J. Chen, and Q. Ye, “mmBody benchmark: 3D body reconstruction dataset and analysis for millimeter wave radar,” in Proc. 30th ACM Int. Conf. Multimedia, 2022, pp. 3501–3510.