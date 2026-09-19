# DynaSLAM II: Tightly-Coupled Multi-Object Tracking and SLAM

Berta Bescos , Carlos Campos, Juan D. Tardós , and José Neira

Abstract—The assumption of scene rigidity is common in visual SLAM algorithms. However, it limits their applicability in populated real-world environments. Furthermore, most scenarios including autonomous driving, multi-robot collaboration and augmented/virtual reality, require explicit motion information of the surroundings to help with decision making and scene understanding. We present in this paper DynaSLAM II, a visual SLAM system for stereo and RGB-D camera configurations that tightly integrates the multi-object tracking capability. DynaSLAM II makes use ofinstance semantic segmentation and ORB features to track dynamic objects. The structures of the static scene and the dynamic objects are optimized jointly with the trajectories of both the camera and the moving agents within a novel bundle adjustment proposal. The 3D bounding boxes of the objects are also estimated and loosely optimized within a fixed temporal window. We demonstrate that tracking dynamic objects does not only provide rich clues for scene understanding but can be also beneficial for camera tracking.

Index Terms—Dynamic objects, SLAM, semantics, tracking.

## I. INTRODUCTION

V <sup>ISUAL</sup> <sup>Simultaneous</sup> <sup>Localization</sup> <sup>and</sup> <sup>Mapping</sup> <sup>(SLAM)</sup> is the problem of creating a map of an unknown environment and estimating the robot pose within such map, only from the data streams of its on-board cameras. Most SLAM approaches assume a static scene and can only handle small fractions of dynamic content by labeling them as outliers to such static model [1]–[3]. Whilst the static premise holds for some robotic applications, it limits its use in populated situations for autonomous driving, service robots or AR/VR.

The problem of dealing with dynamic objects in SLAM has been widely targeted in recent years. The biggest part of the literature tackles this problem by detecting moving regions within the observed scene and rejecting such areas for the SLAM problem [4]–[7]. Some works process the image streams outside of the localization pipeline by translating the images that show dynamic content into realistic images with only static content [8]–[10]. On the other hand, a small but growing part of the robotics community has addressed this

![](images/2021_DynaSLAM-II/d957c8e6de4cb4fdded325cc5a8533528f6261ff32a968f49df830c4c21cf660.jpg)  
(a) The 3D bounding box and the speed of the objects are inferred in the image. Static and dynamic key points are in green and red respectively.

![](images/2021_DynaSLAM-II/922dea78b4dffcb64bda5ef715cb20472a4dc29ae263fac3e35c738d8a0c8d2c.jpg)  
(b) Joint estimation of the camera ego motion (green car), the sparse static 3D map (black points) and the trajectories of the dynamic objects. The cyan key frames allow to optimize the map dynamic structure, whereas the blue ones only optimize the camera pose and the static structure.

Fig. 1. Qualitative results with the KITTI tracking dataset.

issue by incorporating the dynamics of moving objects into the problem [11]–[14]. Whereas the two first groups mostly focus on achieving an accurate ego-motion estimation from the static scene, the objective of the last group is twofold: they do not only solve the SLAM problem but also provide information about the poses and trajectories of other dynamic agents .

Understanding surrounding dynamic objects is of crucial importance for the frontier requirements of emerging applications within AR/VR or autonomous navigation. Whereas it is tolerable to rule out minor movements in quasi-static environments, most scenarios including autonomous driving, multi-robot collaboration and AR/VR require explicit motion information of the surroundings to aid in decision-making and scene understanding. For example, in VR, dynamic objects need to be explicitly tracked to allow the interaction of virtual objects with real-world moving instances. In autonomous driving scenarios, a car must not only localize itself but must also reliably perceive other vehicles and passers-by to avoid collisions.

The vast majority of the literature that specifically addresses this issue detect moving objects and track them separately from the SLAM formulation by using traditional multi-target tracking approaches [15]–[19]. Their accuracy highly depends on the camera pose estimation, which is susceptible to failure in complex dynamic environments where the presence of reliable static structure is not guaranteed. In recent years, the robotics community has made its first steps towards addressing object tracking jointly with SLAM, adding an extra layer of complexity to the problem. These systems are often tailored for special use cases and several priors are exploited to constraint the space solutions: planar object movement in driving scenarios [12], or even the use of object 3D models [20].

At this point, we take the opportunity to introduce DynaSLAM II. An example of the output of our system can be seen in Fig. 1. DynaSLAM II is an open-source RGB-D and stereo SLAM system for dynamic scenes which simultaneously estimates the poses of the camera, the map and the trajectories of the scene moving objects with the below contributions:

\- A matching approach for dynamic-object feature that is guided by a higher-lever 2D instance matching.

\- A cost-efficient bundle adjustment solution with new measurements between cameras, points and dynamic objects.

\- A decoupled optimization for bounding boxes to find out a common reference across objects of the same class.

\- Our experiments demonstrate that camera motion estimation and multi-object tracking can be mutually beneficial.

## II. RELATED WORK

## A. Loosely-Coupled Multi-Object Tracking and SLAM

The traditional manner ofaddressing 3D multi-object tracking implies detecting and tracking the moving objects separately from the SLAM formulation [15]–[19]. Among them, Wang et al. [15] derived the Bayes formula of the SLAM with tracking of moving objects and provided a solid basis for understanding and solving this problem. Wangsiripitak et al. [16] proposed the parallel implementation of SLAM with a 3D object tracker: the SLAM provides the tracker with information to register map objects, and the tracker allows to mark features on objects. Rogers et al. [17] applied an EM technique to a graph based SLAM approach and allowed landmarks to be dynamic. More recently, Barsan et al. [18] presented a stereo-based dense mapping algorithm for urban environments that simultaneously reconstructs the static background and the moving objects. There is a new work by Rosinol et al. [19] that reconciles visual-inertial SLAM and dense mesh tracking, focusing mostly on humans, that shows impressive results in simulation. The main drawback of these approaches is that their accuracy is highly correlated with that of the camera pose estimation. That is, if the camera pose estimation fails, which is quite likely in complex dynamic environments, multi-object tracking also fails directly.

The idea of simultaneously estimating camera motion and multiple moving objects motion originated from the SLAM-MOT work [21]. They established a mathematical framework to integrate a filtering-based SLAM and moving object tracking demonstrating that it satisfied navigation and safety requirements in autonomous driving. Later on, works using RGB-D cameras followed up this idea to densely reconstruct static indoors scenes along with moving objects using pixel-wise instance segmentation, showing impressive results [22]–[24]. Since it is of crucial importance for dense approaches to obtain accurate segmentation, Mask-Fusion [23] and MID-Fusion [24] refine it by assuming that human-made objects are convex.

## B. Tightly-Coupled Multi-Object Tracking and SLAM

Among the feature-based approaches, as is ours, few aim to merge information from static and dynamic objects into a single framework to boost estimation accuracy. Henein et al. [25] were among the first ones to tightly combine the problems of tracking dynamic objects and the camera ego motion. However, they only reported experiments on synthetic data showing limited real results. Li et al. [11] use a CNN trained in an end-toend manner to estimate the 3D pose and dimensions of cars, which is further refined together with camera poses. The use of data-driven approaches often provides excellent accuracy in 6 DoF object pose estimation, but also a loss of generality and thus, they can only track cars. Huge amounts of data would be required to track generic objects with their approach. The authors of CubeSLAM [12] showed impressive results with only a monocular camera by making use of a 3D bounding box proposal generation based on 2D bounding boxes and vanishing points. They assume that objects have a constant velocity within a hard-coded duration time interval and exploit object priors such as car sizes, road structure and planar non-holonomic object wheel motion models. Moreover, they only track objects whose 3D bounding box is observable, i.e., only once two or more faces of the cuboid-shape object are seen. On the other hand Cluster-SLAM [26] proposes a SLAM back end with no scene priors to discover individual rigid bodies and compute their motions in dynamic environments. Since it acts as a back end instead of as a full system, its performance relies heavily on the landmark tracking and association quality. The same authors recently developed the full system ClusterVO [13], which models the object points with a probability of object belonging to deal with segmentation inaccuracies. Given that they assume no priors, they obtain good tracking results in indoor and outdoor scenes, but with an inaccurate estimation of the 3D bounding boxes. VDO-SLAM [14] is a recent work that uses dense optical flow to maximise the number of tracked points on moving objects. They implement a bundle adjustment with cameras, objects and points that gives good results but is computationally complex.

## C. A Common Reference for Each Object Class

Some approaches in the current literature do consider the tracking of dynamic objects to be complete with the tracking of dynamic feature points. Examples of these works are ClusterSLAM [26] and VDO-SLAM [14]. However, we believe that it is also of key importance to find a common spatial reference for objects of the same semantic class, as well as an estimate of their dimensions and space occupancy.

Alternatively, the basis of CubeSLAM [12] and of the work by Li et al. [11] is the discovery of object 3D bounding boxes. Only once bounding boxes are discovered, are objects tracked along frames. That is, if the camera viewing angle does not allow to estimate an object bounding box (partial view), the object tracking does not take place. Whereas this is not a problem for Li et al. [11] because CNNs are by nature robust to partial views of objects, CubeSLAM struggles to initialize bounding boxes from views of occluded objects.

In light of these advances, it is apparent that the featurebased SLAM community is searching for the best optimization formulation to combine cameras, objects and structure points. In our proposal, we use a tightly-coupled bundle adjustment formulation with new measurements between cameras, objects and points giving special attention to its computationally complexity and number of parameters involved without introducing hard-coded priors. For this, we integrate instance semantic priors together with sparse image features. This formulation allows the estimation of both the camera, the map structure and the dynamic objects to be mutually beneficial at a low computational cost. On the other hand, part of the current literature focuses on the estimation of the point cloud structure of dynamic objects and of the trajectory of a random object reference [13], [14], [26], whereas another part of the literature seeks to find a common reference for objects of the same class as well as a more informative occupancy volume [11], [12]. We intend to carry out these two tasks independently in order to leverage the benefits of both and not suffer their disadvantages.

## III. METHOD

DynaSLAM II builds on the popular ORB-SLAM2 [1]. It takes synchronized and calibrated stereo/RGB-D images as input, and outputs the camera and the dynamic-object poses for each frame, as well as a spatial/temporal map containing the dynamic objects. For each incoming frame, pixel-wise semantic segmentation is computed and ORB features [27] are extracted and matched across stereo image pairs. We first associate the static and dynamic features with the ones from the previous frame and the map assuming a constant velocity motion for both the camera and the observed objects. Object instances are then matched based on the dynamic feature correspondences. The static matches are used to estimate the initial camera pose, and the dynamic ones yield the object’s SE(3) transform. Finally, the camera and objects trajectories, as well as the objects bounding boxes and 3D points are optimized over a sliding window with marginalization and a soft smooth motion prior. The different contributions and building blocks ofDynaSLAM II are explained in the following subsections.

## A. Notation

We would like to point out that there are two types of dynamic objects in terms of their nature and influence on visual SLAM:

1) The former type consists of objects that inherently possess the capability to move (people, animals and vehicles). We name these objects apriori dynamic or movable. While the SLAM sensor observes them, they either move or remain static.

2) The latter type consists though of objects that are moving while the visual SLAM sensor observes them regardless of their semantic class. We name them moving objects.

Regarding the scene geometry we will use the following notation: a stereo/RGB-D camera i has a pose $\mathbf { T } _ { \mathbb { C } W } ^ { i } \in \mathrm { S E } ( 3 )$ in the world coordinates W at time i (see Fig. 2). The camera i observes 1) static 3D map points $\mathbf { x } _ { \mathbb { W } } ^ { l } \in \mathbb { R } ^ { \breve { 3 } }$ and 2) dynamic objects with pose $\mathbf { T } _ { \mathsf { W } 0 } ^ { k , i } \in \mathrm { S E } ( 3 )$ and linear and angular velocity $\mathbf { v } _ { i } ^ { k } , \mathbf { w } _ { i } ^ { k } \in \mathbb { R } ^ { 3 }$ at time i, all in object coordinates. Each observed object k contains dynamic-object points $\mathbf { x } _ { 0 } ^ { j , k } \in \mathbb { R } ^ { 3 }$

## B. Object Data Association

For each incoming frame the below procedure is followed:

1) Pixel-wise semantic segmentation is computed and ORB features [27] are extracted and matched across stereo pairs. We hypothesize that dynamic features are those belonging to a priori dynamic instances, regardless of their motion.

2) We first associate the static features with the ones from the previous frame and the map to initially estimate the camera pose, following the ORB-SLAM implementation.

![](images/2021_DynaSLAM-II/ecd5c4cb613d438d5dac84e05e3cf52b7e9e90d01f4acb3888965366c9c59b4b.jpg)  
Fig. 2. Notation used to model the dynamic structure. The cameras i and i + 1 observe the dynamic object k (- - -) and the static structure $\left( -- \right)$ . The objects with poses $\mathbf { T } _ { \mathbb { W } 0 } ^ { k , i }$ and $\dot { \mathbf { T } } _ { \mathbb { W } 0 } ^ { k , i + 1 }$ are the same moving body at consecutive observations (– – –).

3) A parallel instance-to-instance matching between consecutive frames is built with the Munkres algorithm [28] using the 2D bounding boxes Intersection over Union as cost.

4) Next, dynamic features are associated with the dynamic points from the local map in two different ways:

a) if the map objects velocity is known, the matches are searched by reprojection assuming an inter-frame constant velocity motion. The instance matching results are used to discover outliers.

b) if the objects velocity is not initialized or not enough matches are found following (a), we constrain the brute force matching to those features belonging to the most overlapping instance in consecutive frames.

5) A higher level association between instances and objects is also required. If most of the new instance key points are matched with points belonging to one same map object, the instance is attributed the same object track id.

6) If an instance corresponding to an a priori dynamic class contains more than seven unobserved key points whose stereo 3D projection is close to the camera (less than 55 times the stereo baseline), a new object instance is created. Key points are then assigned to the corresponding object.

Note that our framework handles occlusions that last less than two seconds. This threshold can be extended though if needed: our framework handles occlusions if the object velocity remains constant or almost constant. Because of this feature, extending the threshold might not always lead to better results if the velocity of the tracked object changes in this window.

The SE(3) pose of the first object of a track is initialized with the center of mass of the 3D points and with the identity rotation. To predict the poses of further objects from a track, we use a constant velocity motion model and refine the object pose estimate by minimizing the matches reprojection error.

The reprojection error formulation in multi-view geometry problems for a camera i with pose $\mathbf { T } _ { \mathtt { C W } } ^ { i } \in \mathrm { S E } ( 3 )$ and a 3D map point l with homogeneous coordinates $\bar { \bf x } _ { \scriptscriptstyle \mathrm { W } } ^ { l } \in \mathbb { R } ^ { 4 }$ in reference W with a stereo key point correspondence $\ddot { \bf u } _ { i } ^ { l } = [ u , v , u _ { R } ] \in \mathbb { R } ^ { 3 }$ is

$$
{ \bf e } _ { \bf r e p r } ^ { i , l } = { \bf u } _ { i } ^ { l } - \pi _ { i } ( { \bf T } _ { \mathbb { C } \mathbb { W } } ^ { i } \bar { \bf x } _ { \mathbb { W } } ^ { l } ) ,\tag{1}
$$

where $\pi _ { i }$ is the reprojection function for a rectified stereo/RGB-D camera that projects a 3D homogeneous point in the camera coordinates into the camera frame pixel. Unlike this formulation, which is valid for static representations, we propose to restate

![](images/2021_DynaSLAM-II/10a25b3feb39b589d03e17c01ad90fb1e7c9ab440cc92c26316488ef536230a1.jpg)  
Fig. 3. Relationship between the number of required parameters when objects are used and when object points are tracked independently (No objects).

the reprojection error as

$$
{ \bf e } _ { { \bf r e p r } } ^ { i , j , k } = { \bf u } _ { i } ^ { j } - \pi _ { i } ( { \bf T } _ { \mathbb { C } _ { W } } ^ { i } { \bf T } _ { \mathbb { W } 0 } ^ { k , i } \bar { \bf x } _ { 0 } ^ { j , k } ) ,\tag{2}
$$

where $\mathbf { T } _ { \mathbb { W } 0 } ^ { k , i } \in \mathrm { S E } ( 3 )$ is the inverse pose of the object k in the world coordinates when the camera i is observing it, and $\bar { \mathbf { x } } _ { 0 } ^ { j , k } \in$ $\mathbb { R } ^ { 4 }$ represents the 3D homogeneous coordinates of the point j in its object reference k with observation in the camera $\mathbf { u } _ { i } ^ { j } \in \mathbb { R } ^ { 3 }$ This formulation enables us to optimize either jointly the poses of the cameras and of the different moving objects, as well as the positions of their 3D points.

## C. Object-Centric Representation

Given the extra complexity and mainly the extra number of parameters that the task of tracking moving objects implies on top of the SLAM ones, it is of high importance to keep this number as reduced as possible to maintain a real-time performance. Modeling dynamic points as repeated 3D points by forming independent point clouds as in usual dynamic SLAM implementations results in a prohibitive amount of parameters. Given a set of $N _ { c }$ cameras, $N _ { o }$ dynamic objects with $N _ { o p }$ 3D points each observed in all cameras, the number of parameters needed to track dynamic objects becomes $N = 6 N _ { c } + N _ { c } \times N _ { o } \times 3 N _ { o p }$ as opposed to $N = 6 N _ { c } + N _ { o } \times 3 N _ { o p }$ in conventional static SLAM representations. This number of parameters becomes prohibitive for long –and not so long– operations and deployment. If the concept of objects is introduced, 3D object points become unique and can be referred to their dynamic object. Therefore it is the pose of the object that is modelled along time and the number of required parameters shifts to $N ^ { \prime } =$ $6 N _ { c } + N _ { c } \times 6 N _ { o } + N _ { o } \times 3 \dot { N } _ { o p }$ . Fig. 3 shows the parameter compression ratio defined as $\frac { N ^ { \prime } } { N }$ for 10 objects. This modelling of dynamic objects and points brings great savings in the number of utilized parameters.

## D. Bundle Adjustment With Objects

Bundle Adjustment (BA) is known to provide accurate estimates of camera poses and sparse geometrical reconstruction, given a strong network of matches and good initial guesses. We hypothesize that BA might bring similar benefits if object poses are also jointly optimized (Fig. 4). Static map point 3D locations $\bar { \mathbf { X } } _ { \mathbb { W } } ^ { l }$ and camera poses $\mathbf { T } _ { \mathrm { C W } } ^ { i }$ are optimized by minimizing the reprojection error with respect to the matched key points $\mathbf { u } _ { i } ^ { \bar { l } }$ (Eqn. 1). Similarly, for dynamic representations, object points $\bar { \mathbf { x } } _ { 0 } ^ { j , k }$ , camera poses $\mathbf { T } _ { \mathrm { C W } } ^ { i }$ and object poses $\mathbf { T } _ { \mathbb { W } 0 } ^ { k , i }$ can be refined by minimizing the reprojection error formulation in (2).

In our implementation, a key frame can be inserted in the map for two different reasons: a) the camera tracking is weak, b) the tracking of any scene object is weak. The reasons for the former are the same ones than in ORB-SLAM. The latter though happens if a dynamic instance with a relatively large amount of features has few points tracked in the current frame. The following optimization scenarios can then appear:

![](images/2021_DynaSLAM-II/f1221b25cdc8f2086cf95a1f47a0ebd43be22d50c291ebfb33de6c18332f67a8.jpg)  
Fig. 4. BA factor graph representation with dynamic objects.

\- If a key frame is inserted only because the camera tracking is weak, the local BA optimizes the currently processed key frame, all the key frames connected to it in the covisibility graph, and all the map points seen by those key frames, following the implementation of ORB-SLAM.

If a key frame is inserted only because an instance tracking is weak, a new object with new object points is created. This key frame does not introduce new static structure, and if the rest of dynamic objects have a stable tracking, new objects for these tracks are not created. In such case the local BA optimizes the object’s pose, velocity and points, and the camera along a 2 seconds temporal tail.

\- Finally, if a key frame is inserted because both camera and object tracking is weak, camera poses, map structure, object poses, velocities and points are jointly optimized.

To avoid non-physically feasible object dynamics, a smooth trajectory is forced by assuming a constant velocity in consecutive observations. The linear and angular velocity of an object k at observation i are respectively denoted as $\mathbf { \bar { v } } _ { i } ^ { k } \in \mathbb { R } ^ { 3 }$ and ${ \bf w } _ { i } ^ { k } \in \mathbb { R } ^ { 3 }$ . We define the following error term:

$$
\mathbf { e } _ { \mathbf { v c t e } } ^ { i , k } = \left( \mathbf { v } _ { \mathbf { i } + 1 } ^ { \mathbf { k } } - \mathbf { v } _ { \mathbf { i } } ^ { \mathbf { k } } \right)\tag{3}
$$

An additional error term is needed to couple the object velocities and poses with their corresponding 3D points. This term can be seen in (4), where $\Delta \mathbf { T } _ { 0 _ { k } } ^ { i , \dot { i } + 1 }$ is the pose transformation that the object k undergoes in the time interval $\Delta t _ { i , i + 1 }$ between consecutive observations i and i + 1.

$$
\begin{array} { r } { \mathbf { e } _ { \mathbf { v c t e } , \mathbf { X } \mathbf { Y } \mathbf { Z } } ^ { i , j , k } = \left( \mathbf { T } _ { \mathbb { W } } ^ { k , i + 1 } - \mathbf { T } _ { \mathbb { W } } ^ { k , i } \Delta \mathbf { T } _ { \mathbb { 0 } _ { k } } ^ { i , i + 1 } \right) \bar { \mathbf { x } } _ { \mathrm { 0 } } ^ { j , k } } \end{array}\tag{4}
$$

The term $\Delta \mathbf { T } _ { 0 _ { k } } ^ { i , i + 1 }$ is defined from the linear and angular velocity of the object k at time ${ \bf \Xi } _ { ; } ^ { \dag } \left( { { \bf v } _ { i } ^ { k } } \right.$ and $\mathbf { w } _ { i } ^ { k } )$ as in (5), where Exp : $\mathbb { R } ^ { 3 } $ $\mathrm { S O ( 3 ) }$ is the exponential map for SO(3).

$$
\begin{array} { r } { \Delta \mathbf { T } _ { 0 _ { k } } ^ { i , i + 1 } = \left( \begin{array} { c c } { \mathrm { E x p } ( \mathbf { w } _ { i } ^ { k } \Delta t _ { i , i + 1 } ) } & { \mathbf { v } _ { i } ^ { k } \Delta t _ { i , i + 1 } } \\ { \mathbf { 0 } _ { 1 \times 3 } } & { 1 } \end{array} \right) } \end{array}\tag{5}
$$

Finally, the following is our BA problem for a set of cameras in the optimizable local window $\mathcal { C }$ with each camera i observing a set of map points $\mathcal { M P } _ { i }$ and an object set $\mathcal { O } _ { i }$ containing each object k the set of object points $O P _ { k }$

![](images/2021_DynaSLAM-II/b5eb1f9a91f0935bff831f685e9e57d121bbebb96bf8d1d7352e2d37997f4c5e.jpg)  
Fig. 5. Hessian matrix for 5 key frames (KFs), 1 object with 10 object points (OPs) and 10 static map points (MPs).

$$
\begin{array} { r l } & { \displaystyle \underset { \theta } { \operatorname* { m i n } } \sum _ { i \in \mathcal { C } } ( \sum _ { l \in \mathcal { M P } _ { i } } \rho ( \| \mathbf { e } _ { \mathbf { r e p r } } ^ { i , l } \| _ { \Sigma _ { i } ^ { l } } ^ { 2 } ) + \sum _ { k \in \mathcal { O } _ { i } } ( \rho ( \| \mathbf { e } _ { \mathbf { v c t e } } ^ { i , k } \| _ { \Sigma _ { \Delta t } } ^ { 2 } )  } \\ & {  + \sum _ { j \in \mathcal { O P } _ { k } } ( \rho ( \| \mathbf { e } _ { \mathbf { r e p r } } ^ { i , j , k } \| _ { \Sigma _ { i } ^ { j } } ^ { 2 } ) + \rho ( \| \mathbf { e } _ { \mathbf { v c t e } , \mathbf { X Y Z } } ^ { i , j , k } \| _ { \Sigma _ { \Delta t } } ^ { 2 } ) ) ) , } \end{array}\tag{6}
$$

where $\rho$ is the robust Huber cost function to downweigh outlier correspondences and Σ is the covariance matrix. In the case of the reprojection error $\Sigma$ is associated to the scale of the key point in the camera i observing the points l and $j$ respectively. For the two other error terms Σ is associated to the time interval between two consecutive observations of an object, i.e., the longer time the more uncertainty there is about the constant velocity assumption. The parameters to be optimized are $\boldsymbol { \theta } = \{ \mathbf { T } _ { \mathbb { C } \mathbb { W } } ^ { i } , \mathbf { T } _ { \mathbb { W } 0 } ^ { k , i } , \mathbf { x } _ { \mathbb { W } } ^ { l } , \mathbf { x } _ { 0 } ^ { j , k } , \mathbf { v } _ { i } ^ { k } , \mathbf { w } _ { i } ^ { k } \}$

Fig. 5 shows the boolean Hessian matrix (H) of the problem described. The Hessian can be built from the Jacobian matrices associated to each edge in the factor graph. In order to have a non-zero (i, j) block matrix, there must be an edge between i and j node in the factor graph. Notice the difference in the sparsity patterns of the map points and the object points. The size of the Hessian matrix is dominated by the number of map points $N _ { m p }$ and object points, which in typical problems is several orders of magnitude larger than the number of cameras and objects. Applying Schur complement trick and solving the system has a run-time complexity of $\mathcal { O } ( N _ { c } ^ { 3 } + N _ { c } ^ { 2 } N _ { m p } + \bar { N } _ { c } N _ { o } \bar { N } _ { o p } )$ , where either the second or third term will dominate the cost depending on the number of static and dynamic points.

## E. Bounding Boxes

We propose to decouple the estimation of the trajectories and the bounding boxes of the dynamic objects. The former provides the system tracking with rich clues for ego-motion estimation, and the conjunction of both are useful to understand the dynamics of the surroundings. The output of the data association and the BA stages contains the camera poses, the structure of the static scene and the dynamic objects, and the 6 DoF trajectory of one point for each object. This one point is the center of mass of the object 3D points when it is first observed. Even though the center of mass changes along time with new points observations, the object pose that is tracked and optimized is referred to this first center of mass. To have a full understanding of the moving surroundings, it is important to know the objects dimensions and space occupancy. Tackling the two problems independently allows to track dynamic objects from the first frame in which they appear independently of the camera-object view point.

We initialize an object bounding box by searching two perpendicular planes that fit roughly the majority of the object points. We hypothesize that many man-made objects can approximately fit a 3D bounding box. In the case in which only one plane is found, we add a prior on the rough dimensions ofthe non-observable direction that is related to the object class. This procedure is done within a RANSAC scheme: we choose the computed 3D bounding box that has the largest IoU of its image projection with the CNN 2D bounding box. This bounding box is computed once for every object track.

To refine the bounding box dimensions and its pose relative to the object tracking reference, an image-based optimization is performed within a temporal window. This optimization seeks to minimize the distance between the 3D bounding box image projection and the CNN 2D bounding box prediction. Given that this problem is not observable for less than three views of an object, this is only performed once an object has at least three observing key frames. Also, to constraint the solution space in case the view of an object makes this problem non-observable (e.g., a car observed from the back), a soft prior about the object dimensions is included. Since this prior is tightly related to the object class, we believe that adding this soft prior does not mean a loss of generality. Finally, the initial bounding box pose is set as a prior so that the optimization solution remains close.

## IV. EXPERIMENTS

In this section we detail the experiments carried out to test DynaSLAM II. It is divided in two main blocks: one that assesses the effect of tracking objects on the estimation of camera motion (Subsection IV-A), and one that analyzes the multi-object tracking performance (Subsection IV-B).

## A. Visual Odometry

For the visual odometry experiments we have chosen the KITTI tracking (Table I) and raw (Table II) datasets [29]. They contain several gray-scale and RGB stereo sequences of urban and road scenes recorded from a car perspective with circulating vehicles and pedestrians, as well as its GPS data.

Tables I and II detail comparisons of our system’s performance against ORB-SLAM2 and our previous work DynaSLAM [4]. ORB-SLAM2 is the base SLAM system on which we build DynaSLAM II, and does not specifically address dynamic objects. DynaSLAM adds ORB-SLAM2 the capability to detect the features belonging to dynamic objects and classes but uniquely ignores them and does not track them. Both DynaSLAM I and II use instance semantic priors. However, DynaSLAM I uses them to ignore information belonging to dynamic objects and DynaSLAM II uses them to track the different dynamic objects in the scene and have additional clues for the camera ego motion estimation. The difference in the results of ORB-SLAM2 and DynaSLAM gives an idea of how dynamic each sequence is. Theoretically, if dynamic objects are representative in the scene and they are in circulation, DynaSLAM has better performance, as can be seen in sequences 0020 and 1003-0047 in Tables I and II respectively. However, if dynamic objects are representative in the scene but not in motion, e.g., parked cars, DynaSLAM shows a larger trajectory error. This happens because the features belonging to the static nearby vehicles, useful for pose estimation, are not used. This is seen for example in the sequence 0001 in Table I. Besides that,

TABLE I  
EGOMOTION COMPARISON ON THE KITTI TRACKING DATASET. RESULTS OF SEQUENCES WITHOUT EGOMOTION ARE NOT SHOWN
<table><tr><td rowspan="2">seq</td><td colspan="3"> $\mathrm { O R B - S L A M 2 \ [ 1 ] }$ </td><td colspan="3">DynaSLAM [4]</td><td colspan="3">VDO-SLAM [14]</td><td colspan="3">Ours</td></tr><tr><td>ATE [m]</td><td>RPEt[m/f]</td><td> $\mathrm { R P E } _ { \mathrm { R } } [ ^ { \circ } / \mathrm { f } ]$ </td><td>ATE [m]</td><td>RPEt[m/f]</td><td> $\mathrm { R P E } _ { \mathrm { R } } [ ^ { \circ } / \mathrm { f } ]$ </td><td>ATE [m]</td><td>RPEt[m/f]</td><td> $\mathrm { R P E } _ { \mathrm { R } } [ ^ { \circ } / \mathrm { f } ]$ </td><td>ATE [m]</td><td></td><td>RPE{[m/f] RPER[°/f]</td></tr><tr><td>0000</td><td>1.32</td><td>0.04</td><td>0.06</td><td>1.35</td><td>0.04</td><td>0.06</td><td></td><td>0.05</td><td>0.05</td><td>1.29</td><td>0.04</td><td>0.06</td></tr><tr><td>0001</td><td>1.95</td><td>0.05</td><td>0.04</td><td>2.42</td><td>0.05</td><td>0.04</td><td></td><td>0.12</td><td>0.04</td><td>2.31</td><td>0.05</td><td>0.04</td></tr><tr><td>0002</td><td>0.95</td><td>0.04</td><td>0.03</td><td>1.04</td><td>0.04</td><td>0.03</td><td></td><td>0.04</td><td>0.02</td><td>0.91</td><td>0.04</td><td>0.02</td></tr><tr><td>0003</td><td>0.74</td><td>0.07</td><td>0.04</td><td>0.78</td><td>0.07</td><td>0.04</td><td></td><td>0.09</td><td>0.04</td><td>0.69</td><td>0.06</td><td>0.04</td></tr><tr><td>0004</td><td>1.44</td><td>0.07</td><td>0.06</td><td>1.52</td><td>0.07</td><td>0.06</td><td></td><td>0.11</td><td>0.05</td><td>1.42</td><td>0.07</td><td>0.06</td></tr><tr><td>0005</td><td>1.23</td><td>0.06</td><td>0.03</td><td>1.22</td><td>0.06</td><td>0.03</td><td></td><td>0.10</td><td>0.02</td><td>1.34</td><td>0.06</td><td>0.03</td></tr><tr><td>0006</td><td>0.19</td><td>0.02</td><td>0.04</td><td>0.19</td><td>0.02</td><td>0.04</td><td></td><td>0.02</td><td>0.05</td><td>0.19</td><td>0.02</td><td>0.04</td></tr><tr><td>0007</td><td>2.47</td><td>0.05</td><td>0.07</td><td>2.69</td><td>0.05</td><td>0.07</td><td></td><td></td><td></td><td>3.10</td><td>0.05</td><td>0.07</td></tr><tr><td>0008</td><td>1.40</td><td>0.08</td><td>0.04</td><td>1.29</td><td>0.08</td><td>0.04</td><td></td><td></td><td></td><td>1.68</td><td>0.10</td><td>0.04</td></tr><tr><td>0009</td><td>4.00</td><td>0.06</td><td>0.05</td><td>3.55</td><td>0.06</td><td>0.05</td><td></td><td></td><td></td><td>5.02</td><td>0.06</td><td>0.06</td></tr><tr><td>0010</td><td>1.68</td><td>0.07</td><td>0.04</td><td>1.84</td><td>0.07</td><td>0.04</td><td></td><td></td><td></td><td>1.30</td><td>0.07</td><td>0.03</td></tr><tr><td>0011</td><td>0.97</td><td>0.04</td><td>0.03</td><td>1.05</td><td>0.04</td><td>0.03</td><td></td><td></td><td></td><td>1.03</td><td>0.04</td><td>0.03</td></tr><tr><td>0013</td><td>1.18</td><td>0.04</td><td>0.05</td><td>1.18</td><td>0.04</td><td>0.05</td><td></td><td></td><td></td><td>1.10</td><td>0.04</td><td>0.04</td></tr><tr><td>0014</td><td>0.13</td><td>0.03</td><td>0.08</td><td>0.13</td><td>0.03</td><td>0.08</td><td></td><td></td><td></td><td>0.12</td><td>0.03</td><td>0.08</td></tr><tr><td>0018</td><td>0.89 2.31</td><td>0.05</td><td>0.03</td><td>1.00</td><td>0.05</td><td>0.03</td><td></td><td>0.07</td><td>0.02</td><td>1.09</td><td>0.05</td><td>0.02</td></tr><tr><td>0019 0020</td><td>16.80</td><td>0.05 0.11</td><td>0.03</td><td>2.35</td><td>0.05</td><td>0.03</td><td></td><td></td><td></td><td>2.25</td><td>0.05</td><td>0.03</td></tr><tr><td></td><td></td><td></td><td>0.07</td><td>1.10</td><td>0.05</td><td>0.04</td><td></td><td>0.16</td><td>0.03</td><td>1.36</td><td>0.07</td><td>0.04</td></tr><tr><td>mean</td><td>2.33</td><td>0.055</td><td>0.046</td><td>1.45</td><td>0.051</td><td>0.045</td><td></td><td>0.084</td><td>0.036</td><td>1.54</td><td>0.053</td><td>0.043</td></tr></table>

TABLE II

EGOMOTION COMPARISON ON THE KITTI RAW DATASET
<table><tr><td rowspan="2">seq</td><td colspan="3">ORB-SLAM2 [1] ATE [m] RPE{[m] RPER[rd]</td><td colspan="3">DynaSLAM [4] ATE [m] RPE{[m] RPER[rd]</td><td colspan="3">ClusterSLAM [26] ATE [m] RPE{[m] RPER[rd]</td><td colspan="3">ClusterVO [13]</td><td colspan="3">Ours</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>ATE [m] RPE{[m] RPER[rd]</td><td></td><td></td><td>ATE [m] RPE{[m] RPER[rd]</td><td></td></tr><tr><td>0926-0009</td><td>0.83</td><td>1.85</td><td>0.01</td><td>0.81</td><td>1.80</td><td>0.01</td><td>0.92</td><td>2.34</td><td>0.03</td><td>0.79</td><td>2.98</td><td>0.03</td><td>0.85</td><td>1.87</td><td>0.01</td></tr><tr><td>0926-0013</td><td>0.32</td><td>1.04</td><td>0.01</td><td>0.30</td><td>0.99</td><td>0.01</td><td>2.12</td><td>5.50</td><td>0.07</td><td>0.26</td><td>1.16</td><td>0.01</td><td>0.29</td><td>0.93</td><td>0.00</td></tr><tr><td>0926-0014</td><td>0.50</td><td>1.22</td><td>0.01</td><td>0.60</td><td>1.62</td><td>0.01</td><td>0.81</td><td>2.24</td><td>0.03</td><td>0.48</td><td>1.04</td><td>0.01</td><td>0.48</td><td>1.35</td><td>0.01</td></tr><tr><td>0926-0051</td><td>0.38</td><td>1.16</td><td>0.00</td><td>0.46</td><td>1.17</td><td>0.00</td><td>1.19</td><td>1.44</td><td>0.03</td><td>0.81</td><td>2.74</td><td>0.02</td><td>0.44</td><td>1.14</td><td>0.00</td></tr><tr><td>0926-0101</td><td>2.97</td><td>13.63</td><td>0.03</td><td>3.52</td><td>15.14</td><td>0.03</td><td>4.02</td><td>12.43</td><td>0.02</td><td>3.18</td><td>12.78</td><td>0.02</td><td>4.33</td><td>15.02</td><td>0.04</td></tr><tr><td>0929-0004 1003-0047</td><td>0.62</td><td>1.38</td><td>0.01</td><td>0.56</td><td>1.36</td><td>0.01</td><td>1.12</td><td>2.78</td><td>0.02</td><td>0.40</td><td>1.77</td><td>0.02</td><td>0.64</td><td>1.41</td><td>0.01</td></tr><tr><td></td><td>20.49</td><td>32.59</td><td>0.08</td><td>2.87</td><td>5.95</td><td>0.02</td><td>10.21</td><td>8.94</td><td>0.06</td><td>4.79</td><td>6.54</td><td>0.05</td><td>3.03</td><td>6.85</td><td>0.02</td></tr><tr><td>mean</td><td>3.73</td><td>7.55</td><td>0.02</td><td>1.30</td><td>4.00</td><td>0.01</td><td>2.91</td><td>5.10</td><td>0.04</td><td>1.53</td><td>4.14</td><td>0.02</td><td>1.44</td><td>4.08</td><td>0.01</td></tr></table>

DynaSLAM II achieves a performance better than both ORB-SLAM and DynaSLAM in these two types of scenarios in many of the evaluated sequences. On the one hand, when dynamic instances are moving, DynaSLAM II estimates the velocity of the corresponding objects and provide the BA with rich clues for camera pose estimation when the static representation is not sufficient. This occurs when dynamic objects occlude nearby scene regions and thus static features only provide valuable hints for accurately estimating the camera rotation. Note that, we have not evaluated the estimated speed of the vehicle because there is no ground truth for this. On the other hand, when dynamic classes instances are static, DynaSLAM II tracks their features estimating that their velocity is close to zero, i.e., these object points act much like static points. However, our camera tracking performance is seen slightly degraded compared to ORB-SLAM because we allow for more flexibility when estimating the dynamic-object motion status.

Tables I and II present our ego motion results compared to those of state-of-the-art systems that also track dynamic objects in a joint SLAM framework. ClusterSLAM [26] acts as a back end rather than a SLAM system and is highly dependent on the camera poses initial estimates. ClusterVO [13] and VDO-SLAM [14] are SLAM systems as ours with the multi-object tracking capability. The former can handle stereo and RGB-D data, whereas the latter only handles RGB-D. The reported errors are given with different metrics so that we can directly use the values that the authors provide. DynaSLAM II achieves in all sequences a lower translational relative error $\left( \mathrm { R P E } _ { \mathrm { t } } \right)$ than that of VDO-SLAM. However, VDO-SLAM usually achieves a lower rotational pose error $( \mathrm { R P E _ { R } } )$ . Since far points are the ones that provide the richest clues for rotation estimation, we believe that this difference in accuracy does not depend on the object tracking performance and is therefore due to the underlying camera pose estimation algorithm and sensor suite. Regarding the performance of ClusterVO, it achieves an accuracy which is in most sequences quite similar to ours.

## B. Multi-Object Tracking

Once the utility of tracking dynamic objects for ego motion estimation is demonstrated, we have chosen again the KITTI tracking [29] and Oxford Multimotion [30] datasets to validate our multi-object tracking results. On the KITTI dataset, dynamic-object trajectories and 3D bounding boxes are provided thanks to expensive manual annotations on LIDAR point clouds.

First of all, we would like to draw the attention of the reader to Fig. 1 to have a look at our qualitative results on this dataset. The bounding boxes of the two purple cars on the left are well estimated despite their partial view. This scene is also challenging because the other two front cars are far from the camera and yet are correctly tracked.

In the last decade Bernardin et al. [31] introduced the CLEAR MOT metrics to allow for objective comparison of tracker characteristics, focusing on their precision in estimating object locations, their accuracy in recognizing object configurations and their ability to consistently label objects over time. Whereas these metrics are well established in the computer vision and robotics communities and provide valuable insights about the per-frame performance of trackers, they do not take into account the quality of the tracked object trajectories. We suggest that to correctly evaluate multi-object tracking within a SLAM framework, one needs to report the CLEAR MOT metric MOTP<sup>1</sup> as well as the common trajectory error metrics. Most related works on SLAM and multi-object tracking only report the CLEAR MOT metric MOTP [11]–[13] and besides that, the authors of VDO-SLAM [14] uniquely report the relative pose error of all objects trajectories of one sequence as a single ensemble. We think that, to facilitate comparison, this metric should be instead reported for individual trajectories.

TABLE III  
OBJECTS MOTION COMPARISON ON THE KITTI TRACKING DATASET
<table><tr><td>sequence object id (class)</td><td></td><td>0003 1 (car)</td><td>0005 31 (car)</td><td>0010</td><td colspan="2">0011</td><td colspan="2">0018</td><td colspan="2">0019</td><td></td><td>0020 12 (car)</td><td></td></tr><tr><td></td><td></td><td></td><td></td><td>0 (car)</td><td>0 (car)</td><td>35 (car)</td><td>2 (car)</td><td>3 (car)</td><td>63 (car)</td><td>72 (car)</td><td>0 (car)</td><td></td><td>122 (car)</td></tr><tr><td></td><td>ATE [m]</td><td>0.69</td><td>0.51</td><td>0.95</td><td>1.05</td><td>1.25</td><td>1.10</td><td>1.13</td><td>0.86</td><td>0.99</td><td>0.56</td><td>1.18</td><td>0.87</td></tr><tr><td></td><td>RPEt [m/m]</td><td>0.34</td><td>0.26 13.50</td><td>0.40 2.84</td><td>0.43 12.51</td><td>0.89</td><td>0.30 9.27</td><td>0.55 20.05</td><td>1.45</td><td>1.12 3.36</td><td>0.45 1.30</td><td>0.40</td><td>0.72 5.75</td></tr><tr><td></td><td>RPER [° /m]</td><td>1.84</td><td></td><td></td><td></td><td>16.64</td><td></td><td></td><td>48.80</td><td></td><td></td><td>6.19</td><td></td></tr><tr><td>2D</td><td>TP (%)</td><td>50.00</td><td>28.96</td><td>81.63</td><td>72.65</td><td>53.17</td><td>86.36</td><td>53.33</td><td>35.26</td><td>29.11</td><td>63.68</td><td>42.77</td><td>34.90</td></tr><tr><td></td><td>MOTP [%]</td><td>71.79</td><td>60.30</td><td>73.51</td><td>74.78</td><td>65.25</td><td>74.81</td><td>70.94</td><td>63.50</td><td>62.59</td><td>78.54</td><td>76.77</td><td>78.76</td></tr><tr><td>BV</td><td>TP (%)</td><td>39.34</td><td>14.48</td><td>70.41</td><td>61.66</td><td>19.05</td><td>67.05</td><td>21.75</td><td>29.48</td><td>29.43</td><td>43.78</td><td>37.64</td><td>34.51</td></tr><tr><td></td><td>MOTP [%]</td><td>56.61</td><td>46.84</td><td>47.60</td><td>50.74</td><td>31.95</td><td>45.47</td><td>41.45</td><td>45.69</td><td>55.48</td><td>45.00</td><td>49.29</td><td>48.05</td></tr><tr><td></td><td>TP (%)</td><td>38.53</td><td>11.45</td><td>68.37</td><td>52.28</td><td>6.35</td><td>62.12</td><td>16.84</td><td>26.48</td><td>29.43</td><td>31.84</td><td>36.23</td><td>29.02</td></tr><tr><td>3D</td><td>MOTP [%]</td><td>48.20</td><td>34.20</td><td>40.28</td><td>47.35</td><td>26.02</td><td>34.80</td><td>35.80</td><td>33.89</td><td>39.81</td><td>46.15</td><td>40.81</td><td>44.43</td></tr></table>

TABLE IV  
TABLE V  
ATE [M] FOR MULTI-OBJECT TRACKING AND EGOMOTION IN SWINGING\_4\_UNCONSTRAINED (OXFORD MULTIMOTION DATASET)

MOTP EVALUATION ON THE KITTI TRACKING DATASET. THE CATEGORIES EASY, MODERATE AND HARD ARE BASED ON THE 2D BOUNDING BOXES HEIGHT, OCCLUSION AND TRUNCATION LEVEL
<table><tr><td></td><td colspan="3">MOTPBV</td><td colspan="3">MOTP3D</td></tr><tr><td></td><td>Easy</td><td>Moderate</td><td>Hard</td><td>Easy</td><td>Moderate</td><td>Hard</td></tr><tr><td>[32]</td><td>81.34 %</td><td>70.70 %</td><td>66.32 %</td><td>80.62 %</td><td>70.01 %</td><td>65.76 %</td></tr><tr><td>[11]</td><td>88.07 %</td><td>77.83 %</td><td>72.73 %</td><td>86.57 %</td><td>74.13 %</td><td>68.96 %</td></tr><tr><td>[18]</td><td>71.83 %</td><td>47.16 %</td><td>40.30 %</td><td>64.51 %</td><td>43.70 %</td><td>37.66 %</td></tr><tr><td>[13]</td><td>74.65 %</td><td>49.65 %</td><td>45.62 %</td><td>55.85 %</td><td>38.93 %</td><td>33.55 %</td></tr><tr><td>Ours</td><td>64.69 %</td><td>58.75 %</td><td>58.36 %</td><td>53.14 %</td><td>48.66 %</td><td>48.57 %</td></tr></table>

Table IV shows an evaluation of all object detections in the KITTI tracking dataset with the KITTI 3D object detection benchmark. This allows us to directly compare our multi-object tracking results to those of state-of-the-art similar systems (Table IV). The CNNs of Chen et al. [32] and specially of Li et al. [11] achieve excellent results thanks to the single-view network accuracy itself and the multi-view refinement approach of the latter one, to the detriment of a generality loss. On the other hand, the accuracy of Barsan et al. [18] and Huang et al. [13] in detecting bounding boxes is remarkable but very sensitive to object truncation and occlusion. Our results show that we handle objects truncation and occlusion with a minor loss in precision. However, less bounding boxes are usually discovered. Our intuition is that our system feature-based nature renders this step specially challenging, opposite to the work by Barsan et al. [18], which computes dense stereo matching.

To evaluate our estimation of object trajectories, in Table III we have chosen the 12 longest sequences of the KITTI tracking dataset whose 2D detections are neither occluded nor truncated, and whose height is at least of 40 pixels. These chosen objects are labeled with their ground-truth object id. For each of these ground truth trajectories we look for the most overlapping bounding boxes in our estimations (the overlapping has to be of at least 25 %). In the case of the trajectory metrics (ATE and RPE) and the 2D MOTP, this overlapping is computed as the IoU of the 3D bounding boxes projected over the current frame. For the other two evaluations (BV and 3D), the overlapping is computed as the IoU of the bounding boxes in bird view and in 3D respectively. This evaluation gives an idea of our framework tracking performance and our bounding boxes quality. Regarding the true positives percentage, we can see that objects are tracked for the majority of their trajectory. Missing detections occur because the objects lay far from the camera and the stereo matching does not provide enough features for a rich tracking. Note that, the passersby tracking accuracy is lower than that of the cars due to their non-rigid shape (seq. 0017). The trajectory errors of the cars are acceptable but they are far from the ego-motion estimation performance. Our intuition is that our algorithm feature-based nature renders the bounding box estimation specially challenging. A larger amount of 3D points would always provide richer clues for object tracking.

<table><tr><td>System</td><td>Ego Camera</td><td>Obj. 1</td><td>Obj. 2</td><td>Obj. 3</td><td>Obj. 4</td></tr><tr><td>MVO [30]</td><td>0.93</td><td>0.36</td><td>0.64</td><td>0.45</td><td>5.94</td></tr><tr><td>ClusterVO [13]</td><td>0.62</td><td>0.24</td><td>0.45</td><td>0.24</td><td>4.69</td></tr><tr><td>Ours</td><td>0.21</td><td>0.41</td><td>0.37</td><td>1.09</td><td>0.28</td></tr></table>

Finally, to underline the robustness and generality of the presented approach and not to only focus on an outdoor driving scenario where constant velocity models are pretty convenient, we have also evaluated the multi-object tracking performance of DynaSLAM II on the swinging\_4\_unconstrained sequence from the Oxford Multimotion dataset [30]. This sequence is recorded with a RGB-D and a stereo camera in an indoor environment with four textured boxes hanging from the ceiling and balancing with non-constant velocities. The egomotion and tracking results of DynaSLAM II and ofother similar systems on this dataset can be observed in Table V. Our system achieves a significantly higher accuracy for ego motion estimation than compared methods, while objects tracking seems to be more robust with a lower highest error. Experiments also evince that using a soft constant velocity prior for object motion does not restrict our approach to tailored cases. Comparison against VDO-SLAM has been omitted since there exists no feasible way to compute the RPE for object tracking on this dataset.

## C. Timing Analysis

To complete our proposal evaluation, Table VI shows the average computational time for its different building blocks. The timing of DynaSLAM II is highly dependent on the number of objects to be tracked. In sequences like KITTI tracking 0003 there are only two objects at a time as maximum and runs thus at 12 fps. However, the sequence 0020 can have up to 20 objects at a time and its performance is seen slightly compromised, but still achieves a real time performance at ∼10 fps. We do not include within these numbers the computational time of the semantic segmentation CNN since it depends on the GPU power and CNN model complexity. Algorithms such as YOLACT [33] can run in real time and provide high-quality instance masks.

TABLE VI  
DYNASLAM II AVERAGE COMPUTATIONAL TIME
<table><tr><td>Sequence</td><td colspan="2">Building block</td><td colspan="2">Time [ms]</td></tr><tr><td rowspan="2">KITTI tracking 0003</td><td colspan="2">Tracking thread</td><td colspan="2"> $8 0 . 1 0 \pm 0 . 7 8$   $6 1 . 3 7 \pm 6 . 7 0$ </td></tr><tr><td colspan="2">Local BA Bounding Boxes BA</td><td colspan="2"> $0 . 0 7 \pm 0 . 0 1$ </td></tr><tr><td rowspan="2">KITTI tracking 0020</td><td colspan="2">Tracking thread</td><td colspan="2"> $9 4 . 5 6 \pm 1 . 2 7$ </td></tr><tr><td colspan="2">Local BA</td><td colspan="2"> $6 5 . 0 3 \pm 1 7 . 7 2$ </td></tr><tr><td></td><td colspan="2">Bounding Boxes BA</td><td colspan="2"> $0 . 6 0 \pm 0 . 0 5$ </td></tr><tr><td></td><td colspan="2">|[11]|[14]|[26]</td><td colspan="2">|[13] |</td></tr><tr><td>fps</td><td>5.8</td><td>|5 - 8 | 7</td><td>8</td><td>Ours 10 - 12</td></tr></table>

Finally, the last rows of Table VI collect the average timing results for systems that jointly perform SLAM and multi-object tracking in the KITTI dataset. DynaSLAM II is the only system that can provide at present a real-time solution.

## V. CONCLUSIONS AND FUTURE WORK

We have proposed an object-level SLAM system with novel measurement functions between cameras, objects and 3D map points. This allows us to track dynamic objects and tightly optimize the trajectories of self and surroundings to let both estimations be mutually beneficial. We decouple the problem of object tracking from that of bounding boxes estimation and, differently from other works, we do not make any assumptions about the objects motion, pose or model. Our experiments show that DynaSLAM II achieves a state-of-the-art accuracy at real time performance, which renders our framework suitable for a large number of real world applications.

The feature-based core of our system limits its ability to discover accurate 3D bounding boxes, and also to track objects with low texture. Fully exploiting the dense visual information would certainly push these limits forward. We would also like to explore the –even more– challenging task of multi-object tracking and SLAM with only a monocular camera. This is an interesting direction since dynamic-object tracking can provide rich clues about the scale of the map.

## REFERENCES

[1] R. Mur-Artal and J. D. Tardós, “ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1255–1262, Oct. 2017.

[2] J. Engel, V. Koltun, and D. Cremers, “Direct sparse odometry,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 3, pp. 611–625, Mar. 2018.

[3] C. Forster, M. Pizzoli, and D. Scaramuzza, “SVO: Fast semi-direct monocular visual odometry,” in Proc. IEEE Int. Conf. Robot. Automat., 2014, pp. 15–22.

[4] B. Bescos, J. M. Fácil, J. Civera, and J. Neira, “DynaSLAM: Tracking, mapping, and inpainting in dynamic scenes,” IEEE Robot. Automat. Lett., vol. 3, no. 4, pp. 4076–4083, Oct. 2018.

[5] Y. Sun, M. Liu, and M. Q.-H. Meng, “Improving RGB-D SLAM in dynamic environments: A motion removal approach,” Robot. Auton. Syst., vol. 89, pp. 110–122, 2017.

[6] S. Li and D. Lee, “RGB-D SLAM in dynamic environments using static point weighting,” IEEERobot.Automat. Lett., vol. 2, no. 4, pp. 2263–2270, Oct. 2017.

[7] L. Xiao, J. Wang, X. Qiu, Z. Rong, and X. Zou, “Dynamic-SLAM: Semantic monocular visual localization and mapping based on deep learning in dynamic environment,” Robot. Auton. Syst., vol. 117, pp. 1–16, 2019.

[8] B. Bescos, J. Neira, R. Siegwart, and C. Cadena, “Empty cities: Image inpainting for a dynamic-object-invariant space,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 5460–5466.

[9] B. Bescos, C. Cadena, and J. Neira, “Empty cities: A dynamic-objectinvariant space for visual slam,” IEEE Trans. Robot., 2020.

[10] B. Beši´c and A. Valada, “Dynamic object removal and spatio-temporal RGB-D inpainting via geometry-aware adversarial learning,” 2020, arXiv:2008.05058.

[11] P. Li, T. Qin and S. Shen, “Stereo vision-based semantic 3D object and ego-motion tracking for autonomous driving,” in Proc. IEEE Eur. Conf. Comput. Vis., 2018, pp. 646–661.

[12] S. Yang and S. Scherer, “CubeSLAM: Monocular 3-D object SLAM,” IEEE Trans. Robot., vol. 35, no. 4, pp. 925–938, Aug. 2019.

[13] J. Huang, S. Yang, T.-J. Mu, and S.-M. Hu, “ClusterVO: Clustering moving instances and estimating visual odometry for self and surroundings,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2020, pp. 2168–2177.

[14] J. Zhang, M. Henein, R. Mahony, and V. Ila, “VDO-SLAM: A. Visual dynamic object-aware SLAM system,” 2020, arXiv:2005.11052.

[15] C.-C. Wang, C. Thorpe, and S. Thrun, “Online simultaneous localization and mapping with detection and tracking of moving objects: Theory and results from a ground vehicle in crowded urban areas,” in Proc. IEEE Int. Conf. Robot. Automat., 2003, pp. 842–849.

[16] S. Wangsiripitak and D. W. Murray, “Avoiding moving outliers in visual SLAM by tracking moving objects,” in Proc. IEEE Int. Conf. Robot. Automat., 2009, pp. 375–380.

[17] J. G. Rogers, A. J. Trevor, C. Nieto-Granda, and H. I. Christensen, “SLAM with expectation maximization for moveable object tracking,” Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2010, pp. 2077–2082.

[18] I. A. Bârsan, P. Liu, M. Pollefeys, and A. Geiger, “Robust dense mapping for large-scale dynamic environments,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 7510–7517.

[19] A. Rosinol, A. Gupta, M. Abate, J. Shi, and L. Carlone, “3D dynamic scene graphs: Actionable spatial perception with places, objects, and humans,” Robot. Sci. Sys. (RSS), 2020, arXiv:2002.06289.

[20] M. Hosseinzadeh, K. Li, Y. Latif, and I. Reid, “Real-time monocular object-model aware sparse SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 7123–7129.

[21] C.-C. Wang, C. Thorpe, S. Thrun, M. Hebert, and H. Durrant-Whyte, “Simultaneous localization, mapping and moving object tracking,” Int. J. Robot. Res., vol. 26, no. 9, pp. 889–916, 2007.

[22] M. Rünz and L. Agapito, “Co-fusion: Real-time segmentation, tracking and fusion of multiple objects,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 4471–4478.

[23] M. Runz, M. Buffier, and L. Agapito, “Maskfusion: Real-time recognition, tracking and reconstruction of multiple moving objects,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2018, pp. 10–20.

[24] B. Xu, W. Li, D. Tzoumanikas, M. Bloesch, A. Davison, and S. Leutenegger, “MID-fusion: Octree-based object-level multi-instance dynamic SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 5231–5237.

[25] M. Henein, G. Kennedy, R. Mahony, and V. Ila, “Exploiting rigid body motion for SLAM in dynamic environments,” in Proc. IEEE Int. Conf. Robot. Automat., vol. 18, 2018, p. 19.

[26] J. Huang, S. Yang, Z. Zhao, Y.-K. Lai, and S.-M. Hu, “ClusterSLAM: A. slam backend for simultaneous rigid body clustering and motion estimation,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2019, pp. 5875–5884.

[27] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: An efficient alternative to SIFT or SURF,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2011, pp. 2564–2571.

[28] J. Munkres, “Algorithms for the assignment and transportation problems,” J. Soc. Ind. Appl. Math., vol. 5, no. 1, pp. 32–38, 1957.

[29] A. Geiger, P. Lenz, C. Stiller, and R. Urtasun, “Vision meets robotics: The kitti dataset,” Int. J. Robot. Res., vol. 32, no. 11, pp. 1231–1237, 2013.

[30] K. M. Judd, J. D. Gammell, and P. Newman, “Multimotion visual odometry (MVO): Simultaneous estimation of camera and third-party motions,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 3949–3956.

[31] K. Bernardin and R. Stiefelhagen, “Evaluating multiple object tracking performance: The CLEAR MOT metrics,” in Proc. EURASIP J. Image Video Process., 2008, pp. 1–10.

[32] X. Chen, K. Kundu, Y. Zhu, H. Ma, S. Fidler, and R. Urtasun, “3D object proposals using stereo imagery for accurate object class detection,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 5, pp. 1259–1272, May 2018.

[33] D. Bolya, C. Zhou, F. Xiao, and Y. J. Lee, “Yolact: Real-time instance segmentation,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2019, pp. 9157– 9166.