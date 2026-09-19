# Large-Scale Autonomous Flight With Real-Time Semantic SLAM Under Dense Forest Canopy

Xu Liu , Guilherme V. Nardari , Fernando Cladera Ojeda , Yuezhan Tao , Alex Zhou , Thomas Donnelly, Chao Qu, Steven W. Chen , Roseli A. F. Romero , Camillo J. Taylor , Member, IEEE, and Vijay Kumar , Fellow, IEEE

Abstract—Semantic maps represent the environment using a set of semantically meaningful objects. This representation is storageefficient, less ambiguous, and more informative, thus facilitating large-scale autonomy and the acquisition ofactionable information in highly unstructured, GPS-denied environments. In this letter, we propose an integrated system that can perform large-scale autonomous flights and real-time semantic mapping in challenging under-canopy environments. We detect and model tree trunks and ground planes from LiDAR data, which are associated across scans and used to constrain robot poses as well as tree trunk models. The autonomous navigation module utilizes a multi-level planning and mapping framework and computes dynamically feasible trajectories that lead the UAV to build a semantic map of the user-defined region of interest in a computationally and storage efficient manner. A drift-compensation mechanism is designed to minimize the odometry drift using semantic SLAM outputs in real time, while maintaining planner optimality and controller stability. This leads the UAV to execute its mission accurately and safely at scale.

Index Terms—Aerial systems: perception and autonomy, field robotics, robotics and automation in agriculture and forestry, SLAM.

## I. INTRODUCTION

UTONOMOUSLY surveying large-scale environments and building semantic maps are key capabilities for au  
tonomous mobile robots in many applications such as precision   
agriculture, infrastructure inspection, and search and rescue.

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/6998f67c0d0afda056491c2f62ddadffe3c43a3634920ebebd981f02036bd0fd.jpg)  
Fig. 1. Falcon 4 UAV platform. Our Falcon 4 platform used for this work is the successor of our Falcon 450 platform [1] and is equipped with a 3D LiDAR, an Open Vision Computer (OVC) 3 [2] which has a hardware synchronized IMU and stereo cameras, an Intel NUC onboard computer, and a Pixhawk 4 flight controller. The platform has a total weight of 4.2 kg and a 30-minute flight time.

Semantic maps of forests encode actionable information such as timber volume, yield estimation and forecasting. Unmanned Aerial Vehicles (UAVs) have unique advantages for such tasks: they can hover and fly fast in 3D environments, and they are not as affected by undergrowth or terrain elevation changes as ground robots.

Prior research and commercial solutions using UAVs for precision agriculture and forestry mostly focus on overhead flight through wide-open spaces. This simplifies operations, but limits what is possible to measure. On the other hand, it is impractical for human pilots to gather data on a large scale under the forest canopy, especially considering the communication range limit in radio and first-person view systems. Thus, UAVs capable of autonomous under-canopy flights are indispensable for acquiring such data at scale.

Therefore, developing an autonomous UAV system that can operate under the tree canopy while extracting actionable information on a large scale is critical for precision agriculture and forestry. This task, however, is still challenging since: (1) GPS has unsatisfactory accuracy under dense forest canopies [3]. Differential or RTK GPS requires reliable communication with a base station, which is impractical for long-range operation in under-canopy environments; (2) the environment is unstructured and dynamic (e.g. leaves blowing in the wind), which is challenging for traditional Simultaneous Localization and Mapping (SLAM) algorithms that rely on static geometric features; (3) the environment is highly cluttered, which requires a capable and reliable navigation system.

We integrate autonomous flight with semantic SLAM to address these challenges. The main contributions of our paper are: (1) To our knowledge, this is the first system that integrates semantic SLAM in real-time into an autonomous UAV feedback loop, while relying only on onboard sensing and computation. The semantic SLAM, coupled with a drift-compensation mechanism minimizes the UAV odometry drift. (2) We propose a UAV hardware and software system that is capable of long-range autonomous flights in large-scale, unstructured, cluttered, and GPS-denied environments. The customized UAV hardware platform (Fig. 1) has over 30 minutes flight time. Our autonomous flight stack and semantic SLAM algorithm are open-sourced at, respectively:

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/637ee9552a325b1d2f044eb3a20265831705ea3c4b99328b149b9f0d86b649f8.jpg)  
Fig. 2. Area where the experiments were performed (Wharton State Forest). The left panel is a view from an over-canopy UAV, which shows the scale of the environment and the thick tree canopy; The middle panel is a view from an under-canopy UAV, which illustrates that the environment is highly unstructured and cluttered; The right panel shows that the ground is covered by undergrowth which moves with the wind and is unevenly illuminated. These factors cause traditional VIO or LIDAR odometry systems to drift over time.

https://github.com/KumarRobotics/kr\_autonomous\_flight https://github.com/KumarRobotics/sloam

The rest of the paper is arranged as follows: Section II surveys related work, followed by a problem statement in Section III. Section IV-A presents our UAV platform. Section IV-B and Section IV-C present the state estimation and perception modules. Section IV-D presents the planning and control modules, and explains how the integration between autonomy and semantic SLAM is achieved using the driftcompensation module. Section V shows our results and analysis in both simulated (V-A) and real-world experiments (V-B). Section VI presents the conclusion. A demo video can be found at: https://youtu.be/Ad3ANMX8gd4.

## II. RELATED WORK

In this section we provide an overview of related literature. Compared with geometric features, semantic features are usually sparser, less ambiguous and repetitive, and thus can improve robot localization [4]. In addition, semantic maps are more informative and descriptive, facilitating high-level task or global planning [5]. Generic shape models, such as dual quadrics [6] and 3D bounding boxes [7] can be used to build semantic maps without being limited to specific semantic classes or requiring prior shape models.

Several SLAM algorithms are designed specifically for autonomous navigation. Bavle et al. proposed VPS-SLAM [8], the horizontal and vertical planes are extracted from semantic objects and maintained in the map. The centroids of the planes are then used to provide additional constraints in the graph SLAM optimization. Even though these approaches demonstrate good performance on pre-collected datasets, they are either without autonomy [8], [9], or with autonomy that does not use any feedback from semantic SLAM [10].

Bavle et al. [11] propose a particle-filter-based approach that combines visual, inertial and semantic information for UAV localization, which is then used for navigation. However, their system is demonstrated in small-scale indoor environments with an average flight speed of 0.3 m/s. The semantic objects are not modeled but treated as 3D point landmarks. Most importantly, the UAV relies on a ground station for computation, which means it does not achieve independent autonomy, and is not suitable for outdoor large-scale applications.

Several autonomous flight systems that can handle relatively complicated environments with onboard sensing and computation have been proposed. Lin et al. described an autonomous UAV system that relies only on a monocular camera and an Inertial Measurement Unit (IMU) to estimate UAV poses and build a dense Truncated Signed Distance Field (TSDF) map in real time [12]. Oleynikova et al. present an autonomous flight system that is capable of exploring and building dense Euclidean Signed Distance Field (ESDF) maps of unknown environments [13]. Building ESDF maps is computationally demanding but usually required for planning algorithms that use gradient-based optimization. Zhou et al. propose EGO-Planner [14] that bypasses this requirement, although an occupancy grid map is still needed. However, these systems are demonstrated in environments that are less cluttered and have smaller scale than our test environments (Fig. 2). More importantly, none of them incorporate semantic information. The geometric-based odometry algorithms accumulate large drift in under-canopy environment, and the storage demand for maintaining a dense 3D volumetric map is high.

Therefore, developing an autonomous UAV navigation system that requires no external infrastructure and can integrate semantic SLAM into its feedback loop in real time is of key importance for large-scale surveying and mapping missions.

We build upon the system proposed by Mohta et al. [1], [15], and make the following improvements: (1) We integrate semantic SLAM into the autonomy stack in real time, which produces more reliable, drift-minimized robot localization, as well as a semantically meaningful map. A drift compensation mechanism is designed to achieve this. (2) We introduce a multilevel planning and mapping framework to improve computation and storage efficiency and support large-scale coverage and mapping missions.

In addition, we modify the semantic lidar odometry and mapping framework (SLOAM) proposed in Chen et al. [16] by: (1) employing RangeNet++ [17], a neural network designed for Light Detection and Ranging (LiDAR) data, giving more accurate segmentation results; (2) integrating the state estimation step with a visual-inertial odometry (VIO) algorithm to increase robustness in case of sensor or segmentation failure; and (3)

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/1d7f4472a309f989ffda03e60abaf711b29db348d82360a48579f26b6c4d8563.jpg)  
Fig. 3. Software architecture. The system uses information from multiple sensors including 3D LIDAR, stereo cameras, and an IMU. It first detects objects in LiDAR scans using RangeNet++ [17]. Object detections are then tracked across the data sequence. This semantic data association is converted into constraints on both the robot poses as well as the object landmark models using the semantic SLAM module, where the relative motion estimated by a stereo VIO algorithm [18] is used as an initial guess, and SLOAM [16] is used to further optimize the UAV pose and landmarks. The output of the semantic SLAM module is used to reduce the odometry drift through a drift-compensation mechanism. This information is then used by the planning module in real-time. The high level planner uses a boustrophedon-based coverage planning algorithm [19] to cover a polygon region. The mid-level global planner relies on a sparse map to plan the shortes collision-free path to each coverage waypoint, and the local trajectory planner uses a small but finely discretized robot-centric map to plan a dynamically feasible and collision-free trajectory. Meanwhile, a UKF that runs at 200 Hz is used for the low-level control loop.

splitting the least-squares pose optimization into two steps to improve robustness.

## III. PROBLEM STATEMENT

Let $\mathcal { G }$ be a mission defined by the ordered set ${ \mathcal { G } } \triangleq$ $\{ g _ { 0 } , g _ { 1 } , \ldots , g _ { n } \}$ , where each $g _ { i } \in \mathcal { G }$ is a waypoint relative to the robot’s takeoff position. The user can either directly specify G or draw a region of interest using a set of polygons, and the coverage planner will automatically compute the optimal $\mathcal { G }$ to cover this region. Without a prior map, the UAV must autonomously navigate through the environment and visit all waypoints while detecting and modeling the semantic objects observed. For our current application to forestry, the semantic objects are tree trunks and ground planes.

## IV. PROPOSED APPROACH

In this section, we will introduce the individual modules in our system. The system diagram is shown in Fig. 3.

## A. UAV Platform

The Falcon 4 UAV platform used for our experiments is custom built with a carbon fiber frame. It is equipped with various onboard computers and sensors, as shown in Fig. 1.

Our sensor choices are based on thorough analysis and real-world verification. Cameras and IMUs are inexpensive, lightweight, and have high update rates. However, in undercanopy environments as shown in Fig. 2, varying illumination in sunlit forests makes it challenging for any VIO system. Exposure must be constantly adjusted, which is detrimental for many algorithms that track visual features since this violates the constant brightness assumption. On the other hand, 3D LiDARs, although expensive, can have dense and accurate range measurements up to several hundred meters while consuming negligible computation compared to stereo or multi-view depth estimation. They are also robust to environments with direct sunlight and varying illumination. These unique advantages make them indispensable for our purposes of large-scale autonomous navigation and accurate semantic mapping in under-canopy environments. To achieve good robustness, mapping accuracy, and odometry update rates, our system combines stereo VIO and LiDAR. The platform has a payload capacity (including battery) of ∼3 kg and can be reconfigured to carry additional onboard computers and other mission-specific sensors. Finally, the platform is powered with a 17,000 mAh Li-ion battery and achieves a flight time of ∼30 minutes with all onboard sensors and computers running.

## B. State Estimation

1) Odometry: We use the same odometry system proposed in our previous work [1]. The innermost loop of our odometry system is an Unscented Kalman Filter (UKF). The UKF relies on measurements from stereo VIO [18] and runs at 200 Hz, outputting smooth pose estimates. However, as mentioned in Fig. 2, there are many challenges for robot localization under the forest canopy. The geometric features used in traditional VIO or LiDAR odometry systems will thus become unreliable. Therefore, we use semantic SLAM to correct this drift before feeding it to the planning loop.

2) Semantic Lidar Odometry: Assuming that objects make contact with the ground and LiDAR observations will be gravity aligned, an object detected in a LiDAR beam will likely be present in other beams below it. Using this insight, SLOAM [16] computes a trellis graph from the point cloud with semantic labels, where nodes are organized as slices. Each node represents a group ofpoints ofthe same tree from the same horizontal LiDAR beam. Weighted edges are calculated based on the Euclidean distance between nodes. Finally, individual trees are given by the shortest paths starting from the first slice of the graph, which is computed using a greedy algorithm.

Using nodes that belong to the same tree, SLOAM computes a cylinder model c that estimates the diameter and growth direction of the tree trunk. To represent the ground, we divide the LiDAR reading according to a circular grid defined by the distance and angle of the space around the sensor. Within each grid cell, we retain the lowest points in the z direction. These points define the set of ground features and are used to fit one plane model π per bin. With these models, the robot can estimate its pose $\mathbf { T } _ { k }$ at each time step k and create a semantic map with important information for timber inventory, such as diameter at breast height and total tree count.

The main problem of relying purely on semantic information is that the state estimation will also fail if object detection fails. In the following sections, we present improvements to the SLOAM framework to increase stability and robustness for large-scale autonomy.

3) VIO Integration: VIO will drift under the forest canopy because of light variation and the movement of underbrush and tree branches, where most of the features are detected. However, a robust VIO algorithm with high quality sensors should still provide pose estimates that are consistent within short time intervals. On the other hand, SLOAM may fail if not enough trees are detected to constrain the pose estimation.

We use S-MSCKF [18] as an initial guess for pose estimation and run SLOAM to correct the drift over longer intervals. When the VIO odometry estimates a translational movement of at least 0.5 meters, a new keyframe is created. For a keyframe k, SLOAM will detect trees, perform data association, estimate a pose $\mathbf { T } ^ { \mathrm { S L O A M } }$ and update the semantic map. This integration is especially valuable to the autonomy stack presented in this work. It is essential in saving computational resources, which enables the whole system to run in real-time.

Since SLOAM’s data association is based on nearest-neighbor matching, if the motion between two keyframe poses is large, the association may fail or give false positive matches. We can use the relative motion estimated with VIO to initialize SLOAM and perform data association reliably.

Due to drift in VIO, the VIO pose $\mathbf { T } _ { k } ^ { \mathrm { V I O } }$ and SLOAM pose $\mathbf { T } _ { k } ^ { \mathrm { S L O A M } }$ may be different. To provide SLOAM with an initial guess, at every keyframe k, we store a tuple of poses $( \mathbf { T } _ { k } ^ { \mathrm { S L O A M } } , \mathbf { T } _ { k } ^ { \mathrm { V I O } } )$ . The initial guess of relative motion between keyframes estimated by the VIO is $\mathbf { T } _ { k } ^ { \mathrm { R E L } } = ( \mathbf { T } _ { k - 1 } ^ { \mathrm { V I O } } ) ^ { - 1 } \cdot \mathbf { T } _ { k } ^ { \mathrm { V I O } }$ . It can then be combined with the previous SLOAM pose $\mathbf { T } _ { k - 1 } ^ { \mathrm { S L O A M } }$ to form $\mathbf { T } _ { k } ^ { \mathrm { G U E S S } } = \mathbf { T } _ { k } ^ { \mathrm { R E L } } \cdot \mathbf { T } _ { k - 1 } ^ { \mathrm { S L O A M } }$ , which is used to initialize the SLOAM optimization.

Let the semantic map $\mathcal { M } \triangleq \{ \mathcal { L } _ { i } \} _ { i = 1 } ^ { N }$ with landmarks L be the set of all landmarks detected by SLOAM over the robot trajectory. For a new observation, we extract a submap $S _ { k } \subseteq { \mathcal { M } }$ by selecting from the semantic map a set of trees that are close to the current estimated position $\mathbf { T } _ { k } ^ { \mathrm { G U E S S } }$ of the robot based on a distance threshold $\omega .$ That is,

$$
S _ { k } = \{ \left\| \mathbf { c } ^ { m } - \mathbf { T } _ { k } ^ { \mathrm { G U E S S } } \right\| ^ { 2 } < \omega : \mathbf { c } ^ { m } \in \mathcal { M } \} .
$$

We store references to each tree in a KD-Tree indexed by their 2D position in the map coordinate frame to query the semantic map efficiently. SLOAM receives the submap $\boldsymbol { S _ { k } }$ , the initial guess $\mathbf { T } _ { k } ^ { \mathrm { G U E S S } }$ , and a LiDAR reading $\mathcal { P } _ { k } ^ { \mathrm { R O B } }$ in the robot frame. In this formulation, SLOAM will estimate T<sup>SLOAM</sup> by performing data association between the current LiDAR reading and the submap.

4) Lidar State Estimation: Let $\mathcal { P } _ { k } ^ { \mathrm { R O B } }$ be the LiDAR observation at the current time $t _ { k } .$ . To perform nearest-neighbor data association between the submap $S _ { k }$ given an arbitrary robot motion, we transform $\mathcal { P } _ { k } ^ { \mathrm { R O B } }$ using the initial guess $\mathcal { P } _ { k } ^ { \mathrm { G U E S S } } =$ $\mathbf { T } _ { k } ^ { \mathrm { G U E S S } } \cdot \mathcal { P } _ { k } ^ { \mathrm { R O B } }$ . All features and models from this new observation will already be in the initial guess frame.

Given a set of tree models $\mathcal { L } _ { k } \triangleq \{ \mathbf { c } _ { k } ^ { i } \} _ { i = 1 } ^ { N _ { k } }$ of size $N _ { k }$ , for each cylinder $\mathbf { c } _ { k } ^ { i } .$ , let $\mathcal { T } _ { k } ^ { i } \triangleq \{ \mathbf { p } _ { j } \} _ { j = 1 } ^ { \delta _ { i , k } }$ be the set of tree feature points of size $\delta _ { i , k }$ associated with the landmark. The features in $\mathcal { T } _ { k } ^ { i }$ are associated to the nearest cylinder $\mathbf { c } ^ { m }$ from the submap $\boldsymbol { S _ { k } }$ . The optimization objective is to minimize the point to cylinder distance of the associated features. Since the robot may not observe the entire tree trunk, the cylinder models are not constrained in the Z direction. For this reason, these models can constrain the X, Y, and Yaw motion. The cost function is given by

$$
\underset { \mathbf { T } _ { k } ^ { \mathrm { C Y L I N D E R } } } { \mathrm { a r g m i n } } \quad \sum _ { i = 1 } ^ { N _ { k } } \sum _ { j = 1 } ^ { \delta _ { i , k } } d _ { s } ( \mathbf { c } _ { j } ^ { m } , \mathbf { p } _ { j } ) .\tag{1}
$$

where $\mathbf { T } _ { k } ^ { \mathrm { C Y L I N D E R } } \mathrm { i s } \mathrm { a n } \mathbf { S } \mathbf { E } ( 3 )$ transformation with three degrees of freedom (translation Z, Pitch, and Roll are fixed).

$\mathcal { G } _ { k } \triangleq \{ \mathbf { p } _ { l } \} _ { l = } ^ { \gamma _ { k } }$ defines the set of ground features for the current keyframe. Each feature p<sub>l</sub> is associated to a ground model π<sub>l</sub> from the previous keyframe based on its euclidean distance to the plane centroid. The optimization objective is to find the transformation that minimizes the point-to-plane distance $d _ { \pi }$ Since the ground planes will always be below the robot, it can constrain the Z, pitch, and roll motion. The cost function is given by

$$
\underset { \mathbf { T } _ { k } ^ { \mathrm { G R O U N D } } } { \mathrm { a r g m i n } } \quad \sum _ { l = 1 } ^ { \gamma _ { k } } d _ { \pi } \big ( \pi _ { l } , \mathbf { p } _ { l } \big ) ,\tag{2}
$$

where $\mathbf { T } _ { k } ^ { \mathrm { G R O U N D } }$ is an SE(3) transformation with three degrees of freedom (translation X, Y and Yaw are fixed). Finally, the SLOAM pose $\mathbf { T } _ { k } ^ { \mathrm { S L O A M } }$ is given by

$$
\mathbf { T } _ { k } ^ { \mathrm { S L O A M } } = \mathbf { T } _ { k } ^ { \mathrm { G U E S S } } \cdot \mathbf { T } _ { k } ^ { \mathrm { G R O U N D } } \cdot \mathbf { T } _ { k } ^ { \mathrm { C Y L I N D E R } } .\tag{3}
$$

LeGO-LOAM [20] proposes a similar two-step optimization formulation, where the authors report 35% reduction in computation and similar accuracy when compared to estimating the full pose in one problem. LeGO-LOAM uses the output of the ground optimization to constrain the X,Y and Yaw parameter estimation. However, since we have an initial guess from the odometry, we solve (1) and (2) independently. Our primary motivation for splitting the optimization is to increase robustness in failure cases. This formulation can provide constraints to part of the robot pose even in observations where the object detection or data association partially fails.

A user-defined threshold controls the minimum number of feature-to-model matches required to perform the optimization. That is, if the number of ground features is low, we can set $\mathbf { T } _ { k } ^ { \mathrm { G R O U N D } } = I$ , where I is the identity transform. If not enough tree features are matched, we can set $\mathbf { T } _ { k } ^ { \mathrm { C Y L I N D E R } } = I .$ If both ground and tree features have few matches, SLOAM will use the initial guess to update the map.

## C. Perception

1) Semantic Mapping: Given a new pose estimate $\mathbf { T } _ { k } ^ { \mathrm { S L O A M } }$ SLOAM will project the models derived from the current sweep, and associate cylinder models to the ones in the submap using a cylinder-to-cylinder distance. These associations update the tree models to the latest estimate, and cylinders that do not have a match are added to the map.

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/c7786abc48c13634e5b9a0c15e92eace2fb9415c7dc78aa96122ef9b634a67b6.jpg)  
Fig. 4. Semantic map vs voxel map. The green cylinders show the semantic map, the black bounding box shows a local voxel map with a 0.1 m resolution, and the blue curve shows the SLOAM-estimated trajectory. It requires 1250 MB to store such a voxel map for a 1 km × 1 km region. However, storing individual tree models requires ∼2 MB for a dense forest of the same size.

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/17d2774463f5a9aaafc37d59396be9ef0a678f9b9294c3fda5b9e2073593bff4.jpg)  
Fig. 5. Visualization of real-time perception and decision making of the autonomy stack in a real-world forest. The green cylinders are tree models estimated by SLOAM, the small bounding box is the local voxel map, and the black line shown at the top of the middle panel is the edge of the global voxel map (1 km × 1 km × 10 m). The left panel shows a zoomed-in view of the SLAM and odometry reference frames. The VIO and SLOAM estimated poses are used to calculate a transform between the two reference frames, which compensates the VIO drift. In the right panel, the drift compensation is used by the planning module to output trajectories that guide the UAV to execute its mission accurately.

2) Semantic Segmentation on Point Clouds: The original SLOAM [16] implementation used a lightweight neural network to perform semantic segmentation at LiDAR frequency on the CPU. However, we observed that the segmentation would not be accurate along object edges, considering leaves and small branches as part of the trunk. As explained in section IV-B2, this is mitigated with the Trellis Graph instance detection. However, it is still challenging to filter these unwanted points that add noise to the cylinder model and, consequently, state estimation. Without the need to run inference on every scan, we improve SLOAM by using RangeNet++ [17], a more robust architecture designed for LiDAR data.

## D. Planning and Control

Our previous efforts [1], [15] focused on fast autonomous flight in cluttered and GPS-denied environments. However, to autonomously acquire data in dense under-canopy environments that are significantly more cluttered, unstructured, and larger in scale, various challenges need to be addressed. First, we introduce a hierarchical planning and mapping framework to reduce computation and leverage the sparsity of the semantic map to alleviate memory demand. Second, we introduce a real-time drift compensation module to handle the odometry drift correction by semantic SLAM, while maintaining the planner optimality and the controller stability.

1) Hierarchical Planning and Mapping Framework: We introduce a multi-level mapping and planning framework as illustrated in Fig. 5. Our top-level map consists of semantic models that helps the UAV to accurately localize itself over long distances and keep track of the information of interest in an efficient manner. Our mid-level (global) map is a large but coarsely discretized voxel map. Our low-level (local) map is a small but finely discretized, robot-centric voxel map, and is updated at the LiDAR rate (10 Hz). The high level planner uses a boustrophedon-based coverage planning algorithm [19] to cover the region of interest defined by the user using one polygon to denote the boundary, and additional polygons to denote obstacles or uninterested regions. The user also needs to specify the sensing radius and overlap ratio. The mid-level global planner uses the jump point search algorithm to generate the shortest collision-free and long-horizon path to the next coverage waypoint $g _ { i }$ . Each waypoint is parameterized by the quadrotor’s flat outputs [21], i.e., $\mathbf { \Psi } g _ { i } = [ \bar { x } _ { i } , y _ { i } , z _ { i } , \psi _ { i } ] ^ { \mathbf { T } }$ where $\mathbf { x } _ { i } = [ x _ { i } , y _ { i } , z _ { i } ] ^ { \mathbf { T } }$ are the 3D positions of the quadrotor’s center of mass, and ψ is the yaw angle.

The local planner uses a motion-primitive-based algorithm, where the 3-rd order derivative of positions (i.e., jerk) in quadrotor’s flat outputs are used as control inputs. This algorithm can generate safe and dynamically-feasible trajectories. We refer to the original work [22] for more details. The local goal is generated by finding the first intersection between the global path and the local map boundaries (the global goal will be used if it lies inside the local map). The planner only needs to reach the local goal within a position tolerance of 4 m, and the velocity is not specified unless the UAV is close to the final waypoint $g _ { n }$ This avoids unnecessary acceleration and deacceleration, and reduces the replan computation cost.

2) Trajectory Tracking and Control: To speed up the replanning process, our trajectory tracker assumes that the UAV can track the planned trajectory tightly. The assumption is valid since our trajectory is dynamically feasible. By making this assumption, we can reuse the motion primitive graph built during the previous re-plan steps. Since the local planner is running at 2 Hz, the updates in the local map and local goal are usually small. Thus, the planner only needs to expand a small number of additional nodes to reach the new goal, significantly reducing the re-plan time. In addition to tracking the position outputs, if yaw alignment is turned on, the trajectory tracker will use a constant ψ<sup>˙</sup> to align $\psi$ with the direction of displacement on the x-y plane. For every time step t, the trajectory tracker outputs $\mathbf { s } _ { t } \mathbf { \bar { \Gamma } } = [ \mathbf { x } _ { t } ^ { \mathbf { T } } , \dot { \mathbf { x } } _ { t } ^ { \mathbf { T } } , \ddot { \mathbf { x } } _ { t } ^ { \mathbf { T } } , \ddot { \mathbf { x } } _ { t } ^ { \mathbf { T } } , \psi _ { t } , \dot { \psi _ { t } } ] ^ { \mathbf { T } }$ . s is calculated at 200 Hz. Under extreme circumstances, such as large external disturbance, the UAV may fail to track the trajectory tightly. Thus, the distance d from the UAV to the trajectory is constantly monitored. When $d > = 0 . 5$ m, the state machine will automatically trigger the stopping policy. The stopping policy calculates $\mathbf { s } _ { \underline { { t } } }$ using the maximum allowed x and $\ddot { \psi } ,$ which brings x˙ and $\dot { \psi }$ to zero as soon as possible while keeping x¨ within the maximum values. When the UAV safely stops, the state machine will start a new re-plan process.

A nonlinear geometric-based controller [23], which has been demonstrated to have good tracking performance for fast quadrotor flights [15], is used as the position controller. Based on $\mathbf { s } _ { t }$ and the odometry, it will calculate the desired thrust, orientation, and body rate at 200 Hz. The Pixhawk 4 flight controller takes in these values and runs an onboard filter and orientation controller to calculate the motor speeds.

3) Real-Time Drift Compensation Module: A naive approach to correct the drift induced by the VIO is to feed the semantic SLAM estimated UAV poses directly into the planner and controller. However, unlike the filtering-based VIO that gives smooth pose estimates, the semantic SLAM estimated poses may not be locally smooth (due to, e.g., loop closures). This will cause the UAV to deviate from the existing trajectory. In such scenario, the motion primitive graph can no longer be reused, thus resulting in a much longer re-plan time. Even if the re-planning process is triggered when this happens, the new trajectory will not be immediately available due to the re-plan time. Such delay will cause the UAV to have no feasible trajectory to track, thus resulting in dangerous behaviors.

A more reliable solution is to maintain separate SLAM and odometry reference frames. Given a SLOAM pose T<sup>SLOAM</sup>, the odometry drift is calculated by $\mathbf { T } _ { k } ^ { \mathrm { D R I F T } } = \mathbf { T } _ { k } ^ { \mathrm { S L O A M } } \cdot \mathbf { \Phi } ^ { \kappa } ( \mathbf { T } _ { k } ^ { \mathrm { V I O } } ) ^ { - 1 }$ The inverse of the drift, which is the transformation from the semantic SLAM estimated pose to the VIO estimated pose, is applied on the odometry reference frame.

The global goal, given by a waypoint $g \in { \mathcal { G } }$ along the coverage path, lies in the SLAM reference frame and is transformed into the odometry reference frame $g ^ { \mathrm { O D O M } } = ( \mathbf { T } _ { k } ^ { \mathrm { D R I F T } } ) ^ { - 1 } \cdot g ^ { \mathrm { S L O A M } }$ Using $g ^ { \mathrm { O D O M } }$ the global planner will output a global path that is drift compensated when re-planning. This compensation leads the local planner to generate a drift-compensated local trajectory, while tightly tracking the existing trajectory. The optimality of the local trajectory planner is also maintained. Thus, the trajectory and control commands will guide the UAV to execute its long-range mission accurately without frequent stops. This mechanism also allows our autonomous flight system to directly work with other SLAM or even GPS estimated poses.

## V. RESULTS AND ANALYSIS

Our system is first developed and tested in simulated environments using the Unity engine,<sup>1</sup> and then demonstrated and improved in real-world forests. The simulation is especially important for training and testing our LiDAR-based semantic segmentation module as well as the behaviour ofthe autonomous navigation module without the risk of damaging the UAV. We conduct real-world experiments at the Wharton State Forest, a natural forest located in the U.S. state of New Jersey. It consists of approximately 122,880 acres of pinelands as shown in Fig. 2. There is significant variability in tree densities, sizes, shapes, and undergrowth conditions, thus making this forest ideal for verifying our system’s robustness and generalizability.

An important prerequisite is to train the segmentation neural network. We train RangeNet++ on simulator data and then finetune on real-world data. It is worth noting that we do not use any labeled data from the Wharton State Forest for training. Instead, we use data collected in a different pine forest in Arkansas, US. We label 50 LiDAR scans to fine-tune the model, 37 for training and 13 for validation. An example of the segmentation result is shown in Fig. 5. We use this model across all experiments without any re-training. It runs at 2 Hz using up to 3 threads of the 12-thread i7-10710 U processor of the onboard Intel NUC 10 computer. This is sufficient since semantic segmentation is performed only on keyframes.

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/30550d6a3923f234b2a92789d950dd893852117e27ac9b44a85792d243568c38.jpg)  
Fig. 6. Large-scale coverage experiment. The upper left panel shows the overhead LiDAR data of a 0.77 $\mathrm { k m ^ { 2 } }$ forest stand that is used to create the simulated environment. The upper right panel shows the automatically generated coverage plan based on a user-defined coverage region. The UAV is able to finish this mission autonomously within 1 h, without any human intervention. The trajectory length is ∼10 km and the average flight speed is ∼2.9 m/s. The lower left panel shows the semantic map generated by SLOAM consisting of 10,897 tree models, where the semantic segmentation uses the same model as we use for the physical experiments. The lower right panel shows a close-up of the semantic map.

## A. Simulation Experiments and Analysis

We set up an automatic pipeline that allows us to generate simulated environments based on real-world tree positions that we collected during field trips. We use various high-fidelity tree models that are available in the Unity asset store and customize their sizes, shapes, orientation, as well as undergrowth conditions to simulate the real-world forests.

In the first simulated experiment, we perform large-scale coverage. Using LiDAR data from an overhead flight, we detect trees with a Local Maximum Filter on the z axis, i.e., we assume that the trees’ crowns are the highest observed objects in a forest, and all local maxima are considered tree detections. These detections are used to position the tree models in our simulated environment. This environment covers around 0.77 $\mathrm { k m ^ { 2 } }$ (190 acres) and contains 10,692 trees. Using the overhead data as a reference, the user draws polygons to specify the region of interest, and the coverage algorithm will then compute a coverage path as an ordered set ofwaypoints G. This is illustrated in Fig. 6. We run SLOAM with no pose noise to generate a semantic map as the robot flies for ∼10 km autonomously. The final tree cylinder count is 10,897.

The second set of experiments measures the impact of environment density on the autonomous flight stack, providing insight to the system’s potential to perform fast flights as well as the design considerations of autonomous UAVs. We use tree positions from a real forest and generate ten simulated forests with the same distribution but different densities (by scaling up or down the x and y positions). Taking the UAV and tree trunks’ diameters into account, the average gap between trees for the densest forest is 2.19 meters. These simulated experiments are performed with the maximum velocity set as 11.0 m/s and without pose estimation noise. Four missions with a combined trajectory length of ∼1 km are sent to the robot for each environment.

From Fig. 7(a), there is a clear trend showing that the average velocity decreases with increasing tree density. Fig. 7(b) shows that the actual distance traveled by the UAV increases with tree density due to obstacle avoidance. This, combined with the decrease of average velocity, leads to a more drastic increase in time to complete mission, as shown in the red curve. These experiments not only show our autonomous flight system’s fast flight capabilities, but also indicate that the difficulty of accomplishing flights with a limited battery life increases rapidly with the density of the environment. Moreover, under a dense canopy, the UAV’s average velocity is limited by the environment instead ofits maximum design velocity. Thus, flight time is the dominant factor for the mission range. This supports our choice of using a Li-Ion battery over a Li-Po battery because the former has higher specific energy (Wh/kg).

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/8db323ae96cb0af2ccb1e0db36afc545da96d8294e3bafa7b0144f0ae791e3ce.jpg)  
(a) Average flight velocity

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/fd7b04cb9723d96e304eca31764e06c9bae574b7a841166037eb143e4c2a099f.jpg)  
(b) Travel distance and execution time

Fig. 7. Autonomous flight system performance v.s. tree density. 10 environments with varying densities are used for this set of experiments. For each environment, 4 flight missions are executed. These missions’ total straight-line length is ∼1 km. On the left panel, the purple curve shows that average flight velocity decreases as the tree density increases. On the right panel, the blue curve shows that the distance traveled by the robot increases as the tree density increases. These two combined lead to a more drastic increase in time to complete the mission, as shown by the red curve on the right.  
![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/e6290da1fddb2014489ba5c2154950fc230a8d15770908c252df4b46e30b8e74.jpg)

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/4dbd5defde3f39542b8267aa3a384c0fe52ca9b23b8fb1ec1d298b48f10da80a.jpg)  
Fig. 8. Top-down view of the 800 m trajectory. In this experiment, the autonomy stack relies only on VIO to control the UAV. The objective is to perform two square loops and return home. While according to VIO the robot is successful, both SLOAM and noisy GPS show that the UAV drifts significantly. The GPS sensor reported a ∼10 m standard deviation for X and Y position estimates. However, as the GPS does not drift over time, it provides a useful high-level reference of the global position. Therefore, here we use it for qualitative evaluation.

## B. Real-World Experiments and Analysis

In the first field experiment, our goal is to show that relying only on VIO will lead to failure in completing the mission. As illustrated in Fig. 8, the autonomy stack relies only on VIO to control the UAV. The commanded mission is to perform two square loops of ∼800 m and return home. According to VIO, the robot is successful, but according to SLOAM and GPS, the VIO drifts significantly (∼60 m). SLOAM and GPS agree with each other within a 10∼20 m margin. A possible reason why VIO drifts significantly in this experiment is that the UAV’s motion is smooth and close to a constant velocity state, which may have formed degenerate cases for VIO [24].

In the second field experiment, we demonstrate that the VIO drift can be corrected with semantic SLAM. To quantify the drift, we manually pilot the UAV so that the takeoff and landing positions are exactly the same. As shown in Fig. 9, the trajectory is a loop of ∼1.1 km, covering most of $\mathrm { ~ a ~ } 2 0 0 \ m ^ { 2 }$ area while detecting and modeling 1157 trees. The GPS was not able to get a lock during this experiment. Therefore, we cannot include it in the results of this test.

![](images/2022_Large-Scale_Autonomous_Flight_With_Real-Time_Semantic_SL/c76acd69872995aabdf37084660919008d3267f623fd3550a2dc8139706693ae.jpg)  
Fig. 9. Top-down view of the 1.1 km trajectory. In this experiment, the UAV is manually piloted to guarantee returning to take-off position, so that the drift can be quantified. The black dots represent the tree trunks detected by our mapping algorithm.

TABLE I  
DISTANCE OF THE FINAL ESTIMATED POSE FROM THE BEGINNING OF THE TRAJECTORY ON THE 1.1 KM LOOP
<table><tr><td>Method</td><td>XY Drift (m)</td><td>Z Drift (m)</td><td>Total Drift (m)</td></tr><tr><td>VIO</td><td>7.92</td><td>-6.27</td><td>10.10</td></tr><tr><td> $\mathrm { \underline { { S L O A M + V I O } } }$ </td><td>3.93</td><td>0.71</td><td>3.99</td></tr></table>

In Table I, we compare the accumulated drift based on the starting and ending positions of the pose estimated by VIO and SLOAM + VIO. Although stereo VIO outputs consistent local estimates, it is unreliable over long-range flights, especially along the Z direction. Directly using such estimates will lead to the robot deviating from the desired trajectory. SLOAM benefits from the local consistency of VIO but can reduce the drift by using landmarks and the ground model. Most of the Z-axis drift is corrected by using our semantic LiDAR framework, and the XY drift is reduced. Other LiDAR state estimators such as LIO-SAM [25] and Fast-Lio2 [26] were also tested, but due to a LiDAR connector malfunction that caused intermittent data loss, these methods failed. This is also a good example of why having sensor redundancy is key to long-range operation, since a hardware malfunction such as this can happen anytime to a robot system.

## C. Discussion

Navigating in forests is difficult, especially when the density is high. Even with the same planner configuration, the average speed, mission time and length vary significantly. There is a tight connection between perception and the decision-making loop. If there is drift in state estimation, the planning will deviate and fail to execute the mission; additionally, the state estimation results can be influenced by the UAV’s motion.

Traditional odometry systems are good at outputting smooth estimation over a moderate distance, but semantic SLAM can reduce drift for longer flights or different environment densities. Therefore, using semantics to compensate for the drift in VIO results in greater robustness and reliability. Moreover, different sensors have different degenerate cases and failure modes.

Forest-like environments aggravate this due to perception aliasing, excessive heat, humidity, sunlight, and dust. Therefore, sensor redundancy can increase reliability.

A Smoothing and Mapping (SAM) framework such as the one proposed by LIO-SAM [25] can be extended to leverage our semantic map and could further reduce the global drift from SLOAM. However, the computation demand for handling loop closures with large maps is high. Thus, the use of SAM at for large-scale semantic maps is left as future work.

## VI. CONCLUSION AND FUTURE WORK

Generating semantic maps offorests and orchards is important for understanding the magnitude of the carbon challenge and to develop new strategies for precision agriculture. Semantic mapping requires under-canopy flight and measurements at the individual tree or plant level. In this paper, we described the key hardware and software components required for an autonomous UAV to build large-scale maps of unstructured forests in a GPSchallenged environment with varying illumination (sunlight, shadows) and wind, without any human intervention. These include the use of cameras, an IMU, and a LiDAR for state estimation; the segmentation of trees and ground planes using a LiDAR; and the real-time integration of the semantic map in the control and planning loop to minimize drift and to facilitate localization, mapping, and planning. To our knowledge, this is the first time that real-time semantic SLAM has been integrated into the feedback loop for autonomous UAV navigation, and detailed, multi-resolution maps over a large area have been built in an automated fashion, using only onboard sensing and computation. We hope our work can inspire future research in not only SLAM but also aerial autonomy. Future work involves incorporating the semantic map to actively choose actions that reduces uncertainty of the information gathered by the system.

## ACKNOWLEDGMENT

The authors would like to thank the Distributed and Collaborative Intelligent Systems and Technology Collaborative Research Alliance (DCIST) team for building the simulation infrastructure, and New Jersey State Forestry Services staff for supporting experiments in the Wharton State Forest and also would like to thank Dr. Avraham Cohen for the original design of Falcon 4 platform, Dr. Ke Sun and Dr. Kartik Mohta for their insights and help regarding autonomous flight system development and field experiments, and Dr. Luiz Rodriguez for the over-canopy LiDAR data.

## REFERENCES

[1] K. Mohta et al., “Experiments in fast, autonomous, gps-denied quadrotor flight,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 7832–7839.

[2] M. Quigley et al., “The open vision computer: An integrated sensing and compute system for mobile robots,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 1834–1840.

[3] J. Carreiras, J. B. Melo, and M. J. Vasconcelos, “Estimating the aboveground biomass in miombo savanna woodlands ( Mozambique, east africa) using l-band synthetic aperture radar data,” Remote Sens., vol. 5, no. 4, pp. 1524–1548, 2013.

[4] S. L. Bowman, N. Atanasov, K. Daniilidis, and G. J. Pappas, “Probabilistic data association for semantic slam,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 1722–1729.

[5] I. Kostavelis and A. Gasteratos, “Semantic mapping for mobile robotics tasks: A survey,” Robot. Auton. Syst., vol. 66, pp. 86–103, 2015.

[6] L. Nicholson, M. Milford, and N. Sünderhauf, “Quadricslam: Dual quadrics from object detections as landmarks in objectoriented slam,” IEEE Robot. Automat. Lett., vol. 4, no. 1, pp. 1–8, Jan. 2019.

[7] S. Yang and S. Scherer, “Cubeslam: Monocular 3-d object slam,” IEEE Trans. Robot., vol. 35, no. 4, pp. 925–938, Aug. 2019.

[8] H. Bavle, P. De La Puente, J. P. How, and P. Campoy, “Vps-slam: Visual planar semantic slam for aerial robotic systems,” IEEE Access, vol. 8, pp. 60 704–60 718, 2020.

[9] V. Murali, H.-P. Chiu, S. Samarasekera, and R. T. Kumar, “Utilizing semantic visual landmarks for precise vehicle navigation,” in Proc. IEEE 20th Int. Conf. Intell. Transp. Syst., 2017, pp. 1–8.

[10] K. Ok, K. Liu, K. Frey, J. P. How, and N. Roy, “Robust object-based slam for high-speed autonomous navigation,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 669–675.

[11] H. Bavle, S. Manthe, P. De La Puente, A. Rodriguez-Ramos, C. Sampedro, and P. Campoy, “Stereo visual odometry and semantics based localization of aerial robots in indoor environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 1018–1023.

[12] Y. Lin et al., “Autonomous aerial navigation using monocular visualinertial fusion,” J. Field Robot., vol. 35, no. 1, pp. 23–51, 2018.

[13] H. Oleynikova et al., “An open-source system for vision-based microaerial vehicle mapping, planning, and flight in cluttered environments,” J. Field Robot., vol. 37, no. 4, pp. 642–666, 2020.

[14] X. Zhou, Z. Wang, H. Ye, C. Xu, and F. Gao, “Ego-planner: An esdf-free gradient-based local planner for quadrotors,” IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 478–485, Apr. 2021.

[15] K. Mohta et al., “Fast, autonomous flight in gps-denied and cluttered environments,” J. Field Robot., vol. 35, no. 1, pp. 101–120, 2018.

[16] S. W. Chen et al., “Sloam: Semantic lidar odometry and mapping for forest inventory,” IEEE Robot. Automat. Lett., vol. 5, no. 2, pp. 612–619, Apr. 2020.

[17] A. Milioto, I. Vizzo, J. Behley, and C. Stachniss, “RangeNet: Fast and accurate LiDAR semantic segmentation,” in Proc. IEEE/RSJ Intl. Conf. Intell. Robots Syst., 2019, pp. 4213–4220.

[18] K. Sun et al., “Robust stereo visual inertial odometry for fast autonomous flight,” IEEE Robot. Automat. Lett., vol. 3, no. 2, pp. 965–972, Apr. 2018.

[19] R. Bähnemann, N. Lawrance, J. J. Chung, M. Pantic, R. Siegwart, and J. Nieto, “Revisiting boustrophedon coverage path planning as a generalized traveling salesman problem,” in Field Serv. Robot., 2021, pp. 277–290.

[20] T. Shan and B. Englot, “Lego-loam: Lightweight and ground-optimized lidar odometry and mapping on variable terrain,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 4758–4765.

[21] D. Mellinger and V. Kumar, “Minimum snap trajectory generation and control for quadrotors,” in Proc. IEEE Int. Conf. Robot. Automat., 2011, pp. 2520–2525.

[22] S. Liu, K. Mohta, N. Atanasov, and V. Kumar, “Search-based motion planning for aggressive flight in SE(3),” IEEE Robot.Automat. Lett., vol. 3, no. 3, pp. 2439–2446, Jul. 2018.

[23] T. Lee, M. Leok, and N. H. McClamroch, “Geometric tracking control of a quadrotor uav on se (3),” in Proc. 49th IEEE Conf. Decis. Control, 2010, pp. 5420–5425.

[24] Y. Yang, P. Geneva, K. Eckenhoff, and G. Huang, “Degenerate motion analysis for aided ins with online spatial and temporal sensor calibration,” IEEE Robot. Automat. Lett., vol. 4, no. 2, pp. 2070–2077, Apr. 2019.

[25] T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and R. Daniela, “Liosam: Tightly-coupled lidar inertial odometry via smoothing and mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5135–5142.

[26] W. Xu and F. Zhang, “Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter,” IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 3317–3324, Apr. 2021.

[27] “The Internet of Things for precision agriculture (IoT4Ag), an NSF engineering research center”. [Online]. Available: https://iot4ag.us