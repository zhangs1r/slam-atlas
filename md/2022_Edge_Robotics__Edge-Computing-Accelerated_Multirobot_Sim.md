# Edge Robotics: Edge-Computing-Accelerated Multirobot Simultaneous Localization and Mapping

Peng Huang , Graduate Student Member, IEEE, Liekang Zeng , Graduate Student Member, IEEE, Xu Chen , Senior Member, IEEE, Ke Luo , Graduate Student Member, IEEE, Zhi Zhou , Member, IEEE, and Shuai Yu , Member, IEEE

Abstract—With the wide penetration of smart robots in multifarious fields, the simultaneous localization and mapping (SLAM) technique in robotics has attracted growing attention in the community. Yet collaborating SLAM over multiple robots still remains challenging due to performance contradiction between the intensive graphics computation of SLAM and the limited computing capability of robots. While traditional solutions resort to the powerful cloud servers acting as an external computation provider, we show by real-world measurements that the significant communication overhead in data offloading prevents its practicability to real deployment. To tackle these challenges, this article promotes the emerging edge-computing paradigm into multirobot SLAM and proposes RecSLAM, a multirobot laser SLAM system that focuses on accelerating the map construction process under the robot–edge–cloud architecture. In contrast to the conventional multirobot SLAM that generates graphic maps on robots and completely merges them on the cloud, RecSLAM develops a hierarchical map fusion technique that directs robots’ raw data to edge servers for real-time fusion and then sends to the cloud for global merging. To optimize the overall pipeline, an efficient multirobot SLAM collaborative processing framework is introduced to adaptively optimize robot-to-edge offloading tailored to heterogeneous edge resource conditions, meanwhile ensuring the workload balancing among the edge servers. Extensive evaluations show RecSLAM can achieve up to 39.31% processing latency reduction over the state of the art. Besides, a proof-of-concept prototype is developed and deployed in real scenes to demonstrate its effectiveness.

Index Terms—Distributed and parallel processing, edge intelligence, edge offloading, multirobot laser simultaneous localization and mapping (SLAM).

## I. INTRODUCTION

S A KEY-ENABLING technology in robotics, simultaneous localization and mapping (SLAM) is a

Manuscript received 1 October 2021; revised 24 December 2021; accepted 19 January 2022. Date of publication 26 January 2022; date of current version 25 July 2022. This work was supported in part by the National Science Foundation of China under Grant U20A20159 and Grant 61972432; in part by the Program for Guangdong Introducing Innovative and Entrepreneurial Teams under Grant 2017ZT07X355; and in part by the Pearl River Talent Recruitment Program under Grant 2017GC010465. Part of the results in the work has been presented in IEEE/CIC International Conference on Communications in China (ICCC) 2021 [DOI: 10.1109/ICCC52777.2021.9580413]. (Peng Huang and Liekang Zeng contributed equally to this work.) (Corresponding author: Xu Chen.)

computationally intensive approach that targets at simultaneously constructing a graphic map and tracking the agent’s location in an unknown environment [1]–[3]. It has been broadly used across a number of applications, opening a wide door to smart robots for understanding and interacting with environments wherever indoor, aerial, or underground. For example, in autonomous robotics, SLAM has been employed to process the surrounding information from laser scans to guide the self-contained navigation [4]–[7]. In disaster relief, snake-like robots apply SLAM to explore fragile buildings and carry out a rescue in human-unreachable places [8]–[10].

Generally, SLAM can be categorized into two classes according to the used sensors, namely, visual SLAM [11] and laser SLAM [12]. Visual SLAM utilizes the images captured by cameras to reckon geometric graphics. It strives to exploit the 2-D visual input to construct 3-D semantic models. Laser SLAM forks another technical path that extracts data from laser scans to shape a grid occupancy map. Particularly, laser data differs from visual images by containing depth-wise information and high-precision positions of objects. Benefited by such advantage in sources, laser SLAM is preferred and employed for a much wider spectrum of robotic applications for achieving much higher physical accuracy [13].

The widespread use of smart robots catalyzes SLAM to extend its deployment over multiple robots [14]–[16]. Specifically, a group of robots is deployed in a swarm manner, and they are committed to cooperating with each other in an unknown environment in order to construct the geometric mapping and determine their own locations through SLAM. An example scenario is to build the graphic maps of an apartment as shown in Fig. 1, where multiple robots are employed to perform SLAM simultaneously for global map construction. Another example is in mission-critical search and rescue (SAR) tasks [14], [15], where multiple robots are driven collaboratively to share map data with each other for rapid exploration in broken buildings. In these circumstances, traditional wisdom [17]–[19] to reconcile the distributed data sources calls for robot–cloud synergy, where robots run SLAM individually and upload their local map mutually to a centralized cloud server for global merging. Yet the two-tier architecture suffers from dual drawbacks. On the one hand, processing SLAM workloads locally on commodity robots typically demand plenty of latency and can even fail to complete under resource constraints, e.g., limited memory capacity. In our measurements on an ordinary robot (as will be shown in Section II-B), running SLAM can force CPU workload at a high level consistently (always >83.60%), leading to excessive latency in on-device execution. Such significant and excessive latency impedes many multirobot applications (e.g., disaster relief and high-resolution autonomous navigation) that generally demand real-time information fusion. On the other hand, transferring robots’ local maps of a large volume to the remote cloud through the unreliable and delay-significant wide-area Internet connection not only incurs the communication bottleneck but raises users’ concerns on security and privacy. Quantitatively, as we will show later, the data uploading time overhead can dominate the whole processing in common 4G, 5G and WiFi networks, exhibiting vulnerable performance.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/d3542b3173fedcbd65c5a1f923c3bc15d2f54179f7e54ed0b5dc6ba85cbda7e8.jpg)  
(a)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/fb6de66ed5ce22f679e9a1523fcf16f59b30a1d4c61e1da13c6cd2b65f4c25bc.jpg)  
(b)  
Fig. 1. Example multirobot SLAM application at the edge, where the robots are navigated to walk in the apartment (a) and perform SLAM to construct the corresponding geometric indoor map (b).

To remedy these limitations, this article intends to leverage the emerging edge-computing [20]–[22] paradigm to perform multirobot laser SLAM in low latency. Instead of relying on geographically distant data centers, edge computing concentrates on utilizing vicinal computing resources (e.g., 5G/WiFi edge severs) in physical proximity to end devices and, therefore, remarkably shortens data communication distance, lowers offloading transmission delay, and allows the advanced quality of services [23], [24]. Nevertheless, enabling multirobot SLAM with edge-computing architecture is nontrivial with three-fold challenges. First, the spatially distributed nature of multirobot deployment requires a unified flow for data collection, organization, and computation due to the versatile sources as input. For instance, to collect data from two physically isolated robots, the edge server needs to identify their source location, timestamp, and sensory format (e.g., raw data from whether laser scan or inertial measurement unit (IMU) sensors). Second, distributing the map fusion, which is traditionally centralized processed, to multiple edge servers desires a retrofit on handling SLAM maps, which is intractable under resource dynamics. Third, simultaneously orchestrating the computation and communication among multiple robots and multiple edge servers presents inherent complexity in formulation and optimization on SLAM execution and data transmission. Furthermore, the network dynamics among robots and edge servers further complicate the problem.

To tackle these challenges, this article proposes RecSLAM, a system built upon the Robot–edge–cloud architecture to enable multirobot laser SLAM in low-latency services.

RecSLAM’s design is motivated by the observations on realworld measurements that: 1) migrating SLAM workloads from robots to edge servers can effectively augment the robots’ processing capability and 2) preparative merging a subset of local maps at the edge can shrink the sizes of data to be uploaded to the cloud and, therefore, further reduces communication costs. By exploiting them, RecSLAM considers a hierarchical robot– edge–cloud pipeline, where each robot individually collects data and selectively transfers them to one of the edge servers. The edge servers commit to performing SLAM on their received robot data, merging the corresponding local maps, and uploading the preparative fusion results to a dedicated cloud server for global map fusion. An adaptive coordinator (i.e., task scheduler) is further developed to optimize the data flow between robots and edge servers, aiming at minimizing the end-to-end latency. We implement RecSLAM on both simulation platforms and real robots. Simulation results show that our system can outperform the state-of-the-art solutions by up to 39.31% latency reduction. The proof-of-concept prototype in an indoor scene corroborates its feasibility and efficiency, demonstrating the promising advantage of edge-accelerated SLAM. In summary, we make the following key contributions.

1) We conduct a fine-grained investigation on the processing costs of existing SLAM processing solutions. The robotics-based measurements reveal that the cloud offloading mechanism suffers from the considerable transmission latency in sensory data uploading, while the local execution falls short at efficiency due to the limited onboard computing resource.

2) We propose RecSLAM, a collaborative SLAM system based on the hierarchical robot–edge–cloud architecture to enable real-time SLAM serving. RecSLAM decouples the conventional SLAM pipeline to distribute them to multiple edge servers, where each robot can selectively offload their raw frames to one of the edge servers and the edge server can fuse multiple frames ahead of global merging. By extending the centralized computation to distributed and parallel processing, RecSLAM significantly improves the utilization of edge resources.

3) We develop a novel graph-based framework to optimize the collaborative SLAM processing between robots and edge servers. Specifically, we separate the workflow into two stages, namely, robot data grouping and edge offloading. For robot data grouping, we abstract an undirected graph to describe the distributed robot data processing issue with workload balancing among the edge servers, and apply an efficient balanced graph partitioning algorithm to make a balanced robot data grouping for offloaded processing at the edge. For edge offloading, we devise an efficient resource-aware collaborative processing strategy to adaptively offload the grouping data from the robots to the proper edge servers, in order to minimize the total processing latency.

4) We implement and evaluate RecSLAM in both simulation and realistic deployment. The simulation on the Gazebo [25] platform demonstrates the effectiveness of RecSLAM and its collaborative processing algorithms, showing up to 39.31% latency speedup upon the cloud offloading approach. The realistic prototype on three robots in an indoor experimental scene verifies its feasibility and validity in rendering efficient SLAM services for edge applications.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/7647f2af5c8a604cb944dd9c873b82e8535061e07f989e384e823ea72eba4087.jpg)  
Fig. 2. Illustration of a typical SLAM computation. The example scenario covers several robots that run simultaneously to sketch an indoor map for the apartment, where each robot is represented by a dot and its scan field is abstracted as a sector. A robot walks intentionally to acquire the environment’s laser scan data, in the form of a floating-point vector, and passes it to the local SLAM computation module to generate a geometric grid map.

The remainder of this article is organized as follows. Section II characterizes the existing multirobot SLAM solution. Sections III–V present the design, optimization, and implementation of the proposed system. Section VI evaluates in terms of both simulation and prototype. Section VII reviews related works and Section VIII concludes.

## II. BACKGROUND AND MOTIVATION

This section dives into multirobot SLAM processing by characterizing the state-of-the-art cloud-based solution. We first briefly introduce its workflow, and next break down the componentwise performance, which reveals the opportunities and challenges of edge computing.

## A. Multirobot SLAM

SLAM is typically run at a robot with various sensors equipped, aiming at simultaneously localizing the robot itself and maintaining a graphic map of the environment [26]. Fig. 2 shows a typical scenarios of SLAM computation. The input to SLAM is the sensory data from robots in a form of floating-point vectors, which describe the scanning angle, laser distance, etc. The output is an occupancy grid map, where the black grids are obstacles, the whites represent available safe space and the grays mean the places that have not been detected or uncertain.

Multirobot SLAM is an extended use case of single-robot SLAM, which manages a cluster of robots to perform SLAM for constructing a virtual graphic sense corresponding to reality [27]. Its core procedure beyond single-robot SLAM is the map fusion procedure that merges multiple grid maps to yield a global map of the targeted scene. Given that grid maps are collected from multiple distributed robots, state-of-the-art solutions [17]–[19] resort to the centralized cloud server to fulfill map fusion, as illustrated in Fig. 3. Particularly, it works in two stages. First, each robot collects sensory data and individually performs SLAM locally. Next, they upload their local SLAM result, i.e., grid maps, to the cloud, where all these maps are merged to acquire a global map. Specifically, two local maps that share an overlapping area will be fused, while those independent ones will be simply appended to constitute the global map. This global map can be used for downstream tasks, such as navigation and furniture design.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/90aa1d6ca06fe4a5c1da2f411008714af837f88f219a685a0a0bcea7968e16b6.jpg)  
Fig. 3. Pipeline of the traditional cloud-based multirobot SLAM solution, where the robots performs SLAM individually and upload their data via the Internet to a centralized cloud server for global map fusion.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/ee31c0f6b21dcb70f368fa0960440352e1432bb6fa9120d57485512b4b576932.jpg)  
Fig. 4. Latency breakdown of a cloud-based multirobot SLAM solution. The communication latency contributes a major portion out of the total and is sensitive to the networking condition.

Multirobot SLAM has been widely adopted in a broad range of scenarios. For example, some indoor mapping services employ multiple robots to scan the house and perform mul tirobot SLAM to construct a 3-D graphic model that shapes the indoor environment [15]. In some geographical mapping applications, many robots are run to scan landforms and build topographic maps [17].

## B. Performance Implications of State-of-The-Art

State-of-the-art cloud-based solutions highly rely on the Internet to gather local frames from distributed robots, which makes it sensitive to the vulnerable network. To make a clearer understanding of how the network conditions impact, we explicitly examine the costs of a typical cloud serving process. Specifically, we deploy a multirobot SLAM prototype using three Turtlebot3 and a cloud server, aiming at measuring the duration from laser scan input at robots to a global map obtained on the cloud. The Turtlebot is equipped with Raspberry Pi as the processing core, which has 1.4-GHz ARM Cortex-A53 CPUs, 1-GB LPDDR2 without GPU. The cloud server is with Intel Xeon CPU E5-2678, deployed at the available region that robots locate. The robots contact the cloud via three channels: 3G, 4G, and WiFi, all are under commercial operation networks.

Fig. 4 visualizes the measurements, where the latency breaks down in robot computing, frames uploading, and cloud merging. We can witness that the total latency is highly sensitive to the channel switching with an increase from about 3.83 (WiFi) to 5.36 s (5G). Looking closer, we observe that such cost fluctuation comes from the communication side, which dramatically varies under different network conditions.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/d3c6f0de5202ea0d5f0ecad821965a7e58e25cc5b0a93ce9eed31ab82b936d8a.jpg)  
Fig. 5. Monitored background load level during SLAM runtime on the robot. The load curve consistently lies at a high level with a minimum of 83.60% and an average of 93.82%, indicating the robot is overloaded.

Concretely, the uploading latency takes 3.13, 2.35, and 1.88 ms for 4G, 5G, and WiFi, respectively.

Another expensive workload revealed is the local computing cost. To compute SLAM at the robot consumes around 1.51 s, occupying a percentage of 49.74% in total latency under WiFi channel. To inspect the cause behind that, we log the background load level on robots and plot the trace as in Fig. 5. As shown in the figure, the trajectory of the load always goes on top of 80%. It records an average of 93.82% and a minimum of 83.60%, sometimes even reaching 100.00%, indicating that the robot is overloaded and cannot sufficiently support the local SLAM computation.

In summary, the existing cloud-based solution suffers from dual bottlenecks. One is the inherent communication bottleneck from the remote Internet transmission that stresses the local frame uploading process between robots and the cloud. The other is the resource-hungry SLAM computation tasks, conflicting with the limited computing capability of the robots. The integration of these two knobs makes the cloud serving fall short in rendering low latency multirobot SLAM, desiring a holistic retrofit on the execution pipeline.

## C. Opportunities and Challenges With Edge Computing

To tackle the knobs of cloud-based approaches in both data offloading and robot computing, we promote edge computing to enable multirobot SLAM processing. Edge computing sinks computing power to the physical proximity to the robots and, thereby, provides a potential assist to greatly reduce the computation stress for robots while significantly lowering the transmission latency compared to cloud offloading paradigms [21].

With edge computing, there are opportunities to redesign the multirobot map fusion pipeline to accomplish both efficient SLAM computation and low latency transmission. Specifically, we can utilize the distributed edge servers to take over the workload from both robots and the cloud, acting as agents to resolve the processing bottlenecks smoothly.

Nevertheless, different from the cloud data center that provides unified, powerful, and closed resource access, edge servers exhibit a loosely coupled and uneven nature: they are usually distributed, heterogeneous, and dynamic. To orchestrate the serving between robots, edge servers, and cloud, the communication protocol should be carefully designed considering data transmissions in robot-to-edge, edge-to-edge, and edge-to-cloud. Besides, the workload balance among the multiple edge servers, as another impacted aspect related to the overall performance, should also be taken into account given the network dynamics.

## III. RECSLAM SYSTEM DESIGN

In this section, we introduce the design details of RecSLAM, a multirobot laser SLAM system that leverages the hierarchical robot–edge–cloud architecture to accelerate multirobot map construction. We present the design of RecSLAM by explaining its modular details following the data flow.

## A. Overview

The environment we envisaged to deploy RecSLAM spans multiple robots with laser scans and multiple edge servers in proximity. The robots can walk randomly or along with a preset route and, therefore, their sensory data are time-varying and may be overlapped in some moments. The edge servers can accept data from robots simultaneously and promise to be available during an epoch of execution.

RecSLAM works upon robot–edge–cloud architecture and designs specific modules in each tier. Fig. 6 shows a high-level view of RecSLAM. In the beginning, each robot captures environmental perceptions via its installed sensors and acquires the raw sensory data. In RecSLAM, we mainly consider two kinds of typical data: 1) laser data from the laser scans and 2) acceleration data from the IMU. Instead of running SLAM locally in a traditional mechanism, we immediately pack the sensory data and offload it to a certain edge server. When it accepts a data package, the edge server launches the SLAM execution and obtains a map corresponding to it. For clarity, we identify this map as local map or robot map, indicating a direct mapping to a robot’s raw data. For an edge server that serves multiple robots, a sequence of the local maps are generated and will be merged in situ to an edge map. This merging process is referred to as preparative fusion, as it is a preparatory action toward global fusion in the cloud. All the edge maps from distributed edge servers are finally merged in a centralized cloud to calculate the global map.

## B. Robots: Data Collection and Packing

As discussed in Section II-B, one of the performance bottlenecks is the excessive computing latency on robots, which comes from the contradiction between resource-hungry SLAM computation and resource-constrained onboard processors. To alleviate this, we propose to reserve only the lightweight data collection procedure on robots. Specifically, in our implementation, the sensory data to be collected are laser data and IMU data. The former is a structure named sensor\_msgs::LaserScan, including the timestamp, range data, angular distance, and so on, while the latter contains information for coordinates’ transformation. Though our current deployment only takes these two types of sensory data into account, it is convenient to accommodate additional formats in RecSLAM.

To offload the SLAM computation, we pack these raw data and transfer them to edge servers through robot operating system (ROS). Particularly, each robot establishes a connection and offloads its data to exactly one edge server, while each edge server can serve multiple robots at the same time. Here we assume that these connections are relatively well and stable, which can be relevant [28], [29], provided that they are within the same LAN. Therefore, the mobility of robots will not influence the data transmission and the consequent procedures. We note that in realistic deployment the robot– edge communication may be fluctuated and even fail, which raises robustness issues in the multirobot SLAM system. To address that, we can leverage multiaccess mobile-edge computing [30]–[32] to enable partial offloading, where a robot can transfer its data to multiple edge servers to ensure service availability and improve system performance.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/62136d3f8b13008ffac939822739b27213a68b3d5978a8b84344807151dc416b.jpg)  
Fig. 6. RecSLAM architecture overview. Each robot individually collects its sensory data and directly packs and transfers to a dedicated edge server. Each edge server runs SLAM using the received data and performs a preparative fusion to aggregate an edge map with its owned robot maps. All the edge maps will be sent to a centralized cloud for global map fusion.

## C. Edge Servers: Preparative Fusion for Edge Maps

The robots can access a set of edge servers in proximity via 5G/WiFi connections, and the edge servers, take charge of the SLAM computation and assisting the map fusion. Each edge server accepts raw data from multiple robots nearby and sends its result to a cloud.

The SLAM computation is inherently tightly coupled. If it is forced to split, the explosive complexity would lead to additional overhead in handling the graphic algorithms. Therefore, we treat the SLAM computation as a whole by encapsulating it as a function module for processing the raw data from a robot. In this case, the input to SLAM is collected from robots, and the output will be passed to map fusion. In our prototype, we develop the SLAM module with GMapping [33], a popular lightweight opensource SLAM implementation. For further acceleration, we manage a pool of execution instances to enable parallel and asynchronous serving, i.e., launch a separated process to compute SLAM whenever a robot’s raw data arrive.

The results of these processing threads are called robot maps, which will be merged via a fusion module. We refer to this module as preparative fusion, in contrast to the global fusion on the cloud. Particularly, the preparative fusion targets at the robot maps on the edge server and perform fusion by first checking the overlapping degree of robot maps and next merging those overlapping ones. We design to separate such a preparatory process from the global fusion for two reasons. On the one hand, by sinking the fusion workload to the edge servers, we can shrink the computing latency by avoiding excessive delay-significant transmissions to the remote cloud. On the other hand, fusing the robot maps on an edge server can effectively deduplicate the redundancy across overlapping robot maps and thus reduce the data size of edge maps, for which the communication overhead between edge servers and cloud is saved.

## D. Cloud: Aggregated Fusion for Global Map

The cloud is responsible to collect all edge maps from edge servers. We designate the cloud as the destination of final merging because a large number of downstream applications of SLAM are deployed on the cloud. However, if some service needs to deploy a SLAM-based task on end devices, we can expediently assign an edge server to finish global fusion and push the result to the specific device.

In the robot–edge–cloud architecture of RecSLAM, each edge server runs its SLAM and preparative fusion individually as long as it receives the robot data, which implies a parallel and distributed processing paradigm among the edge servers. To optimize the performance in such workflow, we need to carefully decide on a robot data processing assignment strategy with a target of balanced workload distribution among the edge servers to avoid the straggler effect. Nevertheless, this problem is nontrivial in its two-fold challenges. First, the overlapping degrees (i.e., information redundancy) across the robot maps (wrt. the robot’s sensory data) vary greatly among the robots given the mobility and randomness of the robots’ walk, while they can largely impact the computing latency of SLAM on edge servers and the data size of the fused edge maps. Both these two metrics are key performance contributors toward total execution time. Second, the network conditions between the robots and cloud can be heterogeneous. This requires the robot data processing offloaded from the robots to the edge servers to be aware of the diverse network connections. To overcome these challenges, we design an efficient optimization framework for robot–edge collaborative processing in the next section.

## IV. ROBOT–EDGE COLLABORATION OPTIMIZATION ALGORITHMS

This section concentrates on scheduling the data flow between robots and edge servers in order to minimize endto-end multirobot SLAM latency. We first formulate the cost optimization problem in terms of the latency expired by robots, edge servers, and cloud server. Next, we address it via a two-step workflow, i.e., balanced robot data grouping and network-aware edge offloading.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/ed0ca5ec633a07363a00b560d7f2ba274ddf5d384ebf327f41fc3bfc286d6dde.jpg)  
(a)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/b0aaf1d870faa92ab845af594d77e4160b69b1061b7ff76096e32e111cab4ea7.jpg)  
(b)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/edddaa6355932fc3967aaafad2fa1782b93c5d39584261b37850a2a04cdd74a9.jpg)  
(c)  
Fig. 7. Analysis of the map fusion procedure. (a) Computing time of map fusion procedure with respect to the five functions that it applies: feature extraction, pairwise matching, component connection, transform optimization, and grid composing. (b) observes that as the overlapping degree increases, the data size of the merged map decreases. Note that the two inputs are of equal data size, and the overlapping degree is defined by the percentage of the overlapping area out of the total areas. (c) Map fusion procedure with two robot maps, where it first checks the overlapping area of the inputs via their feature information, and next connects these overlapping parts and constructs a merged map. The larger the overlapping area, the smaller data size the merged map has

## A. Problem Formulation

We consider a cluster of R robots, where each robot r can walk freely or with the user-defined routes, and pushes its sensory data to a specific edge server e among the E edge servers. To obtain a robot $r \mathrm { { s } }$ robot map processed with the assist of an edge server $e ,$ it counts the time for data packing, transferring and frame transformation on e

$$
t _ { r } = t _ { r } ^ { \mathrm { p a c k } } + t _ { r  e } ^ { \mathrm { t r a n s } } + t _ { e } ^ { \mathrm { f r a m e } } .\tag{1}
$$

Moving forward the data flow, calculating the edge map on an edge server e needs to wait until all its owned robot maps ready and fuse them in situ, which takes the time

$$
t _ { e } = \operatorname* { m a x } _ { r \in e } t _ { r } + t _ { e } ^ { \mathrm { f u s e } }\tag{2}
$$

where $r \in e$ means the robots whose data are assigned to the edge server e, $t _ { e } ^ { \mathrm { f u s e } }$ represents the fusion latency in edge server e. From a cloud view, it strives to aggregate all the edge maps from edge servers to reckon the global map, which takes the time of receiving all edge maps and performing global fusion

$$
t _ { c } = \operatorname* { m a x } _ { e } \left( t _ { e } + t _ { e \to c } ^ { \mathrm { t r a n s } } \right) + t _ { c } ^ { \mathrm { f u s e } } .\tag{3}
$$

Essentially, $t _ { c }$ is the total execution time that characterizes the duration from raw data collection to global map completion, which is exactly the optimizing objectives of RecSLAM. $t _ { c } ^ { \mathrm { f u s e } }$ represents the fusion latency in cloud. Therefore, we can derive the execution latency optimization problem as

$$
\begin{array} { c } { { \operatorname* { m i n } t _ { c } } } \\ { { \mathrm { s . t . } ( 2 ) , ( 3 ) . } } \end{array}\tag{4}
$$

Discussion: The formulation provides an abstracted form of the problem with various details hidden. Among them, we mainly consider two factors. One is the computation side, where we expect the assignment from robots to edge servers is relatively even such that the effect of parallel processing is maximized. The rationale behind can be seen in Fig. 7(a), where the total computing time of map fusion grows quadratically as the number of frames linearly increases. In Fig. 7(a), we further break down map fusion procedure into five successive functions: feature extraction extracts a frame’ edge features, and next pairwise matching performs feature points matching between different data frames; component connection and transformation optimization functions intend to solve the pose, and finally grid compose is performed to fuse the frames. We observe from functionwise latency that this trend primarily comes from the pairwise matching function, which performs k(k−1)/2 times of pairwise checking given k frames. Therefore, if excessive robot data is assigned to a few edge servers, the imbalance workload distribution will lead to a much higher completion time of the total process. Reflecting on (3), it is for minimizing $\operatorname* { m a x } _ { e } ( t _ { e } + t _ { e \to c } ^ { \operatorname { t r a n s } } )$

The other factor is the communication side that focuses on the data size of edge maps. This metric, as we observe in measurements, is highly sensitive to the overlapping degree, i.e., the ratio of the overlapping area to the total area of input maps. To illustrate that Fig. 7(b) plots the data size of the corresponding edge map with varying overlapping degree and Fig. 7(c) depicts how map fusion works. In Fig. 7(b), we take two robot maps of equal data size as input, and control the content of them to adjust the overlapping area. The result reports that as the overlapping degree increases, the edge map’s data size shrinks. Particularly, a 50% overlapping degree of two inputs indicates they are fully overlapped and, thus, the obtained edge map is exactly one of the inputs and has the same data size of a single robot map. Fig. 7(c) explains this impact visually, where the blue area indicates the overlapping part. The larger the overlapping area, the higher the overlapping degree and, thus, the smaller data size the edge map has. This observation drives us to assign robot maps with as higher overlapping degree as possible together, so as to yield smaller edge maps and save the transmission cost between edge servers and the cloud.

Overall, accommodating both computation and communication aspects to optimize the problem (4) is inherently complicated. Essentially, it is hard to solve the optimal solution

(b)

fast due to its combinatorial nature. To achieve efficient solving, we intend to decouple it into two subproblems by separately accounting for robot data grouping and robot data offloading. On the one hand, considering the characteristics of map fusion performed by SLAM, robot data grouping for workload balancing needs to ensure that the number of nodes in each group tends to be uniform, while the in-group overlapping degree is as large as possible, so as to avoid the straggler effect due to imbalanced processing workload assignment and reduce the total size of fused edge maps. On the other hand, bandwidth conditions of the edge servers may be different, so network-aware offloading optimization for efficient robot-to-edge data transmission and computation is also desired.

## B. Balanced Robot Data Grouping

The robot data grouping targets at dividing V robots’ SLAM sensory data into N groups (i.e., N edge servers) with the objective of evenness. Here, evenness means the number of robots across groups should be approximately equal, in order to achieve balanced processing workloads among the edge servers later on. Besides, for minimizing the size of edge maps after fusion, the robot data within the same group are desired to yield robot maps with a higher overlapping degree as much as possible. We accomplish these goals by first abstracting an overlapping graph to shape the relations between robot maps.

Graph Abstraction: Fig. 8 illustrates how we abstract an overlapping graph. Given the robots in Fig. 8(a)’s factory layout plane, we can collect their position and localization information in a bottom-up manner: the robots report their meta sensory data to the nearby edge servers and the edges push these data to the cloud to infer the global overlapping information. In many well-planned deployments, this procedure can be even accomplished offline as long as we know the robots’ scanning routes prior to runtime. With the information, we can therefore represent the robots’ corresponding maps in Fig. 8(b), where each one reflects the area that a robot’s scanning field covers. It is represented by circle, because the robot can scan the surrounding $3 6 0 ^ { \circ }$ during the movement. Let a pair of maps with an overlapping area share a link, we abstract a graph in Fig. 8(c), named as the overlapping graph. A vertex in such a graph is a robot map, whereas a link quantifies the overlapping degree between two robot maps. Particularly, two vertices are not connected if there is no overlap between the two robot maps. We denote the overlapping degree as $w ( u , \nu )$ which represents the weight of the link between vertices u and v.

Balanced Grouping Algorithm: Using the overlapping graph model, we observe that the robot map grouping problem can be approximated to a graph clustering variant. Concretely, we can operate on the overlapping graph to decide a balanced grouping with maximal in-group overlapping degree.<sup>1</sup> To accomplish this, we develop a balanced grouping solution in Algorithm 1 on the basis of the balanced graph partitioning method [34], [35]. The key idea is first to generate a feasible grouping plan and next use tabu searching for further optimization. Particularly, the way that Algorithm 1 finds an initial grouping is to iteratively put the vertex with maximum performance gain to a group such that for each group the ingroup overlapping degree is maximized, and the tabu searching optimization is to avoid a local optimum result and will be described in Algorithm 2 detailedly.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/956730dc973cb3ec2d508ad2827a61af68ccd0f3afed2a202664a6253d115a30.jpg)

(a)  
![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/3c67de1fa4ce38fb48086a3879f90ee4fa088dd60116c763e2d0526dc8d8b0f7.jpg)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/42051025a7aa5cae8d0f315872ea483603bcfa57b3331b7149071e5f41826f07.jpg)  
(c)  
Fig. 8. Example abstraction flow of the overlapping graph. For the robots in the factory scene (a), we first abstract robot maps in (b) and then derive the overlapping graph in (c).

The input of Algorithm 1 are an undirected graph (with V vertices), the number of groups (i.e., the number of edge servers) N, and a neighboring function $\mathcal { N } ( \nu )$ that returns the neighbor vertices of v. The output is the grouping result $\langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle$ , where $P _ { i }$ is a set that includes the vertices assigned to the ith partition.

At the beginning of Algorithm 1, we initialize an empty buffer set R and an empty candidate set $C ,$ and immediately add all vertices in the overlapping graph $\mathcal { G }$ to C. We also initialize empty grouping result $\langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle$ , and then goes into an iterating process. For each iteration, we first check whether the buffer R is empty: if empty, it means there is no suitable candidate related to vertices in the current group $P _ { i } ,$ thus we add the vertex $\nu _ { \mathrm { m i n } } ^ { \mathrm { n e i g h } }$ with the least number of neighbors from the candidate set C, and put it to $P _ { i }$ (lines 7–9). The vertices associating with $\nu _ { \mathrm { m i n } } ^ { \mathrm { n e i g h } }$ , obtained by $\mathcal { N } ( \nu )$ , will be added to R for the coming assignment (line 10). Whenever a node is deleted from C, we check whether $| C | \le ( V / N )$ . If the condition is satisfied, which means the difference of nodes number in any two groups is less than 2, it will set the exit flag as true and jump out of the loop (lines 11–13). It should be noted that these neighboring vertices $\mathcal { N } ( \nu )$ have never been visited, i.e., they have not been assigned to a specific group. If R is not empty, there remains vertices in the buffer, and we will select the vertex $\nu _ { \mathrm { m a x } } ^ { \mathrm { g a i n } }$ with the maximum gain from candidates C and move it to $P _ { i } .$ . The gain is estimated using (5): given a vertex v in $C ,$ we reckon the sum of $w ( u , \nu )$ in $P _ { i }$ and C, respectively, and calculate their difference. Such difference values actually measure the weight change brought by the vertex migration. The greater the difference, the greater the possibility of the point being selected

Algorithm 1 Balanced Robot Data Grouping Algorithm Algorithm 2 Tabu-Searching-Based Optimization   
Input: The overlapping graph $\overline { { \mathcal { G } } }$ (with V vertices), Input: The overlapping graph ${ \overline { { \mathcal { G } } } } ,$   
The number of edge servers N, Initial grouping $\langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle$   
The neighboring function $\mathcal { N } _ { C } ( \nu )$ that returns the neighbor The maximum iterations times $\kappa ,$   
vertices of v in a set C The maximum tabu list length τ   
Output: Grouping result $\langle P _ { 1 } , P _ { 2 } , \cdots , P _ { N } \rangle$ Output: Optimized grouping result $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \cdots , P _ { N } ^ { * } \rangle$   
1: Initialize an empty buffer set R and an candidate set C 1: Initialize an empty tabu list T of vertex pairs, an empty   
2: Assign all vertices in $\mathcal { G }$ to $C ,$ set exit $\hbar a g = 0$ list G of obtained grouping results, and an empty list S of   
3: Initialize empty grouping result $\langle P _ { 1 } , P _ { 2 } , \cdots , P _ { N } \rangle$ vertex pairs to be swapped   
4: repeat 2: $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \cdot \cdot \cdot , P _ { N } ^ { * } \rangle  \langle P _ { 1 } , P _ { 2 } , \cdot \cdot \cdot , P _ { N } \rangle$   
5: if R is empty then 3: repeat   
6: for $i = 1 , 2 , \cdots , N - 1$ do 4: for vertices $\nu , u \in { \mathcal { G } }$ do   
7: Select vertex $\nu _ { \mathrm { m i n } } ^ { \mathrm { n e i g h } }$ from C such that $\nu _ { \mathrm { m i n } } ^ { \mathrm { n e i g h } }$ has the 5: if v and u are not in the same group then   
least number of neighbors 6: Add (v, u) to S   
8: $P _ { i }  P _ { i } + \{ \nu _ { \mathrm { m i n } } ^ { \mathrm { n e i g h } } \}$ 7: Swap the locations of v and u to obtain a new   
9: $C \gets C - \{ \nu _ { \mathrm { m i n } } ^ { \mathrm { n e i g h } } \}$ grouping $\langle P _ { 1 } ^ { \prime } , P _ { 2 } ^ { \prime } , \cdots , P _ { N } ^ { \prime } \rangle$   
10: $R  R + C \cap \mathcal { N } _ { C } ( \nu _ { \operatorname* { m i n } } ^ { \mathrm { n e i g h } } ) - \{ \nu _ { \operatorname* { m i n } } ^ { \mathrm { n e i g h } } \}$ 8: Add $\langle P _ { 1 } ^ { \prime } , P _ { 2 } ^ { \prime } , \cdots , P _ { N } ^ { \prime } \rangle$ to G   
11: ${ \textbf { i f } } | C | \leq { \frac { V } { N } }$ then 9: end if   
12: set $\hbar a g = 1$ and break 10: end for   
13: end if 11: k ← arg min <sub>i,g ∈G</sub> $\{ f ( g _ { i } ) \}$ using Equation (6)   
14: end for 12: if $f ( G _ { k } ) < \bar { f } ( \langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \cdot \cdot , P _ { N } ^ { * } \rangle )$ then   
15: else 13: if $S _ { k } \in T$ then   
16: for $i = 1 , 2 , \cdots , N _ { \mathrm { - } } - 1$ do 14: Remove $S _ { k }$ from T   
17: Select vertex $\nu _ { \mathrm { m a x } } ^ { \mathrm { g a i n } }$ from $C$ such that $\nu _ { \mathrm { m a x } } ^ { \mathrm { g a i n } }$ has the 15: end if   
maximum gain using Equation (5) 16: $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \cdot \cdot \cdot , P _ { N } ^ { * } \rangle  G _ { k }$   
18: $P _ { i }  P _ { i } + \{ \nu _ { \operatorname* { m a x } } ^ { \mathrm { g a i n } } \}$ 17: end if   
19: $C \gets C - \{ \nu _ { \operatorname* { m a x } } ^ { \mathrm { g a i n } } \}$ 18: if $S _ { k } \notin T$ then   
20: $R  R + C \cap \mathcal { N } _ { C } ( \nu _ { \operatorname* { m a x } } ^ { \mathrm { g a i n } } ) - \{ \nu _ { \operatorname* { m a x } } ^ { \mathrm { g a i n } } \}$ 19: Add $S _ { k }$ to $T$   
21: ${ \begin{array} { r } { { \mathbf { i f } } \ | C | \leq { \frac { V } { N } } } \end{array} }$ then 20: end if   
22: set $\hbar a g = 1$ and break 21: if $\vert T \vert > \tau$ then   
23: end if 22: Remove the first element of $T$   
24: end for 23: end if   
25: end if 24: $\langle P _ { 1 } , P _ { 2 } , \cdot \cdot \cdot , P _ { N } \rangle \gets G _ { k }$   
26: until $\hbar a g = 1$ 25: until Iterating for κ times   
27: $P _ { N }  C$ 26: return $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \cdots , P _ { N } ^ { * } \rangle$   
28: Apply Algorithm 2 to optimize the grouping result   
29: return $\langle P _ { 1 } , P _ { 2 } , \cdots , P _ { N } \rangle$

$$
\mathrm { g a i n } ( \nu ) = \sum _ { u \in P _ { i } } w ( u , \nu ) - \sum _ { u \in C \backslash \{ \nu \} } w ( u , \nu ) .\tag{5}
$$

The principle behind exploiting the gain formula is to make each in-group overlapping degree as large as possible, while the out-group is as small as possible. Again, we assign $\nu _ { \mathrm { m a x } } ^ { \mathrm { g a i n } }$ to the current group $P _ { i }$ and remove it from C, and the neighboring vertices $\bar { \mathcal { N } ( \nu _ { \mathrm { m a x } } ^ { \mathrm { g a i n } } ) }$ will be added to the buffer R (lines 17–20). In this step, we still need to check whether $| C | \le ( V / N )$ is met (lines 21–23). The above iterating processing continues until exit flag is true, which indicates that all the vertices have been allocated in a balanced manner. The remaining vertices in C will be directly assigned to the Nth group $P _ { N }$ and, therefore, we obtain a feasible initial grouping result (line 26).

At the same time, $\mathcal { N } ( \nu )$ will be added to R. Finally, when all the vertices are visited, we can get an appropriate grouping result (line 27). However, this result is based on the heuristic assignment and may terminate at a local optimum. To avoid such circumstances, we further apply tabu searching to seek a better grouping result (line 28), as described in Algorithm 2.

Tabu-Searching-Based Optimization: The key idea of Algorithm 2 is to use a tabu list to save the local optimal solution, to avoid falling into the same result. The input of the algorithm is the overlapping graph , the initial grouping result $\langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle$ , the maximum iterations times κ, and the maximum tabu list length τ. The output is an optimized grouping result $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \ldots , P _ { N } ^ { * } \rangle$

Algorithm 2 begins with initializing an empty tabu list T with length τ , an empty list G of obtained grouping results, and an empty list S of vertex pairs to be swapped. We instantiate the expected grouping $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \ldots , P _ { N } ^ { * } \rangle$ with the initial grouping $\langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle$ , and goes into an iterating process. For each iteration, we first construct all possible swapping pairs (lines 4–10): find vertices v and u that are not in the same group, add them to $S ,$ apply a swapping to obtain an updated grouping $\langle P _ { 1 } ^ { \prime } , P _ { 2 } ^ { \prime } , \ldots , P _ { N } ^ { \prime } \rangle$ , and put the updated grouping to G. With these swapped candidates, we then expect to filter those that improve the quality of grouping. Specifically, we define a fitness function f(·) in the following:

$$
f ( \langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle ) = \sum _ { u \in P _ { i } , \nu \in P _ { j } , i \neq j } w ( u , \nu ) .\tag{6}
$$

The fitness defines the suitability of the grouping by calculating the sum of weighted edges between different groups in (6). It is designed for weighing the sum of edges in an overlapping graph, and we can therefore use it in our optimization on workload grouping. The smaller the fitness value, the better the grouping result. In Algorithm 2, we find the swapping vertex pair, noted with index k, such that it has the smallest fitness within all candidates in G (line 11). If this vertex pair $S _ { k }$ has smaller fitness than the current optimized counterpart’s $( \langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \ldots , P _ { N } ^ { * } \rangle )$ , we will update this optimized result (if it is exactly in the tabu list T, just remove it from T). Otherwise, $S _ { k }$ is accepted and will be added $S _ { k }$ to the tabu list (lines 18– 20), which means we will accept the local optimal solution and prevent it from being visited again, unless the amnesty rules are met. If the tabu list T is overlength than the preset maximum length τ, we will remove the first element, the one that is the most out-of-date, from $T ,$ based on the first-infirst-out principle. The current grouping will also be updated will kth grouping. The above iterations continue until reaching the maximum κ times, and the final optimized grouping $\langle P _ { 1 } ^ { * } , P _ { 2 } ^ { * } , \ldots , P _ { N } ^ { * } \rangle$ will be returned.

Complexity Analysis: Assuming an overlapping graph has V vertices and L links and we need to divide N groups, the time complexity of the balanced grouping algorithm is $O ( V { + } L )$ . In the worst case, each partition has to traverse $V / N$ vertices. The operation of selecting vertices is regarded as O(1), because this part can be optimized by maintaining the tree on the data structure. Moreover, the optimization requires traversing the edges, so the total complexity is $O ( V / N + L ) = O ( V + L )$

## C. Resource-Aware Edge Offloading

After robot data grouping, we have several groups of robot data and need to decide their placement to the proper edge servers for edge map fusions to minimize the total collaborative processing time. Since the data transmission from the robots and edge servers can be a bottleneck and the network conditions of the edge servers can be different, the decision needs to tailor for the available bandwidth. Moreover, the computing capacity of the edge servers can also be heterogeneous. Therefore, we propose a resource-aware edge offloading strategy in Algorithm 3.

Algorithm 3 Resource-Aware Edge Offloading Algorithm   
Input: The grouping result $\langle P _ { 1 } , P _ { 2 } , \cdots , P _ { N } \rangle$   
The edge servers list $\mathcal { E } = \langle 1 , 2 , \cdots , N \rangle$   
Bandwidth $B _ { j }$ between edge server j and the cloud,   
Latency prediction model $L _ { j } ( P _ { i } )$ that returns the comput   
ing latency of $P _ { i }$ on edge server $j ,$   
The function $\mathcal { D } ( P _ { i } )$ that returns the output data size of   
partition $P _ { i }$   
Output: Offloading plan $\langle \theta _ { 1 } , \theta _ { 2 } , \cdots , \theta _ { N } \rangle$   
1: for $i = 1 , 2 , \cdots , N$ do   
2: $\begin{array} { r } { j \gets \arg \operatorname* { m i n } _ { i \in \mathcal { E } } ( L _ { j } ( P _ { i } ) + \frac { \mathcal { D } ( P _ { i } ) } { B _ { j } } ) } \end{array}$   
3: $\theta _ { i } \gets j$   
4: Remove edge server j from $\mathcal { E }$   
5: end for   
6: return $\langle \theta _ { 1 } , \theta _ { 2 } , \cdots , \theta _ { N } \rangle$

Algorithm 3 works in a greedy fashion and the input is the robot data grouping result $\langle P _ { 1 } , P _ { 2 } , \ldots , P _ { N } \rangle$ from Algorithm 1, the edge servers list $\mathcal { E } = \langle 1 , 2 , \ldots , N \rangle$ , and $B _ { j }$ that records the bandwidths between edge server j and the cloud. Besides, two functions are also employed: 1) the latency prediction function $L _ { j } ( P _ { i } )$ that returns the computing latency of $P _ { i }$ on edge server j and 2) the function $\mathcal { D } ( P _ { i } )$ that returns the data size of partition $P _ { i } .$ . The output is the offloading strategy $\langle \theta _ { 1 } , \theta _ { 2 } , \ldots , \theta _ { N } \rangle$ where $\theta _ { i }$ logs the edge server that grouping i will be placed. In Algorithm 3, we iteratively select a group $P _ { i }$ and pick an edge server j such that the robot-to-edge offloading (including transmission and computation) latency is minimized (line 2). Particularly, we use $L _ { j } ( P _ { i } )$ to estimate the computing latency on the edge server and $( \mathcal { D } ( P _ { i } ) / B _ { j } )$ to calculate the transmission time. Since this latency simultaneously tackles the computing aspect, i.e., $L _ { j } ( P _ { i } )$ , and the bandwidth $B _ { j } ,$ , it is aware of the available resource in both computation and communication, and can support heterogeneous cases. The selected edge server j that minimizes the latency will be assigned to $\theta _ { i }$ and be removed from the available edge server list $\bar { \mathcal { E } } .$ We carry out this process until all edge servers are assigned with a group, and return the final offloading plan $\langle \theta _ { 1 } , \theta _ { 2 } , \ldots , \theta _ { N } \rangle$

Complexity Analysis: In our implementation, we use an offline profiling method to obtain the estimation of computing time and output data size, which will be detailed in Section V. Using the profiling result, the overhead of calculating the computing/transmission time of each server is O(1), and the complexity of Algorithm 3 is O(N), which is linear and fast.

## V. IMPLEMENTATION

We have implemented RecSLAM in both realistic prototype and simulation platform. The proof-of-concept prototype is built with Jetson TX2 and Turtlebot 3 as shown in Fig. 9, while the simulations are carried on Gazebo [25], a high-precision simulator of robotics emulations. Fig. 10 shows the graphical interface of the scene in Fig. 1(a).

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/3aa1e7f3748c1c9b72d94d21421ab43ad756cce2cb99dc06a350167ad3f3b30b.jpg)  
Fig. 9. Our prototype employs Turtlebot 3 as the robot and Jetson TX2 as the edge server. The Turtlebot is equipped with a RPLIDAR A1 as the laser scan and a Raspberry Pi as the processing module.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/a53800c1f7db80e0a0956477086e099891d10f23f3061a5a271cb794a81892ad.jpg)  
Fig. 10. Simulated scene of Fig. 1(a) in the Gazebo [25] simulator, where robots and edge servers are placed inside to emulate multirobot SLAM applications in physical edge scenarios.

SLAM Integration: We adopt GMapping as the SLAM kernel on edge servers. However, our system can support different SLAM toolkits as long as the robot can provide the necessary data. Each time a robot offloads data to the edge, the edge server will create a dedicated GMapping instance for the robot to calculate the local map with a specific topic. For example, when two robots offload data to an edge server at the same time, the topics of them are tb4\_0 and tb4\_2, respectively. At this time, there will be two GMapping instances on the edge that subscribe the data with tb4\_0/ and tb4\_2/ namespace, and then publish the occupancy grid map with tb4\_0/map and tb4\_2/map topics, respectively.

To leverage the proposed scheduling algorithms over GMapping, we implement them in a centralized manner. Specifically, we select one of the available edge servers as the coordinator to be responsible for collecting robots’ maps information, estimating the overlapping degrees, and running scheduling algorithms. According to the scheduling result, the coordinator will route the robot data to corresponding edge servers, and orchestrate them to collaboratively run map fusion tasks. The algorithms can also be implemented distributionally, where a tailored mechanism is desired for global information sharing and consensus management, and we leave it as a future work.

Robot–Edge Communication: The communications among robots, edge servers, and the coordinator are conducted via

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/a943268ebe959035e5915ddbb2c952f19b651e5edb5201a61e8861243b2bb9fe.jpg)  
Fig. 11. Regression result of SLAM computing latency on the Jetson TX2. The dots are the measurements and the curve is the regression.

ROS.<sup>2</sup> They all communicate using the publisher–subscriber model. In such a mechanism, the subscriber node will first register on the node master, and then the node master will search among the registered publishers, find the matching publisher, and, finally, the publisher and the subscriber will establish a communication channel.

The communications involve multiple subscribing topics. In RecSLAM, the data format released by the robot cluster is tbn\_x, where x is the specific robot number. The data format of the GMapping instance subscripter in the edge is tbn\_x, and the data with topic tbn\_x/map will be published after processing. The Map Fusion module receives coon\_x/map and publishes the data with edgem\_x/map at the same time.

For the coordinator, it will receive tbn\_x/map’s data. The workload grouping algorithm in the coordinator will split the robots into groups, and then the offloading algorithm will generate allocation decisions and publish the data with the topic coon\_x/map. The communication mechanism based on ROS ensures the reliability of communication between the robot and edge.

Parameter Profiling: The coordinator derives the overlapping degree matrix using a modified package called weight\_cal. It is based on multirobot\_map\_merge, a generic package in ROS. It can read multiple maps in a batch, extract their features, perform matching checking, and output a rough overlapping degree matrix. To run it in a lightweight style, we invoke this package periodically, i.e., calculates and saves the overlapping degree matrix of the robot maps in a preset regular frequency.

To estimate the time of preparative fusion $t _ { e } ^ { \mathrm { f u s e } }$ at the edge server, we employ an offline profiling mechanism that trains regression models for performance prediction. In our deployment, we carry out profiling on Turtlebot 3 and record the computing latency of map fusion with respect to the number of data frames. Fig. 11 illustrates the regression result. An interesting observation of the figure is that the computa tion latency rises in a quadratic trend as the frame number increases from 2 to 12, This is because the computation of merging requires pairwise matching of data frames, and the number of matches has an exponential relationship with the number of frames.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/3b2a7acb1fdb118aae36f3de88e1ffbe6fbe60a03f4fc843205aaa29d6830579.jpg)  
(a)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/b83df0b44f3248d04a40064bd836af5cb56bd7b8a652988d82961ca5b0ccc4a2.jpg)  
(b)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/dfc060e865e5e89cfd2d619a7aebbc75c8cff007b601cde90bfc00d93abe5005.jpg)  
(c)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/cf5e3ea84a1c6092ceb2112defe0d566002d396e9e3af7f8bdefa6e85f9baa61.jpg)  
(d)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/15f77805996ea8d49a5fb1937759f444683db0dedab9468e35b1a788599dc907.jpg)  
(e)  
Fig. 12. Proof-of-concept prototype experiments with three robots and one edge server in a self-built real scene. Due to the limitations in robotics hardware and physical space, the testbed in (a) is not built perfectly and gaps exist between walls, for which the corresponding global map in (e) has a rough boundary Despite that, the global map still shows its feasibility by sketching the complete scene and reversing the obstacles’ details, which verifies the practicality of RecSLAM. (a) Prototype scene. (b) Robot map 1. (c) Robot map 2. (d) Robot map 3. (e) Edge map.

For the transmission latency between robots, edge servers, and the cloud, there exist many network bandwidth measure tools that are available, which enables the real-time transmission time estimation based on the data size of the input frames. In RecSLAM, we utilize the principle of tolerance and exclusion formula to estimate the output size of preparative fusion. Since the number of robot maps is relatively small, the estimation error of the overlapping area is within a tolerable range.

## VI. EVALUATION

This section evaluates RecSLAM in both a simulator platform and a prototype deployment. Particularly, we focus on the feasibility of global map construction and the end-to-end execution time of running a multirobot SLAM process.

## A. Functional Verification

We first conduct functional verification of RecSLAM based on a small-scale realistic prototype. Specifically, we mainly focus on collecting realistic robotics measurements and examining the effectiveness of the hierarchical map fusion procedure and, thus, providing necessary data to facilitate subsequent larger-scale simulation experiments. We use the TurtleBot3 Waffle Pi mobile robot for laser data collection and the Jetson TX2 as the edge server. The TurtleBots are all installed with Raspberry Pi 3b+ as their processing module, which has 1.4-GHz ARM Cortex-A53 CPUs and 1-GB LPDDR2 memory. The Jetson TX2 is equipped with NVIDIA Denver and ARM Cortex-A57 CPUs, an NVIDIA Pascal GPU with 256 CUDA-cores, and 8-GB LPDDR4 memory. All the robots and the edge server are connected via the 802.11ac wireless signal and in the same LAN. The cloud server employs an instance (8vCPU, 16-GB memory, Ubuntu 16.04) that locates at the same available zone with the robots and their physical distance is about 200 km. The testbed scene is built in a $6 \times 5 . 5 ~ \mathrm { m } ^ { 2 }$ conference room, where we manually add walls and obstacles, as shown in Fig. 12. The linear velocity of the robots is 0.20 m/s, and the angular velocity is set to 0.20– $0 . 8 0 ~ \mathrm { m / s ^ { 2 } }$ during the runtime. Although RecSLAM’s design allows robots to walk freely, we have set determined routes for robots in advance for the sake of simplicity and experimental repeatability. As the robots move around, the equipped laser sensors perceive the surrounding environment at a fixed frequency and continue collecting sensory data.

Fig. 12 presents the mapping results of our prototype. Particularly, Fig. 12(b)–(d) visualizes the robot maps that are computed based on the robots’ sensory data, where we can see that the robots pave three different routes on the left, center and right of the scene, respectively. Overlapping areas can be observed from these robot maps, e.g., robot map 1 in Fig. 12(b) and robot map 2 in Fig. 12(c) share the same area at their bottom. Fig. 12(e) shows the obtained edge map that is fused from the robot maps, where the complete scene is sketched and the obstacles’ details are reserved.

To further demonstrate RecSLAM’s effectiveness, we make additional simulation experiments and build several virtual scenes in Gazebo [25], a popular platform<sup>3</sup> that provides high-precision and near-physical experimental environments. Specifically, we design typical apartment indoor scenes in Gazebo’s virtual environments and place robots and edge servers to emulate multirobot scenarios at the network edge. In all scenes, we distribute and preset routes for robots in advance, and drive them to walk around the apartment. During the runtime of multirobot SLAM, we record the robots’ sensory data and calculate the maps by continuously replaying the collected data. To fully reflect the specifications in our prototype, we configure the same motion parameters, such as velocity and scanning field, and replay the robots’ movement for all simulations.

The first scene, as shown in Fig. 1(a), is the main scene for our simulation experiments, which is a $1 3 \times 1 8 ~ \mathrm { m } ^ { 2 }$ apartment scene with three edge servers (and a varying number of robots on demand). Fig. 1(b) is exactly the global map generated by RecSLAM. We also built another two scenes in Figs. 13 and 14 to examine the effectiveness in constructing the whole map. The scene in Fig. 13(a) is in $\textbf { a } 4 \times 4 \ \mathrm { m } ^ { 2 }$ square and is designed to emulate a symmetrical layout. The subfigures Fig. 13(b)–(e) depicts the robot maps constructed using the laser scan data from the four robots, while the Fig. 13(f) and (g) are the corresponding edge maps and Fig. 13(h) is the global map in the cloud. Specifically, Fig. 13(f) is obtained by fusing Fig. 13(b) and (c), and Fig. 13(g) is from Fig. 13(d) and (e). The last scene is in a $5 \times 5 \ : m ^ { 2 }$ room as shown in Fig. 14(a), which exhibits an asymmetrical layout. In contrast to the neat robot maps in Fig. 13, the maps in Fig. 14 shows relatively irregular styles due to the scene’s asymmetry. Nevertheless, it can be seen from both scenes that the global map after map fusion matches the corresponding scenes with respect to the whole layout and the obstacle positions.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/853cc3216503127b60d198d35057c9bd0c4996fb7651218d8f4521526728c8a0.jpg)  
(a)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/95159694669600042deefb49d6145e59a9cb89e6cef4222dfd89e6a667f37138.jpg)  
(b)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/c69951566d0c16bbf8fa92ec48e266a4cd989cf431588bf28d57c949dbb9e479.jpg)  
(c)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/1a83af3bede6c713e0b9ab38833d03ffcd6a5031b5a0565e285edd0f80333cc6.jpg)  
(d)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/72667ed8b9f9f058687ac1cc9eac44db6d5fd46e23d198d691e6bb0faac30b8a.jpg)  
(e)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/9071390d92922ada0cd784c885c25b6db628bd3140cd0d885c5c068e024c876a.jpg)  
(f)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/05b23eaf9ddbab37b416d906f0aa3a3b5ab811da2b0b78b8d6b7a8c3492ef1b3.jpg)  
(g)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/20f5a243f29b1f47efdfe053462a3666eaa6d1ff5dab9ae15e2f6138c057edff.jpg)  
(h)

Fig. 13. Simulation results with four robots and two edge servers in a symmetrical apartment scene. The edge servers are not drawn in (a) for simplicity. The robots walk according to preset routes and pass their sensory data to edge servers for multirobot SLAM computation. The complete global map demonstrates RecSLAM’s effectiveness in merging multirobot sources. (a) Scene. (b) Robot map 1. (c) Robot map 2. (d) Robot map 3. (e) Robot map 4. (f) Edge map 1. (g) Edge map 2. (h) Global map.  
![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/79e9fba3c343a14e467048a712bfb1fdcbfc885b22c844bebd76dc109eb01530.jpg)  
(a)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/2743234a6e56d59f07b162f2f8835fbdf2951acda5a9d361df3366941fe49838.jpg)  
(b)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/65bf2b1cc9183cbed26f3e498dfae7837454b6d984b7ce1d498e699ff178cbc4.jpg)  
(c)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/cb982c48f2245819835109ee54dcf1eaa0dd20e23164a9ca62d5f448b9961d83.jpg)  
(d)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/be5a34066ffe855c396a93063a39174b1d41ccf078a3413e26aa4086ea34f666.jpg)  
(e)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/02841347d1c9a74be2321e6be6e59e3e7cfcc2da822398e292bc3ff8d2dd2853.jpg)  
(f)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/198594295b9f9717c0175463a14a54dfef39ecda0f140810a4cbc3835b2b4e75.jpg)  
(g)

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/9c18230ba3b30503a42bbd6c258a6f4e4d4851bcb2ccdda1a56ba183655de5a8.jpg)  
(a)  
Fig. 14. Simulation results with four robots and two edge servers in an asymmetrical apartment scene. The edge servers are not drawn in (a) for simplicity. RecSLAM effectively renders the complete mapping again, and note that the flaws in the global map mean the area that the robots have not scanned. (a) Scene. (b) Robot map 1. (c) Robot map 2. (d) Robot map 3. (e) Robot map 4. (f) Edge map 1. (g) Edge map 2. (h) Global map.

Remark that the global mapping results from RecSLAM and the cloud approach are the same since both of them reserve complete raw scan data from robots and run the same SLAM module. Particularly, RecSLAM improves multirobot SLAM beyond traditional cloud approach by mitigating the workload of SLAM and fusion to edge servers and achieves much lower execution latency without sacrificing map accuracy.

## B. Performance Evaluation and Comparison

Due to the limitations of the robotics hardware and physical space in our laboratory, we focus on verifying the functional feasibility of RecSLAM’s multirobot map fusion. To fully evaluate multirobot SLAM performance with more robots and more edge servers, as well as in larger scenes, we conduct large-scale simulation experiments in Gazebo to demonstrate RecSLAM’s effectiveness and efficiency.

Algorithm Runtime: RecSLAM consists of two parts, namely, the grouping algorithm in Algorithm 1 and the tabusearching-based optimization algorithm in Algorithm 2. In order to explore the algorithm’s applicability in different situations, we inspect the performance of RecSLAM by breaking down the latency of grouping and tabu searching in Fig. 15. It should be noted that the undirected graphs generated in the test scenario before are all sparse matrices. Here, we use randomly generated matrices for testing.

See Fig. 15, as the robot number increases, the total scheduling overhead climbs. In our scenario, tabu search always contributes the largest cost in the entire algorithm operation since the number of robots is small (< 20). But in dense scenarios, when the number of robots increases, we can find that the grouping overhead becomes larger, even exceeding the tabu search. Because grouping is also a heuristic algorithm, it is greatly affected by the number of machines, but the algorithm runtime is still very small.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/0076024c02a97d188a2766bfcfaefc032d28d4b4bdd5d4e17f70d19799933241.jpg)  
Fig. 15. Algorithm runtime of different components in RecSLAM with varying number of simulated robots.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/06bc9a962878aa5d3d6dbd606e73bafb98916dc3fb071f4f3d6545f779ebc575.jpg)  
Fig. 16. Optimization effect of RecSLAM’s grouping algorithm with and without tabu optimization. Lower cross-group edge weights is better.

Since tabu search overhead accounts for a relatively large amount of the algorithm runtime in our scenario, we further explored the impact of tabu search though it is small. It can be seen from Fig. 16 that with the number of robots from 10 to 50, tabu search can obviously find a better solution, making the weights sum of the cross-group edges smaller.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/bb077f6685a874260bc35d7d88ac364c7bcfd36fb4890a953d636e22d67c24f2.jpg)  
Fig. 17. Total execution time of RecSLAM and the state-of-the-art cloud approach under WiFi environment.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/f65b359f50f99d0795173b58ae289790fa2a0b0513c6668dbeb438f1563f4de5.jpg)

Fig. 18. Recorded load level of the robot under the cloud approach and RecSLAM.  
![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/c96c28e1d1cb94694ef0d50c3c115576120db159fd54c21adb27d6f6b6daebc8.jpg)  
Fig. 19. Total execution time of RecSLAM and ablated scheduling policies with varying number of robots.

Performance Comparison: Our first experiment targets at comparing RecSLAM with the cloud serving approach. Fig. 17 shows their latency results with varying robot numbers, where RecSLAM enjoys lower latency across all settings. Concretely, RecSLAM reduces 39.31%, 26.50%, and 12.96% latency with six, eight, and ten robots, respectively. The result demonstrates the advantages came from edge servers’ assists. Specifically, the cloud approach lets robots compute SLAM locally and collects all robot maps in the cloud for centralized aggregation, where the workload on robots is always at a high level (Fig. 5).

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/baf891cb61aa0ec6552a87e10de13b0421fac87ae22cd1f02d53819e3eee8361.jpg)  
Fig. 20. Algorithm runtime of RecSLAM and the greedy counterpart with varying number of robots.

In contrast, RecSLAM forces to transfer the SLAM workload and separate part of the fusion tasks to edge servers, therefore alleviating the computing stress on both sides. As we measure during the RecSLAM runtime, the load level in robots records a mean of 53.91% and a minimum of 36.18%, significantly reducing the robot workload as shown in Fig. 18.

To assess the effectiveness of RecSLAM’s optimizing algorithms, we compare it with two heuristic counterparts and visualize the achieved latency results in Fig. 19, where RecSLAM refers to the corresponding scheduling policy mentioned above. The random algorithm randomly distributes the robots’ data to edge servers directly, recording the highest execution time among the three approaches. Particularly, when serving ten robots, it takes 2.79 s, which is 2.15× higher than RecSLAM. The greedy algorithm assigns the robot–edge data flow by first randomly initializing the grouping and next greedily selecting and swapping the robots in different groups. For example, given r robots and n edge servers $( r \geq n )$ , it first selects any r robots to, respectively, distribute to n groups. For the rest of the operations, we orderly select and then swap robots in different groups such that the overlapping degree with the edge server’s existing robot maps is maximized. Such a mechanism partially utilizes the overlapping information, but still lacks a global determination on overall overlapping degrees. As a result, it yields better partitioning than the random algorithm but is slower than RecSLAM. RecSLAM considers the relations between robots’ data by constructing an overlapping graph and scheduling the data flow in a global view. Fig. 19 shows that it reduces up to 53.41% and 12.16% latency over the random and greedy counterparts.

We further examine the algorithm runtime of RecSLAM and the greedy algorithm in Fig. 20 by simulating multiple robots in Gazebo [25] with numbers from 10 to 50. In the figure, we observe that the RecSLAM consumes a smaller overhead than Greedy with an average of 58.71% and up to 12.16% lower latency. Particularly, it takes less than 1 ms for ten robots and only 36 ms for 50 robots, indicating that RecSLAM runs lightweight even with a large number of robots.

## VII. RELATED WORK

To the best of our knowledge, RecSLAM is the first one that optimizes multirobot laser SLAM with the assist of edge computing. Only a few works are close to RecSLAM, and we list the most relative ones in Table I.

TABLE I  
COMPARING RECSLAM WITH RELATIVE EDGE-ASSISTED SLAM FRAMEWORKS
<table><tr><td colspan="2">Multi-robot support</td><td>Multi-edge deployment</td><td>Task decomposition</td><td>Bandwidth adaptability</td><td>Laser SLAM</td></tr><tr><td>CCM-SLAM [36]</td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>EdgeSLAM [37]</td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>Edge-SLAM [28]</td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>RecSLAM (Ours)</td><td></td><td></td><td></td><td></td><td></td></tr></table>

Multirobot SLAM Processing: The research on SLAM has experienced decades [38], [39]. With years of development and practice, a number of single-robot SLAM algorithms [40]–[47] has been proposed and deployed in the real world. For example, ORBSLAM2 [44] can utilize a variety of sensors to collect image data and estimate pose during movement to achieve positioning and mapping, and has been used in underwater scenarios [48], [49] and autonomous driving [50]. Cartographer [46] is a graph-optimized laser SLAM framework, which has been deployed on Google’s street view services.

While these single-robot SLAM systems have been widely used, multirobot SLAM remains to be intractable and still under exploration. A considerable part of the works focuses on the algorithm layer, including multisensor fusion [51]–[55], deep learning assistance [56]–[58], etc. CCM-SLAM [36] proposes a data-sharing mechanism between multiple robots to solve communication problems. CoSLAM [59] proposes a collaborative SLAM system with multiple moving cameras in a possibly dynamic environment. CorbSLAM [60] also uses a centralized method to detect the overlapping area of multiple local maps through the DBoW method and uses global optimization through bundling adjustment to fuse these maps. However, a majority of these works implicitly rely on centralized servers, ignoring the practical scenarios of edge deployment.

Edge-Assisted SLAM Computation: The emerging edge computing [20], [21], [61] is a distributed computing paradigm that sinks the workload from the remote cloud to the servers in physically approximate to the end devices and, therefore, significantly shortens the transmission distance and reduces the communication overhead. In recent years, the community has been explored using edge computing to assist SLAM computation. Dey and Mukherjee [29] applied a particle filtering algorithm to an edge-cloud architecture and proposed a dynamic offloading strategy. However, it only accounts for a single robot scenario without considering transmission bottlenecks and realistic prototype implementation. EdgeSLAM [37] propose an edge-assisted mobile semantic visual SLAM framework, which can perform visual SLAM in real time. However, incorporating semantic segmentation brings great limitations to the system’s scalability. Besides, it does not take multiple edge servers into consideration. Edge-SLAM [28] decouples SLAM from a modularity perspective, with the main focus on the resource usage on mobile devices. However, it only involves a single robot and an edge node, and cannot be directly extended to the communication between multirobot and multiedge. While these systems intend to optimize SLAM computation from edge-computing perspectives, they all focus on the single-robot scenario with peer-to-peer robot–edge synergy, ignoring the emerging multirobot scenarios. In contrast, our proposed system tackles a more complex SLAM deployment with the mutual presence of both multiple robots and multiple edge servers. Besides developing a collaborative execution framework between robots and edge servers, we further design a coordinating policy to enable high-performance multirobot SLAM map fusion.

## VIII. CONCLUSION

This article proposed RecSLAM, a multirobot laser SLAM system under the hierarchical robot–edge–cloud architecture. RecSLAM introduced edge servers to embrace the workload of SLAM computation and map fusion, which remarkably reduces the robot computing stress and saves the communication overhead between the edge and cloud. To optimize the overall performance, an efficient collaboration framework was further developed to orchestrate the data flow between robots and edge servers tailored to the heterogeneous network conditions. Extensive evaluations with both simulation and real prototype implementation demonstrated the effectiveness and efficiency of RecSLAM over existing solutions.

## REFERENCES

[1] C. Cadena et al., “Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1309–1332, Dec. 2016.

[2] B. Huang, J. Zhao, and J. Liu, “A survey of simultaneous localization and mapping with an envision in 6G wireless networks,” 2019, arXiv:1909.05214.

[3] S. Huang and G. Dissanayake, “A critique of current developments in simultaneous localization and mapping,” Int. J. Adv. Robot. Syst., vol. 13, no. 5, 2016, Art. no. 1729881416669482.

[4] P. Sun et al., “Scalability in perception for autonomous driving: Waymo open dataset,” in Proc. IEEE/CVF CVPR, Seattle, WA, USA, 2020, pp. 2446–2454.

[5] A. Buyval, A. Gabdullin, R. Mustafin, and I. Shimchik, “Realtime vehicle and pedestrian tracking for DiDi Udacity self-driving car challenge,” in Proc. IEEE ICRA, Brisbane, QLD, Australia, 2018, pp. 2064–2069.

[6] H. Fan et al., “Baidu Apollo EM motion planner,” 2018, arXiv:1807.08048.

[7] C. Badue et al., “Self-driving cars: A survey,” Expert Syst. Appl., vol. 165, Mar. 2021, Art. no. 113816.

[8] D. Rückert and M. Stamminger, “Snake-SLAM: Efficient global visual inertial SLAM using decoupled nonlinear optimization,” in Proc. Int. Conf. Unmanned Aircraft Syst. (ICUAS), Athens, Greece, 2021, pp. 219–228.

[9] X. Xiao and R. Murphy, “A review on snake robot testbeds in granular and restricted maneuverability spaces,” Robot. Auton. Syst., vol. 110, pp. 160–172, Dec. 2018.

[10] H. Wang, C. Zhang, Y. Song, B. Pang, and G. Zhang, “Threedimensional reconstruction based on visual SLAM of mobile robot in search and rescue disaster scenarios,” Robotica, vol. 38, no. 2, pp. 350–373, 2020.

[11] T. Taketomi, H. Uchiyama, and S. Ikeda, “Visual SLAM algorithms: A survey from 2010 to 2016,” IPSJ Trans. Comput. Vis. Appl., vol. 9, no. 1, pp. 1–11, 2017.

[12] D. M. Cole and P. M. Newman, “Using laser range data for 3D SLAM in outdoor environments,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Orlando, FL, USA, 2006, pp. 1556–1563.

[13] C. Wang, C. Wen, Y. Dai, S. Yu, and M. Liu, “Urban 3D modeling with mobile laser scanning: A review,” Virtual Reality Intell. Hardw., vol. 2, no. 3, pp. 175–212, 2020.

[14] S. Saeedi, M. Trentini, M. Seto, and H. Li, “Multiple-robot simultaneous localization and mapping: A review,” J. Field Robot., vol. 33, no. 1, pp. 3–46, 2016.

[15] R. Dubé, A. Gawel, H. Sommer, J. Nieto, R. Siegwart, and C. Cadena, “An online multi-robot slam system for 3D LiDARs,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Vancouver, BC, Canada, 2017, pp. 1004–1011.

[16] Y. Rizk, M. Awad, and E. W. Tunstel, “Cooperative heterogeneous multirobot systems: A survey,” ACM Comput. Surv., vol. 52, no. 2, pp. 1–31, 2019.

[17] P. Zhang, H. Wang, B. Ding, and S. Shang, “Cloud-based framework for scalable and real-time multi-robot SLAM,” in Proc. IEEE ICWS, San Francisco, CA, USA, 2018, pp. 147–154.

[18] V. K. Sarker, J. P. Queralta, T. N. Gia, H. Tenhunen, and T. Westerlund, “Offloading SLAM for indoor mobile robots with edge-fog-cloud computing,” in Proc. 1st Int. Conf. Adv. Sci. Eng. Robot. Technol. (ICASERT), Dhaka, Bangladesh, 2019, pp. 1–6.

[19] P. Yun, J. Jiao, and M. Liu, “Towards a cloud robotics platform for distributed visual SLAM,” in Proc. Int. Conf. Comput. Vis. Syst., 2017, pp. 3–15.

[20] W. Shi, J. Cao, Q. Zhang, Y. Li, and L. Xu, “Edge computing: Vision and challenges,” IEEE Internet Things J., vol. 3, no. 5, pp. 637–646, Oct. 2016.

[21] Z. Zhou, X. Chen, E. Li, L. Zeng, K. Luo, and J. Zhang, “Edge intelligence: Paving the last mile of artificial intelligence with edge computing,” Proc. IEEE, vol. 107, no. 8, pp. 1738–1762, Aug. 2019.

[22] S. Deng, H. Zhao, W. Fang, J. Yin, S. Dustdar, and A. Y. Zomaya, “Edge intelligence: The confluence of edge computing and artificial intelligence,” IEEE Internet Things J., vol. 7, no. 8, pp. 7457–7469, Aug. 2020.

[23] L. Zeng, X. Chen, Z. Zhou, L. Yang, and J. Zhang, “CoEdge: Cooperative DNN inference with adaptive workload partitioning over heterogeneous edge devices,” IEEE/ACM Trans. Netw., vol. 29, no. 2, pp. 595–608, Apr. 2021.

[24] T. Ouyang, X. Chen, L. Zeng, and Z. Zhou, “Cost-aware edge resource probing for infrastructure-free edge computing: From optimal stopping to layered learning,” in Proc. IEEE Real-Time Syst. Symp. (RTSS), Hong Kong, 2019, pp. 380–391.

[25] N. Koenig and A. Howard, “Design and use paradigms for gazebo, an open-source multi-robot simulator,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), vol. 3. Sendai, Japan, 2004, pp. 2149–2154.

[26] B. Schreck, C. Victorri-Vigneau, M. Guerlais, E. Laforgue, and M. Grall-Bronnec, “SLAM practice: A review of the literature,” Eur. Addict. Res., vol. 27, no. 3, pp. 161–178, 2021.

[27] A. Gautam and S. Mohan, “A review of research in multi-robot systems,” in Proc. IEEE 7th Int. Conf. Ind. Inf. Syst. (ICIIS), Chennai, India, 2012, pp. 1–5.

[28] A. J. B. Ali, Z. S. Hashemifar, and K. Dantu, “Edge-SLAM: Edgeassisted visual simultaneous localization and mapping,” in Proc. 18th Int. Conf. Mobile Syst. Appl. Serv., 2020, pp. 325–337.

[29] S. Dey and A. Mukherjee, “Robotic SLAM: A review from fog computing and mobile edge computing perspective,” in Proc. Adjunct 13th Int. Conf. Mobile Ubiquitous Syst. Comput. Netw. Serv., 2016, pp. 153–158.

[30] Y. Wu, K. Ni, C. Zhang, L. P. Qian, and D. H. K. Tsang, “NOMAassisted multi-access mobile edge computing: A joint optimization of computation offloading and time allocation,” IEEE Trans. Veh. Technol., vol. 67, no. 12, pp. 12244–12258, Dec. 2018.

[31] Y. Wu, L. P. Qian, K. Ni, C. Zhang, and X. Shen, “Delay-minimization nonorthogonal multiple access enabled multi-user mobile edge computation offloading,” IEEE J. Sel. Topics Signal Process., vol. 13, no. 3, pp. 392–407, Jun. 2019.

[32] Y. Wu, L. P. Qian, H. Mao, X. Yang, H. Zhou, and X. Shen, “Optimal power allocation and scheduling for non-orthogonal multiple access relay-assisted networks,” IEEE Trans. Mobile Comput., vol. 17, no. 11, pp. 2591–2606, Nov. 2018.

[33] G. Grisetti, C. Stachniss, and W. Burgard, “Improved techniques for grid mapping with Rao-blackwellized particle filters,” IEEE Trans. Robot., vol. 23, no. 1, pp. 34–46, Feb. 2007.

[34] J. Wu, G. Jiang, L. Zheng, and S. Zhou, “Algorithms for balanced graph bi-partitioning,” in Proc. HPCC, CSS, ICESS, Paris, France, 2014, pp. 185–188.

[35] K. Andreev and H. Räcke, “Balanced graph partitioning,” Theory Comput. Syst., vol. 39, no. 6, pp. 929–939, 2006.

[36] P. Schmuck and M. Chli, “CCM-SLAM: Robust and efficient centralized collaborative monocular simultaneous localization and mapping for robotic teams,” J. Field Robot., vol. 36, no. 4, pp. 763–781, 2019.

[37] J. Xu et al., “Edge assisted mobile semantic visual SLAM,” in Proc. INFOCOM, Toronto, ON, Canada, 2020, pp. 1828–1837.

[38] R. C. Smith and P. Cheeseman, “On the representation and estimation of spatial uncertainty,” Int. J. Robot. Res., vol. 5, no. 4, pp. 56–68, 1986.

[39] R. Smith, M. Self, and P. Cheeseman, “Estimating uncertain spatial relationships in robotics,” in Autonomous Robot Vehicles. New York, NY, USA: Springer, 1990, pp. 167–193.

[40] A. J. Davison, I. D. Reid, N. D. Molton, and O. Stasse, “MonoSLAM: Real-time single camera SLAM,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 29, no. 6, pp. 1052–1067, Jun. 2007.

[41] G. Klein and D. Murray, “Parallel tracking and mapping for small AR workspaces,” in Proc. 6th IEEE ACM Int. Symp. Mixed Augmented Reality, Nara, Japan, 2007, pp. 225–234.

[42] T. Qin, P. Li, and S. Shen, “VINS-mono: A robust and versatile monocular visual-inertial state estimator,” IEEE Trans. Robot., vol. 34, no. 4, pp. 1004–1020, Aug. 2018.

[43] M. Labbé and F. Michaud, “RTAB-map as an open-source LiDAR and visual simultaneous localization and mapping library for large-scale and long-term online operation,” J. Field Robot., vol. 36, no. 2, pp. 416–446, 2019.

[44] R. Mur-Artal and J. D. Tardós, “ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1255–1262, Oct. 2017.

[45] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM: A versatile and accurate monocular SLAM system,” IEEE Trans. Robot., vol. 31, no. 5, pp. 1147–1163, Oct. 2015.

[46] W. Hess, D. Kohler, H. Rapp, and D. Andor, “Real-time loop closure in 2D LiDAR SLAM,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Stockholm, Sweden, 2016, pp. 1271–1278.

[47] G. Grisettiyz, C. Stachniss, and W. Burgard, “Improving grid-based SLAM with Rao-blackwellized particle filters by adaptive proposals and selective resampling,” in Proc. IEEE Int. Conf. Robot. Autom., Barcelona, Spain, 2005, pp. 2432–2437.

[48] E. Vargas et al., “Robust underwater visual SLAM fusing acoustic sensing,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Xi’an, China, 2021, pp. 2140–2146.

[49] F. Hidalgo, C. Kahlefendt, and T. Bräunl, “Monocular ORB-SLAM application in underwater scenarios,” in Proc. OCEANS-MTS/IEEE Kobe Techno-Oceans (OTO), Kobe, Japan, 2018, pp. 1–4.

[50] F. Nobis, O. Papanikolaou, J. Betz, and M. Lienkamp, “Persistent map saving for visual localization for autonomous vehicles: An ORB-SLAM 2 extension,” in Proc. 15th Int. Conf. Ecol. Veh. Renew. Energies (EVER), Monte-Carlo, Monaco, 2020, pp. 1–9.

[51] T. Du, Y. H. Zeng, J. Yang, C. Z. Tian, and P. F. Bai, “Multi-sensor fusion SLAM approach for the mobile robot with a bio-inspired polarised skylight sensor,” IET Radar Sonar Navig., vol. 14, no. 12, pp. 1950–1957, 2020.

[52] W. Shao, S. Vijayarangan, C. Li, and G. Kantor, “Stereo visual inertial LiDAR simultaneous localization and mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Macau, China, 2019, pp. 370–377.

[53] T.-M. Nguyen, S. Yuan, M. Cao, T. H. Nguyen, and L. Xie, “VIRAL SLAM: Tightly coupled camera-IMU-UWB-Lidar SLAM,” 2021, arXiv:2105.03296.

[54] M. Palieri et al., “LOCUS: A multi-sensor lidar-centric solution for highprecision odometry and 3D mapping in real-time,” IEEE Robot. Autom. Lett., vol. 6, no. 2, pp. 421–428, Apr. 2021.

[55] X. Zuo et al., “LIC-fusion 2.0: LiDAR-inertial-camera odometry with sliding-window plane-feature tracking,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Las Vegas, NV, USA, 2020, pp. 5112–5119.

[56] R. Kang, J. Shi, X. Li, Y. Liu, and X. Liu, “DF-SLAM: A deeplearning enhanced visual slam system based on deep local features,” 2019, arXiv:1901.07223.

[57] S. Wang, R. Clark, H. Wen, and N. Trigoni, “DeepVO: Towards end-toend visual odometry with deep recurrent convolutional neural networks,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Singapore, 2017, pp. 2043–2050.

[58] Y. Almalioglu, M. R. U. Saputra, P. P. de Gusmão, A. Markham, and N. Trigoni, “GANVO: Unsupervised deep monocular visual odometry and depth estimation with generative adversarial networks,” in Proc. Int. Conf. Robot. Autom. (ICRA), Montreal, QC, Canada, 2019, pp. 5474–5480.

[59] D. Zou and P. Tan, “CoSLAM: Collaborative visual SLAM in dynamic environments,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 35, no. 2, pp. 354–366, Feb. 2013.

[60] F. Li, S. Yang, X. Yi, and X. Yang, “CORB-SLAM: A collaborative visual slam system for multiple robots,” in Proc. Int. Conf. Collab. Comput. Netw. Appl. Worksharing, 2017, pp. 480–490.

[61] Y. Mao, C. You, J. Zhang, K. Huang, and K. B. Letaief, “A survey on mobile edge computing: The communication perspective,” IEEE Commun. Surveys Tuts., vol. 19, no. 4, pp. 2322–2358, 4th Quart., 2017.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/227f7f621aabdfeba98aed91c60dd9afabee0a2d0bba9e1474777125a9f25673.jpg)  
Peng Huang (Graduate Student Member, IEEE) received the bachelor’s degree from Sun Yat-sen University, Guangzhou, China, in 2019, where he is currently pursuing the master’s degree with the School of Computer Science and Engineering.  
His research interests include mobile-edge computing and SLAM.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/5666e6ec286f2de8bb0fd53c0cc9f901bec8759df733fcf10ea44d147d2be6f9.jpg)

Liekang Zeng (Graduate Student Member, IEEE) received the bachelor’s degree from Sun Yat-sen University, Guangzhou, China, in 2018, where he is currently pursuing the Ph.D. degree.

He has published technical papers in leading international journals and conferences, such as IEEE/ACM TRANSACTIONS ON NETWORKING, IEEE TRANSACTIONS ON WIRELESS COMMUNICATIONS, ACM WWW, and IEEE RTSS. His current research interests include edge intelligence, mobile computing, and

distributed machine learning system.  
![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/286f5dbbb9eda3ba165900edee940c2e8ba7fa9d6a446f3c7b09ac9494b8c16a.jpg)

Xu Chen (Senior Member, IEEE) received the Ph.D. degree from The Chinese University of Hong Kong, Hong Kong, in 2012.

He worked as a Postdoctoral Research Associate with Arizona State University, Tempe, AZ, USA, from 2012 to 2014, and a Humboldt Scholar Fellow with the University of Göttingen, Göttingen, Germany, from 2014 to 2016. He is currently a Full Professor with Sun Yat-sen University, Guangzhou, China.

Prof. Chen received the 2014 Hong Kong Young 18

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/0351e185943966473d1b732842149882cd6edd742182def68053bf33d7c44ccf.jpg)

Scientist Runner-Up Award, the 2017 IEEE Communication Society Asia– Pacific Outstanding Young Researcher Award, the 2017 IEEE ComSoc Young Professional Best Paper Award, the Best Paper Runner-Up Award of 2014 IEEE International Conference on Computer Communications (INFOCOM), and the Best Paper Award of 2017 IEEE International Conference on Communications (ICC). He is an Associate Editor of IEEE TRANSACTIONS ON WIRELESS COMMUNICATIONS, IEEE INTERNET OF THINGS JOURNAL, and IEEE JOURNAL ON SELECTED AREAS IN COMMUNICATIONS Series on Network Softwarization and Enablers.

Ke Luo (Graduate Student Member, IEEE) received the B.S. degree in computer science from the School of Data and Computer Science, Sun Yat-sen University, Guangzhou, China, in 2018, where he is currently pursuing the Ph.D. degree.

His current research interests include cloud computing, mobile-edge computing, and distributed systems.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/7b8cf96ef16cfc5780defa053b9fd2519d5ea4d486cc481b97cb11a92f614f5e.jpg)

Zhi Zhou (Member, IEEE) received the B.S., M.E., and Ph.D. degrees from the School of Computer Science and Technology, Huazhong University of Science and Technology, Wuhan, China, in 2012, 2014, and 2017, respectively.

He is currently an Associate Professor with the School of Data and Computer Science, Sun Yat-sen University, Guangzhou, China. In 2016, he was a Visiting Scholar with the University of Göttingen, Göttingen, Germany.

Dr. Zhou was a recipient of the 2018 ACM Wuhan

and Hubei Computer Society Doctoral Dissertation Award and the Best Paper Award from the IEEE UIC 2018. He was nominated for the 2019 China Computer Federation Outstanding Doctoral Dissertation Award.

![](images/2022_Edge_Robotics__Edge-Computing-Accelerated_Multirobot_Sim/2579cd07e5d0682899769195d2977d7e5afff30f878754bc1c35cab1602072bb.jpg)

Shuai Yu (Member, IEEE) received the B.S. degree from Nanjing University of Post and Telecommunications, Nanjing, China, in 2009, the M.S. degree from Beijing University of Post and Telecommunications, Beijing, China, in 2014, and the Ph.D. degree from the University Pierre and Marie Curie (currently, Sorbonne Université), Paris, France, in 2018.

He is currently a Postdoctoral Research Fellow with the School of Data and Computer Science, Sun Yat-sen University, Guangzhou, China. His research

interests include wireless communications, edge computing, and machine learning.