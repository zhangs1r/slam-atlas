# FusionPortable: A Multi-Sensor Campus-Scene Dataset for Evaluation of Localization and Mapping Accuracy on Diverse Platforms

Jianhao Jiao<sup>2,4∗</sup>, Hexiang Wei<sup>2∗</sup>, Tianshuai Hu<sup>2∗</sup>, Xiangcheng Hu<sup>2∗</sup>, Yilong Zhu<sup>2</sup>, Zhijian He<sup>2</sup>, Jin Wu<sup>2</sup>, Jingwen Yu<sup>2,5</sup>, Xupeng Xie<sup>2</sup>, Huaiyang Huang<sup>2</sup>, Ruoyu Geng<sup>2</sup>, Lujia Wang<sup>2,4</sup>, Ming Liu<sup>1,2,3</sup>

Abstract— Combining multiple sensors enables a robot to maximize its perceptual awareness of environments and enhance its robustness to external disturbance, crucial to robotic navigation. This paper proposes the FusionPortable benchmark, a complete multi-sensor dataset with a diverse set of sequences for mobile robots. This paper presents three contributions. We first advance a portable and versatile multi-sensor suite that offers rich sensory measurements: 10Hz LiDAR point clouds, 20Hz stereo frame images, high-rate and asynchronous events from stereo event cameras, 200Hz inertial readings from an IMU, and 10Hz GPS signal. Sensors are already temporally synchronized in hardware. This device is lightweight, selfcontained, and has plug-and-play support for mobile robots. Second, we construct a dataset by collecting 17 sequences that cover a variety of environments on the campus by exploiting multiple robot platforms for data collection. Some sequences are challenging to existing SLAM algorithms. Third, we provide ground truth for the decouple localization and mapping performance evaluation. We additionally evaluate state-of-theart SLAM approaches and identify their limitations. The dataset, consisting of raw sensor measurements, ground truth, calibration data, and evaluated algorithms, will be released.

## I. INTRODUCTION

## A. Motivation

Multi-sensor fusion for robust perception is fundamental to various robotic applications. Different sensors can complement each other, and thus the system’s perception capability is enhanced with sensor fusion. Over the past decades, research on multi-sensor SLAM has made substantial progress. High-quality open datasets, which are collections of multisensor data and provide a suite of benchmark tools, significantly contribute to this advancement. On one hand, these datasets can waive inhibitive requirements on budget and workforce, such as system integration calibration and field operations. On the other hand, they investigate the advantages and limitations of current SLAM solutions and elaborately design practical, but challenging sequences [1], [2]. Several of them also introduce novel sensors and indicate future research opportunities [3]. Researchers can easily develop, validate, and rank their algorithms with others, thus accelerating the breakthroughs. However, existing datasets were mostly collected with a single data collection platform or simplified sensor configuration. Researchers may only utilize limited sensors to develop algorithms that has a risk of overfitting to a benchmark. Hence, we consider that a desirable dataset should fulfill the following four requirements.

1) Various sensors are required, making it possible to explore novel approaches to utilize them jointly.

2) Algorithm evaluation should be fairely conducted on various mobile robots. These robots perform different motion patterns that may challenge several SLAM algorithms’ assumptions.

3) Sequences have to cover from room-scale (meter-level) to large-scale (kilometer-level) environments to evaluate algorithms’ scalability.

4) Ground-truth trajectories and 3D maps are required to evaluate algorithms’ localization and surface reconstruction accuracy, respectively.

## B. Contributions

There appears to be an absence of compatible public datasets that satisfy these requirements, motivating us to propose a new SLAM benchmark.

This paper proposes the FusionPortable benchmark, a novel multi-sensor dataset with a set of sequences from diverse environments. Our contributions are presented threefold. First, a portable and versatile multi-sensor device is elaborately manufactured. Two RGB frame cameras are mounted on the left and right side, one high-frequency and high-precision IMU is mounted internally, and one RTK-GPS is installed on the top position. Moreover, thanks to current progress in sensory technology, both novel event cameras and high-resolution 3D LiDAR are available. Thus, we also integrate them with our sensor rig and investigate their performance. All these sensors are mounted on the same rigid aluminum-alloy-based parts. Thus, their spatial relation has a tiny dynamic deviation. The complete device has its own clock synchronization unit, processor, and battery, thus self-contained. Since its size, weight, and extensibility (see Fig. 1) are satisfying, we advance that it would be a plugand-play support to various mobile robots.

Second, we install the sensor rig on various platforms ranging from the handheld mode with a gimbal stabilizer, a quadruped robot, and an autonomous vehicle in performing distinguishable motion for the dataset construction. Various structured or semi-structured environments on The Hong Kong University of Science and Technology (HKUST) campus, including the lab, garden, canteen, corridor, escalator, and outdoor road, are examined in the dataset. Also, the collected sequences present several environmental changes caused by external light, moving objects, and scene texture. These issues are challenging to SLAM algorithms.

TABLE I  
COMPARISON WITH PREVIOUS DATASETS ON DATA-ACQUISITION PLATFORM, ENVIRONMENT, SENSOR TYPE, AND GROUND-TRUTH METHOD.
<table><tr><td rowspan="2">Dataset</td><td rowspan="2">Platform</td><td rowspan="2">Environment</td><td colspan="5">Sensor</td><td rowspan="2">GT Pose</td><td rowspan="2">GT Map</td></tr><tr><td>IMU</td><td></td><td></td><td></td><td>GPS LiDAR Frame Cam. Event Cam.</td></tr><tr><td>UZH-Event [3]</td><td>Handheld</td><td>In/Outdoors</td><td>√</td><td></td><td></td><td></td><td>√</td><td>Mocap</td><td></td></tr><tr><td>ETH-EuRoc [4]</td><td>MAV</td><td>Indoors</td><td>√</td><td></td><td></td><td>√</td><td></td><td>Mocap/LT</td><td>Nova MS50</td></tr><tr><td>TUM VI [5]</td><td>Handheld</td><td>In/Outdoors</td><td>√</td><td></td><td></td><td>√</td><td></td><td>Mocap</td><td></td></tr><tr><td>MIT DARPA [6]</td><td>Car</td><td>Urban</td><td>√</td><td>√</td><td>√</td><td>√</td><td></td><td>GPS/INS</td><td></td></tr><tr><td>KITTI [7]</td><td>Car</td><td>Urban</td><td>√</td><td>√</td><td>√</td><td>√</td><td></td><td>RTK-GPS/INS</td><td></td></tr><tr><td>Oxford RobotCar [8]</td><td>Car</td><td>Urban</td><td>√</td><td>√</td><td>V</td><td>√</td><td></td><td>GPS/INS/SLAM</td><td></td></tr><tr><td>UrbanLoc [9]</td><td>Car</td><td>Urban</td><td>√</td><td>√</td><td>√</td><td>√</td><td></td><td>GPS/INS</td><td></td></tr><tr><td>Newer College [10]</td><td>Handheld</td><td>Outdoors</td><td>√</td><td></td><td>√</td><td>√</td><td></td><td>6DoF ICP</td><td>BLK360</td></tr><tr><td>NCLT [11]</td><td>UGV</td><td>In/Outdoors</td><td>√</td><td>√</td><td>√</td><td>√</td><td></td><td>RTK-GPS/SLAM</td><td></td></tr><tr><td>M2DGR [12]</td><td>UGV</td><td>In/Outdoors</td><td>√</td><td>√</td><td>√</td><td>√</td><td>√</td><td>RTK-GPS/Mocap/LT</td><td></td></tr><tr><td>MVSEC [13]</td><td>Handheld/UAV/Motorcycle/Car</td><td>In/Outdoors</td><td>√</td><td>√</td><td>√</td><td>√</td><td>√</td><td>Mocap/SLAM</td><td></td></tr><tr><td>Ours (FusionPortable)</td><td>Handheld/Quad. Robot/UGV</td><td>In/Outdoors</td><td>√</td><td>√</td><td>√</td><td>√</td><td>√</td><td>Mocap/RTK-GPS/6DoF NDT</td><td>BLK360</td></tr></table>

Mocap: Motion capture system. LT: Laser tracker.

Third, besides ground-truth poses, we also provide groundtruth maps of most indoor sequences. We consider that measuring the mapping accuracy is crucial for evaluation. We also benchmark several state-of-the-art (SOTA) SLAM systems, including two vision-based methods and four LiDARbased approaches. To benefit the community, the dataset will be publicly released: https://ram-lab.com/file/ site/multi-sensor-dataset.

## II. RELATED WORK

There are extensive datasets for robotic perception. Here, we introduce related works with a focus on SLAM.

Several datasets were specifically designed for one type of sensor. Mueggler et al. [3] proposd the event camera dataset for the purpose of overcoming illumination and motion blur issues caused by frame cameras. Pomerleau et al. [1] proposed the point cloud dataset that covers a large spectrum of environmental structures to challenge registration algorithms. Handa et al. [14] promoted the research on RGB-D cameras by publishing the ICL-NUIM dataset.

Complementing vision sensors with inertial measurements, visual-inertial odometry (VIO) approaches can tremendously improve camera tracking accuracy and robustness. Relevant datasets have been reported. Burri et al. [4] presented the EuRoc dataset collected by a micro aerial vehicle (MAV) in an industrial environment and a room. Schubert et al. [5] put forward the TUM VI benchmark by collecting handheld sequences with a careful photometric calibration forwards.

The DARPA challenge has driven the development of autonomous vehicles. Huang et al. [6] presented the MIT DARPA dataset with over 90km sequence. Geiger et al. [7] presented the KITTI driving benchmark where diverse perception tasks are explored. There are other datasets targeting at long-term navigation [15] and urban challenges [9].

Several datasets were collected by handheld devices and other types of ground robots. Ramezani et al. [10] collected the Newer College Dataset with a handheld device. The NCLT dataset [11] facilitated the long-term SLAM research by collecting sequences in a college campus, over 147.4km traverse and 15 months. The M2DGR dataset covers various challenging scenarios such as entering lifts and indooroutdoor traverse [12] with a ground robot. Zhu et al. [13] proposed a multi-vehicle dataset for event-based perception.

Table I compares existing datasets with our work. In summary, our dataset is more complete from xx aspects: 1) raw and rich sensory measurements; 2) data collection on three different platforms including a legged robot; 3) groundtruth trajectories and 3D maps for algorithm evaluation.

## III. SYSTEM OVERVIEW

This section introduces sensors used in our dataset and how we achieve the spatio-temporal calibration between each sensor. Fig. 1 shows the handheld device equipped with multiple sensors and how it is mounted on three data collection platforms.

## A. Sensor Configuration

Sensors’ characteristics can be found in Table II. We use the Intel NUC to run sensor drivers, attach timestamps of sensor messages, and record messages into ROS bags on the Ubuntu system. The PC uses an i7 processor, 1TB solid-state drive (SSD), and 64GB DDR4 memory. Below, we provide detailed description of these sensors.

1) 3D LiDARs: We configure the OS1-128 LiDAR to provide accurate measurements of surrounding environments. This LiDAR has two attractive properties. First, an internal synchronized IMU outputs 100Hz linear accelerations and angular velocities. Second, it additionally outputs depth images, signal images, and ambient images of surroundings.

2) Stereo Frame Cameras: Two FILR BFS-U3-31S4C global-shutter color cameras are mounted at two sides on the system, facing directly forward. They are synchronized by an external trigger and capture high-resolution images at 20 fps. Their exposure time is set as fixed values to minimize the relative latency. Our experiments show that the average difference in timestamps of these images is below 1ms.

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/4564e9758b2319c2834ddedb88cb2c321bd795dfd0e64bad924d1e9862433177.jpg)  
(a)

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/f14737962037caa98dc84cfa32266436b577f4ae5077b556a697411449d31e8d.jpg)  
(b)

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/efa91e8321ccc91e02f838f86072849a6c2ddbb71aac1b7b0705cbb75c27f849.jpg)  
(c)

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/93a20257a0066b7746144857babff80143e73971473870bbd96af587a7420f20.jpg)  
(d)

Fig. 1. The multi-sensor device and data collection platform: (a) CAD model of the sensor rig, where axis directions are colored: red: X, green: Y, blue: Z. The sensor rig is rigidly mounted on (b) a gimbal stabilizer, (c) a quadruped robot, and (d) an apollo autonomous vehicle.  
![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/cea84577060698c7d9d5ca27172bf1486f0f9d115a71784112698652e4c1c87b.jpg)

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/feb9498a6f0d02d702d5527795642ce80c69f2ef987b76c6aa0d50393b7cd72b.jpg)  
(a) Garden

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/7f19883f596e42dbee6fdab366df7e0e562a233512ee25bbe341c079733f19cd.jpg)  
(b) Building  
(c) Campus Road  
Fig. 2. Scene Images of places of several sequences.

3) Stereo Event Cameras: Two event cameras are also configured. They possess several desirable properties: high temporal resolution, high dynamic range, and low power consumption. The cameras have a 346 × 260 resolution and an internal high-rate IMU output. Event cameras are synchronized using the trigger signal generated from the left camera (master) to deliver sync pulses to the right (slave) through an external wire. But there is no way to synchronize the image acquisition (around 10-20ms offset). To suppress the LiDAR’s laser light, both cameras are equipped with additional infrared filters. For indoor sequences, we manually set and fix the APS exposures, which helps to minimize the latency between cameras. For outdoor sequences, we use auto-exposure to avoid over- or under-exposure.

4) Inertial Measurement Unit: A tactical-grade STIM300 IMU that is rigidly mounted below the LiDAR is employed as the main inertial sensor of the system. It features a high update rate (200Hz) and low noisy and drifting measurements. Its bias Instability is around 0.3<sup>◦</sup>/h.

5) Global Positioning Systsem: We additionally install a ZED-F9P RTK-GPS device on the top of the LiDAR. In outdoor scenes, the GPS is activated and provides accurate latitude, longitude, and altitude readings. But it may sometimes become unstable due to buildings’ occlusion.

## B. Sensor Calibration

We carefully calibrate intrinsics of individual sensors, extrinsics, and overall time latency between sensors in advance. We define the coordinate system of the STIM300 IMU as the body frame. We provide calibration data and reports in the dataset website.

1) Clock Synchronization: We use an FPGA to generate an external signal trigger to synchronize clocks of all sensors. This can guarantee data collection across multiple sensors with minimum latency. The FPGA receives a pulse-persecond (PPS) signal from the GPS and outputs 200, 20, 10Hz signal to the IMU, cameras, and LiDAR, respectively. The FPGA switches to use its internal clock to enable the time synchronization in GPS-denied scenes.

TABLE II  
SENSORS AND CHARACTERISTICS
<table><tr><td>Sensor</td><td>Characteristics</td></tr><tr><td>3D LiDAR</td><td>OS1-128, 120m range@10Hz; FOV: 45°vert., 360°horiz. Image: 1028 × 128@10Hz IMU: ICM20948@100Hz, 9-axis MEMS, intrinsic calibrated</td></tr><tr><td>Frame</td><td>Stereo color cameras: 2 FILR BFS-U3-31S4C</td></tr><tr><td>Camera</td><td>Resolution: 1024 × 768, global shutter@20Hz FOV: 66.5° vert., 82.9°horiz.</td></tr><tr><td>Event Camera</td><td>Stereo color event cameras: 2 DAVIS346 Resolution: 346 × 240; FOV: 67°vert., 83° horiz.</td></tr><tr><td>IMU</td><td>IMU: MPU6150@1000Hz, 6-axis MEMS, intrinsic calibrated STIM300@200Hz, Bias Instability 0.3° /h, Allan Var. @25°C</td></tr><tr><td>GPS</td><td>ZED-F9P RTK-GPS@10Hz, 4 concurrent GNSS, L1/L2/L5 RTK</td></tr></table>

2) Stereo Camera Calibration: Intrinsics and extrinsics of our stereo frame and event cameras are estimated using the Matlab calibration toolbox, where the pinhole camera and radial-tangential distortion model are used. We move the sensor suite before a checkerboard to collect a sequence of images. We evenly sample images as the calibration data and manually remove outliers with high reprojection errors.

3) Camera-IMU Extrinsic Calibration: The intrinsics of IMUs are calibrated using the Allen derivation toolbox<sup>1</sup> that estimates the noisy density and random walk for gyroscope and accelerometer measurements. After that, the spatial and temporal parameters of a camera w.r.t. an IMU are obtained by the Kalibr [16]. Our system consists of 4 IMUs: STIM300, ICM20948 in the LiDAR, and two MPU6050 in the DAVIS346 event cameras. Thus, we calibrate the intrinsics of these IMUs, and estimate extrinsics of these sensor pairs: hSTIM300, frame camerasi, hSTIM300, event camerasi, hleft MPU6050, left DAVIS346i, and hright MPU6050, right DAVIS346i.

4) Camera-LiDAR Extrinsic Calibration: Given initial extrinsics, we further refine the camera-LiDAR extrinsics. The checkerboard is the calibration target that provides distinctive corners and boundaries for data association. We extend the work proposed by Zhou et al. [17] by improving feature extraction and matching step. We instead extract the outer corners of the board from point clouds and images. The extrinsics are optimized by minimizing the distance of all corresponding corners.

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/0c2d592d8219bac88a33ee0daeb90dbb2809ed06153f306dcb14ef2dade696bf.jpg)  
(a) Canteen

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/5ab37ddbec5decb93bc8abec17336ce627d74a7f3145c0f5adfdc360443b2cc5.jpg)  
(b) Escalator

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/9e69f892b973adee04d7ffbf421502622b954f5efe6768f4e18cf2a47695fa24.jpg)  
(c) Corridor

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/43356f9fda8ff812471797a633a908a6efe83e08ebe95b5390e41355928c45c3.jpg)  
(d) Road

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/847aadefe84c3ed16c996419e49e6eaf5bf4a79a528a1d191e4f98ef511d5d29.jpg)

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/ff17ecb540c3ffe76d9cbbd29fd7e53ff0eecb4acd057a0b80d02e3e3beb0d46.jpg)  
(e) MCR

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/af3ec0ea60be0cbc9dee0c1eb41a9567655aaa439fe41a4ab684b24621a221c1.jpg)  
(g) Road

(f) Building  
![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/8e693d323df77e77d5579c283c65dde51bdefa1e4fe49ce90c224fd4a20309cf.jpg)  
(h) Canteen

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/789db09c266db5b9c88b37b3006d776643ea5af9037fa2856c4f56639152ce5b.jpg)  
(i) Road  
Fig. 3. Sample sensor measurements. (a)-(d): images captured by the frame camera. (e)-(f): images augmented by positive events (red) and negative events (blue). (h)-(i): 3D point clouds of the LiDAR. The grid size is 10m.

## IV. DATASET DESCRIPTION

This section first introduces the overall features of different sequences, which stand as our basic criteria for data collection. Details are then described, including the ground truth estimation method and dataset format.

## A. Sequences

The collected sequences should cover various environments, lighting conditions, motion patterns, dynamic objects, etc. We categorize major characteristics of our collected sequences as follows:

1) Location: Environmental locations are divided into indoors and outdoors. GPS signal is available but sometimes unstable in outdoor environments.

2) Structure: Structured environments can mainly be explained using geometric primitives (e.g., offices or buildings), while semi-structured environments have both geometric and complex elements like trees and sundries. Scenarios like narrow corridors are structured but may cause state estimators.

3) Lighting Condition: Frame cameras are sensitive to external lighting conditions. Both weak and strong light may raise challenges to visual processing algorithms.

4) Appearance: Texture-rich scenes facilitate visual algorithms to extract stable features (e.g., points and lines), while textureless may negatively affect the performance. Also, many events are triggered in texture-rich scenes.

5) Motion Pattern: Slow, normal, and fast motion may be performed. Regarding mounted platforms, the handheld device performs arbitrary 6-DoF and jerky motions, the device installed on a gimbal stabilizer conducts 6- DoF but stable motions, the quadruped robot mostly performs planar but jerky motions. In contrast, the vehicle performs planar movements at a constant speed.

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/8c30c6224f70a40678d523a318ba54abe973b2fb44c1197c30c664bebc3e813b.jpg)  
(a) Motion Capture Room

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/2947b34cb25dd42e5ecdf3b4e12d9ececb60b21af931e633fe8fbcee40c76f46.jpg)  
(b) Building  
Fig. 4. Ground-truth point cloud in color of the motion capture room, corridor, and building scenario. Point cloud data was recorded by the Leica BLK360 laser scanner. They are used to generate trajectory groundtruth and evaluate algorithms’ reconstruction accuracy.

6) Object Motion: In dynamic environments, several elements are moving while the data are captured. The more time of the data capture, the more deformed the elements will be (e.g., pedestrians or cars) [1]. In contrast, moving objects are few in static environments.

Table III summaries key features of each sequence, Fig. 2 shows several scene pictures, and Fig. 3 illustrates sample sensor data. The motion capture room is abbreviated as the MCR in the following sensors.

## B. Groundtruth Generation

Most sequences provide ground-truth poses for algorithm evaluation. In several indoor scenes, we also provide groundtruth maps of surrounding environments. The ground truth generation is detailed as follows:

• Ground-truth maps: In small- or middle-scale environments, we use the Leica BLK360 laser scanner to record the structure’s high-resolution colorized 3D dense map with millimeter accuracy from multiple locations. Fig. 4 visualizes three examples.

• Ground-truth poses: In the motion capture room, we use the OptiTrack system to measure the pose of the center of reflective balls at 120Hz with millimeter accuracy. The OptiTrack is directly connected with the same PC to record poses to minimize the time latency. The extrinsics from the balls’ center to the body frame of the sensor rig are solved by the hand-eye calibration approach. In middle-scale environments that are covered by the ground-truth maps, we employ the NDT-based 6-DoF localization [18] to estimate LiDAR’s poses in a prior map as the ground-truth trajectory. In outdoor environments, we fuse the RTK GPS signal with LiDARinertial measurements to obtain accuracy trajectories based on the LIO-SAM [19].

## C. Data Format and Post-Processing

Data were collected in the ROS environment. We provide both ROS bags and individual data files for better usage:

1) env.bag is the raw rosbag obtained from the data collection process. It can be parsed using ROS tools.

2) env ref.bag is the refined rosbag where sensor data are post-processed with below steps.

TABLE III  
SOME STATISTICS AND FEATURES OF EACH SEQUENCE
<table><tr><td>Platform</td><td>Sequence</td><td>T[s]</td><td>D[m]</td><td>||¯||[m/s]</td><td>Location</td><td>Structure</td><td>Lighting</td><td>Texture</td><td>Motion</td><td>Object</td><td>GT Pose</td><td>GT Map</td></tr><tr><td rowspan="10">Handheld</td><td>canteen_night</td><td>290</td><td>270</td><td>0.93</td><td>indoors</td><td>structured</td><td>weak</td><td>rich</td><td>6-DoF</td><td>static</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>canteen_day</td><td>230</td><td>250</td><td>1.09</td><td>indoors</td><td>structured</td><td>normal</td><td>rich</td><td>6-DoF</td><td>static</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>garden_night</td><td>280</td><td>265</td><td>0.94</td><td>indoors</td><td>structured</td><td>weak</td><td>rich</td><td>6-DoF</td><td>static</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>garden_day</td><td>170</td><td>173</td><td>1.02</td><td>indoors</td><td>structured</td><td>normal</td><td>rich</td><td>6-DoF</td><td>static</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>corridor_day</td><td>572</td><td>669</td><td>1.17</td><td>indoors</td><td>structured</td><td>weak</td><td>less</td><td>6-DoF</td><td>static</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>escalator_day</td><td>315</td><td>263</td><td>0.84</td><td>indoors</td><td>structured</td><td>strong</td><td>rich</td><td>6-DoF, height changes</td><td>dynamic</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>building_day</td><td>599</td><td>666</td><td>1.11</td><td>indoors</td><td>structured</td><td>normal</td><td>rich</td><td>6-DoF</td><td>dynamic</td><td>6-DoF NDT</td><td>Yes</td></tr><tr><td>MCR_slow</td><td>48</td><td>50</td><td>1.03</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>6-DoF, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_normal</td><td>45</td><td>52</td><td>1.26</td><td>indoors</td><td>semi-structured semi-structured</td><td>normal</td><td>rich</td><td>6-DoF, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_fast</td><td>34</td><td>59</td><td>1.76</td><td>indoors</td><td></td><td>normal</td><td>rich</td><td>6-DoF, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td rowspan="6">Quadruped Robot</td><td>MCR_slow_00</td><td>147</td><td>26</td><td>0.18</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_slow_01</td><td>127</td><td>28</td><td>0.28</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_normal_00</td><td>103</td><td>48</td><td>0.54</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_normal_01</td><td>95</td><td>43</td><td>0.52</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_fast_00</td><td>99</td><td>48</td><td>0.56</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>MCR_fast_01</td><td>121</td><td>90</td><td>0.83</td><td>indoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar, jerky</td><td>static</td><td>OptiTrack</td><td>Yes</td></tr><tr><td>Apollo</td><td>campus_road</td><td>1186</td><td>1887</td><td>1.62</td><td>outdoors</td><td>semi-structured</td><td>normal</td><td>rich</td><td>planar</td><td>dynamic</td><td>SLAM</td><td>No</td></tr></table>

T: Total time. D: Total distance traveled. MCR: motion capture room. ||v||: Mean linear velocity.

3) data/ stores individual sensor data from the env.bag. Each data has its timestamp that can be retrieved from the timestamps.txt.

4) data ref kitti/ follows the KITTI format [7] to store sensor data from data/.

We have three steps to post-process the raw data to generate the env ref.bag: 1) caused by unperfect IMUs (like the MPU6050), several missing measurements are linearly interpolated; 2) poses provided by the motion capture system are transformed into the body frame with the hand-eye calibration results; and 3) event packages are republished at around 1000 Hz for several event-based algorithms [20].

Unrectified RGB images are stored. Events are stored with timestamps, pixel locations, and polarity. IMU measurements are also stored with timestamps, gyroscope measurements, accelerometer measurements, and covariances. Calibration parameters are stored in yaml files.

## V. EXPERIMENT

As one of the applications, we can use this dataset to benchmark SOTA SLAM systems. Here, we evaluate several open-source systems with different sensor combinations and methodologies: VINS-Fusion (IMU+stereo frame cameras) [21], ESVO (stereo event cameras) [20], A-LOAM (LiDARonly) [22], LIO-Mapping (IMU+LiDAR) [23], LIO-SAM (IMU+LiDAR) [19], and FAST-LIO2 (IMU+LiDAR) [24]. Their data loaders are modified to fit our dataset format and also released. We calculate the mean absolute trajectory error (ATE) of estimated trajectories w.r.t. the ground truth. For LiDAR-based systems, we also report the mapping accuracy on two sequences by calculating the mean point-to-point error of algorithms’ maps w.r.t. the ground-truth maps.

The quantitative localization results are reported in Table IV. “LC” indicates that the loop closure module is used. “×” means that algorithms fail to finish the sequence. ESVO’s results are not shown here since it cannot finish all sequences. It requires events to be continuously triggered to generate reliable time surface maps for camera tracking. But all these sequences contain textureless scenarios or static motion. Its immediate results on mapping and tracking are shown in the dataset website. VINS-Fusion and FAST-LIO2 fail in some cases since they cannot initialize well at the beginning of the sequence. Without the aid of the IMU, A-LOAM cannot handle jerky and rapid motion and thus performs poorly on two MCR sequences and all sequences on the quadruped robot. Although FAST-LIO2 has a superior realtime performance based on the filter-based state estimator and efficient tree structure, it sometimes has unreliable results on several sequences. Surprisingly, LIO-SAM performs well on all quadruped robot-based sequences, even at large rotated and fast motion. The corridor day sequence is challenging to all methods, where the scene is textureless and structureless.

TABLE IV  
LOCALIZATION ACCURACY.
<table><tr><td>Platform</td><td>Sequence</td><td>VINS- Fusion (LC)</td><td>A- LOAM</td><td>LIO- Mapping</td><td>LIO- SAM</td><td>FAST- LIO2</td></tr><tr><td rowspan="9">Handheld</td><td>canteen_night</td><td>0.409</td><td>0.067</td><td>0.097</td><td>0.063</td><td>0.071</td></tr><tr><td>canteen_day</td><td>0.691</td><td>0.057</td><td>0.088</td><td>0.053</td><td>0.057</td></tr><tr><td>garden_night</td><td>0.328</td><td>0.567</td><td>0.242</td><td>0.254</td><td>0.205</td></tr><tr><td>garden_day</td><td>0.518</td><td>0.528</td><td>0.097</td><td>0.069</td><td>0.068</td></tr><tr><td>corridor_day</td><td>1.807</td><td>0.416</td><td>1.755</td><td>0.594</td><td>1.563</td></tr><tr><td>escalator_day</td><td>2.127</td><td>0.981</td><td>0.346</td><td>0.207</td><td>4.193</td></tr><tr><td>building_day</td><td>12.861</td><td>1.580</td><td>0.916</td><td>0.222</td><td>0.146</td></tr><tr><td>MCR_slow</td><td>X</td><td>0.087</td><td>0.042</td><td>0.063</td><td>0.114</td></tr><tr><td>MCR_normal</td><td>0.168</td><td>0.328</td><td>0.052</td><td>0.082</td><td>0.121</td></tr><tr><td rowspan="7">Quad.</td><td>MCR_fast</td><td>X</td><td>0.416</td><td>0.099</td><td>0.117</td><td>X</td></tr><tr><td>MCR_slow_00</td><td>0.096</td><td>0.120</td><td>0.032</td><td>0.023</td><td>0.047</td></tr><tr><td>MCR_slow_01</td><td>0.081</td><td>0.054</td><td>0.030</td><td>0.030</td><td>0.051</td></tr><tr><td>MCR_normal_00</td><td>0.094</td><td>0.492</td><td>0.093</td><td>0.042</td><td>0.127</td></tr><tr><td>MCR_normal_01</td><td>0.086</td><td>0.635</td><td>0.390</td><td>0.040</td><td>0.068</td></tr><tr><td>MCR_fast_00</td><td>0.264</td><td>4.601</td><td>2.405</td><td>0.052</td><td>0.408</td></tr><tr><td>MCR_fast_01</td><td>0.130</td><td>8.264</td><td>2.210</td><td>0.066</td><td>1.495</td></tr></table>

We also evaluate the mapping quality of A-LOAM and LIO-SAM on the corridor day and garden day sequences. The distance map is in Fig. 6. The mean distance is 0.938m and 0.597m respectively. Especially for the corridor mapping, A-LOAM’s map has a large drift on the z-axis.

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/1b421080670fdcadabf97299b5664d34cd5f9aa75a1f50458cde64c7ce31c10a.jpg)  
(a) MCR fast 00

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/bb206c0cf024fb5c866f6c83d1c495c61a2a1a509dc455bf5a407a18c43a90c9.jpg)  
(b) Campus road day

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/14089fbbb633a3b2a9ceb1d5798ddd55e533e31e77d0b64e5176ec9b45036489.jpg)  
(c) Garden day

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/d2b9c9301cab1293d3b98aef6e40190cbb025a0f48ece5e8064a124d3733ab85.jpg)  
(d) Escalator day  
Fig. 5. Trajectories of the algorithms on four sequences: MCR fast 00, campus road day, garden day, and escalator day w.r.t. the ground truth.

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/5d13652cd5d33bf27a81bb3347825a9e67a53d04d177ca7913b30e8eed3afd6f.jpg)  
(a) Corridor day

![](images/2022_FusionPortable__A_Multi-Sensor_Campus-Scene_Dataset_for_/357415c413f16ca508f5fb7dcfd487d57e29e8392f83aaa193251950fba65aef.jpg)  
(b) Garden day  
Fig. 6. Evaluation of (a) A-LOAM’s and (b) LIO-SAM’s mapping accuracy.

## VI. CONCLUSION

This paper presented the FusionPortable benchmark, a multi-sensor dataset from diverse campus scenes on various platforms. We advanced the self-contained and plug-and-play multi-sensor rig that significantly enhances the preception capability of mobile robots. With the release of this dataset, we intended to challenge current SLAM approaches and encouraged future research. As the future work, we plan to extend this dataset beyond the campus-scale environments.

## REFERENCES

[1] F. Pomerleau, M. Liu, F. Colas, and R. Siegwart, “Challenging data sets for point cloud registration algorithms,” The International Journal of Robotics Research, vol. 31, no. 14, pp. 1705–1711, 2012.

[2] W. Wang, D. Zhu, X. Wang, Y. Hu, Y. Qiu, C. Wang, Y. Hu, A. Kapoor, and S. Scherer, “Tartanair: A dataset to push the limits of visual slam,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 4909–4916.

[3] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, and D. Scaramuzza, “The event-camera dataset and simulator: Event-based data for pose estimation, visual odometry, and slam,” The International Journal of Robotics Research, vol. 36, no. 2, pp. 142–149, 2017.

[4] M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M. W. Achtelik, and R. Siegwart, “The euroc micro aerial vehicle datasets,” The International Journal of Robotics Research, vol. 35, no. 10, pp. 1157–1163, 2016.

[5] D. Schubert, T. Goll, N. Demmel, V. Usenko, J. Stuckler, and D. Cre-¨ mers, “The tum vi benchmark for evaluating visual-inertial odometry,” in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018, pp. 1680–1687.

[6] A. S. Huang, M. Antone, E. Olson, L. Fletcher, D. Moore, S. Teller, and J. Leonard, “A high-rate, heterogeneous data set from the darpa urban challenge,” The International Journal of Robotics Research, vol. 29, no. 13, pp. 1595–1601, 2010.

[7] A. Geiger, P. Lenz, C. Stiller, and R. Urtasun, “Vision meets robotics: The kitti dataset,” The International Journal of Robotics Research, vol. 32, no. 11, pp. 1231–1237, 2013.

[8] W. Maddern, G. Pascoe, C. Linegar, and P. Newman, “1 year, 1000 km: The oxford robotcar dataset,” The International Journal of Robotics Research, vol. 36, no. 1, pp. 3–15, 2017.

[9] W. Wen, Y. Zhou, G. Zhang, S. Fahandezh-Saadi, X. Bai, W. Zhan, M. Tomizuka, and L.-T. Hsu, “Urbanloco: A full sensor suite dataset for mapping and localization in urban scenes,” in 2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2020, pp. 2310–2316.

[10] M. Ramezani, Y. Wang, M. Camurri, D. Wisth, M. Mattamala, and M. Fallon, “The newer college dataset: Handheld lidar, inertial and vision with ground truth,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 4353– 4360.

[11] N. Carlevaris-Bianco, A. K. Ushani, and R. M. Eustice, “University of michigan north campus long-term vision and lidar dataset,” The International Journal of Robotics Research, vol. 35, no. 9, pp. 1023– 1035, 2016.

[12] J. Yin, A. Li, T. Li, W. Yu, and D. Zou, “M2dgr: A multi-sensor and multi-scenario slam dataset for ground robots,” IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 2266–2273, 2021.

[13] A. Z. Zhu, D. Thakur, T. Ozaslan, B. Pfrommer, V. Kumar, and<sup>¨</sup> K. Daniilidis, “The multivehicle stereo event camera dataset: An event camera dataset for 3d perception,” IEEE Robotics and Automation Letters, vol. 3, no. 3, pp. 2032–2039, 2018.

[14] A. Handa, T. Whelan, J. McDonald, and A. J. Davison, “A benchmark for rgb-d visual odometry, 3d reconstruction and slam,” in 2014 IEEE international conference on Robotics and automation (ICRA). IEEE, 2014, pp. 1524–1531.

[15] D. Barnes, M. Gadd, P. Murcutt, P. Newman, and I. Posner, “The oxford radar robotcar dataset: A radar extension to the oxford robotcar dataset,” in 2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2020, pp. 6433–6438.

[16] P. Furgale, J. Rehder, and R. Siegwart, “Unified temporal and spatial calibration for multi-sensor systems,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2013, pp. 1280–1286.

[17] L. Zhou, Z. Li, and M. Kaess, “Automatic extrinsic calibration of a camera and a 3d lidar using line and plane correspondences,” in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018, pp. 5562–5569.

[18] K. Koide, J. Miura, and E. Menegatti, “A portable three-dimensional lidar-based system for long-term and wide-area people behavior measurement,” International Journal of Advanced Robotic Systems, vol. 16, no. 2, p. 1729881419841532, 2019.

[19] T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus, “Lio-sam: Tightly-coupled lidar inertial odometry via smoothing and mapping,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 5135–5142.

[20] Y. Zhou, G. Gallego, and S. Shen, “Event-based stereo visual odometry,” IEEE Transactions on Robotics, vol. 37, no. 5, pp. 1433–1450, 2021.

[21] T. Qin, S. Cao, J. Pan, and S. Shen, “A general optimization-based framework for global pose estimation with multiple sensors,” arXiv preprint arXiv:1901.03642, 2019.

[22] J. Zhang and S. Singh, “Loam: Lidar odometry and mapping in realtime.” in Robotics: Science and Systems, vol. 2, no. 9. Berkeley, CA, 2014, pp. 1–9.

[23] H. Ye, Y. Chen, and M. Liu, “Tightly coupled 3d lidar inertial odometry and mapping,” in 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019, pp. 3144–3150.

[24] W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, “Fast-lio2: Fast direct lidar-inertial odometry,” IEEE Transactions on Robotics, 2022.