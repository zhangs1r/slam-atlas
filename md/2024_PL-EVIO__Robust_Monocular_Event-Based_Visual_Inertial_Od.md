# PL-EVIO: Robust Monocular Event-Based Visual Inertial Odometry With Point and Line Features

Weipeng Guan , Peiyu Chen , Yuhan Xie , Member, IEEE, and Peng Lu

Abstract— Robust state estimation in challenge situations is still an unsolved problem, especially achieving onboard pose feedback control for aggressive motion. In this paper, we propose robust and real-time event-based visual-inertial odometry (VIO) that incorporates event, image, and inertial measurements. Our approach utilizes line-based event features to provide additional structure and constraint information in human-made scenes, while point-based event and image features complement each other through well-designed feature management. To achieve reliable state estimation, we tightly couple the point-based and line-based visual residuals from the event camera, the point-based visual residual from the standard camera, and the residual from IMU pre-integration using a keyframe-based graph optimization framework. Experiments in the public benchmark datasets show that our method can achieve superior performance compared with the state-of-the-art image-based or event-based VIO. Furthermore, we demonstrate the effectiveness of our pipeline through onboard closed-loop quadrotor aggressive flight and large-scale outdoor experiments. Videos of the evaluations can be found on our website: https://youtu.be/KnWZ4anBMK4.

Note to Practitioners—Driven by the need for real-time closedloop control for drones under aggressive motion and broad illumination environments, many existing VIO systems fail to meet these requirements due to the inherent limitations of standard cameras. Event cameras are bio-inspired sensors that capture pixel-level illumination changes instead of the intensity image with a fixed frame rate, which can provide reliable visual perception during high-speed motions and in high dynamic range scenarios. Therefore, developing state estimation algorithms based on event cameras offers exciting opportunities for robotics. However, adopting event cameras is challenging due to the event streams being composed of asynchronous events which are fundamentally different from the synchronous intensity images. Moreover, event cameras output minimal information or even noise when the relative motion between the camera and the scene is limited, such as in a still state, while standard cameras can provide rich perception information in most scenarios. In this paper, we propose a robust, high-accurate, and realtime optimization-based monocular event-based VIO framework

that tightly fuses the event, image, and IMU measurement together. Owing to the well-designed framework and good feature management, our system can provide robust and reliable state estimation in challenging environments. The efficiency of our system is adequate to achieve real-time operation on platforms with limited resources, such as providing onboard pose feedback for quadrotor flights.

Index Terms— Event cameras, event-based VIO, aggressive quadrotor, sensor fusion, robotics, SLAM.

## I. INTRODUCTION

## A. Motivations

TANDARD cameras have inherent limitations, including Ssensing latency and low dynamic range, which is challenging for image-based Visual Odometry (VO), Visual Inertial Odometry (VIO), and Simultaneous Localization and Mapping (SLAM) systems to detect and track features under high-speed motion or high-dynamic-range (HDR) scenarios. Specifically, robust state estimation is vital for real-time feedback control in aggressive motion (e.g. onboard quadrotor flip, as shown in Fig.1(a)), since even tiny drifts or momentary poor feature tracking can potentially lead to a crash. Event cameras offer exciting opportunities to solve the aforementioned problems, which possess several advantages over standard cameras, including low latency (µs-level), HDR (140 dB), and no motion blur [1].

Most of the research in both image and event-based SLAM/VO/VIO rely on point-based features, while it is important to note that human-made structures often exhibit regular geometric shapes, such as lines or planes. Therefore, pointbased features may not always be the optimal representation for visual tracking in all scenarios. Performance degeneracy might occur when only using point-based features, while point-based features are more common in natural scenes. Therefore, for heterogeneous event-based information utilization, we design and extract the line-based feature in the event stream to improve the performance of purely pointbased features, since the line-based features can reflect more geometric structure information than point-based features [2], [3] [4]. As can be seen in Fig.1(b) Fig.3, and Fig.11(b), the integration of the line-based feature and point-based feature can further ensure a more uniform distribution of the features and provide additional constraints on scene structure.

In addition, compared to standard cameras, event cameras are capable of providing reliable visual perception during high-speed motion and HDR scenarios. However, when the event camera and the scene have restricted relative motion, such as in a static state, event cameras may produce limited information or even noise. Although the standard camera encounter difficulties during high-speed motion or in HDR scenarios, it can provide rich intensity value of the scenes under uniform motion or favorable lighting conditions.

Point-based Image Features  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/6c0d9195ebfc3735837702d1232130345c655992a984b46024a84e981232c513.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/04e43c5b1629ffa41c76a62f9d7b6cd79b0b5c29d16a70013b9c06f7f570ba3c.jpg)

(a)  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/bac967678794b2bcc930b1f128c171ce63aa3217583133272300f7f3f64aad10.jpg)  
(b)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/6e98227cff0c0e9be54a9d138cf2101b0edc5803ffcaf5c5bb7de37baba991bf.jpg)  
Fig. 1. (a) Our PL-EVIO combines events, images, and IMU to provide robust state estimation during aggressive motion. It can provide onboard feedback-control for quadrotors with limited computational resources. (b) Our PL-EVIO in the outdoor environment. Left: event-corner features in the event; Middle: line-based features in the event; Right: point-based features in the image.

Observing this complementary, we propose a monocular VIO framework for a sensor setup that includes event, image, and inertial measurement unit (IMU) data, with a well-designed feature management system. Our VIO framework includes the purely event-based VIO (EIO), and the event with image-based VIO (EVIO). More specifically, we first implement a motion compensation algorithm using the IMU data to correct the motion of each event according to its individual timestamp, including rotation and translation motion, into the same timestamp. After that, utilizing the event-corner features detection and tracking approach developed in our previous EIO work [5]. We conduct an EIO framework, including the line-based event features and the event-corner point features, termed PL-EIO (Event+IMU), to perform robust state estimation. Finally, we integrate image measurements into our PL-EIO framework as the PL-EVIO (Event+Image+IMU), in which visual landmarks include event-corner features, linebased event features, and point-based image features. These three kinds of features are well integrated together to leverage additional structure or constraint information for more accurate and robust state estimation.

## B. Contributions

Our contributions are summarized as follows:

1) In order to handle the HDR situations and aggressive motion, especially the onboard aggressive motion, we propose the PL-EVIO pipeline, which tightly fuses the event-corner features, line-based event features, and point-based image features together, to provide robust and reliable state estimation.

2) To address the performance degradation when only using point-based features in human-made structures, we design the line-based feature and descriptor in event-based representation for front-end incremental estimation.

3) We validate that our PL-EVIO can achieve state-of-theart performance in different challenging datasets. It also can be used as onboard pose feedback control for the quadrotor to achieve aggressive motion, e.g. flip.

The remainder of the paper is organized as follows: Section II introduces the related works. Section III introduces the principle of our proposed method. Section IV presents the experiments and results. Finally, the conclusion is given in Section V.

## II. RELATED WORKS

## A. Event-Based Representation and Feature Extraction

Event cameras are motion-activated sensors that capture pixel-level illumination changes instead of the intensity image with a fixed frame rate. An event is triggered only when the intensity of an individual pixel varies beyond a specific threshold $T _ { t h r e s h o l d } .$ which can be represented as the spatio-temporal coordinates of the intensity change and its sign:

$$
e = \{ t , x , y , p \} \Leftrightarrow I ( x , y , t + \triangle t ) - I ( x , y , t ) = p \cdot T _ { t h r e s h o l d }\tag{1}
$$

where t is the timestamp that the intensity of a pixel $I ( x , y )$ changes, and p is the polarity that indicates the direction of the intensity change. The generation model of the event stream endowed some good properties, which also allow the event camera to confer robustness to vision-based localization in challenging scenarios. However, adopting the event camera into the SLAM/VO/VIO is a very challenging task since the event streams are in asynchronous formats which is fundamentally different from the synchronous image data. Therefore, most methods and concepts developed for conventional image-based cameras can not be directly applied. To enable the asynchronous event data into the synchronous data representation, different kinds of event representation have been proposed:

(i) The first method is directly working on the raw event stream without any frame-like accumulation. Reference [6] proposed a feature tracker that employe the descriptors for event data. Reference [7] presented a feature tracker based on Expectation Maximisation (EM). References [8] and [9] extracted the line feature from the raw asynchronous events. There are several other ways to represent the raw event, such as Voxel Grid or Event spike tensor [10]. However, these higher dimensional or learning-based event representations will not be discussed here.

(ii) The second approach is combining with the image sensor, or generating the intensity image from the event through learning-based methods. References [11] and [12] firstly detected the features on the grayscale image frames, and then track the features asynchronously using event streams.

(iii) The third representation is the motion-compensated event image, or edge image, which is generated by aggregating a group of neighbor events within the spatio-temporal window into an edge image. References [13] and [14] adopted the conventional corner detection algorithms, such as FAST corners [15] or Shi-Tomasi [16] for feature detection, and the Lucas Kanade (LK) optical flow [17] for feature tracking in the event image.

(iv) The last method is the time surface (TS) or Surface of Active Event (SAE), which is a 2D map where each pixel stores the time value. It can summarize and update the event stream at any given instant, or encode the spatio-temporal constraints of the historical events. Using an exponential decay kernel, TS can emphasize recent events over past events [18]. $t _ { l a s t }$ is the timestamp of the last event at each pixel coordinate ${ \pmb x } = ( u , v ) ^ { T }$ , the TS at time $t \geq t _ { l a s t } ( \pmb { x } )$ is defined by:

$$
T ( \pmb { x } , t ) = \exp ( - \frac { t - t _ { l a s t } ( \pmb { x } ) } { \eta } )\tag{2}
$$

where $\eta$ is the decay rate parameter. References [19] and [20] use the SAE or TS to inspect previously triggered events in the stream and the adjacent pixels for classifying a new event as an event-corner.

## B. Event-Based Motion Estimation

Event-based state estimation has been extensively developed to handle challenging scenes in recent years, particularly in scenarios where traditional cameras struggle to perform well, such as high-speed motion estimation or HDR perception. Reference [21] proposed the first event-based SLAM system, which is limited to tracking planar motions while reconstructing the 2D ceiling map with an upward-looking event camera. References [11] and [22] proposed the event-based VO to track camera motion. However, these methods still relied on the standard camera, which was still susceptible to motion blur and low dynamic range. The first purely event-based 6-DoF (Degree-of-Freedom) VO was presented in [23], which performed real-time event-based SLAM through three decoupled probabilistic filters that jointly estimate the 6-DoF camera pose, 3D map of the scene, and image intensity. However, it is computationally expensive and requires GPU to achieve realtime performance. EVO [24] was proposed to solve the SLAM problem without recovering image intensity, thus reducing computational complexity, and it can run in real-time on a standard CPU. It performs a tracking approach based on image-to-model alignment and adopts the 3D reconstruction method from EMVS [25] to perform the mapping. However, the EVO is needed to run in the scene that is planar to the sensor, for up to several seconds, for bootstrapping the system. ESVO [26] is the first stereo event-based VO method, which follows a parallel tracking-and-mapping scheme to estimate the ego-motion and the semi-dense 3D map of the scene. However, it barely operates in real-time in DAVIS346 (346\*260) and also faces limitations due to rigorous initialization as well as unreliable pose tracking. Reference [27] proposed stereo VO for event cameras based on features. The pose estimation is done by re-projection error minimization, while the features are stereo and temporally matched through the consecutive left and right event TS. It solves the problems of ESVO mentioned above. However, it still cannot operate in real-time in highresolution event cameras (640\*480).

The robustness of event-based SLAM/VO systems can be improved by incorporating IMU measurements. The first EIO method was proposed in [28] which fused a purely event-based tracking algorithm with pre-integration IMU measurement through the Extended Kalman Filter. Another EIO method was proposed in [13]. It detects and tracks the features in the edge image, which is generated from motion-compensated event streams, through traditional image-based feature detection and tracking methods. Finally, the tracked features are combined with IMU measurement using keyframe-based nonlinear optimization. The authors extended their method to leverage the complementary advantages of both standard and event cameras in Ultimate-SLAM [14] to fuse events image frames, standard frames, and IMU. To some extent, these methods use the edge image to realize VIO, this might introduce bottlenecks since it requires substantial parameter adjustments depending on the varying number of generated events in the scene. EKLT-VIO [29] combined the event-based tracker [12] as the front-end with a filter-based back end to perform the EVIO for Mars-like sequences. However, it is pretty hard to perform in real-time even in the lowest resolution event camera. Reference [30] proposed to fuse events and IMU measurement into a continuous-time framework. While their approach cannot achieve real-time since the expensive optimization is required to update the spline parameters upon receiving every event [13]. In our previous work [5], we proposed a monocular EIO which the event-corner features with IMU measurement to provide real-time 6-DoF state estimation even in high-resolution event cameras. Furthermore, this EIO framework can bootstrap from unknown initial states and can ensure global consistency thanks to the loop closure function. Nonetheless, it still cannot provide onboard pose estimation for closed-loop control of the quadrotor flight since event cameras produce minimal information or noise when stationary. Recently, there have been several studies focusing on stereo EVIO [31], [32].

There are several works in event-based vision that utilize line features. IDOL [9] calculates the normal vectors in the spatio-temporal space for each incoming event by utilizing a local neighborhood. Events with similar normal vectors are clustered together to form lines, and an EIO algorithm uses these detected lines and inertial measurements to estimate camera poses. However, this approach assumes that lines move at nearly a constant speed in short intervals, leading to the aggressive motion being avoided in their validation experiments and loss of the advantages of event cameras. What’s more, this method lacks real-time capabilities even with low-resolution event cameras (240\*180). Reference [33] employed the Hough transformation on spatial images generated from a 3D point-based map to cluster event data into a collection of 3D lines. These lines are subsequently integrated into the Kalman filter to estimate the 3D lines and camera pose. However, their event-to-line matching method suffers from the sudden surge of incoming events [34] caused by aggressive motion, scene complexity, and sudden illumination changes. Additionally, this approach is sensitive to event sparsity and requires at least 6 non-parallel 3D lines, a knownscale predefined marker, or ground-truth pose readings for system bootstrapping. Both spatio-temporal relationship [9] and Hough transformation [33] are utilized in event data to cluster events that belong to the same straight lines. In contrast, our approach utilizes the line segment detector (LSD) [35] to extract event-based line features and strike a balance between performance and computational efficiency. Unlike these two methods that solely rely on the line feature and may be susceptible to high levels of texture in the scene, our method leverages the complementarity of point and line features to enhance its robustness. Moreover, [8] utilized event cameras for powerline tracking. Their method involved detecting planes in the spatio-temporal signal to identify lines in the event streams and subsequently incorporating events into these lines while tracking them over time. However, their approach is restricted to powerline inspection tasks and does not involve the data association of event-based line features or utilization for incremental pose estimation.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/82ee833e0d1f46cac6ea55a410a9a7ba5e6d2abc48c88075bfa63be36a32b963.jpg)  
Fig. 2. The framework of our PL-EIO (Event+IMU) and PL-EVIO (Event+Image+IMU).

## III. METHODOLOGY

## A. Framework Overview

The structure of our proposed method is illustrated in Fig.2, which is composed of two sections: (i) The EIO Front-end takes the motion-compensated event stream as input and extracts the event-corner features and the line-based event features. There are two kinds of event representations: the TS with polarity $T _ { p } ( \pmb { x } , t ) = p \cdot \mathrm { e x p } ( - ( t - t _ { l a s t } ( \pmb { x } ) ) / \eta )$ and the normalized TS without polarity $T _ { n p } ( { \bf r } , t ) = 2 5 5 . 0$ $( T ^ { \prime } - \mathrm { m i n } ( T ^ { \prime } ) ) / ( \mathrm { m a x } ( T ^ { \prime } ) - \mathrm { m i n } ( T ^ { \prime } ) )$ ), which are generated from the SAE for point & line feature tracking and loop closure detection, respectively. More detailed discussions of these two kinds of event representations can be seen in APPENDIX A and [5] (ii) The EIO Back-end tightly fuses the point landmarks, line landmarks, and the IMU pre-integration to estimate the 6-DoF state, while the loop closure is used to eliminate the accumulated drifts. Finally, to achieve low latency, we also directly forward propagate (loosely-coupled) the latest estimation with the IMU measurements to achieve IMU-rate state outputs which can be up to 1000 Hz. This can ensure the requirement of closed-loop autonomous quadrotor flight.

For the keyframe in the sliding window, it is selected by two criteria and only based on the event-corner features: (i) When the average parallax of the tracked event-corner features between two consecutive timestamps exceeds a threshold (10 is set in our experiment). (ii) When the number of successfully tracked event-corner features from the last timestamp falls below a certain threshold (20 is set in our experiment).

As for the initialization procedure of our framework, which is adopted from [36] and [37], our pipeline commences with a vision-only structure from motion (SfM) to establish the up-to-scale structure of camera pose and event-corner feature positions. By loosely aligning the SfM with the pre-integrated IMU measurements, it can bootstrap the system from unknown initial states rather than using marker [33] or assuming the local scene is planar to the sensor [24]. It is worth mentioning that if the image is available in the framework, we only employ the point-based image visual measurement for SfM initialization to ensure reliable visual-inertial alignment and up-to-scale camera poses.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/ea989ca06196bfb84bbded20d58d1ba456998c8f99f5fe8fffeba4bda99dda31.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/46fe83e3cb1bf0ed8ea6cffb85ec9516beee8d126b7daa131a003c242e0a235b.jpg)  
(a)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/4b593c581d6803c6724c67c5246980ee71df8fe8ace821a68d6871d1eb90fbc6.jpg)  
(b)  
Fig. 3. Three different kinds of features in our PL-EVIO framework: event-corner features, event-based line features, and image-based point features.

Regarding loop closure, extra event corners are detected in the EIO Front-end, subsequently described by the BRIEF descriptor, and fed to the Back-end. These additional event-corner features are used to achieve a better recall rate on loop detection. Thanks to our designed normalized TS without polarity, which is triggered in scenes with strong edges, it can eliminate accumulated drifts and ensure global consistency. The correspondences are found through the BRIEF descriptor matching by calculating Hamming distance. When the number of corresponding event descriptors is greater than a certain threshold (16)-25 in our experiments), the loop closure is detected. After detecting the loop, the connection residual of the previous keyframe and the current keyframe are integrated into the nonlinear optimization as a re-localization residual.

We further extend our PL-EIO framework to include point-based image features to provide a more robust state estimation (PL-EVIO). Fig.3(a) shows the complementarity of the image and event information. For the bad lighting area, the event can provide reliable event-corner features, while the image can provide rich point-based features in other areas. This enables the uniform distribution of the point-based event and image features in the scene. While the line-based event feature can provide more constraints (shown in Fig.3(b)) even when the successfully tracked point-based event and image features are less in the scene. Our framework can provide a more robust and accurate state estimation. More details of event-based point and line feature detection and tracking in our framework can be seen in the APPENDIX A.

## B. Motion Compensation for the Event Stream Using IMU Measuremnet

Events can be triggered either by moving objects or by the ego-motion of the camera. Similar to Ref. [38], we only rely on the IMU for motion compensation, which guarantees efficiency and speed. For the new event stream coming, we use the angular velocity and linear acceleration from the IMU averaged over the time window where the events are grouped in the same event stream, to estimate the ego-rotation and ego-translation of each event. Using this ego-motion to warp the events into the timestamp of the first event in the same event stream. The motion (considering both rotation R and translation T) of each event can be calculated through:

$$
\pmb { \Delta } ( \delta t ) = \left[ \begin{array} { c c } { \pmb { R } ( \omega _ { I M U } \delta t ) } & { \pmb { T } } \\ { 0 } & { 1 } \end{array} \right] = \left[ \pmb { R } ( \omega _ { I M U } \delta t ) \quad \begin{array} { c c } { \frac 1 2 \pmb { \alpha } _ { I M U } \delta t ^ { 2 } } \\ { 0 } & { 1 } \end{array} \right]\tag{3}
$$

where $\omega _ { I M U }$ and $\pmb { \alpha } _ { I M U }$ are the angular velocity and linear acceleration measurements from the IMU in the current event stream timestamp. While $\pmb { R } ( \omega _ { I M U } \delta t )$ is the rotation matrix generated from the angular velocity $\omega _ { I M U }$ and the time difference $\delta t .$ . Each event $\boldsymbol { e } _ { i } = \{ e t _ { i } , e x _ { i } , e y _ { i } , e p _ { i } \}$ of the event stream is then warped by $\Delta ( \delta t ) = \Delta ( e t _ { i } - t _ { f i r s t \_ e v e n t } )$ , where $t _ { f i r s t \_ e v e n t }$ is the timestamp of the first event of the current event stream and $e t _ { i }$ is the timestamp of event $e _ { i }$

## C. Event-Corner Feature Detection and Tracking

The SAE would be updated through the motioncompensated event stream, while the existing event-corner features are tracked by the LK optical flow on the TS with polarity $T _ { p } ( { \pmb x } , t )$ which is generated from the updated SAE (shown in Fig.4(c)). Different from our previous EIO [5], in this work, we use a two-way tracking strategy to track event-corner features between two consecutive timestamps. For any event-corner feature $F _ { e }$ on last timestamp $T _ { p } ( { \pmb x } , t )$ is tracked to $F _ { e } ^ { \prime }$ on current timestamp $T _ { p } ( { \pmb x } , t )$ , we would reverse the tracking process by tracking $F _ { e } ^ { \prime }$ on current timestamp $T _ { n p } ( { \pmb x } , t )$ back to $F _ { e } ^ { \prime \prime }$ on last timestamp $T _ { p } ( { \pmb x } , t )$ . If the distance between $F _ { e } ^ { \prime \prime }$ and $F _ { e } ^ { \prime }$ is smaller than a threshold (1.0 pixel in our experiment), this event-corner feature would be viewed as being successfully tracked. The event-corner features that are not successfully tracked in the current timestamp would be discarded immediately.

Whenever the number of the tracked features falls below a certain threshold (150-250 in our experiment), new event-corner features would be detected from the latest motion-compensated event stream (shown in Fig.4(a)) for future feature tracking. Modified from the publicly available implementation of the $\operatorname { A r c } ^ { * }$ algorithm [20] for event-based corner detection, we extract the event corners on the individual event by leveraging the SAE rather than adopting the conventional corner detection algorithms in frame-like accumulation (like References [13], [14]). The newly detected event-corner features would be further selected by setting the TS with polarity as the mask (shown in Fig.4(b)). To enforce the uniform distribution, a minimum distance (10-20 pixels for different resolution event cameras) is set between two neighboring event-corner features. Meanwhile, we maintain the event corners, where the pixel value of the TS with polarity is not equal to 128.0, to emphasize the detected event-corner features located in the strong edges rather than the numerous noisy features in low texture areas.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/ec988f5139bc495b5488932acbbaf27dca9cb2070d2abd8f8f9f79e51a319312.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/e06a118e1a9a068cf2f40053fa8b04f34d59fdf349a9b2eb170c5747fbeb5444.jpg)

(a)  
(b)  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/6b756d62c6375d208aac8dea744750bfc5b51f1ddc39617b53bf9d711ee15328.jpg)  
(c)  
Fig. 4. The event-corner feature detection and tracking: (a) Detecting features from raw event streams; (b) Using the TS with polarity as the mask for uniform distribution of the event-corner features; (c) Tracking feature in the TS with polarity.

Furthermore, all the event-corner features in the front-end are undistorted based on the camera distortion model and projected to a normalized camera coordinate system. To remove outliers, we also use the Random Sample Consensus (RANSAC) for outliers filtering. Finally, we recover the inverse depth of the event-corner features that are successfully tracked between two consecutive timestamps through triangulation. The point-based landmark whose 3D position has been successfully calculated would be fed to the sliding window for nonlinear optimization.

## D. Line-Based Feature Detection and Matching on Event Stream

Utilizing the line-based features to improve the performance of point-based VIO is effective as line features can provide additional constraints and structure information in the scene, especially for the human-made environment. Therefore, for incoming new event streams, after using motion compensation, the streams are mapped into the Opencv-Mat format (event mat). Given that events are typically triggered in scenes with strong edges, generating line features using the event mat can prevent the generation of invalid line features and enhance efficiency. To efficiently extract line-based features and descriptors from the raw event streams, we have modified the LSD algorithm [35] in Opencv. Utilizing the Sobel filter, we compute the orientation of each event in the event mat and group events with similar angles into a line support region. Additionally, we have studied the hidden parameter tuning and length rejection strategy of the LSD algorithm, drawing inspiration from [4] to filter out short line features using a length rejection strategy:

$$
L _ { \mathrm { m i n } } = \eta \cdot \mathrm { m i n } ( W , H )\tag{4}
$$

where min(W, H) denotes the smaller value between the width and the height of the event camera. η is the ratio factor (0.125 is set in our experiments). After that, we adopt the Line Band Descriptor (LBD) [39] to describe and match line features, respectively. In particular, to ensure good tracking performance and be consistent with point-based event-corner features, we also use the TS with polarity for LBD generation and line-based feature matching. We further execute the line features refinement schemes to identify line features as good matches for successful line tracking:

• The Hamming distance between matching line features is less than 30;

• The square error of the endpoint between the matching line features is less than $2 0 \ast 2 0 p i x e l ^ { 2 }$ ;

• The angular between matching line features is less than 0.1 rad;

The successfully tracked line-based event features would be further refined by undistorting the endpoints of the lines and projected onto a unit sphere after passing outlier rejection. The outlier rejection is performed using RANSAC with a fundamental matrix model. Then, we obtain the line-based landmark by triangulating the correspondences of two line features. The line-based landmark whose 3D position has been successfully calculated would be fed to the sliding windows for the nonlinear optimization.

E. Sliding Windows Graph-Based Optimization Based on the Point-Line Features

1) Formulation of the Nonlinear Optimization: The full state vector in the sliding windows is defined as:

$$
\pmb { \chi } = [ \pmb { \chi } _ { b } , \pmb { \lambda } _ { e } , \pmb { \phi } _ { l } , \pmb { \lambda } _ { c } , \pmb { T } _ { c } ^ { b } , \pmb { T } _ { e } ^ { b } ]\tag{5}
$$

where $\begin{array} { r c l } { { \lambda _ { e } } } & { { = } } & { { [ \lambda _ { 0 } , \hdots , \lambda _ { m _ { e v e n t } ^ { t h } } ] , } } \end{array}$ , and $\begin{array} { r c l } { \lambda _ { c } } & { = } & { [ \lambda _ { 0 } , \hdots , \lambda _ { m _ { i m a g e } ^ { t h } } ] } \end{array}$ is the inverse depth of the $m _ { e v e n t } ^ { t h }$ event-corner features and $m _ { i m a g e } ^ { t h }$ point-based image features, respectively, while $\pmb { \phi } _ { l } = [ \phi _ { 0 } ^ { \sim } , \ldots , \phi _ { m _ { l i n e } ^ { t h } } ] , \phi _ { m _ { l i n e } ^ { t h } } = [ \pmb { \theta } ^ { T } , o ]$ is the four-parameter orthonormal representation ( as shown in Eq.(12) and $\operatorname { E q . } ( 1 3 ) )$ of the $m _ { l i n } ^ { t h }$ line-based event features, in the sliding windows. $\pmb { T } _ { c } ^ { b } = [ \pmb { \tilde { R } } _ { c } ^ { b } , \pmb { t } _ { c } ^ { b } ] \ \mathrm { o r } \ T _ { e } ^ { b } = [ \pmb { R } _ { e } ^ { b } , \pmb { t } _ { e } ^ { b } ]$ is the extrinsic transformation from camera frame (the image c or event e) to the body (IMU) frame b $( \pmb { T } _ { c } ^ { b } = \pmb { T } _ { e } ^ { b }$ when using the DAVIS which can simultaneously output the image and event data); ${ \pmb \chi } _ { b } = [ X _ { 1 } , \dots , X _ { K } ]$ is the optimization vector in the sliding windows, which comprises the state of the IMU, with $K \ ( K = 1 0$ in our experiments), the total number of keyframes in the sliding windows. The system state $X _ { k }$ at $k ^ { t h }$ keyframe is given by the position $\pmb { p } _ { b _ { k } } ^ { w }$ , orientation quaternion $\pmb { q } _ { b _ { k } } ^ { w }$ , and the velocity $\pmb { v } _ { b _ { k } } ^ { w }$ of the IMU in the world frame, and the accelerometer bias $\pmb { b } _ { a _ { k } }$ and gyroscope bias $\pmb { b } _ { g _ { k } }$ as follows:

$$
X _ { k } = [ { \pmb { p } } _ { b _ { k } } ^ { w } , { \pmb q } _ { b _ { k } } ^ { w } , { \pmb v } _ { b _ { k } } ^ { w } , { \pmb b } _ { a _ { k } } , { \pmb b } _ { g _ { k } } ]\tag{6}
$$

Joint nonlinear optimization is solved for the maximum a posteriori estimation of $\pmb { \chi } .$ , while the cost function can be written as:

$$
\begin{array} { c } { { \displaystyle { J ( \chi ) = \sum _ { k = 0 } ^ { K - 1 } \sum _ { l \in \zeta } | | e _ { e v e n t } ^ { k , l } | | _ { W _ { e v e n t } ^ { k } } ^ { 2 } + \sum _ { k = 0 } ^ { K - 1 } \sum _ { l \in \ell } | | e _ { l i n e } ^ { k , l } | | _ { W _ { l i n e } ^ { k } } ^ { 2 } } } } \\ { { + \displaystyle { \sum _ { k = 0 } ^ { K - 1 } | | e _ { i m u } ^ { k } | | _ { W _ { i m u } ^ { k } } ^ { 2 } + \sum _ { k = 0 } ^ { K - 1 } \sum _ { l \in \zeta } | | e _ { i m a g e } ^ { k , l } | | _ { W _ { i m a g e } ^ { k } } ^ { 2 } } } } \\ { { + | | e _ { m } | | _ { W _ { m } } ^ { 2 } + | | e _ { r } | | _ { W _ { r } } ^ { 2 } } } \end{array}\tag{7}
$$

Eq.(7) contains the point-based event residual $\pmb { e } _ { e v e n t } ^ { k , l }$ with weight $W _ { e v e n t } ^ { k } ;$ the line-based event residual $\pmb { e } _ { l i n e } ^ { k , l }$ with weight $W _ { l i n e } ^ { k } ;$ the IMU pre-integration residuals $\pmb { e } _ { i m u } ^ { k }$ with weight $W _ { i m u } ^ { k } ;$ the point-based image residual $\boldsymbol { e } _ { i m a g e } ^ { k , l }$ with weight $W _ { i m a g e } ^ { k } ;$ the marginalization residuals $\boldsymbol { e } _ { m }$ with weight $W _ { m } ;$ the re-localization residuals $\scriptstyle { e _ { r } }$ with weight $W _ { r } ;$ while $\zeta .$ ℓ, and $\xi$ are the set of event-corner features, line-based event features, and point-based event features, respectively, which have been successfully tracked or matched at least twice in the current sliding window.

2) Point-Based Event Visual Measurement Residual: The $\mathbf { \Delta } _ { e v e n t } ^ { k , l }$ in Eq.(7) is the event-corner measurement residual from the re-projection function. Considering the $l ^ { t h }$ event-corner feature that is first observed in the $i ^ { t h }$ keyframe, the residual for its observation in the $k ^ { t h }$ keyframe is defined as:

$$
e _ { e v e n t } ^ { k , l } = \left[ \begin{array} { l } { u _ { k } ^ { l } } \\ { v _ { k } ^ { l } } \end{array} \right] - \pi _ { e } \cdot ( T _ { e } ^ { b } ) ^ { - 1 } \cdot T _ { w } ^ { b _ { k } } \cdot T _ { b _ { i } } ^ { w } \cdot T _ { e } ^ { b } \cdot \pi _ { e } ^ { - 1 } ( \frac { 1 } { \lambda _ { e } } , \left[ \begin{array} { l } { u _ { i } ^ { l } } \\ { v _ { i } ^ { l } } \end{array} \right] )\tag{8}
$$

where, $\left[ u _ { i } ^ { l } , v _ { i } ^ { l } \right] ^ { \mathrm { T } }$ is the first observation of the $l ^ { t h }$ eventcorner feature in the $i ^ { t h }$ keyframe. $\left[ u _ { k } ^ { l } , v _ { k } ^ { l } \right] ^ { \mathrm { T } }$ is the observation of the same event-corner feature in the $k ^ { t h }$ keyframe, $\pi _ { e }$ and $\pi _ { e } ^ { - 1 }$ are the projection and back-projection function of the event camera, respectively, which include the intrinsic parameters for the transform between the 2D pixel coordinates and normalized event camera coordinate. $\pmb { T } _ { b _ { i } } ^ { w }$ indicates the movement of the body frame related to the world frame in timestamp $i , \pmb { T } _ { w } ^ { b _ { k } }$ is the transpose of the pose of the body in the world frame in the $k ^ { t h }$ keyframe.

3) Line-Based Event Visual Measurement Residual: The $\pmb { e } _ { l i n e } ^ { k , l }$ in $\operatorname { E q . } ( 7 )$ is the line-based event measurement residual which is generated from line re-projection model. The line re-projection residual is modeled as the distance from the endpoints of the line to the projected line in the normalized image plane. The $l ^ { t h }$ line-based landmarks in the world frame can be defined using the Plücker Coordinate: $\pmb { L } _ { w } ^ { l } = \left[ \pmb { n } _ { w } ^ { l } , \pmb { d } _ { w } ^ { l } \right] ^ { \mathrm { T } }$ $\pmb { n } _ { w } ^ { l }$ denotes the normal vector of the plane determined by $\pmb { L } _ { w } ^ { l }$ and the origin of the world frame, while $\pmb { d } _ { w } ^ { l }$ denotes the direction vector determined by the two endpoints of $\pmb { L } _ { w } ^ { l } .$ Given the transformation matrix ${ \bf \dot { \cal T } } _ { w } ^ { b _ { k } } = \left[ { \cal R } _ { w } ^ { b _ { k } } , \bar { \pmb { t } } _ { w } ^ { b _ { k } } \right]$ indicates the movement of the body frame related to the world frame in timestamp k, we can obtain the transformation from the world frame to the event frame in timestamp k through $T _ { w } ^ { e _ { k } } =$ $T _ { b } ^ { e } \cdot T _ { w } ^ { b _ { k } }$ , where $\pmb { T } _ { b } ^ { e } = [ \pmb { R } _ { b } ^ { e } , \pmb { t } _ { b } ^ { e } ]$ is the extrinsic transformation from the body (IMU) frame b to the event camera frame e. Then, we can transform the $l ^ { t h }$ line-based event feature $\pmb { L } _ { w } ^ { l }$ in $k ^ { t h }$ keyframe from world frame to event camera frame by [40], [41]:

$$
\begin{array} { r } { L _ { e _ { k } } ^ { l } = \left[ \begin{array} { c } { { \pmb { n } } _ { e _ { k } } ^ { l } } \\ { { \pmb { d } } _ { e _ { k } } ^ { l } } \end{array} \right] = \left[ \begin{array} { c c } { { \pmb { R } } _ { w } ^ { e _ { k } } } & { [ { \pmb { t } } _ { w } ^ { e _ { k } } ] \times { \pmb { R } } _ { w } ^ { e _ { k } } } \\ { 0 } & { { \pmb { R } } _ { w } ^ { e _ { k } } } \end{array} \right] \left[ \begin{array} { c } { { \pmb { n } } _ { w } ^ { l } } \\ { { \pmb { d } } _ { w } ^ { l } } \end{array} \right] } \end{array}\tag{9}
$$

where $\pmb { R } _ { w } ^ { e _ { k } } = \pmb { R } _ { b } ^ { e } \cdot \pmb { R } _ { w } ^ { b _ { k } } , \pmb { t } _ { w } ^ { e _ { k } } = \pmb { R } _ { b } ^ { e } \cdot \pmb { t } _ { w } ^ { b _ { k } } + \pmb { t } _ { b } ^ { e } .$

The transformation for the Plücker Coordinates of the $l ^ { t h }$ line-based event feature from $i ^ { t h }$ keyframe to $k ^ { t h }$ keyframe in the body frame can be represented as follows:

$$
\begin{array} { r } { \mathbf { { \cal L } } _ { b _ { k } } ^ { l } = \biggl [ \begin{array} { c } { \pmb { n } _ { b _ { k } } ^ { l } } \\ { \pmb { d } _ { b _ { k } } ^ { l } } \end{array} \biggr ] = \biggl [ \begin{array} { c c } { \pmb { R } _ { b _ { i } } ^ { b _ { k } } } & { \quad [ \pmb { t } _ { b _ { i } } ^ { b _ { k } } ] \times \pmb { R } _ { b _ { i } } ^ { b _ { k } } } \\ { 0 } & { \quad \pmb { R } _ { b _ { i } } ^ { b _ { k } } } \end{array} \biggr ] \biggl [ \begin{array} { c } { \pmb { n } _ { b _ { i } } ^ { l } } \\ { \pmb { d } _ { b _ { i } } ^ { l } } \end{array} \biggr ] } \end{array}\tag{10}
$$

where $T _ { b _ { i } } ^ { b _ { k } } = \left\lceil R _ { b _ { i } } ^ { b _ { k } } , t _ { b _ { i } } ^ { b _ { k } } \right\rceil$ indicates the movement of the body frame related to the world frame in $i ^ { t h }$ keyframe to $k ^ { t h }$ keyframe.

The Plöcker Coordinates $\pmb { L } _ { w } ^ { l }$ can be represented using a four-parameter orthonormal representation, known for its superior convergence performance [40]. As a result, we transfer the line-based landmark to the four-parameter orthonormal representation for the optimization process. The orthonormal representation $( U , W ) ~ \in ~ ( S O ( 3 ) , S O ( 2 ) )$ of the Plücker Coordinates $\pmb { L } _ { w } ^ { l }$ can be computed using the QR decomposition [4], [40]:

$$
[ \pmb { n } _ { w } ^ { l } | \pmb { d } _ { w } ^ { l } ] = \pmb { U } \left[ \begin{array} { c c } { w _ { 1 } } & { 0 } \\ { 0 } & { w _ { 2 } } \\ { 0 } & { 0 } \end{array} \right] , s e t : \pmb { W } = \left[ \begin{array} { c c } { w _ { 1 } } & { - w _ { 2 } } \\ { w _ { 2 } } & { w _ { 1 } } \end{array} \right]\tag{11}
$$

where U and $W$ denote a three and a two dimensional rotation matrix, respectively. Let $\pmb { R } ( \pmb \theta ) = \pmb U$ and $\pmb { R } ( o ) = \pmb { W }$ be the corresponding rotation transformations, where $U = [ u _ { 1 } , u _ { 2 } , u _ { 3 } ]$ With this notation, we can now express the relationship as follows:

$$
\pmb { R } ( \pmb { \theta } ) = \pmb { U } = \left[ \frac { \pmb { n } _ { w } ^ { l } } { | | \pmb { n } _ { w } ^ { l } | | } , \frac { \pmb { d } _ { w } ^ { l } } { | | \pmb { d } _ { w } ^ { l } | | } , \frac { \pmb { n } _ { w } ^ { l } \times \pmb { d } _ { w } ^ { l } } { | | \pmb { n } _ { w } ^ { l } \times \pmb { d } _ { w } ^ { l } | | } \right]\tag{12}
$$

$$
\begin{array} { r l r } { \pmb { R } ( o ) = \pmb { W } = \bigg [ \pmb { c o s } ( o ) } & { - s i n ( o ) \bigg ] } & \\ { = \frac { 1 } { \sqrt { | | \pmb { n } _ { w } ^ { l } | | ^ { 2 } + | | \pmb { d } _ { w } ^ { l } | | ^ { 2 } } } \bigg [ | | \pmb { n } _ { w } ^ { l } | | } & { - | | \pmb { d } _ { w } ^ { l } | | \bigg ] } & \end{array}\tag{13}
$$

Up to this point, we have established the connection between the four-parameter orthonormal representation $\phi _ { m _ { l i n e } ^ { t h } } = [ \pmb { \theta } ^ { \mathrm { T } } , o ]$ of $\operatorname { E q . } ( 5 )$ and the Plöcker Coordinates $\pmb { L } _ { w } ^ { l }$

The Plücker Coordinates $\pmb { L } _ { e _ { k } } ^ { l }$ in the event camera frame can be obtained from $\pmb { L } _ { w } ^ { l }$ through Eq.(9), and then can be projected to the line $l _ { e _ { k } } ^ { l }$ in the event imaging plane by

$$
\boldsymbol { l } _ { e _ { k } } ^ { l } = \left[ l _ { 1 } , l _ { 2 } , l _ { 3 } \right] ^ { \mathrm { T } } = \pi _ { e } \boldsymbol { n } _ { e _ { k } } ^ { l }\tag{14}
$$

where $\pi _ { e }$ is the projection function of the event camera, and the ${ \pmb n } _ { e _ { k } } ^ { l }$ can be obtained from $\operatorname { E q . } ( 9 )$ . The line re-projection error in Eq.(7) can be defined as:

$$
e _ { l i n e } ^ { k , l } = \left[ \begin{array} { l } { d ( S _ { { l } _ { e _ { k } } ^ { l } } , { l } _ { e _ { k } } ^ { l } ) } \\ { d ( E _ { { l } _ { e _ { k } } ^ { l } } , { l } _ { e _ { k } } ^ { l } ) } \end{array} \right]\tag{15}
$$

where $S _ { l _ { e _ { k } } ^ { l } }$ and $E _ { l _ { e _ { k } } ^ { l } }$ are the homogeneous coordinates of the endpoints of the line feature $l _ { e _ { k } } ^ { l }$ in the image plane, and $d ( m , l _ { e _ { k } } ^ { l } )$ denotes the point-to-line distance function from the endpoints to the projection line $l _ { e _ { k } } ^ { l }$

$$
\begin{array} { l } { { d ( m , l _ { e _ { k } } ^ { l } ) = \displaystyle \frac { m l _ { e _ { k } } ^ { l } } { \sqrt { l _ { 1 } ^ { 2 } + l _ { 2 } ^ { 2 } } } } } \\ { { S _ { l _ { e _ { k } } ^ { l } } = ( u _ { k , S } ^ { l } , v _ { k , S } ^ { l } , 1 ) , E _ { l _ { e _ { k } } ^ { l } } = ( u _ { k , E } ^ { l } , v _ { k , E } ^ { l } , 1 ) } } \end{array}\tag{16}
$$

4) Point-Based Image Visual Measurement Residual: The $\boldsymbol { e } _ { i m a g e } ^ { k , l }$ in Eq.(7) is the point-based image measurement residual from the re-projection function. Similar to the event-corner measurement, the $l ^ { t h }$ point-based image feature that is first observed in the $i ^ { t h }$ keyframe, the residual for its observation in the $k ^ { t h }$ keyframe is defined as:

$$
e _ { i m a g e } ^ { k , l } = \left[ \begin{array} { l } { u _ { k } ^ { l } } \\ { v _ { k } ^ { l } } \end{array} \right] - \pi _ { c } \cdot ( { \pmb T } _ { c } ^ { b } ) ^ { - 1 } \cdot { \pmb T } _ { w } ^ { b _ { k } } \cdot { \pmb T } _ { b _ { i } } ^ { w } \cdot { \pmb T } _ { c } ^ { b } \cdot \pi _ { c } ^ { - 1 } ( \frac { 1 } { \lambda _ { c } } , \left[ u _ { i } ^ { l } \right] )\tag{17}
$$

where, $\left[ u _ { i } ^ { l } , v _ { i } ^ { l } \right] ^ { \mathrm { T } }$ is the first observation of the $l ^ { t h }$ point-based image feature in the $i ^ { t h }$ keyframe. $\left[ u _ { k } ^ { l } , v _ { k } ^ { l } \right] ^ { \mathrm { T } }$ is the observation of the same point-based image feature in the $k ^ { t h }$ keyframe. $\pi _ { c }$ and $\pi _ { c } ^ { - 1 }$ are the projection and back-projection function of the standard camera, respectively, which include the intrinsic parameters for the transform between the 2D pixel coordinates and normalized camera coordinate.

5) IMU Measurement Residual: The $\pmb { e } _ { i m u } ^ { k }$ in $\operatorname { E q . } ( 7 )$ is the IMU residual from the IMU pre-integration. The raw measurement of angular velocity $\omega _ { k }$ and acceleration $\pmb { a } _ { k }$ from IMU at time $t _ { k }$ are:

$$
\begin{array} { l } { { \hat { \pmb { a } } _ { k } = { \pmb { a } } _ { k } - \pmb { R } _ { \omega } ^ { b _ { k } } \pmb { g } ^ { \omega } + { \pmb { b } } _ { a _ { k } } + { \pmb { n } } _ { a } } } \\ { { \hat { \pmb { \omega } } _ { k } = \pmb { \omega } _ { k } + { \pmb { b } } _ { \omega _ { k } } + { \pmb { n } } _ { \omega } } } \end{array}\tag{18}
$$

where ${ \pmb n } _ { a } , { \pmb n } _ { \omega }$ are modeled as additive Gaussian noise. $\mathbf { \delta } _ { b _ { a _ { k } } } , b _ { \omega _ { k } }$ are modeled as random walks. The Notation <sup>ˆ</sup>(·) is used to represent noisy measurements. Given the time interval $[ t _ { k } , t _ { k + 1 } ]$ corresponding to keyframe $b _ { k }$ and $b _ { k + 1 } . \pmb { p } _ { b _ { k + 1 } } ^ { \omega } , \pmb { v } _ { b _ { k + 1 } } ^ { \omega } , \pmb { q } _ { b _ { k + 1 } } ^ { \omega }$ can be propagated in such time interval by using gyroscope and accelerometer measurements in the world frames as follows:

$$
\begin{array} { l } { p _ { b _ { k + 1 } } ^ { \omega } = p _ { b _ { k } } ^ { \omega } + v _ { b _ { k } } ^ { \omega } \Delta t + \displaystyle { \iint _ { t _ { k } } ^ { t _ { k + 1 } } } ( R _ { b _ { k } } ^ { \omega } a _ { k } ) \delta t ^ { 2 } } \\ { v _ { b _ { k + 1 } } ^ { \omega } = v _ { b _ { k } } ^ { \omega } + \displaystyle { \int _ { t _ { k } } ^ { t _ { k + 1 } } } ( R _ { b _ { k } } ^ { \omega } a _ { k } ) \delta t } \\ { q _ { b _ { k + 1 } } ^ { \omega } = \displaystyle { \int _ { t _ { k } } ^ { t _ { k + 1 } } } q _ { b _ { k } } ^ { \omega } \otimes \displaystyle { \left[ \begin{array} { l } { 0 } \\ { \displaystyle { \frac { 1 } { 2 } } \omega _ { k } } \end{array} \right] } \delta t } \end{array}\tag{19}
$$

Based on Eq.(18), Eq.(19) can be rewritten as follows:

$$
\begin{array} { r l } & { \displaystyle p _ { b _ { k + 1 } } ^ { \omega } - p _ { b _ { k } } ^ { \omega } - v _ { b _ { k } } ^ { \omega } \Delta t - \frac { 1 } { 2 } g ^ { \omega } \Delta t ^ { 2 } } \\ & { \displaystyle = \iint _ { t _ { k } } ^ { t _ { k + 1 } } ( R _ { b _ { k } } ^ { \omega } ( \hat { a } _ { k } - b _ { a _ { k } } - n _ { a } ) ) \delta t ^ { 2 } } \\ & { \displaystyle v _ { b _ { k + 1 } } ^ { \omega } - v _ { b _ { k } } ^ { \omega } - g ^ { \omega } \Delta t } \\ & { \displaystyle = \int _ { t _ { k } } ^ { t _ { k + 1 } } ( R _ { b _ { k } } ^ { \omega } \hat { a } _ { k } \delta t - R _ { b _ { k } } ^ { \omega } b _ { a _ { k } } \delta t - R _ { b _ { k } } ^ { \omega } n _ { a } \delta t ) } \end{array}
$$

$$
\begin{array} { l } { \pmb { q } _ { b _ { k + 1 } } ^ { \omega } } \\ { = \displaystyle \int _ { t _ { k } } ^ { t _ { k + 1 } } \pmb { q } _ { b _ { k } } ^ { \omega } \otimes \left[ \begin{array} { c } { 0 } \\ { \displaystyle \frac { 1 } { 2 } ( \hat { \omega } _ { k } - b _ { \omega _ { k } } - \pmb { n } _ { \omega } ) } \end{array} \right] \delta t } \end{array}\tag{20}
$$

In order to ensure the pre-integration term is only related to the inertial measurements and biases in $[ t _ { k } , t _ { k + 1 } ] , R _ { \omega } ^ { b _ { k } }$ is multiplied on both sides of Eq.(20), and we define the pre-integration term $\pmb { \alpha } _ { b _ { k + 1 } } ^ { b _ { k } } , \pmb { \beta } _ { b _ { k + 1 } } ^ { b _ { k } } , \pmb { \gamma } _ { b _ { k + 1 } } ^ { b _ { k } }$ as follows:

$$
\begin{array} { l } { \displaystyle \alpha _ { b _ { k + 1 } } ^ { b _ { k } } = R _ { \omega } ^ { b _ { k } } \iint _ { t _ { k } } ^ { t _ { k + 1 } } ( R _ { b _ { k } } ^ { \omega } ( \hat { a } _ { k } - b _ { a _ { k } } - n _ { a } ) ) \delta t ^ { 2 } } \\ { \displaystyle \beta _ { b _ { k + 1 } } ^ { b _ { k } } = R _ { \omega } ^ { b _ { k } } \int _ { t _ { k } } ^ { t _ { k + 1 } } ( R _ { b _ { k } } ^ { \omega } \hat { a _ { k } } \delta t - R _ { b _ { k } } ^ { \omega } b _ { a _ { k } } \delta t - R _ { b _ { k } } ^ { \omega } n _ { a } \delta t ) } \\ { \displaystyle \gamma _ { b _ { k + 1 } } ^ { b _ { k } } = q _ { b _ { k } } ^ { \omega } \otimes q _ { b _ { k + 1 } } ^ { \omega } } \end{array}\tag{21}
$$

Discretizing Eq.(21) by the zero-order discretization method as follows:

$$
\begin{array} { r l } & { \hat { \alpha } _ { b _ { i + 1 } } ^ { b _ { k } } = \hat { \alpha } _ { b _ { i } } ^ { b _ { k } } + \hat { \beta } _ { b _ { i } } ^ { b _ { k } } \delta t + \displaystyle \frac { 1 } { 2 } R ( \hat { \gamma } _ { b _ { i } } ^ { b _ { k } } ) ( \hat { a } _ { i } - b _ { a _ { i } } ) \delta t ^ { 2 } } \\ & { \hat { \beta } _ { b _ { i + 1 } } ^ { b _ { k } } = \hat { \beta } _ { b _ { i } } ^ { b _ { k } } + R ( \hat { \gamma } _ { b _ { i } } ^ { b _ { k } } ) ( \hat { a } _ { i } - b _ { a _ { i } } ) \delta t } \\ & { \hat { \gamma } _ { b _ { i + 1 } } ^ { b _ { k } } = \hat { \gamma } _ { b _ { i } } ^ { b _ { k } } \otimes \left[ \displaystyle \frac { 1 } { 2 } ( \hat { \omega } _ { i } - b _ { \omega _ { i } } ) \right] \delta t } \end{array}\tag{22}
$$

Eventually, the IMU residual can be derived as follows:

$$
e _ { i m u } ^ { k } = \left[ \begin{array} { c } { R ^ { b _ { k } } ( p _ { b _ { k + 1 } } ^ { \omega } - p _ { b _ { k } } ^ { \omega } - v _ { b _ { k } } ^ { \omega } \Delta t - \frac { 1 } { 2 } \pmb { g } ^ { \omega } \Delta t ^ { 2 } ) - \hat { \pmb { \alpha } } _ { b _ { k + 1 } } ^ { b _ { k } } } \\ { R ^ { b _ { k } } ( \pmb { v } _ { b _ { k + 1 } } ^ { \omega } - \pmb { v } _ { b _ { k } } ^ { \omega } - \pmb { g } ^ { \omega } \Delta t ) - \hat { \pmb { \beta } } _ { b _ { k + 1 } } ^ { b _ { k } } } \\ { 2 \Big [ ( \pmb { q } _ { b _ { k } } ^ { \omega } ) ^ { - 1 } \otimes \pmb { q } _ { b _ { k + 1 } } ^ { \omega } \otimes ( \hat { \pmb { \gamma } } _ { b _ { k + 1 } } ^ { b _ { k } } ) ^ { - 1 } \Big ] _ { x y z } } \\ { b _ { a _ { k + 1 } } - b _ { a _ { k } } } \\ { b _ { \omega _ { k + 1 } } - b _ { \omega _ { k } } } \end{array} \right]\tag{23}
$$

## IV. EVALUATION

In this section, we evaluate the effectiveness of our framework in various challenging sequences using both quantitative and qualitative methods in subsection IV-A and IV-B. We implemented our method with C++ in Ubuntu 20.04 and ROS Noetic. All sequences are evaluated in real-time using a laptop with Intel Core i7-11800H and are recorded in videos (shown on our project website). In subsection IV-C and IV-D, we demonstrate the quadrotor flight using our method for the closed-loop state estimator and aggressive flip. Meanwhile, large-scale experiments are carried out to illustrate the long-time practicability in subsection IV-F.

## A. Evaluation in High-Dynamic-Range Scenarios

For demonstrating the robustness, accuracy, and real-time capability, we initially evaluate our PL-EIO using different resolution event cameras (DAVIS346 (346\*260) and DVXplorer (640\*480)) with the ground truth from VICON. All sequences<sup>1</sup> are recorded in broad illumination range conditions, or under aggressive motion. Without loss of generality, we use the raw image from DAVIS346 to run the VINS-MONO [36], PL-VINS [4], and ORB-SLAM3 [42], as image-based comparisons. In addition, based on the source code of Ultimate SLAM [14], we also test the EVIO and EIO versions of Ultimate SLAM for event-based comparison. The estimated and ground-truth trajectories are aligned with a 6-DOF transformation (in SE3), using 5 seconds [0-5s] of the resulting trajectory. We compute the mean position error (Euclidean distance in meters) as percentages of the total travel distance of the ground truth, which is calculated by the publicly available tool [43]. As can be seen from the results in Table I, our PL-EIO has better performances compared with the other methods in different resolution event cameras. Especially, for the results of vicon\_aggressive\_hdr, our PL-EIO produces reliable and accurate pose estimation even when the image-based VIO and VO fail. Besides, compared with our previous EIO [5], the introduction of the line feature, known as PL-EIO, demonstrates significant performance improvements across different resolution event cameras. While the performance of PL-EVIO, which incorporates image measurements, surpasses our EIO [5]. Our experimental observations indicate that although the image-aid one (PL-EVIO) exhibits notable performance gains in most sequences, it underperforms in low-light environments such as vicon\_dark1 and vicon\_dark2), as compared to PL-EIO. This could be attributed to the degradation of point-based image feature tracking in dark environments.

TABLE I  
ACCURACY COMPARISON OF OUR PL-EIO WITH OTHER IMAGE-BASED OR EVENT-BASED VIO WORKS
<table><tr><td rowspan="2">Sequence</td><td colspan="9">DAVIS346 (346*260)</td><td colspan="3">DVXplorer (640*480)</td></tr><tr><td>VINS-MONO [36] ORB-SLAM3 [42] PL-VINS [4] Ultimate SLAM [14] Ultimate SLAM [14]</td><td>VO</td><td>VIO</td><td>EIO</td><td>EVIO</td><td>Our EIO [5] EIO</td><td>Our PL-EIO Our PL-EIO+ Our PL-EVIO EIO</td><td>EIO</td><td>EVIO</td><td>Ultimate SLAM [14] Our EIO [5] EIO EIO</td><td>EIO</td><td>Our PL-EIO Our PL-EIO+ EIO</td></tr><tr><td>vicon_hdr1</td><td>VIO 0.96</td><td>0.32</td><td>0.67</td><td>1.49</td><td>2.44</td><td>0.59</td><td>0.67</td><td>0.57 0.17</td><td>1.94</td><td>0.30</td><td>0.47</td><td>0.41</td></tr><tr><td>vicon_hdr2</td><td>1.60</td><td>0.75</td><td>0.90</td><td>1.28</td><td>1.11</td><td>0.74</td><td>0.45</td><td>0.54 0.12</td><td>2.38</td><td>0.37</td><td>0.22</td><td>0.21</td></tr><tr><td>vicon_hdr3</td><td>2.28</td><td>0.60</td><td>0.69</td><td>0.66</td><td>0.83</td><td>0.72</td><td>0.74</td><td>0.69 0.19</td><td>0.83</td><td>0.69</td><td>0.47</td><td>0.36</td></tr><tr><td></td><td></td><td></td><td>0.66</td><td></td><td>1.49</td><td>0.37</td><td>0.37</td><td>0.32 0.11</td><td></td><td>0.26</td><td></td><td>0.25</td></tr><tr><td>vicon_hdr4</td><td>1.40</td><td>0.70</td><td></td><td>1.84</td><td></td><td></td><td></td><td></td><td>2.09</td><td></td><td>0.27</td><td></td></tr><tr><td>vicon_darktolight1</td><td>0.51</td><td>0.75</td><td>0.84</td><td>1.33</td><td>1.00</td><td>0.81</td><td>0.78</td><td>0.66 0.14</td><td>1.96</td><td>0.80</td><td>0.71</td><td>0.71</td></tr><tr><td>vicon_darktolight2</td><td>0.98</td><td>0.76</td><td>1.50</td><td>1.48</td><td>0.79</td><td>0.42</td><td>0.44</td><td>0.51 0.12</td><td>1.57</td><td>0.57</td><td>0.56</td><td>0.47</td></tr><tr><td>vicon_lighttodark1</td><td>0.55</td><td>0.41</td><td>0.64</td><td>1.79</td><td>0.84</td><td>0.29</td><td>0.42</td><td>0.33 0.13</td><td>2.48</td><td>0.81</td><td>0.43</td><td>0.54</td></tr><tr><td>vicon_lighttodark2</td><td>0.55</td><td>0.58</td><td>0.93</td><td>1.32</td><td>1.49</td><td>0.79</td><td>0.73</td><td>0.53 0.16</td><td>1.37</td><td>0.75</td><td>0.67</td><td>0.60</td></tr><tr><td>vicon_dark1</td><td>0.88 0.52</td><td>failed 0.60</td><td>0.53 failed</td><td>1.75 1.10</td><td>3.45 0.63</td><td>1.02 0.49</td><td>0.64 0.30 0.38</td><td>0.35 0.43 0.47</td><td>3.79</td><td>0.35</td><td>0.51</td><td>0.41</td></tr><tr><td>vicon_dark2 vicon_aggressive_hdr</td><td>failed</td><td>failed</td><td>1.94</td><td>failed</td><td>2.30</td><td>0.66</td><td>0.62</td><td>0.50 1.97</td><td>2.81 failed</td><td>0.41 0.65</td><td>0.38</td><td>0.41 0.50</td></tr><tr><td>Average</td><td>1.02</td><td>0.61</td><td>0.93</td><td>1.40</td><td>1.49</td><td>0.63</td><td>0.56</td><td>0.49 0.36</td><td>2.12</td><td>0.54</td><td>0.62 0.48</td><td>0.45</td></tr></table>

Unit:%/m, 0.45 means the average error would be 0.45m for 100m motion.

Regarding the proposed motion compensation algorithm, as evident from the results, the motion compensation version (PL-EIO+) does not exhibit significant enhancements across various sequences, particularly in scenarios involving aggressive motion. This outcome could potentially stem from biases present in the IMU during such aggressive motion. On the other hand, in this evaluation, the event stream rate is 60Hz for DAVIS346 and 50Hz for DVXplorer. Such high frequencies reduced time differences within the same event stream, Additionally, we observe that motion compensation for event streams may not be an optimal choice for high-resolution event cameras due to the trade-off between computational burden and performance improvement.

It is worth mentioning that the Ultimate-SLAM is just for reference since we do not deeply fine-tune the parameters for different sequences (being failure-free is difficult). Since the illumination would change greatly in our dataset, and it is very difficult for Ultimate-SLAM to choose a certain stationary threshold to integrate the event stream into the edge image. We have tried our best to fine-tune the parameters of Ultimate-SLAM in sequence vicon\_hdr3 to achieve good performance and use the same parameters to evaluate other sequences. This also shows that the generalization ability to integrate the event streams into the edge-image for VIO is pretty bad since the number of triggered events depends on many factors, including the resolution of the camera, the texture of the sense, the illumination, etc.

TABLE II  
ACCURACY COMPARISON OF OUR PL-EVIO WITH OTHER IMAGE/EVENT-BASED VIO IN UZH-FPV DATASET [44]
<table><tr><td rowspan="2">Sequence</td><td colspan="2">Snapdragon (640*480)</td><td colspan="3">DAVIS346 (346*260)</td></tr><tr><td>VINS-Fusion [45] ORB-SLAM3 [42] Stereo VIO</td><td>Stereo VIO</td><td>VINS-MONO [36] Ultimate SLAM [14] VIO</td><td>EVIO</td><td>Our PL-EVIO EVIO</td></tr><tr><td>Indoor_forward_3</td><td>0.84</td><td>0.55</td><td>0.65</td><td>failed</td><td>0.38</td></tr><tr><td>Indoor_forward_5</td><td>failed</td><td>1.19</td><td>1.07</td><td>failed</td><td>0.90</td></tr><tr><td>Indoor_forward_6</td><td>1.45</td><td>failed</td><td>0.25</td><td>failed</td><td>0.30</td></tr><tr><td>Indoor_forward_7</td><td>0.61</td><td>0.36</td><td>0.37</td><td>failed</td><td>0.55</td></tr><tr><td>Indoor_forward_9</td><td>2.87</td><td>0.77</td><td>0.51</td><td>failed</td><td>0.44</td></tr><tr><td>Indoor_forward_10</td><td>4.48</td><td>1.02</td><td>0.92</td><td>failed</td><td>1.06</td></tr><tr><td>Indoor_45_degree_2</td><td>failed</td><td>2.18</td><td>0.53</td><td>failed</td><td>0.55</td></tr><tr><td>Indoor_45_degree_4</td><td>failed</td><td>1.53</td><td>1.72</td><td>9.79</td><td>1.30</td></tr><tr><td>Indoor_45_degree_9</td><td>failed</td><td>0.49</td><td>1.25</td><td>4.74</td><td>0.76</td></tr><tr><td>Average</td><td>5.26</td><td>2.10</td><td>0.81</td><td>7.26</td><td>0.70</td></tr></table>

Unit:%/m, 0.70 means the average error would be 0.70m for 100m motion

## B. Evaluation in Aggressive Motions

In this section, we evaluate our PL-EVIO in UZH-FPV dataset [44], which is a high-speed, aggressive visual-inertial odometry dataset. This dataset includes fast laps around a racetrack with drone racing gates, as well as free-form trajectories around obstacles. We compare our PL-EVIO with ORB-SLAM3 (stereo VIO) [42], VINS-Fusion (stereo VIO) [45], VINS-MONO (monocular VIO) [36], and Ultimate SLAM (EVIO) [14]. We also computed the mean position error as percentages of the total traveled distance, while the estimated trajectories and ground-truth were aligned in SE3 with all alignments. As can be seen from the results in Table II, our proposed PL-EVIO achieve better performance even compared with the stereo VIO using a higher resolution camera. This dataset is so challenging that most of the sequences using Ultimate-SLAM and VINS-Fusion failed, while our Pl-EVIO still can provide reliable and satisfying results. To achieve optimal performance, deep fine-tuning of parameters is also required for VINS-MONO.

Furthermore, we also evaluate our PL-EVIO with the other EIO works in publicly available Event Camera Datasets [46], which is acquired by the DAVIS240C (240\*180, eventsensor, image-sensor, IMU sensor). It contains extremely fast 6-Dof motion and scenes with HDR. We directly report the raw result in [5], [13], [14], [28], [29], and [47].

TABLE III  
ACCURACY COMPARISON OF OUR PL-EVIO WITH OTHER EIO/EVIO WORKS IN DAVIS240C DATASET [46]
<table><tr><td>Sequence</td><td>Ref. [28] EIO</td><td>Ref. [13] EIO</td><td>Ref. [14] EIO</td><td>Ref. [14] EVIO</td><td>Ref. [47] EIO</td><td>Ref. [29] EVIO</td><td>EIO</td><td>Our EIO [5] Our PL-EVIO EVIO</td></tr><tr><td>boxes_translation</td><td>2.69</td><td>0.57</td><td>0.76</td><td>0.27</td><td>2.55</td><td>0.48</td><td>0.34</td><td>0.06</td></tr><tr><td>hdr_boxes</td><td>1.23</td><td>0.92</td><td>0.67</td><td>0.37</td><td>1.75</td><td>0.46</td><td>0.40</td><td>0.10</td></tr><tr><td>boxes_6dof</td><td>3.61</td><td>0.69</td><td>0.44</td><td>0.30</td><td>2.03</td><td>0.84</td><td>0.61</td><td>0.21</td></tr><tr><td>dynamic_translation</td><td>1.90</td><td>0.47</td><td>0.59</td><td>0.18</td><td>1.32</td><td>0.40</td><td>0.26</td><td>0.24</td></tr><tr><td>dynamic_6dof</td><td>4.07</td><td>0.54</td><td>0.38</td><td>0.19</td><td>0.52</td><td>0.79</td><td>0.43</td><td>0.48</td></tr><tr><td>poster_translation</td><td>0.94</td><td>0.89</td><td>0.15</td><td>0.12</td><td>1.34</td><td>0.35</td><td>0.40</td><td>0.54</td></tr><tr><td>hdr_poster</td><td>2.63</td><td>0.59</td><td>0.49</td><td>0.31</td><td>0.57</td><td>0.65</td><td>0.40</td><td>0.12</td></tr><tr><td>poster_6dof</td><td>3.56</td><td>0.82</td><td>0.30</td><td>0.28</td><td>1.50</td><td>0.35</td><td>0.26</td><td>0.14</td></tr><tr><td>Average</td><td>2.58</td><td>0.69</td><td>0.47</td><td>0.25</td><td>1.45</td><td>0.54</td><td>0.39</td><td>0.24</td></tr></table>

Unit:%/m, 0.24 means the average error would be 0.24m for 100m motion

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/5203a074f0ebe9937a8739c9799c55228e70a128cc5138dc2125ffba5832204e.jpg)

(a) boxes\_translation  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/49eab468377d664a59576901e62d139996217cdcc9397adccc5d788e46d271c1.jpg)

(b) dynamic\_translation  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/55dc616355fdd51894466191376fab751b0e5764a06fd699e500f4d47e493f3c.jpg)  
(c) poster\_6dof  
Fig. 5. The relative pose error comparison of our PL-EVIO with EIO [13], Ultimate-SLAM [14], and our EIO [5].

As can be seen from Table III, our PL-EVIO achieves state-of-the-art performance. Fig.5 presents the relative error of our PL-EVIO against other methods, for the sequence box\_translation, dynamic\_translation and poster\_6dof. It’s important to note that, although the Ultimate-SLAM [14] (EVIO version) demonstrates performance similar to ours, it relies on different parameters for different sequence. While we consider parameter tuning to be impractical, we evaluate our methods using fixed parameters for various sequences during the evaluations.

## C. Online Quadrotor-Flight Evaluation

To further demonstrate the capabilities of our PL-EVIO, we perform real-world experiments on a self-designed quadrotor platform (shown in Fig.6), carrying a forward-looking IniVation DAVIS346 sensor. An Intel NUC10i7FNH computer running Ubuntu 20.04 is mounted on our quadrotor for onboard computational support. We use Pixracer (FMUv4) autopilot to run the PX4 flight stack. To alleviate disturbance from the motion capture system’s infrared light on the event camera, we add an infrared filter on the lens surface of the DAVIS346 camera. Note that the introduction of the infrared filter might cause the degradation of perception for both the event and image camera during the evaluation in subsection IV-A, IV-C, IV-D, and IV-F. The overall weight of our quadrotor is 1.364kg (GS330 frame with T-Motor F60).

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/5309adc7ffb6736a34aac70a41b7f0339da29bb2ec1eec345979cbdad3b33881.jpg)  
Fig. 6. Our self-designed quadrotor platform.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/a10795f69e94c6421d38edd301e56425d4b2081b37bbfdf621cfbb63a356fefc.jpg)

Fig. 7. The estimated trajectory of our PL-EVIO on the quadrotor flight and its comparison against the ground truth (Taking the Onboard\_test\_1 as an example).  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/51358056e369ccbe9ba32b7845142bbe052293798fcbde04d884ad359bcfd34c.jpg)  
Fig. 8. Onboard quadrotor flight in screw pattern using our PL-EVIO as feedback control.

In the experiments, the reference trajectories are generated offline. The polynomial trajectory generation method [48] is used to ensure the motion feasibility of the quadrotor. To follow the generated trajectory, a cascaded feed-forward P.I.D. controller is constructed as a high-level position controller running on NUC. Given the position, velocity, and acceleration as inputs, the high-level feed-forward controller computes desired attitude and throttle sent to the low-level controller running on PX4.

We conduct four flight experiments to test the performance of autonomous trajectory tracking using our PL-EVIO. The quadrotor is commanded to track different patterns as follows (Offboard and Onboard means using the VICON and our PL-EVIO as pose feedback control, respectively, while our PL-EVIO runs real-time and online calculations in the onboard computer):

1) Offboard\_test\_1 and Onboard\_test\_1: The states estimate from the VICON (Offboard\_test\_1) and our PL-EVIO (Onboard\_test\_1) are used for feedback control of the quadrotor which is commanded to track a figure-eight pattern with each circle being 0.625m in radius and 1.2m in height, shown in Fig.7. The yaw angle of the commanded figure-eight pattern is fixed. The quadrotor follows this trajectory ten times continuously during the experiment. The 1000-HZ online calculation of our PL-EVIO is also recorded for accuracy comparison.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/56a360f2ece1819d289431f6a13224861330b0bcfefb5af97630236891065a20.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/dad8eaad1c92cf4cfad87702d3643567161e6223fd7b0d66c6f07f88877c9399.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/f2138a29d1fbd2bbcb0d053af7df55f1ec7a68f58858896b556cb9be3d183d98.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/8609a422826af4cde2cdf5231ad88ab1d4aac528b2341e3faffde770164caec6.jpg)

(a) X-axis  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/ca082f20824d966d4d9fc3c8ec7511b7ce9ce106b739b426c94f7bd871f10108.jpg)  
(b) Y-axis

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/df9a7aeb798e402dffeba612b830ecb66452fffd3f61d1927da6ce9f2f9c105f.jpg)  
(c) Z-axis

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/182cd0caeeb2a90261876c1b3ef9459133916641eaeda5bf3f39264ced55d91a.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/986f59aefa9d92817cc145a0180f32408e9dc5eb18a31bb6e6586e7dc625fc9b.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/2fbfc126dca10178135cc7b72a1a92204fa34e6002a995564cefdcd23df7d217.jpg)  
(d) Roll-axis

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/eb8ac97131899ef1be354724d8860016d4f2f0e429f025db748516d29c5a448d.jpg)

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/c75a37e49090afe43dd74b96dce6688b26f41f385449c427d877424364fbb95e.jpg)  
(e) Pitch-axis

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/ace14d93e84cc61573789061e47bca411d973f9931f0fda1197e44aedcbef978.jpg)  
(f) Yaw-axis  
Fig. 9. The position, orientation, and the corresponding errors of our PL-EVIO in onboard flight compared with the ground truth from VICON (Taking the Onboard\_test\_1 as example).

TABLE IV  
ACCURACY COMPARISON OF OUR PL-EVIO WITHGROUNDTRUTH IN QUADROTOR FLIGHT
<table><tr><td rowspan="2">Sequence</td><td colspan="3">Translation Error</td><td colspan="3">Rotation Error</td></tr><tr><td>Mean</td><td>RMSE</td><td>Std</td><td>Mean</td><td>RMSE</td><td>Std</td></tr><tr><td>Offboard_test_1</td><td>0.054</td><td>0.061</td><td>0.028</td><td>0.094</td><td>0.095</td><td>0.015</td></tr><tr><td>Onboard_test_1</td><td>0.078</td><td>0.084</td><td>0.030</td><td>0.078</td><td>0.087</td><td>0.039</td></tr><tr><td>Onboard_test_2</td><td>0.081</td><td>0.093</td><td>0.046</td><td>0.056</td><td>0.059</td><td>0.019</td></tr></table>

Unit: m for translation and deg for rotation

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/21a07c3987c2c12aaef6db1c308b1b6bdfb10f2834a229aa27a07ac0c4081a2d.jpg)  
Fig. 10. The estimated trajectory of our PL-EVIO on the quadrotor flip, and image/event measurement.

2) Onboard\_test\_2: The states estimate from our PL-EVIO are used for feedback control of the quadrotor which is commanded to track a screw pattern shown in Fig.8. The quadrotor follows this trajectory ten times continuously during the experiment. The 1000-HZ onboard state estimates of our PL-EVIO enable real-time feedback control of the quadrotor. The ground truth is obtained from VICON. The translation and rotation error are shown in Table IV. Taking the Onboard\_test\_1 as an example, in Fig.7 and Fig.9, we further illustrate the estimated trajectories (translation and rotation) of our PL-EVIO against the ground truth, as well as their corresponding errors. The total trajectory length is 101.15m. The translation errors in the X, Y, and Z dimensions are all within 0.1m, while the rotation error of the Roll and Pitch dimensions are within 2<sup>◦</sup>, and the one in the Yaw dimension is within 6<sup>◦</sup>.

## D. Aggressive Quadrotor-Flip Evaluation

In this section, we further conduct onboard quadrotor flip experiments to evaluate the performance of our PL-EVIO in aggressive motion. The estimated trajectory of our PL-EVIO compared with the ground truth from VICON during the flip evaluations can be seen in Fig.10. The total length of the trajectory is 15m. The mean translation error and the mean angular error are 0.097m and 6.0<sup>◦</sup>, respectively. Despite the extreme velocity of the motion, our PL-EVIO successfully tracks the quadrotor pose with high accuracy. Note that our PL-EVIO is run onboard during the quadrotor flip experiments. There are only a few image measurements captured during aggressive motion due to motion blur, whereas the event measurements are severely limited when the quadrotor is hovering. Thanks to our well-designed feature management and the complementarity of three kinds of features, our PL-EVIO can provide robust and good performance in multiple quadrotor flip experiments.

## E. Real-Time Analysis

We assessed the real-time performance of our system on quadrotor flight using an Intel NUC10i7FNH as the computing platform. The computational allocations are presented in Table V. The proposed algorithm sequentially processes

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/03b6c9cc0c4b3884e47d240f605a0b1434e4e4d5d2250ae1b7c0079ebf81d39b.jpg)  
(b) HKU\_main\_building

Fig. 11. (a) The estimated trajectory of our PL-EVIO in the outdoor environment. We also visualize the detection and tracking situation of the event-corner features, line-based event features, and point-based image features, during the experiment. The combination of these features provides more structures and constraints in the scene that ensure robustness. (b) The estimated trajectory of our PL-EVIO as well as the detection and matching performance of the line-based event features.

the event queue, with the event front-end completed within 9 ms, the image front-end completed within 3 ms, and overall optimization completed within 50 ms, without any hardware acceleration. In order to achieve low latency, we employ a loosely-coupled approach to directly propagate the latest EVIO estimation along with the IMU measurements. This results in IMU-rate EVIO outputs that can reach up to 1000 Hz. This is critical for achieving onboard quadrotor flight, using our PL-EVIO as pose feedback control, as discussed in Sections IV-C and IV-D. Due to the reliable, low-drift, and low-latency characteristics of our PL-EVIO, the flight control system can quickly obtain accurate pose feedback, thereby ensuring the success of onboard quadrotor flight. To further demonstrate the real-time capabilities of our proposed

PL-EVIO system, which offers onboard pose feedback for quadrotor flights, we encourage readers to refer to the video demos.<sup>2</sup>

## F. Outdoor Large-Scale Evaluation

1) Natural Scenarios: In this section, we evaluate our PL-EVIO system in a large-scale environment that encompasses the HKU campus. This environment includes features such as moving pedestrians, low-texture areas, long-term movement, strong sunlight, and indoor & outdoor transitions. We also return to the same location after a large loop to evaluate the loop closure. The total evaluation length is approximately 980 m, covering an area of approximately 160 m in length, 100 m in width, and 10 m in height changes. The estimated trajectory is aligned with the Google map and can be seen in Fig.11(a). The results show that our PL-EVIO performed almost drift-free in this long-term motion evaluation. The complementarity of three different kinds of features (e.g. line-based event features for humanmade environment, point-based event features for HDR scene, and point-based image features for good lighting scene) ensure the robust and reliable state estimation.

TABLE V  
TIME CONSUMPTION OF DIFFERENT MODULES IN OUR PL-EVIO
<table><tr><td colspan="3">Modules Time-cost (ms)</td></tr><tr><td rowspan="6">Event Front-end</td><td>Point-based event feature detection</td><td>0.41</td></tr><tr><td>Point-based event feature tracking</td><td>0.62</td></tr><tr><td>Line-based event feature detection</td><td>2.88</td></tr><tr><td>Line-based event feature Matching</td><td>3.57</td></tr><tr><td>Total</td><td>8.68</td></tr><tr><td>Point-based image feature detection</td><td>0.70</td></tr><tr><td rowspan="3">Image Front-end</td><td>Point-based image feature tracking</td><td>0.42</td></tr><tr><td>Total</td><td>2.29</td></tr><tr><td>Construct point-based event residual</td><td>0.089</td></tr><tr><td rowspan="6">Back-end</td><td>Construct line-based event residual</td><td>0.0026</td></tr><tr><td>Construct point-based image residual</td><td>0.66</td></tr><tr><td>Construct marginalization residuals</td><td>7.50</td></tr><tr><td>Solve graph optimization using Ceres</td><td>37.66</td></tr><tr><td>IMU forward</td><td>0.0056</td></tr><tr><td>Total</td><td>50.49</td></tr></table>

2) Human-Made Scenarios: We further conduct additional evaluations specifically focusing on building scenarios. Utilizing the line features can better represent the geometric information constraints in Human-made structures, as illustrated in Fig.11(b). The experimental results demonstrate that after a long-distance loop of approximately 260 m within the interior of the building, our PL-EVIO system maintains high accuracy, forming a complete square shape without significant drifts. To assess this accuracy, we specifically choose the gate of the building as the starting and ending point for quantitative evaluation, and the end-to-end distance showed an error of 0.61 m. Owing to the additional geometric structural information provided by our proposed event-based line features, our PL-EVIO achieves low drift and reliable performance in this large-scale environment.

## V. CONCLUSION

In this paper, we propose a robust, highly-accurate, and realtime optimization-based monocular VIO that tightly fuses the event, image, and IMU together, with point and line features. The combination of point-based event-corner features, linebased event features, and point-based image features would provide more geometric constraints on the structure of the environment. Finally, we show superior performance by comparing against other state-of-the-art open-source image-based or event-based VIO implementations in different challenge datasets. Meanwhile, through extensive experiments including extremely aggressive motion and large-scale evaluation, we also show that our PL-EVIO pipeline is able to leverage the properties of the standard camera and the event camera with different features to provide robust state estimation. We hope that this work can inspire other researchers and industries to push wide applications for event cameras on robotics and perception. In our future work, event-based multi sensor fusion, including a wider range of local perception (such as LiDAR), and global perception (such as visible light positioning [49] for indoor, or GPS for outdoor), might be deeply studied to exploit the complementary advantage of different sensors with event cameras.

## APPENDIX A ABLATION STUDY

Our approach involves extracting event-corner features from events-only data and line-based features from the event mat. These two types of features are then associated using a spatio-temporal locality scheme based on exponential decay, which is commonly referred to as TS. The TS was converted from SAE with the exponential decay kernel. To enhance the accuracy of event-based point and line tracking, we incorporate polarity into the TS $( T _ { p } ( { \pmb x } , t ) )$ , which can be represented as follows:

$$
T _ { p } ( \pmb { x } , t ) = p \cdot \mathrm { e x p } ( - \frac { t - t _ { l a s t } ( \pmb { x } ) } { \eta } )\tag{A-1}
$$

We use the LK optical flow on the $T _ { p } ( { \pmb x } , t )$ to associate the current event-corner with the most recent event-corner in the kernel operation, assuming that it is the same event-corner in a recent position. Our approach utilizes the motion variance characteristics of the $T _ { p } ( { \pmb x } , t )$ to retain relevant context while associating event-based point and line features into tracks to ensure computational efficiency in the front-end. In our previous work [5], we have presented the process of generating uniform event-corner features (as shown in Fig.12) and discussed the rationale for employing our $T _ { p } ( { \pmb x } , t )$ for tracking event-corner features. Additionally, we also provided the normalized TS without polarity $( T _ { n p } ( { \pmb x } , t ) )$

$$
T _ { n p } ( { \pmb x } , t ) = ( \frac { 2 5 5 . 0 } { \operatorname* { m a x } ( T ^ { \prime } ) - \operatorname* { m i n } ( T ^ { \prime } ) } ) \cdot ( T ^ { \prime } - \operatorname* { m i n } ( T ^ { \prime } ) )\tag{A-2}
$$

where, $T ^ { \prime }$ can be obtained from Eq.2.

## A. Ablation Study on Different Event Representations for Event-Based Feature Tracking

In this section, we focus on conducting an ablation study of the event-based point tracking performance using different event representations, including our $T _ { p } ( { \pmb x } , t ) , \ T _ { n p } ( { \pmb x } , t )$ TS in [26], and the event accumulated image in [14] and [24]. It should be noted that we previously only used $T _ { n p } ( { \pmb x } , t )$ for loop closure detection, while we merely investigate its feature tracking performance in front-end for this ablation study. During the ablation experiments, we employ our PL-EIO framework to control variables by only altering different event representations used for event-based feature tracking, while keeping the event feature points generated from asynchronous event streams unchanged. The qualitative results are presented in Fig. 13, and the quantitative evaluations are reported in the “Event Representations” section of Table VI. We utilize absolute trajectory error (ATE) aligning the estimated trajectory with ground truth using 6-DOF transformation (in SE3) to quantitatively evaluate the accuracy. The results indicate that only our proposed $T _ { p } ( { \pmb x } , t )$ is capable of reliably estimating the state, while other event representations used for event-based feature tracking failed in the challenge situation. This could be due to the insufficient intensity information available to satisfy the requirements of LK optical flow for event-based feature tracking. For example, the image generated from event streams [14], [24] is a binary edge image consisting of only two possible pixel values (0 or 1), which lacks the necessary information for accurately calculating gradients. Consequently, it becomes difficult to determine the direction and magnitude of motion of feature points, resulting in increased difficulty for optical flow to remove local outliers. In contrast, our TS with polarity can ensure reliable data association between event features of adjacent frames, which effectively prevents missmatches.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/4080090aedb3e04ac5b471b1e1dd1d910646e9f9c661de097e340dc600802b38.jpg)  
(a) Raw Event Stream

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/f0c8217e2c7d63ffcb27e0b43f17d09f5be7b45bca23d02e5cbc303e9a257cc7.jpg)  
(b) TS with Polarity

Fig. 12. Event-corner features generation. The event-corner features are firstly extracted from the asynchronous event stream (a), then the TS with polarity (b) is used as a mask to further select the event-corner features, ensuring a uniform distribution.  
TS with polarity  
Normalized TS without polarity  
TS  
Event accumulated image  
![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/f68935ea29fe05161dce9a438d2143056aeacb447755a37ad3b92a4081078172.jpg)  
Fig. 13. The performance of the event-corner features tracking in different event representations. Note that the VIO is a highly nonlinear system, it is hard to prove the performance through a single timestamp. Therefore we refer the readers to our video demo:https://b23.tv/eIRQMST, which shows the reliable performance of our PL-EVIO.

TABLE VI  
ACCURACY RESULT OF THE ABLATION STUDY IN HKU\_AGG\_FLIP
<table><tr><td>Event Representations</td><td>Tp(x, t)</td><td> $\underline { { T _ { n p } ( { \pmb x } , t ) } }$ </td><td>TS</td><td>Event-image</td></tr><tr><td>PL-EIO Event+IMU</td><td>0.25</td><td>failed</td><td>failed</td><td>failed</td></tr><tr><td>Time Decay Kernel</td><td>10</td><td>20</td><td>60</td><td>100</td></tr><tr><td>PL-EIO Event+IMU</td><td>0.30</td><td>0.25</td><td>failed</td><td>failed</td></tr><tr><td rowspan="2">Methods</td><td>PL-EVIO</td><td>Ref. [14]</td><td>Ref. [24]</td><td>Ref. [26]</td></tr><tr><td>0.12</td><td>Event+Image+IMU Event+Image+IMU 2.66</td><td>Event failed</td><td>Stereo Event failed</td></tr></table>

## B. Ablation Study on Time Decay Kernels of the TS With Polarity

In order to further investigate the impact of time decay kernels on our event representations $( T _ { p } ( { \pmb x } , t ) )$ used for event feature tracking, we conducted ablation experiments on the time decay kernels. The qualitative results on the event-based point and line features are shown in Fig.14. From Fig.14(a), we can observe that the tracking and matching performance of event-based point features and line features are not significantly different across various exponential decay kernels. This may be attributed to the normal texture conditions and fewer triggered events at a far distance in outdoor environments. However, upon careful observation, we still can notice that larger time decay kernels result in coarser edge contours in the edge regions, which may introduce systematic errors to the visual front-end. In contrast, in indoor environments (as shown in Fig.14(b)) with abundant texture, we found that the larger time decay kernels lead to a significant decrease in the successful matching of both event-based line and point features. This might be due to the trailing effect caused by a large time decay kernel, which can create negative effects (i.e. lead to failure during aggressive motion) similar to motion blur. Therefore, we choose a time decay kernel of 20ms to

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/e11767596b03c3aa35b8ad98576ef376de7855c716c00e81347124d5620f9e8e.jpg)  
(b) HKU\_agg\_flip  
Fig. 14. The tracking and matching performance of event-based point and line features in various time decay parameters. We only evaluate the tracking and matching performance in (a) outdoor environments with large-scale and (b) indoor environments with aggressive motion, respectively.

ensure sufficient information for event-based feature match- We quantitatively evaluate the influence of the time decay ing and tracking while minimizing blur and trailing effects. kernel on the performance of pose estimation through the Time

Decay Kernel part of Table VI. Furthermore, we also compare the performance of our PL-EVIO using $T _ { p } ( { \pmb x } , t )$ as event representation with other event representations from [14], [24], and [26], in the Methods part of Table VI.

## REFERENCES

[1] G. Gallego et al., “Event-based vision: A survey,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 44, no. 1, pp. 154–180, Jan. 2022.

[2] A. Pumarola, A. Vakhitov, A. Agudo, A. Sanfeliu, and F. Moreno-Noguer, “PL-SLAM: Real-time monocular visual SLAM with points and lines,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), May 2017, pp. 4503–4508.

[3] B. Xu, P. Wang, Y. He, Y. Chen, Y. Chen, and M. Zhou, “Leveraging structural information to improve point line visual-inertial odometry,” IEEE Robot. Autom. Lett., vol. 7, no. 2, pp. 3483–3490, Apr. 2022.

[4] Q. Fu et al., “PL-VINS: Real-time monocular visual-inertial SLAM with point and line features,” 2020, arXiv:2009.07462.

[5] W. Guan and P. Lu, “Monocular event visual inertial odometry based on event-corner using sliding windows graph-based optimization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2022, pp. 2438–2445.

[6] I. Alzugaray and M. Chli, “ACE: An efficient asynchronous corner tracker for event cameras,” in Proc. Int. Conf. 3D Vis. (3DV), Sep. 2018, pp. 653–661.

[7] A. Z. Zhu, N. Atanasov, and K. Daniilidis, “Event-based feature tracking with probabilistic data association,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), May 2017, pp. 4465–4470.

[8] A. Dietsche, G. Cioffi, J. Hidalgo-Carrió, and D. Scaramuzza, “Powerline tracking with event cameras,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2021, pp. 6990–6997.

[9] C. Le Gentil, F. Tschopp, I. Alzugaray, T. Vidal-Calleja, R. Siegwart, and J. Nieto, “IDOL: A framework for IMU-DVS odometry using lines,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2020, pp. 5863–5870.

[10] G. Chen, H. Cao, J. Conradt, H. Tang, F. Rohrbein, and A. Knoll, “Eventbased neuromorphic vision for autonomous driving: A paradigm shift for bio-inspired visual sensing and perception,” IEEE Signal Process. Mag., vol. 37, no. 4, pp. 34–49, Jul. 2020.

[11] B. Kueng, E. Mueggler, G. Gallego, and D. Scaramuzza, “Low-latency visual odometry using event-based feature tracks,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2016, pp. 16–23.

[12] D. Gehrig, H. Rebecq, G. Gallego, and D. Scaramuzza, “EKLT: Asynchronous photometric feature tracking using events and frames,” Int. J. Comput. Vis., vol. 128, no. 3, pp. 601–618, Mar. 2020.

[13] H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Real-time visualinertial odometry for event cameras using keyframe-based nonlinear optimization,” in Proc. Brit. Mach. Vis. Conf., 2017, pp. 1–8.

[14] A. R. Vidal, H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Ultimate SLAM? Combining events, images, and IMU for robust visual SLAM in HDR and high-speed scenarios,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 994–1001, Apr. 2018.

[15] E. Rosten, R. Porter, and T. Drummond, “Faster and better: A machine learning approach to corner detection,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 32, no. 1, pp. 105–119, Jan. 2010.

[16] J. Shi, “Good features to track,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jul. 1994, pp. 593–600.

[17] B. D. Lucas and T. Kanade, “An iterative image registration technique with an application to stereo vision,” in Proc. 7th Int. Joint Conf. Artif. Intell., Vancouver, BC, Canada, vol. 2, 1981, pp. 674–679.

[18] X. Lagorce, G. Orchard, F. Galluppi, B. E. Shi, and R. B. Benosman, “HOTS: A hierarchy of event-based time-surfaces for pattern recognition,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 39, no. 7, pp. 1346–1359, Jul. 2017.

[19] V. Vasco, A. Glover, and C. Bartolozzi, “Fast event-based Harris corner detection exploiting the advantages of event-driven cameras,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2016, pp. 4144–4149.

[20] I. Alzugaray and M. Chli, “Asynchronous corner detection and tracking for event cameras in real time,” IEEE Robot. Autom. Lett., vol. 3, no. 4, pp. 3177–3184, Oct. 2018.

[21] D. Weikersdorfer, R. Hoffmann, and J. Conradt, “Simultaneous localization and mapping for event-based vision systems,” in Proc. Int. Conf. Comput. Vis. Syst. Cham, Switzerland: Springer, 2013, pp. 133–142.

[22] A. Censi and D. Scaramuzza, “Low-latency event-based visual odometry,” in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), May 2014, pp. 703–710.

[23] H. Kim, S. Leutenegger, and A. J. Davison, “Real-time 3D reconstruction and 6-DoF tracking with an event camera,” in Proc. Eur. Conf. Comput. Vis. Cham, Switzerland: Springer, 2016, pp. 349–364.

[24] H. Rebecq, T. Horstschaefer, G. Gallego, and D. Scaramuzza, “EVO: A geometric approach to event-based 6-DOF parallel tracking and mapping in real time,” IEEE Robot. Autom. Lett., vol. 2, no. 2, pp. 593–600, Apr. 2017.

[25] H. Rebecq, G. Gallego, E. Mueggler, and D. Scaramuzza, “EMVS: Event-based multi-view stereo—3D reconstruction with an event camera in real-time,” Int. J. Comput. Vis., vol. 126, no. 12, pp. 1394–1414, Dec. 2018.

[26] Y. Zhou, G. Gallego, and S. Shen, “Event-based stereo visual odometry,” IEEE Trans. Robot., vol. 37, no. 5, pp. 1433–1450, Oct. 2021.

[27] A. Hadviger, I. Cvišic, I. Markovic, S. Vražic, and I. Petrovic, “Featurebased event stereo visual odometry,” in Proc. Eur. Conf. Mobile Robots (ECMR), Aug. 2021, pp. 1–6.

[28] A. Z. Zhu, N. Atanasov, and K. Daniilidis, “Event-based visual inertial odometry,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jul. 2017, pp. 5816–5824.

[29] F. Mahlknecht et al., “Exploring event camera-based odometry for planetary robots,” IEEE Robot. Autom. Lett., vol. 7, no. 4, pp. 8651–8658, Oct. 2022.

[30] E. Mueggler, G. Gallego, H. Rebecq, and D. Scaramuzza, “Continuoustime visual-inertial odometry for event cameras,” IEEE Trans. Robot., vol. 34, no. 6, pp. 1425–1440, Dec. 2018.

[31] P. Chen, W. Guan, and P. Lu, “ESVIO: Event-based stereo visual inertial odometry,” IEEE Robot. Autom. Lett., vol. 8, no. 6, pp. 3661–3668, Jun. 2023.

[32] Z. Liu, D. Shi, R. Li, and S. Yang, “ESVIO: Event-based stereo visualinertial odometry,” Sensors, vol. 23, no. 4, p. 1998, 2023.

[33] W. Chamorro, J. Solà, and J. Andrade-Cetto, “Event-based line SLAM in real-time,” IEEE Robot. Autom. Lett., vol. 7, no. 3, pp. 8146–8153, Jul. 2022.

[34] W. O. C. Hernández, J. Andrade-Cetto, and J. Solà Ortega, “High-speed event camera tracking,” in Proc. 31st Brit. Mach. Vis. Virtual Conf., 2020, pp. 1–12.

[35] R. G. von Gioi, J. Jakubowicz, J.-M. Morel, and G. Randall, “LSD: A fast line segment detector with a false detection control,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 32, no. 4, pp. 722–732, Apr. 2010.

[36] T. Qin, P. Li, and S. Shen, “VINS-Mono: A robust and versatile monocular visual-inertial state estimator,” IEEE Trans. Robot., vol. 34, no. 4, pp. 1004–1020, Aug. 2018.

[37] T. Qin and S. Shen, “Robust initialization of monocular visual-inertial estimation on aerial robots,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2017, pp. 4225–4232.

[38] D. Falanga, K. Kleber, and D. Scaramuzza, “Dynamic obstacle avoidance for quadrotors with event cameras,” Sci. Robot., vol. 5, no. 40, Mar. 2020, Art. no. eaaz9712.

[39] L. Zhang and R. Koch, “An efficient and robust line segment matching approach based on LBD descriptor and pairwise geometric consistency,” J. Vis. Commun. Image Represent., vol. 24, no. 7, pp. 794–805, Oct. 2013.

[40] G. Zhang, J. H. Lee, J. Lim, and I. H. Suh, “Building a 3-D linebased map using stereo SLAM,” IEEE Trans. Robot., vol. 31, no. 6, pp. 1364–1377, Dec. 2015.

[41] A. Bartoli and P. Sturm, “The 3D line motion matrix and alignment of line reconstructions,” Int. J. Comput. Vis., vol. 57, no. 3, pp. 159–178, May 2004.

[42] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel, and J. D. Tardós, “ORB-SLAM3: An accurate open-source library for visual, visual–inertial, and multimap SLAM,” IEEE Trans. Robot., vol. 37, no. 6, pp. 1874–1890, Dec. 2021.

[43] Michael Grupp. (2017). EVO: Python Package for the Evaluation of Odometry and SLAM. [Online]. Available: https://github. com/MichaelGrupp/evo

[44] J. Delmerico, T. Cieslewski, H. Rebecq, M. Faessler, and D. Scaramuzza, “Are we ready for autonomous drone racing? The UZH-FPV drone racing dataset,” in Proc. Int. Conf. Robot. Autom. (ICRA), May 2019, pp. 6713–6719.

[45] T. Qin, J. Pan, S. Cao, and S. Shen, “A general optimization-based framework for local odometry estimation with multiple sensors,” 2019, arXiv:1901.03638.

[46] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, and D. Scaramuzza, “The event-camera dataset and simulator: Event-based data for pose estimation, visual odometry, and SLAM,” Int. J. Robot. Res., vol. 36, no. 2, pp. 142–149, Feb. 2017.

[47] I. Alzugaray and M. Chli, “Asynchronous multi-hypothesis tracking of features with event cameras,” in Proc. Int. Conf. 3D Vis. (3DV), Sep. 2019, pp. 269–278.

[48] D. Mellinger and V. Kumar, “Minimum snap trajectory generation and control for quadrotors,” in Proc. IEEE Int. Conf. Robot. Autom., May 2011, pp. 2520–2525.

[49] Z. Yan, W. Guan, S. Wen, L. Huang, and H. Song, “Multirobot cooperative localization based on visible light positioning and odometer,” IEEE Trans. Instrum. Meas., vol. 70, pp. 1–8, 2021.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/08aafcbb60e85ccd979adc97f2cc3902e2dfc1520c72287093847b2ef42bcf80.jpg)

Weipeng Guan received the bachelor’s and master’s degrees from the South China University of Technology. He is currently pursuing the Ph.D. degree with The University of Hong Kong. He has worked with several reputable organizations, including Samsung Electronics, Huawei Technologies, TP-LINK, The Chinese Academy of Sciences, The Chinese University of Hong Kong, and The Hong Kong University of Science and Technology. He was a Technical Consultant for multiple companies, such as TCL. Moreover, he has authored or coauthored over 60 research articles in prestigious international journals and conferences and holds more than 40 authorized patents. His research interests primarily focus on robotics, event-based VO/VIO/SLAM, and visible light positioning.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/2b239760ae2189864652620a67366fb99b7bbe4c4d5205f69260a94963d2e7e5.jpg)

Peiyu Chen received the B.Sc. degree in automation from the Nanjing University of Science and Technology, China, in 2020, and the M.Sc. degree in computer control and automation from Nanyang Technological University, Singapore, in 2022. He is currently pursuing the Ph.D. degree with The University of Hong Kong. His research interests include robotics, visual-inertial simultaneous localization and mapping, and nonlinear control.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/82e43ec2ea32783cc4c29bc056efe9780bc3af4ee72a9662fc10c778aa8a313a.jpg)

Yuhan Xie (Member, IEEE) received the B.Eng. degree in automation from the School of Automation Science and Electrical Engineering, Beihang University, in 2020. She is currently pursuing the M.Phil. degree in robotics planning and control with the Department of Mechanical Engineering, The University of Hong Kong. Her research interests include unmanned aerial vehicles plan and control with deep reinforcement learning.

![](images/2024_PL-EVIO__Robust_Monocular_Event-Based_Visual_Inertial_Od/a0e16a4ab4377a11ebe0ceea73c5614c1b54b92b464433c51b10d5f32893942b.jpg)

Peng Lu received the B.Sc. degree in automatic control and the M.Sc. degree in nonlinear flight control from Northwestern Polytechnical University (NPU) and the Ph.D. degree from the Delft University of Technology (TU Delft) in 2016. He continued his journey on flight control with TU Delft. After that, he shifted a bit from flight control and started to explore control for ground/construction robotics with ETH Zürich (ADRL Laboratory) as a Post-Doctoral Researcher in 2016. He also had a short but nice journey with the University of Zurich & ETH Zürich (RPG Group), where he was working on vision-based control for UAVs as a Post-Doctoral Researcher. He was an Assistant Professor in autonomous UAVs and robotics with The Hong Kong Polytechnic University prior to joining The University of Hong Kong in 2020.

He has received several awards, such as third place in 2019 IROS autonomous drone racing competition and best graduate student paper finalist in AIAA GNC (top conference in aerospace). He serves as an Associate Editor for 2020 IROS (top conference in robotics) and the Session Chair/Co-Chair for conferences, such as IROS and AIAA GNC for several times. He also gave a number of invited/keynote speeches at multiple conferences, universities, and research institutes.