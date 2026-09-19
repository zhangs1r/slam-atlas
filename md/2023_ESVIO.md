# ESVIO: Event-Based Stereo Visual Inertial Odometry

Peiyu Chen , Weipeng Guan, and Peng Lu

Abstract—Event cameras that asynchronously output lowlatency event streams provide great opportunities for state estimation under challenging situations. Despite event-based visual odometry having been extensively studied in recent years, most of them are based on the monocular, while few research on stereo event vision. In this letter, we present ESVIO, the first event-based stereo visual-inertial odometry, which leverages the complementary advantages of event streams, standard images, and inertial measurements. Our proposed pipeline includes the ESIO (purely event-based) and ESVIO (event with image-aided), which achieves spatial and temporal associations between consecutive stereo event streams. A well-design back-end tightly-coupled fused the multisensor measurement to obtain robust state estimation. We validate that both ESIO and ESVIO have superior performance compared with other image-based and event-based baseline methods on public and self-collected datasets. Furthermore, we use our pipeline to perform onboard quadrotor flights under low-light environments. Autonomous driving data sequences and real-world large-scale experiments are also conducted to demonstrate long-term effectiveness. We highlight that this work is a real-time, accurate system that is aimed at robust state estimation under challenging environments.

Index Terms—Visual-Inertial SLAM, sensor fusion, aerial systems: perception and autonomy.

## I. INTRODUCTION

VENT cameras are novel bio-inspired sensors [1], which of standard cameras) to handle broad illumination conditions. Unlike standard cameras that output fixed-rate image frames, event cameras respond to pixel-level intensity changes and output asynchronous event streams at the latency of microsecond level, which endows these novel sensors to tackle high-speed motion without motion blur.

Most of the existing event-based visual odometers (VO) use monocular event camera [2], [3], [4], while few research on visual odometry based on stereo event cameras [5], [6]. Since event cameras output asynchronous event streams rather than fixedrate image frames, the traditional image-based instantaneous matching cannot be directly implemented on event streams. For consecutive stereo event streams, merely relying on temporal constraints to extract and match event-corner features might lead to many false correspondences. Temporal deviations between event streams, noise impacts, different contrast sensitivity of sensors, etc. cause the above problem. Therefore, it is crucial to extract apposite event-corner features and design proper constraints to achieve data association between stereo event-corner features.

Compared with standard cameras, event cameras do not suffer from motion blur under aggressive motion. However, when the relative motion between event cameras and scenes is restricted, e.g. in the stationary state, event streams might not be reliably generated and transmitted, whereas standard cameras are able to provide rich information most of the time (e.g. low-speed motion and well-lit scenes).Observing this complementarity, leveraging both of the advantages of the aforementioned different sensors in combination with an inertial measurement unit (IMU) results in a robust and accurate visual-inertial odometry (VIO) pipeline [3], [4].

In this letter, we propose, to the best ofour knowledge, the first published event-based stereo visual-inertial odometry (ESVIO). Our contributions are summarized as follows:

1) In order to achieve robust state estimation under aggressive motion and low-light scenarios, we propose the first purely event-based stereo inertial odometry (ESIO) pipeline with sliding windows graph-based optimization, and further extend it with image-aided (ESVIO) which tightly integrates stereo event streams, stereo image frames, and IMU together.

2) To tackle the problem of event-based stereo feature tracking and matching, we design geometry-based spatial and temporal data associations in consecutive stereo event streams. The spatial and temporal constraints ensure accurate and reliable state estimation. Moreover, a motion compensation method is designed to emphasize the edge of scenes by warping each event.

3) We evaluate that our ESVIO can achieve state-of-theart performance on publicly available datasets. We also release a very challenging event-based VO/VIO dataset, featuring aggressive motion and HDR scenarios. Finally, we perform onboard closed-loop quadrotor flight using our ESVIO as the estimator.

The remainder of the letter is organized as follows: Section II introduces the related works. Section III introduces the methodology of our methods. Section IV presents the experiments and results. Finally, the conclusion is given in Section V.

## II. RELATED WORKS

## A. Event-Based Monocular Visual Odometry

Event-based monocular VO has been intensively researched for challenging scenarios in recent years. [7] is the first work using feature tracks to achieve event-based VO, which detects features firstly from grayscale frames and then uses event streams tracked features asynchronously. EVO [2] proposed a monocular event-based parallel tracking-and-mapping philosophy which applies the image-to-model alignment for tracking and Event-based Multi-View Stereo (EMVS) [8] for mapping. [9] proposed the first event-based VIO that tackles the incomplete estimation of scale and provides accurate 6-DoF state estimation based on Extended Kalman Filter (EKF). [10] obtains a discrete number of states based on a spatio-temporal window of event streams, and introduces virtual event frames to achieve nonlinear optimization that refines estimated poses. Ultimate SLAM [3] furthered the aforementioned research by combining event streams, image frames, and IMU measurements with nonlinear optimization, which leverages the complementary advantages of event cameras and standard cameras. [11] adopted a continuous-time framework based on cubic spline for smooth trajectory estimation and fused both event streams and IMU together. DEVO [12] proposed a novel VO based on a hybrid setup of depth and event cameras, which construct a semi-dense depth map by thresholding time-surface maps. EKLT-VIO [13] integrated an accurate state-of-the-art event-based feature tracker EKLT [14] with EKF backend to achieve event-based state estimation on Mars-like datasets. [15] proposed a real-time monocular event-based VIO based on graph optimization, which directly utilizes the asynchronous raw events for feature detection. PL-EVIO [4] extended the above method to leverage the complementary advantages of standard and event cameras, which tightly combined event-based point features, event-based line features, image-based point features, and IMU measurements together.

## B. Event-Based Stereo Visual Odometry

In contrast to monocular VO, which requires sufficient parallax to recover the depth of corner features, stereo VO can directly obtain the depth of features at the current timestamp. This solves scale uncertainty and even tracking failure caused by insufficient parallax. However, most of the recent research on stereo event cameras has focused on depth estimation and constructing semi-dense or dense maps [16], [17], with less research on VO/SLAM fields. ESVO [5] proposed the first event-based stereo VO pipeline, which achieves parallel 3D semi-dense mapping thread and tracking thread by maximizing the spatio-temporal consistency of stereo event streams. [6] adopted stereo feature detection and matching with the geometry method, which adopts reprojection error minimization to achieve pose estimation. However, these algorithms merely use events to estimate the state, which might result in tracking failure when the system is stationary. In addition, these aforementioned approaches without combining IMU might lead to losses of visual tracks under textureless areas. Our work fills a gap based on combining stereo event streams, stereo image frames, and IMU together, which can operate in real-time under high-resolution event streams with better performance than previously proposed methods.

## III. METHODOLOGY

Since the procedure of the image measurement is very similar to that of event streams, we only introduce the ESIO in this section. The core of our framework lies in the pre-processing of raw event streams using motion compensation (Section II-I-A) and the data association between consecutive stereo event streams in temporal and spatial (Section III-B). After that, we design the event-based constraint for graph optimization (Section III-C). The pipeline of ESIO can be represented by the ESVIO (shown in Fig. 2) without image-based processing.

![](images/2023_ESVIO/76686ad08e0e052b1c6e7ba249ce6c454d00f71ea1452b5ce75c1a19a83732c9.jpg)  
Fig. 1. Our ESVIO provides robust and accurate, real-time pose feedback for drones under aggressive motion. Events provide rich and reliable features, while only a few features are tracked in image frames in high-speed motion. Left bottom: stereo event-based feature tracking. Right bottom: stereo image-based feature tracking.

![](images/2023_ESVIO/def24400fec4d5a4a9b58561ee5b53f410f29afce5982f4bc66a41940110bf2c.jpg)  
Fig. 2. The structure of our ESVIO and ESIO pipeline.

## A. Motion Compensation for Event Streams

Motion compensation corrects the curved event streams by aligning events corresponding to the same scene edge. [18] only uses the angular velocity ofIMU to achieve rotational compensation while ignoring the effect oftranslation. [19] achieve the rotational and translational compensation by IMU and depth camera respectively. We design a motion compensation approach to correct each raw event position, which uses the angular velocity from the IMU sensor and the linear velocity from our ESVIO back-end to achieve rotational and translational compensation respectively. Since our motion compensation utilizes the estimated velocity from the back-end, it works after the successful initialization.

![](images/2023_ESVIO/a3d55cb929a663b4e419d7f7d360dbd68831f1b242305b9236632b2a3ace1851.jpg)  
Fig. 3. The event streams without and with motion compensation.

Given the kth event as $e _ { k } = \{ l _ { k } , t _ { k } , p _ { k } \}$ , where $l _ { k } = \{ x _ { k } , y _ { k } \}$ represents the pixel location of the event $e _ { k } . t _ { k }$ is the timestamp and $p _ { k }$ represents its polarity. The event $e _ { k }$ is warped from $t _ { k }$ to $t _ { r e f }$ , and the compensated location $^ { r e f } l _ { k }$ define as

$$
^ { r e f } l _ { k } = \mathcal { M } [ x _ { k } , y _ { k } , t _ { k } , \Theta ]\tag{1}
$$

where $\mathcal { M }$ is the motion compensation function. $\Theta$ represents compensation parameters. Since the short time interval $\Delta t$ between $t _ { r e f }$ and $t _ { k }$ , we assume that the motion during this period is uniform motion, thereby the ego-motion estimation of each event can be formulated as follow:

$$
^ { r e f } \mathbf { R } _ { k } = \exp { ( ( \tilde { \omega } _ { k } - \mathbf { b } _ { g } ( t _ { k } ) - \mathbf { n } _ { g } ( t _ { k } ) ) \Delta t ) }\tag{2a}
$$

$$
{ \mathbf { \Lambda } ^ { r e f } } { \mathbf { L } } _ { k } = { \mathbf { \Lambda } ^ { r e f } } { \mathbf { R } } _ { k } { \mathbf { L } } _ { k } + { \mathbf { v } } _ { r e f } \Delta t\tag{2b}
$$

where exp denotes the exponential map $s e ( 3 ) {  } S E ( 3 ) . \ ^ { r e f } { \bf R } _ { k }$ is the rotation matrix converted from the Euler angle $\omega _ { k } \Delta t$ $\omega _ { k } = \tilde { \omega } _ { k } - \mathbf { b } _ { g } ( t _ { k } ) - \mathbf { n } _ { g } ( t _ { k } ) . \ \tilde { \omega } _ { k }$ is the measurement of $\mathrm { g y - }$ roscope, while $\mathbf { b } _ { g } ( t _ { k } )$ and ${ \bf n } _ { g } ( t _ { k } )$ are bias and noise variable of gyroscope respectively. $\mathbf { L } _ { k } = \{ x _ { k } , y _ { k } , 1 \}$ is homogeneous matrix that extended from $l _ { k } . \mathbf { v } _ { r e f }$ represents the velocity of our ESVIO back-end at $t _ { r e f }$ timestamp. Finally, we convert ${ \mathit { r e f } } _ { L _ { k } }$ to homogeneous matrix and obtain the compensated location $^ { r e f } l _ { k }$ Fig. 3 compares raw event streams and motion-compensated event streams, where raw event streams produce a certain extent of distortion while the compensated event streams show clear contours of scenes.

## B. Event-Based Spatial and Temporal Data Associations

After the motion compensation, the stereo event streams are fed to generate two (positive and negative) surface-of-activeevent (SAE) which store the event pixels and timestamps. For the new arrival event streams, the existing event-corner features are firstly temporally tracked by the LK optical approach [20] and then spatially matched in left and right event streams. The event-corner features that are not successfully tracked and matched in the current timestamp would be discarded immediately. While new event-corner features are extracted on the motion-compensated event streams to maintain a minimum number (100-200) of features in each timestamp. Modified from the publicly available implementation ofthe $\operatorname { A r c } ^ { * }$ algorithm [21] for event-based corner detection, we extract the event corners on the individual event by leveraging the SAE. We only select those events whose timestamps are within a short interval from that of the current time surface, thereby retaining event-corner features at the dense event streams. To reduce the influence of noisy events, we further apply time surface (TS) with polarity as a mask to filter effective event-corner features, and the TS is converted from the SAE in real time. Meanwhile, TS is also used to distribute adjacent event-corner features uniformly by setting a minimum distance $d _ { \mathrm { m i n } }$ value.

![](images/2023_ESVIO/077aef8ecf0ef85be25232b207b1c944f2ba0a7eedd2be523278960add0fdce7.jpg)  
Fig. 4. Stereo event-corner features: (a) Geometry principle; (b) Temporally and spatially associating the event-corner features on the time surface; (c) Eventcorner features tracking on the event streams.

The geometry principle of temporal and spatial event-based associations is depicted in Fig. 4(a). To ensure that the matched features between the left and right event streams lie along the epipolar line, we instantaneously match stereo-rectified time surfaces for the spatial association. Stereo event-corner features, $\mathcal { F } _ { l } ^ { i }$ and $\mathcal { F } _ { r } ^ { i }$ , are instantaneously matched by forward and inverse LK optical flow between the left and right time surface at the current timestamp i. Our ESVIO executes spatial and temporal association at each frame. Meanwhile, the temporal association also uses forward and inverse optical flow to track event-corner features of left event streams, $\mathcal { F } _ { l } ^ { i }$ and $\mathcal { F } _ { l } ^ { j }$ , at consecutive two timestamps i and $j .$ In Fig. 4(b), red and blue dots represent event-corner features extracted on the left and right time surface at timestamp i respectively, while yellow dots denote the features on the left time surface at the previous timestamp $j .$ Note that the left bottom and right bottom pictures form the spatial event-based association, while the left bottom and left top pictures form the temporal event-based association. Green lines connect matched event-corner features between the left and right time surfaces at the same timestamp, while yellow lines connect tracked features with two consecutive left time surfaces at i and j. Fig. 4(c) and the bottom of Fig. 1 show our proposed method can achieve correct temporal and spatial association in consecutive stereo event-corner features, even when the image-based tracking is failed caused by the motion blur.

Finally, we recover the inverse depth of event-corner features by RANSAC outlier rejection and triangulation. As depicted in Fig. 4(a), the matched corner features of the 3D point $P$ on the imaging plane of left and right event cameras at timestamp i are $p _ { L } ^ { \ i }$ and $p _ { R } ^ { i }$ respectively. The inverse depth of $P$ can be formulated through epipolar geometry and triangulation between $p _ { L } ^ { i }$ and $p _ { R } ^ { i }$ . Similarly, $p _ { L } ^ { i }$ and $p _ { L } ^ { j }$ can also be used to recover the inverse depth. Based on these associated event-corner features, corresponding residual constraints can be constructed to conduct graph-based optimization.

C. The Construction of Event-Based Residual Constraint for the Graph-Based Optimization

The full state vector in the sliding window is defined as

$$
\boldsymbol { \chi } = \left[ \mathbf { x } _ { b _ { 0 } } , . . . , \mathbf { x } _ { b _ { n } } , \mathbf { x } _ { e } ^ { b } , \mathbf { x } _ { c } ^ { b } , \mathbf { \Lambda } _ { \Lambda _ { e s } } , \mathbf { \Lambda } _ { \Lambda _ { e t } } , \mathbf { \Lambda } _ { \Lambda _ { c } } \right]\tag{3a}
$$

$$
\mathbf { x } _ { b _ { k } } = \left[ \mathbf { p } _ { b _ { k } } ^ { w } , \mathbf { q } _ { b _ { k } } ^ { w } , \mathbf { v } _ { b _ { k } } ^ { w } , \mathbf { b } _ { a _ { k } } , \mathbf { b } _ { g _ { k } } \right] \quad k \in [ 0 , n ]\tag{3b}
$$

where $\mathbf { x } _ { b _ { k } }$ is the state of IMU at timestamp $k$ in the world frame, which consists of the position $\mathbf { p } _ { b _ { k } } ^ { w }$ , the orientation quaternion $\mathbf { q } _ { b _ { k } } ^ { w }$ , the velocity ${ \bf v } _ { b _ { k } } ^ { w }$ , the accelerometer bias $\mathbf { b } _ { a _ { k } }$ and the gyroscope bias ${ \bf b } _ { g _ { k } } . ~ { \bf x } _ { e } ^ { b }$ and $\mathbf { x } _ { c } ^ { b }$ are the extrinsic transformation from event cameras and standard cameras to IMU respectively. $\mathbf { \Lambda } _ { e s } = [ \lambda _ { e s _ { 0 } } , \lambda _ { e s _ { 1 } } , . . . , \lambda _ { e s _ { l } } ] , \mathbf { \Lambda } _ { { \bf { \Lambda } } _ { e t } } = [ \lambda _ { e t _ { 0 } } , \lambda _ { e t _ { 1 } } , . . . , \lambda _ { e t _ { l } } ]$ $\lambda _ { e s _ { l } } , \lambda _ { e t _ { l } }$ represent the inverse depth of the event-corner features es<sub>l</sub>, et<sub>l</sub> respectively. $\mathbf { \Lambda } _ { \mathbf { \Lambda } _ { c } } = [ \lambda _ { c _ { 0 } } , \lambda _ { c _ { 1 } } , . . . , \lambda _ { c _ { l } } ] , \lambda _ { c _ { l } }$ is the inverse depth of the image-based features $c _ { l } .$ . n is the total number of keyframes, and l is the total number of features in the sliding window.

Combining event, image, and IMU residual terms, the eventvisual-inertial odometry can be formulated as thejoint nonlinear optimization problem as follow

$$
\begin{array} { l } { \displaystyle \underset { \boldsymbol { x } } { \mathrm { m i n } } \left( \sum _ { k \in { b } } \left\| \mathbf { r } _ { b } ( \hat { \mathbf { z } } _ { b _ { k + 1 } } ^ { b _ { k } } , \boldsymbol { \chi } ) \right\| _ { \Omega _ { b } } ^ { 2 } + \sum _ { ( l , k ) \in { e s } } \left\| \mathbf { r } _ { e s } ( \hat { \mathbf { z } } _ { e s _ { k } } ^ { l } , \boldsymbol { \chi } ) \right\| _ { \Omega _ { e s } } ^ { 2 } \right. } \\ { \displaystyle \left. + \sum _ { ( l , k ) \in e t } \left\| \mathbf { r } _ { e t } ( \hat { \mathbf { z } } _ { e t _ { k } } ^ { l } , \boldsymbol { \chi } ) \right\| _ { \Omega _ { e t } } ^ { 2 } + \sum _ { ( l , k ) \in c } \left\| \mathbf { r } _ { c } ( \hat { \mathbf { z } } _ { c _ { k } } ^ { l } , \boldsymbol { \chi } ) \right\| _ { \Omega _ { c } } ^ { 2 } \right) } \end{array}\tag{4}
$$

where $\mathbf { r } _ { b } ( \hat { \mathbf { z } } _ { b _ { k + 1 } } ^ { b _ { k } } , \pmb { \chi } )$ is the residual for IMU measurement with information matrix $\Omega _ { b } . \ \mathbf { r } _ { e s } ( \hat { \mathbf { z } } _ { e s _ { k } } ^ { l } , \pmb { \chi } )$ and $\mathbf { r } _ { e t } ( \hat { \mathbf { z } } _ { e t _ { k } } ^ { l } , \pmb { \chi } )$ are the residuals for event-based spatial and temporal association measurement, with corresponding information matrix $\Omega _ { e s }$ and $\Omega _ { e t }$ respectively. $\mathbf { r } _ { c } ( \hat { \mathbf { z } } _ { c _ { k } } ^ { l } , \bar { \boldsymbol { x } } )$ represents the residuals for standard cameras measurement with information matrix $\Omega _ { c }$ . The definition of event-based residuals will be presented below, while other residual terms can be found in [4], [22].

For the stereo event cameras, we construct the spatial association factor $\mathbf { r } _ { e s } ( \hat { \mathbf { z } } _ { e s _ { k } } ^ { l } , \pmb { \chi } )$ between the left and right event streams and the temporal association factor $\mathbf { r } _ { e t } ( \hat { \mathbf { z } } _ { e t _ { k } } ^ { l } , \pmb { \chi } )$ in the consecutive event streams. Consider the lth feature that is observed in the ith right event stream, the residual for the event-corner feature observation in the ith left event stream is defined as:

$$
\mathbf { r } _ { e s } = \left[ \begin{array} { l } { u _ { l e s _ { i } } ^ { l } } \\ { v _ { l e s _ { i } } ^ { l } } \end{array} \right] - \pi _ { e } \left( \mathbf { T } _ { r e } ^ { l e } \pi _ { e } ^ { - 1 } \left( \frac { 1 } { \lambda _ { e s } } , \left[ \begin{array} { l } { u _ { r e s _ { i } } ^ { l } } \\ { v _ { r e s _ { i } } ^ { l } } \end{array} \right] \right) \right)\tag{5}
$$

where $[ u _ { l e s _ { i } } ^ { l } , v _ { l e s _ { i } } ^ { l } ]$ is the observation of the lth event-corner feature in the ith left event stream. $[ u _ { r e s _ { i } } ^ { l } , v _ { r e s _ { i } } ^ { l } ]$ is the same event-corner feature in the ith right event stream. $\pi _ { e }$ and $\pi _ { e } ^ { - 1 }$

are the projection and back-projection functions of the event camera respectively. $\mathbf { T } _ { r e } ^ { l e }$ represents the extrinsic transformation from the right to left event camera.

Consider the lth feature that is first observed in the ith left event stream, the residual for the event-corner feature observation in the kth left event stream is defined as:

$$
\begin{array} { l } { { \bf { r } } _ { e t } = \ \left[ \begin{array} { l } { u _ { { e t } _ { k } } ^ { l } } \\ { v _ { { e t } _ { k } } ^ { l } } \end{array} \right] } \\ { - \pi _ { e } \left( ( { \bf { T } } _ { l e } ^ { b } ) ^ { - 1 } { \bf { T } } _ { w } ^ { b _ { k } } { \bf { T } } _ { b _ { i } } ^ { w } { \bf { T } } _ { l e } ^ { b } \pi _ { e } ^ { - 1 } \left( \frac { 1 } { \lambda _ { e t } } , \left[ u _ { { e t } _ { i } } ^ { l } \right] \right) \right) } \end{array}\tag{6}
$$

where $[ u _ { e _ { k } } ^ { l } , v _ { e _ { k } } ^ { l } ]$ is the observation of the lth event-corner feature in the kth event stream. $[ u _ { e _ { i } } ^ { l } , v _ { e _ { i } } ^ { l } ]$ is the same event-corner feature in the ith event stream. $\mathbf { T } _ { l e } ^ { b }$ represents the extrinsic transformation from the left event camera to the body coordinate. $\mathbf { T } _ { b _ { i } } ^ { w }$ indicates the pose of the body center related to the world frame at timestamp $i , \mathbf { T } _ { w } ^ { b _ { k } }$ is the transpose of the pose of the body coordinate in the world frame at the kth keyframe.

## IV. EVALUATION

We perform both dataset and real-world experiments to evaluate our proposed methods. We first evaluate our proposed ESIO and ESVIO in the self-collected dataset which is acquired by two DAVIS346 $( 3 4 6 \times 2 6 0$ , event-sensor, image-sensor, IMU sensor) and VICON. It contains extremely fast 6-Dof motion and scenes with HDR. In Section IV-B, we compare our methods with other event-based and image-based methods on two publicly available datasets: MVSEC [23] and VECtor [24]. We perform quantitative analysis to evaluate the accuracy of our system. The accuracy is measured with mean position error (MPE, %) and mean rotation error (MRE, <sup>◦</sup>/m) aligning the estimated trajectory with ground truth using 6-DOF transformation (in SE3), which is calculated by the tool [25]. Finally, in Section IV-C we evaluate our ESVIO in the onboard quadrotor flighting. While Sections IV-D1 and IV-D2 perform the evaluation of the autonomous-driving dataset and outdoor large-scale environment, respectively. All experiments run in real-time on an Intel NUC computer equipped with Intel i7-1260P, 32 GB RAM, and Ubuntu 20.04 operation system.

## A. Evaluation of Our ESVIO in Challenging Situations

1) Experiment Data Description: The dataset contains stereo event data at 60 Hz and stereo image frames at 30 Hz with resolution in $3 4 6 \times 2 6 0$ , as well as IMU data at 1000 Hz. Timestamps between all sensors are synchronized in hardware. We also provide ground truth poses from a motion capture system VICON at 50 Hz for each sequence, which can be used for accuracy comparison. The dataset consists of handheld sequences including rapid motion and HDR scenarios. The full setup including the attached infrared filter can be seen in Fig. 5. These two DAVIS346 are rigidly attached with a baseline of 6.0 cm and USB 3.0 interfaces are used to transmit sensor measurements to the NUC. However, since the limitation of our hardware and cost, we use DAVIS346-COLOR and DAVIS346- MONO for the data collection. Although this might introduce some artificial inconsistency, we think it is acceptable for the method evaluation. The DAVIS comprises an image camera and event camera on the same pixel array, thus calibration can be done using standard image-based methods, such as Kalibr<sup>1</sup>, on the image frames and then are applied to the event camera. For the benefit ofthe research community, we also release the dataset and the configuration files on our project website.

TABLE I  
THE ACCURACY COMPARISON OF OUR ESVIO WITH OTHER IMAGE-BASED OR EVENT-BASED METHODS ON HKU DATASET
<table><tr><td rowspan="2">Sequence</td><td>ORB-SLAM3 [26] Stereo VIO</td><td>VINS-Fusion [22] Stereo VIO</td><td>USLAM [10] Mono EIO</td><td>USLAM [3] Mono EVIO</td><td>PL-EVIO [4] Mono EVIO</td><td>Our ESIO Stereo EIO</td><td>Our ESIO+ Stereo EIO</td><td>Our ESVIO Stereo EVIO</td></tr><tr><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td></tr><tr><td>hku_agg_translation</td><td>0.15 / 0.075</td><td>0.11 / 0.019</td><td>16.22 / 0.45</td><td>0.59 / 0.020</td><td>0.07 / 0.091</td><td>0.59 / 0.16</td><td>0.55 / 0.16</td><td>0.10 / 0.016</td></tr><tr><td>hku_agg_rotation</td><td>0.35 / 0.11</td><td>1.34 / 0.024</td><td>failed</td><td>3.14 / 0.026</td><td>0.23 / 0.12</td><td>1.33 / 0.048</td><td>0.78 / 0.045</td><td>0.17 / 0.015</td></tr><tr><td>hku_agg_flip</td><td>0.36 / 0.39</td><td>1.16 / 2.02</td><td>11.15 / 2.11</td><td>6.86 / 2.04</td><td>0.39 /  2.23</td><td>3.79 / 0.23</td><td>3.17 / 0.23</td><td>0.36 / 0.12</td></tr><tr><td>hku_agg_walk</td><td>failed</td><td>failed</td><td>failed</td><td>2.00 / 0.16</td><td>0.42 / 0.14</td><td>1.49 / 0.23</td><td>1.30 / 0.23</td><td>0.31  / 0.026</td></tr><tr><td>hku_hdr_circle</td><td>0.17 / 0.12</td><td>5.03 / 0.60</td><td>0.92 / 0.58</td><td>1.32 / 0.54</td><td>0.14 / 0.62</td><td>1.38 / 0.10</td><td>0.46 / 0.099</td><td>0.16 / 0.035</td></tr><tr><td>hku_hdr_slow</td><td>0.16 / 0.058</td><td>0.13  / 0.026</td><td>failed</td><td>2.80 / 0.099</td><td>0.13 / 0.068</td><td>0.29 / 0.38</td><td>0.31  / 0.39</td><td>0.11 / 0.028</td></tr><tr><td>hku_hdr_tran_rota</td><td>0.30 / 0.042</td><td>0.11 / 0.021</td><td>failed</td><td>2.64 / 0.13</td><td>0.10 / 0.064</td><td>0.84 / 0.30</td><td>0.91 / 0.31</td><td>0.10 / 0.018</td></tr><tr><td>hku_hdr_agg</td><td>0.29 / 0.085</td><td>1.21 / 0.27</td><td>failed</td><td>2.47  / 0.27</td><td>0.14 / 0.30</td><td>2.33 / 0.16</td><td>1.41 / 0.14</td><td>0.10 / 0.021</td></tr><tr><td>hku_dark_normal</td><td>failed</td><td>0.86 / 0.028</td><td>failed</td><td>2.17 / 0.031</td><td>1.35 / 0.081</td><td>0.30 / 0.12</td><td>0.35 / 0.12</td><td>0.42 / 0.015</td></tr><tr><td>Average</td><td>0.16 / 0.12</td><td>0.76 / 0.38</td><td>5.06 / 1.05</td><td>1.69 / 0.39</td><td>0.26 / 0.41</td><td>0.89 / 0.19</td><td>0.66 / 0.19</td><td>0.14 / 0.033</td></tr></table>

\*EIO means event-based inertial odometry, EVIO means event-based VIO with image-aided

![](images/2023_ESVIO/412d5322113ddc5ad1b3d9203eef29ce62296e6e2d3d17138a8721869c81ff12.jpg)  
Fig. 5. Our self-designed quadrotor platform.

2) Methods Evaluation: Table I compares the performance of our ESIO and ESVIO with the other state-of-the-art eventbased or image-based systems. Our ESIO has good performance, especially for the sequence hku\_agg\_walk and hku\_dark\_normal, our ESIO still can produce reliable and accurate pose estimation even when the state-of-the-art image-based VIO method, ORB-SLAM3, fails. Due to motion blur, both ORB-SLAM3 and VINS-Fusion fail to extract reliable features in hku\_agg\_walk sequence, resulting in system failure. In the hku\_dark\_normal sequence, ORB-SLAM3 cannot extract any feature due to poor light conditions (more qualitative details is in the supplementary material). As for the MRE evaluation criterion, our ESVIO shows significant improvement compared to other advanced algorithms, e.g. the average MRE of ESVIO is 0.033<sup>◦</sup>/m while the value of ORB-SLAM3 is 0.12<sup>◦</sup>/m. Even if it does not perform too much improvement in MPE compared with our previous work PL-EVIO [4] in most sequences, ESIO and ESVIO are still a breakthrough for event-based stereo VIO. For the motion compensation version (ESIO+), it shows effective improvement in most of the data sequences compared to ESIO, e.g. the average MPE of ESIO+ is 0.66% compared to 0.89% of ESIO, which is opposite to the conclusion from Ref. [4]. This might be thanks to the well-designed motion-compensated methods that process the reliable and optimized compensated measurements from the ESVIO back-end.

Note that we also evaluate EVO [2] and ESVO [5] in our selfcollected datasets, but they failed in all sequences. This might be caused by three factors: Firstly, both EVO and ESVO have strict initialization requirements. For example, EVO requires running in a uniform scene for a few seconds to boost the system. Secondly, they are sensitive to parameter tuning, even in their open-source project, they use different parameters for different sequences in the same scenarios. We might fail to correctly tune parameters for their successful running. Finally, our dataset is so challenging that only reliable methods can perform well.

## B. Evaluation of Our ESVIO on Public Datasets

In this section, we evaluate our ESVIO on publicly available datasets. The VECtor [24] dataset consists of a hardwaresynchronized sensor suite that includes stereo event cameras, stereo standard cameras, an RGB-D sensor, a LiDAR, and an IMU. It covers the full spectrum of 6 DoF motion dynamics, environment complexities, and illumination conditions for both small and large-scale scenarios. To the best of our knowledge, we provide the first results on this new event-based dataset. For the MVSEC [23], we select the sequence captured in the indoor flying room. We use the stereo event camera (640 × 480) and the regular stereo camera (1224 × 1024) from the VECtor, and the DAVIS346 (346 × 260 for both event and image) from the MVSEC, for evaluation, respectively.

As can be seen in Table II, our proposed ESVIO can achieve fairly good results in most of the sequences. Although the MPE criterion of ORB-SLAM3 is slightly better than ours in some sequences (e.g. robot-normal, desk-normal, mountain-normal), our ESVIO provides more reliable and accurate results in most of the sequences under harsh situations with HDR or aggressive motion. The proposed ESVIO is more precise than our previous PL-EVIO, especially in large-scale environments, this might be due to the better event-corner depth estimation. While the traditional event-based methods [2], [3], [5] failed in most of the sequences in these two datasets.

Please note that we think that parameter tuning is infeasible. Therefore, we evaluate our methods using fixed parameters for all sequences during the evaluations. However, the generalization capability of [2] and [5] is slightly poor. Although we have put the utmost effort to tune parameters, they fail in most sequences. Besides, we emphasize real-time performance when evaluating our methods. Table III illustrates the running time of our modules under different resolutions. We also provide a qualitative comparison between our method and the other methods in the accompanying video<sup>2</sup> and supplementary material.

TABLE II  
THE ACCURACY COMPARISON OF OUR ESVIO WITH OTHER IMAGE-BASED OR EVENT-BASED METHODS ON PUBLIC DATASET
<table><tr><td colspan="2">Sequence</td><td>ORB-SLAM3 [26] Stereo VIO</td><td>VINS-Fusion [22] Stereo VIO</td><td>EVO [2] Mono EO</td><td>ESVO [5] Stereo EO</td><td>Ultimate SLAM [3] Mono EVIO</td><td>PL-EVIO [4] Mono EVIO</td><td>Our ESVIO Stereo EVIO</td></tr><tr><td colspan="2"></td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td><td>MPE / MRE</td></tr><tr><td rowspan="10">VECtor [24]</td><td>corner-slow robot-normal</td><td>1.49 /  14.28 0.73 / 1.18</td><td>1.61  /  14.06 0.58 / 1.18</td><td>4.33 / 15.52 3.25 / 2.00</td><td>4.83 / 20.98 failed</td><td>4.83 / 14.42 1.18 / 1.11</td><td>2.10 / 14.21 0.68 / 1.25</td><td>1.49 / 14.03</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>1.08 / 1.17</td></tr><tr><td>robot-fast</td><td>0.71 / 0.70</td><td>failed</td><td>failed</td><td>failed</td><td>1.65 / 0.56</td><td>0.17 / 0.74</td><td>0.20  / 0.56</td></tr><tr><td>desk-normal</td><td>0.46 / 0.41</td><td>0.47  / 0.36</td><td>failed</td><td>failed</td><td>2.24 / 0.56</td><td>3.66 / 0.45</td><td>0.61  / 0.38</td></tr><tr><td>desk-fast</td><td>0.31  / 0.41</td><td>0.32 / 0.33</td><td>failed</td><td>failed</td><td>1.08 / 0.38</td><td>0.14 /  0.48</td><td>0.13 / 0.32</td></tr><tr><td>sofa-normal</td><td>0.15 / 0.41</td><td>0.13 / 0.40</td><td>failed</td><td>1.77 / 0.60</td><td>5.74  / 0.39</td><td>0.19 / 0.46</td><td>0.16 / 0.40</td></tr><tr><td>sofa-fast</td><td>0.21  / 0.43</td><td>0.57  / 0.34</td><td>failed</td><td>failed</td><td>2.54 / 0.36</td><td>0.17 / 0.47</td><td>0.17 / 0.35</td></tr><tr><td>mountain-normal</td><td>0.35 /  1.00</td><td>4.05 / 1.05</td><td>failed</td><td>failed</td><td>3.64 / 1.06</td><td>4.32 / 0.76</td><td>0.59 / 0.77</td></tr><tr><td>mountain-fast</td><td>2.11 / 0.64</td><td>failed</td><td>failed</td><td>failed</td><td>4.13 / 0.62</td><td>0.13 / 0.56</td><td>0.16 / 0.45</td></tr><tr><td>hdr-normal hdr-fast</td><td>0.64 /  1.20 0.22 / 0.45</td><td>1.27 / 1.10 0.30 / 0.34</td><td>failed failed</td><td>failed failed</td><td>5.69 / 1.65 2.61 / 0.34</td><td>4.02 / 1.52</td><td>0.57  /  1.06 0.21  / 0.33</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td>0.20 / 0.50</td><td></td></tr><tr><td>corridors-dolly</td><td>1.03 / 1.37 1.32 / 1.31</td><td>1.88 / 1.37 0.50 / 1.31</td><td>failed</td><td>failed</td><td>failed</td><td>1.58 /  1.37</td><td>1.13 / 1.33</td></tr><tr><td>corridors-walk school-dolly</td><td>0.73 /  1.02</td><td>1.42 /  1.06</td><td>failed</td><td>failed 10.87 / 1.08</td><td>failed failed</td><td>0.92 /  1.31</td><td>0.43 / 1.32 0.42 / 0.73</td></tr><tr><td>school-scooter</td><td>0.70 / 0.49</td><td>0.52 / 0.61</td><td>failed failed</td><td>9.21 / 0.63</td><td>6.40 / 0.61</td><td>2.47 / 0.97 1.30 / 0.54</td><td>0.59 / 0.56</td></tr><tr><td>units-dolly</td><td>7.64 / 0.41</td><td>4.39 / 0.42</td><td>failed</td><td>failed</td><td>failed</td><td>5.84 / 0.44</td><td>3.43  / 0.022</td></tr><tr><td>units-scooter</td><td>6.22 / 0.22</td><td>4.92 / 0.24</td><td>failed</td><td>failed</td><td>failed</td><td>5.00 / 0.42</td><td>2.85 / 0.39</td></tr><tr><td rowspan="4">MVSEC [23]</td><td>Indoor Flying 1</td><td>5.31  / 0.37</td><td>1.50 / 0.13</td><td>5.09 / 0.92</td><td>4.00 / 0.50</td><td>failed</td><td>1.35 / 0.11</td><td>0.94 / 0.14</td></tr><tr><td>Indoor Flying 2</td><td>5.65 / 0.41</td><td>6.98 / 0.15</td><td>failed</td><td>3.66 / 0.43</td><td>failed</td><td>1.00 / 0.16</td><td>1.00 / 0.11</td></tr><tr><td>Indoor Flying 3</td><td>2.90 / 0.30</td><td>0.73 / 0.048</td><td>2.58 /  1.25</td><td>1.71 / 0.18</td><td>failed</td><td>0.64 / 0.065</td><td>0.47  / 0.043</td></tr><tr><td>Indoor Flying 4</td><td>6.99 / 0.79</td><td>3.62 / 0.39</td><td>failed</td><td>failed</td><td>2.77 / 0.14</td><td>5.31  / 0.23</td><td>5.55 / 0.21</td></tr></table>

The bold values highlight the best accuracy result among the evaluated algorithms in the sequence

TABLE III  
RUNNING TIME OF OUR MODULES IN DIFFERENT RESOLUTION EVENT CAMERAS (MS)
<table><tr><td>Modules</td><td>346 × 260</td><td>640 × 480</td></tr><tr><td>Motion compensation</td><td>2.70</td><td>11.11</td></tr><tr><td>Creation of event representation</td><td>5.12</td><td>15.47</td></tr><tr><td>Spatial event association</td><td>0.82</td><td>2.57</td></tr><tr><td>Temporal event association</td><td>0.83</td><td>3.31</td></tr><tr><td>The whole process of front-end</td><td>10.44</td><td>35.69</td></tr><tr><td>Back-end optimization</td><td>19.30</td><td>35.59</td></tr></table>

Last but not the least, although our proposed ESVIO achieves satisfactory results, it still has limitations in the low-texture environment. For example, the scenarios in sequence units-dolly and units-scooter are so special that the visual-only method might be easy to degenerate or mismatch during the loop-closure detection. This also indicates that either the event camera or the standard camera has limitations. Although event cameras play a complementary role to the traditional image-based method, event-based multi-sensor fusion, especially combining nonvision-based sensors (such as GPS, and lidar), should be further developed to exploit the advantage of different sensors.

## C. Indoor Quadrotor Flight Evaluation

To further demonstrate the practicability of our methods, we perform real-world experiments on a self-designed quadrotor platform. We choose Pixracer autopilot as our flight platform with T-Motor F90 PRO, as shown in Fig. 5. The states estimate from our ESVIO is used to provide onboard pose feedback control for the quadrotor. The quadrotor is commanded to follow a circular pattern eight times continuously during the experiment. The robust and accurate onboard state estimates of our ESVIO enable real-time feedback control. Meanwhile, we also record the ground truth from VICON for further quantitative evaluation.

1) Quadrotor Flight in HDR Scenarios <sup>3</sup>: We show the relative pose error (RPE) of our ESVIO against the VICON in Fig. 6(a). The total trajectory length is 56.0 m. The boxplot [27] shows that the average relative error for the translation part is around 0.1 m. For the rotational part, the average relative error is around 7<sup>◦</sup>. While there are many outliers from 50 to 60 seconds, which is caused by rapid change in yaw at that moment resulting in the estimated pose being slightly slower than VICON. The root-mean-square error (RMSE) of absolute trajectory error in HDR flight is 0.17 m. Fig. 1 qualitatively evaluates the moment when the yaw angle changes rapidly, where few features can be extracted and tracked by the image thread, while event-corner features can still be tracked well.

2) Quadrotor Flight in Aggressive Motion <sup>4</sup>: In this section, the yaw angle of the commanded pattern is changed drastically, for aggressive motion. The performance of our ESVIO is qualitatively demonstrated in Fig. 1 and quantitatively evaluated in Fig. 6(b). The RMSE of ATE in aggressive flight is 0.26 m. Note that it would have some outliers during the comparison with the VICON. For example, there are some rotation errors of more than 20<sup>◦</sup> within 20-30 seconds. This is caused by VICON’s ball is not well observed during the aggressive flight, resulting in an inaccurate measurement of the VICON at that moment. However, our reliable ESVIO state estimator still provides robust and accurate onboard pose feedback for the quadrotor.

zurich\_city\_04\_c  
zurich\_city\_04\_b  
![](images/2023_ESVIO/dca187f65a0eae623d7a55fe24aec0d97edfaa01a104bf9bafdcead60760c0c1.jpg)  
(a)

![](images/2023_ESVIO/1980c00790fa59f554136b3b859a1c89d81224ae98df54ba18d82797109c34e3.jpg)  
(b)  
Fig. 6. The relative error comparison of our proposed ESVIO with the VI-CON: (a) Onboard quadrotor flight in low-illumination conditions; (b) Onboard quadrotor flight in aggressive motion.

## D. Outdoor Large-Scale Evaluation

In this section, we evaluate our ESIO and ESVIO in outdoor large-scale environments, including the public-available autonomous driving dataset and the self-collected HKU campus dataset (More details can be seen on our website).

1) DSECDataset: DSEC [28] is collected by high-resolution stereo event cameras (640 × 480) under driving scenarios, which is challenging for event-based sensors, as forward motions typically produce considerably fewer events at the center. Qualitative evaluation can be seen in Fig. 7. Since the DSEC dataset does not provide the ground truth 6-DoF poses, we only show the estimated trajectory and the tracking performance of our event-based and image-based features. Both our ESIO (available in supplementary material) and ESVIO can achieve satisfactory results.

2) HKU Large-Scale Environment: This section carried out a large-scale experiment on the HKU campus to illustrate the long-time practicability of our ESVIO, features with large-scale, indoor-outdoor conversion, pedestrians in the scene generating outlier events, etc. The path length of the outdoor evaluation is around 1.8 km and the duration is 34.9 minutes. The evaluation covers the place around 310 m in length, 170 m in width, and

![](images/2023_ESVIO/52491f3b14ddb9739ed1c7147c0139b3f9deb25e37b206cf312daaf54141eb21.jpg)

![](images/2023_ESVIO/05b96a005f742b69ac3fd03273d27fd30f5d30af5f700fd54a2fb231ea724a8a.jpg)

![](images/2023_ESVIO/054390ff1b855690add65d230b3f198094bcf935dc91e7036e075a62040222e6.jpg)

![](images/2023_ESVIO/5ecc63864c716a6852b21166fc6132d6f5ec53aed4694c0817e1a45c13cf3a46.jpg)

![](images/2023_ESVIO/245cc9dc8e9c508c48bdc41b85f6ca331fef670f0c44a2228a246d0a85bdecef.jpg)

![](images/2023_ESVIO/4a116208060f84dc86cfb5184c645f3d229383debc5fec5d3be3ec66855c1a86.jpg)

![](images/2023_ESVIO/2f3b1012be15185356632e32455476b5379057935ee3d75b0e4c03bf18972652.jpg)  
zurich\_city\_04\_a

![](images/2023_ESVIO/0d922f6ca9682685d4f7ec63ec22581bc2c042775f65dfebfa1c4683df3d3ed4.jpg)

![](images/2023_ESVIO/aa6f1dc90eaacff48d0a68b9dbef0c9c18ed940448beaa438a4a93b73c3b6d83.jpg)  
Fig. 7. The qualitative results of ESVIO in DSEC dataset for sequences zurich\_city\_04 (a) to (c). Top: The stereo event-corner feature tracking performance; Middle: The stereo image-based feature tracking performance; Bottom: The estimated trajectories produced by our ESVIO.

![](images/2023_ESVIO/8784f86e4024d616371bcd0e9ae769d04bfb9070f0263a25cfd7ff625534ddd7.jpg)  
Fig. 8. The estimated trajectory of our ESVIO in the large-scale environment. The detection and tracking situation ofthe stereo event-corner features and stereo image features during the experiment are also visualized.

55 m in height changes. Since the VICON is not available outdoors, we only show the qualitative performance and the estimated trajectory overlaid with the Google map for visual comparison. As can be seen from Fig. 8, our ESVIO performs well in long-term motion evaluation, the estimated trajectory is aligned and almost coincide with the Google map.

## V. CONCLUSION

In this letter, we propose a robust, real-time event-based stereo VIO, ESVIO, which tightly fuses the stereo event streams, stereo image frames, and IMU measurements using sliding windows graph-based optimization. Geometry-based spatial and temporal associations between consecutive stereo event streams are designed to ensure robust state estimation. In addition, the motion compensation approach corrects the curved event streams with IMU and ESVIO back-end to emphasize the contour of scenes. Extensive evaluations demonstrate that our ESVIO achieves superior performance compared to other state-of-the-art algorithms on public datasets and our self-collected challenging datasets. Furthermore, we also perform various onboard closedloop flights using the proposed ESVIO under low-light scenes and aggressive motion. In our future work, we might further explore the event-based mapping for the quadrotor system which should be able to support autonomous navigation and obstacle avoidance.

## MULTIMEDIA MATERIAL

WEBSITE [Online]. Available: https://github.com/arclabhku/Event\_based\_VO-VIO-SLAM.

Video Demo [Online]. Available: https://b23.tv/V23SVzC. Supplementary Material [Online]. Available: https://github. com/arclab-hku/Event\_based\_VO-VIO-SLAM/tree/main/ES VIO/supply.

## REFERENCES

[1] G. Gallego et al., “Event-based vision: A survey,” IEEE Trans. Pattern Anal. Mach. Intell., Vol. 44, no. 1, pp. 154–180, Jan. 2022.

[2] H. Rebecq, T. Horstschäfer, G. Gallego, and D. Scaramuzza, “EVO: A geometric approach to event-based 6-DOF parallel tracking and mapping in real time,” IEEE Robot. Automat. Lett., vol. 2, no. 2, pp. 593–600, Apr. 2017.

[3] A. R. Vidal, H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Ultimate SLAM? combining events, images, and IMU for robust visual SLAM in HDR and high-speed scenarios,” IEEE Robot. Automat. Lett., vol. 3, no. 2, pp. 994–1001, Apr. 2018.

[4] W. Guan, P. Chen, Y. Xie, and P. Lu, “PL-EVIO: Robust monocular event-based visual inertial odometry with point and line features,” 2022, arXiv:2209.12160.

[5] Y. Zhou, G. Gallego, and S. Shen, “Event-based stereo visual odometry,” IEEE Trans. Robot., vol. 37, no. 5, pp. 1433–1450, Oct. 2021.

[6] A. Hadviger, I. Cviši´c, I. Markovi´c, S. Vraži´c, and I. Petrovi´c, “Featurebased event stereo visual odometry,” in Proc. Eur. Conf. Mobile Robots, 2021, pp. 1–6.

[7] B. Kueng, E. Mueggler, G. Gallego, and D. Scaramuzza, “Low-latency visual odometry using event-based feature tracks,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2016, pp. 16–23.

[8] H. Rebecq, G. Gallego, E. Mueggler, and D. Scaramuzza, “EMVS: Eventbased multi-view stereo–3D reconstruction with an event camera in realtime,” Int. J. Comput. Vis., vol. 126, no. 12, pp. 1394–1414, 2018.

[9] A. Zihao Zhu, N. Atanasov, and K. Daniilidis, “Event-based visual inertial odometry,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2017, pp. 5391–5399.

[10] H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Real-time visualinertial odometry for event cameras using keyframe-based nonlinear optimization,” in Proc. Brit. Mach. Vis. Conf., 2017, pp. 16–1.

[11] E. Mueggler, G. Gallego, H. Rebecq, and D. Scaramuzza, “Continuoustime visual-inertial odometry for event cameras,” IEEE Trans. Robot., vol. 34, no. 6, pp. 1425–1440, Dec. 2018.

[12] Y. Zuo, J. Yang, J. Chen, X. Wang, Y. Wang, and L. Kneip, “DEVO: Depth-event camera visual odometry in challenging conditions,” in Proc. Int. Conf. Robot. Automat., 2022, pp. 2179–2185.

[13] F. Mahlknecht et al., “Exploring event camera-based odometry for planetary robots,” IEEE Robot. Automat. Lett., vol. 7, no. 4, pp. 8651–8658, Oct. 2022.

[14] D. Gehrig, H. Rebecq, G. Gallego, and D. Scaramuzza, “EKLT: Asynchronous photometric feature tracking using events and frames,” Int. J. Comput. Vis., vol. 128, no. 3, pp. 601–618, 2020.

[15] W. Guan and P. Lu, “Monocular event visual inertial odometry based on event-corner using sliding windows graph-based optimization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2022, pp. 2438–2445.

[16] S. Tulyakov, F. Fleuret, M. Kiefel, P. Gehler, and M. Hirsch, “Learning an event sequence embedding for dense event-based deep stereo,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2019, pp. 1527–1537.

[17] Y. Nam, M. Mostafavi, K.-J. Yoon, and J. Choi, “Stereo depth from events cameras: Concentrate and focus on the future,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 6114–6123.

[18] D. Falanga, K. Kleber, and D. Scaramuzza, “Dynamic obstacle avoidance for quadrotors with event cameras,” Sci. Robot., vol. 5, no. 40, 2020, Art. no. eaaz9712.

[19] B. He et al., “FAST-dynamic-vision: Detection and tracking dynamic objects with event and depth sensing,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 3071–3078.

[20] B. D. Lucas and T. Kanade, “An iterative image registration technique with an application to stereo vision,” in Proc. 7th Int/ Joint Conf. Artif. Intell., pp. 674–679, vol. 2, 1981.

[21] I. Alzugaray and M. Chli, “Asynchronous corner detection and tracking for event cameras in real time,” IEEE Robot. Automat. Lett., vol. 3, no. 4, pp. 3177–3184, Oct. 2018.

[22] T. Qin, J. Pan, S. Cao, and S. Shen, “A general optimization-based framework for local odometry estimation with multiple sensors,” 2019, arXiv:1901.03638.

[23] A. Z. Zhu, D. Thakur, T. Özaslan, B. Pfrommer, V. Kumar, and K. Daniilidis, “The multivehicle stereo event camera dataset: An event camera dataset for 3D perception,” IEEE Robot. Automat. Lett., vol. 3, no. 3, pp. 2032–2039, Jul. 2018.

[24] L. Gao et al., “VECtor: A versatile event-centric benchmark for multisensor SLAM,” IEEE Robot. Automat. Lett., vol. 7, no. 3, pp. 8217–8224, Jul. 2022.

[25] M. Grupp, “EVO: Python package for the evaluation of odometry and SLAM,” 2017. [Online]. Available: https://github.com/MichaelGrupp /evo

[26] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. Montiel, and J. D. Tars, “ORB-SLAM3: An accurate open-source library for visual, visual– inertial, and multimap SLAM,” IEEE Trans. Robot., vol. 37, no. 6, pp. 1874–1890, Dec. 2021.

[27] Z. Zhang and D. Scaramuzza, “A tutorial on quantitative trajectory evaluation for visual(-inertial) odometry,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 7244–7251.

[28] M. Gehrig, W. Aarents, D. Gehrig, and D. Scaramuzza, “DSEC: A stereo event camera dataset for driving scenarios,” IEEE Robot. Automat. Lett., vol. 6, no. 3, pp. 4947–4954, Jul. 2021.