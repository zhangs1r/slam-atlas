# Exploring Event Camera-Based Odometry for Planetary Robots

Florian Mahlknecht , Daniel Gehrig , Member, IEEE, Jeremy Nash ,

Friedrich M. Rockenbauer , Graduate Student Member, IEEE, Benjamin Morrell , Member, IEEE, Jeff Delaune , and Davide Scaramuzza , Senior Member, IEEE

Abstract—Due to their resilience to motion blur and high robustness in low-light and high dynamic range conditions, event cameras are poised to become enabling sensors for vision-based exploration on future Mars helicopter missions. However, existing event-based visual-inertial odometry (VIO) algorithms either suffer from high tracking errors or are brittle, since they cannot cope with significant depth uncertainties caused by an unforeseen loss of tracking or other effects. In this work, we introduce EKLT-VIO, which addresses both limitations by combining a state-of-the-art event-based frontend with a filter-based backend. This makes it both accurate and robust to uncertainties, outperforming eventand frame-based VIO algorithms on challenging benchmarks by 32%. In addition, we demonstrate accurate performance in hoverlike conditions (outperforming existing event-based methods) as well as high robustness in newly collected Mars-like and highdynamic-range sequences, where existing frame-based methods fail. In doing so, we show that event-based VIO is the way forward for vision-based exploration on Mars.

Index Terms—Vision-based navigation, space robotics and automation, visual-inertial SLAM.

## MULTIMEDIA MATERIAL

For code and dataset please visit https://uzh-rpg.github.io/ eklt-vio/.

## I. INTRODUCTION

TATE estimation is critical for enabling autonomous nav-S igation and control of mobile robots, with widespread applications from space exploration to household cleaning robots.

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/533bce17c4ff7f48e554b44397c73f976432a012f27ae1c6b6c420e2b33e4f82.jpg)

(a) Mission scenario  
![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/c476c349629951e75aad384da2f4dab068d885f0dded940131ddf5db6de346b4.jpg)

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/c2f8af6f23e434dfa89df81e8a9120e5067ebaf7c2f5134fb142b6e66100340c.jpg)  
(b) Ingenuity Mars Helicopter  
(c) Lava tube  
Fig. 1. New mission scenario (a) enabled by EKLT-VIO for a Mars helicopter (b) scouting the entrance of lava tubes (c).

There exist well-established algorithms, such as [1]–[4] which estimate ego-motion from visual-inertial data. However, visionbased navigation is drastically impacted by the known limitations of conventional cameras, such as motion blur and low dynamic range.

Event cameras promise to address these limitations [5]. Unlike a standard camera that measures absolute pixel brightness using a global exposure time, event camera pixels independently detect positive or negative brightness changes at microsecond resolution. Event cameras can provide data at 1 MHz and 120 dB dynamic range, both orders of magnitude greater than what can be achieved with a standard 60 dB camera. This leads to a significant reduction in motion blur, and enables operation in high dynamic range (HDR), low light, and fast motion conditions [6], [7].

On the application side, computer vision is increasingly used in modern planetary robotic missions [8]–[12].The resilient properties of event cameras may enable robots to explore in conditions where frame cameras cannot operate without introducing the size, weight, power, and range limitations of a 3D LiDAR.

In this letter, we focus on a scenario involving the exploration of the entrance of a lava tube by a Mars helicopter, as illustrated in Fig. 1. Lava tubes are natural tunnels created by lava flows in volcanic terrains. Those found on Mars have drawn significant attention because ofthe possibility that they might host microbial life [13]. The natural protection from radiation offered by lava tubes also makes them candidates to host the first human base on Mars.

Before sending a robotic mission [14] or astronauts to a specific lava tube, it would be desirable to scout and map several locations. Mars helicopters are candidate platforms to scout multiple lava tubes throughout a single mission. However, Mars helicopters cannot fly LiDARs and have to rely on passive cameras for navigation. Frame cameras are ill-suited to explore lava tubes because of the HDR conditions created by the shadow at the entrance of the tube, as well as the low-light conditions once inside. This capability gap is filled by event cameras, which offer the potential to explore and map the lava tube for potentially tens of meters using residual light from the entrance.

Mars helicopters come with their own requirements on the state estimation system [10], [15]. They must rely on small passive lightweight cameras to observe the full state up to scale and gravity direction. The camera is fused with an inertial measurement unit (IMU), which makes gravity observable, enables a high estimation rate, and acts as an emergency landing sensor in case of camera failure. Finally, a laser range finder is used to observe scale in the absence of accelerometer excitation. The estimation backend must be able to handle feature depth uncertainty associated with helicopter hovering and rotation-only dynamics. Due to this uncertainty successful feature triangulation is often inhibited in these cases, leading to failure of optimization-based backends, which critically rely on triangulated features. By contrast, filter-based approaches leverage priors to initialize depth measurements and thus do not suffer from this issue [16]. This proved critical in Ingenuity Mars helicopter’s sixth flight on Mars, where an image timestamping anomaly caused roll and pitch oscillations greater than 20 degrees [17]. Such rotations cause a loss of features, which can lead to estimation failure in non-filter-based state estimation approaches, which are fundamentally unable to handle the depth uncertainty of the new feature tracks without a dedicated re-initialization procedure.

State-of-the-art event-based VIO methods are unsuitable in these conditions since they either (i) use optimization-based backends, which do not model depth uncertainty, thus featuring brittle performance in mission-typical rotation-only motion, or when a significant portion of features are lost [6], or (ii) show a higher tracking error, due to the use of suboptimal event-based frontends [18]. Image-based VIO methods such as [15], [19] have addressed this by using depth priors [15] or motion classification [19].

In this work, we introduce EKLT-VIO, which builds on the EKF backend in [15] which handles pure rotational motion, and combines it with the state-of-the-art event-based feature tracker EKLT [20], thereby addressing the limitations above. EKLT-VIO is accurate, outperforming previous state-of-theart frame-based and event-based methods on the challenging Event-Camera Dataset [21], with a 32% improvement in terms of pose accuracy. Moreover, by leveraging depth uncertainty it reduces its reliance on triangulating features, which both increase robustness during purely-rotational motion, and facilitates rapid initialization, both ofwhich are limitations ofexisting optimization-based methods. This is because they require lengthy bootstrapping sequences, which would be impractical on Mars. Additionally, it maintains state-estimate, even when frame-based methods fail due to excessive motion blur. We show that our event-based EKLT frontend has a higher tracking performance than existing methods on newly collected data in Mars-like conditions. This demonstrates the viability of our EKLT-VIO on Mars. Our contributions are:

We introduce EKLT-VIO, an event-based VIO method that combines an accurate state-of-the-art event-based feature tracker EKLT with an EKF backend. It outperforms stateof-the-art event- and frame-based methods, reducing the overall tracking error by 32%.

We show accurate and robust tracking even in rotationonly sequences, which are closest to the hover-like scenarios experienced by Mars helicopters, outperforming optimization-based and frame-based methods.

\- We outperform existing methods on newly collected Marslike sequences collected in the JPL Mars Yard and Wells Cave for planetary exploration.

## II. RELATED WORK

Frame-based VIO: An overview of existing approaches is discussed in [22]. Frame-based VIO algorithms can be roughly segmented into two classes: optimization-based and filter-based algorithms [22]. While both algorithms focus on tracking camera poses by minimizing both visual and inertial residuals, optimization-based methods solve this by performing iterative Gauss-Newton steps, while filtering-based methods achieve this through Kalman Filtering steps.

Since optimizing both 3D landmarks (i.e., SLAM features) and camera poses is costly, several filtering-based techniques exist that focus on refining camera poses from bearing measurements (i.e., multi-state constraint Kalman filter (MSCKF) features [23]) directly. However, MSCKF features need translational motion and provide updates only after the full feature track is known. The filtering-based approach, xVIO [15], combines the advantages of both features, with robustness to depth uncertainty in rotation-only motion and computational efficiency with many MSCKF features.

Event-based VIO: First event-based, 6-DOF visual odometry (VO) algorithms only started to appear recently [24], [25]. Later work incorporated an IMU to improve tracking performance and stability [18], [26], achieving impressive tracking on a fast spinning leash [26]. Despite their robustness, these methods are affected by drift due to the differential nature of the used sensors. This is why Ultimate SLAM (USLAM) [6] used a combination of events, frames, and IMU, all provided by the Dynamic and Active Vision Sensor (DAVIS) [27]. It tracks FAST corners [28] on frames and motion-compensated event frames separately using the Lucas-Kanade tracker (KLT) [29] and fuses these feature tracks with IMU measurements in a sliding window.

While addressing drift, USLAM still relies on a sliding window optimization scheme, which is expensive and does not allow pose-only optimization through the use of MSCKF features. Moreover, its FAST/KLT frontend, first introduced in [26], is optimized explicitly for frame-like inputs and was shown to transfer suboptimally to event-based frames [20]. In this work, we incorporate the state-of-the-art event-based tracker EKLT [20], which takes a more principled approach to fusing events and frames, and thus achieves better feature tracking performance compared to [6], [26].

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/2ca998c97c6b5d88b82110eecbdeb836b70957cb3a03732f077e3f169946601e.jpg)  
Fig. 2. We combine the feature tracker EKLT, which use frames and events, with the filter-based backend xVIO to enable low-translation state-estimation. In contrast to standard, frame-based VIO, an additional synchronization step converts asynchronous tracks to synchronous matches, which are used by the backend. This enables variable-rate backend updates.

## III. METHODOLOGY

In this section we present EKLT-VIO, which is illustrated in Fig. 2. It is an event-based VIO algorithm based on the stateof-the-art event tracker EKLT, coupled with a filter-based $\mathrm { { x V I O } }$ backend.

## A. Backend

We start by providing a summary of the xVIO backend. For more details see [15]. The backend fuses data from an inertial measurement unit (IMU) and feature tracks from the frontend. It does this by using an extended Kalman filter (EKF) with an IMU state $\mathbf { x } _ { I }$ and a visual state $\mathbf { x } _ { V } \mathbf { : }$

$$
\mathbf { x } = [ \mathbf { x } _ { I } ^ { \mathsf { T } } \quad \mathbf { x } _ { V } ^ { \mathsf { T } } ] ^ { \mathsf { T } }\tag{1}
$$

The IMU state follows an inertial propagation scheme as described in [30]. The visual state $\mathbf { x } _ { V }$ is split into sliding window states $\mathbf { x } _ { S }$ and feature states $\mathbf { x } _ { F }$ :

$$
\begin{array} { r } { { \bf x } _ { V } = [ { \bf x } _ { F } ^ { _ { \mathsf { T } } } { \bf x } _ { S } ^ { _ { \mathsf { T } } } ] ^ { \intercal } , { \bf x } _ { F } = [ { \bf f } _ { 1 } \quad \dots \quad { \bf f } _ { N } ] ^ { \intercal } } \end{array}\tag{2}
$$

$$
\begin{array} { r } { \mathbf { x } _ { S } = [ \mathbf { p } _ { w } ^ { c _ { 1 } \mathsf { T } } \quad \hdots \quad \mathbf { p } _ { w } ^ { c _ { M } \mathsf { T } } \quad \mathbf { q } _ { w } ^ { c _ { 1 } \mathsf { T } } \hdots \quad \mathbf { q } _ { w } ^ { c _ { M } \mathsf { T } } ] ^ { \mathsf { T } } } \end{array}\tag{3}
$$

The sliding window states contain the positions, $\mathbf { p } _ { w } ^ { c _ { i } }$ , and attitudes parameterized as quaternions, $\mathbf { q } _ { w } ^ { c _ { i } }$ , ofthe last M camera poses $\left\{ c _ { i } \right\}$ with respect to a world frame $\{ w \}$ . The feature states contain the 3D positions, f<sub>j</sub>, of N SLAM features. In this work $N = 1 5$ and $M = 1 0$

We use a discrete-time VIO approach, as opposed to one based on splines [31]–[34]. Although they can incorporate eventdata [31] more elegantly they are notoriously computationally expensive [31] and less established. This is why we opt for discrete-time VIO and leave splines for future work.

SLAM features are parametrized with respect to an anchor pose ${ \bf p } _ { w } ^ { c _ { a _ { j } } }$ in the sliding window, and defined as $\mathbf { f } _ { j } ~ =$ $\left[ \alpha _ { j } \beta _ { j } \quad \rho _ { j } \right]$ with $\alpha _ { j }$ and $\beta _ { j }$ being normalized image coordinates and $\rho _ { j }$ being the inverse depth. Each time the feature tracks are updated, each SLAM feature $j$ is converted from inverse-depth to Cartesian coordinates in the associated anchor camera frame $\{ c _ { a _ { j } } \}$

$$
\mathbf { p } _ { c _ { i } } ^ { j } = \mathbf { C } ( \mathbf { q } _ { w } ^ { c _ { i } } ) \left( \mathbf { p } _ { w } ^ { c _ { a _ { j } } } + \frac { 1 } { \rho _ { j } } \mathbf { C } ( \mathbf { q } _ { w } ^ { c _ { a _ { j } } } ) ^ { \top } \left[ \begin{array} { l } { \alpha _ { j } } \\ { \beta _ { j } } \\ { 1 } \end{array} \right] - \mathbf { p } _ { w } ^ { c _ { i } } \right) ,\tag{4}
$$

The measurement model is the normalized feature:

$$
\mathbf { z } _ { j } = \pi ( \mathbf { p } _ { c _ { i } } ^ { j } ) + \mathbf { n } _ { j } , \quad \pi ( \mathbf { x } ) = \left[ x _ { 1 } / x _ { 3 } \quad x _ { 2 } / x _ { 3 } \right] ^ { T } ,\tag{5}
$$

where $\pi ( \mathbf { x } )$ performs feature projection, $\mathbf { n } _ { j }$ is Gaussian noise, and $\mathbf { z } _ { j }$ are the new feature observations by the frontend, expressed in normalized image coordinates. Eqs. (4) and (5) can be used to develop the EKF update by linearizing the SLAM feature reprojection. Details are given in [15].

In addition to SLAM features, the backend maintains MSCKF features that additionally constrain the camera poses without an explicit inverse depth. MSCKF features are thus not part of the state, resulting in a smaller computational cost per feature. They need to be observed for the last $2 \leq m \leq M$ frames, providing a corresponding observation for each pose in the sliding window. MSCKF features require triangulation using those pose priors, so they can only be processed once a track with significant translation is observed. Successfully triangulated MSCKF features are used to initialize SLAM features. When there is insufficient translation for triangulation, xVIO instead initializes the inverse depth with $\begin{array} { r } { \rho _ { 0 } = \frac { 1 } { 2 d _ { \mathrm { m i n } } } } \end{array}$ and uncertainty $\begin{array} { r } { \sigma _ { 0 } = \frac { 1 } { 4 d _ { \mathrm { m i n } } } } \end{array}$ , corresponding to a semi-infinite depth prior, and discards the MSCKF feature track [35]. This depth prior is especially useful during pure rotation or initialization, where few features can be triangulated, since it can directly contribute to reducing the state covariance.

## B. Frontend

Here we provide a summary of our EKLT frontend, and refer the reader to [20] for more details. EKLT tracks Harris corners, extracted on frames, by aligning the predicted and measured brightness increment in a patch around the corners. It minimizes the normalized distance between these patches to recover the warping parameters p and normalize optical flow v as

$$
\left\{ \mathbf { p } , \mathbf { v } \right\} = \underset { \mathbf { p } , \mathbf { v } } { \arg \operatorname* { m i n } } \left\| \frac { \Delta L ( \mathbf { u } ) } { \left\| \Delta L ( \mathbf { u } ) \right\| } - \frac { \Delta \hat { L } ( \mathbf { u } , \mathbf { p } , \mathbf { v } ) } { \left\| \Delta \hat { L } ( \mathbf { u } , \mathbf { p } , \mathbf { v } ) \right\| } \right\| .\tag{6}
$$

TABLE I  
MEDIAN MEAN POSITION ERROR (MMPE) [%] ON THE EVENT CAMERA DATASET FOR DIFFERENT EKF EVENT UPDATE THRESHOLDS
<table><tr><td> $n _ { e }$ </td><td>500</td><td>1000</td><td>3200</td><td>4800</td><td>7200</td><td>9200</td><td>15000</td><td>20000</td></tr><tr><td>MMPE</td><td>0.57</td><td>0.55</td><td>0.49</td><td>0.59</td><td>0.60</td><td>0.68</td><td>0.83</td><td>1.72</td></tr></table>

While $\Delta L$ is defined as an aggregation of events in a local patch, $\Delta \hat { L }$ is defined as the negative dot product between the local log image gradient and optical flow vector, following the linearized event generation model [36]. Here $W ( \mathbf { u } , \mathbf { p } )$ aligns the image gradient with the measured brightness increments according to the alignment parameters $p .$ EKLT minimizes (6) using Gauss-Newton and the Ceres library [37], and recovers alignment parameters $p$ and optical flow v. As opposed to the reference implementation of EKLT, which optimizes in a sliding window fashion after a fixed number of events, we trigger the optimization only when the adaptive number of events is reached, using each event batch only once. This entails a significant speed-up without loss in accuracy.

## C. Frontend Adaptations

Asynchronous feature updates: We convert the asynchronous feature tracks provided by EKLT to synchronous feature tracks via a synchronization step (Fig. 2. This step produces a temporally synchronized list of feature positions, which are passed to the backend. The backend uses the associated correspondences $\mathbf { z } _ { i } \Longleftrightarrow \mathbf { z } _ { j }$ together with consecutive camera poses $c _ { i }$ and $c _ { j }$ to update the state as discussed in Section III-A. It is performed by selecting the most recent feature in the currently tracked feature set and extrapolating the positions of all other features to its timestamp. We synchronize every time, a fixed number of events $n _ { e }$ is triggered, enabling variable-rate backend updates. We empirically found $n _ { e } = 3 2 0 0$ to work best, see Table I. We argue that reducing $n _ { e }$ will introduce additional noisy updates to the EKF which reduce the accuracy, while having too high $n _ { e }$ makes our approach less robust during high-speed motion.

This variable rate allows our algorithm to adapt to the scene dynamics (Fig. 3), leading to fewer EKF updates in slow sequences (Fig. 3, left) and a lower tracking error during highspeed sequences, compared to fixed rate updating. These features motivate the use of an event-based frontend since a purely frame-based one is limited by the framerate of the camera. Although, this may lead to drift in purely stationary environments where no events are triggered, this can easily be amended by enforcing a minimal backend update rate. Or by enforcing a no-motion prior when the event rate goes below a threshold, as in [6].

Outlier rejection: For EKLT we exclusively reject outliers by setting a maximum threshold on the optimized residual of the alignment score in (6). This allows outliers to be rejected quickly, without the need for costly geometric verification, such as 8-point RANSAC.

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/9ef4f71ba35e5a0e70acfa937fdf405e27a6ba7be1e1e8b7b30446693b4dce75.jpg)  
Fig. 3. Synchronous feature updates (red) tend to generate too many updates during slow sequences and too few during fast sequences, leading to high tracking error. Our irregular update strategy (purple) adapts to the event-rate, and thus maintains low tracking error in both scenarios.

## IV. EXPERIMENTS

We start by validating our approach on standard benchmarks in Section IV-B, where we compare the performance of EKLT-VIO against state-of-the-art event-based [18], frame-based [15] and event- and frame-based methods [6]. To study the effect on the event-based feature tracker, we also study an additional baseline, based on the HASTE feature tracker [38]. We then proceed to demonstrate the suitability of our approach on two important use-cases motivated by the Mars exploration scenario: (i) pure rotational motion, imitating hover-like conditions on Mars (Section IV-C), and (ii) challenging HDR conditions on newly collected datasets in the JPL Mars Yard and at the entrance of the Wells Cave, emulating the entry into lava tubes (Section IV-D).

## A. Baselines and Compared Methods

USLAM [6] is an event- and frame-based VIO method, which fuses feature tracks derived from frames and event-frames in an optimization-based backend.

EVIO [18] uses only events and IMU. Events are used to generate asynchronous feature tracks, which are then fused in a filter-based backend. Since open-source code is not available, we only report results on real sequences.

KLT-VIO [15] is a frame-based VIO method that fuses feature tracks based on FAST/KLT in a filter-based backend, and is specifically designed for use during helicopter flight.

HASTE-VIO [38] Finally, we combine the state-of-the-art purely event-based tracker HASTE [38] with xVIO as an additional baseline. Similar to EKLT, it produces asynchronous feature tracks which are first synchronized using the method described in Section III-C, before being fed into the backend.

## B. Real Data

We benchmark our methods on the Event-Camera Dataset [21], recorded with a DAVIS 240C [27] with synchronized images, events, IMU measurements, and very fast hand-held motions in an HDR scenario. An OptiTrack is used for ground-truth camera trajectories. We evaluate the pose tracking accuracy using the same protocol as [6], and report mean position error (MPE) in % of the total trajectory length and mean yaw error (MYE) in deg/m in Table II. In [6], USLAM uses different parameters for each sequence, and correct IMU bias initialization, resulting in the gray columns in Table II. We mark this method as USLAM\*. However, on Mars, VIO systems should perform robustly in unknown environments, making, parameter tuning and bias initialization infeasible. For this reason, we retune the parameters of USLAM to perform best on all sequences simultaneously resulting in the black values in Table II. All other methods were tuned in the same way. Comparing USLAM\* with USLAM shows that IMU bias initialization, and per-sequence hyperparameter tuning are clearly important to achieve low tracking error, reducing the error from 0.89% to 0.24%. Our EKLT-VIO, on the other hand, achieves an average error of 0.54% without bias initialization, 39% lower than USLAM. This improvement indicates that EKLT-VIO is simultaneously more robust to zero IMU bias initialization, and per-sequence hyperparameter tuning.

TABLE II  
POSE ESTIMATE ACCURACY COMPARISON ON THE EVENT-CAMERA DATASET [21] IN TERMS OF MEAN POSITION ERROR (MPE) IN % AND MEAN YAW ERROR (MYE) IN DEG/M. GRAYED-OUT RESULTS WITH (\*) BY USLAM [6] WERE ACHIEVED THROUGH PER-SEQUENCE PARAMETER TUNING AND CORRECT IMU BIAS INITIALIZATION, WHILE RESULTS IN BLACK USED A SINGLE PARAMETER SET, TUNED ON ALL SEQUENCES SIMULTANEOUSLY, AND WERE INITIALIZED WITH AN IMU BIAS OF ZERO
<table><tr><td>Dataset</td><td colspan="2">USLAM* [6]</td><td colspan="2">USLAM [6]</td><td colspan="2">EVIO [18]</td><td colspan="2">KLT-VIO [15]</td><td colspan="2">HASTE-VIO</td><td colspan="2">EKLT-VIO (ours)</td></tr><tr><td></td><td>MPE</td><td>MYE</td><td>MPE</td><td>MYE</td><td>MPE</td><td>MYE</td><td>MPE</td><td>MYE</td><td>MPE</td><td>MYE</td><td>MPE</td><td>MYE</td></tr><tr><td>Boxes 6DOF</td><td>0.30</td><td>0.04</td><td>0.68</td><td>0.03</td><td>4.13</td><td>0.92</td><td>0.97</td><td>0.05</td><td>2.03</td><td>0.03</td><td>0.84</td><td>0.09</td></tr><tr><td>Boxes Translation</td><td>0.27</td><td>0.02</td><td>1.12</td><td>2.62</td><td>3.18</td><td>0.67</td><td>0.33</td><td>0.08</td><td>2.55</td><td>0.46</td><td>0.48</td><td>0.25</td></tr><tr><td>Dynamic 6DOF</td><td>0.19</td><td>0.10</td><td>0.76</td><td>0.09</td><td>3.38</td><td>1.20</td><td>0.78</td><td>0.03</td><td>0.52</td><td>0.06</td><td>0.79</td><td>0.06</td></tr><tr><td>Dynamic Translation</td><td>0.18</td><td>0.15</td><td>0.63</td><td>0.22</td><td>1.06</td><td>0.25</td><td>0.55</td><td>0.06</td><td>1.32</td><td>0.06</td><td>0.40</td><td>0.04</td></tr><tr><td>HDR Boxes</td><td>0.37</td><td>0.03</td><td>1.01</td><td>0.31</td><td>3.22</td><td>0.15</td><td>0.42</td><td>0.02</td><td>1.75</td><td>0.09</td><td>0.46</td><td>0.06</td></tr><tr><td>HDR Poster</td><td>0.31</td><td>0.05</td><td>1.48</td><td>0.09</td><td>1.41</td><td>0.13</td><td>0.77</td><td>0.03</td><td>0.57</td><td>0.02</td><td>0.65</td><td>0.04</td></tr><tr><td>Poster 6DOF</td><td>0.28</td><td>0.07</td><td>0.59</td><td>0.03</td><td>5.79</td><td>1.84</td><td>0.69</td><td>0.02</td><td>1.50</td><td>0.03</td><td>0.35</td><td>0.02</td></tr><tr><td>Poster Translation</td><td>0.12</td><td>0.04</td><td>0.24</td><td>0.02</td><td>1.59</td><td>0.38</td><td>0.16</td><td>0.02</td><td>1.34</td><td>0.02</td><td>0.35</td><td>0.03</td></tr><tr><td>Shapes 6DOF</td><td>0.10</td><td>0.04</td><td>1.07</td><td>0.03</td><td>2.52</td><td>0.61</td><td>1.80</td><td>0.03</td><td>2.35</td><td>0.02</td><td>0.60</td><td>0.03</td></tr><tr><td>Shapes Translation</td><td>0.26</td><td>0.06</td><td>1.36</td><td>0.01</td><td>4.56</td><td>2.60</td><td>1.38</td><td>0.02</td><td>1.09</td><td>0.02</td><td>0.51</td><td>0.03</td></tr><tr><td>Average</td><td>0.24</td><td>0.06</td><td>0.89 *per-sequence hyperparameter tuning and correct IMU bias intialization</td><td>0.34</td><td>3.08</td><td>0.88</td><td>0.79</td><td>0.04</td><td>1.50</td><td>0.08</td><td>0.54</td><td>0.07</td></tr></table>

TABLE III

MEAN POSITION AND YAW ERROR (MPE AND MYE) IN % AND DEG/M ON ROTATION-ONLY SEQUENCES
<table><tr><td>Dataset</td><td colspan="2">USLAM [6]</td><td colspan="2">KLT-VIO [15]</td><td colspan="2">HASTE-VIO</td><td colspan="2">EKLT-VIO (ours)</td></tr><tr><td></td><td>MPE</td><td>MYE</td><td>MPE MYE</td><td>MPE</td><td>MYE</td><td></td><td>MPE</td><td>MYE</td></tr><tr><td>Dynamic Rotation</td><td colspan="2"></td><td colspan="2">9.97 0.13</td><td>6.22</td><td>2.32</td><td>7.71</td><td>1.52</td></tr><tr><td>Boxes Rotation</td><td colspan="2">unfeasible</td><td colspan="2">diverging</td><td>20.57</td><td>1.32</td><td>8.78</td><td>1.36</td></tr><tr><td>Poster Rotation</td><td colspan="2"></td><td colspan="2">diverging</td><td>3.96</td><td>0.09</td><td>1.44</td><td>0.09</td></tr><tr><td>Shapes Rotation</td><td colspan="2"></td><td colspan="2">diverging</td><td colspan="2">diverging</td><td>6.95</td><td>4.59</td></tr></table>

In terms of position error, EKLT-VIO outperforms all other methods on 5 out of 10 sequences. With an average MPE of 0.54% EKLT-VIO shows a 32% lower MPE than runner-up KLT-VIO with 0.79%. Finally, with a 3.08% MPE, EVIO [18] is outperformed by EKLT-VIO by 82%.

## C. Rotation-Only Sequences

As a next step, we show the suitability of EKLT-VIO in a Mars Mission-like scenario. To do this, we evaluate all methods on the rotation-only sequences of the Event-Camera Dataset, which are challenging for optimization-based backends such as USLAM [6]. Similar to hover-like conditions expected during Mars missions, these sequences translate only little compared to the average scene depth, which poses a challenge for keyframe generation and triangulation.

We adopt the same evaluation protocol as before and report results for all methods in Table III. We observed during this experiment that USLAM did not initialize during these sequences since it could never detect sufficient translation to insert a new keyframe, and it is thus marked with unfeasible. Frame-based KLT-VIO tracks well for the first 30 s, but diverges in the second part, where rapid shaking motion causes motion blur on the frames, and high feature displacements, both of which significantly impact the accuracy of the KLT frontend. This leads to a diverging state estimate. By contrast, event-based methods EKLT-VIO and HASTE-VIO can track robustly, because their event-based front-ends are unaffected by motionblur. EKLT-VIO, however, is the only method to converge on all sequences and yields a consistently lower tracking error compared to all compared methods. In summary, EKLT-VIO leverages the advantages of event-based frontends for robust high-speed tracking and the advantages of a filter-based backend to fuse small translational motions. This shows that EKLT-VIO is most suitable in these conditions.

## D. Mars-Mission Scenario: Wells Cave and JPL Mars Yard

Finally, we show the capabilities of EKLT-VIO in Marslike exploration scenarios, by comparing it to image-based methods KLT-VIO [16], ORB-SLAM3 [41], OpenVINS [42],

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/97fbced6aa7799f20bc88cb137510cee5df7afde40244d127e2927eb1d718f97.jpg)  
(a) Mars Yard preview

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/e2eaf434fbd6370ad4f5a19efe8e7a2ae9adbc6aa0d821a293ef29d8bb29ea23.jpg)  
(b) Overexposed image

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/288e7542834cfb28d244c7a40336bceb90fc67b145d4223c919029ad336ce7bd.jpg)  
(c) Recons. from events

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/4c4c28b25e8d2ad455177624b91d7a5885d0ab5579622b8ab93d47a639de36d5.jpg)  
(d) Trajectories

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/b4e094fd9f55a824c82dcb454d8d42212d5fae881038b9672c676b9ceb3d9ffd.jpg)  
(e) Wells Cave preview

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/03092214214a2e7dcb62ceb894da14a2d7a31ea381b8ee4ef351a0e6b893a94c.jpg)  
(f) Underexposed image

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/8b366aa4acdd9e6847d5298d752d7d040d0b1c2b1ab9eee2559203e4402b8b40.jpg)  
(g) Recons. from events

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/5a83dcde9b4048f56bc7101bcb7c02b93ad4772b636bfac32b984d88742348fb.jpg)  
(h) Trajectories

Fig. 4. In the Mars Yard (a) we test HDR conditions which cause severe oversaturation artefacts in standard images (b). Instead in the Wells Cave (e) we stud low light scenarios encountered in lava tubes, which cause undersaturation (f). HDR images reconstructed from events [39] (c,g) do not suffer from these artefacts, and are used by our method. As a result, we outperform existing frame-based approaches KLT-VIO [15] and ROVIO [40] on both trajectories.  
![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/ed7220b398a1f6bd6143598b0f86181b339564713b45d395da60675cbbd80960.jpg)  
Fig. 5. Tracked features on the Wells Cave sequence. While KLT-VIO and ROVIO quickly diverge, due to lacking features, EKLT-VIO can track successfully.

VINS-Mono [3] and ROVIO [40] on sequences recorded at the JPL Mars Yard (Fig. 4(a)), and Wells Cave Nature Preserve (Fig. 4(e)). The Mars Yard sequence features rapid illumination changes that challenge the autoexposure and result overexposures in the images (Fig. 4(b)). The Wells Cave instead is a cave system used by JPL to emulate lava tubes on Mars. It features a low illumination, leading to underexposure in the images (Fig. 4(f)). In the Wells Cave we use the DAVIS 346 [27], and in the Mars Yard, we use a mvBlueFOX-MLC200wG standard camera, a DVXplorer event camera, and an MPU9250 IMU.

Here we show that EKLT-VIO can run on events alone, by using images reconstructed from events provided by the method E2VID [39]. They feature a much higher dynamic range than the standard images (Fig. 4 (c,g)). We reconstruct frames every 15’000 events, resulting in an HDR video used by our method.

For a resolution of 640 × 480 these images can be provided with 30 FPS on a Quadro RTX 4000 GPU. However, EKLT-VIO only needs a subset of these images, since it only uses them for feature initialization.

Mars Yard: The trajectory used in this analysis is a hand-held circular motion with a diameter of 1.5 meters over a sharp shadow with increasing speed. The trajectories tracked by all methods are shown in Fig. 4(d). While EKLT-VIO consistently tracks the circular motion for at least two revolutions, filterbased methods KLT-VIO and ROVIO diverge due to a lack of features caused by motion blur and HDR conditions. The optimization-based methods ORB-SLAM3 and VINS-MONO fail to initialize, since the sequence starts directly from hover, and misses an initialization trajectory, with which to generate an initial map. OpenVINS fails to initialize due to missing parallax. These methods are therefore not plotted. This shows that thanks to the use of an event-based frontend and filter-based backend EKLT-VIO can overcome this condition.

Wells Cave: Finally, the trajectories in the Wells Cave, for all methods are shown in Fig. 4(h). Only filter-based methods KLT-VIO and ROVIO manage to initialize, but diverge quickly. EKLT-VIO tracks consistently, until reaching the tunnel entrance. Again, ORB-SLAM3 and VINS-Mono fail to initialize and therefore are not plotted.OpenVINS fails to initialize due to missing features. As shown in Fig. 5, EKLT-VIO consistently maintains SLAM features, while KLT-VIO only does so once it exits the cave.

## E. Limitations

We study EKLT-VIO, KLT-VIO, and HASTE-VIO in terms of their real-time factor (RTF, Fig. 6(a)) and report the RTF per feature (b) and computation allocations (c) for EKLT-VIO. We conduct all our experiments on a laptop with an Intel i7-7700HQ

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/7b68135aae7946b839514c5443a237ca2f821cb8a813e36a7b1906e472254e6f.jpg)  
(a) Realtime factor

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/09be92503f2e384fa5dee035e718494c2419273dbab746405b6e2169bd211178.jpg)  
(b) Realtime factor vs. Event rate

![](images/2022_Exploring_Event_Camera-Based_Odometry_for_Planetary_Robo/62ab14631852811bacf9c82d63e0cea81347498563d144c30c49e33bd4176951.jpg)  
(c) Computational pie chart  
Fig. 6. Real-time factor (RTF) (a) for EKLT-VIO (orange), HASTE-VIO (green) and KLT-VIO (blue) on Poster 6DOF. The RTF per tracked feature (b) increases with the event rate. Our method can process 89’000 events per second when tracking 45 features. As seen in (c), EKLT-VIO spends most of its computation tim tracking features.

## TABLE IV

REAL-TIME FACTOR SPEEDUP ON POSTER 6DOF. WE COMPARE RANDOM FILTERING (RF), REFRACTORY PERIOD FILTERING (RPF), REDUCING THE NUMBER OF FEATURES, AND INCREASING $n _ { e } .$ OUR BASELINE TRACKS 45 FEATURES AND UPDATES EACH FEATURE, EVERY $n _ { e } = 3 2 0 0$ EVENTS. RTF $> 1$ IS SLOWER THAN REAL-TIME
<table><tr><td>Speedup method</td><td>MPE</td><td>MYE</td><td>RTF Max</td><td>RTF Median</td></tr><tr><td>Baseline</td><td>0.36</td><td>0.02</td><td>43.6</td><td>17.9</td></tr><tr><td>RF r = 2</td><td>diverging</td><td></td><td>11.2</td><td>5.20</td></tr><tr><td> $\mathrm { R F } r = 5$ </td><td>diverging</td><td></td><td>5.70</td><td>2.20</td></tr><tr><td> $\mathrm { R P F } \left( \tau = 1 ~ \mathrm { m s } \right)$ </td><td>0.27</td><td>0.02</td><td>37.3</td><td>15.40</td></tr><tr><td> $\mathrm { R P F } \ ( \tau = 1 0 \ \mathrm { m s } )$ </td><td>0.48</td><td>0.02</td><td>15.7</td><td>7.70</td></tr><tr><td> $n _ { e } = 6 4 0 0$ </td><td>0.24</td><td>0.02</td><td>21.2</td><td>9.70</td></tr><tr><td>15 Features</td><td>0.31</td><td>0.02</td><td>18.6</td><td>8.70</td></tr><tr><td>15 Features, RPF (τ = 10 ms)</td><td>0.41</td><td>0.02</td><td>8.20</td><td>4.20</td></tr><tr><td>15 Features,  $\mathrm { R P F } ~ ( \tau = 1 0 ~ \mathrm { m s } ) , n _ { e } = 6 4 0 0$ </td><td>3.79</td><td>0.02</td><td>4.39</td><td>2.05</td></tr></table>

quadcore processor, exploiting however only a single core in the current implementation. The RTF measures how much time is spent to process a second of real-time, and RTF< 1 indicates real-time performance. As seen in Fig. 6(a) there exists a clear speed-accuracy trade-off between EKLT-VIO, HASTE-VIO, and KLT-VIO, since EKLT-VIO achieves a maximum real-time factor of around 45. Note that this is 45 times slower than real-time. For EKLT-VIO, the real-time factor correlates with the event rate (Fig. 6(b)), which depends on the scene texture and camera speed. On Poster 6DOF it can process 89’000 kEv/sec.

## F. Speedup Strategies

Fig. 6(c) shows that, the EKLT frontend remains the bottleneck, which directs future work toward speeding up EKLT. Table IV illustrates three speedup strategies to achieve realtime capabilities evaluated Poster 6DOF. (i) We reduce the number of tracked frontend features from 45 to 15, (ii) we increased $n _ { e }$ the number of events before triggering an update, by a factor of two and (iii) we reduce the event rate with random filtering (RF), randomly keeping every $r ^ { \mathrm { t h } }$ event, or refractory period filtering (RPF), where events within a time τ of the previous event are discarded. To improve the convergence in (ii) we additionally implemented IMU-based feature prediction [40], to improve the initial guess. While naive RF degrades performance, RPF with $\tau = 1 0$ ms reduces the median RTF to 7.7. Reducing the frontend features results in an RTF of 8.7, and, when combined with filtering, leads to an RTF of 4.2. These steps lead to a minimal increase of the MPE from 0.36 to 0.41. Setting $n _ { e } = 6 4 0 0$ results in an RTF of9.7, while reducing the MPE from 0.36 to 0.24. However, when combined with additional filtering, we found that the method diverges with an MPE of 3.79, but a lower RTF of 2.05. The remaining gap can be closed by software-side techniques, such as distributing the workload to multiple cores (see https: //github.com/Doch88/rpg\_eklt\_multithreading). There, up to four cores were parallelized, leading to a 3.6-fold speedup.

## V. CONCLUSION

Future planetary missions, require us to venture into previously inaccessible domains, such as lava-tubes on Mars, which pose challenging lighting conditions for traditional image-based VIO. We explored the use of event cameras, which promise to shed light in these domains due to their high dynamic range. We present EKLT-VIO which integrates the state-of-the-art feature tracker EKLT with the filter-based backend xVIO thus leveraging the advantages of both. The event-based frontend provides robust high-speed feature measurements even in low-light and HDR scenarios while the filter-based backend addresses the limitations of traditional optimization-based VIO algorithms in near-hovering conditions. We show an evaluation on Mars-like sequences and challenging hand-held sequences of the Event-Camera dataset. On these sequences, we demonstrate the robust pose tracking the performance of our methods, showing a mean position error reduction of up to 32% compared to event- and frame-based state-of-the-art methods. Additionally, we showcase the advantages of our backend and frontend in the first successful evaluation on the rotation-only sequences of the Event-Camera Dataset with fast motion and challenging lighting conditions. Finally, we demonstrate our method’s robustness in visually challenging conditions recorded in the JPL Mars Yard and in the Wells Cave, replicating our mission scenario. To spur further research in this direction, we open-source the implementation of this work and release our Mars-like sequences.

## ACKNOWLEDGMENT

The authors would like to thank Konstantin Kalenberg for the feature prediction implementation improving EKLT’s computational efficiency.

## REFERENCES

[1] M. Li and A. I. Mourikis, “Optimization-based estimator design for vision-aided inertial navigation,” Robotics: Science and Systems, Berlin, Germany, 2013, pp. 241–248.

[2] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, “Keyframe-based visual-inertial SLAM using nonlinear optimization,” Int. J. Robot. Res., vol. 34, pp. 314–334, 2015.

[3] T. Qin, P. Li, and S. Shen, “VINS-Mono: A robust and versatile monocular visual-inertial state estimator,” IEEE Trans. Robot., vol. 34, no. 4, pp. 1004–1020, Aug. 2018.

[4] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “On-manifold preintegration for real-time visual–inertial odometry,” IEEE Trans. Robot., vol. 33, no. 1, pp. 1–21, Feb. 2017.

[5] G. Gallego et al., “Event-based vision: A survey,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 44, no. 1, pp. 154–180, Jan. 2022.

[6] A. Rosinol Vidal, H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Ultimate SLAM? combining events, images, and IMU for robust visual SLAM in HDR and high speed scenarios,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 994–1001, Apr. 2018.

[7] S. Sun, G. Cioffi, C. De Visser, and D. Scaramuzza, “Autonomous quadrotor flight despite rotor failure with onboard vision sensors: Frames vs. events,” IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 580–587, Apr. 2021.

[8] A. Johnson et al., “The lander vision system for mars 2020 entry descent and landing,” in Proc. AAS Guidance, Navigation, Control Conf., 2017, pp. 2–7.

[9] B. Bos et al., “Touch and go camera system (TAGCAMS) for the OSIRIS-REx asteroid sample return mission,” Space Sci. Rev., vol. 214, p. 37, no. 1, 2018.

[10] D. S. Bayard et al., “Vision-based navigation for the NASA mars helicopter,” in Proc. AIAA Scitech Forum, 2019, Art. no. 1411.

[11] M. Maimone, Y. Cheng, and L. Matthies, “Two years of visual odometry on the mars exploration rovers,” J. Field Robot., vol. 24, no. 3, pp. 169–186, 2007.

[12] J. Delaune et al., “Extended navigation capabilities for a future mars science helicopter concept,” in Proc. IEEE Aerosp. Conf., 2020, pp. 1–10.

[13] B. Carrier et al., “Mars extant life: What’s next? conference report,” Astrobiology, vol. 20, no. 5, pp. 785–814, 2020.

[14] C. Phillips-Lander et al., “Mars astrobiological cave and internal habitability explorer (MACIE): A new frontiers mission concept,” 38th Mars Exploration Prog. Anal. Group, vol. 4, 2020.

[15] J. Delaune, D. S. Bayard, and R. Brockers, “Range-visual-inertial odometry: Scale observability without excitation,” IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 2421–2428, Apr. 2021.

[16] J. Delaune, D. S. Bayard, and R. Brockers, “xVIO: A range-visual-inertial odometry framework,” 2020, arXiv:2010.06677.

[17] H. Grip, “Surviving an in-flight anomaly: What happened on ingenuity’s sixth flight,” NASA, Tech. Rep., 2021. Accessed: Feb. 20, 2022. [Online]. Available: https://mars.nasa.gov/technology/helicopter/status/ 305/surviving-an-in-flight-anomaly-what-happened-on-ingenuityssixth-flight/

[18] A. Z. Zhu, N. Atanasov, and K. Daniilidis, “Event-based visual inertial odometry,” in Proc. IEEE Conf. Comput. Vis. Pattern Recog., 2017, pp. 5391–5399.

[19] D. G. Kottas, K. J. Wu, and S. I. Roumeliotis, “Detecting and dealing with hovering maneuvers in vision-aided inertial navigation systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robot. Syst., 2013, pp. 3172–3179.

[20] D. Gehrig, H. Rebecq, G. Gallego, and D. Scaramuzza, “EKLT: Asynchronous photometric feature tracking using events and frames,” Int. J. Comput. Vis., vol. 128, pp. 601–618, 2020.

[21] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, and D. Scaramuzza, “The event-camera dataset and simulator: Event-based data for pose estimation, visual odometry, and SLAM,” Int. J. Robot. Res., vol. 36, no. 2, pp. 142–149, 2017.

[22] J. Delmerico and D. Scaramuzza, “A benchmark comparison of monocular visual-inertial odometry algorithms for flying robots,” in Proc. IEEE Int. Conf. Robot. Autom., 2018, pp. 2502–2509.

[23] A. I. Mourikis and S. I. Roumeliotis, “A multi-state constraint Kalman filter for vision-aided inertial navigation,” in Proc. IEEE Int. Conf. Robot. Autom., 2007, pp. 3565–3572.

[24] H. Kim, S. Leutenegger, and A. J. Davison, “Real-time 3D reconstruction and 6-DoF tracking with an event camera,” in Proc. Eur. Conf. Comput. Vis., 2016, pp. 349–364.

[25] H. Rebecq, T. Horstschäfer, G. Gallego, and D. Scaramuzza, “EVO: A geometric approach to event-based 6-DOF parallel tracking and mapping in real-time,” IEEE Robot. Autom. Lett., vol. 2, no. 2, pp. 593–600, Apr. 2017.

[26] H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Real-time visualinertial odometry for event cameras using keyframe-based nonlinear optimization,” in Proc. Brit. Mach. Vis. Conf., 2017, pp. 1–8.

[27] C. Brandli, R. Berner, M. Yang, S.-C. Liu, and T. Delbruck, “A 240x180 130 dB 3 µs latency global shutter spatiotemporal vision sensor,” IEEE J. Solid-State Circuits, vol. 49, no. 10, pp. 2333–2341, Oct. 2014.

[28] E. Rosten and T. Drummond, “Machine learning for high-speed corner detection,” in Proc. Eur. Conf. Comput. Vis., 2006, pp. 430–443.

[29] B. D. Lucas et al., “An iterative image registration technique with an application to stereo vision,” in Proc. Int. Joint Conf. Artif. Intell., Vancouver, British Columbia, 1981, vol. 81, pp. 674–679.

[30] S. Weiss, M. Achtelik, S. Lynen, M. Chli, and R. Siegwart, “Real-time onboard visual-inertial state estimation and self-calibration of MAVs in unknown environments,” in Proc. IEEE Int. Conf. Robot. Autom., 2012, pp. 957–964.

[31] E. Mueggler, G. Gallego, H. Rebecq, and D. Scaramuzza, “Continuoustime visual-inertial odometry for event cameras,” IEEE Trans. Robot., vol. 34, no. 6, pp. 1425–1440, Dec. 2018.

[32] G. Cioffi, T. Ciesleski, and D. Scaramuzza, “Continuous-time vs. discretetime vision-based slam: A comparative study,” IEEERobot.Automat. Lett., vol. 7, no. 2, pp. 2399–2406, Apr. 2022.

[33] M. Li and A. Mourikis, “Vision-aided inertial navigation with rollingshutter cameras,” Int. J. Robot. Res., vol. 33, no. 11, pp. 1490–1507, 2014.

[34] K. Eckenhoff, P. Geneva, and G. Huang, “MIMC-VINS: A versatile and resilient multi-IMU multi-camera visual-inertial navigation system,” IEEE Trans. Robot., vol. 37, no. 5, pp. 1360–1380, Oct. 2021.

[35] J. Montiel, J. Civera, and A. Davison, “Unified inverse depth parametrization for monocular SLAM,” Robot.: Sci. Syst., 2006.

[36] G. Gallego, J. E. A. Lund, E. Mueggler, H. Rebecq, T. Delbruck, and D. Scaramuzza, “Event-based, 6-DOF camera tracking from photometric depth maps,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 10, pp. 2402–2412, Oct. 2018.

[37] S. Agarwal, K. Mierle, and T. C. S. Team, “Ceres solver,” 2022. Accessed: Feb. 20, 2022. [Online]. Available: https://github.com/ceres-solver/ceressolver

[38] I. Alzugaray and M. Chli, “Asynchronous multi-hypothesis tracking of features with event cameras,” in Proc. Int. Conf. 3D Vis., 2019, pp. 269–278.

[39] H. Rebecq, R. Ranftl, V. Koltun, and D. Scaramuzza, “High speed and high dynamic range video with an event camera,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 43, no. 6, pp. 1964–1980, Jun. 2021.

[40] M. Bloesch, S. Omari, M. Hutter, and R. Siegwart, “Robust visual inertial odometry using a direct EKF-based approach,” in Proc. IEEE/RSJ Int. Conf. Intell. Robot. Syst., 2015, pp. 298–304.

[41] C. Campos, R. Elvira, J. Rodriguez, J. Montiel, and J. Tardos, “ORB-SLAM3: An accurate open-source library for visual, visual-inertial and multi-map SLAM,” vol. 37, no. 6, pp. 1874–1890, Dec. 2021.

[42] P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, and G. Huang, “OpenVINS: A research platform for visual-inertial estimation,” in Proc. IEEE Int. Conf. Robot. Autom., Paris, France, 2020, pp. 4666–4672.