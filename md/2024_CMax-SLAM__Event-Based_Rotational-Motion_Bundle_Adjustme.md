# CMax-SLAM: Event-Based Rotational-Motion Bundle Adjustment and SLAM System Using Contrast Maximization

Shuang Guo , Graduate Student Member, IEEE, and Guillermo Gallego , Senior Member, IEEE

Abstract—Event cameras are bioinspired visual sensors that capture pixelwise intensity changes and output asynchronous event streams. They show great potential over conventional cameras to handle challenging scenarios in robotics and computer vision, such as high speed and high dynamic range. This article considers the problem of rotational motion estimation using event cameras. Several event-based rotation estimation methods have been developed in the past decade, but their performance has not been evaluated and compared under unified criteria yet. In addition, these prior works do not consider a global refinement step. To this end, we conduct a systematic study of this problem with two objectives in mind: Summarizing previous works and presenting our own solution. First, we compare prior works both theoretically and experimentally. Second, we propose the first event-based rotation-only bundle adjustment (BA) approach. We formulate it leveraging the state-of-the-art contrast maximization (CMax) framework, which is principled and avoids the need to convert events into frames. Third, we use the proposed BA to build CMax-simultaneous localization and mapping (SLAM), the first event-based rotationonly SLAM system comprising a front-end and a back-end. Our BA is able to run both offline (trajectory smoothing) and online (CMax-SLAM back-end). To demonstrate the performance and versatility of our method, we present comprehensive experiments on synthetic and real-world datasets, including indoor, outdoor, and space scenarios. We discuss the pitfalls of real-world evaluation and propose a proxy for the reprojection error as the figure of merit to evaluate event-based rotation BA methods. We release the source code and novel data sequences to benefit the community. We hope this work leads to a better understanding and fosters further research on event-based egomotion estimation.

Index Terms—Computer vision, event-based camera, localization, mapping, SLAM, smart cameras.

## I. INTRODUCTION

VENT cameras are novel bio-inspired visual sensors that to the images/frames produced by traditional cameras, the output of an event camera is a stream of asynchronous events, $e _ { k } = ( \mathbf { x } _ { k } , t _ { k } , p _ { k } )$ , comprising the timestamp $t _ { k } ,$ pixel location $\mathbf { x } _ { k } \doteq ( x _ { k } , y _ { k } ) ^ { \flat }$ , and polarity $p _ { k }$ of the brightness changes. This unique working principle offers potential advantages over frame-based cameras: High temporal resolution (in the order of $\mu \mathrm { s } )$ , high dynamic range (HDR) (140 dB versus 60 dB of standard cameras), temporal redundancy suppression, and low power consumption (20 mW versus 1.5 W of standard cameras) [3]. They are beneficial for challenging tasks in robotics and computer vision, such as egomotion estimation [4], [5], [6], [7], [8], [9], [10] and simultaneous localization and mapping (SLAM) [11], [12], [13], [14], [15], [16], [17], [18], [19], [20], [21] in high-speed, extreme lighting, or HDR conditions.

Rotational motion estimation is an essential problem in vision and robotics, and it acts as a foundation for higher order motion formulations [e.g., six degrees-of-freedom (6-DOF)]. Despite decades of research, reliably estimating the motion of a purely rotating camera is still challenging, especially using frame-based cameras since motion blur, under/over-exposure and large interimage displacements are likely to occur and ruin data association. Conversely, event cameras do not suffer from blur, low dynamic range, or large displacements between data bits [3], hence, they can take on high-speed and/or HDR rotational motion estimation. The challenge consists of developing novel algorithms to process the unconventional, motion-dependent data produced by event cameras to reliably establish data association, and thus enable robust motion estimation.

Several works have demonstrated the capabilities of event cameras to estimate rotational motion in difficult scenarios (high speed, HDR) [6], [12], [15], [22]. Kim et al. [12] proposed a 3-DOF simultaneous mosaicing and tracking (SMT) method consisting of two Bayesian filters operating in parallel to estimate the camera motion and scene map. Also working in parallel, but using nonlinear least squares (NLLS), a real-time panoramic tracking and probabilistic mapping was presented in [15]. Recently, Kim and Kim[22] extended the contrast maximization (CMax) framework in [6] to simultaneously estimate angular velocity and absolute orientation. However, these methods have not been compared or evaluated under unified criteria yet. Carrying out such a benchmark is challenging and useful to identify best practices that foster further improvements. Besides, all these methods are short-term (i.e., they estimate the rotation for the current set of events), corresponding to the front-ends of SLAM systems [23], and therefore, do not consider a long-term (i.e., global) refinement step. Such a step, implemented in the back-end of modern SLAM systems, is a desirable feature because it improves accuracy and robustness.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/0d7db225491e16f0e73b3186f3e1ff828717f8f75b1bdec3abb0f2ef2b336475.jpg)  
Fig. 1. Proposed CMax-SLAM system takes as input the asynchronous event stream to estimate the rotational egomotion of the camera while producing a sharp panoramic map (IWE) as by-product. We can further reconstruct a grayscale map by feeding the events and the estimated trajectory to the mapping module of SMT [12].

## Contributions

This work aims at filling these gaps. Hence, we conduct a systematic comparative study on rotational motion estimation using event cameras, summarizing previous works, and presenting our own solution to the problem. We propose a novel rotation-only BA approach for event cameras, which acts as a back-end. In contrast with the reprojection or photometric errors used in frame-based BA [24], we seek to exploit the state-of-the-art CMax framework [25], [26], [27], [28], [29], [30], [31], [32], [33], [34], [35], [36], [37], [38] for the task of BA. We explicitly model the camera trajectory continuously in time and use it to warp the events onto a global (panoramic) map. We cast the problem of event-based BA as that of finding the trajectory of the rotating camera that achieves the sharpest (i.e., motion-compensated) scene map. Hence, the map of the scene is internally obtained as a by-product of the optimization (see Fig. 1). Furthermore, the effectiveness of our BA is demonstrated for the refinement of the trajectories produced by the above-mentioned event-based rotation estimation front-ends.

We pair the proposed BA (back-end) with an event-based angular velocity estimator as front-end [6] to implement an event-based rotational SLAM system, called CMax-SLAM. We show that our system is able to work reliably on challenging synthetic and real-world datasets, including indoor and outdoor scenarios, while outperforming previous works. Furthermore, we also show how our method can handle very different scenes, including the data produced by an event camera attached to a telescope, in a star tracking task.

In summary, our contributions are as follows:

1) We theoretically compare and experimentally benchmark several event-based rotational motion estimation methods under unified criteria, in terms of accuracy and efficiency (see Sections II and IV).

2) We propose the first event-based rotation-only BA method to refine the continuous-time trajectory of an event camera while reconstructing a sharp panoramic map of scene edges (see Section III-B).

3) We propose an event-based rotation-only SLAM system called CMax-SLAM, comprising both a front-end and a back-end for the first time (see Section III-C).

4) We demonstrate the method on a variety of scenarios: Indoor, outdoor, and space applications, which shows the versatility of our approach (see Sections IV and V).

5) We highlight the potential pitfalls of evaluating rotationonly methods on nonstrictly rotational data and propose a sensible figure of merit (FOM) for event-based rotational BA in real-world scenarios (see Section IV).

6) We release the source code and novel data sequences with high spatial resolution.

We hope our work leads to a better understanding of the field of event-based egomotion estimation and helps taking on related motion-estimation tasks.

## II. EVENT-BASED ROTATION ESTIMATION METHODS

Let us review the event-based rotational motion estimation approaches involved in this benchmark. The primary features of these works are summarized and compared in Table I. For more details, we refer to [6], [12], [15], and [22].

## A. Event-Based SMT

SMT [12] consists of two modules operating in a parallel tracking-and-mapping fashion [39]. In the tracking module, a particle filter (PF) estimates the rotation of the camera given a map of the scene. The mapping module builds a grayscale panoramic map of the scene in two steps: First, extended Kalman filters (EKFs) incrementally update the grayscale gradient at each pixel of the panoramic map. Second, Poisson integration recovers the absolute intensity [see Fig. 2(a)]. Both modules collaborate by assuming that the output from each other is accurate. However, this assumption often does not hold, which leads to drift accumulation and tracking failure.

The event generation model (EGM) [3] in its original form (for the PF) or in its linearized version (for the EKFs) is used in the measurement models of the Bayesian filters. Specifically, the orientation tracker sets the likelihood of each event $p ( e _ { k } | \mathtt { R } )$ by computing the brightness increment

$$
z = M ( \mathbf { p } _ { m } ^ { t _ { k } } ) - M ( \mathbf { p } _ { m } ^ { t _ { k } - \Delta t _ { k } } )\tag{1}
$$

TABLE I  
COMPARISON OF THE MAIN CHARACTERISTICS OF THE METHODS CONSIDERED AND THE NEW ONE (LAST COLUMN)
<table><tr><td></td><td>Kim et al. [12] (SMT)</td><td>Gallego et al. [6] (CMax-ω)</td><td>Reinbacher et al. [15] (RTPT)</td><td>Kim et al. [22] (CMax-GAE)</td><td>Ours (Back-end)</td></tr><tr><td>Problem solved</td><td>Absolute poses</td><td>Angular velocity</td><td>Absolute poses</td><td>Angular velocity and absolute poses</td><td>Trajectory refinement</td></tr><tr><td>Tracking</td><td>PF/EKF</td><td>CMax (Local only)</td><td>Minimize photometric error (4)</td><td>CMax (Local and global)</td><td>CMax (Global)</td></tr><tr><td>Mapping</td><td>Grayscale panoramic map</td><td>Local IWE</td><td>Probabilistic panoramic map</td><td>3D sphere of points</td><td>Panoramic (global) IWE</td></tr><tr><td>Is EGM used?</td><td>Yes (1), (2)</td><td>No</td><td>Yes (5), No, using [11]</td><td>No</td><td>No</td></tr><tr><td>Is polarity required?</td><td>Yes</td><td>Optional</td><td>No</td><td>Yes (for ω) /No (for ∆R)</td><td>No</td></tr><tr><td>Event processing</td><td>By event</td><td>By batch</td><td>By batch</td><td>By batch</td><td>By event (by batch for speed-up)</td></tr><tr><td>Bootstrapping</td><td>Random</td><td>N/A</td><td>Random</td><td>Run [6]</td><td>N/A</td></tr></table>

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/e376f19beddc0873f0cee92d12285dde7175d6a2a26b8ef536ddf7229415d817.jpg)  
(a) SMT [12]

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/40f224a63983bcc5388ca33f6dfc5b43e4aeb3872b3b6ad709e07b9c10b5d08b.jpg)  
(b) CMax-ω [6]

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/a91eba4f2b51f26eebfc7b0873bc87e4c00dab715c79d379e562139070ab2ab8.jpg)  
(c) RTPT [15]

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/82c94a06170677628607cc6fd4a3809a44fe10ea1b2982172d95c5c00bb81815.jpg)  
(d) CMax-GAE [22]  
Fig. 2. Maps produced by the rotation estimators involved in the benchmark. (a) Grayscale panoramic map by [12] (red and blue dots indicate positive and negative inlier events whereas yellow and green dots represent positive and negative outlier events in the current field of view (FOV), respectively). (b) Local IWE after CMax by [6], where the darkness represents the amount of accumulated events. (c) Probabilistic map by [15], where dark means ≈ 1 (edge), and white means ≈ 0 (no edge). The green dots indicate the events in the current FOV. (d) Global map containing projected visible events by [22] (blue and red dots have the same meaning as in Fig. 2(a), and pink dots indicate pixels with both positive and negative events).

and using it to look up the value of a Gaussian-shaped curve centered at C. Here, M is the logarithm of the grayscale panoramic map and $\mathbf { p } _ { m } ^ { t _ { k } } \equiv \mathbf { p } ( \mathtt { R } ( t _ { k } ) , \mathbf { x } _ { k } )$ is the map point obtained by transferring event coordinates $\mathbf x _ { k } = ( x _ { k } , y _ { k } ) ^ { \top }$ according to the rotation at time $t _ { k } , \mathtt { R } ( t _ { k } )$ . In the mapping module, each map point’s EKF sets the likelihood of each event $p ( e _ { k } | M )$ using the following measurement model:

$$
z = { \frac { 1 } { \Delta t _ { k } } } , h ( \mathbf { g } ) = { \frac { - \mathbf { g } \cdot \mathbf { v } } { p _ { k } C } }\tag{2}
$$

where the state is $\mathbf { g } \doteq \nabla M ( \mathbf { p } _ { m } ^ { t _ { k } } )$ , i.e., the spatial gradient of the intensity at the map point corresponding to the current event $e _ { k }$ and rotation $\mathtt { R } ( t _ { k } )$ . The velocity is given by v ≈ $\Delta \mathbf { p } _ { m } ^ { t _ { k } } / \Delta t _ { k }$ where $\Delta \mathbf { p } _ { m } ^ { t _ { k } } \doteq \dot { \mathbf { p } } _ { m } ^ { t _ { k } } - \mathbf { p } _ { m } ^ { t _ { k } - \Delta t _ { k } }$ approximates the displacement traveled by the spatial gradient during $\Delta t _ { k }$

## B. Event-Based Angular Velocity Estimation

Gallego and Scaramuzza [6] proposed estimating the angular velocity ω of a rotating camera by aligning events corresponding to the same scene edge. Events $\mathbf { \check { \mathcal { E } } } = \{ e _ { k } \mathbf  \check { \mathcal { \} } } _ { k = 1 } ^ { N _ { e } }$ are processed in packets and warped according to the candidate motion, $e _ { k } \mapsto$ $\bar { e } _ { k } ^ { \prime } \doteq \mathbf { W } ( e _ { k } ; \omega )$ , yielding an image of warped events (IWE) $I ( \omega , \mathcal { E } )$ . Then, the problem is formulated as finding the angular velocity that achieves the sharpest IWE [see Fig. 2(b)]. Several alignment objective functions are available [26], [40]. The variance of the IWE (i.e., image contrast) is adopted in [6] to quantify event alignment, and the nonlinear conjugate gradient method of Fletcher and Reeves (CG-FR) is used as optimizer

$$
\omega ^ { * } = \arg \operatorname* { m a x } _ { \omega } \mathrm { V a r } \left( I ( \omega , \mathcal { E } ) \right)\tag{3}
$$

where $\mathcal { E }$ is the current packet of events.

The sharp IWE acts as a local map of the scene. Since [6] focuses on estimating angular velocity rather than absolute orientation, it does not need a global (panoramic) map. The method assumes events are caused by moving edges, and thus the sharp IWE is the spatial gradient entangled with the motion [25], [41]: $| I ( \omega ^ { * } , \mathcal { E } ) | \propto | \nabla L \cdot \mathbf { v } |$ (not simply $| \nabla L | )$ , where L is the log brightness on the sensor’s image plane.

## C. RTPTfor Event Cameras

Similarly to SMT, real-time panoramic tracking (RTPT) also has two parallel modules for tracking and mapping. The tracking part uses direct alignment techniques [42] to minimize the error between the current events (“local map”) and the global map built from past events

$$
\arg \operatorname* { m i n } _ { \pmb { \vartheta } } \frac { 1 } { 2 } \sum _ { k = 1 } ^ { N _ { e } } \bigl ( 1 - M ( \pi ( \mathbf { x } _ { k } ; \pmb { \vartheta } ) ) \bigr ) ^ { 2 }\tag{4}
$$

where $\pi ( \mathbf { x } _ { k } ; \vartheta ) \equiv \mathbf { p } _ { m } ^ { t _ { k } }$ is the map point corresponding to the current event and rotation parameters ϑ (e.g., exponential coordinates) and M is the global (panoramic) map. Hence, the tracking problem is formulated as a NLLS problem.

The mapping module builds a probabilistic 2-D map of scene edges M(p) (or “spatial event rate”), where higher values indicate events are more likely to be produced when sensor pixels cross that map point [see Fig. 2(c)]. The values are interpreted as probabilities, hence, they are bounded, $M \in [ 0 , 1 ]$ , and they are designed so that the residual (4) vanishes if the rotated events $\mathbf { p } _ { m } ^ { t _ { k } }$ (i.e., local map) align with high values of the edge map M. The map is theoretically justified using the linearized EGM, deriving a formula for the probability of an event being triggered by a map point p (factoring out the camera motion by integrating over a uniform prior) [15, Eq. (7)]

$$
P ( e | \mathbf { p } ) = 1 - \frac { 2 } { \pi } \arcsin \left( \frac { C } { \| \nabla \log I ( \mathbf { p } ) \| } \right)\tag{5}
$$

where log $I ( \mathbf { p } )$ is the logarithm of the intensity at p. This probability curve model grows with a concave shape from 0 at ∇ log $I ( \mathbf { p } ) \| / C = 1$ 1 to 1 for ∇ log $I ( \mathbf { p } ) \| \gg$ 1. In practice, the map is built using a frequentist approach from prior work [11]: $M ( \mathbf { p } ) = O ( \mathbf { p } ) / N ( \mathbf { p } )$ , where $O ( \mathbf { p } )$ and $N ( \mathbf { p } )$ count the number of triggered events and the number of possible event occurrences at every map point, respectively.

Actually, the event stream is fed to the tracking module in packets to obtain a pose/rotation update using a Gauss–Newtontype scheme. Then, the mapping module updates the map using the whole event packet. In this benchmark, RTPT is the only one whose code relies on GPU for acceleration.

## D. CMax Over Globally Aligned Events (CMax-GAE)

Kim and Kim, [22] extended the method in [6] to jointly estimate angular velocity and absolute rotation. That is, besides aligning all events in the current packet, the local IWE is also aligned to a global IWE by solving the problem

$$
\underset { \omega , \Delta \mathtt { R } } { \mathrm { a r g m a x } } \ \| I _ { L } \left( \omega , \Delta \mathtt { R } \right) + I _ { G } \left( \mathtt { R } \right) \| ^ { 2 }\tag{6}
$$

where $\omega$ and $\Delta \mathbb { R }$ are the angular velocity and incremental pose change for the current event packet, R is the camera pose maintained during the running sequence, $I _ { L }$ is the local IWE rotated by $\Delta \mathtt { R }$ so that it is aligned to the global map, and $I _ { G }$ is the global map centered at $I _ { L }$ (twice as high and wide as $I _ { L } )$ $I _ { G }$ is obtained by projecting all visible past events (stored on the unit sphere, which acts as the scene map) onto the image plane [see Fig. 2(d)].

Once the optimal incremental rotation $\Delta \mathbb { R } ^ { * }$ has been obtained for the current packet by solving (6), the camera pose is updated in preparation for the next packet of events

$$
\begin{array} { r } { { \tt R }  { \tt R } \Delta { \tt R } ^ { * } . } \end{array}\tag{7}
$$

Note that in [22] the mean square metric is adopted to compute the contrast ofthe IWE, and the RMSProp optimizer is employed to maximize the objective function (instead of the variance and CG-FR method used in [6] and in Section II-B).

## E. Discussion ofthe State-of-the-Art Motion Estimators

Let us highlight some of the pros and cons of each approach. SMT [12] recovers the richest form of scene map (absolute brightness panorama), in which the EGM is used and the contrast threshold $\bar { C }$ is required. However, for real event cameras, it is difficult to accurately know the value of C and it varies greatly even within a single dataset [43]. In addition, the error propagation between tracking and mapping parts is inevitable, which is especially noticeable when the camera revisits past regions of the scene.

RTPT emphasizes the idea that event polarity is not needed for panoramic tracking; only the space-time coordinates of the events are needed. Hence, the resulting map (an edge map) is significantly different from the grayscale map in SMT. Formulating the tracking problem as NLLS enables the use of Gauss– Newton’s method and acceleration to converge faster to the motion parameters. Thanks to the probabilistic paradigm, RTPT is, to some extent, robust to scenes with moving objects. However, the implementation of this approach stores all events throughout the whole running process, which is memory-consuming and intractable for long-term tracking. Both PF-SMT [12] and RTPT require a GPU to run at moderate speeds.

The methods based on CMax [6], [22] do not require the specification of the contrast threshold C. These methods, while not being formulated as NLLS (to exploit fast convergence of Gauss–Newton), run at moderate speeds on standard CPUs, i.e., without requiring a GPU. CMax-ω [6] continuously provides accurate angular velocity estimations and does not suffer from error propagation since only a local IWE is needed. However, it does not consider absolute pose estimation, which may give rise to accumulated drift if the velocities were integrated into poses. CMax-GAE [22] partially reduces drift by additionally introducing a global alignment term in the CMax objective function. However, it maintains a 3-D sphere consisting of all past events as well as their visibility in the current FOV, which takes up a lot of memory and, therefore, does not support long-term tracking.

Our method (last column of Table I) builds on the strengths of previous works: Specifying C is not needed, building an edge map is sufficient for egomotion estimation, a global map helps reduce drift, storing all events is avoided because it is expensive (time and memorywise). We also observe that an explicit continuous trajectory model is currently missing in the event-based rotation-only estimation literature, hence, we incorporate it to facilitate the formulation of the solution.

## III. METHODOLOGY

This section first describes the theoretical formulation of the CMax-based BA (see Sections III-A and III-B). Then, it introduces the pipeline of CMax-SLAM (see Section III-C), including both front-end (see Section III-C1) and back-end (see Section III-C2), with a focus on how to adapt the proposed BA to an online system. Finally, the characteristics of our method are briefly discussed (see Section III-D).

## A. Principle ofCMaxfor Rotational Motion Estimation

Our method builds on the idea of event alignment put forward by the CMax framework [6], [25]. Assuming constant illumination, when the event camera rotates relative to the scene, edge patterns on the image plane move along point trajectories defined by the camera motion, producing events. Simply summing events pixelwise or along arbitrary point trajectories yields a blurred image (or histogram) of warped events. Instead, summing events along the point trajectories defined by the camera motion yields a sharp image of the (motion-compensated) edges that caused the events. This insight is leveraged in [6]: During the time span of a small packet of events, the point trajectories can be parameterized by a constant angular velocity ω model and CMax recovers the camera’s velocity by searching for the ω whose point trajectories best align with the events, producing the sharpest IWE. Shiba et al. [38], [44] shows that the point trajectories defined by this angular velocity warp are well-behaved, not suffering from “event collapse” (i.e., the IWEs cannot be “over-sharp”). As we show, the above idea can be extended to longer time intervals, and consequently, more complex ways to parameterize the camera motion R(t) (than simply constant ω) are needed. A continuous-time model $\mathtt { R } ( t )$ is the most generic one we can think of. While the CMax objective function seems to be ill-posed in the general (6-DOF) camera motion case [38], [44], our method does not suffer from event collapse. The proof is given in Appendix A.

## B. Event-Based BA

BA is a mature technique in frame-based visual SLAM [23], [45], where it is usually formulated as the minimization of the reprojection error or the photometric error, so as to refine camera poses, scene structure, and even calibration parameters as well. In contrast, the asynchronous event stream contains neither trackable motion-invariant features nor intensity information, so frame-based BA algorithms are not directly applicable to event cameras.

Inspired by [6] and[25], which recover local motion parameters by aligning events in a short-time IWE, we leverage CMax to formulate event-based BA as follows. The camera trajectory is described by a continuous-time model $\mathtt { R } ( t )$ . Each event $e _ { k }$ is warped according to its corresponding rotation $\mathtt { R } ( t _ { k } )$ , projected and accumulated into a global IWE. Then, we define the rotational BA problem as finding the camera trajectory $\mathtt { R } ( t )$ that maximizes event alignment (e.g., sharpness) of the global map, as measured by some objective function f [26]

$$
\underset { \mathbb { R } ( t ) } { \arg \operatorname* { m a x } } f \big ( I ( \mathbb { R } ( t ) ; \mathcal { E } ) \big ) .\tag{8}
$$

Solving (8) yields the refined camera trajectory $\mathtt { R } ^ { * } ( t )$ and its associated sharp map $M \equiv I ^ { * } ( { \mathrm { i } } . { \mathrm { e } }$ ., motion-compensated). Since the map is fully determined by the events and the camera trajectory, the search only needs to be carried out over the space of camera trajectories, as opposed to searching over a larger space of camera trajectories and maps.

1) Trajectory Parameterization: As presented in [17], due to the high-temporal resolution and asynchronous nature of the event camera it is suboptimal to estimate or refine its trajectory as a set of discrete-time poses. To this end, we represent the continuous-time camera trajectory in (8) using linear or cubic B-splines, which are parameterized by a set of temporally equispaced control poses in $S O ( 3 )$ . Using B-splines as a model of continuous-time trajectory has following advantages:

a) They reduce the number of pose states (a few control poses versus a different pose per event/timestamp);

b) they have a local support (each control pose has a limited influence on the overall trajectory);

c) they unify, allowing us to choose the degree of smoothness of the trajectories by varying one parameter (the order of the spline);

d) they enable the fusion of data from different sensors (synchronous and asynchronous), in a principled way, querying each sensor at the precise timestamp of their measurements, and

e) they also facilitate the estimation of the time offset between sensors.

The pose at any time of interest $\mathtt { R } ( t )$ is obtained by interpolation of a certain number of neighboring control poses $\mathtt { R } ( t _ { i } )$ For linear splines, the pose at $t \in [ t _ { i } , t _ { i + 1 } )$ is simply the linear interpolation in the Lie group sense of the two adjacent control poses $\mathtt { R } ( t _ { i } )$ and $\mathtt { R } ( t _ { i + 1 } )$ , given by [46]

$$
\mathsf { R } ( t ) = \mathtt { R } ( t _ { i } ) \exp \left( \frac { t - t _ { i } } { t _ { i + 1 } - t _ { i } } \log \left( \mathtt { R } ^ { \top } ( t _ { i } ) \mathtt { R } ( t _ { i + 1 } ) \right) \right) .\tag{9}
$$

Although linear splines are continuous, they have limited smoothness, hence, many control poses are required to achieve sufficient smoothness. Therefore, we also employ higher order splines: Cubic splines are smoother than linear ones.

In the case of cubic B-splines, the interpolation of the pose at $t \in [ t _ { i } , t _ { i + 1 } )$ requires four control poses, which occur at $\{ t _ { i - 1 } , \bar { t _ { i } } , t _ { i + 1 } , t _ { i + 2 } \}$ . Following the cumulative cubic B-spline formulation [17], [47], we write the camera trajectory as

$$
\mathtt { R } ( u ( t ) ) = \mathtt { R } _ { i - 1 } \prod _ { j = 1 } ^ { 3 } \exp \Big ( \tilde { \mathbf { B } } _ { j } ( u ( t ) ) \boldsymbol { \Omega } _ { i + j - 1 } \Big )\tag{10}
$$

where $u ( t ) \doteq ( t - t _ { i } ) / \Delta t \in [ 0 , 1 )$ is the normalized time representation, $\Delta t \doteq t _ { i + 1 } - t _ { i }$

$$
\Omega _ { i } \doteq \log \left( \mathbb { R } _ { i - 1 } ^ { \top } \mathbb { R } _ { i } \right)\tag{11}
$$

is the incremental pose between two consecutive control poses, and $\tilde { \mathbf { B } } _ { j }$ indicates the jth entry (0-based) of B<sup>˜</sup> , which is the matrix representation of the cumulative basis functions for the cubic B-splines

$$
\tilde { \mathbf { B } } ( u ) \doteq \frac { 1 } { 6 } \left( \begin{array} { c c c c } { 6 } & { 0 } & { 0 } & { 0 } \\ { 5 } & { 3 } & { - 3 } & { 1 } \\ { 1 } & { 3 } & { 3 } & { - 2 } \\ { 0 } & { 0 } & { 0 } & { 1 } \end{array} \right) \left( \begin{array} { c } { 1 } \\ { u } \\ { u ^ { 2 } } \\ { u ^ { 3 } } \end{array} \right) .\tag{12}
$$

We denote the control poses of the linear and cubic splines slightly differently $\left( \mathbb { R } ( t _ { i } ) \right)$ and $\mathrm { R } _ { i } .$ , respectively) because linear splines pass through the control points whereas cubic splines do not, in general.

Despite the elegance of the continuous-time trajectory representation, its high computational complexity (especially the derivative computation for trajectory optimization) presents a challenge for the application of cubic B-splines in trajectory estimation. We tackle this by adopting an efficient method recently presented in [48] to compute the analytical derivatives of an $S O ( 3 )$ spline trajectory with respect to its control poses. In this way, the complexity of the derivative computation becomes linear in the order of the spline, which promotes the efficiency of the proposed BA approach.

2) Panoramic IWE: The global map I in (8) is generated by warping and counting aligned events on a panoramic image. Every incoming event $e _ { k }$ is rotated according to its corresponding pose $\mathtt { R } ( t _ { k } )$ on the spline trajectory

$$
\mathbf { X } _ { k } ^ { \prime } = \mathsf { R } \left( t _ { k } \right) \mathbf { X } _ { k }\tag{13}
$$

where $\mathbf { X } _ { k } = \operatorname { K } ^ { - 1 } \big ( \mathbf { x } _ { k } ^ { \top } , 1 \big ) ^ { \top }$ is the bearing direction (3-D point) of $e _ { k }$ in calibrated camera coordinates, and K is the intrinsic parameter matrix of the camera. The rotated point $\mathbf { X } _ { k } ^ { \prime } = { }$ $( X _ { k } ^ { \prime } , Y _ { k } ^ { \prime } , Z _ { k } ^ { \prime } ) ^ { \top }$ is then projected onto the panorama using the equirectangular projection

$$
\begin{array} { r } { \mathbf { p } _ { k } \doteq \left( \begin{array} { c } { \frac { w } { 2 } + \frac { w } { 2 \pi } \arctan \left( \frac { X _ { k } ^ { \prime } } { Z _ { k } ^ { \prime } } \right) } \\ { \frac { h } { 2 } + \frac { h } { \pi } \arcsin \left( \frac { Y _ { k } ^ { \prime } } { \sqrt { X _ { k } ^ { \prime 2 } + Y _ { k } ^ { \prime 2 } + Z _ { k } ^ { \prime 2 } } } \right) } \end{array} \right) } \end{array}\tag{14}
$$

where w and h are the width and height of the panoramic map. The mapping $\mathbf { x } _ { k } \overset { \mathbf { W } } { \mapsto } \mathbf { p } _ { k }$ defines the event warping from the sensor image plane to the panoramic map.

Finally, the warped event is accumulated on the panoramic map I using bilinear voting since it may not have integer coordinates [6], [49]. Note that event polarities are not used.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/365d6a29c098bb2b0d2fa5901edc673db9b2a5b24d740f82bd75cfb3c232950a.jpg)  
Fig. 3. Overview of the proposed rotational event-based SLAM system.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/b8d3b8e29290d6ba025494204c66f93035c85ab42bd46cbd740efd41e6e12bfb.jpg)  
(a) Constant event number (K = 4)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/9d0158cafc71b26a1dadc4e79573df247f2c80505e6837f3478ab5f4e3115da0.jpg)  
(b) Constant duration (d = τ)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/824e9500406d59c10efe494a426b45bcdfc9a1c8fc0d0ad7dc3fd1b892ec350e.jpg)  
(c) Ours (K = 4, d = 1/f)  
Fig. 4. Three event slicing strategies: (a) Constant event count, (b) constant duration, and (c) proposed strategy that selects a fixed number of events around equispaced timestamps (with output frequency f). Green and yellow dots represent processed and skipped events, respectively. Blue arrows are the boundaries of the event slices. Red dashed lines indicate the selected timestamps for angular velocity estimation.

## C. CMax-SLAM System

The proposed rotational SLAM system consists of two parts (see Fig. 3). The front-end takes raw events as input and estimates the camera’s angular velocity ω using CMax [6]. The back-end integrates ω to obtain a set of (absolute) rotations that is used to initialize the camera trajectory, which is refined via the above BA method.

1) Front-End: We adopt the CMax-based angular velocity estimator (see Section II-B) as front-end of the proposed SLAM system and modify it to work with the back-end. This method assumes constant angular velocity within each packet (or “slice”) of events, so a proper slicing strategy of the event stream is critical. Two commonly-used possibilities are constant event count and constant time span [50]. The former creates packets of constant number of events K [see Fig. 4(a)], which would guarantee sufficient data for every angular velocity estimate; however, the output rate would vary wildly depending on the event rate. The latter strategy, with packets of fixed time duration d [see Fig. 4(b)], would guarantee a fixed rate of angular velocity estimates, but accuracy would be unstable (a slow-moving camera would produce few events per packet, which would not be enough to reliably estimate motion).

In our system, the back-end requires the front-end to provide pose estimates with constant rate and stable accuracy to achieve reliable trajectory initialization. Hence, we combine the above two strategies. As shown in Fig. 4(c), we select a series of equispaced timestamps (i.e., constant frequency) at which the angular velocity will be estimated, and then, slice a fixed number of events around each timestamp. This event slicing strategy is similar to that in [18]; the difference lies in the fact that our event packets are centered at the selected timestamps, while the packets in [18] consist of the events before the selected timestamps. If the time span of a single slice is too large (in our implementation, $1 0 / { \bar { f } }$ , where f is the angular velocity frequency), we assume the camera is not moving and, therefore, set the angular velocity to zero.

As depicted in Fig. 3, before event slicing we may perform an optional downsampling of the events, (e.g., keeping one out of Q events, where Q is the event sampling rate), to allow for flexibly trading off accuracy and speed.

2) Back-end: The BA introduced in (8) is described as trajectory refinement, in which we already have all event data and an initial trajectory to start the optimization search. Next, let us explain how to adapt (8) to an online SLAM system.

a) Trajectory refinement via a sliding window: There are two issues with the BA defined in (8) that prevent its direct application to the back-end of a real SLAM system. First, if the trajectory is temporally long, the amount of events and the number of control poses that need to be optimized will be very large, becoming intractable. Second, the initial trajectory is required to be close enough to the real one, otherwise the initial panoramic map will be blurred, which may cause BA optimization to get stuck in a local optimum. To this end, the BA in our back-end follows the structure of mature SLAM frameworks [23], working in a sliding-window fashion (fixed time window). Past control poses that fall outside the current time window are fixed, whereas new poses in the current time window are initialized and refined as follows.

Initialization: Once the angular velocities estimated by the front-end are available for the current time window, the backend integrates them into a set of poses (starting from the last optimized pose in the previous window) and initializes a new trajectory segment by fitting a spline curve to such front-end poses. Given the SO(3) nature of the problem, we follow a “lift–solve–retract” approach [46], [51] to solve for the control poses of the new trajectory segment

1) Lift: Use the first new front-end pose as offset, and compute the rotation increments with respect to it, so that the problem is lifted to the tangent vector space.

2) Solve: Form the linear system [52] in the tangent space, and solve for the control rotation increments.

3) Retract: Apply the offset to the control rotation increments, so that control poses in $S O ( 3 )$ are obtained.

Refinement: After initialization, all events in the current time window are projected onto the global, panoramic IWE. Following the optimization goal of the proposed BA (i.e., maximizing the contrast of the IWE), leads to an objective that only depends

on the camera trajectory

$$
\underset { \{ \mathtt { R } _ { i } \} _ { i \in \mathcal { R } } } { \operatorname { a r g m a x } } \ V \mathrm { a r } \left( I _ { L } \left( \left\{ \mathtt { R } _ { i } \right\} _ { i \in \mathcal { R } } , \mathcal { E } _ { \mathrm { c u r r } } \right) + I _ { G } \left( \mathcal { E } _ { \mathrm { p r e v } } \right) \right)\tag{15}
$$

where R is the set of indices of the varying control poses and ${ \mathcal E } _ { \mathrm { c u r r } }$ are the current events, which are projected to generate the local map $I _ { L }$ . The global map $I _ { G }$ is built from all past events $\mathcal { E } _ { \mathrm { p r e v } }$ and settled poses.

b) Balancing current and past events: The current segment of the camera trajectory is refined by solving the BA problem (15) using CG-FR. In practice, the number of events accumulated into the panoramic map increases rapidly as the camera rotates, which could lead to two issues. First, the influence of the local map $I _ { L }$ decreases with respect to the growing values of $I _ { G }$ , which may encourage local events to align to wrong edges of the global map. Second, if the camera kept moving around some regions while rarely observing others, the majority of events would accumulate in the over-observed regions. This could lead to a highly unbalanced edge map, especially for long-term tracking. The following strategies mitigate the above two issues.

Regarding the first issue, we replace $I _ { L } + I _ { G }$ in (15) by a weighted sum, $I _ { L } + \alpha I _ { G }$ , with α adaptively calculated from the event densities of $I _ { L }$ and $I _ { G }$ . The event density $\rho ( H )$ of an IWE H is defined as the ratio of the number of events $N _ { e } ( H )$ to the image area that they occupy, event area (EA) [26]

$$
\rho ( H ) \doteq N _ { e } ( H ) / \operatorname { a r e a } { ( H ) } .\tag{16}
$$

We adopt the formula [26]: area $( H ) = 1 - \exp ( - ( H ( \mathbf { x } ) / \lambda _ { 0 } )$ that is, pixels with $H ( \mathbf { x } ) \gtrsim \lambda _ { 0 }$ warped events contribute more to the area functional than those with $H ( \mathbf { x } ) \lesssim \lambda _ { 0 }$ . Empirically, we use $\lambda _ { 0 } = 1$ . From the event densities of $I _ { L }$ and $I _ { G }$ , the weight α is computed as

$$
\alpha \doteq \rho ( I _ { L } ) / \rho ( I _ { G } ) .\tag{17}
$$

Using (16) and (17), we may rewrite the objective function for the sliding-window CMax-based BA (15) as

$$
\underset { \{  { \mathbb { R } } _ { i } \} _ { i \in \mathcal { R } } } { \arg \operatorname* { m a x } } ~ \mathrm { V a r } \big ( I _ { L } \big ( \{ \mathtt { R } _ { i } \} _ { i \in \mathcal { R } } , \mathcal { E } _ { \mathrm { c u r r } } \big ) + \alpha I _ { G } \big ( \mathcal { E } _ { \mathrm { p r e v } } \big ) \big ) .\tag{18}
$$

Note that $I _ { L }$ would change during the optimization (and so would $\alpha )$ , however, the event density variations with respect to the initial $I _ { L }$ (aligned by the CMax-ω front-end) are small, hence, we use the latter to compute α and reutilize it during the optimization for each time window in (18).

Regarding the second issue, we may stop updating the pixels of $I _ { G }$ once we received sufficient evidence for them. In the implementation, we monitor how long each map pixel is observed and stop accumulating events onto the pixel once the duration reaches a threshold (e.g., 10 s in our experiments). A very low event rate in the current time window serves as a proxy to determine whether the camera is stationary, so that the BA is not performed, and consequently disregard such observation duration. Note that this mitigation strategy is optional. After the optimization, the image $I _ { G }$ contains the sharp map M produced by this approach.

For efficiency in (18), we interpolate poses at the midpoint of each small batch of events (e.g., 100 events) and use it for all events in the batch, which reduces a lot the number of times for querying the camera pose and its derivatives in the spline trajectory. This does not spoil the nature of continuous-time trajectory because the event rate is usually much higher (the average time span of a batch of 100 event is usually less than 0.5 ms). As shown in Fig. 3, we can also perform (optional) event downsampling for the back-end, to tradeoff accuracy and speed depending on the application.

c) Analytical derivatives and locality exploitation: Derivatives are analytically computed during optimization

$$
{ \frac { \partial } { \partial \Phi } } \mathrm { V a r } \left( I \left( \mathbf { p } ; \Phi \right) \right) = { \frac { 1 } { \left| \Omega \right| } } \int _ { \Omega } 2 \rho ( \mathbf { p } ; \Phi ) { \frac { \partial \rho ( \mathbf { p } ; \Phi ) } { \partial \Phi } } d \mathbf { p }\tag{19}
$$

where $\rho ( \mathbf { p } ; \Phi ) \doteq I ( \mathbf { p } ; \Phi ) - \mu ( I ( \mathbf { p } ; \Phi ) )$ and $\Phi \doteq \{ \mathtt { R } _ { i } \} _ { i \in \mathcal { R } }$ is the set of all involved control poses. By the chain rule

$$
{ \displaystyle \frac { \partial I ( { \bf p } ; \Phi ) } { \partial \Phi } = - \sum _ { k } \frac { \partial I } { \partial { \bf p } _ { k } } \ \frac { \partial { \bf p } _ { k } } { \partial { \bf X } _ { k } ^ { \prime } } \ \frac { \partial { \bf X } _ { k } ^ { \prime } } { \partial \Phi } } .\tag{20}
$$

Due to the locality of B-splines, the contribution of each event to the panoramic IWE only depends on a few neighboring control poses $( \mathrm { e . g . }$ , two for linear splines and four for cubic ones [17]). Therefore, for each event, the Jacobian $\partial { \bf X } _ { k } ^ { \prime } / \partial \Phi$ in (20) is sparse, with only few nonzero elements, which is beneficial for speeding up the computation of the derivatives.

While $I = I _ { L } + \alpha I _ { G } \mathrm { i n } ( 1 8 )$ , note that the derivative (20) only needs to be computed for $I _ { L }$ , since $I _ { G }$ does not depend on the varying poses Φ $( \mathrm { i . e . }$ , its derivative vanishes).

## D. Discussion

The CMax-based BA defined in (8) is an ideal model. It suggests the “best” theoretical trajectory under assumptions of purely rotational motion, perfect representation of the continuous camera motion, perfect projection model (including calibration), and lack of noise, which are unachievable conditions in the real world. We have proposed several techniques (i.e., approximations) to turn (8) into a tractable model (18).

The CMax-SLAM system does not have a separate mapping part, and the maintained global panoramic map is just a byproduct, which is only used for trajectory refinement. The system also does not rely on an explicit bootstrapping step.

The consistency between front-end and back-end is also notable. Both have the same goal (CMax), albeit with different motion parameterizations: Constant angular velocity for short intervals (front-end) versus arbitrary continuous-time rotation (spline-based) for longer time intervals (back-end). In both cases, the IWE acts as an edge-like brightness map that is used for motion estimation.

While [22] also uses CMax, it operates on each incoming time interval of events (of 25 ms), estimating the angular velocity and absolute rotation for that interval alone. It works on a discrete set of poses that are updated one at a time (7). Hence, it has no continuous trajectory model and no capability to refine past poses (i.e., a back-end).

## IV. EXPERIMENTS

This section presents a comprehensive comparative evaluation of the rotation-only motion estimators reviewed in Section II, as well as our proposed BA and CMax-SLAM system. First, we introduce the experimental setup, including datasets (see Section IV-A) and metrics (see Section IV-B). Then, we present the results on synthetic (see Section IV-C) and real-world data (see Section IV-D), discussing the issues of the latter. We also assess the runtime of the methods (see Section IV-E), demonstrate

CMax-SLAM in complex scenes (see Section IV-F) and show its super-resolution capabilities (see Section IV-G). Finally, we present a sensitivity analysis (see Section IV-H).

## A. Experimental Setup

1) Datasets: To evaluate the accuracy and robustness of rotation estimation, we conduct experiments on six synthetic sequences as well as six real-world sequences from standard datasets [22], [53]. All of them contain events, frames (not used), inertial measurement unit (IMU) data, and groundtruth (GT) poses.

The synthetic sequences are generated using the panoramic renderer of the ESIM simulator [54], with input panoramas downloaded from the Internet. The generated sequences cover indoor, outdoor, daylight, night, human-made, and natural scenarios. The sizes of the input panoramas vary from 2 K (playroom), 4 K (bicycle), 6 K (city and street), to 7 K (town and bay) resolution. In the simulation, playroom is a classical sequence, generated with a DVS128 camera model (128 × 128 px) and a duration of 2.5 s, whereas the other five sequences are generated with a DAVIS240C camera model (240 × 180 px) and a duration of 5 s.

All six real-world datasets are recorded by DAVIS240C event cameras, whose IMU operates at 1 kHz. The Event Camera Dataset (ECD) [53] provides four 60 s long rotational motion sequences: shapes, poster, boxes, and dynamic. The first three sequences feature static indoor scenes with increasing texture complexity (yielding about 20–200 million events), whereas the last one presents a dynamic scene (≈ 70 million events). The camera moves with increasing speed, first rotating around each axis and, then, rotating freely in 3-DOFs. A motion capture (mocap) system outputs GT poses with millimeter precision at 200 Hz. For these four ECD sequences, we use the first 30 s of data for accuracy evaluation, where the magnitude of translational motion is relatively small. Two sequences from [22] are also considered: 360<sup>◦</sup> indoor and fast motion. Both of them consist of ≈ 4.5 million events, with mocap GT poses given at 100 Hz. In sequence 360<sup>◦</sup> indoor, the camera rotates 360<sup>◦</sup> around the vertical axis, while in fast motion, the camera performs random fast rotations throughout the whole sequence. Since the GT from the mocap(s) is noisy, it is filtered with a Gaussian low-pass filter in the Lie group sense using a time span of 50 ms.

2) Hardware Configuration: All experiments are conducted on a standard laptop (Intel Core i7-1165G7 CPU @ 2.80GHz), with the exception of those involving RTPT [15], which runs on another laptop with an NVIDIA GeForce GTX 970M GPU due to software incompatibilities (running 2017 CUDA code).

3) Downsampling: Event downsampling is only enabled in the sensitivity study on the event sampling rate (see Section IV-H4), and the nearly real-time demonstration shown in the video of the Supplementary Material (see Section IV-F). In all other experiments of our methods, all input events are processed.

## B. Evaluation Metrics

To comprehensively characterize the performance of the methods, we evaluate both the output trajectories and obtained maps. For the former, we calculate the absolute and relative errors of the estimated rotations with respect to the GT. For the latter, we compute panoramic maps (i.e., IWEs) using the trajectories output by each approach, and extend the reprojection error from feature-based SLAM, to assess the quality of trajectories and maps.

1) Absolute Error: The absolute rotation error quantifies the global consistency of the estimated camera poses [55]. At timestamp $t _ { k }$ , the absolute rotation error $\Delta \mathtt { R } _ { k }$ is given by

$$
\Delta \mathrm { R } _ { k } = \mathrm { R } _ { k } ^ { \prime \top } \mathrm { R } _ { k }\tag{21}
$$

where $\mathtt { R } _ { k }$ is the estimated rotation at $t _ { k }$ and $\mathtt { R } _ { k } ^ { \prime }$ is the corresponding GT rotation (obtained by linear interpolation). In the benchmark, each method outputs rotations at a different rate, hence, we calculate a series of absolute errors at these timestamps when the rotation is estimated, and further compute the root mean square (RMS) to represent the accuracy. To evaluate the continuous trajectories refined by the proposed BA, we query poses and compute errors at 50 Hz.

2) Relative Error: The relative rotation error measures the local accuracy of the pose estimates [55]. At timestamp $t _ { k }$ , the relative rotation error $\delta \mathtt { R } _ { k }$ is

$$
\delta \mathtt { R } _ { k } = \left( \mathtt { R } _ { k } ^ { \prime \top } \mathtt { R } _ { k + \Delta } ^ { \prime } \right) ^ { - 1 } \left( \mathtt { R } _ { k } ^ { \top } \mathtt { R } _ { k + \Delta } \right)\tag{22}
$$

where $\left\{ \mathbb { R } _ { k } , \mathbb { R } _ { k + \Delta } \right\}$ and $\{ \mathrm { R } _ { k } ^ { \prime } , \mathrm { R } _ { k + \Delta } ^ { \prime } \}$ are pairs of estimated and GT poses, respectively, which are selected based on some criterion $\Delta$ . We set the pose pairs to have a time span of 1 s, measuring the error produced per second, and sample the trajectory to get these pose pairs every 0.1 s.

In the experiments, we use the angle of the difference between two rotations [46] to measure estimation errors

$$
\angle \mathrm { B } = \operatorname { a r c c o s } \left( { \frac { \operatorname { t r a c e } ( \mathrm { B } ) - 1 } { 2 } } \right)\tag{23}
$$

where B is $\Delta \mathtt { R } _ { k }$ or $\delta \mathtt { R } _ { k } .$ , as adopted in [56].

In all experiments, we align the output trajectories to the GT at $t _ { 0 }$ before evaluation: $\hat { \mathrm { R } } _ { k } = \mathrm { R } ^ { \prime } ( t _ { 0 } ) \mathrm { R } ^ { \top } ( t _ { 0 } ) \mathrm { R } _ { k } .$ , where $\hat { \sf R } _ { k }$ is the aligned pose and $\mathtt { R } ^ { \prime } ( t _ { 0 } )$ is the GT pose at $t = t _ { 0 }$ . For synthetic and real-world sequences, t<sub>0</sub> is set to 0.1 s and 1.0 s, respectively.

3) Reprojection Error: For real-world data, where the camera motion is not purely rotational, problems arise to characterize rotational errors. As will be explained in Section IV-D1, we propose assessing the performance of the methods by means of the reprojection error. Without GT data association, we will argue in favor of using the EA [26] of the panoramic map as a proxy for the reprojection error. We use the percentage of EA with respect to the total image area, as it is more intuitive to understand.

We also report the gradient magnitude (GM) [26] of the IWE

$$
\mathrm { G M } ( I ) = \bigg ( \frac { 1 } { N } \int _ { \Omega } \| \nabla I ( \mathbf { x } ) \| ^ { 2 } d \mathbf { x } \bigg ) ^ { \frac { 1 } { 2 } }\tag{24}
$$

where N is the number of pixels and $\nabla I = ( I _ { x } , I _ { y } ) ^ { \top }$ is the spatial gradient of I (e.g., calculated using Sobel’s operator).

## C. Experiments on Synthetic Data

In this section, we compare the front-end methods, the proposed BA and CMax-SLAM on the six synthetic sequences. More details of the front-ends are given in Appendix B.

TABLE II  
ABSOLUTE [<sup>◦</sup>] AND RELATIVE [<sup>◦</sup>/S] RMSE ON SYNTHETIC DATASETS
<table><tr><td rowspan="2"></td><td rowspan="2">Sequence</td><td colspan="2">playroom</td><td colspan="2">bicycle</td><td colspan="2">city</td><td colspan="2">street</td><td colspan="2">town</td><td colspan="2">bay</td></tr><tr><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td></tr><tr><td rowspan="4">Front-end Sec. IV-C1</td><td>PF-SMT [12] EKF-SMT [57]</td><td>9.038 6.059</td><td>6.457</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td></td><td></td><td>4.270</td><td>1.382</td><td>0.935</td><td>1.715</td><td>1.376</td><td>3.481</td><td>1.706</td><td>4.331</td><td>1.599</td><td>2.649</td><td>1.908</td></tr><tr><td>RTPT [15] CMax-GAE [6]</td><td>4.794</td><td>4.178</td><td>1.666</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>CMax-ω front-end [6]</td><td>3.228</td><td>2.641</td><td>1.731</td><td>1.397 1.576</td><td>5.322</td><td>3.988</td><td>1.917</td><td>1.712</td><td>6.970 1.910</td><td>4.568 1.887</td><td>3.696</td><td>3.078</td></tr><tr><td rowspan="3">BA: linear Sec. IV-C2</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>PF-SMT [12]</td><td>0.631</td><td>1.140</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td></tr><tr><td>EKF-SMT [57] CMax-GAE [22]</td><td>0.250 1.588</td><td>0.257 0.869</td><td>0.434 0.637</td><td>0.289 0.538</td><td>0.299 N/A</td><td>0.321 N/A</td><td>0.331 N/A</td><td>0.334</td><td>0.954</td><td>0.507 0.855</td><td>0.476 N/A</td><td>0.353 N/A</td></tr><tr><td rowspan="3">BA: cubic Sec. IV-C2</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>N/A</td><td>1.640</td><td></td><td></td><td></td></tr><tr><td>PF-SMT [12]</td><td>10.242</td><td>8.518</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td></tr><tr><td>EKF-SMT [57] CMax-GAE [22]</td><td>1.265 1.274</td><td>0.774 1.789</td><td>0.627</td><td>0.510</td><td>0.312</td><td>0.359</td><td>0.508</td><td>0.538</td><td>2.031</td><td>1.135</td><td>2.912</td><td>0.659</td></tr><tr><td></td><td></td><td></td><td></td><td>1.004</td><td>0.737</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>1.337</td><td>0.880</td><td>N/A</td><td>N/A</td></tr><tr><td>System</td><td>CMax-SLAM (linear)</td><td>0.763</td><td>0.593</td><td>0.327</td><td>0.414</td><td>0.509</td><td>0.721</td><td>0.470</td><td>0.584</td><td>0.553</td><td>0.554</td><td>0.617</td><td>0.475</td></tr><tr><td>Sec. IV-C3</td><td>CMax-SLAM (cubic)</td><td>0.796</td><td>0.538</td><td>0.368</td><td>0.472</td><td>0.499</td><td>0.621</td><td>0.412</td><td>0.524</td><td>0.541</td><td>0.527</td><td>0.674</td><td>0.424</td></tr></table>

“" means the method fails on that sequence, and “N/A" indicates that BA refinement is not applicable because the corresponding front-end failed on this sequence

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/79454c1582def19d4dacef90e46dbaa50a167a52df491e56c99562fc0dff97ec.jpg)  
Fig. 5. Effect ofBA (offline smoothing). Parts of the panoramic IWEs obtained with the estimated trajectories (before / after BA refinement) and GT. Synthetic data, as in Table II. Gamma correction γ = 0.75 applied for better visualization.(a) Scenes. (b) EKF-SMT trajectories. (c) Refined EKF-SMT (linear). (d) CMax-GAE trajectories. (e) Refined CMax-GAE (linear). (f) Groundtruth.

1) Comparison ofFront-Ends: First, we benchmark all frontends with synthetic data. The top part of Table II reports the accuracy of the corresponding camera trajectories.

SMT: Overall, EKF-SMT achieves the best performance among all front-end methods, whereas PF-SMT only completes tracking forplayroom. Experimentally, EKF-SMT is more stable than PF-SMT and also less sensitive to the map quality.

RTPT: Due to the probabilistic operation of RTPT, it may output different results in different trials. Hence, we report the best results we obtained. RTPT fails on all synthetic sequences because of its limitation on the range of camera motions that can be tracked. It is due to the fact that RTPT monitors the tracking quality during operation and stops the map update when the quality decreases below a threshold, which often happens if the camera’s FOV gets close to the left or right boundary of the panoramic map.

CMax: CMax-ω achieves the best absolute errors in three out of six sequences, which demonstrates the effectiveness of the event slicing strategy (see Section III-C1). CMax-GAE fails city, street, and bay, which have highly textured areas to trigger a massive amount of events. On these sequences, events are densely clustered on the map of CMax-GAE [see Fig. 2(d)], which presents an obstacle for its optimizer to align events.

2) Back-end (BA): Next, we use the proposed BA method to smooth the trajectories estimated by the front-end methods that do not lose track. The refinement is conducted offline, in a sliding window manner (as described in Section III-C2).

As reported in the middle part of Table II, the proposed BA is able to decrease both the absolute and relative RMSE of all front-ends that do not lose track. For example, for bicycle, the absolute RMSE ofEKF-SMT is reduced from 1.382<sup>◦</sup> (front-end) to 0.434<sup>◦</sup> (BA: linear) or 0.627<sup>◦</sup> (BA: cubic). The trajectories estimated by CMax-GAE are also considerably refined by our BA method. The improvement in the refinement of the trajectories is also noticeable in the quality of the map, as shown in Fig. 5; the IWEs look sharper after refinement.

3) CMax-SLAM: Finally, the bottom part of Table II reports the accuracy numbers of the proposed CMax-SLAM system on the same sequences. CMax-SLAM outperforms all baseline methods on all synthetic sequences. In addition, there is an obvious improvement from CMax-ω to CMax-SLAM in all trials. This further supports the effectiveness of the proposed

TABLE III  
ABSOLUTE [<sup>◦</sup>] AND RELATIVE [<sup>◦</sup>/S] RMSE ON REAL-WORLD DATASETS
<table><tr><td rowspan="3"></td><td rowspan="3">Sequence</td><td colspan="2">shapes [53]</td><td colspan="2">poster [53]</td><td colspan="2">boxes [53]</td><td colspan="2">dynamic [53]</td><td colspan="2">360 indoor [22]</td><td colspan="2">fast motion [22]</td></tr><tr><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td><td>Abs</td><td>Rel</td></tr><tr><td rowspan="6">Front-end Sec. IV-D2</td><td>PF-SMT [12]</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>10.562</td><td>12.041</td><td></td><td></td></tr><tr><td>EKF-SMT [57]</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>11.317</td><td>7.596</td><td>2.358</td><td>1.656</td></tr><tr><td>RTPT [15]</td><td>3.224</td><td>3.167</td><td>8.135</td><td>9.423</td><td>2.973</td><td>2.375</td><td>1.873</td><td>1.991</td><td></td><td></td><td>10.566</td><td>14.111</td></tr><tr><td>CMax-GAE [22]</td><td>3.961</td><td>2.777</td><td>4.010</td><td>3.612</td><td></td><td></td><td>4.840</td><td>1.755</td><td>5.145</td><td>1.574</td><td>4.010</td><td>3.850</td></tr><tr><td>CMax-ω front-end [6]</td><td>7.164</td><td>6.536</td><td>7.263</td><td>5.985</td><td>9.241</td><td>6.846</td><td>5.535</td><td>3.696</td><td>7.889</td><td>1.609</td><td>3.519</td><td>2.485</td></tr><tr><td>IMU dead reckoning</td><td>46.803</td><td>6.470</td><td>45.743</td><td>5.746</td><td>46.979</td><td>6.911</td><td>44.531</td><td>4.183</td><td>63.396</td><td>9.156</td><td>15.236</td><td>10.035</td></tr><tr><td rowspan="2">BA Sec. IV-D3</td><td>IMU (linear)</td><td>4.939</td><td>7.159</td><td>5.624</td><td>6.658</td><td>5.423</td><td>7.063</td><td>3.289</td><td>3.643</td><td>4.579</td><td>1.370</td><td>1.695</td><td>2.844</td></tr><tr><td>IMU (cubic)</td><td>4.934</td><td>6.791</td><td>5.716</td><td>6.803</td><td>5.393</td><td>6.963</td><td>3.337</td><td>3.645</td><td>4.990</td><td>1.380</td><td>2.053</td><td>3.053</td></tr><tr><td>System</td><td>CMax-SLAM (linear)</td><td>4.953</td><td>6.712</td><td>5.653</td><td>6.357</td><td>5.418</td><td>6.751</td><td>3.380</td><td>3.586</td><td>5.046</td><td>1.564</td><td>1.322</td><td>2.182</td></tr><tr><td>Sec. IV-D4</td><td>CMax-SLAM (cubic)</td><td>4.974</td><td>6.821</td><td>5.674</td><td>6.497</td><td>5.391</td><td>6.761</td><td>3.374</td><td>3.618</td><td>5.002</td><td>1.578</td><td>1.320</td><td>2.038</td></tr></table>

“-"means the method fails on that sequence.

BA back-end, which works in an online manner here. The linear and cubic spline trajectories report similar accuracy. In short, CMax-SLAM is more accurate than all previously existing rotation estimation methods.

## D. Experiments on Real-World Data

This section compares the front-end methods, the proposed BA and CMax-SLAM on real-world data.

1) Evaluation Issues on Real-world Data: The main difficulty of real-world evaluation lies in finding real data that is compatible with the purely rotational motion assumption of the problem. Real-world sequences from established datasets [22], [53] were recorded hand-held, hence, they contain some translational motion. Meanwhile, all methods impose the rotational motion constraint. Hence, problems arise when testing 3-DOF methods on nonstrictly 3-DOF data.

In the input events, one cannot disentangle the rotational part from the translational part. Hence, by design, all methods explain the additional DOFs in the events using only rotational DOFs. If the translational motion is nonnegligible, comparing the rotations that explain additional DOFs to the rotational component of the GT provided by a 6-DOF mocap system [22], [53] can be misleading (increased RMSE) and inconsistent with the visual results (e.g., maps with double and blurred edges). Therefore, a more sensible FOM is needed to characterize rotation estimation methods.

In classical BA, the reprojection error (in the image domain) is the FOM that measures the goodness of fit between the unknowns (scene structure/map and camera motion) and the visual data. Lifting the visual data onto 3-D through noisy poses leads to multiple misaligned copies of the same scene part, which when projected gives rise to reprojection errors. In our event-based rotational motion scenario, such a misalignment is noticeable on the panoramic map (IWE) in the form of blurred edges or multiple copies of the same scene edge (as seen on synthetic data in Fig. 5). Hence, the thickness of the edges or area occupied by the warped events on the IWE (i.e., the EA [26]) is a proxy for the reprojection error. The approximation character is due to the fact that reprojection errors require explicit data association (indicating which events correspond to the same edge), but since there is no GT data association we adopt the implicit and soft data association of the IWE: The closer the warped events, the higher their correspondence [25]. The proxy reprojection error highlights better the issues ofthe BA problem considered, hence, we use it to guide our discussion in the upcoming experiments.

2) Comparison of Front-Ends: The top part of Table III reports the RMSE of the trajectories estimated by all front-ends on real-world sequences. In addition, Fig. 6 depicts the trajectories produced by all front-ends on the ECD sequences [53], compared against the GT, while the corresponding error statistics are displayed in Fig. 7.

SMT: 360<sup>◦</sup> indoor is the only sequence that PF-SMT is able to track completely. EKF-SMT reports the highest accuracy on fast motion, whereas shows similar performance as PF-SMT on 360<sup>◦</sup> indoor. However, both SMT variants fail on all ECD sequences. As shown in Fig. 6, SMT tracks accurately at the beginning (thanks to the stable IMU initialization), but then it loses track suddenly at some point, instead of accumulating drift (like the IMU dead reckoning does). Tracking failure happens mostly when the camera changes the rotation direction abruptly (e.g., turning back). We suspect it is due to the error propagation between the tracking and mapping threads. Small errors in the poses or the map are amplified, corrupting the states and their uncertainty in the Bayesian filters.

RTPTmanages to track well the sequences where the camera’s FOV moves around the center of the panoramic map [53]. However, once the camera explores a larger region, RTPT reports dramatically increased errors (e.g., fast motion), or even loses tracking (e.g., 360<sup>◦</sup> indoor). Again, this evidences a limitation on the range of trackable camera motions.

CMax: Our CMax-ω front-end computes accurate angular velocities, producing good results on all sequences. As shown in Fig. 6, it is clearly better than IMU integration. On real-world data, it shows robustness and does not suffer from dramatic increase in errors. CMax-GAE jointly estimates angular velocities and absolute rotations, which endows better consistency for long-term tracking, as shown by its competitive performance on most real-world sequences. Due to reason described in Section IV-C1, it fails on boxes, where the texture of the scene is higher than in others.

3) Back-end (BA): Next, our offline BA approach is tested on real-world data. Its improvement is most salient on camera trajectories that drift considerably from the GT, like IMU dead reckoning. Fig. 8 shows the estimated camera trajectories of the ECD sequences before/after refinement. The refined trajectories are very close to the GT and have smaller and more concentrated absolute error distributions than the initial ones (see Fig. 9 and the middle part of Table III). The relative errors do not change considerably, which is reasonable because they approximately measure the error in angular velocity, and the IMU provides accurate angular velocity data.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/f75460289bb8d27b58100c4f000bd78fa10556a42bb8881e119278c179eeeb34.jpg)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/296cefaa36cebbb25e1115aeaba98a6a80dfb876ff149cbdfb254c27951c4f8c.jpg)  
(a) shapes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/8ff7a4b737a5d1ed57ad89f6691bf300644894fffbdda3a73f01f1817e7007e2.jpg)  
(b) poster

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/589118ac4fa30cdac2f842ea407c78e25eb068bc5d0ba26a259575a92c487e78.jpg)  
(c) boxes  
(d) dynamic  
Fig. 6. Front-ends (before BA). Comparison of camera trajectories of all front-end methods involved in the benchmark. EKF-SMT and PF-SMT do not show up because they fail all sequences from ECD [53] dataset. (a) shapes. (b) poster. (c) boxes. (d) dynamic.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/86bb561ce2a33a5a68a85768731f57a0be2b780cd86902790ed9cb6a2d7b526e.jpg)  
(a) shapes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/fae7345bf685a197ce3761b8787800cff9e3008c0fd6cc2cf6a34f8e7b2fc4db.jpg)  
(b) poster

Relative rotation error [deg/s]  
![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/06417ff027699cb399e3df48b5543c714304101c02feab821863711e8f6f1d07.jpg)  
(c) boxes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/fa04820b082b7ea91fcdccb2b299141768c8d99e2e03ebd707a8000a7c026e69.jpg)  
(d) dynamic  
Fig. 7. Front-ends (before BA). Absolute and relative errors of the visual front-end methods. SMT is not included since it does not work on most sequences. (a) shapes. (b) poster. (c) boxes. (d) dynamic.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/6277f8dad940f4e52690f89d9e0a01509c19eb4b637113801b150ab200c96299.jpg)  
(a) shapes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/73eae149d5abc3fb6d75f792ebcedf92ff651e27d3698c6a78c9cde978cd7824.jpg)  
(b) poster

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/1a306c41f2bd2f35227ec3b3142ad39964974acc307e7f236300702ed2364dd5.jpg)  
(c) boxes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/690a7c679b31a051b299ca0c4f1a2295489595893544481619fa1a13728a13ed.jpg)  
(d) dynamic  
Fig. 8. Camera trajectories: IMU dead reckoning and its refined trajectories (linear and cubic) using BA. (a) shapes. (b) poster. (c) boxes. (d) dynamic.

A qualitative comparison of the maps associated to the rotations before/after BA refinement is given in Fig. 10. The maps become considerably sharper with the refinement. Table IV reports that the corresponding reprojection errors (EA) also decrease after BA refinement. For comparison, the last column of Fig. 10 shows the maps obtained with the GT rotations; they are not sharp (because the camera motion is not purely rotational), which is also noticeable in higher reprojection errors (EA) than for the refined rotations.

Similar observations apply to the refined trajectories from the visual front-ends. Fig. 11 compares the grayscale maps obtained using the trajectories estimated by CMax-GAE and RTPT before/after linear BA refinement, as well as the GT rotations. The grayscale maps are obtained by feeding the trajectory and events to the mapping module of SMT. The CMax-GAE, RTPT, and GT rotations result in blurred grayscale panoramas (columns a, c, and e). In contrast, the proposed BA improves the alignment of the visual data (columns b and d). Quantitatively, this is supported by the reduction of the proxy reprojection error (EA) in Table IV.

Absolute rotation error [deg]  
![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/9e0931a1c62137907eef91dc20bca9219f876cf791fcbcc8d11fa76baee35719.jpg)  
(a) shapes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/e99c3f9e4f573aaebc8540292fb43a7a991493185dd2c38f7f6095caea8b6c64.jpg)  
(b) poster

Relative rotation error [deg/s]  
![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/37705fae5cdccae69324274f24dbac5a2c3c8c9b53cac1e5e8fca066ae096240.jpg)  
(c) boxes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/5cee44018ecc5d9c57cc635ac27a82316c1bc40cdc4b17eb38f4a741a71b77c9.jpg)  
(d) dynamic

Fig. 9. Refinement of IMU dead reckoning using BA (offline smoothing). Absolute and relative errors of the input and refined trajectories. “l” and “c” indicate the BA algorithm with linear and cubic splines, respectively. (a) shapes. (b) poster. (c) boxes. (d) dynamic.  
![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/6a4a5b07ef5d61b3720a2aff67f7bc1c4ba855a89136e0201a6f3b236a4c0a80.jpg)  
(a) Scene  
(b) IMU dead reckoning  
(c) Refined trajectories (linear splines)  
(d) Refined trajectories  
(cubic splines)  
(e) Groundtruth  
(only rotation)  
Fig. 10. Effect of BA (offline smoothing). Central part of the panoramic IWEs generated using the estimated trajectories (before/after BA refinement) and GT Gamma correction: γ = 0.75. Data from ECD [53], using events in [1,11]s. (a) Scene. (b) IMU dead reckoning. (c) Refined trajectories (linear splines). (d) Refined trajectories (cubic splines). (e) Groundtruth (only rotation).

4) CMax-SLAM: Finally, we test the proposed CMax-SLAM system on the same sequences. The estimated trajectories and errors are plotted in Figs. 12 and 13, RMSE numbers are given in the bottom part of Table III, and proxy reprojection errors (EA) are given in Table IV. The trajectories estimated by CMax-SLAM are very close to the GT (see Fig. 12). As the violin plots in Fig. 13 show, in most cases, the absolute errors decrease and their distributions become more concentrated than with respect to the CMax-ω front-end. The effect is most pronounced on shapes and dynamic, where the amount of translational motion is smaller than poster and boxes. The relative errors remain almost unchanged, thus, not spoiling the accurate angular velocities estimated by the front-end. In terms of proxy reprojection errors, CMax-SLAM achieves the smallest values on all ECD sequences (see Table IV).

Remark: Table IV also reports the GM (24) of the panoramic IWE. It provides a measure of IWE sharpness different from the one used in the optimization (i.e., variance). For completeness, Table IV also reports the values of synthetic sequences bicycle and town. The numbers show that, both in real-world and synthetic data, the criteria are aligned: the sharper the IWE (the larger the GM), the smaller the proxy reprojection errors (EA).

TABLE IV  
PROXY REPROJECTION ERROR GIVEN BY THE EA OF THE PANORAMIC IWE
<table><tr><td rowspan="2"></td><td rowspan="2">Sequence</td><td colspan="2">bicycle (synth)</td><td colspan="2">town (synth)</td><td colspan="2">shapes (real)</td><td colspan="2">poster (real)</td><td colspan="2">boxes (real)</td><td colspan="2">dynamic (real)</td></tr><tr><td>EA [%] ↓</td><td>GM↑</td><td>EA [%] ↓</td><td>GM ↑</td><td>EA [%] ↓</td><td>GM↑</td><td>EA [%] ↓</td><td>GM↑</td><td>EA [%] ↓</td><td>GM↑</td><td>EA [%] ↓</td><td>GM↑</td></tr><tr><td rowspan="4">Front-end Sec. IV-C1 Sec. IV-D2</td><td>EKF-SMT [57]</td><td>4.390</td><td>47.693</td><td>8.452</td><td>108.893</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>RTPT [15]</td><td>4.424</td><td>48.474</td><td></td><td></td><td>0.459</td><td>14.448</td><td>3.309</td><td>22.231</td><td>2.942</td><td>15.744</td><td>2.167</td><td>17.143</td></tr><tr><td>CMax-GAE [22] IMU dead reckoning</td><td>4.152</td><td>65.198</td><td>8.082 7.393</td><td>115.517 172.964</td><td>0.514 0.604</td><td>10.186 5.467</td><td>3.412 3.601</td><td>16.718 7.830</td><td>2.980 3.146</td><td>12.432 9.371</td><td>2.273</td><td>12.611 10.507</td></tr><tr><td>Groundtruth</td><td>4.152</td><td>65.216</td><td>7.391</td><td>173.055</td><td>0.532</td><td>6.410</td><td>3.396</td><td>9.648</td><td>2.957</td><td>11.812</td><td>2.473 2.222</td><td>12.602</td></tr><tr><td></td><td>EKF-SMT [57]</td><td>4.163</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>BA: linear</td><td>RTPT [15]</td><td>N/A</td><td>64.634 N/A</td><td>7.453</td><td>169.618</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td></tr><tr><td>Sec. IV-C2</td><td>CMax-GAE [22]</td><td>4.185</td><td>63.472</td><td>N/A 7.506</td><td>N/A 165.736</td><td>0.332 0.334</td><td>35.187</td><td>3.038</td><td>46.776</td><td>2.724</td><td>38.690</td><td>1.856</td><td>46.882</td></tr><tr><td>Sec. IV-D3</td><td>IMU dead reckoning</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>0.331</td><td>34.818 35.389</td><td>3.039</td><td>46.679</td><td>2.742</td><td>37.055</td><td>1.859</td><td>46.373 47.813</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>3.041</td><td>46.557</td><td>2.724</td><td>38.695</td><td>1.848</td><td></td></tr><tr><td>BA: cubic</td><td>EKF-SMT [57]</td><td>4.210</td><td>61.765</td><td>7.623</td><td>159.119</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td><td>N/A</td></tr><tr><td>Sec. IV-C2</td><td>RTPT [15]</td><td>N/A 4.205</td><td>N/A</td><td>N/A</td><td>N/A</td><td>0.335</td><td>34.587</td><td>3.041</td><td>46.420</td><td>2.733</td><td>37.787</td><td>1.878</td><td>43.811</td></tr><tr><td>Sec. IV-D3</td><td>CMax-GAE [22] IMU dead reckoning</td><td>N/A</td><td>62.256 N/A</td><td>7.561 N/A</td><td>161.898 N/A</td><td>0.344 0.333</td><td>32.861</td><td>3.054</td><td>45.299</td><td>2.878</td><td>26.509</td><td>1.906</td><td>40.789</td></tr><tr><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td>34.965</td><td>3.064</td><td>44.208</td><td>2.725</td><td>38.623</td><td>1.855</td><td>46.932</td></tr><tr><td>Sec. IV-C3</td><td>CMax-SLAM (linear)</td><td>4.166</td><td>64.305</td><td>7.494</td><td>166.333</td><td>0.332</td><td>35.243</td><td>3.037</td><td>46.961</td><td>2.725</td><td>38.602</td><td>1.850</td><td>47.443</td></tr><tr><td>Sec. IV-D4</td><td>CMax-SLAM (cubic)</td><td>4.166</td><td>64.244</td><td>7.471</td><td>168.254</td><td>0.340</td><td>33.977</td><td>3.040</td><td>46.708</td><td>2.727</td><td>38.378</td><td>1.862</td><td>45.882</td></tr></table>

"" means the method fails on a sequence (same as in Tab. III), and "N/A" indicates that the BA refinement is not applicable because the corresponding front-end failed on this sequence. BA is marked as "N/A" on trajectories from IMU dead reckoning on synthetic data because here the IMU is noise-free.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/83821868daf33993b63a1f53d6ab967dfd90acf64400d012ab7dc2006773f862.jpg)  
(a) CMax-GAE trajectories  
(b) Refined CMax-GAE  
trajectories, linear splines  
(c) RTPT trajectories  
(d) Refined RTPT  
trajectories, linear splines  
(e) Groundtruth (only rotation)  
Fig. 11. Effect of BA (offline smoothing). Central part of the panoramic grayscale maps generated using the estimated trajectories (before/after BA refinement) and GT. The grayscale map size is 1024 × 512 px. Same events as Fig. 10. (a) CMax-GAE trajectories. (b) Refined CMax-GAE trajectories, linear splines. (c) RTPT trajectories. (d) Refined RTPT trajectories, linear splines. (e) Groundtruth (only rotation).

## E. Runtime Evaluation

1) Comparison of Front-ends: Since the pipelines of the tested methods are completely different, to compare their efficiency we run them on selected segments from the ECD dataset and measure their processing time (see Table V). All front-ends process all events except for CMax-GAE, which samples one out of four events (i.e., only processes 25% of the events, as per its original implementation). EKF-SMT is faster than the others in high-texture scenes (poster and boxes), whereas CMax-GAE reports the shortest runtime in low-texture scenes (shapes and dynamic) by throwing away events. CMax-ω is competitive while processing all events and producing poses at 100 Hz (versus 40 Hz of CMax-GAE). Overall, none of the front-end methods show real-time performance except on shapes, whose texture is very simple.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/36ec0cc39febb3981b582a6b7387ac77320c451e0e17d37d0b35f94138baf1df.jpg)  
(a) shapes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/00780f3d951471bdaf05201e85e0153fa3ccb22e42b1a459517b6ba7c01fce4b.jpg)  
(b) poster

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/f4d6ba2e9e4665b284fe439dee30cb8dcef9b09bb4add90ec1e233fe498e24c5.jpg)  
(c) boxes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/7f3878134c9e19bf01331b9588964f0c260c3f1ebb7a3d8cb2f72f82619419ee.jpg)  
(d) dynamic  
Fig. 12. CMax-SLAM (online). Trajectory comparison of CMax front-end and CMax-SLAM (linear and cubic). (a) shapes. (b) poster. (c) boxes. (d) dynamic.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/cd3cd1dad40f8eeb776b5b26fef2152b5932bdeea1d16b436df8083f4ca70d8f.jpg)  
(a) shapes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/07de846c41a1fc894571e318ac413dce5db30b8e9cd0a057be4ce86ca4c758b1.jpg)  
(b) poster  
Relative rotation error [deg/s]

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/75367b3bc4390b0c0335226d1a9a93c513e758985689911c0b552de255663bf3.jpg)  
(c) boxes

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/0114413549d966e290fb9bb6b116aab521a3a67bac9e375232bb1b67f6ff5367.jpg)  
(d) dynamic  
Fig. 13. CMax-SLAM (online). Absolute and relative errors of the compared CMax-SLAM. “l” and “c” indicate the CMax-SLAM with linear and cubic spline trajectories, respectively. (a) shapes. (b) poster. (c) boxes. (d) dynamic.

TABLE V  
RUNTIME EVALUATION OF COMPARED FRONT-END METHODS [S]
<table><tr><td>Front-end methods</td><td>PF-SMT (CPU)</td><td>EKF-SMT (CPU)</td><td>CMax-ω (CPU)</td><td>CMax-GAE (CPU)</td><td>RTPT (GPU)</td></tr><tr><td>shapes</td><td>28.29</td><td>7.92</td><td>10.65</td><td>6.03</td><td>6.50</td></tr><tr><td>poster</td><td>174.78</td><td>26.90</td><td>52.34</td><td>36.25</td><td>38.43</td></tr><tr><td>boxes</td><td>170.66</td><td>27.08</td><td>47.65</td><td>42.12</td><td>38.24</td></tr><tr><td>dynamic</td><td>132.38</td><td>18.13</td><td>23.04</td><td>16.77</td><td>29.15</td></tr></table>

TABLE VI

CMAX-SLAM RUNTIME EVALUATION [μS/EVENT PROCESSED], FOR A MAP OF SIZE 1024 × 512 PX
<table><tr><td>Sequence</td><td>Front-end</td><td>Back-end (linear)</td><td>Back-end (cubic)</td></tr><tr><td>shapes (DAVIS240C)</td><td>1.725</td><td>11.204</td><td>20.406</td></tr><tr><td>poster (DAVIS240C)</td><td>1.369</td><td>5.021</td><td>8.045</td></tr><tr><td>boxes (DAVIS240C)</td><td>1.374</td><td>4.638</td><td>7.642</td></tr><tr><td>dynamic (DAVIS240C)</td><td>1.344</td><td>5.888</td><td>10.740</td></tr><tr><td>DAVIS346</td><td>1.710</td><td>7.872</td><td>13.565</td></tr></table>

2) CMax-SLAM: Table VI reports the computational cost of CMax-SLAM for several sequences at two different event camera resolutions: $2 4 0 \times 1 8 0$ px (DAVIS240C in the ECD dataset [53]) and 346 × 260 px (DAVIS346). It shows that the higher the camera resolution, the longer the processing time. In addition, the cubic spline trajectory representation is reasonably more expensive than the linear one. Currently, the CMax-SLAM system has been implemented without optimizing for real-time performance. It has the potential for nearly real-time performance by processing fewer events and reducing the size of the panoramic map (see Sections IV-F and IV-G).

In addition, in the accompanying video, we use a DAVIS346 to run CMax-SLAM in nearly real-time, by processing only a small portion of events and using a small panoramic map. This demonstrates the tradeoff mentioned in Sections III-C1 and III-C2, which is further analyzed in the sensitivity studies (see Sections IV-H3 and IV-H4) and can serve users to tune the system according to their needs.

## F. Experiments in the Wildfor CMax-SLAM

To assert that CMax-SLAM can work reliably in complex natural scenes (i.e., with arbitrary photometric variations), we test it on sequences in the wild, where no GT is available.

We create a new real-world event camera dataset for rotational motion study (see Table VII), which contains ten sequences recorded with a DVXplorer from iniVation AG (640 × 480 px). All sequences have events and IMU data (operating at around 800 Hz). For some sequences we place the camera on a motorized mount (Suptig RSX-350) to produce approximate uniform rotational motion around the Y axis. Despite the motorized mount, the camera cannot perform pure rotational motion because the center of rotation still deviates from the camera’s optical center. For the remaining sequences (crossroad, bridge, and river), the camera is hand-held, which may introduce more irregular residual translations.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/5e368c009f449ec08616d871b5599ff83a47743685a97577df03724254a83d0c.jpg)  
Fig. 14. Experiments in the wild. Panoramic IWEs produced by CMax-SLAM at 4096 × 2048 px resolution, on the data from a 640 × 480 px camera. The map sharpness is a proxy for the motion estimation quality. Gamma correction: γ = 0.75.

TABLE VII  
SELF-RECORDED DATASET WITH HIGH RESOLUTION EVENT CAMERA; DESCRIPTION OF SEQUENCES
<table><tr><td>Camera setup</td><td>Sequence</td><td># events [Mev]</td><td>Duration [s]</td><td>Y-angle [°]</td></tr><tr><td rowspan="6">Motorized mount</td><td>Brandenburg Gate</td><td>97.4</td><td>8.0</td><td>360</td></tr><tr><td>Charlottenburg Palace</td><td>115.4</td><td>8.4</td><td>360</td></tr><tr><td>Victory Column</td><td>4.9</td><td>10.0</td><td>90</td></tr><tr><td>TUB main building</td><td>116.2</td><td>8.5</td><td>360</td></tr><tr><td>TUB MAR building</td><td>33.8</td><td>4.0</td><td>≈ 90</td></tr><tr><td>square center</td><td>112.9 120.1</td><td>8.8 8.0</td><td>360 360</td></tr><tr><td rowspan="4">Hand-held</td><td>square side</td><td></td><td></td><td></td></tr><tr><td>crossroad</td><td>124.3</td><td>10.2 5.5</td><td>360</td></tr><tr><td>river bridge</td><td>69.3</td><td>7.5</td><td>random</td></tr><tr><td></td><td>89.0</td><td></td><td>random</td></tr></table>

Compared with the six indoor sequences in mocap rooms [22], [58], these outdoor sequences contain more difficult brightness conditions (e.g., reflections in the river and windows), dynamic objects (e.g., moving pedestrians, bicycles, cars, leaves, and water on the river) and direct sunlight observation causing glare in the lens, which make this dataset more challenging.

We run CMax-SLAM on the above sequences and produce panoramic IWEs. The results in Fig. 14 indicate that CMax-SLAM recovers precise global, sharp IWEs for the above challenging scenes. Results on additional scenes are presented in the accompanying video in Supplementary Material. For the sequences recorded with the motorized mount, the camera dominantly pans, so the edges that are parallel to the ground trigger very few events. Therefore, for Brandenburg Gate and Charlottenburg Palace in Fig. 14, vertical edges are quite dark and sharp whereas horizontal edges are not as clear. However, this situation is alleviated for crossroad because the motion is hand-held. Both types of motion demonstrate the capability of CMax-SLAM to work robustly in outdoor scenes.

## G. Super-Resolution

1) CMax-SLAM at Super-Resolution: The resolution of the panoramic map in the back-end is, to some extent, independent of the resolution of the event camera. Fig. 15 shows the results of running CMax-SLAM with several map resolutions, from 512 × 256 px to 8192 × 4096 px. The larger the map, the larger the memory requirements and the slower the bilinear voting used in the IWE (due to memory access, despite voting complexity being linear with the number of events), hence, the slower the optimization. As shown in Fig. 16(a), every time the map size is doubled, the computational cost of the CMax-SLAM back-end (BA) to process one event approximately increases exponentially. A very low resolution map warps many events into few map pixels. A very large resolution map may contain empty pixels (without warped events), which could be filled in by smoothing the map. In both extreme cases, the egomotion algorithm may fail. In intermediate cases [see Fig. 15, columns (b)–(d)], the CMax-SLAM works well, producing sharp maps because the continuous-time camera trajectory and the hightemporal resolution of the events allows us to warp the events in an almost continuous way, thus recovering fine details by converting high temporal resolution into high spatial resolution. A sensible choice of the map size consists of making the individual map pixels cover approximately the same scene area as those of the event camera.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/e8ed42651a65697f75ba0b4b15b17f2942e83d95a35e3b6bdbaced2a585674ce.jpg)  
(a) 512 × 256 px  
(b) 1024 × 512 px  
(c) 2048 × 1024 px  
(d) 4096 × 2048 px  
(e) 8192 × 4096 px  
Fig. 15. CMax-SLAM at several resolutions. Results of running CMax-SLAM at different map resolutions (columns), and using the estimated trajectories to generate panoramic IWEs at the corresponding resolutions. Gamma correction: γ = 0.75. Data from boxes (same events as Fig. 10). (a) 512 × 256 px. (b) 1024 × 512 px. (c) 2048 × 1024 px. (d) 4096 × 2048 px. (e) 8192 × 4096 px.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/cbcdd90852782f6cdc102ccf85b2db1ff7e3f04f6b14dae9184d814603d6834e.jpg)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/d083a4c3d107d02206b6dc4c39f584ddf14306ae56ac5cc8a4eac8928179ca1d.jpg)  
(a) Runtime  
(b) Map quality  
Fig. 16. Results of running CMax-SLAM at super resolution (Fig. 15). (a) Runtime evaluation of CMax-SLAM back-end at different resolutions. (b) Map quality evaluation of running CMax-SLAM at different resolutions. The lower the NIQE and PIQE scores, the better.

We evaluate the quality of the central parts of the panoramic IWEs (the first row in Fig. 15) by means of the naturalness image quality evaluator (NIQE) [59] and the perception-based image quality evaluator (PIQE) [60]: the lower the scores, the higher the image quality. As depicted in Fig. 16(b), the panoramic IWE at the resolution of 2048 × 1024 px has the highest quality, closely followed by that of 4096 × 2048 px, which agrees with a visual inspection.

2) Grayscale Map Reconstruction at Super-Resolution: In another experiment, we run CMax-SLAM with a map size of 1024 × 512 px, and utilize the estimated trajectory and the events as input to the mapping module ofSMT. Fig. 17 compares the resulting grayscale panoramas, with resolutions ranging from 512 × 256 px to 8192 × 4096 px. Hence, the continuous trajectory and the accurate timing of the events allows us to reach super-resolution. As the resolution increases, more details of the scene are recovered (e.g., the checkerboard in the second row of Fig. 17). Moving from each column to the next one in Fig. 17, the number of pixels $N _ { p }$ quadruples, bilinear voting becomes slower (due to memory access), and the cost of Poisson reconstruction via the fast fourier transform (FFT) (whose complexity is $O ( N _ { p } \log N _ { p } ) ;$ also approximately quadruples. Every time the map size doubles, the runtime of the EKFs that are used in the mapping module of SMT approximately increases linearly, but that of the Poisson reconstruction increases considerably faster [see Fig. 18(a)]. We also evaluate the quality of the central parts of the reconstructed grayscale panoramas (first row of Fig. 17) using NIQE and PIQE metrics. As shown in Fig. 18(b), the panorama with the resolution of 1024 × 512 px has the highest quality, closely followed by that of 2048 × 1024 px.

## H. Sensitivity Analysis

Finally, we evaluate the sensitivity of our method with respect to several parameters (see Fig. 19).

1) Control Pose Frequency: In this set of experiments, we set the time window size to 0.2 s and the resolution of panoramic IWE to 1024 × 512 px. The results are presented in Fig. 19(a). For bicycle (synthetic data), the accuracy of CMax-SLAM almost does not change as the control pose frequency varies from 10 Hz to 40 Hz. For dynamic (real-world data), the error increases obviously when the control pose frequency decreases from 20 Hz to 10 Hz. It seems that the control pose frequency is not a major factor to affect the accuracy, once it reaches some value (e.g., 20 Hz for dynamic).

2) Time (Sliding-)Window Size: In this set of experiments, we set the control pose frequency to 20 Hz and the resolution of panoramic IWE to 1024 × 512 px. The stride we use to slide the time window is half of its size. Overall, the accuracy of CMax-SLAM slightly decreases as the time window size increases on both bicycle and dynamic [see Fig. 19(b)]. The window size of 2 s is not tested on bicycle since it is just 5 s long.

3) Panoramic IWE Resolution: In this set of experiments, we set the control pose frequency to 20 Hz and the time window size to 0.2 s. As Fig. 19(c) shows, the errors decrease as the map size grows. However, as described in Section IV-G, there should be lower and upper limits of the map size for CMax-SLAM to work. Moreover, the computational cost increases rapidly as the map size grows [see Fig. 16(a)].

4) Event Sampling Rate: In this set of experiments, we set the control pose frequency to 20 Hz, the time window size to 0.2 s, and the panoramic map resolution to 1024 × 512 px. The events are systematically downsampled before being fed to the front-end and back-end. More specifically, an event sampling rate of 5 means both front-end and back-end process one event out of five (20% of events). As illustrated in Fig. 19(d), dynamic

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/505a2fb20a176dcb074cd8ec5fd7c067264835379f0c74a5e1eeaf9c9a825a68.jpg)  
(a) 512 × 256 px

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/8577cb92716efdcb4ecbb3af706430ba5f147db7aff562ce252bc487d0cb4c0c.jpg)  
(b) 1024 × 512 px

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/0447c5acbd82de89ea89c9b76ab7be9a7994d7b104a2f0565ff8e003ad687535.jpg)  
(c) 2048 × 1024 px

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/cb647e41507bdd84462c512439fb4aa30abe2e54e4137a9f0aa9c620aa1ca55d.jpg)  
(d) 4096 × 2048 px

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/5978a449cfa9d4af27abd100f5d1f0c04b7cd69fe11fb426b957b732c713bc6f.jpg)  
(e) 8192 × 4096 px  
Fig. 17. Grayscale map reconstruction at super-resolution. Results of running CMax-SLAM at 1024 × 512 pixel resolution to estimate the camera trajectory and feed it to the mapping module of SMT to obtain panoramic grayscale maps at different resolutions (columns). The location of the zoomed-in region is indicated with red rectangles. Same events as Fig. 10. (a) 512 × 256 px. (b) 1024 × 512 px. (c) 2048 × 1024 px. (d) 4096 × 2048 px. (e) 8192 × 4096 px.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/d937e376eb2d2424f08f50a46aaae58e67fd405a2c6308a5ac9d9fbf3cbe2f0b.jpg)  
(a) Runtime

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/e482569f0ad7eabd6171bbcd8392249adc06b01904634cfcb25fd17654fb3441.jpg)  
(b) Map quality  
Fig. 18. Results of grayscale map reconstruction at super resolution (Fig. 17). (a) Runtime evaluation of Poisson reconstruction at different resolutions. (b) Map quality evaluation of grayscale map reconstruction at different resolutions.

is not sensitive to the event sampling rate until it grows to 10. In contrast, the accuracy on bicycle declines gradually as the event sampling rate increases.

In addition, Fig. 19(e) depicts how the event sampling rate affects the processing time of CMax-SLAM: as expected, the runtime decreases as fewer events are processed, but in a nonlinear way (processing half of the events does not reduce the runtime by half). In all cases, the cubic B-spline is always more expensive than the linear one. With Fig. 19(d) and (e), users can set the tradeoff for their own applications.

## V. SPACE APPLICATION

The low latency, HDR and low power consumption characteristics of event cameras make them attractive to space applications [61], [62], [63]. For example, the authors in[64], [65] apply event cameras to the star tracking problem, which consists of estimating the egomotion of a rotating camera by tracking stars. Data may be acquired by an event camera on Earth or on a satellite.

Let us apply the proposed method to address the star tracking problem while simultaneously reconstructing a panoramic star map. The experiments show that our method outperforms prior work in terms of accuracy and robustness.

Our method does not convert events into frames nor does it extract correspondences because it is based on CMax [25], which handles data association implicitly (by the warped events that vote on the same pixel). Our BA also differs from prior ones used in star tracking, which are feature-based (between 2-D-3- D point correspondences) after converting events into frames [64] or fitting line segments and extracting their end points [65]. Therefore, our method is more adapted to the characteristics of event data than prior work.

## A. Experiments

1) Dataset: We test the proposed method on the star dataset [64]. It has eleven 45 s long sequences recorded with a DAVIS240C event camera (240 × 180 px) while observing a star field rotating at a constant angular velocity of4<sup>◦</sup>/s displayed on a screen. Like [64], [65], experiments are conducted on Seqs. 1–6, which contain 1.4–6.2 M events. Since the format in which the data is provided (in txt files and floating-point undistorted events, using a combined homography-calibration matrix) is not directly compatible with our implementation that uses raw events and known camera calibration matrix, we run the front-end and BA offline.

2) Metrics: Previous works [64], [65] perform rotation averaging [66] by feeding an additional set of absolute poses from a star identification system, which act as anchors for trajectory refinement. Such data is not available in the dataset, hence, we can only compare the performance in terms of relative rotation estimation. For a direct comparison, we adopt the benchmark in [64]: measuring the angular distance between relative rotations in a time interval of 400 ms. We also use the metrics in [64]: RMSE and standard deviation.

3) Results: Table VIII compares our approach with the state of the art. Our method produces accurate results (< 1<sup>◦</sup> RMSE): better than the state of the art in Seqs. 1, 2, 4, and comparable in the rest. Seq. 3 is known to be problematic [65]. Further, our method is the most consistent, as reported by its smaller standard deviation compared to other methods.

Fig. 20 shows the output of our method for one of the sequences. Since the sequences have no loops, rotation drift is inevitable in Fig. 20(b) without additional inputs (anchor absolute poses). Nevertheless, the estimation is good: angular velocity errors are small [see Fig. 20(a)], and the by-product map is sharp [see Fig. 20(c) and (d)], indicating that the method has successfully estimated the motion that caused the event data.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/e74abc6bd1a78e945646c8d268b86ce493850342d75066302247fa90eca5b2a4.jpg)  
(a)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/403a089bb84694bb7a311fa2ac56ec1fe5ad0b069bcc524ae99f57f3109256c3.jpg)  
(b)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/d564a504c03e75a19d270ebaf32a7697887e6e2e5ebde5c21ffd8ad011fa23ae.jpg)  
(c)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/98dad0604ed262213340c152c1ce87effe9c7621517e75e15c5eab02e271ddd6.jpg)  
(d)

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/0e7d473315c315f53b15f4ae385b0a574017bc7874c7b0180fe769b1b558994b.jpg)  
(e)  
Fig. 19. Sensitivity Analysis. From left to right, effect of varying: The frequency of the control poses, the size of the time window, the resolution of the IWE map, and the event sampling rate (on absolute rotation errors and runtime).

TABLE VIII  
SPACE APPLICATION ERRORS OF RELATIVE ROTATIONS (RMSE AND STANDARD DEVIATION σ) [<sup>◦</sup>]
<table><tr><td rowspan="2">Sequence</td><td colspan="2">1</td><td colspan="2">2</td><td colspan="2">3</td><td colspan="2">4</td><td colspan="2">5</td><td colspan="2">6</td></tr><tr><td>RMSE</td><td>σ</td><td>RMSE</td><td>σ</td><td>RMSE</td><td>σ</td><td>RMSE</td><td>σ</td><td>RMSE</td><td>σ</td><td>RMSE</td><td>σ</td></tr><tr><td>Chin et al. [64]</td><td>0.951</td><td>7.098</td><td>0.731</td><td>8.880</td><td>12.461</td><td>14.467</td><td>13.435</td><td>19.612</td><td>18.248</td><td>8.141</td><td>12.799</td><td>17.882</td></tr><tr><td>Bagchi et al. [65]</td><td>0.850</td><td>一</td><td>0.900</td><td></td><td>0.700</td><td>一</td><td>0.970</td><td>一</td><td>0.200</td><td></td><td>0.900</td><td>一</td></tr><tr><td>Ours</td><td>0.487</td><td>0.207</td><td>0.503</td><td>0.213</td><td>18.544</td><td>18.161</td><td>0.968</td><td>0.413</td><td>0.905</td><td>0.386</td><td>0.966</td><td>0.413</td></tr></table>

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/58e599b8051057e0bb7be261bc2d2a75e05b49a863259fc1d0355e1a5892e354.jpg)  
(a) Angular velocity

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/8e6f1c6175371394272dcd0bb93f78db47c5a62078d0b81acd7d8b50958cf745.jpg)  
(b) Euler angles

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/95f987ad787aa4b43ef25d078d668267ef16ba5103e99c21e4d3dbcbaad996bb.jpg)  
(c) Local IWE

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/b8af10783143dccb2cbad31083fe449db9a0ab5af414d317a541bd301a4e5d5f.jpg)  
(d) Global IWE  
Fig. 20. Space Application. Results on sequence 5 of [64]: (a) Estimated angular velocity. (b) Absolute rotation. (c) Local IWE before/after CMax (generated b the front-end with the event polarity used), corresponding to the region marked by the red rectangle in (d). (d) A zoomed-in part of motion-compensated event-based star map (generated by the back-end with the event polarity not used). In (a) and (b) blue solid lines are estimated whereas red dashed lines are the GT.

## VI. LIMITATIONS

Event cameras require the presence of contrast in the scene. It is difficult to track in textureless regions or with few edges. In the absence of data, one could act on the camera parameters, decreasing the value of the contrast threshold C to generate more events, and hopefully be able to sense small edges.

All methods compared assume brightness constancy. Events caused by flickering lights or hot pixels do not follow this assumption, and hence, could be problematic if they dominated over events produced by moving edges. In practice, our method exhibits some robustness to the few flickering lights in the scene, such as residual light from mocap systems.

While our system comprises a back-end for refinement, it lacks an explicit module with loop closure capabilities (all tested methods lack it), and therefore, inevitably accumulates drift. This is most noticeable in sequences where the scene is revisited after a full 360<sup>◦</sup> rotation around the vertical axis. To the best of the authors’ knowledge, none of the methods on event-based rotation-only egomotion estimation detect loop closure (we even dare to extend this to the event-based 6-DOF egomotion estimation literature, too). This topic is still in its infancy in event-based vision, and hence, is left as future work.

Event cameras with 640 × 480 px and higher spatial resolution may produce a colossal amount of events (e.g., 1Gev/s [2]), which requires a considerable amount ofmemory and computing power if all events are to be used. More proficient processors, ideally massively parallel like neuromorphic processors, could take on this problem.

## VII. CONCLUSION

We have presented the first event-based rotation-only BA and the first SLAM system comprising both a front-end and a backend. Both are principled, based on event alignment (CMax), and the back-end has a continuous-time camera trajectory model. Two trajectory models have been exemplified (linear and cubic

B-splines), with similar accuracy but different complexity (cubic is more demanding). Other continuous-time models, such as Gaussian processes, may be used. To the best of the authors knowledge, no prior work on the same task has considered past events for trajectory refinement without converting them into frames.

While the problem ofCMax is not formulated as an NLLS one (hence powerful second-order algorithms such as Gauss-Newton do not apply), we use first-order nonlinear conjugate gradient efficiently. We compute derivatives analytically and search only in the space of trajectories, since the map is naturally determined by the trajectory, the events and the projection model. This enables potential real-time operation, depending on the event rate and processing capabilities of the platform.

This work provides the most comprehensive benchmark of rotation-only estimation methods with event cameras, evaluating prior works and reimplementing, to the best of the authors understanding, previous methods whose code is unavailable. The experiments demonstrate that our proposal is accurate: On synthetic data, our method outperforms all baselines. On realworld data, we propose a more sensible FOM for the evaluation of event-based rotation estimation; and the results show that the front-end gives results on par with the state of the art; the back-end is closely integrated with the front-end and is able to refine trajectories, as demonstrated by small proxy reprojection errors and sharp maps.

Finally, due to the versatility of our method (the experiments show that it works indoors, outdoors, in natural scenes, and in space data), it could be used on satellites as well as on rovers. It could also be used for sky mapping (with wide FOV lenses, since for narrow FOV lenses a simpler linear approximation suffices for event warping). Hence, our work has a direct impact on event-based HDR sky mapping, situational awareness (SDA) and space robotics. We hope our work will spark ideas to advance the field of egomotion estimation (not only in 3-DOF but also in higher DOFs).

## ACKNOWLEDGMENT

The authors would like to thank Mr. Yunfan Yang and Ms. Nan Cai for assistance in recording the data sequences.

## REFERENCES

[1] P. Lichtsteiner, C. Posch, and T. Delbruck, “A 128 120 dB 15 μs latency asynchronous temporal contrast vision sensor,” IEEE J. Solid-State Circuits, vol. 43, no. 2, pp. 566–576, Feb. 2008.

[2] T. Finateu et al., “A 1280x720 back-illuminated stacked temporal contrast event-based vision sensor with 4.86 μm pixels, 1.066Geps readout, programmable event-rate controller and compressive data-formatting pipeline,” in Proc. IEEE Int. Solid-State Circuits Conf., 2020, pp. 112–114.

[3] G. Gallego et al., “Event-based vision: A survey,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 44, no. 1, pp. 154–180, Jan. 2022.

[4] D. Weikersdorfer and J. Conradt, “Event-based particle filtering for robot self-localization,” in Proc. IEEE Int. Conf. Robot. Biomimetics, 2012, pp. 866–870.

[5] G. Gallego, J. E. A. Lund, E. Mueggler, H. Rebecq, T. Delbruck, and D. Scaramuzza, “Event-based, 6-DOF camera tracking from photometric depth maps,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 10, pp. 2402–2412, Oct. 2018.

[6] G. Gallego and D. Scaramuzza, “Accurate angular velocity estimation with an event camera,” IEEE Robot. Automat. Lett., vol. 2, no. 2, pp. 632–639, Feb. 2017.

[7] S. Bryner, G. Gallego, H. Rebecq, and D. Scaramuzza, “Event-based, direct camera tracking from a photometric 3D map using nonlinear optimization,” in Proc. IEEE Int. Conf. Robot. Automat., 2019, pp. 325–331.

[8] A. Z. Zhu, L. Yuan, K. Chaney, and K. Daniilidis, “Unsupervised eventbased learning of optical flow, depth, and egomotion,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2019, pp. 989–997.

[9] W. Chamorro, J. A.-Cetto, and J. S. Ortega, “High-speed event camera tracking,” in Proc. Brit. Mach. Vis. Conf., 2020 pp. 1–12.

[10] J. Jiao, H. Huang, L. Li, Z. He, Y. Zhu, and M. Liu, “Comparing representations in tracking for event camera-based SLAM,” in Proc. IEEE Conf. Comput. Vis. Pattern Recog. Workshops, 2021, pp. 1369–1376.

[11] D. Weikersdorfer, R. Hoffmann, and J. Conradt, “Simultaneous localization and mapping for event-based vision systems,” in Proc. Int. Conf. Comput. Vis. Syst., 2013, pp. 133–142.

[12] H. Kim, A. Handa, R. Benosman, S.-H. Ieng, and A. J. Davison, “Simultaneous mosaicing and tracking with an event camera,” in Proc. Brit. Mach. Vis. Conf., 2014, pp. 1–12. [Online]. Available: https://bmvaarchive.org.uk/bmvc/2014/papers/paper066/index.html

[13] H. Kim, S. Leutenegger, and A. J. Davison, “Real-time 3D reconstruction and 6-DoF tracking with an event camera,” in Proc. Eur. Conf. Comput. Vis., 2016, pp. 349–364.

[14] H. Rebecq, T. Horstschäfer, G. Gallego, and D. Scaramuzza, “EVO: A geometric approach to event-based 6-DOF parallel tracking and mapping in real-time,” IEEE Robot. Automat. Lett., vol. 2, no. 2, pp. 593–600, Feb. 2017.

[15] C. Reinbacher, G. Munda, and T. Pock, “Real-time panoramic tracking for event cameras,” in Proc. IEEE Int. Conf. Comput. Photography, 2017, pp. 1–9.

[16] A. Z. Zhu, N. Atanasov, and K. Daniilidis, “Event-based visual inertial odometry,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2017, pp. 5816–5824.

[17] E. Mueggler, G. Gallego, H. Rebecq, and D. Scaramuzza, “Continuoustime visual-inertial odometry for event cameras,” IEEE Trans. Robot., vol. 34, no. 6, pp. 1425–1440, Dec. 2018.

[18] A. R. Vidal, H. Rebecq, T. Horstschaefer, and D. Scaramuzza, “Ultimate SLAM? combining events, images, and IMU for robust visual SLAM in HDR and high speed scenarios,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 994–1001, Apr. 2018.

[19] Y. Zhou, G. Gallego, and S. Shen, “Event-based stereo visual odometry,” IEEE Trans. Robot., vol. 37, no. 5, pp. 1433–1450, May 2021.

[20] W. Guan and P. Lu, “Monocular event visual inertial odometry based on event-corner using sliding windows graph-based optimization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robot. Syst., 2022, pp. 2438–2445.

[21] W. Chamorro, J. Solá, and J. A. Cetto, “Event-based line slam in real-time,” IEEE Robot. Automat. Lett., vol. 7, no. 3, pp. 8146–8153, Mar. 2022.

[22] H. Kim and H. J. Kim, “Real-time rotational motion estimation with contrast maximization over globally aligned events,” IEEE Robot. Automat. Lett., vol. 6, no. 3, pp. 6016–6023, Mar. 2021.

[23] C. Cadena et al., “Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1309–1332, Jun. 2016.

[24] R. Szeliski, Computer Vision: Algorithms and Applications, Ser. Texts in Computer Science. Berlin. Germany: Springer, 2010.

[25] G. Gallego, H. Rebecq, and D. Scaramuzza, “A unifying contrast maximization framework for event cameras, with applications to motion, depth, and optical flow estimation,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2018, pp. 3867–3876.

[26] G. Gallego, M. Gehrig, and D. Scaramuzza, “Focus is all you need: Loss functions for event-based vision,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2019, pp. 12272–12281.

[27] T. Stoffregen, G. Gallego, T. Drummond, L. Kleeman, and D. Scaramuzza, “Event-based motion segmentation by motion compensation,” in Proc. Int. Conf. Comput. Vis., 2019, pp. 7243–7252.

[28] D. Liu, A. Parra, and T.-J. Chin, “Globally optimal contrast maximisation for event-based motion estimation,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2020, pp. 6348–6357.

[29] U. M. Nunes and Y. Demiris, “Entropy minimisation framework for eventbased vision model estimation,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 161–176.

[30] X. Peng, Y. Wang, L. Gao, and L. Kneip, “Globally-optimal event camera motion estimation,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 51–67.

[31] C. Gu, E. L.-Miller, D. Sheldon, G. Gallego, and P. Bideau, “The spatio-temporal Poisson point process: A simple model for the alignment of event camera data,” in Proc. Int. Conf. Comput. Vis., 2021, pp. 13495–13504.

[32] U. M. Nunes and Y. Demiris, “Robust event-based vision model estimation by dispersion minimisation,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 44, no. 12, pp. 9561–9573, Dec. 2022.

[33] Y. Zhou, G. Gallego, X. Lu, S. Liu, and S. Shen, “Event-based motion segmentation with spatio-temporal graph cuts,” IEEE Trans. Neural Netw. Learn. Syst., vol. 34, no. 8, pp. 4868–4880, Aug. 2023.

[34] X. Peng, L. Gao, Y. Wang, and L. Kneip, “Globally-optimal contrast maximisation for event cameras,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 44, no. 7, pp. 3479–3495, Jul. 2022.

[35] S. Shiba, Y. Aoki, and G. Gallego, “Secrets of event-based optical flow,” in Proc. Eur. Conf. Comput. Vis., 2022, pp. 628–645.

[36] S. Ghosh and G. Gallego, “Multi-event-camera depth estimation and outlier rejection by refocused events fusion,” Adv. Intell. Syst., vol. 4, no. 12, 2022, Art. no. 2200221.

[37] S. McLeod et al., “Globally optimal event-based divergence estimation for ventral landing,” in Proc. Eur. Conf. Comput. Vis. Workshops, 2022, pp. 3– 20. [Online]. Available: https://link.springer.com/chapter/10.1007/978-3- 031-25056-9\_1

[38] S. Shiba, Y. Aoki, and G. Gallego, “A fast geometric regularizer to mitigate event collapse in the contrast maximization framework,” Adv. Intell. Syst., 2022, Art. no. 2200251.

[39] G. Klein and D. Murray, “Parallel tracking and mapping for small AR workspaces,” in Proc. IEEE ACM Int. Sym. Mixed Augmented Reality, 2007, pp. 225–234.

[40] T. Stoffregen and L. Kleeman, “Event cameras, contrast maximization and reward functions: An analysis,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2019, pp. 12292–12300.

[41] Z. Zhang, A. Yezzi, and G. Gallego, “Formulating event-based image reconstruction as a linear inverse problem with deep regularization using optical flow,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 45, no. 7, pp. 8372–8389, Jul. 2023.

[42] J. Engel, V. Koltun, and D. Cremers, “Direct sparse odometry,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 3, pp. 611–625, Mar. 2018.

[43] T. Stoffregen et al., “Reducing the sim-to-real gap for event cameras,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 534–549.

[44] S. Shiba, Y. Aoki, and G. Gallego, “Event collapse in contrast maximization frameworks,” Sensors, vol. 22, no. 14, pp. 1–20, 2022.

[45] B. Triggs, P. McLauchlan, R. Hartley, and A. Fitzgibbon, “Bundle adjustment–A modern synthesis,” in Vision Algorithms: Theory and Practice, ser.LNCS, W. A. Triggs Zisserman and R. Szeliski, Eds., vol. 1883. Heidelberg, Berlin: Springer, 2000, pp. 298–372.

[46] T. D. Barfoot, State Estimation for Robotics - A Matrix Lie Group Approach. Cambridge, U.K.: Cambridge Univ. Press, 2015.

[47] A. P.-Perez, S. Lovegrove, and G. Sibley, “A spline-based trajectory representation for sensor fusion and rolling shutter cameras,” Int. J. Comput. Vis., vol. 113, no. 3, pp. 208–219, 2015.

[48] C. Sommer, V. Usenko, D. Schubert, N. Demmel, and D. Cremers, “Efficient derivative computation for cumulative b-splines on lie groups,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2020, pp. 11145–11153.

[49] H. Rebecq, G. Gallego, E. Mueggler, and D. Scaramuzza, “EMVS: Eventbased multi-view stereo–3D reconstruction with an event camera in realtime,” Int. J. Comput. Vis., vol. 126, no. 12, pp. 1394–1414, Dec. 2018.

[50] M. Liu and T. Delbruck, “Adaptive time-slice block-matching optical flow algorithm for dynamic vision sensors,” in Proc. Brit. Mach. Vis. Conf., 2018, pp. 1–12.

[51] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “On-manifold preintegration for real-time visual-inertial odometry,” IEEE Trans. Robot., vol. 33, no. 1, pp. 1–21, 2017.

[52] C.-K. Shene, “Curve global interpolation,” in Introduction to Computing with Geometry Notes. Houghton, MI, USA: Michigan Technological University.

[53] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, and D. Scaramuzza, “The event-camera dataset and simulator: Event-based data for pose estimation, visual odometry, and SLAM,” Int. J. Robot. Res., vol. 36, no. 2, pp. 142–149, 2017.

[54] H. Rebecq, D. Gehrig, and D. Scaramuzza, “ESIM: An open event camera simulator,” in Proc. 2nd Conf. Robot Learn., 2018, pp. 969–982.

[55] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robot. Syst., 2012, pp. 573–580.

[56] Z. Zhang and D. Scaramuzza, “A tutorial on quantitative trajectory evaluation for visual(-inertial) odometry,” in Proc. IEEE/RSJ Int. Conf. Intell. Robot. Syst., 2018, pp. 7244–7251.

[57] H. Kim, “Real-time visual SLAM with an event camera,” Ph.D. dissertation, Imperial College London, London, U.K., 2018.

[58] E. Mueggler, C. Bartolozzi, and D. Scaramuzza, “Fast event-based corner detection,” in Proc. Brit. Mach. Vis. Conf., 2017 pp. 1–11.

[59] A. Mittal, R. Soundararajan, and A. C. Bovik, “Making a completely blind image quality analyzer,” IEEE Signal Process. Lett., vol. 20, no. 3, pp. 209–212, Mar. 2013.

[60] N. Venkatanath, D. Praneeth, M. C. Bh, S. S. Channappayya, and S. S. Medasani, “Blind image quality evaluation using perception based features,” in Proc. 21st Nat. Conf. Commun., 2015, pp. 1–6.

[61] G. Cohen et al., “Event-based sensing for space situational awareness,” J. Astronaut. Sci., vol. 66, no. 2, pp. 125–141, 2019.

[62] S. Afshar, A. P. Nicholson, A. v. Schaik, and G. Cohen, “Event-based object detection and tracking for space situational awareness,” IEEE Sensors J., vol. 20, no. 24, pp. 15117–15132, Dec. 2020.

[63] M. G. McHarg et al., “Falcon neuro: An event-based sensor on the international space station,” Opt. Eng., vol. 61, no. 8, 2022, Art. no. 085105.

[64] T.-J. Chin, S. Bagchi, A. P. Eriksson, and A. v. Schaik, “Star tracking using an event camera,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. Workshops, 2019, pp. 1646–1655.

[65] S. Bagchi and T.-J. Chin, “Event-based star tracking via multiresolution progressive Hough transforms,” in Proc. IEEE Winter Conf.Appl. Comput. Vis., 2020, pp. 2132–2141.

[66] R. Hartley, J. Trumpf, Y. Dai, and H. Li, “Rotation averaging,” Int. J. Comput. Vis., vol. 103, no. 3, pp. 267–305, 2013.

[67] G. Gallego, “Variational image processing algorithms for the stereoscopic space-time reconstruction of water waves,” Ph.D. dissertation, Georgia Institute of Technology, Atlanta, GA, USA, 2011.

[68] G. Gallego, C. Forster, E. Mueggler, and D. Scaramuzza, “Eventbased camera pose tracking using a generative event model,” 2015, arXiv:1510.01972.

[69] C. Brandli, R. Berner, M. Yang, S.-C. Liu, and T. Delbruck, “A 240x180 130 dB 3 μs latency global shutter spatiotemporal vision sensor,” IEEE J. Solid-State Circuits, vol. 49, no. 10, pp. 2333–2341, Oct. 2014.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/5e00a3f5486c6bc8cc8d39a35ae5280722b83131a947dcde6b66b643f29ac3e9.jpg)  
Shuang Guo (Graduate Student Member) received the B.Sc. and M.Sc. degrees in aerospace engineering from Harbin Institute of Technology, Harbin, China, in 2019 and 2021, respectively. He is currently working toward the Ph.D. degree, where he is advised by Prof. Guillermo Gallego, with Robotic Interactive Perception Laboratory, the Department of Electrical Engineering and Computer Science, Technische Universität Berlin, Berlin, Germany, .

His research interests include event-based vision and robotics.

![](images/2024_CMax-SLAM__Event-Based_Rotational-Motion_Bundle_Adjustme/f1a476b0bca0670846721ac1a741bcad9fc0761ccb82542b5105c2f79b5175d7.jpg)

Guillermo Gallego (Senior Member, IEEE) received the Ph.D. degree in electrical and computer engineering from the Georgia Institute of Technology, Atlanta, GA, USA, in 2011.

He is currently an Associate Professor with Technische Universität Berlin, Berlin, Germany, and with the Einstein Center Digital Future, Berlin, where he leads the Robotic Interactive Perception Laboratory. He is also a Principal Investigator with the Science of Intelligence Excellence Cluster and coDirector of the HEIBRiDS research school, Berlin, Germany.

Dr. Gallego was the recipient of the Fulbright Scholarship. He is an Associate Editor for IEEE TRANSACTIONS ON PATTERN ANALYSIS AND MACHINE INTELLIGENCE, IEEE ROBOTICS AND AUTOMATION LETTERS, and International Journal of Robotics Research.