# AirVO: An Illumination-Robust Point-Line Visual Odometry

Kuan Xu<sup>1</sup>, Yuefan Hao<sup>2</sup>, Shenghai Yuan<sup>1</sup>, Chen Wang<sup>3</sup>, Lihua Xie<sup>1</sup>, Fellow, IEEE

Abstract— This paper proposes an illumination-robust visual odometry (VO) system that incorporates both accelerated learning-based corner point algorithms and an extended line feature algorithm. To be robust to dynamic illumination, the proposed system employs the convolutional neural network (CNN) and graph neural network (GNN) to detect and match reliable and informative corner points. Then point feature matching results and the distribution of point and line features are utilized to match and triangulate lines. By accelerating CNN and GNN parts and optimizing the pipeline, the proposed system is able to run in real-time on low-power embedded platforms. The proposed VO was evaluated on several datasets with varying illumination conditions, and the results show that it outperforms other state-of-the-art VO systems in terms of accuracy and robustness. The open-source nature of the proposed system allows for easy implementation and customization by the research community, enabling further development and improvement of VO for various applications.

## I. INTRODUCTION

Due to the good balance in cost and accuracy, VO has been used in an extensive range of applications, especially in the domain of augmented reality and robotics [1]. Despite the existence of numerous well-known works, such as MSCKF [2], VINS-Mono [3] and OKVIS [4], the existing solutions are not robust enough for illumination-challenging conditions [5]. For example, in dynamic illumination environments, visual tracking becomes more challenging and thus the quality of the estimated trajectory is severely affected [6].

On the other hand, deep learning technology has made great progress in many computer vision tasks, which has triggered another research trend [7]. A lot of learning-based feature extraction and matching methods have been proposed and they have been proven to be more robust than handcrafted methods in illumination-challenging environments [8]–[10]. However, they often require huge computational resources and thus are impractical for real-time applications with lightweight robotics platforms such as unmanned aerial vehicles.

Therefore, in this paper, we propose AirVO, an illuminationrobust stereo visual odometry. We employ both learning-based feature extraction and matching methods to make our system robust enough in illumination-challenging environments. To achieve real-time and cost-effective performance, we accelerate the CNN and GNN parts and optimize the pipeline, making the feature extraction and tracking five times faster than the original work and the whole system able to run at a rate of about 15Hz on a low-power embedded device.

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/2ae23782ce7a087df1e2ebdce7be1573e3d813de507d746d693840010dd034f4.jpg)  
Fig. 1: AirVO is an accurate and robust stereo visual odometry in illumination-challenging environments. More demos are available at https://youtu.be/YfOCLll\_PfU.

To improve the accuracy, we also introduce line features into our system. We argue that long lines can provide more stable and accurate constraints, so we merge the short lines detected by LSD [11]. However, line detection is usually unstable in dynamic illumination environments, which makes line tracking and matching more difficult than in good lighting conditions. Besides, line feature triangulation is more difficult than point feature triangulation, because it suffers more from degenerate motion [12]. Therefore, we also propose a fast and efficient illumination-robust line processing pipeline in this paper. Observing that the point tracking in our system is very robust, we associate points with lines according to their distances. Then lines can be matched and triangulated using the matching and triangulation results of related points. The proposed line processing method is shown to be very robust even when the line detection is not stable and the lighting conditions are challenging. It is also very fast due to that it does not need to extract line descriptors. Overall, our contributions are as follows:

• The key contribution in this paper is that we propose a novel hybrid VO system that can effectively handle varying illumination conditions. Our proposed system combines the efficiency of traditional optimization techniques with the robustness of learning-based methods. To our best knowledge, AirVO is the first visual odometry that employs both learning-based feature detection and matching algorithms and can run in real-time on lowpower embedded platforms.

• We propose a new line processing pipeline for VO in this paper. Our approach associates 2D lines with learningbased 2D points on the image, leading to more robust feature matching and triangulation. This novel method enhances the accuracy and reliability of VO, especially in illumination-challenging environments.

• We perform extensive experiments that prove the efficiency and effectiveness of the proposed methods. The results show that AirVO outperforms other state-of-theart VO and visual-inertial odometry (VIO) systems, especially in illumination-challenging environments. Through optimization and acceleration of the relevant parts, our point feature detection and matching achieve more than 5× faster than the original work. Additionally, the system can run at a rate of about 15Hz on a lowpower embedded device and 40Hz on a notebook PC. We release source code at https://github.com/ sair-lab/AirVO to benefit the community.

## II. RELATED WORKS

## A. Feature Extraction and Tracking for Visual SLAM

Various key-point features have been proposed and applied to different computer vision tasks. Many of these features, e.g., ORB [13], FAST [14], and BRISK [15] are applied to VO and SLAM systems, e.g., ORB-SLAM [16], VINS-Mono [3], because of their balanced effectiveness and efficiency. Two methods are widely used to track the feature points. The first is to use optical flow [3], and the other is matching by descriptor [2], [17]. However, most of the current visual SLAM systems based on the above methods are evaluated in well-lighted environments and make a brightness consistency assumption. Thereby, their performances are significantly affected by challenging lighting conditions, such as dark, over-bright or dynamic illumination conditions.

With the development of deep learning techniques, many learning-based feature extraction and matching methods have been proposed and started to be applied to visual SLAM. Kang et al. [18] introduce TFeat network [19] to extract descriptors for FAST corners in a traditional VSLAM pipeline. Tang et al. [20] use a neural network to extract robust key-points and binary feature descriptors with the same shape of the ORB. Han et al. [21] combine SuperPoint [9] feature extractor with a traditional back-end. Bruno et al. proposed LIFT-SLAM [22], where they use LIFT [8] to extract features. Li et al. [23] replace ORB feature with SuperPoint in ORB-SLAM2 and optimize the feature extraction with Intel OpenVINO toolkit. However, the above methods still adopt traditional methods to track or match these learning-based features, making them not robust enough to changing illumination. Sarlin et al. propose HF-Net [24], where they integrate SuperPoint and SuperGlue [10] into COLMAP [25], a structure from motion software. HF-Net achieves good performance for visual place recognition tasks but requires huge computing resources and is unable to build maps in real-time.

Unlike current methods, AirVO introduces both learningbased feature extraction and matching methods in the VO system, which makes our system robust enough in illuminationchallenging environments. By accelerating the CNN and GNN parts, our system can perform pose estimation and build maps in real-time on low-power platforms.

## B. Line Matching for Visual SLAM

Line features widely exist in man-made environments, which can provide additional constraints. One of the challenges of using lines in visual SLAM is to perform line matching. A method used in many current SLAM systems [12], [26]–[28] is to match lines through LBD [29] descriptor. This method may make the line matching fail as the traditional line detection method, such as LSD [11], may be unstable. To overcome this, some systems [30]–[32] sample some points on a line, and then track the line by tracking these points. However, using either minimizing photometric error along the epipolar line or zero-normalized cross-correlation (ZNCC) matching method [33] cannot ensure robust line tracking in dynamic illumination environments.

## C. Visual SLAM for Dynamic Illumination

Several methods have been proposed to improve the robustness of VO and Visual SLAM to illumination changes. DSO [34] models brightness changes and jointly optimizes camera poses and photometric parameters. DRMS [35] and AFE-ORB-SLAM [36] utilize various image enhancements. Some systems try different methods, such as ZNCC, locally-scaled sum of squared differences (LSSD) and dense descriptor computation, to achieve robust tracking [37]–[39]. These methods mainly focus on either global or local illumination change for all kinds of images, however, lighting conditions often affect the scene differently in different areas [40].

Other related methods include that of Huang and Liu [41], which presents a multi-feature extraction algorithm to extract two kinds of image features when a single-feature algorithm fails to extract enough feature points. Kim et al. [42] employ a patch-based affine illumination model during direct motion estimation. Chen et al. [43] minimize the normalized information distance (NID) with nonlinear least square optimization for image registration. Alismail et al. propose a binary feature descriptor using a descriptor assumption to avoid the brightness constancy [44].

## III. METHODOLOGY

## A. System Overview

The proposed framework is shown in Fig. 2. It is a hybrid VO system where we utilize both the learning-based front end and the traditional optimization backend. For each stereo image pair, we first employ SuperPoint [9] to extract feature points on the left image and match them with the last keyframe using SuperGlue [10], and in parallel, we also extract line features. Then the two kinds of features are associated according to their distances and line features are matched using the matching results of associated points. After that, we perform an initial pose estimation and reject outliers. Based on the results, we select keyframes, extract features on the right image and triangulate 2D points and lines of keyframes. Finally, the local bundle adjustment will be performed to optimize points, lines and keyframe poses.

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/21a354e1fa0fd60b7fd2ab796b6ce05829eba7bb27c0171a8af2fb7f93867b10.jpg)  
Fig. 2: The framework of AirVO. The system is split into two main threads, which are represented by two different colored regions. The modules in the red dotted box and green dotted box run on the CPU and GPU, respectively.

To improve system efficiency, We replace the 32-bit floating-point arithmetic of CNNs and GNNs in our system with 16-bit floating-point arithmetic, which makes feature extraction and tracking more than five times faster than the original code on the embedded device. We also design a multithread pipeline that utilizes both CPU and GPU resources. A producer-consumer model is used to split the system into two main threads, i.e., the feature thread and the optimization thread. In the feature thread, we use two sub-threads to process point features and line features separately. In one sub-thread, the point feature extraction and matching with the last frame are put on the GPU while in parallel, the other sub-thread is used to extract line features on the CPU. In the optimization thread, we perform initial pose estimation and keyframe decision. If a new keyframe is selected, we extract both point and line features on its right image and optimize its pose with a local map.

## B. 2D Line Processing

We first give the details of 2D line processing in our system, which includes line segment detection and matching.

1) Detection: Line detection of AirVO is based on a traditional method (i.e., LSD [11]) for efficiency. LSD is a popular line detection algorithm. However, it suffers from the problem of dividing a line into multiple segments. Therefore, we improve it by merging two line segments $\mathbf { l } _ { 1 }$ and $\mathbf { l } _ { 2 }$ if the following conditions are satisfied:

• The angle between $\mathbf { l } _ { 1 }$ and $\mathbf { l } _ { 2 }$ is less than a threshold $\delta _ { \theta }$

• The distance between the midpoint of one line and the other line is not greater than a certain value $\delta _ { \mathbf { d } }$

• If projections of l and l on X-coordinate axis and Y-coordinate axis do not have overlap, the distance of the two closest endpoints is smaller than a threshold $\delta _ { \mathbf { e p } }$

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/909454704cb2e735779965dbdb85a4f596a28fca53a50e7306f04f4ce9ffcd2a.jpg)  
Fig. 3: Lines detected by LSD (left) and by AirVO (right). We merge unstable short lines into stable longer lines.

The line features detected in our system and comparison with LSD are shown in Fig. 3. We argue that long line segments are more repetitive and less affected by noise than the short ones, so after the merger, line segments whose lengths are less than a preset threshold will be filtered out so that only long line segments are used in the following stages.

2) Matching: Most of the current VO and SLAM systems use LBD algorithm or tracking sample points to match or track lines. LBD algorithm extracts the descriptor from a local band region of the line, so it suffers from unstable line detection in dynamic illumination environments where the line length may change and thus the local band region would be different between two frames. Tracking sample points can track the line which has different lengths in two frames, but current SLAM systems usually use optical flow to track the sample points, which have a bad performance when the light conditions change rapidly or violently. Some learning-based line feature matching methods [45], [46] are also proposed, however, they are rarely used in current SLAM systems as a result of the requirement for huge computational resources. We do not employ them either because it is difficult to make the system run in real-time on low-power embedded platforms if both learning-based point features and learning-based line features are used simultaneously.

Therefore, to address both the effectiveness problem and efficiency problem, we design a fast and robust line-matching method for dynamic illumination environments. First, we associate point features with line segments through the distances between points and lines. Assume that M key-points and N line segments are detected on the image, where each point is denoted as $\mathbf { p } _ { i } = \left( x _ { i } , y _ { i } \right)$ and each line segment is denoted as $\mathbf { l } _ { j } = \left( A _ { j } , B _ { j } , C _ { j } , x _ { j , 1 } , y _ { j , 1 } , x _ { j , 2 } , y _ { j , 2 } \right)$ , where $( A _ { j } , B _ { j } , C _ { j } )$ are line parameters of $\mathbf { l } _ { j }$ and $( x _ { j , 1 } , y _ { j , 1 } , x _ { j , 2 } , y _ { j , 2 } )$ are the endpoints. We first compute the distance between $\mathbf { p } _ { i }$ and $\mathbf { l } _ { j }$ through:

$$
d _ { i j } = d \left( \mathbf { p } _ { i } , \mathbf { l } _ { j } \right) = \frac { \left| A _ { j } \cdot x _ { i } + B _ { j } \cdot y _ { i } + C _ { j } \right| } { \sqrt { A _ { j } ^ { 2 } + B _ { j } ^ { 2 } } } .\tag{1}
$$

If $d _ { i j } < 3$ and the projection of $\mathbf { p } _ { i }$ on the coordinate axis lies within the projections of line segment endpoints, i.e., min $( x _ { j , 1 } , x _ { j , 2 } ) \leq x _ { i } \leq \operatorname* { m a x } ( x _ { j , 1 } , x _ { j , 2 } )$ or mi $\begin{array} { r } { \imath \left( y _ { j , 1 } , y _ { j , 2 } \right) \leq y _ { i } \leq } \end{array}$ $\operatorname* { m a x } ( y _ { j , 1 } , y _ { j , 2 } )$ , we will say $\mathbf { p } _ { i }$ belongs to $\mathbf { l } _ { j }$ . Then the line segments on two images can be matched based on the pointmatching result of these two images. For $k _ { \mathbf { l } _ { m } }$ on image k and $\boldsymbol { k } { + } \boldsymbol { 1 } _ { \boldsymbol { \mathbf { I } } _ { n } }$ on image $k + 1$ , we compute a score to represent the confidence of that they are the same line:

$$
S _ { m n } = \frac { N _ { p m } } { \operatorname* { m i n } ( { } ^ { k } N _ { m } , { } ^ { k + 1 } N _ { n } ) } ,\tag{2}
$$

where $N _ { p m }$ is the matching number between point features belonging to $k _ { \mathbf { l } _ { m } }$ and point features belonging $\mathbf { \hat { \mu } _ { t o } } \ k { + } 1 _ { \mathbf { l } _ { n } . } \ k _ { N _ { m } }$ and $\bar { k + 1 } \bar { N _ { n } }$ are the numbers of point features belonging to ${ } ^ { k } \mathbf { l } _ { m }$ and $\textstyle k + 1 _ { \mathbf { I } _ { n } } .$ , respectively. Then if $S _ { m n } > \delta _ { S }$ and $N _ { p m } > \delta _ { N }$ where $\delta _ { S }$ and $\delta _ { N }$ are two preset thresholds, we will regard $k _ { \mathbf { l } _ { m } }$ and $\boldsymbol { k } { + } \boldsymbol { 1 } _ { \boldsymbol { \mathbf { I } } _ { n } }$ as the same line.

Because the point matching is illumination-robust and feature association is not affected by lighting changes, the proposed line tracking method is very robust to dynamic illumination environments, as shown in Fig. 4.

## C. 3D Line Processing

In this part, we will introduce our 3D line processing methods. Compared with 3D points, 3D lines have more degrees of freedom, so we first introduce their representations in different stages. Then the methods of line triangulation, i.e., constructing a 3D line from some 2D line segments, and line re-projection, i.e., projecting the 3D line to the image plane, will be illustrated in detail.

1) Representation: We use Plücker coordinates [47] to represent a 3D spatial line:

$$
\mathbf { L } = \left[ { \begin{array} { l } { \mathbf { n } } \\ { \mathbf { v } } \end{array} } \right] \in \mathbb { R } ^ { 6 } ,\tag{3}
$$

where v is the direction vector of the line and n is the normal vector of the plane determined by the line and the origin. Plücker coordinates are used for 3D line triangulation, transformation, and projection to the image. It is overparameterized because it is a 6-dimensional vector, but a 3D line has only four degrees of freedom. In the graph optimization stage, the extra degrees of freedom will increase the computational cost and cause the numerical instability of the system [27]. Therefore, we also use orthonormal representation [47] to represent a 3D line:

$$
( \mathbf { U } , \mathbf { W } ) \in S O ( 3 ) \times S O ( 2 )\tag{4}
$$

The relationship between Plücker coordinates and orthonormal representation is similar to $S O ( 3 )$ and $s o ( 3 )$ . Orthonormal

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/0cc5c7f54f3d441d50ea16579eb65cf0600337eb201f5c6cece999ffce21655b.jpg)  
Fig. 4: Line matching of AirVO in challenging scenes. Matched lines are drawn in the same color. Circles on a line are the points associated with the line. A larger radius indicates that the point is associated with more lines.

representation can be obtained from Plücker coordinates by:

$$
\mathbf { L } = \left[ \mathbf { n } \mid \mathbf { v } \right] = \underbrace { { \left[ \begin{array} { l l l } { \mathbf { \frac { n } { \| \mathbf { n } \| } } } & { \mathbf { \frac { v } { \| \mathbf { v } \| } } } & { \mathbf { \frac { n \times \mathbf { v } } { \| \mathbf { n } \times \mathbf { v } \| } } } \\ { \mathbf { U } \in S O ( 3 ) } \right]} & { \mathbf { \frac { \mu } { \Sigma _ { 3 \times 2 } } } } \end{array}  } _ { \mathbf { U } \in S O ( 3 ) } \underbrace { { \left[ \begin{array} { l l } { \| \mathbf { n } \| } & \\ & { \| \mathbf { v } \| } \\ & \\ & { \mathbf { U } \subseteq \sigma } \end{array} \right] } } _ { \Sigma _ { 3 \times 2 } } ,\tag{5}
$$

where ${ \Sigma } _ { 3 \times 2 }$ is a diagonal matrix and its two non-zero entries defined up to scale can be represented by an $S O ( 2 )$ matrix, i.e., W. In practice, this conversion can be done simply and quickly with the QR decomposition.

2) Triangulation: Triangulation is to initialize a 3D line from two or more 2D line observations. Two methods are used to triangulate a 3D line in our system. The first is similar to the line triangulation algorithm B in [12], where a 3D line can be computed from two planes. To achieve this, we select two line segments, $\mathbf { l } _ { 1 }$ and $\mathbf { l } _ { 2 } ,$ , on two images, which are two observations of a 3D line. $\mathbf { l } _ { 1 }$ and $\mathbf { l } _ { 2 }$ can be back-projected and construct two 3D planes, $\pi _ { 1 }$ and $\pi _ { 2 }$ . Then the 3D line can be regarded as the intersection of $\pi _ { 1 }$ and $\pi _ { 2 }$

However, triangulating a 3D line is more difficult than triangulating a 3D point, because it suffers more from degenerate motions [12]. Therefore, we also employ a second line triangulation method if the above method fails, where points are utilized to compute the 3D line. In Section III-B.2, we have associated point features with line features. So to initialize a 3D line, two triangulated points $\mathbf { X } _ { 1 }$ and $\mathbf { X } _ { 2 }$ which belong to this line and have the shortest distance from this line on the image plane are selected. Then the Plücker coordinates of this line can be obtained through:

$$
\mathbf { L } = \left[ \begin{array} { l } { \mathbf { n } } \\ { \mathbf { v } } \end{array} \right] = \left[ \begin{array} { l } { \mathbf { X } _ { 1 } \times \mathbf { X } _ { 2 } } \\ { \frac { \mathbf { X } _ { 1 } - \mathbf { X } _ { 2 } } { \lVert \mathbf { X } _ { 1 } - \mathbf { X } _ { 2 } \rVert } } \end{array} \right] .\tag{6}
$$

Because the selected 3D points have been triangulated in the point triangulating stage, this method requires little extra computation. It is very efficient and robust.

3) Re-projection: We use Plücker coordinates to transform and re-project 3D lines. First, we convert the 3D line from

TABLE I: Translational error (RMSE) without loop closing and re-localization on the OIVIO dataset (unit: m). L refers to tracking lost and D refers to sequences where RMSEs are larger than 10m.
<table><tr><td>Sequence</td><td>VINS-Fusion</td><td>StructVIO</td><td>UV-SLAM</td><td>PL-SLAM</td><td>OKVIS</td><td>ORB-SLAM2</td><td>Basalt-VIO</td><td>AirVO</td></tr><tr><td>MN_015_GV_01</td><td>0.1033</td><td>8.6098</td><td>0.4991</td><td>1.3166</td><td>0.0663</td><td>0.0762</td><td>0.2157</td><td>0.0537</td></tr><tr><td>MN_015_GV_02</td><td>D</td><td>L</td><td>D</td><td>0.9523</td><td>1.5320</td><td>0.0776</td><td>0.1533</td><td>0.0619</td></tr><tr><td>MN_050_GV_01</td><td>D</td><td>D</td><td>D</td><td>1.1538</td><td>0.7785</td><td>0.0839</td><td>0.1857</td><td>0.0756</td></tr><tr><td>MN_050_GV_02</td><td>D</td><td>D</td><td>D</td><td>1.0055</td><td>0.7385</td><td>0.0755</td><td>0.1026</td><td>0.0717</td></tr><tr><td>MN_100_GV_01</td><td>D</td><td>D</td><td>D</td><td>0.8455</td><td>0.8729</td><td>0.0892</td><td>0.1965</td><td>0.0646</td></tr><tr><td>MN_100_GV_02</td><td>D</td><td>D</td><td>D</td><td>0.6708</td><td>0.4360</td><td>0.0848</td><td>0.0922</td><td>0.0770</td></tr><tr><td>TN_015_GV_01</td><td>0.1541</td><td>7.5849</td><td>1.6695</td><td>1.8856</td><td>0.3063</td><td>0.0902</td><td>0.1478</td><td>0.1009</td></tr><tr><td>TN_050_GV_01</td><td>0.2079</td><td>D</td><td>2.5948</td><td>1.9335</td><td>0.2262</td><td>0.0965</td><td>0.5214</td><td>0.0971</td></tr><tr><td>TN_100_GV_01</td><td>0.4063</td><td>D</td><td>1.4496</td><td>1.5263</td><td>0.3984</td><td>0.1044</td><td>0.1162</td><td>0.0578</td></tr></table>

the world frame to the camera frame:

$$
\begin{array} { r } { { { ^ c { \bf { L } } } } = \left[ \begin{array} { c } { { ^ c { \bf { n } } } } \\ { { ^ c { \bf { v } } } } \end{array} \right] = \left[ \begin{array} { c c } { { ^ c _ { w } { \bf { R } } } } & { { ^ [ c _ { w } { \bf { t } } ] } _ { \times } { ^ c _ { w } { \bf { R } } } } \\ { { ^ { \bf { 0 } } } } & { { ^ { ^ c _ { { \bf { R } } } } } } \end{array} \right] \left[ \begin{array} { c } { { ^ c _ { { \bf { n } } } } } \\ { { ^ w { \bf { v } } } } \end{array} \right] = _ { w } ^ { c } { \bf { H } } ^ { w } { \bf { L } } , } \end{array}\tag{7}
$$

where $\mathbf { \omega } ^ { c } \mathbf { L }$ and ${ } ^ { w } \mathbf { L }$ are Plücker coordinates of 3D line in the camera frame and world frame, respectively. ${ \bf \Sigma } _ { w } ^ { c } { \bf R } \in S O ( 3 )$ is the rotation matrix from world frame to camera frame and $\mathbf { \Delta } _ { w } ^ { c } \mathbf { t } \in \mathbb { R } ^ { 3 }$ is the translation vector. $[ \cdot ] _ { \times }$ denotes the skewsymmetric matrix of a vector and $^ c _ { w } \mathbf { H }$ is the transformation matrix of 3D lines from world frame to camera frame.

Then the 3D line $\mathbf { \epsilon } ^ { c } \mathbf { L }$ can be projected to the image plane through a line projection matrix ${ } _ { c } ^ { i } \mathbf { P } \colon$

$$
\begin{array} { r } { i _ { \mathbf { l } } = \left[ \begin{array} { c } { A } \\ { B } \\ { C } \end{array} \right] = \frac { i } { c } \mathbf { P } ^ { c } \mathbf { L } _ { [ : 3 ] } = \left[ \begin{array} { c c c } { f _ { x } } & { 0 } & { 0 } \\ { 0 } & { f _ { y } } & { 0 } \\ { - f _ { y } c _ { x } } & { - f _ { x } c _ { y } } & { f _ { x } f _ { y } } \end{array} \right] ^ { c } \mathbf { n } , } \end{array}\tag{8}
$$

where $\mathbf { \psi } ^ { i } \mathbf { I } = \left[ \begin{array} { l l l } { A } & { B } & { C } \end{array} \right] ^ { T }$ is the re-projected 2D line on image plane. ${ } ^ { c } \mathbf { L } _ { [ : 3 ] }$ donates the first three rows of vector $\mathbf { \epsilon } ^ { c } \mathbf { L }$

## D. Keyframe Selection

Observing that the learning-based data association method used in our system is able to track two frames that have a large baseline, so different from the frame-by-frame tracking strategy used in other VO or visual SLAM systems, we only match the current frame with the last keyframe, as this can reduce the tracking error. A frame will be selected as a keyframe if any of the following conditions is satisfied:

• The distance to the last keyframe is larger than $\delta _ { d } ^ { k f }$

• The angle with the last keyframe is larger than $\delta _ { \theta } ^ { \check { k } f }$

• The number of tracked map-points is smaller than $N _ { 1 } ^ { k f }$ and bigger than $N _ { 2 } ^ { k f }$ , where $\dot { N } _ { 2 } ^ { k f } < N _ { 1 } ^ { k f }$

• Tracked map-points are more than $N _ { ? } ^ { k \dot { f } }$ but the trackinglost happened in the last frame, i.e., map-points tracked by the last frame are less than $N _ { 2 } ^ { k f }$

where $\delta _ { d } ^ { k f } , \delta _ { \theta } ^ { k f } , N _ { 1 } ^ { k f }$ and $N _ { 2 } ^ { k f }$ are all preset thresholds.

## E. Graph Optimization

We select $N _ { k f } ^ { g o }$ keyframes and construct a co-visibility graph similar to ORB-SLAM [16], where map points, 3D lines, and keyframes are vertices and constraints are edges. Both point constraints and line constraints are used in our system and the related error terms are defined as follows.

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/33186e8a4b475025814a925d0ed5c383370c73440b197dd3c40501b2c1d24b65.jpg)  
Fig. 5: Comparison based on the OIVIO dataset. The vertical axis is the proportion of pose errors that are less than the given alignment error threshold on the horizontal axis.

1) Line Re-projection Error: If the frame $k$ can observe the 3D line ${ } ^ { w } \mathbf { L } _ { i } ,$ then the re-projection error is defined as:

$$
\mathbf { E } _ { l _ { k , i } } = e _ { l } \left( { k } \overline { { \mathbf { I } } } _ { i } , { k } _ { c } \mathbf { P } \big ( _ { w } ^ { c } \mathbf { H } ^ { w } \mathbf { L } _ { i } \big ) _ { [ : 3 ] } \right) \in \mathbb { R } ^ { 2 } ,\tag{9a}
$$

$$
\begin{array} { r } { e _ { l } \left( { } ^ { k } \bar { \mathbf { \bar { l } } } _ { i } , { } ^ { k } \mathbf { l } _ { i } \right) = \left[ \begin{array} { l l } { d \left( { } ^ { k } \bar { \mathbf { p } } _ { i , 1 } , { } ^ { k } \mathbf { l } _ { i } \right) } & { d \left( { } ^ { k } \bar { \mathbf { p } } _ { i , 2 } , { } ^ { k } \mathbf { l } _ { i } \right) } \end{array} \right] ^ { T } , } \end{array}\tag{9b}
$$

where $\bar { \mathbf { \rho } } _ { \mathbf { l } _ { i } } ^ { k }$ is the observation of ${ } ^ { w } { \bf L } _ { i }$ on frame $k , d ( \mathbf { p } , \mathbf { l } )$ is the distance between point p and line l, and ${ } ^ { k } \bar { \mathbf p } _ { i } ,$ <sub>1</sub> and $\mathbf { \bar { p } } _ { i , 2 }$ are the endpoints of $k _ { \mathbf { I } _ { i } } ^ { - }$

2) Point Re-projection Error: If the frame $k$ can observe the 3D point ${ } ^ { w } \mathbf { X } _ { q } ,$ , then the re-projection error is defined as:

$$
\begin{array} { r } { { \bf { E } } _ { p _ { k , q } } = ^ { k } \bar { \bf { x } } _ { q } - \pi \left( { ^ c _ { w } { \bf { R } } ^ { w } } { \bf { X } } _ { q } + { ^ c _ { w } } { \bf { t } } \right) , } \end{array}\tag{10}
$$

where $k _ { \bar { \mathbf { X } } _ { q } }$ is the observation of ${ } ^ { w } { \bf X } _ { q }$ on frame k and $\pi ( \cdot )$ represents the camera projection.

## IV. EXPERIMENTS

In this section, experimental results will be presented to demonstrate the performance of our method. We take pretrained SuperPoint and SuperGlue to detect and match feature points without any fine-tuning training. The experiments are conducted on two datasets: OIVIO dataset [48] and UMA visual-inertial dataset [6]. To prove the efficiency of the proposed line processing pipeline, we compare AirVO with state-of-the-art point-line VO and visual SLAM systems, i.e., PL-SLAM [26], StructVIO [31] and UV-SLAM [28]. PL-SLAM and UV-SLAM use the LBD descriptor to match line features while StructVIO tracks line features by tracking sampling points on lines. We also add stereo-mode VINS-Fusion [3], ORB-SLAM2 [17], Basalt-VIO [39] and VIOmode OKVIS [49] to the baselines. To handle the dynamic illumination problem, VINS-Fusion uses a failure detection and recovery module, while StructVIO uses the ZNCC method and Basalt-VIO uses the LSSD KLT method. The following experiments will prove that AirVO outperforms these methods in illumination-challenging environments. As the proposed method is a VO system, we disabled the loop closure part and re-localization from the above baselines.

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/9aebfb67b6c931d8e3e9e064a48aae8944be42c5ccb64221019c6489bf02b237.jpg)  
Fig. 6: A challenging sequence in UMA-VI dataset with significant illumination changes. The image may suddenly go dark as a result of turning off the lights, which is very difficult for feature tracking.

TABLE II: Translational error (RMSE) on the UMA-VI dataset (unit: m). The best results are highlighted.
<table><tr><td>Sequence</td><td>PL-SLAM</td><td>OKVIS</td><td>AirVO</td></tr><tr><td>conference-csc1</td><td>2.6974</td><td>1.1181</td><td>0.5236</td></tr><tr><td>conference-csc2</td><td>1.5956</td><td>0.4696</td><td>0.1607</td></tr><tr><td>third-floor-csc1</td><td>4.4779</td><td>0.2525</td><td>0.1760</td></tr><tr><td>third-floor-csc2</td><td>6.0675</td><td>0.2161</td><td>0.1312</td></tr><tr><td>average</td><td>3.7096</td><td>0.5141</td><td>0.2479</td></tr></table>

Note that as a result of lacking the ground truth of line matching and triangulation in dynamic illumination environments, we prove the effectiveness of the proposed line processing method by comparing it with other point-line systems and an ablation study instead of designing an extra line matching or triangulation comparison. Like [50]–[52], we compare with ORB-SLAM2 instead of ORB-SLAM3 [53] because the newly added atlas and IMU in ORB-SLAM3 are unfair to compare with the visual-only odometry, and they are difficult to remove because of the high coupling system. We also do not add DX-SLAM [23] and GCNv2-SLAM [20] to the baselines since they are based on RGB-D inputs and thus cannot run on the stereo datasets.

## A. Results on OIVIO Benchmark

OIVIO dataset collects visual-inertial data in tunnels and mines. In each sequence, the scene is illuminated by an onboard light of approximately 1300, 4500, or 9000 lumens. We used all nine sequences with ground truth acquired by the Leica TCRP1203 R300. The performance of translational error is presented in Table I. The two most accurate results are highlighted and underlined, respectively. AirVO achieves the best performance on 7 sequences and the second-best performance on the other 2 sequences, which outperforms other state-of-the-art algorithms. We notice that VINS-Fusion, StructVIO and UV-SLAM lost track on many sequences, this may be because their feature tracking methods, i.e., LSSD KLT sparse optical flow, ZNCC, LBD descriptor, are not

TABLE III: Ablation study. Translational error (RMSE) of AirVO<sup>w/o</sup> <sup>line</sup> and AirVO on the UMA-VI and OIVIO datasets (unit: m). The best results are highlighted.
<table><tr><td colspan="2">Sequence</td><td> $\mathrm { A i r V O ^ { w / o } l i n e }$ </td><td>AirVO</td></tr><tr><td>UMA-VI</td><td>conference-csc1 conference-csc2 third-floor-csc1 third-floor-csc2</td><td>2.4789 0.2323 0.1736 0.1629</td><td>0.5236 0.1607 0.1760 0.1312</td></tr><tr><td>OIVIO</td><td>MN_015_GV_01 MN_015_GV_02 MN_050_GV_01 MN_050_GV_02 MN_100_GV_01 MN_100_GV_02 TN_015_GV_01 TN_050_GV_01 TN_100_GV_01</td><td>0.1035 0.0668 0.1051 0.1049 0.1177 0.0921 0.1155 0.0987</td><td>0.0537 0.0619 0.0756 0.0717 0.0646 0.0770 0.1009</td></tr></table>

robust enough in illumination-challenging environments.

We show a comparison of our method with selected baselines on OIVIO TN\_100\_GV\_01 sequence in Fig. 5. In this case, the robot goes through a mine with onboard illumination. The distance is about 150 meters and the average speed is about 0.84m/s. The plot shows the proportion of pose errors on the horizontal axis that are less than the given alignment error threshold on the horizontal axis. AirVO achieves the most accurate result on this sequence.

## B. Results on UMA-VI Benchmark

UMA-VI dataset is a visual-inertial dataset gathered in illumination-challenging scenarios with handheld custom sensors. We selected sequences with illumination changes to evaluate our system. As shown in Fig. 6, it contains many sub-sequences where the image suddenly goes dark as a result of turning off the lights. It is more challenging than OIVIO dataset, so we only select methods proved to be illuminationrobust in Section IV-A as baselines, i.e., ORB-SLAM2, PL-SLAM, OKVIS and Basalt-VIO. The translational errors are presented in Table II. As ORB-SLAM2 and Basalt-VIO lost track on all 4 sequences, we do not list their results. It can be seen that AirVO outperforms other methods. Its average translational error is only 6.7% of PL-SLAM and 48.2% of OKVIS. We notice that the aligned errors are larger than those on the OIVIO dataset. It is because the UMA-VI dataset only gives the ground truth of the beginning and end of each sequence, which makes the errors appear larger, and the scenes are more difficult for VO or VIO systems.

![](images/2023_AirVO__An_Illumination-Robust_Point-Line_Visual_Odometry/e55d8afe4d3f3794fff4495521ed3f1a2ba781e16ae14518955b439be87052e0.jpg)  
Fig. 7: Bar chat showing the efficiency of different algorithms, as measured by CPU usage (%) and per-frame processing time (ms) on Nvidia Jetson AGX Xavier (2018).

TABLE IV: The average running time comparison of principal components with PL-SLAM.
<table><tr><td></td><td>PE</td><td>LE</td><td>PM</td><td>LM</td><td>IPE</td><td>BA</td></tr><tr><td>PL-SLAM</td><td>70 ms</td><td>96 ms</td><td>1 ms</td><td>29 ms</td><td>1 ms</td><td>101 ms</td></tr><tr><td>AirVO</td><td>25 ms</td><td>29 ms</td><td>10 ms</td><td>2 ms</td><td>4 ms</td><td>264 ms</td></tr></table>

We also compare the trajectory of AirVO with OKVIS and PL-SLAM on conference-csc2 sequence as shown in Fig. 1. The traveling distance of this sequence is about 50 meters and the average speed is about 0.75m/s. It clearly shows that AirVO produces the best accuracy in this challenging case. The drift error of AirVO is about 1.0%. OKVIS and PL-SLAM are 1.5% and 7.1%, respectively.

## C. Ablation Study

To show the effectiveness of the proposed line processing method, we remove line features from AirVO and name it as AirVO<sup>w/o</sup> <sup>line</sup>. The comparison results of AirVO and AirVO<sup>w/o</sup> <sup>line</sup> on OIVIO and UMA-VI datasets are presented in Table III. It can be seen that AirVO outperforms AirVO<sup>w/o</sup> <sup>line</sup> on 12 of 13 sequences, and utilizing line features reduces the translational error by 58.2% on average, which demonstrates that the proposed line processing method can improve the performance of the system.

## D. Runtime Analysis

This section presents the running time analysis of the proposed system. The evaluation is performed on the Nvidia Jetson AGX Xavier (2018), which is a low-power embedded platform with an 8-core ARM v8.2 64-bit CPU and a lowpower 512-core NVIDIA Volta GPU. The resolution of the input image sequence is 640 × 480. For all algorithms, we extract 200 points and disabled the loop closure, relocalization and visualization part for a fair comparison.

1) CNN and GNN Acceleration: We first verify the acceleration of the point detection and matching network. In our system, detecting and tracking feature points for one image take 64 ms, while the original code needs 342 ms. So it is about 5.3× faster than the origin.

2) Efficiency Comparison: We also compare the algorithm efficiency, as measured by CPU usage and per-frame processing time. The result is presented in Fig. 7. It can be seen that AirVO is one of the fastest methods (about 15 FPS) while the CPU usage is roughly the same as other methods because of utilizing the GPU resources. Notice that only the binary executable file of Struct-VIO is available, which is compiled on the x86 computer and cannot run on the Jetson platform, so we did not add it to this comparison.

3) Detailed Running Time: We also present the detailed running time of each module of PL-SLAM and AirVO in Table IV, where PE is point extraction, LE is line extraction, PM is point matching, LM is line matching, IPE is initial pose estimation and BA is keyframe processing and local bundle adjustment. It can be seen that the line processing pipeline of AirVO is much more efficient than PL-SLAM. Our BA module has higher runtime than PL-SLAM, this may be because more stable features are detected, tracked and taken into optimization in our system. Notice that these modules run in parallel and BA module is a non-real-time back-end thread, so the running time of the whole system is not the simple accumulation of each module.

## V. CONCLUSIONS

In this work, we presented an illumination-robust visual odometry based on learning-based key-point detection and matching methods. To improve the accuracy, line features are also utilized in our system. We proposed a novel line processing pipeline to make line tracking robust enough in illumination-dynamic environments. In the experiments, we showed that the proposed method achieved superior performance in illumination-dynamic environments and could run in real-time on low-power devices. We open the source code to benefit the robotic community. For future work, we will extend AirVO to a SLAM system by adding loop closing, re-localization and map reuse. We hope to build an illumination-robust visual map for long-term localization.

## REFERENCES

[1] A. Macario Barros, M. Michel, Y. Moline, G. Corre, and F. Carrel, “A comprehensive survey of visual slam algorithms,” Robotics, vol. 11, no. 1, p. 24, 2022.

[2] A. I. Mourikis and S. I. Roumeliotis, “A multi-state constraint kalman filter for vision-aided inertial navigation,” in Proceedings 2007 IEEE international conference on robotics and automation. IEEE, 2007, pp. 3565–3572.

[3] T. Qin, P. Li, and S. Shen, “Vins-mono: A robust and versatile monocular visual-inertial state estimator,” IEEE Transactions on Robotics, vol. 34, no. 4, pp. 1004–1020, 2018.

[4] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, “Keyframe-based visual–inertial odometry using nonlinear optimization,” The International Journal of Robotics Research, vol. 34, no. 3, pp. 314–334, 2015.

[5] C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. Reid, and J. J. Leonard, “Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age,” IEEE Transactions on robotics, vol. 32, no. 6, pp. 1309–1332, 2016.

[6] D. Zuñiga-Noël, A. Jaenal, R. Gomez-Ojeda, and J. Gonzalez-Jimenez, “The uma-vi dataset: Visual–inertial odometry in low-textured and dynamic illumination environments,” The International Journal of Robotics Research, vol. 39, no. 9, pp. 1052–1060, 2020.

[7] K. Xu, C. Wang, C. Chen, W. Wu, and S. Scherer, “Aircode: A robust object encoding method,” IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 1816–1823, 2022.

[8] K. M. Yi, E. Trulls, V. Lepetit, and P. Fua, “Lift: Learned invariant feature transform,” in European conference on computer vision. Springer, 2016, pp. 467–483.

[9] D. DeTone, T. Malisiewicz, and A. Rabinovich, “Superpoint: Selfsupervised interest point detection and description,” in Proceedings of the IEEE conference on computer vision and pattern recognition workshops, 2018, pp. 224–236.

[10] P.-E. Sarlin, D. DeTone, T. Malisiewicz, and A. Rabinovich, “Superglue: Learning feature matching with graph neural networks,” in Proceedings of the IEEE/CVF conference on computer vision and pattern recognition, 2020, pp. 4938–4947.

[11] R. G. Von Gioi, J. Jakubowicz, J.-M. Morel, and G. Randall, “Lsd: A line segment detector,” Image Processing On Line, vol. 2, pp. 35–55, 2012.

[12] Y. Yang, P. Geneva, K. Eckenhoff, and G. Huang, “Visual-inertial odometry with point and line features,” in 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019, pp. 2447–2454.

[13] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: An efficient alternative to sift or surf,” in 2011 International conference on computer vision. IEEE, 2011, pp. 2564–2571.

[14] D. G. Viswanathan, “Features from accelerated segment test (fast),” in Proceedings of the 10th workshop on image analysis for multimedia interactive services, London, UK, 2009, pp. 6–8.

[15] S. Leutenegger, M. Chli, and R. Y. Siegwart, “Brisk: Binary robust invariant scalable keypoints,” in 2011 International conference on computer vision. IEEE, 2011, pp. 2548–2555.

[16] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM: a versatile and accurate monocular slam system,” IEEE transactions on robotics, vol. 31, no. 5, pp. 1147–1163, 2015.

[17] R. Mur-Artal and J. D. Tardós, “Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras,” IEEE transactions on robotics, vol. 33, no. 5, pp. 1255–1262, 2017.

[18] R. Kang, J. Shi, X. Li, Y. Liu, and X. Liu, “Df-slam: A deep-learning enhanced visual slam system based on deep local features,” arXiv preprint arXiv:1901.07223, 2019.

[19] V. Balntas, E. Riba, D. Ponsa, and K. Mikolajczyk, “Learning local feature descriptors with triplets and shallow convolutional neural networks.” in Bmvc, vol. 1, no. 2, 2016, p. 3.

[20] J. Tang, L. Ericson, J. Folkesson, and P. Jensfelt, “Gcnv2: Efficient correspondence prediction for real-time slam,” IEEE Robotics and Automation Letters, vol. 4, no. 4, pp. 3505–3512, 2019.

[21] X. Han, Y. Tao, Z. Li, R. Cen, and F. Xue, “Superpointvo: A lightweight visual odometry based on cnn feature extraction,” in 2020 5th International Conference on Automation, Control and Robotics Engineering (CACRE). IEEE, 2020, pp. 685–691.

[22] H. M. S. Bruno and E. L. Colombini, “Lift-slam: A deep-learning feature-based monocular visual slam method,” Neurocomputing, vol. 455, pp. 97–110, 2021.

[23] D. Li, X. Shi, Q. Long, S. Liu, W. Yang, F. Wang, Q. Wei, and F. Qiao, “Dxslam: A robust and efficient visual slam system with deep features,” in 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020, pp. 4958–4965.

[24] P.-E. Sarlin, C. Cadena, R. Siegwart, and M. Dymczyk, “From coarse to fine: Robust hierarchical localization at large scale,” in Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, 2019, pp. 12 716–12 725.

[25] J. L. Schonberger and J.-M. Frahm, “Structure-from-motion revisited,” in Proceedings of the IEEE conference on computer vision and pattern recognition, 2016, pp. 4104–4113.

[26] R. Gomez-Ojeda, F.-A. Moreno, D. Zuniga-Noël, D. Scaramuzza, and J. Gonzalez-Jimenez, “Pl-slam: A stereo slam system through the combination of points and line segments,” IEEE Transactions on Robotics, vol. 35, no. 3, pp. 734–746, 2019.

[27] X. Zuo, X. Xie, Y. Liu, and G. Huang, “Robust visual slam with point and line features,” in 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017, pp. 1775–1782.

[28] H. Lim, J. Jeon, and H. Myung, “UV-SLAM: Unconstrained line-based slam using vanishing points for structural mapping,” IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 1518–1525, 2022.

[29] L. Zhang and R. Koch, “An efficient and robust line segment matching approach based on LBD descriptor and pairwise geometric consistency,” Journal of Visual Communication and Image Representation, vol. 24, no. 7, pp. 794–805, 2013.

[30] L. Zhou, S. Wang, and M. Kaess, “Dplvo: Direct point-line monocular visual odometry,” IEEE Robotics and Automation Letters, vol. 6, no. 4, pp. 7113–7120, 2021.

[31] D. Zou, Y. Wu, L. Pei, H. Ling, and W. Yu, “Structvio: visual-inertial

odometry with structural regularity of man-made environments,” IEEE Transactions on Robotics, vol. 35, no. 4, pp. 999–1013, 2019.

[32] B. Xu, P. Wang, Y. He, Y. Chen, Y. Chen, and M. Zhou, “Leveraging structural information to improve point line visual-inertial odometry,” IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 3483–3490, 2022.

[33] L. Di Stefano, S. Mattoccia, and F. Tombari, “Zncc-based template matching using bounded partial correlation,” Pattern recognition letters, vol. 26, no. 14, pp. 2129–2134, 2005.

[34] J. Engel, V. Koltun, and D. Cremers, “Direct sparse odometry,” IEEE transactions on pattern analysis and machine intelligence, vol. 40, no. 3, pp. 611–625, 2017.

[35] Q. Gu, P. Liu, J. Zhou, X. Peng, and Y. Zhang, “Drms: Dim-light robust monocular simultaneous localization and mapping,” in 2021 International Conference on Computer, Control and Robotics (ICCCR). IEEE, 2021, pp. 267–271.

[36] L. Yu, E. Yang, and B. Yang, “Afe-orb-slam: robust monocular vslam based on adaptive fast threshold and image enhancement for complex lighting environments,” Journal of Intelligent & Robotic Systems, vol. 105, no. 2, pp. 1–14, 2022.

[37] G. G. Scandaroli, M. Meilland, and R. Richa, “Improving ncc-based direct visual tracking,” in European conference on Computer Vision. Springer, 2012, pp. 442–455.

[38] A. Crivellaro and V. Lepetit, “Robust 3d tracking with descriptor fields,” in Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, 2014, pp. 3414–3421.

[39] V. Usenko, N. Demmel, D. Schubert, J. Stückler, and D. Cremers, “Visual-inertial mapping with non-linear factor recovery,” IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 422–429, 2019.

[40] S. Park, T. Schöps, and M. Pollefeys, “Illumination change robustness in direct visual slam,” in 2017 IEEE international conference on robotics and automation (ICRA). IEEE, 2017, pp. 4523–4530.

[41] J. Huang and S. Liu, “Robust simultaneous localization and mapping in low-light environment,” Computer Animation and Virtual Worlds, vol. 30, no. 3-4, p. e1895, 2019.

[42] P. Kim, H. Lee, and H. J. Kim, “Autonomous flight with robust visual odometry under dynamic lighting conditions,” Autonomous Robots, vol. 43, no. 6, pp. 1605–1622, 2019.

[43] Z. Chen and C. Heckman, “Robust pose estimation based on normalized information distance,” in 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2021, pp. 2217–2223.

[44] H. Alismail, M. Kaess, B. Browning, and S. Lucey, “Direct visual odometry in low light using binary descriptors,” IEEE Robotics and Automation Letters, vol. 2, no. 2, pp. 444–451, 2016.

[45] R. Pautrat, J.-T. Lin, V. Larsson, M. R. Oswald, and M. Pollefeys, “Sold2: Self-supervised occlusion-aware line description and detection,” in Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, 2021, pp. 11 368–11 378.

[46] R. Pautrat, D. Barath, V. Larsson, M. R. Oswald, and M. Pollefeys, “Deeplsd: Line segment detection and refinement with deep image gradients,” arXiv preprint arXiv:2212.07766, 2022.

[47] A. Bartoli and P. Sturm, “Structure-from-motion using lines: Representation, triangulation, and bundle adjustment,” Computer vision and image understanding, vol. 100, no. 3, pp. 416–441, 2005.

[48] M. Kasper, S. McGuire, and C. Heckman, “A benchmark for visualinertial odometry systems employing onboard illumination,” in 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019, pp. 5256–5263.

[49] S. Leutenegger, P. Furgale, V. Rabaud, M. Chli, K. Konolige, and R. Siegwart, “Keyframe-based visual-inertial slam using nonlinear optimization,” Proceedings of Robotis Science and Systems (RSS) 2013, 2013.

[50] I. Cvišic, I. Markovi ´ c, and I. Petrovi ´ c, “Soft2: Stereo visual odometry´ for road vehicles based on a point-to-epipolar-line metric,” IEEE Transactions on Robotics, 2022.

[51] R. Song, R. Zhu, Z. Xiao, and B. Yan, “Contextavo: Local context guided and refining poses for deep visual odometry,” Neurocomputing, 2023.

[52] S. Zhang, J. Zhang, and D. Tao, “Towards scale consistent monocular visual odometry by learning from the virtual world,” in 2022 International Conference on Robotics and Automation (ICRA). IEEE, 2022, pp. 5601–5607.

[53] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. Montiel, and J. D. Tardós, “Orb-slam3: An accurate open-source library for visual, visual– inertial, and multimap slam,” IEEE Transactions on Robotics, vol. 37, no. 6, pp. 1874–1890, 2021.