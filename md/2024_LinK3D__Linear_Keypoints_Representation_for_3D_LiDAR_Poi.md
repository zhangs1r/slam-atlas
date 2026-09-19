# LinK3D: Linear Keypoints Representation for 3D LiDAR Point Cloud

Yunge Cui , Yinlong Zhang , Member, IEEE, Jiahua Dong , Haibo Sun, Xieyuanli Chen , and Feng Zhu

Abstract—Feature extraction and matching are the basic parts of many robotic vision tasks, such as 2D or 3D object detection, recognition, and registration. As is known, 2D feature extraction and matching have already achieved great success. Unfortunately, in the field of 3D, the current methods may fail to support the extensive application of 3D LiDAR sensors in robotic vision tasks due to their poor descriptiveness and inefficiency. To address this limitation, we propose a novel 3D feature representation method: Linear Keypoints representation for 3D LiDAR point cloud, called LinK3D. The novelty of LinK3D lies in that it fully considers the characteristics (such as the sparsity and complexity) of LiDAR point clouds and represents the keypoint with its robust neighbor keypoints, which provide strong constraints in the description of the keypoint. The proposed LinK3D has been evaluated on three public datasets, and the experimental results show that our method achieves great matching performance. More importantly, LinK3D also shows excellent real-time performance, faster than the sensor frame rate at 10 Hz of a typical rotating LiDAR sensor. LinK3D only takes an average of 30 milliseconds to extract features from the point cloud collected by a 64-beam LiDAR and takes merely about 20 milliseconds to match two LiDAR scans when executed on a computer with an Intel Core i7 processor. Moreover, our method can be extended to LiDAR odometry task, and shows good scalability.

Index Terms—3D LiDAR point cloud, feature extraction and matching, real-time, LiDAR SLAM.

## I. INTRODUCTION

EATURE extraction and matching are the building blocks and reconstruction [2] tasks. In the field of 2D vision, a variety of famous 2D feature extraction methods (such as SIFT [3] and ORB [4]), have been proposed and widely used. However, in the field of 3D vision, there are still several unsolved issues for 3D feature representation and matching. Current methods [5], [6], [7], [8], [9], [10], [11] may not be suitable for the high frequency (usually 10 Hz) of 3D LiDAR and the large-scale complex scenes, especially in terms of efficiency and reliability. The irregularity, sparsity, and disorder of the LiDAR point cloud make it infeasible for 2D methods directly applied to 3D.

Existing 3D feature point representation methods can be mainly divided into two categories in terms of extraction strategies, i.e., hand-crafted features and learning-based features. The hand-crafted features [5], [6], [7], [12] mainly describe features in the form of histograms, and they use local or global statistical information to represent features. As there are usually many similar local features (such as local planes) in large-scale scenes that LiDAR uses, these local statistical features can easily lead to mismatches. The global features [13], [14], intuitively, are unlikely to generate accurate point-to-point matches inside the point cloud. Learning-based methods [8], [9], [10], [11], [15], [16] have made great progress. However, the efficiency and generalization performance of these methods are still to be improved. In addition, some methods [5], [6], [7] were proposed for the point clouds collected from small-scale object surfaces (e.g., the Stanford Bunny<sup>1</sup> point clouds). Obviously, there are some differences between the small-scale objects and the large-scale scenes using 3D LiDAR (e.g., the city scene of KITTI [17]). Specifically, the main differences are as follows:

The small object’s surface is usually smooth and continuous, and its local surface is unique. However, the 3D LiDAR point cloud contains lots of discontinuous and similar local surfaces (e.g., similar local planes, trees, poles, etc.), and they easily lead to mismatches.

Compared with the point cloud of small-scale objects, the LiDAR point cloud is usually sparser, and the points are unevenly distributed in space. If there are not enough points in a fixed-size space, an effective statistical description may not be yielded.

\- Different from static and complete small-scale objects, there are often dynamic objects (cars, pedestrians, etc.)

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/7c76b737dc9b47367f904083b3e387b750ab6cf026aff690d5386c4f98b91fd9.jpg)  
Fig. 1. Core idea of the proposed LinK3D and the matching results of two LiDAR scans based on LinK3D. LinK3D represents the current keypoint with its neighbor keypoints. The green lines are the valid matches. Accurate point-to-point matches can be obtained by matching the corresponding LinK3D descriptors.

and occlusions in LiDAR scans. This can easily lead to inconsistent descriptions of the same local surface in the current and subsequent LiDAR scans.

According to the differences, in this letter, we propose a novel 3D feature for LiDAR point clouds. Our method first extracts the robust aggregation keypoints. Then the extracted aggregation keypoints are fed into the descriptor generation algorithm. As shown in Fig. 2, the algorithm generates the descriptor in the form ofa novel keypoint representation. After obtaining LinK3D descriptors, the matching algorithm can quickly match the descriptors of two LiDAR scans. In experiments, the proposed LinK3D achieves great matching performance and also shows impressive efficiency. Furthermore, LinK3D can be potentially applied to downstream 3D vision tasks, and we have applied LinK3D to LiDAR odometry. To summarize, our main contributions are as follows:

\- Strong matching performance: The proposed LinK3D feature considers the characteristics of LiDAR point clouds, and achieves significant progress in matching performance for sparse LiDAR point clouds.

\- Real-time performance on CPU: The proposed LinK3D shows impressive efficiency, which makes it more suitable for the 3D applications of mobile robots with limited computing resources.

\- Good scalability: LinK3D can be potentially applied to downstream 3D vision tasks. In this letter, LinK3D has been applied to the LiDAR odometry task.

## II. RELATED WORK

Based on the extraction strategy, current 3D feature extraction methods can be divided into hand-crafted methods and deep neural network (DNN) methods.

Hand-crafted methods: The histograms are usually used to represent different characteristics of the local surface. PFH [12] generates a multi-dimensional histogram feature of point pairs in the support region. FPFH [5] builds a Simplified Point Feature Histogram (SPFH) for each point by calculating the relationships between the point and its neighbors. SHOT [6] combines the spatial and geometric statical information and encodes the histograms of the surface normals in different spatial locations. In order to improve the matching efficiency, a binary quantization method B-SHOT [7] is proposed that converts a real-valued vector to a binary vector. 3DHoPD [18] transforms the 3D keypoints into a new 3D space to generate histogram descriptions.

In addition, the global descriptor Seed [14] is a segmentationbased method for the place recognition task of LiDAR SLAM. Moreover, GOSMatch [19] extracts the histogram-based graph descriptor for the place recognition of LiDAR SLAM. Due to the sparsity of the LiDAR point cloud, the statistical methods may fail to generate effective feature representation when there are not enough points.

DNN-based methods: 3DFeatNet [15] learns both 3D feature detectors and descriptors for point cloud matching using weak supervision. FCGF [16] extracts 3D features in a single pass by a 3D fully-convolutional network and presents metric learning losses to improve performance. DeepVCP [8] generates keypoints based on learned matching probabilities among a group of candidates. DH3D [9] designs a hierarchical network to perform local feature detection, local feature description, and global descriptor extraction in a single forward pass. D3Feat [10] utilizes a self-supervised detector loss guided by the on-the-fly feature matching results during training. The semantic graph representation method [20] reserves the semantic and topological information of the raw point cloud for the place recognition of LiDAR SLAM. StickyPillars [11] uses a handcrafted method to extract keypoints and combines the DNN method to generate descriptors, which is efficient in keypoint extraction but inefficient in descriptor generation. Geo Transformer [21] encodes pair-wise distances and triplet-wise angles, making it robust in low-overlap cases and invariant to rigid transformation. In general, DNN-based methods usually require GPUs to speed up processing. In addition, the generalization of these methods is yet to be improved.

## III. METHODOLOGY

The pipeline of our method mainly consists of two parts: feature extraction and feature matching. The process of feature extraction is shown in Fig. 1. The edge points of LiDAR scans are first extracted, then they are fed into the edge keypoint aggregation algorithm, where the robust aggregation keypoints are further extracted for subsequent descriptor generation. In the descriptor generation algorithm, the distance table and the direction table are built for fast descriptor generation.

## A. Keypoint Extraction

1) Edge Point Extraction: In keypoint extraction, we roughly divide a LiDAR point cloud into two types: edge points and plane points. The main difference between edge points and plane points is the smoothness of the local surface where the points are located. Given a 3D LiDAR point cloud $P _ { c } ,$ let i be a point in $P _ { c } .$ $P _ { s }$ is a set of continuous points on the same scan line as point i, and evenly distributed on both sides of i. S is the cardinality of $P _ { s }$ . The smooth term of the current point i is defined as follows:

$$
\nabla _ { i } = \frac { 1 } { | S | } \left\| \sum _ { j \in P _ { s } , j \neq i } { ( \vec { p _ { j } } - \vec { p _ { i } } ) } \right\| ^ { 2 }\tag{1}
$$

where $\vec { p _ { i } }$ and $\vec { p _ { j } }$ are the coordinates of the two points i and $j ,$ respectively. The edge points (as shown in Fig. 3(a)) are extracted with greater than a threshold Th .

2) Edge Keypoint Aggregation: After obtaining the edge points, there are lots of points whose are above the threshold $T h _ { \nabla }$ , but they are not stable. Specifically, these unstable points appear in the current scan but may not appear in the next scan.

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/5726c2b5a456b741bc0f58a7dfa3e861f3420415ecf6d7bf30abbc3914412f4d.jpg)  
Fig. 2. Workflow of the proposed LinK3D in terms of keypoint extraction and description. The keypoint extraction is first executed to generate aggregation keypoints. Afterward, the descriptor generation algorithm is performed to derive an efficient keypoint descriptor.

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/162d62bc2a23a74ab37f1c9129b494c3d8b63dfb08a0b24e863e3dedc3c18422.jpg)  
Fig. 3. Scattered edge points (marked by the red dashed box) and the clustered edge keypoints (marked by the blue dashed box) are in (a). The clustered edge keypoints are what we need. (b) Shows the extracted edge keypoints by Algorithm 1.

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/d6ebdfce19abcd363a86234a60ff57ab9c803620d2c43e238583785bf1f068e8.jpg)  
Fig. 4. Illustration of the aggregation process. The points are first divided into corresponding sector areas based on the angle in the XoY plane of the LiDAR coordinate system. Then the clustering operation is only performed within each sector area, rather than directly within the whole space. The purple points are what we need to generate the aggregation keypoints (red points), and the green points are the scattered points that will be filtered out.

As marked by the red dashed boxes in Fig. 3(a), the unstable points are usually scattered. Therefore, it is necessary to filter out these points and find the valid edge keypoints. As marked by the blue dashed boxes in Fig. 3(a), the valid edge keypoints are usually distributed vertically in clusters.

In this letter, a keypoint aggregation algorithm is designed to find valid edge keypoints. As illustrated in Fig. 4, the angle information guidance is used for accelerating the aggregation process. The motivation is that the points belonging to the same vertical edge usually have approximately the same angle in the XoY plane (assuming the Z-axis is upward) of the LiDAR coordinate system. The angle of point $\vec { p _ { i } }$ is given by:

$$
\theta _ { i } = \arctan { ( \vec { p _ { i } } . y / \vec { p _ { i } } . x ) }\tag{2}
$$

Therefore, we first divide the XoY plane centered on the origin of the LiDAR coordinate system into $N _ { s e c t }$ sector areas equally, then only cluster the points in each sector area rather than cluster in the whole space.

The specific algorithm is shown in Algorithm 1. It is worth noting that our algorithm runs about 25 times faster than the classical K-Means algorithm when we set $N _ { s e c t } = 1 2 0$ in our experiment. The extracted edge keypoints are shown in Fig. 3(b). It can be seen that our algorithm can filter out the invalid edge points and find the positive edge keypoints. In addition, the centroid of each cluster point is calculated and named as an aggregation keypoint, which will be used for subsequent descriptor generation.

## B. Descriptor Generation

1) Descriptor Generation Process: In descriptor generation, all aggregation keypoints are first projected to the XoY plane (assuming the Z-axis of LiDAR is upward), which can eliminate the influence caused by the uneven distribution of clustered edge keypoints along the Z-axis direction. For fast matching, our LinK3D descriptor is represented as a 180-dimensional vector, which uses 0 or the distance between the current keypoint and its neighboring keypoints to represent each dimension. As shown in Fig. 5, we divide the XoY plane into 180 sector areas centered on the current keypoint $k _ { 0 } ,$ , and each descriptor dimension corresponds to a sector area. Inspired by the 2D descriptor SIFT [3], which searches the main direction to ensure the rotation invariance, the main direction of LinK3D is also searched and represented as the direction vector from the current keypoint $k _ { 0 }$ to its closest keypoint $k _ { 1 }$ , which is located in the first sector area. The other sector areas are arranged in counterclockwise order. Afterward, the closest keypoint of $k _ { 0 }$ is searched in each sector. If there is the closest keypoint in a sector, we use the distance between the current keypoint and the closest keypoint to represent the corresponding dimension value in the descriptor. Otherwise, the value is set to 0.

During the process, the direction from the current point k<sub>0</sub> to other points $k _ { j } ( j \neq 1 )$ is expressed as ${ \vec { m } } _ { 0 j } ,$ , and we use the angle between $\vec { m } _ { 0 j }$ and the main direction m- <sub>01</sub> to determine which sector $k _ { j }$ belongs to. The angle is calculated by:

$$
\theta _ { j } = \left\{ \begin{array} { l l } { \operatorname { a r c c o s } \frac { \vec { m } _ { 0 1 } \cdot \vec { m } _ { 0 j } } { \vert \vec { m } _ { 0 1 } \vert \vert \vec { m } _ { 0 j } \vert } } & { i f D _ { j } > 0 } \\ { 2 \pi - \operatorname { a r c c o s } \frac { \vec { m } _ { 0 1 } \cdot \vec { m } _ { 0 j } } { \vert \vec { m } _ { 0 1 } \vert \vert \vec { m } _ { 0 j } \vert } } & { i f D _ { j } < 0 } \end{array} \right.\tag{3}
$$

Algorithm 2: Descriptor Generation Algorithm.   
Algorithm 1: Keypoints Aggregation Algorithm.   
Input : P e: ∇ > Thy edge points Input : $K _ { a } \mathrm { : }$ AggregationKeypoints   
Output: ValidEdgeKeypoints, AggregationKeypoints Output: Descriptors   
1 Main Loop: 1 Main Loop:   
2 for each point $p _ { i } \in P _ { e }$ do 2 for each point $k _ { i } \in K _ { a }$ do   
Sectors←DividePointToSectorBasedOnEq.2(pi); 3 for each point $k _ { j } \in K _ { a } , k _ { j } \ne k _ { i }$ do   
4 end 4 Tabledist ← ComputeDistance(ki, kj);   
5 for each Sector ∈ Sectors do 5 Tabledire ← ComputeDirection(ki, kj);   
6 FirstCluster ← CreateCluster(any point in Sector); 6 end   
7 Clusters.InsertCluster(FirstCluster); 7 end   
8 for other point $p _ { j } \in$ Sector do 8 for each point $k _ { i } \in K _ { a }$ do   
9 for each Cluster ∈ Clusters do 9 ClosestPt ← SearchClosestPoint(ki , $T a b l e _ { d i s t } ) ;$   
10 Center ← ComputeClusterCenter(Cluster):   
10 Sectors.InsertPointToFirstSector(ClosestPt) ;   
11 dist ← ComputeHorizontalDist(p, Center); 11 MainDire ← GetDirection(ki, ClosestPt, Tabledire);   
12 if dist $< T h _ { d i s t }$ then 12 for each point $k _ { j } \in K _ { a } , k _ { j } \neq k _ { i } , k _ { j }$ ≠ ClosestPoint do   
13 Cluster.UpdateCluster $( p _ { j } ) ;$   
13 OtherDire ← GetDirection $( k _ { i } , k _ { j } , T a b l e _ { d i r e } ) ;$   
14 else if Cluster is the end one then   
14 $\theta _ { j } $ ComputeθInEq3(MainDire, OtherDire);   
15 NewCluster ← CreateCluster(p);   
15 Sectors.InsertPointBasedOnθ(k, θj);   
16 Clusters.Insert(NewCluster);   
16 end   
17 else   
18 continue; 17 Define a 180-dimensional Descriptor;   
19 end 18 for each sector ∈ Sectors do   
20 end 19 if sector.NumberOfPoints == 0 then   
21 end 20 CorrespondingDimentionInDescriptor = 0;   
22 for each Cluster ∈ Clusters do 21 else   
23 Npoint ← CountNumberOfPoint(Cluster); 22 Dis ← SearchClosestDist(sector, Tabledist);   
24 $N _ { l i n e } \gets$ CountNumberOfScanLine(Cluster): 23 CorrespondingDimInDescriptor = Dis;   
25 if $N _ { p o i n t } > T h _ { p o i n t }$ && $N _ { l i n e } > T h _ { l i n e }$ then 24 end   
26 ValidEdgeKeypoints.Insert(Cluster.Points); 25 end   
27 AggregationKeypoints.Insert(Cluster.Center); 26 Descriptors.InsertNewDescriptor(Descriptor);   
28 end 27 end   
29 end   
30 end

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/5a7640bd10755ec5b0e71d93baab6df5190bfeda3df2c9b84766832adf4a80ea.jpg)  
Fig. 5. Illustration of the descriptor generation. The XoY plane, centered on the current keypoint $k _ { o }$ is divided into 180 sector areas. We first search for the closest keypoint k<sub>1</sub> of k<sub>0</sub>, then the main direction is the vector from $k _ { 0 }$ to $k _ { 1 }$ . Then the closest keypoint in each sector area is searched for. The searched keypoints k<sub>1</sub>, k<sub>2</sub>, k<sub>3</sub>, etc., are used for describing the current $k _ { \mathrm { 0 } } .$ . Through the vectorizing operation, by using the distance values between $k _ { o }$ and k<sub>1</sub>, k<sub>2</sub>, k<sub>3</sub>, etc., the 180-dimensional LinK3D descriptor will be obtained.

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/cb91dba77d319e8102f6d46a2e2734774d60e3697c99f391bc41c9530c30f874.jpg)  
Fig. 6. Value of each dimension in the final descriptor corresponds to the non-zero value with the highest priority among Des1, Des2, and Des3.

where $D _ { j }$ is defined as:

$$
D _ { i } = { \bigg | } { x _ { 1 } \atop x _ { j } \quad y _ { j } } { \bigg | }\tag{4}
$$

2) Issues of Descriptor Generation Process: There are two main issues with the above-mentioned algorithm. One issue is that the algorithm is sensitive to the closest keypoint. In the presence of interference from an outlier keypoint, the matching will fail. The other issue is that we have to calculate the relative distance and direction between two points frequently, so there will be lots of repeated calculations. To solve the first issue, we search for a certain number of the closest keypoints. Suppose we search for the 3 closest keypoints, and the corresponding 3 descriptors are calculated, as shown in Fig. 6. Des1 corresponds to the closest keypoint, and Des3 corresponds to the third closest keypoint. We define the priorities based on the distances between them and the current keypoint. Des1 has the highest priority due to its closest distance, and Des3 has the lowest priority because of its furthest distance. The value of each dimension in the final descriptor corresponds to the non-zero value with the highest priority among them. As marked by the red dashed box in Fig. 6, Des1 has a non-zero value $D _ { 0 } ^ { 1 }$ , and its corresponding value in the final descriptor is also set as ${ \dot { D } } _ { 0 } ^ { 1 }$ due to its high priority. The other two cases are shown in purple and black dashed boxes in Fig. 6. To solve the second issue, we build the distance table and the direction table for all keypoints to avoid repeated calculations by looking up the tables. The specific descriptor generation is shown in Algorithm 2, which shows the process of extracting one descriptor.

Algorithm 3: Matching Algorithm.   
Input : Descriptors\_1, Descriptors\_2   
Output: MatchPairs: matched descriptor index pairs   
1 Main Loop:   
2 Define $S e t O f I D _ { j } ;$   
3 Define RBtree $\dot { I D } _ { j \_ I D _ { i } } , R B t r e e \_ I D _ { i } .$ \_Score;   
4 for each descriptor $D _ { i } \in I$ Descriptors\_1 do   
5 Define HighestScore and $H i g h e s t S c o r e I D _ { j } ;$   
6 for each descriptor $D _ { j } \in$ Descriptors\_2 do   
7 Score ← GetSimilarityScore $( D _ { i } , D _ { j } ) ;$   
8 if Score > HighestScore then   
9 HighestScore = Score;   
10 $H i g h e s t S c o r e I D _ { j } = I D _ { j } ;$   
11 end   
12 end   
13 $S e t O f I D _ { j }$ .Insert(HighestScoreID);   
14 RBtree $\underline { { { I D } } } _ { j \_ I } \underline { { { I D } } } _ { i } . \mathrm { I n s e r t } ( H i g h e s t \underline { { { S c o r e I D } } } _ { j } , I D _ { i } ) ;$   
15 RBtree\_IDi\_Score.Insert(IDi, HighestScore);   
16 end   
# Removing one-to-multiple matches for Descriptors\_2   
17   
18 for each $I D _ { j } \in S e t O f I D _ { j }$ do   
19 $A l l I D _ { i } \gets \mathrm { G e t A l l O f } D _ { i } ( I D _ { j } , R B t r e e \_ I D _ { j \_ I D _ { i } } ) ;$   
20 HighestScore ${ \mathbf { } } I D _ { i } \gets$ GetHighestScoreD(AllIDi,   
RBtree $I D _ { i . }$ \_Score);   
21 if HighestScore\_IDi.Score $\ge T h _ { s c o r e }$ then   
22 MatchPairs.Insert(HighestScore $\underline { { I D _ { i } } } . I D _ { i } , I D _ { j } ) ;$   
23 end   
24 end

## C. Matching Two Descriptors

To measure the similarity oftwo descriptors, we only count the non-zero dimensions of two descriptors. Specifically, the absolute value of the difference between the corresponding non-zero dimension is computed in the two descriptors. If the difference is less than 0.2, the similarity score is increased by 1. If the similarity score of a match is greater than the threshold $T h _ { s c o r e } ,$ the match is considered valid. The specific matching algorithm is shown in Algorithm 3. After matching two descriptors, the matches between the edge keypoints are searched in clusters. Specifically, the corresponding edge keypoints that have the highest (smooth term) in each scan line are first selected. Then, the edge keypoints located on the same scan line are matched.

## IV. EXPERIMENTS

In this section, we first perform the basic matching performance and real-time performance evaluation, then we extend LinK3D to the LiDAR odometry task. KITTI odometry [17], M2DGR [22] and StevenVLP [23] datasets are used for the evaluation. The point clouds in KITTI were collected from different street environments (such as inner city, suburb, forest, and high-way) by a 64-beam Velodyne LiDAR at a rate of 10 Hz. The point clouds of M2DGR are collected by a 32-beam Velodyne LiDAR, and the point clouds of Steven were collected from the campus scene by a 16-beam Velodyne LiDAR at a rate of 10 Hz. The frequency (10 Hz) of the LiDAR requires the processing time of all algorithms to run within 100 milliseconds. The parameters of our algorithm are set as follows: $T h _ { \nabla } = 1 0$ in Section $\mathrm { I I I - A 1 ; } T h _ { d i s t } = 0 . 4 , T h _ { p o i n t } = 1 2 , T h _ { l i n e } = 4$ in Algorithm $1 ; T h _ { s c o r e } = 3$ in Section III-C. The code is executed on a desktop with an Intel Core i7-12700 @ 2.10 GHz processor and 16 GB of RAM.

## A. Matching Performance Comparisons With Hand-Crafted Features

Feature matching is a basic function for hand-crafted 3D local features. In this subsection, the typical city scene (Seq.00), rural scene (Seq.03), dynamic scene (Sec.04), large pose (Sec.08), and forest scene (Sec.09) in KITTI are used for the evaluation. Gate\_01, Street\_03 in M2DGR, and the first sequence in Steven VLP16 are used for the evaluation. The state-of-the-art handcrafted 3D features: 3DSC [26], PFH [12], FPFH [5], SHOT [27], BSHOT [7], and 3DHoPD [18] are used for the comparison. We first extract our LinK3D aggregation keypoints for comparison methods and compare their matching results with our method on the same scenes. Then we also extract the ISS [24] and Sift3D [25] keypoints for comparison descriptors on the same scenes, and the number of the two keypoints is similar to the number of edge keypoints in LinK3D. For metric methods, we follow the metrics used in [4], and use the number of inliers and inliers% as the validation metrics. RANSAC [28] is used for removing the mismatches, and its acceptance threshold is set as 0.5. For a fair comparison, we first compute the average number ofinliers and inliers % for our method in different scenes (except the large pose situation, which is a randomly selected reverse loop), then use two matched LiDAR scans, the number of inliers and the inlier% of which are approximately the same as the average, rather than use the scans with more inliers and a higher inlier% for our method. Then we extract a similar number of ISS and Sift3D keypoints for comparison methods. The used scan ID and the specific number of edge keypoints are shown in Table I. The matching results of our method on each scene are shown in Fig. 7, and the comparison results on different scenes are shown in Tables II and III.

From Tables II and III, we can see that our method obtains more inliers on each scene than comparison methods. On the challenging forest scene, even though the edge features are unobvious and have a bad effect on the inliers% of our method, the number of inliers generated by LinK3D is still considerable. On the challenging dynamic scene, the highly dynamic carrier has a bad effect on our method, which reduces the inlier% for our method. It is noted that our method can effectively match the two scans with a large pose transformation (reverse direction loop), which can be seen in Fig. 7(d). Moreover, although the comparison methods achieve comparable results on the point clouds collected from 64-beam LiDAR, due to the sparser points collected from 32-beam and 16-beam LiDAR (the sparsity can be seen from Fig. 7(f)–(h)) on the M2DGR and Steven VLP16 datasets, the number of inliers for most comparison methods is zero, and they may be unreliable on the point clouds generated by 32-beam and 16-beam LiDAR.

TABLE I  
NUMBER OF EXTRACTED KEYPOINTS IN DIFFERENT SCENES
<table><tr><td>Keypoint</td><td>City Scene (169/170)</td><td>Rural Scene (580/581)</td><td>Dynamic Scene (87/88)</td><td>Lage Pose (232/1647)</td><td>Forest Scene (17/18)</td><td>Gate_01 (121/122)</td><td>Street_03 (109/110)</td><td>Steven VLP16 (22/23)</td></tr><tr><td>ISS [24]</td><td>2985/2863</td><td>2459/2508</td><td>3460/3396</td><td>2467/2404</td><td>2869/2949</td><td>1184/1143</td><td>1153/1120</td><td>222/213</td></tr><tr><td>Sift3D [25]</td><td>2897/2842</td><td>2485/2442</td><td>3428/3356</td><td>2497/2478</td><td>2853/2877</td><td>1162/1155</td><td>1248/1158</td><td>229/237</td></tr><tr><td>LinK3D(ours)</td><td>2890/2902</td><td>2359/2284</td><td>3380/3428</td><td>2411/2604</td><td>2928/3006</td><td>1177/1162</td><td>1204/1156</td><td>218/226</td></tr></table>

The specific IDs of two matched scans are enclosed in parentheses.

![](images/2024_LinK3D__Linear_Keypoints_Representation_for_3D_LiDAR_Poi/cdfa230dded41219c15870fca6cc008ae4d41776486748f4157e8e241325f38c.jpg)  
Fig. 7. Matching results of LinK3D on different scenes of KITTI, M2DGR, and Steven VLP16. The green lines are the valid matches after RANSAC.

TABLE II  
COMPARISON RESULTS OF MATCHING PERFORMANCE IN DIFFERENT SCENES OF THE KITTI DATASET
<table><tr><td rowspan="2">Keypoint</td><td rowspan="2">Descriptor</td><td colspan="3">City Scene</td><td colspan="3">Rural Scene</td><td colspan="3">Dynamic Scene</td><td colspan="3">Large Pose</td></tr><tr><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td></tr><tr><td rowspan="6">LinK3D</td><td>3DHoPD [18]</td><td>32</td><td>0</td><td>0</td><td>20</td><td>0</td><td>0</td><td>49</td><td>0</td><td>0</td><td>35</td><td>0</td><td>0</td></tr><tr><td>PFH [12]</td><td>60</td><td>11</td><td>18.3</td><td>17</td><td>5</td><td>29.4</td><td>65</td><td>0</td><td>0</td><td>31</td><td>0</td><td>0</td></tr><tr><td>3DSC [26]</td><td>1</td><td>0</td><td>0</td><td>0</td><td>0</td><td></td><td>1</td><td>0</td><td>0</td><td>0</td><td>0</td><td>-</td></tr><tr><td>SHOT [27]</td><td>0</td><td>0</td><td>-</td><td>0</td><td>0</td><td></td><td>2</td><td>0</td><td>0</td><td>0</td><td>0</td><td></td></tr><tr><td>FPFH [5]</td><td>8</td><td>0</td><td>0</td><td>0</td><td>0</td><td>=</td><td>6</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td></tr><tr><td>BSHOT [7]</td><td>45</td><td>33</td><td>73.3</td><td>20</td><td>7</td><td>35.0</td><td>35</td><td>15</td><td>42.9</td><td>24</td><td>12</td><td>50.0</td></tr><tr><td colspan="2">LinK3D (Aggregation)</td><td>135</td><td>71</td><td>52.6</td><td>110</td><td>48</td><td>43.6</td><td>147</td><td>39</td><td>26.5</td><td>134</td><td>50</td><td>37.3</td></tr><tr><td rowspan="6">ISS</td><td>3DHoPD [18]</td><td>2423</td><td>280</td><td>11.6</td><td>2119</td><td>83</td><td>3.9</td><td>2967</td><td>341</td><td>11.5</td><td>2001</td><td>7</td><td>0.3</td></tr><tr><td>PFH [12]</td><td>1916</td><td>360</td><td>18.8</td><td>839</td><td>66</td><td>7.9</td><td>1943</td><td>75</td><td>3.9</td><td>717</td><td>9</td><td>1.3</td></tr><tr><td>3DSC [26]</td><td>1987</td><td>6</td><td>0.3</td><td>2214</td><td>4</td><td>0.2</td><td>2969</td><td>4</td><td>0.13</td><td>2169</td><td>0</td><td>0</td></tr><tr><td>SHOT [27]</td><td>219</td><td>111</td><td>50.7</td><td>89</td><td>34</td><td>38.2</td><td>479</td><td>121</td><td>25.2</td><td>23</td><td>4</td><td>17.4</td></tr><tr><td>FPFH [5]</td><td>423</td><td>101</td><td>23.9</td><td>104</td><td>8</td><td>7.7</td><td>442</td><td>34</td><td>7.7</td><td>193</td><td>4</td><td>2.1</td></tr><tr><td>BSHOT [7]</td><td>641</td><td>380</td><td>59.3</td><td>330</td><td>109</td><td>33.0</td><td>539</td><td>129</td><td>23.9</td><td>274</td><td>52</td><td>19.0</td></tr><tr><td rowspan="6">Sift3D</td><td>3DHoPD [18]</td><td>2197</td><td>162</td><td>7.4</td><td>1809</td><td>3</td><td>0.17</td><td>2736</td><td>224</td><td>8.2</td><td>1722</td><td>4</td><td>0.2</td></tr><tr><td>PFH [12]</td><td>1803</td><td>383</td><td>21.2</td><td>743</td><td>5</td><td>0.7</td><td>1753</td><td>116</td><td>6.6</td><td>755</td><td>0</td><td>0</td></tr><tr><td>3DSC [26]</td><td>1777</td><td>0</td><td>0</td><td>1840</td><td>0</td><td>0</td><td>2336</td><td>0</td><td>0</td><td>1874</td><td>0</td><td>0</td></tr><tr><td>SHOT [27]</td><td>158</td><td>80</td><td>50.6</td><td>35</td><td>20</td><td>57.1</td><td>298</td><td>111</td><td>37.2</td><td>9</td><td>0</td><td>0</td></tr><tr><td>FPFH [5]</td><td>534</td><td>113</td><td>21.2</td><td>238</td><td>0</td><td>0</td><td>458</td><td>25</td><td>5.5</td><td>340</td><td>0</td><td>0</td></tr><tr><td>BSHOT [7]</td><td>623</td><td>403</td><td>64.7</td><td>332</td><td>72</td><td>21.7</td><td>495</td><td>137</td><td>27.7</td><td>246</td><td>47</td><td>19.1</td></tr><tr><td colspan="2">LinK3D (Edge)</td><td>1090</td><td>602</td><td>55.2</td><td>816</td><td>403</td><td>49.4</td><td>1068</td><td>413</td><td>38.7</td><td>681</td><td>367</td><td>53.9</td></tr></table>

## B. Real-Time Performance Evaluation

In this section, we evaluate the efficiency of our method and compare it with the DNN-based and handcrafted methods. KITTI 00 is used for the evaluation of handcrafted methods, which includes 4500+ LiDAR scans, and each LiDAR scan contains approximately 120000+ points. We compute the average runtime for our method when the sequential LiDAR scans are matched. We compute the average runtime after multiple measurements on the used city scene in Section IV-A for other hand-crafted methods. For DNN-based methods, the runtime of StickyPillars is evaluated on KITTI, and others are evaluated on 3DMatch [30], which is collected from an RGB-D camera and each frame of which contains approximately 27000+ points (< KITTI’s) on average. Table IV shows the comparison results. We can see that the DNN methods usually take more time, although fewer points are used and GPUs are required. Other handcrafted methods cannot achieve the real-time performance. LinK3D only takes about 50 milliseconds to extract and match features on average, and it shows great real-time performance.

TABLE III  
COMPARISON RESULTS OF MATCHING PERFORMANCE ON THE FOREST SCENE OF KITTI AND THE M2DGR AND STEVEN VLP16 DATASETS
<table><tr><td rowspan="2">Keypoint</td><td rowspan="2">Descriptor</td><td colspan="3">Forest Scene</td><td colspan="3">Gate_01</td><td colspan="3">Street_03</td><td colspan="3">一 StevenVLP</td></tr><tr><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td><td> $N _ { M }$ </td><td> $N _ { i n l i e r s }$ </td><td>inliers%</td></tr><tr><td rowspan="6">LinK3D</td><td>3DHoPD [18]</td><td>31</td><td>0</td><td>0</td><td>17</td><td>7</td><td>41.2</td><td>23</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td></tr><tr><td>PFH [12]</td><td>68</td><td>5</td><td>7.4</td><td>27</td><td>16</td><td>59.3</td><td>16</td><td>10</td><td>62.5</td><td>3</td><td>0</td><td>0</td></tr><tr><td>3DSC [26]</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td><td></td><td>0</td><td>0</td><td>=</td><td>0</td><td>0</td><td>=</td></tr><tr><td>SHOT [27]</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td><td>=</td><td>2</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td></tr><tr><td>FPFH [5]</td><td>26</td><td>0</td><td>0</td><td>2</td><td>0</td><td>0</td><td>2</td><td>0</td><td>0</td><td>3</td><td>0</td><td>0</td></tr><tr><td>BSHOT [7]</td><td>48</td><td>28</td><td>58.3</td><td>29</td><td>21</td><td>72.4</td><td>33</td><td>21</td><td>63.6</td><td>10</td><td>5</td><td>50.0</td></tr><tr><td>LinK3D (Aggregation)</td><td></td><td>147</td><td>61</td><td>41.5</td><td>70</td><td>56</td><td>80.0</td><td>77</td><td>54</td><td>70.1</td><td>15</td><td>12</td><td>80.0</td></tr><tr><td rowspan="6">ISS</td><td>3DHoPD [18]</td><td>2447</td><td>337</td><td>13.8</td><td>775</td><td>38</td><td>4.9</td><td>741</td><td>4</td><td>0.5</td><td>51</td><td>0</td><td>0</td></tr><tr><td>PFH [12]</td><td>1507</td><td>98</td><td>6.5</td><td>617</td><td>4</td><td>0.6</td><td>540</td><td>0</td><td>0</td><td>141</td><td>0</td><td>0</td></tr><tr><td>3DSC [26]</td><td>2530</td><td>4</td><td>0.16</td><td>1057</td><td>0</td><td>0</td><td>1004</td><td>0</td><td>0</td><td>203</td><td>0</td><td>0</td></tr><tr><td>SHOT [27]</td><td>159</td><td>88</td><td>55.3</td><td>30</td><td>19</td><td>63.3</td><td>22</td><td>15</td><td>68.2</td><td>0</td><td>0</td><td>=</td></tr><tr><td>FPFH [5]</td><td>340</td><td>32</td><td>9.4</td><td>476</td><td>0</td><td>0</td><td>393</td><td>0</td><td>0</td><td>120</td><td>0</td><td>0</td></tr><tr><td>BSHOT [7]</td><td>438</td><td>228</td><td>52.1</td><td>253</td><td>92</td><td>36.4</td><td>253</td><td>113</td><td>44.7</td><td>59</td><td>10</td><td>16.9</td></tr><tr><td rowspan="6">Sift3D</td><td>3DHoPD [18]</td><td>2128</td><td>195</td><td>9.2</td><td>673</td><td>0</td><td>0</td><td>669</td><td>6</td><td>0.9</td><td>82</td><td>0</td><td>0</td></tr><tr><td>PFH [12]</td><td>1310</td><td>100</td><td>7.6</td><td>636</td><td>0</td><td>0</td><td>518</td><td>0</td><td>0</td><td>142</td><td>0</td><td>0</td></tr><tr><td>3DSC [26]</td><td>1679</td><td>0</td><td>0</td><td>851</td><td>0</td><td>0</td><td>866</td><td>0</td><td>0</td><td>116</td><td>0</td><td>0</td></tr><tr><td>SHOT [27]</td><td>82</td><td>56</td><td>68.3</td><td>19</td><td>12</td><td>63.2</td><td>17</td><td>16</td><td>94.1</td><td>1</td><td>0</td><td>0</td></tr><tr><td>FPFH [5]</td><td>343</td><td>5</td><td>1.5</td><td>429</td><td>0</td><td>0</td><td>337</td><td>0</td><td>0</td><td>115</td><td>0</td><td>0</td></tr><tr><td>BSHOT [7]</td><td>387</td><td>192</td><td>49.6</td><td>236</td><td>102</td><td>43.2</td><td>218</td><td>97</td><td>44.5</td><td>25</td><td>9</td><td>36.0</td></tr><tr><td colspan="2">LinK3D (Edge)</td><td>1015</td><td>455</td><td>44.8</td><td>346</td><td>280</td><td>80.9</td><td>405</td><td>298</td><td>73.6</td><td>100</td><td>87</td><td>87.0</td></tr></table>

TABLE IV

EFFICIENCY COMPARISON OF DIFFERENT METHODS
<table><tr><td rowspan="2">Method</td><td rowspan="2"> $T _ { e x t r a c t i o n }$ </td><td rowspan="2"> $T _ { m a t c h i n g }$ </td><td rowspan="2"></td><td rowspan="2">GPU required</td></tr><tr><td> $T _ { t o t a l }$ </td></tr><tr><td>3DFeatNet [15]</td><td>0.928</td><td></td><td>0.928</td><td>√</td></tr><tr><td>3DSmoothNet [29]</td><td>0.414</td><td>一</td><td>0.414</td><td>√</td></tr><tr><td>DNN-bsed DH3D [9]</td><td>0.080</td><td></td><td>0.080</td><td>√</td></tr><tr><td>FCGF [16]</td><td>0.360</td><td>一</td><td>0.360</td><td>√</td></tr><tr><td>D3Feat [10]</td><td>0.130</td><td></td><td>0.130</td><td>√</td></tr><tr><td>StickyPillars [11]</td><td>0.015</td><td>0.101</td><td>0.116</td><td>√</td></tr><tr><td>PFH [12] Ha-ted</td><td>11.148</td><td>0.077</td><td>11.225</td><td>X</td></tr><tr><td>FPFH [5]</td><td>5.601</td><td>0.015</td><td>5.616</td><td>X</td></tr><tr><td>3DSC [26]</td><td>0.023</td><td>7.106</td><td>7.129</td><td>X</td></tr><tr><td>SHOT [27]</td><td>0.490</td><td>2.206</td><td>2.696</td><td>X</td></tr><tr><td>BSHOT [7]</td><td>0.574</td><td>0.057</td><td>0.631</td><td>X</td></tr><tr><td>3DHoPD [18]</td><td>0.414</td><td>0.005</td><td>0.419</td><td>X</td></tr><tr><td>LinK3D (ours)</td><td>0.030</td><td>0.020</td><td>0.050</td><td>X</td></tr></table>

The data for 3DFeatNet, 3DSmoothNet, and DH3D is from [9], and the data for other DNN-based methods is from their source paper. All units are in seconds

## C. Matching Failure Case on Unstructured Scenes

When we evaluated our algorithm on KITTI (from sequence 00 to 10), we found that our method may fail to generate true matches between two sequential LiDAR scans after RANSAC in the unstructured KITTI 01, 02, and 09 with fewer valid edge features. KITTI 01 is collected from a high-way scene, and some LiDAR scans of KITTI 02 and 09 are collected from the forest scene. We use the ground truth to determine the success rate on the three sequences, which is shown in Table V. The results indicate that our method may not be robust for the unstructured scenes with fewer effective edge features. In the future, we will continue to improve the robustness of our method for unstructured scenes.

SUCCESS RATE OF LINK3D IN UNSTRUCTURED HIGH-WAY AND FOREST SCENES ON THE KITTI DATASET  
TABLE V
<table><tr><td></td><td>KITTI 01</td><td>KITTI 02</td><td>KITTI 09</td></tr><tr><td>Success Rate (%)</td><td>88.6</td><td>97.6</td><td>99.9</td></tr></table>

TABLE VI  
COMPARISON RESULTS OF LIDAR ODOMETRY ON KITTI DATASET
<table><tr><td rowspan="2">Sequence</td><td colspan="2">CLS [32]</td><td colspan="2">LOAM [33]</td><td colspan="2">LO-Net [34]</td><td colspan="2">Ours</td></tr><tr><td> $t _ { r e l }$ </td><td> $r _ { r e l }$ </td><td> $t _ { r e l }$ </td><td> $r _ { r e l }$ </td><td> $t _ { r e l }$ </td><td> $r _ { r e l }$ </td><td> $t _ { r e l }$ </td><td> $r _ { r e l }$ </td></tr><tr><td>00</td><td>2.11</td><td>0.95</td><td>0.78</td><td>0.53</td><td>0.78</td><td>0.42</td><td>0.71</td><td>0.32</td></tr><tr><td>01</td><td>4.22</td><td>1.05</td><td>1.43</td><td>0.55</td><td>1.42</td><td>0.40</td><td>4.81</td><td>0.50</td></tr><tr><td>02</td><td>2.29</td><td>0.86</td><td>0.92</td><td>0.55</td><td>1.01</td><td>0.45</td><td>1.33</td><td>0.40</td></tr><tr><td>03</td><td>1.63</td><td>1.09</td><td>0.86</td><td>0.65</td><td>0.73</td><td>0.59</td><td>1.39</td><td>0.64</td></tr><tr><td>04</td><td>1.59</td><td>0.71</td><td>0.71</td><td>0.50</td><td>0.56</td><td>0.54</td><td>0.57</td><td>0.40</td></tr><tr><td>05</td><td>1.98</td><td>0.92</td><td>0.57</td><td>0.38</td><td>0.62</td><td>0.35</td><td>0.63</td><td>0.27</td></tr><tr><td>06</td><td>0.92</td><td>0.46</td><td>0.65</td><td>0.39</td><td>0.55</td><td>0.33</td><td>0.64</td><td>0.28</td></tr><tr><td>07</td><td>1.04</td><td>0.73</td><td>0.63</td><td>0.50</td><td>0.56</td><td>0.45</td><td>0.63</td><td>0.26</td></tr><tr><td>08</td><td>2.14</td><td>1.05</td><td>1.12</td><td>0.44</td><td>1.08</td><td>0.43</td><td>1.03</td><td>0.32</td></tr><tr><td>09</td><td>1.95</td><td>0.92</td><td>0.77</td><td>0.48</td><td>0.77</td><td>0.38</td><td>0.74</td><td>0.29</td></tr><tr><td>10</td><td>3.46</td><td>1.28</td><td>0.79</td><td>0.57</td><td>0.92</td><td>0.41</td><td>1.17</td><td>0.42</td></tr><tr><td>Mean</td><td>2.12</td><td>0.91</td><td>0.84</td><td>0.51</td><td>0.82</td><td>0.43</td><td>1.24</td><td>0.37</td></tr></table>

## D. Application: LiDAR Odometry

LiDAR odometry is usually the front-end of the LiDAR SLAM system, and the estimated results of LiDAR odometry have an effect on the accuracy of the SLAM back-end. In this part, we embed LinK3D in combination with a subsequent mapping step. For this purpose, we replace the scan-to-scan registration step of A-LOAM<sup>2</sup> with the registration results of SVD solution [31] based on the matching results ofLinK3D. The original registration results of A-LOAM are used when LinK3D encounters matching failure cases. We compare our method with CLS [32], LOAM [33] and LO-Net [34] in Table VI, and use the KITTI odometry metrics [17] to quantitatively analyze the accuracy ofLiDAR odometry. The results are shown in Table VI. It can be seen that the odometry based on LinK3D achieves comparable estimation results.

## V. CONCLUSION AND FUTURE WORK

In this work, we propose a novel 3D feature representation method to solve the issue that the existing methods are not fully applicable to the sparse 3D LiDAR point cloud, which is called LinK3D. The core idea of LinK3D is to describe the current keypoint with its neighboring keypoints. The experimental results show that LinK3D can generate a large number of valid matches on sparse 16-, 32-, and 64-beam LiDAR point clouds in real time. In the future, we will continue to improve the robustness of LinK3D for unstructured scenes. It is promising to extend LinK3D into more possible 3D vision tasks (e.g., place recognition and relocalization of mobile robots) to improve the efficiency and accuracy of different LiDAR-based mobile robot systems.

## REFERENCES

[1] Y. Hu, S. Fang, W. Xie, and S. Chen, “Aerial monocular 3D object detection,” IEEE Robot. Automat. Lett., vol. 8, no. 4, pp. 1959–1966, Apr. 2023.

[2] S. Lee, L. Chen, J. Wang, A. Liniger, S. Kumar, and F. Yu, “Uncertainty guided policy for active robotic 3D reconstruction using neural radiance fields,” IEEE Robot. Automat. Lett., vol. 7, no. 4, pp. 12070–12077, Oct. 2022.

[3] D. G. Lowe, “Distinctive image features from scale-invariant keypoints,” Int. J. Comput. Vis., vol. 60, no. 2, pp. 91–110, 2004.

[4] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: An efficient alternative to sift or surf,” in Proc. Int. Conf. Computer Vis., 2011, pp. 2564–2571.

[5] R. B. Rusu, N. Blodow, and M. Beetz, “Fast point feature histograms (FPFH) for 3D registration,” in Proc. IEEE Int. Conf. Robot. Automat., 2009, pp. 3212–3217.

[6] S. Salti, F. Tombari, and L. Di Stefano, “SHOT: Unique signatures of histograms for surface and texture description,” Comput. Vis. Image Understanding, vol. 125, pp. 251–264, 2014.

[7] S. M. Prakhya, B. Liu, and W. Lin, “B-SHOT: A binary feature descriptor for fast and efficient keypoint matching on 3D point clouds,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2015, pp. 1929–1934.

[8] W. Lu, G. Wan, Y. Zhou, X. Fu, P. Yuan, and S. Song, “DeepVCP: An end-to-end deep neural network for point cloud registration,” in Proc. IEEE/CVF Int. Conf. Computer Vis., 2019, pp. 12–21.

[9] J. Du, R. Wang, and D. Cremers, “DH3D: Deep hierarchical 33D descriptors for robust large-scale 6DOF relocalization,” in Eur. Conf. Computer Vis., 2020, pp. 744–762.

[10] X. Bai, Z. Luo, L. Zhou, H. Fu, L. Quan, and C.-L. Tai, “D3Feat: Joint learning of dense detection and description of 3D local features,” in Proc. EEE/CVF Conf. Comput. Vis. Pattern Recognit., 2020, pp. 6359–6367.

<sup>2</sup>[Online]. Available: https://github.com/HKUST-Aerial-Robotics/A-LOAM

[11] K. Fischer, M. Simon, F. Olsner, S. Milz, H.-M. Groß, and P. Mader, “Stickypillars: Robust and efficient feature matching on point clouds using graph neural networks,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 313–323.

[12] R. B. Rusu, N. Blodow, Z. C. Marton, and M. Beetz, “Aligning point cloud views using persistent feature histograms,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2008, pp. 3384–3391.

[13] G. Kim, S. Choi, and A. Kim, “Scan context++ : Structural place recognition robust to rotation and lateral variations in urban environments,” IEEE Trans. Robot., vol. 38, no. 3, pp. 1856–1874, Jun. 2022.

[14] Y. Fan, Y. He, and U.-X. Tan, “Seed: A segmentation-based egocentric 3D point cloud descriptor for loop closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5158–5163.

[15] Z. J. Yew and G. H. Lee, “3DFeat-Net: Weakly supervised local 3D features for point cloud registration,” in Proc. Eur. Conf. Comput. Vis., 2018, pp. 607–623.

[16] C. Choy, J. Park, and V. Koltun, “Fully convolutional geometric features,” in Proc. IEEE/CVF Int. Conf. Computer Vis., 2019, pp. 8958–8966.

[17] A. Geiger, P. Lenz, and R. Urtasun, “Are we ready for autonomous driving? the KITTI vision benchmark suite,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2012, pp. 3354–3361.

[18] S. M. Prakhya, J. Lin, V. Chandrasekhar, W. Lin, and B. Liu, “3DHoPD: A fast low-dimensional 3-D descriptor,” IEEE Robot. Automat. Lett., vol. 2, no. 3, pp. 1472–1479, Jul. 2017.

[19] Y. Zhu, Y. Ma, L. Chen, C. Liu, M. Ye, and L. Li, “GOSMatch: Graph-of-semantics matching for detecting loop closures in 3D lidar data,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5151–5157.

[20] X. Kong et al., “Semantic graph based place recognition for 3D point clouds,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 8216–8223.

[21] Z. Qin, H. Yu, C. Wang, Y. Guo, Y. Peng, and K. Xu, “Geometric transformer for fast and robust point cloud registration,” in Proc. IEEE/CVF Conf. Computer Vis. Pattern Recognit., 2022, pp. 11143–11152.

[22] J. Yin, A. Li, T. Li, W. Yu, and D. Zou, “M2DGR: A multi-sensor and multi-scenario SLAM dataset for ground robots,” IEEE Robot. Automat. Lett., vol. 7, no. 2, pp. 2266–2273, Apr. 2022.

[23] T. Shan and B. Englot, “LeGO-LOAM: Lightweight and ground-optimized lidar odometry and mapping on variable terrain,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 4758–4765.

[24] Y. Zhong, “Intrinsic shape signatures: A shape descriptor for 3D object recognition,” in Proc. IEEE 12th Int. Conf. Computer Vis. Workshops, 2009, pp. 689–696.

[25] P. Scovanner, S. Ali, and M. Shah, “A 3D sift descriptor and its application to action recognition,” in Proc. 15th ACM Int. Conf. Multimedia, 2007, pp. 357–360.

[26] A. Frome, D. Huber, R. Kolluri, T. Bülow, and J. Malik, “Recognizing objects in range data using regional point descriptors,” in Proc. Eur. Conf. Comput. Vis., 2004, pp. 224–237.

[27] F. Tombari, S. Salti, and L. D. Stefano, “Unique signatures of histograms for local surface description,” in Proc. Eur. Conf. Computer Vis., 2010, pp. 356–369.

[28] M. A. Fischler and R. C. Bolles, “Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography,” Commun. ACM, vol. 24, no. 6, pp. 381–395, 1981.

[29] Z. Gojcic, C. Zhou, J. D. Wegner, and A. Wieser, “The perfect match: 3D point cloud matching with smoothed densities,” in Proc. IEEE/CVF Conf. Computer Vis. Pattern Recognit., 2019, pp. 5545–5554.

[30] A. Zeng, S. Song, M. Nießner, M. Fisher, J. Xiao, and T. Funkhouser, “3DMatch: Learning local geometric descriptors from RGB-D reconstructions,” in Proc. IEEE Conf. Computer Vis. Pattern Recognit., 2017, pp. 1802–1811.

[31] K. S. Arun, T. S. Huang, and S. D. Blostein, “Least-squares fitting of two 3-D point sets,” IEEE Trans. Pattern Anal. Mach. Intell., vol. PAMI-9, no. 5, pp. 698–700, Sep. 1987.

[32] M. Velas, M. Spanel, and A. Herout, “Collar line segments for fast odometry estimation from velodyne point clouds,” in Proc. IEEE Int. Conf. Robot. Automat., 2016, pp. 4486–4495.

[33] J. Zhang and S. Singh, “Loam: Lidar odometry and mapping in real-time,” in Proc. Robot.: Sci. Syst., 2014, vol. 2.

[34] Q. Li et al., “LO-Net: Deep real-time lidar odometry,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 8473–8482.