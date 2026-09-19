# SeqOT: A Spatial–Temporal Transformer Network for Place Recognition Using Sequential LiDAR Data

Junyi Ma , Student Member, IEEE, Xieyuanli Chen , Student Member, IEEE, Jingyi Xu, and Guangming Xiong

Abstract—Place recognition is an important component for autonomous vehicles to achieve loop closing or global localization. In this article, we tackle the problem of place recognition based on sequential 3-D LiDAR scans obtained by an onboard LiDAR sensor. We propose a transformerbased network named SeqOT to exploit the temporal and spatial information provided by sequential range images generated from the LiDAR data. It uses multiscale transformers to generate a global descriptor for each sequence of LiDAR range images in an end-to-end fashion. During online operation, our SeqOT finds similar places by matching such descriptors between the current query sequence and those stored in the map. We evaluate our approach on four datasets collected with different types of LiDAR sensors in different environments. The experimental results show that our method outperforms the state-of-the-art LiDAR-based place recognition methods and generalizes well across different environments. Furthermore, our method operates online faster than the frame rate of the sensor.

Index Terms—Deep learning methods, LiDAR place recognition, sequence matching.

## I. INTRODUCTION

IVEN a map, place recognition can be defined as whether is in the map. It plays an important role for most navigation systems during simultaneous localization and mapping or global localization. Vision-based place recognition has been well discussed in existing research papers [1], [2], [3]. However, cameras are usually influenced by illumination and seasonal changes, thus less reliable when used in large-scale outdoor environments. In contrast, LiDAR-based place recognition is more robust to such changes and has attracted many research interests in recent years [4], [5], [6]. To further enhance the robustness of long-term place recognition, there are several works [7], [8], [9] exploiting sequential sensor data. They generate descriptors for every single observation and mainly focus on matching the sequence of such descriptors.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/5c8e4597997ad131631f650a74ed22e2302c681edceb38e76f62890567ae2a4a.jpg)  
Fig. 1. Our proposed SeqOT exploits sequential range images from LiDAR sensors as input to extract spatial and temporal features at the same time and generate a final sequence enhanced global descriptor.

In this article, we propose a novel sequential-data-enhanced place recognition method named SeqOT based on our previous work OverlapTransformer (OT) [4]. It utilizes sequences of range images generated from 3-D LiDARs mounted on autonomous vehicles to achieve online place recognition. Unlike the existing methods using sequence-based descriptor matching, our proposed SeqOT uses multiscale transformers and generates only a single global descriptor for each sequence of LiDAR range images in an end-to-end fashion. It first uses a devised single-scan transformer (SST) module to extract features of each scan. Then, a multiscan transformer (MST) module is applied to fuse spatiotemporal information and generate subdescriptors for every continuous three scans. In the end, it exploits a pooling module to fuse subdescriptors into a final descriptor of the LiDAR sequence, which is finally used to find the correct places in the map efficiently and reliably.

The main contribution of this paper is an end-to-end network that exploits sequential LiDAR range images to achieve reliable long-term place recognition performance, which is illustrated in Fig. 1. Benefiting from the proposed yaw-rotation-invariant architecture, SeqOT is robust to the viewpoint change and the order of input scans, thus achieving reliable place recognition even when the car drives in opposite directions. To our best knowledge, it is the first network exploiting multiscale transformer modules to fuse spatial and temporal information of sequential LiDAR data and generate global descriptors for fast retrieval. Besides, our work is also the first one that achieves and mathematically proves the perfect yaw-rotation invariance using sequential LiDAR scans as input. We thoroughly evaluated our SeqOT on the Haomo and NCLT datasets. The experimental results show the superiority of our method compared with both state-of-the-art single-scan and sequence-enhanced baseline methods. We also provide zero-shot place recognition results on the KITTI and MulRan datasets using the model pretrained with the NCLT dataset to show the good generalization ability of our method. To thoroughly evaluate our approach, we furthermore provide the ablation studies on the sequence length, the transformer modules, and the yaw-rotation invariance.

In sum, we make four claims that our approach is able to 1) achieve good long-term place recognition in outdoor large-scale environments using only sequential LiDAR data, 2) generalize well into the different environments using LiDAR data obtained from different types of LiDAR sensors without fine-tuning, 3) recognize places with changing viewpoints and input sequence order based on the proposed yaw-rotation-invariant architecture, and 4) achieve online operation with runtime less than 100 ms.

## II. RELATED WORK

Place recognition is a classic topic in robotics and computer vision [2], [3], [10], [11], [12]. In this article, we focus mainly on LiDAR-based methods and refer vision-based place recognition to the survey paper by Lowry et al. [3].

Many works have been done in LiDAR-based place recognition and loop closure detection due to their accurate range measurements and the robustness of LiDAR sensors toward illumination changes. For example, Kim and Kim [6] propose scan context to count the maximum height of separated point clouds to generate a 2-D image as a descriptor for finding similar scans. Delight by Cop et al. [13] and intensity scan context by Wang et al. [14] both utilize geometric and remission information of LiDAR data to enhance the discrimination of generated descriptors. Instead of using LiDAR data in spatial domain, LiDAR Iris by Wang et al. [15] and BVMatch by Luo et al. [16] transform the point clouds into the frequency domain and generate yaw-rotation-invariant descriptors to find scans in the same place taken from different viewpoints. There are also works [17], [18] using LiDAR range images to calculate the similarities between LiDAR scans and later combined with Monte Carlo localization to achieve global localization.

Besides the abovementioned hand-crafted methods, learningbased methods have recently attracted attention with the advent of neural networks. PointNetVLAD by Uy and Lee [19] is the first method introducing the NetVLAD [11] architecture into learnable LiDAR-based place recognition. Based on NetVLAD, Liu et al. [20] propose LPD-Net using multiple local features extracted from a point cloud as input of graph neural network to generate global descriptors. In contrast, OverlapNet by Chen et al. [5], [21] utilizes siamese networks to estimate the similarity between a pair of LiDAR scans and later uses such similarities for global localization [22]. Kong et al. [23] later also propose a siamese network with multiple cues as input to learn a submap-based model. More recently, Cao et al. [24], [25] have proposed cylindrical-image-based methods to mitigate the influence of viewpoint changes. There are also works using high-level semantic information to improve LiDAR-based place recognition results, such as SGPR by Kong et al. [26] and RINet by Li et al. [27]. However, such high-level semantic information is not always available in different environments. To exploit point cloud 3-D spatial information for place recognition, Minkloc3D by Komorowski [28] and LoGG3D-Net by Vidanapathirana et al. [29] use sparse convolution for effectively extracting pointwise features. More recently, an attention mechanism [30] has been introduced to generate more discriminative descriptors for LiDAR-based place recognition. For example, PPT-Net by Hui et al. [31], NDT-Transformer by Zhou et al. [32], and our previous work OT [4] use the transformer network [30] to enhance the descriptiveness of descriptors and improve the place recognition performance.

Single-observation-based place recognition methods have achieved reasonable results. However, they are less reliable when used for long-time-span place recognition with appearance changes. Multiple vision-based place recognition methods use sequential/temporal images to enhance the performance for long-term place recognition. For example, SeqSLAM by Milford and Wyeth [7] matches sequences of images using the normalized sum of pixel intensity differences. Based on that, Fast-SeqSLAM by Siam and Zhang [33] uses a nearest neighbor algorithm to find possible candidates and reduces time complexity significantly for long-term visual place recognition. In contrast, Vysotska and Stachniss [12] propose a hashingbased image retrieval strategy for more efficient sequence-based relocalization. More recently, Garg and Milford [9] have proposed SeqNet, which combines the convolution network with a simplified version of SeqSLAM [7] to better use the relations between sequences of descriptors. They later also propose SeqMatchNet [34], which uses a triplet loss to supervise a convolutional network for image sequence matching. It improves the optimization process by using a sequence-based distance metric instead of single summary vector metrics compared with previous methods.

Although the sequence-enhanced methods have been well studied in vision-based place recognition, there are not many works for LiDAR-based methods. The early work SeqLPD by Liu et al. [8] exploits the lightweight version of their previous single-scan-based LiDAR place recognition network LPD-Net [20] together with a coarse-to-fine sequence matching approach to recognize places using LiDAR sequences. More recently, Yin et al. [35] have proposed FusionVLAD to generate multiview representations with dense submaps from sequential LiDAR scans and encoded both the top-down and spherical views of LiDAR scans. They later also propose SeqSphereVLAD [36], [37], which locates the best match using a particle-filter-based method in the global searching, thus improving the place recognition robustness. These methods have achieved comparable performance by combining single-scan/submap-based place recognition methods with sequence matching. However, they exploit the spatial and temporal information of sequential scans in a two-step manner and may not fuse them very well. Besides, they are not robust enough to yaw rotation in real vehicle applications. In this article, we propose an end-to-end sequence-enhanced place recognition method. Different to the existing sequence-enhanced methods [8], [9], [12], [36], [37], we fuse the spatial and temporal information using yaw-rotation-invariant transformer networks and directly generate one single global descriptor for each LiDAR sequence in an end-to-end fashion for fast LiDAR-sequence-based place recognition.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/e04200c10496dc67afed6663079224a3bb0cbfe1ac14ccf3d289bed6f35b4198.jpg)  
Fig. 2. Pipeline overview of our proposed SeqOT. It first uses the single-scan module to extract features for each range image in the sequence Then, the MSM fuses every three continuous features to exploit the spatial–temporal information and generates a subdescriptor. In the end, a GeM pooling module is applied to fuse the subdescriptors and generate a global descriptor for fast LiDAR sequence-based place recognition.

## III. OUR APPROACH

The overview of our proposed method is depicted in Fig. 2. Our proposed SeqOT is composed of three modules, including a single-scan module, a multiscan module (MSM), and a pooling module. It takes sequential range images in the queue as the input of the proposed spatial–temporal network and first extracts features from every single LiDAR range image using the single-scan module. Then, it uses the MSM to generate a subdescriptor of every continuous feature of three scans. In the end, a GeM pooling module is applied to fuse the subdescriptors into a 1-D global descriptor for fast LiDAR-sequence-based place recognition. More details about the architecture design are presented in Section III-A. Benefiting from the devised network architecture, we mathematically show in Section III-B that our proposed descriptor is yaw-rotation invariant. For training the proposed network, we propose a two-phase training method and use the triplet loss with the calculated overlap to better distinguish the positive and negative training examples presented in Section III-C.

## A. SeqOT Network Architecture

Our method takes only sequential LiDAR data as the input. Following our previous work [4], [21], we use the range image representation of LiDAR scans. The correspondence between a LiDAR point $p _ { i } \in \mathcal { P } , p _ { i } = ( x _ { i } , y _ { i } , z _ { i } )$ and the pixel coordinates $( u _ { i } , v _ { i } )$ in the corresponding range image R can be represented as

$$
\begin{array} { r } { \left( \begin{array} { l } { u _ { i } } \\ { v _ { i } } \end{array} \right) = \left( \begin{array} { c } { \frac { 1 } { 2 } \left[ 1 - \arctan ( y _ { i } , x _ { i } ) / \pi \right] w } \\ { \left[ 1 - ( \arcsin ( z _ { i } / r _ { i } ) + \mathrm { f } _ { \mathrm { u p } } ) / \mathrm { f } \right] h } \end{array} \right) } \end{array}\tag{1}
$$

where $r _ { i } = | | { \bf p } _ { i } | | _ { 2 }$ is the range measurement of the corresponding point, $\mathrm { f } = \mathrm { f } _ { \mathrm { u p } } + \mathrm { f } _ { \mathrm { d o w n } }$ is the vertical field of view of the sensor, and w and h are the width and height of the resulting range image, respectively. The range image is dense but lightweighted, which enables our method to operate online.

Unlike the previous works [4], [21] that only use one LiDAR scan as input, our SeqOT uses a sequence of range images to exploit both spatial and temporal information. At the current timestamp t, we use m past range images $\mathbb { R } _ { t } = \{ \mathcal { R } _ { t - m + 1 } , . . . ,$ $\mathcal { R } _ { t - 1 } , \mathcal { R } _ { t } \big \}$ . For all the range images in the sequence R<sub>t</sub>, we first use a single-scan module to extract features that exploit the spatial information contained in the single range image. Following OT [4], we use an encoder called OverlapNetLeg consisting of several fully convolutional layers to compress the range image with the size of $1 \times h \times w$ into a feature volume with the size of $c \times 1 \times w .$ . The extracted feature keeps the same width as the original range image while compressing the height to 1 and expanding the channel size to c. By doing so, we extract a coarse feature of the range image while maintaining the horizontal information to enable yaw-rotation invariance. Then, we apply a transformer module on such feature volume to further exploit the spatial information and generate a refined feature. We concatenate it with the coarse feature to exploit information obtained by both the convolutional layers and attention mechanisms and use the combined feature as the input to the following module.

After generating the features for all range images in the sequence, we apply the MSM on every three scans to exploit the temporal information. To this end, we concatenate features of every three scans along the width yielding a long feature with the size of $2 c \times 1 \times$ 3w and use these long features as the input to the MSM. It first applies the self-attention mechanism [30] of the MST module and then a NetVLAD with MLP [4] on the output feature of the transformer yielding a $1 \times 2 5 6$ subdescriptor vector, which fuses the spatial and temporal information together.

There are two transformer modules utilized separately in the proposed single-scan module and MSM. They have the same architecture but play different roles. Both transformers consist of a multihead self-attention (MHSA), a feedforward network (FFN), and layer normalization (LN). The input feature of MHSA is split into several subfeatures along the channel dimension according to the number of heads. The so-called query, key, and value $\{ Q , K , V \}$ features of MHSA are assigned as same as the input feature with multihead subfeatures, and the dot-product attention mechanism ofMHSA can be formulated as

$$
\bar { A } = \mathrm { A t t e n t i o n } ( Q , K , V ) = \mathrm { s o f t m a x } \left( \frac { Q K ^ { T } } { \sqrt { d _ { k } } } \right) V\tag{2}
$$

where $d _ { k }$ represents the dimension of splits and $\bar { A }$ is the output feature volume of the single-head self-attention in MHSA. The output of MHSA is then fed to the FFN for positionwise linear transformations. The LN for a layerwise normalization yielding the final output feature A of the transformer. We refer more details to the original transformer paper [30].

The SST module is designed to exploit the spatial information of a single scan and generate an intermediate feature for each scan. The MST module operates on the concatenated features from consecutive three observations focusing on the relations of temporal features. Different from existing spatial–temporal networks [9], [38], [39] using convolution networks on multiple scans, we use two transformers at different scales to capture both local information of a single scan and sequential relations between multiple scans. We apply the MST on three consecutive scans to exploit spatial–temporal information while keeping the network lightweight and efficient.

In the end, we use a generalized mean (GeM) pooling [40] to fuse all the subdescriptors generated by the MSM to further exploit the temporal information in an even longer range. The GeM pooling fuses the subdescriptors into one global descriptor along the time dimension, which is invariant to the input order of the subdescriptors. Based on such lightweight global descriptors, our method achieves very fast LiDAR-sequence-based descriptor matching.

## B. Yaw-Rotation Invariance

Our devised SeqOT is yaw-rotation invariant, leading to more robust place recognition performance in real applications, e.g., an autonomous vehicle operating on two-way streets. In this section, we provide the mathematical derivation to show that our SeqOT is yaw-rotation invariant against sequential LiDAR scans obtained at the same place but from different viewpoints.

It has been proved in OT [4] that the SST module is yawrotation equivariant. The yaw rotation θ of a point cloud P leads to the horizontal shift s of the corresponding feature volume $X$ We follow OT [4] to use the term $C _ { s }$ that represents the column shift of the feature volume X by matrix right multiplication, and $R _ { \theta }$ represents the yaw-rotation matrix of the point cloud P and the following equation holds:

$$
X C _ { s } = \mathrm { S S M } ( R _ { \theta } \mathcal { P } )\tag{3}
$$

where SSM(·) represents the single-scan module operation.

After obtaining the features from the single-scan module, our SeqOT concatenates three feature volumes $X _ { 1 } , X _ { 2 }$ , and $X _ { 3 }$ of three consecutive scans along the width and generates a long feature volume $X _ { \mathrm { l o n g } } = [ X _ { 1 } , X _ { 2 } , X _ { 3 } ]$ . According to (3), the yaw rotations of the raw input point clouds lead to shifted feature volumes $[ X _ { 1 } C _ { s 1 } , X _ { 2 } C _ { s 2 } , X _ { 3 } C _ { s 3 } ]$ . The feature volume $\bar { A } _ { s }$ extracted by the MHSA of the multiscan transformer in (2) then becomes

$$
\bar { A } _ { s } = \mathrm { A t t e n t i o n } ( Q _ { s } , K _ { s } , V _ { s } ) = \mathrm { s o f t m a x } \left( \frac { Q _ { s } { K _ { s } } ^ { T } } { \sqrt { d _ { k } } } \right) V _ { s }\tag{4}
$$

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/521c172afc04a13cd848f7ce999a3fc24727e762edad5c0df57d61317e4e2938.jpg)  
Fig. 3. Output of the MST is yaw-rotation equivariant individually, leading to the same subdescriptor generated by the NetVLAD with the MLP.

$$
= \mathrm { s o f t m a x } \left( \frac { [ X _ { 1 } C _ { s 1 } , X _ { 2 } C _ { s 2 } , X _ { 3 } C _ { s 3 } ] \left[ C _ { s 2 } ^ { T } X _ { 1 } ^ { T } \right] } { \sqrt { d _ { k } } } \right)
$$

$$
[ X _ { 1 } C _ { s 1 } , X _ { 2 } C _ { s 2 } , X _ { 3 } C _ { s 3 } ]\tag{5}
$$

$$
= \mathrm { s o f t m a x } \left( { \frac { X _ { 1 } X _ { 1 } ^ { T } + X _ { 2 } X _ { 2 } ^ { T } + X _ { 3 } X _ { 3 } ^ { T } } { \sqrt { d _ { k } } } } \right)
$$

$$
[ X _ { 1 } C _ { s 1 } , X _ { 2 } C _ { s 2 } , X _ { 3 } C _ { s 3 } ]\tag{6}
$$

$$
= W [ X _ { 1 } C _ { s 1 } , X _ { 2 } C _ { s 2 } , X _ { 3 } C _ { s 3 } ]\tag{7}
$$

$$
\mathbf { \Psi } = [ W X _ { 1 } C _ { s 1 } , W X _ { 2 } C _ { s 2 } , W X _ { 3 } C _ { s 3 } ]\tag{8}
$$

where W is a weighting matrix generated by the softmax and not affected by feature shifting $C _ { s }$ . As can be seen, the new feature $\bar { A } _ { s }$ also consists of three subfeatures $\bar { A } _ { s } = [ W X _ { 1 } C _ { s 1 } ,$ V $V X _ { 2 } C _ { s 2 } , W X _ { 3 } C _ { s 3 } ]$ , where each subfeature is yaw-rotation equivariant individually. The LN and FFN also does not affect the yaw-rotation equivariance of each part [4], and we have $A _ { s } = f ( \bar { A } _ { s } )$ , where $f ( \cdot )$ represents the following operations after MHSA in the transformer module. Therefore, the yaw rotation in point clouds only causes the permutation change along the width dimension in the output feature of the MST. Since the following NetVLAD module is permutation invariant [4], [19], it generates the yaw-rotation-invariant subdescriptor D as

$$
D _ { s } = \operatorname { V l a d } ( A _ { s } )\tag{9}
$$

$$
\mathbf { \Psi } = \operatorname { V l a d } ( f ( [ W X _ { 1 } C _ { s 1 } , W X _ { 2 } C _ { s 2 } , W X _ { 3 } C _ { s 3 } ] ) )\tag{10}
$$

$$
= \operatorname { V l a d } ( f ( [ W X _ { 1 } , W X _ { 2 } , W X _ { 3 } ] ) )\tag{11}
$$

$$
= \operatorname { V l a d } ( A ) = D\tag{12}
$$

which is also illustrated in Fig. 3.

## C. Network Training

One of the bottlenecks of using neural networks on sequential LiDAR data is the large computational and memory costs. To tackle this problem, we use a two-phase scheme to train the transformer modules and the pooling module separately. For a recorded LiDAR range image sequence $\mathbb { R } _ { \mathrm { a l l } } = \{ \mathcal { R } _ { i } \} _ { i = 0 } ^ { n }$ with $n + 1$ scans, we group each scan $\mathcal { R } _ { i }$ with its $l - 1$ adjacent scans to form a subsequence training sample, where $l = 3$ is the same number as the input scans of our MSM. For each training sample, we use the overlap/similarity value proposed by Chen et al. [21] between the anchor scan $\mathcal { R } _ { i }$ and all other anchor scans in $\mathbb { R } _ { \mathrm { a l l } }$ to supervise the network. The overlap value between two anchor scans $\mathcal { R } _ { i }$ and $\mathcal { R } _ { j }$ is calculated as

$$
O _ { \mathcal { R } _ { i } \mathcal { R } _ { j } } = \frac { \sum _ { ( u , v ) } \mathbb { I } \{ | \mathcal { R } _ { i } ( u , v ) - \mathcal { R } _ { j } ^ { \prime } ( u , v ) | \} \leq \delta \} } { \operatorname* { m i n } ( \mathrm { v a l i d } ( \mathcal { R } _ { i } ) , \mathrm { v a l i d } ( \mathcal { R } _ { j } ^ { \prime } ) ) }\tag{13}
$$

where $\mathcal { R } _ { j } ^ { \prime }$ is the reprojected range image of $\mathcal { R } _ { j }$ in the coordinate frame of the query scan $\mathscr { R } _ { q } . \mathbb { I } ( a ) = 1$ if a is true and $\mathbb { I } ( a ) = 0$ otherwise; valid(R) refers to the counts of valid pixels of range image R, and δ is the threshold to decide the overlapped pixel.

We train the transformer networks using the triplet loss. If the overlap value between two training samples is larger than 0.3, we take that pair as a positive sample, otherwise a negative sample.

In the first phase, we train the network without the pooling module. The triplet loss is directly applied on the subdescriptors generated by our MSM. For each query subdescriptor $D _ { \mathrm { q } } , N _ { \mathrm { p o s } }$ positive subdescriptors ${ \mathcal { D } } _ { \mathrm { p o s } } = \{ D _ { p } \}$ and $N _ { \mathrm { n e g } }$ negative subdescriptors ${ \mathcal { D } } _ { \mathrm { n e g } } = \{ D _ { n } \}$ are used to calculate the triplet loss by

$$
\begin{array} { r l } {  { \mathcal { L } _ { 1 } ( D _ { \ P } , \mathcal { D } _ { \mathrm { p o s } } , \mathcal { D } _ { \mathrm { n e g } } ) } \quad } & { } \\ & { = N _ { \mathrm { p o s } } ( \alpha + \operatorname* { m a x } _ { p } ( d ( D _ { \ P } , D _ { p } ) ) ) - \sum _ { N _ { \mathrm { n e g } } } ( d ( D _ { \ P } , D _ { n } ) ) } \end{array}\tag{14}
$$

where α is the margin to keep the loss positive and $d ( \cdot )$ computes the squared Euclidean distance. We use the triplet loss to minimize the distance between the query subdescriptor and the hardest case in positive reference subdescriptors and maximize the distance between the query and all sampled negative reference subdescriptors.

In the second phase, we use the pretrained transformer network to generate subdescriptors $\mathcal { D } _ { \mathrm { a l l } }$ for all the scans in $\mathbb { R } _ { \mathrm { a l l } }$ . We then train the GeM pooling with the subdescriptors $\mathcal { D } _ { \mathrm { a l l } }$ using the same overlap labels used in the first phase. For each query global descriptor $G _ { \mathrm { q } } , N _ { \mathrm { p o s } }$ positive descriptors $\mathcal { G } _ { \mathrm { p o s } } = \{ G _ { p } \}$ and $N _ { \mathrm { n e g } }$ negative descriptors ${ \mathcal G } _ { \mathrm { n e g } } = \{ G _ { n } \}$ are used to calculate the triplet loss by

$$
\begin{array} { r l } {  { \mathcal { L } _ { 2 } ( G _ { \mathfrak { q } } , \mathcal { G } _ { \mathrm { p o s } } , \mathcal { G } _ { \mathrm { n e g } } ) } \quad } & { } \\ & { = N _ { \mathrm { p o s } } ( \alpha + \operatorname* { m a x } _ { p } ( d ( G _ { \mathfrak { q } } , G _ { p } ) ) ) - \sum _ { N _ { \mathrm { n e g } } } ( d ( G _ { \mathfrak { q } } , G _ { n } ) ) . } \end{array}\tag{15}
$$

For online operation, we store the subdescriptors of past scans generated by our transformer network and apply the pooling module directly on them. Thus, only one feedforward is needed for every incoming scan.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/2cfade8d915f992515e9e54c028865f9287066f0d6918484c4fc7ebc8a6d4bda.jpg)  
Fig. 4. Haomo dataset is collected by an AGV built by HAOMO.AI Technology company in the urban environments of Beijing, which is introduced in detail in our recent work [4].

## IV. EXPERIMENTAL EVALUATION

The experimental evaluation is designed to showcase the performance of our approach and to support the claims that our approach is able to: 1) achieve good long-term place recognition in outdoor large-scale environments using only sequential LiDAR data without any other information; 2) generalize well into the different environments using LiDAR data obtained by different types of LiDAR sensors without fine-tuning; 3) recognize places with changing viewpoints and input sequence order based on the yaw-rotation-invariant architecture; and 4) achieve online operation with runtime less than 100 ms.

## A. Implementation and Experimental Setup

We use several different datasets to evaluate our method, including the NCLT [41], KITTI odometry [42], and Mul-Ran [43] datasets. In addition, we also use our recorded Haomo dataset, which was first open source in our recent work [4]. The Haomo dataset is collected by an AGV equipped with a LiDAR sensor (HESAI PandarXT, 32-beam), a wide-angle camera (SENSING SG2, HFOV 106<sup>◦</sup>, VFOV 56<sup>◦</sup>), an RTK GNSS (Asensing INS570D, 0.1<sup>◦</sup> error in roll/pitch, 0.2<sup>◦</sup> error in yaw, 2 cm + 1 ppm in RTK position), a mini computer (Intel Xeon E, 3.5 GHz, 80 W), and a Nivida Tesla T4 (16-GB memory, 70 W), which is shown in Fig. 4.

Following previous works [4], [21], we use range images with size of $1 \times 3 2 \times 9 0 0$ . The length of used LiDAR sequence is set to $m = 2 0$ . The subdescriptors generated by our MSM are 1-D vectors with the size of $1 \times 2 5 6 .$ , and so are the final global descriptors generated by our GeM pooling module. For training, we set $N _ { \mathrm { p o s } } = 6 , N _ { \mathrm { n e g } } = 6 .$ , and $\alpha = 0 . 5$ for the triplet loss. We train our proposed transformer network without the pooling module for 20 epochs, using the ADAM optimizer to update the weights of the network with an initial learning rate of 5e-6 and a weight decay of 0.9 applied every five steps. To train the GeM pooling module, we use the similar configuration but increase the initial learning rate to 5e-5.

In the following experiments, we compare our proposed SeqOT with both state-of-the-art single-scan-based methods

TABLE I  
COMPARISON OF PLACE RECOGNITION PERFORMANCE ON THE THREE CHALLENGES OF THE HAOMO DATASET
<table><tr><td rowspan="2">Approach</td><td colspan="3">Sequence 1-2</td><td colspan="3">Sequence 1-3</td><td colspan="3">Sequence 2-1</td></tr><tr><td>AR@1</td><td>AR@5</td><td>AR@20</td><td>Recall@1</td><td>AR@5</td><td>AR@20</td><td>AR@1</td><td>AR@5</td><td>AR@20</td></tr><tr><td>PointNetVLAD [19]</td><td>0.913</td><td>0.936</td><td>0.957</td><td>0.400</td><td>0.521</td><td>0.623</td><td>0.861</td><td>0.932</td><td>0.944</td></tr><tr><td>MinkLoc3D [28]</td><td>0.948</td><td>0.971</td><td>0.993</td><td>0.450</td><td>0.653</td><td>0.808</td><td>0.858</td><td>0.924</td><td>0.940</td></tr><tr><td>OverlapTransformer [4]</td><td>0.974</td><td>0.987</td><td>0.995</td><td>0.776</td><td>0.878</td><td>0.931</td><td>0.903</td><td>0.933</td><td>0.945</td></tr><tr><td>OT_AVG [7]</td><td>0.985</td><td>0.989</td><td>0.998</td><td>0.803</td><td>0.843</td><td>0.895</td><td>0.894</td><td>0.943</td><td>0.964</td></tr><tr><td>HRS [12]</td><td>0.980</td><td>0.988</td><td>0.998</td><td>0.806</td><td>0.877</td><td>0.949</td><td>0.901</td><td>0.941</td><td>0.961</td></tr><tr><td>CIMV [44]</td><td>0.990</td><td>0.999</td><td>1.</td><td>0.810</td><td>0.887</td><td>0.963</td><td>0.910</td><td>0.948</td><td>0.965</td></tr><tr><td>SeqNet [9]</td><td>0.988</td><td>0.997</td><td>1.</td><td>0.816</td><td>0.879</td><td>0.962</td><td>0.906</td><td>0.946</td><td>0.963</td></tr><tr><td>SeqLPD [8]</td><td>0.982</td><td>0.996</td><td>1.</td><td>0.645</td><td>0.713</td><td>0.878</td><td>0.918</td><td>0.946</td><td>0.971</td></tr><tr><td>SeqOT (ours)</td><td>0.998</td><td>1.</td><td>1.</td><td>0.824</td><td>0.899</td><td>0.967</td><td>0.932</td><td>0.952</td><td>0.966</td></tr></table>

The bold entities indicate the best performance.

PointNetVLAD [19], MinkLoc3D [28], and OT [4], and sequence-enhanced methods using the features generated by OT, including a simple baseline by averaging single-scan matching scores of sequences (OT\_AVG), the hashing-based retrieval strategy (HRS) [12], the group-based condition-invariant multiview (CIMV) method [44], and SeqNet [9]. OT\_AVG is a simplified version of SeqSLAM [7], which only considers the diagonal line of the difference matrix for faster retrieval. It is natural and straightforward to try out the visual-descriptor-based methods directly on LiDAR range images since there are few LiDARbased sequence-enhanced methods. We, therefore, use CIMV and SeqNet with LiDAR range image descriptors as the baselines and show that our devised novel method works better than directly applying existing methods on LiDAR range images. We also compare our method with SeqLPD [8] using their opensource implementation. The descriptor generated by the original OT is with the size of 1 × 256. The output of SeqNet is also set to 1 × 256, and we set the convolutional kernel tensor of size 256 × 3 × 256 for the local temporal window ofSeqNet for a fair comparison. If not explicitly stated otherwise, we use m = 20 consecutive scans for all methods in the following experiments.

Note that, since all sequence-enhanced baseline methods are not order and yaw-rotation invariant, we apply a forward matching and a reverse matching and choose the better one for comparison. This improves the performance of baseline methods, especially in the case where the car drives in the opposite direction with respect to the database. In contrast, our method only matches once for retrieval, benefiting from our yaw-rotation invariance design and the use of the GeM pooling.

## B. Evaluation for Place Recognition

The first experiment supports our claim that our approach achieves good place recognition in large-scale outdoor environments with long time spans using only sequential LiDAR data without any other information.

Following the experimental setup of OT [4], we train our approach and baseline methods on the relatively old sequences, such as sequences 1-1 and 2-1 in the Haomo dataset [4] and sequence 2012-01-08 in the NCLT dataset [41], and evaluate on newly records such as sequences 1-2, 1-3, and 2-2 in the Haomo dataset, and sequences 2012-02-05, 2012-06-15, 2013-02-23, and 2013-04-05 in the NCLT dataset. The Haomo dataset is recorded in a relatively short period of around three months but provides multiple challenges including opposite driving directions, while the NCLT dataset provides multiple sequences recorded in the same environment across more than one year and is usually used for evaluating long-term place recognition.

We use average top 1 recall (AR@1), top 5 recall (AR@5), and top 20 recall (AR@20) as the evaluation metrics. For each scan sampled from query sequences, we acquire its top N candidates and ground truth reference scans with overlap values larger than 0.3 in the database. Once one of the true references is found, we consider that query as a successful loop closure. We can get the recall rate by calculating the proportion of successful retrievals for each sequence. The evaluation results are shown in Tables I and II for the Haomo dataset and the NCLT dataset, respectively. As can be seen, sequence-enhanced methods are generally more robust than single-scan-based methods, and our method outperforms all the baseline methods in terms of AR@1 on both the datasets and has competitive performance when also considering more place candidates. Our SeqOT directly extracts and fuses spatial–temporal information from the sequential LiDAR data, which outperforms other sequence-based baselines in most cases in terms of LiDAR sequence-based place recognition. Since OT and our SeqOT are both yaw-rotation invariant, they outperform other counterparts in the opposite driving challenges ofsequence 1-3 of the Haomo dataset. In addition, the experimental results also show that our method works for long-time-span place recognition. The time gaps between sequences in the NCLT dataset are more than one year, leading to significant appearance changes in the environment. Our proposed SeqOT is designed to capture the temporal and spatial changes of consecutive observations, thus not overfitting on specific appearance features. In contrast, OT may focus more on specific spatial features, e.g., a temporarily parked car, without exploiting temporal information, thus less reliable in long-term place recognition.

In addition, we utilize the t-SNE visualization of the global descriptors generated by OT, the output of the MSM, and the global descriptors generated by SeqOT on sequence

TABLE II  
COMPARISON OF PLACE RECOGNITION PERFORMANCE ON THE NCLT DATASET
<table><tr><td rowspan="2">Approach</td><td colspan="3">2012-02-05</td><td colspan="3">2012-06-15</td><td colspan="3">2013-02-23</td><td colspan="3">2013-04-05</td></tr><tr><td>AR@1</td><td>AR@5</td><td>AR@20</td><td>AR@1</td><td>AR@5</td><td>AR@20</td><td>AR@1</td><td>AR@5</td><td>AR@20</td><td>AR@1</td><td>AR@5</td><td>AR@20</td></tr><tr><td>PointNetVLAD [19]</td><td>0.746</td><td>0.823</td><td>0.875</td><td>0.612</td><td>0.720</td><td>0.782</td><td>0.469</td><td>0.604</td><td>0.719</td><td>0.449</td><td>0.576</td><td>0.683</td></tr><tr><td>MinkLoc3D [28]</td><td>0.802</td><td>0.864</td><td>0.926</td><td>0.630</td><td>0.685</td><td>0.774</td><td>0.507</td><td>0.616</td><td>0.751</td><td>0.482</td><td>0.587</td><td>0.685</td></tr><tr><td>OverlapTransformer [4]</td><td>0.861</td><td>0.899</td><td>0.930</td><td>0.639</td><td>0.697</td><td>0.780</td><td>0.536</td><td>0.645</td><td>0.764</td><td>0.496</td><td>0.603</td><td>0.715</td></tr><tr><td>OT_AVG [7]</td><td>0.876</td><td>0.925</td><td>0.952</td><td>0.610</td><td>0.718</td><td>0.840</td><td>0.561</td><td>0.698</td><td>0.830</td><td>0.529</td><td>0.658</td><td>0.791</td></tr><tr><td>HRS [12]</td><td>0.869</td><td>0.925</td><td>0.955</td><td>0.624</td><td>0.716</td><td>0.821</td><td>0.557</td><td>0.669</td><td>0.814</td><td>0.498</td><td>0.643</td><td>0.752</td></tr><tr><td>CIMV [44]</td><td>0.871</td><td>0.925</td><td>0.957</td><td>0.642</td><td>0.730</td><td>0.852</td><td>0.564</td><td>0.707</td><td>0.835</td><td>0.527</td><td>0.663</td><td>0.797</td></tr><tr><td>SeqNet [9]</td><td>0.889</td><td>0.933</td><td>0.960</td><td>0.645</td><td>0.745</td><td>0.859</td><td>0.569</td><td>0.725</td><td>0.847</td><td>0.517</td><td>0.674</td><td>0.801</td></tr><tr><td>SeqLPD [8]</td><td>0.873</td><td>0.928</td><td>0.952</td><td>0.663</td><td>0.791</td><td>0.884</td><td>0.658</td><td>0.713</td><td>0.847</td><td>0.582</td><td>0.719</td><td>0.835</td></tr><tr><td>SeqOT (ours)</td><td>0.917</td><td>0.947</td><td>0.968</td><td>0.762</td><td>0.844</td><td>0.899</td><td>0.691</td><td>0.723</td><td>0.874</td><td>0.639</td><td>0.724</td><td>0.826</td></tr></table>

The bold entities indicate the best performance.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/e1d98d6385f25201d1cfd59948fc5dbefb461d48d890ea98217ee1ec5314d370.jpg)  
(a)

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/56219f3d66a37909fb8eab7c559d85fc82c38b441bb3a84c513b723b2a110eca.jpg)  
(b)

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/0bfd4433e971426801f821205a9c6802499dadaa1a434b669bf9fd7e579d32a6.jpg)  
(c)  
Fig. 5. t-SNE visualization of place clustering. (a) OT. (b) Output of MSM. (c) SeqOT.

2012-01-08 of the NCLT dataset for qualitative evaluation. As shown in Fig. 5(a) and (b), compared with OT, the t-SNE results of the MSM show more distinguishable clusters, which indicates a better feature extraction benefiting from applying the MST. By comparing Fig. 5(b) and (c), we see that with the pooling-based aggregation of subdescriptors generated by the MSM, SeqOT can further generate more discriminative descriptors.

## C. Generalization Analysis

In this experiment, we show that our proposed method can generalize well to the different environments with different LiDAR sensors in a zero-shot manner. We train our SeqOT and all the baseline methods only on sequence 2012-01-08 of the NCLT dataset and operate them directly on the KITTI odometry and MulRan datasets.

We run all the methods to find similar places on the areas traveled multiple times in KITTI 00 sequence, while for the MulRan dataset, we use sequence kaist-01 as the database and sequence kaist-02 as the query. Note that the data of the NCLT dataset for training the network were collected using a Velodyne-32 LiDAR, while the KITTI odometry data were collected with Velodyne-64 and the MulRan dataset was with Ouster-64. In line with OT [4], we use the precision–recall curve to evaluate all the methods, and the experimental results are shown in Fig. 6(a) and (b). As can be seen, SeqOT outperforms other baselines and achieves the best generalization capability. The results show the superiority of our SeqOT in both the datasets without fine-tuning, which indicates a solid generalization ability.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/dce9f4cd24734295aa1c98c4db7fe6eee8732f78439b518d85a0b431877d6855.jpg)  
(a)

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/8d0a9179371ff6506604b82d4af056b69ea13657c8261504d0d430d8bc03048a.jpg)  
(b)  
Fig. 6. Generalization analysis. (a) Evaluation on the KITTI dataset. (b) Evaluation on the MulRan dataset.

## D. Ablation Study on Sequence Length

This ablation study investigates the effect of different lengths of input sequences. We change the input sequence length from 10 to 40 LiDAR frames, using OT as the backbone for all the baseline methods. The performance of the single-scan-based OT is also shown as a baseline. We evaluate the average recall AR@1 performance on sequence 1-3 ofthe Haomo dataset and sequence 2013-04-05 of the NCLT dataset. The results are shown in

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/2903ba1e581fdfabf84d2478f091e966c7ac3476b9ac6a926eb599f86a678a04.jpg)  
Fig. 7. Ablation study on sequence length on the Haomo dataset.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/44efae7006951e9b699a8b0138d804621951eb537c73ea04cc873897aea04c6b.jpg)  
Fig. 8. Ablation study on sequence length on the NCLT dataset.

Figs. 7 and 8. As can be seen, our proposed SeqOT consistently outperforms all the baseline methods with all tested input lengths. Unlike the baseline methods, our proposed method only applies one forward matching, while the other sequenceenhanced methods need two-direction matching to obtain good performance since the car drives in opposite directions. We also evaluate OT\_AVG with only one forward matching process to show the improvement of two directions matching for baseline sequence matching methods. The experimental results demonstrate that our method is more robust to driving directions benefiting from our devised yaw-rotation-invariant network, while OT\_AVG with one direction matching loses efficacy quickly, especially on the Haomo dataset where the car drives in the opposite directions. The performance of all methods improves slowly when the sequence length is larger than 20, and the reason could be the redundant information. Therefore, we set the input sequence length to 20 for all the methods in other experiments in tradeoff efficiency and performance.

## E. Ablation Study on Transformer Modules

This ablation study is aimed to show the effectiveness of the proposed SST and the MST. We use sequence 2012-01-08 of the NCLT dataset as database and sequence 2012-02-05 as query to compare four different setups, including Conv+Conv, SST+Conv, Conv+MST, and SST+MST. Conv+Conv replaces each transformer module with two 1×1 convolution layers without changing the channel numbers. SST+Conv only uses SST, while Conv+MST only uses MST. SST+MST represents the holistic SeqOT. As shown in Table III, the models with transformers consistently outperform the one with only convolution layers. The performance increases significantly if we use the MST to fuse spatial and temporal features rather than convolution layers. Besides, the effectiveness of the MST is more significant than that of the SST, which means extracting spatial– temporal features with transformer gains more improvement in place recognition compared to only exploiting spatial features.

TABLE III  
ABLATION STUDY ON THE TRANSFORMER MODULES
<table><tr><td>Network</td><td>Runtime [ms]</td><td>AR@1</td><td>AR@5</td><td>AR@20</td></tr><tr><td>Conv+Conv</td><td>67.55</td><td>0.605</td><td>0.800</td><td>0.924</td></tr><tr><td>SST+Conv</td><td>88.74</td><td>0.769</td><td>0.871</td><td>0.941</td></tr><tr><td>Conv+MST</td><td>77.11</td><td>0.875</td><td>0.924</td><td>0.960</td></tr><tr><td>SST+MST</td><td>98.32</td><td>0.917</td><td>0.947</td><td>0.968</td></tr></table>

The bold entities indicate the best performance.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/9027c68c68996d01e92df0814c6d785d320f22615727961ef11de1d0f82e1933.jpg)  
Fig. 9. Study on yaw-rotation invariance.

## F. Study on Yaw-Rotation Invariance

In this experiment, we validate that our SeqOT is yaw-rotation invariant against LiDAR scans obtained at the same place but from different viewpoints. We compare our method with SeqLPD conducted on the Haomo dataset. We rotate each query scan of sequence 2-2 along the yaw axis in steps of 30 degrees and search the places in the same original database. We use Recall@1 as the criterion to evaluate the effect of yaw angle change on our method and SeqLPD. The experimental results are shown in Fig. 9. As can be seen, both OT and SeqOT are not affected by the rotation along the yaw axis due to our devised yaw-rotation-invariant global descriptors, while the performance of SeqLPD decreases quickly with the yaw angle increasing. This experiment also verifies again that our sequence-enhanced method consistently outperforms the single-scan baseline methods with large viewpoint changes.

## G. Runtime

We further evaluate the runtime and show that our method can achieve online operation. We conduct all the experiments on a system with an Intel i7-11700K CPU and an Nvidia RTX 3070 GPU. We run all the models to find top-20 candidates. Here, we consider the most extreme case in which all the subdescriptors are generated from scratch. The holistic SeqOT takes 98.32 ms for generating the descriptor of each scan and finding the candidates, which is less than 100 ms, thus faster than the sensor frame rate. Besides, the number of parameters of SeqOT is 12.82M, which means there is only a little memory consumption.

## V. CONCLUSION

In this article, we presented a novel end-to-end transformer network for LiDAR-sequence enhanced place recognition. Our approach utilizes transformer modules at different scales to generate subdescriptors that fuse the spatial and temporal information provided in sequential LiDAR range images. In the end, a GeM pooling is used to further exploit longer temporal information, fuse the subdescriptors, and generate a lightweight global descriptor for each sequence. We compared the place recognition performance of our method with the state-of-the-art single-scan and sequence-enhanced methods on four different datasets. The experimental results suggest that our method outperforms the state-of-the-art methods in terms of place recognition performance. The additional zero-shot evaluations on the KITTI and MulRan datasets also show the strong generalization ability of our method. Furthermore, our proposed SeqOT is yaw-rotation invariant and operates online, which can be used for real-world applications.

## REFERENCES

[1] P. Yin et al., “A multi-domain feature learning method for visual place recognition,” in Proc. IEEE Int. Conf. Robot. Autom., 2019, pp. 319–324.

[2] S. Hausler, S. Garg, M. Xu, M. Milford, and T. Fischer, “Patch-NetVLAD: Multi-scale fusion of locally-global descriptors for place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 14136– 14147.

[3] S. Lowry et al., “Visual place recognition: A survey,” IEEE Trans. Robot., vol. 32, no. 1, pp. 1–19, Feb. 2016.

[4] J. Ma, J. Zhang, J. Xu, R. Ai, W. Gu, and X. Chen, “OverlapTransformer: An efficient and yaw-angle-invariant transformer network for LiDAR-based place recognition,” IEEE Robot. Autom. Lett., vol. 7, no. 3, pp. 6958–6965, Jul. 2022.

[5] X. Chen et al., “OverlapNet: Loop closing for LiDAR-based SLAM,” in Proc. Robot.: Sci. Syst. Conf., 2020, pp. 1–10.

[6] G. Kim and A. Kim, “Scan context: Egocentric spatial descriptor for place recognition within 3D point cloud map,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 4802–4809.

[7] M. Milford and G. Wyeth, “SeqSLAM: Visual route-based navigation for sunny summer days and stormy winter nights,” in Proc. IEEE Int. Conf. Robot. Autom., 2012, pp. 1643–1649.

[8] Z. Liu et al., “SeqLPD: Sequence matching enhanced loop-closure detection based on large-scale point cloud description for self-driving vehicles,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 1218–1223.

[9] S. Garg and M. Milford, “SeqNet: Learning descriptors for sequence-based hierarchical place recognition,” IEEE Robot. Autom. Lett., vol. 6, no. 3, pp. 4305–4312, Jul. 2021.

[10] O. Vysotska and C. Stachniss, “Effective visual place recognition using multi-sequence maps,” IEEE Robot. Autom. Lett., vol. 4, no. 2, pp. 1730–1736, Apr. 2019.

[11] R. Arandjelovic, P. Gronat, A. Torii, T. Pajdla, and J. Sivic, “NetVLAD: CNN architecture for weakly supervised place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2016, pp. 5297–5307.

[12] O. Vysotska and C. Stachniss, “Relocalization under substantial appearance changes using hashing,” in Proc. IROS Workshop Planning, Perception, Navigat. Intell. Veh., 2017, pp. 1–7.

[13] K. P. Cop, P. V. Borges, and R. Dubé, “Delight: An efficient descriptor for global localisation using LiDAR intensities,” in Proc. IEEE Int. Conf. Robot. Autom., 2018, pp. 3653–3660.

[14] H. Wang, C. Wang, and L. Xie, “Intensity scan context: Coding intensity and geometry relations for loop closure detection,” in Proc. IEEE Int. Conf. Robot. Autom., 2020, pp. 2095–2101.

[15] Y. Wang, Z. Sun, C.-Z. Xu, S. E. Sarma, J. Yang, and H. Kong, “LiDAR IRIS for loop-closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5769–5775.

[16] L. Luo, S.-Y. Cao, B. Han, H.-L. Shen, and J. Li, “BVMatch: LiDAR-based place recognition using Bird’s-eye view images,” IEEE Robot. Autom. Lett., vol. 6, no. 3, pp. 6076–6083, Jul. 2021.

[17] X. Chen, I. Vizzo, T. Läbe, J. Behley, and C. Stachniss, “Range imagebased LiDAR localization for autonomous vehicles,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 5802–5808.

[18] H. Dong, X. Chen, and C. Stachniss, “Online range image-based pole extractor for long-term LiDAR localization in urban environments,” in Proc. Eur. Conf. Mobile Robot., 2021, pp. 1–6.

[19] M. A. Uy and G. H. Lee, “PointNetVLAD: Deep point cloud based retrieval for large-scale place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2018, pp. 4470–4479.

[20] Z. Liu et al., “LPD-Net: 3D point cloud learning for large-scale place recognition and environment analysis,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2019, pp. 2831–2840.

[21] X. Chen, T. Läbe, A. Milioto, T. Röhling, J. Behley, and C. Stachniss, “OverlapNet: A siamese network for computing LiDAR scan similarity with applications to loop closing and localization,” Auton. Robots, vol. 46, pp. 61–81, 2021.

[22] X. Chen, T. Läbe, L. Nardi, J. Behley, and C. Stachniss, “Learning an overlap-based observation model for 3D LiDAR localization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 4602–4608.

[23] D. Kong, X. Li, Y. Hu, Q. Xu, A. Wang, and W. Hu, “Learning a novel LiDAR submap-based observation model for global positioning in longterm changing environments,” IEEE Trans. Ind. Electron., vol. 70, no. 3, pp. 3147–3157, Mar. 2023.

[24] F. Cao, F. Yan, S. Wang, Y. Zhuang, and W. Wang, “Season-invariant and viewpoint-tolerant LiDAR place recognition in GPS-denied environments,” IEEE Trans. Ind. Electron., vol. 68, no. 1, pp. 563–574, Jan. 2021.

[25] F. Cao, H. Wu, and C. Wu, “An end-to-end localizer for long-term topological localization in large-scale changing environments,” IEEE Trans. Ind. Electron., early access, doi: 10.1109/TIE.2022.3189091.

[26] X. Kong et al., “Semantic graph based place recognition for 3D point clouds,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 8216– 8223.

[27] L. Li et al., “RINet: Efficient 3D LiDAR-based place recognition using rotation invariant neural network,” IEEE Robot. Autom. Lett., vol. 7, no. 2, pp. 4321–4328, Apr. 2022.

[28] J. Komorowski, “MinkLoc3D: Point cloud based large-scale place recognition,” in Proc. IEEEWinterConf.Appl. Comput. Vis., 2021, pp. 1789–1798.

[29] K. Vidanapathirana, M. Ramezani, P. Moghadam, S. Sridharan, and C. Fookes, “LoGG3D-Net: Locally guided global descriptor learning for 3D place recognition,” in Proc. IEEE Int. Conf. Robot. Autom., 2022, pp. 2215– 2221.

[30] A. Vaswani et al., “Attention is all you need,” in Proc. Int. Conf. Neural Inf. Process. Syst., 2017, pp. 6000–6010.

[31] L. Hui, H. Yang, M. Cheng, J. Xie, and J. Yang, “Pyramid point cloud transformer for large-scale place recognition,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 6078–6087.

[32] Z. Zhou et al., “NDT-transformer: Large-scale 3D point cloud localisation using the normal distribution transform representation,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 5654–5660.

[33] S. Siam and H. Zhang, “Fast-SeqSLAM: A. fast appearance based place recognition algorithm,” in Proc. IEEE Int. Conf. Robot. Autom., 2017, pp. 5702–5708.

[34] S. Garg, M. Vankadari, and M. Milford, “SeqMatchNet: Contrastive learning with sequence matching for place recognition & relocalization,” in Proc. 5th Annu. Conf. Robot Learn., 2022, pp. 429–443.

[35] P. Yin, L. Xu, J. Zhang, and H. Choset, “FusionVLAD: A multi-view deep fusion networks for viewpoint-free 3D place recognition,” IEEE Robot. Autom. Lett., vol. 6, no. 2, pp. 2304–2310, Apr. 2021.

[36] P. Yin, F. Wang, A. Egorov, J. Hou, Z. Jia, and J. Han, “Fast sequencematching enhanced viewpoint-invariant 3-D place recognition,” IEEE Trans. Ind. Electron., vol. 69, no. 2, pp. 2127–2135, Feb. 2022.

[37] P. Yin, F. Wang, A. Egorov, J. Hou, J. Zhang, and H. Choset, “Seq-SphereVLAD: Sequence matching enhanced orientation-invariant place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5024–5029.

[38] B. Mersch, X. Chen, J. Behley, and C. Stachniss, “Self-supervised point cloud prediction using 3D spatio-temporal convolutional networks,” in Proc. Conf. Robot Learn., 2021, pp. 1444–1454.

[39] M. Toyungyernsub, M. Itkina, R. Senanayake, and M. J. Kochenderfer, “Double-prong ConvLSTM for spatiotemporal occupancy prediction in dynamic environments,” in Proc. IEEE Int. Conf. Robot. Autom., 2021, pp. 13931–13937.

[40] F. Radenovi´c, G. Tolias, and O. Chum, “Fine-tuning CNN image retrieval with no human annotation,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 41, no. 7, pp. 1655–1668, Jul. 2019.

[41] N. Carlevaris-Bianco, A. K. Ushani, and R. M. Eustice, “University of Michigan north campus long-term vision and LiDAR dataset,” Int. J. Robot. Res., vol. 35, no. 9, pp. 1023–1035, 2016.

[42] A. Geiger, P. Lenz, and R. Urtasun, “Are we ready for autonomous driving? The KITTI vision benchmark suite,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2012, pp. 3354–3361.

[43] G. Kim, Y. Park, Y. Cho, J. Jeong, and A. Kim, “MulRan: Multimodal range dataset for urban place recognition,” in Proc. IEEE Int. Conf. Robot Autom., 2020, pp. 6246–6253.

[44] J. M. Facil, D. Olid, L. Montesano, and J. Civera, “Condition-invariant multi-view place recognition,” 2019, arXiv:1902.09516.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/ef3af2dc4e6c2e7442b0d7a992c93cb7ef0ba878e52119cc178334cd879e4bf2.jpg)

Jingyi Xu received the B.E. degree in vehicle engineering in 2020 from the Beijing Institute of Technology, Beijing, China, where she is currently working toward the master’s degree.

Her research interests include energy management strategies for intelligent vehicles, semantic segmentation, and motion planning of robots.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/192c1b9c3dc06bcd44d218b9d28476d5e541f68673bfbe9389826832cb62a59c.jpg)

Junyi Ma (Student Member, IEEE) received the B.E. degree in vehicle engineering in 2020 from the Beijing Institute of Technology, Beijing, China, where he is currently working toward the master’s degree.

He is trying to apply machine learning methods to robotics and use multiple sensor data for enhanced perception capability of robots and intelligent vehicles. His research interests include simultaneous localization and mapping, place recognition, and robotics.

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/01b753205f1db9a7607ab6718d853a2bb29d904f13701a672f8a71eecf96a8f5.jpg)

![](images/2023_SeqOT__A_Spatial_Temporal_Transformer_Network_for_Place_/93eb46aa7d111e5338d673b8ccef9df0ddd3b864b8c5f413b383fd0d3224093b.jpg)

Xieyuanli Chen (Student Member, IEEE) received the bachelor’s degree in electrical engineering and automation from Hunan University, Changsha, China in 2015, the master’s degree in robotics from the National University of Defense Technology, Changsha, in 2017, and the Ph.D. degree in robotics from the Photogrammetry and Robotics Laboratory, University of Bonn, Bonn, Germany, in 2022.

He is currently with the National University of Defense Technology. He is a Member of the

Guangming Xiong received the Ph.D. degree in mechanical engineering from the Beijing Institute of Technology, Beijing, China, in 2005.

He is currently an Associate Professor with the School of Mechanical Engineering, Beijing Institute of Technology. His research interests include intelligent vehicles, mobile robotics, machine vision, and multivehicle coordination.

Technical Committee of the RoboCup Rescue Robot League. He was a Member of the Organizing Committee of the RoboCup Rescue Robot League.