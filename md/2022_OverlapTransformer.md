# OverlapTransformer: An Efficient and Yaw-Angle-Invariant Transformer Network for LiDAR-Based Place Recognition

Junyi Ma , Jun Zhang , Jintao Xu, Rui Ai, Weihao Gu, and Xieyuanli Chen

Abstract—Place recognition is an important capability for autonomously navigating vehicles operating in complex environments and under changing conditions. It is a key component for tasks such as loop closing in SLAM or global localization. In this letter, we address the problem of place recognition based on 3D LiDAR scans recorded by an autonomous vehicle. We propose a novel lightweight neural network exploiting the range image representation of Li-DAR sensors to achieve fast execution with less than 2 ms per frame. We design a yaw-angle-invariant architecture exploiting a transformer network, which boosts the place recognition performance of our method. We evaluate our approach on the KITTI and Ford Campus datasets. The experimental results show that our method can effectively detect loop closures compared to the state-of-the-art methods and generalizes well across different environments. To evaluate long-term place recognition performance, we provide a novel dataset containing LiDAR sequences recorded by a mobile robot in repetitive places at different times.

Index Terms—SLAM, deep learning methods, data sets for robot learning.

## I. INTRODUCTION

LACE recognition plays an essential role for autonomously recognition [1]–[5], LiDAR-based place recognition [6]–[9] is comparably robust to day-and-night light changes and different weather conditions. Therefore, LiDARs are an attractive sensing modality that can be used for autonomous driving in outdoor large-scale environments. LiDAR-based place recognition is the task to determine if the LiDAR sensor is currently in a place that has been visited before by comparing the current LiDAR observations with the map or a database of formerly taken observations. It supports tasks such as simultaneous localization and mapping (SLAM) to find loop closure candidates and global localization to obtain an initial guess of the robot’s position.

![](images/2022_OverlapTransformer/e846295005de74eb51fe79e8a1efa4b9a42b0fd7db1e8d8fcc4fb017008776d1.jpg)  
Fig. 1. Query scan (blue) and reference scan (orange) with adjacent locations but opposite viewpoints in our novel Haomo dataset. Our OverlapTransformer is able to generate yaw-angle-invariant global descriptors with only range images, which is robust to viewpoint changing for place recognition.

In this letter, we propose a novel place recognition method utilizing range images produced by 3D LiDARs installed on an autonomous vehicle. The range image is a natural representation of a single 3D scan from a rotating LiDAR sensor such as Velodyne or Ouster sensors. It is a compact representation and is especially suitable for online tasks such as online SLAM [10], loop closing [6], [11], or localization [12], [13] due to its image-like structure. Instead of using handcrafted descriptors [14]–[16], we propose a new transformer neural network to extract yaw-angle-invariant global descriptors from LiDAR scans. We apply a similar overlap concept as described in OverlapNet [6], [11] to supervise the network learning and to estimate the similarity between pairs of scans. Different to OverlapNet, which exploits multiple cues such as normal, intensity, and semantic information as the network input, in this work we only use the depth information of the range image to achieve faster online performance and make the approach easier to generalize. Due to the yaw-angle-invariant design of the network, our approach can recognize places even when the vehicle drives in different directions, such as the reverse loop in our new Haomo dataset illustrated in Fig. 1.

The main contribution of this letter is a lightweight transformer neural network that exploits only depth information of range images to achieve place recognition. Our approach is very fast to execute and at the same time yields very good recognition results. Based on the attention mechanism of the Transformer [17] and the NetVLAD head [1], our proposed OverlapTransformer compresses LiDAR range images into global descriptors. We build the architecture of our OverlapTransformer to ensure that each descriptor is yaw-angle-invariant, which makes our method robust to viewpoint changes. We train the proposed OverlapTransformer only on a part of the KITTI dataset and evaluate it on both, KITTI and Ford Campus datasets with the loop closure metric in line with OverlapNet [11]. Besides, we recorded and released a new dataset, which contains three different challenges including place recognition for long time spans, reverse driving, and different appearance scenes to evaluate different methods.

In sum, we make the claims that our approach is able to (i) detect loop closure candidates for SLAM using only LiDAR data without any other information, and generalize well into the different environments without fine-tuning, (ii) achieve longterm place recognition on our Haomo dataset in outdoor largescale environments with a different LiDAR sensor, (iii) recognize places with changing viewpoints exploiting the proposed yaw-angle-invariant descriptors up to potentially discretization errors, (iv) run faster than most state-of-the-art place recognition methods.

## II. RELATED WORK

Place recognition is a common topic in computer vision and robotics with a large number of scientific work proposed using RGB images [1], [2], [4], [5], [18]. We refer more image-based methods to survey by Lowry [3] and focus our discussion here more on 3D LiDAR-based approaches.

Due to the high accuracy of the range information and illumination invariance, LiDAR-based place recognition has attracted attention in the field of autonomous vehicles. For example, He et al. [14] propose M2DP, which projects point cloud to multiple planes and combines descriptors from different planes to generate the global signature. Röhling et al. [15] count the height of point cloud to generate the histogram-based 1-D global descriptor for fast retrieval. Scan Context (SC) proposed by Kim et al. [7] encodes the maximum height of point cloud in different bins to generate the 2D global descriptor for high discrimination, but leads to an increased computational matching time. In contrast to SC using only geometric information, Cop et al. [19] propose Delight to encode intensity-reading of LiDAR into a group of histograms for comprehensive utilization of intensity and geometric information. Inspired by Cop et al., Wang et al. [20] extend SC by also exploiting both geometry and intensity, which outperforms geometric-only descriptors with the same space division method as used in SC. Recently, Wang et al. [9] propose LiDAR Iris exploiting the Fourier transform to generate a binary signature image. It firstly generates LiDAR iris images by expanding the bird-eye view of the LiDAR scan into an image strip. Then, it applies Fourier transform on LiDAR iris images and solves the spatial place recognition in the frequency domain.

With the development of deep learning, more learning-based approaches are utilized for place recognition divided into two groups, local feature-based methods, and global descriptorbased methods. Local feature-based approaches often have two steps, first extracting local features from the LiDAR scans and then recognizing places based on the extracted local features. For example, Dube et al. [21] propose SegMatch to firstly segment the filtered point cloud into sets of point clusters and then use features encoded by a CNN on such clusters to find place matches. Based on SegMatch, Vidanapathirana et al. [22] propose Locus, which uses higher-order pooling along with a non-linear transformation to aggregate multi-level features and generate a fixed-length global descriptor for place recognition. LPD-Net by Liu et al. [23] uses ten types of local features from raw point cloud clusters as input of a graph neural network to generate global descriptors. There are other learning-based approaches directly generating global descriptors on LiDAR scans for place recognition. For example, PointNetVLAD by Uy et al. [8] firstly uses PointNet [24] to map laser points into a higher dimension and then uses the NetVLAD architecture [1] previously used for visual place recognition to generate global descriptors for LiDAR-based place recognition. Komorowski et al. [25] utilize sparse 3D convolutions based on MinkowskiEngine with pooling layers to generate descriptors. In contrast, LoGG3D-Net by Vidanapathirana et al. [26] uses a local consistency loss to improve the performance of the global descriptor. SOE-Net by Xia et al. [27] combines the orientation-encoding module with PointNet to generate point-wise features, which are fed to a selfattention network to generate discriminative and compact global descriptors. Different to SOE-Net, Zhou et al. [28] propose NDT-Transformer, which transforms raw point cloud into NDT cells and uses the attention mechanism from Transformer [17] to improve the representation ability. PPT-Net by Hui et al. [29] exploits the pyramid point transformer module to enhance the discrimination oflocal features and generates a global descriptor.

Our method also directly generates global descriptors on LiDAR scans. Different from methods that use local point cloud maps [8], [23], [27]–[29], our method only uses range images generated from single 3D LiDAR scans. This yields fast computations suitable for online operation and natural yaw-angleinvariance for better place recognition performance. Our method also uses the Transformer similar to the work by Zhou et al. [28] to boost place recognition performance, but our method operates on range images instead of NDT cells.

Most recently, there are also works exploiting semantic information for place recognition. For example, Chen et al. [6], [11] propose OverlapNet to exploit multiple cues generated from LiDAR scan, including depth, normal, intensity, and semantics for LiDAR-based loop closure detection and localization. SGPR by Kong et al. [30] exploits the semantics and topological information of the raw point cloud and extracts the semantic graph representation with graph neural networks to find loop closures. Li et al. [31] use semantics to enhance SC and propose semantic scan context. Similarly, Cramariuc et al. [32] utilize semantic information to improve the performance of SegMatch. In contrast to these semantic-enhanced methods, our approach only uses the raw depth information to achieve online performance, which enables our method easier to generalize to different environments and datasets collected by different LiDAR sensors.

## III. OUR APPROACH

The overview of our OverlapTransformer is depicted in Fig. 2. The range image from raw point cloud is fed to an encoder, which is a modified OverlapNetLeg to extract features from range images (see Section III-A). Then, the encoded feature volume is fed to the transformer module, where we utilize a transformer to embed the relative location of features and the global information across the whole range image (see Section III-B). In the end, we use a global descriptor generator with NetVLAD and multi-layer perceptrons (MLPs) to compress the features and generate a yaw-angle-invariant 1-D global descriptor (see Section III-C). During training, we use the triplet loss with calculated overlap labels to better distinguish the positive and negative training examples (see Section III-D).

![](images/2022_OverlapTransformer/faf673b9dec35808f858a73ec4d6f16f872365672cd0cb91916dc9f9c360bef0.jpg)  
Fig. 2. Pipeline overview of our proposed approach. We take a 64-beam LiDAR as an example to introduce the dimensions of feature maps. The Range Image Encoder (RIE) compresses range images from the Lidar sensor to yaw-angle-equivariant feature maps. The Transformer Module (TM) concatenates feature maps from Range Image Encoder and discriminativeness-enhanced feature maps from Transformer Attention. Global Descriptor Generator (GDG) generates yaw-angle-invariant descriptors exploiting the combination of MLP and NetVLAD. The generated 1-D global descriptors are used for fast retrieval of place recognition.

## A. Range Image Encoder

We use range images from LiDAR scans as input data. The range image is an intermediate representation of LiDAR data obtained from a typical spinning mechanical LiDAR scanner with enough vertical resolution. Given the LiDAR sensor parameters, there is a projection transformation between a point cloud and a range image. A point cloud P can be projected to a range image R, $\Pi : \mathbb { R } ^ { 3 } \mapsto \mathbb { R } ^ { 2 }$ , where each pixel contains one 3D point. Each point $\pmb { p } _ { i } = ( x , y , z )$ is converted to image coordinates $( u , v )$ by

$$
\binom { u } { v } = \binom { \frac { 1 } { 2 } \left[ 1 - \mathrm { a r c t a n } ( y , x ) \pi ^ { - 1 } \right] w } { \left[ 1 - \left( \mathrm { a r c s i n } ( z r ^ { - 1 } ) + \mathrm { f } _ { \mathrm { u p } } \right) \mathrm { f } ^ { - 1 } \right] h } ,\tag{1}
$$

where $r = | | \pmb { p } | | _ { 2 }$ is the range, $\mathrm { f } = \mathrm { f } _ { \mathrm { u p } } + \mathrm { f } _ { \mathrm { d o w n } }$ is the vertical fieldof-view of the sensor, and w, h are the width and height of the resulting range image R.

The yaw rotation θ of a point cloud is yaw-angle-equivariant to the horizontal shift s of the corresponding range image, and the following equations hold:

$$
\left( { \begin{array} { c } { u ^ { \prime } } \\ { v ^ { \prime } } \end{array} } \right) \ = \ \left( { \begin{array} { c } { u + s } \\ { v } \end{array} } \right) , s = { \frac { 1 } { 2 } } \left[ 1 - \theta \pi ^ { - 1 } \right] w ,\tag{2}
$$

$$
\begin{array} { r } { \mathcal { R } C _ { s } = \Pi ( R _ { \theta } \mathcal { P } ) . } \end{array}\tag{3}
$$

The term $C _ { s }$ represents the column shift of the range image R by matrix right multiplication, and $R _ { \theta }$ represents the yaw rotation matrix of the point cloud P. Note that, there might be discretization errors in pixel coordinates when generating the range images from point clouds.

The range image with size of $1 \times h \times w$ is fed to the modified OverlapNetLeg, where 1 refers to the one range channel, while h, w are the height and width of the range image. Compared to OverlapNet [11], the convolution filters in our encoder only compress the range image in the vertical dimension but not the width dimension to avoid the discretization error to the yaw equivariance. Besides, there is no padding and dropout in our proposed architecture to keep the yaw equivariance for every intermediate feature generated by each network layer. We denote the output feature volume of the range image encoder as ${ \mathcal { F } } =$ RIE(R), with the size of $c \times 1 \times w$ , and $\mathcal { F } \dot { C _ { s } } = \mathrm { R I E } ( \Pi ( R _ { \theta } \mathcal { P } ) )$

holds. The variable c refers to the channel number of the encoded feature.

## B. Transformer Module

Inspired by NDT-transformer [28], we also exploit a transformer to extract more distinctive features for LiDAR place recognition. As shown in the middle of Fig. 2, our attentional feature transformer is composed of three modules, multi-head self-attention (MHSA), feed-forward network (FFN), and layer normalization (LN). Different from NDT-transformer, we use one transformer block in our transformer module to achieve high accuracy as well as efficiency.

The MHSA can learn the relationship between features captured by a self-attention mechanism. We denote the feature volume extracted by the MHSA as A, and the self-attention mechanism of the transformer can then be fomulated as:

$$
\mathcal { A } = \mathrm { A t t e n t i o n } ( Q , K , V ) = \mathrm { s o f t m a x } \left( \frac { Q K ^ { T } } { \sqrt { d _ { k } } } \right) V ,\tag{4}
$$

where $\{ Q , K , V \}$ are the query, key and value splits along the channels of the feature volumes generated by our range image encoder and $d _ { k }$ represents the dimension of splits.

A is then fed into the FFN and LN to generate the final attentional feature volume S, which can be calculated as:

$$
\mathcal { S } = \mathrm { L N } ( \mathrm { F F N } ( \mathrm { L N } ( \mathrm { C o n c } ( \mathcal { F } , \mathcal { A } ) ) ) + \mathrm { L N } ( \mathrm { C o n c } ( \mathcal { F } , \mathcal { A } ) ) ) ,\tag{5}
$$

where Conc(·) represents the concatenation operation across channels. In this way, a coarse feature $\mathcal { F }$ extracted by our range image encoder is upgraded to an attentional feature S.

Here we further show that our transformer module is also yaw-angle-equivariant. It is obvious that concatenation across channels, linear transformations, ReLU function, and LN are yaw-angle-equivariant. Since the query, key, and value feature volumes in attention mechanism are split along the channel dimension, given a shifted feature generated by our range image encorder $\mathcal { F } \bar { C } _ { s } = \operatorname { C o n c } ( Q C _ { s } , K \bar { C _ { s } } , V C _ { s } )$ , we have:

$$
\mathrm { A t t e n t i o n } ( \mathcal { F } C _ { s } ) = \mathrm { A t t e n t i o n } ( Q C _ { s } , K C _ { s } , V C _ { s } )\tag{6}
$$

$$
= \operatorname { s o f t m a x } \left( { \frac { Q C _ { s } ( K C _ { s } ) ^ { T } } { \sqrt { d _ { k } } } } \right) V C _ { s }\tag{7}
$$

$$
= \mathrm { s o f t m a x } \left( { \frac { Q K ^ { T } } { \sqrt { d _ { k } } } } \right) V C _ { s }\tag{8}
$$

$$
= { \mathcal { A } } C _ { s } .\tag{9}
$$

Since FFN is applied to each element in the feature volume separately and identically [17], it is therefore also yaw-angleequivariant towards features. Then, for the rest part of our attentional feature transformer, we also have:

$$
\begin{array} { r l } & { \mathrm { L N } ( \mathrm { F F N } ( \mathrm { L N } ( \mathrm { C o n c } ( { \mathcal F C } _ { s } , { \mathcal A C } _ { s } ) ) ) } \\ & { \quad + \mathrm { L N } ( \mathrm { C o n c } ( { \mathcal F C } _ { s } , { \mathcal A C } _ { s } ) ) ) } \\ & { = \mathrm { L N } ( \mathrm { F F N } ( \mathrm { L N } ( \mathrm { C o n c } ( { \mathcal F } , { \mathcal A } ) ) ) } \\ & { \quad \quad + \mathrm { L N } ( \mathrm { C o n c } ( { \mathcal F } , { \mathcal A } ) ) ) C _ { s } = { \mathcal S C } _ { s } . } \end{array}\tag{10}
$$

The transformer module finds the inner connections between different parts of the input feature volume, which exploits the spatial relations of different features in the scene. Similar to humans using distinctive landmarks and their relationship to determine the places, our transformer module also focuses on spatial features and their relationship using the attention mechanism, which boosts the place recognition performance of our method.

## C. Global Descriptor Generator

NetVLAD was first proposed by Arandjelovic et al. [1] to tackle image-based place recognition in an end-to-end manner and outperform its non-learning-based counterparts. Uy et al. [8] transfer it for LiDAR-based place recognition and proof that NetVLAD is permutation invariant, thus suitable for unordered point clouds. However, the original PointNetVLAD is not able to directly generate yaw-angle-invariant global descriptors for the point cloud because the yaw rotation of the point cloud leads to a different input of point coordinates and features. This makes the PointNetVLAD less robust to the rotation or noise of the sensor pose.

In this work, we leverage the permutation invariance of NetVLAD together with our yaw-angle-equivariant features to achieve yaw-rotation invariance and generate yaw-angleinvariant descriptors. We represent the input feature volume with size of $c \times 1 \times$ w as a set of 1-D vectors over channel dimension $\mathcal { S } = \mathcal { Z } = \{ z _ { 1 } , z _ { 2 } , . . . , z _ { i } , . . . , z _ { w } \}$ , where $z _ { i }$ is the i-th vector with size of c channels. Given that NetVLAD is permutation invariant [8], we have:

$$
\begin{array} { r l } & { \mathrm { G D G } ( \mathcal { Z } C _ { s } ) = \mathrm { G D G } ( \{ z _ { w - s } , . . . , z _ { w } , z _ { 1 } , . . . , z _ { w - s - 1 } \} ) } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad ( } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad ( } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad ( \mathrm { G D G } ( \{ z _ { 1 } , z _ { 2 } , . . . , z _ { i } , . . . , z _ { w } \} ) } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad ( } \\ & { \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad \quad ( } \end{array}\tag{11}
$$

(12)

We denote the output descriptor of our global descriptor generator as V and we have:

$$
\mathcal { V } = \mathrm { G D G } ( \mathcal { S } ) = \mathrm { G D G } ( \mathcal { S } C _ { s } ) ,\tag{13}
$$

which means our proposed global descriptor generator is yawangle-invariant.

In Fig. 3, we depict a toy example to illustrate that our model can generate yaw-angle-invariant descriptors with yaw-angleequivariant range image representations. We show at each row a different yaw rotation and the visualizations of the intermediate results of each module of our method. The yaw rotation of point clouds corresponds to the horizontal shift of range image pixels along the width dimension. Supposing the first row is the results of the raw LiDAR data, rotated $0 ^ { \circ }$ , the second and the third are rotated by 90 and 180 degrees along the yaw angle, which corresponds to the range images shifted by $\begin{array} { r } { { \frac { 1 } { 4 } } w } \end{array}$ and ${ \scriptstyle { \frac { 1 } { 2 } } } w$ respectively. As can be seen, the outputs of the yaw-angleequivariant OverlapNetLeg and transformer module are also shifted accordingly (shown by the second and the third column). Since the global descriptor generator is yaw-angle-invariant, it thus generates the same global descriptor in all three cases as shown in the fourth column.

![](images/2022_OverlapTransformer/85bfc96e206fd5defcb417d6017c430de85a489e6cbaacfb8b807bf469cc4e2e.jpg)  
Fig. 3. An illustration of our yaw-angle-invariant descriptors exploiting a toy example. As can be seen that the range image presentation, our range image encoder, and attentional feature transformer are yaw-angle-equivariant, and our global descriptor generator generates yaw-angle-invariant global features.

During online operation, we use the yaw-angle-invariant global descriptors to represent LiDAR scans and use the Euclidean distance between pairs of descriptors to find the nearest reference places.

## D. Network Training

We follow OverlapNet [11] using overlap to supervise the network rather than directly using ground truth distances. Overlap is a more natural way to describe the similarity between two LiDAR scans than distance. Moreover, the overlap between two LiDAR scans also corresponds to the quality of the following registration, which can be a good criterion for the final metric localization [11]. For a query scan $\mathcal { R } _ { q }$ and a reference scan $\mathcal { R } _ { r } ,$ we use the ground truth poses to reproject $\mathcal { R } _ { r }$ into the coordinate frame of $\mathcal { R } _ { q }$ and get $\mathcal { R } _ { r } ^ { \prime }$ . The overlap between them is calculated as:

$$
O _ { \mathcal { R } _ { q } \mathcal { R } _ { r } } = \frac { \sum _ { ( u , v ) } \mathbb { I } \left\{ | | \mathcal { R } _ { q } ( u , v ) - \mathcal { R } _ { r } ^ { \prime } ( u , v ) | | \leq \delta \right\} } { \operatorname* { m i n } \left( \mathrm { v a l i d } ( \mathcal { R } _ { q } ) , \mathrm { v a l i d } ( \mathcal { R } _ { r } ^ { \prime } ) \right) } ,\tag{14}
$$

where $\mathbb { I } ( a ) = 1$ if a is true and $\mathbb { I } ( a ) = 0$ otherwise, valid(R) refers to the counts of valid pixels of range image $\mathcal { R }$ , and δ is the threshold to decide the overlapped pixel.

For each training tuple, we utilize one query descriptor $\gamma _ { q } ,$ $k _ { p }$ positive descriptors $\left\{ \gamma _ { p } \right\}$ , and $k _ { n }$ negative descriptors $\{ \mathcal { V } _ { n } \}$ to compute lazy triplet loss:

$$
\begin{array} { r l } {  { \mathcal { L } _ { T } ( \mathcal { V } _ { q } , \{ \mathcal { V } _ { p } \} , \{ \mathcal { V } _ { n } \} ) } \quad } & { } \\ & { = k _ { p } ( \alpha + \operatorname* { m a x } _ { p } ( d ( \mathcal { V } _ { q } , \mathcal { V } _ { p } ) ) ) - \sum _ { k _ { n } } ( d ( \mathcal { V } _ { q } , \mathcal { V } _ { n } ) ) , } \end{array}\tag{15}
$$

where α is the margin to avoid negative loss and $d ( \cdot )$ is the squared Euclidean distance. We take a pair of scans whose overlap is larger than 0.3 as a positive sample, otherwise a negative sample. We use the triplet loss to minimize the distance between the query and the hardest positive global descriptors, and maximize the distance between the query and all sampled negative global descriptors.

TABLE I  
STATISTICS OF HAOMO DATASET
<table><tr><td>Sequence</td><td>1-1</td><td>1-2</td><td>1-3</td><td>2-1</td><td>2-2</td></tr><tr><td>Date</td><td>2021.12.8</td><td>2021.12.8</td><td>2021.12.8</td><td>2021.12.28</td><td>2022.1.13</td></tr><tr><td> $\mathrm { N _ { s c a n s } }$ </td><td>12500</td><td>22345</td><td>13500</td><td>100887</td><td>88154</td></tr><tr><td> $\underline { { \mathrm { N } _ { \mathrm { p o s . } } } }$ </td><td>40000</td><td>40000</td><td>一</td><td>48000</td><td>一</td></tr><tr><td> $\underline { { \mathrm { N } _ { \mathrm { n e g . } } } }$ </td><td>60000</td><td>60000</td><td>一</td><td>72000</td><td>一</td></tr><tr><td> $\underline { { \mathrm { ~ N } _ { \mathrm { q u e r i e s } } } }$ </td><td></td><td>一</td><td>1350</td><td></td><td>8815</td></tr><tr><td>Length</td><td>2.3 km</td><td>2.3 km</td><td>2.3 km</td><td>11.5km</td><td>11.1 km</td></tr><tr><td>Direction</td><td>Same</td><td>Reverse</td><td>一</td><td>Same</td><td></td></tr><tr><td>Role</td><td>Database</td><td>Database</td><td>Query</td><td>Database</td><td>Query</td></tr></table>

N<sub>scans</sub>, $\mathrm { N } _ { \mathrm { p o s . } }$ $\mathrm { N } _ { \mathrm { n e g . } }$ and $\mathrm { N _ { q u e r i e s } }$ are the numbers of scans, positive samples, negative samples and query scans respectively.

## IV. HAOMO DATASET

There are several LiDAR-based datasets for autonomous driving [33]–[35], which can be used for evaluating loop closure detection methods. However, not many of the publicly available datasets show significant repetitive reverse routes for long-term large-scale place recognition of autonomous driving. In this work, we therefore provide such a new challenging dataset called Haomo dataset and released it together with our code to support future research. The dataset was collected in urban environments of Beijing by a mobile robot built by HAOMO.AI Technology company equipped with a HESAI PandarXT 32- beam LiDAR sensor, a SENSING-SG2 wide-angle camera, and an ASENSING-INS570D RTK GNSS. There are currently five sequences in Haomo dataset as listed in Table I. Sequences 1-1 and 1-2 are collected from the same route in 8th December 2021 with opposite driving direction. An additional sequence 1-3 from the same route is utilized as the online query with respect to both 1-1 and 1-2 respectively to evaluate place recognition performance of forward and reverse driving. Sequences 2-1 and 2-2 are collected along a much longer route from the same direction, but on different dates, 2-1 on 28th December 2021 and 2-2 on 13th January 2022, where the old one is used as a database while the newer one is used as query. The two sequences are for evaluating the performance for large-scale long-term place recognition.

## V. EXPERIMENTAL EVALUATION

The experimental evaluation is designed to showcase the performance of our approach and to evaluate the claims that our approach is able to: (i) detect loop closure candidates for SLAM using only LiDAR data without any other information, and generalize well into the different environments without fine-tuning, (ii) achieve long-term place recognition on our Haomo dataset in outdoor large-scale environments, (iii) recognize places with changing viewpoints exploiting the proposed yaw-angle-invariant descriptors, (iv) run faster than most stateof-the-art place recognition methods.

## A. Implementation and Experimental Setup

We use three different datasets to evaluate our method, including KITTI dataset [33] collected in Germany with a 64- beam LiDAR sensor, Ford Campus dataset [34] collected in U.S. with a 64-beam LiDAR sensor, and our Haomo dataset collected in China with a 32-beam LiDAR sensor. Following OverlapNet [11], we use range images of size $1 \times 6 4 \times 9 0 { \bar { 0 } }$ for 64-beam LiDAR data of KITTI and Ford Campus datasets, and range images with size of $1 \times 3 2 \times 9 0 0$ for our Haomo dataset with 32-beam LiDAR data. For our transformer module, we set the embedding dimension $d _ { m o d e l } = 2 5 6$ , the number of heads $n _ { h e a d } = 4$ , and the intermediate dimension of the feedforward layer $d _ { f f n } = 1 0 2 4$ . We do not utilize dropout to achieve yaw-angle-equivariant. For NetVLAD, we set the intermediate feature dimension $d _ { i n t e r } = 1 0 2 4$ , the output feature dimension $d _ { o u t p u t } = 2 5 6$ , and the number of clusters $d _ { K } = 6 4$ . The output of our global descriptor generator is a vector of size 256. For calculating overlap ground truth by 14, we set $\delta = 1$ for KITTI odometry benchmark with provided 64-beam LiDAR scans, $\delta = 1 . 2$ for our Haomo dataset depending on the density of the points. We set $k _ { p } = 6 , k _ { n } = 6$ , and $\alpha = 0 . 5$ for the triplet loss.

## B. Evaluation for Loop Closure Detection

The first experiment supports our claim that our approach detects loop closure candidates for SLAM using only LiDAR data without any other information, and generalizes well into the different environments without fine-tuning. Following the experimental setup of OverlapNet [11], we train and evaluate our approach and other learning-based baseline methods on the KITTI Odometry Benchmark [33]. We use sequences 03–10 for training, sequence 02 for validation, and sequence 00 for evaluation. We regard two scans as a loop closure if their overlap value is larger than 0.3. To evaluate the generalization ability of our method, we also test it on sequence 00 of the Ford Campus dataset [34]. Note that we did not train our approach on the Ford Campus dataset, which shows the generalization ability of our method.

Since the semantics are not always available for the different datasets, we compare our method with existing methods which do not utilize semantic information, including Histogram [15], Scan Context [7] with augmentation, LiDAR Iris [9], PointNetVLAD [8], OverlapNet (Delta-Geo-Only) [11], NDT-Transformer-P (4096 cells) [28], and MinkLoc3D [25]. Loop closure detection during SLAM takes previous scans as the database, excluding the nearby 100 scans to avoid detecting the most recent scans. We utilize AUC, F1 max scores, recall@1 and recall@1% for evaluation and the results are shown in Table II. As can be seen, our method outperforms all the baseline methods on the KITTI dataset. Using our network pre-trained on KITTI directly on an unseen environment, Ford Campus dataset, without any fine-tuning, our proposed method still outperforms other methods, and only for recall@1%, our method is on par with the state-of-the-art non-learning-based method Scan Context [7], which shows the good generalization ability of our method.

## C. Evaluation for Place Recognition

In the second experiment, we investigate the long-term place recognition of our method on our Haomo dataset in outdoor large-scale environments with a different type of LiDAR sensor. The difference between loop closure detection and place recognition is that for place recognition the database/map is usually given and we need to find the location of the vehicle within the database/map. For loop closing in contrast, the database grows during operation. Therefore, we train our approach and other learning-based baseline methods on the database sequences and evaluate all the methods using the same new query sequences that are not parts of the database. We have three challenges in Haomo dataset. From easy to difficult, the first is a short-term same-direction challenge, sequence 1-1 as database and 1-3 for query. The second is the short-term inverse-direction challenge, sequence 1-2 as database and 1-3 for query. The last is a long-term large-scale challenge, sequence 2-1 as database and 2-2 for query. We use database sequences for training too. As shown in Table I, we firstly downsample the database and for each sampled laser scan, we calculate the overlaps between this scan to all other scans. For testing, we take one scan sampled from every ten scans in query sequences, and acquire its ground truth reference scans with overlap values larger than 0.3.

TABLE II COMPARISON OF LOOP CLOSURE DETECTION PERFORMANCE
<table><tr><td>Dataset</td><td>Approach</td><td>AUC</td><td>F1max</td><td>Recall @1</td><td>Recall @1%</td></tr><tr><td rowspan="9">KITTI</td><td>Histogram [15]</td><td>0.826</td><td>0.825</td><td>0.738</td><td>0.871</td></tr><tr><td>Scan Context [7]</td><td>0.836</td><td>0.835</td><td>0.820</td><td>0.869</td></tr><tr><td>LiDAR Iris [9]</td><td>0.843</td><td>0.848</td><td>0.835</td><td>0.877</td></tr><tr><td>PointNetVLAD [8]</td><td>0.856</td><td>0.846</td><td>0.776</td><td>0.845</td></tr><tr><td>OverlapNet [11]</td><td>0.867</td><td>0.865</td><td>0.816</td><td>0.908</td></tr><tr><td>NDT-Transformer-P [28]</td><td>0.855</td><td>0.853</td><td>0.802</td><td>0.869</td></tr><tr><td>MinkLoc3D [25]</td><td>0.894</td><td>0.869</td><td>0.876</td><td>0.920</td></tr><tr><td>Ours</td><td>0.907</td><td>0.877</td><td>0.906</td><td>0.964</td></tr><tr><td>[Histogram [15]</td><td>0.841</td><td>0.800</td><td>0.812</td><td>0.897</td></tr><tr><td rowspan="8">Ford Campus</td><td>Scan Context [7]</td><td>0.903</td><td>0.842</td><td>0.878</td><td>0.958</td></tr><tr><td>LiDAR Iris [9]</td><td>0.907</td><td>0.842</td><td>0.849</td><td>0.937</td></tr><tr><td>PointNetVLAD [8]</td><td>0.872</td><td>0.830</td><td>0.862</td><td></td></tr><tr><td>OverlapNet [11]</td><td>0.854</td><td>0.843</td><td>0.857</td><td>0.938 0.932</td></tr><tr><td>NDT-Transformer-P [28]</td><td>0.835</td><td>0.850</td><td>0.900</td><td>0.927</td></tr><tr><td>MinkLoc3D [25]</td><td>0.871</td><td>0.851</td><td>0.878</td><td>0.942</td></tr><tr><td>Ours</td><td>0.923</td><td>0.856</td><td>0.914</td><td>0.954</td></tr><tr><td></td><td></td><td></td><td></td><td></td></tr></table>

![](images/2022_OverlapTransformer/53d5e14321106f68e6cf61f1b4de8a3b572e0fee38f20dd590e5790b57776033.jpg)  
Fig. 4. Place recognition results using Haomo dataset sequence 1-3 as query and 1-1 as database (driving forward).

We utilize recall@N to evaluate the performance of algorithms on Haomo dataset as the most large-scale place recognition approaches do. We consider one query as a successful loop closure once we find one of the true references. The results of the first two challenges are shown in Figs. 4 and 5. As can be seen, our method outperforms other methods on these two challenges, especially for the reverse driving one due to the yawangle-invariant architecture design of our method. Although Scan Context and Histogram are also robust to pure rotation, our method outperforms them, since in real application there is not only pure yaw rotation but also changes in locations and environment appearance due to the dynamic objects. Our method is more robust to all challenging conditions compared to baseline methods. The evaluation on long-term large-scale sequences 2-2 and 2-1 are illustrated in Fig. 6, and our method also keeps its superiority on large-scale long-term datasets, which shows the ability of our method to be used for real autonomous driving applications.

![](images/2022_OverlapTransformer/704045fa6da81fa6f6ff09abdd8de8119413631162d9f7fc37bf2351c056e7b2.jpg)  
Fig. 5. Place recognition results using Haomo dataset sequence 1-3 as query and 1-2 as database (driving in reverse).

![](images/2022_OverlapTransformer/f043935e301b63b4422a0df1f1feda453c382f499e7115e3d504cd0bc16cfedc.jpg)  
Fig. 6. Place recognition results using Haomo dataset sequence 2-2 as query and 2-1 as database (driving in large-scale environments).

## D. Study on Yaw-Angle-Invariance

The third experiment investigates the yaw-angle-invariance of our approach. The results support the claim that our method generates yaw-angle-invariant descriptors and can recognize places with changing viewpoints. For that, we rotate each query scan of the KITTI dataset along the yaw-axis in steps of 30 degrees and search the places with respect to the same original database. In this experiment, we test all baselines and use Recall@1 as the evaluation metric. As illustrated in Fig. 7, our proposed OverlapTransformer is not influenced by pure rotation along the yaw-axis, and maintains the best performance compared to the baseline methods. On the contrary, PointNetVLAD, Scan Context, OverlapNet, NDT-Transformer-P and MinkLoc3D are all affected by rotation. PointNetVLAD, OverlapNet, and NDT-Transformer-P lose efficacy quickly for increasing yaw angle discrepancies. Here, we only use OverlapNet with the head of the similarity estimation for a fair comparison. Scan Context loses efficacy to some extent but is better than PointNetVLAD since the ring key representing occupancy ratio is yaw-angleinvariant. MinkLoc3D has the similar performance as Scan

![](images/2022_OverlapTransformer/66120916a88422f4d3c0151581f4e14f8746260a2f91dc0f8afab310224f7385.jpg)  
Fig. 7. yaw-angle-invariant test.

TABLE III  
ABLATION STUDY ON THE TRANSFORMER MODULE
<table><tr><td>Network</td><td>Runtime [ms]</td><td>Recall@1</td><td>Recall@5</td><td>Recall@20</td></tr><tr><td>RIE+GDG</td><td>0.86</td><td>0.642</td><td>0.750</td><td>0.841</td></tr><tr><td>RIE+Conv+GDG</td><td>0.92</td><td>0.698</td><td>0.841</td><td>0.923</td></tr><tr><td>RIE+1TM+GDG</td><td>1.37</td><td>0.776</td><td>0.878</td><td>0.931</td></tr><tr><td>RIE+3TM+GDG</td><td>2.39</td><td>0.788</td><td>0.875</td><td>0.940</td></tr><tr><td>RIE+6TM+GDG</td><td>3.95</td><td>0.741</td><td>0.852</td><td>0.913</td></tr></table>

Context. The LiDAR Iris and histogram-based method are also yaw-angle-invariant as our method but have lower recall@1 than ours. LiDAR Iris achieves yaw-angle-invariant by rotating its features multiple times and choosing the best scored one, which makes it very slow as shown in Section V-F. Our OverlapTransformer outperforms other baseline methods significantly due to our devised yaw-angle-invariant model exploiting yaw-angleequivariant range images.

## E. Ablation Study on Transformer Module

This ablation study validates the effectiveness of the transformer module (TM) we used between our range image encoder (RIE) and global descriptor generator (GDG). We use Haomo dataset sequence 1-2 as database and 1-3 for query to compare 5 different setups, including RIE+GDG, RIE+Conv+GDG, RIE+1TM+GDG, RIE+3TM+GDG, and RIE+6TM+GDG. RIE+GDG uses no transformer module. RIE+Conv+GDG replaces the transformer module with two 1×1 convolution layers without changing the channel numbers. RIE+1/3/6TM+GDG uses 1, 3, 6 TMs respectively. As shown in Table III, RIE+3TM+GDG and RIE+1TM+GDG outperform the other setups for place recognition. The results show that one transformer block already increases the performance significantly, while more transformer blocks lead to lower efficiency. When using more than 3 transformer blocks, the performance even decreases slightly. More transformer blocks might need more training data and time to obtain good performance. Thus, we only use one transformer block.

## F. Runtime

The experiment evaluates the runtime requirements of our method. It supports our last claim that our method runs faster than most of the state-of-the-art place recognition methods and that it can run at 730 Hz, i.e., much faster than the scanning rate. We compare the runtime of our OverlapTransformer with all baseline methods. We conduct all experiments on a system with an Intel i7-11700 K CPU and an Nvidia RTX 3070 GPU. We run all the methods to find top-1 candidates for one query scan with respect to a database consisting of 2000 reference scans. We report the averaged results over ten experiments. As shown in Table IV, we count the runtime separately for descriptor generation and searching. For the time cost by generating the descriptor, we take also the LiDAR data preprocessing into consideration. As can be seen, for descriptor generation, our method is the fastest method among the state-of-the-art learning-based methods with 1.37 ms for each scan, and slightly slower than the pure geometric histogram-based method. PointNetVLAD, NDT-Transformer-P, and MinkLoc3D need an extra downsampling process for raw point cloud, which is accelerated by GPU. OverlapNet needs to estimate the normals. For the time cost by searching, our method outperforms all baselines including the non-learning-based methods. Since Histogram, PointNetVLAD, NDT-Transformer-P, MinkLoc3D and our method do not need an ad-hoc function to compute the similarity of descriptors, FAISS library [36] is used to accelerate the searching. Note that, PointNetVLAD, NDT-Transformer-P and MinkLoc3D generate descriptors with the same size (1 × 256) as ours, but consume more time. The reason could be that the descriptors generated by our method are more descriptive than those by PointNetVLAD, NDT-Transformer-P, and MinkLoc3D, and thus need less time for searching.

TABLE IV  
COMPARISON OF RUNTIME WITH STATE-OF-THE-ART METHODS
<table><tr><td colspan="2">Approach</td><td>Descriptor Extraction [ms]</td><td>Searching [ms]</td></tr><tr><td rowspan="3">Hand crafted</td><td>Histogram [15]</td><td>1.07</td><td>0.46</td></tr><tr><td>Scan Context [7]</td><td>57.95</td><td>492.63</td></tr><tr><td>LiDAR Iris [9]</td><td>7.13</td><td>9315.16</td></tr><tr><td rowspan="5">Learning based</td><td>PointNetVLAD [8]</td><td>13.87</td><td>1.43</td></tr><tr><td>OverlapNet [11]</td><td>4.85</td><td>3233.30</td></tr><tr><td>NDT-Transformer-P [28]</td><td>15.73</td><td>0.49</td></tr><tr><td>MinkLoc3D [25]</td><td>15.94</td><td>8.10</td></tr><tr><td>Ours</td><td>1.37</td><td>0.44</td></tr></table>

## VI. CONCLUSION

In this letter, we presented a novel approach for LiDAR-based place recognition with a runtime less than 2 ms and outperforms the state-of-the-art methods in terms of place recognition performance. Our approach utilizes a lightweight network with a Transformer attention mechanism to generate yaw-angleinvariant descriptors, which allows us to handle challenging place recognition effectively and efficiently. We trained our method on the KITTI dataset and evaluated it on both, the KITTI and the Ford Campus dataset for loop closure detection, which shows a solid generalization capability. For further evaluating the place recognition ability on long time spans, reverses driving, and large-scale scenes, we developed a new challenging Haomo dataset and conducted extensive evaluations with multiple baseline approaches on it. The experimental results suggest that our method outperforms the other state-of-the-art methods in different challenging environments in terms of recognition performance and speed.

## REFERENCES

[1] R. Arandjelovic, P. Gronat, A. Torii, T. Pajdla, and J. Sivic, “NetVLAD: CNN architecture for weakly supervised place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2016, pp. 1437–1451.

[2] S. Hausler, S. Garg, M. Xu, M. Milford, and T. Fischer, “Patch-NetVLAD: Multi-scale fusion of locally-global descriptors for place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 14136–14147.

[3] S. Lowry et al., “Visual place recognition: A survey,” IEEE Trans. Robot., vol. 32, no. 1, pp. 1–19, Feb. 2016.

[4] O. Vysotska and C. Stachniss, “Lazy data association for image sequences matching under substantial appearance changes,” IEEE Robot. Automat. Lett., vol. 1, no. 1, pp. 213–220, Jan. 2016.

[5] O. Vysotska and C. Stachniss, “Effective visual place recognition using multi-sequence maps,” IEEE Robot. Automat. Lett., vol. 4, pp. 1730–1736, Apr. 2019.

[6] X. Chen et al., “OverlapNet: Loop closing for LiDAR-based SLAM,” in Proc. Robot.: Sci. Syst., 2020, pp. 1–10.

[7] G. Kim and A. Kim, “Scan context: Egocentric spatial descriptor for place recognition within 3D point cloud map,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 4802–4809.

[8] M. A. Uy and G. H. Lee, “PointNetVLAD: Deep point cloud based retrieval for large-scale place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2018, pp. 4470–4479.

[9] Y. Wang, Z. Sun, C.-Z. Xu, S. E. Sarma, J. Yang, and H. Kong, “LiDAR Iris for loop-closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5769–5775.

[10] X. Chen, A. Milioto, E. Palazzolo, P. Giguére, J. Behley, and C. Stachniss, “SuMa: Efficient LiDAR-based semantic SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2019, pp. 4530–4537.

[11] X. Chen, T. Läbe, A. Milioto, T. Röhling, J. Behley, and C. Stachniss, “OverlapNet: A. siamese network for computing LiDAR scan similarity with applications to loop closing and localization,” Auton. Robots, vol. 46, pp. 61–81, 2021.

[12] X. Chen, T. Läbe, L. Nardi, J. Behley, and C. Stachniss, “Learning an overlap-based observation model for 3D LiDAR localization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 4602–4608.

[13] X. Chen, I. Vizzo, T. Läbe, J. Behley, and C. Stachniss, “Range imagebased LiDAR localization for autonomous vehicles,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 5802–5808.

[14] L. He, X. Wang, and H. Zhang, “M2DP: A. novel 3D point cloud descriptor and its application in loop closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2016, pp. 231–237.

[15] T. Röhling, J. Mack, and D. Schulz, “A fast histogram-based similarity measure for detecting loop closures in 3-D LIDAR data,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2015, pp. 736–741.

[16] B. Steder, R. Rusu, K. Konolige, and W. Burgard, “NARF: 3D range image features for object recognition,” in Proc. IROS Workshop Defining Solving Realistic Percep. Problems Pers. Robot., 2010, pp. 1–2.

[17] A. Vaswani et al., “Attention is all you need,” in Proc. Adv. Neural Inf. Process. Syst., 2017, pp. 6000–6010.

[18] H. Jégou, M. Douze, C. Schmid, and P. Pérez, “Aggregating local descriptors into a compact image representation,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2010, pp. 3304–3311.

[19] K. P. Cop, P. V. Borges, and R. Dubé, “Delight: An efficient descriptor for global localisation using LiDAR intensities,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 3653–3660.

[20] H. Wang, C. Wang, and L. Xie, “Intensity scan context: Coding intensity and geometry relations for loop closure detection,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 2095–2101.

[21] R. Dubé, D. Dugas, E. Stumm, J. Nieto, R. Siegwart, and C. Cadena, “SegMatch: Segment based place recognition in 3D point clouds,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 5266–5272.

[22] K. Vidanapathirana, P. Moghadam, B. Harwood, M. Zhao, S. Sridharan, and C. Fookes, “Locus: LiDAR-based place recognition using spatiotemporal higher-order pooling,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 5075–5081.

[23] Z. Liu et al., “LPD-Net: 3D point cloud learning for large-scale place recognition and environment analysis,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2019, pp. 2831–2840.

[24] R. Q. Charles, H. Su, M. Kaichun, and L. J. Guibas, “PointNet: Deep learning on point sets for 3D classification and segmentation,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2017, pp. 77–85.

[25] J. Komorowski, “MinkLoc3D: Point cloud based large-scale place recognition,” in Proc. IEEE Winter Conf. Appl. Comput. Vis., 2021, pp. 1789–1798.

[26] K. Vidanapathirana, M. Ramezani, P. Moghadam, S. Sridharan, and C. Fookes, “LoGG3D-Net: Locally guided global descriptor learning for 3D place recognition,” 2021, arXiv:2109.08336.

[27] Y. Xia et al., “SOE-Net: A self-attention and orientation encoding network for point cloud based place recognition,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 11343–11352.

[28] Z. Zhou et al., “NDT-transformer: Large-scale 3D point cloud localisation using the normal distribution transform representation,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 5654–5660.

[29] L. Hui, H. Yang, M. Cheng, J. Xie, and J. Yang, “Pyramid point cloud transformer for large-scale place recognition,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 6078–6087.

[30] X. Kong et al., “Semantic graph based place recognition for 3D point clouds,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 8216–8223.

[31] L. Li et al., “SSC: Semantic scan context for large-scale place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 2092–2099.

[32] A. Cramariuc et al., “SemSegMap–3D segment-based semantic localization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 1183–1190.

[33] A. Geiger, P. Lenz, and R. Urtasun, “Are we ready for autonomous driving? The KITTI vision benchmark suite,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2012, pp. 3354–3361.

[34] G. Pandey, J. McBride, and R. Eustice, “Ford campus vision and LiDAR data set,” Int. J. Robot. Res., vol. 30, no. 13, pp. 1543–1552, 2011.

[35] X. Huang, P. Wang, X. Cheng, D. Zhou, Q. Geng, and R. Yang, “The apolloscape open dataset for autonomous driving and its application,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 42, no. 10, pp. 2702–2719, Oct. 2020.

[36] J. Johnson, M. Douze, and H. Jégou, “Billion-scale similarity search with GPUs,” IEEE Trans. Big Data, vol. 7, no. 3, pp. 535–547, Jul. 2021.