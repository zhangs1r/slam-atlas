# An Object SLAM Framework for Association, Mapping, and High-Level Tasks

Yanmin Wu , Student Member, IEEE, Yunzhou Zhang , Member, IEEE, Delong Zhu , Zhiqiang Deng , Wenkai Sun , Xin Chen, and Jian Zhang , Member, IEEE

Abstract—Object SLAM is considered increasingly significant for robot high-level perception and decision-making. Existing studies fall short in terms of data association, object representation, and semantic mapping and frequently rely on additional assumptions, limiting their performance. In this article, we present a comprehensive object SLAM framework that focuses on object-based perception and object-oriented robot tasks. First, we propose an ensemble data association approach for associating objects in complicated conditions by incorporating parametric and nonparametric statistic testing. In addition, we suggest an outlier-robust centroid and scale estimation algorithm for modeling objects based on the iForest and line alignment. Then a lightweight and object-oriented map is represented by estimated general object models. Taking into consideration the semantic invariance of objects, we convert the object map to a topological map to provide semantic descriptors to enable multimap matching. Finally, we suggest an object-driven active exploration strategy to achieve autonomous mapping in the grasping scenario. A range ofpublic datasets and real-world results in mapping, augmented reality, scene matching, relocalization, and robotic manipulation have been used to evaluate the proposed object SLAM framework for its efficient performance.

Index Terms—Augmented reality, data association, robotics, semantic mapping, visual SLAM.

Manuscript received 11 October 2022; revised 18 February 2023; accepted 3 April 2023. Date of publication 17 May 2023; date of current version 8 August 2023. This work was supported in part by the National Natural Science Foundation of China under Grant 61973066, in part by the Major Science and Technology Projects of Liaoning Province under Grant 2021JH1/10400049, in part by the Fundation of Key Laboratory of Aerospace System Simulation under Grant 6142002200301, in part by the Fundation of Key Laboratory of Equipment Reliability under Grant WD2C20205500306, and in part by the Fundamental Research Funds for the Central Universities under Grant N2004022. This paper was recommended for publication by Associate Editor J. Civera and Editor F. Chaumette upon evaluation of the reviewers’ comments. (Corresponding author: Yunzhou Zhang.)

## I. INTRODUCTION

the past two decades, which enables a wide application of visual SLAM in robots, autonomous driving, and augmented reality. The next generation of SLAM will require support for more intelligent tasks with a better capacity that we call “geometric and semantic Spatial AI perception” [1]. This will greatly extend the scope of traditional geometric localization and mapping.

In terms of geometric perception (e.g., point-based appearance modeling and handcrafted feature-based localization), more visual landmarks, such as the line [2], edge [3], and plane [4], are exploited to overcome environmental and motion challenges. Omnidirectional geometric perception is achieved by multisensor fusion of visual, thermal, inertia, LiDAR, GNSS, and UWB [5], [6], [7]. In onboard applications, these versatile and robust algorithms are extensively employed. However, due to the absence of semantic cues, geometric clues alone are insufficient for intelligent robot interaction and active decisionmaking, such as semantic mapping, object goal navigation, and object searching. This article focuses on another aspect of the next-generation SLAM: semantic perception, aiming at representing and understanding environmental information at a semantic level, which extends beyond the basic geometric appearance and position perception.

In semantic SLAM, the semantic cues provided by deep learning technology play an essential role in various subcomponents, e.g., localization, mapping, loop closure, and optimization. In this work, we focus on semantic-aided mapping and exploring multiple high-level applications based on the semantic map. Popular semantic mapping pipelines [8], [9], [10] parallelize the geometric SLAM workflow and learning-based semantic segmentation, and then annotate 3-D point clouds (or volume, mesh) with 2-D image segmentation labels. Finally, the multiframe segmentation results are fused with probabilistic approaches to build a global semantic map. Although, these point clouds-based semantic maps are visually appealing, they are not detailed and lack sufficient instance-specific information to assist the robot in performing fine-grained tasks. Therefore, the first insight of this article is that a helpful semantic map for robot operation should be instance- and object-oriented.

Object SLAM is an object-oriented branch of semantic SLAM that focus on constructing the map with objects as central entities and typically takes instance-level segmentation or object detection as the semantic network. Most studies on sparse SLAM [11], [12] associate point clouds with object landmarks and take the centroids of point clouds as the positions of objects. Other studies [13], [14], [15], [16] on dense SLAM improve the mapping results by denser point clouds and more precise segmentation/detection, enabling object-level reconstruction and dense semantic representation of objects. Nonetheless, these studies focus on the accuracy of object position, while the orientation and size of objects are not investigated, which are indeed indispensable for robotic tasks, such as manipulation and navigation. The second point of view presented in this article is that the object’s position, orientation, and size in the map should all be parameterized.

Object parameterization or representation is one of the primary missions of object SLAM. To address this problem, typical studies [17], [18], [19] usually include object models as a prior and the point clouds or shapes of target objects are known. The pose estimation of objects is then achieved by model retrieval and matching. The prior model is also integrated into the map and engaged in object-level bundle adjustment. Studies [20], [21], [22], [23] are examples that focus on categorized object models, which only take a partial knowledge of the object, such as the structure and shape, as the prior and use requires one model to represent one category. Although the object parameters are well encoded in the prior instance model or category model, obtaining the prior knowledge is difficult and expensive. In addition, the generalization capability of these models is limited. The third observation made in this work is that objects should be represented by general models with a high degree of generality and a low cost prior, such as the cube, cylinder, and quadric.

To summarize, this work aims to present an object SLAM framework that generates an object-oriented map with general models, which can parameterize the position, orientation, and size of objects in the map. In addition, we further explore high-level applications based on the object-oriented map. Some previous studies [24], [25] pursued a similar objective but encountered the following challenges.

1) The data association algorithms are insufficiently robust and accurate for dealing with complex settings involving various classes and numbers of objects.

2) Object parametrization is sloppy, typically depending on strict assumptions or achieving only incomplete modeling, both of which are difficult to achieve in practice.

3) Most studies focus on creating the object or semantic map, but the application in downstream tasks is not explored, nor is the map’s utility demonstrated. Instead, we discuss not only the fundamental techniques of object mapping but also high-level and object map-oriented applications.

In this article, we propose an object SLAM framework to achieve the desired objective while overcoming the aforementioned challenges. First, we integrate the parametric and nonparametric (NP) statistic tests and the traditional IoU-based method to conduct model ensembling for data association. Compared with conventional methods, our approach sufficiently exploits the nature of different statistics, e.g., Gaussian, non-Gaussian, 2-D, and 3-D measurements, hence exhibiting significant advantages in association robustness. Then, for object parametrization, we offer an algorithm for centroid, size, and orientation estimation and an object pose initialization approach based on the isolation forest (iForest) and line alignment. The proposed methods are robust to outliers and exhibit high accuracy, which significantly facilitates the joint pose optimization process. Finally, an object-oriented map is constructed using the general models taking cubes and quadrics as representations. Based on the map, we develop an augmented reality system to enable virtual–real fusion and interaction, transplant a framework for the robot arm to realize common objects’ modeling and grasping, and propose a novel object descriptor for subscene matching and relocalization.

This article extends our previous work [26], [27]. Extensions include semantic descriptor-based scene matching/relocalization (see Sections VI and VIII-F) and expanded experiments and analysis (see Section IX). The contributions are summarized as follows.

1) We propose an ensemble data association strategy that can effectively aggregate different measurements of the objects to improve association accuracy.

2) We propose an object pose estimation framework based on the iForest and line alignment, which is robust to outliers and can accurately estimate the pose and size of objects.

3) We build a lightweight and object-oriented map with general models, upon which we develop an augmented reality application aware of occlusion and collisions.

4) We extend the object map to a topological map and design a semantic descriptor based on the parameterized object information to enable multiple scene matching and objectbased relocalization.

5) We integrate object SLAM with robotic grasping tasks to propose an object-driven active exploration strategy that accounts for object observation completeness and pose estimation uncertainty, achieving accurate object mapping and complex robotic grasping.

6) We propose a comprehensive object SLAM framework that explores the key challenges and powerfully demonstrates its utility in various scenarios and tasks.

## II. RELATED WORK

## A. Data Association

Data association establishes the 2-D–3-D relationship between objects in image frames and the global map and the 2-D–2-D correspondence of objects between sequential frames. The most popular strategy considers it an object-tracking issue [11], [28], [29]. Li et al. [30] projected 3-D objects to the image plane and then perform association using the projected 2-D bounding boxes via the Hungarian object tracking algorithm. Some approaches [16], [31], [32], [33] use intersection over union (IoU) algorithm to track objects between frames, while tracking-based approaches are prone to create erroneous priors in complicated contexts resulting in wrong association results.

Some studies increase the utilization of shared information. Liu et al. [34] created a descriptor representing the topological relationships between objects, and instances with the greatest number of the shared descriptors are considered identical. Instead, Yang et al. [24] suggested using the number of matched map points on detected objects as an association criterion. Grinvald et al. [15] preseted a measurement of semantic label similarity, while Ok et al. [35] proposed to leverage the hue saturation histogram correlation. Sünderhauf et al. [14] compared the distance between distinct instances more directly. Typically, the designed criteria are inadequately general, exhaustive, or robust, leading to incorrect associations.

In terms of learning-based studies, Xiang et al. [36] suggested utilizing recurrent neural networks to achieve semantic label data association between consecutive images. However, they only focus on pixel-level associations. Similarly, Li et al. [37] used an attention-based GNN to maintain the detected 2-D and 3-D attributes. Merrill et al. [38] proposed a keypoint-based objectlevel SLAM system that projects the 3-D key points to the image as the prior ofthe objects in the next frame. However, this method is not verified on the SLAM dataset and cannot be generalized to previously unseen objects. Using a deep graph convolutional network, Xing et al. [39] extracted object features and perform feature matching. Nevertheless, this method is only suitable for well-constructed maps and is challenging for incremental maps of real-time SLAM.

Another viable option is the probabilistic-based solution. Bowman et al. [20] used a probabilistic method to model the data association process and leverage the EM algorithm to identify correspondences between observed landmarks. Subsequent studies [40], [41] extend the concept to associate dynamic objects or perform dense semantic reconstructions. However, their efficiency is limited by the high cost of the EM optimizers. Weng et al. [13] presented an NP Dirichlet process for semantic data association, which can address the challenges that arise when the statistics do not follow a Gaussian distribution. Later, Zhang et al. [42] and Ran et al. [43] introduced two variations of the hierarchical Dirichlet method for lowering association uncertainty. Iqbal et al. [12] also demonstrated the efficiency of NP data association. However, this strategy cannot properly address statistics with Gaussian distributions and is thus incapable of adequately leveraging diverse data in SLAM. We combine the parametric and NP methods to execute model ensembling, which exhibits superior association performance in complex scenarios with numerous object categories.

## B. Object Representation

Object representation in object SLAM can be divided into shape reconstruction-based and model-based methods. For the former category, Sucar et al. [23] inferred object volume from images using a variational auto encoder and thenjointly optimize object shape and pose. Wang et al. [32] adopted DeepSDF [44] as shape embedding, minimizing the surface consistency and depth rendering loss by observed point clouds. Similarly, Xu et al. [33] trained a shape completion network based on the pretrained DeepSDF to achieve complete shape reconstruction ofpartially seen objects. However, these methods are data-driven and significantly dependent on large-scale shape priors.

Model-based object representations are classified broadly into three types: prior instance-level models [17], [18], [19], [45], category-specific models, and general models. Prior instancelevel models rely on a well-established or trained database, such as detailed point clouds or CAD models. Since such models must be known in advance, their application scenarios are limited. In addition, studies [21], [22], [23] on category-specific models focus on identifying category-level characteristics. Parkhiya et al. [21] and Joshi et al. [22] represented different categories through the combination of line segments, but the category-specific feature is insufficiently general and is impossible to describe an excessive number of classes.

The general object models are represented by simple geometric elements, e.g., cube, quadric, and cylinder, which are the most efficient models. There are two typical modeling method. The first type infers the 3-D pose from the 2-D detection result. Yang et al. [24] leveraged the vanishing point to sample 3-D cube proposal from a single view, and then optimize the object pose using geometric measurements. Nicholson et al. [25] combined multiview observation to parametrize object landmarks as constrained dual quadrics. Subsequent studies [35], [46] refine quadric representation by incorporating shape and semantic priors and plane constraints. However, this inference from the 2-D object has a poor precision with significant errors. Li et al. [37] applied superquadric to tune between 3-D boxes and quadrics adaptively. However, they rely on additional 3-D object detection. Another type of methods resolve the 3-D object pose by 3-D point cloud measurements. Some studies [11], [12], [13] portray object position using point cloud centers, which is an imprecise way of expressing object properties. Runz et al. [47] got a dense object reconstruction result using more accurate instance and geometry-based segmentation. While the object’s position and size are viable, the orientation is ignored. Some other studies [24], [48], [49] involving direction estimation use the geometric characteristics of images or point clouds for orientation sampling and analysis. However, they face the problem of insufficient robustness. In contrast, studies [50], [51] use learning-based methods from orientation regression from the image, but there are issues in terms of accuracy and generalization. In this work, based on the general object model, we propose an outlier-robust object pose estimation algorithm using the iForest and line alignment method for better parametrization of object size and orientation.

## C. Semantic Scene Matching

Scene matching is critical for robot relocalization, loop closure, and multiagent collaboration. Conventional studies [52], [53] rely on keyframes and geometric features, which are vulnerable to failure when faced with changes in viewpoint, illumination, and appearance. Conversely, semantic-based scene matching is more efficient because of the time and space invariance of the semantic information (e.g., label and size).

Gawel et al. [54] focused on global scene matching for multiview robots and they propose a random-walk-based semantic descriptor to enable global localization by semantic graph match. Guo et al. [55] investigated large-scale scene matching problem and suggest a semantic histogram-based fast graph matching algorithm, resulting in more accurate and faster localization and map merging. However, these methods only account for the global matching of large scenes, disregarding the local information. In addition, the semantic information is not at the object level. Liu et al. [34] are interested in the issue of localization when environmental appearance changes. They suggest characterizing the scene with a dense semantic topology map and performing 6-DOF object localization by matching object descriptors. Similarly, Li et al. [30] focused on the relocalization of perspective changes. They use object landmarks to establish the correspondence between different views and conduct relocalization through graph matching based on the Hungarian algorithm. However, given the limited number of objects and the fact that they are not well-parameterized, their method is doubtful in a complicated setting with several repeating objects. To address the loop closure problem in multiobject scenes, Qin et al. [56] proposed to generate semantic subgraphs using objects’ semantic labels and then leverage Kuhn–Munkres to align subgraphs for estimating the transformation. However, the semantic clues are only used to determines the resemblance of scenes, while the translation between them is still calculated by geometric measures instead of semantic measures. In this work, we focus on scene matching and scene translation with multiple objects. Similar to previous studies, we create a topological map and design an object descriptor. In the map, the objects are fully parameterized, and the matching strategy based on object descriptors are also improved.

## D. Active Perception and Object Map-Based Grasping

Active perception is the process of actively adjusting sensor states by analyzing existing data to gather more valuable information for executing specific tasks, which is a critical characteristic of robot autonomy. Zhang et al. [57] leveraged Fisher information to predict the optimal sensor position to reduce localization uncertainty. Zeng et al. [58] exploited prior knowledge between objects to establish a semantic link graph for active object search. More specifically, active mapping is a specific type of active perception task concerned with autonomous map construction. Charrow et al. [59] utilized the quadratic mutual information to guide 3-D dense mapping. Wang et al. [60] also leveraged the mutual information to perform next best view (NBV) selection on a sparse road map, which subsequently acts as a semantic landmark to aid the mapping process. Kriegel et al. [61] proposed a surface reconstruction method for single unknown objects. In addition to the information gain, they also integrate the measurement of reconstruction quality into the objective function, achieving high accuracy and completeness. The key to active mapping is defining the measurement and strategy to guide the agent moving autonomously. We propose an information entropy-based uncertainty quantification and an object-driven active exploration strategy. Another significant difference from other methods is that the output of our suggested method is an object map compatible with complex robot manipulation tasks.

The object map encoded with object pose is available for robot object manipulation tasks, such as object placement and arrangement. Wada et al. [62] proposed reconstructing objects by incremental object-level voxel mapping. Voxel points initialize the object pose, and the ICP algorithm is then used to align the initialized object with the CAD model to optimize the pose further, which is heavily dependent on the CAD model’s registration accuracy. In NodeSLAM [23], the object is regarded as a landmark and is involved in joint optimization to help generate an accurate object map. The primary deficiency of this method is that the model requires a tedious category-level training process for each object. Labbé et al. [19] presented a single-view 6-DoF object pose estimation method and utilize the object-level bundle adjustment in the SLAM framework to optimize the object map. However, this method only focuses on known objects. Almeida et al. [63] leveraged the SLAM framework to densely map unknown objects for accurate grasping point detection, but the object pose is not estimated. In this work, we use the proposed SLAM framework to generate the object map actively, which enables the global perception to aid the robot in performing more intelligent tasks autonomously. In addition, unlike previous studies, we focus on the pose estimation of unknown objects.

## III. SYSTEM OVERVIEW

The proposed object SLAM framework is demonstrated in Fig. 1 including four parts. The tracking module builds upon the ORB-SLAM2 [52], which generates incremental sparse point clouds and estimates camera pose by extracting and matching multiview features. Our main contributions lie in the remaining three parts. The semantic module employs YOLO [64] as the object detector to provide semantic labels and bounding boxes which are then combined with point cloud measurements to associate the 2-D detected objects with 3-D global objects. After that, the iForest and the line alignment algorithms are applied to refine the point clouds and 2-D lines generated by the tracking module. Based on the association and refinement results, the objects are parameterized using the cube and quadric models.

The object map comprises of multiple parameterized objects and achieves a lightweight representation of the environment, which is a vital component of the application module. For the augmented reality application, virtual models’ 3-D registration is based on the real-world object pose rather than the conventional point-based approach. In addition, we convert the object map to a topological map, a graph representation of the objects and their relative poses. Based on this map, a semantic descriptor is designed to enable multiscene matching and relocalization tasks.

The accurate object pose is encoded in the object map, which provides the fundamental clues (e.g., grasping points) for robotic grasping applications. Notably, the object map is created actively, as depicted in the exploration module. Here, we propose an uncertainty measurement model to predict the NBV for exploration. The manipulator then actively moves to scan the table with the best view until building up a complete and accurate object map.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/94cc15c844c1ccbff5b33c7951d3fd595e003ba849fa6b9b045077b65a61108a.jpg)  
Fig. 1. Proposed object SLAM framework.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/b752c6d0aeec42f0902a81069111c54ce687f42f87778a8fb4b1e4b86e2a9640.jpg)  
Fig. 2. Pipeline of object-level data association.

In short, the proposed object SLAM leverages geometric and semantic measurements to simultaneously realize camera localization and object map building, resulting in a comprehensive system that addresses various challenges in this field and facilitates many intelligent and fascinating applications. The rest of this article is organized as follows. Sections IV and V present the principal theories of data association and object parameterization. The semantic descriptor and scene-matching method are defined in Section VI. The active exploration strategy is introduced in Section VII. Section VIII demonstrates the performance of our system through comprehensive experiments. Section IX provides the discussion and analysis. Finally, Section X concludes this article.

## IV. OBJECT-LEVEL DATA ASSOCIATION

Fig. 2 presents the pipeline of the proposed data association strategy. The local object is a 3-D instance observed in the current single view (t), where the point clouds correspond to

ORB features that lie in the 2-D bounding box, and the centroid is the mean of the points. The global object is an entity observed by multiple frames (before t) and already exists on the map, where point clouds and centroids also come from incremental measurement of multiviews. Data association aims to determine which global object in the map is associated with the local object in the current view. As shown in the pipeline, the camera motion IoU (M-IoU), NP test, single sample-t (S-t) test, and project IoU (P-IoU) will be used to determine whether or not the association is successful. In the experiment, the successful case should satisfy the fourth item and any ofthe first three items. If so, the existing global object will be updated; otherwise, a new global object will be created. Finally, the double sample-t (D-t) test is utilized to check whether duplicates exist.

Throughout this section, the following notations are used.

$P \in \mathbb { R } ^ { 3 \times | P | } , Q \in \mathbb { R } ^ { 3 \times | Q | } ,$ —the point clouds of the local object and the global object.

\- R—the rank (position) of a data point in a sorted list.

$\mathbf { c } \in \mathbb { R } ^ { 3 \times 1 }$ —the currently observed local object centroid.

$C = [ \mathbf { c } _ { 1 } , \mathbf { c } _ { 2 } , \hdots , \mathbf { c } _ { | C | } ] \in \mathbb { R } ^ { 3 \times | C | } \mathrm { - a }$ series of centroids of a global object observed by historical views. $C _ { 1 } , C _ { 2 }$ are similar.

\- f(·)—the probability function used for statistic test.

$m ( \cdot ) , \sigma ( \cdot ) \in \mathbb { R } ^ { 3 \times 1 }$ <sup>1</sup>—the mean and variance functions.

## A. IoU Model

If a global object is observed in the previous two frames (t − 1 and t − 2), we then predict the bounding box in the current frame (t) based on the hypothesis of uniform motion, and calculate the IoU between the predicted box ofa global object and the detected box of a local object, which we defined as M-IoU (see Fig. 2, M-IoU part). If the IoU value is large enough, there may be a potential association between the two objects. After NP and S-t (see Sections IV-B and IV-C), the P-IoU will validate this association by projecting 3-D point clouds of the global object to 2-D points on the current frame and fitting a box to these points. After that, we calculate the IoU between the projected and detected boxes (see Fig. 2, P-IoU part).

## B. NP Test Model

The m-IoU model provides a straightforward and efficient way of dealing with the scenario of consecutive frames. However, it will malfunction when:

1) the object is missed by the detector;

2) the object is occluded; or

3) the object disappears from the camera view.

The NP test model does not require continuous observations of the object, and can be directly applied to process two sets of point clouds, P and $Q$ (see Fig. 2, NP part), based on the hypothesis that point clouds follow a non-Gaussian distribution (which will be demonstrated in Section VIII-A). Theoretically, if P and Q represent the same object, they should follow the same distribution, $\mathrm { i } . \mathrm { e } . , f _ { P } = f _ { Q }$ . We use the Wilcoxon rank-sum test [65] to verify whether the null hypothesis holds.

We first mix the two point clouds $X = [ P | Q ] = [ \mathbf { x } _ { 1 }$ $\mathbf { x } _ { 2 } , \mathbf { \ j } . . . , \mathbf { x } _ { | X | } ] \in \mathbb { R } ^ { 3 \times ( | P | + | Q | ) }$ , and then sort X in three dimensions, respectively. Define $W _ { P } \in \mathbb { R } ^ { 3 \times 1 }$ as follows:

$$
W _ { P } = \left\{ \sum _ { k = 1 } ^ { | X | } \mathcal { R } ( 1 \{ \mathbf { x } _ { k } \in P \} ) - \frac { | P | ( | P | + 1 ) } { 2 } \right\}\tag{1}
$$

and $W _ { Q }$ is with the same formula. The Mann–Whitney statistics is $W { = } \mathrm { \dot { m } i n } ( W _ { P } , W _ { Q } )$ , which is proved to follow a Gaussian distribution asymptotically [66], [67]. Herein, we essentially construct a Gaussian statistics using the non-Gaussian point clouds. The mean and variance of W are calculated as follows:

$$
m ( W ) = ( | P | | Q | ) / 2\tag{2}
$$

$$
\sigma ( W ) = \frac { | P | | Q | \Delta ^ { + } } { 1 2 } - \frac { | P | | Q | ( \sum _ { i } \tau _ { i } ^ { 3 } - \sum _ { i } \tau _ { i } ) } { 1 2 ( | P | + | Q | ) \Delta ^ { - } }\tag{3}
$$

where, $\Delta ^ { + } = | P | + | Q | + 1 , \Delta ^ { - } = | P | + | Q | - 1$ , and $\tau \in$ $P \cap Q$ . τ represents the number of shared points between two objects; because its value is small, the complicated and lowcontributing second term in (3) is ignored in our implementation.

To make the null hypothesis stand, W should meet the following constraints:

$$
f ( W ) \geq f \left( r _ { r } \right) = f \left( r _ { l } \right) = \alpha / 2\tag{4}
$$

where, α is the significance level, $1 - \alpha$ is the confidence level, and $[ r _ { l } , r _ { r } ] \approx [ m - s \sqrt { \sigma } , m + s \sqrt { \sigma } ]$ defines the confidence region. The scalar $s > 0$ is defined on a normalized Gaussian distribution $\mathcal { N } ( s | 0 , 1 ) { = } \alpha$ . In summary, if the Mann–Whitney statistics W of two point clouds P and Q satisfies (4), we temporarily assume they come from the same object.

## C. Single-Sample and Double-Sample T-Test Model

The single-sample t-test is used to process object centroids observed in different views (see Fig. 2, S-t part), which typically follow a Gaussian distribution (see Section VIII-A).

Suppose the null hypothesis is that $C$ and c are from the same object, and define t statistics as follows:

$$
t = { \frac { m ( C ) - c } { \sigma ( C ) / { \sqrt { | C | } } } } \sim t ( | C | - 1 ) .\tag{5}
$$

For the null hypothesis to hold, t should satisfy

$$
f ( t ) \geq f ( t _ { \alpha / 2 , v } ) = \alpha / 2\tag{6}
$$

where, $t _ { \alpha / 2 , v }$ is the upper $\alpha / 2$ quantile of the t-distribution of v degrees of freedom, and $v = | C | - 1$ . If t statistics satisfy (6), we temporarily assume c and C come from the same object.

Some existing objects may be misidentified as new due to the abovementioned strict data association strategy, poor observation views, or erroneous object detection, resulting in duplicates. Consequently, a double-sample t-test is leveraged to determine whether to merge the two objects by analyzing their historical centroids (see Fig. 2, D-t part).

Construct t-statistics for $C _ { 1 }$ and $C _ { 2 }$ as follows:

$$
t = \frac { m ( C _ { 1 } ) - m ( C _ { 2 } ) } { \sigma _ { d } } \sim t ( | C _ { 1 } | + | C _ { 2 } | - 2 )\tag{7}
$$

$$
\sigma _ { d } = \sqrt { \frac { \left( | C _ { 1 } | - 1 \right) \sigma _ { 1 } ^ { 2 } + \left( | C _ { 2 } | - 1 \right) \sigma _ { 2 } ^ { 2 } } { | C _ { 1 } | + | C _ { 2 } | - 2 } \left( \frac { 1 } { | C _ { 1 } | } + \frac { 1 } { | C _ { 2 } | } \right) }\tag{8}
$$

where, $\sigma _ { d }$ is the pooled standard deviation of the two objects. Similarly, if t satisfies $( 6 ) , v = | C _ { 1 } | + | C _ { 2 } | - 2$ , it means that $C _ { 1 }$ and $C _ { 2 }$ belong to the same object, then we merge them.

## V. OBJECT PARAMETERIZATION

Data association provides the global object with multiview measurements that ensure more observations for parameterization to model objects effectively. Throughout this section, the following notations are used.

$\mathbf { \boldsymbol { t } } = [ t _ { x } , t _ { y } , t _ { z } ] ^ { T }$ —the translation (location) of object frame in world frame.

$\pmb { \theta } = [ \theta _ { r } , \theta _ { y } , \theta _ { p } ] ^ { T }$ —the rotation of object frame w.r.t. world frame. $R ( \pmb \theta )$ is matrix representation.

$T = \{ R ( \theta ) , t \}$ —the transformation of object frame w.r.t.   
world frame.

$\pmb { s } = [ s _ { l } , s _ { w } , s _ { h } ] ^ { T }$ —half of the side length of a 3-D bound ing box, i.e., the scale of an object.

$P _ { o } , P _ { w } \in \mathbb { R } ^ { 3 \times 8 }$ - the coordinates of eight vertices of a cube in object and world frame, respectively.

$Q _ { o } , Q _ { w } \in \mathbb { R } ^ { 4 \times 4 }$ —the quadric parameterized by its semiaxis in object and world frame, respectively, where $Q _ { o } =$ diag $\{ s _ { l } ^ { 2 } , \bar { s } _ { w } ^ { 2 } , s _ { h } ^ { 2 } , - 1 \}$

$\alpha ( \cdot ) -$ calculate the angle of line segments in the image.

$K , T _ { c }$ —the intrinsic and extrinsic parameters of camera.

$\pmb { p } \in \mathbb { R } ^ { 3 \times 1 }$ —the coordinates of a point in world frame.

## A. Object Representation

In this work, we leverage the cubes and quadrics/cylinders to represent objects, rather than the complex instance-level or category-level model. For objects with regular shapes, such as the book, keyboard, and chair, we use cubes (encoded by their vertices $P _ { o } )$ to represent them. For nonregular objects without an explicit direction, such as the ball, bottle, and cup, the quadric/cylinder (encoded by its semiaxis $Q _ { o } )$ is used for representation, and its orientation parameter is ignored. Here, $P _ { o }$ and $Q _ { o }$ are expressed in the object frame and only depend on the scale s. To register these elements to the global map, we also need to estimate their translation t and orientation θ w.r.t. the global frame. Cubes and quadrics in the global frame are expressed as follows:

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/c31b6bb545968bc8e3dcb27bd8f3c0379ef1fc7491fc95a54696d9f5811ab029.jpg)  
Fig. 3. Demonstration of object parameterization and iForest. (a-c) Demonstration of object parameterization. (d-e) Demonstration of iForest.

$$
P _ { w } = R ( \pmb { \theta } ) P _ { o } + \pmb { t }\tag{9}
$$

$$
Q _ { w } = T Q _ { o } T ^ { T } .\tag{10}
$$

These two models can be switched conveniently, as shown in Fig. 3(c). Assuming that objects are placed parallel with the ground, as in other works [68], [69], i.e., $\theta _ { r } { = } \theta _ { p } { = } 0 $ , we only need to estimate $[ \theta _ { y } , t , s ]$ for a cube and [t, s] for a quadric.

## B. Estimation of Translation(t) and Scale(s)

Assuming that the object point clouds X are in the global frame, we follow conventions and denote its mean by t, based on which the scale can be calculated by $s = ( \operatorname* { m a x } ( X ) -$ $\operatorname* { m i n } ( X ) ) / 2$ , as shown in Fig. 3(a). The main challenge here is that X is typical with many outliers, which will introduce a substantial bias to t and s. One of our major contributions in this article is the development of an outlier-robust centroid and scale estimation algorithm based on the iForest [70] to improve the estimation accuracy. The detailed procedure of our algorithm is presented in Algorithm 1.

The key idea ofthe algorithm is to recursively separate the data space into a series of isolated data points, and then take the easily isolated ones as outliers. The philosophy is that, normal points are typically located more closely and thus need more steps to isolate, while the outliers usually scatter sparsely and can be easily isolated with fewer steps. As indicated by the algorithm, we first create t isolated trees (the iForest) using the point cloud of an object (lines 2 and 14–33), and then identify the outliers by counting the path length of each point $\mathbf { x } \in X$ (lines 3–9), in which the score function is defined as follows:

$$
s ( \mathbf { x } ) = 2 \exp { \frac { - E ( h ( \mathbf { x } ) ) } { C } }\tag{11}
$$

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/edda763edcfb10af42261f4b5cc2fc13a5aa32822917687f3527fd8c8345cbad.jpg)  
Fig. 4. Line alignment to initialize object orientation. (a) Object and line detection in 2D image. (b-d) Angle sampling in 3D space. (e-g) Projection of angle sampling process in 2D images.

$$
C = 2 H ( | X | - 1 ) - \frac { 2 ( | X | - 1 ) } { | X | }\tag{12}
$$

where, C is a normalization parameter, H is a harmonic number $H ( i ) = \ln { ( i ) } + 0 . 5 7 7 2 1 5 6 6 4 9 , h ( \mathbf { x } )$ is the height of point x in the isolated tree, and E is the operation to calculate the average height. As demonstrated in Fig. 3(d)–(e), the yellow point is isolated after four steps; hence its path length is 4, whereas the green point has a path length of 8. Therefore, the yellow point is more likely to be an outlier. In our implementation, points with a score greater than 0.6 are removed and the remaining are used to calculate t and s (lines 10–12). Based on s, we can initially construct the cubics and quadratics in the object frame, as shown in Fig. 3(a)–(c).

## C. Estimation of Orientation(θ<sub>y</sub>)

The estimation of $\theta _ { y }$ is divided into two steps, namely, to find a good initial value for $\theta _ { y }$ first and then conduct numerical optimization based on the initial value. Since pose estimation is a nonlinear process, a good initialization is very important to help improve the optimality of the estimation result. Conventional methods [30], [47] usually neglect the initialization process, which typically yields inaccurate results.

The detail of orientation initialization algorithm is presented in Algorithm 2. The inputs are obtained as follows. 1) Line segment detector (LSD [71]) segments are extracted from t consecutive images, and those falling in the bounding boxes are assigned to the corresponding objects [see Fig. 4(a)]. 2) The initial pose of an object is assumed to be consistent with the global frame, i.e., $\theta _ { 0 } = 0$ [see Fig. 4(b)]. In the algorithm, we first uniformly sample 30 angles within $[ - \pi / 2 , \pi / 2 ]$ (line 2). For each sample, we then evaluate its score by calculating the accumulated angle errors between LSD segments $Z _ { \mathrm { l s d } }$ and the projected 2-D edges of 3-D edges Z of the cube (lines 3–12). The error is defined as follows:

$$
\begin{array} { l } { { e ( { \pmb \theta } ) = | | \alpha ( \hat { Z } ( { \pmb \theta } ) ) - \alpha ( Z _ { \mathrm { l s d } } ) | | ^ { 2 } } } \\ { { \hat { Z } ( { \pmb \theta } ) = K T _ { c } \left( R ( { \pmb \theta } ) Z + { \pmb t } \right) . } } \end{array}\tag{13}
$$

Algorithm 1: Centroid and Scale Estimation Based on iFor  
est.   
Input: X - The point cloud of an object, t - The number of   
iTrees in iForest, ψ - The subsampling size for an iTree.   
Output: $\mathcal { F }$ - The iForest, a set of iTrees, t - The origin of   
local frame, s - The initial scale of the object.   
1: procedure PARAOBJECT $( X , t , \psi )$   
2: $\mathcal { F } $ BUILDFOREST $( X , t , \psi )$   
3: for point x in X do   
4: $E ( h ) \gets :$ averageDepth $( \mathbf { x } , { \mathcal { F } } )$   
5: s ← score(E(h), C) - (11) and (12)   
6: if $s > 0 . 6$ then - an empirical value   
7: remove(x) - remove x from X   
8: end if   
9: end for   
10: t ← meanValue(X)   
11: $s \gets ( \mathrm { m a x } ( X ) \cdot \mathrm { m i n } ( X ) ) / 2$   
12: return $\mathcal { F } , t , s$   
13: end procedure   
14: procedure BUILDFOREST $( X , t , \psi )$   
15: ${ \mathcal { F } } \gets \phi$   
16: l ← ceiling(log ψ) - maximum times of   
iterations   
17: for $i = 1$ to t do   
18: X<sup>(i)</sup> ← randomSample(X, ψ)   
19: ${ \mathcal { F } } \gets { \mathcal { F } } \cup$ BUILDTREE $( X ^ { ( i ) } , 0 , l )$   
20: end for   
21: return $\mathcal { F }$   
22: end procedure   
23: procedure BUILDTREE $( X , e , l )$   
24: ${ \mathfrak { i f } } e \geq l { \mathrm { ~ o r ~ } } | X | \leq 1$ then   
25: return exNode{|X|} - record the size of X   
26: end if   
27: i ← randomDim(1, 3) - get one dimension   
28: q ← randomSpitPoint(X[i])   
29: $X _ { l } , X _ { r } \gets \mathrm { s p l i t } ( X [ i ] , q )$   
30: L ← BUILDTREE $( X _ { l } , e + 1 , l )$ - get child   
pointer   
31: R ← BUILDTREE $( X _ { r } , e + 1 , l )$   
32: return inNode $\{ L , R , i , q \}$   
33: end procedure

The demonstration of the calculation of $e ( \pmb \theta )$ is visualized in Fig. 4(e)–(g). The score function is defined as follows:

$$
\mathrm { S c o r e } \ = \frac { N _ { \mathrm { p } } } { N _ { \mathrm { a } } } ( 1 + 0 . 1 ( \xi - E ( e ) ) )\tag{14}
$$

where, $N _ { \mathrm { a } }$ is the total number of line segments of the object in the current frame, $N _ { \mathfrak { p } }$ is the number of line segments that satisfy $e < \xi , \xi$ is a manually defined error threshold (five degrees here), and $E ( e )$ is the average error of these line segments with $e < \xi .$ After evaluating all the samples, we choose the one that achieves the highest score as the initial yaw angle for optimization (line 13).

Algorithm 2: Initialization for Object Pose Estimation.   
Input: $\overline { { Z _ { 1 } , Z _ { 2 } , \ldots , Z _ { t } - 1 } }$ Line segments detected by LSD in   
t consecutive images, $\theta _ { 0 }$ - The initial guess of yaw angel.   
Output: θ - The estimation result of yaw angel, e - The   
estimation errors.   
1: $s , \mathcal { E }  \phi$   
2: Θ ← sampleAngles(θ<sub>0</sub>, 30) - see Fig. 4(b)–(d)   
3: for sample θ in Θ do   
4: $s _ { \theta } , e _ { \theta } \gets 0$   
5: for Z in $\{ Z _ { 1 } , Z _ { 2 } , \ldots , Z _ { t } \}$ do   
6: s, e ← score $( \theta , Z )$ - (13) and (14)   
7: $s _ { \theta } \gets s _ { \theta } + s$   
8: $e _ { \theta } \gets e _ { \theta } + e$   
9: end for   
10: $S \gets S \cup \{ s _ { \theta } \}$   
11: ${ \mathcal { E } } \gets { \mathcal { E } } \cup \{ e _ { \theta } \}$   
12: end for   
13: $\theta ^ { * } \gets \mathrm { a r g m a x } ( S )$   
14: return $\theta ^ { \ast } , e _ { \theta ^ { \ast } }$

## D. Object Pose Optimization

After obtaining the initial s and $\theta _ { y }$ , we then jointly optimize object and camera poses

$$
\{ O , T _ { c } \} ^ { * } = \underset { \{ \theta _ { y } , s \} } { \operatorname { a r g m i n } } \sum \left( e ( \pmb { \theta } ) + e ( \pmb { s } ) \right) + \underset { \{ T _ { c } \} } { \operatorname { a r g m i n } } \sum e ( p )\tag{15}
$$

where, the first term is the object pose error defined in (13) and the scale error $e ( s )$ is defined as the distance between the projected edges of a cube and their nearest parallel LSD segments. The second term $e ( p )$ is the commonly used reprojection error in the traditional SLAM framework.

## VI. OBJECT DESCRIPTOR ON THE TOPOLOGICAL MAP

After the step of object parameterization, we obtain the label, size, and pose information of a single object. To present the relationship between objects and that between objects and the scene, we create a topological map. The map is then used to generate an object descriptor for scene matching.

## A. Semantic Topological Map

The topological map is an abstract representation of the scene. In this work, to construct the semantic topological map, the 3-D object centroid is used to represent the node N that encodes the semantic label l and the object parameters t, θ, s. Then, under the distance and number constraints, we generate the undirected edge E between objects, which includes the distance d and angle α of two objects

$$
N = \left. l , t , \theta , s \right. , E = \left. d , \alpha \right. .\tag{16}
$$

Fig. 5(a) presents a real-world scene with multiple objects. Fig. 5(b) shows the object modeling result by the method of Section V, which is then used to create a semantic topological map [see Fig. 5(c)] that expresses the scene in an abstract way and shows the connection relationship between objects as symbolized in (16).

(a)  
![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/8727760b77d2fbdc3cdf8722416a62e5ecb8cf69948e40243c0c00b4271d584a.jpg)  
(b)  
(c)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/6bc01df4545bb9f97928448af0c5c65b7f21797169e598a6ae18aab6094b673f.jpg)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/1291bb3302abc2551ddf91b222efd7878639482d8f52eec07bffd97f37c12e70.jpg)  
Fig. 5. Semantic topological map and object descriptor. (a) Real-world scene. (b) Object-level map. (c) Semantic topological map. (d) Random walk descriptor. (e) 3D matrix visualization of a single descriptor.

## B. Semantic Descriptor

Since the object information, including semantic label, position, and scale, is not unique, the computation for undirected graph matching, an NP problem [72], is extremely high. To reduce the computational complexity and enhance the matching accuracy, we introduce a random-walk descriptor that weights multineighborhood measurements to describe an object, improving object uniqueness and the relationship with the scene.

The random-walk descriptor is represented by a 2-D matrix, as shown in Fig. 5(d), with each row storing a walking route that starts at the described object, and randomly points to the next object. It is worth noting that each object only appears once in a route and the process ends when reaching a certain depth i or time j limit.

The previous work [54] only considers the semantic label $\mathbf { l } = \left( l _ { 1 } , l _ { 2 } , \ldots , l _ { i } \right)$ as the descriptor. Benefiting from the abovementioned accurate object parameterization, we add three additional measurements, object size $\mathbf { s } = ( s _ { 1 } , s _ { 2 } , \ldots , s _ { i } )$ , distance $\mathbf { d } = ( d _ { 1 1 } , d _ { 1 2 } , \ldots , d _ { 1 i } )$ , and angle $\pmb { \alpha } = \left( \alpha _ { 1 1 } , \alpha _ { 1 2 } , \ldots , \alpha _ { 1 i } \right)$ , to improve the robustness of the descriptor. As shown in Fig. 5(e), thus we transfer the random-walk descriptor to a 3-D matrix form

$$
\boldsymbol { v } = ( r _ { 1 } , r _ { 2 } , . . . , r _ { j } ) ^ { T } , r _ { j } = ( 1 , \mathbf { s } , \mathbf { d } , \alpha ) ^ { T } .\tag{17}
$$

In our implementation, the additional measurement does not increase the computation. Instead, it accelerates the matching process by eliminating irrelevant candidates with more clues, such as label and size.

Algorithm 3 describes the procedure for scene matching. First, each object’s semantic descriptor is generated in two independent subtopological maps (lines 3–4, 10–17). Then, find the best matching object-pair by scoring the similarity ofeach element (l, s, d, α) (lines 5–7, 18–21). Finally, the transformation between two scenes is solved by singular value decomposition (SVD) according to the multiple object pairs (line 8).

Algorithm 3: Scene Matching Based on Object Descriptor.   
Input: $T _ { 1 } , T _ { 2 } \cdot$ Two sub-topo maps, $i , j$ - threshold of   
depth and number of random-walk.   
Output: $\mathbb { T }$ - Transformation between two maps.   
1: procedure POSESOLVE $( T _ { 1 } , T _ { 2 } , i , j )$   
2: $\mathcal { V } _ { 1 } , \mathcal { V } _ { 2 } , \mathcal { M }  \phi$   
3: $\mathcal { V } _ { 1 }$ ← OBJECTDESCRIPTOR $( T _ { 1 } , i , j )$   
4: $\nu _ { 2 }$ ← OBJECTDESCRIPTOR $( T _ { 2 } , i , j )$   
5: for object $v _ { 1 }$ in $\mathcal { V } _ { 1 }$ do   
6: $\mathcal { M }  \mathcal { M } \cup$ MATCH $( v _ { 1 } , \nu _ { 2 } )$   
7: end for   
8: return $\mathbb { T } \gets \operatorname { S V D } ( \mathcal { M } )$   
9: end procedure   
10: procedure OBJECTDESCRIPTOR $( T , i , j )$   
11: for object o in T do   
12: v.row ← random-walk from o to the ith object   
13: v.col ← repeat random-walk $j$ times   
14: $\mathcal { V }  \mathcal { V } \cup \boldsymbol { v }$ - (17) and Fig. 5(d),(e)   
15: end for   
16: return $\nu$   
17: end procedure   
18: procedure MATCH $( v _ { 1 } , \nu _ { 2 } )$   
19: v<sub>2</sub> ← maxScore $( v _ { 1 } , \nu _ { 2 } )$   
20: return $( v _ { 1 } , v _ { 2 } )$   
21: end procedure

There are some points worth mentioning in the following. 1) Scale ambiguity: Two maps are initialized with different depths resulting in distinct scales. While object size, such as Li et al. [50], provides a scale by length, width, and height, it is insufficiently robust. Instead, we find the matched object pair between two maps, then calculate the scale factor by averaging the ratio of the distance d. 2) Anomalous object: The mismatch resulting from the error object or novel object may cause a considerable inaccuracy in the resolution of the translation; therefore, the RANSAC algorithm is used to eliminate the disturbance caused by anomalous objects.

## VII. OBJECT-DRIVEN ACTIVE EXPLORATION

Object parameterization is good for quantifying the incompleteness of the object or map, and the incompleteness provides a driving force for active exploration. We consider the robotic grasping scene as an example. As shown in Fig. 6, the robot arm is fitted with a camera, the motion module controls the robot to execute observation commands. The perception module parametrized the object map by Sections IV and V. The analysis module measures object uncertainty and predicts different camera views’ information gains. The view with the greatest information gain is selected as the NBV and passed to the motion module to enable active exploration. We aim to incrementally build a global object map with the minimum effort and the maximum accuracy for robotic grasping.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/5e0a0249fe39bdfca5498b8c06413b01a4695a232040e3ea17f65832d66f8148.jpg)  
Fig. 6. Active mapping framework.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/ef415c7bbcfc3381ce6d0718ed38ba23eeedb932d9440e5ad2604b040ae97afa.jpg)  
Fig. 7. Illustration of observation completeness measurement. Left: Raw image. Center: Objects with point cloud. Right: Objects with surface grids.

## A. Observation Completeness Measurement

We focus on active map building and regard the incompleteness of the map as a motivating factor for active exploration. Existing studies usually take the entire environment as the exploration target [59], [73] or focus on reconstructing a single object [61], [74], neither of which is ideal for building the object map required by robotic grasping. The reasons are as follows. 1) The insignificant environmental regions will interfere with the decisions made for exploration and misguide the robot into the nonobject area. 2) It will significantly increase the computational cost and thus reduce the efficiency of the whole system. We propose an object-driven active exploration strategy for building the object map incrementally. The strategy is designed based on the observation completeness of the object, which is defined as follows.

As demonstrated in Fig. 7, the point clouds of an object are translated from the world frame to the object frame and then projected onto the five surfaces of the estimated 3-D cube. Here, the bottom face is not considered. Each of the five surfaces is discretized into a surface occupancy grid map [75] with cell size m ∗ m (m = 1 cm in our implementation). Each grid cell can be in one of three states as follows.

\- Unknown: the grid is not observed by the camera.

\- Occupied: the grid is occupied by the point clouds.

\- Free: the grid can be seen by the camera but is not occupied by the point clouds.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/f79d6954ac4bcf7b97a64a0fe97d6eb2c5493b0f6f8efde960697c468797e0fa.jpg)  
Fig. 8. Demonstration of the object-driven exploration. (a) Different definitions of information gain in exploration. (b) Information gain under different camera views.

We use information entropy [76] to determine the completeness of observations based on the occupancy grid map, as information entropy has the property of symptomatizing uncertainty. The entropy of each grid cell is defined by a binary entropy function

$$
H _ { \mathrm { g r i d } } ( p ) = - p \log ( p ) - ( 1 - p ) \log ( 1 - p )\tag{18}
$$

where, p is the probability of a grid cell being occupied and its initial value before exploration is set to 0.5. The total entropy is therefore defined as

$$
H _ { \mathrm { o b j } } = \sum _ { o \in \mathbb { O } } H _ { o } + \sum _ { f \in \mathbb { F } } H _ { f } + \sum _ { u \in \mathbb { U } } H _ { u }\tag{19}
$$

and the normalized total entropy is

$$
\bar { H } _ { \mathrm { o b j } } = H _ { \mathrm { o b j } } / ( | \mathbb { O } | + | \mathbb { F } | + | \mathbb { U } | )\tag{20}
$$

where, $H _ { o } , H _ { f } , H _ { u }$ are the entropy of occupied, free, and unknown grids, O, F, U are sets of the occupied, free, and unknown grid cells, respectively. |X| represents the size of X. As objects continue to be explored, the number of unknown grid cells is gradually reduced, making all grids’ normalized entropy $\bar { H } _ { \mathrm { g r i d } }$ a smaller value. The lower the $\bar { H } _ { \mathrm { g r i d } }$ is, the higher the observation completeness is. The exploration objective is to minimize $\bar { H } _ { \mathrm { g r i d } }$

## B. Object-Driven Exploration

Information Gain Definition: As illustrated in Fig. 8(b), object-driven exploration aims to predict the information gain of different candidate camera views and then select the one to explore that maximizes the information gain, i.e., the NBV. The information in this work is defined as the uncertainty of the map, as mentioned in Section VII-A. The information gain is thus defined as the measurement of uncertainty reduction and accuracy improvement after the camera is placed at a specific pose. Conventionally, information gain is defined based on the area of unknown regions of the environment, e.g., the black holes in the medium subfigure of Fig. 8(a), which may mislead the object map building. Compared with the conventional one, our proposed information gain is built on the observation completeness measurement of the object, shown in the right subfigure of Fig. 8(a), and incorporates the influence on object pose estimation.

Information gain modeling: As indicated by the definition, information gain is contingent on many factors; thus, we create a utility function to model the information gain by manually designing a feature vector to parameterize those factors. The following is the design of the feature vector used to characterize the object x:

$$
\mathbf { x } = \left( H _ { \mathrm { o b j } } , \bar { H } _ { \mathrm { o b j } } , R _ { o } , R _ { \mathrm { I o U } } , \bar { V } _ { \mathrm { o b j } } , s \right)\tag{21}
$$

where, $H _ { \mathrm { o b j } }$ , and $\bar { H } _ { \mathrm { o b j } }$ are defined by (18)–(20), $R _ { o }$ is the ratio of occupied grids to the total grids of the object, which indicates the richness of its surface texture, $R _ { \mathrm { I o U } }$ is the 2-D mean IoU with adjacent objects used for modeling occlusion under a specific camera view, $\bar { V } _ { \mathrm { o b j } }$ is the current volume of the object, and s is a binary value used for indicating whether the object is fully explored.

The utility function for NBV selection then is defined as follows:

$$
f = \sum _ { \mathbf { x } \in I } \left( ( 1 - R _ { o } ) H _ { \mathrm { o b j } } + \lambda ( H _ { \mathrm { I o U } } + H _ { V } ) \right) s ( \mathbf { x } )\tag{22}
$$

where, I is the predicted camera view, λ is a weight coefficient $( \lambda = 0 . 2$ in our implementation), and $H _ { \mathrm { I o U } }$ , H<sub>V</sub> share the same formula

$$
H = - p \log ( p ) .\tag{23}
$$

The first item $\textstyle \sum _ { \mathbf { x } \in I } ( 1 - R _ { o } ) H _ { \mathrm { o b j } }$ in (22) is used to model the total weighted uncertainty of the object map under the predicted camera view. Here, we give more weight to the unknown grids and the free ones by using $1 - R _ { o }$ . The reason is to encourage more explorations in free regions to find more image features that are neglected by previous sensing.

The second item $\textstyle \sum _ { \mathbf { x } \in I } H _ { \mathrm { I o U } }$ in (22) defines the uncertainty of object detection, which is one of the critical factors affecting object pose estimation. The uncertainty is essentially caused by occlusions between objects. We use this item to encourage a complete observation of the object. The variable in (23) is the rescaled 2-D IoU, $\mathrm { i . e . , } p = R _ { \mathrm { I o U } } / 2$

The third item $\textstyle \sum _ { \mathbf { x } \in I } H _ { V }$ in (22) models the uncertainty of object pose estimation. Under different camera views, the estimated object poses are usually different and induce the changes in object volume. Here, we first fit a standard normal distribution using the normalized history volumes $\{ \bar { V } _ { \mathrm { o b j } } ^ { ( 0 ) } , \bar { V } _ { \mathrm { o b j } } ^ { ( 1 ) } , \ldots , \bar { V } _ { \mathrm { o b j } } ^ { ( t ) } \}$ of each object, and then take the probability density of $\bar { V } _ { \mathrm { o b j } } ^ { ( t ) }$ as the value p in (23). This item essentially encourages the camera view that can converge the pose estimation process.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/c5f5e6cdc8ca03f70b523a9624c0ddf578efaceca039e3c86192a05583d913ab.jpg)  
(a)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/c73c47453447d9e935a79da60def82287e23b8cdf4b9e4c47f9739aaa3399f90.jpg)  
(b)  
Fig. 9. Distributions of different statistics in data association. (a) Position distribution of point clouds in three directions. (b) Distance error distribution of centroids.

The $s ( \mathbf { x } )$ in (22) indicates whether the object should be considered during the calculation of the utility function. Set $s ( \mathbf { x } ) { = } 0$ , if the following condition is satisfied: $( \bar { H } _ { \mathrm { g r i d } } < 0 . 5 \vee$ $R _ { o } > 0 . 5 ) \wedge p ( \bar { V } _ { \mathrm { o b j } } ^ { ( t ) } ) > 0 . 8 $ . If this condition holds for all the objects, or the maximum tries are achieved (10 in this work), the exploration will be finished.

Based on the utility function, the NBV that maximizes f is continuously selected and leveraged to guide the exploration process, during which the global object map is also incrementally constructed, as depicted in Fig. 6.

## VIII. EXPERIMENT

The experiment will demonstrate the performance of essential techniques, such as data association, object parameterization, and active exploration. In addition, the proposed object SLAM framework will be evaluated by various applications, such as object mapping, augmented reality, scene matching, relocalization, and robotic grasping.

## A. Distributions ofDifferent Statistics

For data association, the adopted 3-D statistics for statistical testing include the point clouds and their centroids of an object. To verify our hypothesis about the distributions of different statistics, we analyze a large amount of data and visualize their distributions in Fig. 9.

Fig. 9(a) shows the distributions of the point clouds from 13 objects during the data association in the TUM RGB-D fr3\_long\_office sequence [77]. Obviously, these statistics do not follow a Gaussian distribution. The distributions are related to specific characteristics of the objects, and do not show consistent behaviors. Fig. 9(b) shows the error distribution of object centroids, which typically follow the Gaussian distribution. This error is computed between the centroids of objects detected in each frame and the object centroid in the final, well-constructed map. This result verifies the reasonability of applying the NP Wilcoxon rank-sum test for point clouds and the t-test for object centroids.

## B. Object-Level Data Association Experiments

We compare our method with the commonly used IoU method, NP test, and t-test. Fig. 10 shows the association results of these methods in the TUM RGB-D fr3\_long\_office sequence. It can be seen that some objects are not correctly associated in Fig. 10(a)–(c). Due to the lack of association information, existing objects are often misrecognized as new ones by these methods once the objects are occluded or disappear in some frames, resulting in many unassociated objects in the map. In contrast, our method is much more robust and can effectively address this problem [see Fig. 10(d)]. The results of other sequences are shown in Table I, and we use the same evaluation metric as [12], [78], which measures the number of objects that are finally present in the map. The GT represents the ground-truth object number. As we can see, our method achieves a high success rate of association, and the number of objects in the map goes closer to GT, which significantly demonstrates the effectiveness of the proposed method.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/7b09d93bee00ec500b03c9b550a40a17f72e416f9af5ea98c884e3d95d1e6f3b.jpg)  
(a)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/26af18ffd933189ea88673c64529123d0d23ce146a269617477f1f1d49f55533.jpg)  
(b)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/4902ac8d8d60ca69c480141686a9bde2835bb8736ecc4b49b1f1a61bd5e92037.jpg)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/da7bbe0284c4202928ae5845d90edf1ad041b6d7d66e4f76f04c513922cfdeb7.jpg)  
Fig. 10. Qualitative comparison of data association results. (a) IoU method. (b) IoU and NP test. (c) IoU and t-test. (d) Our ensemble method.

TABLE I  
DATA ASSOCIATION RESULTS
<table><tr><td></td><td>IoU</td><td>IoU+NP</td><td>IoU+t-test</td><td>Ours</td><td>GT</td></tr><tr><td>Fr1_desk</td><td>62</td><td>47</td><td>41</td><td>14</td><td>16</td></tr><tr><td>Fr2_desk</td><td>83</td><td>64</td><td>52</td><td>22</td><td>25</td></tr><tr><td>Fr3_office</td><td>150</td><td>128</td><td>130</td><td>42</td><td>45</td></tr><tr><td>Fr3_teddy</td><td>32</td><td>17</td><td>21</td><td>6</td><td>7</td></tr></table>

Bold means better.

The results of our comparison with [12], [78], which is based on the NP test, are reported in Table II. As indicated, our method can significantly outperform [12], [78]. Especially in the TUM dataset, the number of successfully associated objects by our method is almost twice that by [12], [78]. The advantage in Microsoft RGBD [79] and Scenes V2 [80] is not apparent since the number of objects is limited. Reasons for the inaccurate association of [12], [78] lie in two folds as follows. 1) The method does not exploit different statistics and only uses NP statistics, thus resulting in many unassociated objects. 2) A clustering algorithm is leveraged to tackle the abovementioned problem, but it removes most of the candidate objects.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/412c5d0030abfbbf88bd293d4c2f8417864a5fd489de698be8e38828894e4481.jpg)  
(a)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/11b382630630f921a270d33d89c811adb2ed8c164a68d3aa8caf8de4819d5913.jpg)  
(b)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/907835b2fb14bdb959670334249317f525a5dc1014d16011d7c5d533a5903d33.jpg)  
(c)  
Fig. 11. Visualization of the pose estimation. (a) Initial object pose and size. (b) Object pose and size after iForest. (c) Object pose and size after iForest and line alignment.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/d2075d96c63ac5f98f476a5a65abca073bf0c927c4bb336908a7644f21983f2d.jpg)  
Fig. 12. Results of object pose estimation. Odd columns: original RGB images. Even column: estimated object poses.

## C. Qualitative Assessment ofObject Parameterization

To demonstrate the accuracy of object parameterization, We superimpose the cubes and quadrics of objects on semidense maps for qualitative evaluation. Fig. 11 is the 3-D top view of a keyboard [see Fig. 4(a)] where the cube characterizes its pose. Fig. 11(a) is the initial pose with large-scale error; Fig. 11(b) is the result after using iForest; Fig. 11(c) is the final pose after our joint pose estimation. Fig. 12 presents the pose estimation results of the objects in 14 sequences of the three datasets, in which the objects are placed randomly and in different directions. As is shown, the proposed method achieves promising results with a monocular camera, which demonstrates the effectiveness of our pose estimation algorithm.

## D. Object-Oriented Map Building

Then, we build the object-oriented semantic maps based on the robust data association algorithm, the accurate object pose estimation algorithm and a semidense mapping system [81]. Fig. 13 shows three examples of TUM fr3\_long\_office and fr2\_desk, where (d) and (e) show semidense semantic and object-oriented maps built by our object SLAM. Compared with the sparse map of ORB-SLAM2, our maps can express the environment much better. Moreover, the object-oriented map shows superior performance in environment understanding than the semidense map.

TABLE II  
QUANTITATIVELY ANALYZED DATA ASSOCIATIONS
<table><tr><td rowspan="2">Seq</td><td colspan="4">TUM</td><td colspan="5">Microsoft RGBD</td><td colspan="5">Scenes V2</td></tr><tr><td>fr1_desk</td><td>fr2_desk</td><td>fr3_long_office</td><td>fr3_teddy</td><td>Chess</td><td>Fire</td><td>Office</td><td>Pumpkin</td><td>Heads</td><td>01</td><td>07</td><td>10</td><td>13</td><td>14</td></tr><tr><td>[12], [78]</td><td>一</td><td>11</td><td>15</td><td>2</td><td>5</td><td>4</td><td>10</td><td>4</td><td>一</td><td>5</td><td>一</td><td>6</td><td>3</td><td>4</td></tr><tr><td>Ours</td><td>14</td><td>22</td><td>42</td><td>6</td><td>13</td><td>6</td><td>21</td><td>6</td><td>15</td><td>7</td><td>7</td><td>7</td><td>3</td><td>5</td></tr><tr><td>GT</td><td>16</td><td>23</td><td>45</td><td>7</td><td>16</td><td>6</td><td>27</td><td>6</td><td>18</td><td>8</td><td>7</td><td>7</td><td>3</td><td>6</td></tr></table>

Bold means better.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/2c49bd07041b96fb3f87a8219f264770bffc9ed2731f70e8464650e9ed867b3f.jpg)

(a)  
![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/169220f61dd74f8be3f426fdf2807b32e028b2bcf42cd6707445e1feefb08ee3.jpg)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/138403e46a093340251dda5c47fdd303d14364ee675bce02c2b761d55113cde9.jpg)  
(b)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/8e08299f521cf0e8fe0ca5adaacf3fffed7ff2c376eacb2d97470cf234d4af46.jpg)  
(c)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/0b9443fea16d69466a40b19f74acfd2c2468ae2456d583af1f9e6558dce817e7.jpg)  
(d)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/7f3ade976648a26aa53f11687dc5fa2ad4aa5790cc3c33b5b2bdcf62d762a3ef.jpg)  
(e)  
Fig. 13. Different map representations. (a) RGB images. (b) Sparse map. (c) Semidense map. (d) Our semidense semantic map. (e) Our lightweight and object-oriented map. (d) and (e) are built by the proposed method.  
TUM: fr1\_desk

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/bd1162d8d76aa4cc58e33df9acd99868a070c84040722c58c77eefed4f767f59.jpg)  
Fig. 14. Mapping results on the three datasets. Top: raw images. Bottom: semidense object-oriented map.

The mapping results of other sequences in TUM, Microsoft RGB-D, and Scenes V2 datasets are shown in Fig. 14. It can be seen that the system can process multiple classes of objects with different scales and orientations in complex environments. Inevitably, there are some inaccurate estimations. For instance, in the fire sequence, the chair is too large to be well observed by the fast-moving camera, thus yielding an inaccurate estimation.

We also conduct the experiment in a real scenario (see Fig. 15). It can be seen that even if the objects are occluded, they can be accurately estimated, which further verifies the robustness and accuracy of our system.

## E. Augmented Reality Experiment

Early augmented reality used QR codes, 2-D manual features, or image temples to register virtual 3-D models, resulting in a restricted range of motion and poor tracking. The sparse point cloud map created by SLAM enables large-scale tracking and high-robust registration for AR. Geometric SLAM-based AR, however, is only concerned with accuracy and robustness, not authenticity. Conversely, our object SLAM-based AR provides complete environment information, thus a more realistic immersive experience can be achieved.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/674b6d6459795315a90a98183903ecd0eaafe86980c3f93e8ed4669eb0a8825b.jpg)  
Fig. 15. Mapping results in a real scenario. Top: raw images. Middle: semidense object-oriented map. Bottom: lightweight and object-oriented map.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/030bdea9a7beec83756eec95d2feae5512690f9d87dd788ca34046ac243db443.jpg)  
Fig. 16. Raw image and the corresponding object map.

In the case of the desk scene in Fig. 16(a), we use the method described previously to construct an object map, as shown in Fig. 16(b), in which we model objects such as the book, keyboard, and bottles.

3-D registration: We present an object-triggered virtual model registration method, instead of 3-D registration triggered by a plane or position humanly specified. As shown in Fig. 18(a), the top row represents three raw frames from the video stream, while the bottom row represents the corresponding real–virtual integration scene. Virtual models can be seen registered on the desk to replace real objects based on the object semantics, pose, and size encoded in the object map.

Occlusion and collision: Physical occlusion and collision between the actual scene and virtual models is the crucial reflection of augmented reality. The top row, as seen in Fig. 18(b), is the result of common augmented reality, where virtual models are registered on the top layer of the image, resulting in an unrealistic separation of real and virtual scenes. The bottom row of Fig. 18(b) shows the outcome of our object SLAM-based augmented reality, in which the foreground and background are distinguished, and the real object obscures a portion of the virtual model, where the virtual and physical worlds are fused together. Similarly, Fig. 18(c) depicts the collision effect. The virtual model in the top row falls on the desk without colliding with the bottle. Contrarily, the bottom row shows the outcome of our object SLAM-based augmented reality, in which the virtual model falls and collides with the real bottle, with the dropping propensity changing.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/a4ff5db873262891bba3d21767106db93bebdff7804d50125e2b17051547e1dd.jpg)  
Fig. 17. Demonstration of interaction. Reactions are visualized as a series of augmented reality events.

Semantic interaction: Interaction, cascading user command with the real scene and virtual models, plays a crucial role in augmented reality applications. As shown in Fig. 17, clicking different real-world objects produces different virtual interactive effects.

The above functions, object-triggered 3-D registration, occlusion, collision, and interaction, rely on accurate object perception of the proposed object SLAM framework. The experimental results demonstrate that object SLAM-based augmented reality has a fascinating benefit in areas, such as gaming, military training, and virtual decorating.

## F. Object-Based Scene Matching and Relocalization

Scene matching: In this experiment, we evaluate the performance of the proposed object descriptor-based scene matching, which is crucial for multiagent collaboration, scene reidentification, and multimaps merging at different periods. We acquire two separate trajectories and their associated object maps in the same scene, then utilize the suggested method to figure out their relationship. Fig. 19 illustrates the map-matching results in three settings.

The results of the TUM and Microsoft sequences are shown in Fig. 19(a) and (b). The two maps with different scales and numbers ofobjects match accurately, and the translation between them is also resolved. The match is not based on point clouds or bag of words of keyframes, but the semantic object descriptor constructed by the object topological map. In addition, the scale inconsistency of the two maps is also eliminated. Fig. 19(c) shows a real-world example of the matched result. Apart from the previous features, what is worth noting is that the two maps were recorded under different illumination. With this scenario, the traditional appearance-based method is trends to fail, demonstrating the robustness of the proposed object descriptor with the semantic level invariance property.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/cd7508fb74375765ce0f82567135b0b7fd4fbfb32dea9a0a6001726443377798.jpg)  
Fig. 18. 3-D registration, occlusion, and collision in object SLAM-based augmented reality. (a) Object-triggered 3-D registration. Top: raw images of the scene. Bottom: augmented reality scene with registered virtual models in place of the original objects. (b) Demonstration of occlusion. Top: the standard AR without occlusion. Bottom: our object SLAM-based AR. (c) Demonstration of collision. Top: the standard augmented reality. Bottom: our object SLAM-based augmented reality with the awareness of collision.

TABLE III  
TIME ANALYSIS OF SCENE MATCHING (MS)
<table><tr><td rowspan="2">Scene</td><td rowspan="2">Object num.</td><td colspan="2">Dscriptor generate</td><td rowspan="2">Match</td><td rowspan="2">Pose resolve</td><td rowspan="2">Total</td></tr><tr><td>Map1</td><td>Map2</td></tr><tr><td>1</td><td>6+4</td><td>0.203</td><td>0.167</td><td>0.458</td><td>0.286</td><td>0.744</td></tr><tr><td>2</td><td>10+8</td><td>0.213</td><td>0.184</td><td>0.673</td><td>0.383</td><td>1.056</td></tr><tr><td>3</td><td>14+14</td><td>0.483</td><td>0.437</td><td>1.001</td><td>0.891</td><td>1.892</td></tr><tr><td>Ave</td><td>10+8.7</td><td>0.300</td><td>0.263</td><td>0.711</td><td>0.520</td><td>1.231</td></tr></table>

TABLE IV

RELOCALIZATION SUCCESS RATE UNDER DIFFERENT PARALLAX
<table><tr><td rowspan="7">Query QQQ Prior keyframes keyframes Parallax 日</td><td rowspan="3">Total times</td><td rowspan="3">Succ. times</td><td rowspan="3">Parallax</td><td colspan="2">Success rate(%)</td></tr><tr><td>Ours</td><td>[82]</td></tr><tr><td>14.9</td><td>32.5</td></tr><tr><td>329 648</td><td>49 96</td><td>&lt;20 20-50</td><td>14.8</td><td>0</td></tr><tr><td>900</td><td>109</td><td>50-100</td><td>12.1</td><td>0</td></tr><tr><td></td><td></td><td></td><td></td><td></td></tr></table>

Bold means better.

Table III analyzes the performance time of the algorithm. The matching duration is found to be the primary cost, and the time is positively related to the number of objects. The average total duration is approximately 1.23 ms, which is both practical and economical in various robot applications.

Relocalization: We perform relocalization experiments with parallax to demonstrate the robustness of the proposed matching method to viewpoint changes. As illustrated in the figure in Table IV, we first construct a prior map with a set of prior keyframes and then utilize query keyframes for relocalization, which do not overlap the trajectories of prior keyframes and have parallax. We conduct several repeated experiments under different parallax conditions and compare the success rate of relocalization with ORB-SLAM3 [82]. When the parallax is less than 20<sup>◦</sup>, as shown in Table IV, ORB-SLAM3 achieves a relocalization success rate of 32.5%; however, the rate drops sharply to 0 when the parallax is greater than 20<sup>◦</sup>, which demonstrates that the appearance-based descriptor represented by ORB-SLAM3 is extremely sensitive to parallax. Conversely, our method is robust to parallax and achieves a success rate of over 12% even under challenging large parallax.

TABLE V  
RELOCALIZATION SUCCESS RATE UNDER DIFFERENT PUBLIC OBJECT RATIO
<table><tr><td>Total times 500</td><td>Succ. times</td><td>Public obj. ratio (%)</td><td>Success rate (%)</td></tr><tr><td></td><td>500</td><td>60</td><td>100</td></tr><tr><td>500</td><td>500</td><td>55</td><td>100</td></tr><tr><td>500</td><td>500</td><td>50</td><td>100</td></tr><tr><td>500</td><td>499</td><td>44</td><td>99.8</td></tr><tr><td>500</td><td>467</td><td>38</td><td>93.4</td></tr><tr><td></td><td>406</td><td>33</td><td>81.2</td></tr></table>

However, the accuracy of 14.9% is still unsatisfactory. We found that the primary reason is that the observations of the two sets of keyframes are incomplete, thus resulting in inaccurate object modeling. To prove our hypothesis, we manually generate a scene with objects and divide it into a prior map and query map, assuming that prior and query keyframes generate them, respectively, and that the poses of the objects are obtained from the ground truth. As depicted in the figure in Table V, we adjust the proportion of shared objects across the two maps and measure the success rate of relocalization. As demonstrated in Table V, we obtain a 100% success rate with a public object ratio of over 50% and retain over 80% accuracy with a ratio of 33%. The result demonstrates the effectiveness of our proposed object descriptor and matching algorithm. It also illustrates its sensitivity to object pose and suggests that more accurate object modeling methods can improve its performance.

## G. Evaluation of Active Mapping

To validate the effectiveness of the active map building and the viability of robot manipulation led by the map, we conduct extensive evaluations in both simulation and real-world environments. The simulated robotic manipulation scene is set in Sapien [83], shown in Fig. 20, where the number of objects and the scene complexities vary in different scenes.

The accurate position estimate is critical for successful robotic manipulation operations, such as grasping, placing, arranging, and planning. However, precision is difficult to ensure when the robot estimates autonomously. To quantify the effect of active exploration on object pose estimation, like previous studies [74], [84], we compare our object-driven method with two typically used baseline strategies, i.e., randomized exploration (Random.) and coverage exploration (Cover.). As indicated in Fig. 20, for randomized exploration, the camera pose is randomly sampled from the reachable set relative to the manipulator, while for coverage exploration, a coverage trajectory based on boustrophedon decomposition [85] is leveraged to scan the scene. At the beginning of all the explorations, an initialization step (Init.), in which the camera is sequentially placed over the four desk corners from a top view, is applied to start the object mapping process. The simulator provides the ground truth of object position, orientation, and size. Correspondingly, the accuracy of pose estimation is evaluated by the center distance error (CDE, cm), the yaw angle error (YAE, degree), and the IoU (including 2-D IoU from the top view and 3-D IoU) between the ground truth and our estimated results.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/25f5834ac7625ab84fa347f8ea4a9398fe48ef5adc5e926170499e604c33dc51.jpg)

(a)  
![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/a91e727d9bc400327e911916ef854df28c0ac4a1c39064e3c80b60b43bad37d5.jpg)

(b)  
![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/3d31191d0e8126c1b8385dca1438b3febaa0c748648d837a7128e7dfdadf51d4.jpg)  
(c)  
Fig. 19. Quantitative analysis of scene matching. (a) Matching result in TUM RGB-D fr2\_desk sequence. (b) Matching result in RGB-D Scenes v2 scene\_01 sequence. (c) Matching result under different lighting conditions in the real world.

TABLE VI  
ACCURACY OF OBJECT POSE ESTIMATION
<table><tr><td>Scene</td><td>Metrics</td><td>Ours</td><td>Random.</td><td>Cover.</td><td>Init.</td></tr><tr><td rowspan="4">1</td><td>3-D IoU</td><td>0.427</td><td>0.3056</td><td>0.3329</td><td>0.3586</td></tr><tr><td>2-D IoU</td><td>0.6225</td><td>0.4571</td><td>0.5221</td><td>0.5212</td></tr><tr><td>CDE</td><td>1.5272</td><td>2.2699</td><td>1.7876</td><td>2.2022</td></tr><tr><td>YAE</td><td>3.5</td><td>4.8</td><td>3.8</td><td>2.8</td></tr><tr><td rowspan="4">2</td><td>3-D IoU</td><td>0.4307</td><td>0.3017</td><td>0.4224</td><td>0.3400</td></tr><tr><td>2-D IoU</td><td>0.8679</td><td>0.6422</td><td>0.7730</td><td>0.6480</td></tr><tr><td>CDE</td><td>1.4646</td><td>2.1096</td><td>1.5931</td><td>1.9822</td></tr><tr><td>YAE</td><td>2.4</td><td>1.8</td><td>2.7</td><td>2.4</td></tr><tr><td rowspan="4">3</td><td>3-D IoU</td><td>0.4132</td><td>0.3125</td><td>0.3685</td><td>0.2617</td></tr><tr><td>2-D IoU</td><td>0.6225</td><td>0.4915</td><td>0.5517</td><td>0.3909</td></tr><tr><td>CDE</td><td>1.5503</td><td>2.0672</td><td>1.4841</td><td>2.7489</td></tr><tr><td>YAE</td><td>3.9</td><td>3.7</td><td>3.8</td><td>4.9</td></tr><tr><td rowspan="4">4</td><td>3-D IoU</td><td>0.4790</td><td>0.3824</td><td>0.3664</td><td>0.3007</td></tr><tr><td>2-D IoU</td><td>0.6536</td><td>0.5886</td><td>0.4788</td><td>0.4869</td></tr><tr><td>CDE</td><td>1.3335</td><td>1.3514</td><td>1.7508</td><td>1.927</td></tr><tr><td>YAE</td><td>2.9</td><td>2.8</td><td>2.1</td><td>2.1</td></tr><tr><td rowspan="4">5</td><td>3-D IoU</td><td>0.5177</td><td>0.2696</td><td>0.2884</td><td>0.3720</td></tr><tr><td>2-D IoU</td><td>0.6263</td><td>0.4297</td><td>0.4326</td><td>0.6142</td></tr><tr><td>CDE</td><td>1.3704</td><td>2.5077</td><td>2.1753</td><td>2.0084</td></tr><tr><td>YAE</td><td>3.9</td><td>2.1</td><td>3.9</td><td>2.1</td></tr><tr><td rowspan="4">6</td><td>3-D IoU</td><td>0.4411</td><td>0.3000</td><td>0.3597</td><td>0.2916</td></tr><tr><td>2-D IoU</td><td>0.5437</td><td>0.4850</td><td>0.4783</td><td>0.5042</td></tr><tr><td>CDE</td><td>2.5998</td><td>3.4278</td><td>2.8411</td><td>3.4965</td></tr><tr><td>YAE</td><td>2.3</td><td>2.7</td><td>3</td><td>2.7</td></tr><tr><td rowspan="4">7</td><td>3-D IoU</td><td>0.4626</td><td>0.2118</td><td>0.4133</td><td>0.3153</td></tr><tr><td>2-D IoU</td><td>0.6017</td><td>0.3839</td><td>0.5569</td><td>0.4541</td></tr><tr><td>CDE</td><td>1.49928</td><td>2.3822</td><td>1.4832</td><td>2.0467</td></tr><tr><td>YAE</td><td>2.1</td><td>4.5</td><td>2.5</td><td>2.5</td></tr><tr><td rowspan="4">Mean</td><td>3-D IoU</td><td>0.453</td><td>0.2977</td><td>0.3645</td><td>0.3200</td></tr><tr><td>2-D IoU</td><td>0.6483</td><td>0.4969</td><td>0.5419</td><td>0.5171</td></tr><tr><td>CDE</td><td>1.6207</td><td>2.3022</td><td>1.8736</td><td>2.3446</td></tr><tr><td>YAE</td><td>3</td><td>3.2</td><td>3.1</td><td>2.8</td></tr></table>

Bold means better

Table VI shows the evaluation results in seven scenes (see Fig. 20). We can see our proposed object-driven exploration strategy achieves a 3-D IoU of 45.3%, which is 15.53%, 8.85%, and 13.3% higher than that of the randomized exploration, the coverage exploration, and the initialization, respectively. For 2-D IoU, our method achieves an accuracy of 64.83%, which is 15.14%, 10.64%, and 13.12% higher than baseline methods. In terms of CDE, our method reaches 1.62 cm, significantly less than other methods. For YAE, all exploration strategies achieve an error ofapproximately 3<sup>◦</sup>, which verifies the robustness ofour line alignment-based yaw angle optimization method. The level of abovementioned precision attained is sufficient for robotic manipulation [86]. Moreover, we also find that randomized exploration sometimes performs worse than the initialization result (rows 2, 5, and 7), which indicates that increasing observations do not necessarily result in more accurate pose estimation, and purposeful exploration is necessary.

The mapping results are shown in Fig. 20. The cubes and cylinders are used to model the objects, including poses and

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/8b99ca2f2234ecd3ffbac9e7117c56d05c11e9f0f207a63094f7dee96d2c72cb.jpg)  
Fig. 20. Comparison of mapping results. The first column in the subpicture: the scene image; the second column: the result of our object-driven exploration; the third column: the result of the coverage exploration; the fourth column: the result of the randomized exploration.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/b377a6aeafed2b88b5816d7f6cf03c3c5836a4fcaf9835fd1cdf72487a58a343.jpg)  
Fig. 21. Demonstration of grasping process. (a) Grasping process in the simulated environment. (b) Grasping process in the real world.

scales (analyzed previously), based on their semantic categories.   
The following characteristics are present.

1) The system can accurately model various objects as the number of objects increases, as shown in Fig. 20(a)–(e), demonstrating its robustness.

2) Among objects of various sizes, our method focuses more on large objects with lower observation completeness [see Fig. 20(f)].

3) When objects are distributed unevenly, our proposed strategy can swiftly concentrate the camera on object regions, thus avoiding unnecessary and time-consuming exploration [see Fig. 20(g)].

4) For scenes with objects close to each other, our method can focus more on regions with fewer occlusions [see Fig. 20(h)].

These behaviors verify the effectiveness of our exploration strategy. In addition, our method has a shorter exploration path yet produces a more precise object posture.

## H. Object Grasping and Placement

This experiment uses the incrementally generated object map to perform object grasping. Fig. 21(a) and (b) illustrate the grasping process in simulated and real-world environments, with the object map included. After extensive testing, we obtained a grasping success rate of approximately 86% in the simulator and 81% in the real world, which may be affected by environmental or manipulator noises. It is found that the center and direction of the objects have a significant influence on grasping performance. The proposed method performs well regarding these two metrics, thus ensuring high-quality grasping. Overall, our object SLAM-based pose estimation results can satisfy the requirements of grasping.

We argued that the proposed object map level perception outperforms object pose-only perception and provides information for more intelligent robotics decision-making tasks in addition to grasping. Such include avoiding collisions with other objects, updating the map after grasping, object arrangement and placement based on object properties, and object delivery requested by the user. We design the object placement experiments to verify the global perception capabilities introduced by object mapping. As shown in Fig. 22, the robot is required to manipulate the original scene [see Fig. 22(a)] to the target scene [see Fig. 22(c)] according to object sizes and classes encoded in the object map.

The global object map is shown in Fig. 22(b), which contains the semantic labels, size, and pose of the objects. The two little blocks are picked up and placed in the large cup [see Fig. 22(d)], while the cups are ordered by volume [see Fig. 22(e)] and the bottles by height [see Fig. 22(f)]. This task is challenging for the conventional grasping approach since lacking global perception, such as object’s height on the map, its surroundings, and could interact with which objects.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/e39fc8fe8ebb3191308e21d63fa96fed65a65d1920cfcc8a01df3e66faff5ca5.jpg)  
Fig. 22. Object placement according to the global object map.

TABLE VII  
TIME ANALYSIS OF DATA ASSOCIATION AND OBJECT PARAMETERIZATION(MS/FRAME)
<table><tr><td colspan="2">Data association</td><td colspan="2">Object parameterization</td></tr><tr><td>Distribution-based</td><td>IoU-based</td><td>iForest</td><td>Line alignment</td></tr><tr><td>4.92</td><td>7.83</td><td>3.86</td><td>2.71</td></tr></table>

## IX. DISCUSSION AND ANALYZE

In this section, we analyze the limitations and implementation details of our method and provide potential solutions for the object SLAM community.

1) Data association: Experiments revealed that the two primary reasons for the failure of data association could be summed up as follows: a) Long-tailed distribution. In some cases, object centroids are located in the tail of the distribution, which violates the Gaussian distribution assumption and causes the association to fail. Although our multiple association strategy alleviates this to some extent. b) Detected semantic label mistakes. Even if the IoU-based or distribution-based method determines an association between two objects, the association will fail if the labels are inconsistent. Error in label recognition is one of the most common issues with detectors. More generalized, accurate detectors or fine-turning on specific datasets are potential alternatives.

The running time of data association is shown in the first two columns of Table VII. Distribution-based methods include NP and t-tests, while IoU-based methods include M-IoU and P-IoU, where projecting 3-D points to 2-D images takes most of the time. Note that the total duration of data association is less than the sum in the table, approximately 8 ms per frame, because sometimes some strategies can be skipped. We perform data association on every frame, which would be more time-efficient if simply performed on keyframes, as CubeSLAM does.

TABLE VIII  
TIME ANALYSIS OF AUGMENTED REALITY
<table><tr><td>Module</td><td>Time (ms/frame)</td></tr><tr><td>LSM</td><td>81.84</td></tr><tr><td>ROS data transfer</td><td>10.96</td></tr><tr><td>Virtural-real rendering</td><td>25.00</td></tr></table>

2) Object parameterization: Two factors typically cause failure situations of object pose estimation as follows. a) Object surfaces lack texture, or objects are only partially seen due to occlusion or camera viewpoint. In this case, few object point clouds are collected, significantly reducing the pose estimation performance. b) Too many outliers lead the object to be estimated too large, or the modeled object is extremely small since the iForest algorithm falls into a local optimum. In the alternatives, the 3-D detector [87] based on the complete point cloud may not be optimal due to the incremental characteristic of SLAM; image-based 6-DOF pose estimation [19], [88] is limited by the scale of the training data, resulting in poor generalization [51]. Conversely, incremental detection/segmentation [89] and joint point cloud-image multimodal RGB-D 3-D object detection [90], [91] are potentially feasible.

The runtime of object parameterization is shown in the last two columns in Table VII, which takes around 6.5 ms per frame on average. The full SLAM system (for camera tracking and semantic mapping) runs at about 10 fps.

3) Augmented reality: Augmented reality performance depends on object modeling, camera localization accuracy, and the rendering effect. Here we provide detailed engineering implementations for SLAM developers to migrate their algorithms to augmented reality applications. The AR system comprises three modules: a) The localization and semantic mapping (LSM) modules provide camera position, point cloud, and object parameters. Sections IV and V introduce the techniques. b) ROS [92] data transfer module: send images captured by the camera to the LSM module and then publish the estimated camera pose and map elements. c) Virtual–real rendering module: Use the Unity3D engine to subscribe to topics published by ROS, construct a virtual 3-D scene, and render it to a 2-D image plane. Table VIII details the duration ofeach module, which is executed in parallel.

4) Scene matching: The principal causes for the failure of scene matching and relocalization are as follows. a) There are few common objects between the two maps, resulting in a significant difference in the descriptors of the same object in the two maps, leading to matching fails. b) The parallax of the trajectories of the two maps is excessively large, and the observation is insufficient, which affects the accuracy of object modeling and the construction of descriptors. Regarding the first issue, other nonobject-level landmarks, such as planes and structural components, can be considered for descriptor construction. For the second challenge, more accurate object modeling techniques can improve the performance of matching and relocalization, as demonstrated by our experiments.

5) Object grasping: There are two limitations to the object grasping task as follows. a) Textured objects and tabletops are required for point-based SLAM tracking to succeed. b)

Objects are all regular cube and cylinder shapes in our experiments. Complex irregular objects may necessitate more detailed shape reconstruction and grasp point detection. Nonetheless, we demonstrate the potential of object SLAM for grasping tasks without object priors. Model-free and unseen object grasping will be the future trend. In terms of running time, the speed is even faster than 10fps because, in this setting, the data association is more straightforward, and more time is spent on the active mapping analysis process.

## X. CONCLUSION

We presented an object mapping framework that aims to create an object-oriented map using general models that parameterize the object’s position, orientation, and size. First, we investigated related fundamental techniques for object mapping, including multiview data association and object pose estimation. We then center on the object map and validate its potential in multiple high-level tasks, such as augmented reality, scene matching, and object grasping. Finally, we analyzed the limitations and failure instances of our method and gave possible alternatives to inspire the development of related fields. The following points will be given significant consideration in future work. 1) Dynamic objects data association, tracking, and trajectory prediction. 2) Irregular and unseen object modeling and tightly coupled optimization with SLAM. 3) Object-level relocalization and loop closure. 4) Omnidirectional perception with multisensor and multiple semantic networks to realize spatial AI.

## REFERENCES

[1] A. J. Davison, “FutureMapping: The computational structure of spatial AI systems,” 2018, arXiv:1803.11288.

[2] Q. Wang, Z. Yan, J. Wang, F. Xue, W. Ma, and H. Zha, “Line flow based simultaneous localization and mapping,” IEEE Trans. Robot., vol. 37, no. 5, pp. 1416–1432, Oct. 2021.

[3] Y. Zhou, H. Li, and L. Kneip, “Canny-VO: Visual odometry with RGB-D cameras based on geometric 3-D–2-D edge alignment,” IEEE Trans. Robot., vol. 35, no. 1, pp. 184–199, Feb. 2019.

[4] R. Yunus, Y. Li, and F. Tombari, “ManhattanSLAM: Robust planar tracking and mapping leveraging mixture of manhattan frames,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 6687–6693.

[5] S. Zhao, P. Wang, H. Zhang, Z. Fang, and S. Scherer, “TP-TIO: A robust thermal-inertial odometry with deep thermalpoint,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 4505–4512.

[6] S. Cao, X. Lu, and S. Shen, “GVINS: Tightly coupled GNSS–visualinertial fusion for smooth and consistent state estimation,” IEEE Trans. Robot., vol. 38, no. 4, pp. 2004–2021, Aug. 2022.

[7] T.-M. Nguyen, S. Yuan, M. Cao, T. H. Nguyen, and L. Xie, “Viral SLAM: Tightly coupled Camera-IMU-UWB-lidar SLAM,” 2021, arXiv:2105.03296.

[8] J. McCormac, A. Handa, A. Davison, and S. Leutenegger, “SemanticFusion: Dense 3D semantic mapping with convolutional neural networks,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 4628–4635.

[9] S. Yang, Y. Huang, and S. Scherer, “Semantic 3D occupancy mapping through efficient high order CRFs,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2017, pp. 590–597.

[10] A. Rosinol, M. Abate, Y. Chang, and L. Carlone, “Kimera: An open-source library for real-time metric-semantic localization and mapping,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 1689–1696.

[11] D. Frost, V. Prisacariu, and D. Murray, “Recovering stable scale in monocular SLAM using object-supplemented bundle adjustment,” IEEE Trans. Robot., vol. 34, no. 3, pp. 736–747, Jun. 2018.

[12] A. Iqbal and N. R. Gans, “Localization of classified objects in SLAM using nonparametric statistics and clustering,” in Proc. IEEE/RSJInt. Conf. Intell. Robots Syst., 2018, pp. 161–168.

[13] B. Mu, S.-Y. Liu, L. Paull, J. Leonard, and J. P. How, “SLAM with objects using a nonparametric pose graph,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2016, pp. 4602–4609.

[14] N. Sünderhauf, T. T. Pham, Y. Latif, M. Milford, and I. Reid, “Meaningful maps with object-oriented semantic mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2017, pp. 5079–5085.

[15] M. Grinvald et al., “Volumetric instance-aware semantic mapping and 3D object discovery,” IEEE Robot. Automat. Lett., vol. 4, no. 3, pp. 3037–3044, Jul. 2019.

[16] A. Sharma, W. Dong, and M. Kaess, “Compositional and scalable object SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 11626–11632.

[17] R. F. Salas-Moreno, R. A. Newcombe, H. Strasdat, P. H. Kelly, and A. J. Davison, “SLAM++: Simultaneous localisation and mapping at the level of objects,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2013, pp. 1352–1359.

[18] S. Choudhary et al., “Multi robot object-based SLAM,” in Proc. Int. Symp. Exp. Robot., 2016, pp. 729–741.

[19] Y. Labbé, J. Carpentier, M. Aubry, and J. Sivic, “Cosypose: Consistent multi-view multi-object 6D pose estimation,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 574–591.

[20] S. L. Bowman, N. Atanasov, K. Daniilidis, and G. J. Pappas, “Probabilistic data association for semantic SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 1722–1729.

[21] P. Parkhiya, R. Khawad, J. K. Murthy, B. Bhowmick, and K. M. Krishna, “Constructing category-specific models for monocular object-SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2018, pp. 4517–4524.

[22] N. Joshi, Y. Sharma, P. Parkhiya, R. Khawad, K. M. Krishna, and B. Bhowmick, “Integrating objects into monocular SLAM: Line based category specific models,” in Proc. 11th Indian Conf. Comput. Vis., Graph. Image Process., 2018, pp. 1–9.

[23] E. Sucar, K. Wada, and A. Davison, “NodeSLAM: Neural object descriptors for multi-view shape reconstruction,” in Proc. Int. Conf. 3D Vis., 2020, pp. 949–958.

[24] S. Yang and S. Scherer, “CubeSLAM: Monocular 3-D object SLAM,” IEEE Trans. Robot., vol. 35, no. 4, pp. 925–938, Aug. 2019.

[25] L. Nicholson, M. Milford, and N. Sünderhauf, “QuadricSLAM: Dual quadrics from object detections as landmarks in object-oriented SLAM,” IEEE Robot. Automat. Lett., vol. 4, no. 1, pp. 1–8, Jan. 2019.

[26] Y. Wu, Y. Zhang, D. Zhu, Y. Feng, S. Coleman, and D. Kerr, “Eao-SLAM: Monocular semi-dense object SLAM based on ensemble data association,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 4966–4973.

[27] Y. Wu et al., “Object SLAM-based active mapping and robotic grasping,” in Proc. Int. Conf. 3D Vis., 2021, pp. 1372–1381.

[28] K. Chen, J. Liu, Q. Chen, Z. Wang, and J. Zhang, “Accurate object association and pose updating for semantic SLAM,” IEEE Trans. Intell. Transp. Syst., vol. 23, no. 12, pp. 25169–25179, Dec. 2022.

[29] Y. Zhang, C. Wang, X. Wang, W. Zeng, and W. Liu, “FairMOT: On the fairness of detection and re-identification in multiple object tracking,” Int. J. Comput. Vis., vol. 129, no. 11, pp. 3069–3087, 2021.

[30] J. Li, D. Meger, and G. Dudek, “Semantic mapping for view-invariant relocalization,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 7108–7115.

[31] J. McCormac, R. Clark, M. Bloesch, A. Davison, and S. Leutenegger, “Fusion++: Volumetric object-level SLAM,” in Proc. Int. Conf. 3D Vis., 2018, pp. 32–41.

[32] J. Wang, M. Rünz, and L. Agapito, “DSP-SLAM: Object oriented SLAM with deep shape priors,” in Proc. Int. Conf. 3D Vis., 2021, pp. 1362–1371.

[33] B. Xu, A. J. Davison, and S. Leutenegger, “Learning to complete object shapes for object-level mapping in dynamic scenes,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2022, pp. 2257–2264.

[34] Y. Liu, Y. Petillot, D. Lane, and S. Wang, “Global localization with objectlevel semantics and topology,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 4909–4915.

[35] K. Ok, K. Liu, K. Frey, J. P. How, and N. Roy, “Robust object-based SLAM for high-speed autonomous navigation,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 669–675.

[36] Y. Xiang and D. Fox, “DA-RNN: Semantic mapping with data associated recurrent neural networks,” in Proc. Robot.: Sci. Syst., Cambridge, Massachusetts, Jul. 2017. [Online]. Available: https://www. roboticsproceedings.org/rss13/p13.html

[37] K. Li et al., “ODAM: Object detection, association, and mapping using posed RGB video,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 5978–5988.

[38] N. Merrill et al., “Symmetry and uncertainty-aware object SLAM for 6DoF object pose estimation,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 14881–14890.

[39] C. Xing et al., “Descriptellation: Deep learned constellation descriptors for SLAM,” 2022, arXiv:2203.00567.

[40] M. Strecke and J. Stuckler, “EM-Fusion: Dynamic object-level SLAM with probabilistic data association,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2019, pp. 5864–5873.

[41] S. Yang, Z.-F. Kuang, Y.-P. Cao, Y.-K. Lai, and S.-M. Hu, “Probabilistic projective association and semantic guided relocalization for dense reconstruction,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 7130–7136.

[42] J. Zhang, M. Gui, Q. Wang, R. Liu, J. Xu, and S. Chen, “Hierarchical topic model based object association for semantic SLAM,” IEEE Trans. Visual. Comput. Graph., vol. 25, no. 11, pp. 3052–3062, Nov. 2019.

[43] T. Ran, L. Yuan, J. Zhang, L. He, R. Huang, and J. Mei, “Not only look but infer: Multiple hypothesis clustering of data association inference for semantic SLAM,” IEEE Trans. Instrum. Meas., vol. 70, 2021, Art. no. 3515409.

[44] J. J. Park, P. Florence, J. Straub, R. Newcombe, and S. Lovegrove, “DeepSDF: Learning continuous signed distance functions for shape representation,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 165–174.

[45] M. Han et al., “Reconstructing interactive 3D scenes by panoptic mapping and cad model alignments,” in Proc. IEEE Int. Conf. Robot. Automat., 2021, pp. 12199–12206.

[46] Z. Cao et al., “Object-aware SLAM based on efficient quadric initialization and joint data association,” IEEE Robot. Automat. Lett., vol. 7, no. 4, pp. 9802–9809, Oct. 2022.

[47] M. Runz, M. Buffier, and L. Agapito, “MaskFusion: Real-time recognition, tracking and reconstruction of multiple moving objects,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2018, pp. 10–20.

[48] S. Lin, J. Wang, M. Xu, H. Zhao, and Z. Chen, “Topology aware objectlevel semantic mapping towards more robust loop closure,” IEEE Robot. Automat. Lett., vol. 6, no. 4, pp. 7041–7048, Oct. 2021.

[49] J. Lu, B. Tian, H. Shen, and X. Zhang, “Real-time instance-aware segmentation and semantic mapping on edge devices,” IEEE Trans. Instrum. Meas., vol. 72, 2023, Art. no. 7501109.

[50] J. Li, K. Koreitem, D. Meger, and G. Dudek, “View-invariant loop closure with oriented semantic landmarks,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 7943–7949.

[51] Y. Ming, X. Yang, and A. Calway, “Object-augmented RGB-D SLAM for wide-disparity relocalisation,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2021, pp. 2203–2209.

[52] R. Mur-Artal and J. D. Tardós, “ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1255–1262, Oct. 2017.

[53] P. Schmuck and M. Chli, “CCM-SLAM: Robust and efficient centralized collaborative monocular simultaneous localization and mapping for robotic teams,” J. Field Robot., vol. 36, no. 4, pp. 763–781, 2019.

[54] A. Gawel, C. Del Don, R. Siegwart, J. Nieto, and C. Cadena, “X-view: Graph-based semantic multi-view localization,” IEEE Robot. Automat. Lett., vol. 3, no. 3, pp. 1687–1694, Jul. 2018.

[55] X. Guo, J. Hu, J. Chen, F. Deng, and T. L. Lam, “Semantic histogram based graph matching for real-time multi-robot global localization in large scale environment,” IEEE Robot. Automat. Lett., vol. 6, no. 4, pp. 8349–8356, Oct. 2021.

[56] C. Qin, Y. Zhang, Y. Liu, and G. Lv, “Semantic loop closure detection based on graph matching in multi-objects scenes,” J. Vis. Commun. Image Representation, vol. 76, 2021, Art. no. 103072.

[57] Z. Zhang and D. Scaramuzza, “Beyond point clouds: Fisher information field for active visual localization,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 5986–5992.

[58] Z. Zeng, A. Röfer, and O. C. Jenkins, “Semantic linking maps for active visual object search,” in Proc. IEEE Int. Conf. Robot. Automat., 2020, pp. 1984–1990.

[59] B. Charrow et al., “Information-theoretic planning with trajectory optimization for dense 3D mapping,” in Proc. Conf. Robot.: Sci. Syst., 2015, vol. 11, pp. 3–12.

[60] C. Wang, D. Zhu, T. Li, M. Q. Meng, and C. W. de Silva, “Efficient autonomous robotic exploration with semantic road map in indoor environments,” IEEE Robot. Automat. Lett., vol. 4, no. 3, pp. 2989–2996, Jul. 2019.

[61] S. Kriegel, C. Rink, T. Bodenmüller, and M. Suppa, “Efficient next-bestscan planning for autonomous 3D surface reconstruction of unknown objects,” J. Real-Time Image Process., vol. 10, no. 4, pp. 611–631, 2015.

[62] K. Wada, E. Sucar, S. James, D. Lenton, and A. J. Davison, “Morefusion: Multi-object reasoning for 6D pose estimation from volumetric fusion,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2020, pp. 14528–14537.

[63] D. Almeida, E. Ataer-Cansizoglu, and R. Corcodel, “Detection, tracking and 3D modeling of objects with sparse RGB-D SLAM and interactive perception,” in Proc. IEEE-RAS 19th Int. Conf. Humanoid Robots, 2019, pp. 1–8.

[64] J. Redmon and A. Farhadi, “YOLOv3: An incremental improvement,” 2018, arXiv:1804.02767.

[65] F. Wilcoxon, “Individual comparisons by ranking methods,” in Breakthroughs in Statistics. Berlin, Germany: Springer, 1992, pp. 196–202.

[66] S. Sidney, “Nonparametric statistics for the behavioral sciences,” J. Nervous Ment. Dis., vol. 125, no. 3, 1957, Art. no. 497.

[67] E. L. Lehmann and H. J. D’Abrera, Nonparametrics: Statistical Methods Based on Ranks. San Francisco, CA, USA: Holden-Day, 1975.

[68] S. Yang and S. Scherer, “Monocular object and plane SLAM in structured environments,” IEEE Robot. Automat. Lett., vol. 4, no. 4, pp. 3145–3152, Oct. 2019.

[69] T. Pire, J. Corti, and G. Grinblat, “Online object detection and localization on stereo visual SLAM system,” J. Intell. Robot. Syst., vol. 98, no. 2, pp. 377–386, 2020.

[70] F. T. Liu, K. M. Ting, and Z.-H. Zhou, “Isolation-based anomaly detection,” ACM Trans. Knowl. Discov. From Data, vol. 6, no. 1, pp. 1–39, 2012.

[71] R. G. Von Gioi, J. Jakubowicz, J.-M. Morel, and G. Randall, “LSD: A line segment detector,” Image Process. On Line, vol. 2, pp. 35–55, 2012.

[72] S. A. Cook, “The complexity of theorem-proving procedures,” in Proc. 3rd Annu. ACM Symp. Theory Comput., 1971, pp. 151–158.

[73] G. Kahn et al., “Active exploration using trajectory optimization for robotic grasping in the presence of occlusions,” in Proc. IEEE Int. Conf. Robot. Automat., 2015, pp. 4783–4790.

[74] E. Arruda, J. Wyatt, and M. Kopicki, “Active vision for dexterous grasping of novel objects,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2016, pp. 2881–2888.

[75] A. Elfes, “Using occupancy grids for mobile robot perception and navigation,” Computer, vol. 22, no. 6, pp. 46–57, 1989.

[76] C. E. Shannon, “A mathematical theory of communication,” Bell Syst. Tech. J., vol. 27, no. 3, pp. 379–423, 1948.

[77] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2012, pp. 573–580.

[78] A. Iqbal and N. R. Gans, “Data association and localization of classified objects in visual SLAM,” J. Intell. Robotic Syst., vol. 100, pp. 113–130, 2020.

[79] J. Shotton, B. Glocker, C. Zach, S. Izadi, A. Criminisi, and A. Fitzgibbon, “Scene coordinate regression forests for camera relocalization in RGB-D images,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2013, pp. 2930–2937.

[80] K. Lai, L. Bo, and D. Fox, “Unsupervised feature learning for 3D scene labeling,” in Proc. IEEE Int. Conf. Robot. Automat., 2014, pp. 3050–3057.

[81] S. He, X. Qin, Z. Zhang, and M. Jagersand, “Incremental 3D line segment extraction from semi-dense SLAM,” in Proc. 24th Int. Conf. Pattern Recognit., 2018, pp. 1658–1663.

[82] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. Montiel, and J. D. Tardós, “ORB-SLAM3: An accurate open-source library for visual, visual–inertial, and multimap SLAM,” IEEE Trans. Robot., vol. 37, no. 6, pp. 1874–1890, Dec. 2021.

[83] F. Xiang et al., “SAPIEN: A simulated part-based interactive environment,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2020, pp. 11094–11104.

[84] D. Morrison, P. Corke, and J. Leitner, “Multi-view picking: Next-bestview reaching for improved grasping in clutter,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 8762–8768.

[85] D. Kaljaca, B. Vroegindeweij, and E. van Henten, “Coverage trajectory planning for a bush trimming robot arm,” J. Field Robot., vol. 37, no. 2, pp. 283–308, 2020.

[86] C. Wang et al., “DenseFusion: 6D object pose estimation by iterative dense fusion,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 3338–3347.

[87] Z. Liu, Z. Zhang, Y. Cao, H. Hu, and X. Tong, “Group-free 3D object detection via transformers,” in Proc. IEEE/CVF Int. Conf. Comput. Vis., 2021, pp. 2929–2938.

[88] H. Wang, S. Sridhar, J. Huang, J. Valentin, S. Song, and L. J. Guibas, “Normalized object coordinate space for category-level 6D object pose and size estimation,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2019, pp. 2637–2646.

[89] S.-C. Wu, J. Wald, K. Tateno, N. Navab, and F. Tombari, “ScenegraphFusion: Incremental 3D scene graph prediction from RGB-D sequences,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2021, pp. 7511– 7521.

[90] Y. Wang, X. Chen, L. Cao, W. Huang, F. Sun, and Y. Wang, “Multimodal token fusion for vision transformers,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., 2022, pp. 12176–12185.

[91] H. Yang, C. Shi, Y. Chen, and L. Wang, “Boosting 3D object detection via object-focused image fusion,” 2022, arXiv:2207.10589.

[92] M. Quigley et al., “ROS: An open-source robot operating system,” in Proc. ICRA Workshop Open Source Softw., 2009, Art. no. 5.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/baa37270445bd9e5afb98af4a618cf39d628f1609fa2bf64fea726e646d54d6b.jpg)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/700504624567ed9cf2c7c78ded55510b4c23b9f4fea4fefe0b724d175010c557.jpg)

Yanmin Wu (Student Member, IEEE) received the B.S. degree in electronic information engineering from Shenyang Normal University, Shenyang, China, in 2018, and the M.S. degree in robot science and engineering from Northeastern University, Shenyang, China, in 2021. He is currently working toward the Ph.D. degree in computer applied technology with the School of Electronic and Computer Engineering, Peking University Shenzhen Graduate School, Shenzhen, China.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/bc0b37d392d7322824164e43345e34f2ec539e33fd7b03e1ac4685dcb1e5ab13.jpg)

Zhiqiang Deng received the B.S. degree in automation from Shenyang Jianzhu University, Shenyang, China, in 2020. He is currently working toward the master’s degree in pattern recognition and intelligent systems from the College of Information Science and Engineering, Northeastern University, Shenyang, China.

His research interests include visual simultaneous localization and mapping and augmented reality.

His research interests include visual SLAM, 3-D reconstruction, and scene understanding.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/7ff40b1af3389b4117263cd9ba6bc2028b94b34178a31f80da5f2f625962fe2a.jpg)

Yunzhou Zhang (Member, IEEE) received the B.S. and M.S. degrees in mechanical and electronic engineering from the National University of Defense Technology, Changsha, China, in 1997 and 2000, respectively, and the Ph.D. degree in pattern recognition and intelligent system from Northeastern University, Shenyang, China, in 2009.

He is currently a Professor with the Faculty of Robot Science and Engineering, Northeastern University. He leads the Cloud Robotics and Visual Perception Research Group. His research has been

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/c06bacc42530be866ac056e3648f23472da904f04db9627157f026875bfbf0ef.jpg)

His research interests include semantic simultaneous localization and mapping and augmented reality.

supported by funding from various sources, such as the National Natural Science Foundation of China, the Ministry of Education of China, and some famous high-tech companies. He has authored or coauthored many journal articles and conference papers on intelligent robots, computer vision, and wireless sensor networks. His research interests include intelligent robots, computer vision, and sensor networks.

Xin Chen received the B.S. degree in mechanical and electronic engineering from the Harbin University of Science and Technology, Harbin, China, in 2019, and the M.S. degree in robot science and engineering from Northeastern University, Shenyang, China, in 2022.

Wenkai Sun received the B.S. degree in automation from Tiangong University, Tianjin, China, in 2020. He is currently working toward the master’s degree from the College of Information Science and Engineering, Northeastern University, Shenyang, China.

His research interests include object pose estimation and robot manipulation.

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/fb048e6a9bc499c4b4865402968e03be7a3869f64e866bf3b0df949bd9238b97.jpg)

![](images/2023_An_Object_SLAM_Framework_for_Association__Mapping__and_H/a7587dcc4d6d3a8e648e0eb7ee2459323a2e7dd20ba647ac03b2f09a9028b258.jpg)

Jian Zhang (Member, IEEE) received the B.S. degree in mathematics and applied mathematics from the Department of Mathematics, Harbin Institute of Technology (HIT), Harbin, China, in 2007, and the M.Eng. degree in computer science and technology and Ph.D. degree in computer applied technology from the School of Computer Science and Technology, HIT, in 2009 and 2014, respectively.

Delong Zhu received the B.S. degree in computer science and technology from Northeastern University, Shenyang, Liaoning, China, in 2015, and the Ph.D. degree in electronic engineering from the Department of Electronic Engineering, The Chinese University of Hong Kong, Hong Kong, in 2020.

He was a Visiting Scholar for nine months with Robotics Institute, Carnegie Mellon University, Pittsburgh, PA, USA. His research interests include motion planning in dynamic environments and deep reinforcement learning.

He is currently an Assistant Professor with the School of Electronic and Computer Engineering, Peking University Shenzhen Graduate School, Shenzhen, China. From 2014 to 2018, he was a Postdoctoral Researcher with Peking University, Hong Kong University of Science and Technology, and King Abdullah University of Science and Technology. He has authored or coauthored more than 90 technical articles in refereed international journals and proceedings. His research interests include intelligent multimedia processing, deep learning, and optimization.

Dr. Zhang was a recipient of the Best Paper Award at the 2011 IEEE Visual Communications and Image Processing (VCIP) and was a co-recipient of the Best Paper Award of 2018 IEEE MultiMedia.