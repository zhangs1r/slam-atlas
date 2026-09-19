# The Revisiting Problem in Simultaneous Localization and Mapping: A Survey on Visual Loop Closure Detection

Konstantinos A. Tsintotas , Senior Member, IEEE, Loukas Bampis ,

and Antonios Gasteratos , Senior Member, IEEE

Abstract— Where am I? This is one of the most critical questions that any intelligent system should answer to decide whether it navigates to a previously visited area. This problem has long been acknowledged for its challenging nature in simultaneous localization and mapping (SLAM), wherein the robot needs to correctly associate the incoming sensory data to the database allowing consistent map generation. The significant advances in computer vision achieved over the last 20 years, the increased computational power, and the growing demand for long-term exploration contributed to efficiently performing such a complex task with inexpensive perception sensors. In this article, visual loop closure detection, which formulates a solution based solely on appearance input data, is surveyed. We start by briefly introducing place recognition and SLAM concepts in robotics. Then, we describe a loop closure detection system’s structure, covering an extensive collection of topics, including the feature extraction, the environment representation, the decision-making step, and the evaluation process. We conclude by discussing open and new research challenges, particularly concerning the robustness in dynamic environments, the computational complexity, and scalability in long-term operations. The article aims to serve as a tutorial and a position paper for newcomers to visual loop closure detection.

Index Terms— Loop closure detection, mapping, SLAM, visualbased navigation.

## I. INTRODUCTION

OOP closure detection, which has long been acknowlneous localization and mapping (SLAM) system, historically represents a relevant and challenging task for the robotic community. Originally being introduced as “the revisiting problem,” it concerns the robot’s ability to recognize whether the sensory data just captured matches with any already collected, i.e., a previously visited area, aiming for SLAM to revise its position [1]. As the accumulated dead-reckoning errors in the map may persistently grow when global positioning information is not available, loop closure detection is essential for autonomous navigation, mainly when operating in largely closed route scenarios. An important aspect is that loops inherently occur sparsely. Therefore, if the current sensor information does not match any previously visited location, the new observation is added to the robot’s internal map, keeping only the constraints that relate the current with the penultimate pose. Since an erroneous loop closure detection might turn out to be fatal for any SLAM framework, a reliable pipeline should detect a small number or preferably zero false-positives while still avoiding false-negatives. The former refers to situations where the robot erroneously asserts a closed loop. The latter occurs when an event has been missed due to the system’s misinterpretation. Hence, “closing the loop” is a decision-making problem of paramount importance for consistent map generation of unknown environments (Fig. 1).

![](images/2022_LoopClosure-Survey/9950c848fbc76427a021a7cbfcbb2b58bec4dc3fd0005e1d33d06891fb9cb6d0.jpg)  
Fig. 1. A representative example of a pose graph constructed under a visual loop closure detection mechanism: map as generated initially (left) and subsequently a loop closure detection (right) in the New College dataset. [2]. The trajectory is drawn in green, while the chosen image pair is depicted in blue. When loops are detected, the system’s accumulated drift error and uncertainty regarding the estimated position and orientation (pose) are bounded, allowing consistent map generation. Lastly, the robot’s map is rectified extensively at both sides of the loop event (with permission from [3]).

Due to the above, the popularity of loop closure detection in the last 30 years is not surprising considering the notable SLAM evolution. Similar surveys can be found in the related literature, although they focus more on mapping techniques [4]–[6] or place recognition, relevant to robotics and other research areas, including computer vision, viz., Lowry et al. [7], Zhang et al. [8], Masone and Caputo [9], and Garg et al. [10]. This article gives a broad overview of the loop closure pipelines in the last three decades, offering the perspective of how such a framework is structured into a SLAM system. We mainly go through a historical review of the problem, focusing on how it was addressed during the years reaching the current age.

## A. Foundation of Loop Closure Detection

In the early years, several kinds of methods were exploited to map a robot’s environment, such as measuring bearings revolutions and range finders; however, advances were limited by the computational resources and sensor capabilities available at the time. During the last two decades, researchers have been able to access to an enviable array of sensing devices, including massively produced multi-megapixel digital cameras and computers that are more potent in processing power and storage [11]. Images, which effectively capture the environment’s appearance with high distinctiveness, are obtained through devices ranging from low-cost web cameras to high-end industrial ones. Not surprisingly, since modern robot navigation systems push towards effectiveness and efficiency, SLAM frameworks adopted such sensors and computational advances. Moreover, due to their reduced size and handiness, they can be easily attached to mobile platforms and allow the development of numerous localization and mapping pipelines with applications in different fields, such as autonomous cars [12], small aircrafts [13], and commercial devices [14].

Like any other computer vision task, visual loop closure detection firstly extracts distinct features from images; the similarities are then calculated, and finally, confidence metrics are determined. However, vital differences exist among image classification, image retrieval, and visual loop closure detection. More specifically, the first deals with categorizing a query image into a class from a finite number of available ones. Nevertheless, aiming to solve the nearest neighbor searching problem, image retrieval and visual loop closure detection systems face similar challenges as they try to find whether the current image matches any from the past. Due to this fact, an image retrieval module serves as the first part of any visual loop closure detection framework, in most cases. However, the underlying goals differ between these two areas regarding the sets upon which they operate. In particular, the latter searches for images depicting the exact same area that the robot is observing, operating only on the previously recorded image set. In contrast, image retrieval operates on an extensive database comprising not necessarily related images. This essentially means the typical goal is to retrieve instances of similar objects or entities, which may be different than the original in the query entry. For instance, a successful image retrieval could seek for buildings when the frame of a building is queried, or winter instances when the frame of a snowy road is used on a cross-season dataset. Hence, a considerable interest in the community’s effort has been directed towards robust image processing techniques since sensory data representations, though appropriate for image retrieval, may not perform effectively in visual loop closure detection and vice versa.

Rather than working directly with image pixels, feature extraction techniques derive discriminative information from the recorded camera frames [15]. Hand-crafted descriptors, both global (based on the entire image) and local (based on a region-of-interest), were widely used as feature extractors. However, due to their invariant properties over viewpoint changes, local features were often selected for loop closure detection pipelines [16]. Deep learning has revolutionized many research areas [17], [18], with convolutional neural networks (CNNs) being used for various classification tasks as they can inherently learn high-level visual features. As expected, the robotics community exploited their capabilities in visual loop closure detection, especially in situations of extreme environmental changes [19]. Nevertheless, their extensive computational requirements limit their applicability in real-time applications and often induce the utilization of power-demanding general-purpose graphical processing units (GPGPUs) [20].

Through the extracted features, the robot’s traversed path is described by a database of visual representations. To gain confidence about its position in the map and decide whether a loop occurs, the robot needs to compute a similarity score between the query and any previously seen observation. Several techniques exist for comparing images, ranging from pixel-wise comparisons to more complex ones based on feature correspondences. Then, a similarity threshold determines if a location can be considered as loop closure or should be declined, while additional steps, such as consistency checks based on multi-view geometry [21], can verify the matching pair. However, each of the aforementioned steps has to operate under real-time constraints during the robot’s mission. With an increasing demand for autonomous systems in a broad spectrum of applications, e.g., search and rescue [22], [23], space [24], [25] and underwater explorations [26], [27], the robots need to operate precisely for an extended period. As their complexity is at least linear to the traversed path, this limitation constitutes a crucial factor, severely affecting their capability to perform life-long missions.

In recent years, visual loop closure detection algorithms have matured enough to support continually enlarging operational environments. Thus, the research focus has shifted from recognizing scenes without notable appearance changes towards more complicated and more realistic changing situations. In such cases, detections need to be successful despite the variations in the images’ content, e.g., varying illumination (daytime against night) or seasonal conditions (winter against summer). Regardless of the advancements that have been achieved, the development of systems, which are condition invariant to such changes, remains an open research field. Finally, the growing interest of the robotics community is evidenced by the number of dedicated visual loop closure detection pipelines, as depicted in Fig. 2.

As we approach the third decade of visual loop closure detection, we need to acknowledge the groundwork laid out so far and build upon the following achievements

1) Robust performance: visual loop closure detection can operate with a high recall rate in a broad set of environments and viewpoint changes (i.e., different robot’s orientations), especially when a location is revisited by a vehicle in the same direction as previously.

![](images/2022_LoopClosure-Survey/2de56092e1a8bc82deef0f64a2b5782ff82476e5208dc357134f6864878fe017.jpg)  
Fig. 2. An illustrative evolution histogram of loop closure detection pipelines, which is based on the methods cited in this article specifically addressing the visual loop closure detection task. Starting from the first approach [28] in 2006, the growth of appearance-based systems for indicating previously visited areas shows that they remain a growing research field. The peak observed in 2014 is highly related to the increased growth of visual place recognition in computer vision, along with the introduction of deep learning techniques.

2) High-level understanding: visual loop closure detection can extend beyond basic hand-crafted methods to get a high-level understanding and semantics of the viewing scene.

3) Data management: visual loop closure detection can choose useful perceptual information and filters out irrelevant sensor data to address different tasks. Moreover, it supports the creation of adaptive environment representations, whose complexity varies due to the task at hand.

## B. Paper Organization

Despite its unique traits, visual loop closure detection is a task inextricably related to visual place recognition. Thus, the article would not be complete unless it briefly examines the general concept of the latter in the robotics community (see Section II). Similarly, Section III provides a brief introduction to SLAM. The differences between the commonly used terms of localization, re-localization, and loop closure detection are also distinguished and discussed. In Section IV, an overview of the currently standard problem formulation for visual loop closure detection is given. The following sections review each of its modules in more detail. More specifically, Section V provides the methodology to describe the environment’s appearance through feature extraction, including traditional hand-crafted and recent deep learning techniques. Section VI presents the environment’s representation, viz., database, and how locations are indexed during query. The robot’s confidence originated from similarity metrics is addressed in Section VII, while Section VIII provides some benchmarking approaches. Section IX expands the discussion and examines the current open challenges in visual loop closure detection, e.g., map scalability for long-term operations, recognition under environmental changes, and computational complexity. As a final note, considering this survey as a tutorial and positioning paper for experts and newcomers in the field, we prompt each reader to jump to the individual’s section of interest directly. A map diagram of the topics discussed in the article at hand is depicted in Fig. 3. Light-grey boxes indicate the topics described in brief, while darker boxes are the ones presented in detail.

![](images/2022_LoopClosure-Survey/3e712f4dbd230314383ce9fa832a070c058605fb28a7442c59d4faf4e6e7a00a.jpg)  
Fig. 3. Diagram depicting the taxonomy of topics discussed in the proposed article. Starting from the general concept of place recognition in computer vision and robotics, we describe how visual place recognition is used into SLAM pipelines to address the task of loop closure detection. The darker colored topics are the ones described in detail within this survey, while the lighter ones are briefly reported.

## II. VISUAL PLACE RECOGNITION

If we have been there before, we realize that viewing a single photograph is sufficient to understand where the picture was captured. This fact highlights the impact of appearance cues in localization tasks [29]–[32]. Historically, visual place recognition depicts a related task, studied intensively by the researchers in computer vision society within a broad spec trum of applications [33], including 3D reconstruction [34], map fusion [35], augmented reality [36], and structure-frommotion [37]. However, visual place recognition in robotics is somehow different. Since the knowledge of the environment, is a prerequisite for complex robotics tasks, it is vital for the vast majority of localization implementations or re-localization and loop closure detection pipelines within SLAM. Generally, it identifies the ability of a system to match a previously visited place using onboard computer vision tools. In robotics, the recognizer has to generalize as much as possible, in the sense that it should support robust associations among different recordings of the same place against viewpoint and environmental variations under run-time, storage, and processing power restrictions. Moreover, the application requirements for both domains are also driving this differentiation. The robotics community focuses on having highly confident estimates when predicting a revisited place, viz., to perform visual loop-closure detection. At the same time, researchers in computer vision typically prefer to retrieve as many prospective matches of a query image as possible, e.g., for 3D-model reconstruction [38]. More specifically, the former has to identify only one reference candidate associated with the previously visited traversal under varied conditions, while the latter can retrieve more matches, corresponding to a broad collection of images. Furthermore, in robotics, visual place recognition involves sequential imagery, significantly affecting the recognizer’s performance. Finally, visual place recognition in robotics, apart from performing a topological constraint that indicates the same place in the database, produces geometric information that can be used to correct the trajectory, such as in the case of loop closure detection. In [7], Lowry et al. discuss the problem and provide a comprehensive survey of visual place recognition, while the work of Garg et al. [10] gives a broader discussion about the differences between visual place recognition in computer vision and robotics communities.

## III. SIMULTANEOUS LOCALIZATION AND MAPPING

A robot’s capability to build a map (deriving the model of an unknown environment) and localizing (estimating its position) within that map is essential for intelligent autonomous operations and, during the last three decades, one of the most famous research topics [39]. This is the classic SLAM problem, which has evolved as a primary paradigm for providing a solution for autonomous systems’ navigation without depending on absolute positioning measurements, such as the ones given by global navigation satellite systems (GNSS). Nevertheless, given the noise in the sensors’ signal and modeling inaccuracies, drift is presented even if the most accurate state estimators are used. Therefore, the robot’s motion estimation degenerates as the explored environment size grows, specifically with the traversed cycles’ size therein [40]. A SLAM architecture commonly comprises a front-end and a backend component. The former handles the unprocessed sensor data modeling that is amenable for estimation, and the latter performs assumptions based on the incoming sensory inputs. Loop closure detection belongs to the front-end, as it is required to create constraints among locations once the robot returns to an earlier visited area [3], while outlier (i.e., falsepositive loop closures) rejection is assigned to the back-end of SLAM [41]–[43]. In what follows, the role of loop closure detection and re-localization in the localization and mapping engines of SLAM is analyzed, and its dependence on the utilized sensing devices are examined.

## A. Localization

Localization refers to the robot’s task to establish its pose concerning a known frame of reference [44]. More specifically, “the wake-up robot problem,” [45] i.e., global localization, addresses the difficulty of recovering the robot’s pose within a previously built map. At the same time, similar to the previous task, the re-localization task into SLAM, which is also known as the “kidnapped-robot problem,” [46] concerns the position recovery based on a beforehand generated map following an arbitrary “blind” displacement, viz., without awareness of the displacement, happening under heavy occlusions or tracking failures. The above tasks attempt to identify a correspondence connecting the robot’s current observation with a stored one in the database. Nevertheless, the robot has no information about its previous pose during that process, and it can be considered lost. Contrariwise, constraints between the current and the previous pose are known during loop closure detection. More specifically, the system tries to determine whether or not the currently recorded location belongs to an earlier visited area and compute an additional constrain to further improve its localization and mapping (see Section III-B) accuracy. However, each of the aforementioned cases is addressed by similar mechanisms using the most recent observation and a place recognizer. If a match is successful, it provides correspondence and, in many cases, a transformation matrix between the current and the database poses in the map.

## B. Mapping

Trajectory mapping, which is of particular interest in autonomous vehicles, provides the robot with a modeled structure to effectively localize, navigate, and interact with its surroundings. Three major mapping models exist within SLAM, viz., metric, topological, and hybrid (metric-topological) maps. Metric maps provide geometrically accurate representations of the robot’s surroundings, enabling centimeter-level accuracy for localization [47]. However, when the appearance information is not considered, more frequent loop closure detection failures in environments with repetitive geometrical structures are indicated. In addition, this model is also computationally infeasible when large distances are dealt with [48]. Relying on a higher representation level than metric ones, topological maps mimic the humans’ and animals’ internal maps [49]–[51]. A coarse, graph-like description of the environment is generated, where each new observation is added as a node, corresponding to a specific location. Furthermore, edges are used to denote neighboring connections, i.e., if a location is accessible from a different one. This flexible model, introduced by Kuipers and Byun [52], provides a more compact structure that scales better with the traversed route’s size. Regardless of the robot’s estimated metric position, which becomes progressively less accurate, these approaches attempt to detect loops only upon the similarity between sensory measurements [53]–[55]. On the one hand, two nodes become directly connected during re-localization, enabling the robot to continue its mapping process. On the other hand, loop closure detection forms additional connections between the current and a corresponding node while the current node rectifying this way the accumulated mapping error [56] (see Fig. 4). An extensive review regarding topological mapping is provided by the authors in [5]. Finally, in metric-topological maps, the environment is represented via a graph-based model whose nodes are related to local metric maps, i.e., a topological map is constructed, which is further split into a set of metric sub-maps [57]–[60].

## C. Sensing

Aiming at overcoming GNSS limitations and detecting loop closures, different sensors have been used over the years, including wheel encoders, sonars, lasers, and cameras. Generally, range finders are chosen because of their capability to measure the distance of the robot’s surroundings with high precision [61]–[63]. However, they are also bounded with some limitations. The sonar is fast and inexpensive but frequently very crude, whereas a laser sensor is active and accurate; however, it is slow. Within the last years, since 3D maps [64] became more popular over traditional 2D [65], light detection and ranging (LiDAR) is established as the primary sensor for large-scale 3D geometric reconstructions [66]. Yet, they remain unsuitable for mass installation on mobile robots due to their weight, price, and power consumption. Furthermore, its measurements, i.e., scan readings, cannot be distinguished during loop closure detection from locations with similar shapes but different appearances, such as corridors. Although successful mapping techniques based on range-finders are implemented [67]–[69], these types of sensors tend to be associated with, or replaced by, single cameras [70]–[76] or stereo camera rigs [77]–[82]. This is mainly due to the rich textural information embedded in images, the cameras’ low cost, and their applicability to various mobile robots with limited computational powers, such as the unmanned aerial vehicles (UAVs) [83]. Yet, even if multi-sensor frameworks [84]–[87] can improve performance, especially in changing environmental conditions [88], such a setup requires expensive hardware and additional calibration than camera-only ones [89]. During the second decade of visual-based navigation, autonomous robots’ trajectory mapping of up to 1000 km has been successfully achieved using cameras as the primary sensory modality [90]. The solution of a monocular camera provides practical advantages concerning size, power, and cost, but also several challenges, such as the unobservability of the scale or state initialization. Nevertheless, these issues could be addressed by adopting more complex setups, such as stereo or RGB-D cameras [91]–[94]. Lastly, even if limited attention has been received regarding the event-based cameras within the vision research community [95], their high dynamic range and the lack of motion blur are proved benefitial in challenging lighting conditions and high-speed applications [96].

![](images/2022_LoopClosure-Survey/0f86db59448b5695c78cc85e74cd657117377b78303dc4245d3736f1b16fb0f1.jpg)  
Fig. 4. A representative example highlighting the differences between topological loop closure detection and re-localization. The query node (shaded observation) searches the database for candidate matches and, subsequently, the most similar is chosen. Two components are connected through a constraint (bottom) when the system re-localizes its pose due to a tracking failure, while an another one edge is created between the two nodes (top) in the case of loop closure detection.

![](images/2022_LoopClosure-Survey/d71330a807581d9b208f2e6bc0dad056fc138e288157502c9158c9d17baac25a.jpg)  
Fig. 5. Schematic structure depicting the essential parts of a visual loop closure detection system. The image is processed to extract the corresponding visual representation, by either using trained data (visual bag of words) or not, and the robot’s internal database is constructed incrementally as the newly captured sensory measurement enters the system (visual data). When the query image arrives, its representation is compared against the database, i.e., the environment representation, aiming to decide whether the robot navigates to an already visited area. Since loop closures occur sparsely, the database is updated accordingly when a match occurs.

## IV. STRUCTURE OF A VISUAL LOOP CLOSURE DETECTION SYSTEM

A loop closure detection system’s generic block diagram is depicted in Fig. 5. Firstly, a system interpreting the environment’s appearance has to detect previously visited locations by employing only visual sensory information; thus, the perceived images have to be interpreted robustly, aiming for an informatively built map. Then, the system’s internal environment representation of the navigated path needs to be addressed. In many cases, such representations are driven by the robot’s assigned mission. Aiming to decide whether or not the robot navigates a previously seen area, the decision extraction module performs data comparisons among the query and the database instances. Confidence is determined via their similarity scores. Lastly, as the system operates on-line, the map is updated accordingly throughout the autonomous mission’s course. Each of the parts mentioned above is detailed in the following sections.

## V. FEATURE EXTRACTION

Aiming at an informative map constructed solely from visual sensing, a suitable representation of the recorded data is needed. It is not surprising that most pipelines use feature vectors extracted from images to describe the traversed route, given their discriminative capabilities. This characteristic extends to the visual loop closure detection task and renders it essential to select an effective visual feature encoder. The traditional choice for such a mechanism refers to hand-crafted features that are manually designed to extract specific image characteristics. Recently, however, the outstanding achievements in several computer vision tasks through deep learning have turned the scientific focus towards learned features extracted from CNN activations. A categorization of these methods is provided in Table I.

## A. Hand-Crafted Feature-Based Representation

It is shown via various experimental studies that humans can rapidly categorize a scene using only the crude global information or “gist” of a scene [177], [178]. Similarly, methods implemented upon global feature extractors describe an image’s appearance holistically utilizing a single vector. Their main advantages are the compact representation and computational efficiency, leading to lower storage consumption and faster indexing while querying the database. However, hand-crafted global extractors suffer from their inability to handle occlusions, incorporate geometric information, and retain invariance over image transformations, such as those originated from the camera’s motion or illumination variations. On the other hand, detecting regions-ofinterest, using hand-crafted local extractors, in the image and subsequently describing them has shown robustness against transformations such as rotation, scale, and some lighting variations, and in turn, allow recognition even in cases of partial occlusions. Moreover, as the local features’ geometry is incorporated, they are naturally intertwined with metric pose estimation algorithms, while their spatial information is necessary when geometrical verification is performed, as discussed in Section VII-C. In the last decade, most of the advances achieved in visual loop closure detection were based on such features. Lastly, it is the scenario wherein the robot needs to operate that drives the selection of feature extraction method. For instance, when an environment is recorded under severe viewpoint variations, a method based on local extractors would be typically preferred, while for robot applications where low computational complexity is critical, global descriptors fit better to the task. An overview of both methods is illustrated in Fig. 6.

TABLE I  
METHOD’S CATEGORIZATION BASED ON THEIR Feature Extraction AND Environment Representation ATTRIBUTES
<table><tr><td>Feature Extraction</td><td colspan="2">Looking Behind</td></tr><tr><td></td><td></td><td>Single-image-based representation | Sequence-of-images-based representation</td></tr><tr><td>Hand-crafted global features</td><td>[97]–[108]</td><td>[109]-[116]</td></tr><tr><td>Hand-crafted local features</td><td>[117]–[139]</td><td>[140]-[144]</td></tr><tr><td>Learned image-based features</td><td>[145]-[149]</td><td>[150]–[156]</td></tr><tr><td>Learned pre-defined region-based features</td><td>[157]-[163]</td><td></td></tr><tr><td>Learned extracted region-based features Learned extracted simultaneously image-based and region-based features</td><td>[164]-[169] [173], [174]</td><td>[170]–[172]</td></tr><tr><td></td><td></td><td></td></tr></table>

1) Global Features: Oliva and Torralba proposed the most recognized global descriptor, widely known as Gist [179]–[181], inspiring several loop closure detection pipelines [97]–[100]. A compact feature vector was generated through image gradients extracted from Gabor filters, ranging in spatial scales and frequencies. Following the Gist’s success, Sünderhauf and Protzel achieved to detect loops through BRIEF-Gist [101], a global model of BRIEF (BRIEF stands for binary robust independent elementary features [182]) local descriptor to represent the entire image. Likewise, using the speeded-up robust features (SURF) method [176], a global descriptor called WI-SURF was proposed in [89]. In [183], the authors showed that when applying disparity information on the local difference binary (LDB) descriptor [102], [103], failures due to perceptual aliasing could be reduced.

Besides, another series of techniques for describing images globally is based on histogram statistics. Different forms, e.g., color histograms [184], histogram-of-orientedgradients (HOG) [104], [110], or composed receptive field histograms [185], were adopted. HOG [175], which is the most frequently used technique, calculates every pixel’s gradient and creates a histogram based on the results (see Fig. 6a), while pyramid-of-HOG (PHOG) describes an image via its local shape and its spatial layout [186]. A differentiable version of HOG was introduced in [187]. Customized descriptors, originated from downsampled patch-based representations [111], constitute another widely utilized description method [113], [114]. A global descriptor derived from principal component analysis (PCA) was employed in [105].

![](images/2022_LoopClosure-Survey/6862a7139f3f9dacff52e8d6219acd76edc27823f5302b1c9404e9cbb808fa8e.jpg)  
Fig. 6. Instances of hand-crafted feature descriptors, both (a) global (based on the entire image) and (b) local (based on regions-of-interest), extracted from the incoming image. (a) Whole image descriptors process each block in the image regardless of its context, e.g., the histogram-of-orientedgradients (HOG) [175]. (b) Local features, like the speeded-up robust features (SURF) [176], are indicated in salient parts of the image and subsequently described. This way, a camera measurement is represented by the total of samples.

As opposed to many of the above mentioned global description techniques, viz., Gist and HOG, which are able to encode viewpoint information through concatenation of grid cells, the model of visual bag of words (BoW) describes the incoming image holistically, retaining invariant viewpoint information. In particular, this model, which was initially developed for language processing and information retrieval tasks [188], allows the images’ description as an aggregation of quantized local features, i.e., “visual words” [189]. More specifically, local features are classified according to a unique database, known as “visual vocabulary,” generated through unsupervised density estimation techniques [190] over a set of training descriptors (either real-valued [106]–[108] or binary ones [93], [115], [141], [143]). An overview of this process is illustrated in Fig. 7. However, as several visual words may occur more frequently than others, the term-frequency inverse-document-frequency (TF-IDF) scheme [191] has been adopted to weight each database element. This way, each visual word is associated with a product proportional to the number of occurrences in a given image (term frequency) and inversely proportional to its instances in the training set (inverse document frequency). Then, every image is represented via a vector of all its TF-IDF word values [192]. Fisher Kernels [193], [194] refine the visual BoW model via fitting a Gaussian mixture over the database entries and the local features. At the same time, VLAD (VLAD stands for vector of locally aggregated descriptors [195], [196]) concatenated the distance vectors between each local feature and its nearest visual words leading to improved performance results in the cost of increasing the memory footprint.

![](images/2022_LoopClosure-Survey/69f39e3009289e0a6ecd7610b907f6ba2344fd26a6c4b57b223e4aaf04e6db40.jpg)  
Fig. 7. The visual bag of words model based on a previously trained visual vocabulary. Speeded-up robust features [176] are extracted from regions-of-interest in the incoming image, and subsequently, their descriptors are connected with the most similar visual word in the vocabulary. The output vector (1 N dimension, where N corresponds to the vocabulary’s size) is a feature vector which represents the frequency of each visual word included in the camera data.

2) Local Features: Historically, the most acknowledged method for extracting local features is the scale-invariant feature transforms (SIFT) [197]. Based on the differenceof-gaussian (DoG) function, regions-of-interest are detected, while HOG computes their neighborhood’s description. SURF (see Fig. 6b), inspired by SIFT, proposes a faster extraction version using an approximation of the determinant of Hessian blob detector for identifying regions-of-interest and the sum of the Haar wavelet response around each point for feature description. CenSurE [198], a lightweight equivalent of SURF, detects regions-of-interest using center-surrounded filters across multiple scales of each pixel’s location. KAZE [199] detects and describes 2D features in a nonlinear scale space by means of nonlinear diffusion filtering, demonstrating improved feature quality. However, it also induces higher computationally complexity. As the research community moved towards the binary description space, various feature extractors were developed offering similar SIFT and SURF performance; yet, exhibiting reduced complexity and memory requirements. Most of them extended BRIEF, which uses simple intensity difference tests to describe regionsof-interest, by incorporating descriptiveness and invariance to scale and rotation variations, such as LDB, ORB [200], BRISK [201], FREAK [202], and M-LDB [203]. Moreover, several local extractors used geometrical cues, such as line segments [204] or integrated lines and points, into a common descriptor [205], aiming to cope with region-of-interest detection in low-textured environments.

When directly describing images using local extractors, a massive quantity of features is created [206]. This dramatically affects the system’s performance, mainly when real-valued features are used [118]. Different loop closure detection pipelines partially reduce their quantity by selecting the most informative ones [117], [136], or utilizing binary descriptors to avoid such cases [119], [129]. Although the utilization of visual BoW is an efficient technique for detecting loop closures when local features are adopted, two weaknesses are presented. First, the visual vocabulary is typically generated a priori from training images and remains constant during navigation, which is practical; however, it does not adapt to the operational environment’s attributes, limiting the overall loop closure detection performance. Secondly, vector quantization discards the geometrical information, reducing the system’s discriminative nature, primarily in perceptual aliasing cases. Consequently, several approaches address these limitations incrementally, i.e., along the navigation course, to generate the visual vocabulary [128]. This concept was introduced by Filliat [120], assuming an initial vocabulary that was gradually increased as new visual features were acquired. Similarly, Angeli et al. [46], [121] merged visual words through a user-defined distance threshold. Nevertheless, most incremental vocabularies (either using real valued-based [122], [124]–[127], [130] or binary descriptors [131]–[133], [135], [139]) are based on the descriptors’ concatenation from multiple frames to obtain a robust representation of each region-of-interest.

## B. Learned Feature-Based Representation

CNN is a concept introduced by LeCun et al. in the late ’80s [207], [208]. Its deployment efficiency is directly associated with the size and quality of the training process,<sup>1</sup> which generally constitute practical limitations [209]. However, its recent successes in the computer vision field are owed to a combination of advances in GPU computational capabilities and large labeled datasets [210]. The remarkable achievements in image classification [210], [211] and retrieval tasks [212], [213] are owed to the capability of CNNs to learn visual features with increased levels of abstraction. Hence, it was reasonable to expect that the robotics community would experiment with learned feature vectors as the loop closure detection’s backbone is oblivious to the type of descriptions used.

A fundamental question is how a trained CNN generates visual representations. To answer this, we need to consider the four following paradigms that achieve feature extraction through different processes: 1) the whole image is directly fed into a network, and the activations from one of its last hidden layers are considered as the image’s descriptor [149], [215]–[217]; 2) specific image regions are introduced to the trained CNN, while the respective activations are aggregated to form the final representation [218]–[222]; 3) the CNN receives the whole image, and via the direct extraction of distinct patterns based on the convolutional layers’ responses, the most prominent regions are detected [214], [223]–[226]; 4) the CNN receives the whole image, and simultaneously predicts global and local descriptors [173], [174]. An illustrative paradigm is shown in Fig. 8. Generally, representing images globally using techniques from the first category shows reduced robustness when effects, such as partial occlusion or severe viewpoint variations, are presented. Image features emerging from the second category usually cope with viewpoint changes more effectively but are computationally costly since they rely on external landmark detectors. Finally, features that emerge from the third category leverage both variations, i.e., viewpoint and appearance, while features from the last category resulting in significant run-time savings.

![](images/2022_LoopClosure-Survey/39f3263934e0ab08257b9578f42be774625e9227dfa2fb6737c2b559c7bfb9c2.jpg)  
Fig. 8. A representative example of a fully-convolutional network that jointly extracts points of interest and their descriptors from an image [214].

1) Image-Based Features: Chen et al. [145] were the first to exploit learned features extracted from all layers of a trained network [215] for object recognition to detect similar locations. However, subsequent studies showed that the utilization of intermediate representations with and without the CNN’s fully connected layers could offer high performances [146], [148] and rich semantic information [227], [228]. Other recent contributions provided helpful insights for better understanding the complex relationship between network layers and their features visualization [229], [230]. Since then, different architectures with slight modifications have been developed and used for visual loop closure detection [147], [151], [153]. Inspired by the success of VLAD, NetVLAD [216] was proposed as a trainable and generalized layer that forms an image descriptor via combining features, while the spatial pyramid-enhanced VLAD (SPE-VLAD) layer improved VLAD features by exploiting the images’ spatial pyramid structure [149]. In all of the above, powerful network models were utilized as the base architecture, viz., AlexNet [210], VGG [231], ResNet [232], Inception [233], DenseNet [234], and MobileNet [235].

2) Pre-Defined Region-Based Features: Compared to the holistic approaches mentioned above, another line of works relied on detecting image landmarks, e.g., semantic segmentation and object distribution, originated from image patches to describe the visual data [157]–[163]. More specifically, in [212], learned local features extracted from image regions were aggregated in a VLAD fashion, while descriptors from semantic histograms and HOG were concatenated in a single vector in [158]. VLASE [159] relied on semantic edges for the image’s description [236]. In particular, pixels which lay on a semantic edge were treated as entities of interest and described with a probability distribution (as given by CNN’s last layer). The rest of the description pipeline was similar to VLAD. Similarly, Benbihi et al. presented the WASABI image descriptor for place recognition across seasons built from the image’s semantic edges’ wavelet transforms [163]. It represented the image content through its semantic edges geometry, exploiting their invariance concerning illumination, weather, and seasons. Finally, a graph-based image representation was proposed in [162], which leveraged both the scene’s geometry and semantics.

3) Extracted Region-Based Features: The idea of detecting salient regions from late convolutional layers instead of using a fixed grid and then describing these regions directly as features have achieved impressive results [164]–[167]. Regions of maximum activated convolutions (R-MAC) used max-pooling on cropped areas of the convolutional layers’ feature maps to detect regions-of-interest [223]. Neubert and Protzel presented a multiscale super-pixel grid (SP-Grid) for extracting features from multiscale patches [224]. Deep local features (DELF) combined traditional local feature extraction with deep learning [225]. Regions-of-interest were selected based on an attention mechanism, while dense, localized features were used for their description. SuperPoint [214] and D2-net [226] were robust across various conditional changes. By extracting unique patterns based on the strongest convolutional layers responses, the most prominent regions were selected in [164]. Multiple learned features were then generated from the activations within each spatial region in the previous convolutional layer. This technique was additionally extended by a flexible attention-based model in [165]. Garg et al. built a local semantic tensor (LoST) from a dense semantic segmentation network [170], while a two-stage system based on semantic entities and their geometric relationships was shown in [167]. Region-VLAD (R-VLAD) [166] combines a low-complexity CNN-based regional detection module with VLAD. DELF was recently extended by R-VLAD via down-weighting all the regional residuals and storing a single aggregated descriptor for each entity of interest [237].

4) Extracted Simultaneously Image-Based and Region-Based Features: Aiming to bridge the gap between robustness and efficiency, an emerging trend in the line of learned features combines the advances in the aforementioned fields to jointly estimate global and local features [173], [174]. Hierarchical feature network (HF-Net) [173], a compressed model trained in a flexible way using multitask distillation, constitutes a fast; yet, robust and accurate technique for localization tasks. In [174], the authors unify global and local features into a single model referred as DELG (stands for DEep Local and Global features). By combining generalized mean pooling for global features and attentive selection for local features, the network enables accurate image retrieval.

![](images/2022_LoopClosure-Survey/9da9280311a3895cfc5c6c5aea745297b1614450240392c4b0bf2b2e6cab2d99.jpg)  
Fig. 9. Depending on their trajectory mapping, appearance-based systems are divided into two main categories, namely (a) single-image-based and (b) sequence-of-images-based. Methods of the former category represent each image in the database as a distinct location, while the latter category’s schemes generate sequences, i.e., groups of individual images, along the navigation course. The observations included in each of these sequences, also referred to as sub-maps, typically consist of common visual data.

## VI. LOOKING BEHIND

As mentioned earlier, visual localization and loop closure detection are quite similar tasks. They share the primary goal of finding the database’s most alike view, but for loop detection, all images acquired during the robot’s first visit to a given area are treated as the reference set for a query view. As the system processes the sensory input data, it incrementally generates the internal map, i.e., database, which plays a vital role in the subsequent steps for location indexing and confidence estimation about its current position. Depending on how the robot maps the environment, visual loop closure detection pipelines are distinguished into singleimage-based and sequence-of-images-based. Frameworks of the first category seek the most identical view in the robot’s route, while techniques belonging in the second category look for the proper location between sub-maps, i.e., groups of individual images. This section’s remainder briefly describes representative approaches by distinguishing them based on how they map the trajectory and how the system searches the database for potential matches.

## A. Environment Representation

Single-image-based mapping is the most common scheme for visual loop closure detection. During navigation, the extracted visual features from each input image are associated with a specific location (see Fig. 9a). When the off-line visual BoW model is used, the map is formulated as a set of vectors denoting visual words at each location [106]. Otherwise, a database of descriptors indexed according to their extracted location is built [118], [136].

In contrast to the conventional single-image-based methods, various frameworks use image-sequence partitioning (ISP) techniques to define group-of-images along the traversed route, which are defined as smaller sub-maps [108], [116], [238]–[240], as illustrated in Fig. 9b. These techniques either use the single-image-based representation for their members [111], or they describe each submap through sequential descriptors [144], [168], [171], [172]. However, many challenges emerge when splitting the map into sub-maps, such as optimal size, sub-map overlapping throughout database searching, and uniform semantic map definition [80]. SeqSLAM [111], the most acknowledged algorithm in sequence-of-images-based mapping, has inspired a wide range of authors since its first introduction [110], [112], [142], [150], [152], [154]–[156], [241]–[243]. The multitude of these pipelines, with SeqSLAM among them, uses a pre-defined quantity of images to segment the trajectory. Nevertheless, the unknown frame density, out-of-order traverses, and diverse frame separation are some of the characteristics which negatively affect the fixed-length sub-mapping methods performance. To avoid such cases, dynamical sequence definition techniques are employed using landmarks’ co-visibility properties [244]–[247], features’ consistency among consecutive images [109], [113], [114], [143], temporal models [70], [156], or transition-based sub-mapping, e.g., through particle filtering [241].

## B. Location Indexing

A visual loop closure detection system must search for similar views among the ones visited to decide whether a query instance corresponds to a revisited location. Firstly, recent database images should not share any familiar landmarks with the query. This is because images immediately preceding the query are usually similar in appearance to the recent view; however, they do not imply that the area is revisited. Aiming to prevent the system from detecting false-positives, these locations are rejected based on a sliding window defined either by a timing constant [127], [130], [135], [169], [248] or environmental semantic changes [46], [90], [121], [137], [141], [143]. Methods based on the off-line visual BoW model employ the inverted indexing technique for searching, wherein the query’s visual words indicate the locations that have to be considered as potential loop events. In contrast, methods that do not follow this model implement an exhaustive search on the database descriptors’ space [71], [119], [130], [134], [137], [139].

## VII. DECISION MAKING

The final step is the decision of whether the robot observes a previously mapped area or not. Different comparison techniques, which are broadly classified according to their map representation, have been proposed to quantify this confidence [4]; the first one is image-to-image, and the second is sequence-to-sequence. The former computes an individual similarity score for each database entry [106], [123], [134], which is then compared against a pre-defined hypothesis threshold to determine whether the new image is topologically connected to the older one. Otherwise, the query cannot match any pre-visited one, resulting in a new location addition to the database. On the contrary, sequence-to-sequence is typically based on the comparison of sub-maps [140]–[144]. Subsequently, loop closing image pairs are considered the groups’ members with the highest similarity scores.

Moreover, to avoid erroneous detections, both temporal and geometrical constraints are employed, primarily to address perceptual aliasing conditions. Representative examples include recognizing a closed-loop only if supported by neighboring ones or if a valid geometrical transformation can be computed between the matched frames. As a final note, the resulting confidence metrics fill a square matrix whose (i, j) indexes denote the similarity between images I<sub>i</sub> and I<sub>j</sub> .

![](images/2022_LoopClosure-Survey/74d4ea8be9e7a20676e4f51c0115db4525c628fb12cfa53d4df9df5554181487.jpg)

![](images/2022_LoopClosure-Survey/c0a4e446858c71a7e4d420a161b1d2c75b82a93c003f7c1e5fba0ac2d2da562f.jpg)

![](images/2022_LoopClosure-Survey/663fa4d7d42cf25256c276ece4e344f75e8f3a6924a9859bec5d606f4e0cc490.jpg)

![](images/2022_LoopClosure-Survey/36fbd33227ee868700f70742676cf22ca991bfaa8595cf118fde509d2578a0d3.jpg)  
Fig. 10. As the most recently obtained image’s local descriptors are extracted at query time, votes are distributed to database locations l from where their nearest neighbor descriptor originates. The colored and gray cubes represent the votes casted to several locations. After the locations’ polling, a voting score is received that is used to evaluate the similarity. The naïve approach is based on the number of votes (top-right); however, since thresholding the number of votes is not intuitive, more sophisticated methods, such as binomial density function [129], utilize the location’s total amount of aggregated votes to compute a probabilistic score which highlights loop closure detections.

## A. Matching Locations

Sum of absolute differences (SAD), a location’s similarity votes density, and Euclidean or cosine distance are the commonly used metrics employed to estimate the matching confidence between two instances. Directly matching the features extracted from two images represents a reasonable similarity measurement when global representations are used (either hand-crafted or learned). However, when local features are selected, voting schemes are selected. These techniques depend on the number of feature correspondences leading to an aggregation of votes, the density of which essentially denotes the similarity [105] between two locations. This is typically implemented by a k-nearest neighbor (k-NN) search [123], [132], [134], [244], [245]. The simple approach is to count the number of votes and apply heuristic normalization [134]; however, in these cases, thresholding is not intuitive and varies depending on the environment. Rather than naïvely scoring the images based on their number of votes, Gehrig et al. [129] proposed a novel probabilistic model originated from the binomial distribution (see Fig. 10) [130], [136], [137]. By casting the problem into a probabilistic scheme, the heuristic parameters’ effect is suppressed, providing an effective score to classify matching and non-matching locations, even under perceptual aliasing conditions. Over the years, probabilistic scores were used to enhance the system’s confidence [241]. The Dempster-Shafer probability theory, which models ignorance without prior knowledge of the environment, was introduced by Savelli and Kuipers [249]. Similar to the Bayes approach discussed in [11], [15], later works followed the Bayesian filtering scheme to evaluate loop closure hypotheses [46], [90], [100], [106], [107], [121], [127], [135], [138].

Each of the techniques mentioned above can be efficiently adopted in sequence-of-images-based methods (e.g., SeqSLAM [111], HMM-SeqSLAM [250], ABLE-M [251], S-VWV [115], MCN [252]). By comparing route segments rather than individual camera observations, global representations are able to provide outstanding results through the utilization of relatively simple techniques. As shown in SeqSLAM, to evaluate the locations’ similarity, the SAD metric is used between contrast-enhanced, low-resolution images avoiding this way the need for key-points extraction. For a given query image, comparisons between the local query sub-map and the database are performed. The likelihood score is the maximum sum of normalized similarity scores over the length of pre-defined constant velocity assumptions, i.e., alignments among the query sequence and the database sequence images. This process is inspired by speech recognition and is referred to as continuous dynamic time warping (DTW) [253]. Alignment is solved by finding the minimum cost path [254], while dynamic programming [255], graph-based optimization [256]–[259], or the incorporation of odometry information [142] strengthens its performance [250]. To improve the systems’ performance, frameworks based on dynamic adjustment of the sequence length are also proposed that leverage feature matching [113], [114], GPS priors [251], or modeling the area hypotheses over different length assumptions [260], [261].

## B. Exploiting the Temporal Consistency

In robot navigation, unlike large-scale image retrieval or classification tasks, where images are disorganized, sensory measurements are captured sequentially and without time gaps [262], [263]. Most pipelines pay a high price for indicating a loop closure, but there is minor harm if one is missed since many chances in the following images are afforded due to the existing temporal continuity. Every sequence-of-imagesbased mapping technique leverages the sequential characteristic of robotic data streams aiming to disambiguate the boisterous single-image-based matching accuracy. The temporal consistency constraint, which is mainly adopted when single-image-based mapping is used, filters out inconsistent loop closures through heuristic methods (e.g., continuous loop hypothesis before a query is accepted [130], [132], [137], [248]) or more sophisticated ones (e.g., the Bayesian filter [100], [106], [121], [127], [135], [138]).

## C. Is the Current Location Known? The Geometrical Verification

After data association, a geometrical verification check is often implemented based on the spatial information provided in local features, either hand-crafted or learned ones, by computing a fundamental/essential matrix or other epipolar constraints [72], [78], [82], [84], [106], [119], [130], [135]–[137], [141], [237], [264]–[266]. Typically, it is performed using some variation of the RANSAC algorithm and additionally provides the relative pose transformation if a successful correspondence is found [267]. Moreover, a minimum number of RANSAC inliers has to be satisfied for a loop to be confirmed [268].

When a stereo camera rig is used [94], a valid spatial transformation between the two pairs of matching images is computed through the widely used iterative closest point (ICP) algorithm for matching 3D geometry [269]. Given an initial starting transformation, ICP iteratively determines the transformation among two point clouds that minimizes their points’ error. Still, a high computational cost accompanies the matching process when the visual and the 3D information are combined [106]. As a final note, geometrical verification is based on the spatial information of hand-crafted local features. Typically, a system that uses single vector representations (either global or visual BoW histograms) needs to further extract local features, adding more complexity to the algorithm.

## VIII. BENCHMARKING

In order to benchmark a given loop closure detection approach, three major components are mainly used: the datasets, the ground truth information, and the evaluation metrics [38]. Accordingly to the case under study, a variety of datasets exist in the literature. The ground truth is typically formed in the shape of a boolean matrix whose columns and rows denote observations recorded at different time indices $( i , j )$ . Hence, the 1 indicates a loop closure event between instances i and j and 0 otherwise. This matrix, together with the similarity one, is used to estimate how the system performs. Typically, the off-diagonal high-similarity elements of the generated similarity matrix indicate the locations where loops are closed. Finally, the chosen evaluation metric is the last component needed for measuring the performance.

## A. Evaluation Metrics

The relatively recent growth of the field has led to the development of a wide variety of datasets and evaluation techniques, usually focusing on precision-recall metrics [282]. These are computed from a loop closure detection algorithm’s outcome: the correct matches are considered true-positives, whereas the wrong ones as false-positives. In particular, whether or not two images originate from the same area determines a positive or negative loop closure event, respectively. Moreover, the outcome of a loop closure detection algorithm can be characterized as true or false based on the information provided by the ground truth. Therefore, we end up with four different detection labels: true-positive, false-positive, truenegative, and false-negative. On the one hand, a true-positive instance is regarded as any database entry identified by the algorithm within a small radius from the query’s location, while false-positives refer to incorrect detections that the system may produce and lie outside this range. On the other hand, false-negatives are the loops that had to be detected; yet, the system could not identify them, while true-negatives are those that the system correctly rejects.

![](images/2022_LoopClosure-Survey/afdd7a0e4c2497e466eff842d327a521d6f82bcf6aca2ed4e3d6b7387a23e9d6.jpg)  
Fig. 11. An illustrative example of two hypothetical precision recall curves monitoring a method’s performance. A curve is extracted by altering one of the system’s parameter. The highest possible recall score for a perfect precision (R ), which is the most common indicator for measuring the system’s performance, is shown by the red and green cycles. The precision at minimum recall $( \mathrm { P _ { R 0 } } )$ is depicted by the black cycle, while the gray color areas denote the area under the curve. At a glance, the two curves suggest that the red curve is better than the green one. Indeed, the corresponding metrics, i.e., the $\mathrm { R } _ { \mathrm { P l 0 0 } }$ at 0.6 and the expected precision at 0.8, confirm that the red curve denotes improved performance even if the area under the curve is larger in the green curve.

Thus precision is defined as the number of accurate matches (true-positives) overall system’s detections (true-positives plus false-positives):

$$
\mathrm { P r e s i c i o n } = \frac { \mathrm { T r u e - p o s i t i v e s } } { \mathrm { T r u e - p o s i t i v e s + F a l s e - p o s i t i v e s } } ,\tag{1}
$$

whereas recall denotes the ratio between true-positives and the whole ground truth (sum of true-positives and false-negatives):

$$
{ \mathrm { R e c a l l } } = { \frac { \mathrm { T r u e - p o s i t i v e } } { \mathrm { T r u e - p o s i t i v e + F a l s e - n e g a t i v e s } } } .\tag{2}
$$

A precision-recall curve shows the relationship between these metrics and can be obtained by varying a system’s parameter responsible for accepting of a positive match, such as the loop closure hypothesis threshold [283]. The area under the precision-recall curve (AUC) [284] is another straightforward metric for indicating the performance [156], [164]. Its value ranges between 0 and 1; yet, any information with respect to the curve’s characteristics is not retained in AUC, including whether or not the precision reaches 100% at any recall value [285]. The average precision is also helpful when the performance needs to be described by a single value [286]. Generally, a high precision across all recall values is the main goal for a loop closure detection system, and average precision is capable of capturing this property. However, the most common performance indicator for evaluating a loop closure detection pipeline is the recall at 100% precision (R<sub>P100</sub>). It represents the highest possible recall score for a perfect precision (i.e., without false-positives), and it is a critical indicator since a single false-positive detection can, in many cases, cause a total failure for SLAM. Nevertheless, $\mathrm { R } _ { \mathrm { P l 0 0 } }$ cannot be determined when the generated curves are unable to reach a score for 100% precision. To overcome this problem, the extended precision (EP) metric is introduced as: $\mathrm { E P } = ( \mathrm { P _ { R 0 } } + \mathrm { R } _ { \mathrm { P 1 0 0 } } ) / 2$ [287]. EP summarizes a precision-recall curve through the combination of two of its most significant features, namely, precision at minimum recall (P<sub>R0</sub>) and R<sub>P100</sub>, into a comprehensible value. In a similar manner, the recall score for 95% precision [288] is another metric for assessing visual loop closure detection systems, as a small number of fault detections can be further validated through SLAM’s back-end optimization techniques, e.g., via robust pose-graph optimization [289]–[292]. In Fig. 11, a representative example of two hypothetical precision recall curves is given. As shown, each depicted evaluation metric indicates that the red curve produces better performance $( \mathrm { R p } _ { 1 0 0 } = 0 . 6$ and EP 0.8) than the green one; although, the area under the curve is larger in the latter hypothesis.

TABLE II  
DESCRIPTION OF LOOP CLOSURE DETECTION DATASETS WITH fixed ENVIRONMENTAL CONDITIONS
<table><tr><td>Dataset</td><td>Sensor characteristics</td><td>Characteristics</td><td>Image resolution &amp; frequency</td><td># Frames</td><td>Traversed distance</td><td>Ground truth</td></tr><tr><td>KITTI (course 00) [270]</td><td>Stereo, Gray, frontal</td><td>Outdoor, urban, dynamic</td><td>1241 × 376, 10 Hz</td><td>4551</td><td>~12.5 km</td><td></td></tr><tr><td>KITTI (course 02) [270]</td><td>Stereo, Gray, frontal</td><td>Outdoor, urban, dynamic</td><td>1241 × 376, 10 Hz</td><td>4661</td><td>~13.0 km</td><td></td></tr><tr><td>KITTI (course 05) [270]</td><td>Stereo, Gray, frontal</td><td>Outdoor, urban, dynamic</td><td>1241 × 376, 10 Hz</td><td>2761</td><td>~7.5 km</td><td></td></tr><tr><td>KITTI (course 06) [270]</td><td>Stereo, Gray, frontal</td><td>Outdoor, urban, dynamic</td><td>1241 × 376, 10 Hz</td><td>1101</td><td>~3.0 km</td><td></td></tr><tr><td>Lip6O [121]</td><td>Mono, color, frontal</td><td>Outdoor, urban, dynamic</td><td>240 × 192, 1 Hz</td><td>1063</td><td>~1.5 km</td><td></td></tr><tr><td>Lip6I [121]</td><td>Mono, color, frontal</td><td>Indoor, static</td><td>240 × 192, 1 Hz</td><td>388</td><td>~0.5 km</td><td></td></tr><tr><td>City Centre [106]</td><td>Stereo, color, lateral</td><td>Outdoor, urban, dynamic</td><td>1024 × 768, 7 Hz</td><td>1237</td><td>~1.9 km</td><td></td></tr><tr><td>New College [106]</td><td>Stereo, color, lateral</td><td>Outdoor, static</td><td>1024 × 768, 7 Hz</td><td>1073</td><td>~2.0 km</td><td></td></tr><tr><td>Eynsham [271]</td><td>Omnidirectional, gray</td><td>Outdoor, urban, rural</td><td>512 × 384, 20 Hz</td><td>9575</td><td>~70.0 km</td><td></td></tr><tr><td>New College vision suite [2]</td><td>Stereo, gray, frontal</td><td>Outdoor, dynamic</td><td>512 × 384, 20 Hz</td><td>52480</td><td>~2.2 km</td><td></td></tr><tr><td>Ford Campus (course 02) [272]</td><td>Omnidirectional, color</td><td>Outdoor, urban</td><td>1600 × 600, 8 Hz</td><td>1182</td><td>~10km</td><td></td></tr><tr><td>Malaga 2009 Parking 6L [273]</td><td>Stereo, color, frontal</td><td>Outdoor, static</td><td>1024 × 768, 7 Hz</td><td>3474</td><td>~1.2km</td><td></td></tr><tr><td>EuRoC Machine Hall 05 [274]</td><td>Stereo, gray, frontal</td><td>Indoor, static</td><td>752 × 480, 20 Hz</td><td>2273</td><td>~0.1km</td><td></td></tr></table>

TABLE III

DESCRIPTION OF LOOP CLOSURE DETECTION DATASETS WITH Changing ENVIRONMENTAL CONDITIONS
<table><tr><td>Dataset</td><td>Sensor characteristics</td><td>Characteristics</td><td>Image resolution &amp; frequency</td><td>Ground truth</td></tr><tr><td>Symphony Lake [275] SFU Mountain [276] Gardens Point [277] St. Lucia [278] Oxford RobotCar [279] Nordland [242]</td><td>Omnidirectional, color, frontal Stereo &amp; mono, color &amp; gray, frontal Mono, color, frontal Mono, color, frontal Trinocular stereo, color, frontal</td><td>Outdoor, changing, static Outdoor, changing, static Outdoor, changing Outdoor, slightly changing, dynamic Outdoor, changing, highly dynamic</td><td>704 × 480, 10 Hz 752 × 480, 30 Hz 1920 × 1080, 30 Hz 640 × 480, 15 Hz 1280 × 960, 16 Hz</td><td></td></tr></table>

## B. Datasets

Most experiments are conducted on publicly available datasets, including urban environments, indoor and outdoor areas, recorded through various platforms. A renowned benchmark environment in the robotics community is the KITTI vision suite giving a wide range of trajectories with accurate odometry information and high-resolution image properties (for both image size and frame rate) [270]. Courses 00, 02, 05, and 06 are mainly used since they present actual loop closures compared to the rest ones. The incoming visual stream is captured via a camera system that is placed on a car. However, when monocular loop closure detection systems are proposed, only one camera stream is considered. The authors in [183] manually obtained the corresponding ground truth based on the dataset’s odometry data. Early studies were based on the Lip6 Outdoor and Indoor image-sequences recorded by a handheld camera facing many loop closures in an outdoor urban environment and a hotel corridor, respectively [121]. Both are considered challenging due to the sensor’s low frame rate and resolution. They contain their own ground truth information concerning the related loop closure events. City Centre, New College, and Eynsham are three datasets extensively utilized in visual SLAM and, in particular, to evaluate loop closure detection pipelines [106], [271]. The first two above were collected by a robotic platform with two cameras positioned on the left and right sides while moving through outdoor urban environments with consistent lighting conditions. Eynsham is a 70 km urban dataset consisting of two 35 km traverses. It was recorded by a Ladybug 2 camera providing panoramic images captured at 7 m intervals. Ground truth information in terms of actual loop closures is also given. Another widely used version of the New College dataset was later presented in [2]. Ford Campus is another collection of several panoramic images [272]. A considerable quantity of loop closure examples exists in both cases. Malaga 2009 Parking 6L [273] was recorded at an outdoor university campus parking lot via the stereo vision system of an electric buggy-typed vehicle. Finally, the EuRoC Machine Hall 05 is part of the EuRoC Micro Aerial Vehicle (MAV) dataset [274] and presents fast velocity changes and various loop examples with minor illumination variations. Cameras placed on a MAV obtain the respective visual data with a high acquisition frame rate. In Fig. 12, representative instances from a dataset designed explicitly for assessing loop closure detection techniques are presented, while in Table II an overview is given.

![](images/2022_LoopClosure-Survey/266d29d2a91fb4db1e25340517fcb6225b1ca77c51fc98fd7af1f5d244b5fee2.jpg)

![](images/2022_LoopClosure-Survey/cb10dcf6146f175c76b8b426b9db828fa0bcdaae35dacaa4b229ae72e5f547c0.jpg)

(a) Positive loop closure  
![](images/2022_LoopClosure-Survey/9a59bd6da83310e3f6e52a53709f31a27c586b8fe5bbb903a9bff95a68b81f9c.jpg)

![](images/2022_LoopClosure-Survey/7c55cb28dae72b541f193c22c2eec9eaaa6cd3ec10c5c9f3a598f947cf997891.jpg)  
(b) Negative loop closure  
Fig. 12. Representative positive and negative instances of Lip6 Outdoor [121].

Of particular challenge are datasets captured over multiple seasons as their appearance is modified due to different weather conditions, sun position, and vegetation state (e.g., Symphony Lake dataset [275]). SFU Mountain provides multiple trajectories of a mobile robot in a semi-structured woodland [276], while the Gardens Point Dataset contains three traverses of the Queensland University of Technology [277]. The first two are captured during daytime by walking on the two opposite sides of the walking path (lateral viewpoint), while the last one throughout the night. The respective sequences are synchronized; thus, ground truth is structured as frame correspondences. The St. Lucia dataset [278] comprises images recorded from a selection of streets at five different periods of a day over two weeks. It only contains images from suburb environments, and the appearance variations of each place are minor. Over 100 traverses with various weather (e.g., direct sun, overcast) and illumination (e.g., day, night) conditions are provided in Oxford RobotCar [279]. Several challenges of pose and occlusions, such as pedestrians, vehicles, and bicycles, are included in the recorded sequences. The Nordland dataset [242] consists of 10 hours of video footage covering four times a 728 km ride in northern Norway, one for each season. It has 37,500 images and is a highly acknowledged dataset for studying seasonal changes in natural environments. However, it does not present any variation concerning the viewpoint between the platform’s paths. In the Mapillary dataset, three sequences were recorded in Berlin city streets, presenting severe viewpoint changes with moderate conditional variations [218]. Synthia [280] is a synthetically created dataset containing trajectories in city-like environments throughout spring and winter. 959 and 947 are the total of query and reference instances, respectively. Consisting of several flybys around buildings, Lagout and Corvin are also two synthetic environments [20]. Lagout sequences $0 ^ { \circ }$ and $1 5 ^ { \circ }$ are used as reference and query datasets, respectively, to test visual place recognition techniques under moderate viewpoint changes. In a similar manner, Corvin’s loops, which are recorded at ground level, are utilized to assess visual place recognition methods under tolerant viewpoint variances Ground truth data regarding the included loop closure events for Lagout and Corvin are made available by their authors. The Newer College dataset [281] contains challenging image-sequences with aggressive shaking, fast motion, textureless surface, and rapid lighting change through a stereo camera system in New College, Oxford. A synopsis of these datasets is given in Table III.

![](images/2022_LoopClosure-Survey/23fdc8776b3c5e37e9d8e1006b920004c5880678a0a9365ff89872776805dd95.jpg)

![](images/2022_LoopClosure-Survey/486a6fb1946cc1cfba0fda016487f9a4405d88ce171fba776a0a974484193aac.jpg)

![](images/2022_LoopClosure-Survey/430e25ef0b2deb186104e31f11059baa5b4bcfe529542caa9762314831b2c757.jpg)

![](images/2022_LoopClosure-Survey/f9225bde5af8c057c7cdb5f010095b1db4ce829b389aee28cf16fcde7dccea08.jpg)  
(a) Day time.

![](images/2022_LoopClosure-Survey/c50ed473f177e8656d1f2de056fc9023187d8d3618ce93b6744b91999025cb5e.jpg)

![](images/2022_LoopClosure-Survey/618c331842864403ef89af1b87ccf33960333a2450487fb39fc3a8c61f9222a4.jpg)

![](images/2022_LoopClosure-Survey/0c7e1fe9c59e6532326681eef9f6c489d63d3b414cd2f581d1ddf54501e8fd38.jpg)  
(b) Night time.

![](images/2022_LoopClosure-Survey/5357d1d5201f8df5536fbc6e4287d976cb80198cd0892971b35d916f0a5afd90.jpg)  
Fig. 13. Example images from the Oxford RobotCar dataset [279] for both (a) day-time and (b) night-time conditions. From left to right: Autumn, winter, spring, and summer. Within long-term and large-scale SLAM autonomy, detections need to be successful despite significant variations in the images context, such as different illumination conditions, e.g., day and night, or year seasons.

## IX. NEW CHALLENGES: LONG-TERM OPERATION

The main objective of any loop closure detection pipeline is to facilitate robust navigation for an extended period and under a broad range of viewing situations. Within long-term and large-scale SLAM autonomy, previously visited locations in dynamic environments need to be recognized under different day periods and scenes with changeable illumination and seasonal conditions [86], [183], [251], [293]. As a result, it becomes increasingly difficult to match two images, mainly since such variations affect the image appearance significantly (Fig. 13). Furthermore, extreme viewpoint variations lead to severe perspective distortions and low overlap between the query and the database frames.

Another critical aspect in long-term applications is the storage requirements needed to map the whole environment effectively. The majority of approaches scale linearly to the map’s size (at best). Consequently, there has been much interest in developing compact appearance representations so as to demonstrate sub-linear scaling in computational complexity and memory demands. These techniques typically trade off memory usage with detection performance, or vice versa, for achieving computational efficiency.

## A. Dynamic Environments

During navigation in a changing environment, the topological information about the robot’s relative movement becomes more important as noise from the sensory inputs is accumulated to an overwhelming degree [294], [295]. Early works exploited the topological information through sequence matching [111], [250], or network flows [243]. However, their output is still dependent on their visual representations’ quality since the utilized hand-crafted features were not distinctive enough so as to form a genuinely reusable map [74], [278], [296]. On the contrary, representations provided by deep learning techniques show promising results on applications with challenging conditional and viewpoint changes [162], [217], [224], [297], [298]. More specifically, deep learning approaches can be utilized to either construct description features with increased robustness to perceptual changes [104], [111], [278] or to predict and negate the effect of appearance variations [211], [243], [299]–[301]. It is also worth noting that for both the above cases, networks that are previously trained for semantic place classification [302] outperform the ones designed for object recognition when applied for place recognition under severe appearance changes [228]. Moreover, as the vast majority of the approaches and datasets assume a static environment which limits the applicability of visual SLAM in many relevant cases, such as intelligent autonomous systems operating in populated real-world environments [303], [304]. Detecting and dealing with dynamic objects is a requisite to estimate stable maps, useful for long-term applications. If the dynamic content is not detected, it becomes part of the 3D map, complicating tracking or localization processes.

1) Robust Visual Representations: Such techniques are mainly based on a single global descriptor. SeqSLAM constitutes a representative example for this category, and it is extensively utilized to recognize similar locations under drastically different weather and lighting conditions by using sequenceof-images-based matching. A series of subsequent works have been developed following the same architecture [243], [305], that does not adopt learned features as their representation mechanism. Among the different variants, the gist-based pipeline [100] compares the learning-based ones [306], [307]. Another approach by Vidas and Maddern [75] utilized two different visual vocabularies by combining SURF-based visual words from the visible and infrared spectrum. Their results showed that hand-crafted features could not achieve high performances in complicated dynamic environments; however, the infrared data were more robust to extreme variations. On the other hand, techniques which are built upon learned features typically demand an extensive labeled training set [163], [164], [214], [226], [286], [308]; however, there exist some exceptions that do not require environment-specific learning samples [156], [309].

2) Learning and Predicting the Appearance Changes: These methods require labeled training data, such as matched frames from the exact locations under different conditions [171], [299], [301], [310], [311]. In [312], an average description of images was learned, viz., a vector of weighted SIFT features. Their system was trained in summer and winter environments looking for valuable features capable of recognizing places under seasonal changes. The features that co-occurred in each image taken at different times of the day were combined into a unique representation with identifiable points from any point of view, irrespective of illumination conditions [109]. Similarly, matching observations with significant appearance changes was achieved using a support-vector machine (SVM) classifier to learn patch-based distinctive visual elements [104], [313]. This approach yields excellent performance but has the highly restrictive requirement that training must occur in the testing environment under all possible environmental conditions. The authors in [305] learned how the appearance of a location changes gradually, while Neubert et al. [299] constructed a map based on visual words originated from two different conditions. A super-pixel dictionary of hand-crafted features specific for each season was built in [311] by exploiting the seasonal appearance changes’ repeatability. Using change-removal, which is similar to dimensionality reduction, showed that by excluding the less discriminative elements of a descriptor, an enhanced performance could be achieved [19], [314]. Another way to tackle such challenges was based on illumination-invariant image conversions [211], [300], [315], and shadow removal [316]–[318]. The former transferred images into an illumination invariant representation; however, it was shown that the hypothesis of a black-body illumination was violated, yielding poor results [211]. Shadow removal techniques were used to obtain invariant illumination images independent of the sun’s positions.

Lategahn et al. [319] were the first to study how the CNNs can be used for learning illumination invariant descriptors automatically. A network selected the subset of the visual features, which were consistent between two different appearances of the same location [320]. Exploiting the visual features extracted from ConvNet [218], a graph-based visual loop detection system was proposed in [263], while a BoW for landmark selection was learned in [321]. Modifying images to emulate similar query and reference conditions is another way to avoid addressing the descriptors for condition invariance. The authors in [322] learned an invertible generator, which transformed the images to opposing conditions, e.g., summer to winter. Their network was trained to output synthetic images optimized for feature matching. Milford et al. [323] proposed a model to estimate the corresponding depth images that are potentially condition-invariant.

## B. Viewpoint Variations

Viewpoint changes are as critical as the appearance variations since visual data of the same location may seem much different when captured from other views [324]. The variation in viewpoint could be a minor lateral change or a muchcomplicated one, such as bi-directional or angular changes coupled with alterations in the zoom, base point, and focus throughout repeated traverses. Over the years, most pipelines were focused on unidirectional loop closure detections. However, in some cases, they were not sufficient for identifying previously visited areas due to bidirectional loop closures, i.e., when a robot traverses a location from the opposite direction. This type of problem is crucial because solely unidirectional detections do not provide robustness in long-term navigation. Traditional pipelines, such as ABLE-P [325], identified bidirectional loops by incorporating panoramic imagery. A correspondence function to model the bidirectional transformation, estimated by a support-vector regression technique, was designed by the authors in [326] to reject mismatches.

To achieve greater viewpoint robustness, semantically meaningful mapping techniques were adopted to detect and correct large loops [146], [164], [327]. Using visual semantics, extracted via RefineNet [328], multi-frame LoST-X [170] accomplished place recognition over opposing viewpoints. Similarly, appearance invariant descriptors (e.g., objects detected with CNN [145], [151], [161], [162], [166], [218], [221], [329], [330] or hand-crafted rules [224], [331]) showed that semantic information can provide a higher degree of invariability. Likewise, co-visibility graphs, generated from learned features, could boost the invariance to viewpoint changes [100], [222].

Finally, another research trend which has recently appeared tries to address the significant changes in viewpoint when images are captured from ground to aerial platforms using learning techniques. In general, the world is observed from much the same viewpoints over repeated visits in cases of ground robots; yet, other systems, such as a small UAV, experience considerably different viewpoints which demand recognition of similar images obtained from very wide baselines [20], [332]. Traditional loop closure detection systems do not usually address such scenarios; novel algorithms have been proposed in complementary areas for ground-to-air association [333]–[338].

## C. Map Management and Storage Requirements

Scalability in terms of storage requirements is one of the main issues every autonomous system needs to address within the long-term mapping. In dense maps, in which every image is considered as a node in the topological graph, the loop closure database increases linearly with the number of images [90], [106], [141], [271]. Consequently, for long-term operations that imply an extensive collection of images, this task becomes demanding not only to the computational requirements but also the system’s performance. This problem is tackled through map management techniques: 1) using sparse topological maps, representing the environment with fewer nodes which correspond to visually distinct and strategically interesting locations (key-frames), 2) representing each node in a sparse map by a group of sequential and visually similar images, and 3) limiting the map’s size by memory scale discretization.

1) Key-Frame Selection: is based on the detection of scenes visual changes by utilizing methods developed for video compression [339]. However, the main difference between key-frame mapping and video abstraction is that the former requires the query image’s localization with a previously visited location. This is vital for the system’s performance since a single area might be recorded by two different locations [56]. Both locations may reach half of the probability mass, and therefore, neither attracts the threshold for successful data matching. Traditionally, the metric for deciding when to create graph nodes was typically an arbitrary one. Representative examples include the distance and angle between observations in space [79], [92], [264], [340], specific time intervals [265], [341], and a minimum number of tracked landmarks [123], [134], [342]–[344]. An illustration of a map represented by key-frames is shown in Fig. 14.

![](images/2022_LoopClosure-Survey/ea5b268546904e4c1a2dc442173b37ead77ab88b74b13fb0bf609903336e428b.jpg)  
Fig. 14. Illustration of a map represented by key-frames.

2) Representing Each Node in a Sparse Map by a Group of Sequential and Visually Similar Images: is a well-established process that offers computational efficiency while also retaining high spatial accuracy. Techniques that fall into this category map the environment hierarchically [345]–[349] and tackle scalability through the formulation of image groups, thus reducing the database’s search space [58], [59], [73], [135], [350]–[353]. Hierarchies have also been found in the mammalian brain, both in the structure of grid cells in the Hippocampus [354] and the visual cortex’s pathway [355]. To limit the number of database instances, clustering [130], [240], [247], [356]–[358] or pruning [359] methods can be used and restrain map’s parts which exceed a threshold based on the spatial density. Hierarchical approaches follow a two-stage process: firstly, less-intensive nodes are selected, and, next, the most similar view in the chosen node is searched [238], [360]. For instance, in [361] and [184], a hierarchical approach based on color histograms allows the identification of a matching image subset, and subsequently, SIFT features are utilized for acquiring a more precise loop closing frame within this subset. Similarly, nodes are formulated by grouping images with common visual properties, represented by an average global descriptor and a set of binary features through on-line BoW [135]. Korrapati and Mezouar [108] used hierarchical inverted files for indexing images. Exploiting the significant run-time improvements of hierarchical mapping, the authors in [173], [174], [362] achieved real-time performance using learned descriptors.

3) Short-Memory Scale Discretization: limit the map’s size, so that loop closure detection pipelines keep a processing complexity under a fixed time constrain and satisfy the on-line requirements in long-term operations [127]. Mobile robots have limited computational resources; therefore, the map must be somewhat forgotten [127], [138], [363]–[365]. Nevertheless, this needs ignoring of locations, a technique that leads to mismatches in future missions. On the contrary, maintaining in random access memory the entire robot’s visual history is also sub-optimal and, in some cases, not possible. Dayoub and Duckett [363] mapped the environment by using reference views, i.e., many known points. Two specific memory time scales are included in every view: a short-term and a longterm. Frequently observed features belonging in the short-term memory advance to the long-term memory, while the ones not frequently observed are forgotten. They showed that the query view presented a higher similarity to these reference views for nine weeks [366]. Following a similar process, real-time appearance-based mapping (RTAB-MAP) [127] used short-term and long-term memory, while the authors in [246] assumed a system that includes working memory and an indexing scheme built upon the coreset streaming tree [367]. The method in [228] encoded regularly repeating visual patterns in the environment, and the management of an incremental visual vocabulary was presented in [138] based on the repetition of tracked features.

## D. Computational Complexity

In contrast to computer vision benchmarks, wherein the recognition accuracy constitutes the most crucial metric regarding performance measurement, robotics depends on flexible algorithms that can perform robustly under certain real-time restrictions. As most visual loop closure detection solutions share the concepts of feature extraction, memorization, and matching, storage and computational costs, which increase drastically with the environment size, constitute such systems’ weaknesses [66], [111], [142]. Given the map management strategies mentioned in Section IX-C for largescale operations, the main constraints to overcome are the visual information storage and the complexity of similarity computations. If one were to take the naïve approach of using an exhaustive nearest neighbor search and directly comparing all the visual features of the current robot view with all of those observed so far, the complexity of the approach would become impractical. This is due to the comparisons performed for images that do not exhibit the same context. This gets progressively less feasible as the run-time is analogous to the size of previously seen locations. Therefore, compact representations [239], [368] and hashing methods [240], [369], [370] have been explored, apart from data structure-based retrieval techniques, e.g., trees [371]–[375] and graphs [135], [376]–[379].

As the computational time of feature matching varies according to the visual feature’s length, encoding the data into compact representations reduces the storage cost and simultaneously accelerates the similarity computations [380], [381]. Using the most discriminant information in highdimensional data, Liu and Zhang [99] performed loop closure detection based on a PCA technique. They achieved to reduce the descriptor space from 960 dimensions to the 60 most discriminative ones while preserving high accuracy. Another line of frameworks adopted binary descriptors to improve computational efficiency [141] or encoded the high-dimensional vectors into compact codes, such as hashing [382]. Typical matching techniques include hashing, e.g., locality sensitive hashing (LSH) [383] or semantic hashing [384]. Although LSH does not need any pre-processing or off-line procedures [240], [370], [385]–[388], its discrete feature representations suffer from data collisions when their size is large [389]. Nevertheless, with a view to avoid data collision and achieve unique mapping, visual information is embedded in continuous instead of discrete lower-dimensional spaces [390].

![](images/2022_LoopClosure-Survey/6e38d8bb1fe941fb55e0c6ef85bd4a1393c286d88ce24d01e44cfb9f01bb62d6.jpg)  
Fig. 15. The structure of a hierarchical visual vocabulary tree used in off-line visual bag of words pipelines [392]. Instead of searching the whole vocabulary to identify the most similar visual word, incoming local feature descriptors traverse the tree significantly reducing the required computations.

Avoiding dimensionality reduction or binary feature vectors, many pipelines were based on GPU-enabled techniques to close loops in real-time with high efficiency [169], [248], [391].

Nister and Stewenius improved the indexing scheme of the off-line visual BoW through a vocabulary tree generated via hierarchical k-means clustering [392], as depicted in Fig. 15. This way, faster indexing was achieved, while high performances were preserved [206], [239], [342], [343]. Subsequent works were based on spatial data structures [393] and agglomerative clustering [357]. The inverted multi-index [394] and different tree structures, e.g., k-d trees [395], randomized k-d forests [396], [397], Chow Liu trees [398], decision trees [399], BK tree [400]. More specifically, data structures, such as pyramid matching [401], [402], were used to detect loop closures when high dimensional image descriptors were adopted [365], [396]. Furthermore, approaches based on the randomized k-d forest [76], [79], [90], [110], [119], [373] were shown to perform better than a single k-d [70] or a Chow Liu tree [106]. It is worth noting that k-d trees are unsuitable when incremental visual vocabularies are selected since they become unbalanced if new descriptors are added after their construction [119]. Yet, this issue is avoided in off-line BoW models since their vocabulary is built a priori, and there is no other time-consuming module regardless of how large the map becomes.

Finally, although impressive outcomes have been achieved by utilizing deep learning, such approaches are yet computationally costly [164]. Increasing the network’s size results in more computations and storage consumption at the time of training and testing. However, efforts to reduce their complexity do exist [213]. To bridge the research gap between learned features and their complexity, a CNN architecture employing a small number of layers pre-trained on the scenecentric [403] database reduced the computational and memory costs [153]. Similarly, the authors in [217] compressed the learned features’ unnecessary data into a tractable number of bits for robust and efficient place recognition.

## X. CONCLUSION

Loop closure detection is one of SLAM’s most challenging research topics, as it permits consistent map generation and rectification. In this work, we place this problem under a survey focusing on approaches that utilize the camera sensor’s input as their primary perception modality. This article revisited the related literature from the topic’s early years, where most works incorporated hand-crafted techniques for representing the incoming images, to modern approaches and trends that utilize CNNs to represent the incoming frames. The paper at hand follows a tutorial-based structure describing each of the main parts needed for a visual loop closure detection pipeline to facilitate the newcomers in this area. In addition, a complete listing of the datasets and their features was analytically apposed, while the evaluation metrics were discussed in detail. Closing this survey, the authors wish to note that much effort has been put to produce efficient and robust methods to obtain accurate and consistent maps since the first loop closure detection system. Nonetheless, SLAM and its components remain at the frontline of research, with autonomous robots and driverless cars still evolving. Towards robust map generation and localization, SLAM is able to adopt semantic information regarding the explored environment [404]. For loop closure detection to adapt in such a framework, humancentered semantics of the environment need to be incorporated into its mechanisms. In such a way, long-term autonomy can be facilitated since contextual information allows for a broader hierarchy for organizing visual knowledge. Summarizing the above, future research directions include

the development of visual loop closure detection pipelines that operate in dynamic environments which include changing conditions and dynamic scenes;

performance improvements for severe viewpoint variations;

• <sup>improvements</sup> <sup>along</sup> <sup>the</sup> <sup>database’s</sup> <sup>management</sup> <sup>in</sup> <sup>order</sup> to facilitate long-term mapping.

## ACKNOWLEDGMENT

The authors would like to thank the IEEE TRANSAC-TIONS ON INTELLIGENT TRANSPORTATION SYSTEMS editorial team, including the editor-in-chief Azim Eskandarian, an associate editor, and all the reviewers, for their prompt, valuable, and constructive feedback, which largely improved the quality of this article.

## REFERENCES

[1] B. Stewart, J. Ko, D. Fox, and K. Konolige, “The revisiting problem in mobile robot map building: A hierarchical Bayesian approach,” in Proc. 19th Conf. Uncertainty Artif. Intell., 2002, pp. 551–558.

[2] M. Smith, I. Baldwin, W. Churchill, R. Paul, and P. Newman, “The new college vision and laser data set,” Int. J. Robot. Res., vol. 28, no. 5, pp. 595–599, May 2009.

[3] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardós, “ORB-SLAM: A versatile and accurate monocular SLAM system,” IEEE Trans. Robot., vol. 31, no. 5, pp. 1147–1163, Oct. 2015.

[4] B. Williams, M. Cummins, J. Neira, P. Newman, I. Reid, and J. Tardós, “A comparison of loop closing techniques in monocular SLAM,” Robot. Auton. Syst., vol. 57, no. 12, pp. 1188–1197, Dec. 2009.

[5] E. Garcia-Fidalgo and A. Ortiz, “Vision-based topological mapping and localization methods: A survey,” Robot. Auton. Syst., vol. 64, pp. 1–20, Feb. 2015.

[6] S. Garg et al., “Semantics for robotic mapping, perception and interaction: A survey,” Found. Trends Robot., vol. 8, nos. 1–2, pp. 1–224, 2020.

[7] S. Lowry et al., “Visual place recognition: A survey,” IEEE Trans. Robot., vol. 32, no. 1, pp. 1–19, Feb. 2016.

[8] X. Zhang, L. Wang, and Y. Su, “Visual place recognition: A survey from deep learning perspective,” Pattern Recognit., vol. 113, May 2021, Art. no. 107760.

[9] C. Masone and B. Caputo, “A survey on deep visual place recognition,” IEEE Access, vol. 9, pp. 19516–19547, 2021.

[10] S. Garg, T. Fischer, and M. Milford, “Where is your place, visual place recognition?” in Proc. 30th Int. Joint Conf. Artif. Intell., Aug. 2021, pp. 4416–4425.

[11] C. Chen and H. Wang, “Appearance-based topological Bayesian inference for loop-closing detection in a cross-country environment,” Int. J. Robot. Res., vol. 25, no. 10, pp. 953–983, Oct. 2006.

[12] Y. N. Kim, D. W. Ko, and I. H. Suh, “Visual navigation using place recognition with visual line words,” in Proc. 11th Int. Conf. Ubiquitous Robots Ambient Intell. (URAI), Nov. 2014, p. 676.

[13] B. Ferrarini, M. Waheed, S. Waheed, S. Ehsan, M. Milford, and K. D. McDonald-Maier, “Visual place recognition for aerial robotics: Exploring accuracy-computation trade-off for local image descriptors,” in Proc. NASA/ESA Conf. Adapt. Hardw. Syst. (AHS), Jul. 2019, pp. 103–108.

[14] E. Ackerman. (2014). Dyson’s Robot Vacuum Has 360-Degree Camera, Tank Treads, Cyclone Suction. [Online]. Available: http://spectrum.ieee.org/automaton/robotics/home-robots/dysonthe-360-eye-robot-vacuum

[15] M. Cummins and P. Newman, “Probabilistic appearance based navigation and loop closing,” in Proc. IEEE Int. Conf. Robot. Automat., Apr. 2007, pp. 2042–2048.

[16] P. Newman and K. Ho, “SLAM-loop closing with visually salient features,” in Proc. IEEE Int. Conf. Robot. Automat., Apr. 2005, pp. 635–642.

[17] Y. LeCun, Y. Bengio, and G. Hinton, “Deep learning,” Nature, vol. 521, no. 7553, pp. 436–444, 2015.

[18] I. Goodfellow, Y. Bengio, and A. Courville, Deep Learning, vol. 1, no. 2. Cambridge, MA, USA: MIT Press, 2016.

[19] C. Kenshimov, L. Bampis, B. Amirgaliyev, M. Arslanov, and A. Gasteratos, “Deep learning features exception for cross-season visual place recognition,” Pattern Recognit. Lett., vol. 100, pp. 124–130, Dec. 2017.

[20] F. Maffra, L. Teixeira, Z. Chen, and M. Chli, “Real-time wide-baseline place recognition using depth completion,” IEEE Robot. Autom. Lett., vol. 4, no. 2, pp. 1525–1532, Jan. 2019.

[21] R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision. Cambridge, U.K.: Cambridge Univ. Press, 2003.

[22] F. Capezio, F. Mastrogiovanni, A. Sgorbissa, and R. Zaccaria, “Robotassisted surveillance in large environments,” J. Comput. Inf. Technol., vol. 17, no. 1, pp. 95–108, 2009.

[23] Y. Baudoin et al., “View-finder: Robotics assistance to fire-fighting services and crisis management,” in Proc. IEEE Int. Workshop Saf., Secur. Rescue Robot., Aug. 2009, pp. 1–6.

[24] I. Kostavelis et al., “SPARTAN: Developing a vision system for future autonomous space exploration robots,” J. Field Robot., vol. 31, no. 1, pp. 107–140, Jan. 2014.

[25] E. Boukas, A. Gasteratos, and G. Visentin, “Introducing a globally consistent orbital-based localization system,” J. Field Robot., vol. 35, no. 2, pp. 275–298, Mar. 2018.

[26] M. Jiang et al., “Underwater loop-closure detection for mechanical scanning imaging sonar by filtering the similarity matrix with probability hypothesis density filter,” IEEE Access, vol. 7, pp. 166614–166628, 2019.

[27] N. Muhammad, J. F. Fuentes-Perez, J. A. Tuhtan, G. Toming, M. Musall, and M. Kruusmaa, “Map-based localization and loopclosure detection from a moving underwater platform using flow features,” Auton. Robots, vol. 43, no. 6, pp. 1419–1434, Aug. 2019.

[28] K. L. Ho and P. Newman, “Loop closure detection in SLAM by combining visual and spatial appearance,” Robot. Auton. Syst., vol. 54, no. 9, pp. 740–749, 2006.

[29] J. O’Keefe and D. H. Conway, “Hippocampal place units in the freely moving rat: Why they fire where they fire,” Exp. Brain Res., vol. 31, no. 4, pp. 573–590, Apr. 1978.

[30] T. Hafting, M. Fyhn, S. Molden, M.-B. Moser, and E. I. Moser, “Microstructure of a spatial map in the entorhinal cortex,” Nature, vol. 436, no. 7052, pp. 801–806, Jun. 2005.

[31] L. M. Giocomo, E. A. Zilli, E. Fransén, and M. E. Hasselmo, “Temporal frequency of subthreshold oscillations scales with entorhinal grid cell field spacing,” Science, vol. 315, no. 5819, pp. 1719–1722, Mar. 2007.

[32] E. I. Moser, E. Kropff, and M.-B. Moser, “Place cells, grid cells, and the brain’s spatial representation system,” Annu. Rev. Neurosci., vol. 31, no. 1, pp. 69–89, Jul. 2008.

[33] R. Szeliski, Computer Vision: Algorithms and Applications. Springer, 2010.

[34] F. Engelmann, K. Rematas, B. Leibe, and V. Ferrari, “From points to multi-object 3D reconstruction,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2021, pp. 4588–4597.

[35] S. Weder, J. Schonberger, M. Pollefeys, and M. R. Oswald, “RoutedFusion: Learning real-time depth map fusion,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2020, pp. 4887–4897.

[36] D. Liu, C. Long, H. Zhang, H. Yu, X. Dong, and C. Xiao, “ARShadowGAN: Shadow generative adversarial network for augmented reality in single light scenes,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2020, pp. 8139–8148.

[37] J. Wang et al., “Deep two-view structure-from-motion revisited,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2021, pp. 8953–8962.

[38] M. Zaffar et al., “VPR-bench: An open-source visual place recognition evaluation framework with quantifiable viewpoint and appearance change,” Int. J. Comput. Vis., vol. 129, no. 7, pp. 2136–2174, Jul. 2021.

[39] C. Cadena et al., “Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1309–1332, Dec. 2016.

[40] S. Thrun, W. Burgard, and D. Fox, “A probabilistic approach to concurrent mapping and localization for mobile robots,” Auton. Robots, vol. 5, nos. 3–4, pp. 253–271, 1998.

[41] G. H. Lee, F. Fraundorfer, and M. Pollefeys, “Robust pose-graph loopclosures with expectation-maximization,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Nov. 2013, pp. 556–563.

[42] L. Xie, S. Wang, A. Markham, and N. Trigoni, “GraphTinker: Outlier rejection and inlier injection for pose graph SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2017, pp. 6777–6784.

[43] A. Rosinol, M. Abate, Y. Chang, and L. Carlone, “Kimera: An opensource library for real-time metric-semantic localization and mapping,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 1689–1696.

[44] M. Xu, N. Snderhauf, and M. Milford, “Probabilistic visual place recognition for hierarchical localization,” IEEE Robot. Autom. Lett., vol. 6, no. 2, pp. 311–318, Apr. 2021.

[45] M. Xu, T. Fischer, N. Sunderhauf, and M. Milford, “Probabilistic appearance-invariant topometric localization with new place awareness,” IEEE Robot. Autom. Lett., vol. 6, no. 4, pp. 6985–6992, Oct. 2021.

[46] A. Angeli, S. Doncieux, J.-A. Meyer, and D. Filliat, “Real-time visual loop-closure detection,” in Proc. IEEE Int. Conf. Robot. Automat., May 2008, pp. 1842–1847.

[47] J. Röwekämper, C. Sprunk, G. D. Tipaldi, C. Stachniss, P. Pfaff, and W. Burgard, “On the position accuracy of mobile robot localization based on particle filters combined with scan matching,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2012, pp. 3158–3164.

[48] A. J. Davison, I. D. Reid, N. D. Molton, and O. Stasse, “MonoSLAM: Real-time single camera SLAM,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 29, no. 6, pp. 1052–1067, Jun. 2007.

[49] E. C. Tolman, “Cognitive maps in rats and men,” Psychol. Rev., vol. 55, no. 4, p. 189, 1948.

[50] F. Strumwasser, “Long-term recording from single neurons in brain of unrestrained mammals,” Science, vol. 127, no. 3296, pp. 469–470, Feb. 1958.

[51] J. O’Keefe and J. Doslrovskv, “The hippocampus as a spatial map. Preliminary evidence from unit activity in the freely-moving rat,” Brain Res., vol. 34, no. 1, pp. 171–175, 1971.

[52] B. Kuipers and Y.-T. Byun, “A robust qualitative method for spatial learning in unknown environments,” in Proc. 7th AAAI Nat. Conf. Artif. Intell., 1988, pp. 774–779.

[53] M. O. Franz, B. Schölkopf, H. A. Mallot, and H. H. Bülthoff, “Learning view graphs for robot navigation,” Auton. Robots, vol. 5, no. 1, pp. 111–125, 1998.

[54] H. Choset and K. Nagatani, “Topological simultaneous localization and mapping (SLAM): Toward exact localization without explicit localization,” IEEE Trans. Robot. Autom., vol. 17, no. 2, pp. 125–137, Apr. 2001.

[55] A. Ranganathan and F. Dellaert, “Online probabilistic topological mapping,” Int. J. Robot. Res., vol. 30, no. 6, pp. 755–771, May 2011.

[56] E. D. Eade and T. W. Drummond, “Unified loop closing and recovery for real time monocular SLAM,” in Proc. Brit. Mach. Vis. Conf., vol. 13, 2008, p. 136.

[57] B. Kuipers, “Modeling spatial knowledge,” Cogn. Sci., vol. 2, no. 2, pp. 129–153, 1978.

[58] Z. Chen, A. Jacobson, U. M. Erdem, M. E. Hasselmo, and M. Milford, “Multi-scale bio-inspired place recognition,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2014, pp. 1895–1901.

[59] Z. Chen, S. Lowry, A. Jacobson, M. E. Hasselmo, and M. Milford, “Bio-inspired homogeneous multi-scale place recognition,” Neural Netw., vol. 72, pp. 48–61, Dec. 2015.

[60] I. Kostavelis, K. Charalampous, A. Gasteratos, and J. K. Tsotsos, “Robot navigation via spatial and temporal coherent semantic maps,” Eng. Appl. Artif. Intell., vol. 48, pp. 173–187, Feb. 2016.

[61] J. Borenstein and L. Feng, “Measurement and correction of systematic odometry errors in mobile robots,” IEEE Trans. Robot. Autom., vol. 12, no. 6, pp. 869–880, Dec. 1996.

[62] C. McManus, P. Furgale, and T. D. Barfoot, “Towards lighting-invariant visual navigation: An appearance-based approach using scanning laserrangefinders,” Robot. Auton. Syst., vol. 61, no. 8, pp. 836–852, 2013.

[63] K. A. Tsintotas, L. Bampis, A. Taitzoglou, I. Kansizoglou, and A. Gasteratos, “Safe UAV landing: A low-complexity pipeline for surface conditions recognition,” in Proc. IEEE Int. Conf. Imag. Syst. Techn. (IST), Aug. 2021, pp. 1–6.

[64] M. Magnusson, H. Andreasson, A. Nuchter, and A. J. Lilienthal, “Appearance-based loop detection from 3D laser data using the normal distributions transform,” in Proc. IEEE Int. Conf. Robot. Automat., May 2009, pp. 23–28.

[65] M. Bosse and R. Zlot, “Keypoint design and evaluation for place recognition in 2D LiDAR maps,” Robot. Auton. Syst., vol. 57, no. 12, pp. 1211–1224, Dec. 2009.

[66] M. Bosse and R. Zlot, “Place recognition using keypoint voting in large 3D LiDAR datasets,” in Proc. IEEE Int. Conf. Robot. Automat., May 2013, pp. 2677–2684.

[67] D. Hahnel, W. Burgard, D. Fox, and S. Thrun, “An efficient fastSLAM algorithm for generating maps of large-scale cyclic environments from raw laser range measurements,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2003, pp. 206–211.

[68] W. Burgard, C. Stachniss, and D. Hähnel, “Mobile robot map learning from range data in dynamic environments,” in Autonomous Navigation in Dynamic Environments. Berlin, Germany: Springer-Verlag, 2007, pp. 3–28.

[69] D. Cattaneo, M. Vaghi, S. Fontana, A. L. Ballardini, and D. G. Sorrenti, “Global visual localization in LiDAR-maps through shared 2D-3D embedding space,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 4365–4371.

[70] M. J. Milford, G. F. Wyeth, and D. Prasser, “RatSLAM: A hippocampal model for simultaneous localization and mapping,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), vol. 1, Apr. 2004, pp. 403–408.

[71] P. Newman, D. Cole, and K. Ho, “Outdoor SLAM using visual appearance and laser ranging,” in Proc. IEEE Int. Conf. Robot. Automat., May 2006, pp. 1180–1187.

[72] F. Fraundorfer, C. Engels, and D. Nistér, “Topological mapping, localization and navigation using image collections,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2007, pp. 3872–3877.

[73] L. A. Clemente, A. J. Davison, I. D. Reid, J. Neira, and J. D. Tardós, “Mapping large loops with a single hand-held camera,” in Proc. Robot., Sci. Syst., 2007, pp. 1–7.

[74] K. Pirker, M. Rüther, and H. Bischof, “CD SLAM–continuous localization and mapping in a dynamic world,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2011, pp. 3990–3997.

[75] S. Vidas and W. Maddern, “Towards robust night and day place recognition using visible and thermal imaging,” in Proc. Robot. Sci. Syst., 2012, pp. 1–6.

[76] W. Tan, H. Liu, Z. Dong, G. Zhang, and H. Bao, “Robust monocular SLAM in dynamic environments,” in Proc. IEEE Int. Symp. Mixed Augmented Reality (ISMAR), Oct. 2013, pp. 209–218.

[77] K. Konolige and M. Agrawal, “FrameSLAM: From bundle adjustment to real-time visual mapping,” IEEE Trans. Robot., vol. 24, no. 5, pp. 1066–1077, Oct. 2008.

[78] P. Newman et al., “Navigating, recognizing and describing urban spaces with vision and lasers,” Int. J. Robot. Res., vol. 28, nos. 11–12, pp. 1406–1433, Nov. 2009.

[79] C. Mei, G. Sibley, M. Cummins, P. Newman, and I. Reid, “A constanttime efficient stereo SLAM system,” in Proc. Brit. Mach. Vis. Conf., 2009, pp. 54.1–54.11.

[80] G. Sibley, C. Mei, I. Reid, and P. Newman, “Vast-scale outdoor navigation using adaptive relative bundle adjustment,” Int. J. Robot. Res., vol. 29, no. 8, pp. 958–980, Jul. 2010.

[81] L. Nalpantidis, G. C. Sirakoulis, and A. Gasteratos, “Non-probabilistic cellular automata-enhanced stereo vision simultaneous localization and mapping,” Meas. Sci. Technol., vol. 22, no. 11, Nov. 2011, Art. no. 114027.

[82] C. Cadena, D. Gálvez-López, J. D. Tardós, and J. Neira, “Robust place recognition with stereo sequences,” IEEE Trans. Robot., vol. 28, no. 4, pp. 871–885, Aug. 2012.

[83] T. Mitroudas, K. A. Tsintotas, N. Santavas, A. Psomoulis, and A. Gasteratos, “Towards 3D printed modular unmanned aerial vehicle development: The safety landing paradigm,” in Proc. IEEE Int. Conf. Imag. Syst. Techn., Jun. 2022, pp. 1–6.

[84] R. Paul and P. Newman, “FAB-MAP 3D: Topological mapping with spatial and visual appearance,” in Proc. IEEE Int. Conf. Robot. Automat., May 2010, pp. 2649–2656.

[85] J. Collier, S. Se, and V. Kotamraju, “Multi-sensor appearance-based place recognition,” in Proc. Int. Conf. Comput. Robot Vis., May 2013, pp. 128–135.

[86] E. Pepperell, P. I. Corke, and M. J. Milford, “All-environment visual place recognition with SMART,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2014, pp. 1612–1618.

[87] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, “Keyframe-based visual-inertial odometry using nonlinear optimization,” Int. J. Robot. Res., vol. 34, no. 3, pp. 314–334, 2015.

[88] S. Hausler, A. Jacobson, and M. Milford, “Multi-process fusion: Visual place recognition using multiple image processing methods,” IEEE Robot. Autom. Lett., vol. 4, no. 2, pp. 1924–1931, Feb. 2019.

[89] H. Badino, D. Huber, and T. Kanade, “Real-time topometric localization,” in Proc. IEEE Int. Conf. Robot. Automat., May 2012, pp. 1635–1642.

[90] M. Cummins and P. Newman, “Appearance-only SLAM at large scale with FAB-MAP 2.0,” Int. J. Robot. Res., vol. 30, no. 9, pp. 1100–1123, Jun. 2011.

[91] P. Henry, M. Krainin, E. Herbst, X. Ren, and D. Fox, “RGB-D mapping: Using depth cameras for dense 3D modeling of indoor environments,” in Proc. 12th Int. Symp. Exp. Robot., 2014, pp. 477–491.

[92] T. Whelan, M. Kaess, J. J. Leonard, and J. McDonald, “Deformationbased loop closure for large scale dense RGB-D SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Nov. 2013, pp. 548–555.

[93] T. Whelan, M. Kaess, H. Johannsson, M. Fallon, J. J. Leonard, and J. McDonald, “Real-time large-scale dense RGB-D SLAM with volumetric fusion,” Int. J. Robot. Res., vol. 34, nos. 4–5, pp. 598–626, 2015.

[94] R. Finman, L. Paull, and J. J. Leonard, “Toward object-based place recognition in dense RGB-D maps,” in Proc. IEEE Int. Conf. Robot. Automat., vol. 76, May 2015, pp. 1–8.

[95] M. Milford, H. Kim, S. Leutenegger, and A. Davison, “Towards visual SLAM with event-based cameras,” in Proc. Robot. Sci. Syst., 2015, pp. 1–8.

[96] T. Fischer and M. Milford, “Event-based visual place recognition with ensembles of temporal windows,” IEEE Robot. Autom. Lett., vol. 5, no. 4, pp. 6924–6931, Oct. 2020.

[97] A. C. Murillo and J. Kosecka, “Experiments in place recognition using gist panoramas,” in Proc. IEEE 12th Int. Conf. Comput. Vis. Workshops (ICCV Workshops), Sep. 2009, pp. 2196–2203.

[98] G. Singh and J. Kosecka, “Visual loop closing using gist descriptors in Manhattan world,” in Proc. IEEE IEEE Int. Conf. Robot. Workshop, Oct. 2010, pp. 4042–4047.

[99] Y. Liu and H. Zhang, “Visual loop closure detection with a compact image descriptor,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2012, pp. 1051–1056.

[100] S. M. A. M. Kazmi and B. Mertsching, “Detecting the expectancy of a place using nearby context for appearance-based mapping,” IEEE Trans. Robot., vol. 35, no. 6, pp. 1352–1366, Dec. 2019.

[101] N. Sünderhauf and P. Protzel, “Brief-gist-closing the loop by simple means,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2011, pp. 1234–1241.

[102] X. Yang and K.-T. Cheng, “LDB: An ultra-fast feature for scalable augmented reality on mobile devices,” in Proc. IEEE Int. Symp. Mixed Augmented Reality (ISMAR), Nov. 2012, pp. 49–57.

[103] X. Yang and K.-T. Cheng, “Local difference binary for ultrafast and distinctive feature description,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 36, no. 1, pp. 188–194, Jan. 2014.

[104] C. Mcmanus, B. Upcroft, and P. Newmann, “Scene signatures: Localised and point-less features for localisation,” in Proc. Robot., Sci. Syst. X, Jul. 2014, pp. 1–9.

[105] G. Dudek and D. Jugessur, “Robust place recognition using local appearance based methods,” in Proc. IEEE Int. Conf. Robot. Automat., Apr. 2000, pp. 1030–1035.

[106] M. Cummins and P. Newman, “FAB-MAP: Probabilistic localization and mapping in the space of appearance,” Int. J. Robot. Res., vol. 27, no. 6, pp. 647–665, 2008.

[107] M. Cummins and P. Newman, “Accelerated appearance-only SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., May 2008, pp. 1828–1833.

[108] H. Korrapati and Y. Mezouar, “Vision-based sparse topological mapping,” Robot. Auton. Syst., vol. 62, no. 9, pp. 1259–1270, 2014.

[109] E. Johns and G.-Z. Yang, “Feature co-occurrence maps: Appearancebased localisation throughout the day,” in Proc. IEEE Int. Conf. Robot. Automat., May 2013, pp. 3212–3218.

[110] S. M. Siam and H. Zhang, “Fast-SeqSLAM: A fast appearance based place recognition algorithm,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2017, pp. 5702–5708.

[111] M. J. Milford and G. F. Wyeth, “SeqSLAM: Visual route-based navigation for sunny summer days and stormy winter nights,” in Proc. IEEE Int. Conf. Robot. Automat., May 2012, pp. 1643–1649.

[112] K. A. Tsintotas, L. Bampis, S. Rallis, and A. Gasteratos, “SeqSLAM with bag of visual words for appearance based loop closure detection,” in Proc. Int. Conf. Robot. Alpe-Adria Danube Reg., 2018, pp. 580–587.

[113] K. A. Tsintotas, L. Bampis, and A. Gasteratos, “DOSeqSLAM: Dynamic on-line sequence based loop closure detection algorithm for SLAM,” in Proc. IEEE Int. Conf. Imag. Syst. Techn. (IST), Oct. 2018, pp. 1–6.

[114] K. A. Tsintotas, L. Bampis, A. Gasteratos, and Fiet, “Tracking-DOSeqSLAM: A dynamic sequence-based visual place recognition paradigm,” IET Comput. Vis., vol. 15, no. 4, pp. 258–273, Jun. 2021.

[115] L. Bampis, A. Amanatiadis, and A. Gasteratos, “Encoding the description of image sequences: A two-layered pipeline for loop closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2016, pp. 4530–4536.

[116] L. Bampis, A. Amanatiadis, and A. Gasteratos, “High order visual words for structure-aware and viewpoint-invariant loop closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2017, pp. 4268–4275.

[117] P. Turcot and D. G. Lowe, “Better matching with fewer features: The selection of useful features in large database recognition problems,” in Proc. IEEE 12th Int. Conf. Comput. Vis. Workshops (ICCV Workshops), Sep. 2009, pp. 2109–2116.

[118] H. Zhang, “BoRF: Loop-closure detection with scale invariant visual features,” in Proc. IEEE Int. Conf. Robot. Automat., May 2011, pp. 3125–3130.

[119] S. Lynen, M. Bosse, P. Furgale, and R. Siegwart, “Placeless placerecognition,” in Proc. 2nd Int. Conf. 3D Vis., Dec. 2014, pp. 303–310.

[120] D. Filliat, “A visual bag of words method for interactive qualitative localization and mapping,” in Proc. IEEE Int. Conf. Robot. Automat., Apr. 2007, pp. 3921–3926.

[121] A. Angeli, D. Filliat, S. Doncieux, and J.-A. Meyer, “Fast and incremental method for loop-closure detection using bags of visual words,” IEEE Trans. Robot., vol. 24, no. 5, pp. 1027–1037, Oct. 2008.

[122] T. Nicosevici and R. Garcia, “On-line visual vocabularies for robot navigation and mapping,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2009, pp. 205–212.

[123] H. Zhang, B. Li, and D. Yang, “Keyframe detection for appearancebased visual SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2010, pp. 2071–2076.

[124] Y. Girdhar and G. Dudek, “Online visual vocabularies,” in Proc. Can. Conf. Comput. Robot Vis., May 2011, pp. 191–196.

[125] A. Kawewong, N. Tongprasit, S. Tangruamsub, and O. Hasegawa, “Online and incremental appearance-based SLAM in highly dynamic environments,” Int. J. Robot. Res., vol. 30, no. 1, pp. 33–55, Jan. 2011.

[126] T. Nicosevici and R. Garcia, “Automatic visual bag-of-words for online robot navigation and mapping,” IEEE Trans. Robot., vol. 28, no. 4, pp. 886–898, Aug. 2012.

[127] M. Labbé and F. Michaud, “Appearance-based loop closure detection for online large-scale and long-term operation,” IEEE Trans. Robot., vol. 29, no. 3, pp. 734–745, Jun. 2013.

[128] Y. Latif, G. Huang, J. J. Leonard, and J. Neira, “An online sparsitycognizant loop-closure algorithm for visual navigation,” in Proc. Robot., Sci. Syst., 2014, pp. 1–9.

[129] M. Gehrig, E. Stumm, T. Hinzmann, and R. Siegwart, “Visual place recognition with probabilistic voting,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2017, pp. 3192–3199.

[130] K. A. Tsintotas, L. Bampis, and A. Gasteratos, “Assigning visual words to places for loop closure detection,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2018, pp. 5979–5985.

[131] E. Garcia-Fidalgo and A. Ortiz, “On the use of binary feature descriptors for loop closure detection,” in Proc. IEEE Emerg. Technol. Factory Automat. (ETFA), Sep. 2014, pp. 1–8.

[132] S. Khan and D. Wollherr, “IBuILD: Incremental bag of binary words for appearance based loop closure detection,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2015, pp. 5441–5447.

[133] G. Zhang, M. J. Lilly, and P. A. Vela, “Learning binary features online from motion dynamics for incremental loop-closure detection and place recognition,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2016, pp. 765–772.

[134] T. Cieslewski, E. Stumm, A. Gawel, M. Bosse, S. Lynen, and R. Siegwart, “Point cloud descriptors for place recognition using sparse visual information,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2016, pp. 4830–4836.

[135] E. Garcia-Fidalgo and A. Ortiz, “Hierarchical place recognition for topological mapping,” IEEE Trans. Robot., vol. 33, no. 5, pp. 1061–1074, Jun. 2017.

[136] K. A. Tsintotas, P. Giannis, L. Bampis, and A. Gasteratos, “Appearance-based loop closure detection with scale-restrictive visual features,” in Proc. Int. Conf. Comput. Vis. Syst., 2019, pp. 75–87.

[137] K. A. Tsintotas, L. Bampis, and A. Gasteratos, “Probabilistic appearance-based place recognition through bag of tracked words,” IEEE Robot. Autom. Lett., vol. 4, no. 2, pp. 1737–1744, Apr. 2019.

[138] K. A. Tsintotas, L. Bampis, and A. Gasteratos, “Modest-vocabulary loop-closure detection with incremental bag of tracked words,” Robot. Auton. Syst., vol. 141, Jul. 2021, Art. no. 103782.

[139] I. T. Papapetros, V. Balaska, and A. Gasteratos, “Visual loop-closure detection via prominent feature tracking,” J. Intell. Robot. Syst., vol. 104, no. 3, pp. 1–13, Mar. 2022.

[140] K. L. Ho and P. Newman, “Detecting loop closure with scene sequences,” Int. J. Comput. Vis., vol. 74, no. 3, pp. 261–286, Sep. 2007.

[141] D. Gálvez-López and J. D. Tardos, “Bags of binary words for fast place recognition in image sequences,” IEEE Trans. Robot., vol. 28, no. 5, pp. 1188–1197, Oct. 2012.

[142] W. Maddern, M. Milford, and G. Wyeth, “CAT-SLAM: Probabilistic localisation and mapping using a continuous appearancebased trajectory,” Int. J. Robot. Res., vol. 31, no. 4, pp. 429–451, Apr. 2012.

[143] L. Bampis, A. Amanatiadis, and A. Gasteratos, “Fast loop-closure detection using visual-word-vectors from image sequences,” Int. J. Robot. Res., vol. 37, no. 1, pp. 62–82, Jan. 2018.

[144] K. A. Tsintotas, L. Bampis, S. An, G. F. Fragulis, S. G. Mouroutsos, and A. Gasteratos, “Sequence-based mapping for probabilistic visual loop-closure detection,” in Proc. IEEE Int. Conf. Imag. Syst. Techn. (IST), Aug. 2021, pp. 1–6.

[145] Z. Chen, O. Lam, A. Jacobson, and M. Milford, “Convolutional neural network-based place recognition,” in Proc. Australas. Conf. Robot. Automat., 2014, pp. 1–8.

[146] N. Sunderhauf, S. Shirazi, F. Dayoub, B. Upcroft, and M. Milford, “On the performance of ConvNet features for place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2015, pp. 4297–4304.

[147] Y. Xia, J. Li, L. Qi, and H. Fan, “Loop closure detection for visual SLAM using PCANet features,” in Proc. Int. Joint Conf. Neural Netw. (IJCNN), Jul. 2016, pp. 2274–2281.

[148] X. Zhang, Y. Su, and X. Zhu, “Loop closure detection for visual SLAM systems using convolutional neural network,” in Proc. 23rd Int. Conf. Automat. Comput. (ICAC), Sep. 2017, pp. 1–6.

[149] J. Yu, C. Zhu, J. Zhang, Q. Huang, and D. Tao, “Spatial pyramidenhanced NetVLAD with weighted triplet loss for place recognition,” IEEE Trans. Neural Netw. Learn. Syst., vol. 31, no. 2, pp. 661–674, Feb. 2020.

[150] D. Bai, C. Wang, B. Zhang, X. Yi, and X. Yang, “CNN feature boosted SeqSLAM for real-time loop closure detection,” Chin. J. Electron., vol. 27, no. 3, pp. 488–499, May 2018.

[151] S. Garg, N. Suenderhauf, and M. Milford, “Don’t look back: Robustifying place categorization for viewpoint- and condition-invariant place recognition,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2018, pp. 3645–3652.

[152] D. Bai, C. Wang, B. Zhang, X. Yi, and X. Yang, “Sequence searching with CNN features for robust and fast visual place recognition,” Comput. Graph., vol. 70, pp. 270–280, Feb. 2017.

[153] S. Wang, X. Lv, X. Liu, and D. Ye, “Compressed holistic ConvNet representations for detecting loop closures in dynamic environments,” IEEE Access, vol. 8, pp. 60552–60574, 2020.

[154] F. Rodrigues et al., “Three level sequence-based loop closure detection,” Robot. Auton. Syst., vol. 133, Nov. 2020, Art. no. 103620.

[155] M.-A. Tomia, M. Zaffar, M. J. Milford, K. D. McDonald-Maier, and S. Ehsan, “ConvSequential-SLAM: A sequence-based, trainingless visual place recognition technique for changing environments,” IEEE Access, vol. 9, pp. 118673–118683, 2021.

[156] M. Chancan, L. Hernandez-Nunez, A. Narendra, A. B. Barron, and M. Milford, “A hybrid compact neural architecture for visual place recognition,” IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 993–1000, Apr. 2020.

[157] J. H. Oh, J. D. Jeon, and B. H. Lee, “Place recognition for visual loopclosures using similarities of object graphs,” Electron. Lett., vol. 51, no. 1, pp. 44–46, 2015.

[158] C. Toft, C. Olsson, and F. Kahl, “Long-term 3D localization and pose from semantic labellings,” in Proc. IEEE Int. Conf. Comput. Vis. Workshops (ICCVW), Oct. 2017, pp. 650–659.

[159] X. Yu et al., “VLASE: Vehicle localization by aggregating semantic edges,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2018, pp. 3196–3203.

[160] Y. Hou, H. Zhang, and S. Zhou, “Evaluation of object proposals and ConvNet features for landmark-based visual place recognition,” J. Intell. Robot. Syst., vol. 92, nos. 3–4, pp. 505–520, Dec. 2018.

[161] J. L. Schonberger, M. Pollefeys, A. Geiger, and T. Sattler, “Semantic visual localization,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., Jun. 2018, pp. 6896–6906.

[162] A. Gawel, C. D. Don, R. Siegwart, J. Nieto, and C. Cadena, “X-view: Graph-based semantic multi-view localization,” IEEE Robot. Autom. Lett., vol. 3, no. 3, pp. 1687–1694, Jul. 2018.

[163] A. Benbihi, S. Aravecchia, M. Geist, and C. Pradalier, “Image-based place recognition on bucolic environment across seasons from semantic edge description,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 3032–3038.

[164] Z. Chen, F. Maffra, I. Sa, and M. Chli, “Only look once, mining distinctive landmarks from ConvNet for visual place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2017, pp. 9–16.

[165] Z. Chen, L. Liu, I. Sa, Z. Ge, and M. Chli, “Learning context flexible attention model for long-term visual place recognition,” IEEE Robot. Autom. Lett., vol. 3, no. 4, pp. 4015–4022, Oct. 2018.

[166] A. Khaliq, S. Ehsan, Z. Chen, M. Milford, and K. McDonald-Maier, “A holistic visual place recognition approach using lightweight CNNs for significant ViewPoint and appearance changes,” IEEE Trans. Robot., vol. 36, no. 2, pp. 561–569, Apr. 2020.

[167] L. G. Camara and L. Pˇreuˇcil, “Spatio-semantic ConvNet-based visual place recognition,” in Proc. Eur. Conf. Mobile Robots (ECMR), Sep. 2019, pp. 1–8.

[168] J. M. Facil, D. Olid, L. Montesano, and J. Civera, “Condition-invariant multi-view place recognition,” 2019, arXiv:1902.09516.

[169] S. An, H. Zhu, D. Wei, K. A. Tsintotas, and A. Gasteratos, “Fast and incremental loop closure detection with deep features and proximity graphs,” J. Field Robot., vol. 39, no. 4, pp. 473–493, Jun. 2022.

[170] S. Garg, N. Suenderhauf, and M. Milford, “LoST? Appearanceinvariant place recognition for opposite viewpoints using visual semantics,” in Proc. Robot., Sci. Syst., 2018, pp. 1–11.

[171] S. Garg, B. Harwood, G. Anand, and M. Milford, “Delta descriptors: Change-based place representation for robust visual localization,” IEEE Robot. Autom. Lett., vol. 5, no. 4, pp. 5120–5127, Oct. 2020.

[172] S. Garg and M. Milford, “SeqNet: Learning descriptors for sequencebased hierarchical place recognition,” IEEE Robot. Autom. Lett., vol. 6, no. 3, pp. 4305–4312, Jul. 2021.

[173] P.-E. Sarlin, C. Cadena, R. Siegwart, and M. Dymczyk, “From coarse to fine: Robust hierarchical localization at large scale,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2019, pp. 12716–12725.

[174] B. Cao, A. Araujo, and J. Sim, “Unifying deep local and global features for image search,” in Proc. Eur. Conf. Comput. Vis., 2020, pp. 726–743.

[175] B. Schiele and J. L. Crowley, “Object recognition using multidimensional receptive field histograms,” in Proc. Eur. Conf. Comput. Vis., 1996, pp. 610–619.

[176] H. Bay, T. Tuytelaars, and L. Van Gool, “SURF: Speeded up robust features,” in Proc. Eur. Conf. Comput. Vis., 2006, pp. 404–417.

[177] M. C. Potter, “Meaning in visual search,” Science, vol. 187, no. 4180, pp. 965–966, Mar. 1975.

[178] I. Biederman, “Aspects and extensions of a theory of human image understanding,” in Computational Processes in Human Vision: An Interdisciplinary Perspective. Norwood, NJ, USA: Ablex, 1988, pp. 370–428.

[179] A. Oliva and A. Torralba, “Modeling the shape of the scene: A holistic representation of the spatial envelope,” Int. J. Comput. Vis., vol. 42, no. 3, pp. 145–175, 2001.

[180] A. Torralba, K. P. Murphy, W. T. Freeman, and M. A. Rubin, “Contextbased vision system for place and object recognition,” in Proc. 9th IEEE Int. Conf. Comput. Vis., vol. 3, Oct. 2003, pp. 273–280.

[181] A. Oliva and A. Torralba, “Building the gist of a scene: The role of global image features in recognition,” Prog. Brain Res., vol. 155, pp. 23–36, Oct. 2006.

[182] M. Calonder, V. Lepetit, C. Strecha, and P. Fua, “BRIEF: Binary robust independent elementary features,” in Proc. Eur. Conf. Comput. Vis., 2010, pp. 778–792.

[183] R. Arroyo, P. F. Alcantarilla, L. M. Bergasa, J. J. Yebes, and S. Bronte, “Fast and effective visual place recognition using binary codes and disparity information,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2014, pp. 3089–3094.

[184] L. Maohai, S. Lining, H. Qingcheng, C. Zesu, and P. Songhao, “Robust omnidirectional vision based mobile robot hierarchical localization and autonomous navigation,” Inf. Technol. J., vol. 10, no. 1, pp. 29–39, Dec. 2010.

[185] J. Luo, A. Pronobis, B. Caputo, and P. Jensfelt, “Incremental learning for place recognition in dynamic environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2007, pp. 721–728.

[186] A. Bosch, A. Zisserman, and X. Munoz, “Representing shape with a spatial pyramid kernel,” in Proc. ACM Int. Conf. Image Video Retr., 2007, pp. 401–408.

[187] W.-C. Chiu and M. Fritz, “See the difference: Direct pre-image reconstruction and pose estimation by differentiating HOG,” in Proc. IEEE Int. Conf. Comput. Vis. (ICCV), Dec. 2015, pp. 468–476.

[188] R. Baeza-Yates and B. Ribeiro-Neto, Modern Information Retrieval, vol. 463. New York, NY, USA: ACM Press, 1999.

[189] J. Sivic and A. Zisserman, “Video Google: A text retrieval approach to object matching in videos,” in Proc. 9th IEEE Int. Conf. Comput. Vis., Oct. 2003, p. 1470.

[190] J. MacQueen et al., “Some methods for classification and analysis of multivariate observations,” in Proc. 5th Berkeley Symp. Math. Statist. Probab., 1967, pp. 281–297.

[191] K. Sparck-Jones, “A statistical interpretation of term specificity and its application in retrieval,” J. Document., vol. 28, no. 1, pp. 11–21, 1972.

[192] D. Hiemstra, “A probabilistic justification for using TF IDF term weighting in information retrieval,” Int. J. Digit. Libraries, vol. 3, no. 2, pp. 131–139, Aug. 2000.

[193] F. Perronnin and C. Dance, “Fisher kernels on visual vocabularies for image categorization,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2007, pp. 1–8.

[194] F. Perronnin, J. Sánchez, and T. Mensink, “Improving the Fisher kernel for large-scale image classification,” in Proc. Eur. Conf. Comput. Vis., 2010, pp. 143–156.

[195] H. Jégou, M. Douze, C. Schmid, and P. Pérez, “Aggregating local descriptors into a compact image representation,” in Proc. IEEE Comput. Soc. Conf. Comput. Vis. Pattern Recognit., Jun. 2010, pp. 3304–3311.

[196] R. Arandjelovic and A. Zisserman, “All about VLAD,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2013, pp. 1578–1585.

[197] D. G. Lowe, “Distinctive image features from scale-invariant keypoints,” Int. J. Comput. Vis., vol. 60, no. 2, pp. 91–110, 2004.

[198] M. Agrawal, K. Konolige, and M. R. Blas, “Censure: Center surround extremas for realtime feature detection and matching,” in Proc. Eur. Conf. Comput. Vis., 2008, pp. 102–115.

[199] P. F. Alcantarilla, A. Bartoli, and A. J. Davison, “KAZE features,” in Proc. Eur. Conf. Comput. Vis., 2012, pp. 214–227.

[200] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: An efficient alternative to SIFT or SURF,” in Proc. Int. Conf. Comput. Vis., Nov. 2011, pp. 2564–2571.

[201] S. Leutenegger, M. Chli, and R. Y. Siegwart, “BRISK: Binary robust invariant scalable keypoints,” in Proc. Int. Conf. Comput. Vis., Nov. 2011, pp. 2548–2555.

[202] A. Alahi, R. Ortiz, and P. Vandergheynst, “FREAK: Fast retina keypoint,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2012, pp. 510–517.

[203] P. F. Alcantarilla and T. Solutions, “Fast explicit diffusion for accelerated features in nonlinear scale spaces,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 34, no. 7, pp. 1281–1298, Oct. 2013.

[204] X. S. Zhou and T. S. Huang, “Edge-based structural features for content-based image retrieval,” Pattern Recognit. Lett., vol. 22, no. 5, pp. 457–468, Apr. 2001.

[205] J. P. Company-Corcoles, E. Garcia-Fidalgo, and A. Ortiz, “Towards robust loop closure detection in weakly textured environments using points and lines,” in Proc. 25th IEEE Int. Conf. Emerg. Technol. Factory Automat. (ETFA), Sep. 2020, pp. 1313–1316.

[206] G. Schindler, M. Brown, and R. Szeliski, “City-scale location recognition,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2007, pp. 1–7.

[207] Y. LeCun et al., “Backpropagation applied to handwritten zip code recognition,” Neural Comput., vol. 1, no. 4, pp. 541–551, 1989.

[208] Y. LeCun, L. Bottou, Y. Bengio, and P. Haffner, “Gradient-based learning applied to document recognition,” Proc. IEEE, vol. 86, no. 11, pp. 2278–2324, Nov. 1998.

[209] Z. Liu, L. Zhang, Q. Liu, Y. Yin, L. Cheng, and R. Zimmermann, “Fusion of magnetic and visual sensors for indoor localization: Infrastructure-free and more effective,” IEEE Trans. Multimedia, vol. 19, no. 4, pp. 874–888, Apr. 2017.

[210] A. Krizhevsky, I. Sutskever, and G. E. Hinton, “ImageNet classification with deep convolutional neural networks,” in Proc. Adv. Neural Inf. Process. Syst., vol. 25, 2012, pp. 1097–1105.

[211] W. Maddern, A. Stewart, C. McManus, B. Upcroft, W. Churchill, and P. Newman, “Illumination invariant imaging: Applications in robust vision-based localisation, mapping and classification for autonomous vehicles,” in Proc. IEEE Int. Conf. Robot. Automat., May 2014, p. 3.

[212] A. Babenko, A. Slesarev, A. Chigorin, and V. Lempitsky, “Neural codes for image retrieval,” in Proc. Eur. Conf. Comput. Vis., 2014, pp. 584–599.

[213] F. Radenovi´c, G. Tolias, and O. Chum, “CNN image retrieval learns from BoW: Unsupervised fine-tuning with hard examples,” in Proc. Eur. Conf. Comput. Vis., 2016, pp. 3–20.

[214] D. DeTone, T. Malisiewicz, and A. Rabinovich, “SuperPoint: Selfsupervised interest point detection and description,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. Workshops (CVPRW), Jun. 2018, pp. 224–236.

[215] P. Rolet, M. Sebag, and O. Teytaud, “Integrated recognition, localization and detection using convolutional networks,” in Proc. Eur. Conf. Mach. Learn., 2012, pp. 1255–1263.

[216] R. Arandjelovic, P. Gronat, A. Torii, T. Pajdla, and J. Sivic, “NetVLAD: CNN architecture for weakly supervised place recognition,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2016, pp. 5297–5307.

[217] R. Arroyo, P. F. Alcantarilla, L. M. Bergasa, and E. Romera, “Fusion and binarization of CNN features for robust topological localization across seasons,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2016, pp. 4656–4663.

[218] N. Sünderhauf et al., “Place recognition with convnet landmarks: Viewpoint-robust, condition-robust, training-free,” in Proc. Robot., Sci. Syst., 2015, pp. 1–10.

[219] A. Mahendran and A. Vedaldi, “Understanding deep image representations by inverting them,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2015, pp. 5188–5196.

[220] K. M. Yi, E. Trulls, V. Lepetit, and P. Fua, “LIFT: Learned invariant feature transform,” in Proc. Eur. Conf. Comput. Vis., 2016, pp. 467–483.

[221] T. Kanji, “Self-localization from images with small overlap,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2016, pp. 4497–4504.

[222] S. Cascianelli, G. Costante, E. Bellocchio, P. Valigi, M. L. Fravolini, and T. A. Ciarfuglia, “Robust visual semi-semantic loop closure detection by a covisibility graph and CNN features,” Robot. Auton. Syst., vol. 92, pp. 53–65, Jun. 2017.

[223] G. Tolias, R. Sicre, and H. Jégou, “Particular object retrieval with integral max-pooling of cnn activations,” in Proc. Int. Conf. Learn. Represent., 2016, pp. 1–12.

[224] P. Neubert and P. Protzel, “Beyond holistic descriptors, keypoints, and fixed patches: Multiscale superpixel grids for place recognition in changing environments,” IEEE Robot. Autom. Lett., vol. 1, no. 1, pp. 484–491, Jan. 2016.

[225] H. Noh, A. Araujo, J. Sim, T. Weyand, and B. Han, “Large-scale image retrieval with attentive deep local features,” in Proc. IEEE Int. Conf. Comput. Vis. (ICCV), Oct. 2017, pp. 3456–3465.

[226] M. Dusmanu et al., “D2-Net: A trainable CNN for joint description and detection of local features,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2019, pp. 8092–8101.

[227] J. Long, E. Shelhamer, and T. Darrell, “Fully convolutional networks for semantic segmentation,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2015, pp. 3431–3440.

[228] E. Stenborg, C. Toft, and L. Hammarstrand, “Long-term visual localization using semantically segmented images,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2018, pp. 6484–6490.

[229] M. D. Zeiler and R. Fergus, “Visualizing and understanding convolutional networks,” in Proc. Eur. Conf. Comput. Vis., 2014, pp. 818–833.

[230] I. Kansizoglou, L. Bampis, and A. Gasteratos, “Deep feature space: A geometrical perspective,” IEEE Trans. Pattern Anal. Mach. Intell., early access, Jul. 7, 2021, doi: 10.1109/TPAMI.2021.3094625.

[231] K. Simonyan and A. Zisserman, “Very deep convolutional networks for large-scale image recognition,” 2014, arXiv:1409.1556.

[232] K. He, X. Zhang, S. Ren, and J. Sun, “Deep residual learning for image recognition,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2016, pp. 770–778.

[233] C. Szegedy, S. Ioffe, V. Vanhoucke, and A. Alemi, “Inception-v4, inception-ResNet and the impact of residual connections on learning,” in Proc. AAAI Conf. Artif. Intell., 2017, pp. 1–7.

[234] G. Huang, Z. Liu, L. Van Der Maaten, and K. Q. Weinberger, “Densely connected convolutional networks,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jul. 2017, pp. 4700–4708.

[235] A. G. Howard et al., “MobileNets: Efficient convolutional neural networks for mobile vision applications,” 2017, arXiv:1704.04861.

[236] Z. Yu, C. Feng, M.-Y. Liu, and S. Ramalingam, “CASENet: Deep category-aware semantic edge detection,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jul. 2017, pp. 5964–5973.

[237] M. Teichmann, A. Araujo, M. Zhu, and J. Sim, “Detect-to-retrieve: Efficient regional aggregation for image search,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2019, pp. 5109–5118.

[238] M. Mohan, D. Gálvez-López, C. Monteleoni, and G. Sibley, “Environment selection and hierarchical place recognition,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2015, pp. 5487–5494.

[239] L. Yu, A. Jacobson, and M. Milford, “Rhythmic representations: Learning periodic patterns for scalable place recognition at a sublinear storage cost,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 811–818, Apr. 2018.

[240] S. Garg and M. Milford, “Fast, compact and highly scalable visual place recognition through sequence-based matching of overloaded representations,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 3341–3348.

[241] Y. Liu and H. Zhang, “Towards improving the efficiency of sequencebased SLAM,” in Proc. IEEE Int. Conf. Mechatronics Automat., Aug. 2013, pp. 1261–1266.

[242] N. Sünderhauf, P. Neubert, and P. Protzel, “Are we there yet? Challenging SeqSLAM on a 3000 km journey across all four seasons,” in Proc. IEEE Int. Conf. Robot. Automat., May 2013, p. 2013.

[243] T. Naseer, L. Spinello, W. Burgard, and C. Stachniss, “Robust visual robot localization across seasons using network flows,” in Proc. AAAI Conf. Artif. Intell., 2014, pp. 2564–2570.

[244] C. Mei, G. Sibley, and P. Newman, “Closing loops without places,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2010, pp. 3738–3744.

[245] E. Stumm, C. Mei, and S. Lacroix, “Probabilistic place recognition with covisibility maps,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Nov. 2013, pp. 4158–4163.

[246] M. Volkov, G. Rosman, D. Feldman, J. W. Fisher, and D. Rus, “Coresets for visual summarization with applications to loop closure,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2015, pp. 3638–3645.

[247] E. S. Stumm, C. Mei, and S. Lacroix, “Building location models for visual place recognition,” Int. J. Robot. Res., vol. 35, no. 4, pp. 334–356, Apr. 2016.

[248] S. An, G. Che, F. Zhou, X. Liu, X. Ma, and Y. Chen, “Fast and incremental loop closure detection using proximity graphs,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Nov. 2019, pp. 378–385.

[249] F. Savelli and B. Kuipers, “Loop-closing and planarity in topological map-building,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), vol. 2, Sep. 2004, pp. 1511–1517.

[250] P. Hansen and B. Browning, “Visual place recognition using HMM sequence matching,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2014, pp. 4549–4555.

[251] R. Arroyo, P. F. Alcantarilla, L. M. Bergasa, and E. Romera, “Towards life-long visual localization using an efficient matching of binary sequences from images,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2015, pp. 6328–6335.

[252] P. Neubert, S. Schubert, and P. Protzel, “A neurologically inspired sequence processing model for mobile robot place recognition,” IEEE Robot. Autom. Lett., vol. 4, no. 4, pp. 3200–3207, Oct. 2019.

[253] L. Rabiner, “Fundamentals of speech recognition,” in Fundamentals of Speech Recognition. Upper Saddle River, NJ, USA: Prentice-Hall, 1993.

[254] B. Talbot, S. Garg, and M. Milford, “OpenSeqSLAM2.0: An open source toolbox for visual place recognition under changing conditions,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2018, pp. 7758–7765.

[255] A. J. Viterbi, “Error bounds for convolutional codes and an asymptotically optimum decoding algorithm,” IEEE Trans. Inf. Theory, vol. IT-13, no. 2, pp. 260–269, Apr. 1967.

[256] A. Jacobson, Z. Chen, and M. Milford, “Online place recognition calibration for out-of-the-box SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2015, pp. 1357–1364.

[257] O. Vysotska and C. Stachniss, “Lazy data association for image sequences matching under substantial appearance changes,” IEEE Robot. Autom. Lett., vol. 1, no. 1, pp. 213–220, Jan. 2016.

[258] E. Stumm, C. Mei, S. Lacroix, J. Nieto, M. Hutter, and R. Siegwart, “Robust visual place recognition with graph kernels,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2016, pp. 4535–4544.

[259] O. Vysotska and C. Stachniss, “Effective visual place recognition using multi-sequence maps,” IEEE Robot. Autom. Lett., vol. 4, no. 2, pp. 1730–1736, Apr. 2019.

[260] H. Zhang, F. Han, and H. Wang, “Robust multimodal sequence-based loop closure detection via structured sparsity,” in Proc. Robot., Sci. Syst., 2016, pp. 1–10.

[261] J. Bruce, A. Jacobson, and M. Milford, “Look no further: Adapting the localization sensory window to the temporal characteristics of the environment,” IEEE Robot. Autom. Lett., vol. 2, no. 4, pp. 2209–2216, Oct. 2017.

[262] A. Banino et al., “Vector-based navigation using grid-like representations in artificial agents,” Nature, vol. 557, no. 7705, pp. 429–433, May 2018.

[263] X. Zhang, L. Wang, Y. Zhao, and Y. Su, “Graph-based place recognition in image sequences with CNN features,” J. Intell. Robot. Syst., vol. 95, no. 2, pp. 389–403, Aug. 2019.

[264] A. Angeli, S. Doncieux, J.-A. Meyer, and D. Filliat, “Incremental vision-based topological SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2008, pp. 1031–1036.

[265] H. Johannsson, M. Kaess, M. Fallon, and J. J. Leonard, “Temporally scalable visual SLAM using a reduced pose graph,” in Proc. IEEE Int. Conf. Robot. Automat., May 2013, pp. 54–61.

[266] S. Rohou, P. Franek, C. Aubry, and L. Jaulin, “Proving the existence of loops in robot trajectories,” Int. J. Robot. Res., vol. 37, no. 12, pp. 1500–1516, Oct. 2018.

[267] M. A. Fischler and R. Bolles, “Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography,” Commun. ACM, vol. 24, no. 6, pp. 381–395, 1981.

[268] D. Nistér, “An efficient solution to the five-point relative pose problem,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 26, no. 6, pp. 756–770, Jun. 2004.

[269] P. J. Besl and N. D. McKay, “Method for registration of 3-D shapes,” Proc. SPIE, vol. 1611, pp. 586–606, Apr. 1992.

[270] A. Geiger, P. Lenz, C. Stiller, and R. Urtasun, “Vision meets robotics: The KITTI dataset,” Int. J. Robot. Res., vol. 32, no. 11, pp. 1231–1237, 2013.

[271] M. Cummins, “Highly scalable appearance-only SLAM-FAB-MAP 2.0,” in Proc. Robot., Sci. Syst., 2009, pp. 1–8.

[272] G. Pandey, J. R. McBride, and R. M. Eustice, “Ford campus vision and LiDAR data set,” Int. J. Robot. Res., vol. 30, no. 13, pp. 1543–1552, 2011.

[273] J.-L. Blanco, F.-A. Moreno, and J. González, “A collection of outdoor robotic datasets with centimeter-accuracy ground truth,” Auton. Robots, vol. 27, no. 4, pp. 327–351, Nov. 2009.

[274] M. Burri et al., “The EuRoC micro aerial vehicle datasets,” Int. J. Robot. Res., vol. 35, no. 10, pp. 1157–1163, 2016.

[275] S. Griffith, G. Chahine, and C. Pradalier, “Symphony lake dataset,” Int. J. Robot. Res., vol. 36, no. 11, pp. 1151–1158, Sep. 2017.

[276] J. Bruce, J. Wawerla, and R. Vaughan, “The SFU mountain dataset: Semi-structured woodland trails under changing environmental conditions,” in Proc. IEEE Int. Conf. Robot. Automat. Workshop, Seattle, WA, USA, May 2015.

[277] A. Glover, “Day and night, left and right,” 2014, doi: 10.5281/ zenodo.4590133.

[278] A. J. Glover, W. P. Maddern, M. J. Milford, and G. F. Wyeth, “FAB-MAP RatSLAM: Appearance-based SLAM for multiple times of day,” in Proc. IEEE Int. Conf. Robot. Automat., May 2010, pp. 3507–3512.

[279] W. Maddern, G. Pascoe, C. Linegar, and P. Newman, “1 year, 1000 km: The Oxford RobotCar dataset,” Int. J. Robot. Res., vol. 36, no. 1, pp. 3–15, 2017.

[280] G. Ros, L. Sellart, J. Materzynska, D. Vazquez, and A. M. Lopez, “The SYNTHIA dataset: A large collection of synthetic images for semantic segmentation of urban scenes,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2016, pp. 3234–3243.

[281] M. Ramezani, Y. Wang, M. Camurri, D. Wisth, M. Mattamala, and M. Fallon, “The newer college dataset: Handheld LiDAR, inertial and vision with ground truth,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2020, pp. 4353–4360.

[282] D. M. W. Powers, “Evaluation: From precision, recall and F-measure to ROC, informedness, markedness and correlation,” 2020, arXiv:2010.16061.

[283] M. Zaffar, A. Khaliq, S. Ehsan, M. Milford, and K. McDonald-Maier, “Levelling the playing field: A comprehensive comparison of visual place recognition approaches under changing conditions,” 2019, arXiv:1903.09107.

[284] J. A. Hanley and B. J. McNeil, “The meaning and use of the area under a receiver operating characteristic (ROC) curve,” Radiology, vol. 143, no. 1, pp. 29–36, 1982.

[285] J. Davis and M. Goadrich, “The relationship between precision-recall and ROC curves,” in Proc. 23rd Int. Conf. Mach. Learn. (ICML), 2006, pp. 233–240.

[286] Y. Hou, H. Zhang, and S. Zhou, “Convolutional neural network-based image representation for visual loop closure detection,” in Proc. IEEE Int. Conf. Inf. Automat., Aug. 2015, pp. 2238–2245.

[287] B. Ferrarini, M. Waheed, S. Waheed, S. Ehsan, M. J. Milford, and K. D. McDonald-Maier, “Exploring performance bounds of visual place recognition using extended precision,” IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 1688–1695, Apr. 2020.

[288] D. M. Chen et al., “City-scale landmark identification on mobile devices,” in Proc. Conf. Comput. Vis. Pattern Recognit., 2011, pp. 737–744.

[289] J. G. Mangelson, D. Dominic, R. M. Eustice, and R. Vasudevan, “Pairwise consistent measurement set maximization for robust multirobot map merging,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2018, pp. 2916–2923.

[290] P.-Y. Lajoie, B. Ramtoula, Y. Chang, L. Carlone, and G. Beltrame, “DOOR-SLAM: Distributed, online, and outlier resilient SLAM for robotic teams,” IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 1656–1663, Apr. 2020.

[291] H. Yang, P. Antonante, V. Tzoumas, and L. Carlone, “Graduated nonconvexity for robust spatial perception: From non-minimal solvers to global outlier rejection,” IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 1127–1134, Apr. 2020.

[292] Y. Tian, Y. Chang, F. Herrera Arias, C. Nieto-Granda, J. How, and L. Carlone, “Kimera-multi: Robust, distributed, dense metric-semantic SLAM for multi-robot systems,” IEEE Trans. Robot., early access, Jan. 20, 2022, doi: 10.1109/TRO.2021.3137751.

[293] H. Lategahn, A. Geiger, and B. Kitt, “Visual SLAM for autonomous ground vehicles,” in Proc. IEEE Int. Conf. Robot. Automat., May 2011, pp. 1732–1737.

[294] E. Shechtman and M. Irani, “Matching local self-similarities across images and videos,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2007, pp. 1–8.

[295] V. Vonikakis, R. Kouskouridas, and A. Gasteratos, “On the evaluation of illumination compensation algorithms,” Multimedia Tools Appl., vol. 77, no. 8, pp. 9211–9231, Apr. 2018.

[296] W. Churchill and P. Newman, “Practice makes perfect? Managing and leveraging visual experiences for lifelong navigation,” in Proc. IEEE Int. Conf. Robot. Automat., May 2012, pp. 4525–4532.

[297] T. Sattler et al., “Benchmarking 6DOF outdoor visual localization in changing conditions,” in Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit., Jun. 2018, pp. 8601–8610.

[298] S. Schubert, P. Neubert, and P. Protzel, “Unsupervised learning methods for visual place recognition in discretely and continuously changing environments,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 4372–4378.

[299] P. Neubert, N. Sünderhauf, and P. Protzel, “Appearance change prediction for long-term navigation across seasons,” in Proc. Eur. Conf. Mobile Robots, Sep. 2013, pp. 198–203.

[300] A. Ranganathan, S. Matsumoto, and D. Ilstrup, “Towards illumination invariance for visual localization,” in Proc. IEEE Int. Conf. Robot. Automat., May 2013, pp. 3791–3798.

[301] S. M. Lowry, M. J. Milford, and G. F. Wyeth, “Transforming morning to afternoon using linear regression techniques,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2014, pp. 3950–3955.

[302] B. Zhou, A. Lapedriza, J. Xiao, A. Torralba, and A. Oliva, “Learning deep features for scene recognition using places database,” in Proc. Adv. Neural Inf. Process. Syst., vol. 27, 2014, pp. 487–495.

[303] B. Bescos, J. M. Fácil, J. Civera, and J. L. Neira, “DynaSLAM: Tracking, mapping, and inpainting in dynamic scenes,” IEEE Robot. Autom. Lett., vol. 3, no. 4, pp. 4076–4083, Oct. 2018.

[304] H. Osman, N. Darwish, and A. Bayoumi, “LoopNet: Where to focus? Detecting loop closures in dynamic scenes,” IEEE Robot. Autom. Lett., vol. 7, no. 2, pp. 2031–2038, Apr. 2022.

[305] W. Churchill and P. Newman, “Experience-based navigation for longterm localisation,” Int. J. Robot. Res., vol. 32, no. 14, pp. 1645–1661, Dec. 2013.

[306] S. M. Lowry, G. F. Wyeth, and M. J. Milford, “Towards trainingfree appearance-based localization: Probabilistic models for wholeimage descriptors,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2014, pp. 711–717.

[307] Z. Chen, S. Lowry, A. Jacobson, Z. Ge, and M. Milford, “Distance metric learning for feature-agnostic place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Sep. 2015, pp. 2556–2563.

[308] P. Panphattarasap and A. Calway, “Visual place recognition using landmark distribution descriptors,” in Proc. Asian Conf. Comput. Vis., 2016, pp. 487–502.

[309] N. Merrill and G. Huang, “Lightweight unsupervised deep loop closure,” in Proc. Robot., Sci. Syst., 2018, pp. 1–10.

[310] N. Carlevaris-Bianco and R. M. Eustice, “Learning visual feature descriptors for dynamic lighting conditions,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2014, pp. 2769–2776.

[311] P. Neubert, N. Sünderhauf, and P. Protzel, “Superpixel-based appearance change prediction for long-term navigation across seasons,” Robot. Auton. Syst., vol. 69, pp. 15–27, Jul. 2015.

[312] X. He, R. S. Zemel, and V. Mnih, “Topological map learning from outdoor image sequences,” J. Field Robot., vol. 23, nos. 11–12, pp. 1091–1104, Nov. 2006.

[313] C. Linegar, W. Churchill, and P. Newman, “Made to measure: Bespoke landmarks for 24-hour, all-weather localisation with a camera,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2016, pp. 787–794.

[314] S. Lowry and M. J. Milford, “Supervised and unsupervised linear learning techniques for visual place recognition in changing environments,” IEEE Trans. Robot., vol. 32, no. 3, pp. 600–613, Jun. 2016.

[315] J. M. Á. Alvarez and A. M. Lopez, “Road detection based on´ illuminant invariance,” IEEE Trans. Intell. Transp. Syst., vol. 12, no. 1, pp. 184–193, Mar. 2011.

[316] P. Corke, R. Paul, W. Churchill, and P. Newman, “Dealing with shadows: Capturing intrinsic scene appearance for image-based outdoor localisation,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Nov. 2013, pp. 2085–2092.

[317] M. Shakeri and H. Zhang, “Illumination invariant representation of natural images for visual place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), Oct. 2016, pp. 466–472.

[318] Z. Ying, G. Li, X. Zang, R. Wang, and W. Wang, “A novel shadow-free feature extractor for real-time road detection,” in Proc. 24th ACM Int. Conf. Multimedia, Oct. 2016, pp. 611–615.

[319] H. Lategahn, J. Beck, B. Kitt, and C. Stiller, “How to learn an illumination robust image feature for place recognition,” in Proc. IEEE Intell. Vehicles Symp., Jun. 2013, pp. 285–291.

[320] S. Hausler, A. Jacobson, and M. Milford, “Feature map filtering: Improving visual place recognition with convolutional calibration,” in Proc. Australas. Conf. Robot. Automat., 2018, pp. 1–10.

[321] M. Zaffar, S. Ehsan, M. Milford, and K. McDonald-Maier, “CoHOG: A light-weight, compute-efficient, and training-free visual place recognition technique for changing environments,” IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 1835–1842, Apr. 2020.

[322] H. Porav, W. Maddern, and P. Newman, “Adversarial training for adverse conditions: Robust metric localisation using appearance transfer,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2018, pp. 1011–1018.

[323] M. Milford et al., “Sequence searching with deep-learnt depth for condition- and viewpoint-invariant route-based place recognition,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. Workshops (CVPRW), Jun. 2015, pp. 18–25.

[324] A. Pronobis, B. Caputo, P. Jensfelt, and H. Christensen, “A discriminative approach to robust visual place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2006, pp. 3829–3836.

[325] R. Arroyo, P. F. Alcantarilla, L. M. Bergasa, J. J. Yebes, and S. Gámez, “Bidirectional loop closure detection on panoramas for visual navigation,” in Proc. IEEE Intell. Vehicles Symp., Jun. 2014, pp. 1378–1383.

[326] X. Li and Z. Hu, “Rejecting mismatches by correspondence function,” Int. J. Comput. Vis., vol. 89, no. 1, pp. 1–17, 2010.

[327] S. Garg, N. Suenderhauf, and M. Milford, “Semantic–geometric visual place recognition: A new perspective for reconciling opposing views,” Int. J. Robot. Res., p. 0278364919839761, 2019, doi: 10.1177/0278364919839761.

[328] G. Lin, A. Milan, C. Shen, and I. Reid, “RefineNet: Multi-path refinement networks for high-resolution semantic segmentation,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jul. 2017, pp. 1925–1934.

[329] S. Ren, K. He, R. Girshick, and J. Sun, “Faster R-CNN: Towards realtime object detection with region proposal networks,” in Proc. Adv. Neural Inf. Process. Syst., 2015, pp. 91–99.

[330] J. Li, D. Meger, and G. Dudek, “Semantic mapping for view-invariant relocalization,” in Proc. Int. Conf. Robot. Automat. (ICRA), May 2019, pp. 7108–7115.

[331] S. Arshad and G.-W. Kim, “Robustifying visual place recognition with semantic scene categorization,” in Proc. IEEE Int. Conf. Big Data Smart Comput. (BigComp), Feb. 2020, pp. 467–469.

[332] I. T. Papapetros, V. Balaska, and A. Gasteratos, “Multi-layer map: Augmenting semantic visual memory,” in Proc. Int. Conf. Unmanned Aircr. Syst. (ICUAS), Sep. 2020, pp. 1206–1212.

[333] A. L. Majdik, D. Verda, Y. Albers-Schoenberg, and D. Scaramuzza, “Air-ground matching: Appearance-based GPS-denied urban localization of micro aerial vehicles,” J. Field Robot., vol. 32, no. 7, pp. 1015–1039, 2015.

[334] T.-Y. Lin, Y. Cui, S. Belongie, and J. Hays, “Learning deep representations for ground-to-aerial geolocalization,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2015, pp. 5007–5015.

[335] H. Altwaijry, E. Trulls, J. Hays, P. Fua, and S. Belongie, “Learning to match aerial images with deep attentive architectures,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2016, pp. 3539–3547.

[336] V. Balaska, L. Bampis, and A. Gasteratos, “Graph-based semantic segmentation,” in Proc. Int. Conf. Robot. Alpe-Adria Danube Region, 2018, pp. 572–579.

[337] V. Balaska, L. Bampis, M. Boudourides, and A. Gasteratos, “Unsupervised semantic clustering and localization for mobile robotics tasks,” Robot. Auton. Syst., vol. 131, Sep. 2020, Art. no. 103567.

[338] V. Balaska, L. Bampis, I. Kansizoglou, and A. Gasteratos, “Enhancing satellite semantic maps with ground-level imagery,” Robot. Auton. Syst., vol. 139, May 2021, Art. no. 103760.

[339] G. Klein and D. Murray, “Parallel tracking and mapping for small AR workspaces,” in Proc. 6th IEEE ACM Int. Symp. Mixed Augmented Reality, Nov. 2007, pp. 225–234.

[340] E. Mouragnon, M. Lhuillier, M. Dhome, F. Dekeyser, and P. Sayd, “Real time localization and 3D reconstruction,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., vol. 1, Jun. 2006, pp. 363–370.

[341] R. Mur-Artal and J. D. Tardós, “Fast relocalisation and loop closing in keyframe-based SLAM,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2014, pp. 846–853.

[342] H. Strasdat, A. J. Davison, J. M. M. Montiel, and K. Konolige, “Double window optimisation for constant time visual SLAM,” in Proc. Int. Conf. Comput. Vis., Nov. 2011, pp. 2352–2359.

[343] J. Lim, J. Frahm, and M. Pollefeys, “Online environment mapping,” in Proc. Conf. Comput. Vis. Pattern Recognit., 2011, pp. 3489–3496.

[344] M. Zaffar, S. Ehsan, M. Milford, and K. D. McDonald-Maier, “Memorable maps: A framework for re-defining places in visual place recognition,” IEEE Trans. Intell. Transp. Syst., vol. 22, no. 12, pp. 7355–7369, Dec. 2021.

[345] C. Estrada, J. Neira, and J. D. Tardós, “Hierarchical SLAM: Real-time accurate mapping of large environments,” IEEE Trans. Robot., vol. 21, no. 4, pp. 588–596, Aug. 2005.

[346] Z. Zivkovic, B. Bakker, and B. Krose, “Hierarchical map building using visual landmarks and geometric constraints,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Aug. 2005, pp. 2480–2485.

[347] O. Booij, B. Terwijn, Z. Zivkovic, and B. Krose, “Navigation using an appearance based topological map,” in Proc. IEEE Int. Conf. Robot. Automat., Apr. 2007, pp. 3927–3932.

[348] O. Booij, Z. Zivkovic, and B. Kröse, “Efficient data association for view based SLAM using connected dominating sets,” Robot. Auton. Syst., vol. 57, no. 12, pp. 1225–1234, Dec. 2009.

[349] G. Grisetti, R. Kümmerle, C. Stachniss, U. Frese, and C. Hertzberg, “Hierarchical optimization on manifolds for online 2D and 3D mapping,” in Proc. IEEE Int. Conf. Robot. Automat., May 2010, pp. 273–278.

[350] G. Sibley, C. Mei, I. Reid, and P. Newman, “Planes, trains and automobiles—Autonomy for the modern robot,” in Proc. IEEE Int. Conf. Robot. Automat., May 2010, pp. 285–292.

[351] K. MacTavish and T. D. Barfoot, “Towards hierarchical place recognition for long-term autonomy,” in Proc. IEEE Int. Conf. Robot. Automat. Workshop, May 2014, pp. 1–6.

[352] X. Fei, K. Tsotsos, and S. Soatto, “A simple hierarchical pooling data structure for loop closure,” in Proc. Eur. Conf. Comput. Vis., 2016, pp. 321–337.

[353] S. Hausler and M. Milford, “Hierarchical multi-process fusion for visual place recognition,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 3327–3333.

[354] H. Stensola, T. Stensola, T. Solstad, K. Frøland, M.-B. Moser, and E. I. Moser, “The entorhinal grid map is discretized,” Nature, vol. 492, no. 7427, pp. 72–78, Dec. 2012.

[355] N. Kruger et al., “Deep hierarchies in the primate visual cortex: What can we learn for computer vision?” IEEE Trans. Pattern Anal. Mach. Intell., vol. 35, no. 8, pp. 1847–1871, Aug. 2013.

[356] K. Konolige and J. Bowman, “Towards lifelong visual maps,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2009, pp. 1156–1163.

[357] N. Tishby, F. C. Pereira, and W. Bialek, “The information bottleneck method,” 2000, arXiv:physics/0004057.

[358] L. Murphy and G. Sibley, “Incremental unsupervised topological place discovery,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2014, pp. 1312–1318.

[359] M. Milford and G. Wyeth, “Persistent navigation and mapping using a biologically inspired SLAM system,” Int. J. Robot. Res., vol. 29, no. 9, pp. 1131–1153, 2010.

[360] O. Guclu and A. B. Can, “Fast and effective loop closure detection to improve SLAM performance,” J. Intell. Robot. Syst., vol. 93, nos. 3–4, pp. 495–517, Mar. 2019.

[361] A. Levin and R. Szeliski, “Visual odometry and map correlation,” in Proc. IEEE Comput. Soc. Conf. Comput. Vis. Pattern Recognit., vol. 1, Jun. 2004, p. 1.

[362] L. G. Camara, C. Gäbert, and L. Pˇreuˇcil, “Highly robust visual place recognition through spatial matching of CNN features,” in Proc. IEEE Int. Conf. Robot. Automat. (ICRA), May 2020, pp. 3748–3755.

[363] F. Dayoub and T. Duckett, “An adaptive appearance-based map for long-term topological localization of mobile robots,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2008, pp. 3364–3369.

[364] A. Pronobis, L. Jie, and B. Caputo, “The more you learn, the less you store: Memory-controlled incremental SVM for visual place recognition,” Image Vis. Comput., vol. 28, no. 7, pp. 1080–1097, Jul. 2010.

[365] A. Kawewong, N. Tongprasit, and O. Hasegawa, “PIRF-Nav 2.0: Fast and online incremental appearance-based loop-closure detection in an indoor environment,” Robot. Auton. Syst., vol. 59, no. 10, pp. 727–739, Oct. 2011.

[366] F. Dayoub, G. Cielniak, and T. Duckett, “Long-term experiments with an adaptive spherical view representation for navigation in changing environments,” Robot. Auton. Syst., vol. 59, no. 5, pp. 285–295, May 2011.

[367] M. Labbe and F. Michaud, “Online global loop closure detection for large-scale multi-session graph-based SLAM,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2014, pp. 2661–2666.

[368] S. Lowry and H. Andreasson, “Lightweight, viewpoint-invariant visual place recognition in changing environments,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 957–964, Apr. 2018.

[369] O. Chum et al., “Near duplicate image detection: Min-hash and TF-IDF weighting,” in Proc. Brit. Mach. Vis. Conf., vol. 810, 2008, pp. 812–815.

[370] H. Shahbazi and H. Zhang, “Application of locality sensitive hashing to realtime loop closure detection,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Sep. 2011, pp. 1228–1233.

[371] T. Liu, A. W. Moore, K. Yang, and A. G. Gray, “An investigation of practical approximate nearest neighbor algorithms,” in Proc. Adv. Neural Inf. Process. Syst., 2005, pp. 825–832.

[372] H. Lejsek, B. T. Jónsson, and L. Amsaleg, “NV-Tree: Nearest neighbors at the billion scale,” in Proc. ACM Int. Conf. Multimedia Retr., 2011, pp. 1–8.

[373] Y. Liu and H. Zhang, “Indexing visual features: Real-time loop closure detection using a tree structure,” in Proc. IEEE Int. Conf. Robot. Automat., May 2012, pp. 3613–3618.

[374] Y. Hou, H. Zhang, and S. Zhou, “Tree-based indexing for real-time convnet landmark-based visual place recognition,” Int. J. Adv. Robot. Syst., vol. 14, no. 1, pp. 1–13, 2017.

[375] D. Schlegel and G. Grisetti, “HBST: A Hamming distance embedding binary search tree for feature-based visual place recognition,” IEEE Robot. Autom. Lett., vol. 3, no. 4, pp. 3741–3748, Oct. 2018.

[376] G. Grisetti, S. Grzonka, C. Stachniss, P. Pfaff, and W. Burgard, “Efficient estimation of accurate maximum likelihood maps in 3D,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., Oct. 2007, pp. 3472–3478.

[377] J. Wang, J. Wang, G. Zeng, Z. Tu, R. Gan, and S. Li, “Scalable k-NN graph construction for visual descriptors,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2012, pp. 1106–1113.

[378] B. Harwood and T. Drummond, “FANNG: Fast approximate nearest neighbour graphs,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jun. 2016, pp. 5713–5722.

[379] M. G. Gollub, R. Dubé, H. Sommer, I. Gilitschenski, and R. Siegwart, “A partitioned approach for efficient graph-based place recognition,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst./Workshop Planning, Perception Navigat. Intell. Veh., Sep. 2017, pp. 1–5.

[380] H. Jégou, F. Perronnin, M. Douze, J. Sánchez, P. Pérez, and C. Schmid, “Aggregating local image descriptors into compact codes,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 34, no. 9, pp. 1704–1716, Dec. 2011.

[381] K. A. Tsintotas, S. An, I. T. Papapetros, F. K. Konstantinidis, G. C. Sirakoulis, and A. Gasteratos, “Dimensionality reduction through visual data resampling for low-storage loop-closure detection,” in Proc. IEEE Int. Conf. Imag. Syst. Technol., Jun. 2022, pp. 1–6.

[382] J. Wang, T. Zhang, J. Song, N. Sebe, and H. T. Shen, “A survey on learning to hash,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 4, pp. 769–790, Apr. 2017.

[383] A. Gionis et al., “Similarity search in high dimensions via hashing,” in Proc. 25th Int. Conf. Very Large Data Bases, 1999, pp. 518–529.

[384] R. Salakhutdinov and G. Hinton, “Semantic hashing,” Int. J. Approx. Reasoning, vol. 50, no. 7, pp. 969–978, Jul. 2009.

[385] M. S. Charikar, “Similarity estimation techniques from rounding algorithms,” in Proc. 34th ACM Symp. Theory Comput., 2002, pp. 380–388.

[386] J. Bian, W.-Y. Lin, Y. Matsushita, S.-K. Yeung, T.-D. Nguyen, and M.-M. Cheng, “GMS: Grid-based motion statistics for fast, ultra-robust feature correspondence,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR), Jul. 2017, pp. 4181–4190.

[387] J. Ma, J. Zhao, J. Jiang, H. Zhou, and X. Guo, “Locality preserving matching,” Int. J. Comput. Vis., vol. 127, no. 5, pp. 512–531, 2019.

[388] X. Jiang, J. Ma, J. Jiang, and X. Guo, “Robust feature matching using spatial clustering with heavy outliers,” IEEE Trans. Image Process., vol. 29, pp. 736–746, 2020.

[389] D. Ravichandran, P. Pantel, and E. Hovy, “Randomized algorithms and NLP: Using locality sensitive hash functions for high speed noun clustering,” in Proc. 43rd Ann. Meeting Assoc. Comput. Linguistics, 2005, pp. 622–629.

[390] M. Muja and D. G. Lowe, “Fast matching of binary features,” in Proc. 9th Conf. Comput. Robot Vis., May 2012, pp. 404–410.

[391] T. B. Terriberry, L. M. French, and J. Helmsen, “GPU accelerating speeded-up robust features,” in Proc. Int. Symp. 3D Data Process. Vis. Transmiss., vol. 8, 2008, pp. 355–362.

[392] D. Nistér and H. Stewénius, “Scalable recognition with a vocabulary tree,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., vol. 2, Jun. 2006, pp. 2161–2168.

[393] H. Samet, The Design and Analysis of Spatial Data Structures, vol. 85. Reading, MA, USA: Addison-Wesley, 1990.

[394] A. Babenko and V. Lempitsky, “The inverted multi-index,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 37, no. 6, pp. 1247–1260, Jun. 2015.

[395] J. L. Bentley, “Multidimensional binary search trees used for associative searching,” Commun. ACM, vol. 18, no. 9, pp. 509–517, 1975.

[396] C. Silpa-Anan and R. Hartley, “Optimised KD-trees for fast image descriptor matching,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., Jun. 2008, pp. 1–8.

[397] M. Muja and D. G. Lowe, “Fast approximate nearest neighbors with automatic algorithm configuration,” in Proc. Int. Conf. Comput. Vis. Theory Appl., 2009, pp. 331–340.

[398] C. Chow and C. Liu, “Approximating discrete probability distributions with dependence trees,” IEEE Trans. Inf. Theory, vol. IT-14, no. 3, pp. 462–467, May 1968.

[399] D. Geman and B. Jedynak, “An active testing model for tracking roads in satellite images,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 18, no. 1, pp. 1–14, Jan. 1996.

[400] K. A. Tsintotas, V. Sevetlidis, I. T. Papapetros, V. Balaska, A. Psomoulis, and A. Gasteratos, “BK tree indexing for active visionbased loop-closure detection in autonomous navigation,” in Proc. Medit. Conf. Control Automat., Jun. 2022, pp. 1–6.

[401] K. Grauman and T. Darrell, “The pyramid match kernel: Discriminative classification with sets of image features,” in Proc. 10th IEEE Int. Conf. Comput. Vis. (ICCV), Oct. 2005, pp. 1458–1465.

[402] S. Lazebnik, C. Schmid, and J. Ponce, “Beyond bags of features: Spatial pyramid matching for recognizing natural scene categories,” in Proc. IEEE Comput. Soc. Conf. Comput. Vis. Pattern Recognit., vol. 2, Jun. 2006, pp. 2169–2178.

[403] B. Zhou, A. Lapedriza, A. Khosla, A. Oliva, and A. Torralba, “Places: A 10 million image database for scene recognition,” IEEE Trans. Pattern Anal. Mach. Intell., vol. 40, no. 6, pp. 1452–1464, Jun. 2018.

[404] S. L. Bowman, K. Daniilidis, and G. J. Pappas, “Robust and efficient semantic SLAM with semantic keypoints,” in Proc. AAAI Nat. Conf. Artif. Intell., 2021, pp. 1–13.

![](images/2022_LoopClosure-Survey/fec04e8e2aadf139fbc69591901bcfb26f9f70ee18793dddcf8ac7111d2f284c.jpg)

Konstantinos A. Tsintotas (Senior Member, IEEE) received the bachelor’s degree from the Department of Automation, Technological Education Institute of Chalkida, Psachna, Greece, in 2010, the master’s degree in mechatronics from the Department of Electrical Engineering, Technological Education Institute of Western Macedonia, Kila Kozanis, Greece, in 2015, and the Ph.D. degree in robotics from the Department of Production and Management Engineering, Democritus University of Thrace, Xanthi, Greece, in 2021. He is currently a Post-Doctoral

Fellow with the Laboratory of Robotics and Automation, Department of Production and Management Engineering, Democritus University of Thrace. His work is supported through several research projects funded by the European Commission and the Greek Government. His research interests include vision-based methods for modern and intelligent mechatronics systems. Details are available at: https://robotics.pme.duth.gr/ktsintotas

![](images/2022_LoopClosure-Survey/75803bdbf4492bfdd122f8ce4a438df71318a91ce646404caf15fd273c4e86ae.jpg)

Loukas Bampis received the Diploma degree in electrical and computer engineering and the Ph.D. degree in machine vision and embedded systems from the Democritus University of Thrace (DUTh), Greece, in 2013 and 2019, respectively. He is currently an Assistant Professor with the Laboratory of Mechatronics and Systems Automation (MeSA), Department of Electrical and Computer Engineering, DUTh. His work has been supported through several research projects funded by the European Space Agency, the European Commission, and the Greek

Government. His research interests include real-time localization and place recognition techniques using hardware accelerators and parallel processing. More details about him are available at: https://robotics.pme.duth.gr/bampis

![](images/2022_LoopClosure-Survey/4af53fbdcd651a726bfb881e20133d92f5591eebd1effbb4cebdd581fd0e0d51.jpg)

Antonios Gasteratos (Senior Member, IEEE) received the M.Eng. and Ph.D. degrees from the Department of Electrical and Computer Engineering, Democritus University of Thrace (DUTh), Greece. He is currently a Professor and the Head of the Department of Production and Management Engineering, DUTh. He is also the Director of the Laboratory of Robotics and Automation (LRA), DUTh, where he is teaching the courses of robotics, automatic control systems, electronics, mechatronics and computer vision. From 1999 to 2000, he was a Visiting Researcher with the Laboratory of Integrated Advanced Robotics (LIRALab), DIST, University of Genoa, Italy. He has served as a reviewer for numerous scientific journals and international conferences. He has published more than 220 papers in books, journals, and conferences. His research interests include mechatronics and robot vision. He is a Fellow Member of IET. He is a Subject Editor of Electronics Letters and an Associate Editor of the International Journal of Optomechatronics and he has organized/co-organized several international conferences. More details about him are available at: https://robotics.pme.duth.gr/antonis