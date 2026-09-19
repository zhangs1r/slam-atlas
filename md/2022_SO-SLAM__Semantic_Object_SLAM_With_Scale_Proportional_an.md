# SO-SLAM: Semantic Object SLAM With Scale Proportional and Symmetrical Texture Constraints

Ziwei Liao , Yutong Hu , Jiadong Zhang, Xianyu Qi,

Xiaoyu Zhang , Graduate Student Member, IEEE, and Wei Wang

Abstract—Object SLAM introduces the concept of objects into Simultaneous Localization and Mapping (SLAM) and helps understand indoor scenes for mobile robots and object-level interactive applications. The state-of-art object SLAM systems face challenges such as partial observations, occlusions, unobservable problems, limiting the mapping accuracy and robustness. This letter proposes a novel monocular Semantic Object SLAM (SO-SLAM) system that addresses the introduction of object spatial constraints. We explore three representative spatial constraints, including scale proportional constraint, symmetrical texture constraint and plane supporting constraint. Based on these semantic constraints, we propose two new methods - a more robust object initialization method and an orientation fine optimization method. We have verified the performance ofthe algorithm on the public datasets and an author-recorded mobile robot dataset and achieved a significant improvement on mapping effects. We will release the code here.<sup>1</sup>

Index Terms—SLAM, semantic scene understanding, semantic SLAM, symmetry.

## I. INTRODUCTION

OR decades, robotic researchers have been exploring the environments autonomously in an open world. Imagine a long-term service robot working in an indoor human-robot coexisting scenarios-homes, museums, offices, etc. To respond to human instructions and carry out tasks, it needs the abilities of robust mapping and localization, understanding environments semantically, and detecting environmental changes for lifelong map maintaining.

However, traditional SLAM algorithms use point, line, and plane features to build maps, lacking semantic information [1]. Artificially designed feature descriptors are difficult to adapt to large viewing angle changes and are susceptible to interference from light and sensor noise [2]. The traditional SLAM algorithms are mostly based on static assumptions of the environment. The maps based on points, lines, and planes are difficult to update according to the changes in the environment. Therefore, traditional SLAM algorithms are far from meeting the needs of indoor service robots.

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/05c6f65b89f78d3f268789019549e25afe3f130ad18f6eac771ca0a78eacb99d.jpg)  
Fig. 1. Object SLAM. It can build a map with objects including center, orientation and occupied space, to help robots understand object-oriented instructions from human.

We believe that as an important component of the indoor environment, objects have the following potential advantages to represent indoor environments. First, the spatial information of an object can be expressed by more advanced abstract features such as center, orientation, and occupied space. It is not sensitive to changes in origin observation data and is intuitively more robust for strong disturbances. Second, the spatial relationships between objects and structures (e.g., walls) can be used as auxiliary constraints to improve the robustness and accuracy, which help the robots’ scene understanding from the geometric level to the semantic level.

Recently, object SLAM systems [3]–[7] introduce objects as landmarks and compactly model the environments. Among them, quadric-based object SLAM systems [7], [9]–[15] use quadrics to compactly model objects’ center, orientation, and occupied space. Compared with RGB-D object SLAM systems [3], [4], monocular camera is more convenient, low-cost, and lightweight, which makes the algorithm available in a wider range of applications, e.g., mobile phones and drones.

This letter’s motivation is to address the following two challenges and limitations of monocular object SLAM systems. First, monocular cameras contain less information to constrain objects and deal with partial observations, occlusions, etc., making monocular Object SLAM fragile in real applications [6], [7]. Especially in the typical forward movement trajectory of a mobile robot, unlike the ideal looking-around trajectory, the robot movement trajectory is difficult to produce multi-frame observations with large angular variations, raising the problem of unobservability [10]. Second, current object SLAM [7], [10] mainly constrains the object’s occupied space, so the orientation is relatively random and not meaningful. We explicitly constrain object’s orientation with symmetry, so that our object-level maps enable robots for semantic indoor navigation, e.g., move to the side or front of a sofa.

In this letter, we propose a novel monocular Semantic Object SLAM (SO-SLAM) system as in Fig. 1. We inherited object’s semantic information and introduced three representative object spatial constraints, including scale proportional constraints, symmetrical texture constraints and plane supporting constraints. We will derive their mathematical representations and constraint models in the SLAM system to participate in both front-end initialization and back-end optimization. In summary, we will make the following contributions:

1. Propose a monocular object SLAM algorithm that fully couples three spatial structure constraints for indoor environments.

2. Propose two new methods based on spatial constraints: a single-frame object initialization method and an object orientation optimization method.

3. Testify the effectiveness of the proposed algorithm on two public datasets, and an author-recorded real mobile robot dataset.

## II. RELATED WORK

## A. Object SLAM

Object SLAM, or object-level SLAM, focuses on the construction of object features, including objects’ position, orientation, occupied space, and relationships with the spatial structures in the map, as in Fig. 1. The early exploration of object SLAM can be traced back to SLAM++ [3]. It establishes an object CAD model database offline, and then uses the depth information of the RGB-D camera to match the object database in actual operation. In 2019, Martin et al. [4] proposed MaskFusion, which uses neural network to detect objects and no database is needed. It detects and tracks dynamic objects in real time. However, limited by their dense object models, the systems mentioned above require heavy computation hardware to achieve real-time and high-rate operations. In 2019, Yang et al. proposed CubeSLAM [5], which uses cuboids to model objects in the environment. Since 2017, researchers [6], [25] have explored the use ofquadric models to represent objects in the field ofstructure from motion. In 2019, Nicholson et al. proposed QuadricSLAM [7], which is the first time to build an object SLAM system with quadrics.

Compared to feature points, cuboids and quadrics can model not only position, but also orientation and occupied space, which are sufficient for robot navigation. Cuboids is a human-defined model, while quadrics have a compact quadratic mathematical representation and complete projective geometry [8]. Recently, the quadric models are getting more and more attention from researchers [9]–[15], and even superquadrics [17] is being explored.

## B. Semantic Priors in Object SLAM

To make the systems more robust, researchers further explore the meaning of orientation. In [12], gravity and supporting plane are introduced, which defines the “top” side. Ok et al. [10] proposed the texture planes, which essentially defines the “front” side. In addition, deep learning methods are applied to help estimate the ellipsoid [13] as well. The authors have explored the introduction of RGB-D camera for quadrics [14] and tried the object orientation estimation based on symmetry under RGB-D data [15]. This letter will further explore the algorithm based on a monocular camera.

In summary, the previous letters are showing that the object SLAM systems based on quadric models are accepted by more and more researchers. However, there is still plenty of room for research to make the systems more robust and accurate in the real world.

## III. MONOCULAR OBJECT SLAM FRAMEWORK

We denote the points in 3D space as set $V = \{ v \}$ and the pixels in the image as $U = \{ u \}$ . The photographic process is denoted as $u = \mathcal { P } ( v ) = P \cdot v ,$ , where P is the camera projection matrix. An ellipsoid is a point set $V q = \{ v q | \Theta \left( v q \right) = \stackrel { . } { 0 } \}$ , where $\Theta ( v ) = v ^ { T } Q \bar { v }$ . Since $\bar { V } q$ is completely determined by Q, we can equally call Q an ellipsoid. Representing an object by Q means assuming that all the surface points of the object are on the ellipsoid Q.

The front-end input of the system includes the monocular images and odometry data. Object detection algorithm (e.g., YOLO [16]) extracts bounding boxes from RGB images. An ellipsoid Q has 9 degrees of freedom which can be estimated using the SVD method, which needs at least 3 frames of observations with enough view variety [7]. As mentioned in the related work, this approach is not only fragile, but also lacks the accuracy of orientation. In the back end, we model the object SLAM problem as a pose graph, including nodes composed of objects and camera poses, and edges composed of constraints. The object SLAM formulation can be represented as a nonlinear optimization problem:

$$
\hat { X } , \hat { Q } = \underset { X , Q } { \arg \operatorname* { m i n } } \left( \sum H ( F _ { Z } ) + \sum H ( F _ { O } ) + \sum H ( F _ { S } ) \right)\tag{1}
$$

where X is the camera poses and Q is the objects in the map. $F _ { z }$ is the camera-object observation constraint, $F _ { o }$ is the odometry constraint, and both have been introduced in detail in [7]. This letter emphasizes the newly added $F _ { s } ,$ , composed of plane supporting constraints $f _ { s u p } ,$ proportional scale constraints $f _ { s s c }$ and symmetrical texture constraints $f _ { s y m }$ , which will be introduced in the following parts. $H ( \cdot )$ is the robust kernel to make the systems more robust to the outliers, and we use Huber Kernel in the experiments.

## IV. SINGLE-FRAME INITIALIZATION WITH SEMANTIC PRIORS

Human-like perception is necessary for service robots to understand and interact with objects. We follow human cognitive habits to set up object coordinate. We consider that the “top” of an artificial object is often the opposite of the supported side of the object, while the “front” is often the direction of symmetry, $\mathrm { e . g . }$ , cars and chairs. The former defines the direction of the Z-axis, and the latter defines the direction of the X-axis, thus the three axes of the object is completely fixed. After that, we can utilize more constraints, such as the supporting relationship and the proportional scale of each object. As shown in Fig. 2, we propose a 9-DOF object initialization method that requires only one frame, overcoming the hard-to-meet requirements of the conventional SVD method.

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/4091b7df8cb15c86dcfd398e7f024a9574d60b39032c597740042a2c3306c3bd.jpg)  
Fig. 2. Single-frame quadrics initiliazation. The tangent planes from the back projection of the object detection bounding box and the object supporting plane will jointly constrain the ellipsoid. The depth uncertainty along the observation direction will be further constrained by the scale propotional constraint.

## A. Object Detection Constraints

As shown in Fig. 2, an object O placed on its supporting plane $\pi _ { s }$ is observed in one frame. The bounding box of the object generated from object detection algorithms in the image is $^ { b . }$ Generally, the depth and scale of the object is unknown from only one observation. Let the four edges of b be $l _ { i } , i \ = \ 1 , 2 , 3 , \dot { 4 }$ and $\mathbf { \xi } _ { l _ { i } }$ is a 3x1 homogeneous vector, with ${ \mathbf { \boldsymbol { u } } } ^ { T } { \boldsymbol { l } } _ { i } = \mathrm { \boldsymbol { 0 } }$ , where u is the homogeneous coordinate of any pixel on the line. Each edge $\mathbf { \xi } _ { l _ { i } }$ can be back projected to produce a plane $\pi _ { i }$ [8]:

$$
\pi _ { i } = P ^ { T } l _ { i } ,\tag{2}
$$

where $_ { P }$ is the camera projection matrix. Each plane will form a tangent constraint with the dual quadrics model $Q ^ { * }$ of the object O, namely:

$$
\pi _ { i } ^ { T } Q ^ { * } \pi _ { i } = 0 , i = 1 , 2 , 3 , 4 .\tag{3}
$$

The dual quadrics $Q ^ { * }$ is a 4 × 4 matrix that is parameterized to have only 9 degrees of freedom to represent ellipsoids only. We refer readers to [7], [8] for more details about quadrics. So, an object detection constraint $f _ { b b o x }$ will constitute a constraint of 4 degrees of freedom with the object, which can be expressed as:

$$
f _ { b b o x } = \sum _ { i } \left\| \pi _ { i } ^ { T } Q ^ { * } \pi _ { i } \right\| _ { \Sigma _ { d e t } } ,\tag{4}
$$

where $\Sigma _ { d e t }$ is the object detection covariance. We use $\Sigma _ { d e t } =$ 10 in the experiments.

## B. Plane Supporting Constraints

In a normal indoor environment, to overcome gravity, objects must form a geometric relationship with spatial structures. E.g., the cup on the table, the lamp under the ceiling, and the paintings on the walls. This letter introduces the most common supporting relationship, when the structural plane is located under the object. The suspension, lean and other relations can be derived in a similar way.

Assuming the supporting plane of the object $Q ^ { * }$ is $\pi _ { s } =$ $( n _ { s } , d )$ , where $\mathbf { \delta } _ { n _ { s } }$ is the normal vector of the plane. If the $Z$ axis of the object is upward in the direction of gravity, then its

$X$ and $Y$ axes must be orthogonal to the normal vector of the supporting plane, so the following constraints can be obtained:

$$
\mathrm { R o t } _ { x } ( Q ^ { * } ) \cdot n _ { s } = 0 ,\tag{5}
$$

$$
\mathrm { R o t } _ { y } ( Q ^ { * } ) \cdot n _ { s } = 0 ,\tag{6}
$$

where $\mathrm { R o t } _ { x } ( Q ^ { * } )$ is the X axis normal of the ellipsoid $Q ^ { * }$ . Also, the quadric $Q ^ { * }$ should be tangent to the plane $\pi _ { s } ,$ as:

$$
\pi _ { s } ^ { T } Q ^ { * } \pi _ { s } = 0 .\tag{7}
$$

${ \mathrm { S o } } ,$ a supporting plane $\pi _ { s }$ can offer three degrees of constraints to the object $Q ^ { * }$ as:

$$
\begin{array} { r l } & { f _ { s u p } ( \pmb { Q } ^ { * } , \pi _ { s } ) = \Big \| \operatorname { R o t } _ { x } ( \pmb { Q } ^ { * } ) \cdot \pmb { n } _ { s } \Big \| _ { \Sigma _ { \theta } } + } \\ & { \qquad \Big \| \operatorname { R o t } _ { y } ( \pmb { Q } ^ { * } ) \cdot \pmb { n } _ { s } \Big \| _ { \Sigma _ { \theta } } + \Big \| \pi _ { s } ^ { \mathbf { T } } \pmb { Q } ^ { * } \pi _ { \mathbf { s } } \Big \| _ { \Sigma _ { \pi } } , } \end{array}\tag{8}
$$

where, $\Sigma _ { \theta }$ is the rotation covariance, and $\Sigma _ { \pi }$ is the tangent covariance. The covariances can be used to adjust the weight of the constraints according to the specific environments’ situations, to make the assumption have better generalizability for practical application. We use $\Sigma _ { \theta } = \Sigma _ { \pi } ~ =$ 10 in the experiments. When the ellipsoid’s $\textsf { Z }$ axis is perpendicular to the support plane and its bottom is tangent to the support plane, the constraint error becomes the smallest.

## C. Semantic Scale Proportional Constraint

The scale of indoor artificial objects in the same category has a certain distribution, which is also a geometric reflection of object semantics. There have been some studies discussing how to apply object scale prior constraints to object mapping. For example, Ok et al. [10] assumed that the size of the car is known. However, its flexibility is limited, and it cannot adapt to the scale ambiguity of specific instances with the same label, $\mathrm { e . g . , }$ a real car and a small toy car.

This letter proposes a new flexible object scale prior- Scale Proportional Constraint (SPC), which constrains object’s proportional scale instead of its specific scale. Assuming the scale of an object is $\begin{array} { r } { \pmb { s } = [ a , b , c ] ^ { T } , } \end{array}$ , where a, b, c is the half scale of its X, $Y ,$ Z axes. Then we can define its scale ratio $\boldsymbol { r } ~ = [ \sigma , \beta ] ^ { T }$ as follows:

$$
\sigma { \bf \Psi } = \frac { a } { c } { \bf \Psi } ,\tag{9}
$$

$$
\beta { \bf \Psi } = \frac { b } { c } { \bf \Psi } .\tag{10}
$$

For objects with different semantic labels, a scale ratio table of common objects can be defined, and the ratio can be obtained by querying the table in practical applications. In actual use, the table can be obtained by averaging the scale of common object types.

Given an object $Q _ { 0 } ^ { * } ,$ , its scale ratio $r _ { 0 } = r ( Q _ { 0 } ^ { * } )$ ) can be calculated according to the definition. Its corresponding semantic scale ratio $r _ { s } = \mathrm { S e m T a b l e } ( l _ { 0 } )$ can be obtained by querying the table according to its semantic label $l _ { 0 } .$ . Assuming the scale variance is $\Sigma _ { s s c } ,$ the scale ratio constraint of the object $Q _ { 0 } ^ { * }$ with the semantic label $l _ { 0 }$ is:

$$
\begin{array} { r } { f _ { s s c } \left( Q _ { 0 } ^ { * } , l _ { 0 } \right) = \| \pmb { r } _ { 0 } - \pmb { r } _ { s } \| _ { \Sigma _ { s s c } } } \\ { = \| \pmb { r } ( \pmb { Q } _ { 0 } ^ { * } ) - \mathrm { S e m T a b l e } ( l _ { 0 } ) \| _ { \Sigma _ { s s c } } } \end{array} .\tag{11}
$$

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/cbc487cd23dd449621b2a7928e9419409f2462ad4618e001433912336bac46c9.jpg)  
Fig. 3. (a) The symmetry relationship between two object points about a plane. (b) The process of solving for symmetric pixels in the image.

We use $\Sigma _ { s s c } = 1$ in the experiments. When the scale value $\mathbf { \boldsymbol { r } } _ { 0 }$ of the object $Q _ { \mathrm { 0 } } ^ { \ast }$ is consistent with its semantic scale prior value $\mathbf { \nabla } _ { r _ { s } , }$ the constraint error becomes smallest.

## D. Solving the Single Frame Initialization

Due to the diverse forms of constraints, it is difficult to directly obtain an analytical solution. We construct a nonlinear optimizer, based on the Levenberg-Marquardt algorithm [18], and iteratively solves the optimal value. The objective function is defined as:

$$
\hat { \pmb { Q } } ^ { * } = \arg \operatorname* { m i n } _ { \pmb { Q } ^ { * } } \left( f _ { b b o x } + f _ { s u p } + f _ { s s c } \right) ,\tag{12}
$$

including the object detection constraint $f _ { b b o x } ,$ the plane supporting constraint $f _ { s u p }$ and the scale proportional constraint f<sub>SSC</sub>.

## V. ORIENTATION OPTIMIZATION WITH TEXTURE SYMMETRY

## A. Mathematical Description of Object Symmetry

We try to further constrain object orientation by their symmetry property, which is commonly found in man-made objects. The following part of this chapter focuses only on symmetric objects. Geometrically, the front of a man-made object is often considered to be the direction of its symmetrical plane. We consider it to correspond to the direction of the X-axis of the object’s coordinate system, as in Fig. 3(a).

The symmetry of an object is mathematically represented by the fact that for any point $v _ { 0 } \in V$ on an object, a point $v _ { 0 } ^ { S } \in V$ can always be found that is symmetric about its plane of symmetry $\pi _ { x z }$ . Since the assumption that the object is ellipsoidal, the plane of symmetry can be represented by the elements in matrix $Q .$ The symmetry relation between two points about a plane has explicit linear representation [26], denoted as $v _ { 0 } ^ { S } = \mathbf { \bar { \mathcal { S } } } ( v _ { 0 } , Q )$ Considering that objects with multiple symmetry planes such as boxes and balls, we uniformly establish the positive X-axis direction in the direction of the first symmetry surface found when the object is initialized.

For a specific object $Q , U q = \{ u q \}$ is the set of points on the surface of $Q$ in the image plane, then the recovery mapping of the pixel points $u \in U q$ on the image to the points $v \in V q$ on the 3D surface of the object is $\mathcal { P } ^ { \dagger } ( \cdot , Q ) : U \dot {  } V$ , such v that $\mathcal { P } ^ { \dagger } ( u , Q ) = v$ , then v satisfies:

$$
{ \Big ( } u = { \mathcal { P } } ( v ) = P \cdot v\tag{a}
$$

$$
\Theta ( v ) = \stackrel { . } { v } ^ { T } Q v = 0\tag{b}
$$

(13)

$$
\textbf { \lfloor } v \mathrm { i s v i s i b l e t o c a m e r a }\tag{c}
$$

Substituting (a) into (b) yields a quadratic equation about v which has at most one solution subject by (c). Therefore, for an object point $u _ { 0 }$ in an image, we can get its symmetry pixel point $u _ { 0 } ^ { S } \colon$

$$
u _ { 0 } ^ { S } = \mathcal { P } \circ S \circ \mathcal { P } ^ { \dagger } ( u _ { 0 } ) = \mathcal { P } \left( S \left[ \mathcal { P } ^ { \dagger } ( u _ { 0 } , Q ) , Q \right] \right) : = \mathbb { S } ( u _ { 0 } , Q ) .\tag{14}
$$

The process is shown in Fig. 3(b), which we write as $\mathbb { S } : U \to$ $U .$ . Having found the symmetric pixel pairs, we hope to find a descriptor $\boxed { \beta ( \cdot ) : U \to \dot { \mathbb { R } } }$ , to describe the symmetry. Specifically, we want $\beta ( \cdot ) _ { } \mathrm { { t o } }$ have the property that:

$$
\mathrm { i f } \ u _ { 1 } = \mathbb { S } ( u _ { 0 } , Q _ { 0 } ) = u _ { 0 } ^ { S } , \mathrm { t h e n } \ \beta ( u _ { 0 } ) = \beta ( u _ { 1 } ) \mathrm { f o r \ a l l } \ u _ { 0 } , u _ { 1 }\tag{15}
$$

When $\beta ( \cdot )$ satisfies (15), we say that $\beta ( \cdot )$ is symmetric projection invariant. After that, we can optimize our ellipsoid $Q$ with cost function $f _ { s y m }$ when the observation is noisy:

$$
f _ { s y m } = \sum _ { u _ { i } } \big ( \beta ( u _ { i } ) - \beta ( u _ { i } ^ { S } ) \big ) ^ { 2 } .\tag{16}
$$

The next step is to find the descriptor $\beta ( \cdot )$

## B. The Construction of Symmetry Descriptor

Descriptors $\beta ( u )$ are needed to reflect some feature of u that is symmetric projection invariant. We made different attempts and compared them in Section VI.

The most preliminary choice is the grayscale value of the pixel $\beta _ { G R A Y } ( u )$ , which, along with its variants, is widely used in the direct-method SLAM, but it is not robust enough in the real situation. Then we tried the BRIEF descriptor $\beta _ { B R I E F } ( u )$ which can reflect the nearby texture information. To ensure symmetry invariance, the sampling order of texture near $u _ { 0 }$ and its symmetry point $u _ { 0 } ^ { S }$ should be symmetrical to each other, as in Fig. 3(a).

However, in the optimization process (16) using $\beta _ { B R I E F } ( u )$ with every sampling points $u _ { i }$ fixed, the symmetry points $u _ { i } ^ { S } =$ $\mathbb { S } ( u _ { i } , Q )$ will change with the optimization iteration of $Q .$ , and thus $\{ u _ { i } ^ { S } | \ i = 1 , 2 \dots n \}$ needs to be resampled and recoded in each iteration step, which seriously slows down the algorithm. We then tried to find a more lightweight descriptor to meet the real-time requirements of SLAM which led us to consider the Distance Transform value of pixels.

$$
\beta _ { 2 D T } ( u _ { 0 } ) = \operatorname* { m i n } _ { u \in U _ { e } } \| u - u _ { 0 } \| , U _ { e } = \{ u _ { e } | u _ { e } \mathrm { i s e d g e p i x e l } \}\tag{17}
$$

The meaning of (17) is the closest distancefrom a pixel point to any pixel at the edge of the image, which partially reflects the object texture. It can be efficiently computed only once for all the pixel points in the object detection frame before the optimization. Then, its value can be queried during each iteration.

However, it is doubtful whether the description is symmetric projection invariant. For example, consider the case in Fig. 4(a). Note that $v _ { 0 } ^ { E }$ is the nearest edge point of $v _ { 0 }$ , and since symmetric objects have symmetric edge lines, we have $\lVert \boldsymbol { v } _ { 0 } - \boldsymbol { v } _ { 0 } ^ { E } \rVert =$ $\lVert \boldsymbol { v } _ { 0 } ^ { S } - \left( \boldsymbol { v } _ { 0 } ^ { S } \right) ^ { E } \rVert$ . However, affected by projection distortion, the equation no longer holds after projection back to the image, that is, $\lVert u _ { 0 } - u _ { 0 } ^ { E } \rVert \neq \lVert u _ { 0 } ^ { S } - \left( u _ { 0 } ^ { S } \right) ^ { E } \rVert$ , so $\beta _ { 2 D T } ( \cdot )$ cannot satisfy (15). Nevertheless, due to the edge symmetry, we found that the nearest edge distance ofpoint $v _ { 0 } .$ , noted as $\mathrm { B } _ { 3 D T } ( v _ { 0 } )$ , satisfies

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/d0dd7b0660cc92be64750acb01aee85b5d4441b1f8a4bfaafca91b18e23d3c6a.jpg)

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/7d7a5040e0c8b1a27a4abdd7178d842ab866a51a7c7fc76510bb87a155a8b0e7.jpg)

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/d9669088b25d2c8019d1a98dd29dd9267131b2627b9bf9f443693850e361a258.jpg)  
Fig. 4. (a) The edge distances of symmetric points are no longer equal after projection distortion. (b) Linearization of the reduction mapping of the edge point.

$$
\forall v _ { 0 } , v _ { 1 } , { \mathrm { i f } } v _ { 1 } = S ( v _ { 0 } , Q ) = v _ { 0 } ^ { S } , { \mathrm { t h e n B } } _ { 3 D T } ( v _ { 0 } ) = \mathrm { B } _ { 3 D T } ( v _ { 1 } )\tag{18}
$$

where the definition of $\mathrm { B } _ { 3 D T } ( \cdot )$ is

$$
\mathrm { B } _ { 3 D T } ( v _ { 0 } ) = \operatorname* { m i n } _ { v \in V _ { e } } \| v - v _ { 0 } \| , V _ { e } = \{ \mathcal { P } ^ { \dagger } ( u _ { e } ) | u _ { e } \in U _ { e } \}\tag{19}
$$

But unlike on the image, the computation of $\mathrm { B } _ { 3 D T } ( v _ { 0 } )$ in 3D space needs traversing over every edge point $v \in V _ { e }$ to find the nearest one in each iteration, which makes the computational cost unacceptable again.

Our solution is to combine the advantages of both $\beta _ { 2 D T } ( \cdot )$ and $\mathrm { B } _ { 3 D T } ( \cdot )$ , and proposing an Improved-DT descriptor. To make it symmetric projection invariant under certain conditions while preserving the lightweight property of query, we assume that the recovery mapped point $\mathcal { P } ^ { \dagger } ( u _ { 0 } ^ { \hat { E } } )$ of the nearest edge pixel $u _ { 0 } ^ { E }$ of pixel $u _ { 0 }$ in the image is exactly the point $v _ { 0 } ^ { E }$ in 3D space, which is the nearest edge point of $v _ { 0 } = \mathcal { P } ^ { \dagger } ( u _ { 0 } )$ . That is

$$
v _ { 0 } ^ { E } = \underset { v \in V _ { e } } { \arg \operatorname* { m i n } } \left\| v - v _ { 0 } \right\| = \mathcal { P } ^ { \dagger } ( u _ { 0 } ^ { E } ) , u _ { 0 } ^ { E } = \underset { u \in U _ { e } } { \arg \operatorname* { m i n } } \left\| u - u _ { 0 } \right\|\tag{20}
$$

The motivation of (20) is to replace $v _ { 0 } ^ { E } = ( \mathcal { P } ^ { \dagger } ( u _ { 0 } ) ) ^ { E }$ by its approximation $\widehat { v _ { 0 } ^ { E } } = \mathcal { P } ^ { \dag } ( u _ { 0 } ^ { E } ) \mathrm { s o }$ that we do not have to iterate over edge points to get the nearest point in 3D space. Further, we can define the descriptor

$$
\beta _ { 3 D T } ( u _ { 0 } ) = \left\| \mathcal { P } ^ { \dagger } ( u _ { 0 } ^ { E } ) - v _ { 0 } \right\| \approx \mathrm { B } _ { 3 D T } ( v _ { 0 } ) , v _ { 0 } = \mathcal { P } ^ { \dagger } ( u _ { 0 } )\tag{21}
$$

Under assumption (20), $\beta _ { 3 D T } ( u _ { 0 } )$ is symmetric projection invariant and can be obtained by simply querying $u _ { i } ^ { E }$ and taking $\boldsymbol { v } _ { i } ^ { E } = \mathcal { P } ^ { \dagger } ( \boldsymbol { u } _ { i } ^ { E } ) . ~ \boldsymbol { u } _ { i } ^ { E }$ can be obtained by slightly modifying the original distance transform algorithm: Save not only each pixel’s shortest distance from the edge but also the corresponding edge pixel coordinate.

## C. Further Acceleration of the Optimization Process

Using the Improved-DT descriptor $\beta _ { 3 D T } ( \cdot )$ , the cost function is

$$
\mathrm { f } _ { s y m } ( Q ) = \sum _ { u _ { i } } \left( \left\| \mathcal { P } ^ { \dagger } ( u _ { i } ^ { E } ) - v _ { i } \right\| - \left\| \mathcal { P } ^ { \dagger } \left( ( u _ { i } ^ { S } ) ^ { E } \right) - v _ { i } ^ { S } \right\| \right) ^ { 2 }\tag{22}
$$

In each iteration during the optimization, $\mathrm { f } _ { s y m } ( Q )$ needs to be recomputed with $Q$ changed and the sampling points $\{ u _ { i } \}$

Fig. 5. (a) Sampling edge points (left) and its cost function (right). (b) Uniform sampling (left) and its loss function (right).

fixed. Although $u _ { i } ^ { S }$ still changes with the update, $( u _ { i } ^ { S } ) ^ { E }$ can be obtained directly by query, which significantly speeds up the optimization process. To further accelerate the process, when computing every time-consuming step (13.b) in nonlinear mapping $\mathcal { P } _ { - } ^ { \dagger } ( \bar { u } _ { 0 } ^ { E } )$ , since $\Theta ( v _ { 0 } ) = { v _ { 0 } } ^ { T } Q { \dot { v } } _ { 0 } = 0$ is already satisfied and $u _ { 0 } ^ { E }$ is near $u _ { 0 } .$ , consider the linearization of $\Theta ( \dot { u _ { 0 } ^ { E } } ) = 0$ at $v _ { 0 } \colon$

$$
\begin{array} { r l r } {  { \Theta ( v _ { 0 } ^ { E } ) = \Theta ( v _ { 0 } + ( v _ { 0 } ^ { E } - v _ { 0 } ) ) = 0 } } \\ & { } & \\ & { } & { = \Theta ( v _ { 0 } ) + ( v _ { 0 } ^ { E } - v _ { 0 } ) ^ { T } \cdot \mathrm { g r a d } \Theta ( v _ { 0 } ) + \mathrm { o } \| v _ { 0 } ^ { E } - v _ { 0 } \| } \\ & { } & \\ & { } & { = { v _ { 0 } } ^ { T } Q v _ { 0 } + ( v _ { 0 } ^ { E } - v _ { 0 } ) ^ { T } \cdot 2 Q v _ { 0 } + \mathrm { o } \| v _ { 0 } ^ { E } - v _ { 0 } \| } \\ & { } & \\ & { } & { \approx 2 ( v _ { 0 } ^ { E } ) ^ { T } Q v _ { 0 } = 2 \bar { \Theta } _ { 0 } ( v _ { 0 } ^ { E } ) } & { ( 2 \mathfrak { I } } \end{array}\tag{}
$$

where $_ { 0 } \lvert | v _ { 0 _ { - } } ^ { E } - v _ { 0 } \rvert |$ is the Peano remainder. The geometric meaning of $\bar { \Theta } _ { 0 } ( v _ { 0 } ^ { E } ) = ( v _ { 0 } ^ { E } ) ^ { T } \cdot Q v _ { 0 } = 0$ is: $v _ { 0 } ^ { E }$ lies on $Q \mathrm { { ' s } }$ tangent plane $Q v _ { 0 }$ at $v _ { 0 } ,$ as shown in $\mathrm { F i g . 4 ( b ) }$ . Replacing (13.b) by $\bar { \Theta } _ { 0 } ( v _ { 0 } ^ { E } ) = 0$ is equivalent to approximating the intersection of a ray with a tangent plane instead of the quadric surface, whose solution has explicit linear representation [8]. Let the approximate calculation be $\widehat { \mathcal { P } ^ { \dagger } } ( \cdot )$ , then the loss function is

$$
f _ { s y m } ( Q ) = \sum _ { u _ { i } } \left( \left\| \widehat { \mathcal { P } ^ { \dagger } } ( u _ { i } ^ { E } ) - v _ { i } \right\| - \left\| \widehat { \mathcal { P } ^ { \dagger } } \left( ( u _ { i } ^ { S } ) ^ { E } \right) - v _ { i } ^ { S } \right\| \right) ^ { 2 }\tag{24}
$$

That is, at each iteration, the sampling point mapping $v =$ $\mathcal { P } ^ { \dagger } ( u )$ is computed accurately and the edge point mapping $\bar { \boldsymbol { v } } ^ { E }$ ≈ $\hat { \mathcal { P } } ^ { \dagger } ( u ^ { E } )$ is computed approximately, which further accelerates of the optimization process.

## D. Strategies of Sampling Points

We have described in detail the process of constructing the descriptors and how to accelerate the optimization process, leaving only how to obtain the sampling points $\{ u _ { i } \}$ . Due to remainder o $| | v _ { 0 } ^ { E } - v _ { 0 } | |$ in (23), the linearization approximation is only valid when $| | \dot { v } _ { 0 } ^ { E } - v _ { 0 } | | \approx 0$ . Hence, we use two point sampling strategies:

One is to sample the corner points, which can be considered as a stricter edge point, and can guarantee $\lvert | v _ { 0 } ^ { E } - v _ { 0 } \rvert | \approx 0$ But also because of the nearness, theoretically there will be $\beta _ { 3 D T } ( u _ { i } ) \equiv 0$ , which may cause the gradient vanishing problem near the optimal value, as in Fig. 5(a). The other is to uniformly sample points in the bounding boxes. As in Fig. 5(b), The gradient problem is significantly improved, but $| | \boldsymbol { v } _ { 0 } ^ { \check { E } } - \boldsymbol { v } _ { 0 } | | \approx 0$ is not quite satisfied. We find that using corner points together with a few uniform points achieves the best effectiveness in our experiments.

## VI. EXPERIMENTS

## A. Backgrounds

To fully verify the single-frame initialization, texture orientation optimization, and complete system performance proposed in this letter, we conduct experiments both on public datasets, and author-recorded real robot datasets. TUM RGB-D [19] and ICL-NUIM [20] datasets are widely used in SLAM, which cover both room-level and desktop-level environments. To better reflect the effectiveness on mobile robot, we conduct experiments on a Turtlebot3 with a Kinect camera operating in a home-like environment, as described in [14]. We take every five images to perform object detection with YOLOv3 to get bounding boxes.

We use the indicators IoU and Rot(deg) to fully evaluate the mapping effects. The IoU evaluates the Intersection over Union between their circumscribed cubes of estimated object and ground-truth object. For objects with symmetry, Rot(deg) evaluates the minimum rotation angle required to align the estimated object’s three rotation axes with any axis of the ground-truth object to a straight line. For a trajectory, the above metrics are the average values of all objects’ evaluation results. Even though our methods can initialize with only one observation, SVD and QuadricSLAM need at least three observations. To make the experiments comparable, we consider those objects with at least three observations and filter those partial bounding boxes (those near to the image edges less than 30 pixels), so that all objects can successfully initialize.

Since the plane extraction is not our focus, in the experiments, we annotated the support plane in the world coordinate system and then transformed it into the local coordinates of each frame to get the ground-truth planes. In this way, we can know the accuracy limit of our proposed methods. In actual scenarios, support planes can be extracted from point cloud generated from SLAM [2], or directly through a plane SLAM system [21]. For wheeled mobile robots, when considering objects on the ground, the ground plane parameters can be obtained after calibrating the camera’s external parameters related to the ground before starting.

## B. Single-frame Object Initialization

We compare our result with the initialization method SVD of the state-of-art algorithm QuadricSLAM [7] and the initialization method of CubeSLAM [5]. The SVD method requires at least three frames of observations. We put together all observations of the object for SVD initialization in the experiment. Like ours, CubeSLAM introduces the supporting plane to constrain the orientation of the object. We take the experimental results on indoor datasets given in the CubeSLAM letter for comparison. We take the IoU between the estimated object and the groundtruth object as the benchmark, and average over all objects in the trajectory.

Table I and Fig. 6 left show the results. The SVD method not only requires a larger number of observations, but the accuracy is also lower. Especially in the robot trajectory, the forward motion of the mobile robot is difficult to produce a sufficient angular difference between observations, resulting in an IoU of only 0.5%. Compared with the SVD method, the initialization of CubeSLAM only needs one observation and obtains better results. CubeSLAM needs extraction of line features to calculate vanish points, which requires the object surface to have obvious straight lines. Ours not only requires one observation, but also has no requirements for the line features of the object. It has a better adaptability to the texture type. Even with the 1:1:1 scale proportional constraint (see Init1-1), Ours achieves an average IoU of 16.3%. With semantic object semantic prior (see InitP), it rises up to 21.8%, which is a significant increase of 13% over SVD. There is also a 15.4% increase over the published data of CubeSLAM on ICL room2. Fr3\_cabinet contains a cuboid object only, and CubeSLAM shows the best result.

TABLE I  
SINGLE FRAME OBJECT INITIALIZATION IOU & ORIENTATION
<table><tr><td>Datasets (#)</td><td>SVD IOU/θ</td><td>Cube IOU/θ</td><td>Init1-1 IOU/θ</td><td>InitP IOU/θ</td><td>InitPT IOU/θ</td></tr><tr><td>ICL room2 (4)</td><td>0.072/32.8</td><td>0.33/ -</td><td>0.363/17.0</td><td>0.484/16.0</td><td>0.478/14.0</td></tr><tr><td>Fr1_desk (13)</td><td>0.066/45.0</td><td>-/-</td><td>0.091/14.5</td><td>0.11/11.7</td><td>0.106/12.6</td></tr><tr><td>Fr2_desk (12)</td><td>0.13/44.7</td><td>-/-</td><td>0.184/10.9</td><td>0.198/13.4</td><td>0.192/8.8</td></tr><tr><td>Fr2_dishes (4)</td><td>0.118/ -</td><td>-/-</td><td>0.138/ -</td><td>0.312/ -</td><td>0.312/ -</td></tr><tr><td>Fr3 3_cabinet (1)</td><td>0.254/20.0</td><td>0.46/ -</td><td>0.296/42.9</td><td>0.345/1.7</td><td>0.344/1.4</td></tr><tr><td>Real-robot (8)</td><td>0.005/30.6</td><td>-/-</td><td>0.148/23.2</td><td>0.228/17.1</td><td>0.232/15.9</td></tr><tr><td>Average</td><td>0.083/39.9</td><td>-/-</td><td>0.163/16.2</td><td>0.218/13.6</td><td>0.215/11.9</td></tr></table>

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/2a9fe19ac83182344cb950e15227ca5c3ba580e1aa9ca63a07b30d36b06dae1f.jpg)

Fig. 6. Examples of object initialization and trajectory estimation.  
![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/4bcba34915ab2bc1f0ec62b833cf8d6c99e3e29de705c198abf1cb855df73366.jpg)  
Fig. 7. Symmetry cost functions of different descriptors.

## C. Orientation Estimation Based on Texture Symmetry

To verify the effectiveness of the Improved-DT descriptor proposed in this letter for representing the symmetry of objects, we analyze the cost compared with Grayscale, BRIEF, and DT descriptors as in Fig. 7. The vertical line marks the groundtruth $( \theta _ { t r u t h } )$ yaw angle of the object in the current frame. The Improved-DT descriptor has an obvious global optimum near the ground-truth value, while others have more than one local optimum. With the change of the object’s orientation, its error changes smoother and more significantly. As a result, there are better local gradients to constrain in the optimization process. Though we demonstrate all the results from −90 to 90 degree, in the actual SLAM system we only use the ±30 degree range to avoid too much angular distortion.

Next, we use the Improved-DT descriptor to estimate the orientation ofthe object as in Fig. 8. Table I and II show the object orientation error after single-frame initialization (see InitPT), and multiple-frames optimization (see OursPT) separately. We only consider the texture constraint of the first observation of each object to avoid constraint conflicts. The orientation result of SVD initialization and QuadricSLAM are given as reference. Although SVD initialization and QuadricSLAM can solve a complete ellipsoid, they do not explicitly constrain the orientation of the object, so the average orientation error is relatively large, reaching 39.9 degrees and 31.7 degrees, respectively. After introducing object supporting constraints and default scale (Init 1-1), real scale proportional prior (InitP) and texture (InitPT), the orientation error was improved and finally achieved 11.9 deg, which is a 64% improvement compared with SVD. With multiple-frame optimization, the orientation (OursPT) was improved to 11.5 deg, and its IoU was increased from 0.215 to 0.286. This accuracy is sufficient for semantic navigation applications involving object orientation, such as commands like “moving to the front of the table” and “moving to the side of the bench”.

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/bfde26529f17c61cb72f56b209515717a1bb3e358db88ecb92d133bfb16c746a.jpg)  
(a)

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/bd7f89b2c9c7cb8d96a994f4346546d1de4506cb48a1d5f2b914aa61ea543ead.jpg)  
(b)

![](images/2022_SO-SLAM__Semantic_Object_SLAM_With_Scale_Proportional_an/8b561f4d7c04fd1ba097bc1e3019ac9e62f91a264d0c87e2e07e2449baaf07ee.jpg)  
(c)  
Fig. 8. Results before and after symmetry constraints (a) object landmarks (b) cost function value (typical case) (c) cost function value (failure case).

MULTIPLE-FRAME OBJECT ESTIMATION IOU & ORIENTATION  
TABLE II
<table><tr><td>Datasets</td><td>Quadric IOU/θ</td><td>OursP IOU/θ</td><td>OursPT IOU/θ</td></tr><tr><td>ICL room2</td><td>0.082/26.96</td><td>0.489/31.60</td><td>0.438/12.23</td></tr><tr><td>Fr1_desk</td><td>0.071/36.97</td><td>0.135/14.38</td><td>0.131/13.23</td></tr><tr><td>Fr2_desk</td><td>0.172/30.77</td><td>0.330/9.64</td><td>0.334/8.70</td></tr><tr><td>Fr2_dishes</td><td>0.293/-</td><td>0.375/-</td><td>0.375/-</td></tr><tr><td>Fr3_cabinet</td><td>0.255/18.31</td><td>0.316/1.03</td><td>0.317/0.95</td></tr><tr><td>Real-robot</td><td>0.034/28.76</td><td>0.308/13.33</td><td>0.343/13.71</td></tr><tr><td>Average</td><td>0.120/31.74</td><td>0.285/14.12</td><td>0.286/11.47</td></tr></table>

We also found some failure cases that could guide future work. On the ICL-NUIM datasets, the orientation error of the large objects such as chair and bench reach about several degree. However, in fr1\_desk and fr2\_desk, small objects’ orientation such as books and keyboards reach 30–40 degrees. We find that the estimation of the center and scale of the small objects is poor, which makes the three-dimensional symmetry point in the texture constraint not accurate. The occlusion also causes decrease in some clustered environments as in Fig. 8(c), which reveals the necessity for multi-frame optimization.

## D. Multiple-frame Optimization

To compare with the state-of-art algorithms, we reproduced the performance of QuadricSLAM, which is a state-of-art monocular object SLAM system with quadrics. QuadricSLAM uses SVD method to initialize objects and then optimize in the back-end. Data association problem decides which objects in the map an observation belongs to. There has been work [22] concentrating on the problem. Our previous work [14] has also discussed combining quadric model with a nonparametric pose graph to solve the data association. QuadricSLAM letter uses manually annotated data association in the experiments. As data association is not the focus of this letter, we also use manually annotated data association for both QuadricSLAM and ours to testify the best effectiveness in the experiments.

TABLE III  
TRAJECTORY ERROR. RMSE (M)
<table><tr><td>Datasets</td><td>VO</td><td>ORB2</td><td>Quadric</td><td>Cube</td><td>Ours</td></tr><tr><td>ICL room2</td><td>0.0270</td><td>0.0266</td><td>0.0574</td><td>0.0300</td><td>0.0264</td></tr><tr><td>Fr3_cabinet</td><td>0.0829</td><td>0.0574</td><td>0.1185</td><td>0.1613</td><td>0.0816</td></tr></table>

We use an optimization framework to exploit observations from multiple frames using (1). Table II shows the IoU and orientation error. Both IoU and orientation of QuadricSLAM is improved compared to SVD initialization. With plane supporting constraints and semantic prior, ours without texture (see OursP) achieved better IoU, orientation of 0.285 and 14.12 deg. After further introducing texture symmetry constraints (see OursPT) on the above basis, the orientation was improved to 11.47 deg, with a slightly improvement on IoU. Totally, ours was improved by 138.3% IoU and 63.9% orientation compared with QuadricSLAM. In fr2\_dishe s and fr3\_cabinet, there is smaller gap compared with QuadricSLAM, because in these datasets, the camera trajectory surrounds the object and produces sufficient observations, which is beneficial to QuadricSLAM’s optimization.

We show trajectory errors in Table III. We use RGB-D version ORB-SLAM2 as baseline and use loop closure-disabled version as odometry (VO) for QuadricSLAM and ours. ORB-SLAM2 fails to complete the full fr3\_cabinet trajectory. To fairly compare, we give an evaluation on the truncated trajectory as in . 6 right. Ours achieves better results than VO in both datasets and achieves even a slightly better result than ORB-SLAM2 in ICL room2, with new constraints from four objects, but has a certain gap in fr3\_cabinet, with limited constraints from only one object. Compared with object SLAM baselines, ours outperforms QuadricSLAM which proves the effectiveness of the proposed semantic constraints. CubeSLAM gives evaluation on ICL room2 and fr3\_cabinet in the letter [5]. We select the data for ICL room2 and get a truncated CubeSLAM result in fr3\_cabinet using the open-source code, which is slightly better than the data in the letter (0.17m). Although ours outperforms CubeSLAM in both datasets, we need to point out that to get the scale, CubeSLAM assumes the pose of the first frame is known, while we use depth information for odometry as the same as QuadricSLAM does in the letter [7].

## E. Computation Analysis

We implemented the algorithm in C++ and used the g2o library for the graph optimization. Except that the object detection runs on GTX 1660 s with 33 Hz, the proposed algorithm runs in

TABLE IV  
RUNNING TIME PERFORMANCE
<table><tr><td></td><td>Init w/o tex</td><td>Init w/t tex</td><td>Multiple-frames w/o tex</td><td>Multiple-frames w/t tex</td></tr><tr><td>Run Time (ms)</td><td>5.8±1.4</td><td>116.5±28.4</td><td>6.9±4.5</td><td>295.3±230.1</td></tr><tr><td>Average Observations</td><td>1</td><td>1</td><td>11.8</td><td>11.8</td></tr></table>

real-time on a common CPU. We present the running-times on a laptop with an Intel Core i5-7200U 2.5GHz CPU, 8GB RAM, on the fr1\_desk dataset in Table IV.

## F. Discussion

We did not find significant trajectory accuracy improvements with the introduction of objects. Both QuadricSLAM and our previous work [14] has shown the same conclusion. We think that it is because the odometry data provided by ORB-SLAM2 is already relatively accurate. We notice that the trajectory improvement mainly results from the proportional scale and supporting plane constraints. Currently, symmetrical points are not maintained in the map, so coupling them with point-based SLAM system will be promising future work to further improve trajectory accuracy. Besides trajectory accuracy, we believe that object features have great potential for bringing high-level understandings and robustness into SLAM systems such as dealing with long-term changes, social navigation, and robotic manipulation.

The texture orientation constraint is still closely related to the accuracy of the quadric surface itself before optimization. We suppose that decoupling the orientation estimation from other degrees of freedom will further improve the orientation estimation. We have explored several types of symmetry descriptors. We leave it as future work to explore other more complex manually designed descriptors, e.g., FREAK [24]. We discuss objects horizontally placed on the support plane, so it is a valuable future work to explore how to estimate objects pose with 3D rotation.

## VII. CONCLUSION

This letter proposes a monocular object SLAM system, which uses quadrics to model objects and builds an object-level map to represent the environment. This letter introduces three spatial structure constraints, including scale proportional constraints, symmetrical texture constraints and supporting plane constraints. Based on these constraints, this letter proposes two new modules- single frame initialization, and orientation fine optimization, which significantly reduce the object SLAM systems’ dependences on the number and change of observations. These methods are expected to make object SLAM better adapt to the real complex environments. The symmetry constraints on object orientation provide information for semantic navigation and help the estimation of the scale and center of objects. Considering future work, it will be promising to further explore more types of objects spatial constraints and semantic priors to help SLAM systems.

## REFERENCES

[1] C. Cadena et al., “Past, present, and future ofsimultaneous localization and mapping: Toward the robust-perception age,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1309–1332, Dec. 2016.

[2] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardós, “ORB-SLAM: A versatile and accurate monocular SLAM system,” IEEE Trans. Robot., vol. 31, no. 5, pp. 1147–1163, Oct. 2015.

[3] R. F. Salas-Moreno, R. A. Newcombe, H. Strasdat, P. H. J. Kelly, and A. J. Davison, “SLAM++: Simultaneous localisation and mapping at the level ofobjects,” in Proc. IEEE Conf. Comput. Vis.pattern Recognit., 2013, pp. 1352–1359.

[4] M. Runz, M. Buffier, and L. Agapito, “Maskfusion: Real-time recognition, tracking and reconstruction of multiple moving objects,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2018, pp. 10–20.

[5] S. Yang and S. Scherer, “CubeSLAM: Monocular 3D object SLAM,” IEEE Trans. Robot., vol. 35, no. 4, pp. 925–938, Aug. 2019.

[6] C. Rubino, M. Crocco, and A. Del Bue, “3D object localisation from multi-view image detections,” IEEE Trans. pattern Anal. Mach. Intell., vol. 40, no. 6, pp. 1281–1294, Jun. 2018.

[7] L. Nicholson, M. Milford, and N. Sünderhauf, “Quadricslam: Dual quadrics from object detections as landmarks in object-oriented SLAM,” IEEE Robot. Automat. Lett., vol. 4, no. 1, pp. 1–8, Jan. 2019.

[8] R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision. Cambridge, U.K.: Cambridge Univ. Press, 2003.

[9] V. Gaudillière, G. Simon, and M. O. Berger, “Camera relocalization with ellipsoidal abstraction of objects,” in Proc. IEEE Int. Symp. Mixed Augmented Reality, 2019, pp. 8–18.

[10] K. Ok et al., “Robust object-based SLAM for high-speed autonomous navigation,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 669–675.

[11] M. Hosseinzadeh, Y. Latif, T. Pham, N. Suenderhauf, and I. Reid, “Structure aware SLAM using quadrics and planes,” in Proc. Asian Conf. Comput. Vis., Springer, Cham, 2018.

[12] N. Jablonsky, M. Milford, and N. Sünderhauf, “An orientation factor for object-oriented SLAM,” 2018, arXiv:1809.06977.

[13] M. Hosseinzadeh, K. Li, Y. Latif, and I. Reid, “Real-time monocular object-model aware sparse SLAM,” in Proc. Int. Conf. Robot. Automat., 2019, pp. 7123–7129.

[14] Z. Liao et al., “RGB-D object SLAM using quadrics for indoor environments,” Sensors, vol. 20, no. 18, 2020, Art. no. 5150.

[15] Z. Liao et al., “Object-oriented SLAM using quadrics and symmetry properties for indoor environments,” 2020, arXiv:2004.05303.

[16] J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, “You only look once: Unified, real-time object detection,” in Proc. IEEE Conf. Comput. Vis. pattern Recognit., 2016, pp. 779–788.

[17] F. Tschopp, J. Nieto, R. Siegwart, and C. Cadena, Superquadric Object Representation for Optimization-based Semantic SLAM. Switzerland: ETH Zurich, Autonomous System Lab, 2021.

[18] S. Boyd, S. P. Boyd, and L. Vandenberghe. Convex Optimization. Cambridge, U.K., Cambridge Univ. Press, 2004.

[19] J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2012, pp. 573–580.

[20] A. Handa, T. Whelan, J. McDonald, and A. J. Davison, “A benchmark for RGB-D visual odometry, 3D reconstruction and SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2014, pp. 1524–1531.

[21] S. Yang, Y. Song, M. Kaess, and S. Scherer, “Pop-up SLAM: Semantic monocular plane SLAM for low-texture environments,” in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2016, pp. 1222–1229.

[22] S. L. Bowman, N. Atanasov, K. Daniilidis, and G. J. Pappas, “Probabilistic data association for semantic SLAM,” in Proc. IEEE Int. Conf. Robot. Automat., 2017, pp. 1722–1729.

[23] S. Thrun and B. Wegbreit, “Shape from symmetry,” in Proc. 10th IEEE Int. Conf. Comput. Vis., 2005, pp. 1824–1831.

[24] A. Alahi, R. Ortiz, and P. Vandergheynst. “Freak: Fast retina keypoint,” in Proc. IEEE Conf. Comput. Vis. Pattern Recognit., 2012, pp. 510–517.

[25] P. Gay, C. Rubino, V. Bansal, and A. Del Bue, “Probabilistic structure from motion with objects (PSfMO),” in Proc. IEEE Int. Conf. Comput. Vis., 2017, pp. 3075–3084.

[26] I. Vaisman,Analytical Geometry, vol. 8, World Scientific Publishing, 1997.