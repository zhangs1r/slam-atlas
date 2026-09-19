/* ============================================================
   data/course-catalog.js — SLAM 论文精读课 · 课程馆数据
   ------------------------------------------------------------
   三层结构：分馆(group) → 来源块(collection) → 课程页(lesson)
   续课/新增块时，只改这个文件即可，不必动 index.html。
   ============================================================ */

window.PAPERLESSON_DATA = {
  /* 首页「当前主线」指向的 collection id */
  featuredCollectionId: "stage-d",

  /* ---------- 分馆 ---------- */
  groups: [
    {
      id: "papers",
      kind: "论文馆",
      title: "SLAM 论文精读馆",
      emoji: "📄",
      description: "按语料库阅读路线的阶段（A–K）分块。每个「来源块」对应一个阶段，块内课程按认知依赖顺序排列；一篇论文可以拆成多节课，也可以几篇同类论文合成一节。",
      meta: ["按阶段分块", "255 篇语料库", "已开到阶段 D · 激光主干"]
    },
    {
      id: "qa",
      kind: "答疑馆",
      title: "追问答疑馆",
      emoji: "🙋",
      description: "读课过程中冒出来的疑问，凡是需要独立推导、跨知识点、或一次问好几个的，都在这里单独成页。这里不占用主线课号（主线永远是 0001、0002、0003…），页面按「源自哪一节课」归档，并与源课双向互链。",
      meta: ["不占主线编号", "按源课归档", "与源课双向互链"]
    },
    {
      id: "tools",
      kind: "索引馆",
      title: "路线图与索引馆",
      emoji: "🗺️",
      description: "放不放「课程」的东西：全量阅读路线图、术语表、阶段总览。它们是所有课程的配套索引，适合当工具书反复查。",
      meta: ["全量路线图", "术语表", "速查与总览"]
    }
  ],

  /* ---------- 来源块 ---------- */
  collections: [
    {
      id: "stage-a",
      groupId: "papers",
      kind: "阶段精读",
      title: "阶段 A · 先建立坐标系",
      subtitle: "综述与全景 · 255 篇语料库的第一季",
      description: "第 0001–0016 课。把语料库里 20 篇综述与全景文章拆成 16 节课：先给整个领域画地图（2016 纲领 → 2021 综述 → 2012 视觉综述），再分头进入激光、学习式方法、场景识别与回环、3D 高斯、事件相机。读完这一季，你会拥有后面所有课程都要用到的共同词汇表。",
      quickOpen: "start lessons\\0001-导览-255篇论文的阅读地图.html",
      meta: ["16 节课", "覆盖 20 篇综述", "约 7 小时"]
    },
    {
      id: "stage-b",
      groupId: "papers",
      kind: "阶段精读",
      title: "阶段 B · 打地基",
      subtitle: "数学、后端与地图表示 · 第二季 19 篇",
      description: "第 0017–0039 课。第一季在讲「怎么把问题写成图」，这一季讲「图上的边怎么算得又快又准、算完的结果怎么存下来」。五个模块：后端与优化（BA / GraphSLAM / g2o / 点云 BA / PCL）→ 几何与重建（VO 教程 / SGM / COLMAP）→ 惯性与连续时间（MSCKF / 预积分 / 连续时间）→ 空间表示（KinectFusion / Voxel Hashing / Voxblox）→ 神经隐式表示（NeRF / Instant NGP / Plenoxels）。第一季欠下的两笔债（BA 稀疏性与预积分）在这一季还清。",
      quickOpen: "start lessons\\0017-导览-第二季开场-打地基.html",
      meta: ["23 节课", "覆盖 19 篇论文", "约 17 小时"]
    },
    {
      id: "stage-c",
      groupId: "papers",
      kind: "阶段精读",
      title: "阶段 C · 视觉主干",
      subtitle: "特征法 / 直接法 / VIO · 第三季 42 篇",
      description: "第 0040 课起。前两季是「知道有什么」与「知道为什么能算」，这一季回答「怎么把它跑起来」：从 2007 年 EKF 时代的 MonoSLAM，到特征法的 ORB-SLAM 三兄弟，到直接法的 DTAM / LSD-SLAM / DSO，再到 VIO 的优化派与滤波派之争，最后走到真实世界的动态物体、光照变化、GNSS 融合与算力约束。第二季欠下的两笔债（「提取 vs 不提取特征」与「IMU 初始化」）在这一季还清。本季新增规矩：凡是晦涩抽象、原文又没有好图的地方，都配一张具象自绘图。",
      quickOpen: "start lessons\\0040-导览-第三季开场-视觉主干.html",
      meta: ["33 节课", "覆盖 42 篇论文", "89 张具象自绘图"]
    },
    {
      id: "stage-d",
      groupId: "papers",
      kind: "阶段精读",
      title: "阶段 D · 激光主干",
      subtitle: "LOAM → FAST-LIO2 → 4D 雷达 · 第四季 32 篇",
      description: "第 0073 课起。第三季把视觉那台机器拆完了，这一季换一个传感器重看一遍：激光雷达直接给你三维点，省掉了尺度麻烦，却带来「一帧扫描要时间」与「点太多还一直长」两个新麻烦。主线是从「挑点」走到「全都要」——LOAM 挑角点与平面点，FAST-LIO2 干脆不挑了；而支撑这个转变的，是一棵能动态插入删除的树。第一季埋下、第二季推迟、第三季再次立起的那笔 ikd-Tree 之债，在这一季第 0084 课正面还清。",
      quickOpen: "start lessons\\0073-导览-第四季开场-激光主干.html",
      meta: ["22 节课", "覆盖 32 篇论文", "50 张具象自绘图"]
    },
    {
      id: "qa-0002",
      groupId: "qa",
      kind: "追问答疑",
      title: "溯源 0002 · 从 MAP 到非线性最小二乘",
      subtitle: "那一节最硬的四行公式，拆开揉碎",
      description: "第 0002 课第 3 节那四行公式的展开版：先补五个数学零件，再逐式拆解，最后用走廊里的一台机器人把 x*=13.2 m 手算到底。与 0002 课原位置（(4) 式之后）互链，随时能原路返回。",
      quickOpen: "start qa\\qa-0002-MAP到非线性最小二乘.html",
      meta: ["1 页答疑", "源自 0002", "含手算算例"]
    },
    {
      id: "roadmap",
      groupId: "tools",
      kind: "路线图",
      title: "255 篇论文阅读路线图",
      subtitle: "11 个阶段 · 12 个技术大类 · 难度与重要性标注",
      description: "语料库全量路线图，带时间轴导航、技术大类筛选、难度阈值筛选与实时搜索。不确定「下一篇该读什么」时，先回这里。",
      quickOpen: "open ..\\outputs\\论文阅读路线图.html",
      meta: ["255 篇", "可交互筛选", "11 阶段 / 12 类"]
    }
  ],

  /* ---------- 课程页 ---------- */
  lessons: [
    {
      id: "0001",
      path: "lessons/0001-导览-255篇论文的阅读地图.html",
      paper: "stage-a",
      title: "导览：255 篇论文的阅读地图",
      subtitle: "课程总纲 · 三种读法 · 路线图勘误",
      emoji: "🗺️",
      duration: "20 分钟",
      tags: ["导览", "方法论"],
      description: "先把整张地图铺开：11 个阶段怎么分、难度与重要性怎么读、三种读法怎么选，以及我们逐篇核对出的两处路线图归类错误。"
    },
    {
      id: "0002",
      path: "lessons/0002-PastPresentFuture-2016-SLAM的全景纲领.html",
      paper: "stage-a",
      title: "SLAM 的全景纲领（2016）",
      subtitle: "Toward the Robust-Perception Age",
      emoji: "🧭",
      duration: "40 分钟",
      tags: ["纲领", "形式化", "开放问题"],
      description: "领域最值得先读的一篇：从 MAP 到非线性最小二乘的标准形式化、三个时代的划分、回环改变了什么，以及「机器人需要 SLAM 吗 / SLAM 解决了吗」的正面回答。"
    },
    {
      id: "0003",
      path: "lessons/0003-SLAM综述2021-从滤波到优化的演进主线.html",
      paper: "stage-a",
      title: "机器人 SLAM 综述（2021）",
      subtitle: "从滤波到优化的四条演进主线",
      emoji: "📈",
      duration: "35 分钟",
      tags: ["EKF", "图优化", "演进主线"],
      description: "用 EKF-SLAM 的状态向量与协方差矩阵看清「为什么贵」，用 RBPF 的因式分解看清「省了什么」，最后拿到结论里的四句演进口号。"
    },
    {
      id: "0004",
      path: "lessons/0004-视觉SLAM早期综述2012-特征匹配与数据关联.html",
      paper: "stage-a",
      title: "视觉 SLAM 早期综述（2012）",
      subtitle: "特征、描述子、匹配与数据关联",
      emoji: "🔍",
      duration: "40 分钟",
      tags: ["特征", "数据关联", "回环铁律"],
      description: "钻进视觉 SLAM 最底层的链子：检测器与描述子全谱、短基线 vs 长基线匹配、NNR 判据、RANSAC，以及那条「假阳性不可挽回」的工程铁律。"
    },
    {
      id: "0005",
      path: "lessons/0005-动态环境下的激光SLAM-2022.html",
      paper: "stage-a",
      title: "动态环境下的激光 SLAM（2022）",
      subtitle: "给 NDT 加一个「静态权重」",
      emoji: "🚗",
      duration: "25 分钟",
      tags: ["动态环境", "NDT", "软加权"],
      description: "动态物体会怎样伤害点云配准、又怎样连带伤害回环检测；以及「给每个点一个静态概率」这种软加权思路为什么比硬剔除更稳。"
    },
    {
      id: "0006",
      path: "lessons/0006-LiDAR里程计综述2024-算法分类与九大挑战.html",
      paper: "stage-a",
      title: "LiDAR 里程计综述（2024）",
      subtitle: "按「传感器组合方式」分类与五类剩余挑战",
      emoji: "📡",
      duration: "40 分钟",
      tags: ["LiDAR", "松紧耦合", "退化"],
      description: "四条分类线、三阶段通用管线与松紧耦合的真正判据，并逐条精读五类 remaining challenges——尤其是「退化」与「降质」这对容易混的词。"
    },
    {
      id: "0007",
      path: "lessons/0007-固态激光雷达与纳米光子学2022-硬件演进.html",
      paper: "stage-a",
      title: "固态激光雷达与纳米光子学（2022）",
      subtitle: "SLAM 的「眼睛」是怎么进化的",
      emoji: "🔧",
      duration: "30 分钟",
      tags: ["硬件", "TOF/FMCW", "MEMS/OPA"],
      description: "三种测距原理与四类固态方案的演进；机械旋转被取代的三条理由；以及硬件形态如何决定算法范式。"
    },
    {
      id: "0008",
      path: "lessons/0008-激光SLAM室内导航横评2022.html",
      paper: "stage-a",
      title: "激光 SLAM 室内导航横评（2022）",
      subtitle: "把 7 种方法拉到同一个擂台",
      emoji: "⚖️",
      duration: "30 分钟",
      tags: ["横评", "scan-to-map", "工程参数"],
      description: "scan-to-scan 与 scan-to-map 的取舍、为什么 BoW 那套只对视觉有效，以及一次把传感器参数、场地、平台全部披露的真实对比实验。"
    },
    {
      id: "0009",
      path: "lessons/0009-DARPA地下挑战赛2024-真实世界的SLAM.html",
      paper: "stage-a",
      title: "DARPA 地下挑战赛（2024）",
      subtitle: "真实世界给 SLAM 泼的冷水",
      emoji: "🕳️",
      duration: "45 分钟",
      tags: ["实战", "Dirty Details", "多机"],
      description: "本季最该读的一节「泼冷水」：指标好看 ≠ 能跑、五重限定下的「问题已解决」、集中式 vs 分布式的架构哲学，以及一张 Dirty Details 检查表。"
    },
    {
      id: "0010",
      path: "lessons/0010-深度学习视觉定位与建图2024.html",
      paper: "stage-a",
      title: "深度学习视觉定位与建图（2024）",
      subtitle: "一张全景地图与它的边界",
      emoji: "🧠",
      duration: "45 分钟",
      tags: ["学习式", "重定位", "混合路线"],
      description: "按「2-D 地图 vs 3-D 地图」的顶层分类走一遍重定位，讲清监督/自监督/混合三种 VO 范式，并直面原文承认的「混合路线才是现阶段赢家」。"
    },
    {
      id: "0011",
      path: "lessons/0011-深度学习视觉SLAM2023-模块化与联合学习.html",
      paper: "stage-a",
      title: "深度学习视觉 SLAM（2023）",
      subtitle: "模块化、联合、置信度与主动学习",
      emoji: "🧩",
      duration: "45 分钟",
      tags: ["四分类", "匹配与检索", "黑箱"],
      description: "按「学习范围」分成 Modular / Joint / Confidence / Active 四类；重点讲透「深度学习为什么先吃下匹配和检索」，以及原文点名的六个仍不成熟环节。"
    },
    {
      id: "0012",
      path: "lessons/0012-动态环境视觉SLAM综述2024-从几何到语义.html",
      paper: "stage-a",
      title: "动态环境视觉 SLAM 综述（2024）",
      subtitle: "从几何到语义的演进",
      emoji: "🚶",
      duration: "40 分钟",
      tags: ["动态 SLAM", "语义", "代价"],
      description: "几何约束 → 语义信息的方法谱系，以及原文最硬的批评：桌面级 GPU 的代价、标签体系不等于物理真值、语义建图质量目前无法定量评估。"
    },
    {
      id: "0013",
      path: "lessons/0013-视觉场景识别教程2024-评测协议的坑.html",
      paper: "stage-a",
      title: "视觉场景识别教程（2024）",
      subtitle: "评测协议的十个坑",
      emoji: "📍",
      duration: "45 分钟",
      tags: ["VPR", "软真值", "R@100P"],
      description: "「同一地点」的两种定义、相似度矩阵如何反推采集轨迹、反直觉的软真值 GT^soft，以及 R@100P 为什么会失效。"
    },
    {
      id: "0014",
      path: "lessons/0014-回环检测综述2022-指标与假阳性铁律.html",
      paper: "stage-a",
      title: "回环检测综述（2022）",
      subtitle: "指标与假阳性铁律",
      emoji: "🔄",
      duration: "40 分钟",
      tags: ["回环检测", "PR 曲线", "铁律"],
      description: "四段流水线（而非「外观/几何/学习」三分类）、PR 曲线与 R_P100 的定义与缺陷，以及那句「一个假阳性就能让 SLAM 完全失败」——并与第 0013 课形成正面交锋。"
    },
    {
      id: "0015",
      path: "lessons/0015-3D高斯泼溅全景2025.html",
      paper: "stage-a",
      title: "3D 高斯泼溅全景（2025 + 2024）",
      subtitle: "从「是什么」到 SLAM 专章",
      emoji: "🎨",
      duration: "45 分钟",
      tags: ["3DGS", "显式 vs 隐式", "SLAM 专章"],
      description: "为什么显式表示赢了速度、高斯六个属性与可微光栅化流程、2025 综述的六分类与五个 SLAM 代表工作，以及这条线卡在哪些地方。"
    },
    {
      id: "0016",
      path: "lessons/0016-事件相机-从原理到SLAM.html",
      paper: "stage-a",
      title: "事件相机：从原理到 SLAM",
      subtitle: "2022 总览 + 事件 SLAM 专述",
      emoji: "⚡",
      duration: "45 分钟",
      tags: ["事件相机", "DVS", "CMax"],
      description: "事件相机的触发原理与九种数据表示、四大方法学分类（特征/直接/运动补偿/深度学习）与各自失效边界，以及这条线为什么至今没有普及。"
    },
    {
      id: "0017",
      path: "lessons/0017-导览-第二季开场-打地基.html",
      paper: "stage-b",
      title: "导览：第二季开场——从「建立坐标系」到「打地基」",
      subtitle: "阶段 B 路线图 · 第一季悬念清单 · 一处新勘误",
      emoji: "🏗️",
      duration: "25 分钟",
      tags: ["导览", "阶段 B", "悬念清算"],
      description: "阶段 A 与 B 的分工差别、第一季五条悬念分别在哪一节还清（哪几条还不了也明说）、五个模块 19 篇论文的排列逻辑，以及又揪出的一处路线图归类错误。"
    },
    {
      id: "0018",
      path: "lessons/0018-BA上-现代综述2000-BA究竟在解什么.html",
      paper: "stage-b",
      title: "BA（上）：它究竟在解什么问题",
      subtitle: "2000 年综述 · 四个纠偏 · 参数化",
      emoji: "🧱",
      duration: "45 分钟",
      tags: ["BA", "最小二乘", "参数化"],
      description: "BA 为什么要「联合最优」、原文对四个想当然的纠偏、残差与代价函数怎么建，以及「远处点不能用欧氏坐标」「旋转不能用三参数欧拉角」这类细节为什么能决定成败。"
    },
    {
      id: "0019",
      path: "lessons/0019-BA下-Schur消元与鲁棒核.html",
      paper: "stage-b",
      title: "BA（下）：Schur 消元与鲁棒核 ★",
      subtitle: "两层稀疏结构 · 变量排序 · gauge freedom",
      emoji: "🔩",
      duration: "55 分钟",
      tags: ["Schur 消元", "稀疏性", "gauge", "鲁棒估计"],
      description: "第一季悬念①在这里还清：两层稀疏结构、Schur 补一步步推导、变量排序与填充、gauge freedom 与「协方差没有绝对含义」，以及原文那句「先剔除外点再优化只是迂回地模拟鲁棒代价函数」。"
    },
    {
      id: "0020",
      path: "lessons/0020-GraphSLAM教程2010-把SLAM变成一张图.html",
      paper: "stage-b",
      title: "GraphSLAM 教程（2010）：把 SLAM 变成一张图",
      subtitle: "滤波 vs 平滑 · 节点与边 · 流形增量",
      emoji: "🕸️",
      duration: "50 分钟",
      tags: ["图优化", "因子图", "流形"],
      description: "换机器人视角把同一套数学再推一遍：从 full SLAM 后验到 HΔx=−b、稀疏性从哪来、3D 为什么必须用流形增量，以及 Intel 数据集 1802 节点 / 3546 边 / 100 ms 这组数字。"
    },
    {
      id: "0021",
      path: "lessons/0021-g2o优化库2011-因子图求解器的工程实现.html",
      paper: "stage-b",
      title: "g2o（2011）：因子图求解器的工程实现",
      subtitle: "顶点 · 边 · 求解器 · 线性求解器",
      emoji: "⚙️",
      duration: "45 分钟",
      tags: ["g2o", "后端工程", "求解器"],
      description: "数学怎么变成一个库：四层抽象、「H 的块结构就是图的邻接矩阵」、符号分解复用，以及 CSparse / CHOLMOD / PCG 三种求解器的实测对比——结论是「没有万能的求解器」。"
    },
    {
      id: "0022",
      path: "lessons/0022-点云BA2023-高效与一致.html",
      paper: "stage-b",
      title: "点云上的高效一致 BA（2023）",
      subtitle: "点簇 · 特征消元 · NEES 一致性",
      emoji: "📡",
      duration: "45 分钟",
      tags: ["点云 BA", "一致性", "点簇"],
      description: "把 BA 搬到激光点云上：为什么「扫不到同一个点」会让视觉 BA 失效、点簇怎么让复杂度与点数无关，以及用 NEES 曲线证明「一致」——和它的噪声上界。"
    },
    {
      id: "0023",
      path: "lessons/0023-PCL点云库2011-点云处理的瑞士军刀.html",
      paper: "stage-b",
      title: "PCL（2011）：点云处理的瑞士军刀",
      subtitle: "九块库 · PPG · ROS nodelet",
      emoji: "🧰",
      duration: "30 分钟",
      tags: ["PCL", "工程底座", "点云"],
      description: "全季唯一的软件系统论文：九块库与代表算法的归属、统一接口与感知处理图，以及必须诚实交代的两件事——原文既没有 limitations 小节，也没有任何性能数字。"
    },
    {
      id: "0024",
      path: "lessons/0024-VO教程上-相机模型与对极几何.html",
      paper: "stage-b",
      title: "VO 教程（上）：相机模型与对极几何",
      subtitle: "内参与归一化像面 · 对极约束 · 尺度",
      emoji: "📷",
      duration: "40 分钟",
      tags: ["VO", "对极几何", "相机模型"],
      description: "后端要解的方程是谁给的：VO 的定义与前提、针孔与球面模型、相对位姿连乘、为什么单目看不见尺度，以及「知道两个相机怎么摆就能把搜索降到一条线上」。"
    },
    {
      id: "0025",
      path: "lessons/0025-VO教程下-单目双目与三条运动估计管线.html",
      paper: "stage-b",
      title: "VO 教程（下）：单目、双目与三条估计管线",
      subtitle: "2D-2D / 3D-3D / 3D-2D · 精度历史",
      emoji: "🎥",
      duration: "40 分钟",
      tags: ["VO", "PnP", "单双目"],
      description: "三条运动估计管线各自的代价与精度、单目与双目优缺点的原文对照，以及「为什么在图像平面上最小化误差比在三维空间里更准」——这句话直接接上 BA 的残差定义。"
    },
    {
      id: "0026",
      path: "lessons/0026-SGM半全局匹配2008-双目深度怎么算.html",
      paper: "stage-b",
      title: "SGM（2008）：双目深度怎么算",
      subtitle: "互信息代价 · 路径聚合 · 层次化",
      emoji: "👁️",
      duration: "45 分钟",
      tags: ["SGM", "双目匹配", "互信息"],
      description: "稠密、不提取特征的一条路：为什么两台相机曝光不同灰度差法就崩、半全局的能量函数怎么设计、16 条路径怎么聚合，以及「为什么它更接近扫描线优化而非常规 DP」。"
    },
    {
      id: "0027",
      path: "lessons/0027-COLMAP-从无序图像到三维重建.html",
      paper: "stage-b",
      title: "COLMAP（2016）：从无序图像到三维重建",
      subtitle: "增量式 SfM 流水线 · 五条贡献",
      emoji: "🏛️",
      duration: "45 分钟",
      tags: ["SfM", "COLMAP", "增量重建"],
      description: "先纠正路线图的一处归类错误——它是方法论文不是综述；再讲「图像靠结构注册、结构靠图像三角化」这个鸡生蛋关系、五条具体改进，以及「它完全不处理在线、因果、回环」这一诚实结论。"
    },
    {
      id: "0028",
      path: "lessons/0028-MSCKF2007-不把路标放进状态向量.html",
      paper: "stage-b",
      title: "MSCKF（2007）：不把路标放进状态向量",
      subtitle: "零空间投影 · 两次投影 · 延迟线性化",
      emoji: "🎯",
      duration: "45 分钟",
      tags: ["MSCKF", "滤波", "零空间"],
      description: "滤波阵营的聪明招：状态里只放位姿历史副本、用左零空间投影把特征坐标抹掉、两次投影把残差降到 2M−3 维——与第 0019 课的 Schur 消元是同一招的两个版本。"
    },
    {
      id: "0029",
      path: "lessons/0029-IMU预积分上-为什么需要预积分.html",
      paper: "stage-b",
      title: "IMU 预积分（上）：为什么需要预积分",
      subtitle: "重复积分问题 · 与状态无关 · 可观性",
      emoji: "🧭",
      duration: "45 分钟",
      tags: ["预积分", "VIO", "流形"],
      description: "第一季悬念②在这里回答：为什么「每次优化迭代都要重新积分」是致命的、式(32) 与式(33) 并排对比看「与状态无关」到底指什么，以及 VIO 的 4 个不可观方向。"
    },
    {
      id: "0030",
      path: "lessons/0030-IMU预积分下-流形上的推导与雅可比.html",
      paper: "stage-b",
      title: "IMU 预积分（下）：流形上的推导与雅可比",
      subtitle: "噪声分离 · 偏置修正 · IMU 因子",
      emoji: "🧮",
      duration: "55 分钟",
      tags: ["预积分", "IMU 因子", "协方差"],
      description: "全季公式密度最高的一节，按四件套组织：噪声怎么分离、偏置为什么不用重积分、协方差怎么递推、9 维 IMU 因子怎么进图——并指出 structureless 视觉因子的 2n−3 与 MSCKF 是同一个数字。"
    },
    {
      id: "0031",
      path: "lessons/0031-连续时间vs离散时间2022-什么时候必须换建模.html",
      paper: "stage-b",
      title: "连续时间 vs 离散时间（2022）：什么时候必须换建模",
      subtitle: "B 样条 · 时偏联合优化 · 受控实验",
      emoji: "⏱️",
      duration: "40 分钟",
      tags: ["连续时间", "B 样条", "时间同步"],
      description: "一篇「对比研究」的读法：离散时间的三条失效机制、连续时间的三层好处、人为注入 0/10/20 ms 时延的受控实验，以及两条容易忽略的反面证据——同步场景下连续时间不划算、原文没做卷帘快门。"
    },
    {
      id: "0032",
      path: "lessons/0032-连续时间落地-弹性地图与固定延迟平滑.html",
      paper: "stage-b",
      title: "连续时间落地：弹性地图与固定延迟平滑",
      subtitle: "ElasticLiDAR++ · CLIO / CLIC · 边缘化",
      emoji: "🪢",
      duration: "50 分钟",
      tags: ["连续时间", "地图中心", "边缘化"],
      description: "两个真实系统：elasticity 到底指「地图可被拉拽」而非轨迹弹性、composition 少控制点反而更准，以及用 Schur 补做边缘化把 1601 s 压到 218 s——顺带回答「连续时间框架为什么不需要预积分」。"
    },
    {
      id: "0033",
      path: "lessons/0033-KinectFusion-稠密建图的开端.html",
      paper: "stage-b",
      title: "KinectFusion（2011）：稠密建图的开端",
      subtitle: "TSDF · 加权滑动平均 · raycasting 闭环",
      emoji: "🧊",
      duration: "45 分钟",
      tags: ["TSDF", "稠密建图", "RGB-D"],
      description: "把 TSDF 真正讲透：符号距离与截断、为什么「存权重」就把优化变成 O(1) 增量更新、raycasting 的两个用途构成 tracking–mapping 闭环，以及那个 ≤7 m³ 的内存墙。"
    },
    {
      id: "0034",
      path: "lessons/0034-VoxelHashing2013-体素地图的内存解法.html",
      paper: "stage-b",
      title: "Voxel Hashing（2013）：体素地图的内存解法",
      subtitle: "稀疏体素块 · 桶链冲突 · GPU 流式",
      emoji: "🗄️",
      duration: "45 分钟",
      tags: ["体素哈希", "GPU", "大场景"],
      description: "对上一节内存墙的工程反击：同一套 TSDF 数学换个容器，<300 MB 对 >5 GB、8 字节体素与 12 字节表项、GPU 上的四个并发难题，以及 46 fps 换来的 20–30 米重建尺度。"
    },
    {
      id: "0035",
      path: "lessons/0035-Voxblox2017-从TSDF到ESDF.html",
      paper: "stage-b",
      title: "Voxblox（2017）：从 TSDF 到 ESDF",
      subtitle: "增量波前传播 · 固定带 · MAV 机载",
      emoji: "🚁",
      duration: "45 分钟",
      tags: ["ESDF", "规划", "Voxblox"],
      description: "目标反转：前两节服务「看」，这一节服务「动」。投影距离与欧氏距离的分别、固定带加 raise/lower 双队列的增量算法、一个数量级的加速，以及机载 4 Hz / 250 ms 预算的实飞结果。"
    },
    {
      id: "0036",
      path: "lessons/0036-NeRF上-隐式表示与体渲染.html",
      paper: "stage-b",
      title: "NeRF（上）：隐式表示与体渲染",
      subtitle: "5D 辐射场 · 体渲染 · 可微性",
      emoji: "🌀",
      duration: "45 分钟",
      tags: ["NeRF", "隐式表示", "体渲染"],
      description: "把「隐式表示」变成三个能画出来的东西：一条光线、一个 MLP、一个可微的合成公式。体密度与透射率用生活类比讲透，并给出 5 MB 权重对 15 GB 体素网格的存储对比。"
    },
    {
      id: "0037",
      path: "lessons/0037-NeRF下-位置编码与分层采样.html",
      paper: "stage-b",
      title: "NeRF（下）：位置编码、分层采样与代价",
      subtitle: "谱偏置 · 两级采样 · 1–2 天",
      emoji: "🔬",
      duration: "45 分钟",
      tags: ["NeRF", "位置编码", "局限"],
      description: "上一节结尾原文自己说了「这些还不够」：位置编码为什么必需（带消融数字）、两级采样怎么工作、训练细节抄准，然后把代价摊开——逐场景 1–2 天，以及原文自陈的两条开放问题。"
    },
    {
      id: "0038",
      path: "lessons/0038-InstantNGP2022-哈希编码把NeRF压到秒级.html",
      paper: "stage-b",
      title: "Instant NGP（2022）：哈希编码把 NeRF 压到秒级",
      subtitle: "多分辨率哈希 · 冲突即平均 · 秒级训练",
      emoji: "⚡",
      duration: "50 分钟",
      tags: ["Instant NGP", "哈希编码", "实时"],
      description: "换编码而不换渲染：五步哈希流程、为什么「不处理冲突、让梯度自己平均」反而成立、8 倍加速的干净对照与秒级 PSNR 曲线，以及反面——颗粒感伪影与 L2 缓存导致的性能断崖。"
    },
    {
      id: "0039",
      path: "lessons/0039-Plenoxels2022-连神经网络都不要.html",
      paper: "stage-b",
      title: "Plenoxels（2022）：连神经网络都不要",
      subtitle: "稀疏体素加球谐 · 梯度稀疏 · 全季收束",
      emoji: "🎆",
      duration: "50 分钟",
      tags: ["Plenoxels", "显式表示", "收官"],
      description: "本季收官：把 MLP 也拿掉，用稀疏体素加球谐直接优化，11 分钟对齐 NeRF 一天的成绩；并回答那个更大的问题——NeRF 里真正不可省的到底是什么。含「隐式→混合→显式」全季对照表。"
    },
    {
      id: "0040",
      path: "lessons/0040-导览-第三季开场-视觉主干.html",
      paper: "stage-c",
      title: "导览：第三季开场",
      subtitle: "视觉主干的地图 · 两笔债 · 一条暗线",
      emoji: "🧭",
      duration: "45 分钟",
      tags: ["导览", "阅读路线", "尺度暗线"],
      description: "第三季总导览：七个模块的地图、第二季欠下的两笔债（提取 vs 不提取特征、IMU 初始化）怎么还、以及一条贯穿全季的暗线——单目尺度是怎么被六个系统用六种方式解决的。含三种读法与按问题查课的对照表。"
    },
    {
      id: "0041",
      path: "lessons/0041-MonoSLAM2007-EKF时代的单目SLAM.html",
      paper: "stage-c",
      title: "MonoSLAM（2007）：把路标放进状态向量",
      subtitle: "EKF 时代唯一的选择 · 协方差的 O(N²)",
      emoji: "🕰️",
      duration: "50 分钟",
      tags: ["MonoSLAM", "EKF", "协方差椭球"],
      description: "视觉 SLAM 的起点：相机与所有路标塞进一个状态向量、所有互相关塞进一个协方差矩阵。讲清为什么这在当年是唯一选择，以及为什么它必然被淘汰。含逆深度思想的伏笔。"
    },
    {
      id: "0042",
      path: "lessons/0042-PTAM2007-跟踪与建图分家.html",
      paper: "stage-c",
      title: "PTAM（2007）：跟踪与建图分家",
      subtitle: "架构革命 · BA 走进实时",
      emoji: "🧵",
      duration: "50 分钟",
      tags: ["PTAM", "双线程", "BA"],
      description: "性能会被超越，架构想法不会。讲清为什么把跟踪与建图拆成两个线程是个天才主意，以及 BA 怎么靠 Schur 消元与固定第一个关键帧从离线走进实时。"
    },
    {
      id: "0043",
      path: "lessons/0043-ORB-SLAM2015-一套特征服务所有任务.html",
      paper: "stage-c",
      title: "ORB-SLAM（2015）：一套特征服务所有任务",
      subtitle: "三线程 · 共视图 · BoW 回环 · sim(3)",
      emoji: "🔶",
      duration: "60 分钟",
      tags: ["ORB-SLAM", "共视图", "BoW", "sim3"],
      description: "前两季所有零件在这篇被装配成一台完整机器：三线程架构、一套 ORB 服务跟踪/建图/重定位/回环、共视图与本质图、DBoW2 倒排索引、用 sim(3) 纠正单目尺度漂移。"
    },
    {
      id: "0044",
      path: "lessons/0044-ORB-SLAM2-2017-绝对尺度与工程标杆.html",
      paper: "stage-c",
      title: "ORB-SLAM2（2017）：绝对尺度与工程标杆",
      subtitle: "双目深度 · 近点与远点 · 至今工业首选",
      emoji: "👓",
      duration: "45 分钟",
      tags: ["ORB-SLAM2", "双目", "绝对尺度"],
      description: "加上双目与 RGB-D 之后，尺度问题被一条基线直接解决。讲清双目极线搜索、为什么只搜水平线、近点与远点为什么区别对待，以及 BA 目标函数在三种传感器下的写法差异。"
    },
    {
      id: "0045",
      path: "lessons/0045-ORB-SLAM3-2021-多地图与视觉惯性.html",
      paper: "stage-c",
      title: "ORB-SLAM3（2021）：多地图与视觉惯性",
      subtitle: "Atlas 多地图 · 地图合并 · 焊接窗口",
      emoji: "🗺️",
      duration: "45 分钟",
      tags: ["ORB-SLAM3", "Atlas", "地图合并", "VI"],
      description: "第一个同时做视觉、视觉惯性、多地图的系统。跟踪失败不再重置而是新建地图，识别到共视区域就把两块地图焊在一起。也是第二季悬念⑤（IMU 初始化）的第一站。"
    },
    {
      id: "0046",
      path: "lessons/0046-DTAM2011-稠密直接法的开端.html",
      paper: "stage-c",
      title: "DTAM（2011）：稠密直接法的开端",
      subtitle: "光度误差 · warp · 逆深度 · 代价体",
      emoji: "🌊",
      duration: "50 分钟",
      tags: ["DTAM", "光度误差", "逆深度"],
      description: "全季最重要的一对概念辨析（光度误差 vs 重投影误差）在这里讲透。讲清 warp 的几何含义、为什么逆深度能让极线采样变成线性的、以及稠密多基线代价体与第二季 SGM 的对照。"
    },
    {
      id: "0047",
      path: "lessons/0047-LSD-SLAM2014-半稠密与尺度漂移.html",
      paper: "stage-c",
      title: "LSD-SLAM（2014）：半稠密与尺度漂移",
      subtitle: "只挑梯度大的像素 · sim(3) 把尺度当变量",
      emoji: "🎯",
      duration: "50 分钟",
      tags: ["LSD-SLAM", "半稠密", "sim3", "尺度漂移"],
      description: "尺度暗线的第 1 站：第一次把「尺度不可观」从污染项变成待估变量。讲清半稠密为什么是稠密与稀疏之间的甜点、深度图的概率表示与空间正则、以及方差归一化的光度残差。"
    },
    {
      id: "0048",
      path: "lessons/0048-DSO2018-光度标定与稀疏直接法.html",
      paper: "stage-c",
      title: "DSO（2018）：光度标定与稀疏直接法",
      subtitle: "四象限坐标系 · 仿射亮度 · 均匀采样",
      emoji: "⚖️",
      duration: "60 分钟",
      tags: ["DSO", "光度标定", "四象限", "边缘化"],
      description: "直接法的巅峰，也是全季最值得细读的一节。讲清四象限坐标系（直接≠稠密）、光度标定的三件事、仿射亮度模型、均匀化采样四策略、滑窗边缘化与 First Estimate Jacobians，以及那段裁决「直接法 vs 间接法」的噪声实验。"
    },
    {
      id: "0049",
      path: "lessons/0049-VI-DSO2018-动态边缘化.html",
      paper: "stage-c",
      title: "VI-DSO（2018）：动态边缘化",
      subtitle: "三个先验的接力棒 · 即时开局",
      emoji: "🔁",
      duration: "50 分钟",
      tags: ["VI-DSO", "动态边缘化", "VIO"],
      description: "为什么普通边缘化会把旧尺度钉死、把系统拉偏？这一节用三个先验（visual / curr / half）的接力机制回答它，并把「直接法 + IMU」的初始化四步讲清。第二季 0032 边缘化的进阶版。"
    },
    {
      id: "0050",
      path: "lessons/0050-SVO2014-半直接与概率深度滤波.html",
      paper: "stage-c",
      title: "SVO（2014）：半直接与概率深度滤波",
      subtitle: "对应关系是直接对齐的副产品 · 55 fps 嵌入式",
      emoji: "⚡",
      duration: "45 分钟",
      tags: ["SVO", "半直接", "概率深度滤波"],
      description: "只在关键帧提特征、其余帧靠直接对齐，速度一下子推到几百帧。重点讲概率深度滤波（Gaussian + Uniform 混合）——这是全季最值得借用的零件之一。也是悬念④「提取 vs 不提取特征」的中间态答案。"
    },
    {
      id: "0051",
      path: "lessons/0051-DVO-SLAM2013-用现成深度做直接法.html",
      paper: "stage-c",
      title: "DVO-SLAM（2013）：用现成深度做直接法",
      subtitle: "两个残差加一个先验 · t 分布鲁棒核",
      emoji: "📐",
      duration: "45 分钟",
      tags: ["DVO-SLAM", "RGB-D", "深度残差"],
      description: "有了现成深度图，直接法省掉了估逆深度这一步。讲清为什么「深度残差」是必要的（它能在白墙上拉住光度残差）、鲁棒核为什么是 t 分布而不是 Huber、以及熵比这一个指标怎么同时干选帧与回环两件事。"
    },
    {
      id: "0052",
      path: "lessons/0052-RGBD-SLAM-v2-2014-稀疏特征加EMM回环.html",
      paper: "stage-c",
      title: "RGBD-SLAM-v2（2014）：稀疏特征 + EMM 回环",
      subtitle: "测地邻域采样 · OctoMap · 回环后的硬伤",
      emoji: "🧱",
      duration: "45 分钟",
      tags: ["RGBD-SLAM", "EMM", "测地邻域", "OctoMap"],
      description: "与上一节同团队、同数据集、两条完全不同的路线。讲清为什么回环候选要用「测地邻域」而不是随机采样，以及一个很硬的矛盾：完成大回环后，稠密体素地图只能整张重建。"
    },
    {
      id: "0053",
      path: "lessons/0053-RTAB-Map2019-记忆管理与长期建图.html",
      paper: "stage-c",
      title: "RTAB-Map（2019）：记忆管理与长期建图",
      subtitle: "STM / WM / LTM 三级记忆 · 长期在线运行",
      emoji: "🗄️",
      duration: "45 分钟",
      tags: ["RTAB-Map", "记忆管理", "长期建图"],
      description: "全季第二篇库论文。核心是三级记忆模型：只有被用到的东西才会被从「库房」调回「桌上」，于是能长期在线运行。记忆管理开与关的对比显示计算量与内存砍掉一半以上而精度几乎不变。"
    },
    {
      id: "0054",
      path: "lessons/0054-CNN-SLAM2017-向学习借深度.html",
      paper: "stage-c",
      title: "CNN-SLAM（2017）：向学习借深度",
      subtitle: "把学到的深度当先验 · 尺度暗线第 4 站",
      emoji: "🎓",
      duration: "45 分钟",
      tags: ["CNN-SLAM", "学习式稠密", "绝对尺度"],
      description: "单目 SLAM 的第一条学习式路线：用网络预测的稠密深度给直接法补上与尺度有关的短板。手算演示逆方差加权融合（谁的不确定度小就听谁的），并讲清一个意外红利——纯旋转这个几何法死区，单帧深度预测反而能扛。含对原文不存在数字的勘误说明。"
    },
    {
      id: "0055",
      path: "lessons/0055-CodeSLAM2018-把稠密场景压成一个码.html",
      paper: "stage-c",
      title: "CodeSLAM（2018）：把稠密场景压成一个码",
      subtitle: "可优化的低维几何表示 · 让深度第一次能被求导",
      emoji: "🎛️",
      duration: "50 分钟",
      tags: ["CodeSLAM", "紧凑表示", "可优化"],
      description: "上一节只能「融合」深度，这一节让它能被「优化」：把整张深度图表示成一个 128 维向量与图像的可微函数，于是深度和位姿可以一起进 BA。核心取舍是为了可优化的雅可比而牺牲表达力，并给出 proximity 参数化的完整曲线与可验算数据。"
    },
    {
      id: "0056",
      path: "lessons/0056-DeepFactors2020-学习先验进因子图.html",
      paper: "stage-c",
      title: "DeepFactors（2020）：学习先验进因子图",
      subtitle: "三种因子各管一件事 · 稠密 SLAM 拿到标准后端",
      emoji: "🧩",
      duration: "50 分钟",
      tags: ["DeepFactors", "因子图", "实时"],
      description: "学习式稠密模块的收尾：把 CodeSLAM 的三条短板（不实时、不泛化、不是完整 SLAM）一次补齐，做法是把学习到的几何先验装进标准因子图。详解光度／重投影／稀疏几何三种因子为何用三种不同的鲁棒核，以及 340 ms 里只有 16 ms 是网络这个反直觉事实。"
    },
    {
      id: "0057",
      path: "lessons/0057-VINS-Mono上-四模块管线与联合初始化.html",
      paper: "stage-c",
      title: "VINS-Mono（上）：四模块管线与联合初始化",
      subtitle: "★ 还清第二季悬念⑤ · 尺度与重力怎么一起解出来",
      emoji: "📐",
      duration: "55 分钟",
      tags: ["VINS-Mono", "VIO", "初始化"],
      description: "教科书级系统论文的上半。先把四模块三线程的骨架立起来，再逐步复述「松对齐」初始化的四步：纯视觉 SfM 给形状 → 陀螺与视觉的旋转差标定偏置 → 线性最小二乘一次解出速度／重力／尺度 → 在切平面上迭代细化重力。这是第二季预积分那篇论文自己承认缺失的那一章。"
    },
    {
      id: "0058",
      path: "lessons/0058-VINS-Mono下-紧耦合后端与4自由度回环.html",
      paper: "stage-c",
      title: "VINS-Mono（下）：紧耦合后端与 4 自由度回环",
      subtitle: "单位球面残差 · 选择性边缘化 · 为什么只优化 4 个自由度",
      emoji: "🎯",
      duration: "55 分钟",
      tags: ["VINS-Mono", "位姿图", "回环"],
      description: "下半讲系统怎么持续跑：残差为何定义在单位球面上、选择性边缘化如何「好留的留、能传的传」、三档输出如何按频率分层。重点是那个漂亮的巧劲——因为重力让 roll/pitch 成为绝对量，回环只需优化 4 个自由度。含 EuRoC 表中三行「不符合叙事」的诚实读数。"
    },
    {
      id: "0059",
      path: "lessons/0059-ROVIO2015-滤波派的直接法.html",
      paper: "stage-c",
      title: "ROVIO（2015）：滤波派的直接法",
      subtitle: "robot-centric 表示 · 多层 patch 的 QR 降维",
      emoji: "🛩️",
      duration: "50 分钟",
      tags: ["ROVIO", "EKF", "直接法"],
      description: "换阵营的一节：同样融合 IMU，但走滤波器那条路，还把直接法搬进了 EKF。两个招牌设计——把坐标原点绑在自己身上（robocentric），以及用 QR 分解把 256 个强度误差压成一个 2×2 约束。含原文主动报告的那个「越错越远」的发散模式。"
    },
    {
      id: "0060",
      path: "lessons/0060-Basalt2020-用非线性因子压缩历史.html",
      paper: "stage-c",
      title: "Basalt（2020）：用非线性因子压缩历史",
      subtitle: "两层架构 · 把一段历史压成一个可复用的因子",
      emoji: "🗜️",
      duration: "45 分钟",
      tags: ["Basalt", "NFR", "全局 BA"],
      description: "讲一个新思路：与其让历史信息在边缘化里变成一根死掉的先验弹簧，不如把一段历史压缩成一个可复用的非线性因子，交给上层的全局 BA。含一处纠错——NFR 常被误传为 Delaunay 三角化选关键帧，实际并不是。"
    },
    {
      id: "0061",
      path: "lessons/0061-OpenVINS2020-一致性与FEJ.html",
      paper: "stage-c",
      title: "OpenVINS（2020）：一致性与 FEJ",
      subtitle: "★ 滤波派最怕的病 · 过度自信怎么治",
      emoji: "🩺",
      duration: "45 分钟",
      tags: ["OpenVINS", "FEJ", "一致性"],
      description: "滤波派真正的命门不是精度，是一致性。这一节把「过度自信」这个抽象概念画出来：协方差椭圆越画越小、真实误差却不变。核心招式 FEJ 就是把线性化点钉在第一次估计上。诚实标注：本文只讲 FEJ，未覆盖 OC-EKF。"
    },
    {
      id: "0062",
      path: "lessons/0062-Kimera2020-度量语义地图.html",
      paper: "stage-c",
      title: "Kimera（2020）：度量-语义地图",
      subtitle: "四模块库 · PCM 最大一致集剔假回环",
      emoji: "🏷️",
      duration: "45 分钟",
      tags: ["Kimera", "语义建图", "鲁棒位姿图"],
      description: "本季第三篇库论文。把「地图」从几何推进到「几何 + 每个面片带一个标签」。四个模块分工清楚，其中 RPGO 用最大一致集剔除错误回环，正是第一季 0014 课那条「假阳性铁律」的系统级实现。"
    },
    {
      id: "0063",
      path: "lessons/0063-DM-VIO2022-延迟边缘化.html",
      paper: "stage-c",
      title: "DM-VIO（2022）：延迟边缘化",
      subtitle: "★ 与 VI-DSO 的动态边缘化是两回事 · 尺度可以事后改",
      emoji: "⏳",
      duration: "50 分钟",
      tags: ["DM-VIO", "边缘化", "尺度"],
      description: "把本季 0049 VI-DSO 的「动态边缘化」往前推了一大步，但两者不是同一个东西：动态边缘化只处理一维尺度，而延迟边缘化是再维护一张晚 100 帧删帧的因子图，于是能「撤销部分边缘化」、能把 IMU 信息回溯注入。开销仅 0.8%。"
    },
    {
      id: "0064",
      path: "lessons/0064-纯惯性初始化2020-冷启动怎么解.html",
      paper: "stage-c",
      title: "纯惯性初始化（2020）：冷启动怎么解",
      subtitle: "★ 悬念⑤的最后一块拼图 · 只有 IMU 也能定出重力与尺度",
      emoji: "🚀",
      duration: "45 分钟",
      tags: ["初始化", "ORB-SLAM3", "冷启动"],
      description: "第二季悬念⑤的收官：VINS-Mono 是「视觉先动起来再对齐」，这一篇更极端——最开始只有 IMU 在动。用一段加速度计读数做优化，把重力方向与尺度同时解出来。这是很反直觉的一步，课程用三格具象图把它拆开讲。"
    },
    {
      id: "0065",
      path: "lessons/0065-GVINS2022-把GNSS放进因子图.html",
      paper: "stage-c",
      title: "GVINS（2022）：把 GNSS 放进因子图",
      subtitle: "室外大尺度的补位 · 多路径与 NLOS",
      emoji: "🛰️",
      duration: "45 分钟",
      tags: ["GVINS", "GNSS", "紧耦合"],
      description: "VIO 在室外大尺度必然漂，GNSS 在城市峡谷又被遮挡与多路径污染。这一节讲怎么把 GNSS 的原始观测当成因子塞进同一张图，并画清楚多路径到底是怎么让定位整体偏掉的。含三处常见误传的纠正。"
    },
    {
      id: "0066",
      path: "lessons/0066-P3-VINS2022-载波相位与厘米级定位.html",
      paper: "stage-c",
      title: "P³-VINS（2022）：载波相位与厘米级定位",
      subtitle: "相位模糊度 · 为什么细尺子能读到更小",
      emoji: "📡",
      duration: "45 分钟",
      tags: ["P³-VINS", "PPP", "载波相位"],
      description: "比 GVINS 更进一步：用载波相位做 PPP。核心概念「相位模糊度」用一个很具象的类比讲清——接收机只知道自己在波上的小数部分，不知道是第几个整周。诚实标注原文并未给出任何具体波长数值。"
    },
    {
      id: "0067",
      path: "lessons/0067-自动驾驶SLAM综述2017-室外大尺度与高精地图.html",
      paper: "stage-c",
      title: "自动驾驶 SLAM 综述（2017）：室外大尺度与高精地图",
      subtitle: "★ 含一处路线图错配的公开勘误",
      emoji: "🛣️",
      duration: "45 分钟",
      tags: ["自动驾驶", "高精地图", "综述"],
      description: "开篇先做勘误：这一篇被路线图标成「ORB-SLAM 早期会议版」，实际是一篇自动驾驶场景的 SLAM 趋势综述，而语料库里根本没有 ORB-SLAM 会议版。本篇用「室内机器人 vs 高速公路自驾」的对比图帮读者建立室外场景的直觉。"
    },
    {
      id: "0068",
      path: "lessons/0068-动态环境视觉惯性2022-五篇横向对比.html",
      paper: "stage-c",
      title: "动态环境视觉惯性（2022–2024）：五篇横向对比",
      subtitle: "判定 / 处理 / 代价 三层框架 · 附五篇对比表",
      emoji: "🏃",
      duration: "55 分钟",
      tags: ["动态环境", "组课", "横向对比"],
      description: "组课形式：不讲单篇，而是把五篇解决同一问题的方法横向摆开。先立起「判定 → 处理 → 代价」三层框架，再逐篇给一句话招式。诚实包含一条判断——哪几篇真新颖、哪几篇偏增量（并注明这是我们的判断）。"
    },
    {
      id: "0069",
      path: "lessons/0069-线特征与光照鲁棒上-点线融合的三条路.html",
      paper: "stage-c",
      title: "线特征与光照鲁棒（上）：点线融合的三条路",
      subtitle: "白墙没有角点，但有边 · 线特征的三个麻烦",
      emoji: "📏",
      duration: "45 分钟",
      tags: ["线特征", "光照鲁棒", "组课"],
      description: "先回答一个具体问题：为什么弱纹理场景下线比点稳——白墙检不出角点，但门框、踢脚线、窗框都在。再讲线特征的三个经典麻烦（端点不确定、匹配歧义、参数化），以及三篇各自的绕法。"
    },
    {
      id: "0070",
      path: "lessons/0070-线特征与光照鲁棒下-消失点与工程鲁棒.html",
      paper: "stage-c",
      title: "线特征与光照鲁棒（下）：消失点与工程鲁棒",
      subtitle: "平行的长直线会汇聚到同一个点 · 两条提升路径",
      emoji: "🔭",
      duration: "45 分钟",
      tags: ["消失点", "UV-SLAM", "工程鲁棒"],
      description: "「消失点」这个概念用一条走廊画开：所有互相平行的长直线在图像里都会汇聚到同一个点。收束时点出提升鲁棒性的两条路——加新特征类型（线、消失点）与改工程实现（重定位、初始化），并诚实说明 RD-VIO 其实不含线特征。"
    },
    {
      id: "0071",
      path: "lessons/0071-神经隐式建图2024-两种组合方式.html",
      paper: "stage-c",
      title: "神经隐式建图（2024）：两种组合方式",
      subtitle: "★ 通向阶段 F 的桥 · 位姿从哪来",
      emoji: "🧠",
      duration: "50 分钟",
      tags: ["NICER-SLAM", "DN-SLAM", "神经隐式"],
      description: "把第二季离线学的 NeRF 搬进在线 SLAM。两篇的共性是「传统前端 + 神经隐式建图」，差别在分工：一个把位姿和场景放在同一个网络里联合反传，另一个让 ORB 前端管位姿、NeRF 只管静态地图。收尾留钩子指向阶段 F。"
    },
    {
      id: "0072",
      path: "lessons/0072-三条务实路线-激光辅助与受限平台.html",
      paper: "stage-c",
      title: "三条务实路线：激光辅助与受限平台",
      subtitle: "第三季收尾 · 不够漂亮但很管用",
      emoji: "🧰",
      duration: "50 分钟",
      tags: ["工程务实", "受限平台", "收尾"],
      description: "整季的收尾，讲的都是「不够漂亮但很管用」的路线：多一种传感器（激光辅助单目）、换一种监督方式（无监督动态掩码）、把算法砍到能跑（受限平台 RGB-D 惯性）。这三条路共同回答的是「在真实约束下怎么把 SLAM 跑起来」。"
    },
    {
      id: "0073",
      path: "lessons/0073-导览-第四季开场-激光主干.html",
      paper: "stage-d",
      title: "第四季开场：激光 SLAM 是怎么学会「不看特征」的",
      subtitle: "第四季导览 · 还债清单与八个模块",
      emoji: "📡",
      duration: "40 分钟",
      tags: ["导览", "激光主干", "还债清单"],
      description: "第四季开场。这一季要还一笔挂了两季的债（ikd-Tree），也重演一场在视觉侧已经看过的辩论（提特征 vs 不提特征）。含「提特征/不提特征 × 滤波/优化」四象限定位图与八个模块的地图。"
    },
    {
      id: "0074",
      path: "lessons/0074-LOAM2017-激光SLAM的分水岭.html",
      paper: "stage-d",
      title: "LOAM（2017）：激光 SLAM 的分水岭",
      subtitle: "两步优化 · 特征提取 · 运动畸变补偿",
      emoji: "📐",
      duration: "60 分钟",
      tags: ["特征法", "两步优化", "去畸变"],
      description: "激光 SLAM 的骨架就是这一篇搭的：把问题拆成「高频低精度的里程计」与「低频高精度的建图」两层，用角点与平面点做 scan-to-map 配准。含「运动畸变为什么会把直线拉成弧」与「两步分工」两张具象自绘图，并顺手埋下 ikd-Tree 的伏笔。"
    },
    {
      id: "0075",
      path: "lessons/0075-LeGO-LOAM2018-为嵌入式瘦身.html",
      paper: "stage-d",
      title: "LeGO-LOAM（2018）：为嵌入式瘦身",
      subtitle: "点云分割 · 地面约束三自由度 · 两步 LM",
      emoji: "🌿",
      duration: "50 分钟",
      tags: ["轻量化", "点云分割", "退化"],
      description: "同一套骨架，砍掉一半算力，还治好了草地噪声。核心是一个反直觉的结论：无限大的平地只能约束六个自由度里的三个。含「地面只管三个自由度」的三视角具象图。"
    },
    {
      id: "0076",
      path: "lessons/0076-两条变体路线-NDT与EKF.html",
      paper: "stage-d",
      title: "两条变体路线：用分布替代特征，用多传感器兜住退化",
      subtitle: "组课 · NDT-LOAM ＋ EKF-LOAM",
      emoji: "🔧",
      duration: "55 分钟",
      tags: ["组课", "NDT", "退化", "多传感器"],
      description: "两篇同年的论文，分别代表「在传感器内部找答案」与「在传感器外部找答案」。NDT 用统计分布替代特征；EKF-LOAM 用轮速与 IMU 兜住隧道里的系统性低估（200 米隧道：误差从 56% 降到 0.18%）。"
    },
    {
      id: "0077",
      path: "lessons/0077-紧耦合3D激光惯性2019-LIO-SAM的前身.html",
      paper: "stage-d",
      title: "紧耦合 3D 激光惯性（2019）：把 IMU 请进优化",
      subtitle: "IMU 的三种用法 · 固定滞后平滑",
      emoji: "🧭",
      duration: "55 分钟",
      tags: ["紧耦合", "IMU", "滞后平滑"],
      description: "IMU 有三种用法：去畸变、给初值、作为约束。前两件是松耦合，把第三件也做了才是紧耦合。含「松耦合 vs 紧耦合」与「先验弹簧」两张具象图。"
    },
    {
      id: "0078",
      path: "lessons/0078-LIO-SAM2020-四类因子的因子图.html",
      paper: "stage-d",
      title: "LIO-SAM（2020）：四类因子装进一张图",
      subtitle: "IMU / 激光 / GPS / 回环 · 因子图 iSAM2",
      emoji: "🧩",
      duration: "65 分钟",
      tags: ["因子图", "紧耦合", "回环"],
      description: "本模块主干。把激光、IMU、GPS、回环四种测量统一塞进同一张因子图一起解，这也解释了为什么它比滤波更适合做全局一致。含「环形土路上的四色因子图」与「滑动窗」两张具象图。"
    },
    {
      id: "0079",
      path: "lessons/0079-BALM2021-把BA搬到激光上.html",
      paper: "stage-d",
      title: "BALM（2021）：把 BA 搬到激光上",
      subtitle: "特征值判平面/边缘 · 参数闭式消元",
      emoji: "🧱",
      duration: "55 分钟",
      tags: ["激光BA", "特征消元", "稀疏结构"],
      description: "视觉 BA 那套「一起优化特征与位姿」为什么也能用在激光上？因为平面与边缘的参数可以被解析地消掉。含「用三个主方向箭头判断这里像面还是像线」的具象图。注意与第二季 0022 课的 BALM2.0 区分。"
    },
    {
      id: "0080",
      path: "lessons/0080-MULLS2021-三种度量一起算.html",
      paper: "stage-d",
      title: "MULLS（2021）：三种度量一起算",
      subtitle: "点-点 / 点-面 / 点-线 · 粗细层级配准",
      emoji: "📐",
      duration: "50 分钟",
      tags: ["多度量", "特征分类", "回环"],
      description: "同一帧点云里有地面、有墙、还有一根电线杆——能不能让每种东西用最合身的那种「距离」去配准？含「街角三度量」具象图。"
    },
    {
      id: "0081",
      path: "lessons/0081-DLO2022-速度优先的直接法.html",
      paper: "stage-d",
      title: "DLO（2022）：速度优先的直接法",
      subtitle: "稠密点云 ＋ 关键帧 · 效率对照",
      emoji: "⚡",
      duration: "50 分钟",
      tags: ["直接法", "效率", "IMU松耦合"],
      description: "不挑特征，但要挑「代表」。这一篇在本模块里的角色是「效率对照」——它的 IMU 是松耦合的，只提供旋转先验。含「关键帧与稀疏化」与「子地图随车移动」两张具象图。"
    },
    {
      id: "0082",
      path: "lessons/0082-SuMa-2019-面片地图与语义.html",
      paper: "stage-d",
      title: "SuMa++（2019）：面片地图与语义",
      subtitle: "面片地图 · 语义动态剔除",
      emoji: "🧩",
      duration: "55 分钟",
      tags: ["地图表示", "语义", "动态环境"],
      description: "地图不再是一堆点，而是一片片自带朝向、还贴着标签的「瓷砖」——它们能告诉我们：哪些东西不该信。含「面片地图」与「语义剔动态」两张具象图。论文真名是 SuMa++（文件名漏了 ++）。"
    },
    {
      id: "0083",
      path: "lessons/0083-FAST-LIO2021-迭代卡尔曼与新增益公式.html",
      paper: "stage-d",
      title: "FAST-LIO（2021）：迭代卡尔曼与一个新公式",
      subtitle: "新增益公式 · 求逆矩阵从上千阶到 18 阶",
      emoji: "🧱",
      duration: "55 分钟",
      tags: ["迭代EKF", "新增益公式", "滤波派"],
      description: "一行公式，把「必须当场求逆的那个矩阵」从上千阶压到 18 阶——这就是它能把 1200 个以上特征点一次融进滤波器的全部原因。含「求逆矩阵降阶」与「迭代 vs 普通卡尔曼」两张具象图。"
    },
    {
      id: "0084",
      path: "lessons/0084-FAST-LIO2-2022-ikd-Tree与不再提特征.html",
      paper: "stage-d",
      title: "FAST-LIO2（2022）：ikd-Tree 与「不再提特征」",
      subtitle: "还债课 · 直接配准 ＋ 增量式 k-d 树",
      emoji: "🪓",
      duration: "70 分钟",
      tags: ["ikd-Tree", "直接配准", "还债课"],
      description: "挂了两季的 ikd-Tree 之债，本节连本带利还清。两条贡献（直接配准、ikd-Tree）其实是一件事的两面：不挑点了，点就变多，原来那棵每次重建的树就撑不住了。含「ikd-Tree 三板灵魂图」（静态树 / 增量插入 / 双线程重建）。"
    },
    {
      id: "0085",
      path: "lessons/0085-Faster-LIO2022-用体素哈希取代树.html",
      paper: "stage-d",
      title: "Faster-LIO（2022）：用体素哈希取代树",
      subtitle: "iVox · 查询 O(1) · 天然并行",
      emoji: "🚀",
      duration: "50 分钟",
      tags: ["iVox", "体素哈希", "并行"],
      description: "树是「问路」，哈希是「直接算门牌号」。含「树 vs 体素哈希」与「并行」两张具象图。这是同一个招式在三个战场的第三次出现（Voxel Hashing / Instant NGP / iVox）。"
    },
    {
      id: "0086",
      path: "lessons/0086-Point-LIO2023-逐点处理与高带宽.html",
      paper: "stage-d",
      title: "Point-LIO（2023）：逐点处理与高带宽",
      subtitle: "每来一个点就更新一次 · 帧内畸变自然消失",
      emoji: "⚡",
      duration: "55 分钟",
      tags: ["逐点更新", "高带宽", "剧烈运动"],
      description: "如果连「帧」都不要了呢？每来一个点就更新一次状态，运动畸变从根上消失，带宽推到极高。含「帧级 vs 逐点」与「翻滚机动」两张具象图。"
    },
    {
      id: "0087",
      path: "lessons/0087-iG-LIO与LOG-LIO-2024-分布配准与实时法向.html",
      paper: "stage-d",
      title: "iG-LIO 与 LOG-LIO（2024）：分布配准与实时法向",
      subtitle: "组课 · GICP 分布到分布 · 实时法向估计",
      emoji: "🧭",
      duration: "55 分钟",
      tags: ["组课", "GICP", "法向估计"],
      description: "两篇同年的论文在问同一件事：除了「点到面」，还有没有更稳的度量方式？法向又该怎么实时算出来？含「点到面 vs 分布到分布」与「墙角三个法向」两张具象图。"
    },
    {
      id: "0088",
      path: "lessons/0088-KISS-ICP2023-反工程化的极简之作.html",
      paper: "stage-d",
      title: "KISS-ICP（2023）：反工程化的极简之作",
      subtitle: "点对点 ICP 的辩护 · 免调参 · 不用 IMU",
      emoji: "🪶",
      duration: "50 分钟",
      tags: ["极简", "免调参", "跨雷达"],
      description: "在一整季「越来越复杂」之后，有人把系统砍到只剩七个参数——然后赢了。标题本身就是一句宣言：点对点 ICP 只要做对就够用。含「自适应阈值」具象图。"
    },
    {
      id: "0089",
      path: "lessons/0089-直接法LIO群-连续时间运动补偿.html",
      paper: "stage-d",
      title: "直接法 LIO 群：连续时间运动补偿",
      subtitle: "组课 · D-LIOM ＋ Direct LIO ＋ SLICT",
      emoji: "🔭",
      duration: "60 分钟",
      tags: ["组课", "连续时间", "面片地图"],
      description: "把「直接」这一支拉成一条光谱，看它在两端各长什么样。含「离散补偿 vs 连续时间补偿」与「多尺度面片」两张具象图。注意：这两篇用的不是 B 样条，而是解析多项式与滑窗线性插值。"
    },
    {
      id: "0090",
      path: "lessons/0090-强度与鲁棒-反射率这条额外通道.html",
      paper: "stage-d",
      title: "强度与鲁棒：反射率这条额外通道",
      subtitle: "组课 · 两种用强度的方式 ＋ 激光侧初始化",
      emoji: "🔦",
      duration: "60 分钟",
      tags: ["组课", "反射强度", "初始化"],
      description: "几何不够用了怎么办？有人去翻激光的第二个输出通道（反射率），有人回头补上被跳过的第一步（初始化）。含「反射率当照片 vs 当线」与「初始化三联」两张具象图。"
    },
    {
      id: "0091",
      path: "lessons/0091-退化环境与配准-可切换方案与低重叠对齐.html",
      paper: "stage-d",
      title: "退化环境与配准：可切换方案与低重叠对齐",
      subtitle: "组课 · Switch-SLAM ＋ 低重叠点云配准",
      emoji: "🚦",
      duration: "55 分钟",
      tags: ["组课", "退化", "配准"],
      description: "几何彻底不管用时，换谁来带路？含「切换初值」与「低重叠对应」两张具象图。后一篇是回环检测的对齐引擎，也是下一节课的前置。"
    },
    {
      id: "0092",
      path: "lessons/0092-激光回环上-几何路线与词袋路线.html",
      paper: "stage-d",
      title: "激光回环（上）：几何路线与词袋路线",
      subtitle: "组课 · Cartographer ＋ BoW3D ＋ LinK3D",
      emoji: "🧭",
      duration: "60 分钟",
      tags: ["组课", "回环", "词袋"],
      description: "车绕了一大圈回到原点，激光怎么知道自己「来过这儿」？有两条完全不同的路：几何搜索（分支定界）与外观检索（3D 词袋）。含「分支定界缩小搜索窗」与「3D 词袋索引卡」两张具象图。"
    },
    {
      id: "0093",
      path: "lessons/0093-激光回环下-旋转平移不变的地点识别.html",
      paper: "stage-d",
      title: "激光回环（下）：旋转平移不变的地点识别",
      subtitle: "RING++ · 关系类描述子",
      emoji: "📐",
      duration: "55 分钟",
      tags: ["地点识别", "旋转不变", "全局定位"],
      description: "同一个路口，去的时候面朝东、回来的时候面朝南——怎么让「像不像」这件事根本不受朝向影响？含「Gram 表两朝向一致」的具象图。论文真名是 RING++（路线图显示名漏了 ++）。"
    },
    {
      id: "0094",
      path: "lessons/0094-4D雷达SLAM-恶劣天气下的另一条路.html",
      paper: "stage-d",
      title: "4D 雷达 SLAM：恶劣天气下的另一条路",
      subtitle: "组课 · 收官 · 4D 成像雷达",
      emoji: "🌧️",
      duration: "60 分钟",
      tags: ["组课", "4D雷达", "收官"],
      description: "大雨、大雪、浓雾、烟尘里，激光会「瞎」——换一种会测速的雷达，能不能照样建图？本季收官，含四条主线的总收束与全季系统对照表。"
    },
    {
      id: "QA0002",
      path: "qa/qa-0002-MAP到非线性最小二乘.html",
      paper: "qa-0002",
      title: "追问答疑：从 MAP 到非线性最小二乘",
      subtitle: "把 0002 课那四行公式拆开揉碎",
      emoji: "🧮",
      duration: "45 分钟",
      tags: ["追问答疑", "贝叶斯", "最小二乘"],
      description: "对第 0002 课第 3 节四行公式的追问答疑：先补五个数学零件（贝叶斯、高斯、Ω、argmax、log 与取负），再逐式拆解，最后用走廊里的一台机器人把 x*=13.2 m 手算到底。含 7 张自绘图与 7 个新手常见困惑。"
    }
  ],

  /* ---------- 参考资料 ---------- */
  references: [
    {
      path: "reference/glossary.html",
      title: "课程术语表",
      emoji: "📖",
      description: "跨课程的中英对照术语与关键概念入口，按主题分组，方便查读。"
    },
    {
      path: "reference/路线图总览.html",
      title: "11 阶段路线图总览",
      emoji: "🗺️",
      description: "255 篇语料库的 11 个阶段、12 个技术大类与每阶段目标，一页看完。"
    }
  ],

  /* ---------- 学习记录 ---------- */
  learningRecords: [
    {
      path: "learning-records/0001-第一季学习记录.md",
      title: "第一季：建立坐标系",
      emoji: "📝",
      description: "第一季的点滴收获：关键概念、易混点、以及每节课留下的疑问。"
    },
    {
      path: "learning-records/0002-答疑记录-激光雷达分类三类vs两类.md",
      title: "答疑记录：激光雷达三类 vs 两类",
      emoji: "❓",
      description: "就地插进第 0006 课的答疑补丁：为什么原文一会儿说三类、一会儿说两类。"
    },
    {
      path: "learning-records/0003-答疑记录-MAP到非线性最小二乘.md",
      title: "答疑记录：MAP 到非线性最小二乘",
      emoji: "🧮",
      description: "第 0002 课四行公式的答疑记录：拆成 5 个数学零件 + 手算 13.2 m 算例，独立成页放在 `qa/`（不占主线编号），并与 0002 就地补丁双向互链。"
    },
    {
      path: "learning-records/0004-第二季学习记录.md",
      title: "第二季：打地基",
      emoji: "🏗️",
      description: "第二季的收获与债务清单：Schur 消元在五个场景的复用对照表、gauge freedom、「隐式 vs 混合 vs 显式」三篇对照表，以及五条留到后面阶段的疑问。"
    },
    {
      path: "learning-records/0005-第三季学习记录.md",
      title: "第三季：视觉主干",
      emoji: "📷",
      description: "第三季全季的收获与债务清单：六条疑问逐条销账、三条主线总收束（尺度暗线六站 / 架构的分工演化 / 滤波 vs 优化）、89 张具象自绘图的清单，以及留给后面阶段的五条疑问。"
    },
    {
      path: "learning-records/0006-第四季学习记录.md",
      title: "第四季：激光主干",
      emoji: "📡",
      description: "第四季的收获：四条主线（提特征→不提特征 / 数据结构 / 退化与鲁棒 / 地图与回环）、ikd-Tree 这笔债的完整链条与它的四件事，以及三处路线图显示名不符的勘误。"
    }
  ],

  /* ---------- 项目与官网 / 相关产出 ---------- */
  siteLinks: [
    {
      title: "255 篇论文阅读路线图（交互版）",
      url: "../outputs/论文阅读路线图.html",
      emoji: "🗺️",
      description: "带时间轴导航、技术大类筛选、难度阈值与实时搜索的全量路线图。"
    },
    {
      title: "论文语料库总目录",
      url: "../outputs/论文语料库总目录.md",
      emoji: "📚",
      description: "255 篇论文的清单与本地文件对应关系。"
    },
    {
      title: "SLAM 技术演进详解",
      url: "../outputs/SLAM技术演进详解.html",
      emoji: "🔬",
      description: "术语密度较高的速查/进阶版：五次范式迁移因果链与 9 条主线的世代演进。入门后精读。"
    },
    {
      title: "SLAM 图解入门",
      url: "../outputs/SLAM图解入门.html",
      emoji: "🎬",
      description: "通俗图文版：15 个手绘 SVG 动画 + 术语总表 + 场景选型器，用来先建立直觉。"
    },
    {
      title: "本地论文全文（Markdown）",
      url: "../md/",
      emoji: "📄",
      description: "所有课程的原始素材：255 篇论文的 Markdown 全文与插图，位于项目 md/ 目录。"
    }
  ]
};
