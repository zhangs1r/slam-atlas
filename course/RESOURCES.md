# RESOURCES.md — 可信资源清单

本课程只使用以下资源。**任何课上的技术判断，都必须能从这里的材料里指回出处。**

## 一手素材（唯一的内容来源）

| 资源 | 路径 | 说明 |
|---|---|---|
| 论文 Markdown 全文 | `../md/<论文名>.md` | 255 篇论文的正文。由 PDF 经 MinerU 转换而来，**含 OCR 错误，必须甄别**。 |
| 论文插图 | `../md/images/<论文名>/<hash>.jpg` | 9081 张原文插图，路径与 md 文件名严格对应。 |
| 逐篇技术索引 | `../md/_digest.md`、`../md/_compact.md` | 抽取式索引（摘要 / 贡献点 / 方法段 / 实验设置），**不做生成式改写**，可用于快速定位但要读正文核实。 |
| 阅读路线图源数据 | `../meta/reading_map.txt` | 每行一条：`阶段|分类|stem关键词|显示名|难度|重要性|一句话`。 |
| 论文→文件映射 | `../meta/_map.json` | 路线图条目与实际 md 文件的对应关系。 |

## 配套产出（课程之外的材料）

| 资源 | 路径 | 说明 |
|---|---|---|
| 阅读路线图（交互版） | `../outputs/论文阅读路线图.html` | 时间轴导航 + 12 类筛选 + 难度阈值 + 实时搜索 |
| 阅读路线图（纯清单） | `../outputs/论文阅读路线图.md` | 同上的 Markdown 版本 |
| SLAM 技术演进详解 | `../outputs/SLAM技术演进详解.html` | 术语密度高，速查/进阶用 |
| SLAM 图解入门 | `../outputs/SLAM图解入门.html` | 15 个手绘 SVG 动画 + 术语总表 + 场景选型器 |
| 语料库总目录 | `../outputs/论文语料库总目录.md` | 255 篇清单 |
| 跨出版商下载能力实测报告 | `../outputs/跨出版商下载能力实测报告.html` | 语料库是怎么拿到的 |

## 课程内部结构

```
course/
├── index.html                  # 课程馆首页（三层：分馆 → 来源块 → 课程页）
├── data/course-catalog.js      # ★ 唯一需要维护的数据文件
├── data/achievement-cards.js   # 成就卡数据（一张卡对应一个来源块）
├── shared/theme.css            # 全局主题（学术出版风 / 米白纸感 / 赤陶橙）
├── lessons/
│   ├── assets/slam-course.css  # 课程页样式
│   ├── assets/slam-course.js   # 课程页交互（lessonKit）
│   └── 00XX-*.html             # 课程页
├── reference/                  # 术语表、路线图总览、成就卡 HTML
├── learning-records/           # 学习记录
├── MISSION.md / RESOURCES.md / NOTES.md
└── 项目级 skill：../../.workbuddy/skills/slam-paper-course/SKILL.md
```

> 首页还带：`Ctrl/Cmd + K` 课程搜索（支持按标签筛）、每节课的完成标记（`localStorage`
> 键名 `paperlesson_done`）、连学天数（`paperlesson_vtally`）、以及按来源块解锁的**成就馆**全息卡。

## 外部工具与字体

- 公式渲染：KaTeX 0.16.10（CDN）
- 字体：Noto Serif SC（标题）/ Inter（正文）/ JetBrains Mono（代码），均从 Google Fonts 加载
- **插图不走网络**：全部是本地 `md/images/` 相对路径，因此必须**从项目根目录起服务**才能正常显示

## 使用边界

- 本课程是**学习材料**，不是论文原文的替代品。引用任何数字前请回论文原文核对。
- 论文版权归原作者与出版方所有；本地语料库仅供个人学习研究使用。
- 路线图与课程中都标注了**已撤稿论文**（`2023_RETRACTED_ARTICLE`），仅作收藏警示，**不要引用**。
