# NOTES.md — 交付偏好与踩坑记录

> 这份笔记是给"下一次继续做课"的人（包括未来的我）看的。每一条都是真实踩过的。

## 一、内容原则（用户明确要求过）

1. **不要根据标题推测论文内容。** 用户明确强调过这一点。必须读正文：摘要 → 引言贡献点 → 方法核心 → 实验结论。标题只用来"定位"。
2. **技术文档要通俗 + 图解。** 每个专业词首次出现时要解释；先用生活类比讲直觉，再上术语和公式。
3. **插图要"具象、可辨认"。** 抽象的点线方块图对新手无效——这是用户在别的项目里明确反馈过的。
4. **有原图就插原图。** 用正确相对路径，并解释"这张图在说明什么"。**没有关键原图时不要为了凑图硬塞无意义图片。**
5. **练习答案默认收起**，用 `<details>/<summary>`。
6. **讲课要详细、像助教带读**，不要只是提纲式总结；要主动说明"这一节和前面哪几节怎么接上"。

## 二、工程踩坑（会重复出现，务必先看）

### 1. 图片相对路径取决于"服务根目录"

- 课程页在 `course/lessons/`，图片在 `md/images/<stem>/`。
- 所以课程页里必须写 `../../md/images/<stem>/<hash>.jpg`。
- **必须从项目根目录 `G:\project\ieeexplore` 起服务**（`python -m http.server 8000`），
  然后访问 `http://localhost:8000/course/index.html`。
  从 `course/` 起服务会让 `../../md/` 逃出服务根目录，图片全挂。

### 2. Bash 工具的 PATH 缺失

本机 Bash 的 shell runtime 里 `dirname` / `head` / `ls` 会 `command not found`。每次用前先补 PATH：

```bash
export PATH="/c/Users/ZJQ/.workbuddy/binaries/PortableGit/versions/1.2.0/usr/bin:/c/Users/ZJQ/.workbuddy/binaries/PortableGit/versions/1.2.0/bin:$PATH"
```

### 3. 文件名与内容不符（已发现 2 处，可能还有）

- `2022_LiDAR-SLAM-Survey.md` → 正文其实是 **SW-NDT 方法论文**（动态环境激光 SLAM）。
- `2022_Beyond_Dents_and_Scratches_...md` → 正文是**工业图像异常检测**，与 SLAM 无关。
- **做法**：写课前先用 `grep -m1 '^# ' <file>.md` 批量核对真实标题，再决定怎么讲。

### 4. 元数据不一致

- `2024_DeepSLAM-Survey.md`：文件名 2024，正文 2023（IEEE Access）。
- `2020_EventSLAM-Survey.md`：文件名 2020，正文 2024 投稿 / 2026 出版。
- `2024_Recent_advances_in_3D_Gaussian_splatting.md`：**含 NUL 字节**，标准 Read 判为二进制，需 `tr -d '\000'` 先过滤。

### 5. 图片目录里混有非插图

作者头像、期刊 logo、页眉横幅都会混进 `md/images/`。**用图前先核对它是正文引用的插图**，
或在图注里诚实说明"这是表格扫描件 / 非插图"。

### 6. 表格与公式在 md 里普遍错位

OCR 会把 `√/×` 认成 `V/7/L/x/一`、把希腊字母下标弄丢、把连字符连成新词。
**凡要引用具体数值，回原 PDF 核对**；无法核对的，在图注或正文里明确标注不确定性。

### 7. CSS 变量未定义会「静默丢弃整条声明」（踩过，很隐蔽）

`theme.css` 原来的间距刻度只有偶数档（1/2/3/4/6/8/10/12/16/20/24）。
一旦写了 `padding: var(--space-5)`，`var()` 求值失败 → **整条 `padding` 声明被丢弃**（不是回退到 0，
而是整条失效）→ 表现为**文字直接贴着容器边框、元素之间没有间隙**。

已把刻度补全为 4px 基准的完整刻度。**改样式后必跑 skill 第 7 节第 ⑥ 项检查。**

同一类坑还有：`--ink-50` 未定义（theme.css 只有 20/40/60/80）。

### 8. headless Chrome 截图的两个坑

- `--screenshot` 的目标路径必须写 **Windows 风格**（`C:/Users/...`），
  写 `/c/Users/...` 会报 `系统找不到指定的路径 (0x3)`。
- **file:// 会被缓存**：不加 cache-buster 会截到上一版，让人误判"改动没生效"。
  正确姿势：`--incognito` + URL 末尾加 `?cb=$RANDOM`。
- `.html` 里带中文文件名时，URL 里要写成百分号编码，否则 Chrome 可能找不到文件。
  最省事的办法：截图时用 `file:///路径/lessons/0001-*.html?cb=...` 之前先用
  `ls lessons/ | head` 确认，或直接用 `python -c "import urllib.parse;print(urllib.parse.quote('文件名.html'))"` 生成。
- headless 环境缺 emoji 字体，**emoji 会显示成方框**，这是环境问题不是页面问题。

### 9. 条目少的网格会"左边挤、右边空"

`grid-template-columns: repeat(auto-fill, minmax(270px, 1fr))` 在只有 2 个条目时，
卡片占满左半边、右半边空着。改用
`repeat(auto-fit, minmax(300px, 360px))` + `justify-content: center` 让它们居中排布。

### 10. 课程页与首页用不同的样式表

- `index.html` 与 `reference/*.html` 引的是 **`shared/theme.css`**（设计令牌 + 卡片/按钮/表头）。
- `lessons/*.html` 引的是 **`lessons/assets/slam-course.css`**，它是**自成一体的**
  （硬编码颜色值，不依赖 theme.css 的变量）。

因此：课程页里**不能**用 `.tag` / `.btn` / `.site-header` 这类只在 theme.css 里定义的类；
反过来，首页里**不能**用 `.section-card` / `.callout` / `.figure` 这类只在 slam-course.css 里定义的类。
两边共有的只有 `.grid-2` / `.grid-3`（各自都定义了）。

## 三、常见 OCR 错误模式（写课时主动扫一遍）

| 模式 | 例子 | 正确 |
|---|---|---|
| 希腊字母 → 拉丁字母 | `Q is the velocity, Z is the angular` | ν / ω |
| 上下标箭头丢失 | `T_{st}` | `T_{s→t}` |
| 连字丢失（ff/fi/fl） | `eficiency`, `Scafold-GS` | efficiency, Scaffold-GS |
| 乘号丢失 | `16 16 pixel patch`, `9 9 pixels` | 16×16, 9×9 |
| 单独字母被吃掉 | `OCALIZATION`, `ISUAL` | LOCALIZATION, VISUAL |
| 上标 soft → 乱码 | `gt_{ij}^{50\hbar}` | `gt_{ij}^{soft}` |
| 数字 0 ↔ 字母 O | `0₁0₂`（相机中心） | `O₁O₂` |
| 人名重音丢失 | `Hidalgo-Carrio`, `Nister` | Carrió, Nistér |

## 四、HTML 里的 JS 陷阱（用户级记忆里也有）

**中文文案里出现 ASCII 双引号，而外层字符串也用双引号时，会破坏内嵌 JS。**
本课程页的项目里 JS 很少，但 `data/course-catalog.js` 里有中文 `description`——
写的时候**避免在双引号字符串内部再放 ASCII 双引号**，改用中文引号「」或“”。

改完 `course-catalog.js` 后建议自查：
```bash
node --check course/data/course-catalog.js
```

## 五、交付前必做校验

```bash
export PATH="/c/Users/ZJQ/.workbuddy/binaries/PortableGit/versions/1.2.0/usr/bin:/c/Users/ZJQ/.workbuddy/binaries/PortableGit/versions/1.2.0/bin:$PATH"
cd G:/project/ieeexplore/course

# ① catalog 语法
node --check data/course-catalog.js

# ② catalog 里每条 lesson 的 path 是否真实存在
grep -o 'path:"lessons/[^"]*"' data/course-catalog.js | sed 's/path:"//;s/"//' | while read p; do
  [ -f "$p" ] || echo "MISSING LESSON: $p"
done

# ③ 每节课引用的本地图片是否都存在
cd lessons
for f in *.html; do
  grep -o 'src="\.\./\.\./[^"]*"' "$f" | sed 's/src="//;s/"//' | while read p; do
    [ -f "../../$p" ] || echo "MISSING IMG in $f : $p"
  done
done

# ④ 每节课是否有：h1 / toc / 互动块 / 折叠练习 / 返回课程馆
for f in *.html; do
  printf '%s: h1=%s toc=%s quiz=%s ex=%s home=%s\n' "$f" \
    "$(grep -c '<h1>' $f)" "$(grep -c 'class="toc"' $f)" \
    "$(grep -c 'quiz-feedback' $f)" "$(grep -c 'exercise-answer' $f)" \
    "$(grep -c 'index.html' $f)"
done
```

## 六、续课流程（下次照做即可）

1. 从 `../meta/reading_map.txt` 取出下一阶段的论文清单。
2. **逐篇核对真实标题**（`grep -m1 '^# '`），排除误下载与命名错误。
3. 逐篇读正文（摘要 / 贡献点 / 方法 / 实验 / 局限），**并记下可用的插图 hash 与图注**。
4. 按认知顺序把论文拆成课（一篇可拆多节，同类可合并）。
5. 写 `lessons/00XX-*.html`，沿用 `assets/slam-course.css` 与 `lessonKit`。
6. 在 `data/course-catalog.js` 的 `lessons` 数组里追加条目；
   **如果是新阶段，同时在 `collections` 里新增一个 collection，不要重复建已有块。**
7. 跑第五节的校验脚本。
8. 更新 `learning-records/`、必要时更新本文件与本项目的 skill。

---

## 三、答疑的落盘约定（学习者偏好）

学习者明确要求：**对某节课某个知识点的疑问，讲解要插到那个 HTML 的对应位置去**，
不能只留在对话里。因此：

- 小疑问 → 就地插 `qa-patch` 折叠块（默认 `open`，先结论后依据，注明论文小节号，
  用 `<!-- QA-PATCH id=... date=... src=... -->` 包裹，便于检索与撤销）。
- 大疑问 → **独立成页，放进 `course/qa/` 目录**，命名 `qa-<源课号>-<主题>.html`。
  **答疑页不占主线课号**（主线永远是 0001、0002、0003…，第二季从 0017 开始），
  因为在别的会话里可能正并行写阶段 B 的课，两边编号绝不能互相占用。
- **双向互链是硬要求**：
  - 源课 → 答疑页：在卡住的那一段后面插 `qa-patch`，给一句话结论 + 跳转链接；
    并给补丁本体加 `id`（如 `id="q0002-map"`），这样答疑页能直接锚回原位置。
  - 答疑页 → 源课：hero 里放一条显眼的「回到原位置」链接，指向
    `../lessons/00XX-xxx.html#q0002-map`；页脚再给一条。
- **答疑页的相对路径**（它在 `course/qa/`，与 `course/lessons/` 同深度）：
  样式表 `../lessons/assets/slam-course.css`、脚本 `../lessons/assets/slam-course.js`、
  正文互链 `../lessons/00XX-*.html`、返回课程馆 `../index.html`；
  **原文插图仍是 `../../md/images/...`（深度不变）**。
- **catalog 归位**：答疑页归到「追问答疑馆」分馆下的 `qa-<源课号>` collection，
  **不进主线阶段块**（`paper` 不能写成 `stage-a`）。`lesson.id` 用 `QA0002` 这类前缀，
  避免与主线课号混淆（它同时是 localStorage 的完成标记键）。
- 每次答疑都在 `learning-records/` 留一条记录；出现新术语就补进 `reference/glossary.html`。
- 完整规则（含 HTML 片段、插入位置选择规则、catalog 更新方式）见
  `.workbuddy/skills/slam-paper-course/SKILL.md` 第 9 节。
- 已落地范例：
  - 通道 A：`lessons/0006-LiDAR里程计综述2024-算法分类与九大挑战.html` 里的
    `id="q0006-cat"`（辨析"三类 vs 两类"激光雷达分类）；
  - 通道 B：`qa/qa-0002-MAP到非线性最小二乘.html`（源自 0002，含 7 张自绘图 + 手算算例）。

### 为什么用默认展开的 `details`

既保证学习者下次通读时**一眼看到答案**，又能折叠起来不打断正文节奏。
`details.qa-patch` 的样式在 `lessons/assets/slam-course.css` 末尾（已含 `.qa-badge` /
`.qa-verdict` / `.qa-meta` 三个子元素类）。

---

## 四、第二季（阶段 B 打地基）的踩坑与新约定

### 11. 图片 hash 必须先核验再写进课里（新增硬规则）

第二季开工时我把所有计划使用的图片 hash 汇总成一张清单，用 `[ -f ... ]` 批量核验，
结果在 110 个里揪出 **1 个抄错的**：g2o 的 Fig. 5 原稿是
`1e99564f...98bc**7**f7a1b004a356ef489cd717b48eb0d5`，实际文件是
`1e99564f...98bc**1**f7a1b004a356ef489cd717b48eb0d5` —— **单字符之差**。

教训：subagent 汇报的 hash 只能当线索，**必须自己 `ls` 核验**。
建议做法：把本轮要用的 `(stem, hash)` 全部写进一个临时清单文件，跑一遍存在性检查，再交给写课的 agent。

### 12. 写课 agent 的产出必须自己再查三件事

第二季派了 10 个写作 agent 产出 23 节课，回收后自查发现：

- **0020 里残留了一处 Markdown 粗体** `**与传感器无关**` —— HTML 里会字面显示星号。
  查法：`grep -c '\*\*' 00*.html`，必须为 0。
- **子图与母图的归属必须写明**。例如「Fig. 6 的 (b) 子图」「Fig. 9 的两张之一」，
  否则读者回原文核对时会找不到。已要求 agent 一律标注。
- **「原文自己没有的东西」不许补**。本季典型三例：
  ① BA 综述正文**没有** Huber / Tukey（只有 Cauchy）——不能替它补；
  ② PCL 那篇**没有** limitations 小节、**没有任何性能数字**——如实说"这篇回答不了"；
  ③ 预积分那篇**没有**初始化章节、**全文没有 EuRoC**（EuRoC 是它的 2015 前作）——必须纠正。

### 13. 首页有两处"季节硬编码"文案

`index.html` 里「快速跳转」面板与 hero 副标题原先写死了"阶段 A / 第一季"。
换主线时会不一致（角标说 B、面板说 A）。已改为由 `featuredCollectionId` 动态生成。

**以后换主线（比如开阶段 C）只需改 `course-catalog.js` 的 `featuredCollectionId`**，
首页文案会自动跟着走——不要再回头改 index.html 的文案。

### 14. 两类"看起来像 bug 其实不是"的现象

- **首页 dump-dom 里 `class="lesson-card-wrap"` 的计数总比课程数多 1**：
  多出来的是 JS 模板字符串本身（`\`<div class="lesson-card-wrap...\``）。
  真实课程卡数 = 计数 − 1。
- **headless Chrome 里 emoji 显示成方框**：环境缺 emoji 字体，不是页面问题。

---

## 五、第三季（阶段 C 视觉主干）的踩坑与新约定

### 15. ★ 图片 hash 又一次被写错——同一个失败模式，必须提级为"回收后逐字复核"

第三季第一批写了 14 节课，**又在 0044 里抓到一个 hash 抄错**：
`...f77cff3cec**8**d3ce87a44ce98`（多了一个字符，实为 66 位），正确值是
`...f77cff3cec**0**d3ce87a44ce98`（64 位）。

**这是第二次了**（第二季是 g2o Fig.5 的 `98bc7f7a` vs `98bc1f7a`）。
两次都是"给了核验过的清单、agent 仍然抄错"，且**都是单字符级**.

**因此把流程固定成三步，不许省**：
1. 先把本轮要用的 `(stem, hash)` 批量核验，写成"存在性确认过"的清单文件；
2. 把清单连同**就近图注**一起交给写作 agent（`_tmp/figs/<stem>.txt` 就是这个格式）；
3. **回收后必须再跑一遍图片存在性检查**（第 11 节第 ③ 项）——这一步才是真正的防线。
   `--dump-dom` 校验查不出图片 404，只有本地截图或存在性检查能查出来。

### 16. 写作 agent 会漏"折叠练习的答案"与"底部导航"

第三季 0049、0050 两节：练习题干写全了，但**答案没有包在 `<details class="exercise-answer">` 里**
（违反"答案默认收起"），而且**整个 `<footer>` 都没写**（缺前后课导航与返回课程馆）。

**回收后必查这五项**（已有校验脚本，见第 11 节第 ④ 项）：
`h1` / `class="toc"` / `quiz-feedback` / `exercise-answer` / `index.html` —— **任一为 0 就是漏了**。

### 17. 写作 agent 会引用不存在的锚点

0040 的目录写了 4 个 `href="#b"` `#c` `#d`，但页面里只有 `id="a"` 与 `id="b"`；
0052 的目录有 `href="#f"` 但练习区块没给 `id="f"`。
**修法**：给对应标题加 `id`（`<h2 id="map">` 也可以被锚点命中），而不是删目录。

### 18. 自绘 SVG 的三类典型问题（`svgcheck.py` 每次都会抓到 1–3 个）

本批 42 张自绘图，跑出来 3 处问题，全是同一类根因——**坐标是手算的、没留余量**：
- **`rotate(90 cx cy)` 的竖排标签**：旋转后文字朝 +y 方向延伸，很容易撞上下方的横排标题。
  本次 0042 的「当前地图↑」撞上了「Mapping 线程（后台整理地图）」。→ 把竖排标签上移。
- **同一区域两个标签都用了 `text-anchor="middle"`**：各占一半宽度，边界处相撞。
  本次 0051 的「预测点」与「把观测点拉向预测点」只差 10px。→ 拉开 15px 以上。
- **文字 y 坐标超过了 `viewBox` 高度**：本次 0053 的 `viewBox="0 0 900 330"` 里放了 y=335 的文字。
  → 要么把 viewBox 调高，要么把文字上移。

**经验**：给 SVG 写文字时，**先在心里给每个标签划一个矩形**，确认矩形之间、矩形与画布边界之间都有余量。
不确定就把说明文字挪到 `figcaption`——那是最省事的解法。

### 19. 阶段 C 新增的教学规矩：抽象处必须配"具象"自绘图

学习者在本季开工前明确要求："尤其是一些晦涩抽象难懂的公式或者知识点，
要辅以**具象的 svg 图**或者通俗的解释一起解释。"

落到每节课上的硬要求：
- **3–5 张图里，至少 1–2 张是自绘图**（不能只插原文图）；
- **"具象"的标准**：画得出真实场景（房间、相机、桌子、卫星、行人）就绝不画抽象点线方块；
- **`figcaption` 固定三段式**：这张图在画什么 / 怎么看这张图（分步 ①②③）/ 要记住的一句话；
- **曲线与数值点必须用真实公式算**（Python 生成 → `<polyline>`），不许凭手感画；
- **交付前必跑 `course/tools/svgcheck.py`**，`OVERLAP` 与 `OUT-OF-BOX` 必须为空。

### 20. 共享规范文件 + 分篇图片参考 = 大幅省 token 的做法

第三季一次要开 37 节课，如果每份写作简报都复述一遍模板与规则，成本极高。
本次改成**三份共享文件**，agent 自己读：

| 文件 | 内容 |
|---|---|
| `_tmp/stageC-writing-rules.md` | HTML 模板、可用 CSS 类、lessonKit 接口、图片规则、**自绘 SVG 八条规矩**、质量要求、自查清单、前两季线索总表 |
| `_tmp/figs/<stem>.txt` | 该篇论文的**每个 hash + 就近图注**（自动生成） |
| `_tmp/stageC-precis-*.md` | 精读 agent 的完整产出（落盘，供写作 agent 直接引用） |

写作简报里只写"**这一节独有的**内容要点 + 文件名 + 前后课链接"。
**这个做法值得延续到阶段 D。**

---

## 六、第三季第二批（0054–0072）的踩坑与新约定

### 21. ★ 精读 agent 会纠正简报里的错误假设——这是好事，要在简报里主动要求

本批我派了 4 个精读 agent，结果**它们纠正了我简报里的三处错误假设**：

| 我原本以为 | 实际（agent 回正文核对后） |
|---|---|
| NFR 用 Delaunay 三角化选关键帧 | **不是**。NFR 是从被边缘化的子图里恢复一个非线性因子 |
| OpenVINS 覆盖了 OC-EKF | **只讲 FEJ**，未覆盖 OC-EKF |
| GNSS 是 1 Hz、VIO 是 10–20 Hz | **GNSS 实测 10 Hz**、相机 20 Hz、IMU 200 Hz |
| P³-VINS 会用 19 cm / 300 m 讲波长差 | **原文未给任何具体波长数值**，只有符号 λ |

**教训**：给 agent 的简报里如果带了"我记得是这样"的内容，**必须在简报里显式写一句**：
> 「如果我的描述与论文正文不符，**请以正文为准并明确指出来**——我需要知道我的假设错在哪。」

否则 agent 可能顺着我的错误假设去写，错误就被固化进课程了。

### 22. ★ 图片 hash 抄错第三次：`0066` 丢了 3 个字符

本批 `0066` 的 P³-VINS 图片：
- 正确：`ddfb4a036668acf58036155c69927f67af415b9510c3a281b82139c8a83a727b`（64 位）
- agent 写：`ddfb4a036668acf58036155c69927f67af415b9510c3a28139c8a83a727b`（**60 位**，丢了 `b82`）

这是**第三次**（第二季 g2o、第三季 0044、本批 0066），而且三次都是**单字符或少数几位丢失**。

**结论不变，但要再强调一次**：
> 交付前的**图片存在性检查是唯一防线**——`--dump-dom` 查不出 404，只有逐条 `os.path.isfile()` 能查出来。

建议把它写成 checklist 的**必跑第一项**，不要放在最后（越晚跑，返工成本越高）。

### 23. `svgcheck.py` 的新问题类型：文字压 viewBox 下边界

本批抓到 4 处，其中 3 处是**同一类型**：说明文字的 y 坐标离 viewBox 下边界太近（例如 viewBox 高 380、文字 y=378、字号 18），文字下沿直接越界。

**修法**（按省事程度排序）：
1. **把 viewBox 加高 15–20**（最省事，代价是底部略空）；
2. 把文字 y 上移 8–10；
3. 把整句说明挪到 `figcaption`（最干净，也最推荐）。

**已写进写作简报的硬规矩**：
> 底部说明文字的 y 坐标，要离 viewBox 下边界留出 **12px 以上**余量。

另一个类型是「标签压在同色框里、与框内文字相撞」（本批 0071）——修法是把标签移到框外。

### 24. ★ 精读产出必须落盘成文件（本批最重要的流程升级）

**背景**：会话上下文会被压缩。第一批（0040–0053）的精读结果**只在对话里**，压缩后就找不到了，直接导致第二批开工时无法复用。

**本批做法**：所有精读 agent 的产出**必须用 Write 工具落盘**到 `_tmp/precis-<主题>.md`：

| 文件 | 覆盖 |
|---|---|
| `_tmp/precis-学习式稠密.md` | CNN-SLAM / CodeSLAM / DeepFactors |
| `_tmp/precis-VIO主干下.md` | Basalt / OpenVINS / Kimera / DM-VIO / 纯惯性初始化 |
| `_tmp/precis-GNSS与自驾.md` | GVINS / P³-VINS / 自驾综述 |
| `_tmp/precis-动态环境.md` | DynaVINS / DGM-VINS / RLD-SLAM / CFP-SLAM / SG-SLAM |
| `_tmp/precis-线特征.md` | EPLF-VINS / AirVO / AirSLAM / UV-SLAM / DPL-SLAM / RD-VIO |
| `_tmp/precis-杂项与神经隐式.md` | 激光辅助单目 / NICER-SLAM / 无监督掩码 / 受限平台 / DN-SLAM |

**效果**：写作 agent 的简报里**不再复述内容要点**，只写「读哪个文件 + 这一节独有的文件名/标题/前后课 + 我们的硬要求」。省 token、更准、而且**上下文压缩后仍可用**。

### 25. 「组课」这种课型怎么写

本批有 3 节是**组课**（0068 动态环境五篇、0069/0070 线特征六篇）——不是逐篇细讲，而是**横向对比**。

写法要点：
- **先立问题意识**（为什么这是个难题），再立**一个统一框架**（如「判定 → 处理 → 代价」三层），然后**逐篇只给一句话招式**；
- **对比表是主干**（列＝论文，行＝同一组维度），原文没说的填「未提及」——**这个「未提及」很有价值，它诚实地标出了知识的边界**；
- **必须有一句"我们的判断"**：哪几篇真新颖、哪几篇偏增量——但要**注明这是判断而非原文自评**；
- **顺序不是平铺**，而是按框架（如「三条路」「两条提升路径」）组织。


---

## 七、第四季（阶段 D 激光主干）的踩坑与新约定

### 26. ★ 写作 agent 撞 429 速率限制：换模型口径即可恢复，不要停下等

本轮开工时一次派 4 个写作 agent，**全部失败**，报
`429 您的使用量已超出频率限制，将在 ... 重置`。

试了一下给 agent 加上 `model: "lite"` 参数——**立刻恢复**，而且产出的三节课质量完全达标
（自查全过、hash 正确、SVG 无重叠）。

**结论**：遇到 429 不要停在那里等重置，**先换模型口径再试**。重置可能要等 1–2 小时，换口径是几秒钟的事。

### 27. ★ 精读 agent 会主动纠正简报里的错误假设——这件事要主动邀请

本轮 6 个精读 agent 一共纠正了我**五处**错误假设：

| 我原本以为 | 实际（agent 回正文核对后） |
|---|---|
| LOAM「每次迭代重建 k-d 树」 | **每 sweep 重建一次地图树**，LM 每次迭代只做近邻查询，不重建 |
| 2019 那篇叫 VLOAM | 作者**自名 LIO-mapping（缩写 LIOM）** |
| DLO 是紧耦合 | **IMU 松耦合**（只给旋转先验） |
| KISS-ICP 用了 IMU | **不用 IMU**，纯激光 |
| Partially Overlapping 用 GNC / TEASER | **学习特征 + 图注意力 + RANSAC**，GNC/TEASER 只是基线 |

**教训**：给 agent 的简报里如果带了「我记得是这样」的内容，**必须在简报里显式写一句**：
> 「如果我的描述与论文正文不符，**请以正文为准并明确指出来**——我需要知道我的假设错在哪。」

不写这句，agent 可能顺着我的错误假设去写，错误就被固化进课程了。

### 28. ★ 图片 hash 抄错第五次——而且这次错在「精读文件」而不是写作 agent

本季 `0074` 的 Fig.7：

- 正确：`e381ad9e27155866d39d04e9efd63554b9fcef4eae35c65a27377a93e9540e2e`（64 位）
- 精读文件里写：`e381ad9e27155866d39d04e9efd63554b9fce4eae35c65a27377a93e9540e2e`（**63 位，丢了一个 `f`**）

这是**第五次**（第二季 g2o、第三季 0044、第三季 0066、本季 0074）。

**新增的认识**：以前我以为防线是「精读阶段批量核验」；但这次错的是精读文件的转录，
说明**核验必须在「写进课程之后」再跑一遍**——因为中转环节（agent 转录）本身会引入错误。

**所以流程改成四步**：
1. 自建 `(stem, hash)` 清单，**批量核验存在性**；
2. 连**就近图注**一起给写作 agent（`_tmp/figsD/<stem>.txt`）；
3. **写作完成后，对课页里每一个 `src="../../md/images/..."` 再跑一次存在性检查**；
4. 交付前再跑 `svgcheck.py`（几何）与首页 dump-dom（渲染）。

### 29. 「组课」这种课型怎么写（本季 22 节里有 7 节是组课）

阶段 D 有 7 节是**组课**——把 2–4 篇低位论文横向对比，而不是逐篇细讲。写法要点：

- **先立问题意识**（为什么这是个难题），再立**一个统一框架**（如「在传感器内部找答案 vs 在外部找答案」
  「判定 → 处理 → 代价」），然后**逐篇只给一句话招式**；
- **对比表是主干**（列＝论文，行＝同一组维度）；原文没说的填「未提及」——
  **这个「未提及」很有价值，它诚实地标出了知识的边界**；
- **必须有一句「我们的判断」**：哪几篇真新颖、哪几篇偏增量——但要**注明这是判断而非原文自评**；
- **顺序不是平铺**，而是按框架（如「两条变体路线」「两种用强度的方式」）组织。

### 30. 跨季维护：顺手做了两件小事

- **术语表全表 td 类名不统一**：早期组（第 1–8 组）用 `<td class="term">` / `<td class="en">`，
  近三季（第 12–16 组）直接写 `<td>`。已用正则批量补齐 **121 行**，全表现在统一了。
  —— **这属于「发现不一致就顺手修」，不要留着**。
- **0005 学习记录的描述还是「首批」口径**（只写了 0040–0053）：已更新为全季口径。
  —— 每做完一批课，**回头检查上一批的元数据描述是否还准确**。


### 31. ★ 新增 lesson 条目漏了 `id` 字段——而静态校验全都没抓到

本季新增 22 条 lesson 时，我只写了 `path / paper / title / subtitle / emoji / duration / tags / description`，
**漏掉了 `id`**（原有条目的格式是 `id` 在最前面）。

后果：首页每张卡片的圆圈里显示 `undefined`，`markDone('undefined', true)` 也会写坏进度键。

**为什么之前所有校验都没发现**：
- `node --check` 只查语法，`id` 缺失不影响语法；
- 「catalog 路径存在性」检查只查 `path` 指向的文件在不在；
- 「结构自检」查的是课页里的 h1/toc/quiz/练习，不查 catalog 字段完整性。

**唯一抓到它的是首页 `--dump-dom` 渲染检查**——因为只有真的渲染一遍，才会把 `undefined` 变成可见文本。

**因此新增一条硬规矩**：
> **catalog 字段完整性必须单独查**。至少要检查每条 lesson 都有
> `id / path / paper / title / subtitle / emoji / duration / tags / description` 九个字段；
> 并**统计首页渲染出的卡片编号里有没有非数字**（`lesson-num` 应为 4 位数字）。

**同时把「dump-dom 渲染检查」提到必跑项**——它不是「锦上添花」，
而是**目前唯一能发现「字段缺失导致渲染异常」的手段**。

### 32. ★★ 入场动画导致「内容整体看不见」——一次严重故障的完整复盘

**现象**：学习者反馈「主页的课程馆不正常显示，课程都看不到了」。

**根因**（不是布局问题，是可见性问题）：

```css
/* 旧写法（错误） */
.reveal { opacity: 0; transform: translateY(14px); transition: ...; }
.reveal.is-visible { opacity: 1; transform: translateY(0); }
```
```js
// 靠 IntersectionObserver 观察元素，进入视口才加 is-visible
document.querySelectorAll(".reveal:not(.is-visible)").forEach(n => revealObserver.observe(n));
```

课程卡片所在的整个分馆外壳是**JS 动态生成**的（`<section class="group-shell reveal">`），
而 `IntersectionObserver` 在**iframe 预览面板**这类环境里**只对部分元素回调**——
实测 9 个 `.reveal` 元素里只有 4 个拿到了 `is-visible`，
**剩下的（3 个分馆 + 成就馆）永久停在 `opacity: 0`**，于是整块课程内容「消失」。

**诊断路径（很值得记住）**：
1. `--dump-dom` 数卡片数 → **96 张都在**（说明 DOM 有内容，不是数据问题）；
2. 但截图看 → **几乎是空白**（说明是「可见性」问题，不是「有没有」问题）；
3. `grep` CSS 找到 `.reveal { opacity: 0 }`；
4. **决定性一步**：在 `--dump-dom` 输出里统计 `.reveal` 与 `.reveal.is-visible` 的数量 →
   **9 vs 4**，一眼定位"部分元素没被回调"。

**修复（渐进增强的正确姿势）**：

```css
/* 元素静态样式就是「可见」，动画只是播放一次的装饰，并且不设 fill-mode */
@keyframes fadeUp { from { opacity: 0; transform: translateY(14px); } to { opacity: 1; transform: none; } }
.reveal { animation: fadeUp .5s ease; }          /* 注意：没有 both / forwards */
@media (prefers-reduced-motion: reduce) { .reveal { animation: none; } }
```

> **核心原则：永远不要把「内容可见」这件事交给 JS。**
> 动画可以依赖 JS，可见性不可以。
> 具体的写法就是：**不要用 `opacity: 0` 做静态初始态**，改用「静态可见 + 一次性 `animation` 且不设
> `both`/`forwards`」。这样即使动画完全不播、JS 完全没跑、IO 完全不回调，内容也一定看得见。

**同时检查了同类风险**（用脚本扫全站「`animation ... both/forwards` + `from{opacity:0}`」
与「静态 `opacity:0` 且无切换类」两种模式）→ 全站仅此一处，`.holo-card` 是有意设计（成就卡锁定态）已排除。

**教训升级**：
> 视觉校验不能只问「内容在不在」（DOM 数量），必须同时问「看不看得见」（截图）。
> **两者缺一不可**：只看 DOM 会漏掉这次的可见性故障；只看截图会漏掉「卡片少了 3 张」这类数量问题。


### 33. 第五季（阶段 E 多传感器融合）的踩坑与新发现

**① 又发现两处路线图显示名不精确（累计第 6、7 处）**

| 路线图显示名 | 正文实际是 | 处理 |
|---|---|---|
| AdaFusion「视觉与激光的注意力式融合网络」 | **AdaFusion: Visual-LiDAR Fusion With Adaptive Weights for Place Recognition** —— 用途是**地点识别**，不是里程计融合 | 课里如实说明用途，并把它放进「回环/重定位」的语境 |
| 「多传感器鲁棒里程计（2022）」 | **Robust Odometry and Mapping for Multi-LiDAR Systems With Online Extrinsic Calibration** —— 是**多台同一种传感器**（多激光雷达）＋**在线外参标定**，**不是异源融合** | 课里明确澄清这个差别（它的价值在于「把传感器之间的关系当待估变量」与异源融合同类） |

> 到这里，本项目累计揪出的路线图问题：**4 处张冠李戴**（SW-NDT 冒充综述 / 工业异常检测误下载 /
> COLMAP 冒充 SfM 综述 / 自驾综述冒充 ORB-SLAM 会议版）＋ **多处显示名不精确**（SuMa→SuMa++、
> RING→RING++、Cartographer 未点名、AdaFusion 用途、M-LOAM 同质/异源）。**结论没变：文件名与
> 路线图标注都不可信，必须逐篇开正文。**

**② 精读 agent 又纠正了我四处错误假设（这条已经成了固定收益）**

本季四个精读 agent 全部在我的简报里挑出了错：

| 我写的假设 | 正文实际 |
|---|---|
| R³LIVE++ 引入了**视角相关的反射** | 不是——它的辐射量是每通道单值常量（朗伯、与视角无关）；视角相关是 LIV-GaussMap 用球谐才做的 |
| FAST-LIVO（v1）用**序贯更新** | 不是——v1 是**异步更新**，「序贯」是 FAST-LIVO2 的贡献 |
| v1 已经是「**同一份地图**、一个滤波器」 | 不是——v1 是「**两份地图**、一个滤波器」 |
| 第三篇属于 **LIVO 家族** | 不是——**不含 IMU**、特征法、非实时；它的角色是「因子图紧耦合路线」的对照样本 |

**这四处如果没被纠正，会原样写进课程。** 所以写作简报里必须保留那一句：
> 「如果我的描述与论文正文不符，请以正文为准并明确指出来——我需要知道我的假设错在哪。」

**③ `svgcheck.py` 又在「组课」上抓到 4 处重叠**

两节组课（`0096` 的多帧融合小图、`0103` 的两张对照图）共 4 处文字重叠。规律是：
**信息密度越高、元素越挤**（一格里既有圆点、又有标签、又有长说明），越容易撞。
三类典型：
- 一排并列的圆形图标**下方标签**与**再下一行的长说明**在 y 上只差 6px → 撞；
- 图内**左上角标签**与**居中标题**在同一水平带 → 撞；
- 图标内部已经写了字（如 `Cam`），外面又叠一个标签 → 撞。

**修法**（都不用改 viewBox，最省）：
- 拉大垂直间距（标签上移、长说明下移到框外）；
- 把长说明**缩短并挪到空白区**（本次把 15 字改成 13 字并移到右侧空位）。

**④ 我自己的两个老毛病又犯了一次（都在 `0095` 导览课）**

- **SVG 的 `<text>` 里写了 `<b>`**（应为 `<tspan font-weight="700">`）；
- **HTML 属性值里用了 ASCII 双引号**——`data-explanation="这是"用了同一个后端软件"，与耦合方式无关。"`。
  这**直接破坏了属性**（HTML 解析器在第二个 `"` 处就结束了属性），互动判断的解释文本会残缺。

**因此新增一条自检**（已并入 §11／§12）：
> 交付前对**每个 HTML 属性值**做一次「内部是否含未转义 ASCII 双引号」的检查：
> `re.findall(r'\w+="[^"]*"[^"]*"', html)` 命中即人工确认。
> 并且：**中文文案里的引号一律用「」或“”**——这条规矩早就写了，但还是会犯，
> 说明必须靠脚本兜，不能靠记性。

**⑤ 顺手做了一次「首页内容隐身」故障的复盘（见第 32 条）**

本季开工时用户报障「主页课程全看不到」，根因与修复都写在第 32 条。
这次是**把「视觉校验」补成两问**：「内容在不在」（DOM 数量）与「看不看得见」（截图）。


## 八、收编外部答疑页 + 成就馆可见性修复（2026-09-19 晚）

### 34. ★ 收编「别的会话产出的答疑页」，以及一个同族的可见性 bug

**背景**：学习者在别的话题里单独生成了 2 页答疑，直接扔进 `course/qa/`
（文件名是描述性长中文名，不符合 `qa-<源课号>-<主题>.html` 约定）。
他要我决定怎么处理，并挂进「追问答疑馆」。

**处理步骤（值得固化成流程）**：

1. **先验自包含性**——两页都自带 `<style>`（把课程页的样式原语拷进了页面）+ 自带内联
   quiz 绑定脚本（`document.querySelectorAll('.quiz-options button')`），**不引**
   `slam-course.css` / `slam-course.js`。
   → 结论：**保持自包含，不要改成引课程样式表**，否则反而会引入回归。
2. **改名到规范**：`从公式到实车：ROS 小车的…html` → `qa-0002b-从公式到实车.html`；
   `追问答疑：Ω 是矩阵…html` → `qa-0002c-Omega与先验与换传感器.html`
   （源课都是 0002，故与已有的 `qa-0002-*` 共用源课号 + 字母后缀）。
3. **★ 改名前先扫「指向旧名的链接」——抓到一处真断链**：
   `qa-0002c` 里写的是 `qa-0002-从公式到实车.html`，**该文件从来不存在**
   （另一个会话早就在用规范名写链接，只是文件没改名）。
   这是独立于改名的 bug，必须单独修。
4. **串成一条追问链**：三页页脚改成首尾相连（`qa-0002 → qa-0002b → qa-0002c → 0003 课`），
   链上「下一跳」用 `class="next"`，最后一页的 next 指回主线 0003。
5. **归入同一个 collection**（`qa-0002`），title/description/meta 改成「三页追问链」；
   `lessons` 挂 `QA0002 / QA0002B / QA0002C` 三条，`paper` 都填 `qa-0002`。
6. **源课补丁补一句**：0002 课的 `qa-patch` 里加一段「这一页还往下接了两页：…」，
   让读者从主线就能发现整条链。
7. **补跑 `svgcheck.py`（收编 = 认领质量责任）**：收编的 2 页各带 1 处问题——
   `qa-0002b` 的「相机视野」与路标标签重叠 10.3×2.2 px；
   `qa-0002c` 有一行说明文字 y=452 超出 `viewBox` 高度 440。
   修完**又暴露第 3 处**：`qa-0002c` 同一基线两个标签重叠 78.7×15.0 px
   （`C · 导航时…` 与 `先验 = (x₀, y₀, θ₀)…`）。
   **注意这个现象：把 OUT-OF-BOX 修掉之后，同一个 SVG 里的 OVERLAP 才被报出来——
   修完一定要再跑一遍，不要只跑一次。**

### 35. 成就馆「全息展示台」是一个空色块——又一次「可见性没人加类」

- **现象**：成就馆左侧展示台是一整块**空棕色渐变**，看不到 🔒 与「尚未解锁」。
- **根因**：`.holo-card { opacity: 0 }`，只有 `.unlocked` 才置 1；
  而 JS 在未解锁时**只加 `.locked`，CSS 里没有对应的置 1 规则**，
  于是卡片本体与它的 `.lock-overlay` 全程不可见。
- **修法**：加一条 `.holo-card.locked { opacity: 1; }`。
- **这与第 32 条是同族问题**：**「内容是否可见」不能只由某个类决定，而那个类没人加。**
  以后写展示型组件时，**默认态就该是「可见」，需要隐藏才显式加类**。


---

## 九、第六季（阶段 F）与两项全站补齐（2026-09-20）

### 36. ★ 又抓到一处「路线图条目根本不是论文」

阶段 F 的清单有 39 行，逐行解析后**只有 36 篇是唯一论文**，三类要剔除：

| 路线图写法 | 实际情况 |
|---|---|
| 「Co-SLAM」＋「Co-SLAM 期刊版」两行 | **同一篇的重复录入**（正文词数完全相同，两个图片目录内容也相同）；语料库里**没有**期刊版。另外按文件名模糊匹配会误命中 `2022_DiSCo-SLAM.md`（那是**多机器人激光 SLAM**，与 Co-SLAM 无关） |
| 「NeRF-SLAM」＋「实时稠密单目 NeRF-SLAM」两行 | **同一篇的重复录入**（同上） |
| 「前馈式（免优化）高斯重建网络」 | `2024_GridFormer__Residual_Dense_Transformer_with_Grid_Structu.md` 的正文是**图像去雨 / 去雾**（Image Restoration in Adverse Weather Conditions），与 3D 高斯**完全无关** |

**判定方法**（可复用）：① 比较两篇 md 的**去空白词数**是否完全相同；② 比较两个图片目录的文件名集合是否相同；
③ 对可疑篇 `grep -m1 '^# '` 读标题 + 读摘要。本次还用 `ls *.md | grep -iE 'pixelsplat|mvsplat|generaliz|feed'`
去语料库里找**真正的**前馈式高斯重建工作（结论：没有，所以「可泛化」这条线改由论文自身确实做前馈高斯的
`0124` GPS-Gaussian 承担）。

> 这是本项目第 6–8 处路线图问题（前五处见 skill 第 13 节）。**"文件夹里有这个文件名"不等于"存在这篇论文"。**

### 37. ★ 深色模式：一个开关 + 三个坑

**开关**：headless Chrome 里**只有 `--force-dark-mode` 能让页面看到 `prefers-color-scheme: dark`**
（实测 `DARK=true`）。反过来 `--blink-settings=preferredColorScheme=1` 与
`--force-prefers-color-scheme=dark` **实测都无效**，别浪费时间去试。

**坑**：
1. **`--user-data-dir` 不要与截图文件名相同** —— 写成 `--user-data-dir="$OUT/idx_dark.png"` 时，
   Chrome 先建了同名**目录**，随后截图报 `拒绝访问 (0x5)`（看起来像权限问题，其实是路径自撞）。
2. **`.figure` 在深色模式下必须保持浅色**。全站 200+ 张自绘 SVG 的颜色是**写死在文件里**的
   （浅底 + 深笔画），浮在深色页面上笔画会看不见。图框留浅色 = "深色版面上贴一张浅色印刷图"。
3. **深色块要写成 `@media screen and (...)`**，漏了 `screen` 的话，深色模式下打印会整页铺黑
   （`theme.css` 里 `@media print{body{background:white}}` 的意愿会被覆盖）。

**验证的硬证据不是肉眼，是探针**：在 `</body>` 前注入一小段脚本，把
`matchMedia('(prefers-color-scheme: dark)').matches`、`getComputedStyle(body).backgroundColor`、
`getPropertyValue('--paper')` 写进 `document.title`，再 `--dump-dom` 读回来。
基准值：首页浅色 `--paper` = `#F7F4EE` / 深色 `#1b1614`。

### 38. ★★ 窄屏：真凶只有两个，而且都很反直觉

**现象**：手机上打开某页，**连正文都被切掉一半**（不是"字排得难看"，是整页变宽、居中失效、左侧被裁）。

**真凶**（390px 实测全站 139 页，只有这两类）：
1. **表格** —— 多列 + 单元格里有 `/camera/image_raw` 这种**不可断的长串**；
2. **超长行内 `<code>`** —— 论文路径 `md/2016_Past__Present__...`；
3. （修完上面两条后又暴露的）**网格子项的 `min-width:auto`** —— 格子里有一个不可断的长英文词，
   整列就被撑宽（`0027` 与 `0037` 两页是这样，各差 27px / 2px）。

**修法**：
```css
th,td{overflow-wrap:anywhere;word-break:break-word}
code{overflow-wrap:anywhere}
@media (max-width:820px){
  table{display:block;overflow-x:auto;-webkit-overflow-scrolling:touch}
  .lead-grid > *, .grid-2 > *, .grid-3 > *, .summary-grid > *{min-width:0}
  .section-card p, .section-card li, .callout, .mini-card{overflow-wrap:anywhere}
}
```
`table{display:block}` 会让 `thead/tbody` 共同生成一个**匿名 table 包裹盒**，
所以**列宽仍共享、对齐不破**，同时表格自己可横向滚动。

### 39. ★ 新增工具 `course/tools/responsive_check.py`——为什么不能用 `--window-size`

**headless Chrome 的 `--window-size` 对 `--dump-dom` 不生效**（实测视口恒为 485，
且受 Windows 显示缩放影响；`--force-device-scale-factor=1` 也压不住）。
所以窄屏测量必须走「**探针页 + 精确宽度 iframe**」：逐页量 `scrollWidth - clientWidth`，
并**定位真正越界的元素**（排除被祖先 `overflow` 裁掉的那些，否则满屏假警报）。

工具自身也有两个坑：
- 探针写在 `_tmp/_rwd_probe.html`（gitignore 内）并**故意不删** —— 本机有 **safe-delete 钩子**，
  脚本内删除文件会被拦下，并把钩子提示写进重定向文件，**看起来"跑完了"其实什么都没测**。
- iframe 的 `src` **必须用绝对 `file://` URL**。用相对路径会相对探针所在目录（`_tmp/`）解析，
  全部 404 → 报 `Blocked a frame with origin "file://" from accessing a cross-origin frame`（像跨域问题，其实是路径问题）。

### 40. 自包含页面是适配的"盲区"

`qa/qa-0002b`、`qa-qa0002c`、`reference/成就卡-*.html` **不引课程样式表**，
所以课程样式表里的窄屏块与深色块**对它们无效**——必须自己再写一遍。
规则见 skill 第 12.7 节。**新增自包含页面时，这两块是必填项。**

（顺带发现：6 张成就卡里，**第一、二季本来就是深底设计**，无需深色适配；
第三～六季是浅底页面，需要。）


---

## 十、窄屏收口的二次返工（2026-09-20 晚，同日）

> 第 38、39 条是同一晚**写早了的结论**，其中两处是错的。本节把它们改对，
> 并补上真正解决问题的那几块拼图。**以本节为准。**

### 41. ★★ 修正第 38 条：`anywhere` 有代价，正确做法是「分层」

第 38 条说"修法是 `th,td{overflow-wrap:anywhere}`"。**这条只对了一半** ——
它确实能收回溢出，但 `anywhere` 不只会断长串，它是**一条"允许在任意处断行"的许可**，
和容器多宽无关。后果实测：

- 术语表的「课次」列在**桌面宽度**下被压成竖排，`0001` 断成 `000` + `1`
  （不是"窄屏才有的毛病"）；
- 列宽会被压到最窄，因为 `anywhere` **把「最小内容宽度」也算成 1 个字符**。

**正确分层**（现在样式表里就是这么写的）：

| 位置 | 用哪个 | 理由 |
|---|---|---|
| 全局 `th,td` / `code` | `break-word` | 只在"这一整行放不下"时断，`0001` 永远不会被拆 |
| `@media (max-width:820px) th,td` | `anywhere` | 窄屏必须压 min-content（见第 43 条），代价用下一条兜住 |
| `@media (max-width:820px) .term-group td:last-child` | `white-space:nowrap` | 术语表「课次」列显式保护，min-content 固定为几个字符宽 |

> 一句话记法：**能放得下就别断**（break-word）是常态，**必须压到最窄**（anywhere）
> 只在窄屏、且必须点名保护那几个"短而被拆"的列。

### 42. ★★ 修正第 39 条：`--window-size` 的真实行为（我因此误判过一轮）

第 39 条说"`--window-size` 对 `--dump-dom` 不生效、视口恒为 485"。**不准确。**实测：

- `--window-size=390,900` + `--dump-dom` → `clientWidth` = **500**（不是 485）；
  `--window-size=800,900` → 784；`--window-size=1440,900` → 1424。
  也就是说**它生效，但 headless 有「最小窗口宽度 500px」的下限**，
  比 500 小的值一律被夹到 500。
- 更要命的是**截图**：`--window-size=390,2600` 输出的图确实是 390px 宽，
  但**布局视口仍是 500px**，只是把 500px 的画面裁到 390px 输出。
  → 截图里"右侧内容被切掉"是**裁剪**，不是页面溢出。
  我据此写过一次错误的 A/B 结论（"旧版整页被撑宽"），白做了一轮。

**所以视觉验证只有一条可靠路径**：造一个探针页，里面放一个**恰好 N px 宽**的 iframe，
再整页截图 —— iframe 内部就是真实的 N px 布局。工具：`_tmp/shot_narrow.py`。

（测量仍然走 iframe；这条只是补上"截图"这一环的坑。）

### 43. ★★★ 一个反直觉的现象：**文档级 scrollWidth 收不住**

窄屏下 qa-0002b 的表格溢出 164px。表格**自己**明明已经是滚动容器
（`display:block; overflow-x:auto`，自身 rect 只有 444px，`scrollWidth` 552、内部确实在滚），
但文档级 `scrollWidth` 仍然是 554。**试过、全部实测无效**：

| 尝试 | 结果 |
|---|---|
| `html{overflow-x:clip}` | sw 仍 554 |
| `body{overflow-x:clip}` | sw 仍 554 |
| `.page{overflow-x:clip}` | sw 仍 554 |
| `.section-card{overflow-x:hidden}` | sw 仍 554 |
| `.section-card{overflow-x:clip}` | sw 仍 554 |
| 把 `<table>` 包进 `<div style="overflow-x:auto">`，表格改回 `display:table` | sw 仍 456 |
| 让 `.katex` 自己 `overflow-x:auto` 横滚 | sw 仍 456 |
| **`th,td{overflow-wrap:anywhere}`** | **sw → 390 ✓** |

结论：这**不是普通盒树溢出**（否则 clip 一定收得住），而是
**「单元格的最小内容宽度在撑表格」**——溢出量是**内在宽度**决定的，
所以唯一有效的杠杆是**把 min-content 压下去**，而不是"再加一层滚动容器"。

**归因方法**也值得记：不要猜，用「隐藏法」——借 iframe 精确定宽，
依次 `display:none` 掉一类元素再量 `documentElement.scrollWidth`，
谁让数值回落谁就是元凶（工具 `_tmp/hide_iframe.py`）。
本次就是靠它一步步定位到「表格 → 单元格里的长不可断 ASCII 串（KaTeX 源码/长路径）」。

> 另外一个**探针写重了会被静默截断**的坑：`--dump-dom` 的 virtual-time 快照时机不稳，
> 探针里若有"遍历几千个元素 + 逐个 getComputedStyle"的循环，报告经常停在 `pending`。
> 一次只测一个假设、循环尽量短，反而稳定得多。

### 44. ★ `min-width:0` 必须放全局——第 38 条把它放进 media query 返工了一次

第 38 条把 `.lead-grid > *{min-width:0}` 写在 `@media (max-width:820px)` 里。
**1024px 下它不生效**，于是 0060 溢出了 14px（元凶：23 个 `.mini-card`）。

判断标准很简单：**这条规则和视口宽度有关吗？**
- 「能不能收缩」（`min-width:0`）、「长 token 能不能断」（`overflow-wrap`）
  → 与宽度无关，**全局**；
- 「表格要不要变成横滚块」「字号要不要变小」→ 与宽度有关，**media query**。

### 45. ★ KaTeX 的两处约束（表格被撑宽的最后一块拼图）

全部 127 节 + 3 页答疑都从 CDN 引 KaTeX 渲染 `$...$`。它带来两个约束：

1. **`.katex .base{width:min-content; white-space:nowrap}`**
   —— 公式成了一个"恰好等于自身宽度"的**不可断** inline-block，
   于是**单元格的 min-content 被锁死在公式宽度上**（qa-0002b 实测 456px）。
   `overflow-wrap` 对它完全无效（那是"文本断行"属性，管不了 nowrap 的 inline-block）。
   解法：窄屏把 base 放回可换行 —— `.katex .base{white-space:normal;width:auto}`
   （仅窄屏；桌面保持原排版）。**这一条加上后，全站 390px 溢出归零。**
2. **`.katex-display` 的 center-overflow 问题**
   —— `.katex-display > .katex` 是 `text-align:center`。一旦给它加
   `overflow-x:auto` 让它横滚，**溢出到左侧的那段是滚不到的**（经典问题）。
   所以必须同时 `.katex-display > .katex{text-align:left}`。

### 46. 术语表原来一直是"裸表格"

`reference/glossary.html` 一直在用 `.term-group / .term / .en / .note` 四个类，
**而样式表里从来没有定义过它们**——一直靠 `th,td` 的默认对齐在撑。
本次补齐（第 12.8 节），并踩到列宽的一个细节：

窄屏下表格是 `display:block`，列宽百分比要在**匿名内表**的宽度上解析，
而匿名内表宽度是 `auto`（不确定）→ **百分比被当 `auto` 丢掉**，
实测写 `width:26%` 毫无效果；**能用的是 `min-width`**（自动布局里它是硬下限）。
5.5em 让「中文」列从"三个字一行"回到"四个字一行"。

### 47. 工具升级：`_tmp/validate_all_f.py` → `course/tools/validate_delivery.py`

从临时脚本提升为正式工具（和 `svgcheck.py` / `responsive_check.py` 并列）。
修掉三类**假阳性**，每一类都值得记住：

| 假阳性 | 原因 | 修法 |
|---|---|---|
| 报一堆「未定义 CSS 变量」（成就卡自包含页） | 只拿全局两个样式表当"全部定义"，忽略了页面**自带的 `<style>` 与内联 `style="--x:…"`** | 按"该页可见范围"判定：全局(若引用) + 自带 style + 内联 |
| 报「未定义 CSS 类 `.stage-badge` / `.yes` / `.no`」（路线图总览） | 同上，类定义在页面自带 `<style>` 里 | 同上 |
| 报 `.(done` / `.?` / `.+` 这种鬼类名 | 裸抓 `class="…"`，把 JS 字符串片段也抓进来 | 类名 token 必须过 `^[A-Za-z_][A-Za-z0-9_-]*$` |
| 报「未定义 `.qa-patch`」（0002/0006 课页） | 断言里加了 `(?<![\w-])\.`，把 `details.qa-patch` 这种**类型选择器 + 类选择器**的合法写法挡掉了 | 去掉前面的负向回看，只保留"类名后面不再接标识符字符" |

### 48. 本次最终实测数据

```
responsive_check.py --widths 390,768,1024
  宽度 390 px   共 139 页，溢出 0 页   ✓
  宽度 768 px   共 139 页，溢出 0 页   ✓
  宽度 1024 px  共 139 页，溢出 0 页   ✓
validate_delivery.py → 问题总数 0，139 文件 / 538 张本地图片全部存在
```

（起点的状态是：qa-0002b 在 390px 溢出 164px、0027 差 27px、0037 差 2px、
0060 在 1024px 差 14px、术语表「课次」列被 `anywhere` 拆成两行。）
