# -*- coding: utf-8 -*-
"""交付前全量校验器（第六季起的"发版前体检"，一个脚本跑完所有硬性检查）。

用法：
    python course/tools/validate_delivery.py

检查项：
  ① data/course-catalog.js 的条目字段完整性 / id 唯一性 / id 格式 / 路径存在性
  ② 每页：本地图片存在性、站内链接不 404、页内锚点有效、quiz id 对齐
  ③ CSS：变量有定义、类名有定义（**都在"该文件自身可见的样式范围内"判定**）
  ④ 不含 Markdown 语法残渣（`**`）、SVG 内不混入 HTML 标签、SVG 内不写 LaTeX

--------------------------------------------------------------------------
★ 三个必须按"每页各自可见范围"判定的坑（2026-09-20 踩过，全是假阳性）
--------------------------------------------------------------------------
本项目的样式来源有三处，**不同页面的可见范围不同**：
  a) `course/lessons/assets/slam-course.css`  —— 只有引它的页面看得到
  b) `course/shared/theme.css`                —— 只有引它的页面看得到
  c) 页面自带的 `<style>` 块 + 元素上的 `style="--x:..."`

第一版校验器只拿 (a)+(b) 当作"全部定义"，于是：
  · 成就卡（自包含页，变量定义在自己的 `<style>` 里）→ 报一堆"未定义 CSS 变量"；
  · 路线图总览（`.stage-badge/.yes/.no` 定义在自带 `<style>`）→ 报"未定义 CSS 类"。
两批都是假阳性。

第二个坑：**类名提取器必须过滤成合法 CSS 标识符**。`class="..."` 之外，
源码里还有 JS 字符串（如 `classList.add('...')` 拼出的片段），
裸抓会得到 `.(done`、`.?`、`.+` 这种鬼东西，必然报"未定义"。
所以只保留匹配 `^[A-Za-z_][A-Za-z0-9_-]*$` 的 token。
"""
import io, os, re, glob

ROOT = r"G:\project\ieeexplore\course"
LESS = os.path.join(ROOT, "lessons")
QA = os.path.join(ROOT, "qa")
REF = os.path.join(ROOT, "reference")
CSS_PATH = os.path.join(LESS, "assets/slam-course.css")
THEME_PATH = os.path.join(ROOT, "shared/theme.css")
CSS = io.open(CSS_PATH, encoding="utf-8").read()
THEME = io.open(THEME_PATH, encoding="utf-8").read()

# 全局（可被任一页面引用的）样式来源
GLOBAL_CSS = CSS + "\n" + THEME
GLOBAL_VARS = set(re.findall(r'(--[a-zA-Z0-9-]+)\s*:', GLOBAL_CSS))

# 这些类是"状态类"，由 JS 在运行时加上去，不一定有独立规则
SKIP = {"figure", "figcaption", "disabled", "next", "correct", "wrong", "show",
        "active", "exercise-answer", "open", "selected",
        # 语义标记类：样式写在元素的行内 style 上，本来就不该有 CSS 规则
        # （`7747541` 的「对照原文」条：<div class="lesson-source" style="...">，
        #  205 节 + 3 个答疑页都在用）。不放这里会报 208 条假阳性。
        "lesson-source"}

IDENT = re.compile(r'^[A-Za-z_][A-Za-z0-9_-]*$')

cat = io.open(os.path.join(ROOT, "data/course-catalog.js"), encoding="utf-8").read()
ach = io.open(os.path.join(ROOT, "data/achievement-cards.js"), encoding="utf-8").read()

problems = []


def styles_of(s):
    """取出一个 HTML 文件里**自身可见**的样式文本：
    自带 <style> 块 + 元素内联 style="" 里的自定义属性声明。"""
    blocks = "\n".join(re.findall(r'<style[^>]*>([\s\S]*?)</style>', s))
    inline = "\n".join(re.findall(r'style="([^"]*)"', s))
    return blocks, inline


# ── ① catalog 字段完整性 ──
REQ = ["id", "path", "paper", "title", "subtitle", "emoji", "duration", "tags", "description"]
blocks = re.findall(r'\{\s*\n\s*id: "[^"]+",[\s\S]*?\n    \},', cat)
# 只保留「块内确实有 path: 且不含 quickOpen」的课程条目，
# 避免懒匹配把相邻的 collection 块吞进来
les = [b for b in blocks
       if re.search(r'\n\s*path: "(?:lessons|qa)/', b) and "quickOpen:" not in b]
for b in les:
    lid = re.search(r'id: "([^"]+)"', b).group(1)
    miss = [k for k in REQ if not re.search(r'\b' + k + r':', b)]
    if miss:
        problems.append("catalog %s 缺字段 %s" % (lid, miss))
ids = [re.search(r'id: "([^"]+)"', b).group(1) for b in les]
for x in set(ids):
    if ids.count(x) > 1:
        problems.append("catalog id 重复: " + x)
for x in ids:
    if not re.fullmatch(r'\d{4}|QA\d{4}[A-Z]?', x):
        problems.append("catalog id 格式异常: " + x)
# 中文描述里的 ASCII 双引号会把外层字符串提前闭合（曾踩过）
badq = [i + 1 for i, l in enumerate(cat.split("\n"))
        if re.match(r'^\s*(description|subtitle|title):\s*"', l) and l.count('"') > 2]
problems += ["catalog 第 %d 行中文描述含多余 ASCII 引号" % i for i in badq]
for m in re.finditer(r'path:\s*"((?:lessons|reference|learning-records|qa)/[^"]*)"', cat):
    if not os.path.exists(os.path.join(ROOT, m.group(1))):
        problems.append("catalog 路径缺失: " + m.group(1))
for m in re.finditer(r'card_file:\s*"([^"]*)"', ach):
    if not os.path.exists(os.path.join(ROOT, m.group(1))):
        problems.append("成就卡 card_file 缺失: " + m.group(1))

# ── ② 逐页检查 ──
files = sorted(glob.glob(os.path.join(LESS, "*.html"))) + \
        sorted(glob.glob(os.path.join(QA, "*.html"))) + \
        sorted(glob.glob(os.path.join(REF, "*.html"))) + \
        [os.path.join(ROOT, "index.html")]
tot_img = miss_img = 0
for f in files:
    d = os.path.dirname(f)
    base = os.path.basename(f)
    s = io.open(f, encoding="utf-8", errors="ignore").read()
    own_blocks, own_inline = styles_of(s)
    is_selfcontained = ("slam-course.css" not in s and "theme.css" not in s)

    # 该页可见的样式范围：全局 + 自身
    available = GLOBAL_CSS if not is_selfcontained else ""
    available += "\n" + own_blocks + "\n" + own_inline
    avail_vars = set(re.findall(r'(--[a-zA-Z0-9-]+)\s*:', available))

    # 图片
    for x in re.findall(r'src="(\.\./\.\./[^"]*)"', s):
        tot_img += 1
        if not os.path.isfile(os.path.normpath(os.path.join(d, x))):
            miss_img += 1
            problems.append("%s 缺图 %s" % (base, x[-40:]))
    # 站内链接
    for l in re.findall(r'href="([^"#?]*\.html)(?:#[^"]*)?"', s):
        if l.startswith("http") or l.startswith("data:"):
            continue
        if not os.path.isfile(os.path.normpath(os.path.join(d, l))):
            problems.append("%s 断链 -> %s" % (base, l))
    # 锚点
    idset = set(re.findall(r'id="([A-Za-z0-9_-]+)"', s))
    for a in set(re.findall(r'href="#([A-Za-z0-9_-]+)"', s)):
        if a not in idset:
            problems.append("%s 失效锚点 #%s" % (base, a))
    # quiz
    for t in set(re.findall(r"lessonKit\.answer\(this,'([^']*)'", s)):
        if 'quiz-feedback" id="%s"' % t not in s:
            problems.append("%s quiz id 不一致: %s" % (base, t))
    # CSS 变量（只在该页可见范围内判定）
    for v in set(re.findall(r'var\((--[a-zA-Z0-9-]+)\)', s)):
        if v not in avail_vars:
            problems.append("%s 未定义 CSS 变量 %s" % (base, v))
    # CSS 类
    cls = set()
    for m in re.findall(r'class="([^"]*)"', s):
        cls.update(t for t in m.split() if IDENT.match(t))
    for c in sorted(cls - SKIP):
        # 只保留「类名后面不再接标识符字符」这一条约束。
        # ★ 不要在前面再加 (?<![\w-])：那样会把 `details.qa-patch` 这种
        #   「类型选择器 + 类选择器」的合法写法一起挡掉（踩过）。
        if not re.search(r'\.' + re.escape(c) + r'(?![\w-])', available):
            problems.append("%s 未定义 CSS 类 .%s" % (base, c))
    # Markdown 残渣（<pre>/<code> 里允许幂运算）
    stripped = re.sub(r'<pre[\s\S]*?</pre>|<code[\s\S]*?</code>', "", s)
    if stripped.count("**"):
        problems.append("%s 含 Markdown ** (%d)" % (base, stripped.count("**")))
    # SVG 内的 LaTeX / HTML 标签
    for m in re.finditer(r'<svg\b.*?</svg>', s, re.S):
        for t in re.finditer(r'<text\b[^>]*>(.*?)</text>', m.group(0), re.S):
            if '$' in t.group(1) or chr(92) in t.group(1):
                problems.append("%s SVG 内写 LaTeX" % base)
            if re.search(r'<(b|i|em|strong|br|span|p|div)\b', t.group(1)):
                problems.append("%s SVG 内混入 HTML 标签" % base)
    # 深色 + 窄屏：引样式表的页面由 CSS 文件统一处理；自包含页面必须自带
    # （第一、二季成就卡本来就是深底设计，无需深色适配）
    if is_selfcontained and base not in ("成就卡-第一季.html", "成就卡-第二季.html"):
        if "prefers-color-scheme" not in s:
            problems.append("%s（自包含）未做深色模式" % base)
        if "max-width" not in s:
            problems.append("%s（自包含）未做窄屏适配" % base)

problems = sorted(set(problems))
print("=" * 88)
print("文件数:", len(files), " 本地图片引用:", tot_img, " 缺失:", miss_img)
print("lesson 条目:", len(re.findall(r'path:\s*"lessons/', cat)),
      " qa 条目:", len(re.findall(r'path:\s*"qa/', cat)),
      " collection:", len(re.findall(r'quickOpen:', cat)))
print("问题总数:", len(problems))
print("=" * 88)
for p in problems[:40]:
    print("  ✗", p)
if not problems:
    print("  ✓ 全部通过")
