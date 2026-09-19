#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""make_digest2.py — 从 Markdown 全文抽取技术内容，生成"可读索引"

设计原则（对应"必须读正文、不许凭标题推测"的要求）：
  本脚本只做**抽取式摘要**——所有文字都直接来自论文正文，不做任何生成式改写。
  这样后续撰写图谱/综述时，每一条技术描述都能指回原文出处。

每篇抽取：
  ① 标题 ② 摘要 ③ 章节结构 ④ 贡献点句（引言里声称的 novelty）
  ⑤ 方法核心段 ⑥ 实验设置（数据集/基线）⑦ 关键量化结果 ⑧ 结论/局限

用法: python make_digest2.py [--out md/_digest.md] [--min-chars 2000]
"""
import argparse
import glob
import os
import re
import sys

ROOT = r"G:/project/ieeexplore"
MD_DIR = os.path.join(ROOT, "md")

# ── 主题词表：用于给每篇论文自动打标签 ──────────────────────────
TECHS = {
    "特征法": ["ORB", "SIFT", "SURF", "feature point", "feature-based", "keypoint"],
    "直接法": ["direct method", "photometric error", "direct sparse", "DSO", "LSD-SLAM"],
    "半直接法": ["semi-direct", "SVO"],
    "滤波后端": ["EKF", "extended Kalman", "iterated Kalman", "ESKF", "MSCKF", "IEKF", "error-state"],
    "优化后端": ["sliding window", "bundle adjustment", "factor graph", "pose graph",
                "marginalization", "optimization-based", "Gauss-Newton", "Levenberg"],
    "IMU预积分": ["preintegration", "pre-integration", "IMU factor"],
    "激光里程计": ["scan matching", "ICP", "GICP", "NDT", "point-to-plane", "LOAM"],
    "点云地图结构": ["ikd-Tree", "k-d tree", "iVox", "voxel", "octree", "surfel", "hash"],
    "LiDAR-惯性耦合": ["LiDAR-inertial", "lidar inertial", "LIO", "tightly-coupled lidar"],
    "视觉-惯性耦合": ["visual-inertial", "VIO", "visual inertial"],
    "LiDAR-视觉融合": ["LiDAR-visual", "lidar camera fusion", "LIVO", "LVI-SAM", "LIV"],
    "回环检测": ["loop closure", "loop-closure", "place recognition", "relocalization",
                "revisiting", "DBoW", "scan context"],
    "神经隐式场": ["NeRF", "neural radiance", "implicit neural", "SDF", "signed distance",
                  "MLP", "feature grid", "hash encoding", "occupancy network"],
    "3D高斯": ["Gaussian splatting", "3D Gaussian", "splatting", "Gaussians"],
    "深度学习前端": ["CNN", "transformer", "attention", "supervised learning",
                    "self-supervised", "neural network", "deep learning", "end-to-end"],
    "基础模型": ["foundation model", "DINO", "CLIP", "SAM", "DUSt3R", "MASt3R", "vision-language"],
    "语义建图": ["semantic", "panoptic", "instance segmentation", "open-vocabulary"],
    "动态环境": ["dynamic environment", "moving object", "dynamic object", "DynaSLAM"],
    "多机协同": ["multi-robot", "collaborative", "multi-agent", "distributed", "swarm",
                "multi-UAV", "decentralized"],
    "事件相机": ["event camera", "event-based", "event stream", "time surface"],
    "GNSS融合": ["GNSS", "GPS", "global navigation satellite"],
    "稠密重建": ["dense reconstruction", "TSDF", "volumetric", "mesh", "surfel", "truncated signed"],
    "全局一致性": ["global consistency", "globally consistent", "full BA", "submap alignment"],
    "嵌入式实时": ["real-time", "embedded", "Jetson", "onboard", "lightweight"],
}

DATASET_PAT = re.compile(
    r"\b(KITTI|EuRoC|TUM RGB-D|TUM|Newer College|M2DGR|Hilti|Replica|ScanNet|"
    r"Tanks and Temples|FastLIO|NCLT|Boreas|NTU VIRAL|OpenLORIS|ICL-NUIM|"
    r"Waymo|nuScenes|MARS|SubT|Zurich|ETH3D|DTU|BlendedMVS|ARKitScenes)\b")

# 贡献点常见引导语
CONTRIB_PAT = re.compile(
    r"((?:the\s+)?(?:main\s+|key\s+|our\s+|primary\s+)?contributions?\s+(?:of\s+this\s+\w+\s+)?"
    r"(?:are|is|can be summarized as|include)[:\s])", re.I)


def load(md_path):
    return open(md_path, encoding="utf-8", errors="replace").read()


def strip_images(t):
    return re.sub(r"!\[[^\]]*\]\([^)]*\)", "", t)


def clean(t):
    t = re.sub(r"\n{3,}", "\n\n", t)
    t = re.sub(r"[ \t]{2,}", " ", t)
    return t.strip()


def get_title(t):
    m = re.match(r"\s*#\s*(.+)", t)
    return m.group(1).strip() if m else ""


def get_abstract(t):
    # 摘要通常在 Abstract/h1 之后、Introduction 之前
    m = re.search(r"(?:^|\n)#{1,4}\s*Abstract[^\n]*\n+(.{80,3000}?)"
                  r"(?=\n#{1,4}\s|\n\*\*(?:Index Terms|Keywords)|\Z)", t, re.S | re.I)
    if m:
        return clean(strip_images(m.group(1)))
    # 退而求其次：取正文开头
    body = re.sub(r"^#.*?\n", "", t, count=1)
    return clean(strip_images(body[:1600]))


def get_sections(t):
    out = []
    for m in re.finditer(r"^(#{1,3})\s+(.+)$", t, re.M):
        h = m.group(2).strip()
        if len(h) > 100:
            continue
        if re.match(r"^(references|appendix|acknowledg)", h, re.I):
            continue
        out.append(h)
    return out


def get_contributions(t, limit=900):
    """找引言里声称的贡献点，原文照抄，不改写"""
    for m in CONTRIB_PAT.finditer(t):
        seg = t[m.end():m.end() + limit * 2]
        seg = seg.split("\n#")[0]
        seg = clean(strip_images(seg))
        if len(seg) > 120:
            # 截到句号边界
            seg = seg[:limit]
            cut = max(seg.rfind(". "), seg.rfind(".\n"))
            if cut > 300:
                seg = seg[:cut + 1]
            return seg
    return ""


def get_method(t, limit=1100):
    """抽取方法章节的核心段落"""
    m = re.search(r"\n#{1,3}\s*(?:[IVX]+\.?\s*)?(System Overview|Method|Methodology|"
                  r"Approach|Proposed\s+\w+|System\s+\w*)\s*\n", t, re.I)
    if not m:
        m = re.search(r"\n#{1,3}\s*([IVX]+\.\s*[A-Z][^\n]{3,60})\n", t)
    if not m:
        return ""
    seg = clean(strip_images(t[m.end():m.end() + limit * 2]))
    seg = re.split(r"\n#{1,3}\s", seg)[0]
    return seg[:limit]


def get_experiments(t, limit=700):
    """抓实验设置与结论段"""
    out = []
    for kw in ("Experiments", "Experimental Results", "Evaluation", "Results"):
        m = re.search(r"\n#{1,3}\s*[IVX]*\.?\s*%s[^\n]*\n" % kw, t, re.I)
        if m:
            seg = clean(strip_images(t[m.end():m.end() + 1600]))
            seg = re.split(r"\n#{1,3}\s", seg)[0]
            out.append(seg[:limit])
            break
    return "\n".join(out)


def get_limitations(t, limit=420):
    for kw in ("Limitation", "Conclusion", "Discussion and Conclusion"):
        m = re.search(r"\n#{1,3}\s*[IVX]*\.?\s*%s[^\n]*\n" % kw, t, re.I)
        if m:
            seg = clean(strip_images(t[m.end():m.end() + 900]))
            seg = re.split(r"\n#{1,3}\s", seg)[0]
            return seg[:limit]
    return ""


def tech_tags(t):
    low = t.lower()
    return [k for k, kws in TECHS.items() if any(w.lower() in low for w in kws)]


def datasets(t):
    seen, out = set(), []
    for m in DATASET_PAT.finditer(t):
        d = m.group(1)
        if d not in seen:
            seen.add(d)
            out.append(d)
    return out[:10]


def numbers(t):
    """抓实验部分的关键数字（百分比/误差/帧率）"""
    seg = t
    m = re.search(r"\n#{1,3}\s*[IVX]*\.?\s*(Experiments|Experimental Results|Evaluation)",
                  seg, re.I)
    if m:
        seg = seg[m.start():]
    pats = [r"\d+\.?\d*\s*%", r"\d+\.?\d*\s*(?:Hz|fps|FPS)", r"RMSE[^\n]{0,26}?\d+\.?\d*",
            r"ATE[^\n]{0,26}?\d+\.?\d*", r"\b\d+\.\d+\s*(?:m|cm|mm)\b"]
    res = []
    for p in pats:
        res += re.findall(p, seg)
    return res[:12]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(MD_DIR, "_digest.md"))
    ap.add_argument("--min-chars", type=int, default=3000)
    args = ap.parse_args()

    files = sorted(f for f in glob.glob(os.path.join(MD_DIR, "*.md"))
                   if not os.path.basename(f).startswith("_"))
    lines = ["# SLAM 语料库 · 技术内容索引", ""]
    lines.append("> 本索引为**抽取式**：所有文字均直接来自论文正文（摘要/引言贡献点/方法段/实验段），")
    lines.append("> 未做任何生成式改写。每条技术描述都可回溯到 `md/<论文>.md` 原文。")
    lines.append("")
    lines.append("共 %d 篇。" % len(files))
    lines.append("")

    stat = []
    for f in files:
        stem = os.path.basename(f)[:-3]
        t = load(f)
        if len(t) < args.min_chars:
            stat.append((stem, len(t), 0))
            continue
        title = get_title(t) or stem
        ab = get_abstract(t)
        secs = get_sections(t)
        contrib = get_contributions(t)
        meth = get_method(t)
        exp = get_experiments(t)
        lim = get_limitations(t)
        tags = tech_tags(t)
        ds = datasets(t)
        nums = numbers(t)

        lines.append("\n---\n")
        lines.append("## %s" % title)
        lines.append("- **文件** `md/%s.md` · 正文 %s 字符" % (stem, format(len(t), ",")))
        if tags:
            lines.append("- **技术标签** %s" % " / ".join(tags))
        if ds:
            lines.append("- **数据集** %s" % ", ".join(ds))
        if ab:
            lines.append("- **摘要（原文摘录）**\n  > %s" % ab[:1200].replace("\n", "\n  > "))
        if contrib:
            lines.append("- **作者声称的贡献点（原文摘录）**\n  > %s" % contrib.replace("\n", "\n  > "))
        if meth:
            lines.append("- **方法核心段（原文摘录）**\n  > %s" % meth.replace("\n", "\n  > "))
        if secs:
            lines.append("- **章节结构** %s" % " ｜ ".join(secs[:26]))
        if exp:
            lines.append("- **实验设置（原文摘录）**\n  > %s" % exp.replace("\n", "\n  > "))
        if nums:
            lines.append("- **出现的关键数值** %s" % ", ".join(nums[:10]))
        if lim:
            lines.append("- **结论/局限（原文摘录）**\n  > %s" % lim.replace("\n", "\n  > "))
        stat.append((stem, len(t), 1))

    open(args.out, "w", encoding="utf-8").write("\n".join(lines))
    ok = sum(1 for _, _, k in stat if k)
    print("digest -> %s  (%d/%d 篇, %s 字符)" % (args.out, ok, len(files), format(len("\n".join(lines)), ",")))
    skipped = [s for s, n, k in stat if not k]
    if skipped:
        print("跳过的过短文件 (%d): %s" % (len(skipped), ", ".join(skipped[:10])))


if __name__ == "__main__":
    main()
