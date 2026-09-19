#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""make_digest.py — 从 MinerU 产出的 Markdown 生成紧凑阅读索引

每篇抽取：标题、摘要、章节结构、方法关键词、关键实验数字。
输出 md/_digest.md，作为撰写综述时的高效阅读材料。
"""
import glob
import os
import re
import sys

MD_DIR = r"G:/project/ieeexplore/md"
OUT = os.path.join(MD_DIR, "_digest.md")

ORIG = [
    ("激光 / 激光惯性 SLAM", ["2022_FAST-LIO2", "2023_KISS-ICP", "2022_Faster-LIO", "2022_FAST-LIVO",
                            "2025_FAST-LIVO2", "2024_PIN-SLAM", "2024_iG-LIO", "2022_DLO",
                            "2022_DiSCo-SLAM", "2020_LIO-SAM", "2018_LeGO-LOAM", "2018_ScanContext"]),
    ("视觉 / 视觉惯性 SLAM", ["2022_GVINS", "2022_DM-VIO", "2023_Dynam-SLAM", "2023_ESVIO", "2022_DEVO",
                            "2018_UltimateSLAM", "2017_ORB-SLAM2", "2018_VINS-Mono", "2018_DSO",
                            "2020_OpenVINS", "2020_Kimera"]),
    ("NeRF / 隐式表示 SLAM", ["2022_NICE-SLAM", "2023_Co-SLAM", "2023_ESLAM", "2023_GO-SLAM",
                            "2024_Loopy-SLAM", "2023_NeRF-SLAM", "2021_iMAP", "2023_Point-SLAM",
                            "2022_Vox-Fusion", "2023_NeRF-LOAM"]),
    ("3D Gaussian Splatting SLAM", ["2024_MonoGS", "2024_SplaTAM", "2024_GS-SLAM", "2024_Photo-SLAM",
                                    "2024_LIV-GaussMap", "2025_WildGS-SLAM", "2025_Splat-SLAM",
                                    "2024_SNI-SLAM"]),
    ("学习式 / 场景识别 / 特征匹配", ["2022_LCDNet", "2022_OverlapTransformer", "2024_DeepLoopClosing",
                                      "2020_SuperGlue", "2023_LightGlue"]),
    ("多机协同 / 语义 / 动态环境", ["2022_Kimera-Multi", "2024_Swarm-SLAM", "2022_LAMP2", "2024_D2SLAM",
                                    "2020_DOOR-SLAM", "2018_DynaSLAM"]),
    ("综述 / 评测 / 极端环境", ["2025_3DGS-Survey", "2024_DL-VisualLoc-Survey", "2024_DynamicSLAM-Survey",
                                "2022_LoopClosure-Survey", "2024_DARPA-SubT", "2023_Hilti-Oxford"]),
]

KEYWORDS = ["ORB-SLAM", "VINS", "LIO-SAM", "FAST-LIO", "FAST-LIVO", "LOAM", "DROID-SLAM", "SplaTAM",
            "Gaussian Splatting", "NeRF", "NICE-SLAM", "ESLAM", "Co-SLAM", "Point-SLAM", "GO-SLAM",
            "iMAP", "Vox-Fusion", "KISS-ICP", "Photo-SLAM", "MonoGS", "Scan Context", "Kimera",
            "Swarm-SLAM", "SuperGlue", "LightGlue", "DynaSLAM", "DROID", "NeRF-LOAM", "PIN-SLAM",
            "GVINS", "DM-VIO", "OpenVINS", "DSO", "LSD-SLAM", "SVO", "R3LIVE", "Point-LIO",
            "GLIM", "Wildcat", "MASt3R", "DUSt3R", "SAM", "CLIP", "DINO", "LoFTR", "RAFT",
            "ikd-Tree", "IMU", "loop closure", "bundle adjustment", "factor graph",
            "sliding window", "pose graph", "voxel hashing", "TSDF", "occupancy"]


def clean(t):
    t = re.sub(r"\n{3,}", "\n\n", t)
    return t.strip()


def get_abstract(text):
    """取摘要：优先 Abstract 段，否则取开头"""
    m = re.search(r"#+\s*Abstract\s*\n+(.{80,2600}?)(?=\n#+\s|\n\*\*Index Terms|\Z)", text, re.S | re.I)
    if m:
        return clean(m.group(1))
    # 去掉标题行后取前 1800 字符
    body = re.sub(r"^#.*?\n", "", text, count=1)
    return clean(body[:1800])


def get_sections(text):
    heads = re.findall(r"^(#{1,3})\s+(.+)$", text, re.M)
    out = []
    for lvl, h in heads:
        h = h.strip()
        if len(h) > 90 or h.lower() in ("references", "acknowledgment", "acknowledgements"):
            continue
        if re.match(r"^(references|appendix)", h, re.I):
            continue
        out.append(h)
    return out


def get_keynums(text):
    """抓实验部分出现的关键量化结果"""
    nums = re.findall(r"(\d+\.?\d*)\s*%", text)
    return nums[:12]


def main():
    files = sorted(glob.glob(os.path.join(MD_DIR, "*.md")))
    files = [f for f in files if not os.path.basename(f).startswith("_")]
    by_name = {os.path.basename(f)[:-3]: f for f in files}

    lines = ["# SLAM 论文语料库 · 阅读索引", ""]
    lines.append("共 %d 篇。每篇给出标题 / 摘要 / 章节结构 / 高频技术关键词。" % len(files))
    lines.append("")

    seen = set()
    for group, names in ORIG:
        lines.append("\n---\n\n## %s\n" % group)
        for n in names:
            f = by_name.get(n)
            seen.add(n)
            if not f:
                lines.append("- (缺) %s" % n)
                continue
            text = open(f, encoding="utf-8", errors="replace").read()
            title = n
            m = re.match(r"\s*#\s+(.+)", text)
            if m:
                title = m.group(1).strip()
            ab = get_abstract(text)
            secs = get_sections(text)
            kw = [k for k in KEYWORDS if re.search(re.escape(k), text, re.I)]
            nums = get_keynums(text)
            lines.append("\n### %s" % title)
            lines.append("- **文件**: `%s.md` | 字符数 %d" % (n, len(text)))
            lines.append("- **摘要**: %s" % ab[:1400])
            if secs:
                lines.append("- **章节**: %s" % " | ".join(secs[:22]))
            if kw:
                lines.append("- **涉及技术/基线**: %s" % ", ".join(kw[:24]))
            if nums:
                lines.append("- **出现的关键百分比数字**: %s" % ", ".join(nums[:10]))

    rest = [k for k in by_name if k not in seen]
    if rest:
        lines.append("\n---\n\n## 其他\n")
        for n in sorted(rest):
            lines.append("- `%s.md`" % n)

    open(OUT, "w", encoding="utf-8").write("\n".join(lines))
    print("digest -> %s  (%d papers, %d chars)" % (OUT, len(files), len("\n".join(lines))))


if __name__ == "__main__":
    main()
