#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""site_probe.py — 跨出版商 PDF 下载能力探针

回答一个问题：IEExplore 那套"浏览器请求头 + curl"流程，能不能复用到别的论文站？

做法：
  A. 用 OpenAlex 按标题解析出真实 DOI 与开放获取 PDF 地址（不靠猜）
  B. 对每个 PDF 地址用统一的浏览器请求头去下，报告 status / content-type / 是否 %PDF-
  C. 额外测一组各站已知的 PDF 直链模式，判断风控层级

用法: python site_probe.py
"""
import json
import os
import re
import subprocess
import sys
import time
import urllib.parse

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")

# 真实浏览器的完整请求头（IEEE 那套经验的通用化）
HDRS = [
    "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
    "-H", "Accept-Language: en-US,en;q=0.9",
    "-H", "Sec-Fetch-Dest: document",
    "-H", "Sec-Fetch-Mode: navigate",
    "-H", "Sec-Fetch-Site: none",
    "-H", "Sec-Fetch-User: ?1",
    "-H", "Upgrade-Insecure-Requests: 1",
]

TMP = r"G:/project/ieeexplore/meta/_probe"
os.makedirs(TMP, exist_ok=True)

# 覆盖主要出版商的代表性论文标题（跨学科，便于拿到各站真实地址）
TITLES = [
    "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM",
    "LSD-SLAM: Large-Scale Direct Monocular SLAM",
    "Human-level control through deep reinforcement learning",
    "Highly accurate protein structure prediction with AlphaFold",
    "ImageNet classification with deep convolutional neural networks",
    "NeRF: Representing Scenes as Neural Radiance Fields for View Synthesis",
    "PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation",
    "Simultaneous localization and mapping: part I",
    "Deep learning",
    "Adam: A Method for Stochastic Optimization",
    "Segment Anything",
    "DINOv2: Learning Robust Visual Features without Supervision",
    "LoFTR: Detector-Free Local Feature Matching with Transformers",
    "A survey on deep learning for robot vision",
    "Learning transferable visual models from natural language supervision",
]


def curl(args, timeout=120):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, text=True, timeout=timeout + 40)
        return r.stdout or ""
    except subprocess.TimeoutExpired:
        return ""


def openalex_lookup(title):
    q = urllib.parse.urlencode({"search": title, "per-page": "1",
                                "select": "id,doi,title,publication_year,"
                                          "primary_location,best_oa_location,open_access"})
    out = curl(["-H", "User-Agent: " + UA,
                "https://api.openalex.org/works?%s" % q], timeout=40)
    try:
        j = json.loads(out)
        if not j.get("results"):
            return None
        w = j["results"][0]
        pl = w.get("primary_location") or {}
        oa = w.get("best_oa_location") or {}
        src = (pl.get("source") or {}).get("display_name")
        return {
            "doi": (w.get("doi") or "").replace("https://doi.org/", ""),
            "matched": w.get("title"),
            "year": w.get("publication_year"),
            "publisher": src,
            "is_oa": (w.get("open_access") or {}).get("is_oa"),
            "oa_pdf": oa.get("pdf_url"),
            "oa_landing": oa.get("landing_page_url"),
            "pdf_url": pl.get("pdf_url"),
        }
    except Exception as e:
        return {"error": str(e)}


def probe_pdf(url, name):
    """用统一浏览器请求头尝试下载，返回结论"""
    if not url:
        return {"url": None, "verdict": "无地址"}
    f = os.path.join(TMP, name + ".bin")
    meta = curl(["-L", "--compressed", "-o", f,
                 "-w", "%{http_code}|%{content_type}|%{size_download}|%{url_effective}",
                 "-H", "User-Agent: " + UA] + HDRS + [url], timeout=120)
    parts = (meta or "").strip().split("|")
    code = parts[0] if parts else "?"
    ctype = parts[1] if len(parts) > 1 else "?"
    size = parts[2] if len(parts) > 2 else "0"
    magic = ""
    if os.path.exists(f):
        with open(f, "rb") as fh:
            magic = fh.read(5).decode("latin1", "replace")
    if magic == "%PDF-":
        verdict = "✅ 直下成功"
    elif code in ("403", "401"):
        verdict = "⛔ 风控拦截"
    elif code == "404":
        verdict = "⚠️ 地址不对（站点可达）"
    elif "html" in (ctype or "").lower():
        verdict = "⚠️ 返回网页（需先拿跳转/登录）"
    else:
        verdict = "❓ " + str(code)
    try:
        os.remove(f)
    except OSError:
        pass
    return {"url": url, "status": code, "ctype": ctype, "size": size,
            "magic": magic, "verdict": verdict}


# 各站已知的 PDF 直链模式（用于判断风控层级，不看内容只看是否放行）
PATTERNS = [
    ("IEEE Xplore", "https://ieeexplore.ieee.org/stampPDF/getPDF.jsp?tp=&arnumber=9440682&ref="),
    ("arXiv", "https://arxiv.org/pdf/1708.03852"),
    ("Springer Link", "https://link.springer.com/content/pdf/10.1007/978-3-319-10602-1_26.pdf"),
    ("Wiley", "https://onlinelibrary.wiley.com/doi/pdfdirect/10.1002/rob.21950"),
    ("ACM DL", "https://dl.acm.org/doi/pdf/10.1145/3355089.3356513"),
    ("Nature", "https://www.nature.com/articles/s41586-021-03819-2.pdf"),
    ("ScienceDirect", "https://www.sciencedirect.com/science/article/pii/S0921889022002069/pdfft"),
    ("MDPI", "https://www.mdpi.com/1424-8220/23/1/1/pdf"),
    ("Frontiers", "https://www.frontiersin.org/articles/10.3389/frobt.2021.689707/pdf"),
    ("PMC (PubMed Central)", "https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7141517/pdf/"),
    ("IOPscience", "https://iopscience.iop.org/article/10.1088/1742-6596/1237/2/022115/pdf"),
    ("Oxford Academic", "https://academic.oup.com/bioinformatics/article-pdf/36/4/1316/32858927/bty1051.pdf"),
    ("Cambridge Core", "https://www.cambridge.org/core/services/aop-cambridge-core/content/view/S0956796821000028"),
    ("Science.org (AAAS)", "https://www.science.org/doi/pdf/10.1126/science.abj8754"),
    ("Taylor & Francis", "https://www.tandfonline.com/doi/pdf/10.1080/01691864.2020.1855247"),
    ("SAGE", "https://journals.sagepub.com/doi/pdf/10.1177/0278364919878989"),
]


def main():
    print("=" * 108)
    print("Part A：按标题解析真实 DOI → 取开放获取 PDF → 用统一浏览器头直下")
    print("=" * 108)
    rows = []
    for i, t in enumerate(TITLES):
        info = openalex_lookup(t)
        if not info or info.get("error"):
            print("  [解析失败] %s" % t[:70])
            continue
        print("\n▸ %s" % t[:88])
        print("  出版商: %-34s DOI: %s  年份: %s" % (info.get("publisher"), info.get("doi"), info.get("year")))
        target = info.get("oa_pdf") or info.get("pdf_url")
        if target:
            r = probe_pdf(target, "oa_%d" % i)
            print("  OA PDF: %s" % (target[:96]))
            print("  → %s   [http=%s type=%s]" % (r["verdict"], r.get("status"), r.get("ctype")))
        else:
            r = None
            print("  无开放获取 PDF 地址（需订阅权限，看 Part B）")
        rows.append({"title": t, **{k: info.get(k) for k in ("publisher", "doi", "year", "is_oa")},
                     "probe": r})
        time.sleep(0.8)

    print("\n\n" + "=" * 108)
    print("Part B：各站已知 PDF 直链模式 —— 判断风控层级")
    print("=" * 108)
    print("%-24s %-8s %-34s %s" % ("站点", "HTTP", "Content-Type", "结论"))
    print("-" * 108)
    pb = []
    for i, (name, url) in enumerate(PATTERNS):
        r = probe_pdf(url, "pat_%d" % i)
        print("%-24s %-8s %-34s %s" % (name, r.get("status"), (r.get("ctype") or "")[:33], r["verdict"]))
        pb.append({"site": name, "url": url, **r})
        time.sleep(0.5)

    json.dump({"lookup": rows, "patterns": pb},
              open(r"G:/project/ieeexplore/meta/site_probe_result.json", "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("\n结果已写入 meta/site_probe_result.json")


if __name__ == "__main__":
    main()
