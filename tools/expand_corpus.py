#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""expand_corpus.py — 扩大语料库（两路并行）

A 路：复用第一次采集的 IEEE 候选池（1622 篇顶刊顶会），挑出尚未下载的高被引论文
B 路：用 Crossref 免费检索非 IEEE 渠道（Springer / ACM / Wiley / MDPI / Frontiers）

用法: python expand_corpus.py --ieee-top 70 --crossref-top 70
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.parse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matchutil import title_sim, sanitize_query  # noqa: E402

UA = "slam-corpus/1.0 (mailto:research@example.org)"
ROOT = r"G:/project/ieeexplore"
PAPERS = os.path.join(ROOT, "papers")
HARVEST = os.path.join(ROOT, "meta/ieee_harvest.json")

# 下载能力已确认的站点
PREFIX_SITE = {"10.1007": "springer", "10.1002": "wiley", "10.1145": "acm",
               "10.1038": "nature", "10.1088": "iop", "10.1126": "science",
               "10.3389": "frontiers", "10.3390": "mdpi"}
# 非 IEEE 渠道的检索主题（覆盖 SLAM 各子方向）
TOPICS = [
    "SLAM simultaneous localization and mapping",
    "visual inertial odometry",
    "LiDAR inertial odometry",
    "Gaussian splatting SLAM",
    "neural radiance field SLAM",
    "neural implicit SLAM",
    "dynamic SLAM",
    "semantic SLAM",
    "collaborative SLAM multi-robot",
    "event-based visual odometry",
    "loop closure detection",
    "place recognition",
    "3D Gaussian splatting",
    "point cloud registration",
    "visual odometry deep learning",
    "LiDAR place recognition",
    "dense visual SLAM RGB-D",
    "SLAM autonomous driving",
]

# ★ 领域相关性闸门
# Crossref 的 bibliographic 检索非常宽松：查 "SLAM ... mapping" 会返回
# 化学的 "Hirshfeld partition"、基因组的 "single-cell fate mapping" 等完全无关的论文。
# 必须先过关键词闸门，再进入候选池。
STRONG_KW = [
    "slam", "simultaneous localization", "odometry", "place recognition",
    "loop closure", "loop-closure", "radiance field", "gaussian splat",
    "point cloud registration", "visual-inertial", "visual inertial",
    "lidar", "neural implicit", "visual odometry", "metric-semantic",
    "scan matching", "relocalization", "re-localization", "structure from motion",
    "visual localization", "pose estimation", "depth estimation",
    "sensor fusion", "inertial navigation", "bundle adjustment", "pose graph",
    "neural radiance", "3d reconstruction", "mapping system", "localization system",
]
REJECT_KW = [
    "genom", "protein", "gene ", "genes", "cell", "tumor", "cancer", "chemistry",
    "chemical", "drug", "patient", "clinical", "soil", "land cover", "land use",
    "brain", "neuron", "neuro", "climate", "material", "molecul", "schizophren",
    "microenvironment", "chromatin", "rna", "dna", "microbiome", "ecolog",
    "agricultur", "crop", "forest", "ocean", "atmosphere", "glacier", "seismic",
    "galaxy", "stellar", "quantum", "polymer", "catalyst", "vaccine", "epidemi",
    "hospital", "surgery", "dental", "pharma", "cognitive science", "psycholog",
]


def is_relevant(title, venue=""):
    t = (title or "").lower()
    v = (venue or "").lower()
    if any(k in t for k in REJECT_KW):
        return False
    if any(k in t for k in STRONG_KW):
        return True
    # 标题没命中，但发表在明确的机器人/CV 期刊上也算
    return any(k in v for k in ("robotics", "robot", "computer vision", "automation",
                                "mechatronics", "field robotics", "remote sensing",
                                "intelligent vehicles", "unmanned"))


def log(m):
    sys.stdout.write("[expand] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=90):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def existing_titles():
    """已下载论文的标题集合（用于去重）"""
    titles = set()
    md = os.path.join(ROOT, "md")
    if os.path.isdir(md):
        for f in os.listdir(md):
            if f.endswith(".md") and not f.startswith("_"):
                try:
                    t = open(os.path.join(md, f), encoding="utf-8", errors="replace").read(400)
                    mm = re.match(r"\s*#+\s*(.+)", t)
                    if mm:
                        titles.add(re.sub(r"[^a-z0-9]+", "", mm.group(1).lower())[:70])
                except Exception:
                    pass
    # 已规划的 manifest 也算
    for mf in ("missing_manifest.json", "ieee_download_manifest.json", "classics_manifest.json",
               "expansion_manifest.json"):
        p = os.path.join(ROOT, "meta", mf)
        if os.path.exists(p):
            try:
                for x in json.load(open(p, encoding="utf-8")):
                    if x.get("title"):
                        titles.add(re.sub(r"[^a-z0-9]+", "", x["title"].lower())[:70])
            except Exception:
                pass
    return titles


def safe_key(year, title):
    s = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in title)[:56]
    return "%s_%s" % (year or "0000", s)


def part_a_ieee(top_n, have, man, seen_titles):
    """从第一次采集的 IEEE 候选池里补高被引论文"""
    if not os.path.exists(HARVEST):
        log("A路: 未找到 ieee_harvest.json，跳过")
        return 0
    raw = open(HARVEST, encoding="utf-8").read()
    d = json.loads(raw[raw.find("{"):])
    recs = sorted(d["records"], key=lambda x: -(x.get("cites") or 0))
    added = 0
    for r in recs:
        if added >= top_n:
            break
        key_norm = re.sub(r"[^a-z0-9]+", "", r["title"].lower())[:70]
        if key_norm in seen_titles:
            continue
        aid = r["id"]
        if any(x.get("id") == aid for x in man):
            continue
        key = safe_key(r.get("year"), r["title"])
        if key in have:
            continue
        man.append({"key": key, "title": r["title"], "site": "ieee", "id": aid,
                    "cites": r.get("cites"), "venue": r.get("venue"),
                    "out": "%s/%s.pdf" % (PAPERS, key), "src": "ieee_harvest"})
        have.add(key)
        seen_titles.add(key_norm)
        added += 1
    log("A路(IEEE 候选池): 新增 %d 篇" % added)
    return added


def crossref_topic(topic, rows=100, year_from=2022):
    q = urllib.parse.urlencode({
        "query.bibliographic": sanitize_query(topic), "rows": str(rows),
        "filter": "from-pub-date:%d-01-01,type:journal-article" % year_from,
        "sort": "is-referenced-by-count", "order": "desc",
        "select": "DOI,title,issued,is-referenced-by-count,container-title,publisher",
    })
    out = curl(["-H", "User-Agent: " + UA, "https://api.crossref.org/works?%s" % q], timeout=90)
    try:
        return ((json.loads(out).get("message") or {}).get("items")) or []
    except Exception:
        return []


def part_b_crossref(top_n, have, man, seen_titles, min_cites=8):
    """用 Crossref 挖非 IEEE 渠道"""
    added = 0
    stats = []
    for topic in TOPICS:
        if added >= top_n:
            break
        items = crossref_topic(topic)
        got = 0
        for w in items:
            if added >= top_n:
                break
            doi = (w.get("DOI") or "").lower()
            site = PREFIX_SITE.get(doi.split("/")[0])
            if not site:
                continue
            cites = w.get("is-referenced-by-count") or 0
            if cites < min_cites:
                continue
            t = (w.get("title") or [""])[0]
            if not t:
                continue
            venue = (w.get("container-title") or [""])[0]
            if not is_relevant(t, venue):
                continue
            kn = re.sub(r"[^a-z0-9]+", "", t.lower())[:70]
            if kn in seen_titles:
                continue
            year = None
            try:
                year = (w.get("issued") or {}).get("date-parts", [[None]])[0][0]
            except Exception:
                pass
            key = safe_key(year, t)
            if key in have:
                continue
            man.append({"key": key, "title": t, "site": site, "id": doi, "doi": doi,
                        "cites": cites, "venue": (w.get("container-title") or [""])[0][:100],
                        "out": "%s/%s.pdf" % (PAPERS, key), "src": "crossref"})
            have.add(key)
            seen_titles.add(kn)
            added += 1
            got += 1
        stats.append((topic, len(items), got))
        log("B路 %-42s 返回=%-4d 采纳=%d" % (topic[:42], len(items), got))
        time.sleep(0.8)
    log("B路(Crossref 非 IEEE): 新增 %d 篇" % added)
    return added


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ieee-top", type=int, default=70)
    ap.add_argument("--crossref-top", type=int, default=70)
    ap.add_argument("--min-cites", type=int, default=8)
    args = ap.parse_args()

    seen_titles = existing_titles()
    log("已有标题 %d 条（用于去重）" % len(seen_titles))
    man, have = [], set()

    a = part_a_ieee(args.ieee_top, have, man, seen_titles)
    b = part_b_crossref(args.crossref_top, have, man, seen_titles, args.min_cites)

    out = os.path.join(ROOT, "meta/expansion_manifest.json")
    json.dump(man, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    log("\n共新增 %d 篇 -> %s" % (len(man), out))
    by = {}
    for x in man:
        by[x["site"]] = by.get(x["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))
    log("高被引 Top10:")
    for x in sorted(man, key=lambda y: -(y.get("cites") or 0))[:10]:
        log("   %5s cites | %-9s %s" % (x.get("cites"), x["site"], x["title"][:70]))


if __name__ == "__main__":
    main()
