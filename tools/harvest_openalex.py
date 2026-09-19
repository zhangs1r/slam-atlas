#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""harvest_openalex.py — 用 OpenAlex 做跨出版商论文发现

IEEE 之外的渠道（Springer / ACM / Wiley / Nature / Frontiers / MDPI / arXiv）
没有统一的检索接口，OpenAlex 是唯一能一次性覆盖全部出版商、
且自带引用数与 DOI 的免费数据源。

策略：多关键词 × 按被引排序 × 限定年份 → 按出版商分组 → 输出下载 manifest

用法: python harvest_openalex.py <out_manifest.json> [--from 2022] [--min-cites 20]
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.parse

UA = "slam-corpus/1.0 (mailto:research@example.org)"
PAPERS = r"G:/project/ieeexplore/papers"

QUERIES = [
    "SLAM", "visual SLAM", "visual-inertial odometry", "LiDAR SLAM",
    "LiDAR-inertial odometry", "Gaussian splatting SLAM", "radiance field SLAM",
    "neural implicit SLAM", "dynamic SLAM", "semantic SLAM",
    "collaborative SLAM", "multi-robot SLAM", "event-based SLAM",
    "loop closure detection", "place recognition", "visual odometry",
    "3D Gaussian splatting", "point cloud registration", "neural radiance field",
    "simultaneous localization and mapping", "SLAM foundation model",
    "deep feature matching", "LiDAR place recognition", "dense visual SLAM",
    "RGB-D SLAM", "monocular depth SLAM", "multi-sensor fusion odometry",
    "robot localization mapping", "pose graph optimization", "bundle adjustment",
]

# 可下载的出版商（DOI 前缀）→ 站点
PREFIX_SITE = {
    "10.1109": "ieee", "10.48550": "arxiv", "10.1007": "springer", "10.1038": "nature",
    "10.1002": "wiley", "10.1145": "acm", "10.1088": "iop", "10.1126": "science",
    "10.3389": "frontiers", "10.3390": "mdpi", "10.1080": "tandf", "10.1016": "elsevier",
    "10.1093": "oxford", "10.1177": "sage",
}

# 只有这些站点我们确认能下（其余记录下来但不下载）
DOWNLOADABLE = {"ieee", "arxiv", "springer", "wiley", "acm", "nature", "iop", "science", "frontiers", "mdpi"}


def log(m):
    sys.stdout.write("[oa] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=90):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def fetch(query, year_from, per_page=100):
    params = {
        "search": query,
        "filter": "from_publication_date:%s-01-01,type:article" % year_from,
        "sort": "cited_by_count:desc",
        "per-page": str(per_page),
        "select": "doi,title,publication_year,cited_by_count,primary_location,type,referenced_works_count",
    }
    out = curl(["-H", "User-Agent: " + UA,
                "https://api.openalex.org/works?%s" % urllib.parse.urlencode(params)], timeout=90)
    try:
        j = json.loads(out)
        return j.get("results") or [], (j.get("meta") or {}).get("count")
    except Exception:
        return [], None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--from", dest="year_from", type=int, default=2022)
    ap.add_argument("--min-cites", type=int, default=15)
    ap.add_argument("--per-query", type=int, default=100)
    args = ap.parse_args()

    seen, records, stats = set(), {}, []
    for i, q in enumerate(QUERIES, 1):
        res, total = fetch(q, args.year_from, args.per_query)
        kept = 0
        for w in res:
            doi = (w.get("doi") or "").replace("https://doi.org/", "").lower()
            if not doi or doi in seen:
                continue
            cites = w.get("cited_by_count") or 0
            if cites < args.min_cites:
                continue
            pfx = doi.split("/")[0]
            site = PREFIX_SITE.get(pfx)
            if not site:
                continue
            pl = w.get("primary_location") or {}
            venue = ((pl.get("source") or {}).get("display_name")) or ""
            seen.add(doi)
            records[doi] = {
                "doi": doi, "site": site, "id": doi,
                "title": (w.get("title") or "")[:200],
                "year": w.get("publication_year"), "cites": cites,
                "venue": venue[:110],
                "downloadable": site in DOWNLOADABLE,
            }
            kept += 1
        stats.append({"query": q, "oa_total": total, "kept": kept})
        log("%-42s OA命中=%-6s 采纳=%d" % (q, total, kept))
        time.sleep(0.4)

    # 按站点统计
    by_site = {}
    for r in records.values():
        by_site[r["site"]] = by_site.get(r["site"], 0) + 1
    log("\n去重后总计 %d 篇，渠道分布: %s" % (len(records), json.dumps(by_site, ensure_ascii=False)))

    # 生成 manifest（只含有下载能力的站点）
    items = sorted((r for r in records.values() if r["downloadable"]),
                   key=lambda x: -x["cites"])
    manifest = []
    for r in items:
        yr = r["year"] or 0
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in r["title"])[:58]
        key = "%s_%s" % (yr, safe)
        manifest.append({"key": key, "site": r["site"], "id": r["id"], "doi": r["doi"],
                         "title": r["title"], "venue": r["venue"], "cites": r["cites"],
                         "out": "%s/%s.pdf" % (PAPERS, key)})

    json.dump(manifest, open(args.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    json.dump({"stats": stats, "all": list(records.values())},
              open(args.out.replace(".json", "_raw.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    log("manifest -> %s (%d 篇可下载)" % (args.out, len(manifest)))
    for s in sorted(by_site):
        log("   %-10s %3d 篇%s" % (s, by_site[s], "" if s in DOWNLOADABLE else "  (无下载能力，仅记录)"))


if __name__ == "__main__":
    main()
