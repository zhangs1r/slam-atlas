#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""citation_resolve.py — 从引用缓存重算并解析"缺失经典"

复用 citation_analysis.py 已抓好的 _citation_cache.json，
避免重复请求；并修正 Crossref 元数据解析（同样不能带 select 参数）。

用法: python citation_resolve.py [--top 80] [--min-freq 4]
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

UA = "slam-corpus/1.0 (mailto:research@example.org)"
ROOT = r"G:/project/ieeexplore"
CACHE = os.path.join(ROOT, "meta/_citation_cache.json")

# 领域相关性闸门（同 expand_corpus.py 的思路）
STRONG = ["slam", "simultaneous localization", "odometry", "place recognition",
          "loop closure", "loop-closure", "radiance field", "gaussian splat",
          "point cloud", "visual-inertial", "visual inertial", "lidar", "laser",
          "neural implicit", "visual odometry", "metric-semantic", "scan match",
          "relocalization", "re-localization", "structure from motion", "sfm",
          "visual localization", "pose graph", "bundle adjustment", "inertial",
          "sensor fusion", "occupancy", "voxel", "neural radiance", "3d reconstruction",
          "monocular", "stereo", "rgb-d", "depth estimation", "feature match",
          "keyframe", "mapping", "localization", "robot", "navigation"]
REJECT = ["genom", "protein", "gene", "tumor", "chem", "drug", "patient", "clinical",
          "neuro", "brain", "cell ", "chromatin", "rna", "dna", "ecolog", "agricultur",
          "climate", "galaxy", "quantum", "polymer", "vaccine", "epidemi", "surgery"]


def log(m):
    sys.stdout.write("[cres] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=45):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def collect_ours():
    dois = set()
    for mf in glob.glob(os.path.join(ROOT, "meta/*.json")):
        b = os.path.basename(mf)
        if b.startswith("_") or any(k in b for k in ("result","raw","bad","citation")):
            continue
        try:
            d = json.load(open(mf, encoding="utf-8"))
        except Exception:
            continue
        if isinstance(d, list):
            for x in d:
                if isinstance(x, dict) and x.get("doi"):
                    dois.add(x["doi"].lower())
        elif isinstance(d, dict) and "records" in d:
            for r in d["records"]:
                if r.get("doi"):
                    dois.add(r["doi"].lower())
    return dois


def meta_of(doi):
    out = curl(["-H", "User-Agent: " + UA, "https://api.crossref.org/works/%s" % doi], timeout=45)
    try:
        m = json.loads(out).get("message") or {}
        return {
            "title": (m.get("title") or [""])[0],
            "year": (m.get("issued") or {}).get("date-parts", [[None]])[0][0],
            "venue": (m.get("container-title") or [""])[0],
            "publisher": m.get("publisher"),
            "type": m.get("type"),
        }
    except Exception:
        return {}


def relevant(title, venue=""):
    t = (title or "").lower()
    if not t:
        return False
    if any(k in t for k in REJECT):
        return False
    if any(k in t for k in STRONG):
        return True
    return any(k in (venue or "").lower() for k in
               ("robotics", "robot", "computer vision", "automation", "mechatronics",
                "intelligent vehicle", "remote sensing", "field robotics"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--top", type=int, default=80)
    ap.add_argument("--min-freq", type=int, default=4)
    args = ap.parse_args()

    ours = collect_ours()
    cache = json.load(open(CACHE, encoding="utf-8"))
    freq = {}
    for src, refs in cache.items():
        for r in set(refs):
            if not r or r in ours or r.startswith("10.48550"):
                continue
            freq[r] = freq.get(r, 0) + 1
    ranked = sorted(((c, d) for d, c in freq.items() if c >= args.min_freq), reverse=True)
    log("候选缺失经典 %d 个（频次>=%d），开始解析元数据…" % (len(ranked), args.min_freq))

    out = []
    for c, doi in ranked[:args.top * 2]:
        if len(out) >= args.top:
            break
        m = meta_of(doi)
        if not relevant(m.get("title"), m.get("venue")):
            continue
        out.append({"doi": doi, "freq": c, **m})
        time.sleep(0.25)

    dst = os.path.join(ROOT, "meta/citation_missing.json")
    json.dump(out, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    log("\n=== 被本语料反复引用、但不在库中的论文 Top %d ===" % len(out))
    for i, x in enumerate(out, 1):
        log("%3d. [%2d次] %s | %s | %s" % (i, x["freq"], x.get("year"),
                                           (x.get("title") or "")[:76],
                                           (x.get("venue") or "")[:36]))
    log("\n-> %s" % dst)


if __name__ == "__main__":
    main()
