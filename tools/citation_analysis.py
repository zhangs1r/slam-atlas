#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""citation_analysis.py — 用引用网络反查缺失的经典论文

思路（不依赖任何人工记忆）：
  ① 取出我们已有论文的全部 DOI
  ② 从 Crossref 逐篇拉取它们的参考文献列表（message.reference[].DOI）
  ③ 统计每个被引 DOI 在我们语料里出现的次数 —— 出现越多，
     说明它被越多领域论文当作基础工作，也就越是"经典"
  ④ 排除已经在库里的，剩下的按频次排序 = 客观意义上的"缺失经典"

用法: python citation_analysis.py [--limit 200] [--top 60]
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


def log(m):
    sys.stdout.write("[cite] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=60):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def collect_dois():
    """从所有 manifest + IEEE 采集结果里收集已知 DOI"""
    dois = {}
    for mf in glob.glob(os.path.join(ROOT, "meta/*.json")):
        base = os.path.basename(mf)
        if base.startswith("_") or "result" in base or "raw" in base:
            continue
        try:
            d = json.load(open(mf, encoding="utf-8"))
        except Exception:
            continue
        if isinstance(d, list):
            for x in d:
                if isinstance(x, dict) and x.get("doi"):
                    dois[x["doi"].lower()] = x.get("title")
        elif isinstance(d, dict) and "records" in d:
            for r in d["records"]:
                if r.get("doi"):
                    dois[r["doi"].lower()] = r.get("title")
    return dois


def fetch_refs(doi):
    # 注意：不能加 select=reference —— Crossref 的 select 不接受该字段，
    # 一旦指定就会返回空引用列表（踩过一次坑）。
    out = curl(["-H", "User-Agent: " + UA,
                "https://api.crossref.org/works/%s" % doi], timeout=60)
    try:
        msg = json.loads(out).get("message") or {}
        return msg.get("reference") or []
    except Exception:
        return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=250, help="最多查询多少篇我们的论文")
    ap.add_argument("--top", type=int, default=60, help="输出多少个缺失经典候选")
    ap.add_argument("--min-freq", type=int, default=3, help="被引频次下限")
    args = ap.parse_args()

    ours = collect_dois()
    log("已收集 DOI %d 个" % len(ours))
    cache = {}
    if os.path.exists(CACHE):
        try:
            cache = json.load(open(CACHE, encoding="utf-8"))
        except Exception:
            cache = {}

    # ① 拉取参考文献
    targets = [d for d in ours if not d.startswith("10.48550")][:args.limit]
    log("将查询 %d 篇的参考文献（Crossref）" % len(targets))
    for i, doi in enumerate(targets, 1):
        if doi in cache:
            continue
        refs = fetch_refs(doi)
        cache[doi] = [r.get("DOI", "").lower() for r in refs if r.get("DOI")]
        if i % 25 == 0:
            json.dump(cache, open(CACHE, "w", encoding="utf-8"))
            log("  进度 %d/%d，累计引用记录 %d" % (i, len(targets),
                                                sum(len(v) for v in cache.values())))
        time.sleep(0.25)
    json.dump(cache, open(CACHE, "w", encoding="utf-8"))

    # ② 统计频次
    freq = {}
    for src, refs in cache.items():
        for r in set(refs):
            if not r or r in ours:
                continue
            freq[r] = freq.get(r, 0) + 1

    ranked = sorted(((c, d) for d, c in freq.items() if c >= args.min_freq), reverse=True)
    log("共 %d 个被引 DOI（频次>=%d 的有 %d 个）" % (len(freq), args.min_freq, len(ranked)))

    # ③ 解析高价值目标的标题
    log("解析 Top %d 的元数据…" % args.top)
    out = []
    for c, doi in ranked[:args.top]:
        title, year, venue = "", None, ""
        o = curl(["-H", "User-Agent: " + UA,
                  "https://api.crossref.org/works/%s?select=title,issued,container-title" % doi],
                 timeout=45)
        try:
            msg = json.loads(o).get("message") or {}
            title = (msg.get("title") or [""])[0]
            year = (msg.get("issued") or {}).get("date-parts", [[None]])[0][0]
            venue = (msg.get("container-title") or [""])[0]
        except Exception:
            pass
        out.append({"doi": doi, "freq": c, "title": title, "year": year, "venue": venue})
        time.sleep(0.3)

    dst = os.path.join(ROOT, "meta/citation_missing.json")
    json.dump(out, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    log("\n=== 被本语料反复引用、但不在库中的论文（Top %d）===" % len(out))
    for i, x in enumerate(out, 1):
        log("%3d. [%2d次] %s  %s | %s" % (i, x["freq"], x["year"], x["title"][:78], x["venue"][:40]))
    log("\n-> %s" % dst)


if __name__ == "__main__":
    main()
