#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""resolve_arxiv.py — 按标题在 arXiv 上解析论文，拿到 arXiv ID / 摘要 / 年份

用法:
  python resolve_arxiv.py <titles.txt> <out.json>
  (titles.txt 每行一个标题；以 # 开头的行为注释)
"""
import json
import os
import re
import sys
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

API = "https://export.arxiv.org/api/query"
NS = {"a": "http://www.w3.org/2005/Atom"}


def norm(s):
    s = (s or "").lower()
    s = re.sub(r"[^a-z0-9]+", " ", s)
    return " ".join(s.split())


def similarity(a, b):
    """简单的词级 Jaccard + 前缀重合度"""
    wa, wb = set(norm(a).split()), set(norm(b).split())
    if not wa or not wb:
        return 0.0
    jac = len(wa & wb) / len(wa | wb)
    na, nb = norm(a), norm(b)
    pre = 0
    for x, y in zip(na, nb):
        if x != y:
            break
        pre += 1
    pre_score = pre / max(len(na), len(nb), 1)
    return 0.65 * jac + 0.35 * pre_score


def query(title, max_results=5):
    q = 'ti:"%s"' % re.sub(r'"', "", title)
    url = "%s?%s" % (API, urllib.parse.urlencode({
        "search_query": q, "start": 0, "max_results": max_results,
        "sortBy": "relevance", "sortOrder": "descending",
    }))
    for attempt in range(3):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "slam-survey/1.0"})
            with urllib.request.urlopen(req, timeout=60) as r:
                raw = r.read()
            break
        except Exception as e:  # noqa
            if attempt == 2:
                raise
            time.sleep(3 * (attempt + 1))
    root = ET.fromstring(raw)
    out = []
    for e in root.findall("a:entry", NS):
        aid = (e.findtext("a:id", "", NS) or "").strip()
        m = re.search(r"abs/(.+)$", aid)
        arxid = m.group(1) if m else aid
        ver = re.search(r"v(\d+)$", arxid)
        base_id = re.sub(r"v\d+$", "", arxid)
        out.append({
            "arxivId": base_id,
            "version": ver.group(1) if ver else None,
            "title": " ".join((e.findtext("a:title", "", NS) or "").split()),
            "summary": " ".join((e.findtext("a:summary", "", NS) or "").split()),
            "published": e.findtext("a:published", "", NS),
            "updated": e.findtext("a:updated", "", NS),
            "authors": [a.findtext("a:name", "", NS) for a in e.findall("a:author", NS)],
            "primary": (e.find("{http://arxiv.org/schemas/atom}primary_category") is not None
                        and e.find("{http://arxiv.org/schemas/atom}primary_category").get("term") or None),
            "comment": e.findtext("{http://arxiv.org/schemas/atom}comment", "", NS),
        })
    return out


def main():
    src, dst = sys.argv[1], sys.argv[2]
    titles = []
    for line in open(src, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        titles.append(line)

    out = {}
    existed = {}
    if os.path.exists(dst):
        try:
            existed = json.load(open(dst, encoding="utf-8"))
        except Exception:
            existed = {}

    for t in titles:
        if t in existed and existed[t].get("best"):
            out[t] = existed[t]
            print("[skip] %s" % t[:70])
            continue
        try:
            cands = query(t)
        except Exception as e:  # noqa
            print("[ERR] %s -> %s" % (t[:60], e))
            out[t] = {"query": t, "best": None, "error": str(e), "candidates": []}
            continue
        scored = sorted(((similarity(t, c["title"]), c) for c in cands), key=lambda x: -x[0])
        best = None
        if scored and scored[0][0] >= 0.45:
            best = dict(scored[0][1])
            best["score"] = round(scored[0][0], 3)
        out[t] = {"query": t, "best": best,
                  "candidates": [{"score": round(s, 3), "id": c["arxivId"], "title": c["title"],
                                  "published": c["published"][:10]} for s, c in scored]}
        tag = ("%s  %s  (%.2f)" % (best["arxivId"], best["published"][:10], best["score"])) if best else "NO MATCH"
        print("[%s] %s\n        -> %s" % ("ok" if best else "--", t[:78], tag))
        time.sleep(3.2)  # arXiv API 礼貌间隔

    json.dump(out, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ok = sum(1 for v in out.values() if v.get("best"))
    print("\nresolved %d/%d -> %s" % (ok, len(out), dst))


if __name__ == "__main__":
    main()
