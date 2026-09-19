#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""resolve_leftover.py — 把仍未解析的标题通过 OpenAlex 补进 manifest

OpenAlex 能同时给出 arXiv 预印本与其他出版商版本，比 arXiv 官方 API 稳得多。

用法: python resolve_leftover.py <manifest.json> <unresolved.json>
"""
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
PAPERS = r"G:/project/ieeexplore/papers"
PREFIX_SITE = {"10.1109": "ieee", "10.48550": "arxiv", "10.1007": "springer",
               "10.1038": "nature", "10.1002": "wiley", "10.1145": "acm",
               "10.1088": "iop", "10.1126": "science", "10.3389": "frontiers",
               "10.3390": "mdpi", "10.1080": "tandf", "10.1016": "elsevier",
               "10.1093": "oxford", "10.1177": "sage"}


def log(m):
    sys.stdout.write("[left] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=60):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def norm(s):
    return " ".join(re.sub(r"[^a-z0-9]+", " ", (s or "").lower()).split())


def sim(a, b):
    return title_sim(a, b)


def candidates(title, per_page=15):
    q = urllib.parse.urlencode({"search": sanitize_query(title), "per-page": str(per_page),
                                "select": "doi,title,publication_year,cited_by_count,primary_location"})
    out = curl(["-H", "User-Agent: " + UA,
                "https://api.openalex.org/works?%s" % q], timeout=60)
    try:
        res = json.loads(out).get("results") or []
    except Exception:
        return []
    scored = []
    for w in res:
        doi = (w.get("doi") or "").replace("https://doi.org/", "").lower()
        if not doi:
            continue
        s = title_sim(title, w.get("title", ""))
        if s < 0.62:
            continue
        site = PREFIX_SITE.get(doi.split("/")[0])
        if not site:
            continue
        aid = doi.split("arxiv.")[-1] if doi.startswith("10.48550/arxiv.") else doi
        scored.append({"site": site, "id": aid, "doi": doi, "score": round(s, 3),
                       "cites": w.get("cited_by_count") or 0,
                       "year": w.get("publication_year"),
                       "venue": ((w.get("primary_location") or {}).get("source") or {}).get("display_name")})
    # arXiv 优先（最易下载），其次引用数高者
    scored.sort(key=lambda x: (x["site"] != "arxiv", -x["cites"], -x["score"]))
    return scored


def main():
    man_path, unres_path = sys.argv[1], sys.argv[2]
    man = json.load(open(man_path, encoding="utf-8"))
    have = {x["key"] for x in man}
    try:
        unresolved = json.load(open(unres_path, encoding="utf-8"))
    except Exception:
        unresolved = []
    log("待补 %d 篇" % len(unresolved))

    added = 0
    for x in unresolved:
        if x["key"] in have:
            continue
        cands = candidates(x["title"])
        if not cands:
            log("   FAIL %-30s 无可用版本" % x["key"])
            continue
        c = cands[0]
        man.append({"key": x["key"], "title": x["title"], "site": c["site"], "id": c["id"],
                    "doi": c["doi"], "score": c["score"], "cites": c["cites"],
                    "venue": c.get("venue"),
                    "out": "%s/%s.pdf" % (PAPERS, x["key"])})
        have.add(x["key"])
        added += 1
        log("   OK   %-30s -> %-8s %s (%.2f, %s cites)" % (
            x["key"], c["site"], c["id"], c["score"], c["cites"]))
        time.sleep(0.5)

    json.dump(man, open(man_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ok = [x for x in man if not x.get("broken")]
    log("\n新增 %d 篇；manifest 现共 %d 篇可下载" % (added, len(ok)))
    by = {}
    for x in ok:
        by[x["site"]] = by.get(x["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))


if __name__ == "__main__":
    main()
