#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""resolve_crossref.py — 用 Crossref 补齐未解析论文（免费、无额度限制）

为什么用 Crossref 而不是 OpenAlex：
  OpenAlex 已改为按请求计费（$0.001/次，每天 $1 免费额度），批量解析很容易打爆，
  且额度耗尽时返回的是错误 JSON，会被误读成"没有匹配"。
  Crossref 完全免费，覆盖全部主流出版商，polite pool 用 mailto 即可。

用法: python resolve_crossref.py <manifest.json> <unresolved.json>
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
PREFIX_SITE = {"10.1109": "ieee", "10.1007": "springer", "10.1038": "nature",
               "10.1002": "wiley", "10.1145": "acm", "10.1088": "iop",
               "10.1126": "science", "10.3389": "frontiers", "10.3390": "mdpi",
               "10.1080": "tandf", "10.1016": "elsevier", "10.1093": "oxford",
               "10.1177": "sage"}
DOWNLOADABLE = {"ieee", "arxiv", "springer", "wiley", "acm", "nature", "iop",
                "science", "frontiers", "mdpi"}


def log(m):
    sys.stdout.write("[cr] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=60):
    """注意：本机管道会触发 SIGTERM，所有输出必须落文件，不能 | 给别的进程"""
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def crossref(title, rows=5):
    q = urllib.parse.urlencode({"query.bibliographic": sanitize_query(title),
                               "rows": str(rows),
                               "select": "DOI,title,issued,is-referenced-by-count,publisher,container-title"})
    out = curl(["-H", "User-Agent: " + UA, "https://api.crossref.org/works?%s" % q], timeout=60)
    try:
        return ((json.loads(out).get("message") or {}).get("items")) or []
    except Exception:
        return []


def main():
    man_path, unres_path = sys.argv[1], sys.argv[2]
    man = json.load(open(man_path, encoding="utf-8"))
    have = {x["key"] for x in man}
    try:
        unresolved = json.load(open(unres_path, encoding="utf-8"))
    except Exception:
        unresolved = []
    # 也把 manifest 里标记 broken / 落在不可下载站点的挑出来重试
    retry = [{"key": x["key"], "title": x.get("title") or ""} for x in man
             if x.get("broken") or x.get("site") in ("sage", "elsevier", "oxford", "tandf")]
    jobs = []
    seen = set()
    for x in list(unresolved) + retry:
        if x["key"] in seen:
            continue
        seen.add(x["key"])
        jobs.append(x)
    log("待补 %d 篇" % len(jobs))

    added, fixed = 0, 0
    for x in jobs:
        title = x.get("title") or ""
        if not title:
            continue
        items = crossref(title)
        best_s, best = 0.0, None
        for w in items:
            t = (w.get("title") or [""])[0]
            s = title_sim(title, t)
            if s > best_s:
                best_s, best = s, w
        if not best or best_s < 0.62:
            log("   FAIL %-30s Crossref 无匹配 (best=%.2f)" % (x["key"], best_s))
            continue
        doi = (best.get("DOI") or "").lower()
        site = PREFIX_SITE.get(doi.split("/")[0])
        if not site or site not in DOWNLOADABLE:
            log("   SKIP %-30s %s -> %s（不可下载）" % (x["key"], doi, site))
            continue
        entry = next((m for m in man if m["key"] == x["key"]), None)
        if entry:
            entry.update({"site": site, "id": doi, "doi": doi, "score": round(best_s, 3),
                          "cites": best.get("is-referenced-by-count"),
                          "venue": (best.get("container-title") or [""])[0][:100]})
            entry.pop("broken", None)
            fixed += 1
        else:
            man.append({"key": x["key"], "title": title, "site": site, "id": doi, "doi": doi,
                        "score": round(best_s, 3), "cites": best.get("is-referenced-by-count"),
                        "venue": (best.get("container-title") or [""])[0][:100],
                        "out": "%s/%s.pdf" % (PAPERS, x["key"])})
            have.add(x["key"])
            added += 1
        log("   OK   %-30s -> %-8s %s (%.2f)" % (x["key"], site, doi, best_s))
        time.sleep(0.8)

    json.dump(man, open(man_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ok = [x for x in man if not x.get("broken")]
    log("\n新增 %d 篇、修复 %d 篇；manifest 现共 %d 篇可下载" % (added, fixed, len(ok)))
    by = {}
    for x in ok:
        by[x["site"]] = by.get(x["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))
    bad = [x["key"] for x in man if x.get("broken")]
    if bad:
        log("仍不可用: %s" % ", ".join(bad))


if __name__ == "__main__":
    main()
