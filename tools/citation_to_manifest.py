#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""citation_to_manifest.py — 把引用网络发现的"缺失经典"转成可下载清单

10.1109 开头的 DOI 通过 IEEE 检索反查 articleNumber；其余按 DOI 前缀路由到出版商。

用法: python citation_to_manifest.py <citation_missing.json> <out_manifest.json>
"""
import json
import os
import re
import subprocess
import sys
import time
import urllib.parse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matchutil import title_sim  # noqa: E402

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")
CJ = r"G:/project/ieeexplore/meta/_cj_lookup.txt"
PAPERS = r"G:/project/ieeexplore/papers"
PREFIX_SITE = {"10.1007": "springer", "10.1002": "wiley", "10.1145": "acm",
               "10.1038": "nature", "10.1088": "iop", "10.1126": "science",
               "10.3389": "frontiers", "10.3390": "mdpi", "10.1016": "elsevier",
               "10.1093": "oxford", "10.1177": "sage", "10.1561": "arxiv"}


def log(m):
    sys.stdout.write("[c2m] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=90):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def ieee_by_doi(doi):
    """用 DOI 反查 IEEE articleNumber"""
    body = json.dumps({"newsearch": True, "queryText": doi, "highlight": False,
                       "returnFacets": ["ALL"], "returnType": "SEARCH", "matchPubs": True,
                       "rowsPerPage": 5, "pageNumber": 1})
    out = curl(["--compressed", "-b", CJ, "-c", CJ, "-X", "POST",
                "https://ieeexplore.ieee.org/rest/search",
                "-H", "User-Agent: " + UA, "-H", "Content-Type: application/json",
                "-H", "Accept: application/json, text/plain, */*",
                "-H", "Referer: https://ieeexplore.ieee.org/search/searchresult.jsp",
                "-H", "Origin: https://ieeexplore.ieee.org",
                "-H", "Sec-Fetch-Dest: empty", "-H", "Sec-Fetch-Mode: cors",
                "-H", "Sec-Fetch-Site: same-origin", "--data", body], timeout=90)
    try:
        recs = json.loads(out).get("records") or []
    except Exception:
        return None, None
    for r in recs:
        if (r.get("doi") or "").lower() == doi.lower():
            return r.get("articleNumber"), r
    return None, (recs[0] if recs else None)


def safe_key(year, title):
    s = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in title)[:56]
    return "%s_%s" % (year or "0000", s)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    items = json.load(open(src, encoding="utf-8"))
    log("引用网络候选 %d 篇" % len(items))

    # 预热 IEEE
    curl(["--compressed", "-c", CJ, "-b", CJ, "-o", os.devnull,
          "-H", "User-Agent: " + UA, "-H", "Accept: text/html,*/*;q=0.8",
          "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
          "-H", "Sec-Fetch-Site: none", "-H", "Upgrade-Insecure-Requests: 1",
          "https://ieeexplore.ieee.org/"], timeout=60)
    time.sleep(1)

    man, skipped = [], []
    for x in items:
        doi = (x.get("doi") or "").lower()
        title = x.get("title") or ""
        if not doi or not title:
            continue
        pfx = doi.split("/")[0]
        if pfx == "10.1109":
            aid, _ = ieee_by_doi(doi)
            if aid:
                man.append({"key": safe_key(x.get("year"), title), "title": title,
                            "site": "ieee", "id": aid, "doi": doi,
                            "cites": None, "freq": x.get("freq"),
                            "venue": x.get("venue"),
                            "out": "%s/%s.pdf" % (PAPERS, safe_key(x.get("year"), title)),
                            "src": "citation"})
                log("   OK   %-58s -> ieee %s" % (title[:58], aid))
            else:
                skipped.append({"title": title, "doi": doi, "why": "IEEE 反查失败"})
                log("   FAIL %-58s IEEE 反查失败" % title[:58])
            time.sleep(0.7)
        else:
            site = PREFIX_SITE.get(pfx)
            if not site or site in ("elsevier", "oxford", "sage", "tandf"):
                skipped.append({"title": title, "doi": doi, "why": "站点 %s 不可下载" % site})
                log("   SKIP %-58s %s 不可下载" % (title[:58], site or pfx))
                continue
            key = safe_key(x.get("year"), title)
            man.append({"key": key, "title": title, "site": site, "id": doi, "doi": doi,
                        "freq": x.get("freq"), "venue": x.get("venue"),
                        "out": "%s/%s.pdf" % (PAPERS, key), "src": "citation"})
            log("   OK   %-58s -> %s" % (title[:58], site))

    json.dump(man, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    log("\n可下载 %d 篇（跳过 %d 篇）-> %s" % (len(man), len(skipped), dst))
    by = {}
    for m in man:
        by[m["site"]] = by.get(m["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))
    json.dump(skipped, open(dst.replace(".json", "_skipped.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
