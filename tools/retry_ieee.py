#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""retry_ieee.py — 对仍未解析的标题再做一轮 IEEE 检索（用改进的标题匹配）

用法: python retry_ieee.py <manifest.json> <failed.json> [threshold]
"""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matchutil import title_sim  # noqa: E402

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")
CJ = r"G:/project/ieeexplore/meta/_cj_lookup.txt"
PAPERS = r"G:/project/ieeexplore/papers"


def log(m):
    sys.stdout.write("[retry] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=90):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def ieee_search(text, rows=8):
    body = json.dumps({"newsearch": True, "queryText": text, "highlight": False,
                       "returnFacets": ["ALL"], "returnType": "SEARCH", "matchPubs": True,
                       "rowsPerPage": rows, "pageNumber": 1})
    out = curl(["--compressed", "-b", CJ, "-c", CJ, "-X", "POST",
                "https://ieeexplore.ieee.org/rest/search",
                "-H", "User-Agent: " + UA, "-H", "Content-Type: application/json",
                "-H", "Accept: application/json, text/plain, */*",
                "-H", "Referer: https://ieeexplore.ieee.org/search/searchresult.jsp",
                "-H", "Origin: https://ieeexplore.ieee.org",
                "-H", "Sec-Fetch-Dest: empty", "-H", "Sec-Fetch-Mode: cors",
                "-H", "Sec-Fetch-Site: same-origin", "--data", body], timeout=90)
    try:
        return json.loads(out).get("records") or []
    except Exception:
        return []


def main():
    man_path, fail_path = sys.argv[1], sys.argv[2]
    thr = float(sys.argv[3]) if len(sys.argv) > 3 else 0.60
    man = json.load(open(man_path, encoding="utf-8"))
    have = {x["key"] for x in man}
    try:
        failed = json.load(open(fail_path, encoding="utf-8"))
    except Exception:
        failed = []

    curl(["--compressed", "-c", CJ, "-b", CJ, "-o", os.devnull,
          "-H", "User-Agent: " + UA, "-H", "Accept: text/html,*/*;q=0.8",
          "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
          "-H", "Sec-Fetch-Site: none", "-H", "Upgrade-Insecure-Requests: 1",
          "https://ieeexplore.ieee.org/"], timeout=60)
    time.sleep(1)

    added = 0
    for x in failed:
        if x["key"] in have:
            continue
        # 去掉副标题再试一次（冒号后的部分常导致检索失败）
        variants = [x["title"]]
        if ":" in x["title"]:
            variants.append(x["title"].split(":")[0])
        variants.append(" ".join(x["title"].split()[:8]))
        hit = None
        for v in variants:
            recs = ieee_search(v)
            best_s, best_r = 0.0, None
            for r in recs:
                s = title_sim(x["title"], r.get("articleTitle", ""))
                if s > best_s:
                    best_s, best_r = s, r
            if best_r and best_s >= thr:
                hit = (best_s, best_r)
                break
            time.sleep(0.6)
        if hit:
            s, r = hit
            man.append({"key": x["key"], "title": x["title"], "site": "ieee",
                        "id": r.get("articleNumber"), "score": round(s, 3),
                        "venue": r.get("publicationTitle"),
                        "cites": r.get("citationCount"),
                        "out": "%s/%s.pdf" % (PAPERS, x["key"])})
            have.add(x["key"])
            added += 1
            log("   OK   %-30s -> %s (%.2f) %s" % (x["key"], r.get("articleNumber"), s,
                                                   (r.get("publicationTitle") or "")[:44]))
        else:
            log("   FAIL %-30s 仍无匹配" % x["key"])
        time.sleep(0.8)

    json.dump(man, open(man_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ok = [x for x in man if not x.get("broken")]
    log("\n新增 %d 篇；manifest 共 %d 篇可下载" % (added, len(ok)))
    by = {}
    for x in ok:
        by[x["site"]] = by.get(x["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))


if __name__ == "__main__":
    main()
