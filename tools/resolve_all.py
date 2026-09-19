#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""resolve_all.py — 统一解析论文标题 → 下载坐标（IEEE articleNumber 或 出版商+DOI）

流程：① 先查 IEEE /rest/search（命中则走 IEEE 通道）
      ② 未命中转 OpenAlex 拿 DOI → 按 DOI 前缀路由到对应出版商
      ③ 输出可直接喂给 fetch_paper.py 的 manifest

用法: python resolve_all.py <keytitles.txt> <out_manifest.json>
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
CJ = r"G:/project/ieeexplore/meta/_cj_lookup.txt"
PAPERS = r"G:/project/ieeexplore/papers"

# DOI 前缀 → 站点（与 fetch_paper.py 的 SITES 对应）
PREFIX_SITE = {
    "10.1109": "ieee", "10.48550": "arxiv", "10.1007": "springer", "10.1038": "nature",
    "10.1002": "wiley", "10.1145": "acm", "10.1088": "iop", "10.1126": "science",
    "10.3389": "frontiers", "10.3390": "mdpi", "10.1080": "tandf", "10.1016": "elsevier",
    "10.1093": "oxford", "10.1177": "sage",
}


def log(m):
    sys.stdout.write("[resolve] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=90):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 30)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def norm(s):
    return " ".join(re.sub(r"[^a-z0-9]+", " ", (s or "").lower()).split())


def sim(a, b):
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
    return 0.7 * jac + 0.3 * (pre / max(len(na), len(nb), 1))


def ieee_search(title, rows=5):
    body = json.dumps({
        "newsearch": True,
        "queryText": '("Document Title":"%s")' % title.replace('"', ""),
        "highlight": False, "returnFacets": ["ALL"], "returnType": "SEARCH",
        "matchPubs": True, "rowsPerPage": rows, "pageNumber": 1,
    })
    out = curl(["--compressed", "-b", CJ, "-c", CJ, "-X", "POST",
                "https://ieeexplore.ieee.org/rest/search",
                "-H", "User-Agent: " + UA,
                "-H", "Content-Type: application/json",
                "-H", "Accept: application/json, text/plain, */*",
                "-H", "Referer: https://ieeexplore.ieee.org/search/searchresult.jsp",
                "-H", "Origin: https://ieeexplore.ieee.org",
                "-H", "Sec-Fetch-Dest: empty", "-H", "Sec-Fetch-Mode: cors",
                "-H", "Sec-Fetch-Site: same-origin",
                "--data", body], timeout=90)
    try:
        return json.loads(out).get("records") or []
    except Exception:
        return []


def openalex_search(title):
    q = urllib.parse.urlencode({"search": title, "per-page": "5",
                                "select": "doi,title,publication_year,primary_location"})
    out = curl(["-H", "User-Agent: " + UA,
                "https://api.openalex.org/works?%s" % q], timeout=60)
    try:
        return json.loads(out).get("results") or []
    except Exception:
        return []


def main():
    src, dst = sys.argv[1], sys.argv[2]
    entries = []
    for line in open(src, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, _, title = line.partition("|")
        if title:
            entries.append((key.strip(), title.strip()))

    log("待解析 %d 篇" % len(entries))
    # IEEE 预热
    curl(["--compressed", "-c", CJ, "-b", CJ, "-o", os.devnull,
          "-H", "User-Agent: " + UA,
          "-H", "Accept: text/html,*/*;q=0.8",
          "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
          "-H", "Sec-Fetch-Site: none", "-H", "Upgrade-Insecure-Requests: 1",
          "https://ieeexplore.ieee.org/"], timeout=60)
    time.sleep(1)

    manifest, failed = [], []
    for i, (key, title) in enumerate(entries, 1):
        out = "%s/%s.pdf" % (PAPERS, key)
        yr = key.split("_")[0]
        # ① IEEE
        recs = ieee_search(title)
        best = None
        if recs:
            sc = sorted(((sim(title, r.get("articleTitle", "")), r) for r in recs), key=lambda x: -x[0])
            if sc[0][0] >= 0.55:
                r = sc[0][1]
                best = {"site": "ieee", "id": r.get("articleNumber"),
                        "venue": r.get("publicationTitle"), "year": r.get("publicationYear"),
                        "cites": r.get("citationCount"), "score": round(sc[0][0], 3)}
        # ② OpenAlex 兜底
        if not best:
            for w in openalex_search(title):
                doi = (w.get("doi") or "").replace("https://doi.org/", "")
                if not doi:
                    continue
                s = sim(title, w.get("title", ""))
                if s < 0.55:
                    continue
                pfx = doi.split("/")[0].lower()
                site = PREFIX_SITE.get(pfx)
                if not site:
                    continue
                best = {"site": site, "id": doi, "doi": doi, "score": round(s, 3),
                        "venue": ((w.get("primary_location") or {}).get("source") or {}).get("display_name")}
                break

        if best:
            manifest.append({"key": key, "title": title, "out": out, **best})
            log("[%2d/%d] %-44s -> %-9s %s" % (i, len(entries), key, best["site"], best["id"]))
        else:
            failed.append({"key": key, "title": title})
            log("[%2d/%d] %-44s -> ❌ 未解析" % (i, len(entries), key))
        json.dump(manifest, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        time.sleep(1.0)

    log("\n解析成功 %d / %d（失败 %d）" % (len(manifest), len(entries), len(failed)))
    by_site = {}
    for m in manifest:
        by_site[m["site"]] = by_site.get(m["site"], 0) + 1
    log("按渠道分布: %s" % json.dumps(by_site, ensure_ascii=False))
    if failed:
        log("未解析清单:")
        for f in failed:
            log("   %s | %s" % (f["key"], f["title"][:70]))
    json.dump(failed, open(dst.replace(".json", "_failed.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
