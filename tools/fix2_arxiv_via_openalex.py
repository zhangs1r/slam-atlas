#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""fix2_arxiv_via_openalex.py — 用 OpenAlex 找论文的 arXiv 预印本

比 arXiv 官方 API 快且稳（后者常返回畸形 XML / 超时）。
OpenAlex 里 arXiv 论文的 DOI 形如 10.48550/arxiv.XXXX.XXXXX。

用法: python fix2_arxiv_via_openalex.py <manifest.json>
"""
import json
import re
import subprocess
import sys
import time
import urllib.parse

UA = "slam-corpus/1.0 (mailto:research@example.org)"
PAPERS = r"G:/project/ieeexplore/papers"


def log(m):
    sys.stdout.write("[fix2] %s\n" % m)
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


def oa_find_arxiv(title):
    """在 OpenAlex 上找该标题的 arXiv 版本；顺带返回任意可下载版本"""
    q = urllib.parse.urlencode({"search": title, "per-page": "8",
                                "select": "doi,title,publication_year,primary_location"})
    out = curl(["-H", "User-Agent: " + UA,
                "https://api.openalex.org/works?%s" % q], timeout=60)
    try:
        res = json.loads(out).get("results") or []
    except Exception:
        return None, None
    best_arxiv, best_any = None, None
    for w in res:
        doi = (w.get("doi") or "").replace("https://doi.org/", "").lower()
        s = sim(title, w.get("title", ""))
        if s < 0.6:
            continue
        rec = {"doi": doi, "score": round(s, 3), "year": w.get("publication_year")}
        if doi.startswith("10.48550/arxiv."):
            aid = doi.split("arxiv.")[-1]
            rec["arxiv"] = aid
            if not best_arxiv or s > best_arxiv["score"]:
                best_arxiv = rec
        elif not best_any or s > best_any["score"]:
            best_any = rec
    return best_arxiv, best_any


PREFIX_SITE = {"10.1109": "ieee", "10.1007": "springer", "10.1038": "nature",
               "10.1002": "wiley", "10.1145": "acm", "10.1088": "iop",
               "10.1126": "science", "10.3389": "frontiers", "10.3390": "mdpi"}


def main():
    path = sys.argv[1]
    man = json.load(open(path, encoding="utf-8"))
    src_titles = {}
    for line in open(r"G:/project/ieeexplore/meta/missing_classics.txt", encoding="utf-8"):
        if "|" in line and not line.startswith("#"):
            k, _, t = line.strip().partition("|")
            src_titles[k.strip()] = t.strip()

    todo = [it for it in man if it.get("broken") or it.get("site") == "sage"]
    log("待修复 %d 篇" % len(todo))
    fixed = 0
    for it in todo:
        title = it.get("title") or src_titles.get(it["key"], "")
        if not title:
            log("   SKIP %-30s 无标题" % it["key"])
            continue
        arx, anyr = oa_find_arxiv(title)
        if arx:
            it.update({"site": "arxiv", "id": arx["arxiv"], "arxiv": arx["arxiv"],
                       "score": arx["score"]})
            it.pop("broken", None)
            fixed += 1
            log("   OK   %-30s -> arXiv %s (%.2f)" % (it["key"], arx["arxiv"], arx["score"]))
        elif anyr and PREFIX_SITE.get(anyr["doi"].split("/")[0]):
            site = PREFIX_SITE[anyr["doi"].split("/")[0]]
            it.update({"site": site, "id": anyr["doi"], "score": anyr["score"]})
            it.pop("broken", None)
            fixed += 1
            log("   OK   %-30s -> %s %s (%.2f)" % (it["key"], site, anyr["doi"], anyr["score"]))
        else:
            if it.get("site") == "sage":
                it["broken"] = True
            log("   FAIL %-30s 无替代版本" % it["key"])
        time.sleep(0.6)

    json.dump(man, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ok = [x for x in man if not x.get("broken")]
    log("\n可下载 %d 篇，修复 %d 篇" % (len(ok), fixed))
    by = {}
    for x in ok:
        by[x["site"]] = by.get(x["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))
    bad = [x["key"] for x in man if x.get("broken")]
    if bad:
        log("仍不可用: %s" % ", ".join(bad))


if __name__ == "__main__":
    main()
