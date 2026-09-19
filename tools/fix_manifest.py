#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""fix_manifest.py — 修补下载清单里的三类问题

① site=ieee 但 id 是 DOI（不是 articleNumber）→ 用 DOI 反查 IEEE 拿编号
② site=sage（443 不通）→ 去 arXiv 找预印本
③ 完全未解析的标题 → 去 arXiv 找预印本

用法: python fix_manifest.py <manifest.json> <unresolved.json>
"""
import json
import os
import re
import subprocess
import sys
import time
import urllib.parse
import xml.etree.ElementTree as ET

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")
CJ = r"G:/project/ieeexplore/meta/_cj_lookup.txt"
ARXIV = "https://export.arxiv.org/api/query"
NS = {"a": "http://www.w3.org/2005/Atom"}
PAPERS = r"G:/project/ieeexplore/papers"


def log(m):
    sys.stdout.write("[fix] %s\n" % m)
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


def ieee_by_text(text, rows=5):
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


def arxiv_bulk(titles, group=5):
    """批量向 arXiv 查标题，返回 {title: (arxiv_id, matched_title)}"""
    found = {}
    for i in range(0, len(titles), group):
        chunk = titles[i:i + group]
        q = " OR ".join('ti:"%s"' % re.sub(r'"', "", t) for t in chunk)
        url = "%s?%s" % (ARXIV, urllib.parse.urlencode({
            "search_query": q, "start": 0, "max_results": 120,
            "sortBy": "relevance", "sortOrder": "descending"}))
        raw = curl(["-H", "User-Agent: slam-corpus/1.0", url], timeout=180)
        if not raw:
            log("   arXiv 批次无响应: %s" % chunk[0][:50])
            continue
        try:
            root = ET.fromstring(raw)
        except Exception as e:
            log("   arXiv 解析失败: %s" % e)
            continue
        cands = []
        for e in root.findall("a:entry", NS):
            aid = (e.findtext("a:id", "", NS) or "").strip()
            m = re.search(r"abs/(.+)$", aid)
            cands.append({"id": re.sub(r"v\d+$", "", m.group(1) if m else aid),
                          "title": " ".join((e.findtext("a:title", "", NS) or "").split()),
                          "year": (e.findtext("a:published", "", NS) or "")[:4]})
        for t in chunk:
            sc = sorted(((sim(t, c["title"]), c) for c in cands), key=lambda x: -x[0])
            if sc and sc[0][0] >= 0.6:
                found[t] = (sc[0][1]["id"], sc[0][1]["title"], round(sc[0][0], 3))
        time.sleep(3)
    return found


def main():
    man_path, unres_path = sys.argv[1], sys.argv[2]
    man = json.load(open(man_path, encoding="utf-8"))
    try:
        unresolved = json.load(open(unres_path, encoding="utf-8"))
    except Exception:
        unresolved = []

    # ---- ① IEEE 里 id 是 DOI 的，用 DOI 反查编号 ----
    log("① 修复 IEEE DOI→编号")
    for it in man:
        if it["site"] == "ieee" and not str(it["id"]).isdigit():
            doi = it["id"]
            recs = ieee_by_text(doi, rows=3)
            hit = None
            for r in recs:
                if (r.get("doi") or "").lower() == doi.lower():
                    hit = r
                    break
            if not hit and recs:
                hit = recs[0]
            if hit and str(hit.get("articleNumber", "")).isdigit():
                it["id"] = hit["articleNumber"]
                it["venue"] = hit.get("publicationTitle")
                log("   OK   %-28s %s -> %s" % (it["key"], doi[:34], it["id"]))
            else:
                it["broken"] = True
                log("   FAIL %-28s %s" % (it["key"], doi))
            time.sleep(0.8)

    # ---- ② SAGE → 换 arXiv ----
    log("② SAGE 渠道改走 arXiv")
    sage = [it for it in man if it["site"] == "sage"]
    for it in sage:
        it["title_orig"] = it.get("title") or ""
    if sage:
        # 需要标题：manifest 里可能没有，从 unresolved 或原始清单补
        src_titles = {}
        for line in open(r"G:/project/ieeexplore/meta/missing_classics.txt", encoding="utf-8"):
            if "|" in line and not line.startswith("#"):
                k, _, t = line.strip().partition("|")
                src_titles[k.strip()] = t.strip()
        for it in sage:
            t = it.get("title") or src_titles.get(it["key"], "")
            it["title"] = t
        got = arxiv_bulk([it["title"] for it in sage if it["title"]])
        for it in sage:
            g = got.get(it["title"])
            if g:
                it["site"], it["id"], it["arxiv"] = "arxiv", g[0], g[0]
                it.pop("broken", None)
                log("   OK   %-28s -> arXiv %s (%.2f)" % (it["key"], g[0], g[2]))
            else:
                it["broken"] = True
                log("   FAIL %-28s 无 arXiv 版本" % it["key"])

    # ---- ③ 未解析的标题 → arXiv ----
    log("③ 未解析标题改走 arXiv")
    if unresolved:
        titles = [x["title"] for x in unresolved]
        got = arxiv_bulk(titles)
        for x in unresolved:
            g = got.get(x["title"])
            if g:
                man.append({"key": x["key"], "title": x["title"], "site": "arxiv",
                            "id": g[0], "arxiv": g[0], "score": g[2],
                            "out": "%s/%s.pdf" % (PAPERS, x["key"])})
                log("   OK   %-30s -> arXiv %s (%.2f)" % (x["key"], g[0], g[2]))
            else:
                log("   FAIL %-30s 无 arXiv 版本" % x["key"])

    json.dump(man, open(man_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ok = sum(1 for x in man if not x.get("broken"))
    log("\n最终可下载 %d 篇（其中标记 broken 的会跳过）" % ok)
    by = {}
    for x in man:
        if not x.get("broken"):
            by[x["site"]] = by.get(x["site"], 0) + 1
    log("渠道分布: %s" % json.dumps(by, ensure_ascii=False))


if __name__ == "__main__":
    main()
