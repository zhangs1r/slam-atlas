#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ieee_lookup_curl.py — 用 curl 直调 IEEE /rest/search 做标题定向检索（无需浏览器）

用法: python ieee_lookup_curl.py <titles.txt> <out.json> [--rows 5]
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")
CJ = r"G:/project/ieeexplore/meta/_cj_lookup.txt"
HOME = "https://ieeexplore.ieee.org/"


def log(m):
    sys.stdout.write("[lookup] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=90):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, text=True, timeout=timeout + 30)
        return r.stdout or ""
    except subprocess.TimeoutExpired:
        return ""


def warmup():
    curl(["--compressed", "-c", CJ, "-b", CJ, "-o", os.devnull,
          "-H", "User-Agent: " + UA,
          "-H", "Accept: text/html,application/xhtml+xml,*/*;q=0.8",
          "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
          "-H", "Sec-Fetch-Site: none", "-H", "Upgrade-Insecure-Requests: 1", HOME])


def search(query, rows=5):
    body = json.dumps({
        "newsearch": True,
        "queryText": '("Document Title":"%s")' % query.replace('"', ""),
        "highlight": False,
        "returnFacets": ["ALL"],
        "returnType": "SEARCH",
        "matchPubs": True,
        "rowsPerPage": rows,
        "pageNumber": 1,
    })
    out = curl([
        "--compressed", "-b", CJ, "-c", CJ, "-X", "POST",
        "https://ieeexplore.ieee.org/rest/search",
        "-H", "User-Agent: " + UA,
        "-H", "Content-Type: application/json",
        "-H", "Accept: application/json, text/plain, */*",
        "-H", "Referer: https://ieeexplore.ieee.org/search/searchresult.jsp",
        "-H", "Origin: https://ieeexplore.ieee.org",
        "-H", "Sec-Fetch-Dest: empty", "-H", "Sec-Fetch-Mode: cors",
        "-H", "Sec-Fetch-Site: same-origin",
        "--data", body,
    ], timeout=90)
    try:
        return json.loads(out)
    except Exception:
        return None


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("titles")
    ap.add_argument("out")
    ap.add_argument("--rows", type=int, default=5)
    args = ap.parse_args()

    titles = [l.strip() for l in open(args.titles, encoding="utf-8")
              if l.strip() and not l.strip().startswith("#")]
    out = {}
    if os.path.exists(args.out):
        try:
            out = json.load(open(args.out, encoding="utf-8"))
        except Exception:
            out = {}

    warmup()
    time.sleep(1)

    for t in titles:
        if out.get(t, {}).get("best"):
            log("skip %s" % t[:60])
            continue
        j = search(t, args.rows)
        if not j:
            out[t] = {"best": None, "error": "no json"}
            log("ERR  %s" % t[:60])
            continue
        recs = j.get("records") or []
        scored = sorted(((sim(t, r.get("articleTitle", "")), r) for r in recs), key=lambda x: -x[0])
        best = None
        if scored and scored[0][0] >= 0.5:
            r = scored[0][1]
            best = {
                "score": round(scored[0][0], 3),
                "id": r.get("articleNumber"),
                "title": r.get("articleTitle"),
                "venue": r.get("publicationTitle"),
                "year": r.get("publicationYear"),
                "cites": r.get("citationCount"),
                "access": (r.get("accessType") or {}).get("type"),
                "doi": r.get("doi"),
            }
        out[t] = {"best": best, "hits": len(recs)}
        if best:
            log("OK   %-56s id=%-9s %s (%s)" % (t[:56], best["id"], best["year"], best["access"]))
        else:
            near = [r.get("articleTitle", "")[:60] for _, r in scored[:2]]
            out[t]["near"] = near
            log("MISS %-56s near=%s" % (t[:56], near[:1]))
        json.dump(out, open(args.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        time.sleep(1.2)

    ok = sum(1 for v in out.values() if v.get("best"))
    log("resolved %d/%d -> %s" % (ok, len(out), args.out))


if __name__ == "__main__":
    main()
