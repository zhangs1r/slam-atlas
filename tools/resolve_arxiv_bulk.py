#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""resolve_arxiv_bulk.py — 批量(OR 查询)在 arXiv 上解析论文标题 -> arXiv ID

把 N 个标题按 group 个一组拼成 `ti:"A" OR ti:"B" ...` 一次查询，大幅减少请求数。
用法: python resolve_arxiv_bulk.py <titles.txt> <out.json> [group=5]
"""
import json
import re
import sys
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

API = "https://export.arxiv.org/api/query"
NS = {"a": "http://www.w3.org/2005/Atom"}
ATOM = "{http://arxiv.org/schemas/atom}"


def norm(s):
    return " ".join(re.sub(r"[^a-z0-9]+", " ", (s or "").lower()).split())


def sim(a, b):
    wa, wb = set(norm(a).split()), set(norm(b).split())
    if not wa or not wb:
        return 0.0
    inter = len(wa & wb)
    jac = inter / (len(wa | wb))
    na, nb = norm(a), norm(b)
    pre = 0
    for x, y in zip(na, nb):
        if x != y:
            break
        pre += 1
    return 0.7 * jac + 0.3 * (pre / max(len(na), len(nb), 1))


def fetch(query, max_results=100, tries=4):
    url = "%s?%s" % (API, urllib.parse.urlencode({
        "search_query": query, "start": 0, "max_results": max_results,
        "sortBy": "relevance", "sortOrder": "descending",
    }))
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "slam-survey/1.0"})
            with urllib.request.urlopen(req, timeout=90) as r:
                return r.read()
        except Exception as e:  # noqa
            if i == tries - 1:
                raise
            print("    retry %d: %s" % (i + 1, e))
            time.sleep(5 * (i + 1))


def parse(raw):
    root = ET.fromstring(raw)
    out = []
    for e in root.findall("a:entry", NS):
        aid = (e.findtext("a:id", "", NS) or "").strip()
        m = re.search(r"abs/(.+)$", aid)
        arxid = re.sub(r"v\d+$", "", m.group(1) if m else aid)
        pc = e.find(ATOM + "primary_category")
        out.append({
            "arxivId": arxid,
            "title": " ".join((e.findtext("a:title", "", NS) or "").split()),
            "summary": " ".join((e.findtext("a:summary", "", NS) or "").split()),
            "published": (e.findtext("a:published", "", NS) or "")[:10],
            "updated": (e.findtext("a:updated", "", NS) or "")[:10],
            "authors": [a.findtext("a:name", "", NS) for a in e.findall("a:author", NS)],
            "primary": pc.get("term") if pc is not None else None,
            "comment": " ".join((e.findtext(ATOM + "comment", "", NS) or "").split())[:200],
        })
    return out


def main():
    src, dst = sys.argv[1], sys.argv[2]
    group = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    titles = [l.strip() for l in open(src, encoding="utf-8")
              if l.strip() and not l.strip().startswith("#")]

    out = {}
    try:
        out = json.load(open(dst, encoding="utf-8"))
    except Exception:
        out = {}

    todo = [t for t in titles if not (out.get(t, {}).get("best"))]
    print("total=%d todo=%d group=%d" % (len(titles), len(todo), group))

    for i in range(0, len(todo), group):
        chunk = todo[i:i + group]
        q = " OR ".join('ti:"%s"' % re.sub(r'"', "", t) for t in chunk)
        print("--- batch %d: %s" % (i // group + 1, " | ".join(c[:34] for c in chunk)))
        try:
            cands = parse(fetch(q, max_results=120))
        except Exception as e:  # noqa
            print("    ERROR: %s" % e)
            for t in chunk:
                out.setdefault(t, {"query": t, "best": None, "error": str(e)})
            continue
        print("    got %d candidates" % len(cands))
        for t in chunk:
            scored = sorted(((sim(t, c["title"]), c) for c in cands), key=lambda x: -x[0])
            best = None
            if scored and scored[0][0] >= 0.6:
                best = dict(scored[0][1])
                best["score"] = round(scored[0][0], 3)
            out[t] = {
                "query": t, "best": best,
                "candidates": [{"score": round(s, 3), "id": c["arxivId"], "title": c["title"][:110],
                                "published": c["published"]} for s, c in scored[:3]],
            }
            print("    [%s] %-64s -> %s" % ("ok" if best else "--", t[:64],
                                            ("%s %s (%.2f)" % (best["arxivId"], best["published"], best["score"])) if best else "NO MATCH"))
        json.dump(out, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        time.sleep(3)

    ok = sum(1 for v in out.values() if v.get("best"))
    print("\nresolved %d/%d -> %s" % (ok, len(out), dst))


if __name__ == "__main__":
    main()
