#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""site_probe2.py — 第二轮：验证"两步走"（先预热拿 cookie → 再取 PDF）能否解锁更多站点

针对第一轮中返回 HTML 的站点，以及连接失败(000)的站点做定向复测。
DOI 通过 OpenAlex 解析，不靠猜。
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
HDRS = [
    "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
    "-H", "Accept-Language: en-US,en;q=0.9",
    "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
    "-H", "Sec-Fetch-Site: same-origin", "-H", "Sec-Fetch-User: ?1",
    "-H", "Upgrade-Insecure-Requests: 1",
]
TMP = r"G:/project/ieeexplore/meta/_probe2"
os.makedirs(TMP, exist_ok=True)


def curl(args, timeout=120):
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 40)
        return (r.stdout or b"").decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return ""


def oa_search(query, per_page=100):
    q = urllib.parse.urlencode({"search": query, "per-page": str(per_page),
                                "select": "doi,title,primary_location,publication_year"})
    out = curl(["-H", "User-Agent: " + UA, "https://api.openalex.org/works?%s" % q], timeout=60)
    try:
        return json.loads(out).get("results", [])
    except Exception:
        return []


def find_doi(results, prefix):
    """按 DOI 前缀匹配出版商（比按源名匹配可靠得多）"""
    for w in results:
        doi = (w.get("doi") or "").replace("https://doi.org/", "")
        if doi.lower().startswith(prefix.lower()):
            src = ((w.get("primary_location") or {}).get("source") or {})
            return doi, (src.get("display_name") or "")
    return None, None


def two_step(landing, pdf_url, jar, tag):
    """① 预热 landing 取 cookie  ② 带 cookie + Referer 取 PDF"""
    # ①
    curl(["--compressed", "-c", jar, "-b", jar, "-o", os.devnull,
          "-H", "User-Agent: " + UA,
          "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
          "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
          "-H", "Sec-Fetch-Site: none", "-H", "Upgrade-Insecure-Requests: 1",
          landing], timeout=90)
    time.sleep(1.5)
    # ②
    f = os.path.join(TMP, tag + ".bin")
    meta = curl(["-sS", "-L", "--compressed", "-c", jar, "-b", jar, "-o", f,
                 "-w", "%{http_code}|%{content_type}|%{size_download}",
                 "-H", "User-Agent: " + UA, "-H", "Referer: " + landing] + HDRS + [pdf_url],
                timeout=120)
    p = (meta or "").split("|")
    code = p[0] if p else "?"
    ctype = p[1] if len(p) > 1 else "?"
    size = p[2] if len(p) > 2 else "0"
    magic = ""
    if os.path.exists(f):
        with open(f, "rb") as fh:
            magic = fh.read(5).decode("latin1", "replace")
    if magic == "%PDF-":
        v = "✅ 两步走成功"
    elif code in ("403", "401"):
        v = "⛔ 风控拦截"
    elif code == "000":
        v = "❌ 连接失败"
    elif "html" in (ctype or "").lower():
        v = "⚠️ 仍是网页"
    else:
        v = "❓ " + code
    try:
        os.remove(f)
    except OSError:
        pass
    return {"verdict": v, "status": code, "ctype": ctype, "size_mb": round(int(size) / 1048576, 2) if size.isdigit() else 0}


TARGETS = [
    ("Wiley", "10.1002"),
    ("ACM DL", "10.1145"),
    ("MDPI", "10.3390"),
    ("Frontiers", "10.3389"),
    ("Oxford Academic", "10.1093"),
    ("SAGE", "10.1177"),
    ("Taylor & Francis", "10.1080"),
    ("Elsevier (ScienceDirect)", "10.1016"),
    ("Springer Link", "10.1007"),
    ("IOPscience", "10.1088"),
    ("Nature", "10.1038"),
    ("IEEE Xplore", "10.1109"),
]

PATTERN = {
    "Wiley": "https://onlinelibrary.wiley.com/doi/pdfdirect/{doi}",
    "ACM DL": "https://dl.acm.org/doi/pdf/{doi}",
    "MDPI": "https://www.mdpi.com/{doi}/pdf",
    "Frontiers": "https://www.frontiersin.org/articles/{doi}/pdf",
    "Oxford Academic": "https://academic.oup.com/doi/pdf/{doi}",
    "SAGE": "https://journals.sagepub.com/doi/pdf/{doi}",
    "Taylor & Francis": "https://www.tandfonline.com/doi/pdf/{doi}",
    "Elsevier (ScienceDirect)": None,
    "Springer Link": "https://link.springer.com/content/pdf/{doi}.pdf",
    "IOPscience": "https://iopscience.iop.org/article/{doi}/pdf",
    "Nature": "https://www.nature.com/articles/{suffix}.pdf",
    "IEEE Xplore": None,
}

print("%-26s %-44s %s" % ("站点", "解析到的 DOI (出版商)", "两步走结果"))
print("-" * 130)
rows = []
# 广撒网：一次拉 200 条，覆盖多家出版商的 DOI
pool = []
for q in ["robot", "SLAM", "deep learning", "sensor", "navigation", "control"]:
    pool += oa_search(q, per_page=200)
    time.sleep(0.6)
print("候选池: %d 条记录\n" % len(pool))

for label, prefix in TARGETS:
    doi, pub = find_doi(pool, prefix)
    if not doi:
        print("%-26s %-44s %s" % (label, "(池中未出现)", "—"))
        rows.append({"site": label, "doi": None, "verdict": "未解析到 DOI"})
        continue
    pat = PATTERN.get(label)
    landing = "https://doi.org/%s" % doi
    if label == "Nature":
        pdf = "https://www.nature.com/articles/%s.pdf" % doi.split("/")[-1]
    elif pat:
        pdf = pat.format(doi=doi, suffix=doi.split("/")[-1])
    else:
        pdf = None
    if not pdf:
        print("%-26s %-44s %s" % (label, "%s (%s)" % (doi, (pub or "")[:18]), "无通用 PDF 模式，需从落地页解析"))
        rows.append({"site": label, "doi": doi, "publisher": pub, "verdict": "无通用 PDF 模式"})
        continue
    jar = os.path.join(TMP, re.sub(r"\W+", "_", label) + ".txt")
    r = two_step(landing, pdf, jar, re.sub(r"\W+", "_", label))
    print("%-26s %-44s %s  [http=%s type=%s %sMB]" % (
        label, "%s (%s)" % (doi, (pub or "")[:18]), r["verdict"],
        r["status"], (r["ctype"] or "")[:24], r["size_mb"]))
    rows.append({"site": label, "doi": doi, "publisher": pub, "pdf_url": pdf, **r})
    time.sleep(0.8)

json.dump(rows, open(r"G:/project/ieeexplore/meta/site_probe2_result.json", "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
print("\n→ meta/site_probe2_result.json")
