#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""fetch_ieee.py — 通过校园 VPN(IP 认证) + 浏览器 UA 用 curl 批量下载 IEEE PDF

关键点:
  1. stampPDF/getPDF.jsp 会 302 到 ...&tag=1，必须跟随跳转
  2. 必须带浏览器 User-Agent + Referer，否则被 Incapsula 拦成 418
  3. 落盘后校验 %PDF- 魔数，避免把付费墙 HTML 当成 PDF

用法: python fetch_ieee.py <manifest.json> [--workers 3] [--force]
"""
import argparse
import concurrent.futures as cf
import json
import os
import re
import subprocess
import sys
import time

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")


def log(m):
    sys.stdout.write("[fetch] %s\n" % m)
    sys.stdout.flush()


# 关键：必须补齐真实浏览器的 Accept / Sec-Fetch-* 头，否则 Incapsula 会把请求
# 判成脚本并返回 202/200 的 HTML 挑战页（看起来"成功"其实不是 PDF）。
BROWSER_HEADERS = [
    "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
    "-H", "Accept-Language: en-US,en;q=0.9",
    "-H", "Sec-Fetch-Dest: document",
    "-H", "Sec-Fetch-Mode: navigate",
    "-H", "Sec-Fetch-Site: same-origin",
    "-H", "Sec-Fetch-User: ?1",
    "-H", "Upgrade-Insecure-Requests: 1",
]


def _run(cmd, timeout=420):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (r.stdout or "").strip()
    except subprocess.TimeoutExpired:
        return "timeout|-|0|-"


def curl_pdf(url, referer, out, cookiejar, tries=3):
    """两步走：先访问文献页拿 cookie/通过挑战，再取 PDF。"""
    tmp = out + ".part"
    code = ctype = "?"
    size = 0
    magic = b""
    origin = "https://ieeexplore.ieee.org"
    for attempt in range(tries):
        # step1: 预热，建立会话 cookie
        _run(["curl", "-sS", "--noproxy", "*", "-m", "90", "--compressed",
              "-c", cookiejar, "-b", cookiejar, "-o", os.devnull,
              "-H", "User-Agent: " + UA,
              "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
              "-H", "Accept-Language: en-US,en;q=0.9",
              "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
              "-H", "Sec-Fetch-Site: none", "-H", "Sec-Fetch-User: ?1",
              "-H", "Upgrade-Insecure-Requests: 1",
              origin + "/"])
        time.sleep(1.5 + attempt)
        # step2: 取 PDF
        meta = _run(["curl", "-sS", "-L", "--noproxy", "*", "-m", "360", "--compressed",
                     "-c", cookiejar, "-b", cookiejar, "-o", tmp,
                     "-w", "%{http_code}|%{content_type}|%{size_download}|%{url_effective}",
                     "-H", "User-Agent: " + UA,
                     "-H", "Referer: " + referer] + BROWSER_HEADERS + [url])
        parts = meta.split("|")
        code = parts[0] if parts else "?"
        ctype = parts[1] if len(parts) > 1 else "?"
        size = int(parts[2]) if len(parts) > 2 and parts[2].isdigit() else 0
        if os.path.exists(tmp):
            with open(tmp, "rb") as fh:
                magic = fh.read(5)
            if magic == b"%PDF-":
                os.replace(tmp, out)
                return {"ok": True, "code": code, "type": ctype, "bytes": size}
            try:
                os.remove(tmp)
            except OSError:
                pass
        if attempt < tries - 1:
            time.sleep(8 * (attempt + 1))
    return {"ok": False, "code": code, "type": ctype, "bytes": size,
            "magic": magic.decode("latin1", "replace")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    items = json.load(open(args.manifest, encoding="utf-8"))
    todo = []
    for it in items:
        out = it["out"]
        if (not args.force) and os.path.exists(out) and os.path.getsize(out) > 200000:
            with open(out, "rb") as fh:
                if fh.read(5) == b"%PDF-":
                    log("SKIP %s (已存在 %d B)" % (os.path.basename(out), os.path.getsize(out)))
                    continue
        if os.path.exists(out):
            os.remove(out)
        todo.append(it)

    log("待下载 %d 篇（并发 %d）" % (len(todo), args.workers))
    results = {}

    def work(idx_it):
        idx, it = idx_it
        t0 = time.time()
        cj = os.path.join(os.path.dirname(args.manifest), "_cj_%d.txt" % (idx % max(args.workers, 1)))
        r = curl_pdf(it["pdfUrl"], it["pageUrl"], it["out"], cj)
        r["key"] = it.get("key")
        r["secs"] = round(time.time() - t0, 1)
        if r["ok"]:
            log("OK   %-28s %7.2f MB  %ss" % (r["key"], r["bytes"] / 1048576, r["secs"]))
        else:
            log("FAIL %-28s http=%s type=%s magic=%r" % (r["key"], r["code"], r["type"], r.get("magic")))
        return r

    with cf.ThreadPoolExecutor(max_workers=args.workers) as ex:
        for r in ex.map(work, list(enumerate(todo))):
            results[r["key"]] = r

    ok = sum(1 for v in results.values() if v["ok"])
    log("完成: %d/%d 成功" % (ok, len(results)))
    fails = {k: v for k, v in results.items() if not v["ok"]}
    if fails:
        log("失败清单:")
        for k, v in fails.items():
            log("   %s  http=%s type=%s" % (k, v["code"], v["type"]))
    json.dump(results, open(args.manifest.replace(".json", "") + ".fetch.json", "w",
                            encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
