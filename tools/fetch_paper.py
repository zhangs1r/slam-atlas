#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""fetch_paper.py — 多出版商通用论文 PDF 下载器

把"IEEE 那套"抽象成站点无关的可复用实现：
  ① 补齐真实浏览器请求头（这是绕过基础风控的关键）
  ② 两步走：先预热落地页拿 cookie，再带 Referer 请求 PDF（解锁 Wiley/ACM/IEEE 等）
  ③ 强制校验 %PDF- 魔数（不做这一步会把风控挑战页当成 PDF 存下来）

用法:
  # 单篇探测
  python fetch_paper.py --probe wiley 10.1002/rob.21950

  # 批量（manifest 为 JSON 数组）
  python fetch_paper.py manifest.json [--workers 3] [--force]

manifest 每项:
  {"site":"wiley", "id":"10.1002/rob.21950", "out":"papers/xxx.pdf", "key":"可选标签",
   "pdf": "可选，直接指定 PDF URL 覆盖站点模板",
   "landing": "可选，直接指定落地页覆盖站点模板"}

支持的站点见下方 SITES；不在表里的站可用 "generic" + 显式给 pdf/landing。
"""
import argparse
import concurrent.futures as cf
import json
import os
import subprocess
import sys
import time

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36")

# ★ 关键：完整浏览器指纹头。缺 Sec-Fetch-* 会被 Incapsula/Cloudflare 判成脚本，
#   返回 202 或 200+text/html 的挑战页（看起来"成功"，其实是陷阱）。
BROWSER_HEADERS = [
    "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8",
    "-H", "Accept-Language: en-US,en;q=0.9",
    "-H", "Sec-Fetch-Dest: document",
    "-H", "Sec-Fetch-Mode: navigate",
    "-H", "Sec-Fetch-Site: same-origin",
    "-H", "Sec-Fetch-User: ?1",
    "-H", "Upgrade-Insecure-Requests: 1",
]

# 站点配置表（2026-09 实测）
#   warm=True  : 必须先预热落地页拿 cookie，否则 PDF 端点返回 HTML
#   warm=False : 可以直接取 PDF
SITES = {
    "arxiv":     {"landing": "https://arxiv.org/abs/{id}",
                  "pdf": "https://arxiv.org/pdf/{id}", "warm": False},
    "ieee":      {"landing": "https://ieeexplore.ieee.org/document/{id}",
                  "pdf": "https://ieeexplore.ieee.org/stampPDF/getPDF.jsp?tp=&arnumber={id}&ref=",
                  "warm": True, "note": "PDF 端点 302 到 &tag=1，需 -L"},
    "springer":  {"landing": "https://link.springer.com/article/{id}",
                  "pdf": "https://link.springer.com/content/pdf/{id}.pdf", "warm": False},
    "nature":    {"landing": "https://www.nature.com/articles/{id}",
                  "pdf": "https://www.nature.com/articles/{id}.pdf", "warm": False,
                  "note": "id 用 doi 后缀，如 s41586-021-03819-2"},
    "wiley":     {"landing": "https://onlinelibrary.wiley.com/doi/{id}",
                  "pdf": "https://onlinelibrary.wiley.com/doi/pdfdirect/{id}", "warm": True},
    "acm":       {"landing": "https://dl.acm.org/doi/{id}",
                  "pdf": "https://dl.acm.org/doi/pdf/{id}", "warm": True},
    "iop":       {"landing": "https://iopscience.iop.org/article/{id}",
                  "pdf": "https://iopscience.iop.org/article/{id}/pdf", "warm": False},
    "science":   {"landing": "https://www.science.org/doi/{id}",
                  "pdf": "https://www.science.org/doi/pdf/{id}", "warm": False,
                  "note": "AAAS"},
    "frontiers": {"landing": "https://www.frontiersin.org/articles/{id}/full",
                  "pdf": "https://www.frontiersin.org/articles/{id}/pdf", "warm": True,
                  "note": "⚠️ 旧路径可能 curl 56 断连；若失败改用 "
                          "https://www.frontiersin.org/journals/<刊名>/articles/{id}/pdf"},
    "mdpi":      {"landing": "https://doi.org/{id}", "pdf": None, "warm": True,
                  "note": "🔴 PDF 地址非 DOI 派生（/issn/vol/issue/page/pdf），需从落地页解析"},
    "tandf":     {"landing": "https://www.tandfonline.com/doi/full/{id}",
                  "pdf": "https://www.tandfonline.com/doi/pdf/{id}", "warm": True,
                  "note": "🟡 端点常返回 HTML，需从落地页解析真实链接"},
    "elsevier":  {"landing": "https://doi.org/{id}", "pdf": None, "warm": True,
                  "note": "🔴 403 强风控 + PII 无法从 DOI 推导，脚本路线不可行"},
    "oxford":    {"landing": "https://academic.oup.com/doi/{id}", "pdf": None, "warm": True,
                  "note": "🔴 403 风控拦截"},
    "sage":      {"landing": "https://journals.sagepub.com/doi/{id}",
                  "pdf": "https://journals.sagepub.com/doi/pdf/{id}", "warm": True,
                  "note": "🔴 实测 443 连接层面就不通"},
    "generic":   {"landing": None, "pdf": None, "warm": True,
                  "note": "需显式提供 landing 与 pdf"},
}

# DOI 前缀 → 站点（按前缀匹配比按期刊名匹配可靠得多）
DOI_PREFIX = {
    "10.1109": "ieee", "10.48550": "arxiv", "10.1007": "springer", "10.1038": "nature",
    "10.1002": "wiley", "10.1145": "acm", "10.1088": "iop", "10.1126": "science",
    "10.3389": "frontiers", "10.3390": "mdpi", "10.1080": "tandf", "10.1016": "elsevier",
    "10.1093": "oxford", "10.1177": "sage",
}


def log(m):
    sys.stdout.write("[fetch] %s\n" % m)
    sys.stdout.flush()


def curl(args, timeout=180):
    """返回 (stdout_text, exit_code)。统一 --noproxy 避免本地代理劫持。"""
    try:
        r = subprocess.run(["curl", "-sS", "--noproxy", "*", "-m", str(timeout)] + args,
                           capture_output=True, timeout=timeout + 60)
        return (r.stdout or b"").decode("utf-8", "replace"), r.returncode
    except subprocess.TimeoutExpired:
        return "", -1


def warmup(url, jar, timeout=90):
    """① 预热：访问落地页，建立会话 cookie / 通过基础风控"""
    if not url:
        return
    curl(["--compressed", "-c", jar, "-b", jar, "-o", os.devnull,
          "-H", "User-Agent: " + UA,
          "-H", "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
          "-H", "Accept-Language: en-US,en;q=0.9",
          "-H", "Sec-Fetch-Dest: document", "-H", "Sec-Fetch-Mode: navigate",
          "-H", "Sec-Fetch-Site: none", "-H", "Sec-Fetch-User: ?1",
          "-H", "Upgrade-Insecure-Requests: 1", url], timeout=timeout)
    time.sleep(1.5)


def download(pdf_url, landing_url, out, jar, tries=3):
    """② 取 PDF 并校验魔数"""
    tmp = out + ".part"
    code = ctype = "?"
    size = 0
    magic = b""
    for attempt in range(tries):
        meta, _ = curl(["-sS", "-L", "--compressed", "-c", jar, "-b", jar, "-o", tmp,
                        "-w", "%{http_code}|%{content_type}|%{size_download}",
                        "-H", "User-Agent: " + UA,
                        "-H", "Referer: " + (landing_url or pdf_url)] + BROWSER_HEADERS + [pdf_url],
                       timeout=300)
        p = (meta or "").split("|")
        code = p[0] if p else "?"
        ctype = p[1] if len(p) > 1 else "?"
        size = int(p[2]) if len(p) > 2 and p[2].isdigit() else 0
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
        # 诊断分级
        if code in ("403", "401"):
            hint = "风控拦截，脚本路线大概率不可行"
        elif code == "000":
            hint = "连接层面失败（网络/TLS 被拒）"
        elif "html" in (ctype or "").lower():
            hint = "返回网页：可能需要预热 / 需从落地页解析真实 PDF 链接"
        else:
            hint = "HTTP %s" % code
        if attempt < tries - 1:
            log("   重试 %d：%s" % (attempt + 1, hint))
            time.sleep(6 * (attempt + 1))
    return {"ok": False, "code": code, "type": ctype, "bytes": size,
            "magic": magic.decode("latin1", "replace"), "hint": hint}


def resolve(site, ident):
    """站点 + 标识 → (landing, pdf_url, note)"""
    cfg = SITES.get(site)
    if cfg is None:
        # 未知站点：尝试按 DOI 前缀推断
        site = DOI_PREFIX.get(ident.split("/")[0], "generic")
        cfg = SITES[site]
    landing = cfg["landing"].format(id=ident) if cfg.get("landing") else None
    pdf = cfg["pdf"].format(id=ident) if cfg.get("pdf") else None
    return landing, pdf, cfg.get("note"), site


def probe(site, ident):
    landing, pdf, note, real = resolve(site, ident)
    log("站点=%s  标识=%s" % (real, ident))
    if note:
        log("  备注: %s" % note)
    if not pdf:
        log("  该站无通用 PDF 模板 —— %s" % (note or "需从落地页解析"))
        return {"ok": False, "reason": "no_pattern"}
    log("  落地页: %s" % landing)
    log("  PDF:    %s" % pdf)
    jar = os.path.join(os.environ.get("TEMP", "."), "_probe_%s.txt" % real)
    warmup(landing, jar)
    out = os.path.join(os.environ.get("TEMP", "."), "_probe_%s.pdf" % real)
    r = download(pdf, landing, out, jar, tries=1)
    if r["ok"]:
        log("  ✅ 成功 %.2f MB  -> %s" % (r["bytes"] / 1048576, out))
    else:
        log("  ❌ 失败 http=%s type=%s magic=%r" % (r["code"], r["type"], r.get("magic")))
        log("     诊断: %s" % r.get("hint"))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest", nargs="?")
    ap.add_argument("--probe", nargs=2, metavar=("SITE", "ID"))
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    if args.probe:
        probe(args.probe[0], args.probe[1])
        return

    if not args.manifest:
        ap.error("需要 manifest 或 --probe")

    items = json.load(open(args.manifest, encoding="utf-8"))
    todo = []
    for it in items:
        out = it["out"]
        if (not args.force) and os.path.exists(out) and os.path.getsize(out) > 20000:
            with open(out, "rb") as fh:
                if fh.read(5) == b"%PDF-":
                    log("SKIP %s" % os.path.basename(out))
                    continue
        if os.path.exists(out):
            os.remove(out)
        todo.append(it)

    log("待下载 %d 篇（并发 %d）" % (len(todo), args.workers))

    def work(pair):
        idx, it = pair
        landing = it.get("landing")
        pdf = it.get("pdf")
        if not pdf:
            landing2, pdf2, note, real = resolve(it.get("site", "generic"), it["id"])
            landing = landing or landing2
            pdf = pdf or pdf2
            if not pdf:
                log("FAIL %-24s 无 PDF 模板（%s）" % (it.get("key", it["id"]), note))
                return {"key": it.get("key"), "ok": False, "hint": note}
        os.makedirs(os.path.dirname(it["out"]) or ".", exist_ok=True)
        jar = os.path.join(os.path.dirname(args.manifest), "_cj_%d.txt" % (idx % max(args.workers, 1)))
        t0 = time.time()
        warmup(landing, jar)
        r = download(pdf, landing, it["out"], jar)
        r["key"] = it.get("key")
        if r["ok"]:
            log("OK   %-24s %6.2f MB  %.1fs" % (r["key"], r["bytes"] / 1048576, time.time() - t0))
        else:
            log("FAIL %-24s http=%s type=%s | %s" % (r["key"], r["code"], r["type"], r.get("hint")))
        return r

    results = {}
    with cf.ThreadPoolExecutor(max_workers=args.workers) as ex:
        for r in ex.map(work, list(enumerate(todo))):
            results[r.get("key") or "?"] = r

    ok = sum(1 for v in results.values() if v.get("ok"))
    log("完成: %d/%d 成功" % (ok, len(results)))
    json.dump(results, open(args.manifest.replace(".json", "") + ".fetch.json", "w",
                            encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
