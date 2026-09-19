#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""mineru_batch.py — 用 MinerU v4 API 把本地 PDF 批量转成 Markdown

流程: file-urls/batch 拿预签名上传地址 -> PUT 上传 -> extract/task/batch 提交
      -> 轮询 extract-results/batch/{id} -> 下载 zip -> 解出 full.md

用法:
  python mineru_batch.py <pdf_dir> <out_dir> [--filter REGEX] [--batch 10] [--lang en]
"""
import argparse
import io
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
import zipfile

TOKEN = os.environ.get("MINERU_API_TOKEN") or os.environ.get("MINERU_TOKEN")
BASE = "https://mineru.net/api/v4"
UA = "mineru-batch/1.0"


def log(msg):
    sys.stdout.write("[mineru] %s\n" % msg)
    sys.stdout.flush()


def http(method, url, data=None, headers=None, timeout=180, retries=3):
    h = {"User-Agent": UA}
    if headers:
        h.update(headers)
    body = None
    if data is not None:
        if isinstance(data, (dict, list)):
            body = json.dumps(data).encode("utf-8")
            h.setdefault("Content-Type", "application/json")
        elif isinstance(data, str):
            body = data.encode("utf-8")
        else:
            body = data
    last = None
    for attempt in range(retries):
        try:
            r = urllib.request.Request(url, data=body, headers=h, method=method)
            with urllib.request.urlopen(r, timeout=timeout) as resp:
                raw = resp.read()
                return resp.status, raw
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", "replace")[:400]
            last = "HTTP %s: %s" % (e.code, detail)
            if e.code in (429, 500, 502, 503, 504):
                time.sleep(3 * (attempt + 1))
                continue
            raise RuntimeError(last)
        except Exception as e:  # noqa
            last = str(e)
            time.sleep(2 * (attempt + 1))
    raise RuntimeError(last)


def api_json(method, path, payload=None):
    st, raw = http(method, BASE + path, payload,
                   headers={"Authorization": "Bearer %s" % TOKEN, "Accept": "application/json"})
    try:
        j = json.loads(raw.decode("utf-8"))
    except Exception:
        raise RuntimeError("non-json response (%s): %s" % (st, raw[:300]))
    if j.get("code") not in (0, 200, None):
        raise RuntimeError("api error: %s" % json.dumps(j, ensure_ascii=False)[:400])
    return j


def submit_batch(files, language="en"):
    """files: list of (name, bytes)"""
    payload = {
        "enable_formula": True,
        "enable_table": True,
        "language": language,
        "model_version": "pipeline",
        "files": [{"name": n, "is_ocr": False} for n, _ in files],
    }
    j = api_json("POST", "/file-urls/batch", payload)
    data = j.get("data") or {}
    urls = data.get("file_urls") or []
    batch_id = data.get("batch_id")
    if not urls or not batch_id:
        raise RuntimeError("no upload urls: %s" % json.dumps(j)[:300])
    for (name, blob), url in zip(files, urls):
        st, _ = http("PUT", url, data=blob, headers={"Content-Type": ""}, timeout=600, retries=3)
        log("uploaded %s (%d KB) -> %s" % (name, len(blob) // 1024, st))
    return batch_id


def poll_batch(batch_id, interval=15, max_wait=5400):
    t0 = time.time()
    last_state = None
    while True:
        j = api_json("GET", "/extract-results/batch/%s" % batch_id)
        results = ((j.get("data") or {}).get("extract_result")) or []
        states = {}
        for r in results:
            states[r.get("file_name")] = r.get("state")
        summary = "%d/%d done" % (sum(1 for s in states.values() if s == "done"), len(states))
        if summary != last_state:
            log("progress %s  %s" % (summary, json.dumps(states, ensure_ascii=False)))
            last_state = summary
        if results and all(r.get("state") in ("done", "failed") for r in results):
            return results
        if time.time() - t0 > max_wait:
            raise RuntimeError("poll timeout after %ds" % max_wait)
        time.sleep(interval)


def download_zip(url, timeout=600):
    st, raw = http("GET", url, timeout=timeout)
    return raw


def safe_name(name):
    base = re.sub(r"\.pdf$", "", name, flags=re.I)
    return re.sub(r'[\\/:*?"<>|\s]+', "_", base)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pdf_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--filter", default=None)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--lang", default="en")
    ap.add_argument("--skip-existing", action="store_true")
    ap.add_argument("--only", default=None,
                    help="只处理指定文件名（逗号分隔），用于小样测试")
    args = ap.parse_args()

    if not TOKEN:
        log("ERROR: MINERU_API_TOKEN / MINERU_TOKEN not set")
        sys.exit(2)

    names = sorted(f for f in os.listdir(args.pdf_dir) if f.lower().endswith(".pdf"))
    if args.filter:
        rx = re.compile(args.filter, re.I)
        names = [n for n in names if rx.search(n)]
    if args.only:
        want = {x.strip() for x in args.only.split(",") if x.strip()}
        names = [n for n in names if n in want or safe_name(n) in want]
    if args.skip_existing:
        names = [n for n in names
                 if not os.path.exists(os.path.join(args.out_dir, safe_name(n) + ".md"))]
    log("to convert: %d pdf(s)" % len(names))
    os.makedirs(args.out_dir, exist_ok=True)

    manifest_path = os.path.join(args.out_dir, "_mineru_manifest.json")
    manifest = {}
    if os.path.exists(manifest_path):
        try:
            manifest = json.load(open(manifest_path, encoding="utf-8"))
        except Exception:
            manifest = {}

    for i in range(0, len(names), args.batch):
        chunk = names[i:i + args.batch]
        files = []
        for n in chunk:
            with open(os.path.join(args.pdf_dir, n), "rb") as fh:
                files.append((n, fh.read()))
        log("--- batch %d: %s" % (i // args.batch + 1, ", ".join(chunk)))
        batch_id = submit_batch(files, language=args.lang)
        log("batch_id=%s" % batch_id)
        results = poll_batch(batch_id)
        for r in results:
            fn = r.get("file_name")
            state = r.get("state")
            if state != "done":
                log("FAILED %s: %s" % (fn, r.get("err_msg")))
                manifest[fn] = {"state": "failed", "err": r.get("err_msg")}
                continue
            zurl = r.get("full_zip_url")
            try:
                blob = download_zip(zurl)
                zf = zipfile.ZipFile(io.BytesIO(blob))
                md_member = None
                for m in zf.namelist():
                    if m.endswith("full.md"):
                        md_member = m
                        break
                if not md_member:
                    log("no full.md in zip for %s" % fn)
                    manifest[fn] = {"state": "failed", "err": "no full.md"}
                    continue
                text = zf.read(md_member).decode("utf-8", "replace")
                stem = safe_name(fn)

                # ---- 图片处理 ----
                # MinerU 的 zip 内路径形如 "images/<hash>.jpg"（没有前导斜杠！）
                # 早期版本误用 "/images/" 判断，导致一张图都没存下来。
                # 统一存到 <out_dir>/images/<paper>/ ，并把 md 里的相对路径改写过去。
                img_dir = os.path.join(args.out_dir, "images", stem)
                n_img = 0
                for m in zf.namelist():
                    if m.endswith("/"):
                        continue
                    parts = m.replace("\\", "/").split("/")
                    if "images" not in parts[:-1]:
                        continue
                    os.makedirs(img_dir, exist_ok=True)
                    with open(os.path.join(img_dir, os.path.basename(m)), "wb") as ih:
                        ih.write(zf.read(m))
                    n_img += 1
                if n_img and "images/%s/" % stem not in text:
                    text = re.sub(r"(!\[[^\]]*\]\()images/", r"\1images/%s/" % stem, text)

                out_md = os.path.join(args.out_dir, stem + ".md")
                with open(out_md, "w", encoding="utf-8") as fh:
                    fh.write(text)
                manifest[fn] = {"state": "done", "md": out_md, "chars": len(text),
                                "images": n_img,
                                "zip": zurl,
                                "batch_id": batch_id,
                                "pages": (r.get("extract_progress") or {}).get("extracted_pages")}
                log("OK %s -> %d chars, %d images" % (fn, len(text), n_img))
            except Exception as e:  # noqa
                log("download/extract failed for %s: %s" % (fn, e))
                manifest[fn] = {"state": "failed", "err": str(e)}
        json.dump(manifest, open(manifest_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    ok = sum(1 for v in manifest.values() if v.get("state") == "done")
    log("finished. ok=%d failed=%d" % (ok, sum(1 for v in manifest.values() if v.get("state") == "failed")))


if __name__ == "__main__":
    main()
