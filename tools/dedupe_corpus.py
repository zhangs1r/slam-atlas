#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""dedupe_corpus.py — 按标题去重语料库

不同解析路径可能给同一篇论文生成两个不同文件名
（如 2007_PTAM 与 2007_Parallel_Tracking_and_Mapping_for_Small_AR_Workspaces）。
保留字符数更多的那份，删除重复的 pdf / md / images。

用法: python dedupe_corpus.py [--apply]
"""
import argparse
import glob
import json
import os
import re
import sys

ROOT = r"G:/project/ieeexplore"

sys.path.insert(0, os.path.join(ROOT, "tools"))
from matchutil import title_sim  # noqa: E402


def log(m):
    sys.stdout.write("[dedupe] %s\n" % m)
    sys.stdout.flush()


def title_of(md_path):
    try:
        t = open(md_path, encoding="utf-8", errors="replace").read(500)
        m = re.match(r"\s*#+\s*(.+)", t)
        return m.group(1).strip() if m else ""
    except Exception:
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="真正执行删除（默认只报告）")
    ap.add_argument("--threshold", type=float, default=0.90)
    args = ap.parse_args()

    mds = sorted(f for f in glob.glob(os.path.join(ROOT, "md/*.md"))
                 if not os.path.basename(f).startswith("_"))
    items = []
    for f in mds:
        stem = os.path.basename(f)[:-3]
        items.append({"stem": stem, "md": f, "title": title_of(f),
                      "chars": os.path.getsize(f)})

    # 完全同名标题直接分组
    groups, used = [], set()
    for i, a in enumerate(items):
        if a["stem"] in used:
            continue
        g = [a]
        used.add(a["stem"])
        for b in items[i + 1:]:
            if b["stem"] in used:
                continue
            ta = a["title"] or a["stem"].replace("_", " ")
            tb = b["title"] or b["stem"].replace("_", " ")
            if title_sim(ta, tb) >= args.threshold:
                g.append(b)
                used.add(b["stem"])
        if len(g) > 1:
            groups.append(g)

    log("疑似重复组 %d 组，共 %d 篇" % (len(groups), sum(len(g) for g in groups)))
    removed = []
    for g in groups:
        g.sort(key=lambda x: -x["chars"])
        keep, drop = g[0], g[1:]
        log("  保留: %-56s (%s 字符)" % (keep["stem"][:56], format(keep["chars"], ",")))
        log("     标题: %s" % (keep["title"] or "-")[:88])
        for d in drop:
            log("  删除: %-56s (%s 字符)" % (d["stem"][:56], format(d["chars"], ",")))
            removed.append(d["stem"])
            if args.apply:
                for p in (os.path.join(ROOT, "md", d["stem"] + ".md"),
                          os.path.join(ROOT, "papers", d["stem"] + ".pdf")):
                    if os.path.exists(p):
                        os.remove(p)
                img = os.path.join(ROOT, "md/images", d["stem"])
                if os.path.isdir(img):
                    for f in os.listdir(img):
                        os.remove(os.path.join(img, f))
                    os.rmdir(img)

    log("\n%s %d 篇重复" % ("已删除" if args.apply else "计划删除（加 --apply 执行）", len(removed)))
    json.dump({"removed": removed, "groups": [[x["stem"] for x in g] for g in groups]},
              open(os.path.join(ROOT, "meta/dedupe_report.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
