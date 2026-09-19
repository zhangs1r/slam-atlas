#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""make_compact.py — 生成紧凑技术对照索引（供撰写路线图时逐篇核对）

与 make_digest2.py 的区别：只保留"区分度最高"的信息——
  技术标签 + 作者自述的核心贡献/差异（原文摘录，不改写）
这样 255 篇能压到可一次通读的规模，且每条都能回溯原文。

用法: python make_compact.py [--out md/_compact.md] [--contrib-chars 420]
"""
import argparse
import glob
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from make_digest2 import (load, clean, strip_images, get_title, get_abstract,  # noqa: E402
                          get_contributions, get_method, tech_tags, datasets, get_sections)

ROOT = r"G:/project/ieeexplore"
MD_DIR = os.path.join(ROOT, "md")


def first_sentences(text, n=2, cap=380):
    """取前 n 句"""
    if not text:
        return ""
    parts = re.split(r"(?<=[.!?])\s+", text.strip())
    out = " ".join(parts[:n])
    return out[:cap]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(MD_DIR, "_compact.md"))
    ap.add_argument("--contrib-chars", type=int, default=420)
    args = ap.parse_args()

    files = sorted(f for f in glob.glob(os.path.join(MD_DIR, "*.md"))
                   if not os.path.basename(f).startswith("_"))
    recs = []
    for f in files:
        stem = os.path.basename(f)[:-3]
        t = load(f)
        year = stem[:4] if stem[:4].isdigit() else "0000"
        title = get_title(t) or stem.replace("_", " ")
        abstract = get_abstract(t)
        contrib = get_contributions(t, limit=args.contrib_chars * 2)
        method = get_method(t, limit=900)
        # ★ 技术标签只从「题名 + 摘要 + 贡献点 + 方法段」里判定。
        # 早期版本对全文打标签，会把参考文献里的词也算进来，
        # 导致 2000 年的 Bundle Adjustment 被误标为"3D高斯/神经隐式场"。
        scope = "\n".join([title, abstract, contrib, method])
        recs.append({
            "stem": stem, "year": year, "title": title,
            "tags": tech_tags(scope),
            "contrib": contrib,
            "method": method,
            "abstract": abstract,
            "ds": datasets(scope),
            "chars": len(t),
        })
    recs.sort(key=lambda x: (x["year"], x["stem"]))

    lines = ["# SLAM 语料库 · 紧凑技术对照索引", ""]
    lines.append("> **抽取式**：技术标签与文字均来自论文正文，未做生成式改写。")
    lines.append("> 每篇给出「作者自述的核心贡献/差异」，用于核对技术演进与代际区别。")
    lines.append("> 共 %d 篇，按年份排序。完整版见 `_digest.md`。" % len(recs))
    lines.append("")

    cur = None
    for r in recs:
        if r["year"] != cur:
            cur = r["year"]
            lines.append("\n---\n\n## %s 年\n" % cur)
        lines.append("### %s" % r["title"])
        meta = "`%s`" % r["stem"]
        if r["tags"]:
            meta += " · " + " / ".join(r["tags"])
        if r["ds"]:
            meta += " · 数据集: " + ", ".join(r["ds"][:6])
        lines.append("- %s" % meta)
        con = r["contrib"] or r["method"] or r["abstract"]
        if con:
            lines.append("- **自述差异**：%s" % first_sentences(con, n=3, cap=args.contrib_chars))
        lines.append("")

    open(args.out, "w", encoding="utf-8").write("\n".join(lines))
    n = len("\n".join(lines))
    print("compact -> %s  (%d 篇, %s 字符)" % (args.out, len(recs), format(n, ",")))


if __name__ == "__main__":
    main()
