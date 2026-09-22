#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""fffd_report.py — 扫描语料库里 OCR 丢失的符号（U+FFFD），列出位置与上下文

背景：PDF 经 MinerU 转 Markdown 时，部分希腊字母/变量名识别失败，被写成 U+FFFD（�）。
这些位置的符号**无法从上下文唯一确定**，必须对照原始 PDF 才能安全回填。
猜一个错的符号 = 往语料里注入错误但看起来正常的内容，比留着 � 更糟。

用法:
  python tools/fffd_report.py                 # 汇总 + 逐处列出上下文
  python tools/fffd_report.py --count         # 只报数量（退出码非 0 表示还有残留）
  python tools/fffd_report.py --table out.md  # 生成待核对照表（Markdown）
  python tools/fffd_report.py --file md/2023_3DGS.md   # 只看某篇

注意：course/lessons/*.html 里的 U+FFFD 是**故意保留的 OCR 错样引用**，
     不在本脚本的扫描范围内，也不要修改。
"""
import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MD_DIR = os.path.join(ROOT, 'md')
FFFD = '\ufffd'
TAG = re.compile(r'<[^>]+>')

# 基于论文通用记法的粗略建议（仅供检索参考，**不可**直接采信）
GUESS = [
    (r'blend|Blend|opacity|Opacity', 'α'),
    (r'color|Color|radiance', 'C / c'),
    (r'hash table size|table size|Hash table|parameters', 'T'),
    (r'resolution level|number of levels|\blevels\b', 'L'),
    (r'dimension|dimensional', 'F / d'),
    (r'SH band|band ', 'l'),
    (r'\bview\b|view,|view\)|view ', 'v / V'),
    (r'pixel', 'k / P'),
    (r'primitive|around|Gaussian\b', 'g / i / N'),
    (r'bufers', 'z'),
    (r'Jacobian', 'J'),
    (r'quaternion', 'q'),
]


def iter_targets(only=None):
    if only:
        yield only
        return
    for f in sorted(os.listdir(MD_DIR)):
        if not f.endswith('.md'):
            continue
        p = os.path.join('md', f)
        if FFFD in open(os.path.join(ROOT, p), encoding='utf-8', errors='replace').read():
            yield p


def scan(path):
    full = os.path.join(ROOT, path)
    s = open(full, encoding='utf-8', errors='replace').read()
    out = []
    for m in re.finditer(FFFD + r'+', s):
        line = s[:m.start()].count('\n') + 1
        a, b = max(0, m.start() - 46), min(len(s), m.end() + 46)
        ctx = TAG.sub('', s[a:b].replace('\n', ' '))
        out.append((line, ctx))
    return s.count(FFFD), out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', action='store_true', help='只输出数量')
    ap.add_argument('--table', metavar='OUT', help='生成 Markdown 对照表')
    ap.add_argument('--file', help='只处理指定文件（相对仓库根，如 md/2023_3DGS.md）')
    args = ap.parse_args()

    targets = list(iter_targets(args.file))
    total = sum(scan(p)[0] for p in targets)

    if args.count:
        for p in targets:
            print(f'{scan(p)[0]:5}  {p}')
        print(f'合计残留 {total} 处 U+FFFD')
        sys.exit(0 if total == 0 else 1)

    lines = []
    if args.table:
        lines += ['# 语料库 OCR 待核对照表', '',
                  'PDF 经 MinerU 转 Markdown 时丢失的符号（U+FFFD）。这些位置**无法从上下文唯一确定**，',
                  '需对照原始 PDF（`papers/<同名>.pdf`）逐处核定后再回填。', '',
                  '> 判定原则：宁缺勿错。`�` 是诚实的"此处识别失败"；猜一个错的符号则是静默篡改语料。',
                  '', '| 文件 | 行号 | 上下文 | 扫描建议 | 核定后的符号 |', '|---|---|---|---|---|']
    for p in targets:
        n, hits = scan(p)
        if args.table:
            for line, ctx in hits:
                g = next((sym for pat, sym in GUESS if re.search(pat, ctx)), '')
                lines.append(f'| `{p}` | {line} | …{ctx.replace("|", "\\|")}… | {g} | |')
            continue
        print(f'\n{"=" * 78}\n{p}  （{n} 处）\n{"=" * 78}')
        for line, ctx in hits:
            g = next((sym for pat, sym in GUESS if re.search(pat, ctx)), '')
            print(f'  L{line:<5} …{ctx}…' + (f'   [建议 {g}]' if g else ''))
    if args.table:
        lines += ['', f'合计 **{total}** 处待核定。', '']
        open(args.table, 'w', encoding='utf-8').write('\n'.join(lines))
        print(f'已写入 {args.table}（{total} 处）')
    else:
        print(f'\n合计 {total} 处 U+FFFD')


if __name__ == '__main__':
    main()
