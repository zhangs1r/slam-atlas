#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""fix_view_details.py — 渲染细节修补

1. 对照表这类"本身就包含 U+FFFD"的文档不显示 OCR 提示条（那是它的内容，不是缺陷）；
2. 生成 outputs/index.html 产出索引页。
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)

# ① 对照表页去掉误导性的 OCR 提示条
SKIP_BANNER = ['语料OCR待核对照表']
for st in SKIP_BANNER:
    p = f'outputs/html/{st}.html'
    if os.path.exists(p):
        s = open(p, encoding='utf-8').read()
        s2 = re.sub(r'<div class="banner">.*?</div>\n', '', s, count=1, flags=re.S)
        if s2 != s:
            open(p, 'w', encoding='utf-8').write(s2)
            print(f'已移除提示条：{p}')

# ② 生成产出索引
ITEMS = [
    ('论文阅读路线图.html', '🗺️', 'SLAM 论文阅读路线图（交互版）',
     '255 篇论文按阶段 A–K 编排，带时间轴、技术分类筛选、难度阈值与实时搜索。'),
    ('html/论文阅读路线图.html', '📋', '论文阅读路线图（静态版）',
     '同一份路线图的 Markdown 渲染版，适合通读与打印。'),
    ('SLAM图解入门.html', '🎨', 'SLAM 图解入门',
     '用图解把 SLAM 的整体脉络讲一遍，适合零基础先建立直观印象。'),
    ('SLAM技术演进图谱.html', '🧭', 'SLAM 技术演进图谱',
     '按时间轴铺开的演进图谱，一屏看清各条技术线怎么长出来的。'),
    ('SLAM技术演进详解.html', '🔍', 'SLAM 技术演进详解',
     '演进图谱的文字详解版，逐个节点说明它解决了前人的什么问题。'),
    ('html/SLAM技术发展路线.html', '🛤️', 'SLAM 技术发展路线',
     '分阶段的技术发展路线整理。'),
    ('html/经典论文补充分析.html', '📚', '经典论文补充分析',
     '路线图之外的经典论文补充阅读笔记。'),
    ('html/论文语料库总目录.html', '🗂️', '论文语料库总目录',
     '255 篇语料库的全量目录，按条目检索。'),
    ('跨出版商下载能力实测报告.html', '🧪', '跨出版商下载能力实测报告',
     '12 家出版商的可下载性实测：三档分类、判定依据与推荐路径。'),
    ('html/语料OCR待核对照表.html', '🔤', '语料 OCR 待核对照表',
     'PDF 转 Markdown 时丢失的符号（U+FFFD）逐处清单，待对照原文 PDF 核定。'),
]

cards = []
for path, emoji, title, desc in ITEMS:
    ok = os.path.exists(os.path.join('outputs', path))
    cards.append(
        f'<a class="card" href="{path}">'
        f'<span class="emoji">{emoji}</span>'
        f'<h3>{title}</h3><p>{desc}</p>'
        f'</a>' if ok else
        f'<div class="card disabled"><span class="emoji">{emoji}</span>'
        f'<h3>{title}</h3><p>{desc}</p><p class="miss">文件缺失</p></div>')

html = '''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>相关产出与原始素材 — Slam Atlas</title>
<link rel="stylesheet" href="../course/shared/theme.css">
<style>
  .bar{position:sticky;top:0;z-index:9;display:flex;gap:16px;align-items:center;
    padding:10px 18px;background:var(--paper-light);border-bottom:1px solid var(--border);
    font-size:13.5px;}
  .bar a{color:var(--accent-deep);text-decoration:none;}
  .bar a:hover{text-decoration:underline;}
  main{max-width:var(--content-width);margin:0 auto;padding:36px 22px 90px;}
  h1{font-size:var(--text-4xl);margin-bottom:var(--space-3);}
  .lede{color:var(--ink-60);margin-bottom:var(--space-8);font-size:var(--text-lg);}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:var(--space-5);}
  .card{display:block;background:var(--paper-light);border:1px solid var(--border);
    border-radius:var(--radius-lg);padding:var(--space-5);text-decoration:none;
    transition:box-shadow .2s,transform .15s,border-color .2s;}
  .card:hover{box-shadow:var(--shadow-md);transform:translateY(-2px);border-color:var(--accent-light);text-decoration:none;}
  .card .emoji{font-size:22px;}
  .card h3{font-size:var(--text-lg);margin:var(--space-2) 0 var(--space-2);color:var(--ink);}
  .card p{font-size:var(--text-sm);color:var(--ink-60);margin:0;line-height:1.65;}
  .card.disabled{opacity:.5;}
  .card .miss{color:var(--accent-deep);margin-top:var(--space-2);}
</style>
</head>
<body>
<div class="bar">
  <a href="../course/index.html">← 课程馆</a>
  <a href="../md/index.html">论文原文索引</a>
</div>
<main>
  <h1>相关产出与原始素材</h1>
  <p class="lede">课程之外的配套材料：路线图、演进图谱、图解入门、语料库目录与实测报告。</p>
  <div class="grid">
''' + '\n'.join('    ' + c for c in cards) + '''
  </div>
</main>
</body>
</html>
'''
open('outputs/index.html', 'w', encoding='utf-8').write(html)
print('已生成 outputs/index.html')
missing = [p for p, *_ in ITEMS if not os.path.exists(os.path.join('outputs', p))]
print('指向缺失文件：' + (', '.join(missing) if missing else '无'))
