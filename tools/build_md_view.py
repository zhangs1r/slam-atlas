#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""build_md_view.py — 把仓库里的 Markdown 预渲染成静态 HTML，供站内直接阅读

为什么预渲染而不是前端渲染：
  沙箱与国内网络都拿不到 marked.js / KaTeX（npm、jsdelivr、unpkg 全不可达），
  而 GitHub Pages 对 .md 返回 text/markdown，浏览器会下载而不是渲染。
  所以把 Markdown 在构建期转成 HTML，公式转 MathML（浏览器原生渲染），
  全站零 JS 依赖、零 CDN 依赖，离线可用。

三个数据源：
  papers   md/*.md                        → md/html/*.html          （255 篇论文原文）
  records  course/learning-records/*.md   → course/learning-records/html/*.html（学习记录）
  outputs  outputs/*.md                   → outputs/html/*.html     （产出文档）

用法:
  python tools/build_md_view.py               # 全部渲染
  python tools/build_md_view.py papers        # 只渲染某一组

依赖: pip install markdown latex2mathml
"""
import argparse
import json
import os
import re
import sys
import html as _html

try:
    import markdown
    from latex2mathml.converter import convert as tex2mml
except ImportError:
    sys.exit('缺依赖：pip install markdown latex2mathml')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
FFFD = '\ufffd'

# ── 三组数据源的配置 ────────────────────────────────────────────────
# src/out 相对仓库根；home/index/src_back 是生成页面里的相对链接
SETS = {
    'papers': dict(
        src='md', out='md/html', title_suffix='SLAM 论文原文',
        home='../../course/index.html', home_label='课程馆',
        index='../index.html', index_label='论文原文索引',
        src_back='../{stem}.md', img_prefix='../',
        banner_label='本文件由 OCR 生成',
    ),
    'records': dict(
        src='course/learning-records', out='course/learning-records/html',
        title_suffix='SLAM 学习记录',
        home='../../index.html', home_label='课程馆',
        index='../../../md/index.html', index_label='论文原文索引',
        src_back='../{stem}.md', img_prefix='../../md/',
        banner_label='本文件由 OCR 语料整理而来',
    ),
    'outputs': dict(
        src='outputs', out='outputs/html', title_suffix='SLAM Atlas 产出',
        home='../../course/index.html', home_label='课程馆',
        index='../../md/index.html', index_label='论文原文索引',
        src_back='../{stem}.md', img_prefix='../../md/',
        banner_label='本文件由 OCR 语料整理而来',
    ),
}

TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title} — {suffix}</title>
<link rel="stylesheet" href="{theme}">
<style>
  .bar{{position:sticky;top:0;z-index:5;background:var(--paper-light);border-bottom:1px solid var(--border);
    padding:10px 18px;display:flex;gap:16px;flex-wrap:wrap;font-size:13.5px;align-items:center;}}
  .bar .sp{{flex:1;}}
  main{{max-width:var(--reading-width);margin:0 auto;padding:28px 20px 80px;}}
  main.wide{{max-width:var(--content-width);}}
  img{{max-width:100%;height:auto;display:block;margin:1.2em auto;border:1px solid var(--border);border-radius:var(--radius-md);}}
  table{{display:block;overflow-x:auto;}}
  math{{font-size:1.06em;}}
  .math-block{{overflow-x:auto;margin:1.3em 0;text-align:center;}}
  .math-raw{{background:var(--code-bg);border-left:3px solid var(--accent);padding:8px 12px;font-size:13px;overflow-x:auto;font-family:var(--font-mono);}}
  .ocrmiss{{background:var(--accent-dim);color:var(--accent-deep);border-radius:3px;padding:0 2px;cursor:help;}}
  .banner{{background:var(--accent-dim);border:1px solid var(--accent-light);border-radius:var(--radius-md);
    padding:10px 14px;font-size:13.5px;color:var(--ink-80);margin:0 0 18px;}}
  .doc-foot{{margin-top:48px;padding-top:18px;border-top:1px solid var(--border);font-size:13px;color:var(--ink-40);}}
</style>
</head>
<body>
<div class="bar">
  <a href="{home}">← {home_label}</a>
  <a href="{index}">{index_label}</a>
  <span class="sp"></span>
  <a href="{src_back}" download>下载 .md 源文件</a>
</div>
<main{wide}>
{banner}
{content}
<div class="doc-foot">本页由 <code>{src_path}</code> 预渲染生成。公式以 MathML 呈现，浏览器原生渲染，不依赖外部脚本或字体 CDN。</div>
</main>
</body>
</html>
"""

BLOCK, INLINE = [], []
stat = {'block_ok': 0, 'block_fail': 0, 'inline_ok': 0, 'inline_fail': 0}


def _stash_block(m):
    BLOCK.append(m.group(1))
    return f'\n\n@@BLK{len(BLOCK) - 1}@@\n\n'


def _stash_inline(m):
    INLINE.append(m.group(1))
    return f'@@INL{len(INLINE) - 1}@@'


def render_block(tex):
    t = tex.strip()
    tag = ''
    mt = re.search(r'\\tag\{([^}]*)\}\s*$', t)
    if mt:
        tag, t = mt.group(1), t[:mt.start()].strip()
    try:
        xml = tex2mml(t, display='block')
        stat['block_ok'] += 1
    except Exception:
        stat['block_fail'] += 1
        return '<pre class="math-raw">' + _html.escape(tex.strip()) + '</pre>'
    label = f'<span style="float:right;color:var(--ink-40)">({tag})</span>' if tag else ''
    return f'<div class="math-block">{xml}{label}</div>'


def render_inline(tex):
    try:
        xml = tex2mml(tex.strip(), display='inline')
        stat['inline_ok'] += 1
        return xml
    except Exception:
        stat['inline_fail'] += 1
        return '<code>' + _html.escape('$' + tex + '$') + '</code>'


def convert_md(text):
    text = re.sub(r'\$\$(.+?)\$\$', _stash_block, text, flags=re.S)
    text = re.sub(r'(?<!\$)\$([^$\n]{1,150})\$(?!\$)', _stash_inline, text)
    md = markdown.Markdown(extensions=['tables', 'fenced_code', 'sane_lists', 'attr_list'])
    h = md.convert(text)
    h = re.sub(r'@@BLK(\d+)@@', lambda m: render_block(BLOCK[int(m.group(1))]), h)
    h = re.sub(r'@@INL(\d+)@@', lambda m: render_inline(INLINE[int(m.group(1))]), h)
    return h


def build_one(name, cfg, fname):
    stem = fname[:-3]
    path = os.path.join(cfg['src'], fname)
    src = open(path, encoding='utf-8').read()
    BLOCK.clear(); INLINE.clear()
    body = convert_md(src)
    body = body.replace('src="images/', 'src="' + cfg['img_prefix'] + 'images/')
    body = body.replace('](images/', '](' + cfg['img_prefix'] + 'images/')

    miss = src.count(FFFD)
    if miss:
        body = body.replace(FFFD, '<span class="ocrmiss" title="OCR 丢失的符号，需对照原 PDF">' + FFFD + '</span>')
        banner = (f'<div class="banner">⚠️ {cfg["banner_label"]}，仍有 <b>{miss}</b> 处符号在识别中丢失'
                  f'（标为 <span class="ocrmiss">{FFFD}</span>）。课里的技术结论已由课程勘误，'
                  f'此处保留原样以便对照。</div>')
    else:
        banner = ''

    m = re.search(r'^#\s+(.+)$', src, re.M)
    title = _html.escape(re.sub(r'<[^>]+>', '', m.group(1)).strip() if m else stem)
    depth = len(os.path.normpath(cfg['out']).split(os.sep))
    theme = '../' * depth + 'course/shared/theme.css'
    out = os.path.join(cfg['out'], stem + '.html')
    open(out, 'w', encoding='utf-8', newline='').write(TEMPLATE.format(
        title=title, suffix=cfg['title_suffix'],
        theme=theme,
        home=cfg['home'], home_label=cfg['home_label'],
        index=cfg['index'], index_label=cfg['index_label'],
        src_back=cfg['src_back'].format(stem=stem),
        src_path=path.replace('\\', '/'),
        wide=' class="wide"' if name == 'outputs' else '',
        banner=banner, content=body))
    return {'set': name, 'stem': stem, 'miss': miss, 'out': out}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sets', nargs='*', default=None, choices=[None] + list(SETS))
    args = ap.parse_args()
    names = args.sets or list(SETS)
    report = []

    for name in names:
        cfg = SETS[name]
        if not os.path.isdir(cfg['src']):
            print(f'跳过 {name}：{cfg["src"]} 不存在')
            continue
        os.makedirs(cfg['out'], exist_ok=True)
        files = sorted(f for f in os.listdir(cfg['src']) if f.endswith('.md'))
        miss_total = 0
        for f in files:
            rec = build_one(name, cfg, f)
            miss_total += rec['miss']
            report.append(rec)
        size = sum(os.path.getsize(os.path.join(cfg['out'], x)) for x in os.listdir(cfg['out']))
        print(f'{name:8} {len(files):3} 页 → {cfg["out"]}/  {size / 1048576:.1f} MB'
              + (f'   （含 U+FFFD {miss_total} 处，已在页内标注）' if miss_total else ''))

    print(f'公式：块级 {stat["block_ok"]} 成功 / {stat["block_fail"]} 降级；'
          f'行内 {stat["inline_ok"]} 成功 / {stat["inline_fail"]} 降级')

    # 渲染报告：Linux 审计沙箱里写 /var/minis/...；本机（Windows 等）落到仓库内 _tmp/
    # 注意：Windows 上 '/var/...' 会被解析成「当前盘符:\var\...」，所以必须先判平台，不能靠 except
    _candidates = []
    if os.name != 'nt':
        _candidates.append('/var/minis/workspace/slam-atlas-audit/render_report.json')
    _candidates.append(os.path.join(ROOT, '_tmp', 'render_report.json'))
    for _p in _candidates:
        try:
            os.makedirs(os.path.dirname(_p), exist_ok=True)
            with open(_p, 'w', encoding='utf-8') as fh:
                json.dump(report, fh, ensure_ascii=False)
            print(f'渲染报告写到 {_p}')
            break
        except OSError as e:
            print(f'（渲染报告写不进 {_p}：{e}）')


if __name__ == '__main__':
    main()
