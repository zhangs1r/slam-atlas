#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P1-4：把 md/ 语料预渲染成静态 HTML（md/html/），不依赖任何 CDN / JS。

- Markdown → HTML（markdown 库）
- $$...$$ 与 $...$ → MathML（latex2mathml），浏览器原生渲染，无需 KaTeX
- 图片路径 images/ → ../images/
- U+FFFD 标成可见提示，并在页首给出说明
"""
import os, re, sys, html as _html, json, collections
import markdown
from latex2mathml.converter import convert as tex2mml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
SRC, OUT = 'md', 'md/html'
os.makedirs(OUT, exist_ok=True)

TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title} — SLAM 论文原文</title>
<style>
  :root{{--paper:#FBF9F5;--ink:#2E2A26;--ink2:#8A8178;--terra:#CC785C;--rule:#E4DCD0;}}
  *{{box-sizing:border-box;}}
  body{{margin:0;background:var(--paper);color:var(--ink);
    font-family:Inter,-apple-system,BlinkMacSystemFont,"PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;
    line-height:1.85;font-size:16px;}}
  .bar{{position:sticky;top:0;z-index:5;background:rgba(251,249,245,.94);backdrop-filter:blur(8px);
    border-bottom:1px solid var(--rule);padding:10px 18px;display:flex;gap:16px;flex-wrap:wrap;font-size:13.5px;}}
  .bar a{{color:var(--terra);text-decoration:none;}}
  .bar a:hover{{text-decoration:underline;}}
  .bar .sp{{flex:1;}}
  main{{max-width:820px;margin:0 auto;padding:28px 20px 80px;}}
  h1,h2,h3,h4{{font-family:"Noto Serif SC","Songti SC",Georgia,serif;line-height:1.35;}}
  h1{{font-size:26px;margin:.4em 0 .8em;}}
  h2{{font-size:21px;margin:1.8em 0 .6em;padding-top:.4em;border-top:1px solid var(--rule);}}
  h3{{font-size:17.5px;margin:1.4em 0 .5em;}}
  p{{margin:.85em 0;}}
  img{{max-width:100%;height:auto;display:block;margin:1.2em auto;border:1px solid var(--rule);border-radius:6px;}}
  table{{border-collapse:collapse;width:100%;margin:1.2em 0;font-size:14.5px;display:block;overflow-x:auto;}}
  th,td{{border:1px solid var(--rule);padding:6px 10px;text-align:left;}}
  th{{background:#F4EFE7;}}
  code{{background:#F4EFE7;padding:.1em .35em;border-radius:4px;font-size:.9em;}}
  pre{{background:#F4EFE7;padding:12px;border-radius:8px;overflow-x:auto;}}
  pre code{{background:none;padding:0;}}
  blockquote{{border-left:3px solid var(--rule);margin:1em 0;padding:.2em 0 .2em 14px;color:var(--ink2);}}
  math{{font-size:1.06em;}}
  .math-block{{overflow-x:auto;margin:1.3em 0;text-align:center;}}
  .math-raw{{background:#FDF3F0;border-left:3px solid var(--terra);padding:8px 12px;font-size:13px;overflow-x:auto;}}
  .ocrmiss{{background:#FDE8E2;color:#B3654B;border-radius:3px;padding:0 2px;cursor:help;}}
  .banner{{background:#FDF3F0;border:1px solid #F0D3C9;border-radius:8px;padding:10px 14px;
    font-size:13.5px;color:#8A5A46;margin:0 0 18px;}}
  .foot{{margin-top:48px;padding-top:18px;border-top:1px solid var(--rule);font-size:13px;color:var(--ink2);}}
</style>
</head>
<body>
<div class="bar">
  <a href="../../course/index.html">← 课程馆</a>
  <a href="../index.html">论文原文索引</a>
  <span class="sp"></span>
  <a href="../{stem}.md" download>下载 .md 源文件</a>
</div>
<main>
{banner}
{content}
<div class="foot">本页由 <code>md/{stem}.md</code> 预渲染生成（OCR 语料，插图取自原文）。
公式以 MathML 呈现，浏览器原生渲染，不依赖外部脚本或字体 CDN。</div>
</main>
</body>
</html>
"""

BLOCK = []
INLINE = []
blocks_fail = inlines_fail = 0
blocks_ok = inlines_ok = 0


def render_block(tex):
    global blocks_fail, blocks_ok
    t = tex.strip()
    tag = ''
    mt = re.search(r'\\tag\{([^}]*)\}\s*$', t)
    if mt:
        tag = mt.group(1)
        t = t[:mt.start()].strip()
    try:
        xml = tex2mml(t, display='block')
        blocks_ok += 1
    except Exception:
        blocks_fail += 1
        return '<pre class="math-raw">' + _html.escape(tex.strip()) + '</pre>'
    label = f'<span style="float:right;color:#8A8178">({tag})</span>' if tag else ''
    return f'<div class="math-block">{xml}{label}</div>'


def render_inline(tex):
    global inlines_fail, inlines_ok
    try:
        xml = tex2mml(tex.strip(), display='inline')
        inlines_ok += 1
        return xml
    except Exception:
        inlines_fail += 1
        return '<code>' + _html.escape('$' + tex + '$') + '</code>'


def stash_block(m):
    BLOCK.append(m.group(1))
    return f'\n\n@@BLK{len(BLOCK) - 1}@@\n\n'


def stash_inline(m):
    INLINE.append(m.group(1))
    return f'@@INL{len(INLINE) - 1}@@'


def convert_md(text):
    text = re.sub(r'\$\$(.+?)\$\$', stash_block, text, flags=re.S)
    text = re.sub(r'(?<!\$)\$([^$\n]{1,150})\$(?!\$)', stash_inline, text)
    md = markdown.Markdown(extensions=['tables', 'fenced_code', 'sane_lists', 'attr_list'])
    h = md.convert(text)
    h = re.sub(r'@@BLK(\d+)@@', lambda m: render_block(BLOCK[int(m.group(1))]), h)
    h = re.sub(r'@@INL(\d+)@@', lambda m: render_inline(INLINE[int(m.group(1))]), h)
    return h


files = sorted(f for f in os.listdir(SRC) if f.endswith('.md'))
made = 0
miss_total = 0
report = []
for f in files:
    stem = f[:-3]
    src = open(os.path.join(SRC, f), encoding='utf-8').read()
    # 复位全局暂存
    BLOCK.clear(); INLINE.clear()
    body = convert_md(src)
    body = body.replace('src="images/', 'src="../images/')
    body = body.replace('](images/', '](../images/')
    n_miss = src.count('\ufffd')
    miss_total += n_miss
    if n_miss:
        body = body.replace('\ufffd', '<span class="ocrmiss" title="OCR 丢失的符号，需对照原 PDF">\ufffd</span>')
        banner = (f'<div class="banner">⚠️ 本文件由 OCR 生成，仍有 <b>{n_miss}</b> 处符号在识别中丢失'
                  f'（标为 <span class="ocrmiss">\ufffd</span>）。课里的技术结论已由课程勘误，'
                  f'此处保留原样以便对照。</div>')
    else:
        banner = ''
    m = re.search(r'^#\s+(.+)$', src, re.M)
    title = _html.escape(m.group(1).strip() if m else stem)
    open(os.path.join(OUT, stem + '.html'), 'w', encoding='utf-8', newline='').write(
        TEMPLATE.format(title=title, stem=stem, content=body, banner=banner))
    made += 1
    report.append({'stem': stem, 'miss': n_miss, 'size': len(body)})

size = sum(os.path.getsize(os.path.join(OUT, x)) for x in os.listdir(OUT))
print(f'生成 {made} 个页面 → {OUT}/，合计 {size / 1048576:.1f} MB')
print(f'公式：块级成功 {blocks_ok} / 失败 {blocks_fail}；行内成功 {inlines_ok} / 失败 {inlines_fail}')
print(f'含 U+FFFD 的源文件仍剩 {miss_total} 处（已在页面标注）')
# 渲染报告：Linux 审计沙箱里写 /var/minis/...；本机（Windows 等）落到仓库内 _tmp/
# 注意：Windows 上 '/var/...' 会被解析成「当前盘符:\var\...」，所以必须先判平台，不能靠 except
_candidates = []
if os.name != 'nt':
    _candidates.append('/var/minis/workspace/slam-atlas-audit/render_report.json')
_candidates.append(os.path.join(ROOT, '_tmp', 'render_report.json'))
_report_path = _candidates[-1]
for _p in _candidates:
    try:
        os.makedirs(os.path.dirname(_p), exist_ok=True)
        with open(_p, 'w', encoding='utf-8') as fh:
            json.dump(report, fh, ensure_ascii=False)
        _report_path = _p
        break
    except OSError as e:
        print(f'（渲染报告写不进 {_p}：{e}）')
print(f'渲染报告写到 {_report_path}')
