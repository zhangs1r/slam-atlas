#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""retint_outputs.py — 把 outputs/*.html 的配色接到站点主题上，并支持跟随系统深浅色

背景：这几个产出文档原本是"深色单主题"设计（GitHub 深色配色，硬编码）。
本脚本做三件事：
  1. 抽出各文件 :root 里的调色板定义，改成由共享皮肤统一提供；
  2. 把散落在规则里的硬编码色值抽象成变量（按 hex+用途 命名，如 --t-141c26-bg）；
  3. 生成 outputs/assets/doc-theme.css：浅色值由 HSL 推导（保持色相、翻转明度），
     深色值沿用原值，两套都挂在 prefers-color-scheme 下。

再次运行是幂等的（已抽象的不会再动）。
"""
import colorsys
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)

FILES = [
    'outputs/SLAM图解入门.html',
    'outputs/SLAM技术演进图谱.html',
    'outputs/SLAM技术演进详解.html',
    'outputs/论文阅读路线图.html',
    'outputs/跨出版商下载能力实测报告.html',
]

# 站点令牌映射：产出文档的变量名 → 站点变量
TOKEN_MAP = {
    '--bg': '--paper', '--bg2': '--paper-light', '--bg3': '--code-bg', '--bg4': '--border-light',
    '--line': '--border', '--fg': '--ink', '--fg2': '--ink-60', '--fg3': '--ink-40',
    '--ok': '--success', '--info': '--accent',
}
# 站点主题里没有「警告/错误」语义色，这里显式给一组（浅色深、深色亮）
SEMANTIC = {
    '--warn': ('#9e731a', '#d29922'),
    '--bad':  ('#b3261e', '#f85149'),
}
# 分类强调色：浅色模式下用同色相的深色版本，保证在米白纸上有对比度
ACCENT_VARS = {
    '--c-vision': '#58a6ff', '--c-lidar': '#3fb950', '--c-fusion': '#d29922',
    '--c-neural': '#bc8cff', '--c-gs': '#f778ba', '--c-learn': '#ff7b72',
    '--c-multi': '#39c5cf', '--accent-color': '#58a6ff', '--accent-deep': '#58a6ff',
}

HEX = re.compile(r'#[0-9a-fA-F]{3,8}\b')
ROLE_OF = [
    (r'^background(-color|-image)?$', 'bg'),
    (r'^border(-(top|right|bottom|left))?(-color)?$', 'line'),
    (r'^border-', 'line'),
    (r'^box-shadow$', 'shadow'),
    (r'^color$', 'fg'),
    (r'^fill$|^stroke$', 'fg'),
    (r'^text-shadow$', 'shadow'),
    (r'^outline', 'line'),
]


def role(prop):
    for pat, r in ROLE_OF:
        if re.match(pat, prop.strip()):
            return r
    return 'fg'


def hex2hls(h):
    h = h.lstrip('#')
    if len(h) == 3:
        h = ''.join(c * 2 for c in h)
    r, g, b = (int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))
    return colorsys.rgb_to_hls(r, g, b)


def hls2hex(hh, ll, ss):
    r, g, b = colorsys.hls_to_rgb(hh, max(0.0, min(1.0, ll)), max(0.0, min(1.0, ss)))
    return '#%02x%02x%02x' % (round(r * 255), round(g * 255), round(b * 255))


def to_light(hexval, r):
    """把深色底纹色转成浅色模式下的等价色：保色相，翻转明度，压低饱和"""
    h, l, s = hex2hls(hexval)
    if r == 'bg':
        return hls2hex(h, 0.955 - 0.035 * s, min(s * 0.9, 0.55))   # 极浅底
    if r == 'line':
        return hls2hex(h, 0.86 - 0.06 * s, min(s * 0.85, 0.45))   # 浅描边
    if r == 'shadow':
        return 'rgba(26,26,46,0.08)'
    return hls2hex(h, 0.30 + 0.10 * (1 - s), min(max(s, 0.45), 0.85))  # 深色文字


def to_dark(hexval, r):
    """深色模式沿用原值；纯黑阴影统一成半透明黑"""
    return '#00000066' if r == 'shadow' else hexval


tints = {}   # varname -> (light, dark)
touched = []

for path in FILES:
    if not os.path.exists(path):
        print(f'跳过（不存在）{path}')
        continue
    s = open(path, encoding='utf-8').read()
    if 'assets/doc-theme.css' in s:
        print(f'已处理过，跳过 {path}')
        continue
    m = re.search(r'(<style>)(.*?)(</style>)', s, re.S)
    if not m:
        print(f'找不到 <style>，跳过 {path}')
        continue
    css = m.group(2)

    # 1. 抽出 :root 调色板
    rootm = re.search(r'(:root\s*\{)(.*?)(\})', css, re.S)
    palvals = {}
    if rootm:
        body = rootm.group(2)
        keep = []
        for decl in re.split(r';', body):
            mm = re.match(r'\s*(--[\w-]+)\s*:\s*(.+?)\s*$', decl)
            if not mm:
                if decl.strip():
                    keep.append(decl.strip())
                continue
            name, val = mm.group(1), mm.group(2).strip()
            if name in TOKEN_MAP or name in ACCENT_VARS or name in SEMANTIC or HEX.match(val):
                palvals[val.lower()] = name          # 交给共享皮肤
            else:
                keep.append(f'{name}:{val}')
        css = css[:rootm.start()] + (':root{' + ';'.join(keep) + '}' if keep else '') + css[rootm.end():]

    # 2. 逐条声明处理
    def fix_decl(dm):
        head, prop, val = dm.group(1), dm.group(2), dm.group(3)
        r = role(prop)
        out = val
        for hx in set(HEX.findall(val)):
            low = hx.lower()
            if low in palvals:
                out = re.sub(hx, f'var({palvals[low]})', out, flags=re.I)
                continue
            vn = f'--t-{low.lstrip("#")}-{r}'
            tints[vn] = (to_light(hx, r), to_dark(hx, r))
            out = re.sub(hx, f'var({vn})', out, flags=re.I)
        return f'{head}{prop}:{out}'

    # 只匹配真正的声明：必须紧跟在 { 或 ; 之后，避免把 .paper:hover 的选择器当属性
    css = re.sub(r'([{;]\s*)([a-zA-Z-]+)\s*:\s*([^;{}]+)', fix_decl, css)
    s = s[:m.start(2)] + css + s[m.end(2):]

    # 3. 共享皮肤 link + 顶部返回条
    s = s.replace('</style>', '</style>\n<link rel="stylesheet" href="../course/shared/theme.css">\n'
                               '<link rel="stylesheet" href="assets/doc-theme.css">', 1)
    if 'class="doc-bar"' not in s:
        bar = ('<div class="doc-bar"><a href="../course/index.html">← 课程馆</a>'
               '<a href="../md/index.html">论文原文索引</a>'
               '<a href="index.html">产出索引</a></div>\n')
        s = re.sub(r'(<body[^>]*>)', r'\1\n' + bar, s, count=1)
    open(path, 'w', encoding='utf-8').write(s)
    touched.append(path)

# ── 生成共享皮肤 ──────────────────────────────────────────────
os.makedirs('outputs/assets', exist_ok=True)
lines = ['/* ============================================================',
         '   outputs/assets/doc-theme.css — 产出文档主题层',
         '   由 tools/retint_outputs.py 生成，请勿手改。',
         '   作用：把产出文档的调色板接到 course/shared/theme.css 的令牌上，',
         '        从而与课程站同一套配色，并自动跟随系统深浅色。',
         '   ============================================================ */', '',
         ':root{']
for name, target in TOKEN_MAP.items():
    lines.append(f'  {name}: var({target});')
for name, val in ACCENT_VARS.items():
    h, l, s_ = hex2hls(val)
    lines.append(f'  {name}: {hls2hex(h, 0.36, min(max(s_, 0.5), 0.8))};')
for name, (lt, _dk) in SEMANTIC.items():
    lines.append(f'  {name}: {lt};')
for vn, (lt, _) in sorted(tints.items()):
    lines.append(f'  {vn}: {lt};')
lines += ['}', '',
          '@media screen and (prefers-color-scheme: dark){:root{']
for name, val in ACCENT_VARS.items():
    lines.append(f'  {name}: {val};')
for name, (_lt, dk) in SEMANTIC.items():
    lines.append(f'  {name}: {dk};')
for vn, (_, dk) in sorted(tints.items()):
    lines.append(f'  {vn}: {dk};')
lines += ['}}', '',
          '/* 与课程站统一的版式 */',
          'body{font-family:var(--font-sans);background:var(--paper);color:var(--ink);}',
          'h1,h2,h3,h4{font-family:var(--font-serif);}',
          '.doc-bar{position:sticky;top:0;z-index:9;display:flex;gap:16px;align-items:center;',
          '  padding:10px 18px;background:var(--paper-light);border-bottom:1px solid var(--border);',
          '  font-size:13.5px;font-family:var(--font-sans);}',
          '.doc-bar a{color:var(--accent-deep);text-decoration:none;}',
          '.doc-bar a:hover{text-decoration:underline;}', '']
open('outputs/assets/doc-theme.css', 'w', encoding='utf-8').write('\n'.join(lines))

print(f'处理 {len(touched)} 个文件：')
for t in touched:
    print('  ', t)
print(f'抽象出 {len(tints)} 个底纹变量 → outputs/assets/doc-theme.css')
