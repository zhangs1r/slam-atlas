#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
svgcheck.py —— 课程页 / 答疑页的 SVG 几何体检器（交付前必跑）

用途：检测页面里每一张内联 SVG 是否存在
  ① <text> 越出 SVG 边界；
  ② <text> 两两重叠（文字压文字，最常见也最难看的一类 bug）；
  ③ <rect>/<circle>/<path> 等图形越出画布。

用法（Windows，managed Python）：
  "C:\\Users\\ZJQ\\.workbuddy\\binaries\\python\\versions\\3.13.12\\python.exe" ^
      course\\tools\\svgcheck.py course\\qa\\qa-0002-*.html course\\lessons\\0017-*.html

不带参数时，默认扫描 course/qa/*.html 与 course/lessons/*.html 全部页面。

为什么要用 getBoundingClientRect() 而不是 getBBox()：
  getBBox() 不含祖先 transform，会大量误报；getBoundingClientRect() 拿到的是
  屏幕坐标系里的真实矩形，两两相交判断才可靠。

已知的"假警报"：
  KaTeX 生成的 <svg>（texts=0，盒子很小，如 12x27）使用 overflow:visible，
  其 path 会合法地超出 viewBox —— 本脚本对 texts=0 的 svg 不报图形越界。
"""
import glob
import io
import os
import re
import subprocess
import sys
import urllib.parse

CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
CHROME_FALLBACKS = [
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]
COURSE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TMPDIR = os.path.join(os.environ.get("TEMP", "/tmp"), "svgcheck")
os.makedirs(TMPDIR, exist_ok=True)

JS = r"""
<div id="svgreport" style="display:none"></div>
<script>
window.addEventListener('load', function(){ setTimeout(function(){
  var out=[];
  var svgs=document.querySelectorAll('svg');
  for(var si=0; si<svgs.length; si++){
    var svg=svgs[si];
    var box=svg.getBoundingClientRect();
    var texts=svg.querySelectorAll('text');
    if(!texts.length) continue;            /* KaTeX 的 svg：跳过 */
    var items=[];
    for(var k=0;k<texts.length;k++){
      items.push({t:(texts[k].textContent||'').trim(), r:texts[k].getBoundingClientRect()});
    }
    out.push('[SVG'+si+'] texts='+items.length+' box='+box.width.toFixed(0)+'x'+box.height.toFixed(0));
    for(var a=0;a<items.length;a++){
      var r=items[a].r;
      if(r.left<box.left-0.6||r.right>box.right+0.6||r.top<box.top-0.6||r.bottom>box.bottom+0.6){
        out.push('  OUT-OF-BOX: "'+items[a].t+'"');
      }
    }
    for(var i=0;i<items.length;i++){
      for(var j=i+1;j<items.length;j++){
        var A=items[i].r,B=items[j].r;
        var ox=Math.min(A.right,B.right)-Math.max(A.left,B.left);
        var oy=Math.min(A.bottom,B.bottom)-Math.max(A.top,B.top);
        if(ox>1.5&&oy>1.5){
          out.push('  OVERLAP '+ox.toFixed(1)+'x'+oy.toFixed(1)+' : "'+items[i].t+'" <> "'+items[j].t+'"');
        }
      }
    }
    var shapes=svg.querySelectorAll('rect,circle,ellipse,polyline,path,line');
    var bad=0;
    for(var s=0;s<shapes.length;s++){
      var sr=shapes[s].getBoundingClientRect();
      if(sr.width===0&&sr.height===0) continue;
      if(sr.left<box.left-1||sr.right>box.right+1||sr.top<box.top-1||sr.bottom>box.bottom+1) bad++;
    }
    out.push('  shapes_out_of_canvas='+bad);
  }
  document.getElementById('svgreport').textContent = out.join('\n') || 'NO_SVG_WITH_TEXT';
}, 2500); });
</script>
"""


def chrome_path():
    for p in [CHROME] + CHROME_FALLBACKS:
        if os.path.exists(p):
            return p
    sys.exit("找不到 Chrome / Edge，请改脚本顶部的 CHROME 常量")


def check(abspath):
    html = io.open(abspath, encoding="utf-8").read()
    tmp = os.path.join(os.path.dirname(abspath), "_svgcheck_tmp.html")
    io.open(tmp, "w", encoding="utf-8").write(html.replace("</body>", JS + "\n</body>"))
    url = urllib.parse.quote("file:///" + tmp.replace("\\", "/"), safe=":/?=&")
    try:
        proc = subprocess.run([
            chrome_path(), "--headless=new", "--disable-gpu", "--no-sandbox", "--incognito",
            "--user-data-dir=" + os.path.join(TMPDIR, "_profile"),
            "--window-size=1200,4000", "--virtual-time-budget=15000", "--dump-dom", url,
        ], capture_output=True, timeout=240)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
    dom = proc.stdout.decode("utf-8", "replace")
    m = re.search(r'<div id="svgreport"[^>]*>(.*?)</div>', dom, re.S)
    print("=" * 72)
    print("FILE:", os.path.relpath(abspath, COURSE))
    print((m.group(1) if m else "!! 未取到报告").strip())


def main():
    args = sys.argv[1:]
    if args:
        targets = []
        for a in args:
            targets.extend(glob.glob(a) or [a])
    else:
        targets = sorted(glob.glob(os.path.join(COURSE, "qa", "*.html"))) + \
                  sorted(glob.glob(os.path.join(COURSE, "lessons", "*.html")))
    for t in targets:
        if os.path.exists(t):
            check(os.path.abspath(t))
        else:
            print("跳过（不存在）:", t)
    print("=" * 72)
    print("判读：只看 OVERLAP / OUT-OF-BOX 两类。二者为空即通过。")


if __name__ == "__main__":
    main()
