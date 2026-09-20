# -*- coding: utf-8 -*-
"""
响应式体检：用 headless Chrome 实测页面在手机/平板宽度下是否横向溢出，
并定位真正的「罪魁」元素（排除被祖先 overflow 裁掉的那些）。

原理：headless Chrome 的 --window-size 对 --dump-dom 不生效（实测视口恒为固定值），
      所以改用「一个探针页 + 多个精确宽度的 iframe」来测量：
      探针放在 course/ 根下（保证相对路径成立），逐个加载目标页并读 scrollWidth。

用法：
    python course/tools/responsive_check.py                     # 全站：index + qa + reference + lessons
    python course/tools/responsive_check.py --widths 390,768
    python course/tools/responsive_check.py course/qa/*.html    # 只测指定文件
    python course/tools/responsive_check.py --only-tables        # 只测含表格的课页（风险最高的那批）
"""
import io, os, re, sys, glob, subprocess, tempfile, shutil, json, urllib.parse

CHROME = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]

PROBE_TPL = """<!DOCTYPE html>
<html><head><meta charset="UTF-8"><title>RWD-PROBE</title></head>
<body style="margin:0">
<pre id="report">PENDING</pre>
<script>
var PAGES = __PAGES__;
var WIDTHS = __WIDTHS__;
var out = [];
function desc(el){
  var t=(el.tagName||'').toLowerCase();
  var cls=(el.className&&typeof el.className==='string')?('.'+el.className.trim().split(/\\s+/).slice(0,2).join('.')):'';
  var id=el.id?('#'+el.id):'';
  var tx='';
  if(/^(td|th|h1|h2|code)$/i.test(el.tagName)) tx='['+(el.textContent||'').trim().slice(0,16)+']';
  return t+id+cls.slice(0,28)+tx;
}
function clipped(el){
  var p=el.parentElement;
  while(p && p!==el.ownerDocument.documentElement){
    var ox=p.ownerDocument.defaultView.getComputedStyle(p).overflowX;
    if(ox==='auto'||ox==='scroll'||ox==='hidden'||ox==='clip') return true;
    p=p.parentElement;
  }
  return false;
}
function measure(win, w){
  var d=win.document.documentElement;
  var sw=d.scrollWidth, over=(sw-w>1);
  var list=[];
  if(over){
    win.document.querySelectorAll('body *').forEach(function(el){
      var r=el.getBoundingClientRect();
      if(r.width<=0||r.height<=0) return;
      if(r.right<=w+1) return;
      if(/^(svg|path|g|use|symbol)$/i.test(el.tagName)) return;
      if(clipped(el)) return;
      list.push([Math.round(r.right), desc(el)]);
    });
    list.sort(function(a,b){return b[0]-a[0];});
  }
  var seen={}, top=[];
  for(var i=0;i<list.length && top.length<4;i++){
    if(seen[list[i][1]]) continue; seen[list[i][1]]=1;
    top.push(list[i][1]+'@'+list[i][0]);
  }
  return {sw:sw, cw:w, over:over, top:top};
}
var qi = 0, tasks = [];
PAGES.forEach(function(p){ WIDTHS.forEach(function(w){ tasks.push([p,w]); }); });

function step(){
  if(qi>=tasks.length){
    document.getElementById('report').textContent = 'REPORT_START\\n' + out.join('\\n') + '\\nREPORT_END';
    document.title = 'RWD-DONE';
    return;
  }
  var p = tasks[qi][0], w = tasks[qi][1]; qi++;
  var f = document.createElement('iframe');
  f.style.cssText = 'width:'+w+'px;height:900px;border:0;display:block';
  f.src = p;
  f.onload = function(){
    setTimeout(function(){
      try{
        var m = measure(f.contentWindow, w);
        out.push((m.over?'OVER ':'ok   ') + ' w=' + w + ' sw=' + m.sw + ' ' + p +
                 (m.top.length?('  << ' + m.top.join(' , ')):''));
      }catch(e){
        out.push('ERR  w=' + w + ' ' + p + ' :: ' + e.message);
      }
      f.remove(); step();
    }, 120);
  };
  document.body.appendChild(f);
}
step();
</script>
</body></html>
"""


def main():
    args = sys.argv[1:]
    widths = [390, 768]
    only_tables = False
    rest = []
    i = 0
    while i < len(args):
        if args[i] == "--widths":
            widths = [int(x) for x in args[i + 1].split(",")]; i += 2
        elif args[i] == "--only-tables":
            only_tables = True; i += 1
        else:
            rest.append(args[i]); i += 1

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    course = os.path.join(root, "course")

    targets = []
    if rest:
        for pat in rest:
            for f in (glob.glob(pat) or [pat]):
                fp = os.path.abspath(f)
                if os.path.isfile(fp):
                    targets.append(fp)
    else:
        cand = [os.path.join(course, "index.html")]
        cand += sorted(glob.glob(os.path.join(course, "qa", "*.html")))
        cand += sorted(glob.glob(os.path.join(course, "reference", "*.html")))
        cand += sorted(glob.glob(os.path.join(course, "lessons", "*.html")))
        for f in cand:
            if only_tables and f.endswith(".html") and os.sep + "lessons" + os.sep in f:
                if "<table" not in io.open(f, encoding="utf-8", errors="ignore").read():
                    continue
            targets.append(f)

    rel = [os.path.relpath(f, course).replace("\\", "/") for f in targets]

    # 探针放在 _tmp/ 下（该目录已 gitignore，且不在站点发布范围内）。
    # ★ 注意：iframe 的 src 必须用**绝对 file:// URL**，
    #   否则会相对探针所在目录（_tmp/）去解析，全部变成不存在的路径。
    tmpdir = os.path.join(root, "_tmp")
    os.makedirs(tmpdir, exist_ok=True)
    probe_path = os.path.join(tmpdir, "_rwd_probe.html")
    pages = ["file:///" + urllib.parse.quote(f.replace("\\", "/"), safe="/:")
             for f in targets]
    io.open(probe_path, "w", encoding="utf-8").write(
        PROBE_TPL.replace("__PAGES__", json.dumps(pages, ensure_ascii=False))
                 .replace("__WIDTHS__", json.dumps(widths)))

    exe = next((c for c in CHROME if os.path.isfile(c)), None)
    if not exe:
        print("找不到 Chrome/Edge"); return
    prof = tempfile.mkdtemp(prefix="rwd_")
    url = "file:///" + probe_path.replace("\\", "/")
    budget = max(20000, 900 * len(rel) * len(widths))
    try:
        r = subprocess.run([exe, "--headless=new", "--disable-gpu", "--no-sandbox",
                            "--incognito", "--allow-file-access-from-files",
                            "--user-data-dir=" + prof,
                            "--virtual-time-budget=%d" % budget,
                            "--dump-dom", url],
                           capture_output=True, text=True, timeout=900)
        dom = r.stdout or ""
        m = re.search(r"REPORT_START\n(.*?)\nREPORT_END", dom, re.S)
        if not m:
            print("未取到测量结果（可能超预算）。DOM 长度:", len(dom)); return
        lines = [l for l in m.group(1).split("\n") if l.strip()]
    finally:
        # 探针留在 _tmp/ 不删：一是它在 gitignore 内、不发布；
        # 二是本机有 safe-delete 钩子，脚本内删除会被拦下。
        shutil.rmtree(prof, ignore_errors=True)

    for w in widths:
        sub = [l for l in lines if (" w=%d " % w) in l]
        bad = [l for l in sub if l.startswith("OVER") or l.startswith("ERR")]
        print("\n" + "=" * 96)
        print("宽度 %d px   共 %d 页，溢出 %d 页" % (w, len(sub), len(bad)))
        print("=" * 96)
        for l in bad:
            print("  ✗ " + l)
        if not bad:
            print("  ✓ 全部通过")
    print()


if __name__ == "__main__":
    main()
