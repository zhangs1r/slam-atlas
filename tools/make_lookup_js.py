#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""make_lookup_js.py — 把标题列表注入 ieee_lookup.js，生成可直接 eval 的脚本"""
import json
import sys

titles_file, tpl_file, out_file = sys.argv[1], sys.argv[2], sys.argv[3]
titles = []
for line in open(titles_file, encoding="utf-8"):
    line = line.strip()
    if line and not line.startswith("#"):
        titles.append(line)
tpl = open(tpl_file, encoding="utf-8").read()
js = "window.__TITLES = %s;\n%s" % (json.dumps(titles, ensure_ascii=False), tpl)
open(out_file, "w", encoding="utf-8").write(js)
print("wrote %s with %d titles (%d bytes)" % (out_file, len(titles), len(js)))
