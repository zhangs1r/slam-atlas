#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""matchutil.py — 标题匹配的公共工具（多个解析脚本共用）

踩过的两个坑都收在这里：
  ① 连字符/空格差异会让分词式相似度误判
     —— OpenAlex 返回 "Semidirect"，我们写 "Semi-Direct"，分词后变成两个不同 token。
     解法：额外算一次"去全部非字母数字"的压缩形式相似度，取两者较大值。
  ② OpenAlex 的 search 参数遇到 * 或 ? 会整条查询报错
     —— 标题里带问号（如 KITTI 那篇）就会全军覆没。
     解法：查询前剥掉通配符。
"""
import difflib
import re


def _tokens(s):
    return set(" ".join(re.sub(r"[^a-z0-9]+", " ", (s or "").lower()).split()).split(" "))


def _compact(s):
    return re.sub(r"[^a-z0-9]+", "", (s or "").lower())


def title_sim(a, b):
    """标题相似度 0~1，对连字符/标点/大小写差异鲁棒"""
    a, b = a or "", b or ""
    if not a.strip() or not b.strip():
        return 0.0
    # 分词 Jaccard
    wa, wb = _tokens(a), _tokens(b)
    jac = (len(wa & wb) / len(wa | wb)) if (wa and wb) else 0.0
    # 压缩形式相似度（消除连字符/空格差异）
    comp = difflib.SequenceMatcher(None, _compact(a), _compact(b)).ratio()
    # 前缀重合
    na, nb = a.lower(), b.lower()
    pre = 0
    for x, y in zip(na, nb):
        if x != y:
            break
        pre += 1
    pre_score = pre / max(len(na), len(nb), 1)
    return max(0.55 * jac + 0.25 * comp + 0.20 * pre_score,
               0.75 * comp + 0.25 * pre_score)


def sanitize_query(q):
    """剥掉会让 OpenAlex 报错的通配符"""
    return re.sub(r"[*?]+", " ", q or "").strip()


def best_match(title, candidates, title_key="title", threshold=0.62):
    """从候选里挑最相似的一个，返回 (score, item) 或 (0, None)"""
    best_s, best_c = 0.0, None
    for c in candidates:
        s = title_sim(title, c.get(title_key, "") if isinstance(c, dict) else str(c))
        if s > best_s:
            best_s, best_c = s, c
    if best_s >= threshold:
        return best_s, best_c
    return best_s, None
