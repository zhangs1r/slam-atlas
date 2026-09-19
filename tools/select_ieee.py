#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""select_ieee.py — 从 IEEE 采集结果中按方向筛选候选论文（排除非 SLAM 噪声）"""
import json
import re
import sys

SRC = r"G:/project/ieeexplore/meta/ieee_harvest.json"
raw = open(SRC, encoding="utf-8").read()
d = json.loads(raw[raw.find("{"):])
R = d["records"]

# 明显不是 SLAM 的噪声（3DGS 渲染/重建/avatar 等）
NOISE = re.compile(
    r"(inverse rendering|avatar|human (novel|performance|reconstruction)|portrait|face|"
    r"text-to-3d|generation of |novel view synthesis|image compression|style transfer|"
    r"object detection|segmentation benchmark|super-resolution|deblurring|relighting|"
    r"medical|surgical|agricultur|weed|traffic sign|driver (state|distraction))",
    re.I,
)

CATS = [
    ("1. LiDAR / 激光惯性", r"(lidar|loam|lio|point.?l io|point-lio|scan match|icp|kiss|glim|"
                            r"laser|point cloud (registration|odometry)|range.?inertial|lidar.?inertial)"),
    ("2. 视觉惯性 / VIO", r"(visual.?inertial|vins|vio\b|imu|monocular.*(inertial|odometry)|"
                          r"gnss.*visual|msckf|inertial odometry)"),
    ("3. NeRF / 隐式表示 SLAM", r"(nerf|neural radiance|implicit|sdf|nice-slam|eslam|co-slam|"
                                 r"point-slam|go-slam|imap|vox-fusion|neur.*(slam|mapping)|"
                                 r"radiance field.*(slam|mapping|localization))"),
    ("4. 3D Gaussian Splatting SLAM", r"(gaussian splat|splatam|gs-slam|photo-slam|cg-slam|"
                                      r"sgs-slam|gaussian-slam|gaussmap|gaussian.?lic|splat)"),
    ("5. 学习式 / 基础模型", r"(deep|learning|neural|transformer|foundation model|droid|superglue|"
                            r"lightglue|feature match|mast3r|self-supervised|end-to-end)"),
    ("6. 语义 / 动态环境", r"(semantic|dynamic|dyna|moving object|object.?level|cognitive|"
                           r"metric.?semantic|panoptic)"),
    ("7. 多机 / 协同 SLAM", r"(multi.?robot|collaborative|swarm|distributed|multi.?agent|"
                             r"cooperative|decentralized|peer.?to.?peer|lamp)"),
    ("8. 回环 / 场景识别", r"(loop closure|place recognition|relocali|revisit|scene recognition|"
                           r"scan context|descriptor|retrieval)"),
    ("9. 事件相机 / 其他传感器", r"(event camera|event-based|event.?driven|thermal|sonar|radar|"
                                  r"ultrasonic|wheel odometry|gps|uwb)"),
    ("10. 综述 / 评测 / 极端环境", r"(survey|review|benchmark|dataset|challenge|extreme|"
                                    r"underground|subterranean|comparative analysis|tutorial)"),
]


def ck(title):
    return re.sub(r"[^a-z0-9]+", " ", title.lower())


out = {}
used = set()
for name, pat in CATS:
    rx = re.compile(pat, re.I)
    items = []
    for r in R:
        t = r["title"]
        if NOISE.search(t):
            continue
        if rx.search(t):
            items.append(r)
    items.sort(key=lambda x: -(x.get("cites") or 0))
    out[name] = items

report = []
for name, items in out.items():
    report.append("\n" + "=" * 100)
    report.append("%s   (%d 篇候选)" % (name, len(items)))
    report.append("=" * 100)
    for r in items[:22]:
        star = "*" if (r.get("cites") or 0) >= 100 else " "
        report.append(" %s[%4dc|%6ddl|%s] %s" % (star, r.get("cites") or 0, r.get("downloads") or 0,
                                                 r.get("year"), r["title"][:104]))
        report.append("      id=%s | %s" % (r["id"], r["venue"][:88]))
print("\n".join(report))

json.dump(out, open(r"G:/project/ieeexplore/meta/ieee_candidates.json", "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
