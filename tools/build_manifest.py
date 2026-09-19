#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""build_manifest.py — 生成 IEEE 批量下载清单（cdpgrab batchget 用的 manifest.json）"""
import json
import os

PAPERS = r"G:/project/ieeexplore/papers"

# (输出文件名, articleNumber, 备注/所属方向)
ITEMS = [
    # ===== 激光 / 激光惯性 SLAM =====
    ("2022_FAST-LIO2", "9697912", "LiDAR-inertial, ikd-tree, direct"),
    ("2023_KISS-ICP", "10015694", "LiDAR odometry, minimal DLO 基线"),
    ("2022_Faster-LIO", "9718203", "LiDAR-inertial, 并行稀疏增量体素"),
    ("2022_FAST-LIVO", "9981107", "LiDAR-Inertial-Visual 稀疏直接法"),
    ("2025_FAST-LIVO2", "10757429", "LiDAR-Inertial-Visual, 全局统一体素图"),
    ("2024_PIN-SLAM", "10582536", "LiDAR SLAM + 隐式神经点表示"),
    ("2024_iG-LIO", "10380742", "增量 GICP 紧耦合 LIO"),
    ("2022_DLO", "9681177", "Direct LiDAR Odometry, 关键帧/子图"),
    ("2022_DiSCo-SLAM", "9662965", "分布式多机 LiDAR SLAM + Scan Context"),
    # ===== 视觉 / 惯性 =====
    ("2022_GVINS", "9667780", "GNSS-视觉-惯性紧耦合融合"),
    ("2022_DM-VIO", "9669044", "延迟边缘化 VIO"),
    ("2023_Dynam-SLAM", "9866888", "动态环境立体视觉惯性 SLAM"),
    ("2023_ESVIO", "10107754", "事件相机 立体视觉惯性里程计"),
    # ===== NeRF / 隐式表示 SLAM =====
    ("2022_NICE-SLAM", "9878912", "分层特征网格隐式 SLAM"),
    ("2023_Co-SLAM", "10204198", "坐标编码+稀疏参数编码联合"),
    ("2023_ESLAM", "10205103", "TSDF 混合表示高效隐式 SLAM"),
    ("2023_GO-SLAM", "10378579", "全局优化+即时重建"),
    ("2024_Loopy-SLAM", "10655306", "带回环的稠密神经 SLAM"),
    ("2023_NeRF-SLAM", "10341922", "DROID-SLAM + 概率体密度"),
    # ===== 3D Gaussian Splatting SLAM =====
    ("2024_MonoGS", "10657715", "Gaussian Splatting SLAM (MonoGS)"),
    ("2024_SplaTAM", "10656349", "Splat/Track/Map 三件套"),
    ("2024_GS-SLAM", "10657581", "3DGS 稠密视觉 SLAM"),
    ("2024_Photo-SLAM", "10657868", "ORB 特征 + 3DGS 建图"),
    ("2024_LIV-GaussMap", "10529285", "LiDAR-Inertial-Visual 辐射场建图"),
    ("2025_WildGS-SLAM", "11094879", "动态环境单目 3DGS SLAM"),
    ("2025_Splat-SLAM", "11147480", "RGB-only 全局优化 3DGS SLAM"),
    # ===== 学习式 / 场景识别 =====
    ("2022_LCDNet", "9723505", "深度回环检测 + 点云配准"),
    ("2022_OverlapTransformer", "9785497", "Transformer 激光场景识别"),
    ("2024_DeepLoopClosing", "10494918", "LiDAR SLAM 深度回环与重定位"),
    # ===== 多机 / 语义 =====
    ("2022_Kimera-Multi", "9686955", "分布式度量语义多机 SLAM"),
    ("2024_Swarm-SLAM", "10321649", "稀疏去中心化协同 SLAM"),
    ("2022_LAMP2", "9830862", "地下环境鲁棒多机 SLAM"),
    ("2024_D2SLAM", "10582478", "去中心化分布式协同视觉惯性"),
    ("2024_SNI-SLAM", "108..", "语义神经隐式 SLAM"),
    # ===== 综述 / 评测 =====
    ("2025_3DGS-Survey", "10521791", "3DGS 综述"),
    ("2024_DL-VisualLoc-Survey", "10260323", "深度学习视觉定位与建图综述"),
    ("2024_DynamicSLAM-Survey", "10577209", "动态环境视觉 SLAM 综述"),
    ("2022_LoopClosure-Survey", "9780121", "视觉回环检测综述"),
    ("2024_DARPA-SubT", "10286080", "极端环境 SLAM 现状与未来"),
    ("2023_Hilti-Oxford", "9968057", "毫米级 SLAM 基准数据集"),
]

# 修正 SNI-SLAM 的 id
FIX = {"2024_SNI-SLAM": "10655425"}

manifest = []
for name, aid, note in ITEMS:
    aid = FIX.get(name, aid)
    if not aid.isdigit():
        continue
    manifest.append({
        "key": name,
        "note": note,
        "pageUrl": "https://ieeexplore.ieee.org/document/%s" % aid,
        "pdfUrl": "https://ieeexplore.ieee.org/stampPDF/getPDF.jsp?tp=&arnumber=%s&ref=" % aid,
        "out": "%s/%s.pdf" % (PAPERS, name),
        "ieeeId": aid,
    })

out = r"G:/project/ieeexplore/meta/ieee_download_manifest.json"
json.dump(manifest, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print("manifest: %d papers -> %s" % (len(manifest), out))
for m in manifest:
    print("  %-30s %s" % (m["key"], m["ieeeId"]))
