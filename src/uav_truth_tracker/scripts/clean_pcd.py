#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""clean_pcd.py — 清洗 UAV 建图 PCD：体素降采样去重叠 + 统计离群点剔除去杂散噪点。

累积建图会把多帧 LiDAR 叠起来：同一块地面落很多重复点(重叠) + 空中飘着个别杂散
噪点(无人机自反/飞尘/单次误测)。直接看原始 PCD 就会"空地上有很多奇怪的点云"。
本脚本：
  ① 体素降采样：每个 voxel 只留一个点(取均值) -> 去重叠、地面变薄
  ② 统计离群点剔除(SOR)：每点看 k 个最近邻平均距离，离群(>均值+n·std)的点删掉
     -> 去掉空中飘的杂散噪点
输出一个干净 PCD，原始文件不动。

用法：
  rosrun uav_truth_tracker clean_pcd.py ~/pointcloud_maps/uav_points_map_latest.pcd
  # 或： python3 clean_pcd.py in.pcd -o out.pcd --voxel 0.15 --sor-k 16 --sor-std 1.5
然后把导航指向干净版：
  roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false \
      pcd_file:=~/pointcloud_maps/uav_points_map_latest_clean.pcd
"""

import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree


def load_pcd_xyz(path):
    with open(path, "r") as f:
        lines = f.readlines()
    start = None
    ascii_fmt = True
    for i, ln in enumerate(lines):
        s = ln.strip()
        if s.startswith("DATA"):
            ascii_fmt = s.split()[1].lower() == "ascii"
            start = i + 1
            break
    if start is None:
        raise ValueError("no DATA line")
    if not ascii_fmt:
        raise ValueError("only ASCII PCD supported")
    rows = []
    for ln in lines[start:]:
        p = ln.split()
        if len(p) >= 3:
            try:
                rows.append((float(p[0]), float(p[1]), float(p[2])))
            except ValueError:
                pass
    return np.array(rows, dtype=np.float64)


def write_pcd_xyz(path, pts):
    n = pts.shape[0]
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n"
              % (n, n))
    with open(path, "w") as f:
        f.write(header)
        np.savetxt(f, pts, fmt="%.4f")


def voxel_downsample(pts, voxel):
    if voxel <= 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    # 每个 voxel 取均值
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    n = inv.max() + 1
    sums = np.zeros((n, 3))
    cnt = np.bincount(inv, minlength=n).reshape(-1, 1)
    np.add.at(sums, inv, pts)
    return sums / cnt


def statistical_outlier_removal(pts, k=16, std_mult=1.5):
    if pts.shape[0] <= k:
        return pts, np.ones(pts.shape[0], dtype=bool)
    tree = cKDTree(pts)
    d, _ = tree.query(pts, k=k + 1)      # 含自身
    mean_d = d[:, 1:].mean(axis=1)        # 去掉自身那一列
    thr = mean_d.mean() + std_mult * mean_d.std()
    keep = mean_d <= thr
    return pts[keep], keep


def clean(pts, voxel=0.15, sor_k=16, sor_std=1.5):
    out = voxel_downsample(pts, voxel)
    out, _ = statistical_outlier_removal(out, sor_k, sor_std)
    return out


def main():
    ap = argparse.ArgumentParser(description="清洗 UAV PCD（体素降采样 + 统计离群点剔除）")
    ap.add_argument("pcd")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--sor-k", type=int, default=16)
    ap.add_argument("--sor-std", type=float, default=1.5)
    args = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("__")])

    if not os.path.isfile(args.pcd):
        print("file not found:", args.pcd)
        sys.exit(1)
    out_path = args.output or (os.path.splitext(args.pcd)[0] + "_clean.pcd")
    pts = load_pcd_xyz(args.pcd)
    cleaned = clean(pts, args.voxel, args.sor_k, args.sor_std)
    write_pcd_xyz(out_path, cleaned)
    print("[clean_pcd] %d -> %d points (-%.1f%%)  voxel=%.2f sor_k=%d sor_std=%.1f"
          % (len(pts), len(cleaned), 100 * (1 - len(cleaned) / max(1, len(pts))),
             args.voxel, args.sor_k, args.sor_std))
    print("[clean_pcd] saved:", out_path)


if __name__ == "__main__":
    main()
