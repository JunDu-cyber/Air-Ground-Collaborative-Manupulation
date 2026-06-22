#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹跟踪误差分析：读 trajectory_logger.py 存的 flight_trajectory_*.csv，
算 EGO 期望轨迹 vs 实际飞行的跟踪误差(RMSE/Mean/Max/Std)，并画三张图：
  ① XY 俯视(期望 vs 实际)  ② 高度 z 随时间  ③ 跟踪误差随时间。

用法：
  python3 trajectory_analysis.py                 # 自动取 ~/trajectory_logs 里最新一份
  python3 trajectory_analysis.py <某个csv或目录>  # 指定文件/目录
无 ROS 依赖；只需 numpy(+matplotlib 画图，缺了也能出数字)。
"""

import csv
import glob
import math
import os
import sys

import numpy as np


def find_csv(arg):
    """arg 是文件->直接用；是目录或空->取该目录最新一份 flight_trajectory_*.csv。"""
    if arg and os.path.isfile(arg):
        return arg
    d = os.path.expanduser(arg) if arg else os.path.expanduser("~/trajectory_logs")
    files = sorted(glob.glob(os.path.join(d, "flight_trajectory_*.csv")))
    return files[-1] if files else None


def load(path):
    rows = list(csv.DictReader(open(path)))

    def col(name):
        out = []
        for r in rows:
            v = r.get(name, "")
            out.append(float(v) if v not in ("", None) else np.nan)
        return np.asarray(out, dtype=float)

    return col


def stats(e):
    """RMSE, Mean(abs), Max(abs), Std —— 只统计有期望cmd(非NaN)的样本。"""
    e = e[~np.isnan(e)]
    if e.size == 0:
        return (float("nan"),) * 4
    return (math.sqrt(float(np.mean(e ** 2))), float(np.mean(np.abs(e))),
            float(np.max(np.abs(e))), float(np.std(e)))


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    path = find_csv(arg)
    if not path:
        print("没找到 flight_trajectory_*.csv（先飞一次让 trajectory_logger 存日志，默认在 ~/trajectory_logs）")
        return
    print("分析文件:", path)
    col = load(path)
    t = col("time_rel")
    ax, ay, az = col("actual_x"), col("actual_y"), col("actual_z")
    dx, dy, dz = col("desired_x"), col("desired_y"), col("desired_z")
    e_xy, e_3d, e_z = col("error_xy"), col("error_3d"), col("error_z")

    m = ~np.isnan(e_3d)                     # 有匹配期望cmd的样本
    n_total, n_valid = len(t), int(m.sum())
    dur = float(np.nanmax(t) - np.nanmin(t)) if n_total else 0.0
    print("\n==== 轨迹跟踪误差分析 ====")
    print("时长 %.1fs | 采样 %d (含期望cmd %d, 覆盖 %.0f%%)"
          % (dur, n_total, n_valid, 100.0 * n_valid / max(n_total, 1)))
    if n_valid == 0:
        print("没有任何带期望cmd的样本——EGO 在飞行中有没有发 /planning/pos_cmd？(需 low_altitude 飞起来并执行轨迹)")
        return
    r3, rxy, rz = stats(e_3d[m]), stats(e_xy[m]), stats(e_z[m])
    print("%-10s %8s %8s %8s %8s" % ("误差", "RMSE", "Mean", "Max", "Std"))
    print("%-10s %8.3f %8.3f %8.3f %8.3f" % ("3D (m)", *r3))
    print("%-10s %8.3f %8.3f %8.3f %8.3f" % ("水平XY(m)", *rxy))
    print("%-10s %8.3f %8.3f %8.3f %8.3f" % ("高度Z (m)", *rz))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(16, 5))
        axs[0].plot(ax, ay, "-", lw=1.6, color="C0", label="actual")
        axs[0].plot(dx[m], dy[m], "--", lw=1.2, color="C3", label="desired (EGO)")
        axs[0].set_title("XY trajectory (top-down)")
        axs[0].set_xlabel("x (m)"); axs[0].set_ylabel("y (m)")
        axs[0].axis("equal"); axs[0].grid(True, alpha=.3); axs[0].legend()
        axs[1].plot(t, az, "-", color="C0", label="actual z")
        axs[1].plot(t[m], dz[m], "--", color="C3", label="desired z")
        axs[1].set_title("altitude"); axs[1].set_xlabel("t (s)"); axs[1].set_ylabel("z (m)")
        axs[1].grid(True, alpha=.3); axs[1].legend()
        axs[2].plot(t[m], e_3d[m], "-", color="C2", label="3D error")
        axs[2].plot(t[m], e_xy[m], "-", color="C1", alpha=.7, label="XY error")
        axs[2].axhline(r3[0], ls=":", color="C2", label="3D RMSE %.2f m" % r3[0])
        axs[2].set_title("tracking error"); axs[2].set_xlabel("t (s)"); axs[2].set_ylabel("error (m)")
        axs[2].grid(True, alpha=.3); axs[2].legend()
        out = path[:-4] + "_analysis.png" if path.endswith(".csv") else path + "_analysis.png"
        fig.tight_layout(); fig.savefig(out, dpi=120)
        print("\n图已保存:", out)
    except Exception as exc:  # noqa: BLE001
        print("\n(画图跳过，缺 matplotlib? 数字已在上面) -", exc)


if __name__ == "__main__":
    main()
