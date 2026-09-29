#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SIL 仿真实验出图脚本。

用法：
    python tools/plot.py [csv_dir] [out_dir]
    默认 csv_dir = Host/build/sim_out，out_dir = docs/sil

从 sil_sim 产出的 CSV 生成六张图：
    e1_recovery.png   E1 初始倾角恢复（真值 vs 卡尔曼，含 ±0.5° 收敛带）
    e2_impulse.png    E2 速度冲击抗扰恢复
    e3_filters.png    E3 卡尔曼 vs 互补滤波 vs 纯积分（台架漂移对比）
    e4_trajectories.png  E4 直立环 kp 扫描轨迹族（顺序蓝色阶）
    e4_metrics.png    E4 收敛时间 / 超调量 vs kp（双面板，不同量纲不共轴）
    e5_jitter.png     E5 控制周期抖动敏感性（双面板）

配色遵循 dataviz 规范：分类槽位固定顺序、顺序量用单色蓝阶、
图表底色/网格/文字用 chrome token，已通过 validate_palette 校验。
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- 字体：中文渲染（Windows 优先雅黑） ----
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "Segoe UI", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# ---- chrome / ink tokens（dataviz 参考调色板，light 模式） ----
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASE = "#c3c2b7"
# 分类槽位 1/2/3（已通过 all-pairs 校验）
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
# 顺序蓝阶（ordinal 校验通过：250/350/450/550/650）
BLUES = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281"]

BAND = 0.5  # 收敛带（度）


def style_axes(ax, title, subtitle=None):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASE)
        ax.spines[side].set_linewidth(1.0)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.9)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.set_title(title, color=INK, fontsize=13, loc="left", pad=14 if subtitle else 8)
    if subtitle:
        ax.text(0, 1.04, subtitle, transform=ax.transAxes, color=INK2,
                fontsize=9.5, va="bottom")
    ax.xaxis.label.set_color(INK2)
    ax.yaxis.label.set_color(INK2)
    ax.xaxis.label.set_size(10)
    ax.yaxis.label.set_size(10)


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def direct_label(ax, x, y, text, color, dx=0.0, dy=0.0, ha="left"):
    """序列直接标注（relief 规则：低对比色必须带可见标签）。"""
    ax.annotate(text, (x, y), xytext=(6 + dx, dy), textcoords="offset points",
                color=color, fontsize=9, fontweight="bold", ha=ha,
                va="center")


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


# ---------- E1 / E2 时域恢复 ----------
def plot_recovery(csv_dir, out, csv_base, png_name, title, subtitle, t_mark=None):
    d = read_csv(os.path.join(csv_dir, csv_base + ".csv"))
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    fig.patch.set_facecolor(SURFACE)
    style_axes(ax, title, subtitle)
    ax.axhspan(-BAND, BAND, color=GRID, alpha=0.55, zorder=0,
               label=f"±{BAND}° 收敛带")
    if t_mark is not None:
        ax.axvline(t_mark, color=MUTED, linewidth=1.2, linestyle="--", zorder=1)
        ax.annotate("冲击", (t_mark, ax.get_ylim()[1]), xytext=(5, -12),
                    textcoords="offset points", color=INK2, fontsize=9)
    ax.plot(d["t"], d["theta_true"], color=S1, linewidth=2.0,
            label="车身倾角 θ（真值）")
    ax.plot(d["t"], d["kalman"], color=S2, linewidth=2.0,
            label="卡尔曼滤波输出")
    direct_label(ax, d["t"][-1], d["theta_true"][-1], "θ 真值", S1, dy=7)
    direct_label(ax, d["t"][-1], d["kalman"][-1], "卡尔曼", S2, dy=-8)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("倾角 (°)")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="upper right")
    save(fig, out, png_name)


# ---------- E3 滤波对比 ----------
def plot_e3(csv_dir, out):
    d = read_csv(os.path.join(csv_dir, "e3.csv"))
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    fig.patch.set_facecolor(SURFACE)
    style_axes(ax, "E3 静态台架：三种角度估计的漂移对比",
               "固定倾角 10°（读数坐标系 9°），陀螺初始零偏 1.5°/s + 随机游走，30 s")
    truth = d["theta_true"][0] - 1.0  # 安装偏角修正后的传感器坐标真值
    ax.axhline(truth, color=MUTED, linewidth=1.4, linestyle="--",
               label=f"真值 {truth:.0f}°")
    ax.plot(d["t"], d["pure"], color=S3, linewidth=2.0, label="纯积分")
    ax.plot(d["t"], d["comp"], color=S2, linewidth=2.0, label="互补滤波 α=0.99")
    ax.plot(d["t"], d["kalman"], color=S1, linewidth=2.0, label="卡尔曼（固件实现）")
    n = len(d["t"])
    direct_label(ax, d["t"][n - 1], d["pure"][n - 1], "纯积分", S3, dy=6)
    direct_label(ax, d["t"][n - 1], d["comp"][n - 1], "互补", S2, dy=-8)
    direct_label(ax, d["t"][n - 1], d["kalman"][n - 1], "卡尔曼", S1, dy=8)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("估计角度 (°)")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="center right")
    save(fig, out, "e3_filters.png")


# ---------- E4 参数扫描 ----------
def plot_e4(csv_dir, out):
    srows = read_csv(os.path.join(csv_dir, "e4_summary.csv"))
    kps = srows["kp"]

    # 轨迹族（顺序蓝阶 = 量级编码）
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    fig.patch.set_facecolor(SURFACE)
    style_axes(ax, "E4 直立环 kp 扫描：10° 初始倾角的恢复轨迹",
               "顺序蓝阶由浅到深对应 |kp| 递增；−400 扶正不足发散")
    for i, kp in enumerate(kps):
        fname = f"e4_kp{int(abs(kp)):04d}.csv"
        d = read_csv(os.path.join(csv_dir, fname))
        ax.plot(d["t"], d["theta_true"], color=BLUES[i], linewidth=2.0,
                label=f"kp = {kp:.0f}")
    ax.set_ylim(-16, 16)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("倾角 (°)")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="upper right")
    save(fig, out, "e4_trajectories.png")

    # 指标面板（settle 与 overshoot 量纲不同 → 双面板，绝不同轴）
    stable = [r for r in srows["stable"]] if "stable" in srows else []
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.4, 3.9))
    fig.patch.set_facecolor(SURFACE)
    for ax, key, ylab, ttl in (
        (ax1, "settle_t", "收敛时间 (s)", "收敛到 ±0.5° 所需时间"),
        (ax2, "overshoot", "反向超调 (°)", "过零后的反向最大偏差"),
    ):
        style_axes(ax, ttl, None)
        xs, ys = [], []
        for kp, v in zip(kps, srows[key]):
            if v >= 0:
                xs.append(kp)
                ys.append(v)
        ax.plot(xs, ys, color=S1, linewidth=2.0, marker="o", markersize=6.5,
                markerfacecolor=SURFACE, markeredgewidth=2.0)
        # 不稳定的点标在底部
        for kp, v, st in zip(kps, srows[key], srows["stable"]):
            if not st:
                ax.annotate("发散", (kp, ax.get_ylim()[0]),
                            xytext=(0, 8), textcoords="offset points",
                            color=INK2, fontsize=8.5, ha="center")
        ax.set_xlabel("balance_kp")
        ax.set_ylabel(ylab)
    fig.suptitle("E4 参数扫描指标：kp 过小发散，过大全局超调单调上升",
                 color=INK, fontsize=13, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, out, "e4_metrics.png")


# ---------- E5 抖动敏感性 ----------
def plot_e5(csv_dir, out):
    s = read_csv(os.path.join(csv_dir, "e5_summary.csv"))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.4, 3.9))
    fig.patch.set_facecolor(SURFACE)
    for ax, key, ylab, ttl in (
        (ax1, "settle_t", "收敛时间 (s)", "收敛到 ±0.5° 所需时间"),
        (ax2, "end_rms", "稳态 RMS 误差 (°)", "结尾 1.5 s 倾角 RMS"),
    ):
        style_axes(ax, ttl, None)
        ax.plot(s["jitter_ms"], s[key], color=S1, linewidth=2.0,
                marker="o", markersize=6.5, markerfacecolor=SURFACE,
                markeredgewidth=2.0)
        for x, v in zip(s["jitter_ms"], s[key]):
            ax.annotate(f"{v:.2f}", (x, v), xytext=(0, 8),
                        textcoords="offset points", color=INK2,
                        fontsize=8.5, ha="center")
        ax.set_xlabel("控制周期抖动 ± (ms)")
        ax.set_ylabel(ylab)
    fig.suptitle("E5 控制周期抖动敏感性：周期越确定，收敛越快越干净",
                 color=INK, fontsize=13, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, out, "e5_jitter.png")


def main():
    csv_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        "Host", "build", "sim_out")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join("docs", "sil")
    if not os.path.isdir(csv_dir):
        sys.exit(f"csv dir not found: {csv_dir} (先运行 sil_sim 生成数据)")
    os.makedirs(out, exist_ok=True)

    plot_recovery(csv_dir, out, "e1", "e1_recovery.png",
                  "E1 初始倾角 10° 的直立恢复",
                  "默认参数；1.16 s 进入 ±0.5° 并保持，反向超调 3.7°")
    plot_recovery(csv_dir, out, "e2", "e2_impulse.png",
                  "E2 速度冲击后的抗扰恢复",
                  "t = 2 s 施加 0.3 m/s 速度冲击；1.22 s 恢复到 ±0.5°",
                  t_mark=2.0)
    plot_e3(csv_dir, out)
    plot_e4(csv_dir, out)
    plot_e5(csv_dir, out)
    print("all figures written to", out)


if __name__ == "__main__":
    main()
