# -*- coding: utf-8 -*-
"""hair_guides.py - 在动漫发型参考图上绘制每根发束的放样引导线，并校验与发束中心的重合度。

用法:
    python hair_guides.py --image <参考图> --strands <发束JSON> --out <输出图> [--verify] [--no-label]

发束JSON格式:
    [
      {"id": "bang_L1", "points": [[x1,y1],[x2,y2],...], "color": "#ff4040"},
      ...
    ]
points 从发根到发梢排列，脚本用 Catmull-Rom 样条插值成平滑曲线。
--verify 会沿每条曲线采样，统计采样点落在发束像素上的比例和横向中心偏移。
"""
import argparse
import json
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

PALETTE = [
    "#ff4040", "#40c8ff", "#ffe040", "#60ff80", "#ff80d0",
    "#ffa040", "#a080ff", "#40ffc8", "#ffff80", "#ff8080",
    "#80a0ff", "#80ff40",
]


def catmull_rom(points, samples_per_seg=24):
    """穿过所有控制点的 Catmull-Rom 样条，返回平滑折线点列。"""
    pts = [tuple(map(float, p)) for p in points]
    if len(pts) < 2:
        return pts
    ext = [pts[0]] + pts + [pts[-1]]
    out = []
    for i in range(len(ext) - 3):
        p0, p1, p2, p3 = ext[i], ext[i + 1], ext[i + 2], ext[i + 3]
        for k in range(samples_per_seg):
            t = k / samples_per_seg
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t
                       + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                       + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t
                       + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                       + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, y))
    out.append(pts[-1])
    return out


def resample(curve, n=60):
    """按弧长等距重采样，便于校验时均匀取点。"""
    seg = [math.dist(curve[i], curve[i + 1]) for i in range(len(curve) - 1)]
    total = sum(seg)
    if total == 0:
        return [curve[0]] * n
    targets = [total * i / (n - 1) for i in range(n)]
    out, acc, j = [], 0.0, 0
    for t in targets:
        while j < len(seg) - 1 and acc + seg[j] < t:
            acc += seg[j]
            j += 1
        r = 0.0 if seg[j] == 0 else (t - acc) / seg[j]
        x = curve[j][0] + (curve[j + 1][0] - curve[j][0]) * r
        y = curve[j][1] + (curve[j + 1][1] - curve[j][1]) * r
        out.append((x, y))
    return out


def is_hair(r, g, b):
    """深蓝头发的简单判据:蓝色分量明显大于红色、整体偏暗。"""
    return b > r + 15 and (r + g + b) < 360 and b > 55


def verify(strand, curve, img):
    """沿曲线采样，返回 (on-hair比例, 平均中心偏移px, 最大偏移px)。"""
    h, w = img.shape[:2]
    pts = resample(curve)
    on, offsets = 0, []
    half = 12  # 垂直方向向两侧各取 12px
    for i, (x, y) in enumerate(pts):
        i2 = min(i + 1, len(pts) - 1)
        i1 = max(i - 1, 0)
        dx, dy = pts[i2][0] - pts[i1][0], pts[i2][1] - pts[i1][1]
        n = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / n, dx / n  # 法向
        xs, ys = [], []
        for t in range(-half, half + 1):
            px, py = int(round(x + nx * t)), int(round(y + ny * t))
            if 0 <= px < w and 0 <= py < h:
                xs.append(px)
                ys.append(py)
            else:
                xs.append(None)
                ys.append(None)
        center_ok = False
        weights = []
        for idx, t in enumerate(range(-half, half + 1)):
            px, py = xs[idx], ys[idx]
            if px is None:
                weights.append(0.0)
                continue
            r, g, b = (int(v) for v in img[py, px])
            ok = is_hair(r, g, b)
            weights.append(1.0 if ok else 0.0)
            if t == 0 and ok:
                center_ok = True
        if center_ok:
            on += 1
        s = sum(weights)
        if s > 0:
            c = sum(wgt * t for wgt, t in zip(weights, range(-half, half + 1))) / s
            offsets.append(c)
    ratio = on / len(pts)
    mean_off = float(np.mean(np.abs(offsets))) if offsets else float("nan")
    max_off = float(np.max(np.abs(offsets))) if offsets else float("nan")
    return ratio, mean_off, max_off


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--strands", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--no-label", action="store_true")
    args = ap.parse_args()

    with open(args.strands, encoding="utf-8") as f:
        strands = json.load(f)

    base = Image.open(args.image).convert("RGB")
    draw = ImageDraw.Draw(base)
    try:
        font = ImageFont.truetype("arial.ttf", 13)
    except OSError:
        font = ImageFont.load_default()

    img_np = np.asarray(base).astype(int)
    print(f"{'strand':<12} {'on-hair':>8} {'mean|off|':>10} {'max|off|':>9}")
    for i, s in enumerate(strands):
        color = s.get("color", PALETTE[i % len(PALETTE)])
        curve = catmull_rom(s["points"])
        draw.line(curve, fill=color, width=3)
        for p in s["points"]:
            draw.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3],
                         outline=color, width=1)
        if not args.no_label:
            draw.text((s["points"][0][0] + 4, s["points"][0][1] - 8),
                      s["id"], fill=color, font=font)
        if args.verify:
            ratio, m, mx = verify(s, curve, img_np)
            print(f"{s['id']:<12} {ratio:>7.0%} {m:>9.1f}px {mx:>8.1f}px")

    base.save(args.out)
    print("saved:", args.out)


if __name__ == "__main__":
    main()
