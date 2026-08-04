# -*- coding: utf-8 -*-
"""hair_guides.py - 在动漫发型参考图上绘制每根发束的放样引导线，并校验与发束中心的重合度。

用法:
    # 独立模式（现有行为）
    python hair_guides.py --image <参考图> --strands <发束JSON> --out <输出图> [--verify] [--no-label]

    # kigstudio 集成模式：读取 state.json + 参考图，输出 result.json
    python hair_guides.py --from-kigstudio <state.json路径> --strands <发束JSON> [--verify]

发束JSON格式:
    [
      {"id": "bang_L1", "points": [[x1,y1],[x2,y2],...], "color": "#ff4040"},
      ...
    ]
points 从发根到发梢排列，脚本用 Catmull-Rom 样条插值成平滑曲线。
--verify 会沿每条曲线采样，统计采样点落在发束像素上的比例和横向中心偏移。

kigstudio 集成：
    读取 state.json（包含相机参数、覆盖图状态），将发束坐标转换回 2D 参考图像素坐标，
    输出 result.json 供 kigstudio 自动导入。
"""
import argparse
import json
import math
import os
import sys

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


# ---------------------------------------------------------------------------
# kigstudio integration: 2D pixel ↔ world-space coordinate conversion
# ---------------------------------------------------------------------------

def load_kigstudio_state(state_json_path):
    """读取 kigstudio 导出的 state.json，返回用于坐标转换的参数字典。"""
    with open(state_json_path, encoding="utf-8") as f:
        state = json.load(f)
    return state


def reference_to_world(ref_x, ref_y, state):
    """
    将参考图上的像素坐标 (ref_x, ref_y) 转换为 kigstudio 世界坐标。

    转换链:
      参考图像素 → 视图屏幕坐标 → 归一化设备坐标 [-1,1] → 世界坐标
    """
    overlay = state.get("overlay", {})
    offset_x = overlay.get("offset_x", 0.0)
    offset_y = overlay.get("offset_y", 0.0)
    scale = overlay.get("scale", 1.0)

    # 参考图像素 → 视图相对坐标（相对于 canvas 左上角）
    view_x = offset_x + ref_x * scale
    view_y = offset_y + ref_y * scale

    # 需要 display_size 做归一化。如果 state 没有保存，用 resolution 估算。
    display_size = state.get("display_size", float(state.get("resolution", 1024)))
    half_vp = state.get("viewport_size", 100.0) * 0.5

    # 归一化设备坐标 [-1, 1]
    rx = (view_x / display_size) * 2.0 - 1.0
    ry = 1.0 - (view_y / display_size) * 2.0

    center = state.get("center", [0.0, 0.0, 0.0])
    cam_right = state.get("cam_right", [1.0, 0.0, 0.0])
    cam_up = state.get("cam_up", [0.0, 1.0, 0.0])

    world = [
        center[0] + cam_right[0] * rx * half_vp + cam_up[0] * ry * half_vp,
        center[1] + cam_right[1] * rx * half_vp + cam_up[1] * ry * half_vp,
        center[2] + cam_right[2] * rx * half_vp + cam_up[2] * ry * half_vp,
    ]
    return world


def world_to_reference(world_x, world_y, world_z, state):
    """
    将 kigstudio 世界坐标反投影回参考图像素坐标。
    （用于验证/调试，将已有的 3D 引导线投影回 2D 图像上）
    """
    center = state.get("center", [0.0, 0.0, 0.0])
    cam_right = state.get("cam_right", [1.0, 0.0, 0.0])
    cam_up = state.get("cam_up", [0.0, 1.0, 0.0])

    rel_x = world_x - center[0]
    rel_y = world_y - center[1]
    rel_z = world_z - center[2]

    half_vp = state.get("viewport_size", 100.0) * 0.5

    rx = (rel_x * cam_right[0] + rel_y * cam_right[1] + rel_z * cam_right[2]) / half_vp
    ry = (rel_x * cam_up[0] + rel_y * cam_up[1] + rel_z * cam_up[2]) / half_vp

    display_size = state.get("display_size", float(state.get("resolution", 1024)))

    view_x = (rx * 0.5 + 0.5) * display_size
    view_y = (0.5 - ry * 0.5) * display_size

    overlay = state.get("overlay", {})
    offset_x = overlay.get("offset_x", 0.0)
    offset_y = overlay.get("offset_y", 0.0)
    scale = overlay.get("scale", 1.0)

    ref_x = (view_x - offset_x) / scale if scale > 0 else view_x
    ref_y = (view_y - offset_y) / scale if scale > 0 else view_y

    return ref_x, ref_y


def export_result_json(strands, output_path, state=None):
    """
    将发束数据导出为 kigstudio result.json 格式。

    strands: 发束列表，每项 {'id': str, 'points': [[x,y],...], ...}
    output_path: 输出 JSON 文件路径
    state: 如果提供，将 points 从世界坐标转换为 2D 参考图坐标；
           如果不提供，points 直接作为 2D 坐标输出。
    """
    result = {"version": 1, "strands": []}
    for s in strands:
        entry = {"id": s["id"], "points_2d": []}
        for pt in s.get("points", []):
            if state is not None and len(pt) >= 3:
                # 世界坐标 → 2D 参考图像素
                rx, ry = world_to_reference(pt[0], pt[1], pt[2], state)
                entry["points_2d"].append([round(rx, 1), round(ry, 1)])
            elif len(pt) >= 2:
                # 直接使用 2D 像素坐标
                entry["points_2d"].append([float(pt[0]), float(pt[1])])
        result["strands"].append(entry)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"Exported result.json: {output_path} ({len(result['strands'])} strands)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="hair_guides - 发束引导线标注与校验（支持 kigstudio 集成）")
    ap.add_argument("--image", help="参考图路径（独立模式）")
    ap.add_argument("--strands", required=True,
                    help="发束定义 JSON 文件路径")
    ap.add_argument("--out", help="输出渲染图路径（独立模式）")
    ap.add_argument("--verify", action="store_true",
                    help="校验引导线与发束中心的重合度")
    ap.add_argument("--no-label", action="store_true",
                    help="不绘制发束 id 标签")
    # kigstudio 集成选项
    ap.add_argument("--from-kigstudio", metavar="STATE_JSON",
                    help="kigstudio 集成模式：读取 state.json 并输出 result.json")
    ap.add_argument("--export-result", metavar="RESULT_JSON",
                    help="导出 result.json 供 kigstudio 导入（默认: tmp/result.json）")
    ap.add_argument("--display-size", type=float, default=600.0,
                    help="视图显示尺寸（用于坐标转换，默认 600）")
    args = ap.parse_args()

    # ------------------------------------------------------------------
    # kigstudio 集成模式
    # ------------------------------------------------------------------
    if args.from_kigstudio:
        state_path = args.from_kigstudio
        if not os.path.exists(state_path):
            print(f"ERROR: state.json not found: {state_path}", file=sys.stderr)
            sys.exit(1)

        state = load_kigstudio_state(state_path)
        print(f"Loaded state.json from {state_path}")

        # 确定参考图路径
        overlay = state.get("overlay", {})
        image_path = overlay.get("image_path", "")
        if image_path and os.path.exists(image_path):
            print(f"Using reference image: {image_path}")
        else:
            # 回退：尝试使用同目录下的 render.png
            state_dir = os.path.dirname(state_path)
            fallback = os.path.join(state_dir, "render.png")
            if os.path.exists(fallback):
                image_path = fallback
                print(f"Using fallback render image: {image_path}")
            else:
                print("ERROR: No reference image found. "
                      "Please load a reference image in the ortho editor.",
                      file=sys.stderr)
                sys.exit(1)

        # Ensure display_size is in state for coordinate conversion
        if "display_size" not in state:
            state["display_size"] = args.display_size

        # Load strands JSON
        with open(args.strands, encoding="utf-8") as f:
            strands = json.load(f)

        # Render verification image if requested
        base = Image.open(image_path).convert("RGB")
        draw = ImageDraw.Draw(base)
        try:
            font = ImageFont.truetype("arial.ttf", 13)
        except OSError:
            font = ImageFont.load_default()

        img_np = np.asarray(base).astype(int)
        print(f"{'strand':<12} {'on-hair':>8} {'mean|off|':>10} {'max|off|':>9}")

        kigstudio_strands = []
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

            # Convert 2D reference-image points to world coordinates
            world_pts = []
            for pt in s["points"]:
                wx, wy, wz = reference_to_world(pt[0], pt[1], state)
                world_pts.append([wx, wy, wz])

            kigstudio_strands.append({
                "id": s["id"],
                "points": world_pts,  # now in world space
                "points_2d": s["points"]  # original 2D for result.json
            })

            if args.verify:
                ratio, m, mx = verify(s, curve, img_np)
                print(f"{s['id']:<12} {ratio:>7.0%} {m:>9.1f}px {mx:>8.1f}px")

        # Save verification render
        out_dir = os.path.dirname(state_path)
        out_path = args.out or os.path.join(out_dir, "hair_guides_render.png")
        base.save(out_path)
        print(f"Saved render: {out_path}")

        # Export result.json for kigstudio
        result_path = args.export_result or os.path.join(out_dir, "result.json")
        export_result_json(kigstudio_strands, result_path)
        print("Done. kigstudio will auto-import result.json if 'Watch AI results' is enabled.")
        return

    # ------------------------------------------------------------------
    # 独立模式（现有行为）
    # ------------------------------------------------------------------
    if not args.image or not args.out:
        ap.error("独立模式需要 --image 和 --out 参数，或使用 --from-kigstudio 集成模式")

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
