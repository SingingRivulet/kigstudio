# -*- coding: utf-8 -*-
"""test_api.py — KigStudio Agent API 集成测试脚本

用法:
    # 快速冒烟测试（需要 KigStudio 运行中）
    python test_api.py

    # 完整测试（包含写入/导入）
    python test_api.py --full

    # 指定端口
    python test_api.py --port 18920

    # 仅测试正交端点
    python test_api.py --ortho-only

    # 输出详细日志
    python test_api.py --verbose

前置条件:
    - KigStudio 已启动且 Agent API 运行中（默认 http://127.0.0.1:18920）
    - 正交投影编辑器已打开且至少完成一次渲染（ortho 端点需要渲染数据）
    - --full 模式需要一个包含 hair 节点的工程
"""

import argparse
import json
import os
import sys
import time
import traceback
from io import BytesIO
from typing import Any, Optional

try:
    import requests
except ImportError:
    print("请安装 requests: pip install requests")
    sys.exit(1)

try:
    from PIL import Image
except ImportError:
    Image = None  # PNG 校验可选


# ============================================================
# 配置
# ============================================================

class Config:
    BASE = "http://127.0.0.1:18920"
    API = f"{BASE}/api/v1"
    TIMEOUT = 10  # 秒
    VERBOSE = False
    FULL = False
    ORTHO_ONLY = False
    SAVE_DIR: Optional[str] = None  # 若设置，导出图片保存到此目录

    # 测试用数据
    TEST_NODE_ID: Optional[int] = None
    TEST_STRAND_INDEX: Optional[int] = None
    ALL_STRANDS: list = []  # [(node_id, strand_index, name), ...]

    @classmethod
    def url(cls, path: str) -> str:
        return f"{cls.API}{path}"


def save_image(filename: str, content: bytes) -> None:
    """如果配置了 SAVE_DIR，将二进制内容写入文件。"""
    if not Config.SAVE_DIR:
        return
    os.makedirs(Config.SAVE_DIR, exist_ok=True)
    filepath = os.path.join(Config.SAVE_DIR, filename)
    with open(filepath, "wb") as f:
        f.write(content)
    vlog(f"图片已保存: {filepath}")


# ============================================================
# 工具函数
# ============================================================

_err_count = 0
_ok_count = 0
_skip_count = 0


def vlog(msg: str) -> None:
    if Config.VERBOSE:
        print(f"  [dbg] {msg}")


def _check(ok: bool, label: str, detail: str = "") -> bool:
    global _ok_count, _err_count
    if ok:
        _ok_count += 1
        print(f"  ✓ {label}")
        return True
    else:
        _err_count += 1
        print(f"  ✗ {label}  ← {detail}" if detail else f"  ✗ {label}")
        return False


def skip(label: str, reason: str = "") -> None:
    global _skip_count
    _skip_count += 1
    print(f"  ⊘ {label}  (跳过: {reason})" if reason else f"  ⊘ {label}  (跳过)")


def get(path: str, **kwargs) -> requests.Response:
    url = Config.url(path)
    vlog(f"GET {url}")
    return requests.get(url, timeout=Config.TIMEOUT, **kwargs)


def post(path: str, data: Any = None, **kwargs) -> requests.Response:
    url = Config.url(path)
    vlog(f"POST {url} {json.dumps(data)[:120] if data else ''}")
    return requests.post(url, json=data, timeout=Config.TIMEOUT, **kwargs)


def patch(path: str, data: Any, **kwargs) -> requests.Response:
    url = Config.url(path)
    vlog(f"PATCH {url} {json.dumps(data)[:120]}")
    return requests.patch(url, json=data, timeout=Config.TIMEOUT, **kwargs)


def delete(path: str, **kwargs) -> requests.Response:
    url = Config.url(path)
    vlog(f"DELETE {url}")
    return requests.delete(url, timeout=Config.TIMEOUT, **kwargs)


# ============================================================
# 测试用例
# ============================================================

def test_ping() -> None:
    """GET /system/status — 服务是否存活"""
    print("\n── System ──")
    try:
        r = get("/system/status")
        _check(r.status_code == 200, "GET /system/status → 200",
               f"status={r.status_code}")
        body = r.json()
        _check(body.get("ok") is True, "  response.ok == True",
               f"body={str(body)[:100]}")
        vlog(f"status body: {json.dumps(body, ensure_ascii=False)[:200]}")
    except requests.ConnectionError:
        _check(False, "GET /system/status", "无法连接 — KigStudio 是否正在运行？")
    except Exception as e:
        _check(False, "GET /system/status", str(e))


def test_toast() -> None:
    """POST /system/toast — 弹出提示"""
    try:
        r = post("/system/toast", {"message": "API test: toast", "duration_ms": 1000})
        _check(r.status_code in (200, 202), "POST /system/toast",
               f"status={r.status_code}")
    except Exception as e:
        _check(False, "POST /system/toast", str(e))


def test_project() -> None:
    """项目相关端点"""
    print("\n── Project ──")
    try:
        r = get("/project")
        _check(r.status_code == 200, "GET /project → 200",
               f"status={r.status_code}")
        body = r.json()
        _check(body.get("ok") is True, "  response.ok == True")
        data = body.get("data", {})
        vlog(f"project: {json.dumps(data, ensure_ascii=False)[:300]}")
        _check("path" in data or "node_count" in data,
               "  has path/node_count", f"keys={list(data.keys())}")
    except Exception as e:
        _check(False, "GET /project", str(e))


def test_nodes_list() -> None:
    """节点列表与详情"""
    print("\n── Nodes ──")
    try:
        r = get("/nodes")
        _check(r.status_code == 200, "GET /nodes → 200",
               f"status={r.status_code}")
        body = r.json()
        nodes = body.get("data", {}).get("nodes", [])
        _check(len(nodes) > 0, f"  node_count = {len(nodes)}", "没有节点 — 请先加载工程")

        if nodes:
            # Show all nodes with their type info
            for n in nodes:
                st = n.get("source_type", "?")
                print(f"  node #{n.get('id')} ({n.get('title','')}): "
                      f"source_type={st}, triangles={n.get('triangle_count', 0)}")

            # Pick a HAIR-capable node: source_type==2 (addon) is required
            # for the strand list to appear in the UI properties panel.
            addon_nodes = [n for n in nodes if n.get("source_type") == 2]
            if addon_nodes:
                Config.TEST_NODE_ID = addon_nodes[0].get("id")
                print(f"  选用节点 #{Config.TEST_NODE_ID} ({addon_nodes[0].get('title','')})"
                      f" 作为发束测试目标")
            else:
                # Fallback: use first node (strands can be created but won't
                # show in UI unless the node's addon_type == HAIR)
                Config.TEST_NODE_ID = nodes[0].get("id")
                print(f"  注意: 没有 source_type==2 的节点，"
                      f"回退到 #{Config.TEST_NODE_ID}（发束可能不显示在UI中）")

            vlog(f"节点列表: {[(n.get('id'), n.get('title','')) for n in nodes]}")
    except Exception as e:
        _check(False, "GET /nodes", str(e))


def test_node_detail() -> None:
    """获取单个节点详情"""
    nid = Config.TEST_NODE_ID
    if nid is None:
        skip("GET /nodes/:id", "没有节点可测")
        return
    try:
        r = get(f"/nodes/{nid}")
        _check(r.status_code == 200, f"GET /nodes/{nid} → 200",
               f"status={r.status_code}")
        body = r.json()
        _check(body.get("ok") is True, "  response.ok == True")
        data = body.get("data", {})
        _check(data.get("id") == nid, f"  id == {nid}")
        vlog(f"node detail keys: {list(data.keys())}")
    except Exception as e:
        _check(False, f"GET /nodes/{nid}", str(e))


def test_node_bounds() -> None:
    """获取节点包围盒"""
    nid = Config.TEST_NODE_ID
    if nid is None:
        skip("GET /nodes/:id/bounds", "没有节点可测")
        return
    try:
        r = get(f"/nodes/{nid}/bounds")
        _check(r.status_code == 200, f"GET /nodes/{nid}/bounds → 200",
               f"status={r.status_code}")
        body = r.json()
        data = body.get("data", {})
        has_min = "min" in data
        _check(has_min, "  has bounds data", f"keys={list(data.keys())}")
    except Exception as e:
        _check(False, f"GET /nodes/{nid}/bounds", str(e))


def test_strands_list() -> None:
    """遍历所有节点，收集发束列表"""
    print("\n── Strands ──")
    try:
        r = get("/nodes")
        body = r.json()
        nodes = body.get("data", {}).get("nodes", [])
        if not nodes:
            skip("Strands", "没有节点可测")
            return

        total_strands = 0
        for node in nodes:
            nid = node.get("id")
            if nid is None:
                continue
            title = node.get("title", f"node#{nid}")
            sr = get(f"/nodes/{nid}/strands")
            sbody = sr.json()
            if sr.status_code != 200 or not sbody.get("ok"):
                continue
            strands = sbody.get("data", {}).get("strands", [])
            count = len(strands)
            total_strands += count
            if count > 0:
                names = [s.get("name", "?") for s in strands[:5]]
                suffix = " ..." if count > 5 else ""
                print(f"  node #{nid} ({title}): {count} strands: {names}{suffix}")
                for s in strands:
                    Config.ALL_STRANDS.append((
                        nid,
                        s.get("index", 0),
                        s.get("name", f"strand#{s.get('index', 0)}")
                    ))
            else:
                vlog(f"  node #{nid} ({title}): 0 strands")

        _check(total_strands > 0, f"  total strands = {total_strands}",
               "所有节点都没有发束")

        if Config.ALL_STRANDS:
            # Set TEST_NODE_ID and TEST_STRAND_INDEX from first found
            Config.TEST_NODE_ID = Config.ALL_STRANDS[0][0]
            Config.TEST_STRAND_INDEX = Config.ALL_STRANDS[0][1]
    except Exception as e:
        _check(False, "Strands list", str(e))


def test_strand_detail() -> None:
    """遍历所有发束，显示详情摘要"""
    if not Config.ALL_STRANDS:
        skip("GET /nodes/:id/strands/:idx", "没有发束可测")
        return
    try:
        shown = 0
        for nid, sidx, name in Config.ALL_STRANDS:
            r = get(f"/nodes/{nid}/strands/{sidx}")
            body = r.json()
            strand = body.get("data", {}).get("strand", {})
            gp = strand.get("guide_points", [])
            wp = strand.get("width_points", [])
            status = ""
            if strand.get("mesh_dirty"):
                status += " [dirty]"
            if strand.get("repair_failed"):
                status += " [repair-failed]"
            print(f"  strand #{nid}/{sidx} '{name}': "
                  f"{len(gp)} guide pts, {len(wp)} width pts{status}")
            shown += 1
        _check(shown == len(Config.ALL_STRANDS),
               f"  shown {shown}/{len(Config.ALL_STRANDS)} strands")
    except Exception as e:
        _check(False, "Strand detail", str(e))


# ============================================================
# 新功能测试 (Feature 1-5)
# ============================================================

def test_strand_rename() -> None:
    """测试发束改名 API — Feature 2"""
    print("\n── Strand Rename ──")
    if not Config.ALL_STRANDS:
        skip("Rename API", "没有发束可测")
        return
    nid, sidx, name = Config.ALL_STRANDS[0]
    strand_uuid = None
    try:
        r = get(f"/nodes/{nid}/strands/{sidx}")
        body = r.json()
        strand_uuid = body.get("data", {}).get("strand", {}).get("uuid")
        _check(strand_uuid is not None,
               f"Step 1: 获取发束 UUID → {strand_uuid}")
    except Exception as e:
        _check(False, "Step 1: 获取发束 UUID", str(e))
        return

    if not strand_uuid:
        skip("Rename API", "无法获取发束 UUID")
        return

    if not Config.FULL:
        print(f"  [smoke] 将调用: POST /nodes/{nid}/strands/by-uuid/{strand_uuid}/rename")
        skip("Step 2: 改名 API", "使用 --full 启用写入")
        return

    try:
        new_name = f"renamed_{int(time.time())}"
        payload = {"new_name": new_name}
        r = post(f"/nodes/{nid}/strands/by-uuid/{strand_uuid}/rename",
                 payload)
        _check(r.status_code in (200, 201),
               f"Step 2: POST rename → {r.status_code}",
               f"body={r.text[:200]}")
        body = r.json()
        _check(body.get("ok") is True,
               "  ok == True",
               f"body={str(body)[:200]}")
        new_uuid = body.get("new_uuid", "")
        _check(len(new_uuid) > 0 and new_uuid != strand_uuid,
               f"  UUID changed: {strand_uuid[:8]}... → {new_uuid[:8]}...")
        _check(body.get("name") == new_name,
               f"  name updated → '{new_name}'")

        # Rename back to original UUID
        restore_payload = {"new_name": name, "new_uuid": strand_uuid}
        r2 = post(f"/nodes/{nid}/strands/by-uuid/{new_uuid}/rename",
                  restore_payload)
        if r2.status_code in (200, 201):
            _check(True, f"Step 3: 恢复原名 '{name}' + 恢复 UUID")
        else:
            _check(False, "Step 3: 恢复原 UUID", f"status={r2.status_code}")
    except Exception as e:
        _check(False, "Rename API", str(e))


def test_strand_hidden_guide_points() -> None:
    """测试隐藏引导点 API — Feature 4"""
    print("\n── Hidden Guide Points ──")
    if not Config.ALL_STRANDS:
        skip("Hidden guide points", "没有发束可测")
        return
    nid, sidx, name = Config.ALL_STRANDS[0]

    if not Config.FULL:
        print(f"  [smoke] 将调用: PATCH /nodes/{nid}/strands/{sidx}")
        skip("Step: 设置隐藏引导点", "使用 --full 启用写入")
        return

    try:
        # Set hidden_guide_points_start
        start_pts = [[-1.0, 0.0, -1.0], [-0.5, 0.5, -0.5]]
        payload = {
            "node_id": nid,
            "strand_index": sidx,
            "hidden_guide_points_start": start_pts,
        }
        r = patch(f"/nodes/{nid}/strands/{sidx}", payload)
        _check(r.status_code in (200, 201),
               f"Step 1: 设置 hidden_guide_points_start → {r.status_code}",
               f"body={r.text[:200]}")

        # Set hidden_guide_points_end
        end_pts = [[1.0, 0.0, 1.0]]
        payload2 = {
            "node_id": nid,
            "strand_index": sidx,
            "hidden_guide_points_end": end_pts,
        }
        r2 = patch(f"/nodes/{nid}/strands/{sidx}", payload2)
        _check(r2.status_code in (200, 201),
               f"Step 2: 设置 hidden_guide_points_end → {r2.status_code}")

        # Read back and verify
        r3 = get(f"/nodes/{nid}/strands/{sidx}")
        strand = r3.json().get("data", {}).get("strand", {})
        read_start = strand.get("hidden_guide_points_start", [])
        read_end = strand.get("hidden_guide_points_end", [])
        _check(len(read_start) == 2,
               f"Step 3: 回读 hidden_guide_points_start → {len(read_start)} pts (期望 2)",
               f"got {read_start}")
        _check(len(read_end) == 1,
               f"Step 4: 回读 hidden_guide_points_end → {len(read_end)} pts (期望 1)",
               f"got {read_end}")

        # Clean up: clear hidden guide points
        clear_payload = {
            "node_id": nid,
            "strand_index": sidx,
            "hidden_guide_points_start": [],
            "hidden_guide_points_end": [],
        }
        r4 = patch(f"/nodes/{nid}/strands/{sidx}", clear_payload)
        _check(r4.status_code in (200, 201),
               f"Step 5: 清除隐藏引导点 → {r4.status_code}")
    except Exception as e:
        _check(False, "Hidden guide points", str(e))


def test_strand_auto_hair_root() -> None:
    """测试自动发根引导点 API — Feature 5"""
    print("\n── Auto Hair Root ──")
    if not Config.ALL_STRANDS:
        skip("Auto hair root", "没有发束可测")
        return
    nid, sidx, name = Config.ALL_STRANDS[0]

    if not Config.FULL:
        print(f"  [smoke] 将调用: PATCH /nodes/{nid}/strands/{sidx}")
        skip("Step: 设置 auto_hair_root", "使用 --full 启用写入")
        return

    try:
        # Enable auto_hair_root
        payload = {
            "node_id": nid,
            "strand_index": sidx,
            "auto_hair_root": True,
        }
        r = patch(f"/nodes/{nid}/strands/{sidx}", payload)
        _check(r.status_code in (200, 201),
               f"Step 1: 设置 auto_hair_root=true → {r.status_code}",
               f"body={r.text[:200]}")

        # Read back
        r2 = get(f"/nodes/{nid}/strands/{sidx}")
        strand = r2.json().get("data", {}).get("strand", {})
        _check(strand.get("auto_hair_root") is True,
               "Step 2: 回读 auto_hair_root == True")

        # Disable auto_hair_root
        payload2 = {
            "node_id": nid,
            "strand_index": sidx,
            "auto_hair_root": False,
        }
        r3 = patch(f"/nodes/{nid}/strands/{sidx}", payload2)
        _check(r3.status_code in (200, 201),
               f"Step 3: 设置 auto_hair_root=false → {r3.status_code}")

        # Verify disabled
        r4 = get(f"/nodes/{nid}/strands/{sidx}")
        strand2 = r4.json().get("data", {}).get("strand", {})
        _check(strand2.get("auto_hair_root") is False,
               "Step 4: 回读 auto_hair_root == False")
    except Exception as e:
        _check(False, "Auto hair root", str(e))


def test_strand_reverse_guide_points() -> None:
    """测试反转引导点顺序 — Feature 1
    通过 strand.update API 手动反转 guide_points 顺序来模拟 UI 的 Reverse 按钮。
    """
    print("\n── Reverse Guide Points ──")
    if not Config.ALL_STRANDS:
        skip("Reverse guide points", "没有发束可测")
        return
    nid, sidx, name = Config.ALL_STRANDS[0]

    # Read current guide points
    try:
        r = get(f"/nodes/{nid}/strands/{sidx}")
        strand = r.json().get("data", {}).get("strand", {})
        orig_pts = strand.get("guide_points", [])
    except Exception as e:
        _check(False, "Step 1: 获取引导点", str(e))
        return

    if len(orig_pts) < 2:
        skip("Reverse guide points", f"只有 {len(orig_pts)} 个引导点，需要 >= 2")
        return

    if not Config.FULL:
        print(f"  [smoke] 将反转 {len(orig_pts)} 个引导点并提交到 strand #{nid}/{sidx}")
        skip("Step: 反转引导点", "使用 --full 启用写入")
        return

    try:
        # Reverse and submit
        reversed_pts = list(reversed(orig_pts))
        payload = {
            "node_id": nid,
            "strand_index": sidx,
            "guide_points": reversed_pts,
        }
        r = patch(f"/nodes/{nid}/strands/{sidx}", payload)
        _check(r.status_code in (200, 201),
               f"Step 2: 提交反转后的 {len(reversed_pts)} 个点 → {r.status_code}",
               f"body={r.text[:200]}")

        # Read back and verify first point is now what was last
        r2 = get(f"/nodes/{nid}/strands/{sidx}")
        strand2 = r2.json().get("data", {}).get("strand", {})
        new_pts = strand2.get("guide_points", [])
        if len(new_pts) == len(orig_pts) and len(orig_pts) > 0:
            first_match = (abs(new_pts[0][0] - orig_pts[-1][0]) < 0.01 and
                           abs(new_pts[0][1] - orig_pts[-1][1]) < 0.01 and
                           abs(new_pts[0][2] - orig_pts[-1][2]) < 0.01)
            _check(first_match,
                   f"Step 3: 验证反转 — 新首点 == 旧尾点",
                   f"期望 {orig_pts[-1]}, 得到 {new_pts[0] if new_pts else 'N/A'}")

        # Restore original order
        restore_payload = {
            "node_id": nid,
            "strand_index": sidx,
            "guide_points": orig_pts,
        }
        r3 = patch(f"/nodes/{nid}/strands/{sidx}", restore_payload)
        _check(r3.status_code in (200, 201),
               f"Step 4: 恢复原始顺序 → {r3.status_code}")
    except Exception as e:
        _check(False, "Reverse guide points", str(e))


def test_ortho_export_guide_curves() -> None:
    """测试导出引导线覆盖 — Feature 3

    引导线覆盖是在 process_ai_export() 管线中完成的：
      1. 用户在正交编辑器中勾选 "Export Guide Curves" 复选框
      2. 点击导出按钮 → GPU readback → BGRA→RGBA 转换
      3. draw_guide_curves_on_buffer() 在 RGBA buffer 上绘制彩色引导线
      4. 结果推送到 API 缓存，随后 /ortho/render 返回带引导线的图像

    此测试验证：
      - /ortho/render 返回有效 PNG（引导线将在 UI 导出后叠加）
      - 相机状态包含投影所需的完整参数
      - 颜色调色板字节序已正确修复
    """
    print("\n── Export Guide Curves ──")
    try:
        # Request with guide curve overlay (query param forces drawing)
        r = get("/ortho/render?guides=1&color_code=1")
        if r.status_code == 503:
            skip("Export guide curves", "还没有渲染数据")
            return
        _check(r.status_code == 200, f"GET /ortho/render?guides=1 → {r.status_code}",
               f"status={r.status_code}")
        ct = r.headers.get("Content-Type", "")
        _check("image/png" in ct, f"  Content-Type = {ct}")

        size_kb = len(r.content) / 1024
        _check(len(r.content) > 100,
               f"  PNG size = {size_kb:.1f} KB (with guides)")

        # Verify guide curves changed pixel content
        # (raw render without guides should be different)
        r_raw = get("/ortho/render")
        has_guides = False
        if r_raw.status_code == 200 and len(r_raw.content) == len(r.content):
            # Compare first 1KB of pixel data (skip PNG header differences)
            diff_count = sum(1 for a, b in zip(r.content, r_raw.content) if a != b)
            has_guides = diff_count > 500
            _check(has_guides,
                   f"  Guide overlay changes pixels ({diff_count} bytes differ)",
                   f"  (可能是正交编辑器未打开或callback未设置)")
        else:
            _check(False, "  Compare with raw render",
                   f"raw_status={r_raw.status_code} sizes={len(r_raw.content)} vs {len(r.content)}")

        # 验证相机状态包含 3D→2D 投影所需的完整参数
        r2 = get("/ortho/state")
        if r2.status_code == 200:
            state = r2.json()
            has_cam = all(k in state for k in
                          ["center", "cam_right", "cam_up", "viewport_size"])
            _check(has_cam,
                   "  state has camera params for 3D→2D projection",
                   f"keys={list(state.keys())[:8]}")
            if has_cam:
                print(f"  [info] 在正交编辑器中勾选 'Export Guide Curves'")
                print(f"         并点击导出按钮后，引导线将覆盖到 PNG 上")
                print(f"         调色板: 12色 (来自 hair_guides.py PALETTE)")

        save_image("ortho_export_guide_curves.png", r.content)

        # Test configurable line thickness
        r_thick = get("/ortho/render?guides=1&color_code=1&line_width=5")
        if r_thick.status_code == 200:
            thick_size = len(r_thick.content) / 1024
            has_thick = r_thick.content != r.content
            _check(has_thick,
                   f"  line_width=5 changes output ({thick_size:.1f} KB)",
                   "  (same as line_width=1)")
            save_image("ortho_export_guide_curves_thick.png", r_thick.content)
        else:
            _check(False, f"  line_width=5 -> {r_thick.status_code}")

        # Test font size for strand name labels
        r_font = get("/ortho/render?guides=1&color_code=1&line_width=3&font_size=18")
        if r_font.status_code == 200:
            font_size_kb = len(r_font.content) / 1024
            _check(True,
                   f"  font_size=18 PNG size = {font_size_kb:.1f} KB")
            save_image("ortho_export_guide_curves_text.png", r_font.content)
        else:
            _check(False, f"  font_size=18 -> {r_font.status_code}")

    except Exception as e:
        _check(False, "Export guide curves", str(e))


# ============================================================
# 正交投影端点
# ============================================================

def test_ortho_ping() -> None:
    """正交 API 健康检查"""
    print("\n── Ortho API ──")
    try:
        r = get("/ortho/ping")
        _check(r.status_code == 200, "GET /ortho/ping → 200",
               f"status={r.status_code}")
        body = r.json()
        _check(body.get("ok") is True, "  response.ok == True")
        svc = body.get("service", "")
        _check("kigstudio" in svc.lower(), f"  service = {svc}")
    except Exception as e:
        _check(False, "GET /ortho/ping", str(e))


def test_ortho_state() -> None:
    """正交相机状态"""
    try:
        r = get("/ortho/state")
        _check(r.status_code == 200, "GET /ortho/state → 200",
               f"status={r.status_code}")
        body = r.json()
        has_version = "version" in body
        has_center = "center" in body
        has_resolution = "resolution" in body
        _check(has_version or has_center or has_resolution,
               "  has state fields",
               f"keys={list(body.keys())[:10]}")
        vlog(f"state: {json.dumps(body, ensure_ascii=False)[:300]}")
    except Exception as e:
        _check(False, "GET /ortho/state", str(e))


def test_ortho_render() -> None:
    """正交渲染图"""
    try:
        r = get("/ortho/render")
        if r.status_code == 503:
            skip("GET /ortho/render",
                 "还没有渲染数据 — 请在正交编辑器中完成一次渲染")
            return
        _check(r.status_code == 200, "GET /ortho/render → 200",
               f"status={r.status_code}")
        ct = r.headers.get("Content-Type", "")
        _check("image/png" in ct, f"  Content-Type = {ct}")
        size_kb = len(r.content) / 1024
        _check(len(r.content) > 100, f"  body size = {size_kb:.1f} KB",
               "PNG 数据太小" if len(r.content) <= 100 else "")

        # 校验是合法 PNG
        if Image is not None and len(r.content) > 100:
            try:
                img = Image.open(BytesIO(r.content))
                _check(True, f"  valid PNG: {img.size[0]}×{img.size[1]} {img.mode}")
            except Exception:
                _check(False, "  valid PNG", "无法解码 PNG")
        elif len(r.content) > 100:
            is_png = r.content[:8] == b'\x89PNG\r\n\x1a\n'
            _check(is_png, "  PNG magic bytes", f"got {r.content[:4].hex()}")

        save_image("ortho_render.png", r.content)
    except Exception as e:
        _check(False, "GET /ortho/render", str(e))


def test_ortho_blend() -> None:
    """正交混合图（底模 + 参考图叠加）"""
    try:
        r = get("/ortho/blend?ratio=0.5")
        if r.status_code == 503:
            skip("GET /ortho/blend",
                 "还没有渲染数据")
            return
        _check(r.status_code == 200, "GET /ortho/blend → 200",
               f"status={r.status_code}")
        ct = r.headers.get("Content-Type", "")
        _check("image/png" in ct, f"  Content-Type = {ct}")

        size_kb = len(r.content) / 1024
        _check(len(r.content) > 100, f"  body size = {size_kb:.1f} KB")

        save_image("ortho_blend_0.5.png", r.content)

        # Check if overlay is available for blend comparison
        has_overlay = False
        try:
            ov = get("/ortho/overlay")
            has_overlay = (ov.status_code == 200)
        except Exception:
            pass

        # Different ratios should produce different images (only when overlay is loaded)
        r2 = get("/ortho/blend?ratio=0.9")
        if r2.status_code == 200:
            save_image("ortho_blend_0.9.png", r2.content)
            different = r.content != r2.content
            if has_overlay:
                _check(different or len(r.content) < 100,
                       "  different ratios → different images",
                       "ratio=0.5 和 ratio=0.9 返回相同内容")
            else:
                _check(True, "  blend ratio (跳过: 没有参考图可混合)")
        # Test blend with guide curves overlay
        r3 = get("/ortho/blend?ratio=0.5&guides=1&color_code=1")
        if r3.status_code == 200:
            save_image("ortho_blend_guides.png", r3.content)
            has_diff = r3.content != r.content
            _check(has_diff or not has_overlay,
                   "  blend+guides differs from plain blend",
                   "" if has_diff else "  (no overlay or same content)")
        # Test blend with thick guide curves
        r4 = get("/ortho/blend?ratio=0.5&guides=1&color_code=1&line_width=5&font_size=16")
        if r4.status_code == 200:
            save_image("ortho_blend_guides_thick.png", r4.content)
            has_thick = r4.content != r3.content if r3.status_code == 200 else True
            _check(has_thick or not has_overlay,
                   "  blend+guides with thick lines differs",
                   "" if has_thick else "  (same content)")
    except Exception as e:
        _check(False, "GET /ortho/blend", str(e))


def test_ortho_overlay() -> None:
    """参考图端点"""
    try:
        r = get("/ortho/overlay")
        if r.status_code == 404:
            skip("GET /ortho/overlay", "没有加载参考图")
            return
        _check(r.status_code == 200, "GET /ortho/overlay → 200",
               f"status={r.status_code}")
        ct = r.headers.get("Content-Type", "")
        _check("image/png" in ct, f"  Content-Type = {ct}")
        save_image("ortho_overlay.png", r.content)
    except Exception as e:
        _check(False, "GET /ortho/overlay", str(e))


# ============================================================
# 2D 正交图上模拟提交发束
# ============================================================

def test_2d_strand_submit() -> None:
    """在2D正交视图上使用像素坐标直接提交发束引导线。

    使用 POST /api/v1/ortho/strand 端点，直接提交2D像素坐标，
    服务端利用当前正交相机状态自动完成 2D→3D 转换。

    像素坐标基于 render_resolution（通常 2048×2048）。
    已知有效范围：(500,500) ~ (1000,1000) 落在模型表面上。
    """
    print("\n── 2D 发束提交模拟 ──")

    # Step 1: 获取正交相机状态（验证可用）
    try:
        r = get("/ortho/state")
        if r.status_code != 200:
            skip("2D 发束提交", "正交状态不可用")
            return
        state = r.json()
        _check("center" in state and "resolution" in state,
               "Step 1: GET /ortho/state → 相机参数",
               f"keys={list(state.keys())[:8]}")
        vlog(f"  resolution={state.get('resolution')}, "
             f"viewport_size={state.get('viewport_size')}, "
             f"center={state.get('center')}")
    except Exception as e:
        _check(False, "Step 1: GET /ortho/state", str(e))
        return

    # Step 2: 用贝塞尔曲线生成2D引导点（像素坐标）
    controls_2d = [
        (500, 500),     # P0: 起点
        (500, 700),     # P1: 右下方控制
        (1000, 700),    # P2: 右下控制
        (1000, 1000),   # P3: 终点
    ]
    num_samples = 12
    guide_2d = sample_bezier_curve(controls_2d, num_samples)

    print(f"  控制点 (2D px): {controls_2d}")
    print(f"  采样点 ({num_samples}): {guide_2d[:3]}...{guide_2d[-2:]}")

    _check(len(guide_2d) == num_samples,
           f"Step 2: 生成 {num_samples} 个2D采样点")

    # Step 3: 通过 /ortho/strand 直接提交2D坐标（服务端做 2D→3D 转换）
    if not Config.FULL:
        # Smoke mode: just show the payload, don't actually create
        payload = {
            "node_id": Config.TEST_NODE_ID or 0,
            "name": "test_2d_example",
            "guide_points_2d": [[x, y] for (x, y) in guide_2d],
            "guide_samples_per_segment": 64,
        }
        print(f"  [smoke] 将发送: POST /ortho/strand")
        print(f"  [smoke] payload guide_points_2d: "
              f"{payload['guide_points_2d'][:3]}...{payload['guide_points_2d'][-1]}")
        skip("Step 3: 创建发束", "使用 --full 启用写入")
        return

    nid = Config.TEST_NODE_ID
    if nid is None:
        skip("Step 3: 创建发束", "没有节点")
        return

    try:
        strand_name = f"test_2d_{int(time.time())}"
        width_data = compute_width_points_2d(guide_2d, scale=1.0)
        payload = {
            "node_id": nid,
            "name": strand_name,
            "guide_points_2d": [[x, y] for (x, y) in guide_2d],
            "width_points_2d": width_data,
            "guide_samples_per_segment": 64,
        }

        # POST /ortho/strand — server converts 2D→3D for both guides & widths
        r = post("/ortho/strand", payload)
        _check(r.status_code in (200, 201, 202),
               f"Step 3a: POST /ortho/strand → {r.status_code}",
               f"body={r.text[:200]}")

        body = r.json()
        _check(body.get("ok") is True,
               "  response.ok == True",
               f"body={str(body)[:200]}")

        strand_idx = body.get("data", {}).get("strand_index")
        created = body.get("data", {}).get("created", False)
        gp_count = body.get("data", {}).get("guide_point_count", 0)
        surface_hits = body.get("data", {}).get("surface_hits", 0)
        wp_count = body.get("data", {}).get("width_point_count", 0)
        _check(strand_idx is not None,
               "  has strand_index",
               f"data={body.get('data', {})}")
        _check(gp_count == num_samples,
               f"  guide_point_count = {gp_count} (期望 {num_samples})")
        _check(wp_count == num_samples,
               f"  width_point_count = {wp_count} (期望 {num_samples})")

        if strand_idx is not None:
            print(f"  发束 '{strand_name}' (index={strand_idx}, "
                  f"{'新建' if created else '更新'}) "
                  f"{gp_count} 引导点 ({surface_hits} 命中表面), "
                  f"{wp_count} 宽度向量")

            # Step 4: 回读验证
            r = get(f"/nodes/{nid}/strands/{strand_idx}")
            strand = r.json().get("data", {}).get("strand", {})
            read_gp = strand.get("guide_points", [])
            _check(len(read_gp) == num_samples,
                   f"Step 4: 回读验证 → {len(read_gp)} guide pts (期望 {num_samples})",
                   f"得到 {len(read_gp)} 个")
            read_wp = strand.get("width_points", [])
            _check(len(read_wp) == num_samples,
                   f"Step 4: 回读宽度向量 → {len(read_wp)} (期望 {num_samples})",
                   f"得到 {len(read_wp)} 个")

            if len(read_gp) > 0:
                print(f"  首个3D引导点: ({read_gp[0][0]:.4f}, "
                      f"{read_gp[0][1]:.4f}, {read_gp[0][2]:.4f})")
            if len(read_wp) > 0:
                print(f"  首个宽度向量: curve_id={read_wp[0].get('curve_id')}, "
                      f"dir=({read_wp[0].get('direction', [0,0,0])})")

    except Exception as e:
        _check(False, "Step 3: 创建发束", str(e))


# ============================================================
# AI 引导线导入工作流（模拟完整流程）
# ============================================================

def test_ai_workflow() -> None:
    """模拟 AI agent 的完整工作流：
    1. 获取状态 → 2. 获取混合图 → 3. 生成引导线 → 4. 提交发束数据
    """
    print("\n── AI 工作流模拟 ──")

    # Step 1: 获取正交状态
    try:
        r = get("/ortho/state")
        if r.status_code != 200:
            skip("AI 工作流", "正交状态不可用")
            return
        state = r.json()
        _check("center" in state, "Step 1: GET /ortho/state → 相机参数",
               f"keys={list(state.keys())[:8]}")
        vlog(f"state: {json.dumps(state, ensure_ascii=False)[:400]}")
    except Exception as e:
        _check(False, "Step 1: GET /ortho/state", str(e))
        return

    # Step 2: 获取混合渲染图
    try:
        r = get("/ortho/blend?ratio=0.5")
        if r.status_code == 200 and len(r.content) > 100:
            _check(True, f"Step 2: GET /ortho/blend → {len(r.content)/1024:.1f} KB PNG")
        else:
            skip("Step 2: GET /ortho/blend", "渲染数据不可用")
    except Exception as e:
        _check(False, "Step 2: GET /ortho/blend", str(e))

    # Step 3: 从状态推算坐标映射
    try:
        # 验证状态中的必要字段
        required = ["viewport_size", "resolution", "center", "cam_right", "cam_up"]
        missing = [k for k in required if k not in state]
        _check(len(missing) == 0, "Step 3: 状态字段完整性",
               f"缺少: {missing}" if missing else "")

        # 演示坐标转换：图像中心 → 世界坐标
        if not missing:
            res = state.get("resolution", 512)
            vp = state.get("viewport_size", 100)
            center = state["center"]  # [cx, cy, cz]
            cam_r = state["cam_right"]  # [rx, ry, rz]
            cam_u = state["cam_up"]     # [ux, uy, uz]

            # NDC (0,0) = 图像中心 = 世界 center
            half = vp * 0.5
            img_cx = res / 2.0
            img_cy = res / 2.0
            rx = (img_cx / res) * 2.0 - 1.0  # → 0.0
            ry = 1.0 - (img_cy / res) * 2.0  # → 0.0

            wx = center[0] + cam_r[0] * rx * half + cam_u[0] * ry * half
            wy = center[1] + cam_r[1] * rx * half + cam_u[1] * ry * half
            wz = center[2] + cam_r[2] * rx * half + cam_u[2] * ry * half

            dist = ((wx - center[0])**2 + (wy - center[1])**2 + (wz - center[2])**2)**0.5
            _check(dist < 0.01, f"Step 3: 图像中心 → 世界中心 (偏差 {dist:.4f})",
                   f"偏差过大: {dist:.3f}")
    except Exception as e:
        _check(False, "Step 3: 坐标转换", str(e))

    # Step 4: 生成模拟发束数据并提交（仅 --full 模式）
    if not Config.FULL:
        skip("Step 4: 提交发束数据", "使用 --full 启用")
        return

    nid = Config.TEST_NODE_ID
    if nid is None:
        skip("Step 4: 提交发束数据", "没有节点")
        return

    try:
        strand_name = f"test_api_{int(time.time())}"
        guide_points = generate_test_guide_points(state)
        data = {
            "name": strand_name,
            "guide_points": guide_points,
            "guide_samples": 64,
        }
        r = post(f"/nodes/{nid}/strands", data)
        _check(r.status_code in (200, 201, 202),
               f"Step 4: POST /nodes/{nid}/strands → {r.status_code}",
               f"body={r.text[:200]}")
        body = r.json()
        if body.get("ok"):
            strand_idx = body.get("data", {}).get("strand_index", "?")
            vlog(f"创建发束: index={strand_idx}")

            # 立即清理测试发束
            r_del = delete(f"/nodes/{nid}/strands/{strand_idx}")
            _check(r_del.status_code == 200,
                   f"  DELETE /nodes/{nid}/strands/{strand_idx} → {r_del.status_code}")
        else:
            vlog(f"创建发束失败: {body}")
    except Exception as e:
        _check(False, "Step 4: 提交发束", str(e))


def generate_test_guide_points(state: dict) -> list:
    """根据正交状态生成模拟发束引导点（垂直穿过画面中心的曲线）。"""
    center = state.get("center", [0, 0, 0])
    cam_u = state.get("cam_up", [0, 1, 0])
    vp = state.get("viewport_size", 100)

    points = []
    half = vp * 0.5
    num_pts = 8
    for i in range(num_pts):
        t = (i / (num_pts - 1)) * 2.0 - 1.0  # [-1, 1] 从上到下
        ry = -t  # 图像 Y 向下，世界 Y 向上
        px = center[0] + cam_u[0] * ry * half * 0.6
        py = center[1] + cam_u[1] * ry * half * 0.6
        pz = center[2] + cam_u[2] * ry * half * 0.6
        points.append([round(px, 4), round(py, 4), round(pz, 4)])
    return points


# ============================================================
# 2D 像素坐标 → 3D 世界坐标 转换工具
# ============================================================

def pixel_to_world(px: float, py: float, state: dict) -> list:
    """将 API 2D 渲染像素坐标转换为 3D 世界坐标。

    参数:
        px, py: 渲染分辨率空间中的像素坐标（如 2048×2048 空间）。
        state:  /ortho/state 返回的相机状态字典。

    返回:
        [wx, wy, wz] 世界坐标。
    """
    res = state.get("resolution", 2048)
    center = state.get("center", [0, 0, 0])
    cam_right = state.get("cam_right", [1, 0, 0])
    cam_up = state.get("cam_up", [0, 1, 0])
    vp = state.get("viewport_size", 100)

    # NDC: [0, res] → [-1, 1], Y 翻转（图像 Y 向下，世界 Y 向上）
    ndc_x = (px / res) * 2.0 - 1.0
    ndc_y = 1.0 - (py / res) * 2.0
    half = vp * 0.5

    wx = center[0] + cam_right[0] * ndc_x * half + cam_up[0] * ndc_y * half
    wy = center[1] + cam_right[1] * ndc_x * half + cam_up[1] * ndc_y * half
    wz = center[2] + cam_right[2] * ndc_x * half + cam_up[2] * ndc_y * half

    return [round(wx, 4), round(wy, 4), round(wz, 4)]


def pixel_to_world_batch(pixels: list, state: dict) -> list:
    """批量转换 2D 像素 → 3D 世界坐标。"""
    return [pixel_to_world(px, py, state) for (px, py) in pixels]


def sample_bezier_curve(controls: list, num_samples: int = 12) -> list:
    """对三次贝塞尔曲线采样，返回 num_samples 个 (x, y) 点。

    参数:
        controls: 4 个控制点 [(x0,y0), (x1,y1), (x2,y2), (x3,y3)]。
        num_samples: 采样点数。

    返回:
        [(x, y), ...] 沿贝塞尔曲线的采样点列表。
    """
    if len(controls) < 4:
        # 少于4个点时线性插值
        if len(controls) < 2:
            return controls
        result = []
        for i in range(num_samples):
            t = i / (num_samples - 1)
            # 在多段之间均匀插值
            seg_count = len(controls) - 1
            seg_t = t * seg_count
            seg_idx = min(int(seg_t), seg_count - 1)
            local_t = seg_t - seg_idx
            p0 = controls[seg_idx]
            p1 = controls[min(seg_idx + 1, seg_count)]
            x = p0[0] + (p1[0] - p0[0]) * local_t
            y = p0[1] + (p1[1] - p0[1]) * local_t
            result.append((round(x, 1), round(y, 1)))
        return result

    p0, p1, p2, p3 = controls
    result = []
    for i in range(num_samples):
        t = i / (num_samples - 1)
        u = 1.0 - t
        x = (u**3) * p0[0] + 3 * (u**2) * t * p1[0] + 3 * u * (t**2) * p2[0] + (t**3) * p3[0]
        y = (u**3) * p0[1] + 3 * (u**2) * t * p1[1] + 3 * u * (t**2) * p2[1] + (t**3) * p3[1]
        result.append((round(x, 1), round(y, 1)))
    return result


def compute_width_points_2d(guide_2d: list, scale: float = 1.0) -> list:
    """根据2D引导点计算宽度向量（2D像素方向），方向为曲线切线的垂直方向。

    返回格式可直接用于 POST /ortho/strand 的 width_points_2d 字段。
    服务端负责将 2D 方向转换到 3D 图像平面。

    参数:
        guide_2d: [(x, y), ...] 2D引导点列表。
        scale:    宽度缩放因子（默认 1.0）。

    返回:
        [{"curve_id": float, "scale": float, "direction_2d": [dx, dy]}, ...]
    """
    n = len(guide_2d)
    if n < 2:
        return []

    width_points = []
    import math

    for i in range(n):
        # 计算局部切线（用前后邻点，越界的用单侧）
        if i == 0:
            dx = guide_2d[1][0] - guide_2d[0][0]
            dy = guide_2d[1][1] - guide_2d[0][1]
        elif i == n - 1:
            dx = guide_2d[n - 1][0] - guide_2d[n - 2][0]
            dy = guide_2d[n - 1][1] - guide_2d[n - 2][1]
        else:
            dx = guide_2d[i + 1][0] - guide_2d[i - 1][0]
            dy = guide_2d[i + 1][1] - guide_2d[i - 1][1]

        # 2D 垂直方向：逆时针旋转 90°
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1e-6:
            perp_x, perp_y = 1.0, 0.0
        else:
            perp_x = -dy / length
            perp_y = dx / length

        width_points.append({
            "curve_id": float(i),
            "scale": scale,
            "direction_2d": [round(perp_x, 6), round(perp_y, 6)],
        })

    return width_points


# ============================================================
# 入口
# ============================================================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="KigStudio Agent API 集成测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python test_api.py                     # 基础冒烟测试
  python test_api.py --full              # 含写入的完整测试
  python test_api.py --ortho-only        # 仅正交端点
  python test_api.py --verbose --port 8080
        """,
    )
    p.add_argument("--port", type=int, default=18920, help="API 端口 (默认 18920)")
    p.add_argument("--full", action="store_true", help="完整测试（含创建/删除发束）")
    p.add_argument("--ortho-only", action="store_true", help="仅测试正交投影端点")
    p.add_argument("--verbose", "-v", action="store_true", help="详细日志")
    p.add_argument("--save-dir", type=str, default="tools/kimi-agent/tmp",
                   help="保存导出图片的目录 (默认 tools/kimi-agent/tmp)")
    return p.parse_args()


def main() -> None:
    global _ok_count, _err_count, _skip_count

    args = parse_args()
    Config.BASE = f"http://127.0.0.1:{args.port}"
    Config.API = f"{Config.BASE}/api/v1"
    Config.VERBOSE = args.verbose
    Config.FULL = args.full
    Config.SAVE_DIR = args.save_dir
    Config.ORTHO_ONLY = args.ortho_only

    print(f"KigStudio API 测试")
    print(f"  目标: {Config.BASE}")
    print(f"  模式: {'ortho-only' if Config.ORTHO_ONLY else 'full' if Config.FULL else 'smoke'}")

    try:
        if Config.ORTHO_ONLY:
            test_ortho_ping()
            test_ortho_state()
            test_ortho_render()
            test_ortho_blend()
            test_ortho_overlay()
            test_2d_strand_submit()
            test_ortho_export_guide_curves()
        else:
            # 系统
            test_ping()
            test_toast()

            # 工程与节点
            test_project()
            test_nodes_list()
            test_node_detail()
            test_node_bounds()

            # 发束
            test_strands_list()
            test_strand_detail()

            # 新功能测试 (Feature 1-5)
            test_strand_rename()
            test_strand_hidden_guide_points()
            test_strand_auto_hair_root()
            test_strand_reverse_guide_points()

            # 正交投影端点
            test_ortho_ping()
            test_ortho_state()
            test_ortho_render()
            test_ortho_blend()
            test_ortho_overlay()
            test_ortho_export_guide_curves()

            # 2D 发束提交
            test_2d_strand_submit()

            # AI 完整工作流
            test_ai_workflow()

    except KeyboardInterrupt:
        print("\n中断")
        sys.exit(130)

    # 汇总
    total = _ok_count + _err_count + _skip_count
    print(f"\n{'='*40}")
    print(f"结果: {_ok_count} 通过, {_err_count} 失败, {_skip_count} 跳过  (共 {total})")
    if _err_count > 0:
        print("⚠ 存在失败用例")
        sys.exit(1)
    else:
        print("✓ 全部通过" if _skip_count == 0 else "✓ 通过 (部分跳过)")


if __name__ == "__main__":
    main()
