# KigStudio 正交编辑器 API — 截图拉取与引导线提交

本文档介绍如何通过 HTTP API 从 KigStudio 正交投影编辑器拉取渲染截图，在外部工具中对截图进行发束引导线标注，再将标注结果提交回 KigStudio。

---

## 前置条件

1. **启动 KigStudio**，打开一个包含发束节点（hair addon node）的工程。
2. **打开正交投影编辑器**：在节点属性面板中点击进入正交编辑窗口。窗口打开后，内嵌的 HTTP Agent API 自动启动（默认端口 `18920`）。
3. **确认 API 可用**：

```bash
curl http://127.0.0.1:18920/api/v1/ortho/ping
# → {"ok":true,"service":"kigstudio-ortho-api"}
```

---

## 架构概览

```
┌─────────────────────────────────────────────────┐
│           KigStudio (GUI + HTTP API)             │
│                                                  │
│  正交投影编辑器 ──→ 实时渲染 (bgfx GPU)           │
│       │                  │                       │
│       │  参考图叠加      │  底模渲染               │
│       │  (overlay)       │  (base model)          │
│       │                  │                       │
│       ▼                  ▼                       │
│  ┌──────────────────────────────────────────┐   │
│  │    Agent Server (cpp-httplib :18920)     │   │
│  │  GET  /api/v1/ortho/render   底模截图     │   │
│  │  GET  /api/v1/ortho/overlay  参考图       │   │
│  │  GET  /api/v1/ortho/blend    混合图       │   │
│  │  GET  /api/v1/ortho/state    相机参数     │   │
│  │  POST /api/v1/ortho/strand   提交引导线   │   │
│  └──────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
          ↕ HTTP (localhost:18920)
┌──────────────────────┐
│  外部 AI / 标注工具   │
│  (Python, curl, …)   │
│                      │
│  1. 拉取 blend 截图   │
│  2. 标注发束引导线    │
│  3. 2D 像素 → 3D 转换 │
│  4. POST 提交回 KigStudio │
└──────────────────────┘
```

---

## 一、拉取截图

### 1.1 获取底模渲染图

`GET /api/v1/ortho/render`

返回当前正交投影的底模渲染结果（纯 3D 模型，不含参考图叠加）。

```bash
curl http://127.0.0.1:18920/api/v1/ortho/render -o render.png
```

- 返回格式：`image/png`
- 分辨率：由正交编辑器的 `render_resolution` 决定（默认 2048×2048）
- 如果渲染数据尚未就绪（正交编辑器刚打开），返回 `503`

### 1.2 获取参考图（Overlay）

`GET /api/v1/ortho/overlay`

返回用户在正交编辑器中加载的参考图片原始数据。

```bash
curl http://127.0.0.1:18920/api/v1/ortho/overlay -o overlay.png
```

- 未加载参考图时返回 `404`
- 返回的是原始图片数据（保持原始分辨率）

### 1.3 获取混合图（Blend）⭐ 推荐

`GET /api/v1/ortho/blend?ratio=0.5`

将底模渲染与参考图按比例混合，返回一张 PNG。这是标注引导线时最常用的端点——AI 可以同时看到底模轮廓和参考图细节。

```bash
# 50% 混合（底模和参考图各占一半）
curl "http://127.0.0.1:18920/api/v1/ortho/blend?ratio=0.5" -o blend_50.png

# 纯底模
curl "http://127.0.0.1:18920/api/v1/ortho/blend?ratio=0.0" -o blend_0.png

# 纯参考图
curl "http://127.0.0.1:18920/api/v1/ortho/blend?ratio=1.0" -o blend_1.png
```

| 参数 | 说明 |
|------|------|
| `ratio` | 0.0 ~ 1.0，控制参考图叠加透明度。0.0 = 纯底模，1.0 = 纯参考图 |

混合图的分辨率与底模渲染一致（通常 2048×2048），参考图会根据其在编辑器中的位置、缩放进行叠加。

### 1.4 获取相机状态（坐标转换必需）

`GET /api/v1/ortho/state`

返回当前正交相机的完整状态，用于 2D 像素 ↔ 3D 世界坐标转换。

```bash
curl http://127.0.0.1:18920/api/v1/ortho/state
```

响应示例：

```json
{
  "version": 1,
  "resolution": 2048,
  "display_size": 600.0,
  "viewport_size": 120.5,
  "center": [0.12, 1.85, -0.33],
  "cam_right": [1.0, 0.0, 0.0],
  "cam_up": [0.0, 1.0, 0.0],
  "look_dir": [0.0, 0.0, -1.0],
  "overlay": {
    "offset_x": 124.5,
    "offset_y": 89.2,
    "scale": 0.85,
    "blend_ratio": 0.5,
    "enabled": true
  }
}
```

| 字段 | 说明 |
|------|------|
| `resolution` | 渲染分辨率（像素），通常 2048 |
| `viewport_size` | 视口在世界空间中的大小（单位） |
| `center` | 视口中心的世界坐标 `[x, y, z]` |
| `cam_right` | 相机右向量（世界空间），已归一化 |
| `cam_up` | 相机上向量（世界空间），已归一化 |
| `look_dir` | 视线方向 = `cross(cam_right, cam_up)` |
| `overlay.offset_x/y` | 参考图在视图中的偏移（像素，基于 display_size） |
| `overlay.scale` | 参考图缩放因子 |

---

## 二、坐标转换

标注引导线时，在 2D 图像上标注的是**像素坐标**。提交回 KigStudio 时需要转换为**3D 世界坐标**。

### 2.1 渲染图像素 → 3D 世界坐标

```python
def pixel_to_world(px: float, py: float, state: dict) -> list:
    """
    px, py: 渲染分辨率空间中的像素坐标（如 2048×2048 空间）
    state:  /api/v1/ortho/state 返回的相机状态
    返回:   [wx, wy, wz] 世界坐标
    """
    res = state["resolution"]          # 通常是 2048
    center = state["center"]           # [cx, cy, cz]
    cam_right = state["cam_right"]     # [rx, ry, rz]
    cam_up = state["cam_up"]           # [ux, uy, uz]
    vp = state["viewport_size"]

    # 像素 → NDC (归一化设备坐标 [-1, 1])
    # 图像 Y 向下 (0=顶), 世界 Y 向上 → Y 翻转
    ndc_x = (px / res) * 2.0 - 1.0
    ndc_y = 1.0 - (py / res) * 2.0

    half = vp * 0.5
    wx = center[0] + cam_right[0] * ndc_x * half + cam_up[0] * ndc_y * half
    wy = center[1] + cam_right[1] * ndc_x * half + cam_up[1] * ndc_y * half
    wz = center[2] + cam_right[2] * ndc_x * half + cam_up[2] * ndc_y * half

    return [round(wx, 4), round(wy, 4), round(wz, 4)]
```

### 2.2 参考图像素 → 3D 世界坐标

如果你在**原始参考图**（而非混合渲染图）上标注，需要先将参考图像素映射到渲染分辨率空间：

```python
def reference_to_render_pixel(ref_x: float, ref_y: float, state: dict) -> tuple:
    """
    将参考图上的像素坐标映射到渲染分辨率空间。
    参考图在编辑器中有独立的 offset 和 scale。
    """
    overlay = state.get("overlay", {})
    offset_x = overlay.get("offset_x", 0.0)
    offset_y = overlay.get("offset_y", 0.0)
    scale = overlay.get("scale", 1.0)
    display_size = state.get("display_size", 600.0)
    resolution = state.get("resolution", 2048)

    # 参考图像素 → 视图坐标（display_size 空间）
    view_x = offset_x + ref_x * scale
    view_y = offset_y + ref_y * scale

    # 视图坐标 → 渲染分辨率坐标
    render_x = (view_x / display_size) * resolution
    render_y = (view_y / display_size) * resolution

    return render_x, render_y

# 然后继续用 pixel_to_world(render_x, render_y, state) 转世界坐标
```

### 2.3 3D 世界坐标 → 参考图像素（反向投影）

验证/调试用——将已有的 3D 引导线投影回参考图上查看。

```python
def world_to_reference(wx: float, wy: float, wz: float, state: dict) -> tuple:
    """3D 世界坐标 → 参考图像素坐标"""
    center = state["center"]
    cam_right = state["cam_right"]
    cam_up = state["cam_up"]
    vp = state["viewport_size"]
    display_size = state.get("display_size", 600.0)
    resolution = state.get("resolution", 2048)

    half = vp * 0.5
    rel = [wx - center[0], wy - center[1], wz - center[2]]

    # 投影到 cam_right/cam_up 基
    rx = (rel[0]*cam_right[0] + rel[1]*cam_right[1] + rel[2]*cam_right[2]) / half
    ry = (rel[0]*cam_up[0] + rel[1]*cam_up[1] + rel[2]*cam_up[2]) / half

    # NDC → 渲染分辨率
    render_x = (rx * 0.5 + 0.5) * resolution
    render_y = (0.5 - ry * 0.5) * resolution

    # 渲染分辨率 → 视图坐标 → 参考图像素
    overlay = state.get("overlay", {})
    view_x = (render_x / resolution) * display_size
    view_y = (render_y / resolution) * display_size

    ref_x = (view_x - overlay.get("offset_x", 0)) / overlay.get("scale", 1)
    ref_y = (view_y - overlay.get("offset_y", 0)) / overlay.get("scale", 1)

    return ref_x, ref_y
```

---

## 三、提交引导线

### 3.1 方式一：直接提交 2D 像素坐标（推荐）⭐

`POST /api/v1/ortho/strand`

直接在渲染分辨率空间标注，将 2D 像素坐标发送给服务器。**服务器利用当前相机状态自动完成 2D→3D 转换**，你不需要手动做坐标转换。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/ortho/strand \
  -H "Content-Type: application/json" \
  -d '{
    "node_id": 1,
    "name": "刘海-中",
    "guide_points_2d": [
      [1024, 512],
      [1024, 680],
      [1024, 850],
      [1024, 1024]
    ],
    "width_points_2d": [
      {"curve_id": 0.0, "scale": 1.2, "direction_2d": [0.0, 1.0]},
      {"curve_id": 1.0, "scale": 1.0, "direction_2d": [0.0, 1.0]},
      {"curve_id": 2.0, "scale": 0.8, "direction_2d": [0.0, 1.0]},
      {"curve_id": 3.0, "scale": 0.5, "direction_2d": [0.0, 1.0]}
    ],
    "guide_samples_per_segment": 64
  }'
```

**请求参数：**

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `node_id` | int | **是** | 发束节点 ID |
| `name` | string | 否 | 发束名称。如果与已有发束同名则**更新**该发束，否则**创建**新发束 |
| `guide_points_2d` | [[x,y], ...] | **是** | 2D 引导点列表（渲染分辨率像素坐标），从发根到发梢排列 |
| `width_points_2d` | [object, ...] | 否 | 2D 宽度向量列表 |
| `guide_samples_per_segment` | int | 否 | 贝塞尔插值采样数（默认 64） |

**width_points_2d 每条的结构：**

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `curve_id` | float | **是** | 引导曲线上的位置（0.0 = 发根, N-1.x = 段内插值） |
| `scale` | float | 否 | 宽度缩放因子（1.0 = 默认宽度） |
| `direction_2d` | [dx, dy] | 否 | 2D 宽度方向向量（服务器转为 3D） |

**响应：**

```json
{
  "ok": true,
  "data": {
    "strand_index": 0,
    "created": true,
    "guide_point_count": 4,
    "surface_hits": 4,
    "width_point_count": 4
  }
}
```

| 字段 | 说明 |
|------|------|
| `strand_index` | 创建/更新的发束索引 |
| `created` | true=新建，false=更新已有 |
| `surface_hits` | 成功投射到 3D 表面的引导点数量 |

### 3.2 方式二：手动转换后提交 3D 坐标

如果你需要精确控制 3D 位置，可以先做坐标转换，再通过标准 Strand API 提交：

```bash
# 用 PATCH 直接设置 guide_points（3D 世界坐标）
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "guide_points": [
      [0.02, 0.87, 2.34],
      [0.01, 0.45, 2.89],
      [0.00, 0.05, 3.42],
      [0.00, -0.32, 3.95]
    ],
    "width_points": [
      {"curve_id": 0.0, "scale": 1.2, "direction": [0.0, -0.8, 0.6]},
      {"curve_id": 1.0, "scale": 1.0, "direction": [0.0, -0.7, 0.7]},
      {"curve_id": 2.0, "scale": 0.8, "direction": [0.0, -0.6, 0.8]},
      {"curve_id": 3.0, "scale": 0.5, "direction": [0.0, -0.5, 0.9]}
    ]
  }'
```

---

## 四、完整工作流示例（Python）

下面是一个完整的端到端示例：从 KigStudio 拉取截图 → 标注引导线 → 提交回 KigStudio。

```python
"""
完整工作流：拉取截图 → 标注引导线 → 提交回 KigStudio

依赖：pip install requests pillow numpy
"""

import requests
import json
import math
import time
from io import BytesIO
from PIL import Image

BASE = "http://127.0.0.1:18920"
ORTHO = f"{BASE}/api/v1/ortho"
API = f"{BASE}/api/v1"

# ============================================================
# Step 1: 获取相机状态
# ============================================================

state = requests.get(f"{ORTHO}/state").json()
print(f"分辨率: {state['resolution']}")
print(f"视口大小: {state['viewport_size']}")
print(f"中心点: {state['center']}")

# ============================================================
# Step 2: 拉取混合图（底模 + 参考图）
# ============================================================

blend = requests.get(f"{ORTHO}/blend?ratio=0.5")
blend_img = Image.open(BytesIO(blend.content))
blend_img.save("tools/kimi-agent/tmp/blend_for_annotation.png")
print(f"混合图已保存: {blend_img.size}")

# ============================================================
# Step 3: 在混合图上标注引导线
# ============================================================

# 这一步通常是手工或 AI 完成。这里以程序化示例代替：
# 在渲染分辨率 2048×2048 空间中标出 4 个引导点
guide_points_2d = [
    [1024, 512],    # 发根（图像上方 = 头顶）
    [1024, 680],    # 向下
    [1024, 850],    # 向下
    [1024, 1024],   # 发梢
]

# ============================================================
# Step 4: 可选 - 验证坐标转换
# ============================================================

def pixel_to_world(px, py, state):
    """2D 渲染像素 → 3D 世界坐标"""
    res = state["resolution"]
    center = state["center"]
    cr = state["cam_right"]
    cu = state["cam_up"]
    half = state["viewport_size"] * 0.5

    ndc_x = (px / res) * 2.0 - 1.0
    ndc_y = 1.0 - (py / res) * 2.0

    wx = center[0] + cr[0] * ndc_x * half + cu[0] * ndc_y * half
    wy = center[1] + cr[1] * ndc_x * half + cu[1] * ndc_y * half
    wz = center[2] + cr[2] * ndc_x * half + cu[2] * ndc_y * half
    return [round(wx, 4), round(wy, 4), round(wz, 4)]

for i, (px, py) in enumerate(guide_points_2d):
    world = pixel_to_world(px, py, state)
    print(f"  引导点 {i}: ({px}, {py})px → ({world[0]}, {world[1]}, {world[2]})")

# ============================================================
# Step 5: 计算宽度向量（垂直于引导曲线切线）
# ============================================================

def compute_width_vectors_2d(guide_2d):
    """计算 2D 宽度方向向量（垂直于曲线切线）"""
    n = len(guide_2d)
    if n < 2:
        return []

    width_points = []
    for i in range(n):
        # 切线：用前后邻点近似
        if i == 0:
            dx = guide_2d[1][0] - guide_2d[0][0]
            dy = guide_2d[1][1] - guide_2d[0][1]
        elif i == n - 1:
            dx = guide_2d[-1][0] - guide_2d[-2][0]
            dy = guide_2d[-1][1] - guide_2d[-2][1]
        else:
            dx = guide_2d[i+1][0] - guide_2d[i-1][0]
            dy = guide_2d[i+1][1] - guide_2d[i-1][1]

        # 垂直方向：逆时针旋转 90°
        length = math.sqrt(dx*dx + dy*dy) or 1.0
        perp_x = -dy / length
        perp_y = dx / length

        # 发根宽、发梢窄
        t = i / (n - 1)
        scale = 1.2 - t * 0.7  # 发根 1.2 → 发梢 0.5

        width_points.append({
            "curve_id": float(i),
            "scale": round(scale, 2),
            "direction_2d": [round(perp_x, 6), round(perp_y, 6)]
        })
    return width_points

width_2d = compute_width_vectors_2d(guide_points_2d)

# ============================================================
# Step 6: 查询发束节点 ID
# ============================================================

nodes = requests.get(f"{API}/nodes").json()
hair_nodes = [n for n in nodes.get("data", {}).get("nodes", [])
              if n.get("source_type") == 2]  # source_type=2 = addon/hair node

if not hair_nodes:
    print("错误: 没有找到发束节点 (source_type=2)")
    exit(1)

node_id = hair_nodes[0]["id"]
print(f"发束节点 ID: {node_id}")

# ============================================================
# Step 7: 提交引导线到 KigStudio
# ============================================================

strand_name = f"刘海-中-{int(time.time())}"

payload = {
    "node_id": node_id,
    "name": strand_name,
    "guide_points_2d": guide_points_2d,
    "width_points_2d": width_2d,
    "guide_samples_per_segment": 64,
}

resp = requests.post(f"{ORTHO}/strand", json=payload)
result = resp.json()

if result.get("ok"):
    idx = result["data"]["strand_index"]
    created = "新建" if result["data"].get("created") else "更新"
    print(f"✓ {created}发束 '{strand_name}' (index={idx})")
    print(f"  引导点: {result['data']['guide_point_count']}")
    print(f"  表面命中: {result['data']['surface_hits']}")
    print(f"  宽度向量: {result['data']['width_point_count']}")
else:
    print(f"✗ 提交失败: {result}")

# ============================================================
# Step 8: 验证 - 回读发束数据
# ============================================================

if result.get("ok"):
    strand_idx = result["data"]["strand_index"]
    strand_resp = requests.get(f"{API}/nodes/{node_id}/strands/{strand_idx}")
    strand_data = strand_resp.json().get("data", {}).get("strand", {})

    print(f"\n回读验证:")
    pts = strand_data.get("guide_points", [])
    print(f"  3D 引导点数量: {len(pts)}")
    if pts:
        print(f"  首个引导点: ({pts[0][0]:.4f}, {pts[0][1]:.4f}, {pts[0][2]:.4f})")
        print(f"  末个引导点: ({pts[-1][0]:.4f}, {pts[-1][1]:.4f}, {pts[-1][2]:.4f})")
```

---

## 五、使用 test_api.py 快速测试

`tools/kimi-agent/test_api.py` 提供了完整的集成测试，包含上述所有步骤的自动化验证：

```bash
# 冒烟测试（只读，不写入）
python tools/kimi-agent/test_api.py --ortho-only

# 完整测试（含写入发束，会自动清理）
python tools/kimi-agent/test_api.py --full

# 详细日志
python tools/kimi-agent/test_api.py --ortho-only --verbose
```

`--ortho-only` 模式会依次测试：
1. `GET /ortho/ping` — 服务可用性
2. `GET /ortho/state` — 相机状态
3. `GET /ortho/render` — 底模截图
4. `GET /ortho/blend?ratio=0.5` — 混合图
5. `GET /ortho/overlay` — 参考图
6. 2D 发束提交模拟（`--full` 模式实际写入）

---

## 六、批量标注工作流（AI Agent）

当 AI agent 需要对参考图上的多根发束进行标注时，推荐流程：

```
                    ┌─────────────────────┐
                    │  1. 拉取 blend 截图   │
                    │  GET /ortho/blend    │
                    └────────┬────────────┘
                             │ blend.png (2048×2048)
                             ▼
                    ┌─────────────────────┐
                    │  2. AI 分析 + 标注    │
                    │  逐根标 2D 引导点     │
                    │  输出 strands.json   │
                    └────────┬────────────┘
                             │ [{id, points_2d, width_2d}, ...]
                             ▼
                    ┌─────────────────────┐
                    │  3. 逐根 POST 提交    │
                    │  POST /ortho/strand  │
                    │  每根一次请求         │
                    └────────┬────────────┘
                             │
                             ▼
                    ┌─────────────────────┐
                    │  4. 验证 + 微调       │
                    │  GET /nodes/:id/     │
                    │  strands/:idx        │
                    └─────────────────────┘
```

### Python 批量提交示例

```python
def batch_submit_strands(node_id: int, strands: list):
    """
    批量提交发束引导线。

    strands: [{"name": str, "guide_points_2d": [[x,y],...],
               "width_points_2d": [...]}, ...]
    """
    for s in strands:
        resp = requests.post(
            f"{ORTHO}/strand",
            json={
                "node_id": node_id,
                "name": s["name"],
                "guide_points_2d": s["guide_points_2d"],
                "width_points_2d": s.get("width_points_2d", []),
                "guide_samples_per_segment": 64,
            }
        )
        result = resp.json()
        if result.get("ok"):
            idx = result["data"]["strand_index"]
            created = "新建" if result["data"].get("created") else "更新"
            print(f"  ✓ {s['name']}: {created} index={idx}, "
                  f"{result['data']['guide_point_count']} 引导点")
        else:
            print(f"  ✗ {s['name']}: {result}")
```

---

## 七、常见问题

### Q: 拉取截图返回 503？
确保正交投影编辑器窗口已打开，且完成了一次渲染（视口中有图像显示）。API 的渲染数据来自 GPU → CPU 回读，需要编辑器处于活跃状态。

### Q: 提交的引导线位置不对？
检查以下几点：
- 确认使用的是**渲染分辨率**（`resolution`，通常 2048）空间，而非参考图的原始分辨率
- 如果需要基于参考图标注，先用 `reference_to_render_pixel()` 转换坐标
- 确认相机方向（`cam_right`, `cam_up`）与标注时一致。如果用户在编辑器中旋转了视角，需要重新获取 state

### Q: 如何只更新引导线而不改变宽度向量？
调用 `POST /ortho/strand` 时省略 `width_points_2d` 字段，宽度向量保持不变。

### Q: 如何删除一根发束？
```bash
curl -X DELETE http://127.0.0.1:18920/api/v1/nodes/{node_id}/strands/{strand_index}
```

### Q: API 端口是多少？
默认 `18920`。KigStudio 启动时可通过 `--agent-port` 参数修改。

---

## 八、API 端点速查

| 方法 | 端点 | 用途 |
|------|------|------|
| `GET` | `/api/v1/ortho/ping` | 健康检查 |
| `GET` | `/api/v1/ortho/state` | 相机状态（坐标转换必需） |
| `GET` | `/api/v1/ortho/render` | 底模渲染 PNG |
| `GET` | `/api/v1/ortho/overlay` | 参考图 PNG |
| `GET` | `/api/v1/ortho/blend?ratio=0.5` | 混合渲染 + 参考图 PNG |
| `POST` | `/api/v1/ortho/strand` | **提交 2D 引导线**（自动 2D→3D 转换） |
| `GET` | `/api/v1/nodes/:id/strands` | 列出节点所有发束 |
| `GET` | `/api/v1/nodes/:id/strands/:idx` | 获取单根发束详情 |
| `PATCH` | `/api/v1/nodes/:id/strands/:idx` | 更新发束参数（含 3D guide_points） |
| `DELETE` | `/api/v1/nodes/:id/strands/:idx` | 删除发束 |
| `GET` | `/api/v1/system/status` | 系统状态 |
| `POST` | `/api/v1/system/toast` | 弹出 GUI 提示 |

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `tools/kimi-agent/test_api.py` | API 集成测试脚本 |
| `tools/kimi-agent/hair_guides.py` | 发束引导线标注与校验工具 |
| `tools/kimi-agent/hair_strands.json` | 发束标注示例数据 |
| `docs/api-reference.md` | 完整 REST API 参考 |
| `docs/hair-modeling-skill.md` | 发束建模完整指南（含语义坐标系） |
| `docs/ai-agent-api.md` | AI Agent API 架构文档 |
