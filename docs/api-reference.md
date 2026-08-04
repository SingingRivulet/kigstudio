# KigStudio HTTP API Reference

## 基础信息

| 项目 | 说明 |
|------|------|
| 地址 | `http://127.0.0.1:18920` |
| 协议 | HTTP/1.1 REST + JSON |
| 实时推送 | WebSocket `ws://127.0.0.1:18920/api/v1/ws` |
| 字符编码 | UTF-8 |
| 超时 | 30 秒（单次请求） |

### 通用响应格式

成功：
```json
{"ok": true, ...}
```

失败：
```json
{"ok": false, "error": "错误描述", "code": "ERROR_CODE"}
```

### 错误码

| code | HTTP status | 含义 |
|------|-------------|------|
| `INVALID_PARAMS` | 400 | 参数缺失或无效 |
| `NODE_NOT_FOUND` | 404 | 节点 ID 不存在 |
| `STRAND_NOT_FOUND` | 404 | 发束索引越界 |
| `NO_MESH` | 400 | 节点没有缓存网格 |
| `NO_BVH` | 400 | 未调用 setAngleConfig 构建 BVH |
| `NO_ANGLE_CONFIG` | 400 | 该 (X,Y) 未配置角度 |
| `RAY_MISS` | 400 | 射线未命中底模 |
| `BVH_STALE` | 400 | 底模已变更，需重新 setAngleConfig |
| `UNKNOWN_METHOD` | 400 | 不支持的方法名 |
| `LOAD_FAILED` | 500 | 项目加载失败 |
| `SAVE_FAILED` | 500 | 项目保存失败 |

---

## 1. System — 系统状态

### GET /api/v1/system/status
获取系统运行状态。

```bash
curl http://127.0.0.1:18920/api/v1/system/status
```

响应：
```json
{
  "ok": true,
  "queue_running": false,
  "queue_progress": 0.0,
  "fps": 60.0,
  "memory_mb": 256,
  "queue_status": "idle",
  "node_count": 5
}
```

### GET /api/v1/system/queue
获取任务队列状态。

```bash
curl http://127.0.0.1:18920/api/v1/system/queue
```

响应：
```json
{
  "ok": true,
  "running": false,
  "progress": 0.0,
  "status": "idle"
}
```

### GET /api/v1/system/log
获取队列日志。

```bash
curl http://127.0.0.1:18920/api/v1/system/log
```

### POST /api/v1/system/wait-idle
等待队列空闲（阻塞，用于 AI agent 等待上一步操作完成）。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/system/wait-idle \
  -H "Content-Type: application/json" \
  -d '{"timeout_ms": 10000}'
```

响应：
```json
{"ok": true, "timeout": false, "waited_ms": 350}
```

### POST /api/v1/system/toast
在 GUI 中弹出提示消息。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/system/toast \
  -H "Content-Type: application/json" \
  -d '{"message": "AI agent: 开始批量生成发束", "duration_ms": 2000}'
```

---

## 2. Project — 项目管理

### GET /api/v1/project
获取当前项目信息。

```bash
curl http://127.0.0.1:18920/api/v1/project
```

响应：
```json
{
  "ok": true,
  "path": "D:/projects/head_model/project.json",
  "node_count": 8,
  "memory_mb": 512,
  "dirty": true
}
```

### POST /api/v1/project/open
打开一个项目文件。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/project/open \
  -H "Content-Type: application/json" \
  -d '{"path": "D:/projects/head_model/project.json"}'
```

### POST /api/v1/project/save
保存当前项目。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/project/save
```

> 如果 project_path 为空（新项目），会返回 `NO_PROJECT_PATH` 错误，需用 saveAs。

### POST /api/v1/project/save-as
另存为指定路径。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/project/save-as \
  -H "Content-Type: application/json" \
  -d '{"path": "D:/projects/new_project.json"}'
```

### POST /api/v1/project/create
创建空项目（清空所有节点）。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/project/create
```

---

## 3. Node — 节点管理

### GET /api/v1/nodes
列出所有节点。

```bash
curl http://127.0.0.1:18920/api/v1/nodes
```

响应：
```json
{
  "ok": true,
  "nodes": [
    {
      "id": 0,
      "title": "head_base",
      "source_type": 0,
      "source_node_id": -1,
      "segment_mode": 0,
      "dirty": false,
      "children": [1, 2],
      "root_id": 0,
      "triangle_count": 15680
    },
    {
      "id": 1,
      "title": "hair_layer_01",
      "source_type": 0,
      "source_node_id": -1,
      "segment_mode": 9,
      "dirty": false,
      "children": [],
      "root_id": 0,
      "triangle_count": 0
    }
  ]
}
```

### GET /api/v1/nodes/:id
获取单个节点详细信息。

```bash
curl http://127.0.0.1:18920/api/v1/nodes/0
```

响应包含：id, title, root_id, source_type, source_node_id, segment_mode, repair_mode, alpha_wrap_alpha, alpha_wrap_offset, subdivide_level, dirty, children, mesh 信息（triangle_count, stl_path）, voxel 信息, visibility 设置。

### POST /api/v1/nodes
创建新节点。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/nodes
```

响应：
```json
{"ok": true, "id": 3}
```

### DELETE /api/v1/nodes/:id
删除节点。

```bash
curl -X DELETE http://127.0.0.1:18920/api/v1/nodes/3
```

### PATCH /api/v1/nodes/:id
更新节点属性。

```bash
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/0 \
  -H "Content-Type: application/json" \
  -d '{
    "title": "rename_node",
    "alpha_wrap_alpha": 2.0,
    "alpha_wrap_offset": 0.02,
    "subdivide_level": 2,
    "repair_mode": 0
  }'
```

可更新字段：`title`, `alpha_wrap_alpha`, `alpha_wrap_offset`, `subdivide_level`, `repair_mode`

### GET /api/v1/nodes/:id/children
获取节点的子节点 ID 列表。

```bash
curl http://127.0.0.1:18920/api/v1/nodes/0/children
```

### GET /api/v1/nodes/:id/bounds
获取节点体素包围盒。

```bash
curl http://127.0.0.1:18920/api/v1/nodes/0/bounds
```

响应：
```json
{
  "ok": true,
  "min": {"x": -5.2, "y": -8.1, "z": -4.3},
  "max": {"x": 5.2, "y": 2.0, "z": 4.3}
}
```

---

## 4. Mesh — 网格操作

### POST /api/v1/mesh/import
导入 STL / 网格文件。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/import \
  -H "Content-Type: application/json" \
  -d '{
    "path": "D:/models/head.stl",
    "voxel_size": 0.5,
    "load_mode": 0,
    "load_as_sdf": false,
    "precision": "fast"
  }'
```

参数说明：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `path` | string | (required) | STL 文件路径 |
| `node_id` | int | -1 | 目标节点 ID，-1 表示创建新节点 |
| `voxel_size` | float | 0.5 | 体素大小 |
| `load_mode` | int | 0 | 加载模式：0=默认，1=轮廓，2=仅表面，3=仅网格，4=凸包 |
| `load_as_sdf` | bool | false | 是否作为 SDF 加载 |
| `precision` | string | "fast" | 体素精度："fast" 或 "precise" |

### POST /api/v1/mesh/export
导出单个节点网格为 STL。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/export \
  -H "Content-Type: application/json" \
  -d '{
    "node_id": 0,
    "path": "D:/export/head_exported.stl",
    "mode": 0,
    "simplify": false,
    "simplify_ratio": 0.1,
    "subdivisions": 3
  }'
```

### POST /api/v1/mesh/export-all
导出所有节点网格。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/export-all \
  -H "Content-Type: application/json" \
  -d '{"export_dir": "D:/export/", "mode": 0}'
```

### POST /api/v1/mesh/repair
修复网格（同步操作，耗时取决于网格规模）。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/repair \
  -H "Content-Type: application/json" \
  -d '{
    "node_id": 0,
    "method": "alpha_wrap",
    "alpha": 1.0,
    "offset": 0.01
  }'
```

method 可选值：

| method | 说明 |
|--------|------|
| `alpha_wrap` | Alpha Wrap（推荐） |
| `fill_holes` | 填充孔洞 |
| `stitch_borders` | 缝合边界 |
| `merge_vertices` | 合并重复顶点 |
| `orient_volume` | 定向体 |

### POST /api/v1/mesh/subdivide
细分网格。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/subdivide \
  -H "Content-Type: application/json" \
  -d '{"node_id": 0, "level": 2}'
```

`level`: 1=最粗，数值越大越密。

### POST /api/v1/mesh/simplify
简化网格。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/simplify \
  -H "Content-Type: application/json" \
  -d '{"node_id": 0, "ratio": 0.3}'
```

`ratio`: 保留比例，0~1，越小越简化（0.3 = 保留 30% 的面）。

### POST /api/v1/mesh/boolean-union
两个节点的网格布尔并集。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/mesh/boolean-union \
  -H "Content-Type: application/json" \
  -d '{"node_a": 0, "node_b": 1}'
```

响应：
```json
{"ok": true, "result_triangle_count": 8420}
```

### GET /api/v1/mesh/:id/is-manifold
检查网格是否为流形（可用于布尔运算）。

```bash
curl http://127.0.0.1:18920/api/v1/mesh/0/is-manifold
```

响应：
```json
{"ok": true, "is_manifold": true}
```

---

## 5. Strand — 发束管理

发束（Hair Strand）是毛发建模的核心数据结构。每根发束由引导曲线（guide_points）、宽度向量（width_points）、截面形状（section）和精度/修复参数组成。

一个节点可包含多根发束，它们共享中心点（addon_center_point）和附加件选项（addon options）。

### GET /api/v1/nodes/:id/strands
列出节点下所有发束。

```bash
curl http://127.0.0.1:18920/api/v1/nodes/1/strands
```

响应：
```json
{
  "ok": true,
  "strand_count": 3,
  "strands": [
    {
      "index": 0,
      "name": "刘海",
      "guide_point_count": 5,
      "width_point_count": 3,
      "mesh_dirty": true,
      "repair_failed": false
    },
    {
      "index": 1,
      "name": "右侧鬓角",
      "guide_point_count": 3,
      "width_point_count": 2,
      "mesh_dirty": false,
      "repair_failed": false
    }
  ],
  "center_point": {"x": 0.0, "y": -5.0, "z": 0.0, "show": true},
  "addon_options": {
    "addon_type": 0,
    "base_node_id": -1,
    "reveal": false,
    "split": false,
    "sdf_boolean": true,
    "sdf_split": true
  }
}
```

### GET /api/v1/nodes/:id/strands/:index
获取单根发束完整数据。

```bash
curl http://127.0.0.1:18920/api/v1/nodes/1/strands/0
```

返回所有字段：name, section_rotation, guide_samples_per_segment, section_subdiv, repair_alpha, repair_offset, mesh_dirty, repair_failed, expanded, guide_points, width_points（含 per-point section_override）, section_state。

### POST /api/v1/nodes/:id/strands
创建新发束。

```bash
# 自动命名
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands

# 指定名称
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands \
  -H "Content-Type: application/json" \
  -d '{"name": "刘海"}'
```

响应：
```json
{"ok": true, "strand_index": 0}
```

### DELETE /api/v1/nodes/:id/strands/:index
删除发束。自动调整活跃编辑索引（如果正在编辑被删除的发束）。

```bash
curl -X DELETE http://127.0.0.1:18920/api/v1/nodes/1/strands/0
```

### PATCH /api/v1/nodes/:id/strands/:index
更新发束属性。**所有字段均为可选**，发送的字段会被更新，未发送的保持不变。

#### 更新基本参数：
```bash
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "name": "前额发束",
    "section_rotation": -15.0,
    "guide_samples_per_segment": 64,
    "section_subdiv": 16,
    "repair_alpha": 2.0,
    "repair_offset": 0.02
  }'
```

#### 设置引导曲线点（3D 世界坐标）：
```bash
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "guide_points": [
      [0.0, 1.5, 0.0],
      [0.0, 0.5, 1.0],
      [0.0, -0.5, 2.0]
    ]
  }'
```

#### 设置宽度向量：
```bash
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "width_points": [
      {
        "curve_id": 0.3,
        "scale": 1.2,
        "direction": [0.0, -0.8, 0.6],
        "section_override": {
          "vertices": [{"x": -0.2, "y": -0.2}, {"x": 0.2, "y": -0.2}, {"x": 0.0, "y": 0.3}],
          "use_bezier": true
        }
      },
      {
        "curve_id": 1.7,
        "scale": 0.8,
        "direction": [0.0, -0.7, 0.7]
      }
    ]
  }'
```

width_point 字段：

| 参数 | 类型 | 说明 |
|------|------|------|
| `curve_id` | float | 引导曲线上的位置：整数=段索引，小数=段内参数 t（0~1） |
| `scale` | float | 宽度缩放（1.0=原始宽度） |
| `direction` | [x,y,z] | 从曲线指向表面（或反方向）的单位向量 |
| `section_override` | object | 可选，该位置的独立截面覆盖 |

#### 设置全局截面形状：
```bash
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "section_vertices": [
      {"x": -0.3, "y": -0.3},
      {"x": 0.3, "y": -0.3},
      {"x": 0.3, "y": 0.3},
      {"x": -0.3, "y": 0.3}
    ],
    "section_use_bezier": true
  }'
```

所有可更新字段一览：
`name`, `section_rotation`, `guide_samples_per_segment`, `section_subdiv`, `repair_alpha`, `repair_offset`, `guide_points`, `width_points`, `section_vertices`, `section_use_bezier`

### POST /api/v1/nodes/:id/strands/:index/move
调整发束在节点中的顺序。

```bash
# 上移
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/1/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "up"}'

# 下移
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "down"}'
```

### PUT /api/v1/nodes/:id/strands/center-point
设置发根汇聚中心点（所有发束共享）。

```bash
curl -X PUT http://127.0.0.1:18920/api/v1/nodes/1/strands/center-point \
  -H "Content-Type: application/json" \
  -d '{
    "x": 0.0,
    "y": -5.0,
    "z": 0.0,
    "show": true
  }'
```

设置中心点后所有发束的 `mesh_dirty` 会自动标记为 true。

### PUT /api/v1/nodes/:id/addon-options
设置附加件选项。

```bash
curl -X PUT http://127.0.0.1:18920/api/v1/nodes/1/addon-options \
  -H "Content-Type: application/json" \
  -d '{
    "addon_type": 0,
    "base_node_id": 0,
    "reveal": true,
    "split": false,
    "sdf_boolean": true,
    "sdf_split": true
  }'
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `addon_type` | int | 附加件类型：0=毛发（HAIR） |
| `base_node_id` | int | 底模节点 ID，-1 表示未选择 |
| `reveal` | bool | 显露模式：用 SDF/几何减去底模 |
| `split` | bool | 拆分模式：每根发束生成独立节点 |
| `sdf_boolean` | bool | 显露使用 SDF 布尔（false=几何布尔） |
| `sdf_split` | bool | 拆分使用 SDF 相减（false=几何布尔） |

---

## 6. Semantic — 头部语义坐标系

基于头部语义坐标系的射线投射接口。必须先调用 `angle-config` 设置每 (X,Y) 位置
的射线角度，才能调用语义添加接口。  


头部语义坐标系为三维连续坐标系，覆盖全头表面。AI agent 用此坐标系描述发束起点、走向和终止位置。  
三个轴均允许小数，在两个锚点之间线性插值。  

---

### X 轴 — 横向（Horizontal）  

从左到右，绕头部一周。**0 = 前正中线，右侧取正、左侧取负，±10 = 后正中线**。  

| 编号 | 锚点 | 英文 |
|------|------|------|
| 0 | 头顶 / 前正中线 | Midline / Midsagittal |
| ±1 | 鼻翼外缘 | Ala of nose |
| ±2 | 内眼角（内眦） | Inner canthus |
| ±3 | 瞳孔中心 | Pupil center |
| ±4 | 外眼角（外眦） | Outer canthus |
| ±5 | 颧弓最外侧 | Zygomatic arch (lateral) |
| ±6 | 耳屏 | Tragus |
| ±7 | 耳轮外缘 | Helix outer edge |
| ±8 | 乳突 | Mastoid process |
| ±9 | 枕骨外侧（耳后过渡区） | Lateral occipital |
| ±10 | 后正中线 | Posterior midline |

**使用规则：**
- `X=0` 永远是正中线（鼻梁→人中→下巴，或后脑正中）
- 右侧脸 = 正数（X>0），左侧脸 = 负数（X<0）
- `X=±10` 是同一条线（后脑正中），两侧在此汇合
- 超过 ±10 的值自动 wrap：`X=12` → 等价于 `X=-8`（从右侧绕到左侧）

---

### Y 轴 — 纵向（Vertical）  

从头顶向下，**Y=0 = 头顶/发旋（Vertex），正面用正数，后脑用负数**。  
符号本身指示面朝方向：`Y > 0` 正面，`Y < 0` 后脑。

**正面（Y ≥ 0）：**

| 编号 | 锚点 | 英文 |
|------|------|------|
| 0 | 头顶 / 发旋 / 前发际起点 | Vertex / Crown |
| +1 | 额头发际线 | Forehead hairline |
| +2 | 眉毛上缘 | Upper brow |
| +3 | 眉毛下缘 / 眶上缘 | Lower brow / Supraorbital |
| +4 | 鼻根（Nasion） | Nasion |
| +5 | 眼上缘 / 耳上附着点 | Upper eye / Superior ear attachment |
| +6 | 眼下缘 | Lower eye |
| +7 | 鼻尖 / 耳下附着点 | Nose tip / Inferior ear attachment |
| +8 | 鼻底 / 人中 | Nasal base / Philtrum |
| +9 | 嘴唇上缘 | Upper lip |
| +10 | 口裂（唇间线） | Oral fissure |
| +11 | 嘴唇下缘 | Lower lip |
| +12 | 颏部（Menton） | Chin (Menton) |
| +13 | 下颌下缘 | Inferior mandible border |
| +14 | 颈前部（喉结水平） | Anterior neck |

**后脑（Y < 0）：**

| 编号 | 锚点 | 英文 |
|------|------|------|
| 0 | 头顶部 / 发旋 | Vertex / Crown |
| −1 | 冠状缝附近 | Coronal suture area |
| −2 | 顶骨中央 | Parietal center |
| −3 | 顶枕点 | Lambda |
| −4 | 枕骨上部 | Upper occipital |
| −5 | 枕外隆凸 | External Occipital Protuberance |
| −6 | 上项线 | Superior nuchal line |
| −7 | 枕骨下部 | Lower occipital |
| −8 | 后发际线 | Posterior hairline |
| −9 | 颈后连接处 | Posterior neck junction |
| −10 | 颈后下部 | Lower posterior neck |

**正反面过渡规则（耳侧区域）：**
- `|X| ≤ 5`（眼眶以内）：始终使用正面 Y（Y ≥ 0）
- `|X| ≥ 8`（乳突及以后）：始终使用后脑 Y（Y < 0）
- `|X| ∈ (5, 8)`（颧弓外侧到耳轮）：正面 Y 与后脑 Y 之间线性过渡
  - 例如 X=±6.5 时，若正面 Y=+7（鼻尖水平），后脑 Y=−5（枕外隆凸水平），则有效 Y ≈ (+7 + (−5)) / 2 = +1.0

---

### Z 轴 — 深度/径向（Depth / Radial）  

从皮肤表面向内到颅腔中心。**0 = 皮肤表面，负值 = 向内**。  

| 编号 | 层次 | 用途 |
|------|------|------|
| 0 | 皮肤表面 | 发根起点默认深度 |
| −1 | 皮下 / 真皮层 | 毛发植入参考 |
| −2 | 浅筋膜 / 肌肉表面 | — |
| −3 | 颅骨膜 / 骨表面 | 引导曲线可贴附于此 |
| −4 | 颅骨外板 | — |

---

### 面区域速查（Plane Index）  

| 区域 | X 范围 | Y 范围 | 说明 |
|------|--------|--------|------|
| 前额 | (−5,+5) | 0 ~ +4 | 刘海区域 |
| 鬓角（左/右） | ±(3~7) | +1 ~ +5 | Temple |
| 头顶 / 发旋 | (−10,+10) | ~0 | Crown area |
| 后脑上部 | (−10,+10) | −1 ~ −4 | Upper back |
| 枕部 | (−10,+10) | −4 ~ −7 | Occipital |
| 后颈发际 | (−10,+10) | −7 ~ −9 | Nape hairline |
| 侧发区（耳上） | ±(6~8) | +1 ~ +5 | Sideburn / above ear |
| 侧发区（耳后） | ±(7~9) | −5 ~ −7 | Behind ear |

---

### AI Agent 使用示例  

```
# 在右鬓角（X=+5, Y=+3, Z=0）开始一根发束
POST /api/v1/nodes/0/strands
{"name": "右侧鬓角发束"}

# 添加引导点：从鬓角向上到头顶
PATCH /api/v1/nodes/0/strands/0
{"guide_points": [[5.0, 3.0, 0.0], [4.0, 1.5, -1.0], [2.0, 0.5, -1.0]]}

# 语义："眉毛外侧、眼眶上缘"水平，左右约颧弓位置
"X≈±5, Y≈+3~+5" → 眉尾到太阳穴区域
```


### 坐标系参数说明

| 参数 | 范围 | 说明 |
|------|------|------|
| `x` | −10 ~ +10 | 横向：0=中线，正=右，负=左，±10=后中线 |
| `y` | −10 ~ +14 | 纵向：0=头顶，正=正面向下，负=后脑向下 |
| `theta` | −180 ~ +180° | 方位角：0°=正面(+Z)，+90°=右侧(+X) |
| `phi` | −90 ~ +90° | 仰角：0°=水平，+90°=正上方(+Y) |

### POST /api/v1/nodes/:id/hair/angle-config
**必须先调用**。为每个 (X,Y) 位置配置射线角度，同时从底模构建 BVH 加速结构。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/hair/angle-config \
  -H "Content-Type: application/json" \
  -d '{
    "base_node_id": 0,
    "north_pole": [0.0, 1.0, 0.0],
    "front_reference": [0.0, 0.0, 1.0],
    "angles": [
      {"x": 0.0, "y": 1.0, "theta": 0.0, "phi": 60.0},
      {"x": 1.0, "y": 1.0, "theta": 15.0, "phi": 55.0},
      {"x": -1.0, "y": 1.0,"theta": -15.0, "phi": 55.0},
      {"x": 5.0, "y": 3.0, "theta": 55.0, "phi": 30.0},
      {"x": -5.0, "y": 3.0,"theta": -55.0, "phi": 30.0},
      {"x": 3.0, "y": -3.0,"theta": 40.0, "phi": 15.0},
      {"x": -3.0, "y": -3.0,"theta": -40.0, "phi": 15.0}
    ]
  }'
```

`theta` / `phi` 定义了射线方向。`north_pole`（可选，默认 `[0, 1, 0]`）定义球面坐标系的北极方向（`phi=+90°`），即世界坐标 +Y 轴。`front_reference`（可选，默认 `[0, 0, 1]`）与 `north_pole` 共同确定鼻尖所在的矢状面，投影后为 `theta=0°`（正前方）方向。射线从该方向（头外部远处）射向中心点，与底模的交点即为表面坐标。

响应：
```json
{
  "ok": true,
  "angle_count": 7,
  "bvh_triangle_count": 15680
}
```

> **底模变更后**，后续语义添加调用会返回 `BVH_STALE` 错误，需重新调用本接口重建 BVH。

### POST /api/v1/nodes/:id/strands/:index/guide-points/semantic
用语义坐标 (X,Y) 添加引导点。系统查找该位置的角度配置，发射射线到底模表面，
取第一个交点添加到发束的 guide_points。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/guide-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 1.0}'
```

响应：
```json
{
  "ok": true,
  "point": [0.02, 0.87, 2.34],
  "guide_point_index": 0
}
```

| 错误 | 说明 |
|------|------|
| `NO_BVH` | 未调用 angle-config |
| `NO_ANGLE_CONFIG` | 该 (X,Y) 未在 angle-config 中配置 |
| `BVH_STALE` | 底模已变更，需重新调用 angle-config |
| `RAY_MISS` | 射线未命中底模（角度不对或中心点有问题） |

### POST /api/v1/nodes/:id/strands/:index/width-points/semantic
用语义坐标 (X,Y) 添加宽度向量。同样通过射线投射找到表面点，再投影到
引导曲线上计算 `curve_id`，方向为从曲线到表面的单位向量。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/width-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 1.0, "scale": 1.2}'
```

响应：
```json
{
  "ok": true,
  "width_point_index": 0,
  "width_point": {
    "curve_id": 0.0,
    "scale": 1.2,
    "direction": [0.05, -0.82, 0.57],
    "surface_point": [0.02, 0.87, 2.34]
  }
}
```

> **前提**：发束必须已有 ≥ 2 个引导点（用于计算 curve_id）。

---

## 7. WebSocket — 实时事件

### ws://127.0.0.1:18920/api/v1/ws

连接后服务器推送 JSON 格式的进度事件：

```json
{"type": "progress", "data": {"progress": 0.5, "status": "repairing mesh..."}}
```

Python 示例：
```python
import websocket
import json

ws = websocket.create_connection("ws://127.0.0.1:18920/api/v1/ws")
while True:
    msg = json.loads(ws.recv())
    print(f"[{msg['type']}] {msg.get('data')}")
```

---

## 典型工作流

### 完整发束生成流程

```bash
# 1. 检查状态
curl http://127.0.0.1:18920/api/v1/system/status

# 2. 导入头部底模
curl -X POST http://127.0.0.1:18920/api/v1/mesh/import \
  -H "Content-Type: application/json" \
  -d '{"path": "D:/models/head.stl", "voxel_size": 0.5}'
# → {"ok": true, "triangle_count": 15680}
# 假设返回的节点 ID 是 0（或从 nodes 列表查询）

# 3. 修复底模网格（确保 manifold）
curl -X POST http://127.0.0.1:18920/api/v1/mesh/repair \
  -H "Content-Type: application/json" \
  -d '{"node_id": 0, "method": "alpha_wrap", "alpha": 1.0, "offset": 0.01}'

# 4. 创建发束节点
curl -X POST http://127.0.0.1:18920/api/v1/nodes
# → {"ok": true, "id": 1}

# 5. 设置附加件选项（绑定底模）
curl -X PUT http://127.0.0.1:18920/api/v1/nodes/1/addon-options \
  -H "Content-Type: application/json" \
  -d '{"base_node_id": 0, "reveal": true, "sdf_boolean": true}'

# 6. 设置中心点
curl -X PUT http://127.0.0.1:18920/api/v1/nodes/1/strands/center-point \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": -5.0, "z": 0.0, "show": true}'

# 7. 创建发束
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands \
  -H "Content-Type: application/json" \
  -d '{"name": "刘海"}'

# 8. 设置角度配置（构建 BVH）
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/hair/angle-config \
  -H "Content-Type: application/json" \
  -d '{
    "base_node_id": 0,
    "north_pole": [0.0, 1.0, 0.0],
    "front_reference": [0.0, 0.0, 1.0],
    "angles": [
      {"x": 0.0, "y": 1.0, "theta": 0, "phi": 60},
      {"x": 0.0, "y": 2.0, "theta": 0, "phi": 50}
    ]
  }'

# 9. 用语义坐标添加引导点
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/guide-points/semantic \
  -H "Content-Type: application/json" -d '{"x": 0.0, "y": 1.0}'

curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/guide-points/semantic \
  -H "Content-Type: application/json" -d '{"x": 0.0, "y": 2.0}'

# 10. 添加宽度向量
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/width-points/semantic \
  -H "Content-Type: application/json" -d '{"x": 0.0, "y": 1.0, "scale": 1.0}'

curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/width-points/semantic \
  -H "Content-Type: application/json" -d '{"x": 0.0, "y": 2.0, "scale": 0.8}'

# 11. 更新截面和精度参数
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "section_rotation": -10.0,
    "guide_samples_per_segment": 48,
    "section_subdiv": 12
  }'

# 12. 保存项目
curl -X POST http://127.0.0.1:18920/api/v1/project/save-as \
  -H "Content-Type: application/json" \
  -d '{"path": "D:/projects/hair_result.json"}'
```

---

## CLI 启动参数

```
kigstudio.exe [--agent-port PORT] [--no-agent]
```

| 参数 | 说明 |
|------|------|
| `--agent-port 18920` | 指定 HTTP 服务端口（默认 18920） |
| `--no-agent` | 禁用 HTTP 服务 |

---

## 8. MCP (Model Context Protocol)

KigStudio 内建 MCP 支持（SSE 传输），兼容 Claude Desktop、Claude Code 等 MCP 客户端。

### 协议

| 项目 | 说明 |
|------|------|
| 协议版本 | MCP 2024-11-05 |
| 传输 | SSE (Server-Sent Events) over HTTP |
| 端点 | `GET /mcp/sse` + `POST /mcp/messages` |
| JSON-RPC | 2.0 |

### 工作流程

```
Client                          KigStudio
  |                                 |
  |── GET /mcp/sse ───────────────>|  打开 SSE 连接
  |<── event: endpoint              |
  |    data: /mcp/messages?         |
  |    sessionId=abc123... ────────|  返回 session 端点
  |                                 |
  |── POST /mcp/messages ────────>|  发送 JSON-RPC 请求
  |   ?sessionId=abc123            |
  |   {"method":"tools/list",...}   |
  |   (HTTP 202 Accepted) ────────|  
  |<── event: message              |  结果通过 SSE 返回
  |    data: {"result":{...}} ────|
  |                                 |
  |── POST /mcp/messages ────────>|  调用工具
  |   {"method":"tools/call",...}   |
  |<── event: message              |  结果通过 SSE 返回
  |    data: {"result":{...}} ────|
```

### Claude Desktop 配置

在 `claude_desktop_config.json` 或项目的 `.claude/mcp.json` 中添加：

```json
{
  "mcpServers": {
    "kigstudio": {
      "url": "http://127.0.0.1:18920/mcp/sse"
    }
  }
}
```

> **前提**：KigStudio 必须先启动（`kigstudio.exe`），HTTP server 默认监听 18920 端口。

### 可用工具

MCP 的 `tools/list` 返回所有已注册的工具，共 30+ 个，涵盖：

| 分类 | 工具数 | 示例 |
|------|--------|------|
| system | 5 | `system_status`, `system_wait_idle`, `system_toast` |
| project | 5 | `project_open`, `project_save_as`, `project_create` |
| node | 7 | `node_list`, `node_get`, `node_create`, `node_update` |
| mesh | 8 | `mesh_import`, `mesh_export`, `mesh_repair`, `mesh_subdivide`, `mesh_boolean_union` |
| strand | 8 | `strand_list`, `strand_create`, `strand_update`, `strand_move` |
| semantic | 3 | `strand_set_angle_config`, `strand_add_semantic_guide_point`, `strand_add_semantic_width_point` |

每个工具都带有完整的 JSON Schema（inputSchema），AI 模型可以自动理解参数格式并生成正确的调用。

### 调试：用 curl 测试 MCP

```bash
# 1. 打开 SSE 连接（终端1，持续运行）
curl -N http://127.0.0.1:18920/mcp/sse
# 输出:
# event: endpoint
# data: /mcp/messages?sessionId=a1b2c3d4e5f67890
# (此后阻塞等待消息)

# 2. 发送 JSON-RPC 请求（终端2，使用上面返回的 sessionId）
SESSION="a1b2c3d4e5f67890"

# Initialize 握手
curl -X POST "http://127.0.0.1:18920/mcp/messages?sessionId=$SESSION" \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":"1","method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"curl","version":"1.0"}}}'

# 获取工具列表
curl -X POST "http://127.0.0.1:18920/mcp/messages?sessionId=$SESSION" \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":"2","method":"tools/list","params":{}}'

# 调用工具
curl -X POST "http://127.0.0.1:18920/mcp/messages?sessionId=$SESSION" \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":"3","method":"tools/call","params":{"name":"system_status","arguments":{}}}'

# 终端1 的 SSE 流会逐条返回 JSON-RPC 响应
```
