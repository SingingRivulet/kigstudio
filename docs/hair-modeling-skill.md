# KigStudio 发束建模 Skill

## 概述

发束建模（Hair Strand Modeling）是 KigStudio 的核心功能。AI agent 通过 REST API 或 MCP 协议，在头部底模上自动生成毛发几何体。

**核心概念：**
- **底模**（Base Model）：头部的三角形网格，发束附着于其上
- **引导曲线**（Guide Curve）：发束的生长路径，由一系列 3D 点经贝塞尔插值得到
- **宽度向量**（Width Vector）：在引导曲线的每个位置，定义该处截面的缩放和方向
- **截面**（Cross Section）：发束横截面的 2D 闭合多边形
- **中心点**（Center Point）：发根汇聚参考点，所有发束共享
- **Loft 网格**：由引导曲线 + 截面沿曲线扫描生成的三角形网格

```
            ┌─────────────────────────┐
            │     截面 (Section)       │
            │    ┌───────────┐         │
            │    │ 闭合2D     │         │
            │    │ 多边形     │ ← 宽度向量控制缩放
            │    └───────────┘         │
            │         │               │
            │    ┌────┴────┐           │
            │    │ 引导曲线  │  ← 贝塞尔插值
            │    │ Guide    │           │
            │    │ Curve    │           │
            │    └────┬────┘           │
            │         │               │
            │    ┌────┴────┐           │
            │    │  底模    │  ← 头部网格
            │    │  表面    │           │
            │    └─────────┘           │
            │         • ← 中心点       │
            │         (C)             │
            └─────────────────────────┘
```

---

## 前置知识：头部语义坐标系

发束的位置使用头部语义坐标系描述，而非直接使用 3D 世界坐标。详见项目根目录 `api-reference.md` 的"头部语义坐标系"章节。

**快速参考：**

| 轴 | 范围 | 含义 |
|------|------|------|
| X | −10 ~ +10 | 横向：0=前正中线，正=右，负=左，±10=后正中线 |
| Y | −10 ~ +14 | 纵向：0=头顶，正=正面向下，负=后脑向下 |
| theta | −180° ~ +180° | 射线方位角：0°=正面，+90°=右侧 |
| phi | −90° ~ +90° | 射线仰角：0°=水平，+90°=正上方 |

**常用发束区域：**

| 区域 | X 范围 | Y 范围 | 典型角度(theta/phi) |
|------|--------|--------|---------------------|
| 刘海 | −5 ~ +5 | 0 ~ +2 | 0°/50°~70° |
| 鬓角 | ±(3~7) | +1 ~ +5 | ±40°~70°/20°~40° |
| 头顶/发旋 | −10 ~ +10 | ~0 | 0°~360°/80°~90° |
| 后脑上部 | −10 ~ +10 | −1 ~ −4 | 180°/40°~70° |
| 枕部 | −10 ~ +10 | −4 ~ −7 | 180°/20°~40° |
| 侧发区（耳上） | ±(6~8) | +2 ~ +5 | ±70°~90°/10°~30° |
| 侧发区（耳后） | ±(7~9) | −5 ~ −7 | ±100°~120°/10°~30° |

---

## 完整工作流

### 阶段 0：准备工作

```
1. 启动 kigstudio.exe（GUI 自动打开，HTTP server 监听 18920）
2. 确认服务正常：GET /api/v1/system/status → {"ok": true}
```

### 阶段 1：导入底模

```bash
# 导入头部 STL 模型
curl -X POST http://127.0.0.1:18920/api/v1/mesh/import \
  -H "Content-Type: application/json" \
  -d '{
    "path": "/path/to/head_model.stl",
    "voxel_size": 0.5,
    "precision": "fast"
  }'
# → {"ok": true, "triangle_count": 15680}
# 假设创建了节点 0

# 修复网格（确保 manifold，布尔运算需要）
curl -X POST http://127.0.0.1:18920/api/v1/mesh/repair \
  -H "Content-Type: application/json" \
  -d '{"node_id": 0, "method": "alpha_wrap", "alpha": 1.0, "offset": 0.01}'

# 等待修复完成
curl -X POST http://127.0.0.1:18920/api/v1/system/wait-idle \
  -H "Content-Type: application/json" \
  -d '{"timeout_ms": 30000}'
```

### 阶段 2：创建发束节点

```bash
# 创建毛发专用节点
curl -X POST http://127.0.0.1:18920/api/v1/nodes
# → {"ok": true, "id": 1}

# 配置附加件选项：绑定底模 + 启用显露
curl -X PUT http://127.0.0.1:18920/api/v1/nodes/1/addon-options \
  -H "Content-Type: application/json" \
  -d '{
    "base_node_id": 0,
    "addon_type": 0,
    "reveal": true,
    "split": false,
    "sdf_boolean": true,
    "sdf_split": true
  }'

# 设置发根汇聚中心点（约在颅腔中心偏下）
curl -X PUT http://127.0.0.1:18920/api/v1/nodes/1/strands/center-point \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": -3.0, "z": 0.0, "show": true}'
```

### 阶段 3：配置语义坐标角度表

这是连接语义坐标系与 3D 表面的关键步骤。每配置一个 `(X, Y)` 位置，就定义了从哪个方向发射射线来命中底模表面。

```bash
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/hair/angle-config \
  -H "Content-Type: application/json" \
  -d '{
    "base_node_id": 0,
    "north_pole": [0.0, 1.0, 0.0],
    "front_reference": [0.0, 0.0, 1.0],
    "angles": [
      {"x": 0.0, "y": 1.0, "theta": 0, "phi": 60},
      {"x": 0.0, "y": 2.0, "theta": 0, "phi": 45},
      {"x": 0.0, "y": 3.0, "theta": 0, "phi": 30},

      {"x": 3.0, "y": 1.0, "theta": 35, "phi": 55},
      {"x": 3.0, "y": 2.0, "theta": 35, "phi": 45},
      {"x": 3.0, "y": 3.0, "theta": 35, "phi": 30},

      {"x": -3.0, "y": 1.0, "theta": -35, "phi": 55},
      {"x": -3.0, "y": 2.0, "theta": -35, "phi": 45},
      {"x": -3.0, "y": 3.0, "theta": -35, "phi": 30},

      {"x": 6.0, "y": 2.0, "theta": 60, "phi": 35},
      {"x": 6.0, "y": 3.0, "theta": 60, "phi": 25},
      {"x": 6.0, "y": 4.0, "theta": 60, "phi": 15},

      {"x": -6.0, "y": 2.0, "theta": -60, "phi": 35},
      {"x": -6.0, "y": 3.0, "theta": -60, "phi": 25},
      {"x": -6.0, "y": 4.0, "theta": -60, "phi": 15}
    ]
  }'
# → {"ok": true, "angle_count": 15, "bvh_triangle_count": 15680}
```

`north_pole` 定义了球面坐标系的北极方向（`[x, y, z]` 向量），即 `phi=+90°` 对应的 3D 方向。默认值 `[0, 1, 0]` 表示世界坐标 +Y 轴为正上方。

`front_reference` 定义正前方参考方向。它与 `north_pole` 共同确定鼻尖所在的矢状面（sagittal plane）：`front_reference` 投影到与 `north_pole` 垂直的赤道面后，即为 `theta=0°`（正前方）方向。默认值 `[0, 0, 1]` 表示世界 +Z 轴为正前方。调整这两个向量可旋转/倾斜整个语义坐标系。

**角度配置原则：**
- `phi` 越大，射线越从上方射来 → 适合头顶和刘海
- `phi` 越小，射线越水平 → 适合侧面
- `theta` 绝对值越大，越从侧面射来 → 适合鬓角和耳侧
- 避免 `phi < 0`（从下方射来），除非做颈部发际线
- 每个要添加点的 `(X, Y)` 位置都必须预先配置角度，否则添加时会报 `NO_ANGLE_CONFIG` 错误

### 阶段 4：逐根生成发束

对每根发束重复以下步骤：

```bash
# 4a. 创建发束
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands \
  -H "Content-Type: application/json" \
  -d '{"name": "刘海-中"}'
# → {"ok": true, "strand_index": 0}

# 4b. 用语义坐标添加引导点（从发根到发梢）
# 发根：前额中部，靠近发际线
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/guide-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 1.0}'

# 中间：额头上方
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/guide-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 2.0}'

# 发梢：接近头顶
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/guide-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 3.0}'

# 4c. 用语义坐标添加宽度向量
# 发根处：较宽（scale=1.2）
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/width-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 1.0, "scale": 1.2}'

# 中间：正常宽度
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/width-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 2.0, "scale": 1.0}'

# 发梢：收窄（scale=0.6）
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/width-points/semantic \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 3.0, "scale": 0.6}'

# 4d. 微调发束参数
curl -X PATCH http://127.0.0.1:18920/api/v1/nodes/1/strands/0 \
  -H "Content-Type: application/json" \
  -d '{
    "section_rotation": -5.0,
    "guide_samples_per_segment": 48,
    "section_subdiv": 12,
    "repair_alpha": 1.5,
    "repair_offset": 0.02
  }'
```

### 阶段 5：优化和保存

```bash
# 5a. 调节发束顺序（先处理的发束会被后处理的减去）
curl -X POST http://127.0.0.1:18920/api/v1/nodes/1/strands/0/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "down"}'

# 5b. 等待所有队列任务完成
curl -X POST http://127.0.0.1:18920/api/v1/system/wait-idle \
  -H "Content-Type: application/json" \
  -d '{"timeout_ms": 60000}'

# 5c. 保存项目
curl -X POST http://127.0.0.1:18920/api/v1/project/save-as \
  -H "Content-Type: application/json" \
  -d '{"path": "/path/to/hair_project.json"}'
```

---

## 模板：常见发型

### 模板 1：齐刘海

```
特征：前额横向均匀覆盖，Y=1~3，X=−5~+5
角度：theta≈0，phi=30°~60°
引导点：每根 3-4 个点，从发际线到头顶
宽度：顶部 1.0，发梢 0.6
```

```python
# 伪代码：生成 11 根刘海发束
for x in [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]:
    strand_idx = create_strand(node_id, f"刘海-{x}")
    # 发根 → 发梢
    for y in [1.0, 1.8, 2.5, 3.5]:
        add_guide_point(node_id, strand_idx, x, y)
    # 宽度：顶部宽，发梢窄
    add_width_point(node_id, strand_idx, x, 1.0, scale=1.2)
    add_width_point(node_id, strand_idx, x, 2.5, scale=0.8)
```

### 模板 2：侧分刘海（三七分）

```
特征：向一侧梳理
左侧（X<0）：theta=−10°~−30°
右侧（X>0）：theta=+10°~+30°，从右侧梳向左侧
角度随 X 增大逐渐偏移
```

### 模板 3：双侧鬓角

```
特征：太阳穴两侧，自然下垂
位置：X=±4~±7，Y=+2~+5
角度：theta=±40°~±70°，phi=15°~35°
引导点：3-4 个点，从鬓角向下
每侧 3-5 根
```

### 模板 4：后脑全覆盖

```
特征：从头顶到后颈，覆盖整个后脑
位置：X=−10~+10（全覆盖），Y=−1~−9
角度：theta=180°（正后方），phi=20°~70°
按 Y 分层，每层 8-15 根
上层（Y=−1~−3）：phi=50°~70°，较密
中层（Y=−4~−7）：phi=25°~50°
下层（Y=−8~−9）：phi=15°~25°，后发际线
```

### 模板 5：马尾（Ponytail）

```
特征：头发向后汇聚到一点
前侧发束：从 X=±3~±5, Y=+1~+4 出发，引导点向后弯曲
后侧发束：从后脑出发，汇聚到枕外隆凸上方
汇聚点约在 X=0, Y=−5（后脑枕部）
所有发束的引导点最后一个点指向汇聚点
```

---

## AI Agent 最佳实践

### 1. 操作顺序

```
setAngleConfig → create_strands → add_guide_points → add_width_points → update_params
     ↓                                    ↓
  必须先执行                      guide_points ≥ 2 后才能
                                 add_width_points
```

### 2. 错误处理

| 常见错误 | 原因 | 处理 |
|----------|------|------|
| `NO_BVH` | 未调用 `setAngleConfig` | 先调 angle config |
| `NO_ANGLE_CONFIG` | 该 (X,Y) 未配置 | 补充 angle config 条目 |
| `RAY_MISS` | 射线角度不对 | 调整 theta/phi，确保射线能命中头部表面 |
| `BVH_STALE` | 底模已变更 | 重新调用 `setAngleConfig` |
| `NO_GUIDE_POINTS` | 发束引导点不足 | 保证 ≥2 个引导点再调 width-points/semantic |

### 3. 性能建议

- 批量添加角度配置（一次 `setAngleConfig` 覆盖所有需要的 X/Y 位置）
- 引导点和宽度点逐个添加（每次一个 `(X,Y)`，便于错误隔离）
- 大量发束时，每 10 根发束插入一次 `system_wait_idle` 检查队列状态
- 设置 `system_toast` 作为进度提示，方便用户在 GUI 中跟踪

### 4. 参数参考值

| 参数 | 推荐范围 | 说明 |
|------|----------|------|
| `guide_samples_per_segment` | 32~64 | 越大曲线越平滑，性能开销小 |
| `section_subdiv` | 8~16 | 截面细分数，影响截面光滑度 |
| `repair_alpha` | 1.0~3.0 | alpha_wrap 参数，越大越宽松 |
| `repair_offset` | 0.01~0.05 | alpha_wrap 偏移，控制修复精度 |
| `section_rotation` | −30°~+30° | 截面绕曲线切线的旋转角 |
| `scale` (宽度) | 0.3~1.5 | 发根通常 1.0~1.2，发梢 0.3~0.8 |

### 5. 发束命名规范

建议使用描述性名称方便后续识别和调整：

```
"刘海-中", "刘海-左3", "刘海-右2"
"鬓角-左-上", "鬓角-左-下"
"后脑-枕部-第1层-中"
"马尾-汇聚"
```

---

## MCP 工具速查

当通过 MCP 连接时，以下是最常用的发束工具：

| MCP 工具名 | 对应 HTTP 端点 | 用途 |
|-----------|---------------|------|
| `mesh_import` | POST /mesh/import | 导入底模 |
| `mesh_repair` | POST /mesh/repair | 修复底模 |
| `node_create` | POST /nodes | 创建发束节点 |
| `strand_set_addon_options` | PUT /nodes/:id/addon-options | 绑定底模 |
| `strand_set_center_point` | PUT /nodes/:id/strands/center-point | 设置中心点 |
| `strand_set_angle_config` | POST /nodes/:id/hair/angle-config | **配置角度表+构建BVH** |
| `strand_create` | POST /nodes/:id/strands | 创建发束 |
| `strand_add_semantic_guide_point` | POST .../guide-points/semantic | **语义添加引导点** |
| `strand_add_semantic_width_point` | POST .../width-points/semantic | **语义添加宽度向量** |
| `strand_update` | PATCH /nodes/:id/strands/:index | 微调参数 |
| `strand_list` | GET /nodes/:id/strands | 查看发束列表 |
| `system_wait_idle` | POST /system/wait-idle | 等待队列完成 |
| `project_save_as` | POST /project/save-as | 保存项目 |

---

## MCP 调用示例

AI agent 通过 MCP 自动发现工具后，可直接调用：

```
tools/call: strand_set_angle_config
arguments: {
  "node_id": 1,
  "base_node_id": 0,
  "angles": [
    {"x": 0, "y": 1, "theta": 0, "phi": 60},
    {"x": 3, "y": 2, "theta": 35, "phi": 45}
  ]
}

tools/call: strand_create
arguments: {"node_id": 1, "name": "刘海中心"}

tools/call: strand_add_semantic_guide_point
arguments: {"node_id": 1, "strand_index": 0, "x": 0, "y": 1}
# → {"ok":true, "point":[0.02, 0.87, 2.34], "guide_point_index": 0}
```
