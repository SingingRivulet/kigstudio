# 发束编辑器开发参考

## 核心文件

| 文件 | 用途 |
|---|---|
| `ui/render_voxel_list.h` | 数据结构定义（HairStrand, WidthPoint, CollisionEditorSnapshot 等） |
| `ui/render_voxel_render.cpp` | 渲染管线：mesh 构建 (`build_hair_strand_mesh`)，宽度向量放置 (`add_width_point_at`)，3D overlay 绘制 |
| `ui/render_voxel_list_ui_addons.cpp` | 附加件编辑器 UI：发束列表、引导曲线编辑器、宽度编辑器、截面编辑器、发根 checkbox |
| `ui/render_voxel_list_ui_addons_hair.cpp` | 毛发整体编辑器 UI（独立窗口）：角度配置、头顶坐标、批量发根管理 |
| `ui/ui.hpp` | 全局事件处理：鼠标点击添加引导点/宽度点，Ctrl+Z/Y 撤销重做 |
| `ui/render_voxel_list_history.cpp` | 撤销/重做机制实现（`push_undo_now`, `undo`, `redo`, snapshot 捕获/应用） |
| `ui/render_voxel_list_loader.cpp` | JSON 序列化/反序列化 |
| `src/kigstudio/utils/locale.cpp` | 多语言字符串 |

---

## 引导曲线编辑器

### 激活流程

1. 发束列表中点击 "Draw Guide Curve" 按钮 → 设置 `guide_curve_drawing_active = true` + 记录 `active_guide_draw_strand`
2. 与宽度编辑器互斥：打开引导曲线编辑器会自动关闭宽度编辑器，反之亦然
3. 发根编辑器（hair root editor）**不互斥**——可与其他编辑器共存

### 添加引导点

**3D 视口点击添加**（`ui.hpp`）：

- 正常点击 → `push_back()` 追加到末尾
- **Alt + 点击** → `insert(begin())` 插入到开头

编辑器窗口内通过按钮操作：上移、下移、删除、清空、反转顺序。

### 反转顺序

工具栏 "Reverse" 按钮，`std::reverse(guide_points)`。至少需要 2 个点。

---

## 宽度向量与 curve_id 坐标系统

### WidthPoint 结构

```cpp
struct WidthPoint {
    float curve_id;     // 沿引导曲线的参数化位置 [0, N_all-1]
    vec3f direction;    // 单位方向向量（从曲线指向宽度参考点）
    float scale;        // 距离（方向向量的长度 = 宽度半径）
};
```

### curve_id 坐标空间

宽度向量最初在**可见引导点空间**（curve_id ∈ `[0, N_visible-1]`）中存储。引入隐藏引导点（`hidden_guide_points_start` / `hidden_guide_points_end`）后，curve_id 需要统一到**完整曲线空间**（curve_id ∈ `[0, N_all-1]`），其中 `N_all = hidden_start + visible + hidden_end`。

### width_curve_id_v2 标志

```cpp
bool width_curve_id_v2 = false;  // HairStrand 成员
```

- `false`（旧版）：curve_id 引用**仅可见引导点**
- `true`（新版）：curve_id 引用**完整曲线**（hidden + visible + hidden_end）

#### 迁移逻辑

在 `build_hair_strand_mesh` 中，首次遇到 `N_all > N_visible` 且 `width_curve_id_v2 == false` 时自动迁移：

```cpp
float offset = static_cast<float>(strand.hidden_guide_points_start.size());
for (auto& wp : sorted_wp) wp.curve_id += offset;
for (auto& wp : strand.width_points) wp.curve_id += offset;
strand.width_curve_id_v2 = true;
```

`add_width_point_at` 中新放置的宽度向量始终设置 `width_curve_id_v2 = true`，curve_id 基于 `all_guide_points` 计算。

### 切换 auto_hair_root 时的宽度向量处理

**切换 ON**：
1. 若 `width_curve_id_v2 == false` 且有宽度向量 → `curve_id += hidden_n`（迁移到 v2 空间）

**切换 OFF**：
1. 删除落在隐藏发根区间的宽度向量（`curve_id < hidden_n - 0.001f`）
2. 剩余宽度向量：`curve_id -= hidden_n`（回退到可见空间）
3. 若 `hidden_guide_points_end` 也为空 → `width_curve_id_v2 = false`

---

## 自动发根引导点（Auto Hair Root）

### 条件

- `hair_north_pole` 长度 > 0.001
- 底模 `cached_mesh` 或 `source_triangles` 存在

### 碰撞检测

从 `center + north_pole * 500` 沿 `-north_pole` 方向发射射线，对底模三角形做 `ray_triangle_intersect`，找到第一个命中点。

### HairStrand 相关字段

```cpp
bool auto_hair_root;                              // RenderVoxelItem 级别（全局开关）
std::vector<vec3f> hidden_guide_points_start;     // 发根隐藏引导点
std::vector<vec3f> hidden_guide_points_end;       // 发尾隐藏引导点（预留）
bool hair_root_enabled;                           // 单根发束级别开关
```

### 两个编辑器中的表现

- **附加件编辑器**（`ui_addons.cpp`）：操作当前展开的发束
- **发根编辑器**（`ui_addons_hair.cpp`）：操作所有发束，有 "Update All Hair Roots" 按钮

### 3D 渲染

隐藏引导点曲线段和十字叉以**灰色**渲染（区别于正常引导点的白色/黄色），位于 `render_overlay` 引导曲线渲染区域。

---

## 撤销/重做系统

### 机制

`push_undo_now(item_id, snapshot_opt, description)` 捕获完整 `RenderVoxelItem` 快照：

```cpp
// capture_snapshot 捕获的字段包括（render_voxel_list_history.cpp）:
// collision_group, plane, concave_cone, segment_mode, sdf_split_*, chain_min_radius,
// hair_strands (完整发束数组), auto_hair_root, hair_north_pole, hair_angle_config,
// hairline_plane_*, silhouette_*, addon_renderers 等
```

`undo()` / `redo()` 通过 `capture_snapshot` → `apply_snapshot` 来回切换状态。

### 关键注意事项

1. **撤销前先保存状态**：`push_undo_now` 必须在修改数据**之前**调用
2. **ImGui Checkbox 陷阱**：`ImGui::Checkbox` 点击后立即切换值。若在 checkbox 之后才 `push_undo_now`，快照会捕获切换**后**的值。修复方法：用 `prev_auto` 保存旧值，swap 回去拍照后再 swap 回来
3. **Mesh 缓存清理**：当 `build_hair_strand_mesh` 返回空 mesh 时，必须 `addon_renderers.erase(strand.uuid)` 清除旧渲染器，否则旧 mesh 会残留显示
4. **撤销后标记 mesh_dirty**：`undo()` / `redo()` 会标记所有 `hair_strands[].mesh_dirty = true` 以触发重建

### 历史上下文

- 曾存在上下文感知撤销过滤器（`ui.hpp`），在引导曲线/宽度编辑中只允许撤销含"Guide Point"/"Width"关键词的操作。该过滤器已于 2026-08-07 移除，恢复标准撤销行为
- `begin_edit` / `end_edit` 用于多帧拖拽（如体素编辑），使用 `collision_edit_active` 标志防止重复捕获

---

## Mesh 构建管线

### build_hair_strand_mesh 流程

```
1. 合并 all_guide_points = hidden_start + guide_points + hidden_end
2. 若点数 < 2 → 返回空
3. 特殊类型（CANDIED_HAWTHORN / BRAID）→ 分发到专用 builder
   （各图元——芯柱/椭球/辫股/关节环/尖端——通过 `union_all_primitives`
   做 CGAL 增量布尔并集，保证流形输出以用于 3D 打印；单个图元并集失败时
   回退拼接，由构建后的 alpha_wrap 检查兜底流形性）
4. 普通类型：若 width_points 为空 → 返回空
5. 采样引导曲线 (Bezier/Catmull-Rom) → guide_curve 多段线
6. 若 strand.hair_root_generate 且宽度向量非空：在 sorted_wp 开头注入合成发根宽度向量
   （curve_id=0，scale=RenderVoxelItem::hair_root_vector_length，direction=第一个宽度向量的方向），
   放样截面范围因此延伸到起始位置（含隐藏灰色发根区段）
7. 宽度插值：对每个采样点，找到包围的宽度向量，Catmull-Rom 插值 scale + direction
8. 计算截面范围：从第一个宽度向量到最后一个宽度向量
9. 构建 LoftSection → 生成三角形 → alpha_wrap 修复 → 提交到 addon_renderers
```

### mesh_dirty 生命周期

1. 任何修改引导点/宽度向量/参数的操作设置 `strand.mesh_dirty = true`
2. 每帧 `update_addon_meshes()` 遍历所有 strand，仅重建 `mesh_dirty == true` 的
3. 重建后 `mesh_dirty = false`

### addon_renderers

`std::unordered_map<std::string, std::unique_ptr<RenderMesh>>`，以 `strand.uuid` 为 key。在渲染循环中遍历并提交 GBuffer。

### addon_tool_renderers（钻孔/连接面）

与 `addon_renderers` 并行的工具渲染器表，键名规范：
- `"conn"` — 拆分连接面（橙色），由 `compute_connection_faces()` 生成
- `"drill_<uuid>"` — 钻孔圆管（红色），由 `build_cylinder_mesh(points, radius, 16)` 直接放样（不插值）

前缀 `drill_` 用于避免与发束 UUID 键冲突。正常模式走 `renderGBufferAddon`（穿透显示），钻孔拾取激活（`drill_picking_active`）时走 `renderGBuffer`，此时连接面/发束可被鼠标拾取。

### 钻孔（DrillPath）

- 数据：`RenderVoxelItem::drill_paths`（`uuid/name/radius/visible/points/mesh_dirty`），随快照与项目文件持久化。
- 拾取：`ui.hpp` 中 `drill_click_valid` 分支，走 GPU world_pos 通道（同引导点），仅在 `drill_picking_active` 时生效；与其它拾取模式互斥。
- 点编辑：钻孔编辑器窗口（`render_drill_window()`）内 +/- 按钮与 +/- 键沿"指向 `addon_center_point`"方向移动 `drill_last_picked_index` 指向的点。
- 切割：`do_segment()` 中拆分与非拆分、几何与 SDF 模式都会在发束网格/SDF 上先减去所有可见钻孔圆管（`build_drill_tool_meshes()`），再执行后续布尔。
- 连接面：`compute_connection_faces()` 复现拆分布尔顺序（strand i 依次减 0..i-1），用被减 strand 的 SDF 筛选 `|sdf(v)| < eps` 的新生面；结果仅作显示，不参与碰撞。

---

## 快捷键

| 快捷键 | 功能 | 位置 |
|---|---|---|
| Ctrl+Z | 撤销 | `ui.hpp`（全局处理） |
| Ctrl+Y | 重做 | `ui.hpp`（全局处理） |
| Alt + 点击（3D视口） | 在引导曲线开头添加点 | `ui.hpp` |
| +/-（引导曲线编辑器打开时） | 将"上次修改"的引导点沿中心线方向移动 | `ui.hpp` |

### "最后修改"引导点的追踪

`RenderVoxelItem::last_modified_guide_point_index` 在以下场景被更新：
- 3D 视口点击添加引导点（末尾追加或 Alt+开头插入）
- 引导曲线编辑器表格中内联 +/- 按钮
- Edit 弹出窗口中的坐标 stepper 修改
- Pick 拾取按钮从模型表面拾取坐标

### Edit 弹出窗口本地化与拾取

Edit 按钮标签已汉化（`action.edit_guide_point` → "编辑"）。弹出窗口内新增 "Pick"（拾取）按钮，点击后进入拾取模式，下一次在底模表面上点击会将当前引导点坐标更新为拾取位置，同时标记该点为"最后修改"以支持 +/- 键继续调整。

---

## 待完成功能

| 功能 | 状态 |
|---|---|
| 引导曲线反转 | ✅ 已完成 |
| 隐藏引导点（首尾） | ✅ 已完成 |
| 自动发根引导点 | ✅ 已完成 |
| 发束改名（UUID 重生成） | ❌ 未实现 |
| 导出图片含引导线 | ❌ 未实现 |

---

## 调试常见问题

### mesh 不更新

1. 检查 `mesh_dirty` 是否被设为 `true`
2. 检查 `update_addon_meshes()` 中 `strand.visible` 是否为 `true`
3. 若 `build_hair_strand_mesh` 返回空，确认 `addon_renderers.erase()` 被调用（否则旧 mesh 残留）

### 宽度向量消失

- 检查 `width_curve_id_v2` 状态和 curve_id 范围是否匹配当前 `all_guide_points` 结构
- 切换 auto_hair_root 会删除/迁移隐藏区间的宽度向量

### 撤销不恢复预期状态

- 确认 `push_undo_now` 在数据修改**之前**调用
- 确认 `capture_snapshot` 包含目标字段（查看 `render_voxel_list_history.cpp` 中的字段列表）
- 注意 ImGui 控件可能在 `push_undo_now` 前已修改状态（如 Checkbox）

### 历史 Bug 记录

| 日期 | 问题 | 修复 |
|---|---|---|
| 2026-08-07 | 上下文感知撤销过滤器阻止外部操作撤销 | 移除过滤器，恢复标准撤销行为 (`ui.hpp`) |
| 2026-08-07 | 切换 auto_hair_root 后撤销不恢复 checkbox 状态 | `push_undo_now` 前 swap 回旧值拍快照 (`ui_addons.cpp`, `ui_addons_hair.cpp`) |
| 2026-08-07 | 宽度向量全在发根区时切换 OFF，mesh 不消失 | `tris.empty()` 时 `addon_renderers.erase()` (`render_voxel_render.cpp`) |
