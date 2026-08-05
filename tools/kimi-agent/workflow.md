# 发束引导线标注 → 提交 完整工作流程

本文档记录从 KigStudio 正交编辑器拉取参考图、标注发束放样引导线、校验、提交回 KigStudio 的完整流程，
基于 2026-08-05 的实际操作整理，所有步骤均已验证可用。API 细节见 `api.md`，本文档侧重实操。

## 环境

- Python：`D:\tools\conda\python.exe`（依赖 Pillow、numpy、requests）
- KigStudio：打开含发束节点的工程 → 打开正交投影编辑器 → Agent API 自动启动在 `http://127.0.0.1:18920`
- 健康检查：`curl http://127.0.0.1:18920/api/v1/ortho/ping` → `{"ok":true,...}`

## 核心结论：坐标变换（已验证）

overlay 参考图在 blend 渲染图（2048×2048）中的位置满足**直接线性映射**：

```
render_px = overlay.offset + ref_px × overlay.scale
```

其中 `offset_x/offset_y/scale_x/scale_y` 来自 `GET /api/v1/ortho/state` 的 `overlay` 字段。
注意：**不需要**再除以 `canvas_display_size` 换算——offset/scale 已经是渲染分辨率空间下的值。
（2026-08-05 实测：offset=(158.09, 35.93), scale=3.4133，裁剪 blend 图按此式还原 493×435 参考图，
与原图 mean|diff| ≈ 0.64，仅重采样噪声。）

反向：`ref_px = (render_px - offset) / scale`。

因此提交引导线时，直接在 2048 blend 图上标注像素坐标即可 `POST /ortho/strand`，服务器自动完成 2D→3D。

## 标准流程（4 步）

### 1. 拉取数据

```bash
cd tools/kimi-agent
curl -s "http://127.0.0.1:18920/api/v1/ortho/blend?ratio=1.0" -o tmp/ortho_blend_1.0.png   # 纯参考图(2048)
curl -s "http://127.0.0.1:18920/api/v1/ortho/overlay" -o tmp/ortho_overlay_current.png      # 原始参考图
curl -s "http://127.0.0.1:18920/api/v1/ortho/state"   -o tmp/ortho_state.json               # 相机+overlay参数
```

- 标注用 `ratio=1.0`（纯参考图细节最清晰）；如需同时看底模轮廓可再拉 `ratio=0.5`。
- 返回 503 说明正交编辑器未完成首帧渲染，在视口里动一下再拉。

### 2. 标注引导线（参考图空间）

在 493×435 原始参考图空间标注（直观、与历史数据兼容），JSON 格式：

```json
[{"id": "bang_L1", "points": [[178,148],[172,200],[169,245],[168,288]]}, ...]
```

- `points` 按**发根 → 发梢**顺序，4-5 个控制点/根；脚本自动 Catmull-Rom 平滑。
- 发梢位置最关键：刘海看发梢尖、侧发看轮廓、辫子看编织中心。
- 有历史标注（`hair_strands.json`）且参考图相同时直接复用，跳过目测。

精确定位技巧（比盲猜坐标高效）：对目标区域按 3px 步长打印分类字符网格，
`#`=发束（深蓝）/`.`=深色缝隙/空格=其他，发束边界和走向清晰可读：

```python
# 见 tmp/draw_guides_blend.py 所用 is_hair()；示例在 README.md "推荐工作流" 一节
```

### 3. 绘制 + 校验

```bash
D:/tools/conda/python.exe tmp/draw_guides_blend.py --strands tmp/hair_strands_v2.json --tag v2
```

输出到 `tmp/`：标注图 `blend_guides_<tag>.png`、渲染空间坐标 `blend_strands_<tag>.json`、
校验报告 `blend_verify_<tag>.json`。脚本自动做 ref→render 坐标变换。

校验指标（沿曲线法向采样，采样半宽 = 12px × scale 自适应）：
- `on-hair`：采样点落在发束像素上的比例，**≥70% 合格**，低于则线画到了背景/皮肤/饰品上；
- `mean|off|`：法向发束质心相对引导线的横向偏移，渲染空间 **≤10px（≈参考图 3px）合格**。

已知伪影（指标偏高但视觉正确，无需修）：
- 最外侧发束的阴影面蓝度不足，不被 `is_hair()` 分类为发束 → 质心偏向主发团，mean|off| 虚高（如 R_outer）；
- 细发尖处采样窗比发束宽 → on-hair 被拉低（如 L_outer 末梢、R_side）。
- 因此**必须结合放大目检**（ReadMediaFile region 裁剪原分辨率查看），不能只看数字。

迭代：指标差 → 字符网格定位 → 改 JSON → 换 tag 重跑，2-3 轮收敛。

### 4. 提交回 KigStudio

```bash
D:/tools/conda/python.exe tmp/submit_strands.py
```

- 读取 `tmp/blend_strands_final.json` 的 `points_render`（2048 渲染空间坐标）；
- 逐根 `POST /api/v1/ortho/strand`：**同名更新、异名新建**；
- 自动计算宽度向量（垂直切线，发根 1.2 → 发梢 0.5 渐细）；
- 提交后自动回读 `GET /nodes/1/strands/:idx` 验证，报告存 `tmp/submit_report.json`；
- 检查 `surface_hits == guide_point_count`（全部命中 3D 表面）。

发束节点 ID 查询：`GET /api/v1/nodes`，找 `source_type=2` 的节点（本次为 node 1）。

删除发束：`curl -X DELETE http://127.0.0.1:18920/api/v1/nodes/1/strands/<idx>`

## 文件清单

| 文件 | 说明 |
|------|------|
| `api.md` | 正交编辑器 HTTP API 完整文档（端点、坐标转换公式、示例） |
| `README.md` | hair_guides.py 用法（注意其中 HTTP 一节端口 19876 已过时，以 api.md 的 18920 为准） |
| `hair_guides.py` | 独立模式标注/校验工具（`--image/--strands/--out/--verify`） |
| `hair_strands.json` | 历史标注（温迪参考图 493×435，11 根发束） |
| `test_api.py` | API 集成测试（`--ortho-only` 冒烟 / `--full` 含写入） |
| `tmp/draw_guides_blend.py` | **blend 图绘制+校验脚本**（ref→render 变换、自适应采样宽） |
| `tmp/submit_strands.py` | **批量提交脚本**（含回读验证） |
| `tmp/hair_strands_final_ref.json` | 最终引导点（参考图空间） |
| `tmp/blend_strands_final.json` | 最终引导点（2048 渲染空间，可直接提交） |
| `tmp/blend_guides_final.png` | 最终标注图 |
| `tmp/submit_report.json` | 提交响应 + 回读 3D 坐标记录 |

## 参考案例（2026-08-05，温迪参考图）

11 根发束：刘海 5（bang_L1/L2/C/R1/R2）、左右侧发（L_side/R_side）、左右外翘（L_outer/R_outer）、
左右麻花辫（braid_L/R）。最终校验 on-hair 75%-95%，mean|off| 1.0-3.3px（参考图空间），
11 根全部提交成功（surface_hits 全命中），服务器 3D 坐标左右对称、空间分布合理。

踩过的坑：
- overlay 坐标变换不用过 `canvas_display_size`，直接 `offset + ref × scale`（见上文"核心结论"）；
- L_outer 初版上段偏左 5px 落在帽子/背景边界（on-hair 仅 68%），用字符网格修正后 88%；
- 最外侧发束校验指标会因阴影面分类失败而虚高，需目检确认。
