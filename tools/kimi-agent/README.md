# kimi-agent 工具集

> 快速上手：完整的"拉图 → 标注 → 校验 → 提交"流程见 [workflow.md](workflow.md)；HTTP API 细节见 [api.md](api.md)。

## hair_guides.py — 动漫发型放样引导线标注与校验

在发型参考图上为每根发束绘制放样引导线（spine curve），并量化校验引导线是否与发束中心重合。

### 环境

- Python 3.12（本机使用 `D:\tools\conda\python.exe`）
- 依赖：`Pillow`、`numpy`

### 用法

```bash
python hair_guides.py --image <参考图.png> --strands <发束定义.json> --out <输出图.png> [--verify] [--no-label]
```

- `--strands`：JSON 数组，每根发束一项：
  ```json
  {"id": "bang_L1", "points": [[178,148],[172,200],[169,245],[168,288]], "color": "#ffe040"}
  ```
  - `points` 按**发根 → 发梢**顺序给控制点（图像像素坐标），脚本用 Catmull-Rom 样条穿过所有控制点生成平滑曲线；`color` 可省略，自动循环调色板。
- `--verify`：沿每条曲线按弧长均匀采样 60 点，在每点法线方向 ±12px 内做发束像素分类（深蓝色判据见 `is_hair()`），输出：
  - `on-hair`：采样点落在发束像素上的比例（低于 70% 通常说明线画到了背景/皮肤/饰品上）；
  - `mean|off|` / `max|off|`：法向发束像素质心相对引导线的横向偏移，>3px 说明明显偏离发束中心。
- `--no-label`：不画发束 id 文字。

### 推荐工作流

1. 按参考图目测写初版 `points`（发梢位置最关键，刘海看发梢尖、侧发看轮廓、麻花辫看编织中心）。
2. 带 `--verify` 渲染，`on-hair` 过低的先修。
3. 对偏差大的区域，用下面的"字符网格"技巧精确定位，而不是盲猜坐标：
   对目标区域按 3-5px 步长打印分类字符（`#` 发束 / `.` 深色缝隙与背景 / 空格 其他），
   发束之间的阴影缝隙会清晰显出每根发束的边界和走向，逐行读出中心点即可。
4. 放大渲染图逐区域目检，迭代 2-3 轮至 `mean|off|` ≤ 2px 且目测居中。

### 参考案例

`hair_strands.json` 是对 `C:\Users\cgoxo\Downloads\QQ_1785553946792.png`（493x435）的标注结果，
共 11 根发束：刘海 5 根（bang_L1/L2/C/R1/R2）、左右侧发（L_side/R_side）、
左右外侧翘发（L_outer/R_outer）、左右麻花辫（braid_L/R）。
最终校验：on-hair 68%-92%，mean|off| 1.0-3.2px。渲染结果见 `tmp/hair_guides_final.png`。

### 目录约定

- 脚本与可复用的 JSON 放本目录；`tmp/` 存放渲染中间产物。

## HTTP API 服务器
在正交投影编辑器中新增了嵌入式 HTTP API 服务，支持外部工具（如 kimi-agent）直接获取渲染图、覆盖图和混合结果。

启动方式
在正交投影编辑窗口的工具栏中：

点击 "启动API" 按钮，服务器在 http://127.0.0.1:19876 启动
状态指示灯显示绿色 URL
点击 "停止API" 可关闭服务器
关闭编辑窗口时自动停止
API 端点
端点	方法	说明
/ping	GET	健康检查，返回 {"ok":true}
/state	GET	当前相机状态 JSON（viewport、center、cam_right/up、overlay 参数）
/render	GET	底模渲染图的 PNG（需要先点击 "Export for AI" 导出）
/overlay	GET	加载的参考图的 PNG（如果有的话）
/blend?ratio=0.5	GET	底模+参考图混合的 PNG，ratio 参数控制混合比例（0.0=纯底模，1.0=纯参考图）
使用示例

### 在浏览器中打开混合图（默认 50% 混合）
curl http://127.0.0.1:19876/blend?ratio=0.5 -o blended.png

### 获取当前状态
curl http://127.0.0.1:19876/state

### 获取纯底模渲染
curl http://127.0.0.1:19876/render -o render.png