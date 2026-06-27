# kigstudio

基于体素的 3D 打印模型处理工具，支持 STL 体素化、SDF 建模、碰撞编辑、骨骼提取、链式关节设计与智能分割。

## 功能特性

### 核心处理
- **STL 体素化**：加载 STL 模型并转换为体素网格，支持自定义体素大小、精确体素化模式
- **VXGrid 导入/导出**：独立的二进制体素文件格式（zlib 压缩），支持跨项目复用体素数据
- **项目系统**：完整的项目保存/加载（JSON 元数据 + 二进制体素数据），支持 STL/SDF 自动恢复
- **异步加载**：后台线程执行 STL → BVH → 体素化 → Marching Cubes 流水线，UI 不阻塞

### SDF 建模
- **SDF 图元**：球体、立方体、圆柱体、胶囊体、锥体、多边锥体（PolyCone）、平面
- **布尔运算**：并集、交集、差集，支持构建复杂组合形状
- **空间变换**：平移、偏移、仿射变换、帧变换
- **网格转 SDF**：从 STL 网格生成 SDF，支持内部/外部测试
- **SDF 网格**：基于密集 3D 网格的 SDF 表示
- **链式关节 SDF**：参数化关节模型（球头、球窝、锥面切割、圆角、间隙），支持正/负空间布尔集成

### 碰撞与分割
- **物体编辑器**：双 Tab 设计
  - **碰撞编辑**：支持 5 种分割模式
    - 碰撞体分割（基于碰撞组，支持球体/圆柱体/胶囊体/立方体）
    - 平面分割（自定义空间平面）
    - 凹锥分割（Concave Cone — 顶点 + 闭合底面多边形）
    - 断连通块分割（Split Disconnected）
    - BFS 邻近分割（基于标记体素进行广度优先扩散）
  - **体素选择**：表面体素刷选、标记体素管理
- **混合分割**：平面 + 碰撞组可独立开关组合
- **自动/手动更新**：支持自动传播分割结果到子节点，也可手动控制

### 体素标记与刷选
- **表面体素缓存**：自动计算模型表面体素，支持后台线程初始化
- **笔刷刷选**：左键添加 / Shift+擦除标记体素，支持笔刷范围调节
- **独立历史记录**：标记体素拥有独立的撤销/重做栈（50 步），与碰撞编辑历史互不干扰
- **标记体素持久化**：保存/加载为独立的 `.vxgrid` 文件
- **复制粘贴**：Ctrl+C / Ctrl+V 复制粘贴碰撞体配置

### 骨骼与结构分析
- **EDT 骨骼提取**：密集网格 → 欧几里得距离变换 → 脊线体素检测 → 加权骨骼图 → Dijkstra 最短路径 → 中心线提取
- **CGAL 骨骼提取**：基于 Mean Curvature Flow 的网格骨架化
- **CGAL 曲面重建**：Poisson 曲面重建（从定向点云）
- **弱体素检测**：形态学开运算（腐蚀+膨胀）检测薄/弱区域
- **凸包计算**：CGAL 凸包生成

### 工作流系统
- **流程查看器**：可视化节点处理流程，支持输入/输出节点配置
- **自动执行**：按拓扑顺序自动执行工作流节点
- **力导向布局**：节点图自动布局算法

### 渲染与交互
- **多模式渲染**：Mesh 模式、Voxel 模式、原始体素模式可切换显示
- **延迟渲染管线**：基于 bgfx 的 G-Buffer 延迟渲染，多 View ID 管线
- **体素高亮浮起**：标记体素以固定世界距离浮起显示，无视距穿模问题
- **导航节点图**：可视化体素分割的树状结构，支持缩略图
- **碰撞体可视化**：线框渲染碰撞图元，支持轴 Gizmo 拖拽变换
- **鼠标世界坐标拾取**：支持 3D 场景中的精确点击拾取
- **按块增量更新**：体素渲染支持逐 Chunk 更新，无需重建整个 Mesh

### 导入导出
- **导出 ASCII STL**：将当前节点体素导出为 STL 文件
- **批量导出**：一键导出所有叶子节点到 `exported_stl/` 目录
- **重新加载 STL**：支持修改体素大小后重新体素化
- **节点作为加载源**：支持将已有节点作为数据源加载

### 高级几何
- **Cone-Box 网格闭合**：将开放三角面片投影到包围立方体，生成闭合流形网格
- **Cone-BVH 距离查询**：基于锥体包围体的层次结构，用于假发/毛发距离检索
- **八叉树**：完整八叉树实现，支持并/交/差布尔运算和 DFS 迭代器

## 技术栈

| 组件 | 说明 |
|------|------|
| C++20 | 核心语言标准（含协程） |
| [CMake](https://cmake.org/) | 构建系统 |
| [SDL2](https://www.libsdl.org/) | 窗口与输入 |
| [bgfx](https://github.com/bkaradzic/bgfx) | 跨平台图形渲染 |
| [Dear ImGui](https://github.com/ocornut/imgui) / [imnodes](https://github.com/Nelarius/imnodes) | 即时模式 UI 与节点编辑器 |
| [tinyfiledialogs](https://sourceforge.net/projects/tinyfiledialogs/) | 系统文件对话框 |
| [cJSON](https://github.com/DaveGamble/cJSON) | 项目元数据序列化 |
| [zlib](https://github.com/madler/zlib) | 体素数据压缩 |
| [CGAL](https://www.cgal.org/) | 计算几何算法库（骨架提取、曲面重建、凸包、网格简化） |
| [Eigen3](https://eigen.tuxfamily.org/) | 线性代数库 |
| [Boost](https://www.boost.org/) | C++ 通用库集合 |
| [OpenMP](https://www.openmp.org/) | 并行计算加速 |

## 构建

```bash
mkdir build && cd build
cmake ..
cmake --build . --config Debug --parallel 16
```

## 快捷键

| 快捷键 | 功能 |
|--------|------|
| `O` | 打开 STL 文件 |
| `Ctrl+O` | 加载项目 |
| `Ctrl+S` | 保存项目 |
| `Ctrl+Shift+S` | 另存为 |
| `Ctrl+Z` | 撤销（根据当前 Tab 路由到碰撞编辑或标记体素历史） |
| `Ctrl+Y` | 重做 |
| `Ctrl+C` | 复制碰撞体配置 |
| `Ctrl+V` | 粘贴碰撞体配置 |

## 项目结构

```
src/kigstudio/
├── cgal/          # CGAL 封装：凸包、网格简化、Poisson 重建、骨架提取
├── mesh/          # Cone-Box 网格闭合、Cone-BVH 距离查询
├── sdf/           # SDF 系统：图元、布尔运算、变换、网格SDF、链式关节
├── ui/            # 渲染器：轴Gizmo、碰撞可视化、Mesh/Voxel 渲染、日志
├── utils/         # 工具：Vec2/3、矩阵、KDTree、DBVT、内存池、压缩、协程生成器
└── voxel/         # 体素核心：VoxelGrid、八叉树、BVH、体素化、Marching Cubes、
                   #   EDT、骨骼提取、布尔运算、分割、射线、碰撞系统
ui/                # 应用 UI 层：主循环、延迟渲染、流程查看器、物体编辑器、导航图
shader/base/       # bgfx 着色器源码与预编译二进制
tests/             # 单元测试（11 个测试文件覆盖各子系统）
flow/              # 独立的工作流执行入口
assets/            # 测试用 STL 模型文件
scripts/           # 构建脚本与 CMake 模块
dep/               # 第三方依赖（bgfx, CGAL, Eigen3, Boost, SDL2, imnodes 等）
```

## 测试

```bash
cd build
cmake --build . --config Debug
```

测试覆盖：体素化、Marching Cubes、八叉树、DBVT、Cone-Box、Cone-BVH、碰撞检测、射线求交、SVO 等核心子系统。
