# 同时带体素和sdf的物体雕刻功能

## 修改体素内存组织，便于快速处理 ✅

支持4种类型的chunk，但是暴露出相同的接口:
目前已有的类型、全部为1或0的类型（用于降低内存占用）、子树（把边长为32的chunk细分为边长16或8的子节点，便于雕刻时快速处理）

已实现：`Chunk` 重构为 Dense/AllZero/AllOne/Subtree 四种类型（src/kigstudio/voxel/voxel_chunk.h），统一 get/set/getWord/extractWords/equals/压缩/细分接口；AllOne chunk 被 clear 时自动细分为 16³ 子树。布尔运算走 orWith/andWith/andNotWith（含同质短路）。序列化升级 VXGRID2（保留 v1 读取）。测试：tests/test_voxel_chunk_types.cpp。


## sdf局部渲染更新 ✅
提供一个函数，允许指定一个区间，自动寻找对应的chunk或子chunk，对这个范围的sdf mesh进行更新

已实现：mesher 层 `generateSmoothMeshChunked` / `generateSmoothMeshForRegion`（voxel2mesh.h，无 stitch/fill 的按 chunk 生成与 AABB 局部重建），渲染层 `RenderVoxel::loadSDFChunked` / `updateSDFRegion`（render_voxel.h，复用 chunk_meshes_ 热替换）。测试：tests/test_sdf_region_mesh.cpp。

## chunk化sdf支持 ✅
现在sdf也能像体素一样被chunk化，并且支持局部更新，这样在雕刻时可以只更新被修改的chunk，而不是整个sdf，从而提高雕刻效率

已实现：`SDFChunkedGrid`（src/kigstudio/sdf/sdf_chunked.h），稀疏 chunk map（复用 packChunkKey，32³ 体素分辨率、采样点在体素中心），Uniform/Dense 两种存储状态（远场/深内部经窄带截断钳制为 ±kSDFChunkedFar 后压缩为 Uniform）。实现 SDFBase 接口（点采样三线性插值 + 批量采样 chunk 指针缓存），mesher/渲染层零改动。`updateRegion` 只触碰相交 chunk 并返回受影响 chunk key（可直接喂给 updateSDFRegion），恒等更新不分配新 chunk；`fromSDF` 把解析式 SDF 烘焙为分块场。序列化 type="chunked_grid"（sdf.cpp 注册）。测试：tests/test_chunked_sdf.cpp。

## 雕刻