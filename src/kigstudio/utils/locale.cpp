#include "locale.h"

#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <unordered_map>

#ifdef _WIN32
#include <windows.h>
#else
#include <cstdlib>
#endif

namespace sinriv::locale {

namespace {

using TranslationMap = std::unordered_map<std::string, std::string>;
using LocaleMap = std::unordered_map<std::string, TranslationMap>;

constexpr const char* kDefaultLanguage = "en";

LocaleMap locale_strings;
std::unordered_map<std::string, bool> supported_languages;
std::string current_language = kDefaultLanguage;

std::string normalize_language_code(std::string lang) {
    std::transform(lang.begin(), lang.end(), lang.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    std::replace(lang.begin(), lang.end(), '_', '-');

    const size_t dot_pos = lang.find('.');
    if (dot_pos != std::string::npos) {
        lang.erase(dot_pos);
    }

    return lang;
}

std::string resolve_language_code(const std::string& lang) {
    const std::string normalized = normalize_language_code(lang);
    if (supported_languages.find(normalized) != supported_languages.end()) {
        return normalized;
    }

    const size_t dash_pos = normalized.find('-');
    if (dash_pos != std::string::npos) {
        const std::string base = normalized.substr(0, dash_pos);
        if (supported_languages.find(base) != supported_languages.end()) {
            return base;
        }
    }

    return kDefaultLanguage;
}

void add_locale_string(
    const char* key,
    std::initializer_list<std::pair<const char*, const char*>> translations) {
    TranslationMap& entry = locale_strings[key];
    for (const auto& [language, text] : translations) {
        entry[language] = text;
        supported_languages[language] = true;
    }
}

void init_locale_strings() {
    locale_strings.clear();
    supported_languages.clear();

    add_locale_string("window.stl_loader",
                      {{"en", "STL Loader"}, {"zh", "STL 加载器"}});
    add_locale_string("window.item_status",
                      {{"en", "item status"}, {"zh", "项目状态"}});
    add_locale_string("window.async_voxel_loader",
                      {{"en", "async voxel loader"}, {"zh", "异步体素加载"}});
    add_locale_string("window.load_stl_file",
                      {{"en", "Load STL File"}, {"zh", "加载 STL 文件"}});
    add_locale_string("window.object_editor",
                      {{"en", "Object Editor"}, {"zh", "物体编辑器"}});
    add_locale_string("tab.collision_edit",
                      {{"en", "Collision Edit"}, {"zh", "碰撞编辑"}});
    add_locale_string("tab.voxel_picking",
                      {{"en", "Voxel Picking"}, {"zh", "体素选择"}});
    add_locale_string("tab.file_status",
                      {{"en", "Voxel Picking"}, {"zh", "文件"}});
    add_locale_string("window.edit_segment_plane",
                      {{"en", "Edit Segment Plane"}, {"zh", "编辑分割平面"}});
    add_locale_string("window.nav_node_map",
                      {{"en", "nav node map"}, {"zh", "导航节点图"}});
    add_locale_string("action.reset_layout",
                      {{"en", "Reset Layout"}, {"zh", "重置布局"}});
    add_locale_string("label.force_layout",
                      {{"en", "Force Layout"}, {"zh", "力导向布局"}});

    add_locale_string("menu.file", {{"en", "File"}, {"zh", "文件"}});
    add_locale_string("menu.new_node", {{"en", "New Node"}, {"zh", "新建节点"}});
    add_locale_string("menu.view", {{"en", "View"}, {"zh", "视图"}});
    add_locale_string("menu.tools", {{"en", "Tools"}, {"zh", "工具"}});
    add_locale_string("menu.debug", {{"en", "Debug"}, {"zh", "调试"}});
    add_locale_string("menu.debug_voxel_picking",
                      {{"en", "Voxel Picking"}, {"zh", "体素拾取"}});
    add_locale_string(
        "menu.check_non_manifold",
        {{"en", "Check Non-Manifold Edges"}, {"zh", "检查非流形边"}});
    add_locale_string("menu.body", {{"en", "Body"}, {"zh", "主体"}});
    add_locale_string("menu.axis", {{"en", "Axis"}, {"zh", "坐标轴"}});
    add_locale_string("menu.bound", {{"en", "Bound"}, {"zh", "边界"}});
    add_locale_string("menu.history", {{"en", "History"}, {"zh", "历史记录"}});
    add_locale_string("menu.open_stl",
                      {{"en", "Open STL (O)"}, {"zh", "打开 STL (O)"}});
    add_locale_string("menu.import_vxgrid",
                      {{"en", "Import VXGrid"}, {"zh", "导入 VXGrid"}});
    add_locale_string("dialog.vxgrid_files",
                      {{"en", "VXGrid Files"}, {"zh", "VXGrid 文件"}});
    add_locale_string("menu.save_project",
                      {{"en", "Save (Ctrl+S)"}, {"zh", "保存 (Ctrl+S)"}});
    add_locale_string(
        "menu.save_project_as",
        {{"en", "Save As (Ctrl+Shift+S)"}, {"zh", "另存为 (Ctrl+Shift+S)"}});
    add_locale_string("menu.load_project",
                      {{"en", "Load (Ctrl+O)"}, {"zh", "加载 (Ctrl+O)"}});
    add_locale_string("menu.log", {{"en", "Log"}, {"zh", "项目处理日志"}});

    // Queue log messages
    add_locale_string("log.queue.stop",
                      {{"en", "[Queue] Stop"}, {"zh", "[队列] 停止"}});
    add_locale_string("log.queue.start_load_stl",
                      {{"en", "[Queue] Start: Load STL \"%s\""},
                       {"zh", "[队列] 开始: 加载 STL \"%s\""}});
    add_locale_string("log.queue.done_load_stl",
                      {{"en", "[Queue] Done:  Load STL \"%s\""},
                       {"zh", "[队列] 完成: 加载 STL \"%s\""}});
    add_locale_string("log.queue.error_load_stl",
                      {{"en", "[Queue] Error: Load STL \"%s\" - %s"},
                       {"zh", "[队列] 错误: 加载 STL \"%s\" - %s"}});
    add_locale_string("log.queue.start_reload_stl",
                      {{"en", "[Queue] Start: Reload STL item %d \"%s\""},
                       {"zh", "[队列] 开始: 重新加载 STL 项目 %d \"%s\""}});
    add_locale_string("log.queue.done_reload_stl",
                      {{"en", "[Queue] Done:  Reload STL item %d"},
                       {"zh", "[队列] 完成: 重新加载 STL 项目 %d"}});
    add_locale_string("log.queue.error_reload_stl",
                      {{"en", "[Queue] Error: Reload STL item %d - %s"},
                       {"zh", "[队列] 错误: 重新加载 STL 项目 %d - %s"}});
    add_locale_string("log.queue.start_segment",
                      {{"en", "[Queue] Start: Segment item %d"},
                       {"zh", "[队列] 开始: 分割项目 %d"}});
    add_locale_string("log.queue.done_segment",
                      {{"en", "[Queue] Done:  Segment item %d"},
                       {"zh", "[队列] 完成: 分割项目 %d"}});
    add_locale_string("log.queue.error_segment",
                      {{"en", "[Queue] Error: Segment item %d - %s"},
                       {"zh", "[队列] 错误: 分割项目 %d - %s"}});
    add_locale_string(
        "log.queue.start_check_manifold",
        {{"en", "[Queue] Start: Check non-manifold edges for item %d"},
         {"zh", "[队列] 开始: 检查项目 %d 的非流形边"}});
    add_locale_string("log.queue.skip_item_busy",
                      {{"en", "[Queue] Skip: Item %d is busy or not found"},
                       {"zh", "[队列] 跳过: 项目 %d 正忙或不存在"}});
    add_locale_string(
        "log.queue.done_no_manifold",
        {{"en", "[Queue] Done:  No non-manifold edges found in item %d"},
         {"zh", "[队列] 完成: 项目 %d 中未找到非流形边"}});
    add_locale_string(
        "log.queue.found_manifold",
        {{"en", "[Queue] Found %d non-manifold edge(s) in item %d:"},
         {"zh", "[队列] 在项目 %d 中找到 %d 条非流形边:"}});
    add_locale_string(
        "log.queue.manifold_edge",
        {{"en",
          "  Edge: (%.6f, %.6f, %.6f) -> (%.6f, %.6f, %.6f) [%d triangles]"},
         {"zh",
          "  边: (%.6f, %.6f, %.6f) -> (%.6f, %.6f, %.6f) [%d 个三角形]"}});
    add_locale_string("log.queue.error_check_manifold",
                      {{"en", "[Queue] Error: Check non-manifold item %d - %s"},
                       {"zh", "[队列] 错误: 检查项目 %d 的非流形边 - %s"}});
    add_locale_string("log.queue.start_extract_skeleton",
                      {{"en", "[Queue] Start: Extract skeleton for item %d"},
                       {"zh", "[队列] 开始: 提取项目 %d 的骨架"}});
    add_locale_string("log.queue.done_extract_skeleton",
                      {{"en", "[Queue] Done: Extract skeleton for item %d"},
                       {"zh", "[队列] 完成: 提取项目 %d 的骨架"}});
    add_locale_string("log.queue.error_extract_skeleton",
                      {{"en", "[Queue] Error: Extract skeleton item %d - %s"},
                       {"zh", "[队列] 错误: 提取项目 %d 的骨架 - %s"}});
    add_locale_string(
        "log.queue.start_thumbnail",
        {{"en", "[Queue] Start: Generate thumbnail mesh for item %d"},
         {"zh", "[队列] 开始: 为项目 %d 生成缩略图网格"}});
    add_locale_string(
        "log.queue.done_thumbnail",
        {{"en", "[Queue] Done:  Generate thumbnail mesh for item %d"},
         {"zh", "[队列] 完成: 为项目 %d 生成缩略图网格"}});
    add_locale_string(
        "log.queue.error_thumbnail",
        {{"en", "[Queue] Error: Generate thumbnail mesh for item %d - %s"},
         {"zh", "[队列] 错误: 为项目 %d 生成缩略图网格 - %s"}});
    add_locale_string("log.queue.start_fill_interior",
                      {{"en", "[Queue] Start: Fill interior for item %d"},
                       {"zh", "[队列] 开始: 填充项目 %d 的内部"}});
    add_locale_string("log.queue.done_fill_interior",
                      {{"en", "[Queue] Done:  Filled interior for item %d"},
                       {"zh", "[队列] 完成: 项目 %d 的内部已填充"}});
    add_locale_string("log.queue.error_fill_interior",
                      {{"en", "[Queue] Error: Fill interior for item %d - %s"},
                       {"zh", "[队列] 错误: 填充项目 %d 的内部 - %s"}});
    add_locale_string(
        "log.queue.skip_check_busy",
        {{"en",
          "[Queue] Skip: Cannot check non-manifold edges - item busy or not "
          "found"},
         {"zh", "[队列] 跳过: 无法检查非流形边 - 项目正忙或不存在"}});
    add_locale_string("log.queue.unknown_error",
                      {{"en", "Unknown error"}, {"zh", "未知错误"}});
    add_locale_string(
        "log.queue.start_export_stl",
        {{"en", "[Queue] Start: Export STL for item %d to \"%s\""},
         {"zh", "[队列] 开始: 导出节点 %d 的 STL 到 \"%s\""}});
    add_locale_string(
        "log.queue.done_export_stl",
        {{"en", "[Queue] Done:  Exported STL for item %d to \"%s\""},
         {"zh", "[队列] 完成: 已导出节点 %d 的 STL 到 \"%s\""}});
    add_locale_string("log.queue.error_export_stl",
                      {{"en", "[Queue] Error: Export STL for item %d - %s"},
                       {"zh", "[队列] 错误: 导出节点 %d 的 STL - %s"}});
    add_locale_string(
        "log.queue.error_export_stl_empty",
        {{"en", "[Queue] Error: Export STL for item %d - empty mesh"},
         {"zh", "[队列] 错误: 导出节点 %d 的 STL - 网格为空"}});
    add_locale_string("log.queue.error_export_stl_all",
                      {{"en", "[Queue] Error: Batch export STL - %s"},
                       {"zh", "[队列] 错误: 批量导出 STL - %s"}});
    add_locale_string(
        "log.queue.error_export_stl_all_empty",
        {{"en", "[Queue] Error: Batch export STL - no leaf nodes"},
         {"zh", "[队列] 错误: 批量导出 STL - 没有叶子节点"}});
    add_locale_string("log.queue.simplify_result",
                      {{"en", "[Queue] Simplify: %d -> %d triangles"},
                       {"zh", "[队列] 简化: %d -> %d 个三角形"}});
    add_locale_string("log.queue.done_export_stl_all",
                      {{"en", "[Queue] Done:  Exported %d/%d STL files"},
                       {"zh", "[队列] 完成: 已导出 %d/%d 个 STL 文件"}});
    add_locale_string("log.extract_skeleton.result",
                      {{"en", "[Extract Skeleton] Result: %d vertices"},
                       {"zh", "[骨架提取]处理成功，共%d个顶点"}});
    add_locale_string("log.extract_skeleton.buildDenseGrid.result",
                      {{"en",
                        "[Extract Skeleton] Dense grid size: (%d, %d, "
                        "%d)->(%d, %d, %d) voxel_grid_data size: %d"},
                       {"zh",
                        "[骨架提取]体素覆盖范围: (%d, %d, %d)->(%d, %d, %d) "
                        "体素区块数量: %d"}});

    // Status messages
    add_locale_string("status.checking_manifold",
                      {{"en", "Checking non-manifold edges..."},
                       {"zh", "正在检查非流形边..."}});
    add_locale_string(
        "status.extracting_skeleton",
        {{"en", "Extracting skeleton..."}, {"zh", "正在提取骨架..."}});
    add_locale_string("status.generating_thumbnail",
                      {{"en", "Generating thumbnail mesh..."},
                       {"zh", "正在生成缩略图网格..."}});
    add_locale_string("status.filling_interior", {{"en", "Filling interior..."},
                                                  {"zh", "正在填充内部..."}});
    add_locale_string("status.exporting_stl",
                      {{"en", "Exporting STL: "}, {"zh", "正在导出 STL: "}});
    add_locale_string(
        "status.exporting_stl_all",
        {{"en", "Exporting all STL..."}, {"zh", "正在批量导出 STL..."}});
    add_locale_string("status.exporting_stl_all_item",
                      {{"en", "Exporting item %d (%d/%d):"},
                       {"zh", "正在导出节点 %d (%d/%d):"}});
    add_locale_string("status.exporting_stl.cleaning_mesh",
                      {{"en", "Cleaning mesh"}, {"zh", "模型去重"}});
    add_locale_string("status.exporting_stl.simplifying_mesh",
                      {{"en", "Simplifying mesh"}, {"zh", "模型简化"}});
    add_locale_string("status.exporting_stl.saveing_mesh",
                      {{"en", "Saving mesh"}, {"zh", "保存模型"}});
    add_locale_string("status.silhouette.prepare",
                      {{"en", "Preparing silhouette mesh..."},
                       {"zh", "准备轮廓网格..."}});
    add_locale_string("status.silhouette.test_center",
                      {{"en", "Detecting center point..."},
                       {"zh", "检测中心点位置..."}});
    add_locale_string("status.silhouette.clip_cones",
                      {{"en", "Clipping triangles by cone..."},
                       {"zh", "锥体裁剪三角形..."}});
    add_locale_string("status.silhouette.build_bvh",
                      {{"en", "Building acceleration structure..."},
                       {"zh", "构建加速结构..."}});
    add_locale_string("status.silhouette.ray_visibility",
                      {{"en", "Computing ray visibility..."},
                       {"zh", "计算射线可见性..."}});
    add_locale_string("status.silhouette.extract_boundary",
                      {{"en", "Extracting boundary edges..."},
                       {"zh", "提取边界边..."}});
    add_locale_string("status.silhouette.flip_normals",
                      {{"en", "Orienting normals outward..."},
                       {"zh", "调整法线方向..."}});
    add_locale_string("status.silhouette.generate_sides",
                      {{"en", "Generating side faces..."},
                       {"zh", "生成侧面..."}});
    add_locale_string("status.segmenting",
                      {{"en", "Segmenting..."}, {"zh", "正在分割..."}});
    add_locale_string(
        "status.extracting_skeleton_cgal",
        {{"en", "CGAL skeleton extraction..."}, {"zh", "CGAL 骨架提取..."}});
    add_locale_string("status.reading_stl",
                      {{"en", "Reading STL..."}, {"zh", "正在读取 STL..."}});
    add_locale_string("status.building_spatial_index",
                      {{"en", "Building spatial index..."},
                       {"zh", "正在构建空间索引..."}});
    add_locale_string("status.rasterizing_surface",
                      {{"en", "Rasterizing surface..."},
                       {"zh", "正在光栅化表面..."}});
    add_locale_string("status.voxelizing",
                      {{"en", "Voxelizing..."}, {"zh", "正在体素化..."}});
    add_locale_string("status.generating_mesh",
                      {{"en", "Generating mesh..."}, {"zh", "正在生成网格..."}});
    add_locale_string("status.generating_voxel_mesh",
                      {{"en", "Generating voxel mesh..."},
                       {"zh", "正在生成体素网格..."}});
    add_locale_string("status.preparing_mesh",
                      {{"en", "Preparing mesh..."}, {"zh", "正在准备网格..."}});
    add_locale_string("status.saving_cache",
                      {{"en", "Saving cache..."}, {"zh", "正在保存缓存..."}});
    add_locale_string("status.uploading",
                      {{"en", "Uploading..."}, {"zh", "正在上传..."}});
    add_locale_string("status.done",
                      {{"en", "Done"}, {"zh", "完成"}});
    add_locale_string("status.sdf_mesh_prefix",
                      {{"en", "SDF Mesh: "}, {"zh", "SDF 网格: "}});
    add_locale_string(
        "status.generating_silhouette_mesh",
        {{"en", "Generating silhouette mesh..."},
         {"zh", "正在生成轮廓网格..."}});

    // Labels
    add_locale_string("label.no_log_entries",
                      {{"en", "No log entries."}, {"zh", "暂无日志记录。"}});
    add_locale_string("label.load_as_sdf",
                      {{"en", "Load as SDF"}, {"zh", "加载SDF"}});
    add_locale_string(
        "label.use_precise_voxelization",
        {{"en", "Precise voxelization"}, {"zh", "精确体素化"}});
    add_locale_string("label.stl_path",
                      {{"en", "STL Path"}, {"zh", "STL 路径"}});
    add_locale_string("label.stl_load_mode",
                      {{"en", "Load Mode"}, {"zh", "加载模式"}});
    add_locale_string("label.stl_load_mode.default",
                      {{"en", "Default"}, {"zh", "默认"}});
    add_locale_string("label.stl_load_mode.sdf",
                      {{"en", "SDF"}, {"zh", "SDF"}});
    add_locale_string("label.stl_load_mode.silhouette",
                      {{"en", "Silhouette"}, {"zh", "锥化"}});
    add_locale_string("label.stl_load_mode.surface_only",
                      {{"en", "Surface Only"}, {"zh", "表面体素化"}});
    add_locale_string("label.stl_load_mode.mesh_only",
                      {{"en", "Mesh Only"}, {"zh", "仅网格"}});
    add_locale_string("label.stl_load_mode.convex_hull",
                      {{"en", "Convex Hull"}, {"zh", "凸包"}});
    add_locale_string("action.browse",
                      {{"en", "Browse..."}, {"zh", "浏览..."}});
    add_locale_string("label.silhouette_center",
                      {{"en", "Silhouette Center"}, {"zh", "轮廓中心"}});
    add_locale_string("label.show_silhouette_center",
                      {{"en", "Show Center"}, {"zh", "显示中心点"}});
    add_locale_string(
        "action.export_source_stl",
        {{"en", "Export Source as STL"}, {"zh", "导出源模型为STL"}});
    add_locale_string("menu.export_stl_all",
                      {{"en", "Export All STL"}, {"zh", "全部导出stl"}});
    add_locale_string("tooltip.export_stl_all_no_project",
                      {{"en", "Save or load a project first to enable export."},
                       {"zh", "请先保存或加载工程以启用导出。"}});
    add_locale_string("tooltip.export_stl_all_empty",
                      {{"en", "No leaf nodes found to export."},
                       {"zh", "没有找到可导出的叶子节点。"}});

    add_locale_string("action.update_collision",
                      {{"en", "update collision"}, {"zh", "更新碰撞"}});
    add_locale_string("action.extract_skeleton",
                      {{"en", "extract skeleton"}, {"zh", "提取骨架"}});
    add_locale_string("action.undo",
                      {{"en", "Undo (Ctrl+Z)"}, {"zh", "撤销 (Ctrl+Z)"}});
    add_locale_string("action.redo",
                      {{"en", "Redo (Ctrl+Y)"}, {"zh", "重做 (Ctrl+Y)"}});
    add_locale_string("action.open_file_dialog",
                      {{"en", "Open File Dialog"}, {"zh", "选择文件"}});
    add_locale_string("action.open", {{"en", "Open"}, {"zh", "打开"}});
    add_locale_string("action.cancel", {{"en", "Cancel"}, {"zh", "取消"}});
    add_locale_string("action.add_shape",
                      {{"en", "Add shape"}, {"zh", "添加形状"}});
    add_locale_string("action.delete", {{"en", "Delete"}, {"zh", "删除"}});
    add_locale_string("action.add_vertex",
                      {{"en", "add vertex"}, {"zh", "添加顶点"}});
    add_locale_string("action.add_vertex_picking",
                      {{"en", "add vertex (click in scene...)"},
                       {"zh", "添加顶点（在场景中点击...）"}});
    add_locale_string("action.clear_vertices",
                      {{"en", "clear vertices"}, {"zh", "清空顶点"}});
    add_locale_string(
        "action.init_all_joint_radii",
        {{"en", "Initialize All Radii"}, {"zh", "初始化所有半径"}});
    add_locale_string("action.auto_detect_joint_radius",
                      {{"en", "Auto Detect"}, {"zh", "自动识别"}});
    add_locale_string("action.replace", {{"en", "replace"}, {"zh", "替换"}});
    add_locale_string("action.replace_picking",
                      {{"en", "replace (...)"}, {"zh", "替换 (...)"}});
    add_locale_string("action.insert_after",
                      {{"en", "insert after"}, {"zh", "向后插入"}});
    add_locale_string("action.insert_after_picking",
                      {{"en", "insert after (...)"}, {"zh", "向后插入 (...)"}});
    add_locale_string("action.insert_before",
                      {{"en", "insert before"}, {"zh", "向前插入"}});
    add_locale_string(
        "action.insert_before_picking",
        {{"en", "insert before (...)"}, {"zh", "向前插入 (...)"}});
    add_locale_string("action.edit_plane",
                      {{"en", "Edit plane"}, {"zh", "编辑平面"}});
    add_locale_string("action.close", {{"en", "Close"}, {"zh", "关闭"}});
    add_locale_string("action.save_as_stl",
                      {{"en", "Save as STL"}, {"zh", "另存为 STL"}});
    add_locale_string("action.export_stl_all",
                      {{"en", "Export All"}, {"zh", "全部导出"}});
    add_locale_string("action.render_sdf",
                      {{"en", "Render SDF"}, {"zh", "渲染 SDF"}});
    add_locale_string("label.export_mode_standard",
                      {{"en", "Standard"}, {"zh", "标准"}});
    add_locale_string("label.export_mode_smooth",
                      {{"en", "Smooth SDF"}, {"zh", "SDF 平滑"}});
    add_locale_string("label.simplify_model",
                      {{"en", "Simplify model"}, {"zh", "简化模型"}});
    add_locale_string("label.simplification_ratio",
                      {{"en", "Simplification ratio"}, {"zh", "简化比例"}});
    add_locale_string("label.subdivisions_ratio",
                      {{"en", "Subdivisions ratio"}, {"zh", "细分比例"}});
    add_locale_string(
        "hint.simplification_ratio",
        {{"en", "Lower = more simplification (0.01=aggressive, 1.0=none)"},
         {"zh", "数值越小简化越多 (0.01=大量简化, 1.0=不简化)"}});
    add_locale_string(
        "label.use_cgal_skeleton",
        {{"en", "Use CGAL mesh skeleton"}, {"zh", "使用 CGAL 网格骨架"}});
    add_locale_string("action.reload", {{"en", "Reload"}, {"zh", "重新加载"}});
    add_locale_string("action.reload_stl",
                      {{"en", "Reload STL"}, {"zh", "重新加载 STL"}});
    add_locale_string("action.apply", {{"en", "Apply"}, {"zh", "应用"}});
    add_locale_string("action.pick_pos_auto_snapping",
                      {{"en", "auto snapping"}, {"zh", "自动吸附"}});
    add_locale_string("action.pick_pos_auto_snapping_stop",
                      {{"en", "stop auto snapping"}, {"zh", "取消自动吸附"}});

    add_locale_string("label.show_origin_mesh",
                      {{"en", "show origin mesh"}, {"zh", "显示原始网格"}});
    add_locale_string("label.show_mesh",
                      {{"en", "show mesh"}, {"zh", "显示网格"}});
    add_locale_string("label.show_exported_mesh",
                      {{"en", "show exported mesh"}, {"zh", "显示导出网格"}});
    add_locale_string("label.show_collision",
                      {{"en", "show collision"}, {"zh", "显示碰撞"}});
    add_locale_string("label.show_voxels",
                      {{"en", "show voxels"}, {"zh", "显示体素"}});
    add_locale_string("label.show_mesh_axis",
                      {{"en", "show mesh axis"}, {"zh", "显示网格坐标轴"}});
    add_locale_string("label.show_voxel_axis",
                      {{"en", "show voxel axis"}, {"zh", "显示体素坐标轴"}});
    add_locale_string(
        "label.show_collision_axis",
        {{"en", "show collision axis"}, {"zh", "显示碰撞坐标轴"}});
    add_locale_string(
        "label.show_collision_bounds",
        {{"en", "show collision bounds"}, {"zh", "显示碰撞边界"}});
    add_locale_string(
        "label.show_voxel_chunk_bounds",
        {{"en", "show voxel chunk bounds"}, {"zh", "显示体素块边界"}});
    add_locale_string(
        "label.disable_camera_on_pick",
        {{"en", "disable camera on pick"}, {"zh", "拾取时禁用相机旋转"}});
    add_locale_string("label.voxel_picking",
                      {{"en", "Voxel Picking"}, {"zh", "体素选择"}});
    add_locale_string("label.init_surface_cache",
                      {{"en", "Init Surface Cache"}, {"zh", "初始化表面缓存"}});
    add_locale_string(
        "label.surface_cache_ready",
        {{"en", "Surface cache ready"}, {"zh", "表面缓存已就绪"}});
    add_locale_string("label.pick_range",
                      {{"en", "Pick Range"}, {"zh", "选择范围"}});
    add_locale_string("label.neighbor_max_distance",
                      {{"en", "Max Distance"}, {"zh", "最大距离"}});
    add_locale_string("label.sdf_split_target",
                      {{"en", "Source Node"}, {"zh", "源节点"}});
    add_locale_string("action.save_marked_voxels",
                      {{"en", "Save Marked"}, {"zh", "保存标记"}});
    add_locale_string("action.load_marked_voxels",
                      {{"en", "Load Marked"}, {"zh", "加载标记"}});
    add_locale_string("action.undo_marked", {{"en", "Undo"}, {"zh", "撤销"}});
    add_locale_string("action.redo_marked", {{"en", "Redo"}, {"zh", "重做"}});
    add_locale_string(
        "label.history_marked_title",
        {{"en", "Marked Voxels History"}, {"zh", "标记体素历史"}});
    add_locale_string(
        "label.mouse_highlight_range",
        {{"en", "mouse highlight range"}, {"zh", "鼠标高亮范围"}});
    add_locale_string("window.history",
                      {{"en", "History"}, {"zh", "历史记录"}});
    add_locale_string("window.log", {{"en", "Log"}, {"zh", "日志"}});
    add_locale_string("window.debug_voxel_picking",
                      {{"en", "Voxel Picking Debug"}, {"zh", "体素拾取调试"}});
    add_locale_string("label.debug_step_world_to_voxel",
                      {{"en", "World to Voxel"}, {"zh", "世界坐标转体素"}});
    add_locale_string("label.debug_step_iterate_surface",
                      {{"en", "Iterate Surface"}, {"zh", "遍历表面体素"}});
    add_locale_string("label.debug_step_mark_voxels",
                      {{"en", "Mark Voxels"}, {"zh", "标记体素"}});
    add_locale_string("label.debug_step_total",
                      {{"en", "Total"}, {"zh", "总计"}});
    add_locale_string("label.debug_voxel_pick_timings",
                      {{"en", "Timings (ms)"}, {"zh", "耗时统计 (ms)"}});

    // Joint editor
    add_locale_string("label.joint", {{"en", "Joint"}, {"zh", "关节"}});
    add_locale_string(
        "label.picked_skeleton_points",
        {{"en", "Picked skeleton points: %d"}, {"zh", "已选骨架点：%d"}});
    add_locale_string("label.custom_direction",
                      {{"en", "Custom Direction"}, {"zh", "自定义方向"}});
    add_locale_string("label.direction_end",
                      {{"en", "Direction End"}, {"zh", "方向终点"}});
    add_locale_string("action.pick_direction",
                      {{"en", "Pick Direction"}, {"zh", "拾取方向"}});
    add_locale_string(
        "action.stop_picking_direction",
        {{"en", "Stop Picking Direction"}, {"zh", "停止拾取方向"}});
    add_locale_string("label.socket_cone",
                      {{"en", "Socket Cone"}, {"zh", "关节窝圆锥"}});
    add_locale_string("label.head_cone",
                      {{"en", "Head Cone"}, {"zh", "关节头圆锥"}});
    add_locale_string("label.support_cones",
                      {{"en", "Support Cones"}, {"zh", "实体圆锥"}});
    add_locale_string("label.cylinder", {{"en", "Cylinder"}, {"zh", "连接柱"}});
    add_locale_string("label.offset", {{"en", "Offset"}, {"zh", "偏移"}});
    add_locale_string("label.angle", {{"en", "Angle"}, {"zh", "角度"}});
    add_locale_string("label.radius", {{"en", "Radius"}, {"zh", "半径"}});
    add_locale_string(
        "label.socket_support_offset",
        {{"en", "Socket Support Offset"}, {"zh", "关节窝实体偏移"}});
    add_locale_string(
        "label.socket_support_radius",
        {{"en", "Socket Support Radius"}, {"zh", "关节窝实体半径"}});
    add_locale_string(
        "label.head_support_offset",
        {{"en", "Head Support Offset"}, {"zh", "关节头实体偏移"}});
    add_locale_string(
        "label.head_support_radius",
        {{"en", "Head Support Radius"}, {"zh", "关节头实体半径"}});
    add_locale_string("label.cylinder_offset",
                      {{"en", "Cylinder Offset"}, {"zh", "连接柱偏移"}});
    add_locale_string("label.cylinder_radius",
                      {{"en", "Cylinder Radius"}, {"zh", "连接柱半径"}});
    add_locale_string("label.female_gap",
                      {{"en", "Female Gap"}, {"zh", "母连接柱间隙"}});
    add_locale_string("label.slot_extra",
                      {{"en", "Slot Extra"}, {"zh", "槽间隙"}});
    add_locale_string(
        "label.socket_fillet_radius",
        {{"en", "Socket Fillet Radius"}, {"zh", "关节窝圆角半径"}});
    add_locale_string(
        "label.socket_fillet_height",
        {{"en", "Socket Fillet Height"}, {"zh", "关节窝圆角高度"}});
    add_locale_string(
        "label.socket_fillet_offset",
        {{"en", "Socket Fillet Offset"}, {"zh", "关节窝圆角偏移"}});
    add_locale_string(
        "label.head_fillet_height",
        {{"en", "Head Fillet Height"}, {"zh", "关节头圆角高度"}});
    add_locale_string("label.rotation_angle",
                      {{"en", "Rotation Angle"}, {"zh", "旋转角度"}});
    add_locale_string("window.reload_stl",
                      {{"en", "Reload STL"}, {"zh", "重新加载 STL"}});
    add_locale_string("label.reload_stl_hint",
                      {{"en", "Adjust voxel size and reload from source"},
                       {"zh", "调整体素大小并从源文件重新加载"}});
    add_locale_string("label.history_total",
                      {{"en", "Total states"}, {"zh", "总状态数"}});
    add_locale_string("label.history_undo",
                      {{"en", "Undo History"}, {"zh", "可撤销历史"}});
    add_locale_string("label.history_redo",
                      {{"en", "Redo History"}, {"zh", "可重做历史"}});
    add_locale_string("label.history_current",
                      {{"en", "[Current State]"}, {"zh", "[当前状态]"}});
    add_locale_string("label.history_empty", {{"en", "No history records."},
                                              {"zh", "暂无历史记录。"}});
    add_locale_string("label.items_tasks",
                      {{"en", "items:%d tasks:%d"}, {"zh", "项目:%d 任务:%d"}});
    add_locale_string("label.mouse_world_pos",
                      {{"en", "mouse world pos:(%.2f,%.2f,%.2f)"},
                       {"zh", "鼠标世界坐标:(%.2f,%.2f,%.2f)"}});
    add_locale_string("label.current_memory_status",
                      {{"en", "Memory: Current:%.2fMB Peak:%.2fMB"},
                       {"zh", "内存: 当前:%.2fMB 峰值:%.2fMB"}});
    add_locale_string("label.current_fps",
                      {{"en", "FPS: %.2f"}, {"zh", "FPS: %.2f"}});
    add_locale_string("label.load_stl_hint",
                      {{"en", "Click the button below to load an STL file."},
                       {"zh", "点击下方按钮加载 STL 文件。"}});
    add_locale_string("label.selected_file",
                      {{"en", "Selected file: %s"}, {"zh", "已选文件: %s"}});
    add_locale_string("label.no_file_selected",
                      {{"en", "No file selected."}, {"zh", "未选择文件。"}});
    add_locale_string("label.voxel_size",
                      {{"en", "Voxel Size"}, {"zh", "体素大小"}});
    add_locale_string("label.collision_root",
                      {{"en", "collision root"}, {"zh", "碰撞根节点"}});
    add_locale_string("label.collision_group",
                      {{"en", "collision group"}, {"zh", "碰撞组"}});
    add_locale_string("label.new_shape",
                      {{"en", "new shape"}, {"zh", "新形状"}});
    add_locale_string("label.no_collision_shapes",
                      {{"en", "No collision shapes."}, {"zh", "无碰撞形状。"}});
    add_locale_string("label.concave_cone_apex",
                      {{"en", "concave cone apex"}, {"zh", "凹锥顶点"}});
    add_locale_string(
        "label.concave_cone_vertices",
        {{"en", "concave cone vertices"}, {"zh", "凹锥底面顶点"}});
    add_locale_string("label.apex", {{"en", "apex"}, {"zh", "顶点"}});
    add_locale_string("label.vertex",
                      {{"en", "vertex %d##%d"}, {"zh", "顶点 %d##%d"}});
    add_locale_string("label.edit", {{"en", "edit"}, {"zh", "编辑"}});
    add_locale_string("label.segment_plane",
                      {{"en", "segment plane"}, {"zh", "分割平面"}});
    add_locale_string("label.input_mode",
                      {{"en", "input mode"}, {"zh", "输入模式"}});
    add_locale_string("label.pick_p1_by_mouse",
                      {{"en", "Pick P1 by mouse"}, {"zh", "用鼠标拾取 P1"}});
    add_locale_string("label.pick_p2_by_mouse",
                      {{"en", "Pick P2 by mouse"}, {"zh", "用鼠标拾取 P2"}});
    add_locale_string("label.pick_p3_by_mouse",
                      {{"en", "Pick P3 by mouse"}, {"zh", "用鼠标拾取 P3"}});
    add_locale_string("label.pick_point_by_mouse",
                      {{"en", "Pick point by mouse"}, {"zh", "用鼠标拾取点"}});
    add_locale_string(
        "label.pick_normal_by_mouse",
        {{"en", "Pick normal by mouse"}, {"zh", "用鼠标拾取法线"}});
    add_locale_string("label.plane_editor_bound_other",
                      {{"en", "Plane editor is bound to another item."},
                       {"zh", "平面编辑器已绑定到其他项目。"}});
    add_locale_string("label.no_active_item",
                      {{"en", "No active item."}, {"zh", "无活动项目。"}});
    add_locale_string("label.render_item",
                      {{"en", "render item: %d"}, {"zh", "渲染项目: %d"}});
    add_locale_string("label.segment_mode",
                      {{"en", "process mode"}, {"zh", "处理模式"}});
    add_locale_string("label.auto_segment_update",
                      {{"en", "auto segment update"}, {"zh", "自动分割更新"}});
    add_locale_string("label.updating",
                      {{"en", "Updating..."}, {"zh", "更新中..."}});
    add_locale_string("label.node", {{"en", "Node %d"}, {"zh", "节点 %d"}});
    add_locale_string("label.node_updating", {{"en", "Node %d (updating...)"},
                                              {"zh", "节点 %d (更新中...)"}});
    add_locale_string("label.position", {{"en", "Position"}, {"zh", "位置"}});
    add_locale_string("label.rotation_deg",
                      {{"en", "Rotation (deg)"}, {"zh", "旋转 (度)"}});
    add_locale_string("label.center", {{"en", "Center"}, {"zh", "中心"}});
    add_locale_string("label.radius", {{"en", "Radius"}, {"zh", "半径"}});
    add_locale_string("label.start", {{"en", "Start"}, {"zh", "起点"}});
    add_locale_string("label.end", {{"en", "End"}, {"zh", "终点"}});
    add_locale_string("label.half_extent",
                      {{"en", "Half Extent"}, {"zh", "半尺寸"}});
    add_locale_string("label.point", {{"en", "Point"}, {"zh", "点"}});
    add_locale_string("label.normal", {{"en", "Normal"}, {"zh", "法线"}});

    add_locale_string("shape.sphere", {{"en", "Sphere"}, {"zh", "球体"}});
    add_locale_string("shape.cylinder", {{"en", "Cylinder"}, {"zh", "圆柱体"}});
    add_locale_string("shape.capsule", {{"en", "Capsule"}, {"zh", "胶囊体"}});
    add_locale_string("shape.box", {{"en", "Box"}, {"zh", "长方体"}});
    add_locale_string("shape.unknown", {{"en", "Unknown"}, {"zh", "未知"}});

    add_locale_string("mode.three_point",
                      {{"en", "Three-Point"}, {"zh", "三点"}});
    add_locale_string("mode.point_normal",
                      {{"en", "Point-Normal"}, {"zh", "点-法线"}});
    add_locale_string("mode.collision", {{"en", "Collision"}, {"zh", "碰撞"}});
    add_locale_string("mode.plane", {{"en", "Plane"}, {"zh", "平面"}});
    add_locale_string("mode.concave_cone",
                      {{"en", "Concave Cone"}, {"zh", "凹锥"}});
    add_locale_string("mode.split_disconnected",
                      {{"en", "Split Disconnected"}, {"zh", "分离不连通区域"}});
    add_locale_string("mode.neighbor",
                      {{"en", "Neighbor"}, {"zh", "临近分割"}});
    add_locale_string("mode.fill_interior",
                      {{"en", "Fill Interior"}, {"zh", "填充内部"}});
    add_locale_string("mode.chain", {{"en", "Chain"}, {"zh", "链条化"}});
    add_locale_string("mode.sdf_node_split",
                      {{"en", "SDF Node Split"}, {"zh", "SDF节点分割"}});

    add_locale_string(
        "tooltip.mode.collision",
        {{"en",
          "Use collision shapes to split the model. Voxels inside the shapes "
          "are separated from those outside."},
         {"zh",
          "使用碰撞体形状分割模型。形状内部的体素与外部的体素被分离开。"}});
    add_locale_string(
        "tooltip.mode.plane",
        {{"en", "Use an infinite plane to split the model into two halves."},
         {"zh", "使用无限平面将模型分割成两半。"}});
    add_locale_string(
        "tooltip.mode.concave_cone",
        {{"en", "Use a concave cone to carve out a region from the model."},
         {"zh", "使用凹锥从模型中挖出一个区域。"}});
    add_locale_string("tooltip.mode.split_disconnected",
                      {{"en",
                        "Automatically split disconnected voxel regions into "
                        "separate items."},
                       {"zh",
                        "自动将不连通的体素区域分割成独立的项目。\n注意：此操作"
                        "将丢失SDF数据"}});
    add_locale_string("tooltip.mode.neighbor",
                      {{"en",
                        "Split voxels based on BFS distance from marked seed "
                        "voxels.\nNote: This will lose SDF data."},
                       {"zh",
                        "基于与标记种子体素的 BFS "
                        "距离来分割体素。\n注意：此操作将丢失SDF数据"}});
    add_locale_string("tooltip.mode.fill_interior",
                      {{"en",
                        "Fill all enclosed hollow cavities inside the model "
                        "with solid voxels. Produces a single child node."},
                       {"zh",
                        "用实体体素填满模型内部所有封闭的中空腔体。产生一个子节"
                        "点。\n注意：此操作将丢失SDF数据"}});
    add_locale_string(
        "tooltip.load_as_sdf",
        {{"en",
          "Load STL file as signed distance field (SDF) for significantly "
          "improved model accuracy, but SDF data will be lost when using "
          "certain features (e.g. neighbor split)."},
         {"zh",
          "将 STL 文件作为有符号距离场 (SDF) "
          "加载，这将显著提升模型精度，但是输出模型将消耗更长的时间。使用部分功"
          "能时会丢失 SDF 数据（例如邻近分割）。"}});
    add_locale_string(
        "tooltip.use_precise_voxelization",
        {{"en",
          "Use CGAL side-of-mesh test to verify ambiguous voxels and reduce "
          "voxelization errors. Disable for faster loading on large models."},
         {"zh",
          "使用 CGAL 体素内判定来验证可疑体素，降低体素化错误率。对大模型关闭可"
          "以加快加载速度。"}});
    add_locale_string(
        "tooltip.stl_load_mode",
        {{"en", "Choose how the STL file is processed before voxelization."},
         {"zh", "选择 STL 文件在体素化之前的处理方式。"}});
    add_locale_string(
        "tooltip.stl_load_mode.default",
        {{"en",
          "Load model and perform voxelization (suitable for closed models)"},
         {"zh", "使用加载模型并执行体素化（适合封闭模型）"}});
    add_locale_string(
        "tooltip.stl_load_mode.sdf",
        {{"en", "Load as signed distance field for improved accuracy."},
         {"zh", "作为有符号距离场加载，以获得更高精度。"}});
    add_locale_string(
        "tooltip.stl_load_mode.silhouette",
        {{"en",
          "Silhouette mode computes a cone geometry for each triangle relative "
          "to a chosen center point, to achieve a closed model effect. "
          "Suitable for single-layer thin models (e.g., hair from official "
          "game models)."},
         {"zh",
          "选择一个中心点，计算每个三角形相对于中心点的锥体的几何，以达到封闭模"
          "型的效果。适用于单层薄片模型（例如用于来源于游戏官模的头发）。"}});
    add_locale_string(
        "tooltip.stl_load_mode.surface_only",
        {{"en",
          "Only load surface voxels, leaving interior empty. Suitable for thin "
          "models or when you only care about the surface."},
         {"zh",
          "只加载表面体素，内部留空。适用于薄模型或仅关心表面的情况，也可结合“"
          "内部填充”功能使用。"}});
    add_locale_string(
        "tooltip.stl_load_mode.mesh_only",
        {{"en",
          "Load only the mesh geometry without voxelization. Suitable for "
          "direct mesh editing and plane-based mesh splitting."},
         {"zh",
          "仅加载网格几何体，不进行体素化。适用于直接编辑网格和基于平面的网格切"
          "分。"}});
    add_locale_string(
        "tooltip.stl_load_mode.convex_hull",
        {{"en",
          "Compute the convex hull of the source mesh and voxelize it. "
          "Produces a closed, simplified convex representation."},
         {"zh",
          "计算源网格的凸包并进行体素化。生成一个封闭的简化凸体表示。"}});
    add_locale_string(
        "tooltip.render_sdf",
        {{"en",
          "Render SDF as a 3D model. This will clear all marked voxels. "
          "Proceed?"},
         {"zh", "将SDF渲染为网格以供预览。注意：这可能需要较长时间。"}});
    add_locale_string(
        "tooltip.simplify_model",
        {{"en",
          "Simplify the model by merging redundant triangles.\nLower = more "
          "simplification (0.01=aggressive, 1.0=none).\nNote: This may take a "
          "long time."},
         {"zh",
          "合并多余的三角形以简化模型。\n数值越小简化越多（0.01=大量简化, "
          "1.0=不简化）。\n注意：此操作可能会消耗较长时间。"}});
    add_locale_string(
        "tooltip.export_mode_smooth",
        {{"en",
          "Smooth the model on export (will attempt to use SDF data if "
          "available)."},
         {"zh", "导出时对物体进行平滑处理（将尝试使用SDF数据进行处理）"}});
    add_locale_string(
        "tooltip.mode.chain",
        {{"en",
          "Split the model into 3D-printable chains (e.g. for jointed "
          "dragon)."},
         {"zh", "将物体切分为3D打印一体成形的链条（例如关节龙）。"}});
    add_locale_string("tooltip.mode.sdf_node_split",
                      {{"en", "Split current node using another node's SDF."},
                       {"zh", "使用另一个节点的SDF分割当前节点。"}});
    add_locale_string("tooltip.update_collision",
                      {{"en",
                        "Update collision shapes. This will clear all marked "
                        "voxels. Proceed?"},
                       {"zh", "在当前节点上执行设置好的碰撞编辑器。"}});
    add_locale_string(
        "tooltip.pick_pos_auto_snapping",
        {{"en", "Pick position with auto-snapping to nearest voxel vertex"},
         {"zh", "鼠标会尝试吸附到附近的顶点上"}});
    add_locale_string(
        "tooltip.collision_edit",
        {{"en", "Edit collision shapes"}, {"zh", "编辑当前项目的分割规则"}});
    add_locale_string("tooltip.voxel_picking",
                      {{"en", "Pick voxels"}, {"zh", "从模型中选择体素"}});
    add_locale_string("label.chain_min_radius",
                      {{"en", "min radius"}, {"zh", "最小半径"}});
    add_locale_string("tooltip.sdf_resolution",
                      {{"en", "SDF:\n%s"}, {"zh", "此节点有SDF\n%s"}});
    add_locale_string(
        "tooltip.triangle_count",
        {{"en", "Triangles: %zu"}, {"zh", "此节点有STL模型\n三角形: %zu 个"}});

    add_locale_string("dialog.open_stl_title",
                      {{"en", "Open STL"}, {"zh", "打开 STL"}});
    add_locale_string("dialog.stl_file",
                      {{"en", "STL file"}, {"zh", "STL 文件"}});
    add_locale_string("dialog.save_voxel_as_stl",
                      {{"en", "Save Voxel as STL"}, {"zh", "保存体素为 STL"}});
    add_locale_string(
        "dialog.choose_export_method",
        {{"en", "Choose STL export method"}, {"zh", "选择 STL 导出方式"}});
    add_locale_string("dialog.sdf_preview",
                      {{"en", "SDF Preview"}, {"zh", "SDF 预览"}});
    add_locale_string("dialog.sdf_preview_desc",
                      {{"en", "Generate mesh from SDF without saving to file"},
                       {"zh", "从 SDF 生成网格，不保存到文件"}});
    add_locale_string("dialog.stl_files",
                      {{"en", "STL files"}, {"zh", "STL 文件"}});
    add_locale_string("dialog.export_stl_all",
                      {{"en", "Export All STL"}, {"zh", "全部导出 STL"}});
    add_locale_string("dialog.save_project_title",
                      {{"en", "Select Folder to Save Project"},
                       {"zh", "选择文件夹保存项目"}});
    add_locale_string(
        "dialog.load_project_title",
        {{"en", "Select Project Folder"}, {"zh", "选择项目文件夹"}});
    add_locale_string("dialog.info", {{"en", "Info"}, {"zh", "提示"}});
    add_locale_string("dialog.confirm_delete_title",
                      {{"en", "Confirm Delete"}, {"zh", "确认删除"}});
    add_locale_string("dialog.confirm_delete",
                      {{"en", "Are you sure you want to delete this node?"},
                       {"zh", "确定要删除此节点吗？"}});
    add_locale_string("dialog.confirm_manual_update_title",
                      {{"en", "Confirm Update"}, {"zh", "确认更新"}});
    add_locale_string(
        "dialog.confirm_manual_update_message",
        {{"en", "Some child nodes have auto-update disabled. Proceed anyway?"},
         {"zh", "部分子节点已关闭自动更新，是否继续？"}});
    add_locale_string("dialog.confirm_update_clears_mark",
                      {{"en", "Update will clear marked voxels. Continue?"},
                       {"zh", "更新将清除标记的体素，是否继续？"}});
    add_locale_string("dialog.unsaved_changes_title",
                      {{"en", "Unsaved changes"}, {"zh", "未保存的更改"}});
    add_locale_string(
        "dialog.unsaved_changes_message",
        {{"en", "There are unsaved changes. Save before closing?"},
         {"zh", "存在未保存的更改。关闭前是否保存？"}});
    add_locale_string("dialog.save_marked_voxels",
                      {{"en", "Save Marked Voxels"}, {"zh", "保存标记体素"}});
    add_locale_string("dialog.load_marked_voxels",
                      {{"en", "Load Marked Voxels"}, {"zh", "加载标记体素"}});
    add_locale_string("dialog.marked_voxels_file",
                      {{"en", "Marked Voxel File"}, {"zh", "标记体素文件"}});
    add_locale_string("error.save_marked_failed",
                      {{"en", "Failed to save marked voxels."},
                       {"zh", "保存标记体素失败。"}});
    add_locale_string("error.load_marked_failed",
                      {{"en", "Failed to load marked voxels."},
                       {"zh", "加载标记体素失败。"}});

    add_locale_string(
        "error.three_points_collinear",
        {{"en", "Three points are collinear."}, {"zh", "三个点共线。"}});
    add_locale_string("error.save_failed", {{"en", "Failed to save project."},
                                            {"zh", "保存项目失败。"}});
    add_locale_string("error.load_failed", {{"en", "Failed to load project."},
                                            {"zh", "加载项目失败。"}});

    add_locale_string("progress.extract_skeleton.buildDenseGrid",
                      {{"en", "build dense grid"}, {"zh", "构建稠密体素网格"}});
    add_locale_string("progress.extract_skeleton.computeEDT",
                      {{"en", "compute EDT"}, {"zh", "构建EDT"}});
    add_locale_string("progress.extract_skeleton.finalizeEDT",
                      {{"en", "finalize EDT"}, {"zh", "EDT后处理"}});
    add_locale_string("progress.extract_skeleton.extractCenterline",
                      {{"en", "extract center line"}, {"zh", "计算中心线"}});
    add_locale_string(
        "progress.surface_nets.building_dense_grid",
        {{"en", "building dense grid"}, {"zh", "构建稠密体素网格"}});
    add_locale_string("progress.surface_nets.processing_chunk",
                      {{"en", "processing chunk"}, {"zh", "处理块"}});
    add_locale_string("progress.extract_skeleton.extractCenterline",
                      {{"en", "extract center line"}, {"zh", "计算中心线"}});
    add_locale_string("progress.surface_nets.extracting_vertices",
                      {{"en", "extracting vertices"}, {"zh", "提取顶点"}});
    add_locale_string("progress.surface_nets.no_surface_found",
                      {{"en", "no surface found"}, {"zh", "未找到表面"}});
    add_locale_string("progress.surface_nets.building_faces",
                      {{"en", "building faces"}, {"zh", "构建面"}});
    add_locale_string("progress.surface_nets.emitting_triangles",
                      {{"en", "emitting triangles"}, {"zh", "发射三角形"}});
    add_locale_string("progress.surface_nets.done",
                      {{"en", "done"}, {"zh", "完成"}});
    add_locale_string("progress.surface_nets.building_voxel_sdf",
                      {{"en", "building voxel SDF"}, {"zh", "构建体素SDF"}});
    add_locale_string("progress.surface_nets.sdf_sample",
                      {{"en", "SDF sample"}, {"zh", "采样SDF"}});
    add_locale_string("progress.mc.processing_chunk",
                      {{"en", "processing chunk"}, {"zh", "处理块"}});

    // Node source labels
    add_locale_string("label.source_file",
                      {{"en", "File"}, {"zh", "文件"}});
    add_locale_string("label.source_node",
                      {{"en", "Node"}, {"zh", "节点"}});
    add_locale_string("label.source_node_id",
                      {{"en", "Source Node"}, {"zh", "源节点"}});
    add_locale_string("label.source_data_mesh",
                      {{"en", "Mesh"}, {"zh", "网格"}});
    add_locale_string("label.source_data_sdf",
                      {{"en", "SDF"}, {"zh", "SDF"}});
    add_locale_string("label.source_data_voxel",
                      {{"en", "Voxel"}, {"zh", "体素"}});
    add_locale_string("label.node_source_sdf_subdivisions",
                      {{"en", "SDF Subdivisions"}, {"zh", "SDF 细分"}});
    add_locale_string("label.node_source_sdf_simplify",
                      {{"en", "Simplify"}, {"zh", "简化"}});
    add_locale_string("label.node_source_sdf_simplify_ratio",
                      {{"en", "Simplify Ratio"}, {"zh", "简化比例"}});
    add_locale_string("action.reload_from_node",
                      {{"en", "Reload from Node"}, {"zh", "从节点重新加载"}});
}

const std::string& get_locale_string_ref(const std::string& key) {
    if (locale_strings.empty()) {
        init_locale_strings();
    }

    const auto entry_it = locale_strings.find(key);
    if (entry_it == locale_strings.end()) {
        static thread_local std::string fallback_key;
        fallback_key = key;
        return fallback_key;
    }

    const TranslationMap& translations = entry_it->second;
    const auto language_it = translations.find(current_language);
    if (language_it != translations.end()) {
        return language_it->second;
    }

    const auto fallback_it = translations.find(kDefaultLanguage);
    if (fallback_it != translations.end()) {
        return fallback_it->second;
    }

    static const std::string empty;
    return empty;
}

}  // namespace

std::vector<std::string> get_default_font_path() {
    std::string font_paths[] = {
#ifdef _WIN32
        "C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
        "C:/Windows/Fonts/simsun.ttc", "C:/Windows/Fonts/msyh.ttf"
#else
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/opentype/noto/NotoSans-Regular.ttf"
#endif
    };
    std::vector<std::string> result;
    for (auto& path : font_paths) {
        if (std::filesystem::exists(path)) {
            result.push_back(path);
        }
    }
    return result;
}
std::string get_system_language() {
#ifdef _WIN32
    WCHAR localeName[LOCALE_NAME_MAX_LENGTH];

    if (GetUserDefaultLocaleName(localeName, LOCALE_NAME_MAX_LENGTH)) {
        int size = WideCharToMultiByte(CP_UTF8, 0, localeName, -1, nullptr, 0,
                                       nullptr, nullptr);

        std::string result(size, 0);

        WideCharToMultiByte(CP_UTF8, 0, localeName, -1, result.data(), size,
                            nullptr, nullptr);

        return result;
    }
    return "en-US";

#else
    const char* lang = getenv("LC_ALL")        ? getenv("LC_ALL")
                       : getenv("LC_MESSAGES") ? getenv("LC_MESSAGES")
                                               : getenv("LANG");

    return lang ? std::string(lang) : "en_US";
#endif
}

void locale_init() {
    init_locale_strings();
    auto lang = get_system_language();
    current_language = resolve_language_code(lang);
    std::cout << "System language: " << lang
              << ", UI language: " << current_language << std::endl;
}

std::string get_locale_string(const std::string& key) {
    return get_locale_string_ref(key);
}

const char* get_locale_cstr(const std::string& key) {
    return get_locale_string_ref(key).c_str();
}

std::string utf8_to_ansi(const char* str) {
#ifdef _WIN32
    if (!str)
        return {};

    int wlen = MultiByteToWideChar(CP_UTF8, 0, str, -1, nullptr, 0);
    std::wstring w(wlen, 0);
    MultiByteToWideChar(CP_UTF8, 0, str, -1, &w[0], wlen);

    int alen = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, nullptr, 0,
                                   nullptr, nullptr);
    std::string s(alen, 0);
    WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, &s[0], alen, nullptr,
                        nullptr);
    return s;
#else
    return str ? std::string(str) : std::string();
#endif
}
}  // namespace sinriv::locale
