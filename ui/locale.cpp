#include "locale.h"

#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <unordered_map>

namespace sinriv::ui::render {

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
    std::initializer_list<std::pair<const char*, const char*>>
        translations) {
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
    add_locale_string("window.collision_editor",
                      {{"en", "collision editor"}, {"zh", "碰撞编辑器"}});
    add_locale_string("window.edit_segment_plane",
                      {{"en", "Edit Segment Plane"}, {"zh", "编辑分割平面"}});
    add_locale_string("window.nav_node_map",
                      {{"en", "nav node map"}, {"zh", "导航节点图"}});

    add_locale_string("menu.file", {{"en", "File"}, {"zh", "文件"}});
    add_locale_string("menu.view", {{"en", "View"}, {"zh", "视图"}});
    add_locale_string("menu.body", {{"en", "Body"}, {"zh", "主体"}});
    add_locale_string("menu.axis", {{"en", "Axis"}, {"zh", "坐标轴"}});
    add_locale_string("menu.bound", {{"en", "Bound"}, {"zh", "边界"}});
    add_locale_string("menu.open_stl",
                      {{"en", "Open STL (O)"}, {"zh", "打开 STL (O)"}});

    add_locale_string("action.update_collision",
                      {{"en", "update collision"}, {"zh", "更新碰撞"}});
    add_locale_string("action.open_file_dialog",
                      {{"en", "Open File Dialog"}, {"zh", "打开文件对话框"}});
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
    add_locale_string("action.replace", {{"en", "replace"}, {"zh", "替换"}});
    add_locale_string("action.replace_picking",
                      {{"en", "replace (...)"}, {"zh", "替换 (...)"}});
    add_locale_string("action.insert_after",
                      {{"en", "insert after"}, {"zh", "向后插入"}});
    add_locale_string("action.insert_after_picking",
                      {{"en", "insert after (...)"},
                       {"zh", "向后插入 (...)"}});
    add_locale_string("action.insert_before",
                      {{"en", "insert before"}, {"zh", "向前插入"}});
    add_locale_string("action.insert_before_picking",
                      {{"en", "insert before (...)"},
                       {"zh", "向前插入 (...)"}});
    add_locale_string("action.edit_plane",
                      {{"en", "Edit plane"}, {"zh", "编辑平面"}});
    add_locale_string("action.close", {{"en", "Close"}, {"zh", "关闭"}});
    add_locale_string("action.save_as_stl",
                      {{"en", "Save as STL"}, {"zh", "另存为 STL"}});

    add_locale_string("label.show_mesh",
                      {{"en", "show mesh"}, {"zh", "显示网格"}});
    add_locale_string("label.show_collision",
                      {{"en", "show collision"}, {"zh", "显示碰撞"}});
    add_locale_string("label.show_voxels",
                      {{"en", "show voxels"}, {"zh", "显示体素"}});
    add_locale_string("label.show_mesh_axis",
                      {{"en", "show mesh axis"}, {"zh", "显示网格坐标轴"}});
    add_locale_string("label.show_voxel_axis",
                      {{"en", "show voxel axis"}, {"zh", "显示体素坐标轴"}});
    add_locale_string("label.show_collision_axis",
                      {{"en", "show collision axis"}, {"zh", "显示碰撞坐标轴"}});
    add_locale_string("label.show_collision_bounds",
                      {{"en", "show collision bounds"}, {"zh", "显示碰撞边界"}});
    add_locale_string("label.items_tasks",
                      {{"en", "items:%d tasks:%d"}, {"zh", "项目:%d 任务:%d"}});
    add_locale_string("label.mouse_world_pos",
                      {{"en", "mouse world pos:(%.2f,%.2f,%.2f)"},
                       {"zh", "鼠标世界坐标:(%.2f,%.2f,%.2f)"}});
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
    add_locale_string("label.concave_cone_vertices",
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
    add_locale_string("label.pick_normal_by_mouse",
                      {{"en", "Pick normal by mouse"}, {"zh", "用鼠标拾取法线"}});
    add_locale_string("label.plane_editor_bound_other",
                      {{"en", "Plane editor is bound to another item."},
                       {"zh", "平面编辑器已绑定到其他项目。"}});
    add_locale_string("label.no_active_item",
                      {{"en", "No active item."}, {"zh", "无活动项目。"}});
    add_locale_string("label.render_item",
                      {{"en", "render item: %d"}, {"zh", "渲染项目: %d"}});
    add_locale_string("label.segment_mode",
                      {{"en", "segment mode"}, {"zh", "分割模式"}});
    add_locale_string("label.node", {{"en", "Node %d"}, {"zh", "节点 %d"}});
    add_locale_string("label.node_updating",
                      {{"en", "Node %d (updating...)"},
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
    add_locale_string("shape.cylinder",
                      {{"en", "Cylinder"}, {"zh", "圆柱体"}});
    add_locale_string("shape.capsule", {{"en", "Capsule"}, {"zh", "胶囊体"}});
    add_locale_string("shape.box", {{"en", "Box"}, {"zh", "盒体"}});
    add_locale_string("shape.unknown", {{"en", "Unknown"}, {"zh", "未知"}});

    add_locale_string("mode.three_point",
                      {{"en", "Three-Point"}, {"zh", "三点"}});
    add_locale_string("mode.point_normal",
                      {{"en", "Point-Normal"}, {"zh", "点-法线"}});
    add_locale_string("mode.collision", {{"en", "Collision"}, {"zh", "碰撞"}});
    add_locale_string("mode.plane", {{"en", "Plane"}, {"zh", "平面"}});
    add_locale_string("mode.concave_cone",
                      {{"en", "Concave Cone"}, {"zh", "凹锥"}});

    add_locale_string("dialog.open_stl_title",
                      {{"en", "Open STL"}, {"zh", "打开 STL"}});
    add_locale_string("dialog.stl_file",
                      {{"en", "STL file"}, {"zh", "STL 文件"}});
    add_locale_string("dialog.save_voxel_as_stl",
                      {{"en", "Save Voxel as STL"}, {"zh", "保存体素为 STL"}});
    add_locale_string("dialog.stl_files",
                      {{"en", "STL files"}, {"zh", "STL 文件"}});

    add_locale_string("error.three_points_collinear",
                      {{"en", "Three points are collinear."},
                       {"zh", "三个点共线。"}});
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
}  // namespace sinriv::ui::render
