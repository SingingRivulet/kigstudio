#include <cJSON.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <imgui/imgui.h>
#include <tinyfiledialogs.h>

#include "render_voxel_list.h"
#include "utils.h"

namespace sinriv::ui::render {

namespace {

std::string extract_mmd_read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

bool extract_mmd_write_file(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    return f.good();
}

const char* extract_mmd_mode_label(int mode) {
    return mode == 0 ? get_locale_cstr("label.extract_mmd_mode_bones")
                     : get_locale_cstr("label.extract_mmd_mode_materials");
}

}  // anonymous namespace

void RenderVoxelList::render_extract_mmd_dialog() {
    if (!ImGui::BeginPopupModal(get_locale_cstr("dialog.extract_mmd"),
                                nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize)) {
        return;
    }

    // Poll any running subprocess every frame while the dialog is open.
    extract_mmd_poll();

    ImGui::TextUnformatted(get_locale_cstr("label.extract_mmd_pmx"));

    // File path + browse button on the same line.
    char path_buf[1024];
    std::strncpy(path_buf, extract_mmd_pmx_path.c_str(), sizeof(path_buf) - 1);
    path_buf[sizeof(path_buf) - 1] = '\0';
    ImGui::SetNextItemWidth(400.0f);
    if (ImGui::InputText("##pmx_path", path_buf, sizeof(path_buf))) {
        extract_mmd_pmx_path = path_buf;
    }
    ImGui::SameLine();
    if (ImGui::Button(get_locale_cstr("action.browse"))) {
        const char* filters[] = {"*.pmx"};
        const char* file = tinyfd_openFileDialog(
            utf8_to_ansi(get_locale_cstr("dialog.open_pmx_title")).c_str(),
            "", 1, filters,
            utf8_to_ansi(get_locale_cstr("dialog.pmx_files")).c_str(), 0);
        if (file) {
            extract_mmd_pmx_path = tinyfd_path_to_utf8(file);
        }
    }

    bool busy = extract_mmd_listing || extract_mmd_extracting;

    // Mode switch: bones / materials.
    ImGui::TextUnformatted(get_locale_cstr("label.extract_mmd_mode"));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150.0f);
    if (ImGui::BeginCombo("##extract_mmd_mode", extract_mmd_mode_label(extract_mmd_mode))) {
        for (int i = 0; i < 2; ++i) {
            bool selected = extract_mmd_mode == i;
            if (ImGui::Selectable(extract_mmd_mode_label(i), selected)) {
                if (extract_mmd_mode != i) {
                    extract_mmd_mode = i;
                    extract_mmd_items.clear();
                }
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    // Load item list button.
    ImGui::BeginDisabled(extract_mmd_pmx_path.empty() || busy);
    if (ImGui::Button(get_locale_cstr("action.load_items"))) {
        extract_mmd_items.clear();
        extract_mmd_status_msg = get_locale_string("status.loading_items");
        extract_mmd_start_list();
    }
    ImGui::EndDisabled();

    if (!extract_mmd_status_msg.empty()) {
        ImGui::TextUnformatted(extract_mmd_status_msg.c_str());
    }

    ImGui::Separator();

    // Threshold (only meaningful for bone mode) and options.
    ImGui::BeginDisabled(extract_mmd_mode == 1);
    ImGui::SliderFloat(get_locale_cstr("label.threshold"),
                       &extract_mmd_threshold, 0.0f, 1.0f, "%.2f");
    ImGui::EndDisabled();
    if (extract_mmd_mode == 1 && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s", get_locale_cstr("tooltip.threshold_bones_only"));
    }
    ImGui::Checkbox(get_locale_cstr("label.case_sensitive"),
                    &extract_mmd_case_sensitive);

    // Item checkboxes.
    if (!extract_mmd_items.empty()) {
        ImGui::Text("%s (%zu)",
                    extract_mmd_mode == 0
                        ? get_locale_cstr("label.bones")
                        : get_locale_cstr("label.materials"),
                    extract_mmd_items.size());
        ImGui::BeginChild("items_list", ImVec2(0, 250), true);
        for (auto& [name, checked] : extract_mmd_items) {
            ImGui::Checkbox(name.c_str(), &checked);
        }
        ImGui::EndChild();

        if (ImGui::Button(get_locale_cstr("action.select_all_items"))) {
            for (auto& [_, checked] : extract_mmd_items) checked = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(get_locale_cstr("action.deselect_all_items"))) {
            for (auto& [_, checked] : extract_mmd_items) checked = false;
        }
    }

    ImGui::Separator();

    // Extract button.
    bool has_selection = false;
    for (const auto& [_, checked] : extract_mmd_items) {
        if (checked) {
            has_selection = true;
            break;
        }
    }

    ImGui::BeginDisabled(!has_selection || busy);
    if (ImGui::Button(get_locale_cstr("action.extract"))) {
        extract_mmd_status_msg = get_locale_string("status.extracting_mmd");
        extract_mmd_start_extract();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button(get_locale_cstr("action.close"))) {
        if (extract_mmd_process.isRunning()) {
            extract_mmd_process.kill();
        }
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

void RenderVoxelList::extract_mmd_start_list() {
    if (extract_mmd_process.isRunning()) {
        extract_mmd_process.kill();
    }

    std::string exe = sinriv::kigstudio::Process::self_exe_path();
    std::string tmp_dir = path_to_utf8(std::filesystem::temp_directory_path());
    extract_mmd_json_in = tmp_dir + "/kgs_mmd_list_in.json";
    extract_mmd_json_out = extract_mmd_json_in + ".out.json";

    cJSON* in = cJSON_CreateObject();
    cJSON_AddStringToObject(in, "command", "list");
    cJSON_AddStringToObject(in, "pmx", extract_mmd_pmx_path.c_str());
    char* text = cJSON_Print(in);
    cJSON_Delete(in);
    if (!text) {
        extract_mmd_status_msg = get_locale_string("error.internal");
        return;
    }
    extract_mmd_write_file(extract_mmd_json_in, text);
    cJSON_free(text);

    std::vector<std::string> args = {"--tools", "--json", extract_mmd_json_in};

    if (!extract_mmd_process.start(exe, args)) {
        extract_mmd_status_msg = get_locale_string("error.start_subprocess");
        return;
    }
    extract_mmd_listing = true;
}

void RenderVoxelList::extract_mmd_start_extract() {
    if (extract_mmd_process.isRunning()) {
        extract_mmd_process.kill();
    }

    std::vector<std::string> keywords;
    for (const auto& [name, checked] : extract_mmd_items) {
        if (checked) keywords.push_back(name);
    }
    if (keywords.empty()) return;

    std::string exe = sinriv::kigstudio::Process::self_exe_path();
    std::string tmp_dir = path_to_utf8(std::filesystem::temp_directory_path());
    extract_mmd_json_in = tmp_dir + "/kgs_mmd_extract_in.json";
    extract_mmd_json_out = extract_mmd_json_in + ".out.json";
    extract_mmd_result_stl = tmp_dir + "/kgs_mmd_result.stl";

    const char* mode_str = extract_mmd_mode == 0 ? "bones" : "materials";

    cJSON* in = cJSON_CreateObject();
    cJSON_AddStringToObject(in, "command", "extract");
    cJSON_AddStringToObject(in, "pmx", extract_mmd_pmx_path.c_str());
    cJSON_AddStringToObject(in, "mode", mode_str);
    cJSON* kw_arr = cJSON_CreateArray();
    for (const auto& k : keywords) {
        cJSON_AddItemToArray(kw_arr, cJSON_CreateString(k.c_str()));
    }
    cJSON_AddItemToObject(in, "keywords", kw_arr);
    cJSON_AddNumberToObject(in, "threshold", extract_mmd_threshold);
    cJSON_AddBoolToObject(in, "case_sensitive", extract_mmd_case_sensitive);
    cJSON_AddStringToObject(in, "stl", extract_mmd_result_stl.c_str());

    char* text = cJSON_Print(in);
    cJSON_Delete(in);
    if (!text) {
        extract_mmd_status_msg = get_locale_string("error.internal");
        return;
    }
    extract_mmd_write_file(extract_mmd_json_in, text);
    cJSON_free(text);

    std::vector<std::string> args = {"--tools", "--json", extract_mmd_json_in};

    if (!extract_mmd_process.start(exe, args)) {
        extract_mmd_status_msg = get_locale_string("error.start_subprocess");
        return;
    }
    extract_mmd_extracting = true;
}

void RenderVoxelList::extract_mmd_poll() {
    if (!extract_mmd_process.isRunning()) {
        if (extract_mmd_listing) {
            extract_mmd_listing = false;
            std::string text = extract_mmd_read_file(extract_mmd_json_out);
            extract_mmd_process.close(false);
            if (text.empty()) {
                extract_mmd_status_msg = get_locale_string("error.subprocess_no_output");
            } else {
                extract_mmd_finish_list(text);
            }
        } else if (extract_mmd_extracting) {
            extract_mmd_extracting = false;
            std::string text = extract_mmd_read_file(extract_mmd_json_out);
            extract_mmd_process.close(false);
            if (text.empty()) {
                extract_mmd_status_msg = get_locale_string("error.subprocess_no_output");
            } else {
                extract_mmd_finish_extract(text);
            }
        }
        return;
    }

    // Update status while running.
    if (extract_mmd_listing) {
        extract_mmd_status_msg = get_locale_string("status.loading_items");
    } else if (extract_mmd_extracting) {
        extract_mmd_status_msg = get_locale_string("status.extracting_mmd");
    }
}

void RenderVoxelList::extract_mmd_finish_list(const std::string& json_text) {
    cJSON* root = cJSON_Parse(json_text.c_str());
    if (!root) {
        extract_mmd_status_msg = get_locale_string("error.parse_json");
        return;
    }

    cJSON* success = cJSON_GetObjectItem(root, "success");
    if (!cJSON_IsTrue(success)) {
        cJSON* err = cJSON_GetObjectItem(root, "error");
        extract_mmd_status_msg = err && cJSON_IsString(err)
                                     ? err->valuestring
                                     : get_locale_string("error.unknown");
        cJSON_Delete(root);
        return;
    }

    extract_mmd_items.clear();
    if (extract_mmd_mode == 0) {
        cJSON* bones = cJSON_GetObjectItem(root, "bones");
        if (bones && cJSON_IsArray(bones)) {
            int n = cJSON_GetArraySize(bones);
            for (int i = 0; i < n; ++i) {
                cJSON* item = cJSON_GetArrayItem(bones, i);
                cJSON* name = cJSON_GetObjectItem(item, "name_local");
                if (cJSON_IsString(name)) {
                    extract_mmd_items.emplace_back(name->valuestring, false);
                }
            }
        }
    } else {
        cJSON* materials = cJSON_GetObjectItem(root, "materials");
        if (materials && cJSON_IsArray(materials)) {
            int n = cJSON_GetArraySize(materials);
            for (int i = 0; i < n; ++i) {
                cJSON* item = cJSON_GetArrayItem(materials, i);
                cJSON* name = cJSON_GetObjectItem(item, "name_local");
                if (cJSON_IsString(name)) {
                    extract_mmd_items.emplace_back(name->valuestring, false);
                }
            }
        }
    }

    cJSON_Delete(root);
    extract_mmd_status_msg = get_locale_string("status.items_loaded");
}

void RenderVoxelList::extract_mmd_finish_extract(const std::string& json_text) {
    cJSON* root = cJSON_Parse(json_text.c_str());
    if (!root) {
        extract_mmd_status_msg = get_locale_string("error.parse_json");
        return;
    }

    cJSON* success = cJSON_GetObjectItem(root, "success");
    if (!cJSON_IsTrue(success)) {
        cJSON* err = cJSON_GetObjectItem(root, "error");
        extract_mmd_status_msg = err && cJSON_IsString(err)
                                     ? err->valuestring
                                     : get_locale_string("error.unknown");
        cJSON_Delete(root);
        return;
    }

    cJSON* stl = cJSON_GetObjectItem(root, "stl");
    if (cJSON_IsString(stl)) {
        extract_mmd_result_stl = stl->valuestring;
    }

    cJSON_Delete(root);

    // Load the resulting STL as a new mesh-only node.
    if (!extract_mmd_result_stl.empty() &&
        std::filesystem::exists(utf8_path(extract_mmd_result_stl))) {
        queue_load_stl(extract_mmd_result_stl, 0.5f,
                       static_cast<int>(StlLoadMode::MESH_ONLY));
        extract_mmd_status_msg = get_locale_string("status.extract_done_loading");
    } else {
        extract_mmd_status_msg = get_locale_string("error.output_missing");
    }
}

}  // namespace sinriv::ui::render
