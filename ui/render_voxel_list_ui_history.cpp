#include <dear-imgui/imgui_internal.h>
#include <iconfontheaders/icons_font_awesome.h>
#include <iconfontheaders/icons_kenney.h>
#include <imgui/imgui.h>
#include <imnodes.h>
#include <stb/stb_truetype.h>
#include <type_traits>
#include <unordered_set>
#include <variant>
#ifdef _WIN32
#include <windows.h>
#endif
#include "kigstudio/cgal/poisson_reconstruction.h"
#include "kigstudio/cgal/poisson_utils.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/utils/locale.h"
#include "render_voxel_list.h"
#include "tinyfiledialogs.h"
namespace sinriv::ui::render {
void RenderVoxelList::render_history_window() {
    if (!show_history_window)
        return;

    std::lock_guard<std::mutex> lock(locker);
    auto it = items.find(render_id);
    if (it == items.end()) {
        if (ImGui::Begin(get_locale_cstr("window.history"),
                         &show_history_window)) {
            ImGui::TextUnformatted(get_locale_cstr("label.no_active_item"));
        }
        ImGui::End();
        return;
    }

    RenderVoxelItem& item = *it->second;
    if (ImGui::Begin(get_locale_cstr("window.history"), &show_history_window)) {
        ImGui::Text(get_locale_cstr("label.render_item"), item.id);
        if (item.dirty) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "(*)");
        }
        ImGui::Separator();

        const size_t undo_count = item.undo_stack.size();
        const size_t redo_count = item.redo_stack.size();
        const int total = static_cast<int>(undo_count + redo_count + 1);
        const int current_idx = static_cast<int>(undo_count);

        ImGui::Text("%s: %d", get_locale_cstr("label.history_total"), total);
        ImGui::Separator();

        // Copy descriptions locally to avoid accessing modified stacks during
        // iteration
        std::vector<std::string> undo_descs;
        undo_descs.reserve(undo_count);
        for (size_t i = 0; i < undo_count; ++i) {
            undo_descs.push_back(item.undo_stack[i].description.empty()
                                     ? "State"
                                     : item.undo_stack[i].description);
        }
        std::vector<std::string> redo_descs;
        redo_descs.reserve(redo_count);
        for (size_t i = 0; i < redo_count; ++i) {
            redo_descs.push_back(
                item.redo_stack[redo_count - 1 - i].description.empty()
                    ? "State"
                    : item.redo_stack[redo_count - 1 - i].description);
        }

        // Undo list (oldest first)
        if (undo_count > 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_undo"));
            ImGui::PushID("undo");
            for (size_t i = 0; i < undo_count; ++i) {
                char label[128];
                snprintf(label, sizeof(label), "[%zu] %s", i,
                         undo_descs[i].c_str());
                ImGui::PushID(static_cast<int>(i));
                if (i == undo_count - 1) {
                    ImGui::PushStyleColor(ImGuiCol_Text,
                                          ImVec4(0.6f, 0.8f, 1.0f, 1.0f));
                }
                if (ImGui::Selectable(label, i == undo_count - 1)) {
                    // Click to jump: undo until we reach this state
                    size_t steps = undo_count - 1 - i;
                    for (size_t s = 0; s < steps; ++s) {
                        undo(item.id);
                    }
                }
                if (i == undo_count - 1) {
                    ImGui::PopStyleColor();
                }
                ImGui::PopID();
            }
            ImGui::PopID();
        }

        // Current state
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 1.0f, 0.4f, 1.0f));
        ImGui::PushID("current");
        ImGui::Selectable(get_locale_cstr("label.history_current"), true,
                          ImGuiSelectableFlags_Disabled);
        ImGui::PopID();
        ImGui::PopStyleColor();

        // Redo list (newest first, displayed top to bottom)
        if (redo_count > 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_redo"));
            ImGui::PushID("redo");
            for (size_t i = 0; i < redo_count; ++i) {
                char label[128];
                snprintf(label, sizeof(label), "[%zu] %s", i,
                         redo_descs[i].c_str());
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::Selectable(label, false)) {
                    // Click to jump: redo until we reach this state
                    size_t steps = i + 1;
                    for (size_t s = 0; s < steps; ++s) {
                        redo(item.id);
                    }
                }
                ImGui::PopID();
            }
            ImGui::PopID();
        }

        if (undo_count == 0 && redo_count == 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_empty"));
        }

        // === Marked Voxels History ===
        ImGui::Separator();
        ImGui::TextDisabled("%s",
                            get_locale_cstr("label.history_marked_title"));
        ImGui::Separator();

        const size_t marked_undo_count = item.marked_undo_stack.size();
        const size_t marked_redo_count = item.marked_redo_stack.size();
        const int marked_total =
            static_cast<int>(marked_undo_count + marked_redo_count + 1);
        const int marked_current_idx = static_cast<int>(marked_undo_count);

        ImGui::Text("%s: %d", get_locale_cstr("label.history_total"),
                    marked_total);
        ImGui::Separator();

        std::vector<std::string> marked_undo_descs;
        marked_undo_descs.reserve(marked_undo_count);
        for (size_t i = 0; i < marked_undo_count; ++i) {
            marked_undo_descs.push_back(
                item.marked_undo_stack[i].description.empty()
                    ? "State"
                    : item.marked_undo_stack[i].description);
        }
        std::vector<std::string> marked_redo_descs;
        marked_redo_descs.reserve(marked_redo_count);
        for (size_t i = 0; i < marked_redo_count; ++i) {
            marked_redo_descs.push_back(
                item.marked_redo_stack[marked_redo_count - 1 - i]
                        .description.empty()
                    ? "State"
                    : item.marked_redo_stack[marked_redo_count - 1 - i]
                          .description);
        }

        if (marked_undo_count > 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_undo"));
            ImGui::PushID("marked_undo");
            for (size_t i = 0; i < marked_undo_count; ++i) {
                char label[128];
                snprintf(label, sizeof(label), "[%zu] %s", i,
                         marked_undo_descs[i].c_str());
                ImGui::PushID(static_cast<int>(i));
                if (i == marked_undo_count - 1) {
                    ImGui::PushStyleColor(ImGuiCol_Text,
                                          ImVec4(0.6f, 0.8f, 1.0f, 1.0f));
                }
                if (ImGui::Selectable(label, i == marked_undo_count - 1)) {
                    size_t steps = marked_undo_count - 1 - i;
                    for (size_t s = 0; s < steps; ++s) {
                        undo_marked(item.id);
                    }
                }
                if (i == marked_undo_count - 1) {
                    ImGui::PopStyleColor();
                }
                ImGui::PopID();
            }
            ImGui::PopID();
        }

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 1.0f, 0.4f, 1.0f));
        ImGui::PushID("marked_current");
        ImGui::Selectable(get_locale_cstr("label.history_current"), true,
                          ImGuiSelectableFlags_Disabled);
        ImGui::PopID();
        ImGui::PopStyleColor();

        if (marked_redo_count > 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_redo"));
            ImGui::PushID("marked_redo");
            for (size_t i = 0; i < marked_redo_count; ++i) {
                char label[128];
                snprintf(label, sizeof(label), "[%zu] %s", i,
                         marked_redo_descs[i].c_str());
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::Selectable(label, false)) {
                    size_t steps = i + 1;
                    for (size_t s = 0; s < steps; ++s) {
                        redo_marked(item.id);
                    }
                }
                ImGui::PopID();
            }
            ImGui::PopID();
        }

        if (marked_undo_count == 0 && marked_redo_count == 0) {
            ImGui::TextDisabled("%s", get_locale_cstr("label.history_empty"));
        }
    }
    ImGui::End();
}

void RenderVoxelList::render_log_window() {
    if (!show_log_window)
        return;

    std::string log_text_copy;
    {
        std::lock_guard<std::mutex> lock(queue_log_mutex);
        log_text_copy = queue_log_text;
    }

    ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_Once);
    if (ImGui::Begin(get_locale_cstr("window.log"), &show_log_window)) {
        if (log_text_copy.empty()) {
            ImGui::TextDisabled(get_locale_cstr("label.no_log_entries"));
        } else {
            if (log_text_copy.size() + 1 > queue_log_buffer.size()) {
                queue_log_buffer.resize(log_text_copy.size() + 1);
            }
            std::copy(log_text_copy.begin(), log_text_copy.end(),
                      queue_log_buffer.begin());
            queue_log_buffer[log_text_copy.size()] = '\0';
            ImGui::InputTextMultiline(
                "##log", queue_log_buffer.data(), queue_log_buffer.size(),
                ImVec2(-FLT_MIN, -FLT_MIN), ImGuiInputTextFlags_ReadOnly);
        }
    }
    ImGui::End();
}
}