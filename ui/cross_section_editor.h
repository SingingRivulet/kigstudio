#pragma once

#include <vector>
#include <string>

#include "kigstudio/utils/vec2.h"

namespace sinriv::ui::render {

using vec2f = sinriv::kigstudio::vec2<float>;

/// Normalization mode for cross-section vertices on Apply.
/// The polygon is uniformly scaled so the chosen dimension has half-extent = 1.
enum class NormalizeMode {
    NORMALIZE_X = 0,   // scale proportionally so X half-extent = 1
    NORMALIZE_Y = 1,   // scale proportionally so Y half-extent = 1
    NORMALIZE_XY = 2,  // scale proportionally so max(|x|, |y|) half-extent = 1
};

/// Snapshot for the cross-section editor's independent undo/redo.
struct SectionEditorSnapshot {
    std::vector<vec2f> vertices;
    std::string description;
};

/// Per-strand editor state including the polygon vertices and
/// an independent undo/redo stack (not shared with CollisionEditorSnapshot).
struct SectionEditorState {
    // --- polygon data ---
    std::vector<vec2f> vertices;      // working copy (editable in canvas)
    std::vector<vec2f> committed;     // committed copy (set by Apply button)

    // --- Bézier interpolation ---
    bool use_bezier_section = true;  // smooth cross-section via Catmull-Rom

    // --- Normalization mode ---
    NormalizeMode normalize_mode = NormalizeMode::NORMALIZE_XY;

    // --- independent undo/redo ---
    std::vector<SectionEditorSnapshot> undo_stack;
    std::vector<SectionEditorSnapshot> redo_stack;
    static constexpr size_t kMaxUndoSize = 50;

    // --- per-frame interaction state (reset each frame) ---
    int hovered_vertex = -1;  // index of vertex under cursor
    int hovered_edge = -1;    // index of the edge (i, i+1) under cursor
    int dragged_vertex = -1;  // index of vertex currently being dragged
    vec2f drag_offset = {0, 0};  // offset from vertex to mouse at drag start

    // --- multi-frame drag tracking for undo ---
    bool edit_active = false;

    /// Reset per-frame hit-test state (called at top of render).
    void reset_frame_state() {
        hovered_vertex = -1;
        hovered_edge = -1;
    }

    // --- undo/redo methods (implemented in cross_section_editor.cpp) ---
    void push_undo(const std::string& desc = "Edit");
    bool undo();
    bool redo();
    bool can_undo() const;
    bool can_redo() const;
};

}  // namespace sinriv::ui::render
