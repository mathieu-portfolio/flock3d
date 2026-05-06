#pragma once

#include <cstddef>

#include <flock3d/app/ControlHelpers.hpp>

namespace flock3d::app {

struct OverlayLayout {
    int text_x{16};
    int text_y{16};
    int line_height{20};
    int padding{14};
    int font_size{16};
    int panel_width{520};
    std::size_t line_count{46};
    std::size_t parameter_line_start{34};
};

struct OverlayRect {
    int x{};
    int y{};
    int width{};
    int height{};
};

[[nodiscard]] constexpr int overlay_panel_height(const OverlayLayout& layout) noexcept
{
    return static_cast<int>(layout.line_count) * layout.line_height + layout.padding * 2;
}

[[nodiscard]] constexpr OverlayRect overlay_panel_rect(const OverlayLayout& layout) noexcept
{
    return OverlayRect{layout.text_x - layout.padding, layout.text_y - layout.padding, layout.panel_width, overlay_panel_height(layout)};
}

[[nodiscard]] constexpr int overlay_min_visible_height(const OverlayLayout& layout) noexcept
{
    return layout.padding * 2 + layout.line_height * 6;
}

[[nodiscard]] constexpr int overlay_visible_panel_height(const OverlayLayout& layout, int screen_height) noexcept
{
    const auto panel = overlay_panel_rect(layout);
    const int available_height = screen_height - panel.y - layout.padding;
    if (available_height >= panel.height) {
        return panel.height;
    }
    if (available_height <= overlay_min_visible_height(layout)) {
        return overlay_min_visible_height(layout);
    }
    return available_height;
}

[[nodiscard]] constexpr int overlay_max_scroll_offset(const OverlayLayout& layout, int screen_height) noexcept
{
    const int overflow = overlay_panel_height(layout) - overlay_visible_panel_height(layout, screen_height);
    return overflow > 0 ? overflow : 0;
}

[[nodiscard]] constexpr int overlay_clamped_scroll_offset(
    const OverlayLayout& layout, int screen_height, int scroll_offset) noexcept
{
    const int max_scroll = overlay_max_scroll_offset(layout, screen_height);
    if (scroll_offset <= 0) {
        return 0;
    }
    if (scroll_offset >= max_scroll) {
        return max_scroll;
    }
    return scroll_offset;
}

[[nodiscard]] constexpr std::size_t overlay_selected_parameter_line(const OverlayLayout& layout, TunableParameter parameter) noexcept
{
    return layout.parameter_line_start + parameter_index(parameter);
}

[[nodiscard]] constexpr int overlay_text_local_x(const OverlayLayout& layout) noexcept
{
    return layout.padding;
}

[[nodiscard]] constexpr int overlay_line_local_y(const OverlayLayout& layout, std::size_t line_index) noexcept
{
    return layout.padding + static_cast<int>(line_index) * layout.line_height;
}

[[nodiscard]] constexpr OverlayRect overlay_highlight_local_rect(const OverlayLayout& layout, std::size_t line_index) noexcept
{
    return OverlayRect{
        layout.padding - 6,
        overlay_line_local_y(layout, line_index) - 2,
        layout.panel_width - layout.padding * 2 + 2,
        layout.line_height,
    };
}

[[nodiscard]] constexpr bool overlay_is_section_header(std::size_t line_index) noexcept
{
    return line_index == 0 || line_index == 12 || line_index == 19 || line_index == 24 || line_index == 33;
}

} // namespace flock3d::app
