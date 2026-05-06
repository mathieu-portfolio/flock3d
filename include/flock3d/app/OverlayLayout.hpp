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
    std::size_t line_count{29};
    std::size_t parameter_line_start{21};
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
    return line_index == 0 || line_index == 8 || line_index == 13 || line_index == 20;
}

} // namespace flock3d::app
