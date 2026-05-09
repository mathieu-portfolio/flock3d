#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include <flock3d/app/ControlHelpers.hpp>

namespace flock3d::app {

enum class ControlCommandType {
    ToggleOverlay,
    TogglePause,
    ResetSimulation,
    AdjustBoidCount,
    SelectParameter,
    OffsetSelectedParameter,
    AdjustSelectedParameter,
    ApplyNextScenario,
    ApplyPreviousScenario,
    RandomizeSeed,
    ToggleRecording,
    CycleExportMode,
    AdjustSampleRate,
};

struct ControlCommand {
    ControlCommandType type;
    int amount{};
    double scale{1.0};
    TunableParameter parameter{TunableParameter::separation_weight};

    [[nodiscard]] static constexpr ControlCommand toggle_overlay() noexcept
    {
        return ControlCommand{ControlCommandType::ToggleOverlay};
    }

    [[nodiscard]] static constexpr ControlCommand toggle_pause() noexcept
    {
        return ControlCommand{ControlCommandType::TogglePause};
    }

    [[nodiscard]] static constexpr ControlCommand reset_simulation() noexcept
    {
        return ControlCommand{ControlCommandType::ResetSimulation};
    }

    [[nodiscard]] static constexpr ControlCommand adjust_boid_count(int delta) noexcept
    {
        ControlCommand command{ControlCommandType::AdjustBoidCount};
        command.amount = delta;
        return command;
    }

    [[nodiscard]] static constexpr ControlCommand select_parameter(TunableParameter parameter) noexcept
    {
        ControlCommand command{ControlCommandType::SelectParameter};
        command.parameter = parameter;
        return command;
    }

    [[nodiscard]] static constexpr ControlCommand offset_selected_parameter(int direction) noexcept
    {
        ControlCommand command{ControlCommandType::OffsetSelectedParameter};
        command.amount = direction;
        return command;
    }

    [[nodiscard]] static constexpr ControlCommand adjust_selected_parameter(int direction) noexcept
    {
        ControlCommand command{ControlCommandType::AdjustSelectedParameter};
        command.amount = direction;
        return command;
    }

    [[nodiscard]] static constexpr ControlCommand apply_next_scenario() noexcept
    {
        return ControlCommand{ControlCommandType::ApplyNextScenario};
    }

    [[nodiscard]] static constexpr ControlCommand apply_previous_scenario() noexcept
    {
        return ControlCommand{ControlCommandType::ApplyPreviousScenario};
    }

    [[nodiscard]] static constexpr ControlCommand randomize_seed() noexcept
    {
        return ControlCommand{ControlCommandType::RandomizeSeed};
    }

    [[nodiscard]] static constexpr ControlCommand toggle_recording() noexcept
    {
        return ControlCommand{ControlCommandType::ToggleRecording};
    }

    [[nodiscard]] static constexpr ControlCommand cycle_export_mode() noexcept
    {
        return ControlCommand{ControlCommandType::CycleExportMode};
    }

    [[nodiscard]] static constexpr ControlCommand adjust_sample_rate(double scale) noexcept
    {
        ControlCommand command{ControlCommandType::AdjustSampleRate};
        command.scale = scale;
        return command;
    }
};

class ControlCommandQueue {
public:
    void enqueue(ControlCommand command)
    {
        std::scoped_lock lock{mutex_};
        commands_.push_back(command);
    }

    [[nodiscard]] std::vector<ControlCommand> drain()
    {
        std::scoped_lock lock{mutex_};
        return std::exchange(commands_, {});
    }

    [[nodiscard]] bool empty() const
    {
        std::scoped_lock lock{mutex_};
        return commands_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::vector<ControlCommand> commands_;
};

} // namespace flock3d::app
