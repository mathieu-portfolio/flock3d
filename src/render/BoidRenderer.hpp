#pragma once

#include "sim/BoidSimulation.hpp"

namespace flock3d::render {

class BoidRenderer {
public:
    void draw(const sim::BoidSimulation& simulation) const;
};

} // namespace flock3d::render
