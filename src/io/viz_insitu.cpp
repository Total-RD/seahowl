#include "seahowl/io/viz_insitu.h"

#include <seahowl/core/system.h>

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

using seahowl::Vector3d;
using namespace seahowl::io;

void draw_system(const std::shared_ptr<chrono::ChSystem> system,
                 std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application) {}

VisualizationInSitu::VisualizationInSitu() {
    spdlog::info("No concrete in situ visualization application selected.");
}

void VisualizationInSitu::initialize_elasto(seahowl::elasto::SystemElasto& system) {}

void VisualizationInSitu::initialize(seahowl::core::System& system) {}

void VisualizationInSitu::draw() {}
