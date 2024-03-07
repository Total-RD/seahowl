#pragma once

#include <memory>

// forward declarations
namespace seahowl {
namespace core {
class System;
}  // namespace core
namespace elasto {
class SystemElasto;
}  // namespace elasto
}  // namespace seahowl

namespace chrono {
class ChSystem;
namespace irrlicht {
class ChVisualSystemIrrlicht;
}  // namespace irrlicht
}  // namespace chrono

class VisualizationInSitu {
  public:
    VisualizationInSitu();

    /**
     * @brief Initialization of in situ visualization for system.
     *
     * @param[in] system System to visualize.
     */
    virtual void initialize(seahowl::core::System& system);

    /**
     * @brief Initialization of in situ visualization for system.
     *
     * @param[in] system System to visualize.
     */
    virtual void initialize_elasto(seahowl::elasto::SystemElasto& system_elasto);

    /**
     * @brief Draw elements of systems (rigid bodies, FEA beams, etc).
     */
    virtual void draw();
};
