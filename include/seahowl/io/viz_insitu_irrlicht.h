#pragma once

#include "seahowl/io/viz_insitu.h"

#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

namespace chrono {
class ChSystem;
namespace irrlicht {
class ChVisualSystemIrrlicht;
}  // namespace irrlicht
}  // namespace chrono

class VisualizationInSituIrrlicht : public VisualizationInSitu {
  public:
    VisualizationInSituIrrlicht();

    /**
     * @brief Initialization of in situ visualization for system.
     *
     * @param[in] system System to visualize.
     */
    void initialize(seahowl::core::System& system) override;

    /**
     * @brief Initialization of in situ visualization for system.
     *
     * @param[in] system System to visualize.
     */
    void initialize_elasto(seahowl::elasto::SystemElasto& system_elasto) override;

    /**
     * @brief Draw elements of systems (rigid bodies, FEA beams, etc).
     */
    void draw() override;

  private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application_irrlicht;
    std::shared_ptr<chrono::ChSystem> system_chrono;
};
