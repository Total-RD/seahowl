#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

/**
 * @brief Initialization of in situ visualization for system.
 *
 * @param[in] system System to visualize.
 * @param[in] application Application on which system is visualized.
 */
void draw_system_init(std::shared_ptr<chrono::ChSystem> system,
                      std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application);

/**
 * @brief Draw elements of systems (rigid bodies, FEA beams, etc).
 *
 * @param[in] system System to visualize.
 * @param[in] application Application on which system is visualized.
 */
void draw_system(const std::shared_ptr<chrono::ChSystem> system,
                 std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application);
