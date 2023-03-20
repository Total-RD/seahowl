#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/tower.h>
#include <seahowl/servo/controller.h>
#include <seahowl/aero/aerodyn_adapter.h>
#include <seahowl/commons/numerics.h>

#include <vector>

/**@brief Seahowl base namespace */
namespace seahowl {
namespace servo {
class Controller;
}

namespace aero {
class AeroDynAdapter;
}

/**@brief Seahowl core module */
namespace core {

/**
 * @brief Wind turbine (blades, rotor-nacelle assembly, tower).
 *
 * This class controls each component, ensuring proper workflow and communication within and between the component.
 */
class Turbine : public ComponentDynamic {
  public:
    // components
    //
    /** @brief Blades of the turbine. */
    std::vector<std::shared_ptr<Blade>> blades;
    /** @brief Rotor-nacelle assembly of the turbine. */
    Rotor rotor;
    /** @brief Tower of the turbine. */
    Tower tower;
    /** @brief Controller of the turbine. */
    std::shared_ptr<seahowl::servo::Controller> controller;
    /** @brief AeroDyn adapter (only used if AeroDyn is enabled) (@todo this should not be here, move to aero parts). */
    std::shared_ptr<seahowl::aero::AeroDynAdapter> aerodyn;

    // parameters
    //
    /** @brief Efficiency of the generator. */
    double generator_efficiency = 1.0;
    /** @brief Ratio of the gearbox. */
    double gearbox_ratio = 1.0;
    /** @brief Efficiency of the gearbox. */
    double gearbox_efficiency = 1.0;

    /** @brief Whether to use AeroDyn or not (@todo move to aero part). */
    bool use_aerodyn = false;
    /** @brief Option to save VTK in AeroDyn, 0: none; 1: init only; 2: animation (@todo move to aero part). */
    int WrVTK = 0;
    /** @brief VTK save type, 1: surface; 2: lines; 3: both (@todo move to aero part). */
    int WrVTK_Type = 1;
    /** @brief VTK save time step (@todo move to aero part). */
    double WrVTK_dt;

    /**
     * @brief Constructor.
     *
     * Instantiates empty blade list, rotor component, tower component, and controller component.
     */
    Turbine();

    /**
     * @brief Initialize turbine, called before starting the simulation.
     *
     * Calls init for each of its components.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void init(double time, double dt) override;

    /**
     * @brief Prestep for turbine, called before elastodynamic stepping.
     *
     * Calls prestep on each of the components of the turbine.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for turbine, called after elastodynamic stepping.
     *
     * Applies step for the controller (potentially modifying loads with electrical torque and elasto positions due to
     * blade pitching), and then calls poststep on each of the components of the turbine.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    /**
     * @brief Assembles the turbine (elasto part).*
     *
     * Calls assemble for each of the components of the turbine.
     *
     * @param[out] system System on which to add bodies, links, etc.
     * @param[out] mesh Mesh on which to add nodes and elements.
     */
    void assemble(std::shared_ptr<seahowl::elasto::SystemElasto> system,
                  std::shared_ptr<seahowl::elasto::MeshElasto> mesh);

    /**
     * @brief Builds the turbine.
     *
     * Calls build for each of the components of the turbine.
     */
    void build();

    /**
     * @brief Translates the turbine.
     *
     * @param[in] translation_vector The 3D translation vector.
     */
    void translate(Vector3d translation_vector);

    /**
     * @brief Rotates the turbine.
     *
     * @param[in] translation_vector The angle of rotation (in radians).
     * @param[in] axis The axis of rotation (3D vector).
     */
    void rotate(double angle, Vector3d axis);

    /**
     * @brief Computes wind loads on blades and tower.
     *
     * @param[in] wind_model Wind model to use for retrieving wind velocity.
     * @param[in] time Time of the simulation.
     */
    void compute_wind_loads(seahowl::aero::WindModel& wind_model, double time);

    /**
     * @brief Returns generated power.
     */
    double get_generated_power() const;

    /**
     * @brief Returns generator RPM.
     */
    double get_generator_rpm() const;
};

}  // namespace core
}  // namespace seahowl
