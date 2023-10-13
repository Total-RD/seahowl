#pragma once

#include "seahowl/elasto/turbine_elasto.h"  // @todo forward declare Turbine

#include <vector>
#include <deque>
#include <memory>

namespace seahowl {
///@brief Elastodynamic model module
namespace elasto {

/**
 * @brief Elasto system base class.
 */
class SystemElasto {
  public:
    /** @brief Turbines in system. */
    std::deque<std::shared_ptr<TurbineElasto>> turbines{};
    /** @brief Mesh used for FEA elements. */
    std::shared_ptr<MeshElasto> mesh;

    /**
     * @brief Does an elasto step.
     *
     * @param[in] dt Time step length.
     */
    virtual void step(double dt) = 0;

    /**
     * @brief Returns time of simulation.
     */
    virtual double get_time() const = 0;

    /**
     * @brief Does statics step.
     *
     * @param[in] linear Do linear statics if true.
     * @param[in] nonlinear_steps Number of nonlinear steps.
     */
    virtual void do_statics(bool linear, int nonlinear_steps) = 0;

    /**
     * @brief Returns gravitational acceleration.
     */
    virtual Vector3d get_gravitational_acceleration() const = 0;

    /**
     * @brief Sets gravitational acceleration.
     *
     * @param[in] gravitational_acceleration Gravitational acceleration.
     */
    virtual void set_gravitational_acceleration(const Vector3d& gravitational_acceleration) = 0;

    /**
     * @brief Adds body to system.
     *
     * @param[in] body Body to add to system.
     */
    virtual void add(BodyElasto& body) = 0;

    /**
     * @brief Adds mesh to system.
     *
     * @param[in] mesh Mesh to add to system.
     */
    virtual void add(MeshElasto& mesh) = 0;

    /**
     * @brief Adds link to system.
     *
     * @param[in] link Link to add to system.
     */
    virtual void add(Link& link) = 0;
};

}  // namespace elasto
}  // namespace seahowl
