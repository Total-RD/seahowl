// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

// SEAHOWL headers
#include "seahowl/elasto/turbine_elasto.h"

// Standard library
#include <deque>
#include <memory>
#include <vector>

namespace seahowl {
namespace elasto {

/**
 * @brief Elasto system base class.
 */
class SystemElasto : public ComponentElasto {
  public:
    /** @brief Turbines in system. */
    std::deque<std::shared_ptr<TurbineElasto>> turbines{};
    /** @brief Components in system. */
    std::deque<std::shared_ptr<ComponentElasto>> components{};
    /** @brief Mesh used for FEA elements. */
    std::shared_ptr<MeshElasto> mesh;
    /** @brief Whether system has been assembled or not. */
    bool is_assembled = false;
    /**
     * @brief Virtual destructor.
     */
    virtual ~SystemElasto() = default;
    void build() override;

    /**
     * @brief Assembles the system.*
     *
     * Calls assemble for each turbine of the system.
     */
    virtual void assemble() = 0;

    /**
     * @brief Presetup of system.
     *
     * @param[in] fraction Fraction of presetup phase, starting at 0.0 and ending at 1.0.
     */
    // virtual void presetup(double fraction) = 0;

    /**
     * @brief Does an elasto step.
     *
     * @param[in] dt Time step length [s]
     */
    virtual void step(double dt) = 0;

    /**
     * @brief Returns time of simulation [s]
     */
    virtual double get_time() const = 0;

    /**
     * @brief Sets time of simulation.
     *
     * @param[in] time Time of simulation [s]
     */
    virtual void set_time(double time) = 0;

    /**
     * @brief Does statics step.
     *
     * @param[in] linear Do linear statics if true.
     * @param[in] nonlinear_steps Number of nonlinear steps.
     */
    virtual void do_statics(bool linear, int nonlinear_steps) = 0;

    /**
     * @brief Returns gravitational acceleration [m/s^2]
     */
    virtual Vector3d get_gravitational_acceleration() const = 0;

    /**
     * @brief Sets gravitational acceleration.
     *
     * @param[in] gravitational_acceleration Gravitational acceleration [m/s^2]
     */
    virtual void set_gravitational_acceleration(const Vector3d& gravitational_acceleration) = 0;

    /**
     * @brief Returns system mass matrix.
     */
    virtual Eigen::MatrixXd get_mass_matrix() const = 0;

    /**
     * @brief Returns system stiffness matrix.
     */
    virtual Eigen::MatrixXd get_stiffness_matrix() const = 0;

    /**
     * @brief Returns system damping matrix.
     */
    virtual Eigen::MatrixXd get_damping_matrix() const = 0;

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

    /**
     * @brief Adds link to system.
     *
     * @param[in] link Link to add to system.
     */
    virtual void add(LinkMatrixStiffnessDamping& link) = 0;

    /**
     * @brief Adds spring to system.
     *
     * @param[in] spring Spring to add to system.
     */
    virtual void add(SpringLinear& spring) = 0;

    /**
     * @brief Adds actuator to system.
     *
     * @param[in] actuator Actuator to add to system.
     */
    virtual void add(ActuatorRotation& actuator) = 0;

    /**
     * @brief Adds component to system.
     *
     * @param[in] component Component to add to system.
     */
    virtual void add(std::shared_ptr<ComponentElasto> component);

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    virtual void add(std::shared_ptr<TurbineElasto> turbine);

    virtual void translate(const seahowl::Vector3d& translation_vector) const override;
    virtual void rotate(double angle, const seahowl::Vector3d& axis) const override;
    virtual double get_mass() const override;

  protected:
    virtual void assemble_this(seahowl::elasto::SystemElasto& system) override;
};

}  // namespace elasto
}  // namespace seahowl
