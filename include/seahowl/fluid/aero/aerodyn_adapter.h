#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/fluid/aero/turbine_aero.h"
#include "seahowl/fluid/aero/rotor_aero.h"
#include "seahowl/commons/component_fluid.h"

#include <iostream>
#include <cstring>
#include <memory>

namespace seahowl {
namespace env {
class EnvModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {

namespace aero {

// forward declare (defined in .cpp file)
/**
 * @brief Interface to AeroDyn library.
 */
struct AeroDynInflowLib;

/**
 * @brief Adapter to AeroDyn library.
 */
class AeroDynAdapter {
  public:
    std::unique_ptr<seahowl::aero::AeroDynInflowLib> pImpl;
    std::vector<Vector3d> forces_aerodyn;
    std::vector<Vector3d> moments_aerodyn;
    Vector3d disk_averaged_velocity;

    AeroDynAdapter();
    ~AeroDynAdapter();

    void set_aerodyn_infile(const std::string& aerodyn_Infile);
    void set_inflowwind_infile(const std::string& inflowwind_infile);
    void initialize(double time, double dt, seahowl::aero::TurbineAero& turbine);
    void compute_loads(double time, seahowl::aero::TurbineAero& turbine);
    void end();

  private:
    void update_turbine_variables(seahowl::aero::TurbineAero& turbine);
    void update_hub_motion(seahowl::aero::TurbineAero& turbine);
    void update_nacelle_motion(seahowl::aero::TurbineAero& turbine);
    void update_roots_motion(seahowl::aero::TurbineAero& turbine);
    void update_mesh_motion(seahowl::aero::TurbineAero& turbine);
};

/**
 * @brief Wind turbine with aero loads computed from AeroDyn.
 */
class TurbineAeroDyn : public TurbineAero {
  public:
    /** @brief AeroDyn adapter. */
    seahowl::aero::AeroDynAdapter aerodyn;
    /** @brief Option to save VTK in AeroDyn, 0: none; 1: init only; 2: animation. */
    int WrVTK = 0;
    /** @brief VTK save type, 1: surface; 2: lines; 3: both. */
    int WrVTK_Type = 1;
    /** @brief VTK save time step. */
    double WrVTK_dt;

    TurbineAeroDyn(const std::string& aerodyn_Infile);
    void setup_environment(const env::EnvModel& env_model) override;
    void initialize(double time, double dt) override;
    void compute_env_loads(const env::EnvModel& env_model, double time) override;
};

/**
 * @brief Rotor with aero loads computed from AeroDyn.
 */
class RotorAeroDyn : public RotorAeroBEMT {
  public:
    RotorAeroDyn(TowerAero& tower_ref);
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) override;
    virtual void compute_disk_averaged_wind_velocity(const env::EnvModel& env_model, double time) override;
};

}  // namespace aero
}  // namespace seahowl
