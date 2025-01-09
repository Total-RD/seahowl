#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/aero/rotor_aero.h"
#include "seahowl/commons/component_fluid.h"

#include <iostream>
#include <cstring>
#include <memory>

namespace seahowl {
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

/// <summary>
/// Aerodyn module in OpenFAST
/// </summary>

namespace seahowl {

namespace aero {

// forward declare (defined in .cpp file)
struct AeroDynInflowLib;

class AeroDynAdapter {
  public:
    std::unique_ptr<seahowl::aero::AeroDynInflowLib> pImpl;
    std::vector<Vector3d> loads;

    AeroDynAdapter();
    AeroDynAdapter(std::string AerodynInfile, std::string InflowInfile);
    ~AeroDynAdapter();

    void set_infiles(const std::string& AerodynInfile, const std::string& InflowInfile);
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

    TurbineAeroDyn();
    void initialize(double time, double dt) override;
    void compute_fluid_loads(const env::FluidModel& fluid_model, double time) override;
};

class RotorAeroDyn : public RotorAeroBEMT {
  public:
    float* loads_aerodyn;

    RotorAeroDyn(TowerAero& tower_ref);
    virtual void compute_fluid_loads(const env::FluidModel& fluid_model, double time) override;
};

}  // namespace aero
}  // namespace seahowl
