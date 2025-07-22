#pragma once

#include <iostream>
#include <cstring>
#include <memory>

#include <seahowl/env/wind_models.h>

namespace seahowl {

namespace core {
class Turbine;
}

namespace env {
// forward declare (defined in .cpp file)
/**
 * @brief Interface to InflowWind library.
 */
struct InflowWindLib;

/**
 * @brief Adapter to InflowWind library.
 */
class InflowWindAdapter : public WindModel {
  public:
    /** @brief Level below which returned velocity is (0.0, 0.0, 0.0), used for z<0 when using TurbSim for example. */
    double zmin = 0.0;

    std::unique_ptr<InflowWindLib> pImpl;

    /**
     * @brief Returns true is the placement of the model (false otherwise).
     *
     * @param[in] position Position to assess whether inside model or not.
     * @param[in] time Time of simulation.
     */
    virtual bool is_inside(const Vector3d& position, double time = 0.0) const override;

    InflowWindAdapter(std::string inflowwind_infile);
    ~InflowWindAdapter();

    std::string get_inflowwind_infile() const { return inflowwind_infile; }

    void end();

  protected:
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
    std::string inflowwind_infile = "";
};

}  // namespace env
}  // namespace seahowl
