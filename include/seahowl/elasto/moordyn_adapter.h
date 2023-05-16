#pragma once

#include <seahowl/commons/numerics.h>
#include <seahowl/elasto/component_elasto.h>
#include <seahowl/elasto/blade_elasto.h>

#include <MoorDyn2.h>

namespace seahowl {
namespace elasto {
class TurbineElasto;
}  // namespace elasto
}  // namespace seahowl

namespace seahowl {
namespace elasto {

class MoorDynAdapter {
  public:
    Eigen::VectorXd fairlead_position;
    Eigen::VectorXd fairlead_velocity;
    Eigen::VectorXd fairlead_load;

    MoorDynAdapter(std::string MoordynInfile);
    ~MoorDynAdapter();

    void initialize(seahowl::elasto::TurbineElasto& turbine);  // to be changed to floater entity
    void step(double time, double dt, seahowl::elasto::TurbineElasto& turbine);
    void end();
    void saveVTK(std::string filename, double time, double dt);

  private:
    MoorDyn moordynobj;
    int ErrStat = MOORDYN_SUCCESS;
};

}  // namespace elasto
}  // namespace seahowl
