#include "seahowl/env/env_model.h"
#include "seahowl/env/wind_models.h"

using namespace seahowl::env;
using seahowl::Vector3d;

void EnvModel::add_model(const std::shared_ptr<Model>& model) {
    if (std::shared_ptr<FluidModel> fluidmodel = std::dynamic_pointer_cast<FluidModel>(model))
        if (std::shared_ptr<WindModel> windmodel = std::dynamic_pointer_cast<WindModel>(fluidmodel))
            fluid_models.add_model(fluidmodel);
        else
            fluid_models.insert_model(fluidmodel);
    if (std::shared_ptr<SoilModel> soilmodel = std::dynamic_pointer_cast<SoilModel>(model))
        soil_models.add_model(soilmodel);
}
