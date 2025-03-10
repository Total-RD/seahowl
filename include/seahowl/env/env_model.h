#pragma once

#include <vector>
#include <memory>
#include "seahowl/env/model.h"
#include "seahowl/env/fluid_list_model.h"
#include "seahowl/env/soil_list_model.h"

namespace seahowl {
namespace env {
/**
 * @brief Class to store multiple models
 */
class EnvModel {
  public:
    /** @brief List of fluid models */
    FluidListModel fluid_models;
    /** @brief List of soil models */
    SoilListModel soil_models;

    EnvModel() = default;
    ~EnvModel() = default;
    /**
     * @brief Adds a model
     */
    void add_model(const std::shared_ptr<Model>& model);
};
}  // namespace env
}  // namespace seahowl
