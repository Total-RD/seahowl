#pragma once

#include <vector>
#include <memory>
#include "seahowl/env/model.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/commons/entities.h"

namespace seahowl {
namespace env {
/**
 * @brief Class to store multiple models
 */
class EnvModel {
  public:
    EnvModel() = default;
    ~EnvModel() = default;
    /**
     * @brief Adds a model to the list of models
     */
    void addModel(std::shared_ptr<Model> model);

    /**
     * @brief Returns the list of models
     * @param[out] models List of models
     */
    const std::vector<std::shared_ptr<Model>>& getModels() const;

    /**
     * @brief Returns fluid density at given coordinates.
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    double get_fluid_density(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid velocity at given coordinates.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_fluid_velocity(const Vector3d& position, double time) const;

    /**
     * @brief Returns fluid acceleration at given coordinates.
     *
     * @param[in] position Position at which fluid acceleration is extracted.
     * @param[in] time Time of simulation.
     */
    Vector3d get_fluid_acceleration(const Vector3d& position, double time) const;

    /**
     * @brief Returns soil penetration load.
     *
     * @param[in] entity Dynamic entity (potentially) penetrating soil.
     * @param[in] contact_area Contact area of entity penetrating soil.
     * @param[in] entity_mass Mass of entity penetrating soil.
     */
    Vector3d get_penetration_load(const EntityDynamic& entity, double contact_area, double entity_mass) const;

    /**
     * @brief Returns water level.
     */
    double get_water_level(const Vector3d& position, double time) const;

    /**
     * @brief Template function to access elements of a specific type in models
     * @tparam T Type of the elements to access
     * @return Vector of elements of type T
     */
    template <typename T>
    std::vector<std::shared_ptr<T>> get_models_of_type() const;

    /**
     * @brief Template function to check if a specific type of element is present in models
     * @tparam T Type of the element to check
     */
    template <typename T>
    bool has_model_of_type() const;

  private:
    std::vector<std::shared_ptr<Model>> models;
    std::vector<std::shared_ptr<FluidModel>> fluid_models;
    std::vector<std::shared_ptr<SoilModel>> soil_models;
    std::vector<std::shared_ptr<WaveModel>> wave_models;
};

template <typename T>
bool EnvModel::has_model_of_type() const {
    for (const auto& model : models) {
        if (std::dynamic_pointer_cast<T>(model)) {
            return true;
        }
    }
    return false;
}

template <typename T>
std::vector<std::shared_ptr<T>> EnvModel::get_models_of_type() const {
    std::vector<std::shared_ptr<T>> elements;
    for (const auto& model : models) {
        if (std::shared_ptr<T> element = std::dynamic_pointer_cast<T>(model)) {
            elements.push_back(element);
        }
    }
    return elements;
}

}  // namespace env
}  // namespace seahowl
