#pragma once

#include <vector>
#include <memory>
#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace env {

/**
 * @brief Class to store list of fluids
 */
template <typename T>
class ListModel {
  public:
    /**
     * @brief Adds a model to the list
     * @param[in] model Model to add
     */
    void add_model(const std::shared_ptr<T>& model);

    /**
     * @brief Inserts a model to the list
     * @param[in] model Model to insert
     */
    void insert_model(const std::shared_ptr<T>& model);

    /**
     * @brief Returns the list of models
     * @return Vector of models
     */
    const std::vector<std::shared_ptr<T>>& get_models() const;

    /**
     * @brief Returns the fluid model
     * @param[in] position Position at which fluid model is extracted.
     * @param[in] time Time of simulation.
     */
    std::shared_ptr<T>& get_model(const Vector3d& position, double time);

    /**
     * @brief Template function to access elements of a specific type in models
     * @tparam U Type of the elements to access
     * @return Vector of elements of type U
     */
    template <typename U>
    std::vector<std::shared_ptr<U>> get_models_of_type() const;

    /**
     * @brief Returns true if there are models in the list
     */
    bool has_model() const;

  protected:
    /** @brief List of models */
    std::vector<std::shared_ptr<T>> models;
};

template <typename T>
void ListModel<T>::add_model(const std::shared_ptr<T>& model) {
    models.push_back(model);
}

template <typename T>
void ListModel<T>::insert_model(const std::shared_ptr<T>& model) {
    models.insert(models.begin(), model);
}

template <typename T>
bool ListModel<T>::has_model() const {
    return !models.empty();
}

template <typename T>
const std::vector<std::shared_ptr<T>>& ListModel<T>::get_models() const {
    return models;
}

template <typename T>
std::shared_ptr<T>& ListModel<T>::get_model(const Vector3d& position, double time) {
    for (auto& model : models) {
        if (model->is_inside(position, time)) {
            return model;
        }
    }
    throw std::runtime_error("No model found at given position and time.");
}

template <typename T>
template <typename U>
std::vector<std::shared_ptr<U>> ListModel<T>::get_models_of_type() const {
    std::vector<std::shared_ptr<U>> elements;
    for (const auto& model : models) {
        if (std::shared_ptr<U> element = std::dynamic_pointer_cast<U>(model)) {
            elements.push_back(element);
        }
    }
    return elements;
}

}  // namespace env
}  // namespace seahowl
