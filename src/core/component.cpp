#include "seahowl/core/component.h"

#include <typeinfo>
#include <spdlog/spdlog.h>

using namespace seahowl::core;

void ComponentDynamic::initialize(double time, double dt) {
    spdlog::debug("Initialization of component: {}.", std::string(typeid(*this).name()));
    if (is_initialized) {
        throw std::runtime_error("Component already initialized: " + std::string(typeid(*this).name()) + ".");
    }

    initialize_this(time, dt);  // component-specific initialization
    is_initialized = true;

    spdlog::debug("Finished initialization of component: {}.", std::string(typeid(*this).name()));
}
