#pragma once

#include <string>

#include <hydroc/hydro_forces.h>

class FloaterHydro {
  public:
    std::string filepath_h5_potential;
    std::string name;
    TestHydro hydro;

    void set(std::vector<std::shared_ptr<chrono::ChBody>> bodies, HydroInputs& hydro_inputs) {
        // hydro = TestHydro(bodies, filepath_h5_potential, hydro_inputs);
    }
};
