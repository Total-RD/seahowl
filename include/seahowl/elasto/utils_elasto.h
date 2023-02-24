#pragma once

#include <chrono/physics/ChLoaderU.h>
#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {

/**
 * @brief Weighted loader extending Chrono class.
 */
class ChLoaderWeighted : public chrono::ChLoaderUdistributed {
  public:
    std::vector<chrono::ChVector<double>> loads;
    std::vector<double> positions;
    int integration_points;

    ChLoaderWeighted(std::shared_ptr<chrono::ChLoadableU> mloadable) : chrono::ChLoaderUdistributed(mloadable) {
        integration_points = 10;
    };

    void set_positions(std::vector<double> positions) {
        this->positions.clear();
        this->positions.assign(positions.begin(), positions.end());
    }

    void set_loads(std::vector<chrono::ChVector<double>> loads) {
        // check that number of loads is the same as number of positions
        if (loads.size() != this->positions.size()) {
            throw std::runtime_error("Number of loads (" + std::to_string(loads.size()) +
                                     ") for element is different from number of positions (" +
                                     std::to_string(this->positions.size()) + ") along element.");
        }
        this->loads.clear();
        this->loads.assign(loads.begin(), loads.end());
    }

    // Compute F=F(u)
    virtual void ComputeF(const double U,                      // parametric coordinate along element
                          chrono::ChVectorDynamic<>& F,        // resulting loads go here
                          chrono::ChVectorDynamic<>* state_x,  // if !=0 update pos
                          chrono::ChVectorDynamic<>* state_w   // if !=0 update speed
    ) {
        // initialize load
        auto load = chrono::ChVector<double>(0.0, 0.0, 0.0);
        // find between which load positions is U and interpolate
        for (int ii = 0; ii < std::max(0, (int)positions.size() - 1); ii++) {
            if (positions[ii] <= U && U <= positions[ii + 1]) {
                double range = positions[ii + 1] - positions[ii];
                load = (1 - (U - positions[ii]) / range) * loads[ii] +
                       (1 - (positions[ii + 1] - U) / range) * loads[ii + 1];
                break;
            }
        }
        // apply load
        // forces
        F(0) = load.x();
        F(1) = load.y();
        F(2) = load.z();
        // moments
        F(3) = 0.0;
        F(4) = 0.0;
        F(5) = 0.0;
    }

    virtual int GetIntegrationPointsU() { return integration_points; }
};

}  // namespace elasto
}  // namespace seahowl
