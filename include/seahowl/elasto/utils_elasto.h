#pragma once

#include <chrono/physics/ChLoaderU.h>
#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>

#include <seahowl/elasto/reference_point_elasto.h>

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {

class RigidBody : public chrono::ChBody {
  public:
    RigidBody() : chrono::ChBody(){};
    void set_position(Vector3d position) { this->SetPos(position); };
    void set_mass(double mass) { this->SetMass(mass); };
    double get_mass() { return this->GetMass(); };
    void set_rotation(Quaternion rotation) { this->SetRot(rotation); };
    Vector3d get_position() { return this->GetPos(); };
    Vector3d get_velocity() { return this->GetPos_dt(); };
    Vector3d get_acceleration() { return this->GetPos_dtdt(); };
    Quaternion get_rotation() { return this->GetRot(); };
    Vector3d get_direction() { return this->GetRot().GetVector(); };
    Vector3d get_rotational_velocity_local() { return this->GetWvel_loc(); };
    Vector3d get_rotational_acceleration_local() { return this->GetWacc_loc(); };
};

class NodeFEA : public chrono::fea::ChNodeFEAxyzrot {
  public:
    std::shared_ptr<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric> section;

    NodeFEA(Vector3d position, Quaternion rotation)
        : chrono::fea::ChNodeFEAxyzrot(chrono::ChFrame<>(position, rotation)){};

    void set_position(Vector3d position) { this->SetPos(position); };
    void set_rotation(Quaternion rotation) { this->SetRot(rotation); };
    Vector3d get_position() { return Vector3d(this->GetPos()); };
    Vector3d get_velocity() { return Vector3d(this->GetPos_dt()); };
    Vector3d get_acceleration() { return Vector3d(this->GetPos_dtdt()); };
    Quaternion get_rotation() { return Quaternion(this->GetRot()); };
    Vector3d get_direction() { return Vector3d(this->GetRot().GetVector()); };
    Vector3d get_rotational_velocity_local() { return Vector3d(this->GetWvel_loc()); };
    Vector3d get_rotational_acceleration_local() { return Vector3d(this->GetWacc_loc()); };
    Vector3d get_load() { return Vector3d(this->GetForce()); };
    void set_properties(const BladeReferencePointElasto& ref) {
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        // offsets
        section->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
        section->SetCentroidY(ref.offset_elastic.y());
        section->SetCentroidZ(-ref.offset_elastic.x());
        // material properties
        section->SetMassPerUnitLength(ref.mass_matrix(0, 0));
        // axial
        section->SetAxialRigidity(ref.stiffness_matrix(0, 0));
        section->SetXtorsionRigidity(ref.stiffness_matrix(3, 3));
        // flap
        section->SetYbendingRigidity(ref.stiffness_matrix(4, 4));
        // edge
        section->SetZbendingRigidity(ref.stiffness_matrix(5, 5));
        // damping
        section->SetBeamRaleyghDamping(ref.damping_coefficients);
    };
};

class BladeElementFEA : public chrono::fea::ChElementBeamTaperedTimoshenko {
  public:
    BladeElementFEA() : chrono::fea::ChElementBeamTaperedTimoshenko() {
        // create blade section
        auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        this->SetTaperedSection(blade_section);
    };

    void set_nodes(std::shared_ptr<NodeFEA> node1, std::shared_ptr<NodeFEA> node2) {
        // set nodes
        this->SetNodes(node1, node2);
        // set tapered sections
        this->GetTaperedSection()->SetSectionA(node1->section);
        this->GetTaperedSection()->SetSectionB(node2->section);
    };

    void set_prebend(const Quaternion& prebend) { this->SetNodeBreferenceRot(prebend); };
};

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
