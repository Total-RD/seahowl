#include "seahowl/elasto/chrono_adapters.h"

#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/entities_elasto.h"

#include <chrono/core/ChVector3.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementCableANCF.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLoadsBody.h>
#include <chrono/physics/ChLoadContainer.h>
#include <chrono/fea/ChLinkNodeNode.h>
#include <chrono/fea/ChLinkNodeFrame.h>
#include <chrono/physics/ChLinkRevolute.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>

#include <vector>
#include <memory>
#include <spdlog/spdlog.h>
#include <typeinfo>

// default mass value for checking if ChBody mass was set.
const double MASS_NOTSET_VALUE = -1.2345e-12;

using namespace seahowl;
using namespace seahowl::elasto;

chrono::ChVector3<double> vector2ch(const Vector3d& vector_in) {
    return chrono::ChVector3<double>(vector_in[0], vector_in[1], vector_in[2]);
}
Vector3d ch2vec(const chrono::ChVector3<double>& vector_in) {
    return Vector3d(vector_in[0], vector_in[1], vector_in[2]);
}

chrono::ChQuaternion<double> quat2ch(const Quaternion& quaternion_in) {
    return chrono::ChQuaternion<double>(quaternion_in.w(), quaternion_in.x(), quaternion_in.y(), quaternion_in.z());
}

Quaternion ch2quat(const chrono::ChQuaternion<double>& quaternion_in) {
    return Quaternion(quaternion_in[0], quaternion_in[1], quaternion_in[2], quaternion_in[3]);
}

Quaternion node_ch2iec(const chrono::ChQuaternion<double>& quaternion_in) {
    // Convert from Chrono standard ro IEC standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate +90 degrees around Chrono y-axis to transform to IEC convention.
    auto angle = +PI / 2.0;
    return ch2quat(quaternion_in) * Quaternion(cos(angle / 2), 0, sin(angle / 2), 0);
}

chrono::ChQuaternion<double> node_iec2ch(const Quaternion& quaternion_in) {
    // Convert from IEC standard to Chrono standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate -90 degrees around IEC y-axis to transform to Chrono convention.
    auto angle = -PI / 2.0;
    return quat2ch(quaternion_in * Quaternion(cos(angle / 2), 0, sin(angle / 2), 0));
}

Vector3d vec_ch2iec(const chrono::ChVector3<double>& vector_in) {
    // Convert from Chrono standard ro IEC standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate +90 degrees around Chrono y-axis to transform to IEC convention.
    return Vector3d(-vector_in[2], vector_in[1], vector_in[0]);
}

Vector3d vec_iec2ch(const Vector3d& vector_in) {
    // Convert from IEC standard to Chrono standard.
    // IEC convention:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    // ==> need to rotate -90 degrees around IEC y-axis to transform to Chrono convention.
    return Vector3d(vector_in[2], vector_in[1], -vector_in[0]);
}

// convenience matrix for rotation from IEC to Chrono convention for nodes
Eigen::Matrix<double, 6, 6> get_iec2ch_rotation_matrix() {
    // IEC standard:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.

    Eigen::Matrix<double, 3, 3> rot33_iec2ch = AngleAxisd(PI / 2, Vector3d(0.0, 1.0, 0.0)).toRotationMatrix();
    Eigen::Matrix<double, 6, 6> rot66_iec2ch = Eigen::Matrix<double, 6, 6>::Zero();
    for (int ii = 0; ii < 3; ii++) {
        for (int jj = 0; jj < 3; jj++) {
            rot66_iec2ch(ii, jj) = rot33_iec2ch(ii, jj);
            rot66_iec2ch(ii + 3, jj + 3) = rot33_iec2ch(ii, jj);
        }
    }
    return rot66_iec2ch;
}
Eigen::Matrix<double, 6, 6> rot66_iec2ch = get_iec2ch_rotation_matrix();

namespace chrono {

/**
 * @brief Derived Chrono load class for using local 6x6 added mass and damping matrices.
 */
class ChLoadLocal66 : public ChLoadCustom {
  public:
    ChLoadLocal66(std::shared_ptr<ChBody> mloadable) : ChLoadCustom(mloadable) { body_frame = mloadable; }

    ChLoadLocal66(std::shared_ptr<fea::ChNodeFEAxyzrot> mloadable) : ChLoadCustom(mloadable) { body_frame = mloadable; }

    /**
     * @brief "Virtual" copy constructor (covariant return type). Required from chrono inheritance.
     */
    virtual ChLoadLocal66* Clone() const override { return new ChLoadLocal66(*this); }

    void SetAddedMassMatrix(const ChMatrixDynamic<double>& matrix) { added_mass_matrix = matrix; }
    void SetDampingMatrix(const ChMatrixDynamic<double>& matrix) { damping_matrix = matrix; }
    void AccumulateAddedMassMatrix(const ChMatrixDynamic<double>& matrix) { added_mass_matrix += matrix; }
    void AccumulateDampingMatrix(const ChMatrixDynamic<double>& matrix) { damping_matrix += matrix; }
    ChMatrixDynamic<double> GetAddedMassMatrix() const { return added_mass_matrix; }
    ChMatrixDynamic<double> GetDampingMatrix() const { return damping_matrix; }

    // compute external forces manually
    virtual void ComputeQ(ChState* state_x, ChStateDelta* state_w) override {
        // sanity check
        if (this->LoadGetNumCoordsVelLevel() != 6) {
            throw std::runtime_error("6x6 added mass matrix only works with entities with 6 DOFs.");
        }

        // matrices expressed in local system --> transformation needed
        // Chrono sends:
        // - translation components in global system
        // - rotation components in local system
        Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
        ChMatrix33<> rot33(body_frame->GetRot());
        Eigen::Matrix<double, 3, 3> rotI = Eigen::Matrix<double, 3, 3>::Identity();
        rot66.block<3, 3>(0, 0) = rot33.block(0, 0, 3, 3);
        rot66.block<3, 3>(3, 3) = rotI.block(0, 0, 3, 3);

        // reset load_Q
        load_Q = ChVectorDynamic<>(this->LoadGetNumCoordsVelLevel()).setZero();

        // damping force
        auto vv = body_frame->GetPosDt();
        auto rv = body_frame->GetAngVelLocal();
        ChVectorDynamic<> vvv(this->LoadGetNumCoordsVelLevel());
        for (int ii = 0; ii < 3; ii++) {
            vvv[ii] = vv[ii];
            vvv[ii + 3] = rv[ii];
        }
        load_Q += -rot66 * (damping_matrix * (rot66.inverse() * vvv));
    };

    // compute jacobians manually
    virtual void ComputeJacobian(ChState* state_x, ChStateDelta* state_w) override {
        // matrices expressed in local system --> transformation needed
        // Chrono sends:
        // - translation components in global system
        // - rotation components in local system
        Eigen::Matrix<double, 6, 6> rot66 = Eigen::Matrix<double, 6, 6>::Zero();
        ChMatrix33<> rot33(body_frame->GetRot());
        Eigen::Matrix<double, 3, 3> rotI = Eigen::Matrix<double, 3, 3>::Identity();
        rot66.block<3, 3>(0, 0) = rot33.block(0, 0, 3, 3);
        rot66.block<3, 3>(3, 3) = rotI.block(0, 0, 3, 3);

        // mass matrix(6x6)
        m_jacobians->M = rot66 * (added_mass_matrix * rot66.inverse());

        // damping matrix terms (6x6)
        m_jacobians->R = rot66 * (damping_matrix * rot66.inverse());

        // stiffness matrix terms (6x6) - keeping it to zero here
        m_jacobians->K = Eigen::Matrix<double, 6, 6>::Zero();
    };

    virtual void LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override {
        if (!this->m_jacobians)
            return;
        // fetch w as a contiguous vector
        ChVectorDynamic<> grouped_w(LoadGetNumCoordsVelLevel());
        ChVectorDynamic<> grouped_cMv(LoadGetNumCoordsVelLevel());
        unsigned int rowQ = 0;
        for (int i = 0; i < loadable->GetNumSubBlocks(); ++i) {
            if (loadable->IsSubBlockActive(i)) {
                unsigned int moffset = loadable->GetSubBlockOffset(i);

                for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                    grouped_w(rowQ) = w(row + moffset);
                    ++rowQ;
                }
            }
        }

        // do computation R=c*M*v
        grouped_cMv = c * m_jacobians->M * grouped_w;

        rowQ = 0;
        for (int i = 0; i < loadable->GetNumSubBlocks(); ++i) {
            if (loadable->IsSubBlockActive(i)) {
                unsigned int moffset = loadable->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                    R(row + moffset) += grouped_cMv(rowQ);
                    ++rowQ;
                }
            }
        }
    }

    virtual bool IsStiff() override { return true; }  // this to force the use of the inertial M, R and K matrices

  private:
    ChMatrixDynamic<double> added_mass_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    ChMatrixDynamic<double> damping_matrix = Eigen::Matrix<double, 6, 6>::Zero();

    std::shared_ptr<ChBodyFrame> body_frame;
};

/**
 * @brief Derived Chrono load class for force (global frame) and torque (local frame).
 */
class ChLoadForceTorque : public ChLoadCustom {
  public:
    ChLoadForceTorque(std::shared_ptr<ChLoadable> mloadable) : ChLoadCustom(mloadable) {
        chload_force = ChVector3<double>(0.0, 0.0, 0.0);
        chload_torque = ChVector3<double>(0.0, 0.0, 0.0);
    }

    /**
     * @brief "Virtual" copy constructor (covariant return type). Required from chrono inheritance.
     */
    virtual ChLoadForceTorque* Clone() const override { return new ChLoadForceTorque(*this); }

    void SetForce(const ChVector3<double>& force) { chload_force = force; }
    void SetTorque(const ChVector3<double>& torque) { chload_torque = torque; }
    ChVector3<double> GetForce() { return chload_force; }
    ChVector3<double> GetTorque() { return chload_torque; }

    // compute external forces manually
    virtual void ComputeQ(ChState* state_x, ChStateDelta* state_w) override {
        // reset load_Q
        int ndof = this->LoadGetNumCoordsVelLevel();
        load_Q = ChVectorDynamic<>(ndof).setZero();
        load_Q[0] = chload_force[0];
        load_Q[1] = chload_force[1];
        load_Q[2] = chload_force[2];
        if (ndof > 3) {
            load_Q[3] = chload_torque[0];
            load_Q[4] = chload_torque[1];
            load_Q[5] = chload_torque[2];
        }
    };

    virtual bool IsStiff() override { return false; }

  private:
    ChVector3<double> chload_force;
    ChVector3<double> chload_torque;
};

}  // namespace chrono

void EntityDynamicChrono::set_position(const Vector3d& position) {
    chobj->SetPos(vector2ch(position));
}

Vector3d EntityDynamicChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

void EntityDynamicChrono::set_rotation(const Quaternion& rotation) {
    chobj->SetRot(quat2ch(rotation));
}

Quaternion EntityDynamicChrono::get_rotation() const {
    return ch2quat(chobj->GetRot());
}

void EntityDynamicChrono::set_velocity(const Vector3d& velocity) {
    chobj->SetPosDt(vector2ch(velocity));
}

Vector3d EntityDynamicChrono::get_velocity() const {
    return ch2vec(chobj->GetPosDt());
}

void EntityDynamicChrono::set_acceleration(const Vector3d& acceleration) {
    chobj->SetPosDt2(vector2ch(acceleration));
}

Vector3d EntityDynamicChrono::get_acceleration() const {
    return ch2vec(chobj->GetPosDt2());
}

void EntityDynamicChrono::set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local) {
    if (is_local) {
        chobj->SetAngVelLocal(vector2ch(rotational_velocity));
    } else {
        chobj->SetAngVelParent(vector2ch(rotational_velocity));
    }
}

Vector3d EntityDynamicChrono::get_rotational_velocity(bool is_local) const {
    if (is_local) {
        return ch2vec(chobj->GetAngVelLocal());
    } else {
        return ch2vec(chobj->GetAngVelParent());
    }
}

void EntityDynamicChrono::set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local) {
    if (is_local) {
        chobj->SetAngAccLocal(vector2ch(rotational_acceleration));
    } else {
        chobj->SetAngAccParent(vector2ch(rotational_acceleration));
    }
}

Vector3d EntityDynamicChrono::get_rotational_acceleration(bool is_local) const {
    if (is_local) {
        return ch2vec(chobj->GetAngAccLocal());
    } else {
        return ch2vec(chobj->GetAngAccParent());
    }
}

BodyElastoChrono::BodyElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChBody>();
    chloadcontainer = chrono_types::make_shared<chrono::ChLoadContainer>();
    EntityDynamicChrono::chobj = chobj;
    set_mass(MASS_NOTSET_VALUE);
    set_inertia_diagonal(Vector3d(MASS_NOTSET_VALUE, MASS_NOTSET_VALUE, MASS_NOTSET_VALUE));
    chloads_internals = chrono_types::make_shared<chrono::ChLoadForceTorque>(chobj);
    chloadcontainer->Add(chloads_internals);
}

void BodyElastoChrono::set_mass(double mass) {
    chobj->SetMass(mass);
}

void BodyElastoChrono::set_inertia_diagonal(const Vector3d& inertia) {
    chobj->SetInertiaXX(vector2ch(inertia));
}

void BodyElastoChrono::set_inertia_matrix(const Eigen::Matrix<double, 3, 3>& inertia) {
    chobj->SetInertia(inertia);
};

Eigen::Matrix<double, 3, 3> BodyElastoChrono::get_inertia_matrix() const {
    return chobj->GetInertia();
}

void BodyElastoChrono::reset_loads() {
    chobj->EmptyAccumulators();
}

void BodyElastoChrono::reset_loads_internals() {
    chloads_internals->SetForce(chrono::ChVector3(0.0, 0.0, 0.0));
    chloads_internals->SetTorque(chrono::ChVector3(0.0, 0.0, 0.0));
}

Vector3d BodyElastoChrono::get_force(bool is_local) const {
    if (is_local) {
        return get_rotation().inverse() * ch2vec(chobj->GetAccumulatedForce());
    } else {
        return ch2vec(chobj->GetAccumulatedForce());
    }
}

Vector3d BodyElastoChrono::get_force_internals(bool is_local) const {
    if (is_local) {
        return get_rotation().inverse() * ch2vec(chloads_internals->GetForce());
    } else {
        return ch2vec(chloads_internals->GetForce());
    }
}

Vector3d BodyElastoChrono::get_torque(bool is_local) const {
    if (is_local) {
        return ch2vec(chobj->GetAccumulatedTorque());
    } else {
        return get_rotation() * ch2vec(chobj->GetAccumulatedTorque());
    }
}

Vector3d BodyElastoChrono::get_torque_internals(bool is_local) const {
    if (is_local) {
        return ch2vec(chloads_internals->GetTorque());
    } else {
        return get_rotation() * ch2vec(chloads_internals->GetTorque());
    }
}

void BodyElastoChrono::set_force(const Vector3d& force, bool is_local) {
    auto torque = get_torque(true);   // get previously accumulated torque
    chobj->EmptyAccumulators();       // empty accumulated forces and torques
    accumulate_torque(torque, true);  // set previously accumulated torque
    accumulate_force(force, is_local);
}

void BodyElastoChrono::set_torque(const Vector3d& torque, bool is_local) {
    auto force = get_force(false);   // get previously accumulated force
    chobj->EmptyAccumulators();      // empty accumulated forces and torques
    accumulate_force(force, false);  // set previously accumulated force
    accumulate_torque(torque, is_local);
}

void BodyElastoChrono::accumulate_force(const Vector3d& force, bool is_local) {
    if (is_local) {
        chobj->AccumulateForce(force, Vector3d(0.0, 0.0, 0.0), is_local);
    } else {
        chobj->AccumulateForce(force, chobj->GetPos(), is_local);
    }
}

void BodyElastoChrono::accumulate_force_internals(const Vector3d& force, bool is_local) {
    // send internal force in absolute frame
    if (is_local) {
        chloads_internals->SetForce(chloads_internals->GetForce() + get_rotation() * force);
    } else {
        chloads_internals->SetForce(chloads_internals->GetForce() + force);
    }
}

void BodyElastoChrono::accumulate_torque(const Vector3d& torque, bool is_local) {
    chobj->AccumulateTorque(torque, is_local);
}

void BodyElastoChrono::accumulate_torque_internals(const Vector3d& torque, bool is_local) {
    // send internal torque in relative frame
    if (is_local) {
        chloads_internals->SetTorque(chloads_internals->GetTorque() + torque);
    } else {
        chloads_internals->SetTorque(chloads_internals->GetTorque() + get_rotation().inverse() * torque);
    }
}

void BodyElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

bool BodyElastoChrono::is_fixed() const {
    return chobj->IsFixed();
}

double BodyElastoChrono::get_mass() {
    return chobj->GetMass();
}

void BodyElastoChrono::set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    chload66->SetAddedMassMatrix(matrix);
}

Eigen::Matrix<double, 6, 6> BodyElastoChrono::get_added_mass_matrix() const {
    if (!chload66) {
        throw std::runtime_error("Cannot get added mass matrix for " + std::string(typeid(*this).name()) +
                                 ", it was not set.");
    }
    return chload66->GetAddedMassMatrix();
}

void BodyElastoChrono::accumulate_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    chload66->AccumulateAddedMassMatrix(matrix);
}

void BodyElastoChrono::set_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    chload66->SetDampingMatrix(matrix);
}

void BodyElastoChrono::accumulate_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    chload66->AccumulateDampingMatrix(matrix);
}

Eigen::Matrix<double, 6, 6> BodyElastoChrono::get_damping_matrix() const {
    if (!chload66) {
        throw std::runtime_error("Cannot get damping matrix for " + std::string(typeid(*this).name()) +
                                 ", it was not set.");
    }
    return chload66->GetDampingMatrix();
}

NodeElastoChronoBase::NodeElastoChronoBase() {
    chloadcontainer = chrono_types::make_shared<chrono::ChLoadContainer>();
}

NodeElastoChrono::NodeElastoChrono(const Vector3d& position, const Quaternion& rotation) {
    chobj = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(
        chrono::ChFrame<>(vector2ch(position), node_iec2ch(rotation)));
    EntityDynamicChrono::chobj = chobj;
    NodeElastoChronoBase::chobj = chobj;

    chloads_internals = chrono_types::make_shared<chrono::ChLoadForceTorque>(chobj);
    chloadcontainer->Add(chloads_internals);
}

void NodeElastoChrono::set_rotation(const Quaternion& rotation) {
    chobj->SetRot(node_iec2ch(rotation));
}

Quaternion NodeElastoChrono::get_rotation() const {
    return node_ch2iec(chobj->GetRot());
}

Vector3d NodeElastoChrono::get_direction() const {
    return ch2vec(chobj->TransformDirectionLocalToParent(chrono::ChVector3<double>(1.0, 0.0, 0.0)));
}

void NodeElastoChrono::reset_loads() {
    set_force(Vector3d(0.0, 0.0, 0.0), false);
    set_torque(Vector3d(0.0, 0.0, 0.0), true);
}

void NodeElastoChrono::reset_loads_internals() {
    chloads_internals->SetForce(chrono::ChVector3(0.0, 0.0, 0.0));
    chloads_internals->SetTorque(chrono::ChVector3(0.0, 0.0, 0.0));
}

Vector3d NodeElastoChrono::get_force(bool is_local) const {
    if (is_local) {
        return get_rotation().inverse() * ch2vec(chobj->GetForce());
    } else {
        return ch2vec(chobj->GetForce());
    }
}

Vector3d NodeElastoChrono::get_force_internals(bool is_local) const {
    if (is_local) {
        return get_rotation().inverse() * ch2vec(chloads_internals->GetForce());
    } else {
        return ch2vec(chloads_internals->GetForce());
    }
}

Vector3d NodeElastoChrono::get_torque(bool is_local) const {
    if (is_local) {
        return vec_ch2iec(chobj->GetTorque());
    } else {
        return get_rotation() * vec_ch2iec(chobj->GetTorque());
    }
}

Vector3d NodeElastoChrono::get_torque_internals(bool is_local) const {
    if (is_local) {
        return vec_ch2iec(chloads_internals->GetTorque());
    } else {
        return get_rotation() * vec_ch2iec(chloads_internals->GetTorque());
    }
}

void NodeElastoChrono::set_force(const Vector3d& force, bool is_local) {
    if (is_local) {
        chobj->SetForce(vector2ch(get_rotation() * force));
    } else {
        chobj->SetForce(vector2ch(force));
    }
}

void NodeElastoChrono::set_torque(const Vector3d& torque, bool is_local) {
    if (is_local) {
        chobj->SetTorque(vec_iec2ch(torque));
    } else {
        chobj->SetTorque(vec_iec2ch(get_rotation().inverse() * torque));
    }
}

void NodeElastoChrono::accumulate_force(const Vector3d& force, bool is_local) {
    set_force(get_force(is_local) + force, is_local);
}

void NodeElastoChrono::accumulate_force_internals(const Vector3d& force, bool is_local) {
    // send internal force in absolute frame
    if (is_local) {
        chloads_internals->SetForce(chloads_internals->GetForce() + get_rotation() * force);
    } else {
        chloads_internals->SetForce(chloads_internals->GetForce() + force);
    }
}

void NodeElastoChrono::accumulate_torque(const Vector3d& torque, bool is_local) {
    set_torque(get_torque(is_local) + torque, is_local);
}

void NodeElastoChrono::accumulate_torque_internals(const Vector3d& torque, bool is_local) {
    // send internal torque in relative frame, Chrono convention
    if (is_local) {
        chloads_internals->SetTorque(chloads_internals->GetTorque() + vec_iec2ch(torque));
    } else {
        chloads_internals->SetTorque(chloads_internals->GetTorque() + vec_iec2ch(get_rotation().inverse() * torque));
    }
}

void NodeElastoChrono::set_mass(double mass) {
    chobj->SetMass(mass);
}

double NodeElastoChrono::get_mass() {
    return chobj->GetMass();
}

void NodeElastoChrono::set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    // Convert from IEC convention to Chrono convention.
    auto mm = rot66_iec2ch * (matrix * rot66_iec2ch.transpose());
    chload66->SetAddedMassMatrix(mm);
}

void NodeElastoChrono::accumulate_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    // Convert from IEC convention to Chrono convention.
    auto mm = rot66_iec2ch * (matrix * rot66_iec2ch.transpose());
    chload66->AccumulateAddedMassMatrix(mm);
}

Eigen::Matrix<double, 6, 6> NodeElastoChrono::get_added_mass_matrix() const {
    if (!chload66) {
        throw std::runtime_error("Cannot get added mass matrix for " + std::string(typeid(*this).name()) +
                                 ", it was not set.");
    }
    return rot66_iec2ch.inverse() * chload66->GetAddedMassMatrix() * rot66_iec2ch;
}

void NodeElastoChrono::set_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    auto mm = rot66_iec2ch * matrix * rot66_iec2ch.inverse();
    chload66->SetDampingMatrix(mm);
}

void NodeElastoChrono::accumulate_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    if (!chload66) {
        chload66 = std::make_shared<chrono::ChLoadLocal66>(chobj);
        chloadcontainer->Add(chload66);
    }
    auto mm = rot66_iec2ch * matrix * rot66_iec2ch.inverse();
    chload66->AccumulateDampingMatrix(mm);
}

Eigen::Matrix<double, 6, 6> NodeElastoChrono::get_damping_matrix() const {
    if (!chload66) {
        throw std::runtime_error("Cannot get damping matrix for " + std::string(typeid(*this).name()) +
                                 ", it was not set.");
    }
    return rot66_iec2ch.inverse() * chload66->GetDampingMatrix() * rot66_iec2ch;
}

void NodeElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

bool NodeElastoChrono::is_fixed() const {
    return chobj->IsFixed();
}

void NodeElastoChrono::set_properties(const BladeReferencePointElasto& ref, bool fpm) {
    // Convert from IEC convention to Chrono convention.
    auto mm = rot66_iec2ch * (ref.mass_matrix * rot66_iec2ch.transpose());
    auto sm = rot66_iec2ch * (ref.stiffness_matrix * rot66_iec2ch.transpose());

    // damping
    chrono::fea::DampingCoefficients damping_coefficients;
    // damping coefficients: IEC -> Chrono convention
    // Timoshenko beams in Chrono use Rayleigh cofficients squared
    damping_coefficients.bx = sqrt(ref.damping_axial);
    damping_coefficients.by = sqrt(ref.damping_edgewise);
    damping_coefficients.bz = sqrt(ref.damping_flapwise);
    damping_coefficients.bt = sqrt(ref.damping_torsion);
    // mass-proportional coefficient
    damping_coefficients.alpha = ref.damping_mass;

    if (fpm == true) {
        auto sectionFPM = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>();
        section = sectionFPM;
        // offsets
        sectionFPM->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
        sectionFPM->SetCentroidY(ref.offset_elastic.y());
        sectionFPM->SetCentroidZ(-ref.offset_elastic.x());
        // material properties
        sectionFPM->SetMassMatrixFPM(mm);
        sectionFPM->SetStiffnessMatrixFPM(sm);
        // damping
        sectionFPM->SetRayleighDamping(damping_coefficients);
    } else {
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        // offsets
        section->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
        section->SetCentroidY(ref.offset_elastic.y());
        section->SetCentroidZ(-ref.offset_elastic.x());
        // material properties
        // see ChBeamSectionTimoshenkoAdvancedGenericFPM methods: SetMassMatrixFPM, SetStiffnessMatrixFPM
        section->SetMassPerUnitLength(mm(0, 0));
        // inertias per unit length in this order: mIyy, mIzz, mIyz, mQy, mQz
        section->SetInertiasPerUnitLength(mm(4, 4), mm(5, 5), -mm(4, 5), mm(0, 4), -mm(0, 5));
        // axial
        section->SetTorsionRigidityX(sm(3, 3));
        section->SetAxialRigidity(sm(0, 0));
        // flap
        section->SetBendingRigidityY(sm(4, 4));
        section->SetShearRigidityY(sm(1, 1));
        // edge
        section->SetBendingRigidityZ(sm(5, 5));
        section->SetShearRigidityZ(sm(2, 2));
        // damping=
        section->SetRayleighDamping(damping_coefficients);
    }
}

void NodeElastoChrono::set_properties(const TowerReferencePointElasto& ref) {
    // make first section for tapered section
    section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    // material properties
    section->SetMassPerUnitLength(ref.density);
    section->SetInertiasPerUnitLength(ref.inertia_foreaft, ref.inertia_sideside, 0.0, 0.0, 0.0);
    // axial
    section->SetAxialRigidity(ref.stiffness_axial);
    section->SetTorsionRigidityX(ref.stiffness_torsion);
    // foreaft
    section->SetBendingRigidityY(ref.stiffness_foreaft);
    section->SetShearRigidityZ(ref.stiffness_foreaft_shear);
    // sideside
    section->SetBendingRigidityZ(ref.stiffness_sideside);
    section->SetShearRigidityY(ref.stiffness_sideside_shear);
    chrono::fea::DampingCoefficients damping_coefficients;
    // damping coefficients: IEC -> Chrono convention
    // Timoshenko beams in Chrono use Rayleigh cofficients squared
    damping_coefficients.bx = sqrt(ref.damping_axial);
    damping_coefficients.by = sqrt(ref.damping_sideside);
    damping_coefficients.bz = sqrt(ref.damping_foreaft);
    damping_coefficients.bt = sqrt(ref.damping_torsion);
    // mass-proportional coefficient
    damping_coefficients.alpha = ref.damping_mass;
    section->SetRayleighDamping(damping_coefficients);
}

NodeElastoChronoD::NodeElastoChronoD(const Vector3d& position, const Vector3d& direction) {
    chobj = std::make_shared<chrono::fea::ChNodeFEAxyzD>(vector2ch(position), vector2ch(direction));
    NodeElastoChronoBase::chobj = chobj;

    chloads_internals = chrono_types::make_shared<chrono::ChLoadForceTorque>(chobj);
    chloadcontainer->Add(chloads_internals);
}

void NodeElastoChronoD::set_rotation(const Quaternion& rotation) {
    // set rotation assuming direction of node to be local Z
    chobj->SetSlope1(vector2ch(rotation * Vector3d(0.0, 0.0, 1.0)));
}

Quaternion NodeElastoChronoD::get_rotation() const {
    // get rotation assuming direction of node to be local Z
    // note: ChNodeFEAxyzD does not have a true rotation, only a direction
    return Quaternion::FromTwoVectors(Vector3d(0.0, 0.0, 1.0), get_direction());
}

Vector3d NodeElastoChronoD::get_direction() const {
    return ch2vec(chobj->GetSlope1());
}

void NodeElastoChronoD::reset_loads() {
    set_force(Vector3d(0.0, 0.0, 0.0), false);
    set_torque(Vector3d(0.0, 0.0, 0.0), true);
}

void NodeElastoChronoD::reset_loads_internals() {
    chloads_internals->SetForce(chrono::ChVector3(0.0, 0.0, 0.0));
    chloads_internals->SetTorque(chrono::ChVector3(0.0, 0.0, 0.0));
}

Vector3d NodeElastoChronoD::get_force(bool is_local) const {
    if (is_local) {
        throw std::runtime_error("Cannot get force locally from ChNodeFEAxyzD.");
    } else {
        return ch2vec(chobj->GetForce());
    }
}

Vector3d NodeElastoChronoD::get_force_internals(bool is_local) const {
    if (is_local) {
        throw std::runtime_error("Cannot get force locally from ChNodeFEAxyzD.");
    } else {
        return ch2vec(chloads_internals->GetForce());
    }
}

Vector3d NodeElastoChronoD::get_torque(bool is_local) const {
    // no torque on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
}

Vector3d NodeElastoChronoD::get_torque_internals(bool is_local) const {
    // no torque on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
}

void NodeElastoChronoD::set_force(const Vector3d& force, bool is_local) {
    if (is_local) {
        throw std::runtime_error("Cannot set force locally for ChNodeFEAxyzD.");
    } else {
        chobj->SetForce(vector2ch(force));
    }
}

void NodeElastoChronoD::set_torque(const Vector3d& torque, bool is_local) {
    // no torque on ChNodeFEAxyzD
}

void NodeElastoChronoD::accumulate_force(const Vector3d& force, bool is_local) {
    set_force(get_force(is_local) + force, is_local);
}

void NodeElastoChronoD::accumulate_force_internals(const Vector3d& force, bool is_local) {
    // send internal loads in absolute frame
    if (is_local) {
        chloads_internals->SetForce(chloads_internals->GetForce() + get_rotation() * force);
    } else {
        chloads_internals->SetForce(chloads_internals->GetForce() + force);
    }
}

void NodeElastoChronoD::accumulate_torque(const Vector3d& torque, bool is_local) {
    // no torque on ChNodeFEAxyzD
}

void NodeElastoChronoD::accumulate_torque_internals(const Vector3d& torque, bool is_local) {
    // no torque on ChNodeFEAxyzD
}

void NodeElastoChronoD::set_mass(double mass) {
    chobj->SetMass(mass);
}

double NodeElastoChronoD::get_mass() {
    return chobj->GetMass();
}

void NodeElastoChronoD::set_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    throw std::runtime_error("Cannot set added mass matrix for " + std::string(typeid(*this).name()) +
                             ", not implemented for cable nodes.");
}

void NodeElastoChronoD::accumulate_added_mass_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    throw std::runtime_error("Cannot set added mass matrix for " + std::string(typeid(*this).name()) +
                             ", not implemented for cable nodes.");
}

Eigen::Matrix<double, 6, 6> NodeElastoChronoD::get_added_mass_matrix() const {
    throw std::runtime_error("Cannot get added mass matrix for " + std::string(typeid(*this).name()) +
                             ", it was not set.");
}

void NodeElastoChronoD::set_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    throw std::runtime_error("Cannot set damping matrix for " + std::string(typeid(*this).name()) +
                             ", not implemented for cable nodes.");
}

void NodeElastoChronoD::accumulate_damping_matrix(const Eigen::Matrix<double, 6, 6>& matrix) {
    throw std::runtime_error("Cannot set damping matrix for " + std::string(typeid(*this).name()) +
                             ", not implemented for cable nodes.");
}

Eigen::Matrix<double, 6, 6> NodeElastoChronoD::get_damping_matrix() const {
    throw std::runtime_error("Cannot get damping matrix for " + std::string(typeid(*this).name()) +
                             ", it was not set.");
}

void NodeElastoChronoD::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

bool NodeElastoChronoD::is_fixed() const {
    return chobj->IsFixed();
}

void NodeElastoChronoD::set_position(const Vector3d& position) {
    chobj->SetPos(vector2ch(position));
}

Vector3d NodeElastoChronoD::get_position() const {
    return ch2vec(chobj->GetPos());
}

void NodeElastoChronoD::set_velocity(const Vector3d& velocity) {
    chobj->SetPosDt(vector2ch(velocity));
}

Vector3d NodeElastoChronoD::get_velocity() const {
    return ch2vec(chobj->GetPosDt());
}

void NodeElastoChronoD::set_acceleration(const Vector3d& acceleration) {
    chobj->SetPosDt2(vector2ch(acceleration));
}

Vector3d NodeElastoChronoD::get_acceleration() const {
    return ch2vec(chobj->GetPosDt2());
}

void NodeElastoChronoD::set_rotational_velocity(const Vector3d& rotational_velocity, bool is_local) {
    // no rotational velocity on ChNodeFEAxyzD
}

Vector3d NodeElastoChronoD::get_rotational_velocity(bool is_local) const {
    // no rotational velocity on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
}

void NodeElastoChronoD::set_rotational_acceleration(const Vector3d& rotational_acceleration, bool is_local) {
    // no rotational acceleration on ChNodeFEAxyzD
}

Vector3d NodeElastoChronoD::get_rotational_acceleration(bool is_local) const {
    // no rotational acceleration on ChNodeFEAxyzD
    return Vector3d(0.0, 0.0, 0.0);
}

void ElementElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);
}

void ElementElastoChrono::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) const {
    auto chvec = vector2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = node_ch2iec(chquat);
}

void ElementElastoChrono::evaluate_force_torque(double eta, Vector3d& force, Vector3d& torque) const {
    auto chforce = vector2ch(force);
    auto chtorque = vector2ch(torque);
    chobj->EvaluateSectionForceTorque(eta, chforce, chtorque);
    // convert Chrono convention to IEC
    force = vec_ch2iec(chforce);
    torque = vec_ch2iec(chtorque);
}

double ElementElastoChrono::get_mass() {
    return chobj->GetMass();
}

ElementBladeElastoChrono::ElementBladeElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
    ElementElastoChrono::chobj = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
    chobj->SetTaperedSection(blade_section);
}

void ElementElastoChrono::update_properties() {}

void ElementBladeElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChrono>(node2)->chobj);
    // set tapered sections
    chobj->GetTaperedSection()->SetSectionA(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->section);
    chobj->GetTaperedSection()->SetSectionB(std::dynamic_pointer_cast<NodeElastoChrono>(node2)->section);
}

void ElementBladeElastoChrono::set_prebend(const Quaternion& prebend) {
    // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
    auto prebend_ch = chrono::ChQuaternion<double>(prebend.w(), prebend.z(), prebend.y(), prebend.x());
    chobj->SetNodeBreferenceRot(prebend_ch);
}

void ElementBladeElastoChrono::update_properties() {
    chobj->GetTaperedSection()->ComputeAverageSectionParameters();
}

ElementBladeElastoChronoFPM::ElementBladeElastoChronoFPM() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenkoFPM>();
    ElementElastoChrono::chobj = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    chobj->SetTaperedSection(blade_section);
}

void ElementBladeElastoChronoFPM::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChrono>(node2)->chobj);
    // set tapered sections
    chobj->GetTaperedSection()->SetSectionA(
        std::dynamic_pointer_cast<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>(
            std::dynamic_pointer_cast<NodeElastoChrono>(node1)->section));
    chobj->GetTaperedSection()->SetSectionB(
        std::dynamic_pointer_cast<chrono::fea::ChBeamSectionTimoshenkoAdvancedGenericFPM>(
            std::dynamic_pointer_cast<NodeElastoChrono>(node2)->section));
}

void ElementBladeElastoChronoFPM::set_prebend(const Quaternion& prebend) {
    // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
    auto prebend_ch = chrono::ChQuaternion<double>(prebend.w(), prebend.z(), prebend.y(), prebend.x());
    chobj->SetNodeBreferenceRot(prebend_ch);
}

void ElementBladeElastoChronoFPM::update_properties() {
    chobj->GetTaperedSection()->ComputeAverageSectionParameters();
}

ElementMooringElastoChrono::ElementMooringElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementCableANCF>();
    ElementElastoChrono::chobj = chobj;
}

void ElementMooringElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes.clear();
    nodes.push_back(node1);
    nodes.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChronoD>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChronoD>(node2)->chobj);
}

void ElementMooringElastoChrono::set_properties(double density,
                                                double diameter,
                                                double stiffness_axial,
                                                double stiffness_bending) {
    // create mooring section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionCable>();
    chobj->SetSection(section);
    section->SetDensity(density);
    double area = chrono::CH_PI * pow(diameter, 2) / 4.0;
    section->SetDiameter(diameter);
    double young_modulus = stiffness_axial / area;
    section->SetYoungModulus(young_modulus);
    section->SetInertia(stiffness_bending / young_modulus);
}

void ElementMooringElastoChrono::set_rest_length(double rest_length) {
    chobj->SetRestLength(rest_length);
}

double ElementMooringElastoChrono::get_rest_length() const {
    return chobj->GetRestLength();
}

SpringLinearChrono::SpringLinearChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkTSDA>();
}

void SpringLinearChrono::initialize(const BodyElasto& body1, const BodyElasto& body2) {
    chobj->Initialize(dynamic_cast<const BodyElastoChrono&>(body1).chobj,
                      dynamic_cast<const BodyElastoChrono&>(body2).chobj, true,
                      chrono::ChVector3<double>(0.0, 0.0, 0.0), chrono::ChVector3<double>(0.0, 0.0, 0.0));
}

void SpringLinearChrono::initialize_with_anchors(const BodyElasto& body1,
                                                 const BodyElasto& body2,
                                                 bool local,
                                                 const Vector3d& anchor1,
                                                 const Vector3d& anchor2) {
    chobj->Initialize(dynamic_cast<const BodyElastoChrono&>(body1).chobj,
                      dynamic_cast<const BodyElastoChrono&>(body2).chobj, local, vector2ch(anchor1),
                      vector2ch(anchor2));
}

void SpringLinearChrono::set_rest_length(double rest_length) {
    chobj->SetRestLength(rest_length);
}

void SpringLinearChrono::set_spring_coefficient(double spring_coefficient) {
    chobj->SetSpringCoefficient(spring_coefficient);
}

void SpringLinearChrono::set_damping_coefficient(double damping_coefficient) {
    chobj->SetDampingCoefficient(damping_coefficient);
}

double SpringLinearChrono::get_force() {
    return chobj->GetForce();
}

LinkChrono::LinkChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
    chobj->SetConstrainedCoords(true, true, true, true, true, true);
    LinkChronoBase::chobj = chobj;
}

void LinkChrono::initialize(const Entity& entity1, const Entity& entity2) {
    try {
        chobj->Initialize(dynamic_cast<const EntityDynamicChrono&>(entity1).chobj,
                          dynamic_cast<const EntityDynamicChrono&>(entity2).chobj,
                          *dynamic_cast<const EntityDynamicChrono&>(entity2).chobj);
    } catch (const std::bad_cast& e) {
        throw std::runtime_error("Cannot link these entities.");
    }
}

void LinkChrono::set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) {
    chobj->SetConstrainedCoords(surge, sway, heave, roll, pitch, yaw);
}

Vector3d LinkChrono::get_reaction_force() const {
    auto wrench = chobj->GetReaction2();
    return ch2vec(wrench.force);
}

Vector3d LinkChrono::get_reaction_torque() const {
    auto wrench = chobj->GetReaction2();
    return ch2vec(wrench.torque);
}

LinkChronoCable::LinkChronoCable() {}

void LinkChronoCable::initialize(const Entity& entity1, const Entity& entity2) {
    try {
        auto link = chrono_types::make_shared<chrono::fea::ChLinkNodeFrame>();
        auto node = dynamic_cast<const NodeElastoChronoD&>(entity1);
        auto body = dynamic_cast<const BodyElastoChrono&>(entity2);
        link->Initialize(node.chobj, body.chobj);
        chobj = link;
        LinkChronoBase::chobj = chobj;
        return;
    } catch (const std::bad_cast& e) {
    }
    try {
        auto link = chrono_types::make_shared<chrono::fea::ChLinkNodeFrame>();
        auto body = dynamic_cast<const BodyElastoChrono&>(entity1);
        auto node = dynamic_cast<const NodeElastoChronoD&>(entity2);
        link->Initialize(node.chobj, body.chobj);
        chobj = link;
        LinkChronoBase::chobj = chobj;
        return;
    } catch (const std::bad_cast& e) {
    }
    try {
        auto link = chrono_types::make_shared<chrono::fea::ChLinkNodeNode>();
        auto node1 = dynamic_cast<const NodeElastoChronoD&>(entity1);
        auto node2 = dynamic_cast<const NodeElastoChronoD&>(entity2);
        link->Initialize(node1.chobj, node2.chobj);
        chobj = link;
        LinkChronoBase::chobj = chobj;
        return;
    } catch (const std::bad_cast& e) {
    }
    throw std::runtime_error("Cannot link these entities with cable link.");
}

void LinkChronoCable::set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) {
    if (surge != false || sway != false || heave != false || roll != true || pitch != true || yaw != true) {
        throw std::runtime_error("Cable links can only have spherical joint constraints.");
    }
}

Vector3d LinkChronoCable::get_reaction_force() const {
    auto wrench = chobj->GetReaction2();
    return ch2vec(wrench.force);
}

Vector3d LinkChronoCable::get_reaction_torque() const {
    auto wrench = chobj->GetReaction2();
    return ch2vec(wrench.torque);
}

class ChFunctionArray : public chrono::ChFunction {
  public:
    std::vector<double> time_array{0.0, 0.0};
    std::vector<double> values_array{0.0, 0.0};

    virtual ChFunctionArray* Clone() const override { return new ChFunctionArray(*this); }

    virtual double GetVal(double x) const override {
        if (x >= time_array.back()) {
            return values_array.back();
        } else if (x <= time_array.front()) {
            return values_array.front();
        } else {
            for (int ii = 0; ii < time_array.size() - 1; ii++) {
                if (time_array[ii] <= x && time_array[ii + 1] >= x) {
                    auto t1 = time_array[ii];
                    auto v1 = values_array[ii];
                    auto t2 = time_array[ii + 1];
                    auto v2 = values_array[ii + 1];
                    return (v1 + (v2 - v1) * (x - t1) / (t2 - t1));
                }
            }
            // throw error if value not found
            throw std::runtime_error("Cannot find value from time array in ChFunctionArray.");
        }
    }
};

ActuatorRotationChrono::ActuatorRotationChrono() {
    // make actuator bodies (massless)
    body_worker = std::make_unique<BodyElastoChrono>();
    body_controller = std::make_unique<BodyElastoChrono>();

    // make actuator abject
    chobj = std::make_shared<chrono::ChLinkMotorRotationAngle>();
    LinkChronoBase::chobj = chobj;
    chfunc = std::make_shared<ChFunctionArray>();
    chobj->SetMotorFunction(chfunc);

    // make link
    link = std::make_unique<LinkChrono>();

    // set defaults
    reset();

    // unfix actuator at initialization
    set_fixed_actuator(false);
}

void ActuatorRotationChrono::reset() {
    // body worker
    body_worker->set_position(Vector3d(0.0, 0.0, 0.0));
    body_worker->set_rotation(Quaternion(1.0, 0.0, 0.0, 0.0));
    body_worker->set_mass(0.0);
    body_worker->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));

    // body controller
    body_controller->set_position(Vector3d(0.0, 0.0, 0.0));
    body_controller->set_rotation(Quaternion(1.0, 0.0, 0.0, 0.0));
    body_controller->set_mass(0.0);
    body_controller->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));

    // initialize with angle zero
    set_control_timeseries(std::vector<double>{0.0, 0.0}, std::vector<double>{0.0, 0.0});

    // initialize links
    initialize_links();
}

void ActuatorRotationChrono::set_control_timeseries(const std::vector<double>& time_array,
                                                    const std::vector<double>& values_array) {
    std::dynamic_pointer_cast<ChFunctionArray>(chfunc)->time_array = time_array;
    std::dynamic_pointer_cast<ChFunctionArray>(chfunc)->values_array = values_array;
}

double ActuatorRotationChrono::get_control_value(double time) const {
    return std::dynamic_pointer_cast<ChFunctionArray>(chfunc)->GetVal(time);
}

double ActuatorRotationChrono::get_angle() const {
    return chobj->GetMotorAngle();
}

void ActuatorRotationChrono::impose_value_constant(double value) {
    auto current_angle = get_angle();
    increment_value_constant(value - current_angle);
}

void ActuatorRotationChrono::increment_value_constant(double value) {
    auto new_angle = get_angle() + value;
    body_worker->set_rotation(AngleAxisd(value, get_rotation_axis()) * body_worker->get_rotation());
    set_control_timeseries(std::vector<double>{0.0, 0.0}, std::vector<double>{new_angle, new_angle});
    initialize_links();
}

void ActuatorRotationChrono::set_fixed_actuator(bool is_fixed) {
    dynamic_cast<LinkChrono&>(*link).chobj->SetDisabled(!is_fixed);
    chobj->SetDisabled(is_fixed);
}

bool ActuatorRotationChrono::is_fixed_actuator() const {
    return chobj->IsDisabled();
}

void ActuatorRotationChrono::initialize_links() {
    // link
    link->initialize(*body_worker, *body_controller);

    // actuator link
    auto body1ref = dynamic_cast<BodyElastoChrono&>(*body_worker);
    auto body2ref = dynamic_cast<BodyElastoChrono&>(*body_controller);
    auto chframe0 = chrono::ChFrame<double>(chrono::ChVector3(0.0, 0.0, 0.0), quat2ch(reference_rotation));
    chobj->Initialize(body1ref.chobj, body2ref.chobj, true, chframe0, chframe0);
}

LinkMatrixStiffnessDampingChrono::LinkMatrixStiffnessDampingChrono() {
    // empty stiffness and damping matrices
    stiffness_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    damping_matrix = Eigen::Matrix<double, 6, 6>::Zero();
}

void LinkMatrixStiffnessDampingChrono::initialize(const Entity& entity1, const Entity& entity2) {
    try {
        // cast to Chrono bodies
        auto body1 = dynamic_cast<const BodyElastoChrono&>(entity1);
        auto body2 = dynamic_cast<const BodyElastoChrono&>(entity2);

        // instantiate Chrono object
        chobj = chrono_types::make_shared<chrono::ChLoadBodyBodyBushingGeneric>(
            body1.chobj, body2.chobj, body2.chobj->GetFrameCOMToAbs(), stiffness_matrix, damping_matrix);
    } catch (const std::bad_cast& e) {
        throw std::runtime_error("Cannot link these entities with cable link.");
    }
};

void LinkMatrixStiffnessDampingChrono::set_stiffness_matrix(const Eigen::Matrix<double, 6, 6>& stiffness_matrix) {
    this->stiffness_matrix = stiffness_matrix;
    if (chobj) {
        chobj->SetStiffnessMatrix(stiffness_matrix);
        auto kk = chobj->GetStiffnessMatrix();
    }
}

void LinkMatrixStiffnessDampingChrono::set_damping_matrix(const Eigen::Matrix<double, 6, 6>& damping_matrix) {
    this->damping_matrix = damping_matrix;
    if (chobj) {
        chobj->SetDampingMatrix(damping_matrix);
    }
}

MeshElastoChrono::MeshElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChMesh>();
    chloadcontainers.clear();
}

void MeshElastoChrono::add(NodeElasto& node) {
    auto& ref = dynamic_cast<NodeElastoChronoBase&>(node);
    chobj->AddNode(ref.chobj);
    chloadcontainers.push_back(ref.chloadcontainer);

    // if mesh was already added to system, add container to system directly
    if (chobj->GetSystem() != nullptr) {
        chobj->GetSystem()->Add(ref.chloadcontainer);
    }
}

void MeshElastoChrono::add(ElementElasto& element) {
    chobj->AddElement(dynamic_cast<ElementElastoChrono&>(element).chobj);
}

SystemElastoChrono::SystemElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChSystemSMC>();
    set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));
    chloadcontainer = chrono_types::make_shared<chrono::ChLoadContainer>();
    chobj->Add(chloadcontainer);

    // solver
    auto solver = chrono_types::make_shared<chrono::ChSolverSparseLU>();
    chobj->SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    // timestepping
    chobj->SetTimestepperType(chrono::ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(chobj->GetTimestepper());
    mystepper->SetStepControl(false);
    mystepper->SetModifiedNewton(true);

    // make mesh
    mesh = std::make_shared<MeshElastoChrono>();
    add(*(mesh.get()));
}

void SystemElastoChrono::assemble() {
    spdlog::debug("Assembly of system.");
    if (is_assembled) {
        throw std::runtime_error("Component already assembled: " + std::string(typeid(*this).name()) + ".");
    }
    for (auto& turbine : turbines) {
        turbine->assemble(*this);
    }
    for (auto& component : components) {
        component->assemble(*this);
    }
    is_assembled = true;

    // needed for some Chrono (e.g. for moorings or HydroChrono floater)
    chobj->Update();

    // warnings for properties that were not set
    int body_idx = 0;
    for (auto& body : chobj->GetBodies()) {
        if (body->GetMass() == MASS_NOTSET_VALUE) {
            spdlog::warn("Body (index {} in elasto system) mass was not set, making it massless.", body_idx);
            body->SetMass(0.0);
        }
        auto inertia_matrix = body->GetInertia();
        if (inertia_matrix(0, 0) == inertia_matrix(1, 1) && inertia_matrix(1, 1) == inertia_matrix(2, 2) &&
            inertia_matrix(2, 2) == MASS_NOTSET_VALUE && inertia_matrix.sum() == 3 * MASS_NOTSET_VALUE) {
            spdlog::warn("Body (index {} in elasto system) inertia matrix mass was not set, making a null matrix.",
                         body_idx);
            body->SetInertiaXX(chrono::ChVector3<double>(0.0, 0.0, 0.0));
        }
        body_idx += 1;
    }

    spdlog::debug("Finished assembly of system.");
}

void SystemElastoChrono::presetup(double fraction) {
    for (auto& turbine : turbines) {
        turbine->presetup(fraction);
    }
    for (auto& component : components) {
        component->presetup(fraction);
    }
}

void SystemElastoChrono::step(double dt) {
    chobj->DoStepDynamics(dt);
}

double SystemElastoChrono::get_time() const {
    return chobj->GetChTime();
}

void SystemElastoChrono::set_time(double time) {
    chobj->SetChTime(time);
}

void SystemElastoChrono::do_statics(bool linear, int nonlinear_steps) {
    // assemble system if it was not already
    if (!is_assembled) {
        assemble();
    }

    // constrain rotor
    for (auto& turbine : turbines) {
        turbine->rna.link_shaft_hub->set_constraints(true, true, true, true, true, true);
    }

    // linear statics
    if (linear) {
        chobj->DoStaticLinear();
    }
    // nonlinear statics
    if (nonlinear_steps > 0) {
        chobj->DoStaticNonlinear(nonlinear_steps, true);
    }

    // unconstrain rotor
    for (auto& turbine : turbines) {
        // rotor
        turbine->rna.link_shaft_hub->set_constraints(true, true, true, false, true, true);
    }

    spdlog::debug("Performed statics prestep with linear step as {} and {} nonlinear steps.", linear, nonlinear_steps);
};

Vector3d SystemElastoChrono::get_gravitational_acceleration() const {
    return ch2vec(chobj->GetGravitationalAcceleration());
}

void SystemElastoChrono::set_gravitational_acceleration(const Vector3d& gravitational_acceleration) {
    chobj->SetGravitationalAcceleration(gravitational_acceleration);
}

void SystemElastoChrono::add(BodyElasto& body) {
    auto& ref = dynamic_cast<BodyElastoChrono&>(body);
    chobj->Add(ref.chobj);
    chobj->Add(ref.chloadcontainer);
}

void SystemElastoChrono::add(MeshElasto& mesh) {
    auto& mesh_chrono = dynamic_cast<MeshElastoChrono&>(mesh);
    chobj->Add(mesh_chrono.chobj);

    // add all existing load containers to system
    for (auto& container : mesh_chrono.chloadcontainers) {
        chobj->Add(container);
    }
}

void SystemElastoChrono::add(Link& link) {
    chobj->Add(dynamic_cast<LinkChronoBase&>(link).chobj);
}

void SystemElastoChrono::add(LinkMatrixStiffnessDamping& link) {
    auto load_container = chrono_types::make_shared<chrono::ChLoadContainer>();
    load_container->Add(dynamic_cast<LinkMatrixStiffnessDampingChrono&>(link).chobj);
    chobj->Add(load_container);
}

void SystemElastoChrono::add(SpringLinear& spring) {
    chobj->Add(dynamic_cast<SpringLinearChrono&>(spring).chobj);
}

void SystemElastoChrono::add(ActuatorRotation& actuator) {
    chobj->Add(dynamic_cast<ActuatorRotationChrono&>(actuator).chobj);
    add(*actuator.body_worker);
    add(*actuator.body_controller);
    add(*actuator.link);
}
