#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/entities_elasto.h>

#include <chrono/core/ChVector.h>
#include <chrono/core/ChMatrix.h>
#include <chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h>
#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkRevolute.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>

#include <vector>
#include <memory>

namespace seahowl {
namespace elasto {

chrono::ChVector<double> vec2ch(Vector3d vector_in) {
    return chrono::ChVector<double>(vector_in[0], vector_in[1], vector_in[2]);
}
Vector3d ch2vec(chrono::ChVector<double> vector_in) {
    return Vector3d(vector_in[0], vector_in[1], vector_in[2]);
}

chrono::ChQuaternion<double> quat2ch(Quaternion quaternion_in) {
    return chrono::ChQuaternion<double>(quaternion_in.w(), quaternion_in.x(), quaternion_in.y(), quaternion_in.z());
}

Quaternion ch2quat(chrono::ChQuaternion<double> quaternion_in) {
    return Quaternion(quaternion_in[0], quaternion_in[1], quaternion_in[2], quaternion_in[3]);
}

Quaternion node_ch2iec(chrono::ChQuaternion<double> quaternion_in) {
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

chrono::ChQuaternion<double> node_iec2ch(Quaternion quaternion_in) {
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

void EntityDynamicChrono::set_position(Vector3d position) {
    chobj->SetPos(vec2ch(position));
}

Vector3d EntityDynamicChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

void EntityDynamicChrono::set_rotation(Quaternion rotation) {
    chobj->SetRot(quat2ch(rotation));
}

Quaternion EntityDynamicChrono::get_rotation() const {
    return ch2quat(chobj->GetRot());
}

void EntityDynamicChrono::set_velocity(Vector3d velocity) {
    chobj->SetPos_dt(vec2ch(velocity));
}

Vector3d EntityDynamicChrono::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

void EntityDynamicChrono::set_acceleration(Vector3d acceleration) {
    chobj->SetPos_dtdt(vec2ch(acceleration));
}

Vector3d EntityDynamicChrono::get_acceleration() const {
    return ch2vec(chobj->GetPos_dt());
}

void EntityDynamicChrono::set_rotational_velocity_local(Vector3d rotational_velocity_local) {
    chobj->SetWvel_loc(vec2ch(rotational_velocity_local));
}

Vector3d EntityDynamicChrono::get_rotational_velocity_local() const {
    return ch2vec(chobj->GetWvel_loc());
}

void EntityDynamicChrono::set_rotational_acceleration_local(Vector3d rotational_acceleration_local) {
    chobj->SetWacc_loc(vec2ch(rotational_acceleration_local));
}

Vector3d EntityDynamicChrono::get_rotational_acceleration_local() const {
    return ch2vec(chobj->GetWacc_loc());
}

void EntityDynamicChrono::set_rotational_velocity_global(Vector3d rotational_velocity_global) {
    chobj->SetWvel_par(vec2ch(rotational_velocity_global));
}

Vector3d EntityDynamicChrono::get_rotational_velocity_global() const {
    return ch2vec(chobj->GetWvel_par());
}

void EntityDynamicChrono::set_rotational_acceleration_global(Vector3d rotational_acceleration_global) {
    chobj->SetWacc_par(vec2ch(rotational_acceleration_global));
}

Vector3d EntityDynamicChrono::get_rotational_acceleration_global() const {
    return ch2vec(chobj->GetWacc_par());
}

BodyElastoChrono::BodyElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChBody>();
}

void BodyElastoChrono::set_position(Vector3d position) {
    chobj->SetPos(vec2ch(position));
}

Vector3d BodyElastoChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

void BodyElastoChrono::set_rotation(Quaternion rotation) {
    chobj->SetRot(quat2ch(rotation));
}

Quaternion BodyElastoChrono::get_rotation() const {
    return ch2quat(chobj->GetRot());
}

void BodyElastoChrono::set_velocity(Vector3d velocity) {
    chobj->SetPos_dt(vec2ch(velocity));
}

Vector3d BodyElastoChrono::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

void BodyElastoChrono::set_acceleration(Vector3d acceleration) {
    chobj->SetPos_dtdt(vec2ch(acceleration));
}

Vector3d BodyElastoChrono::get_acceleration() const {
    return ch2vec(chobj->GetPos_dt());
}

void BodyElastoChrono::set_rotational_velocity_local(Vector3d rotational_velocity_local) {
    chobj->SetWvel_loc(vec2ch(rotational_velocity_local));
}

Vector3d BodyElastoChrono::get_rotational_velocity_local() const {
    return ch2vec(chobj->GetWvel_loc());
}

void BodyElastoChrono::set_rotational_acceleration_local(Vector3d rotational_acceleration_local) {
    chobj->SetWacc_loc(vec2ch(rotational_acceleration_local));
}

Vector3d BodyElastoChrono::get_rotational_acceleration_local() const {
    return ch2vec(chobj->GetWacc_loc());
}

void BodyElastoChrono::set_rotational_velocity_global(Vector3d rotational_velocity_global) {
    chobj->SetWvel_par(vec2ch(rotational_velocity_global));
}

Vector3d BodyElastoChrono::get_rotational_velocity_global() const {
    return ch2vec(chobj->GetWvel_par());
}

void BodyElastoChrono::set_rotational_acceleration_global(Vector3d rotational_acceleration_global) {
    chobj->SetWacc_par(vec2ch(rotational_acceleration_global));
}

Vector3d BodyElastoChrono::get_rotational_acceleration_global() const {
    return ch2vec(chobj->GetWacc_par());
}

void BodyElastoChrono::set_mass(double mass) {
    chobj->SetMass(mass);
}

void BodyElastoChrono::set_inertia_diagonal(Vector3d inertia) {
    chobj->SetInertiaXX(vec2ch(inertia));
}

void BodyElastoChrono::reset_forces() {
    chobj->Empty_forces_accumulators();
}

void BodyElastoChrono::accumulate_torque(Vector3d torque, bool is_local) {
    chobj->Accumulate_torque(torque, is_local);
}

void BodyElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetBodyFixed(is_fixed);
}

double BodyElastoChrono::get_mass() {
    return chobj->GetMass();
}

NodeElastoChrono::NodeElastoChrono(Vector3d position, Quaternion rotation) {
    chobj = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(
        chrono::ChFrame<>(vec2ch(position), node_iec2ch(rotation)));
}

void NodeElastoChrono::set_position(Vector3d position) {
    chobj->SetPos(vec2ch(position));
}

Vector3d NodeElastoChrono::get_position() const {
    return ch2vec(chobj->GetPos());
}

void NodeElastoChrono::set_rotation(Quaternion rotation) {
    chobj->SetRot(node_iec2ch(rotation));
}

Quaternion NodeElastoChrono::get_rotation() const {
    return node_ch2iec(chobj->GetRot());
}

void NodeElastoChrono::set_velocity(Vector3d velocity) {
    chobj->SetPos_dt(vec2ch(velocity));
}

Vector3d NodeElastoChrono::get_velocity() const {
    return ch2vec(chobj->GetPos_dt());
}

void NodeElastoChrono::set_acceleration(Vector3d acceleration) {
    chobj->SetPos_dtdt(vec2ch(acceleration));
}

Vector3d NodeElastoChrono::get_acceleration() const {
    return ch2vec(chobj->GetPos_dt());
}

void NodeElastoChrono::set_rotational_velocity_local(Vector3d rotational_velocity_local) {
    chobj->SetWvel_loc(vec2ch(rotational_velocity_local));
}

Vector3d NodeElastoChrono::get_rotational_velocity_local() const {
    return ch2vec(chobj->GetWvel_loc());
}

void NodeElastoChrono::set_rotational_acceleration_local(Vector3d rotational_acceleration_local) {
    chobj->SetWacc_loc(vec2ch(rotational_acceleration_local));
}

Vector3d NodeElastoChrono::get_rotational_acceleration_local() const {
    return ch2vec(chobj->GetWacc_loc());
}

void NodeElastoChrono::set_rotational_velocity_global(Vector3d rotational_velocity_global) {
    chobj->SetWvel_par(vec2ch(rotational_velocity_global));
}

Vector3d NodeElastoChrono::get_rotational_velocity_global() const {
    return ch2vec(chobj->GetWvel_par());
}

void NodeElastoChrono::set_rotational_acceleration_global(Vector3d rotational_acceleration_global) {
    chobj->SetWacc_par(vec2ch(rotational_acceleration_global));
}

Vector3d NodeElastoChrono::get_rotational_acceleration_global() const {
    return ch2vec(chobj->GetWacc_par());
}

Vector3d NodeElastoChrono::get_direction() const {
    return ch2vec(chobj->TransformDirectionLocalToParent(chrono::ChVector<double>(1.0, 0.0, 0.0)));
}

void NodeElastoChrono::set_load(Vector3d force) {
    chobj->SetForce(vec2ch(force));
}

void NodeElastoChrono::set_torque(Vector3d torque) {
    chobj->SetTorque(vec2ch(torque));
}

void NodeElastoChrono::set_fixed(bool is_fixed) {
    chobj->SetFixed(is_fixed);
}

Vector3d NodeElastoChrono::get_load() const {
    return ch2vec(chobj->GetForce());
}

Vector3d NodeElastoChrono::get_torque() const {
    return ch2vec(chobj->GetTorque());
}

void NodeElastoChrono::set_properties(const BladeReferencePointElasto& ref, bool fpm) {
    Eigen::Matrix<double, 6, 6> mm = ref.mass_matrix.replicate(1, 1);
    Eigen::Matrix<double, 6, 6> sm = ref.stiffness_matrix.replicate(1, 1);
    // Convert from IEC convention to Chrono convention.
    // IEC standard:
    // x-axis: flapwise pointing towards nacelle,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : longitudinal pointing towards blade tip.
    // Chrono convention:
    // x-axis: longitudinal pointing towards blade tip,
    // y-axis : edgewise pointing towards trailing edge,
    // z-axis : flapwise pointing away from nacelle.
    int jjo, kko;
    for (int jj = 0; jj < 6; jj++) {
        if (jj == 2 || jj == 5) {
            jjo = -2;
        }
        if (jj == 1 || jj == 4) {
            jjo = +0;
        }
        if (jj == 0 || jj == 3) {
            jjo = +2;
        }
        for (int kk = 0; kk < 6; kk++) {
            if (kk == 2 || kk == 5) {
                kko = -2;
            }
            if (kk == 1 || kk == 4) {
                kko = +0;
            }
            if (kk == 0 || kk == 3) {
                kko = +2;
            }
            mm(jj + jjo, kk + kko) = ref.mass_matrix(jj, kk);
            sm(jj + jjo, kk + kko) = ref.stiffness_matrix(jj, kk);
        }
    }
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
        chrono::fea::DampingCoefficients damping_coefficients;
        // damping coefficients: IEC -> Chrono convention
        damping_coefficients.bx = ref.damping_coefficients[2];
        damping_coefficients.by = ref.damping_coefficients[1];
        damping_coefficients.bz = ref.damping_coefficients[0];
        damping_coefficients.bt = ref.damping_coefficients[3];
        damping_coefficients.alpha = ref.damping_coefficients[4];
        sectionFPM->SetBeamRaleyghDamping(damping_coefficients);
    } else {
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        // offsets
        section->SetCenterOfMass(ref.offset_gravity.y(), -ref.offset_gravity.x());
        section->SetCentroidY(ref.offset_elastic.y());
        section->SetCentroidZ(-ref.offset_elastic.x());
        // material properties
        section->SetMassPerUnitLength(mm(0, 0));
        // axial
        section->SetAxialRigidity(sm(0, 0));
        section->SetXtorsionRigidity(sm(3, 3));
        // flap
        section->SetYbendingRigidity(sm(4, 4));
        // edge
        section->SetZbendingRigidity(sm(5, 5));
        // damping
        chrono::fea::DampingCoefficients damping_coefficients;
        // damping coefficients: IEC -> Chrono convention
        damping_coefficients.bx = ref.damping_coefficients[2];
        damping_coefficients.by = ref.damping_coefficients[1];
        damping_coefficients.bz = ref.damping_coefficients[0];
        damping_coefficients.bt = ref.damping_coefficients[3];
        damping_coefficients.alpha = ref.damping_coefficients[4];
        section->SetBeamRaleyghDamping(damping_coefficients);
    }
}

void NodeElastoChrono::set_properties(const TowerReferencePointElasto& ref) {
    // make first section for tapered section
    section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    // material properties
    section->SetMassPerUnitLength(ref.density);
    // axial
    section->SetAxialRigidity(ref.stiffness_axial);
    section->SetXtorsionRigidity(ref.stiffness_torsion);
    // foreaft
    section->SetZbendingRigidity(ref.stiffness_foreaft);
    // sideside
    section->SetYbendingRigidity(ref.stiffness_sideside);
    // damping
    chrono::fea::DampingCoefficients damping_coefficients;
    damping_coefficients.bx = ref.damping_coefficients[0];
    damping_coefficients.by = ref.damping_coefficients[1];
    damping_coefficients.bz = ref.damping_coefficients[2];
    damping_coefficients.bt = ref.damping_coefficients[3];
    damping_coefficients.alpha = ref.damping_coefficients[4];
    section->SetBeamRaleyghDamping(damping_coefficients);
}

ElementBladeElastoChrono::ElementBladeElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
    chobj_base = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
    chobj->SetTaperedSection(blade_section);
}

void ElementBladeElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes0.clear();
    nodes0.push_back(node1);
    nodes0.push_back(node2);

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

double ElementBladeElastoChrono::get_mass() {
    return chobj->GetMass();
}

void ElementBladeElastoChrono::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) {
    auto chvec = vec2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = node_ch2iec(chquat);
}

ElementBladeElastoChronoFPM::ElementBladeElastoChronoFPM() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenkoFPM>();
    chobj_base = chobj;
    // create blade section
    auto blade_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    chobj->SetTaperedSection(blade_section);
}

void ElementBladeElastoChronoFPM::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes0.clear();
    nodes0.push_back(node1);
    nodes0.push_back(node2);

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

double ElementBladeElastoChronoFPM::get_mass() {
    return chobj->GetMass();
}

void ElementBladeElastoChronoFPM::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) {
    auto chvec = vec2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = node_ch2iec(chquat);
}

ElementMooringElastoChrono::ElementMooringElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChElementBeamEuler>();
    chobj_base = chobj;
}

void ElementMooringElastoChrono::set_nodes(std::shared_ptr<NodeElasto> node1, std::shared_ptr<NodeElasto> node2) {
    nodes0.clear();
    nodes0.push_back(node1);
    nodes0.push_back(node2);

    // set nodes
    chobj->SetNodes(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                    std::dynamic_pointer_cast<NodeElastoChrono>(node2)->chobj);
}

double ElementMooringElastoChrono::get_mass() {
    return chobj->GetMass();
}

void ElementMooringElastoChrono::evaluate_position_rotation(double eta, Vector3d& position, Quaternion& rotation) {
    auto chvec = vec2ch(position);
    auto chquat = quat2ch(rotation);
    chobj->EvaluateSectionFrame(eta, chvec, chquat);
    position[0] = chvec[0];
    position[1] = chvec[1];
    position[2] = chvec[2];
    rotation = node_ch2iec(chquat);
}

void ElementMooringElastoChrono::set_properties(double density, double diameter, double stiffness_axial) {
    // create mooring section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionEulerAdvanced>();
    chobj->SetSection(section);
    section->SetDensity(density);
    double area = chrono::CH_C_PI * pow(diameter, 2) / 4.0;
    section->SetArea(area);
    section->SetYoungModulus(stiffness_axial / area);
    section->SetGshearModulus(0.0);
    section->SetAsCircularSection(diameter);
}

LinkChrono::LinkChrono() {
    chobj = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
    chobj->SetConstrainedCoords(true, true, true, true, true, true);
}

void LinkChrono::initialize(std::shared_ptr<BodyElasto> body1, std::shared_ptr<BodyElasto> body2) {
    chobj->Initialize(std::dynamic_pointer_cast<BodyElastoChrono>(body1)->chobj,
                      std::dynamic_pointer_cast<BodyElastoChrono>(body2)->chobj,
                      std::dynamic_pointer_cast<BodyElastoChrono>(body2)->chobj->GetFrame_COG_to_abs());
}

void LinkChrono::initialize(std::shared_ptr<NodeElasto> node1, std::shared_ptr<BodyElasto> body2) {
    chobj->Initialize(std::dynamic_pointer_cast<NodeElastoChrono>(node1)->chobj,
                      std::dynamic_pointer_cast<BodyElastoChrono>(body2)->chobj,
                      std::dynamic_pointer_cast<BodyElastoChrono>(body2)->chobj->GetFrame_COG_to_abs());
}

void LinkChrono::set_constraints(bool surge, bool sway, bool heave, bool roll, bool pitch, bool yaw) {
    chobj->SetConstrainedCoords(surge, sway, heave, roll, pitch, yaw);
}

Vector3d LinkChrono::get_reaction_force() const {
    return ch2vec(chobj->Get_react_force());
}

Vector3d LinkChrono::get_reaction_torque() const {
    return ch2vec(chobj->Get_react_torque());
}

MeshElastoChrono::MeshElastoChrono() {
    chobj = chrono_types::make_shared<chrono::fea::ChMesh>();
}

void MeshElastoChrono::add(std::shared_ptr<NodeElasto> node) {
    chobj->AddNode(std::dynamic_pointer_cast<NodeElastoChrono>(node)->chobj);
}

void MeshElastoChrono::add(std::shared_ptr<ElementElasto> element) {
    chobj->AddElement(std::dynamic_pointer_cast<ElementElastoChrono>(element)->chobj_base);
}

void MeshElastoChrono::add(std::shared_ptr<chrono::fea::ChElementBeam> element) {
    chobj->AddElement(element);
}

SystemElastoChrono::SystemElastoChrono() {
    chobj = chrono_types::make_shared<chrono::ChSystemSMC>();

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
    mystepper->SetModifiedNewton(false);
}

void SystemElastoChrono::step(double dt) {
    chobj->DoStepDynamics(dt);
}

double SystemElastoChrono::get_time() const {
    return chobj->GetChTime();
}

void SystemElastoChrono::do_statics(bool linear, int nonlinear_steps) {
    if (linear) {
        chobj->DoStaticLinear();
    }
    if (nonlinear_steps > 0) {
        chobj->DoStaticNonlinear(nonlinear_steps, true);
    }
};

Vector3d SystemElastoChrono::get_gravitational_acceleration() const {
    return ch2vec(chobj->Get_G_acc());
}

void SystemElastoChrono::set_gravitational_acceleration(Vector3d gravitational_acceleration) {
    chobj->Set_G_acc(gravitational_acceleration);
}

void SystemElastoChrono::add(std::shared_ptr<BodyElasto> body) {
    chobj->Add(std::dynamic_pointer_cast<BodyElastoChrono>(body)->chobj);
}

void SystemElastoChrono::add(std::shared_ptr<MeshElasto> mesh) {
    chobj->Add(std::dynamic_pointer_cast<MeshElastoChrono>(mesh)->chobj);
}

void SystemElastoChrono::add(std::shared_ptr<Link> link) {
    chobj->Add(std::dynamic_pointer_cast<LinkChrono>(link)->chobj);
}

}  // namespace elasto
}  // namespace seahowl
