#include "seahowl/core/reference_point.h"

#include "seahowl/fluid/aero/airfoil.h"

using namespace seahowl::core;

BladeReferencePoint::BladeReferencePoint() {
    stiffness_matrix.setZero();
    mass_matrix.setZero();
}

BladeReferencePoint BladeReferencePoint::operator*(const double factor) const {
    BladeReferencePoint new_point = *this;
    new_point.coordinates *= factor;
    new_point.offset_elastic *= factor;
    new_point.offset_gravity *= factor;
    new_point.fraction *= factor;
    new_point.structural_twist *= factor;
    new_point.mass_matrix *= factor;
    new_point.stiffness_matrix *= factor;
    new_point.damping_coefficients[0] *= factor;
    new_point.damping_coefficients[1] *= factor;
    new_point.damping_coefficients[2] *= factor;
    new_point.damping_coefficients[3] *= factor;
    new_point.damping_coefficients[4] *= factor;
    for (int ii = 0; ii < airfoil_properties.size(); ii++) {
        new_point.airfoil_properties[ii] = airfoil_properties[ii] * factor;
    }
    return new_point;
};

BladeReferencePoint BladeReferencePoint::operator+(const BladeReferencePoint& other) const {
    BladeReferencePoint new_point = *this;
    new_point.coordinates += other.coordinates;
    new_point.offset_elastic += other.offset_elastic;
    new_point.offset_gravity += other.offset_gravity;
    new_point.fraction += other.fraction;
    new_point.structural_twist += other.structural_twist;
    new_point.stiffness_matrix += other.stiffness_matrix;
    new_point.mass_matrix += other.mass_matrix;
    new_point.damping_coefficients[0] += other.damping_coefficients[0];
    new_point.damping_coefficients[1] += other.damping_coefficients[1];
    new_point.damping_coefficients[2] += other.damping_coefficients[2];
    new_point.damping_coefficients[3] += other.damping_coefficients[3];
    new_point.damping_coefficients[4] += other.damping_coefficients[4];
    for (int ii = 0; ii < airfoil_properties.size(); ii++) {
        if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
            throw std::runtime_error("Trying to add airfoil properties with different Reynolds number.");
        }
        new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
    }
    return new_point;
};

TowerReferencePoint::TowerReferencePoint() {}

TowerReferencePoint TowerReferencePoint::operator*(const double factor) const {
    TowerReferencePoint new_point = *this;
    new_point.coordinates *= factor;
    new_point.fraction *= factor;
    new_point.density *= factor;
    new_point.stiffness_axial *= factor;
    new_point.stiffness_foreaft *= factor;
    new_point.stiffness_sideside *= factor;
    new_point.stiffness_torsion *= factor;
    new_point.damping_coefficients[0] *= factor;
    new_point.damping_coefficients[1] *= factor;
    new_point.damping_coefficients[2] *= factor;
    new_point.damping_coefficients[3] *= factor;
    new_point.damping_coefficients[4] *= factor;
    new_point.diameter *= factor;
    new_point.drag_coefficient *= factor;
    return new_point;
};

TowerReferencePoint TowerReferencePoint::operator+(const TowerReferencePoint& other) const {
    TowerReferencePoint new_point = *this;
    new_point.coordinates += other.coordinates;
    new_point.fraction += other.fraction;
    new_point.density += other.density;
    new_point.stiffness_axial += other.stiffness_axial;
    new_point.stiffness_foreaft += other.stiffness_foreaft;
    new_point.stiffness_sideside += other.stiffness_sideside;
    new_point.stiffness_torsion += other.stiffness_torsion;
    new_point.damping_coefficients[0] += other.damping_coefficients[0];
    new_point.damping_coefficients[1] += other.damping_coefficients[1];
    new_point.damping_coefficients[2] += other.damping_coefficients[2];
    new_point.damping_coefficients[3] += other.damping_coefficients[3];
    new_point.damping_coefficients[4] += other.damping_coefficients[4];
    new_point.diameter += other.diameter;
    new_point.drag_coefficient += other.drag_coefficient;
    return new_point;
};
