#include <seahowl/core/reference_point.h>

using namespace seahowl::core;

BladeReferencePoint::BladeReferencePoint() {
    stiffness_matrix.setZero();
    mass_matrix.setZero();
    damping_coefficients.bx = 0.03;
    damping_coefficients.by = 0.03;
    damping_coefficients.bz = 0.03;
    damping_coefficients.bt = 0.06;
    damping_coefficients.alpha = 0.0;
}

BladeReferencePoint::~BladeReferencePoint() {}

BladeReferencePoint BladeReferencePoint::operator*(const double factor) const {
    BladeReferencePoint new_point = *this;
    new_point.coordinates *= factor;
    new_point.offset_elastic *= factor;
    new_point.offset_gravity *= factor;
    new_point.fraction *= factor;
    new_point.structural_twist *= factor;
    new_point.mass_matrix *= factor;
    new_point.stiffness_matrix *= factor;
    new_point.damping_coefficients.bx *= factor;
    new_point.damping_coefficients.by *= factor;
    new_point.damping_coefficients.bz *= factor;
    new_point.damping_coefficients.bt *= factor;
    new_point.damping_coefficients.alpha *= factor;
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
    new_point.damping_coefficients.bx += other.damping_coefficients.bx;
    new_point.damping_coefficients.by += other.damping_coefficients.by;
    new_point.damping_coefficients.bz += other.damping_coefficients.bz;
    new_point.damping_coefficients.bt += other.damping_coefficients.bt;
    new_point.damping_coefficients.alpha += other.damping_coefficients.alpha;
    for (int ii = 0; ii < airfoil_properties.size(); ii++) {
        if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
            throw std::runtime_error("Trying to add airfoil properties with different Reynolds number.");
        }
        new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
    }
    return new_point;
};

TowerReferencePoint::TowerReferencePoint() {
    damping_coefficients.bx = 0.03;
    damping_coefficients.by = 0.03;
    damping_coefficients.bz = 0.03;
    damping_coefficients.bt = 0.06;
    damping_coefficients.alpha = 0.0;
}

TowerReferencePoint::~TowerReferencePoint() {}

TowerReferencePoint TowerReferencePoint::operator*(const double factor) const {
    TowerReferencePoint new_point = *this;
    new_point.coordinates *= factor;
    new_point.fraction *= factor;
    new_point.density *= factor;
    new_point.stiffness_axial *= factor;
    new_point.stiffness_foreaft *= factor;
    new_point.stiffness_sideside *= factor;
    new_point.stiffness_torsion *= factor;
    new_point.damping_coefficients.bx *= factor;
    new_point.damping_coefficients.by *= factor;
    new_point.damping_coefficients.bz *= factor;
    new_point.damping_coefficients.bt *= factor;
    new_point.damping_coefficients.alpha *= factor;
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
    new_point.damping_coefficients.bx += other.damping_coefficients.bx;
    new_point.damping_coefficients.by += other.damping_coefficients.by;
    new_point.damping_coefficients.bz += other.damping_coefficients.bz;
    new_point.damping_coefficients.bt += other.damping_coefficients.bt;
    new_point.damping_coefficients.alpha += other.damping_coefficients.alpha;
    new_point.diameter += other.diameter;
    new_point.drag_coefficient += other.drag_coefficient;
    return new_point;
};
