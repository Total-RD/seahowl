#include <seahowl/elasto/reference_point_elasto.h>

using namespace seahowl::elasto;

BladeReferencePointElasto::BladeReferencePointElasto() {}

BladeReferencePointElasto::BladeReferencePointElasto(const seahowl::core::BladeReferencePoint& point) {
    coordinates = point.coordinates;
    offset_elastic = point.offset_elastic;
    offset_gravity = point.offset_gravity;
    stiffness_matrix = point.stiffness_matrix;
    mass_matrix = point.mass_matrix;
    fraction = point.fraction;
    structural_twist = point.structural_twist;
    damping_coefficients = point.damping_coefficients;
}

BladeReferencePointElasto BladeReferencePointElasto::operator*(const double factor) const {
    BladeReferencePointElasto new_point = *this;
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
    return new_point;
};

BladeReferencePointElasto BladeReferencePointElasto::operator+(const BladeReferencePointElasto& other) const {
    BladeReferencePointElasto new_point = *this;
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
    return new_point;
};

TowerReferencePointElasto::TowerReferencePointElasto() {}

TowerReferencePointElasto::TowerReferencePointElasto(const seahowl::core::TowerReferencePoint& point) {
    coordinates = point.coordinates;
    fraction = point.fraction;
    density = point.density;
    stiffness_axial = point.stiffness_axial;
    stiffness_foreaft = point.stiffness_foreaft;
    stiffness_sideside = point.stiffness_sideside;
    stiffness_torsion = point.stiffness_torsion;
    damping_coefficients = point.damping_coefficients;
}

TowerReferencePointElasto TowerReferencePointElasto::operator*(const double factor) const {
    TowerReferencePointElasto new_point = *this;
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
    return new_point;
};

TowerReferencePointElasto TowerReferencePointElasto::operator+(const TowerReferencePointElasto& other) const {
    TowerReferencePointElasto new_point = *this;
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
    return new_point;
};

ReferencePointElasto::ReferencePointElasto() {}

ReferencePointElasto ReferencePointElasto::operator*(const double factor) const {
    ReferencePointElasto new_point = *this;
    new_point.coordinates *= factor;
    new_point.fraction *= factor;
    return new_point;
};

ReferencePointElasto ReferencePointElasto::operator+(const ReferencePointElasto& other) const {
    ReferencePointElasto new_point = *this;
    new_point.coordinates += other.coordinates;
    new_point.fraction += other.fraction;
    return new_point;
};
