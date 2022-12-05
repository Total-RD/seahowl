#include <seahowl/aero/reference_point_aero.h>

using namespace seahowl::aero;

BladeReferencePointAero::BladeReferencePointAero() {
    rotation = chrono::ChQuaternion<double>(1.0, 0.0, 0.0, 0.0);
}

BladeReferencePointAero::BladeReferencePointAero(seahowl::core::BladeReferencePoint& point) {
    fraction = point.fraction;
    coordinates = point.m_coordinates;
    rotation = chrono::ChQuaternion<double>(1.0, 0.0, 0.0, 0.0);
    chord = point.chord;
    structural_twist = point.structural_twist;
    airfoil_properties = point.airfoil_properties;
}

BladeReferencePointAero::~BladeReferencePointAero() {}

BladeReferencePointAero BladeReferencePointAero::operator*(const double factor) const {
    BladeReferencePointAero new_point = *this;
    new_point.fraction *= factor;
    new_point.coordinates *= factor;
    new_point.rotation *= factor;
    new_point.chord *= factor;
    new_point.structural_twist *= factor;
    for (int ii = 0; ii < airfoil_properties.size(); ii++) {
        new_point.airfoil_properties[ii] = airfoil_properties[ii] * factor;
    }
    return new_point;
};

BladeReferencePointAero BladeReferencePointAero::operator+(const BladeReferencePointAero& other) const {
    BladeReferencePointAero new_point = *this;
    new_point.fraction += other.fraction;
    new_point.coordinates += other.coordinates;
    new_point.rotation += other.rotation;
    new_point.chord += other.chord;
    new_point.structural_twist += other.structural_twist;
    for (int ii = 0; ii < airfoil_properties.size(); ii++) {
        if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
            throw std::runtime_error("Trying to add airfoil properties with different Reynolds number.");
        }
        new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
    }
    return new_point;
};

TowerReferencePointAero::TowerReferencePointAero() {
    rotation = chrono::ChQuaternion<double>(1.0, 0.0, 0.0, 0.0);
    velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
}

TowerReferencePointAero::TowerReferencePointAero(seahowl::core::TowerReferencePoint& point) {
    fraction = point.fraction;
    coordinates = point.coordinates;
    rotation = chrono::ChQuaternion<double>(1.0, 0.0, 0.0, 0.0);
    velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
    diameter = point.diameter;
    drag_coefficient = point.drag_coefficient;
}

TowerReferencePointAero::~TowerReferencePointAero() {}

TowerReferencePointAero TowerReferencePointAero::operator*(const double factor) const {
    TowerReferencePointAero new_point = *this;
    new_point.fraction *= factor;
    new_point.coordinates *= factor;
    new_point.rotation *= factor;
    new_point.velocity *= factor;
    new_point.diameter *= factor;
    new_point.drag_coefficient *= factor;
    return new_point;
};

TowerReferencePointAero TowerReferencePointAero::operator+(const TowerReferencePointAero& other) const {
    TowerReferencePointAero new_point = *this;
    new_point.fraction += other.fraction;
    new_point.coordinates += other.coordinates;
    new_point.rotation += other.rotation;
    new_point.velocity += other.velocity;
    new_point.diameter += other.diameter;
    new_point.drag_coefficient += other.drag_coefficient;
    return new_point;
};
