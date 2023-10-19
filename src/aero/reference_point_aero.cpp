#include "seahowl/aero/reference_point_aero.h"

#include "seahowl/aero/airfoil.h"

#include <spdlog/spdlog.h>

using namespace seahowl::aero;

BladeReferencePointAero::BladeReferencePointAero() {}

BladeReferencePointAero BladeReferencePointAero::operator*(const double factor) const {
    BladeReferencePointAero new_point = *this;
    new_point.fraction *= factor;
    new_point.coordinates *= factor;
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
    new_point.chord += other.chord;
    new_point.structural_twist += other.structural_twist;
    for (int ii = 0; ii < airfoil_properties.size(); ii++) {
        if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
            spdlog::error("Trying to add airfoil properties with different Reynolds number.");
            exit(1);
        }
        new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
    }
    return new_point;
};

TowerReferencePointAero::TowerReferencePointAero() {
    rotation = Quaternion(1.0, 0.0, 0.0, 0.0);
    velocity = Vector3d(0.0, 0.0, 0.0);
}

TowerReferencePointAero TowerReferencePointAero::operator*(const double factor) const {
    TowerReferencePointAero new_point = *this;
    new_point.fraction *= factor;
    new_point.coordinates *= factor;
    // (!) the rotation is not interpolated here because quaternions do not scale this way
    // @todo Fix interpolation of quaternions in reference points (through "interpolate" function possibly).
    new_point.velocity *= factor;
    new_point.diameter *= factor;
    new_point.drag_coefficient *= factor;
    return new_point;
};

TowerReferencePointAero TowerReferencePointAero::operator+(const TowerReferencePointAero& other) const {
    TowerReferencePointAero new_point = *this;
    new_point.fraction += other.fraction;
    new_point.coordinates += other.coordinates;
    // (!) the rotation is not interpolated here because quaternions do not scale this way
    // @todo Fix interpolation of quaternions in reference points (through "interpolate" function possibly).
    new_point.velocity += other.velocity;
    new_point.diameter += other.diameter;
    new_point.drag_coefficient += other.drag_coefficient;
    return new_point;
};
