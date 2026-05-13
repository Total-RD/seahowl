// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/elasto/reference_point_elasto.h"

// Third-party libraries
#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

BladeReferencePointElasto::BladeReferencePointElasto() {}

BladeReferencePointElasto BladeReferencePointElasto::operator*(const double factor) const {
    BladeReferencePointElasto new_point = *this;
    new_point.coordinates *= factor;
    new_point.offset_elastic *= factor;
    new_point.offset_gravity *= factor;
    new_point.fraction *= factor;
    new_point.structural_twist *= factor;
    new_point.mass_matrix *= factor;
    new_point.stiffness_matrix *= factor;
    new_point.damping_flapwise *= factor;
    new_point.damping_edgewise *= factor;
    new_point.damping_axial *= factor;
    new_point.damping_torsion *= factor;
    new_point.damping_mass *= factor;
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
    new_point.damping_flapwise += other.damping_flapwise;
    new_point.damping_edgewise += other.damping_edgewise;
    new_point.damping_axial += other.damping_axial;
    new_point.damping_torsion += other.damping_torsion;
    new_point.damping_mass += other.damping_mass;
    return new_point;
};

TowerReferencePointElasto::TowerReferencePointElasto() {}

TowerReferencePointElasto TowerReferencePointElasto::operator*(const double factor) const {
    TowerReferencePointElasto new_point = *this;
    new_point.coordinates *= factor;
    new_point.fraction *= factor;
    new_point.density *= factor;
    new_point.stiffness_axial *= factor;
    new_point.stiffness_foreaft *= factor;
    new_point.stiffness_sideside *= factor;
    new_point.stiffness_torsion *= factor;
    new_point.stiffness_foreaft_shear *= factor;
    new_point.stiffness_sideside_shear *= factor;
    new_point.inertia_foreaft *= factor;
    new_point.inertia_sideside *= factor;
    new_point.damping_foreaft *= factor;
    new_point.damping_sideside *= factor;
    new_point.damping_axial *= factor;
    new_point.damping_torsion *= factor;
    new_point.damping_mass *= factor;
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
    new_point.stiffness_foreaft_shear += other.stiffness_foreaft_shear;
    new_point.stiffness_sideside_shear += other.stiffness_sideside_shear;
    new_point.inertia_foreaft += other.inertia_foreaft;
    new_point.inertia_sideside += other.inertia_sideside;
    new_point.damping_foreaft += other.damping_foreaft;
    new_point.damping_sideside += other.damping_sideside;
    new_point.damping_axial += other.damping_axial;
    new_point.damping_torsion += other.damping_torsion;
    new_point.damping_mass += other.damping_mass;
    return new_point;
};

void TowerReferencePointElasto::set_properties_cylinder(double density_volume,
                                                        double young_modulus,
                                                        double poisson_ratio,
                                                        double outer_diameter,
                                                        double thickness,
                                                        bool shear,
                                                        double fill_density) {
    auto shear_modulus = 0.5 * young_modulus / (1.0 + poisson_ratio);

    // geometry info
    auto d1 = outer_diameter;
    auto d2 = outer_diameter - 2.0 * thickness;
    auto area = PI * (pow(d1, 2) - pow(d2, 2)) / 4.0;
    // linear density
    auto density_linear = density_volume * area + fill_density * PI * pow(d2, 2) / 4.0;
    // stiffnesses
    auto EI = young_modulus * PI * (pow(d1, 4) - pow(d2, 4)) / 64.;  // bending
    auto EA = young_modulus * area;                                  // axial
    auto kt = shear_modulus * PI * (pow(d1, 4) - pow(d2, 4)) / 32.;  // torsion

    // populate reference point
    stiffness_foreaft = EI;
    stiffness_sideside = EI;
    stiffness_axial = EA;
    stiffness_torsion = kt;
    density = density_linear;
    // inertia from tower thickness density + fill fluid density
    inertia_foreaft = EI / young_modulus * density_volume + PI * (pow(d2, 4)) / 64. * fill_density;
    inertia_sideside = EI / young_modulus * density_volume + PI * (pow(d2, 4)) / 64. * fill_density;

    if (shear) {
        stiffness_foreaft_shear = shear_modulus * area;
        stiffness_sideside_shear = shear_modulus * area;
    } else {
        stiffness_foreaft_shear = 0.0;
        stiffness_sideside_shear = 0.0;
    }
}

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
