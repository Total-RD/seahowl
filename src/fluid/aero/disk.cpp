// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

// SEAHOWL headers
#include "seahowl/fluid/aero/airfoil.h"
#include "seahowl/fluid/aero/bemt.h"
#include "seahowl/fluid/aero/blade_aero.h"
#include "seahowl/fluid/aero/tower_aero.h"

// Standard library
#include <iostream>

using seahowl::Vector2d;
using seahowl::Vector3d;
using seahowl::PI;

// TSR = Omega*(HAWT["RotorDia"]*0.5) / numpy.mean(VN)
// CT = RotorChar['CT']
// CP = RotorChar['CP']
// ct = CT(TSR,pitch*180/numpy.pi); ct = ct[0]
// cp = CP(TSR,pitch*180/numpy.pi); cp = cp[0]

// #print(numpy.mean(VN), TSR, pitch)

// FaxT   += (1/HAWT['Nbla']) *  ct * 0.5 * HAWT['AirDens'] * numpy.pi * (HAWT['RotorDia']*0.5)**2 * numpy.mean(VN)**2
// PaeroT = (1/HAWT['Nbla']) * cp * 0.5 * HAWT['AirDens']  * numpy.pi * (HAWT['RotorDia']*0.5)**2 * numpy.mean(VN)**3
// MaeroT += PaeroT / Omega
// FaxTb = FaxT / HAWT['Nbla']
