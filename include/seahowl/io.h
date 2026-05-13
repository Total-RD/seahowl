// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

#include "seahowl/io/config_manager.h"
#include "seahowl/io/input_structures.h"
#include "seahowl/io/input_reader.h"
#include "seahowl/io/input_reader_json.h"
#include "seahowl/io/input_handler.h"
#include "seahowl/io/read_input.h"
#include "seahowl/io/read_rotor_perf.h"
#include "seahowl/io/command_parser.h"
#include "seahowl/io/output_manager.h"
#include "seahowl/io/write_csv.h"
#include "seahowl/io/utils_io.h"
#include "seahowl/io/viz_insitu.h"
#ifdef HAVE_VTK
    #include "seahowl/io/write_vtk.h"
#endif
#ifdef HAVE_IRRLICHT
    #include "seahowl/io/viz_insitu_irrlicht.h"
#endif
