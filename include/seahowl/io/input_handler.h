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

// SEAHOWL headers
#include "seahowl/io/input_reader.h"

// Standard library
#include <memory>
#include <string>

namespace seahowl {
namespace io {

class InputHandler {
  public:
    std::unique_ptr<InputReader> reader;
    // Constructor
    InputHandler() = default;

    // set filepath
    /**
     * @brief Set the file path for the input handler.
     * @param filepath Path to the input file.
     */
    void set_filepath(const std::string& filepath);

    // Destructor
    ~InputHandler() = default;
};

}  // namespace io
}  // namespace seahowl
