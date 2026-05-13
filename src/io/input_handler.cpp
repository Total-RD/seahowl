// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "seahowl/io/input_handler.h"

// SEAHOWL headers
#include "seahowl/io/input_reader.h"
#include "seahowl/io/input_reader_json.h"

namespace fs = std::filesystem;

namespace seahowl {
namespace io {

void InputHandler::set_filepath(const std::string& filepath) {
    if (filepath.empty() || !fs::is_regular_file(filepath)) {
        throw std::runtime_error("File does not exist.");
    }
    // Initialize the input reader based on the file extension
    fs::path file_(filepath);
    std::string extension = file_.extension().string();
    if (extension == ".json" || extension == ".csv") {
        reader = std::make_unique<InputReaderJson>(filepath);
    } else {
        throw std::runtime_error("Unsupported file format: " + filepath);
    }
}

}  // namespace io
}  // namespace seahowl
