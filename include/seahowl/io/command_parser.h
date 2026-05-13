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
#include "seahowl/io/config_manager.h"

// Standard library
#include <map>
#include <string>

namespace seahowl {
namespace io {
namespace app {

class CommandLineParser {
  public:
    /**
     * @brief Print the helper message.
     * @param cmdOptions Map of command options.
     */
    static void print_helper(const std::map<std::string, app::SpecComputed>& cmdOptions);

    /**
     * @brief Parse the command line arguments.
     * @param argc Number of arguments.
     * @param argv Array of arguments.
     * @param cmdOptions Map of command options.
     */
    static std::map<std::string, std::string> parse_args(int argc,
                                                         char* argv[],
                                                         const std::map<std::string, app::SpecComputed>& cmdOptions);

  private:
    /**
     * @brief Check if the value is of the correct type.
     * @param value Value to check.
     * @param type Type to check.
     */
    static bool is_valid_type(const std::string& value, const std::string& type);
};

}  // namespace app
}  // namespace io
}  // namespace seahowl
