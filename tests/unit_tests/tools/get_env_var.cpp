// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "get_env_var.h"
#include <spdlog/spdlog.h>

path get_test_dir() {
    path test_dir;
    // set test directories roots
    char* const env_test_dir = std::getenv("SEAHOWL_TESTDIR");
    if (env_test_dir == NULL) {
#ifdef SEAHOWL_TESTDIR
        test_dir = path(SEAHOWL_TESTDIR);
#else
        spdlog::critical("SEAHOWL_TESTDIR not defined (need to set environment variable).");
        std::exit(EXIT_FAILURE);
#endif
    } else {
        test_dir = path(env_test_dir);
    }
    return test_dir;
}

path get_data_dir() {
    path data_dir;
    // set data directories roots
    char* const env_datadir = std::getenv("SEAHOWL_DATADIR");
    if (env_datadir == NULL) {
#ifdef SEAHOWL_DATADIR
        data_dir = path(SEAHOWL_DATADIR);
#else
        spdlog::critical("SEAHOWL_DATADIR not defined (need to set environment variable).");
        std::exit(EXIT_FAILURE);
#endif
    } else {
        data_dir = path(env_datadir);
    }
    return data_dir;
}
