
#pragma once

#include <filesystem>

using path = std::filesystem::path;

// Function to get the test directory from environment variable
path get_test_dir();

// Function to get the data directory from environment variable
path get_data_dir();
