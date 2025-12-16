#include <gtest/gtest.h>
#include "tools/get_env_var.h"
#include <filesystem>
#include <cstdlib>
#include <string>

class MyEnvironment : public ::testing::Environment {
  public:
    void TearDown() override {
        // set test directories roots
        auto test_dir = get_test_dir();
        auto data_test_dir = test_dir / "unit_tests/data";
#ifdef _WIN32
        // Code spécifique Windows
        auto env_bin = test_dir / "../build/.venv/Scripts/activate.bat";
        auto cmd = "cmd /C \"./" + env_bin.string();
#elif defined(__linux__)
        // Code spécifique Linux
        auto env_bin = test_dir / "../build/.venv/bin/activate";
        auto cmd = "bash -c 'source " + env_bin.string();
#endif
        std::string scilens_cmd = "scilens run --collect-depth 1 --export-html-add-index";
        int ret = std::system((cmd + " && " + scilens_cmd + " " + data_test_dir.string() + "'").c_str());
    }
};

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new MyEnvironment);
    return RUN_ALL_TESTS();
}
