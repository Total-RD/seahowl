#include <gtest/gtest.h>
#include "tools/get_env_var.h"
#include <filesystem>
#include <spdlog/spdlog.h>
#include <cstdlib>
#include <string>

class MyEnvironment : public ::testing::Environment {
  public:
    void TearDown() override {
        // set test directories roots
        auto test_dir = get_test_dir();
        auto data_test_dir = test_dir / "unit_tests/data";
        std::string scilens_cmd = "scilens --log-level ERROR run --collect-depth 1 --export-html-add-index";

#ifdef HAVE_PYTHON
    #ifdef _WIN32
            // Code spécifique Windows
        #ifdef HAVE_ENV_PYTHON
        auto env_bin = test_dir / "../build/.venv/Scripts/activate.bat";
        auto cmd = "cmd /C \" " + env_bin.string() + " && ";
        #else
        auto cmd = std::string("");
        #endif
        // test if scilens is available
        int ret = std::system((cmd + "scilens version >nul 2>&1").c_str());
        if (ret != 0) {
            spdlog::warn("scilens is not available in the environment.");
            return;
        }
        ret = std::system((cmd + scilens_cmd + " " + data_test_dir.string()).c_str());
    #elif defined(__linux__)
            // Code spécifique Linux
        #ifdef HAVE_ENV_PYTHON
        auto env_bin = test_dir / "../build/.venv/bin/activate";
        auto cmd = "bash -c 'source " + env_bin.string() + " && ";
        #else
        auto cmd = std::string("'");
        #endif
        // test if scilens is available
        int ret = std::system((cmd + "scilens version ' > /dev/null 2>&1").c_str());
        if (ret != 0) {
            spdlog::warn("scilens is not available in the environment.");
            return;
        }
        ret = std::system((cmd + scilens_cmd + " " + data_test_dir.string() + "'").c_str());
    #endif
#endif
    }
};

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new MyEnvironment);
    return RUN_ALL_TESTS();
}
