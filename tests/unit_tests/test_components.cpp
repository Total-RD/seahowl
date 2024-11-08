
#include <gtest/gtest.h>

int main(int argc, char** argv) {
    /* const char* env_p = std::getenv("SEAHOWL_DATADIR");

    if (env_p == nullptr) {
        if (argc < 2) {
            std::cerr << "Usage: test_01.exe [<datadir>] or set SEAHOWL_DATADIR environment variable" << std::endl;
            return 1;
        } else {
            DATADIR = absolute(path(argv[1]));
        }
    } else {
        DATADIR = absolute(path(env_p));
    } */

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}