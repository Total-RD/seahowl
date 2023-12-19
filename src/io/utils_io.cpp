#include "seahowl/io/utils_io.h"

#include <filesystem>

namespace fs = std::filesystem;

void check_file_exists(const std::string& filepath) {
    if (!fs::exists(filepath)) {
        throw std::runtime_error("File \"" + filepath + "\" does not exist.");
    }
}

std::string copy_file_and_increment(const std::string& filepath, const std::string& destination_dir) {
    if (!fs::exists(destination_dir)) {
        fs::create_directories(destination_dir);
    }
    fs::path pfilepath = fs::path(filepath);
    fs::path filecopypath;
    if (fs::exists(destination_dir / pfilepath.filename())) {
        auto filename = pfilepath.stem().generic_string();
        auto fileext = pfilepath.extension().generic_string();
        bool copied = false;
        int file_idx = 0;
        while (!copied) {
            filecopypath = fs::path(destination_dir) / (filename + std::to_string(file_idx) + fileext);
            if (!fs::exists(filecopypath)) {
                fs::copy(pfilepath, filecopypath);
                pfilepath = filecopypath;
                copied = true;
            } else {
                file_idx += 1;
            }
        }
    } else {
        filecopypath = destination_dir / pfilepath.filename();
        fs::copy(pfilepath, filecopypath);
    }
    return filecopypath.generic_string();
}
