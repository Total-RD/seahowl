#include "seahowl/io/input_handler.h"
#include "seahowl/io/input_reader.h"
#include "seahowl/io/input_reader_json.h"

namespace fs = std::filesystem;

namespace seahowl {
namespace io {

InputHandler::InputHandler(const std::string& filepath) {
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
