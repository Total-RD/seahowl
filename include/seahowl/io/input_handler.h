#pragma once

#include <seahowl/io/input_reader.h>
#include <string>
#include <memory>

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
