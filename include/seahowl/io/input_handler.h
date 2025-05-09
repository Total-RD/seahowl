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
    InputHandler(const std::string& filepath);
    // Destructor
    ~InputHandler() = default;
};

}  // namespace io
}  // namespace seahowl
