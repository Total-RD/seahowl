# Dependency install

# Using vcpkg with a Custom Portfile

This README provides a step-by-step guide to using `vcpkg` with a generic setup and a custom `portfile.cmake` to define your library dependencies.

## Prerequisites

1. **Install vcpkg**:
   - Clone the `vcpkg` repository:
     ```bash
     git clone https://github.com/microsoft/vcpkg.git
     cd vcpkg
     ```
   - Bootstrap the `vcpkg` executable:
     ```bash
     ./bootstrap-vcpkg.sh
     ```

2. **Setup Environment** :
   - Add `vcpkg` to your environment path for easier access (assuming you are inside the vcpkg folder):
     ```bash
     export VCPKG_ROOT==$(pwd)
     export PATH=\$PATH:=$(pwd)
     ```
   - Add it also to your .bashrc to have it automatically loaded when opening a new session
     ```bash
       echo "export VCPKG_ROOT=$(pwd)" >> ~/.bashrc
       echo "export PATH=\$PATH:$(pwd)" >> ~/.bashrc
     ```


## Directory Structure

Your project directory is structured as follows:

```
SEAHOWL/
├─ cmake/
├─ ...
├─ external/
   ├─ bash/
   ├─ vcpkg/
      ├─ ports/
         ├─ my-library/
            ├─ portfile.cmake
            ├─ vcpkg.json
         ├─ other-library/
            ├─ portfile.cmake
            ├─ vcpkg.json
│     ├─ vcpkg-configuration.json
│     ├─ vcpkg.json
├─ ...
├─ CMakeLists.txt
```
## Add library in vcpkg registry

Search library in https://vcpkg.io/en/packages and add this to the `vcpkg.json` in your root directory:

   ```
   {
   "dependencies": [
      {
         "name": "vcpkg-cmake",
         "host": true
      },
      {
         "name": "vcpkg-cmake-config",
         "host": true
      },
      {
         "name": "eigen3",
         "version>=": "3.4.0"
      },
      {
         "name": "nlohmann-json",
         "version>=": "3.11.3"
      },
      {
         "name": "spdlog",
         "version>=": "1.15.0"
      },
      {
         "name": "gtest",
         "version>=": "1.15.2"
      },
      {
         "name": "chrono",
         "version>=": "8.0.0"
      },
      {
         "name": "hydrochrono",
         "version>=": "0.2.3"
      },
      {
         "name": "openfast",
         "version>=": "3.5.2"
      }
   ]
   }
   ```

## Writing the Portfile for a specific library

1. Create a custom port directory inside `ports/`.
   For example: `ports/my-library/`.

2. Write the `portfile.cmake` file to specify how to build and install your library.
   Example:
   ```
   vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://my-library/my-repo.git
    REF <commit-or-release-sha>
   )
   vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
      -DXXX_XXX=XX
   )
   vcpkg_install_cmake()
   vcpkg_cmake_config_fixup()
   ```
2. Write the `vcpkg.json` file to specify your library.
   Example:
   ```
   {
      "name": "my-library",
      "version-string": "x.x.x",
      "description": "",
      "homepage": "https://my-library",
      "dependencies": [
         {
            "name": "dependency",
            "version": "x.x.x"
         }
      ]
   }
   ```
