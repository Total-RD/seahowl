# Contributing to SEAHOWL

Thank you for your interest in contributing to SEAHOWL. This document outlines the best practices and guidelines for contributing to this repository.

## Table of Contents

- [Git Workflow](#git-workflow)
- [Code Style](#code-style)

## Git Workflow


### Branch Merging Strategy

- **`main`**: Contains only release versions. Direct commits to `main` are not allowed.
- **`dev`**: Active development branch. All Pull Requests should target this branch.
- **Feature branches**: Create branches from `dev` for your work (e.g., `feature/add-wave-model`, `fix/morison-coefficient`).

```
main (releases only)
  │
  └── dev (development branch)
        │
        ├── feature/your-feature
        ├── fix/your-bugfix
        └── ...
```

### Submitting Pull Requests

1. Fork or clone the repository
2. Create a feature branch from `dev`
3. Make your changes following the guidelines below
4. Run pre-commit hooks and tests locally
5. Submit a Pull Request to the `dev` branch via GitHub


### Pre-Commit Hook

A pre-commit hook is used in this repository to handle formatting automatically when commiting files through Git. Pre-commit is installed as follows (only needs to run once on a fresh clone):
```bash
pip install pre-commit
pre-commit install
```

If some files are automatically by pre-commit reformatted when you try to commit your changes, you can directly stage them then again (`git add ...`) and commit again for the changes to pass pre-commit requirements. Workflow example:

```bash
$ git add -u
$ git commit -m "My commit message"

Trim Trailing Whitespace.................................................Passed
Fix End of Files.........................................................Passed
Check JSON...........................................(no files to check)Skipped
Check for added large files..............................................Passed
Pretty format JSON...................................(no files to check)Skipped
clang-format.............................................................Failed
- hook id: clang-format
- files were modified by this hook
black................................................(no files to check)Skipped

$ git add -u  # stage again the automatically formatted files
$ git commit -m "My commit message"  # commit again

Trim Trailing Whitespace.................................................Passed
Fix End of Files.........................................................Passed
Check JSON...........................................(no files to check)Skipped
Check for added large files..............................................Passed
Pretty format JSON...................................(no files to check)Skipped
clang-format.............................................................Passed
black................................................(no files to check)Skipped
[feature/my_nice_feature 60212a3] My commit message
 Date: Wed Jan 21 11:59:17 2026 +0100
 14 files changed, 423 insertions(+), 178 deletions(-)
 ```

SEAHOWL uses **Chromium-based style** (see `.clang-format`) for C++, Python code is formatted with **Black**, and JSON files are formatted with 2-space indentation and no key sorting (preserve logical order). This is all automatically handled by pre-commit, on top of removing trailing whitespaces, checking for large files, and fixing end of files whitespace.


## Code Style

### Naming Conventions

Due the heavy use of Python bindings when developing a custom SEAHOWL case (i.e. not via the binary driver), naming convention follows the PEP8 guideline when possible, as follows:

- **Classes**: PascalCase (`BladeElastoFEA`, `MorisonNode`)
- **Methods/Functions**: snake_case (`compute_loads`, `get_velocity`)
- **Member variables**: snake_case (`drag_coefficient`, `is_active`)
- **Constants**: UPPER_SNAKE_CASE or snake_case depending on context
- **Namespaces**: lowercase (`seahowl`, `aero`, `elasto`)


### Documentation Requirements

For docstrings, use Doxygen-style documentation with the following format:

```cpp
/**
* @brief Short description of the class.
*
* Longer description if needed, explaining behavior, assumptions,
* or implementation details.
*/
class MyClass {
  public:
    /** @brief Descrition of my class variable 1. */
    double my_var1 = 0.0;
    /** @brief Descrition of my class variable 2. */
    bool my_var2 = false;

    /**
    * @brief Short description of the class/function.
    *
    * Longer description if needed, explaining behavior, assumptions,
    * or implementation details.
    *
    * @param[in] my_arg1 Description of argument 1.
    * @param[in] my_arg2 Description of argument 2.
    * @param[out] my_arg3 Description of argument 3.
    * @return Description of the returned value.
    */
    float my_function(double my_arg1, bool my_arg2, Vector3d& my_arg);


  private:
    /** @brief Descrition of my private variable. */
    double my_private_var = 0.0;
};
```


### Headers

#### External Dependencies

When possible, include statements to external libraries should not be placed in header files (.h) of SEAHOWL. Limit them to implementation files (.cpp) to ensures that external headers do not need to be carried with the compiled library.
For example, all Chrono related includes are contained in `src/elasto/chrono_adapters.cpp` with only forward declarations in `include/seahowl/elasto/chrono_adapters.h` when necessary. The same applies to HydroChrono with its includes only contained in `src/fluid/hydro/hydrochrono_adapters.cpp`.

#### Include Order

Organize includes in logical groups (not alphabetically sorted).
Use quotation marks (`#include "our/header.h"`) for include steatments from the source code of SEAHOWL itself and chevron (`#include <their/header.h>`) for stabdard library and external dependencies. We use `#pragma once` instead of traditional include guards as well. For example:

```cpp
#pragma once

#include "seahowl/core/component.h"      // Project headers first
#include "seahowl/commons/numerics.h"

#include <memory>                         // Standard library
#include <vector>

#include <Eigen/Dense>                    // Third-party libraries
```


#### Forward Declarations

Use forward declarations to minimize header dependencies as much as possible to reduce compile time when recompiling for modifications:

```cpp
// Forward declarations
namespace seahowl {
namespace elasto {
class BladeElasto;
}  // namespace elasto
}  // namespace seahowl

// Then use pointers/references in the class
class Blade {
    seahowl::elasto::BladeElasto& elasto;
};
```
