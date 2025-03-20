#include <vector>
#include <string>
#include <memory>
#include "seahowl/fluid/aero/rotor_aero.h"
#include "seahowl/core/turbine.h"

/**
 * @brief Returns rotor disk performance tables (Cp, Cq, Ct).
 *
 * @param[in] filepath Path of the .txt file describing the rotor disk performance.
 * @param[in] aero Object of class seahowl::aero::RotorAero
 */
void get_disk_perf_from_table(std::string filepath, seahowl::aero::RotorAeroDisk& aero);
