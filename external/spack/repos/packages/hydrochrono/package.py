# Copyright Spack Project Developers. See COPYRIGHT file for details.
#
# SPDX-License-Identifier: (Apache-2.0 OR MIT)

# ----------------------------------------------------------------------------
# If you submit this package back to Spack as a pull request,
# please first remove this boilerplate and all FIXME comments.
#
# This is a template package file for Spack.  We've put "FIXME"
# next to all the things you'll want to change. Once you've handled
# them, you can save this file and test your package like this:
#
#     spack install hydrochrono
#
# You can edit this file again by typing:
#
#     spack edit hydrochrono
#
# See the Spack documentation for more information on packaging.
# ----------------------------------------------------------------------------

import os
from spack_repo.builtin.build_systems.cmake import CMakePackage

from spack.package import *


class Hydrochrono(CMakePackage):
    """FIXME: Put a proper description of your package here."""

    homepage = "https://nrel.github.io/HydroChrono"
    url = "https://github.com/Project-SEA-Stack/HydroChrono/archive/refs/tags/v0.2.8.tar.gz"

    version(
        "0.2.8",
        sha256="e16f6c46d425ed2c4ca6d6c48387eb706ceb60f6387cc4cbf617eb5306da533e",
    )

    depends_on("cxx", type="build")
    depends_on("chrono", type=("build", "link", "run"))
    depends_on("hdf5~mpi+shared+hl+cxx+tools", type=("build", "link", "run"))
    depends_on("zlib", type=("build", "link", "run"))

    def patch(self):
        cmake_lists = "CMakeLists.txt"
        if not os.path.isfile(cmake_lists):
            return

        with open(cmake_lists, "r", encoding="utf-8", errors="ignore") as f:
            data = f.read()

        # Si ZLIB est déjà cherché, ne touche à rien
        if "find_package(ZLIB" in data:
            return

        needle = "find_package(HDF5"
        idx = data.find(needle)
        if idx == -1:
            return

        new = data.replace(needle, "find_package(ZLIB REQUIRED)\n" + needle, 1)

        with open(cmake_lists, "w", encoding="utf-8") as f:
            f.write(new)

    def cmake_args(self):
        args = []
        chrono_prefix = self.spec["chrono"].prefix

        args.append(self.define("CMAKE_PREFIX_PATH", chrono_prefix))

        # Options HydroChrono
        args.append(self.define("HYDROCHRONO_ENABLE_IRRLICHT", False))
        args.append(self.define("HYDROCHRONO_ENABLE_DEMOS", False))
        args.append(self.define("HYDROCHRONO_ENABLE_TESTS", False))

        return args
