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
#     spack install chrono
#
# You can edit this file again by typing:
#
#     spack edit chrono
#
# See the Spack documentation for more information on packaging.
# ----------------------------------------------------------------------------

from spack_repo.builtin.build_systems.cmake import CMakePackage

from spack.package import *


class Chrono(CMakePackage):
    """FIXME: Put a proper description of your package here."""

    homepage = "https://projectchrono.org"
    url = "https://github.com/projectchrono/chrono/archive/refs/tags/9.0.1.tar.gz"

    license("UNKNOWN", checked_by="github_user1")

    version("9.0.1", sha256="5e85f7dcdad63e21323388082c298884f7b6859b")

    depends_on("c", type="build")
    depends_on("cxx", type="build")

    def cmake_args(self):
        args = [
            self.define("BUILD_TESTING", True),
            self.define("BUILD_DEMOS", False),
            self.define("BUILD_BENCHMARKING", False),
            self.define("ENABLE_HDF5", False),
            self.define("ENABLE_MODULE_POSTPROCESS", False),
            self.define("ENABLE_MODULE_PYTHON", False),
            self.define("ENABLE_MODULE_IRRLICHT", True),
            self.define("ENABLE_MODULE_VEHICLE", False),
            self.define("ENABLE_MODULE_MULTICORE", False),
            self.define("ENABLE_MODULE_OPENGL", False),
            self.define("ENABLE_MODULE_SYNCHRONO", False),
            self.define("ENABLE_MODULE_CSHARP", False),
            self.define("ENABLE_MODULE_COSIMULATION", False),
        ]

        return args
