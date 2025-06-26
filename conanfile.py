import os
from io import StringIO
import re

from conan import ConanFile
from conan.errors import ConanException
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.build import can_run
from conan.tools.files import copy, load

class orbbec(ConanFile):
    name = "viam-orbbec"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/orbbec" 
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "shared": [True, False]
    }
    default_options = {
        "shared": True
    }

    exports_sources = "CMakeLists.txt", "LICENSE", "src/*"

    def set_version(self):
        content = load(self, "CMakeLists.txt")
        self.version = re.search("set\(CMAKE_PROJECT_VERSION (.+)\)", content).group(1).strip()

    def configure(self):
        # If we're building static then build the world as static, otherwise
        # stuff will probably break.
        # If you want your shared build to also build the world as shared, you
        # can invoke conan with -o "&:shared=False" -o "*:shared=False",
        # possibly with --build=missing or --build=cascade as desired,
        # but this is probably not necessary.
        if not self.options.shared:
            self.options["*"].shared = False

    def requirements(self):
        # NOTE: If you update the `viam-cpp-sdk` dependency here, it
        # should also be updated in `bin/setup.{sh,ps1}`.
        self.requires("viam-cpp-sdk/0.14.0")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def layout(self):
        cmake_layout(self, src_folder=".")
