import os
import re

from conan import ConanFile
from conan.errors import ConanException
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, load

class orbbec(ConanFile):
    name = "viam-orbbec"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/orbbec"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "meta.json", "first_run.sh", "install_udev_rules.sh", "99-obsensor-libusb.rules"

    def set_version(self):
        content = load(self, "CMakeLists.txt")
        self.version = re.search("set\(CMAKE_PROJECT_VERSION (.+)\)", content).group(1).strip()

    def requirements(self):
        # NOTE: If you update the `viam-cpp-sdk` dependency here, it
        # should also be updated in `bin/setup.{sh,ps1}`.
        self.requires("viam-cpp-sdk/0.16.0")

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

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def deploy(self):
        for dir in ["bin", "lib"]:
            copy(self, "*", src=os.path.join(self.package_folder, dir), dst=os.path.join(self.deploy_folder, dir))

        for pat in ["*.sh", "meta.json", "99-obsensor-libusb.rules"]:
            copy(self, pat, src=self.package_folder, dst=self.deploy_folder)
