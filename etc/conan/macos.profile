# Toolchain of record for the darwin builds. Copied from viam-cpp-sdk's etc/conan/macos.profile,
# which viamconan's darwin binaries are built with, so our package IDs match them. Keep in sync
# with the SDK and with the macos-26 runner in the workflows.
include(default)

# macOS 14 Sonoma
[settings]
os=Macos
os.version=14.0
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
compiler.cppstd=17
build_type=Release
