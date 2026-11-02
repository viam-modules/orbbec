#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

OS=$(uname -s | tr '[:upper:]' '[:lower:]')

if [[ ${OS} == "darwin" ]]; then
    echo "Detected MacOS"

  if ! command -v brew >/dev/null; then
     echo "Brew not installed. Please install brew!"
     exit 1
  fi
  # Install required tools
  brew install cmake python@3.11 wget unzip || true
elif  [[ ${OS} == "linux" ]]; then
    echo "Detected Linux"
    # python3 (>= 3.11) and CMake (>= 3.25, installed into the venv below) come from the
    # build image; apt's versions on focal and bullseye are too old for viam-cpp-sdk.
    sudo apt-get update && sudo apt-get install -y wget
    # Debian and Ubuntu ship ensurepip separately in pythonX.Y-venv, which some build images
    # (e.g. rdk-devenv, used by the publish workflow) do not carry. Install it only when it is
    # missing so images that already provide a working venv are left untouched.
    if ! python3 -c 'import ensurepip' >/dev/null 2>&1; then
        PY_MINOR=$(python3 -c 'import sys; print(sys.version_info.minor)')
        sudo apt-get install -y "python3.${PY_MINOR}-venv"
    fi
else
    echo "Unsupported OS: ${OS}"
    exit 1
fi

if [ ! -f "./venv/bin/activate" ]; then
  echo 'creating and sourcing virtual env'
  # Kept as separate commands: under `set -e` a failure on the left of `&&` does not stop the
  # script, which previously let a failed venv fall through to pip on the system python.
  python3 -m venv venv
  source ./venv/bin/activate
else
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi

# Set up conan
if [ ! -f "./venv/bin/conan" ]; then
  echo 'installing conan'
  python3 -m pip install conan cmake
fi

conan profile detect || echo "Conan is already installed"

# Build the viam-cpp-sdk version that conanfile.py requires. Reading it from there means the
# automated SDK bumps, which only edit conanfile.py, can't leave this script on an older version.
SDK_VERSION=$(sed -nE 's/.*requires\("viam-cpp-sdk\/([^"]+)"\).*/\1/p' conanfile.py)
if [[ ! "${SDK_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Could not read a single viam-cpp-sdk version from conanfile.py (got: '${SDK_VERSION}')"
  exit 1
fi

if [ ! -d "tmp_cpp_sdk/viam-cpp-sdk" ]; then
  # Clone the C++ SDK repo
  mkdir -p tmp_cpp_sdk
  pushd tmp_cpp_sdk
  git clone https://github.com/viamrobotics/viam-cpp-sdk.git
  pushd viam-cpp-sdk
else
  pushd tmp_cpp_sdk
  pushd viam-cpp-sdk
fi

git checkout "releases/v${SDK_VERSION}"

# Build the C++ SDK repo
#
# We want a static binary, so we turn off shared. Elect for C++17
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.
conan create . \
      --build=missing \
      --build="b2/*" \
      -o:a "&:shared=False" \
      -s:a build_type=Release \
      -s:a compiler.cppstd=17

# Cleanup
popd  # viam-cpp-sdk
popd  # tmp_cpp_sdk
rm -rf tmp_cpp_sdk
