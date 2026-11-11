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
else
    echo "Unsupported OS: ${OS}"
    exit 1
fi

# The build images (cpp-sdk-conan-* in CI, cloud-build-base in cloud build) already have conan on
# PATH, so a venv is only needed for local development.
if command -v conan >/dev/null 2>&1; then
  echo "using conan from $(command -v conan)"
else
  # Debian and Ubuntu ship ensurepip separately in pythonX.Y-venv. Install it only when it is
  # missing so systems that already provide a working venv are left untouched.
  if [[ ${OS} == "linux" ]] && ! python3 -c 'import ensurepip' >/dev/null 2>&1; then
    PY_MINOR=$(python3 -c 'import sys; print(sys.version_info.minor)')
    sudo apt-get install -y "python3.${PY_MINOR}-venv"
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

  if [ ! -f "./venv/bin/conan" ]; then
    echo 'installing conan'
    python3 -m pip install conan cmake
  fi
fi

conan profile detect || echo "Conan is already installed"

# viam-cpp-sdk and its dependencies come prebuilt from viamconan, at the versions conan.lock pins.
# It goes first so its binaries win over conancenter's, some of which need a newer glibc than the
# 2.31 cloud build images have.
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 --force
