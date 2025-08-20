#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

if [[ ${OS} == "darwin" ]]; then
    echo "Detected MacOS ${ARCH}"

  if ! command -v brew >/dev/null; then
     echo "Brew not installed. Please install brew!"
     exit 1
  fi
  # Install required tools
  brew install cmake python@3.11 wget unzip || true
elif  [[ ${OS} == "linux" ]]; then
    echo "Detected Linux $ARCH"
    # NOTE: this is written under the assumption that it will be built in canon
    sudo apt -y update && sudo apt -y upgrade && sudo apt install -y cmake python3.11 python3.11-venv wget
else
    echo "Unsupported OS: ${OS}"
    exit 1
fi


rm -rf ${ORBBEC_SDK_DIR}.zip
rm -rf ${ORBBEC_SDK_DIR}
wget https://github.com/orbbec/OrbbecSDK_v2/releases/download/${ORBBEC_SDK_VERSION}/${ORBBEC_SDK_DIR}.zip
unzip ${ORBBEC_SDK_DIR}.zip

# MacOS binary has a different top level dir name than the zip file name
if [[ ${TARGET_OS} == "darwin" ]]; then
TOPDIR=$(ls -d *macOS*/ | head -1 | sed 's#/##')
mv "${TOPDIR}" "${ORBBEC_SDK_DIR}"
fi

# windows binary does not have a top level dir, create one
if [[ ${TARGET_OS} == "windows" ]]; then
TOPDIR=$(ls -d *macOS*/ | head -1 | sed 's#/##')
mv "${TOPDIR}" "${ORBBEC_SDK_DIR}"
fi

# lsusb rules only on linux
if [[ ${TARGET_OS} == "linux" ]]; then
  cp ${ORBBEC_SDK_DIR}/shared/99-obsensor-libusb.rules .
fi


if [ ! -f "./venv/bin/activate" ]; then
  echo 'creating and sourcing virtual env'
  python3 -m venv venv && source ./venv/bin/activate
else
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi

# Set up conan
if [ ! -f "./venv/bin/conan" ]; then
  echo 'installing conan'
  python3 -m pip install conan
fi

conan profile detect || echo "Conan is already installed"

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

# NOTE: If you change this version, also change it in the `conanfile.py` requirements
git checkout releases/v0.16.0

# Build the C++ SDK repo
#
# We want a static binary, so we turn off shared. Elect for C++17
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.
conan create . \
      --build=missing \
      -o:a "&:shared=False" \
      -s:a build_type=Release \
      -s:a compiler.cppstd=17

# Cleanup
popd  # viam-cpp-sdk
popd  # tmp_cpp_sdk
rm -rf tmp_cpp_sdk
