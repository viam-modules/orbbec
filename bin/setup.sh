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
    # NOTE: this is written under the assumption that it will be built in canon
    sudo apt -y update && sudo apt -y upgrade && sudo apt install -y cmake python3.11 python3.11-venv wget
else
    echo "Unsupported OS: ${OS}"
    exit 1
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

# Add the viam conan remote so viam-cpp-sdk (pinned in conanfile.py) resolves
# from there instead of being cloned and built from source here.
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 --force
