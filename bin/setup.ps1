$ErrorActionPreference = "Stop"

if (-not (Get-Command choco.exe -ErrorAction SilentlyContinue)) {
    Write-Host "Chocolatey is not installed. Please install Chocolatey from https://chocolatey.org/install"
    exit 1
}

choco install -y cmake wget python311 unzip git

# Ensure that things installed with choco are visible to us
Import-Module $env:ChocolateyInstall\helpers\chocolateyProfile.psm1
refreshenv

# Path to virtual environment activate script
$venvActivate = ".\venv\Scripts\Activate.ps1"

# Create or activate virtual environment
if (-not (Test-Path $venvActivate)) {
    Write-Host "Creating and activating virtual environment..."
    python -m venv venv
    & $venvActivate
} else {
    Write-Host "Activating virtual environment..."
    & $venvActivate
}

# Set up Conan
$conanExe = ".\venv\Scripts\conan.exe"
if (-not (Test-Path $conanExe)) {
    Write-Host "Installing Conan..."
    python -m pip install conan
} else {
    Write-Host "Conan already installed."
}

# Initialize conan if it hasn't been already
conan profile detect
if (!$?) { Write-Host "Conan is already installed" }

# Clone the C++ SDK repo
mkdir tmp_cpp_sdk
Push-Location tmp_cpp_sdk
git clone https://github.com/viamrobotics/viam-cpp-sdk.git
Push-Location viam-cpp-sdk

# NOTE: If you change this version, also change it in the `conanfile.py` requirements
# and in dockerfile
git checkout releases/v0.16.0

# Build the C++ SDK repo.
#
# We want a static binary, so we turn off shared. Elect for C++17
# compilation, since it seems some of the dependencies we pick mandate
# it anyway. Pin to the Windows 10 1809 associated windows SDK, and
# opt for the static compiler runtime so we don't have a dependency on
# the VC redistributable.
#
# TODO: Note `-tf ""`, which disables the self test. I have not been
# able to get this working on windows.
# boost backtrace contains a unix only library dlfcn.h
conan create . `
      --build=missing `
      -o:a "&:shared=False" `
      -o:a "boost/*:with_stacktrace_backtrace=False" `
      -o:a "boost/*:without_stacktrace=True" `
      -s:a build_type=Release `
      -s:a compiler.cppstd=17 `
      -tf `"`"

Pop-Location  # viam-cpp-sdk
Pop-Location  # tmp_cpp_sdk
Remove-Item -Recurse -Force tmp_cpp_sdk
