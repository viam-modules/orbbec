$ErrorActionPreference = "Stop"

if (-not (Get-Command choco.exe -ErrorAction SilentlyContinue)) {
    Write-Host "Chocolatey is not installed. Please install Chocolatey from https://chocolatey.org/install"
    exit 1
}


# Install required tools using Chocolatey
$packages = @(
    "cmake",
    "python --version=3.11.0",  # ensures Python 3.11
    "wget",
    "unzip"
)

choco install -y cmake wget python311 unzip git

# Ensure that things installed with choco are visible to us
Import-Module $env:ChocolateyInstall\helpers\chocolateyProfile.psm1
refreshenv

# Initialize conan if it hasn't been already
conan profile detect
if (!$?) { Write-Host "Conan is already installed" }


# Remove previous SDK files if they exist
if (Test-Path "$env:ORBBEC_SDK_DIR.zip") {
    Remove-Item -Force -Recurse "$env:ORBBEC_SDK_DIR.zip"
}

if (Test-Path "$env:ORBBEC_SDK_DIR") {
    Remove-Item -Force -Recurse "$env:ORBBEC_SDK_DIR"
}

# Download the SDK zip
$downloadUrl = "https://github.com/orbbec/OrbbecSDK_v2/releases/download/$env:ORBBEC_SDK_VERSION/$env:ORBBEC_SDK_DIR.zip"
Write-Host "Downloading Orbbec SDK from $downloadUrl"
Invoke-WebRequest -Uri $downloadUrl -OutFile "$env:ORBBEC_SDK_DIR.zip"

Write-Host "Extracting Windows SDK zip to folder $env:ORBBEC_SDK_DIR"
Expand-Archive -Path "$env:ORBBEC_SDK_DIR.zip" -DestinationPath "$env:ORBBEC_SDK_DIR" -Force


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

# Clone the C++ SDK repo
mkdir tmp_cpp_sdk
Push-Location tmp_cpp_sdk
git clone https://github.com/viamrobotics/viam-cpp-sdk.git
Push-Location viam-cpp-sdk

# NOTE: If you change this version, also change it in the `conanfile.py` requirements
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
conan create . `
      --build=missing `
      -o:a "&:shared=False" `
      -s:a build_type=Release `
      -s:a compiler.cppstd=17 `
      -c:a tools.microsoft:winsdk_version=10.0.17763.0 `
      -s:a compiler.runtime=static `
      -tf `"`

Pop-Location  # viam-cpp-sdk
Pop-Location  # tmp_cpp_sdk
Remove-Item -Recurse -Force tmp_cpp_sdk
