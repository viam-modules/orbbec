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

# Add the viam conan remote so viam-cpp-sdk (pinned in conanfile.py) resolves
# from there instead of being cloned and built from source here.
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 --force
if (!$?) { throw "Failed to add viamconan remote" }
