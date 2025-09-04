Write-Host "Installing LLVM via Chocolatey..."

# Check for Chocolatey
if (-not (Test-Path "$env:ChocolateyInstall\bin\choco.exe")) {
    Write-Error "Chocolatey not found. Please install it first."
    exit 1
}

# Install LLVM (includes clang-tidy)
& "$env:ChocolateyInstall\bin\choco.exe" install -y llvm

# Ensure things installed via chocolatey are visible.
Import-Module "$env:ChocolateyInstall\helpers\chocolateyProfile.psm1"
refreshenv

$sourcePath = Join-Path -Path $PSScriptRoot -ChildPath "src"
$files = Get-ChildItem -Path $sourcePath -Recurse -Include *.cpp, *.hpp

foreach ($file in $files) {
    & clang-tidy $file.FullName
}

Write-Host "Done linting all files."
