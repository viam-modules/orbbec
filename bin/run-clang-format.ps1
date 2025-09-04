$clangTidy = $null

if (CommandExists "clang-tidy") {
    $clangTidy = "clang-tidy"
} else {
    Write-Host "clang-tidy not found. Installing LLVM via Chocolatey..."

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

    if (CommandExists "clang-tidy") {
        $clangTidy = "clang-tidy"
    } else {
        Write-Error "ERROR: clang-tidy installation failed"
        exit 1
    }
}

# --- Recursively run clang-tidy on all .cpp and .hpp files in ./src ---
$sourcePath = Join-Path -Path $PSScriptRoot -ChildPath "src"
$files = Get-ChildItem -Path $sourcePath -Recurse -Include *.cpp, *.hpp

foreach ($file in $files) {
    & $clangTidy $file.FullName
}

Write-Host "Done linting all files."
