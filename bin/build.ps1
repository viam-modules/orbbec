$ErrorActionPreference = "Stop"


# remove any prior build
Remove-Item -Recurse -Force build-conan -ErrorAction SilentlyContinue

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

conan install . `
      --build=missing `
      -o:a "&:shared=False" `
      -s:a build_type=Release `
      -s:a compiler.cppstd=17 `

conan build . `
      --output-folder=build-conan `
      --build=none `
      -o:a "&:shared=False" `
      -s:a build_type=Release `
      -s:a "&:build_type=RelWithDebInfo" `
      -s:a compiler.cppstd=17 `

# # Rename the executable to remove .exe extension
# $exePath = "build-conan\build\RelWithDebInfo\orbbec-module.exe"
# if (Test-Path $exePath) {
#     Copy-Item $exePath ($exePath -replace '\.exe$', '')
#     Remove-Item $exePath
# }


