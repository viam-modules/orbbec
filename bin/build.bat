set ORBBEC_SDK_DIR=%ORBBEC_SDK_DIR: =%
Powershell.exe -ExecutionPolicy Bypass -NoProfile -Command "& 'bin\build.ps1' exit $LASTEXITCODE"
