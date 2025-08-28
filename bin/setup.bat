set ORBBEC_SDK_VERSION=%ORBBEC_SDK_VERSION: =%
set ORBBEC_SDK_DIR=%ORBBEC_SDK_DIR: =%
Powershell.exe -ExecutionPolicy Bypass -NoProfile -Command "& 'bin\setup.ps1' exit $LASTEXITCODE""
