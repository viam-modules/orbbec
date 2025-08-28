@echo on
set ORBBEC_SDK_VERSION=%ORBBEC_SDK_VERSION: =%
set ORBBEC_SDK_DIR=%ORBBEC_SDK_DIR: =%

Powershell.exe -ExecutionPolicy Bypass -NoProfile -Command "& {
    $ErrorActionPreference = 'Stop'
    $env:ORBBEC_SDK_VERSION = '%ORBBEC_SDK_VERSION%'
    $env:ORBBEC_SDK_DIR = '%ORBBEC_SDK_DIR%'
    
    Write-Host 'Debug: ORBBEC_SDK_VERSION=' $env:ORBBEC_SDK_VERSION
    Write-Host 'Debug: ORBBEC_SDK_DIR=' $env:ORBBEC_SDK_DIR
    
    try {
        & 'bin\setup.ps1'
        if ($LASTEXITCODE -ne 0) {
            Write-Host 'setup.ps1 failed with exit code:' $LASTEXITCODE
            exit $LASTEXITCODE
        }
    } catch {
        Write-Host 'Error executing setup.ps1:' $_.Exception.Message
        exit 1
    }
}"

if errorlevel 1 (
    echo Setup failed with error level %errorlevel%
    exit /b %errorlevel%
)