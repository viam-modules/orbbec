Powershell.exe -ExecutionPolicy Bypass -Command "& { $$env:ORBBEC_SDK_VERSION='$(ORBBEC_SDK_VERSION)'; $$env:ORBBEC_SDK_DIR='$(ORBBEC_SDK_DIR)'; ./bin/setup.ps1; exit $LASTEXITCODE }"
