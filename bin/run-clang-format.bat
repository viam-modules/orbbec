SET "CLANG_FORMAT="

REM Check if clang-format-19 exists
where clang-format-19 >nul 2>&1
IF %ERRORLEVEL%==0 (
    SET "CLANG_FORMAT=clang-format-19"
) ELSE (
    REM Check if clang-format exists
    where clang-format >nul 2>&1
    IF %ERRORLEVEL%==0 (
        SET "CLANG_FORMAT=clang-format"
    ) ELSE (
        echo Installing clang-format...

        REM Check for Chocolatey
        IF DEFINED ChocolateyInstall (
            echo Installing LLVM via Chocolatey...
            choco install -y llvm
        ) ELSE (
            echo ERROR: Chocolatey not found. Please install LLVM clang-format manually.
            exit /b 1
        )

        REM Re-check after installation
        where clang-format-19 >nul 2>&1
        IF %ERRORLEVEL%==0 (
            SET "CLANG_FORMAT=clang-format-19"
        ) ELSE (
            where clang-format >nul 2>&1
            IF %ERRORLEVEL%==0 (
                SET "CLANG_FORMAT=clang-format"
            ) ELSE (
                echo ERROR: clang-format installation failed
                exit /b 1
            )
        )
    )
)

REM --- Format all .cpp and .hpp files under ./src ---
FOR /R "%CD%\src" %%F IN (*.cpp *.hpp) DO (
    "%CLANG_FORMAT%" -i --style=file "%%F"
)
