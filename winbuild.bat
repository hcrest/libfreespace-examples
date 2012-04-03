@echo off

SET RELEASE_VERSION=0.6
SET FS_INSTALLER=libfreespace-examples-%RELEASE_VERSION%.msi

REM Check if the VCINSTALLDIR variable has been set.
REM If it has not been set... try to find the right version of Visual Studio to use.
set VS_BATCH_FILE=vsvars32.bat

if "%VCINSTALLDIR%"=="" (
echo User has not run %VS_BATCH_FILE%, run automatically if available
if not "%VS100COMNTOOLS%"=="" (
call "%VS100COMNTOOLS%\%VS_BATCH_FILE%"
) else if not "%VS90COMNTOOLS%"=="" (
call "%VS90COMNTOOLS%\%VS_BATCH_FILE%"
) else if not "%VS80COMNTOOLS%"=="" (
call "%VS80COMNTOOLS%\%VS_BATCH_FILE%"
) else (
echo Unable to find appropriate Visual Studio version based on VSxxxCOMNTOOLS variable.  
echo Try running %VS_BATCH_FILE%
exit /b 1
)
)

REM Determine the Visual Studio version based on VCINSTALLDIR variable
for /F "usebackq delims=\ tokens=3" %%i in (`set VCINSTALLDIR`) do set VS_VERSION_STR=%%i
if "%VS_VERSION_STR%"=="Microsoft Visual Studio 10.0" (
set VS_VERSION=vs2010
) else if "%VS_VERSION_STR%"=="Microsoft Visual Studio 9.0" (
set VS_VERSION=vs2008
) else if "%VS_VERSION_STR%"=="Microsoft Visual Studio 8" (
set VS_VERSION=vs2005
) else (
echo Unable to determine Visual Studio version based on VCINSTALLDIR variable.  
echo Try running %VS_BATCH_FILE%
exit /b 1
)
echo Using Visual Studio Version: %VS_VERSION%

REM This must be run from the Visual Studio Command Prompt

REM Clean up
if exist %FS_INSTALLER% (del %FS_INSTALLER%)
devenv win\%VS_VERSION%\examples.sln /clean "Release|Win32"

REM Build
devenv win\%VS_VERSION%\examples.sln /build "Release|Win32"

echo Copying installer
copy win\%VS_VERSION%\release\Installer\libfreespace-examples-installer.msi %FS_INSTALLER%

echo Done.
