@echo off

SET RELEASE_VERSION=0.3rc2
SET WORKDIR=libfreespace-examples
SET ZIPNAME=libfreespace-examples-%RELEASE_VERSION%-win32

REM This must be run from the Visual Studio 2005 Command Prompt
devenv src\examples.sln /clean release
devenv src\examples.sln /build release

rd /S /Q %WORKDIR%
mkdir %WORKDIR%
copy src\release\libfreespace.dll %WORKDIR%
copy src\release\*.exe %WORKDIR%

REM Create ZIP file using Info-ZIP
REM http://www.info-zip.org/Zip.html
.\tools\zip -r %ZIPNAME%.zip %WORKDIR%
