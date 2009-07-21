@echo off

SET RELEASE_VERSION=0.4
SET WORKDIR=libfreespace-examples
SET ZIPNAME=libfreespace-examples-%RELEASE_VERSION%-win32

REM This must be run from the Visual Studio 2005 Command Prompt
devenv src\examples.sln /clean release
devenv src\examples.sln /build release

echo Cleanup any previous work directories
rd /S /Q %WORKDIR%
echo Creating work directory
mkdir %WORKDIR%
echo Copying build products
copy src\release\libfreespace.dll %WORKDIR%
copy src\release\*.exe %WORKDIR%

REM Create ZIP file using Info-ZIP
REM http://www.info-zip.org/Zip.html
echo Zipping - be sure to have zip.exe in your path or you will get an error below
zip -r %ZIPNAME%.zip %WORKDIR%
echo Done.
