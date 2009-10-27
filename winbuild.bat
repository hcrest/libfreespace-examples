@echo off

SET RELEASE_VERSION=0.4rc3
SET NAME=libfreespace-examples-%RELEASE_VERSION%.msi

REM This must be run from the Visual Studio 2005 Command Prompt

REM Clean up
erase %NAME%
devenv win\vs2005\examples.sln /clean release

REM Build
devenv win\vs2005\examples.sln /build release

echo Copying installer
copy win\vs2005\release\Installer\libfreespace-examples-installer.msi %NAME%

echo Done.
