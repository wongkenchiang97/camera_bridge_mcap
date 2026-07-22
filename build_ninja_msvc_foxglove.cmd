@echo off
setlocal
cd /d "%~dp0"
if not defined VSCMD_VER call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64
if errorlevel 1 exit /b 1
set "CMAKE_EXE=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA_EXE=C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe"
"%CMAKE_EXE%" -S . -B build-ninja-msvc-foxglove -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
  -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake ^
  -DCAMERA_BRIDGE_MCAP_WITH_FOXGLOVE=ON ^
  -DFOXGLOVE_SDK_ROOT=../foxglove-sdk
if errorlevel 1 exit /b 1
"%CMAKE_EXE%" --build build-ninja-msvc-foxglove
if errorlevel 1 exit /b 1
"%CMAKE_EXE%" --build build-ninja-msvc-foxglove --target test
