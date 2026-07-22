@echo off
setlocal
cd /d "%~dp0"
if not defined VSCMD_VER call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64
if errorlevel 1 exit /b 1
set "CMAKE_EXE=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA_EXE=C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe"
if not exist "%CMAKE_EXE%" set "CMAKE_EXE=cmake"
if not exist "%NINJA_EXE%" set "NINJA_EXE=ninja"
"%CMAKE_EXE%" -S . -B build-ninja-msvc -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
  -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake
if errorlevel 1 exit /b 1
"%CMAKE_EXE%" --build build-ninja-msvc
if errorlevel 1 exit /b 1
"%CMAKE_EXE%" --build build-ninja-msvc --target test
