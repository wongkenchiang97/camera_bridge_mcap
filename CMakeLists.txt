cmake_minimum_required(VERSION 3.20)
project(camera_bridge_mcap VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CAMERA_BRIDGE_MCAP_BUILD_TESTING "Build camera_bridge_mcap tests" ON)
option(CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE "Enable optional live Foxglove WebSocket sink" OFF)
set(FOXGLOVE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../foxglove-sdk"
    CACHE PATH "Foxglove SDK root containing include and lib")
set(CAMERA_BRIDGE_CORE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../camera_bridge_core"
    CACHE PATH "Path to camera_bridge_core")

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(unofficial-mcap CONFIG REQUIRED)

if(NOT TARGET camera_bridge::core)
  if(EXISTS "${CAMERA_BRIDGE_CORE_SOURCE_DIR}/CMakeLists.txt")
    add_subdirectory("${CAMERA_BRIDGE_CORE_SOURCE_DIR}"
                     "${CMAKE_CURRENT_BINARY_DIR}/camera_bridge_core")
  else()
    find_package(camera_bridge_core CONFIG REQUIRED)
  endif()
endif()

add_library(camera_bridge_mcap STATIC
  src/mcap_vendor.cpp
  src/cdr.cpp
  src/ros2_messages.cpp
  src/recorder.cpp
  src/producer.cpp
  src/live_sink.cpp
)
add_library(camera_bridge::mcap ALIAS camera_bridge_mcap)

target_include_directories(camera_bridge_mcap PUBLIC include)
target_link_libraries(camera_bridge_mcap
  PUBLIC camera_bridge::core ${OpenCV_LIBS}
  PRIVATE unofficial::mcap::mcap)

if(CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE)
  find_library(FOXGLOVE_LIBRARY NAMES foxglove
    PATHS "${FOXGLOVE_SDK_ROOT}/lib" "${FOXGLOVE_SDK_ROOT}/lib64"
    NO_DEFAULT_PATH REQUIRED)

  # The distributed foxglove library contains the C ABI. Its public C++ API is
  # provided as thin wrapper sources, so build only the wrappers needed by the
  # WebSocket sink. In particular, do not compile the SDK's mcap.cpp: this
  # project owns its MCAP implementation through the official mcap package.
  add_library(camera_bridge_foxglove_cpp STATIC
    "${FOXGLOVE_SDK_ROOT}/src/channel.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/context.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/error.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/foxglove.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/schemas.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/server.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/server/connection_graph.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/server/fetch_asset.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/server/parameter.cpp"
    "${FOXGLOVE_SDK_ROOT}/src/server/service.cpp"
  )
  target_include_directories(camera_bridge_foxglove_cpp
    PUBLIC "${FOXGLOVE_SDK_ROOT}/include")
  target_link_libraries(camera_bridge_foxglove_cpp PUBLIC "${FOXGLOVE_LIBRARY}")
  if(WIN32)
    target_link_libraries(camera_bridge_foxglove_cpp PUBLIC
      ws2_32 crypt32 ntdll userenv secur32 bcrypt iphlpapi)
    # The Windows Foxglove static archive vendors zstd while the official MCAP
    # target also links zstd. Both provide the same C ABI; allow the linker to
    # retain the first definition instead of rejecting the optional combined
    # build. This is scoped to Foxglove-enabled Windows consumers only.
    target_link_options(camera_bridge_foxglove_cpp PUBLIC /FORCE:MULTIPLE)
  endif()

  target_sources(camera_bridge_mcap PRIVATE src/foxglove_live_sink.cpp)
  target_include_directories(camera_bridge_mcap PRIVATE "${FOXGLOVE_SDK_ROOT}/include")
  target_link_libraries(camera_bridge_mcap PRIVATE camera_bridge_foxglove_cpp)
  target_compile_definitions(camera_bridge_mcap PRIVATE CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE=1)
  if(WIN32 AND EXISTS "${FOXGLOVE_SDK_ROOT}/lib/foxglove.dll")
    file(COPY "${FOXGLOVE_SDK_ROOT}/lib/foxglove.dll" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
endif()

if(CAMERA_BRIDGE_MCAP_BUILD_TESTING)
  enable_testing()
  add_executable(camera_bridge_mcap_cdr_test tests/cdr_test.cpp)
  target_link_libraries(camera_bridge_mcap_cdr_test PRIVATE camera_bridge_mcap)
  add_test(NAME camera_bridge_mcap_cdr_test COMMAND camera_bridge_mcap_cdr_test)

  add_executable(camera_bridge_mcap_roundtrip_test tests/roundtrip_test.cpp)
  target_link_libraries(camera_bridge_mcap_roundtrip_test PRIVATE camera_bridge_mcap)
  if(CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE)
    target_compile_definitions(camera_bridge_mcap_roundtrip_test PRIVATE
      CAMERA_BRIDGE_MCAP_TEST_LIVE_SINK=1)
  endif()
  add_test(NAME camera_bridge_mcap_roundtrip_test COMMAND camera_bridge_mcap_roundtrip_test)
endif()
