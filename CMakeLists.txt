cmake_minimum_required(VERSION 3.8.11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
PROJECT(Template)
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  9)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

# cmake_policy(SET CMP0053 NEW)
# set(CMAKE_BUILD_TYPE ON)
# set(CMAKE_BUILD_TYPE Release)
# add_definitions(-O3)
# add_compile_options(-Wall)
# add_definitions(-DJT128_256)

add_subdirectory(libhesai)

add_definitions(-DQT_MESSAGELOGCONTEXT)

add_library(
    HesaiSDK
    INTERFACE)
add_library(Hesai::SDK ALIAS HesaiSDK)

target_include_directories(
    HesaiSDK
    INTERFACE libhesai
              libhesai/Lidar
              libhesai/UdpParser
              libhesai/UdpParser/include
              libhesai/UdpParser/src
              libhesai/UdpProtocol
              libhesai/Source/include
              libhesai/Container/include
              libhesai/Container/src
              libhesai/UdpParserGpu
              libhesai/UdpParserGpu/include
              libhesai/UdpParserGpu/src
              libhesai/PtcClient/include
              libhesai/SerialClient/include
              libhesai/PtcParser
              libhesai/PtcParser/src
              libhesai/PtcParser/include
              libhesai/Logger/include
              libhesai/include
              driver
              third_party)

target_link_libraries(
    HesaiSDK
    INTERFACE source_lib
              container_lib
              ptcClient_lib
              serialClient_lib
              ptcParser_lib
              log_lib
           )


