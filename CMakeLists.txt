cmake_minimum_required(VERSION 3.8.11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

PROJECT(Template)
set(VERSION_MAJOR 3)
set(VERSION_MINOR 0)
set(VERSION_TINY  0)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

# <------- eCAL/proto ---------->
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON) # needed for eCAL

if (MSVC)
  set(CMAKE_CXX_STANDARD 14)             # needed for eCAL
  set(CMAKE_CXX_STANDARD_REQUIRED ON)    # needed for eCAL
endif()

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(.
    pointcloud
)

set(protobuf_files
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/pcl.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/pcl_header.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/pcl_pfield.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/types.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/lane.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/LinePrimitive.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/Color.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/Pose.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/Point3.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/Quaternion.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/Vector3.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/PointCloud.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/foxglove/PackedElementField.proto
)
# <------- eCAL/proto ---------->


cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)
# add_definitions(-DJT128_256)

# 设置生成平台为 x64
if(MSVC)
add_compile_options("/source-charset:utf-8")
set(CMAKE_GENERATOR_PLATFORM x64)
set(OPENSSL_ROOT_DIR "C:/Program Files/OpenSSL-Win64")
set(OPENSSL_INCLUDE_DIR ${OPENSSL_ROOT_DIR}/include)
endif()


include_directories("${PROJECT_BINARY_DIR}")
find_package(OpenSSL REQUIRED)
add_definitions(-DQT_MESSAGELOGCONTEXT)
add_subdirectory(libhesai)

include_directories(
  libhesai
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
  third_party
  ${OPENSSL_INCLUDE_DIR}
)
link_directories(./libhesai/PtcClient/lib)

#add_executable(sample
#  test/test.cc
#)

add_library(foxglove_handler STATIC)

# Set include files
target_include_directories(foxglove_handler
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/foxglove
        ${Protobuf_INCLUDE_DIR}

)
PROTOBUF_TARGET_CPP(foxglove_handler ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/ ${protobuf_files})

# Set source files
target_sources(foxglove_handler
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/foxglove/FoxglovePointCloudHandler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/foxglove/FoxglovePointCloudUtilities.cpp
)

target_link_libraries(foxglove_handler
  PRIVATE
  protobuf::libprotobuf
)

install(
    TARGETS foxglove_handler
    COMPONENT TARGET_COMPONENT
    DESTINATION lib
)

add_executable(sample_eCal
  test_eCAL/pointcloud/PointCloudHandler.cpp
  test_eCAL/test.cc
)

target_include_directories(sample_eCal
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL
  ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/foxglove
)

PROTOBUF_TARGET_CPP(sample_eCal ${CMAKE_CURRENT_SOURCE_DIR}/test_eCAL/proto/ ${protobuf_files})


target_link_libraries(sample_eCal
  source_lib
  container_lib
  ptcClient_lib
  serialClient_lib
  ptcParser_lib
  log_lib
  ${OPENSSL_LIBRARIES}
  eCAL::core
  protobuf::libprotobuf
  foxglove_handler
)

# if install different cuda version, set the cuda path, like cuda-11.4
# set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4/)
# find_package(CUDA REQUIRED)
find_package(CUDA )
if(${CUDA_FOUND})
	set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
	set(CUDA_SEPARABLE_COMPILATION ON)
	include_directories(${CUDA_INCLUDE_DIRS})
	set(CUDA_PROPAGATE_HOST_FLAGS OFF)
	set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;-std=c++14)#根据具体GPU性能更改算力参数
	link_directories($ENV{CUDA_PATH}/lib/x64)
  file(GLOB_RECURSE CURRENT_HEADERS  *.h *.hpp *.cuh)
  file(GLOB CURRENT_SOURCES  *.cpp *.cu)
  file(GLOB CURRENT_SOURCES  *.cpp *.cu)
  source_group("Include" FILES ${CURRENT_HEADERS}) 
  source_group("Source" FILES ${CURRENT_SOURCES}) 
  CUDA_ADD_EXECUTABLE(sample_gpu ${CURRENT_HEADERS} ${CURRENT_SOURCES} 
  ./test/test.cu
  ./libhesai/UdpParserGpu/src/buffer.cu)
  target_link_libraries(sample_gpu
  source_lib
  container_lib
  ptcClient_lib
  serialClient_lib
  log_lib
  ptcParser_lib
  ${OPENSSL_LIBRARIES}
)
else(${CUDA_FOUND})
	MESSAGE(STATUS "cuda not found!")
endif(${CUDA_FOUND})

