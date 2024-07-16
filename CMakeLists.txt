cmake_minimum_required(VERSION 3.8.11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
PROJECT(Template)
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  7)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)

# 设置生成平台为 x64
if(_MSC_VER)
set(CMAKE_GENERATOR_PLATFORM x64)
set(OPENSSL_ROOT_DIR "C:/Program Files/OpenSSL-Win64")
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

# add_executable(sample
#   test/test.cc
# )

# target_link_libraries(sample
#   source_lib
#   container_lib
#   ptcClient_lib
#   ptcParser_lib
#   log_lib
#   ${OPENSSL_LIBRARIES}
# )
set(HESAI_HEADER_DEST /opt/nvidia/deepstream/deepstream)
set(HESAI_HEADER_DEST_INCLUDE ${HESAI_HEADER_DEST}/sources/includes/Hesai_SDK_2.0)
set(HESAI_HEADER_DEST_LIBS ${HESAI_HEADER_DEST}/lib/Hesai_SDK_2.0)

set(Hesai_libs
  build/libhesai/libcontainer_lib.a
  build/libhesai/liblog_lib.a
  build/libhesai/libptcClient_lib.a
  build/libhesai/libsource_lib.a
  build/libhesai/libptcParser_lib.a
  build/libhesai/libplatutils_lib.a
)
foreach(file ${Hesai_libs})
    install(FILES ${file} DESTINATION ${HESAI_HEADER_DEST_LIBS})
endforeach()


file(GLOB_RECURSE HEADER_FILES "*.h")
install(FILES ${HEADER_FILES} DESTINATION  ${HESAI_HEADER_DEST_INCLUDE})

file(GLOB_RECURSE HEADER_FILES "libhesai/UdpParser/*.cc")
install(FILES ${HEADER_FILES} DESTINATION  ${HESAI_HEADER_DEST_INCLUDE})

set(Hesai_headers
  driver/hesai_lidar_sdk.hpp
  libhesai/Container/src/ring.cc
  libhesai/Container/src/blocking_ring.cc
  ./libhesai/Lidar/lidar.cc
)
foreach(file ${Hesai_headers})
    install(FILES ${file} DESTINATION ${HESAI_HEADER_DEST_INCLUDE})
    endforeach()
install(FILES ./libhesai/UdpParser/src/udp_p40_parser.cc DESTINATION ${HESAI_HEADER_DEST_INCLUDE})
    
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
  log_lib
  ptcParser_lib
  ${OPENSSL_LIBRARIES}
)
else(${CUDA_FOUND})
	MESSAGE(STATUS "cuda not found!")
endif(${CUDA_FOUND})

