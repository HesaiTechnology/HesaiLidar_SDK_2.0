cmake_minimum_required(VERSION 3.8.11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
PROJECT(Template)

set(WITH_PTCS_USE ON CACHE BOOL "Use Ptcs")
# set(FIND_CUDA ON CACHE BOOL "Find CUDA")



if(FIND_CUDA)
    find_package(CUDA )
if(${CUDA_FOUND})
  	project(CommonModules LANGUAGES CXX CUDA)
	set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
	set(CUDA_SEPARABLE_COMPILATION ON)
	include_directories(${CUDA_INCLUDE_DIRS})
	set(CUDA_PROPAGATE_HOST_FLAGS OFF)
	set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;-std=c++14)#根据具体GPU性能更改算力参数
	link_directories($ENV{CUDA_PATH}/lib/x64)
else(${CUDA_FOUND})
	MESSAGE(STATUS "cuda not found!")
endif(${CUDA_FOUND})
endif()





cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)

add_subdirectory(libhesai)

add_library(hesai_sdk_lib
	driver/hesai_lidar_sdk.cc
	)
target_link_libraries(hesai_sdk_lib PUBLIC 
					lidar_lib
					${OTHER_LIB})
set_target_properties(hesai_sdk_lib 
	PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
target_include_directories(hesai_sdk_lib PUBLIC   
    ${OTHER_INCLUDE_DIRS}
	driver
	algorithm
	./
) 


if(FIND_CUDA)
if(${CUDA_FOUND})
	target_compile_definitions(hesai_sdk_lib PUBLIC USE_CUDA=1)
endif(${CUDA_FOUND})
endif()



if(BUILD_LOG_LIB_SHARED)
	target_compile_definitions(hesai_sdk_lib PUBLIC BUILD_LOG_LIB_SHARED=1)
endif(BUILD_LOG_LIB_SHARED)

if(DISENABLE_TEST_CC)

else()
	add_executable(sample
		test/test.cc
		)

	target_link_libraries(sample
		hesai_sdk_lib
		)

	add_executable(multi_test
		test/multi_test.cc
		)

	target_link_libraries(multi_test
		hesai_sdk_lib
		)
endif()
