# ----------------------------------------#
# Hesai Tech <http://www.hesaitech.com>  #
# ----------------------------------------#

cmake_minimum_required(VERSION 3.8.11)

set(HESAI_SDK_VERSION_MAJOR 2)
set(HESAI_SDK_VERSION_MINOR 0)
set(HESAI_SDK_VERSION_PATCH 12)
set(HESAI_SDK_VERSION "${HESAI_SDK_VERSION_MAJOR}.${HESAI_SDK_VERSION_MINOR}.${HESAI_SDK_VERSION_PATCH}")

project(HesaiLidarSDK VERSION ${HESAI_SDK_VERSION})

include(CTest)

set(HESAI_SDK_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/driver/sdk_version.hpp.in"
    "${HESAI_SDK_GENERATED_INCLUDE_DIR}/sdk_version.hpp"
    @ONLY
)

# ============================================
# C++ 标准设置
# ============================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================
# CMake 策略
# ============================================
cmake_policy(SET CMP0053 NEW)

# ============================================
# 构建类型设置
# ============================================
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:C>:-Wall>)

# ============================================
# 构建选项
# ============================================
# 功能开关
option(WITH_PTCS_USE "Enable PTC SSL support" ON)
# option(BUILD_LOG_LIB_SHARED "Build log library as shared" OFF)

# CUDA 支持
# option(FIND_CUDA "Find and use CUDA" OFF)

# 算法选项


# ============================================
# CUDA 支持
# ============================================
if(FIND_CUDA)
    # 设置 CUDA 架构（根据实际 GPU 修改，常见值：50/60/61/70/75/80/86/89/90）
    # 可以通过 -DCMAKE_CUDA_ARCHITECTURES=xx 指定
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "61" CACHE STRING "CUDA architectures")
    endif()
    
    # 检查是否有 CUDA 编译器
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(CUDA_FOUND TRUE)
        set(CUDA_SEPARABLE_COMPILATION ON)
        # 添加 CUDA include 路径供非 CUDA 目标使用
        include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
        message(STATUS "CUDA found: ${CMAKE_CUDA_COMPILER}")
        message(STATUS "CUDA include: ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")
        message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
    else()
        set(CUDA_FOUND FALSE)
        message(STATUS "CUDA compiler not found!")
    endif()
endif(FIND_CUDA)




# ============================================
# 添加 libhesai 子模块
# ============================================
add_subdirectory(libhesai)

# ============================================
# 主 SDK 库（纯头文件模板库）
# ============================================
add_library(hesai_sdk_lib INTERFACE)

target_include_directories(hesai_sdk_lib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/driver
    ${HESAI_SDK_GENERATED_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${OTHER_INCLUDE_DIRS}
)

target_link_libraries(hesai_sdk_lib INTERFACE
    lidar_lib
    ${OTHER_LIB}
)

# ============================================
# 条件编译定义
# ============================================
if(FIND_CUDA AND CUDA_FOUND)
    target_compile_definitions(hesai_sdk_lib INTERFACE USE_CUDA=1)
endif(FIND_CUDA AND CUDA_FOUND)
if(BUILD_LOG_LIB_SHARED)
    target_compile_definitions(hesai_sdk_lib INTERFACE BUILD_LOG_LIB_SHARED=1)
endif(BUILD_LOG_LIB_SHARED)
# 算法相关编译定义
# ============================================
# 测试可执行文件
# ============================================
if(NOT DISENABLE_TEST_CC)
    add_executable(sample
        test/test.cc
    )
    target_link_libraries(sample PRIVATE
        hesai_sdk_lib
    )

    add_executable(multi_test
        test/multi_test.cc
    )
    target_link_libraries(multi_test PRIVATE
        hesai_sdk_lib
    )
endif(NOT DISENABLE_TEST_CC)

if(BUILD_TESTING)
    add_executable(firetimes_path_test
        test/firetimes_path_test.cc
    )
    target_include_directories(firetimes_path_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/libhesai
    )
    target_link_libraries(firetimes_path_test PRIVATE
        udpParser_lib
    )
    add_test(NAME firetimes_path_test COMMAND firetimes_path_test)
endif(BUILD_TESTING)

# ============================================
# 构建信息摘要
# ============================================
message(STATUS "")
message(STATUS "=== HesaiLidarSDK Build Configuration ===")
message(STATUS "  CMAKE_BUILD_TYPE:       ${CMAKE_BUILD_TYPE}")
message(STATUS "  WITH_PTCS_USE:          ${WITH_PTCS_USE}")
message(STATUS "  FIND_CUDA:              ${FIND_CUDA}")
message(STATUS "=========================================")
message(STATUS "")
