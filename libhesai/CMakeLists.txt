# ----------------------------------------#
# Hesai Tech <http://www.hesaitech.com>  #
# ----------------------------------------#

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
project(CommonModules)
if(POLICY CMP0074)
cmake_policy(SET CMP0074 NEW)
else()
cmake_policy(SET CMP0065 NEW)
endif()

set(CMAKE_BUILD_TYPE ON)

# Add each module's entry
set(WITH_LIDARBOARD ON CACHE BOOL "Build lidarboard code")
set(WITH_PLATUTIL ON CACHE BOOL "Build platform utils code")
set(WITH_UDPPARSER ON CACHE BOOL "Build udp parser code")
set(WITH_PTCCLIENT ON CACHE BOOL "Build ptc client code")
set(WITH_PTCPARSER ON CACHE BOOL "Build ptc parser code")
set(WITH_CONTAINER ON CACHE BOOL "Build container code")
set(WITH_LOG ON CACHE BOOL "Build log code")


add_definitions(-DQT_MESSAGELOGCONTEXT)


include_directories(
    ./
    include
    Lidar
    UdpProtocol
    Server
    UdpParser
    UdpParser/include
    UdpParser/src
    PtcParser
    PtcParser/include
    PtcParser/src
    Source/include
    Container/include
    Container/src
    PtcClient/include
    Logger/include
    ${OPENSSL_INCLUDE_DIR}
)

# Make each module an 'OBJECT' library, can be refered to by get_link_libraries
if(WITH_PTCCLIENT)
    add_library(ptcClient_lib 
    PtcClient/src/ptc_client.cc
    PtcClient/src/tcp_client.cc
    PtcClient/src/tcp_ssl_client.cc
    )
    target_link_libraries(ptcClient_lib
                        ${OPENSSL_LIBRARIES}
                        )
    set_target_properties(ptcClient_lib 
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
endif(WITH_PTCCLIENT)

if(WITH_PTCPARSER)
    add_library(ptcParser_lib 
    PtcParser/src/general_ptc_parser.cc
    PtcParser/src/ptc_1_0_parser.cc
    PtcParser/src/ptc_2_0_parser.cc
    PtcParser/ptc_parser.cc)
    set_target_properties(ptcParser_lib 
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
endif(WITH_PTCPARSER)


if(WITH_CONTAINER)
    add_library(container_lib 
    Container/src/ring_2d_shared.cc
    Container/src/ring_2dex.cc)
    set_target_properties(container_lib
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
endif(WITH_CONTAINER)

if(WITH_LIDARBOARD)
    add_library(source_lib 
        Source/src/source.cc
        Source/src/socket_source.cc
        Source/src/pcap_source.cc
        Source/src/pcap_saver.cc
        )
    target_link_libraries(source_lib
        platutils_lib
        container_lib
        ptcClient_lib)
    set_target_properties(source_lib
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
endif(WITH_LIDARBOARD)

if(WITH_PLATUTIL)
    add_library(platutils_lib 
        ./src/plat_utils.cc
        ./src/auto_tick_count.cc)
    set_target_properties(platutils_lib
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
endif(WITH_PLATUTIL)



if(WITH_LOG)
    find_package(Threads)
    add_library(log_lib EXCLUDE_FROM_ALL
        Logger/src/logger.cc)
    target_link_libraries(log_lib
        ${CMAKE_THREAD_LIBS_INIT})
    set_target_properties(log_lib
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
endif(WITH_LOG)

include_directories(${PROJECT_SOURCE_DIR})
