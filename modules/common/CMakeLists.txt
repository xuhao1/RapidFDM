cmake_minimum_required(VERSION 3.0)
set(the_description "The Airdynamics Module")

include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/include
        /usr/local/include/eigen3
        /usr/local/include
        ../../3rdparty/websocketpp
        ../network_protocol/include
        )

set(COMMON_MODULE_SOURCES
        src/resource_manager.cpp
        include/RapidFDM/common/resource_manager.h
        )


add_library(rapidfdm_common STATIC ${COMMON_MODULE_SOURCES})

target_link_libraries(rapidfdm_common ${Boost_LIBRARIES})
