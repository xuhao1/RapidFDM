cmake_minimum_required(VERSION 3.0)
set(the_description "The Control System Module")
include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/
        /usr/local/include
        /usr/local/include/eigen3
        ../../3rdparty/
        ../../3rdparty/websocketpp
        ../aerodynamics/include/
        )

set(CONTROL_SYSTEM_MODULE_SOURCES
        src/base_controller.cpp
        include/RapidFDM/control_system/base_controller.h
        include/RapidFDM/control_system/control_system.h
        )
add_library(rapidfdm_control_system STATIC ${CONTROL_SYSTEM_MODULE_SOURCES})
