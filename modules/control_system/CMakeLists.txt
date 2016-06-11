cmake_minimum_required(VERSION 3.0)
set(the_description "The Control System Module")
include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/
        /usr/local/include/eigen3
        /usr/local/include/PhysX-3.3/PhysXSDK/Include
        ../../3rdparty/
        ../../3rdparty/websocketpp
        )

set(CONTROL_SYSTEM_MODULE_SOURCES
        )