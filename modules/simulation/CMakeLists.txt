cmake_minimum_required(VERSION 3.0)
set(the_description "The Physics Simulation Module")
include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/
        /usr/local/include/eigen3
        ../../3rdparty/PhysX-3.3/PhysXSDK/Include
        ../../3rdparty/PhysX-3.3/PhysXSDK/Include/extensions
        ../../3rdparty/
        ../../3rdparty/websocketpp
        ../aerodynamics/include/
        ../control_system/include/
        /usr/local/include

        )


set(SIMULATION_MODULE_SOURCES
        include/RapidFDM/simulation/simulator_world.h
        include/RapidFDM/simulation/simulator_aircraft.h
        src/simulator_world.cpp
        src/simulator_aircraft.cpp)
add_library(rapidfdm_simulator STATIC ${SIMULATION_MODULE_SOURCES})

