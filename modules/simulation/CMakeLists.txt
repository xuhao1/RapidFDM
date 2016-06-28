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
        include/RapidFDM/simulation/utils.h
        src/simulator_world.cpp

        src/simulator_aircraft.cpp)
set(SIMULATION_MODULES_TESTS
        test/unit_tests.cpp
        )
add_library(rapidfdm_simulator STATIC ${SIMULATION_MODULE_SOURCES})

add_executable(rapidfdm_simulator_test ${SIMULATION_MODULES_TESTS} )

target_link_libraries(rapidfdm_simulator_test
        rapidfdm_simulator
        rapidfdm_aerodynamics
        rapidfdm_control_system
        ${PHYSX_LIBRARIRS}
        )
