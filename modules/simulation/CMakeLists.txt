cmake_minimum_required(VERSION 3.0)
set(the_description "The Physics Simulation Module")

find_package(Boost REQUIRED
        COMPONENTS filesystem system)
include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/include
        /usr/local/include/eigen3
        ../../3rdparty/PhysX-3.3/PhysXSDK/Include
        ../../3rdparty/PhysX-3.3/PhysXSDK/Include/extensions
        ../../3rdparty/websocketpp
        ../../3rdparty/
        ../../3rdparty/mavlink/include
        ../aerodynamics/include/
        ../control_system/include/
        ../network_protocol/include
        /usr/local/include
        ../common/include

        )


set(SIMULATION_MODULE_SOURCES
        include/RapidFDM/simulation/simulator_world.h
        include/RapidFDM/simulation/simulator_aircraft.h
        include/RapidFDM/simulation/utils.h
        src/simulator_world.cpp
        src/simulator_aircraft.cpp src/simulation_websocket_server.cpp
        src/simulation_dji_a3.cpp
        include/RapidFDM/simulation/simulation_dji_a3_adapter.h
        include/RapidFDM/simulation/simulation_hil_adapter.h
        src/simulation_hil_adapter.cpp
        src/simulation_pixhawk_adapter.cpp
        )

set(SIMULATION_MODULE_SOURCES_WS
        src/simulator_aircraft.cpp
        src/simulation_websocket_server.cpp
        )

set(SIMULATION_MODULES_TESTS
        test/unit_tests.cpp
        )
add_library(rapidfdm_simulator STATIC ${SIMULATION_MODULE_SOURCES})
add_executable(rapidfdm_simulator_ws ${SIMULATION_MODULE_SOURCES_WS})
target_link_libraries(rapidfdm_simulator
        ${Boost_LIBRARIES}
        ${PHYSX_LIBRARIES}
        rapidfdm_aerodynamics
        rapidfdm_control_system
        rapidfdm_network_protocol
        )

target_link_libraries(rapidfdm_simulator_ws
        rapidfdm_simulator)
