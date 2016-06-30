cmake_minimum_required(VERSION 3.0)
set(the_description "The Airdynamics Module")

find_package(Boost REQUIRED
        COMPONENTS filesystem)

include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/
        /usr/local/include/eigen3
        /usr/local/include
        ../../3rdparty/websocketpp
        )

set(AIRDYNAMICS_MODULE_SOURCES
        src/airdynamics_parser.cpp
        src/base_component.cpp
        src/joints/Joint.cpp
        src/joints/fixed_joint.cpp
        src/nodes/bodys/aircraft_node.cpp
        src/nodes/Node.cpp
        src/aerodynamics_configurer.cpp
        include/RapidFDM/aerodynamics/FlyingData.h
        include/RapidFDM/aerodynamics/base_component.h
        include/RapidFDM/aerodynamics/aerodynamics.h
        include/RapidFDM/aerodynamics/airdynamics_parser.h
        include/RapidFDM/aerodynamics/joints/Joint.h
        include/RapidFDM/aerodynamics/joints/joints.h
        include/RapidFDM/aerodynamics/nodes/Node.h
        include/RapidFDM/aerodynamics/nodes/nodes.h
        include/RapidFDM/aerodynamics/nodes/bodys/aircraft_node.h
        include/RapidFDM/aerodynamics/nodes/node_helper.h
        include/RapidFDM/aerodynamics/nodes/wings/wing_node.h
        include/RapidFDM/aerodynamics/nodes/engines/easy_propeller.h
        include/RapidFDM/aerodynamics/nodes/engines/base_engine.h
        include/RapidFDM/aerodynamics/geometrys/wing_geometrys.h
        include/RapidFDM/aerodynamics/geometrys/base_geometry.h
        include/RapidFDM/aerodynamics/geometrys/Geometrys.h
        include/RapidFDM/aerodynamics/geometrys/geometry_helper.h
        include/RapidFDM/aerodynamics/geometrys/standard_geometrys.h
        include/RapidFDM/aerodynamics/joints/fixed_joint.h
        include/RapidFDM/aerodynamics/joints/joint_helper.h
        include/RapidFDM/aerodynamics/aerodynamics_configurer.h

        )

SET(AERODYNAMICS_CONFIGURER_SOURCES src/aerodynamics-configure-server.cpp
        src/aerodynamics-configure-server.h
        )

set(AIRDYNAMICS_TEST_SOURCES tests/main.cpp)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${build_dir}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${build_dir}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${build_dir}/bin")

add_library(rapidfdm_aerodynamics STATIC ${AIRDYNAMICS_MODULE_SOURCES})
target_link_libraries(rapidfdm_aerodynamics ${Boost_LIBRARIES})

add_executable(rapidfdm_aerodynamics_test ${AIRDYNAMICS_TEST_SOURCES})

target_link_libraries(rapidfdm_aerodynamics_test rapidfdm_aerodynamics)

add_executable(rapidfdm_aerodynamics_configure_server ${AERODYNAMICS_CONFIGURER_SOURCES})
target_link_libraries(rapidfdm_aerodynamics_configure_server rapidfdm_aerodynamics)