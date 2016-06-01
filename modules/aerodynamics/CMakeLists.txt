cmake_minimum_required(VERSION 3.0)
set(the_description "The Airdynamics Modules")

find_package(Boost REQUIRED
        COMPONENTS filesystem)

include_directories(./include ../../include/ ../../3rdparty/eigen3 ../../3rdparty/rapidjson/ /usr/local/include/eigen3 /usr/local/include)
set(AIRDYNAMICS_MODULE_SOURCES
        src/airdynamics_parser.cpp
        src/base_component.cpp
        src/joints/Joint.cpp
        src/joints/fixed_joint.cpp
        src/nodes/bodys/aircraft_node.cpp
        src/nodes/Node.cpp
        include/FlyingData.h
        include/base_component.h
        include/airdynamics_components.h
        include/airdynamics_parser.h
        include/joints/Joint.h
        include/nodes/Node.h
        include/nodes/bodys/aircraft_node.h
        include/nodes/node_helper.h
        include/nodes/wings/wing_node.h
        include/geometrys/wing_geometrys.h
        include/geometrys/base_geometry.h
        include/geometrys/geometry_helper.h
        include/geometrys/standard_geometrys.h
        include/joints/fixed_joint.h
        include/joints/joint_helper.h)

SET(AERODYNAMICS_CONFIGURER_SOURCES src/aerodynamics-configure-server.cpp
        src/aerodynamics-configure-server.h)

set(AIRDYNAMICS_TEST_SOURCES tests/main.cpp)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${build_dir}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${build_dir}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${build_dir}/bin")

add_library(rapidfdm_airdynamics STATIC ${AIRDYNAMICS_MODULE_SOURCES})
target_link_libraries(rapidfdm_airdynamics ${Boost_LIBRARIES})

add_executable(rapidfdm_test ${AIRDYNAMICS_TEST_SOURCES})

target_link_libraries(rapidfdm_test rapidfdm_airdynamics)

add_executable(rapidfdm_aerodynamics_configure_server ${AERODYNAMICS_CONFIGURER_SOURCES})
target_link_libraries(rapidfdm_aerodynamics_configure_server rapidfdm_airdynamics ${LCM_LIBRARIES})