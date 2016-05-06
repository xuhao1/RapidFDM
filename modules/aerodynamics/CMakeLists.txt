cmake_minimum_required(VERSION 3.0)
set(the_description "The Airdynamics Modules")

find_package(Boost REQUIRED
        COMPONENTS filesystem)

include_directories(./include ../../include/ ../../3rdparty/eigen3 ../../3rdparty/rapidjson/)
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

set(AIRDYNAMICS_TEST_SOURCES tests/main.cpp)

add_library(rapidfdm_airdynamics STATIC ${AIRDYNAMICS_MODULE_SOURCES})
target_link_libraries(rapidfdm_airdynamics ${Boost_LIBRARIES})

add_executable(rapidfdm_test ${AIRDYNAMICS_TEST_SOURCES})

target_link_libraries(rapidfdm_test rapidfdm_airdynamics)
