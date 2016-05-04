set(the_description "The Airdynamics Modules")


include_directories(./modules/airdynamics/include)
set(AIRDYNAMICS_MODULE_SOURCES
        modules/airdynamics/src/joints/Joint.cpp
        modules/airdynamics/src/nodes/Node.cpp
        modules/airdynamics/include/nodes/Node.h
        modules/airdynamics/include/joints/Joint.h
        modules/airdynamics/src/joints/FreeJoint.cpp
        modules/airdynamics/src/airdynamics_parser.cpp
        modules/airdynamics/include/joints/FreeJoint.h
        modules/airdynamics/include/FlyingData.h
        modules/airdynamics/include/airdynamics_components.h
        modules/airdynamics/include/airdynamics_parser.h
        modules/airdynamics/include/nodes/bodys/aircraft_node.h
        modules/airdynamics/src/nodes/bodys/aircraft_node.cpp
        modules/airdynamics/src/base_component.cpp
        )

set(AIRDYNAMICS_TEST_SOURCES modules/airdynamics/tests/main.cpp)

add_library(rapidfdm_airdynamics STATIC ${AIRDYNAMICS_MODULE_SOURCES})

add_executable(rapidfdm_test ${AIRDYNAMICS_TEST_SOURCES})

target_link_libraries(rapidfdm_test rapidfdm_airdynamics)
