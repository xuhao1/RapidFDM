set(the_description "The Airdynamics Modules")


include_directories(./modules/aerodynamics/include)
set(AIRDYNAMICS_MODULE_SOURCES
        modules/aerodynamics/src/airdynamics_parser.cpp
        modules/aerodynamics/src/base_component.cpp
        modules/aerodynamics/src/joints/Joint.cpp
        modules/aerodynamics/src/joints/fixed_joint.cpp
        modules/aerodynamics/src/nodes/bodys/aircraft_node.cpp
        modules/aerodynamics/src/nodes/Node.cpp
        modules/aerodynamics/include/FlyingData.h
        modules/aerodynamics/include/base_component.h
        modules/aerodynamics/include/airdynamics_components.h
        modules/aerodynamics/include/airdynamics_parser.h
        modules/aerodynamics/include/joints/Joint.h
        modules/aerodynamics/include/nodes/Node.h
        modules/aerodynamics/include/nodes/bodys/aircraft_node.h
        modules/aerodynamics/include/nodes/node_helper.h
        modules/aerodynamics/include/nodes/wings/wing_node.h
        modules/aerodynamics/include/geometrys/wing_geometrys.h
        modules/aerodynamics/include/geometrys/base_geometry.h
        modules/aerodynamics/include/geometrys/geometry_helper.h
        modules/aerodynamics/include/geometrys/standard_geometrys.h
        modules/aerodynamics/include/joints/fixed_joint.h
        modules/aerodynamics/include/joints/joint_helper.h
        )

set(AIRDYNAMICS_TEST_SOURCES modules/aerodynamics/tests/main.cpp)

add_library(rapidfdm_airdynamics STATIC ${AIRDYNAMICS_MODULE_SOURCES})

add_executable(rapidfdm_test ${AIRDYNAMICS_TEST_SOURCES} ${AIRDYNAMICS_MODULE_SOURCES})

