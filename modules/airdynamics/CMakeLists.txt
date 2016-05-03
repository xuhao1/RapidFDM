set(the_description "The Airdynamics Modules")


include_directories(./modules/airdynamics/include)
set(AIRDYNAMICS_MODULE_SOURCES
        modules/airdynamics/src/joints/Joint.cpp
        modules/airdynamics/src/nodes/Node.cpp
        modules/airdynamics/include/nodes/Node.h
        modules/airdynamics/include/joints/Joint.h
        modules/airdynamics/src/joints/FreeJoint.cpp
        modules/airdynamics/include/joints/FreeJoint.h
        )

add_library(rapidfdm_airdynamics ${AIRDYNAMICS_MODULE_SOURCES})
