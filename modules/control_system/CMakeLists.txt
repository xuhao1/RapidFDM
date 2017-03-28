cmake_minimum_required(VERSION 3.0)
set(the_description "The Control System Module")

include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/include
        /usr/local/include
        /usr/local/include/eigen3
        ../../3rdparty/
        ../../3rdparty/websocketpp
        ../aerodynamics/include/
        ./so3_adaptive_gen/codegen/lib/L1ControlAttitude
        /Applications/MATLAB_R2016b.app/extern/include

        )
link_directories(/Applications/MATLAB_R2016b.app/bin/maci64)

set(CONTROL_SYSTEM_MODULE_SOURCES
        src/base_controller.cpp
        src/so3_adaptive_controller.cpp
        include/RapidFDM/control_system/base_controller.h
        include/RapidFDM/control_system/control_system.h
        so3_adaptive_gen/codegen/lib/L1ControlAttitude/L1ControlAttitude.c
        so3_adaptive_gen/codegen/lib/L1ControlAttitude/L1ControlAttitude.h
        so3_adaptive_gen/codegen/lib/L1ControlAttitude/L1ControlAttitude_types.h
        )
add_library(rapidfdm_control_system STATIC ${CONTROL_SYSTEM_MODULE_SOURCES})

target_link_libraries(rapidfdm_control_system)


install(TARGETS  rapidfdm_control_system
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)

INSTALL (DIRECTORY ./include/
        DESTINATION include/
        FILES_MATCHING PATTERN "*.h*")
