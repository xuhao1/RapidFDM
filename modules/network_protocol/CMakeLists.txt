cmake_minimum_required(VERSION 3.0)
set(the_description "The Control System Module")

find_package(Boost REQUIRED
        COMPONENTS filesystem)

include_directories(./include
        ../../include/
        ../../3rdparty/eigen3
        ../../3rdparty/rapidjson/
        /usr/local/include
        /usr/local/include/eigen3
        ../../3rdparty/
        ../../3rdparty/websocketpp
        ../aerodynamics/include/
        ../network_protocol/include
        )

set(NETWORK_PROTOCOL_MODULE_SOURCES
        include/RapidFDM/network_protocol/websocket_server.h
        include/RapidFDM/network_protocol/ws_channel_handler.h
        src/websocket_server.cpp
        src/ws_channel_handler.cpp
        )
set(NETWORK_PROTOCOL_MODULE_TEST_SOURCES
        tests/network_protocol_test.cpp
        )

add_library(rapidfdm_network_protocol STATIC ${NETWORK_PROTOCOL_MODULE_SOURCES})
target_link_libraries(rapidfdm_network_protocol ${Boost_LIBRARIES})

add_executable(rapidfdm_network_protocol_test ${NETWORK_PROTOCOL_MODULE_TEST_SOURCES})
target_link_libraries(rapidfdm_network_protocol_test rapidfdm_network_protocol)
