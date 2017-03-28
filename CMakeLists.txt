cmake_minimum_required(VERSION 3.0)
project(RapidFDM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(build_dir "${CMAKE_CURRENT_SOURCE_DIR}/build/")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${build_dir}/bin/")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${build_dir}/lib")
set(CMAKE_HEADERS_OUTPUT_DIRECTORY "${build_dir}/include")
set(CMAKE_INSTALL_PREFIX "${build_dir}/release")


if (WIN32)
set(PHYSX_LIBRARIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/PhysX-3.3/PhysXSDK/Lib/vc14win64)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif (WIN32)
if (APPLE)
set(PHYSX_LIBRARIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/PhysX-3.3/PhysXSDK/Lib/osx64 )
endif (APPLE)

LINK_DIRECTORIES(${PHYSX_LIBRARIES_DIR} )

find_package(Boost REQUIRED
        COMPONENTS filesystem system)
include_directories(${Boost_INCLUDE_DIRS})
set(PHYSX_LIBRARIES
		PhysX3
        LowLevel
        PhysX3Extensions
        PhysX3Vehicle
        PxTask
        PhysX3Common
        LowLevelCloth
        SceneQuery
        SimulationController
        PhysXVisualDebuggerSDK
        PhysXProfileSDK
        ${PHYSX_LIBRARIES_DIR}
	)

link_directories(
${Boost_LIBRARY_DIRS} 
3rdparty/rapidjson/include
)
add_subdirectory(modules/common)
add_subdirectory(modules/network_protocol)
add_subdirectory(modules/aerodynamics)
add_subdirectory(modules/control_system)
add_subdirectory(modules/simulation)


INSTALL (DIRECTORY ./data/
		DESTINATION data/
		)

INSTALL (DIRECTORY ./sample_data/aircrafts/
		DESTINATION aircrafts/
		)


INSTALL (DIRECTORY ./apps/RapidFDM-Visuallizer
		DESTINATION ./
		)

INSTALL(PROGRAMS ./apps/RapidFDM-Visuallizer/rapidfdm
		DESTINATION ./)
