cmake_minimum_required(VERSION 2.6)
project(sca_loop_functions)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(Lua53 REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)

include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
add_library(sca_loop_functions SHARED sca_loop_functions.h sca_loop_functions.cpp)
target_link_libraries(sca_loop_functions ${ARGOS_LIBRARIES} ${LUA_LIBRARIES} argos3plugin_simulator_epuck)
