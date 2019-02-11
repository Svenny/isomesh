cmake_minimum_required (VERSION 3.10 FATAL_ERROR)

project (isomesh VERSION 0.1.0 LANGUAGES CXX)

list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include (GNUInstallDirs)
include (cotire)

set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_STANDARD 17)

include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/deps/glm")

add_subdirectory (src)

option (ISOMESH_TEST "Enable testing" OFF)
if (ISOMESH_TEST)
	enable_testing ()
	add_subdirectory (test)
endif ()
