cmake_minimum_required (VERSION 3.10 FATAL_ERROR)

project (isomesh VERSION 0.1.0 LANGUAGES CXX)

list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include (CMakePackageConfigHelpers)
include (GNUInstallDirs)
include (cotire)

set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_STANDARD 17)

find_package (glm CONFIG REQUIRED)

set (ISOMESH_HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/isomesh)
set (ISOMESH_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (ISOMESH_HEADERS
	include/isomesh/common.hpp
	include/isomesh/isomesh.hpp
	include/isomesh/algo/marching_cubes.hpp
	include/isomesh/algo/uniform_dual_contouring.hpp
	include/isomesh/data/grid.hpp
	include/isomesh/data/grid_edge_storage.hpp
	include/isomesh/data/mesh.hpp
	include/isomesh/field/heightmap.hpp
	include/isomesh/field/scalar_field.hpp
	include/isomesh/qef/qef_solver_3d.hpp
	include/isomesh/util/material_filter.hpp
	include/isomesh/util/material_selector.hpp
	include/isomesh/util/zero_finder.hpp
)
source_group (TREE ${ISOMESH_HEADER_DIR} PREFIX Headers FILES ${ISOMESH_HEADERS})

set (ISOMESH_SOURCES
	src/algo/marching_cubes.cpp
	src/algo/marching_cubes_tables.h
	src/algo/uniform_dual_contouring.cpp
	src/data/grid.cpp
	src/data/grid_edge_storage.cpp
	src/data/mesh.cpp
	src/field/heightmap.cpp
	src/qef/gradient_descent_qef_solver_3d.cpp
	src/qef/qef_solver_3d.cpp
	src/util/bisection_zero_finder.cpp
	src/util/material_filter.cpp
	src/util/regula_falsi_zero_finder.cpp
	src/private/stbi_data.cpp
)
source_group (TREE ${ISOMESH_SOURCE_DIR} PREFIX Sources FILES ${ISOMESH_SOURCES})

add_library (isomesh STATIC ${ISOMESH_HEADERS} ${ISOMESH_SOURCES})
target_link_libraries (isomesh PUBLIC glm)
target_link_libraries (isomesh PRIVATE $<$<CXX_COMPILER_ID:GNU>:stdc++fs>)
target_include_directories (isomesh PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set_target_properties (isomesh PROPERTIES
	DEBUG_POSTFIX _d
	COTIRE_ADD_UNITY_BUILD FALSE
	COTIRE_PREFIX_HEADER_INCLUDE_PATH ${GLM_INCLUDE_DIRS}
)
cotire (isomesh)

option (ISOMESH_TEST "Enable testing" OFF)
if (ISOMESH_TEST)
	enable_testing ()
	add_subdirectory (test)
endif ()

set (ISOMESH_INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/isomesh)

install (TARGETS isomesh EXPORT isomesh_targets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# build tree package
write_basic_package_version_file (${CMAKE_CURRENT_BINARY_DIR}/isomesh-config-version.cmake
	VERSION ${isomesh_VERSION} COMPATIBILITY SameMajorVersion)
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/cmake/isomesh-build-config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/isomesh-config.cmake @ONLY)
export (EXPORT isomesh_targets FILE ${CMAKE_CURRENT_BINARY_DIR}/isomesh-targets.cmake)
export (PACKAGE isomesh)

# install tree package
configure_package_config_file (${CMAKE_CURRENT_SOURCE_DIR}/cmake/isomesh-config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/${ISOMESH_INSTALL_CONFIGDIR}/isomesh-config.cmake
	INSTALL_DESTINATION ${ISOMESH_INSTALL_CONFIGDIR}
	PATH_VARS CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_INCLUDEDIR
	NO_CHECK_REQUIRED_COMPONENTS_MACRO)
install (FILES
	${CMAKE_CURRENT_BINARY_DIR}/${ISOMESH_INSTALL_CONFIGDIR}/isomesh-config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/isomesh-config-version.cmake
	DESTINATION ${ISOMESH_INSTALL_CONFIGDIR})
install (DIRECTORY ${ISOMESH_HEADER_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install (EXPORT isomesh_targets FILE isomesh-targets.cmake DESTINATION ${ISOMESH_INSTALL_CONFIGDIR})
