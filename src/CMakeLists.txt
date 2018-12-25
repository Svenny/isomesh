set (ISOMESH_HEADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/isomesh")
set (ISOMESH_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set (ISOMESH_HEADERS
	${ISOMESH_HEADER_DIR}/isomesh.hpp
	${ISOMESH_HEADER_DIR}/util/zero_finder.hpp
)

set (ISOMESH_SOURCES
	${ISOMESH_SOURCE_DIR}/util/bisection_zero_finder.cpp
	${ISOMESH_SOURCE_DIR}/util/regula_falsi_zero_finder.cpp
)

source_group (TREE ${ISOMESH_HEADER_DIR} PREFIX Headers FILES ${ISOMESH_HEADERS})
source_group (TREE ${ISOMESH_SOURCE_DIR} PREFIX Sources FILES ${ISOMESH_SOURCES})

add_library (isomesh STATIC ${ISOMESH_HEADERS} ${ISOMESH_SOURCES})

set_target_properties (isomesh PROPERTIES
	COTIRE_ADD_UNITY_BUILD FALSE
	COTIRE_PREFIX_HEADER_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../deps"
)

cotire (isomesh)