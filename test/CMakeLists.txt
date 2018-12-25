function (isomesh_add_test NAME)
	set (TARGET_NAME test-${NAME})
	add_executable (${TARGET_NAME} ${NAME}.cpp)
	add_test (NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
endfunction ()

isomesh_add_test (testglm)

