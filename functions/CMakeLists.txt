project(functions)
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Release)

include_directories(src)

add_subdirectory(src)

## Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
if (WIN32)
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
endif (WIN32)

add_library(functions ${BUILD_TYPE}
	    ${FUN_HEADERS} ${FUN_SOURCE}
)

install( TARGETS functions DESTINATION ${INSTALL_LOCATION}lib/ PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

# General tests (David)
 add_executable(function_test ${genetics_SRCS} src/exec/function_test.cpp )
 target_link_libraries(function_test functions)

# Real Vector tests (David)
 add_executable(vector_test ${genetics_SRCS} src/exec/real_vector_test.cpp )
 target_link_libraries(vector_test functions)
 
# Translate trajectory (David)
 add_executable(translate ${genetics_SRCS} src/exec/translate_vector.cpp )
 target_link_libraries(translate functions)
