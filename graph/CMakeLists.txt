cmake_minimum_required(VERSION 2.8)
project(simple_graph)

SET (HEADERS src/SimpleGraph.h src/SimpleVertex.h)

add_library(simple_graph MODULE ${HEADERS} )
SET_TARGET_PROPERTIES(simple_graph PROPERTIES LINKER_LANGUAGE CXX)

# Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
if (WIN32)
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
endif (WIN32)

INSTALL(FILES  ${HEADERS}
        DESTINATION ${INSTALL_LOCATION}include/simple_graph
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ) 

#install( TARGETS simple_graph DESTINATION lib PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

#Graph tests
add_executable(test_graph src/SimpleGraph.h src/SimpleVertex.h src/test.cpp)
target_link_libraries(test_graph functions)

