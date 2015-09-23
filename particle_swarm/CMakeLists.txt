cmake_minimum_required(VERSION 2.6)
PROJECT(particleswarm)

SET(CMAKE_BUILD_TYPE Release)

add_subdirectory(src)

## Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
if (WIN32)
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
endif (WIN32)

add_library(particle_swarm ${BUILD_TYPE}
	    ${PS_HEADERS} ${PS_SOURCE}
)

install( TARGETS particle_swarm DESTINATION ${INSTALL_LOCATION}lib/
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

ADD_EXECUTABLE(particleswarm main.cpp)
target_link_libraries(particleswarm particle_swarm functions)
