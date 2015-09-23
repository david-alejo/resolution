SET(HEADERS SwarmParticle.h ParticleSwarm.h) 

SET(PS_HEADERS src/SwarmParticle.h src/ParticleSwarm.h PARENT_SCOPE)

SET(PS_SOURCE src/SwarmParticle.cpp src/ParticleSwarm.cpp PARENT_SCOPE) 

## Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
if (WIN32)
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
endif (WIN32)

INSTALL(FILES  ${HEADERS}
        DESTINATION ${INSTALL_LOCATION}include/particle_swarm
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ) 

