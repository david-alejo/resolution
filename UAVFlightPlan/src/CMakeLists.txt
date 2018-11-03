SET(HEADERS earthlocation.h UAVFlightPlan.h UAVObstacle.h UAVTrajectory.h UAVWaypoint.h general.h config.h)

SET(FP_HEADERS src/earthlocation.h src/general.h src/UAVFlightPlan.h src/UAVObstacle.h src/UAVTrajectory.h src/UAVWaypoint.h src/config.h PARENT_SCOPE)

SET(FP_SOURCE src/earthlocation.cpp src/general.cpp src/UAVFlightPlan.cpp src/UAVObstacle.cpp src/UAVTrajectory.cpp src/UAVWaypoint.cpp PARENT_SCOPE) 

set(CMAKE_BUILD_TYPE Debug)

## Multiplatform stuff
if (WIN32)
  if (MINGW)
    SET(INSTALL_LOCATION C:/QtSDK/mingw/)
    SET(BUILD_TYPE STATIC)
  else (MINGW)
    SET(INSTALL_LOCATION "C:/Program Files (x86)/")
    SET(BUILD_TYPE STATIC)
  endif (MINGW)
endif (WIN32)

INSTALL(FILES  ${HEADERS}
        DESTINATION ${INSTALL_LOCATION}include/UAVFlightPlan
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
