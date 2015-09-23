project(UAVFlightPlan)
cmake_minimum_required(VERSION 2.6)


if (MINGW) 
  Message("Mingw detected!")
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
elseif (UNIX)
  SET(INSTALL_LOCATION /usr/local/)
  SET(BUILD_TYPE SHARED)
  Message("Unix detected")
  # if use kml is defined uncomment the following line
  SET(LIBKML kmldom kmlbase kmlengine kmlconvenience)
else ()
  #set(BOOST_ROOT "c:\\Program Files (x86)\\boost\\boost_1_51\\")
  #set(Boost_LIBRARY_DIR "c:\\Program Files (x86)\\boost\\boost_1_51\\lib")
  #SET(Boost_INCLUDE_DIR "c:\\Program Files (x86)\\boost\\boost_1_51\\")
  find_package(Boost REQUIRED)
  Message("MSVC assumed")
  set(Boost_USE_STATIC_LIBS   OFF)
  SET(INSTALL_LOCATION "C:/Program Files (x86)/")
  set (CMAKE_CXX_FLAGS "/EHsc")
  set (CMAKE_C_FLAGS "/EHsc")
  set (LIBS )
endif ()

SET(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED)
# find_package(libkml REQUIRED)
include_directories(src)

add_subdirectory(src)

# set (LIBS sparser functions winmm)
set(LIBS UAVFlightPlan functions sparser ${LIBKML} ${LIBXML})


add_library(UAVFlightPlan ${BUILD_TYPE}
            ${FP_HEADERS} ${FP_SOURCE}
)

install( TARGETS UAVFlightPlan DESTINATION ${INSTALL_LOCATION}lib/ PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )


target_link_libraries(UAVFlightPlan ${LIBS})

# Loads a KML log with orientation data and exports it to matlab forma
 add_executable(kml2matlab src/exec/kml2matlab.cpp )
 target_link_libraries(kml2matlab ${LIBS} UAVFlightPlan)

# UAV Navigation file to KML file utility (David)
 add_executable(test_earth_location src/exec/test_earth_location.cpp )
 target_link_libraries(test_earth_location ${LIBS} UAVFlightPlan)

# UAV Navigation file to KML file utility (David)
 add_executable(uav2kml src/exec/uav2kml.cpp )
 target_link_libraries(uav2kml ${LIBS} UAVFlightPlan)

# UAV Navigation file to KML file utility (David)
 add_executable(fp2kml src/exec/fp2kml.cpp )
 target_link_libraries(fp2kml ${LIBS} UAVFlightPlan)
 
 # UAV Navigation file to KML file utility (David)
 add_executable(fp2qgc src/exec/fp2qgc.cpp )
 target_link_libraries(fp2qgc ${LIBS} UAVFlightPlan)

# Creates obstacles (David)
 add_executable(create_obstacles src/exec/create_obstacles.cpp )
 target_link_libraries(create_obstacles ${LIBS} UAVFlightPlan)

# Translates a log into constant sample time log (David)
 add_executable(uavs_sample src/exec/uavs_sample.cpp )
 target_link_libraries(uavs_sample UAVFlightPlan UAVFlightPlan)

# Cuts a log (David)
 add_executable(cut src/exec/cut.cpp )
 target_link_libraries(cut UAVFlightPlan functions ${LIB_KML} ${LIBXML})
 
 # Exports a local trajectory to a global trajectory both in MATLAB format
 add_executable(local2global src/exec/local2global.cpp )
 target_link_libraries(local2global UAVFlightPlan functions ${LIB_KML} ${LIBXML})

