project(simulator)
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Debug)


#find_package(Boost REQUIRED)
# find_package(libkml REQUIRED)
include_directories(src)

add_subdirectory(src)
set (LIBRARIES functions sparser boost_regex boost_mpi  )

## Multiplatform stuff
if (MINGW) 
  Message("Mingw detected!")
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
elseif (UNIX)
  SET(INSTALL_LOCATION /usr/local/)
  SET(BUILD_TYPE SHARED)
  Message("Unix detected")
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
  set (LIBRARIES functions sparser winmm)
endif ()


add_library(simulator ${BUILD_TYPE}
	    ${SIM_HEADERS} ${SIM_SOURCE}
)

install( TARGETS simulator DESTINATION ${INSTALL_LOCATION}lib PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

target_link_libraries(simulator ${LIBRARIES})


# Flight Plan Test (Chur)
 add_executable(test_FlightPlan src/exec/test_FlightPlan.cpp )
 target_link_libraries(test_FlightPlan simulator ${LIBRARIES})

# Simulator tests (David)
 add_executable(test_simulator src/exec/test_simulator.cpp )
 target_link_libraries(test_simulator simulator ${LIBRARIES})

# CD test (Mut)
 add_executable(CD_test src/exec/test_CD.cpp )
 target_link_libraries(CD_test simulator ${LIBRARIES})

# Range test (Chur)
 add_executable(test_Range src/exec/test_Range.cpp )
 target_link_libraries(test_Range simulator ${LIBRARIES})
 
 # Particle Filter Test (Chur)
 add_executable(test_particle_filter src/exec/test_particle_filter.cpp )
 target_link_libraries(test_particle_filter simulator ${LIBRARIES})

