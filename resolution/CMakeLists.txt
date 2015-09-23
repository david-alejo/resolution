cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Debug)


find_package(Boost REQUIRED)
# find_package(libkml REQUIRED)
include_directories(src)

add_subdirectory(src)

add_library(resolution SHARED 
	    ${RES_HEADERS} ${RES_SOURCE}
)

set(LIBS ga simulator functions sparser boost_regex particle_swarm)

target_link_libraries(resolution ${LIBS})


install( TARGETS resolution DESTINATION lib/ PERMISSIONS OWNER_READ GROUP_READ WORLD_READ )


# Simulator tests (David)
 add_executable(test_genetics src/exec/test_genetics.cpp )
 target_link_libraries(test_genetics  ${LIBS} resolution)

 add_executable(multitest src/exec/multitest.cpp )
 target_link_libraries(multitest resolution ${LIBS})

 add_executable(edit_test src/exec/edit_test.cpp )
 target_link_libraries(edit_test resolution ${LIBS})

 add_executable(random_test_generator src/exec/random_test_generator.cpp )
 target_link_libraries(random_test_generator resolution ${LIBS})
 
 add_executable(test_FlightPlanChecker src/exec/test_FlightPlanChecker.cpp )
 target_link_libraries(test_FlightPlanChecker resolution ${LIBS})
 
 add_executable(test_objective src/exec/test_objective.cpp )
 target_link_libraries(test_objective resolution ${LIBS})

  add_executable(test_pf_icra_2011 src/exec/test_pf_icra_2011.cpp )
 target_link_libraries(test_pf_icra_2011 resolution ${LIBS})
