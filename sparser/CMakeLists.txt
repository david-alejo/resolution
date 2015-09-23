project(sparser)
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Release)

include_directories(src)

add_subdirectory(src)


set(Boost_LIBRARIES -lboost_regex)

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
endif ()


add_library(sparser ${BUILD_TYPE}
            ${SPARSER_HEADERS} ${SPARSER_SOURCE}
)

install( TARGETS sparser DESTINATION ${INSTALL_LOCATION}lib/ PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )
target_link_libraries(sparser ${Boost_LIBRARIES})



# General tests (P. Soriano)
 add_executable(sparser_test src/main.cpp )
 target_link_libraries(sparser_test sparser ${Boost_LIBRARIES})
