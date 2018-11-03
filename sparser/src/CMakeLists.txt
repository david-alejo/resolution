SET(HEADERS all.h Checker.h ParseBlock.h ParseContext.h ParseProperty.h Parser.h StringUtil.h) 

SET(SPARSER_HEADERS src/all.h src/Checker.h src/ParseBlock.h src/ParseContext.h src/ParseProperty.h src/Parser.h src/StringUtil.h PARENT_SCOPE)

SET(SPARSER_SOURCE src/Checker.cpp src/ParseBlock.cpp src/ParseProperty.cpp src/Parser.cpp src/StringUtil.cpp PARENT_SCOPE) 

set(CMAKE_BUILD_TYPE Debug)

## Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
if (WIN32)
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
endif (WIN32)

INSTALL(FILES  ${HEADERS}
        DESTINATION ${INSTALL_LOCATION}include/sparser
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ) 
