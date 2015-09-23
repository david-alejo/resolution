SET(HEADERS ParticleFilter.h CD.h ObstacleDetector.h CDs.h CDFactory.h Controller.h ControllerFactory.h FlightPlan.h Controllers.h ControllerSimple.h ControllerSimpleUAV.h ControllerSimpleGlider.h ControllerQuad.h Model.h Models.h ModelFactory.h ModelQuad.h ModelSimpleUAV.h ModelSimpleUAVPolarWind.h ModelSimpleQuad.h ModelSimpleGlider.h Particle.h ParticleFactory.h Particles.h ParticleSimple.h SiCoDe.h SiCoDeExtended.h Simulator.h Thermals.h ThermalModel.h ThermalModelFactory.h ThermalModelFile.h ThermalModelSimple.h ThermalModelTime.h Updraft.h Range.h ControllerQuadCatec.h ModelQuadCatec.h ControllerQuadCatecEasy.h)

SET(SIM_HEADERS src/ParticleFilter.h  src/ObstacleDetector.h src/CD.h src/CDs.h src/CDFactory.h src/Controller.h src/ControllerFactory.h src/FlightPlan.h src/Controllers.h src/ControllerSimple.h src/ControllerSimpleUAV.h src/ControllerSimpleGlider.h src/ControllerQuad.h src/Model.h src/ModelFactory.h src/Models.h src/ModelQuad.h src/ModelSimpleQuad.h src/ModelSimpleUAV.h src/ModelSimpleUAVPolarWind.h src/ModelSimpleGlider.h src/Particle.h src/ParticleFactory.h src/Particles.h src/ParticleSimple.h src/SiCoDe.h src/SiCoDeExtended.h src/Simulator.h src/ThermalModel.h src/ThermalModelFactory.h src/Thermals.h src/ThermalModelFile.h src/ThermalModelSimple.h src/ThermalModelTime.h src/Updraft.h src/Range.h src/ControllerQuadCatec.h src/ModelQuadCatec.h src/ControllerQuadCatecEasy.h PARENT_SCOPE)

SET(SIM_SOURCE src/ParticleFilter.cpp  src/ObstacleDetector.cpp src/CD.cpp src/CDFactory.cpp src/Controller.cpp src/ControllerFactory.cpp src/FlightPlan.cpp src/ControllerSimple.cpp src/ControllerSimpleUAV.cpp src/ControllerSimpleGlider.cpp src/ControllerQuad.cpp src/Model.cpp src/ModelFactory.cpp src/ModelQuad.cpp src/ModelSimpleQuad.cpp src/ModelSimpleUAV.cpp src/ModelSimpleUAVPolarWind.cpp src/ModelSimpleGlider.cpp src/ParticleFactory.cpp src/ParticleSimple.cpp src/SiCoDe.cpp src/SiCoDeExtended.cpp src/Simulator.cpp src/ThermalModelFactory.cpp src/ThermalModel.cpp src/ThermalModelFile.cpp src/ThermalModelSimple.cpp src/ThermalModelTime.cpp src/Updraft.cpp src/Range.cpp src/ControllerQuadCatec.cpp src/ModelQuadCatec.cpp src/ControllerQuadCatecEasy.cpp PARENT_SCOPE)

## Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
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
        DESTINATION ${INSTALL_LOCATION}include/simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ) 
