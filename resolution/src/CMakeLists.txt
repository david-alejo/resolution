SET(HEADERS AlgorithmConfig.h CostConfig.h CRAlgorithm.h CRAlgorithms.h CRAlgorithmStatistics.h CRAlgorithmFactory.h CRGenetics.h CRSwarm.h EvolutionData.h GeneticConfig.h operators.h SwarmConfig.h FlightPlanChecker.h FireworksConfig.h CRFireworks.h )

SET(RES_HEADERS src/AlgorithmConfig.h src/CostConfig.h src/CRAlgorithm.h src/CRAlgorithms.h src/CRAlgorithmStatistics.h src/CRAlgorithmFactory.h src/CRGenetics.h src/CRSwarm.h src/EvolutionData.h src/GeneticConfig.h src/operators.h src/SwarmConfig.h src/FlightPlanChecker.h src/FireworksConfig.h src/CRFireworks.h PARENT_SCOPE) 

SET(RES_SOURCE src/AlgorithmConfig.cpp src/CostConfig.cpp src/CRAlgorithm.cpp src/CRAlgorithmStatistics.cpp src/CRAlgorithmFactory.cpp src/CRGenetics.cpp src/CRSwarm.cpp src/GeneticConfig.cpp src/operators.cpp src/SwarmConfig.cpp src/FlightPlanChecker.cpp src/FireworksConfig.cpp src/CRFireworks.cpp PARENT_SCOPE) 

INSTALL(FILES  ${HEADERS}
        DESTINATION include/resolution
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ) 