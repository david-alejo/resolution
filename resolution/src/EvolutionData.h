#ifndef EVOLUTION_DATA_H
#define EVOLUTION_DATA_H

#include <vector>
#include <simulator/FlightPlan.h>

namespace resolution {

struct EvolutionData { // Used in evolutionary algorithms
        double t;
        double cost;
        std::vector<simulator::FlightPlan> plan;
	double sum_delta_ETA; // Useful when time exploration is selected
	std::vector<double> speed_in_sector; // This vector will contain the speed in each sector of the UAV. UAV_1_s_1, UAV_1_s_2, ..., UAV_N_s_M
	
	EvolutionData() {
	  t = 0.0;
	  cost = 0.0;
	  sum_delta_ETA = 0.0;
	}
	
	~EvolutionData() {
	  plan.clear();
	  speed_in_sector.clear();
	}
};

}

#endif