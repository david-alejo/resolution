/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef COSTCONFIG_H
#define COSTCONFIG_H

#include "AlgorithmConfig.h"
#include <functions/FormattedTime.h>
#include <functions/RealVector.h>

namespace resolution {
  
  enum TimeExplorationTypes {
    INDEPENDENT_VELOCITY, // all intermediate_waypoints have ist own ETA and the final waypoint too (appended to the end of the genome)
    MANTAIN_ETA // The velocity of the last waypoint will minimize the ETA
  };

//! @brief This class defines the weights used to configure the costs and the sampling way
//! @brief It could be desirable to separate the costs and the sampling parameters
class CostConfig:public AlgorithmConfig
{
	public:
	
	//Cost stuff
	double distance_cost;
	double collision_penalty;
	std::string objective_type; // Cost type --> can be "reduced", "min_dist" or "adaptative"
	double min_control_dist; // When modifying trajectory: if two waypoints are separated less than this distance, the segment is united to the next and so on
	bool export_all_evolution;
	
	std::vector<double> upper_bounds; 
	std::vector<double> lower_bounds; 
	std::vector<double> speed_bounds; 
	
	std::vector<functions::RealVector> initial_solution;
	
	// Sampling stuff
	int population; // Number of individuals
	int generations; // Number of times that a generation is defined
	double max_time; // Max allowed time to the simulation
	
	int intermediate_waypoints; // Used when adding waypoints not when modifying trajectory
	int waypoint_dimension; // 2: 2D exploration. 3: 3D exploration (considers z). 0: Only speed planning (time_exploration must be true)
	bool altitude_levels; // The exploration in z axis is discretized according to the altitude_step field
	double altitude_step; // Meters per step: mandatory when altitude levels is true
	double max_course; // Max course deviation between two consec. waypoints (defaults to 1.0 rad)
	double min_z, max_z; // Minimum and maximum altitude
	
	bool modify_trajectory; // Indicates whether the waypoints are modified or added regarding to the final trajectory
	bool time_exploration; // If true, the velocity is included in the "genome"
	bool manoeuvre_selection; // If true --> it performs a modify trajectory, manoeuvre selection (time_exploration and modify trajectory are not used)
	TimeExplorationTypes time_exploration_type; // See the enum in order to get the allowed values
	double speed_factor; // Coeficient of the speed part of the objective
	functions::FormattedTime init_time; // Necessary when working with 4d trajectories
	
	std::string best_individual;
	
	
	// Evolution export stuff
	bool export_evolution; // Exports the position of each particle in each iteration
	std::string evolution_file;
	
	//! @brief initializer from parseblock
	//! @param block The block where the data is allocated
	void init(ParseBlock &block);
	
	//! @brief sets all parameters to their defaut values
	void init();
	
	//! @brief creates a new configuration class for an algorithm. To be implemented in a final class
	//! @param block The block where the data is allocated
	virtual AlgorithmConfig* createAlgorithmConfig(ParseBlock &block) = 0;
	
	//! @brief Virtual destructor. It is necessary to implement in child classes if there is some 
	//! @brief dynamic memory allocation.
	virtual ~CostConfig();
	
	//! @brief Translates the content of the class into a ParseBlock in order to write it to a file
	//! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
	virtual ParseBlock *toBlock() const;
	
	protected:
	
	//! @brief Gets the Checker that will notify whether the Configuration file is OK or not
	Checker *getConfigChecker();
		
	//! @brief Frees all fields that are necessary to free
	virtual void dispose();


};

}

#endif // COSTCONFIG_H
