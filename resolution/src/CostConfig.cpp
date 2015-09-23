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

#include "CostConfig.h"
#include "functions/functions.h"

using namespace functions;

namespace resolution {
void CostConfig::init(ParseBlock& block)
{
  Checker *check = getConfigChecker();

  try {
    resolution::AlgorithmConfig::init(block);
    
    // Cost parameters
    distance_cost = block["cost"]("distance").as<double>();
    collision_penalty = block["cost"]("collision_penalty").as<double>();
    
    // Sampling parameters
    population = block("population").as<int>();
    generations = block("generations").as<int>();
    waypoint_dimension = block("waypoint_dimension").as<int>();
    intermediate_waypoints = block("intermediate_waypoints").as<int>();
    if (block.hasProperty("altitude_levels")) {
      altitude_levels = block("altitude_levels").as<bool>();
    }
    if (block.hasProperty("altitude_step")) {
      altitude_step = block("altitude_step").as<double>();
    }
    
    if (block.hasProperty("export_evolution")) {
      export_evolution = block("export_evolution").as<bool>();
      if (export_evolution && block.hasProperty("evolution_filename")) {
	evolution_file = block("evolution_filename").as<string>();
      } else {
	export_evolution = false;
      }
    }
    if (block.hasProperty("cost_type")) {
      objective_type = block("cost_type").as<string>();
    }
    
    if (block.hasProperty("time_exploration")) {
      time_exploration = block("time_exploration").as<bool>();
    }
    if (block.hasProperty("time_exploration_type")) {
      time_exploration_type = static_cast<resolution::TimeExplorationTypes>(block("time_exploration_type").as<int>());
    }
    
    if (block.hasProperty("init_time")) {
      std::vector<double> v;
      v = block("init_time").as<std::vector<double> >();
      long usec = 0;
      if (v.size() > 1) {
	usec = v.at(1);
      }
      init_time.setTime(v.at(0), usec);		
      
    } else {
      init_time.getTime();
    }
    
    // Bounds
    upper_bounds = block["bounds"]("upper").as<std::vector<double> >();
    lower_bounds = block["bounds"]("lower").as<std::vector<double> >();
    if (block["bounds"].hasProperty("speed_bounds")) {
      speed_bounds = block["bounds"]("speed").as<std::vector<double> >();
    } else {
      speed_bounds = upper_bounds;
    }
    
    if (block.hasProperty("modify_trajectory")) {
      modify_trajectory = block("modify_trajectory").as<bool>();
    }
    
    if (block.hasProperty("min_control_dist")) {
      min_control_dist = block("min_control_dist").as<double>();
    }
    if (block.hasProperty("best_individual")) {
      best_individual = block("best_individual").as<string>();
    }
    
    if (block.hasProperty("initial_solution")) {
      ParseBlock::Properties *iss = block.getProperties("initial_solution");
    
      ParseBlock::Properties::iterator it = iss->begin();
      for (;it != iss->end(); it++) {
	std::vector <double> v = (*it)->as<std::vector<double> >();
	functions::RealVector aux(v);
	initial_solution.push_back(v);
      }
    }
    if (block.hasProperty("export_all_evolution")) {
      export_all_evolution = true;
    }
    if (block.hasProperty("manoeuvre_selection")) {
      manoeuvre_selection = block("manoeuvre_selection").as<bool>();
    } else {
      manoeuvre_selection = false; // default value
    }
    if (block.hasProperty("min_z")) {
      min_z = block("min_z").as<double>();
    } else {
      min_z = 0.0;
    }
    if (block.hasProperty("max_z")) {
      max_z = block("max_z").as<double>();
    } else {
      max_z = 2.0;
    }
    if (block.hasProperty("max_course")) {
      max_course = block("max_course").as<double>();
    } else {
      max_course = 1.0;
    }
    
    
  } catch (std::runtime_error &e) {
    std::cerr << "CostConfig::init --> Exception while reading data from block. Content: " << e.what()<< "\n";
    throw(e);
  }

  delete check;
}

void CostConfig::init()
{
	AlgorithmConfig::init();
	population = 0;
	generations = 0;
	waypoint_dimension = 2;
	intermediate_waypoints = 1;
	altitude_levels = false;
	altitude_step = 1.0;
	export_evolution = false;
	evolution_file = "evo.m";
	objective_type = "reduced";
	time_exploration = false;
	time_exploration_type = INDEPENDENT_VELOCITY;
	speed_factor = 20.0;
	min_control_dist = 3.0;
	best_individual = "";
	manoeuvre_selection = false;
	modify_trajectory = false;
}

Checker* CostConfig::getConfigChecker()
{
	Checker *check = new Checker;
	check->addProperty("population", new NTimes(1));
	check->addProperty("generations", new NTimes(1));
	check->addProperty("waypoint_dimension", new NTimes(1));
	check->addProperty("intermediate_waypoints", new NTimes(1));
	
	Checker *bound_checker = new Checker;
	
	bound_checker->addProperty("upper", new NTimes(1));
	bound_checker->addProperty("lower", new NTimes(1));
	
	check->addBlock("bounds", new NTimes(1));
	check->addChecker("bounds", bound_checker);
	
	Checker *cost_check = new Checker;
	cost_check->addProperty("distance", new NTimes(1));
	cost_check->addProperty("collision_penalty", new NTimes(1));
	check->addChecker("cost", cost_check);
	check->addBlock("cost", new NTimes(1));
	
	return check;
}

CostConfig::~CostConfig()
{
  dispose();
}


void CostConfig::dispose()
{
    resolution::AlgorithmConfig::dispose();
    upper_bounds.clear();
    lower_bounds.clear();
    speed_bounds.clear();
}

ParseBlock* CostConfig::toBlock() const
{
    ParseBlock *ret = resolution::AlgorithmConfig::toBlock();
    
    // Type
    ret->setProperty("cost_type", objective_type);
    
    // Bounds
    
    ParseBlock *bounds_block = new ParseBlock;
    
    bounds_block->setProperty("upper", functions::printVector(upper_bounds));
    bounds_block->setProperty("lower", functions::printVector(lower_bounds));
    bounds_block->setProperty("speed", functions::printVector(speed_bounds));
    
    ret->setBlock("bounds", bounds_block);

    
    // Cost parameters
    ParseBlock *cost_block = new ParseBlock;
    
    cost_block -> setProperty("distance", numberToString(distance_cost));
    cost_block -> setProperty("collision_penalty", numberToString(collision_penalty));
    ret->setBlock("cost", cost_block);
    
    // Sampling parameters
    ret->setProperty("population", numberToString(population));
    ret->setProperty("generations", numberToString(generations));
    ret->setProperty("waypoint_dimension", numberToString(waypoint_dimension));
    ret->setProperty("intermediate_waypoints", numberToString(intermediate_waypoints));
    ret->setProperty("altitude_levels", boolToString(altitude_levels));
    ret->setProperty("altitude_step", numberToString(altitude_step));
    ret->setProperty("export_evolution", boolToString(export_evolution));
    ret->setProperty("evolution_filename", evolution_file);
    ret->setProperty("time_exploration", boolToString(time_exploration));
    switch (time_exploration_type) {
      case INDEPENDENT_VELOCITY:
	ret->setProperty("time_exploration_type", numberToString(INDEPENDENT_VELOCITY));
	break;
	
      default:
	ret->setProperty("time_exploration_type", numberToString(MANTAIN_ETA));
    }
    ret->setProperty("speed_factor", numberToString(speed_factor));
    
    // MS-PSO stuff
    ret->setProperty("manoeuvre_selection", boolToString(manoeuvre_selection));
    ret->setProperty("min_z", numberToString(min_z));
    ret->setProperty("max_z", numberToString(max_z));
    ret->setProperty("max_course", numberToString(max_course));
    
    return ret;
}

}