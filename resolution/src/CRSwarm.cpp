#include "CRSwarm.h"

#include <sys/time.h>
#include <iostream>
#include <functional>
#include <functions/functions.h>
#include <functions/RealVector.h>
#include <boost/bind.hpp>

using namespace std;
using simulator::FlightPlan;
using functions::Point3D;

namespace resolution {



	
void CRSwarm::init()
{
	iteration = 0;
	msg = 0;
	pointersToNULL();
}

CRAlgorithm* CRSwarm::createFromBlock(ParseBlock& block) const
{
	
    CRSwarm* ret = new CRSwarm();
		Checker *check = getAlgorithmChecker();
		
		try {
			block.checkUsing(check);
			SwarmConfig swarm_conf;
			AlgorithmConfig *conf = swarm_conf.createAlgorithmConfig(block["config"]);
			ret->init(block, conf);
		} catch (exception &e) {
			cerr << "CRSwarm::createFromBlock --> error while loading data from block\n";
			delete check;
			throw(e);
		}
		
		delete check;
		
		return ret;
}


void CRSwarm::dispose() throw()
{
	
	CRAlgorithm::dispose();
	pointersToNULL();
}

void CRSwarm::pointersToNULL() {
	CRAlgorithm::pointersToNULL();
}


CRSwarm::CRSwarm()
{
	init();
}

CRSwarm::~CRSwarm()
{
	dispose();
}



CRSwarm* CRSwarm::clone() const
{
	return NULL; // TODO: Implement this function or the copy constructor
}

// TODO; Implement this method properly
std::string CRSwarm::toString() const
{
	ostringstream os;
	
	os << "CRSwarm: state" << endl;
	
	return os.str();
}

CRAlgorithmStatistics CRSwarm::execute()
{
  CRAlgorithmStatistics ret;
  solved = false;
  msg = 0;
  
  SwarmConfig &config = dynamic_cast<SwarmConfig&>(* (this->config));
  
  // Some checks in order to make sure that the parameters are correct
  
  if (!checkProblem() && !config.manoeuvre_selection) {
    cerr << "CRSwarm::run --> Errors while checking the integrity of the problem. Aborting.\n";
    ret.setError(true);
    return ret;
  }
  
  struct timeval t1, t2;
  gettimeofday(&t1, NULL);

  // Make the MonteCarlo simulation obtaining the mean trajectories and the stdDev of the particles set
  // Make a first simulation to check if there exist some collisions in the system
  if (config.debug) {
	  cout << "CRSwarm::run --> Simulating system in order to check for collisions...\n";
  }
	  
  bool collision;
  float min_distance;
  if ( sim->run(collision) ) { // Simulating the whole system saving trajectory.
    if (!collision) {
      cout << "CRSwarm::run --> No collisions found. It is not necessary to run the PS algorithm.\n";
      ret.setSolved(false);
      ret.setCollisionDetected(false);
    } else {
      ret.setCollisionDetected(true);
      cout << "CRSwarm::run --> Collisions found, initiating the CR Algorithm\n";

      // Declare the algorithm. First we have to bind the evaluation function
      boost::function<double(functions::RealVector&)> func = boost::bind(&CRSwarm::calculateObjectiveVec, this, _1);
      
      // If 4d plan is selected, we have to prepare the bounds
      CRAlgorithm::getBounds();
      
      // Now we have all the necessary parameters: population, generations, bounds and the evaluation function
      particle_swarm::ParticleSwarm algorithm(config.population, config.upper_bounds,
				      config.lower_bounds, config.speed_bounds, func, false, config.r0, config.phi0, config.inertia_weight);
      
      if (config.export_all_evolution) {
	algorithm.setSaveEvolution(true);
      }
      
      vector<EvolutionData> ev_data;
      functions::RealVector curr_best = algorithm.getBestPosition();
      
      // Adding initial solutions
      vector<functions::RealVector> &is = config.initial_solution;
      if (is.size() > 0) {
	for (unsigned int i = 0; i < is.size(); i++) { 
	  
	  if (!algorithm.setParticle(i, is.at(i))) {
	    cerr << "CRSwarm::execute --> could not set the initial solution. Genome width: " << algorithm.getBestPosition().size();
	    cerr << ". Initial solution size: " << is.at(i).size() << endl;
	  }
	}
	algorithm.setParticle(is.size(), curr_best);
      }
      
      // Expand the geometries
      sim->expandGeometries(config.geometry_expansion);
      for (iteration = 0; iteration < config.generations; iteration++) {
	curr_best = algorithm.getBestPosition();
	if (config.debug) {
		cout << "CRSwarm::run --> Iteration " << iteration << ". Best cost: " << algorithm.getBestCost() << "\n";
	}
	algorithm.oneStep();
	gettimeofday(&t2, NULL);
	EvolutionData current;
	
	current.cost = algorithm.getBestCost();
	current.t = functions::calculateLapseTime(t1, t2);
	current.plan = getFlightPlan(curr_best);
	updateDeltaETA(current);
	ev_data.push_back(current);
      }
      
      // Generate statistics
      
      if (config.export_evolution) {
	      if (saveEvolutionData(config.evolution_file, ev_data)) {
		      if (config.debug) {
			      cout << "CRSwarm::run --> Evolution saved.\n";
		      } 
	      } else {
		      cerr << "CRSwarm::run --> Could not save evolution data.\n";
	      }
      }
      if (config.export_all_evolution) {
	algorithm.exportEvolution(config.evolution_file);
      }
      ret.setEvolutionData(ev_data);
      
	cout << algorithm.toString() << endl;
      // Calculate spended time and show it
      gettimeofday(&t2,NULL);
      cout << "CRSwarm::run --> Spended time = " << functions::showTime(t1,t2) << endl;
      ret.setExecutionTime(functions::calculateLapseTime(t1, t2));
      ret.setMinObjetive(algorithm.getBestCost());

      // Get the minimum objetive value and store it for statistical purposes...
      // Show the best flight plan
      sim->setCollisionStop(false);
      bool collision, ok;
      
      vector<double> aux(algorithm.getBestPosition());
      vector<simulator::FlightPlan> fp = getFlightPlan(aux);
      
      if (config.best_individual.size() > 0) {
	functions::writeStringToFile(config.best_individual, functions::printVector(aux));
      }
      
      // Contract the geometries
      sim->expandGeometries(-config.geometry_expansion);

      try {
	      ok = simulateSystem(fp, collision);
      } catch(...) {
	      ok = false;
      }
      
      if (config.export_solution) {
	      if (exportSolution(config.solution_filename, fp)) {
		      cout << "Solution exported successfully.\n";
	      }
      }
      if (config.export_trajectories) {
	if (sim->exportTrajectory(config.trajectory_filename)) {
	  cout << "Trajectories exported successfully.\n";
	}
      }

      
      if (collision || !ok) {
	      if (!ok) {
		      cerr << "CRSwarm::run --> Error while simulating the best flight plan\n";
	      }
	      ret.setSolved(false);
	      cout  << "CRSwarm::run --> The problem was not solved.\n"; // Min distance = " << min_d << "\n";
      } else {

	      cout  << "CRSwarm::run --> The problem was solved successfully.\n";// Min distance = " << min_d << "\n";

	      
	      ret.setSolved(true);
      }
      
      if (config.export_catec != "") {
	cout << "CRSwarm::execute --> Exporting the flight plan to " << config.export_catec << endl;
	ostringstream os;
	for (unsigned int i = 0; i < fp.size(); i++) {
	  
	  os << fp.at(i).toCatec(i) << endl;
	}
	
	if (!functions::writeStringToFile(config.export_catec, os.str())) {
	    cerr << "CRSwarm::execute --> could not export the plan in Catec format. " << endl;
	  }
      }
      
      cout << endl << endl;
    }
} else {
    cerr << "CRSwarm::run --> An error was found while simulating the system. Aborting.\n";
    ret.setError(true);
}
  
  return ret;
}

void CRSwarm::init(ParseBlock& block, AlgorithmConfig* conf)
{
	try {
    resolution::CRAlgorithm::init(block, conf);
		SwarmConfig &config = dynamic_cast<SwarmConfig&>(*this->config);
		
		problem_dimension = getProblemDimensionWithoutTimeExploration();
		if (config.time_exploration) {
		  problem_dimension += howManyControlledUAVs() * (config.intermediate_waypoints + 1);
		}
		
	} catch (std::exception &e) {
		cerr << "CRSwarm::init --> Some error ocurred while loading the bounds.\n";
		throw(e);
	}
}



void CRSwarm::getBounds(std::vector< double >& upper, std::vector< double >& lower)
{
    resolution::CRAlgorithm::getBounds(upper, lower); // Clear the vectors
}

// TODO: develop this function in CRGenetics and here too
ParseBlock* CRSwarm::toBlock() const
{
    ParseBlock *ret = resolution::CRAlgorithm::toBlock();
    
    ret->setProperty("algorithm", "ParticleSwarm");
    
    return ret;
}


}
