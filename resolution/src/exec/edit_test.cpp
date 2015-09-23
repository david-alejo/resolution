//! This utility loads a problem, modifies its parameters and save it in a (possibly other) file

#include <stdio.h>
#include <string>
#include <vector>
#include <sys/time.h>
#include "CRAlgorithmFactory.h"
#include "CRAlgorithm.h"
#include "CostConfig.h"
#include "GeneticConfig.h"
#include <functions/ArgumentData.h>
#include <functions/functions.h>

#define MAX_CONT 100
#include <SwarmConfig.h>

using namespace std;
using namespace resolution;
using simulator::FlightPlan;

void print_iicc(CRAlgorithm *alg) {
  for (unsigned int i = 0; i < alg->getSimulator()->howManyUAVs(); i++) {
    cout << "UAV " << i + 1 << " iiccs: " << functions::printVector(alg->getSimulator()->getParticle(i)->getModel()->getInitialState()) << "\t";
    cout << "Cruise speed:" << alg->getSimulator()->getParticle(i)->getController()->getFlightPlan().getCruiseSpeed() << endl;
  }
  
  
}

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Use: " << argv[0] << " <input_data_filename> [--output <output_data_filename>]\n";
    cerr << "Options: \n";
    cerr << "\t --population \t Changes the population.\n";
    cerr << "\t --dimension \t Changes the dimension of the problem.\n";
    cerr << "\t --generations \t Changes the generations of the problem.\n";
//     cerr << "
    return -1;
  }
  
  functions::ArgumentData arg(argc, argv);
  
  CRAlgorithmFactory alg_fac;
  CRAlgorithm *alg = NULL;
  
  try {
    cout << "Loading " << arg.at(1) << " file.\n";
    alg = alg_fac.createFromFile(arg.at(1));
  } catch (exception &e) {
    cerr << "Exception thrown in problem.Content: "<< e.what() << endl;
    alg = NULL;
  }
    
  if (alg != NULL) {
    // First the cost config one
    CostConfig &cost_conf = dynamic_cast<CostConfig &>(alg->getConfig());
  
    if (arg.isOption("population") ) {
      arg.getOption("population", cost_conf.population);
      cout << "Population --> " << cost_conf.population << endl;
    }
    if (arg.isOption("generations") ) {
      arg.getOption("generations", cost_conf.generations);
      cout << "Generations --> " << cost_conf.generations << endl;
    }
    if (arg.isOption("dimension") ) {
      arg.getOption("dimension", cost_conf.waypoint_dimension);
      cout << "Waypoint dimension --> " << cost_conf.waypoint_dimension << endl;
    }
    if (arg.isOption("cost_type") ) {
      arg.getOption("cost_type", cost_conf.objective_type);
      cout << "Waypoint dimension --> " << cost_conf.waypoint_dimension << endl;
    }
    if (arg.isOption("collision_penalty") ) {
      arg.getOption("collision_penalty", cost_conf.collision_penalty);
      cout << "Collision penalty --> " << cost_conf.collision_penalty << endl;
    }
    if (arg.isOption("to_quad_complete")) {
      
      std::string model_filename;
      arg.getOption("to_quad_complete", model_filename);
      
      cout << "Changing model to ModelQuad. Retrieving the model from: " << model_filename << "\n";
      
      simulator::Simulator sim_quad(model_filename);
      simulator::Simulator *sim_alg = alg->getSimulator();
      
      
      simulator::Particle &par_quad = *(sim_quad.getParticle(0));
      vector<double> iicc_quad = par_quad.getModel()->getInitialState();
      for (unsigned int i = 0; sim_alg != NULL && i < sim_alg->howManyUAVs(); i++) {
	simulator::Particle &curr_par = *(sim_alg->getParticle(i));
	
	// Set the new initial state
	vector<double> iicc_old = curr_par.getModel()->getInitialState();
	vector<double> iicc_new(iicc_quad);
	for (unsigned int j = 0; j < iicc_old.size(); j++) {
	  iicc_new.at(j) = iicc_old.at(j);
	}
	// Initial orientation in order to reach the waypoint
	iicc_new.at(3) = atan2(iicc_old.at(1), iicc_old.at(2));
	iicc_new.at(4) = curr_par.getModel()->getParameter().at(0);
	
	// Now the flight plan
	simulator::FlightPlan fp = curr_par.getController()->getFlightPlan();
	fp.setCruiseSpeed(curr_par.getModel()->getParameter().at(0));
	
	
	par_quad.getController()->setFlightPlan(fp);
	
	// Once all hanges have been carried out --> set the controller and the model to the particle
	curr_par.setModel(par_quad.getModel()->clone());
	curr_par.setController(par_quad.getController()->clone());
	curr_par.getModel()->setInitialState(iicc_new);
	
	cout << "New initial conditions: " << functions::printVector(alg->getSimulator()->getParticle(i)->getModel()->getInitialState()) << endl;
	curr_par.getController()->setFlightPlan(fp);
      }
      
    }
    if (arg.isOption("time_exploration")) {
      cost_conf.time_exploration = true;
      
    }
    
    print_iicc(alg);
    
    const double &speed = alg->getSimulator()->getParticle(0)->getController()->getFlightPlan().getCruiseSpeed();
    
    // Set the output file
    string output_file = arg.at(1);
    if (arg.isOption("output")) {
      arg.getOption("output", output_file);
      cout << "Changing the output file to " << output_file << endl;
      
    }
    
    
    // PS things
    
    try {
      if (arg.isOption("r0")) {
	double r0;
	arg.getOption("r0", r0);
	cout << "Changing r0 value to " << r0 << endl;
	SwarmConfig &confi = dynamic_cast<SwarmConfig &>(alg->getConfig());
	confi.r0 = r0;
      }
      if (arg.isOption("phi0")) {
	double phi0;
	arg.getOption("phi0", phi0);
	cout << "Changing phi0 value to " << phi0 << endl;
	SwarmConfig &confi = dynamic_cast<SwarmConfig &>(alg->getConfig());
	confi.phi0 = phi0;
      }
    } catch (exception &e) {
//       cerr << "Exception thrown when configuring Particle Swarm config.Content: "<< e.what() << endl;
    }
    
    
    // Genetic config things
    
    try {
      GeneticConfig &confi = dynamic_cast<GeneticConfig &>(alg->getConfig());
      if (arg.isOption("crossover_type")) {
	string new_type;
	arg.getOption("crossover_type", new_type);
	cout << "Changing crossover type to " << new_type << endl;
	
	confi.crossover_type = new_type;
      }
      if (arg.isOption("pmut")) {
	double p_mut;
	arg.getOption<double>("pmut", p_mut);
	cout << "Changing mutation probability to " << p_mut << endl;
	confi.pMutation = p_mut;
      }
      if (arg.isOption("mutation_deviation")) {
	double mutation_deviation;
	arg.getOption<double>("mutation_deviation", mutation_deviation);
	cout << "Changing mutation deviation to " << mutation_deviation << endl;
	confi.mutation_dev = mutation_deviation;
      }
      if (arg.isOption("pcross")) {
	double pcross;
	arg.getOption<double>("pcross", pcross);
	cout << "Changing crossover probability to " << pcross << endl;
	confi.pCrossover = pcross;
      }
      if (arg.isOption("mutator_type")) {
	string mut_type;
	arg.getOption<string>("mutator_type", mut_type);
	cout << "Changing mutation operator to " << mut_type << endl;
	confi.mutator_type = mut_type;
      }
    } catch (exception &e) {
//     cerr << "Exception thrown when configuring genetic algorithm.Content: "<< e.what() << endl;
  }
    
    print_iicc(alg);
    
    cout << "Writing the new configuration file to " << output_file << endl;
    alg->saveProblem(output_file);
    
    print_iicc(alg);
  }
  
  delete alg;
  
  return 0;
}
