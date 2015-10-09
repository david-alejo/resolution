#include "CRGenetics.h"

#include <sys/time.h>
#include <ga/garandom.h>
#include <iostream>
#include "operators.h"
// #include <functional>
#include <functions/functions.h>

using namespace std;
using simulator::FlightPlan;
using functions::Point3D;

namespace resolution {



	
void CRGenetics::init()
{
	pointersToNULL();
	iterations = 0;
}

CRAlgorithm* CRGenetics::createFromBlock(ParseBlock& block) const
{
	
    CRGenetics* ret = new CRGenetics();
		Checker *check = getAlgorithmChecker();
		
		try {
			block.checkUsing(check);
			GeneticConfig gen_conf;
			AlgorithmConfig *conf = gen_conf.createAlgorithmConfig(block["config"]);
			ret->init(block, conf);
		} catch (exception &e) {
			cerr << "CRGenetics::createFromBlock --> error while loading data from block\n";
			delete check;
			throw(e);
		}
		
		delete check;
		
		return ret;
}


void CRGenetics::dispose() throw()
{
	CRAlgorithm::dispose();
	delete algorithm;
	delete bounds;
	pointersToNULL();
}

void CRGenetics::pointersToNULL() {
	CRAlgorithm::pointersToNULL();
	algorithm = NULL;
	bounds = NULL;
}


CRGenetics::CRGenetics()
{
	init();
}

CRGenetics::~CRGenetics()
{
	dispose();
}

CRGenetics* CRGenetics::clone() const
{
	return NULL; // TODO: Implement this function or the copy constructor
}

// TODO; Implement this method properly
std::string CRGenetics::toString() const
{
	ostringstream os;
	
	os << "CRGenetics: state" << endl;
	
	return os.str();
}

CRAlgorithmStatistics CRGenetics::execute_one_vs_all(std::vector<std::vector<functions::RealVector> > &traj, bool initialize) {
  functions::FormattedTime t1, t2;
  struct timeval t;
  gettimeofday(&t, NULL);
  
  t1.getTime();
  CRAlgorithmStatistics ret;
  GeneticConfig &config = dynamic_cast<GeneticConfig&>(* (this->config));
  
  sim->setTrajectories(traj);
  
  if (config.debug) {
    cout << "CRGenetics::run --> Associating the proper functions to genome algorithm\n";
  }
    
  if (bounds == NULL) {
    cerr << "CRGenetics::execute_one_vs_all() --> This should not happen: bounds = NULL\n";
    return ret;
  } 
  
  if (initialize) {
    GARealGenome genome(*bounds, CRALgorithmOneObjective, (void *)this);
    setGeneticOperators(genome);
  
    // The same process has to be applied to the population
    GAPopulation population(genome, config.population);
    if (config.initializer_type == "Deterministic") {
      population.initializer(GAPopulationInitializer);
    }

    delete algorithm;
    algorithm = new GASimpleGA(population);
    setGeneticParameters();
  }
  
  // Run genetics
  if (!config.custom_evolution) {
      if (config.debug) {
	cout << "CRGenetics::run --> Letting the algorithm evolve.\n";
      }
      algorithm->evolve();
    } else {
      customEvolution(ret, t, initialize);
    }
    
  // Calculate spended time and show it
  t2.getTime();
  ret.setExecutionTime(t2 - t1);
  
  // Get best flight plan
  GARealGenome &best = (GARealGenome &) algorithm->statistics().bestIndividual();
  FlightPlan best_wp(get1vsAllFlightPlan(geneToVector(best)));
    
  // Get the minimum objetive value and store it for statistics purposes...
  ret.setMinObjetive( best.score() );
    
  // Show the best flight plan
  cout << "CRGenetics::run --> Evolution finished. Best flight plan:\n";
  cout << best_wp.toString() << endl;
  
  sim->getParticle(0)->getController()->setFlightPlan(best_wp);
  
  if (config.export_trajectories) {
	if (sim->exportTrajectory(config.trajectory_filename)) {
	  cout << "Trajectories exported successfully.\n";
	}
      }
    
  return ret;
}

CRAlgorithmStatistics CRGenetics::execute()
{
  CRAlgorithmStatistics ret;
  solved = false;
  
  // Load some needed stuff
  if (config->debug) {
    cout << "CRGenetics::run --> Loading system data.\n";
  }
  
  GeneticConfig &config = dynamic_cast<GeneticConfig&>(* (this->config));
  
  iterations = 0;
  struct timeval t1, t2;
  gettimeofday(&t1, NULL);

  if (!ret.getError()) {
    // Make a first simulation to check if there exist some collisions in the system
    if (config.debug) {
      cout << "CRGenetics::run --> Simulating system in order to check for collisions...\n";
    }
    
    bool collision;
    float min_distance;
    if ( sim->run(collision) ) { // Simulating the whole system saving trajectory.
      if (!collision) {
	      cout << "CRGenetics::run --> No collisions found. It is not necessary to run GA algorithm.\n";
// 				cout << "CRGenetics::run --> Min distance = " << min_distance << endl;
	      
	      ret.setSolved(false);
	      ret.setCollisionDetected(false);
      } else {
	      ret.setCollisionDetected(true);
	      cout << "CRGenetics::run --> Collisions found, initiating the CR Algorithm\n";
// 				cout << "CRGenetics::run --> Min distance = " << min_distance << endl;
      }
    } else {
      cerr << "CRGenetics::run --> An error was found while simulating the system. Aborting.\n";
      ret.setError(true);
    }
	
  }
  
  if ( !ret.getError() && ret.getCollisionDetected() ) {
    // Beggining of the algorithm.
    
    // Associate the objetive and population initializer functions to genome and population
    if (config.debug) {
      cout << "CRGenetics::run --> Associating the proper functions to genome algorithm\n";
    }
    
    if (bounds == NULL) {
      cerr << "CRGenetics::execute() --> This should not happen: bounds = NULL\n";
      return ret;
    }
    
    GARealGenome genome(*bounds, CRALgorithmObjective, (void *)this);
    setGeneticOperators(genome);
    
    // The same process has to be applied to the population
    GAPopulation population(genome, config.population);
    if (config.initializer_type == "Deterministic") {
      population.initializer(GAPopulationInitializer);
    }

    // Define parameters. There is an option to do this from file.
    population.userData((void *) this);

    if (config.debug) {
      cout << "CRGenetics::run --> Creating the algorithm.\n";
    }
    
    delete algorithm;
    algorithm = new GASimpleGA(population);
    cout << "CRGenetics::run --> Setting the GAAlgorithm parameters.\n";
    setGeneticParameters();
    
    if (!config.custom_evolution) {
      if (config.debug) {
	cout << "CRGenetics::run --> Letting the algorithm evolve.\n";
      }
      algorithm->evolve();
    } else {
      customEvolution(ret, t1, true);
    }
    
    // Calculate spended time and show it
    gettimeofday(&t2,NULL);
    cout << "\n\n CRGenetics::run --> Spended time = " << functions::showTime(t1,t2) << endl;
    ret.setExecutionTime(functions::calculateLapseTime(t1, t2));

    // Get best flight plan
    GARealGenome &best = (GARealGenome &) algorithm->statistics().bestIndividual();
    vector<FlightPlan> best_wp(getGeneticFlightPlan(best));
    
    // Get the minimum objetive value and store it for statistics purposes...
    ret.setMinObjetive( best.score() );
    
    // Show the best flight plan
    cout << "CRGenetics::run --> Evolution finished. Best flight plan:\n";
    cout << functions::printToStringVector(best_wp) << endl;
    
    // Show genetic statistics
    cout << algorithm->statistics() << endl;
    
    // Check for collisions in the solution
    bool collision;
// 		float min_d;
    
    
    bool ok;
    try {
      ok = simulateSystem(best_wp, collision);
    } catch(...) {
      ok = false;
    }

    
    
    if (collision || !ok) {
      if (!ok) {
	cerr << "CRGenetics::run --> Error while simulating the best flight plan\n";
      }
      ret.setSolved(false);
      cout  << "CRGenetics::run --> The problem was not solved.\n"; // Min distance = " << min_d << "\n";
    } else {
      cout  << "CRGenetics::run --> The problem was solved successfully.\n";// Min distance = " << min_d << "\n";
    
      if (config.export_trajectories) {
	if (sim->exportTrajectory(config.trajectory_filename)) {
	  cout << "Trajectories exported successfully.\n";
	}
      }
      if (config.export_solution) {
	if (exportSolution(config.solution_filename, best_wp)) {
		cout << "CRGenetics::execute() --> Solution flight plans exported successfully.\n";
	}
      }
      if (config.export_evolution) {
	
      }
		    
      ret.setSolved(true);
    }
	
  }
  
  return ret;
}

GAParameterList CRGenetics::getGeneticParams()
{
  GAParameterList params;
  GeneticConfig &config = dynamic_cast<GeneticConfig&>(* (this->config));
  
  GASimpleGA::registerDefaultParameters(params);
  params.set(gaNpCrossover, config.pCrossover);       // likelihood of doing crossover
  params.set(gaNpMutation, config.pMutation);	// probability of mutation
  params.set(gaNnGenerations, config.generations);	// number of generations
  params.set(gaNscoreFrequency, 1);	// how often to record scores
  params.set(gaNflushFrequency, 10);    // how often to flush scores to file
  params.set(gaNscoreFilename, "bog.dat");
  
  return params;
}


vector< FlightPlan> CRGenetics::getGeneticFlightPlan(const GARealGenome& gen) const
{
	return getFlightPlan(geneToVector(gen));
}



std::vector< float > CRGenetics::getInitialWaypointLength() const
{
	vector<float> ret;
	
	for (int i = 0; i < n_uavs; i++) {
		float length = initial_plans[i][1].distance(initial_plans[i][0]);
													
		ret.push_back(length);
	}
	
	return ret;
}

bool CRGenetics::checkGen(const GARealGenome& gen) const 
{
	vector<FlightPlan> v = getGeneticFlightPlan(gen);
	bool ok = true;
	int cont_uav = 0;
	for (int i = 0;i < v.size() && ok; i++) {
		while (!controlled_UAVs[i]) {
			cont_uav++;
		}
		ok = sim->getParticle(cont_uav)->getController()->checkPlanFeasibility(v.at(i)) == 0;
	}
	
	return ok;
}

void CRGenetics::setBounds()
{
	bool ret_val = true;
	
	delete bounds;
	bounds = new GARealAlleleSetArray();
	
	GeneticConfig &conf = dynamic_cast<GeneticConfig&>(*config);
	
	CRAlgorithm::getBounds();
	
	std::vector<double> &lower_bounds = conf.lower_bounds;
	std::vector<double> &upper_bounds = conf.upper_bounds;
	
	
	try {
		for (int i = 0; i < genome_width ; i++) {
			float upper_val, lower_val, mid_val;
		
			// Get the next values from file
			lower_val = lower_bounds.at(i);
			upper_val = upper_bounds.at(i);
			mid_val = 0.5*(lower_val+upper_val); //mamuso

			// Add them to the AlleleSetArray
			if ( i%3 == 2 && conf.waypoint_dimension == 3 && conf.altitude_levels && !conf.manoeuvre_selection) {
				// Altitude levels stuff
				float *levels = NULL;
				int n_levels = calculateLevels(lower_val, upper_val, mid_val, conf.altitude_step, levels);//mamuso
			
				if (config->debug) {
					cout << " CRAlgorithm::loadBounds --> level array: ";
					for (int i = 0; i < n_levels; i++) {
						cout << levels[i] << " ";
					}
					cout << endl;
				}
			
				bounds->add(n_levels, levels);
				delete[] levels;
			} else {
				//  Normal not discretized bounds
				bounds->add(lower_val, upper_val);
				if (config->debug) {
				  cout << "Adding bounds: " << lower_val << ", " << upper_val << endl;
				}
			}
		}	
	} catch (std::exception &e) {
		cerr << "CRGenetics::loadBounds --> Some error ocurred while loading the bounds.\n";
		throw(e);
	}
	
// 	return ret_val;
}

void CRGenetics::init(ParseBlock& block, AlgorithmConfig* conf)
{
	try {
    resolution::CRAlgorithm::init(block, conf);
		GeneticConfig &config = dynamic_cast<GeneticConfig&>(*this->config);
		genome_width = CRAlgorithm::getProblemDimension();
		setBounds();
	} catch (std::exception &e) {
		cerr << "CRGenetics::init --> Some error ocurred while loading the bounds.\n";
		throw(e);
	}
		
}

Checker* CRGenetics::getAlgorithmChecker() const
{
		Checker* ret =  resolution::CRAlgorithm::getAlgorithmChecker();
		
		return ret;
}

void CRGenetics::getBounds(std::vector< double >& upper, std::vector< double >& lower)
{
    if (config  != NULL) {
      GeneticConfig &conf = dynamic_cast<GeneticConfig&>(*config);
      upper = conf.upper_bounds;
      lower = conf.lower_bounds;
    } else {
      upper.clear();
      lower.clear();
    }
}

ParseBlock* CRGenetics::toBlock() const
{
    ParseBlock *ret = resolution::CRAlgorithm::toBlock();
    
    ret->setProperty("algorithm", "Genetic");
    
    return ret;
}

functions::RealVector CRGenetics::geneToVector(const GARealGenome& gen)
{
  functions::RealVector v(gen.size());
  for (unsigned int i = 0; i < gen.size(); i++) {
    v[i] = gen.gene(i);
  }
  return v;
}
 
void CRGenetics::setGeneticOperators(GARealGenome &genome)
{
  GeneticConfig &config = dynamic_cast<GeneticConfig&>(* (this->config));
  if (config.initializer_type == "Random") {
      cout << "CRGenetics::run --> Random initialization\n";
      genome.initializer(GAGenomeRandomInitializer);
    } else {
      if (config.debug) {
	cout << "CRGenetics::setGeneticOperators --> Deterministic initialization\n";
      }
    }
    
    cout << "CRGenetics::run --> Crossover type: ";
    if (config.crossover_type == "Mixer") {
      cout << "Waypoint mixer\n";
      genome.crossover(GAGenomeUniformWaypointMixer); 
    } else if (config.crossover_type == "Uniform") {
      genome.crossover(GARealUniformCrossoverCheck);
      cout << "Uniform\n";
    } else if (config.crossover_type == "TwoPoints" ||
      config.crossover_type == "TwoPoint"
    ) {
      genome.crossover(GARealGenome::TwoPointCrossover);
      cout << "Two point\n";
    } else if (config.crossover_type == "OnePoint") {
      genome.crossover(GARealGenome::OnePointCrossover);
      cout << "One point\n";
    } else {
      // Default crossover operator
      config.crossover_type = "Uniform";
      genome.crossover(GARealUniformCrossoverCheck);
      cout << "Uniform\n";
    }
    
    cout << "CRGenetics::run --> Mutator type: ";
    if (config.mutator_type == "MaMutator") {
      cout << "MaMutator\n";
      genome.mutator(GARealGaussianMaMutator);
    } else if (config.mutator_type == "Gaussian") {
      cout << "Gaussian\n";
      genome.mutator(GARealGaussianMutatorCheck);
    } else {
      // Default mutator
      config.mutator_type = "Gaussian";
      cout << "Gaussian\n";
      genome.mutator(GARealGaussianMutatorCheck);
    }
    
    // The same process has to be applied to the population
//     GAPopulation population(genome, config.population);
//     if (config.initializer_type == "Deterministic") {
//       population.initializer(GAPopulationInitializer);
//     }
}

void CRGenetics::customEvolution(resolution::CRAlgorithmStatistics& ret, timeval& t1, bool initialize)
{
  GeneticConfig &c= dynamic_cast<GeneticConfig&>(* (this->config));
  timeval t2;
  vector<EvolutionData> ev_data;
  cout << "CRGenetics::run --> Letting the algorithm evolve.\n";
  if (initialize) {
    algorithm->initialize(); 
  }
  iterations = 0;
  bool end_time = false; 
  while(iterations < c.generations && !end_time){
    if (c.debug) {
      cout << "CRGenetics::run --> Iteration " << iterations << ".\n";
    }
    algorithm->step();
    gettimeofday(&t2, NULL);
    EvolutionData current;
    const GARealGenome &curr_best = (const GARealGenome &)algorithm->statistics().bestIndividual();
    current.cost = curr_best.score();
    current.t = functions::calculateLapseTime(t1, t2);
    current.plan = getGeneticFlightPlan(curr_best);
    ev_data.push_back(current);
    iterations++;
    
    // See if the time has expired
    gettimeofday(&t2, NULL);
    if (c.max_time > 0.0 && functions::calculateLapseTime(t1, t2) > c.max_time) {
      end_time = true;
    }
  }
  
  if(c.export_evolution) {
    // Exporting the results of evolution
    saveEvolutionData(c.evolution_file ,ev_data);
  }
  
  ret.setEvolutionData(ev_data);
}

void CRGenetics::setGeneticParameters()
{
  GeneticConfig &config = dynamic_cast<GeneticConfig&>(* (this->config));
  algorithm->parameters(getGeneticParams());
  algorithm->minimize();
  
  if (config.debug) {
    cout << "CRGenetics::run --> Setting the selector function.\n";
  }
  
  GASRSSelector select;
  algorithm->selector(select);
  algorithm->populationSize(config.population);
  cout << "CRGenetics::run --> Population size = " << algorithm->populationSize() << endl;
}


}