#include "Simulator.h"
#include "ParticleFactory.h"
#include "CDFactory.h"
#include <functions/functions.h>
#include <fstream>
#include <vector>
#include "ParticleFilter.h"

using namespace std;
using functions::RealVector;

namespace simulator {
	
SimConfig::SimConfig()
{
	init();
}

	
	void SimConfig::parseConfig(ParseBlock& config_block)
	{
		// Restore the default parameters
		init();
		
		// Parse parameters if they exist
		if (config_block.hasProperty("debug")) {
			debug = ( *config_block.getProperties("debug")->begin() )->as<bool>();
		} 
		
		if (config_block.hasProperty("collision_stop")) {
			collision_stop = ( *config_block.getProperties("collision_stop")->begin() )->as<bool>();
		} 
		if (config_block.hasProperty("max_time")) {
			max_time = ( *config_block.getProperties("max_time")->begin() )->as<bool>();
		} 
		if (config_block.hasProperty("time_coefficient")) {
			time_coefficent = config_block("time_coefficient").as<double>();
		} 
		if (config_block.hasProperty("max_cont")) {
			max_cont = ( *config_block.getProperties("max_cont")->begin())->as<long>();
		}
	}
	
void SimConfig::init()
{
	debug = false;
	collision_stop = true;
	max_cont = 10000;
	max_time = true;
	time_coefficent = 1.2;
}

ParseBlock *SimConfig::toBlock() const
{
  ParseBlock *target_block = new ParseBlock;
  target_block->setProperty("debug", functions::boolToString(debug));
  target_block->setProperty("collision_stop", functions::boolToString(collision_stop));
  target_block->setProperty("max_time", functions::boolToString(max_time));
  target_block->setProperty("time_coefficient", functions::numberToString(time_coefficent));
  ostringstream max_cont_str;
  max_cont_str << max_cont;
  target_block->setProperty("max_cont", max_cont_str.str());
  
  return target_block;
}


Simulator::Simulator(const std::string& filename)
{
	ParseBlock data;
	
	try {
		data.load(filename.c_str());
		init(data);
		
	} catch (exception &e) {
		std::cout << "Simulator::init (from file) --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
}

Simulator::Simulator(ParseBlock& block)
{
	init(block);
}

bool Simulator::init(ParseBlock& data)
{
  ParticleFactory p;
  return init(data, &p);
}

	
bool Simulator::init(ParseBlock& data, ParticleFactory *p_)
{
  ParticleFactory &p = *p_;
	obs = NULL;
	min_distance = 0.0;
	identificators.clear();
	particles.clear();
	pointersToNULL();
	Checker *check = NULL;
	bool ret_val = true;
		
	try {
		check = getChecker();
		
		if (data.hasBlock("obstacles")) {
		  obs = new ObstacleDetector(data["obstacles"]);
		} else {
		  obs = NULL;
		}
			
		data.checkUsing(check);
		
		T = data("T").as<double>();
			
		ParseBlock::Blocks *uavs = data.getBlocks("uav");
			
		ParseBlock::Blocks::iterator it = uavs->begin();
		for ( ; it != uavs->end(); it++) {
			ParseBlock &curr_uav = **it;
			if (curr_uav.hasProperty("particle_file")) {
				// Create particle from file
				particles.push_back( p.createFromFile( curr_uav("particle_type").as<std::string>(), curr_uav("particle_file").as<std::string>()) );
			} else {
				// Create particle from block
				particles.push_back( p.createFromBlock( curr_uav("particle_type").as<std::string>(), curr_uav) );
			}
			
			// Retrive the identificator strings
			identificators.push_back(curr_uav("id").value);
		}
		
		// Initialize the geometries:
		setGeometries();
			
		// Retrieve configuration data
		if (data.hasBlock("config")) {
			config.parseConfig(data["config"]);
		}
		
		// See if the collision detector has been declared
		if (data.hasProperty("collision_detector_type")) {
			CDFactory cd_fac;
			detector = cd_fac.create(data("collision_detector_type").value);
		}
		
		if (data.hasProperty("catec_file")) {
		  vector<FlightPlan> fp_v = importCatec(data("catec_file").value);
		  setFlightPlans(fp_v);
		}
	} catch (std::exception &e) {
		std::cout << "Simulator::init (from file) --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
		
	if (config.debug) {
		printParticleState();
	}
	
// 	initial_states = getStates();
		
	delete check;
	return ret_val;
}
	
void Simulator::pointersToNULL()
{
	detector = NULL;
	obs = NULL;
}


Checker* Simulator::getChecker() const
{
  Checker *check = new Checker();
  
  Checker *uav_check = new Checker();
  uav_check->addProperty("id", new NTimes(1));
  uav_check->addProperty("particle_type", new NTimes(1));
  
  check->addBlock("uav", new OneOrMore());
  check->addChecker("uav", uav_check);
  check->addProperty("T", new NTimes(1));
  
  return check;
}

bool Simulator::run(bool& collision, const std::vector< FlightPlan >& plans)
{
	initializeTrajectories();
	setFlightPlans(plans);
	return run(collision);
}


bool Simulator::run(bool& collision)
{
  collision = false;
  bool ret_val = true;
  min_distance = 1e30;
  
  traj.clear();
  
  int uavs = particles.size();
	  
  // ---------------------- DEBUG -------------------
  if (config.debug) {
    cout << "Simulator::run --> Number of UAVs: " << uavs << endl;
    
    for (int i = 0; i <uavs; i++) {
      cout << "Flight plan: " << i + 1 << " " << particles.at(i)->getController()->getFlightPlan().toString() << endl;
    }
  }
	  
  // Vector that saves wich particle has completed its flight plan
  vector<bool> part_stopped(particles.size(), false);
  
  initializeTrajectories();
	  
  bool ended = false;
  int cont = 0;
  
  // Reserve useful things that will be used in the simulation loop
  vector<double> part_state;
  vector<double> part1_state;
  bool new_col;
  
  int max_cont = -1;
  
  for (int i = 0; i < uavs && ret_val; i++) {
    particles.at(i)->setTimeStep(T);
    max_cont = functions::maximum((int)(particles.at(i)->getController()->calculateETA() * config.time_coefficent / T), max_cont);
      
  }
  
  while( ret_val && !ended && (cont < max_cont || config.max_cont == 0)){
    ended = true;
    vector<vector <double> > position; // Will store all objects to be checked
    
    if (config.debug) {
      cout << "Simulator::Run --> T:  " << cont * T << " ";
    }
    
    for (int i = 0; i < uavs && ret_val; i++) {
      // Simulate each particle
      if (!part_stopped[i]) {
	if (config.debug) {
	  cout << " " << "State " << i << " " << functions::printVector(particles[i]->getState()) << endl;
	}
	
	particles[i]->runOneStep();
	traj[i].push_back(particles[i]->getState());
	
	if (!particles[i]->isStopped()) {
	  ended = false; // Still a simulated particle alive --> keep simulating
	}
	part_stopped[i] = particles[i]->isStopped();
      }
      part_state = particles[i]->getState();
      vector<double> pos(part_state.begin(), part_state.begin() + 3);
      position.push_back(pos);
    } 
    
    // TODO: CD call!!!! 
    new_col = collisionCheck(position);
	    
    if (cont == 0 && new_col) {
      if (config.debug)
	cout << "bool Simulator::run --> Initial condition conflictive!!\n";
	      
      ret_val = false;
    }
	    
    if ( config.debug && new_col) {
      cout << "Simulator::run --> Collision detected\n";
    }
    collision |= new_col;
    
    
    if( config.collision_stop ) {
      ended |= collision;
    } else {
      // Calculate the minimum distance between two UAVs 
      for (int i = 0; i < uavs; i++) {
	part1_state = particles[i]->getState();
	for (int j = i + 1; j < uavs; j++) {
	  part_state  = particles[j]->getState();
	  float dist_aux2 = 0.0; 
	  for (int k = 0; k < 3; k++) {
	    dist_aux2 = functions::maximum<double>(fabs(part1_state[k] - part_state[k]), dist_aux2);
	  }
  
	  min_distance = functions::minimum<double>(dist_aux2, min_distance);
	}
      }
    }
    cont++;
  } // End of the simulation loop.
	  
  if (config.collision_stop) {
	  min_distance = 0.0;
  }
  
  if (cont >= config.max_cont || cont >= max_cont) {
// 	  ret_val = false;
  }
  
  return ret_val;
}

bool Simulator::montecarlo(bool& collision, int n_particles, RealVector& max_dev, bool save_traj, bool collision_stop)
{
  collision = false;
  bool ret_val = true;
  min_distance = 1e30;
  max_dev.clear();
  
  // Initialize max_dev
  for (unsigned int i = 0; i < howManyUAVs(); i++) {
    max_dev.push_back(0.0);
  }
  
  traj.clear();
  
  int uavs = particles.size();
	  
  // ---------------------- DEBUG -------------------
  if (config.debug) {
    cout << "Simulator::montecarlo --> Number of UAVs: " << uavs << endl;
    
    for (int i = 0; i <uavs; i++) {
      cout << "Flight plan: " << i + 1 << " " << particles.at(i)->getController()->getFlightPlan().toString() << endl;
      cout << "Geometry: " << i + 1 << " " << functions::printVector(particles.at(i)->getGeometry()) << endl;
    }
  }
  
  initializeTrajectories();
  if (save_traj) {
    initializeMonteCarloTrajectories(n_particles);
  }
  int cont = 0;
  int max_steps = 0;
  
  // Perform a montecarlo simulation for each particle and save the trajectory
  vector<RealVector> void_traj;
  vector<ParticleFilter *> pfs;
  
  // Initialize the particle filters
  for (int i = 0; i < uavs && ret_val; i++) {
    particles.at(i)->setTimeStep(T);
    pfs.push_back(new ParticleFilter(particles.at(i), n_particles));
  }
  vector<bool> ended(uavs, false);
  bool end = false;

  
  vector<vector <double> > position(uavs); // Will store all objects to be checked
  vector<double> pos(3);
  while (!end) {
    end = true;
    for (int i = 0; i < uavs; i++) {
      if (!ended.at(i)) {
	ended.at(i) = pfs.at(i)->runOneStep() != 0;
	end &= ended.at(i);
	
	// Get data when the simulation finishes
	if (ended.at(i)) {
	  if (config.debug) {
	    cout << "simulator-->montecarlo(). Traj " << i + 1 << "size = " << traj.at(i).size() << endl;
	  }
	  max_steps = functions::maximum<int>(traj.at(i).size(), max_steps);
	}
	if (save_traj) {
	  ParticleFilter *pf = pfs.at(i);
	  for (unsigned int n_p = 0; n_p < n_particles; n_p++) {
	    monte_traj.at(i).at(n_p).push_back(pf->getParticle(n_p)->getState());
	  }
	}
	
	traj.at(i).push_back(pfs.at(i)->getMeanPosition());
      }
      
      // Store the position of the particle for collision detection
      for (int k = 0;k < 3;k++) {
	pos[k] = traj.at(i).at(traj.at(i).size() - 1).at(k);
      } 
      position.at(i) = pos;
    }
    
    // Get the current max_dev
    for (int i = 0; i < uavs; i++) {
      double curr_dev = pfs.at(i)->getMaxDev();
      detector->expandGeometry(geometries.at(i), curr_dev);
      max_dev[i] = curr_dev;
    }
    // Store the deviation in the history
    max_devs.push_back(max_dev);
    
    collision |= collisionCheck(position);
    
    end |= collision && collision_stop;
    
    for (int i = 0; i < uavs; i++) {
      detector->expandGeometry(geometries.at(i), -pfs.at(i)->getMaxDev());
    }
  }
  
  if (config.debug) {
    cout << "Simulations done. Max_steps = " << max_steps;
    cout << "\tMax_dev = " << max_dev.toString() << endl;
  }
  
  return ret_val;
}

bool Simulator::montecarlo_1_vs_all(bool& collision, int n_particles, RealVector& max_dev, bool save_traj)
{
  // Initialization
  collision = false;
  bool ret_val = true;
  min_distance = 1e30;
  int uavs = howManyUAVs();

  // Initialize max_dev
  for (unsigned int i = max_dev.size(); i < uavs; i++) {
    max_dev.push_back(0.0);
  }
  max_dev.at(0) = 0.0;
//   max_devs.at(0).clear();
  
  // Initialize the proper trajectories
  vector<RealVector> void_traj;
  for (unsigned int i = max_dev.size(); i < uavs; i++) {
    traj.push_back(void_traj);
  }
  traj.at(0).clear();
  if (save_traj) {
    initializeMonteCarloTrajectory(n_particles, 0);
  }
  
  // Initialize the Particle Filter
  ParticleFilter *pf = new ParticleFilter(particles.at(0), n_particles);
  
  vector<vector <double> > position(uavs); // Will store all objects to be checked
  vector<double> pos(3);
  bool end = false;
  int cont = 0;
  while (!end && !collision) {
    end = pf->runOneStep() != 0;
	
    // Get data when the simulation finishes
    if (save_traj) {
      for (unsigned int n_p = 0; n_p < n_particles; n_p++) {
	monte_traj.at(0).at(n_p).push_back(pf->getParticle(n_p)->getState());
      }
    }
    
    // Store the position
    traj.at(0).push_back(pf->getMeanPosition());
    // Store the position of the particle for collision detection
    for (unsigned int i = 0; i < uavs; i++) {
      for (int k = 0;k < 3;k++) {
	pos[k] = traj.at(i).at(traj.at(i).size() - 1).at(k);
      }
      position.at(i) = pos;
    }
    
    // Get the current max_dev
    double curr_dev = pf->getMaxDev();
    max_dev[0] = curr_dev;
      
    // Broaden the safety region
    for (int i = 1; i < uavs; i++) {
      if (max_devs.size() > cont) {
	max_dev[i] = max_devs.at(cont).at(i);
      }
      detector->expandGeometry(geometries.at(i), max_dev.at(i));
    }
    // Store the deviation in the history
    
    if (cont < max_devs.size()) {
      max_devs.at(cont).at(0) = curr_dev;
    } else {
      max_devs.push_back(max_dev);
    }
    
    collision = collisionCheck1vsAll(position);
    
    for (int i = 0; i < uavs; i++) {
      detector->expandGeometry(geometries.at(i), -max_dev.at(i));
    }

    cont++;
  }
  
  return ret_val;
}



bool Simulator::exportTrajectory(const std::string& filename) const
{
	bool ret_value = true;
	
	try {
		ofstream ofs;
		ofs.open(filename.c_str());
		
		ret_value = ofs.is_open();
	
		for (unsigned int uav = 0; uav < traj.size() && ret_value; uav++) {
			ostringstream os;
			os << "uav{" << uav + 1 << "} ";
			ofs << functions::matrixToMatlabString(os.str(), traj[uav]) << endl;
		}
		
		ofs.close();
	} catch (exception &e) {
		cerr << "Simulator::exportTrajectory --> error while exporting file" << endl;
		throw(e);
	}
	
	return ret_value;
}

Simulator::~Simulator()
{
  dispose();
}

void Simulator::dispose()
{
  delete detector;

  for (unsigned int i = 0; i < particles.size(); i++) {
    delete particles[i];
  }

  particles.clear();

  pointersToNULL();
}

void Simulator::initializeTrajectories()
{
  traj.clear();
  
  vector<RealVector > void_traj;
  
  for (unsigned int i = 0; i < particles.size(); i++) {
    traj.push_back(void_traj);
    particles.at(i)->setStopped(false);
  }
  
  for (int i = 0; i < howManyUAVs(); i++) {
    particles.at(i)->getModel()->setState(particles.at(i)->getModel()->getInitialState());
    particles.at(i)->getController()->setState(particles.at(i)->getModel()->getInitialState());
    particles.at(i)->getController()->reset();
  }
}

void Simulator::initializeMonteCarloTrajectories(int n_particles)
{
  monte_traj.clear();
   vector<vector<functions::RealVector> > void_trajs;
  
  for (unsigned int i = 0; i < particles.size(); i++) {
    monte_traj.push_back(void_trajs);
    initializeMonteCarloTrajectory(n_particles, i);
  }
}

void Simulator::initializeMonteCarloTrajectory(int n_particles, int uav)
{
  vector<functions::RealVector > void_traj;
  vector<vector<functions::RealVector> > void_trajs;
  for (unsigned int i = monte_traj.size(); i <= uav; i++) {
    monte_traj.push_back(void_trajs);
  }
  
  monte_traj.at(uav).clear();
  for (unsigned int j = 0; j < n_particles;j++) {
    monte_traj.at(uav).push_back(void_traj);
  }
}

void Simulator::printParticleState() const
{
  for (unsigned int i = 0; i < particles.size(); i++) {
    cout << "Particle " << i << dynamic_cast<ParticleSimple *>(particles[i])->toString() << endl;
  }
}

vector< FlightPlan > Simulator::getFlightPlans() const
{
  vector<FlightPlan> ret_val;
  
  for (unsigned int i = 0; i < particles.size(); i++) {
    ret_val.push_back(particles[i]->getController()->getFlightPlan());
  }
  
  return ret_val;
}

void Simulator::setGeometries()
{
  geometries.clear();
  for (unsigned int i = 0; i < particles.size(); i++) {
    geometries.push_back(particles[i]->getGeometry());
  }
  
  if (config.debug) {
    cout << "Simulator::setGeometries --> vector obtained: " << functions::printMatrix(geometries) << endl;
  }
}

void Simulator::setFlightPlans(const std::vector< FlightPlan >& new_plans)
{
	if ( new_plans.size() != particles.size() ) {
		std::runtime_error e("Simulator::setFlightPlans --> new plan dimension does not match number of vehicles.\n");
		throw e;
	}
	
	for (unsigned int i = 0; i < new_plans.size(); i++) {
		Particle *par = particles.at(i);
		par->getController()->setFlightPlan(new_plans.at(i));
		
		vector<double> initial = par->getModel()->getInitialState();
		initial.at(0) = new_plans.at(i).at(0).x;
		initial.at(1) = new_plans.at(i).at(0).y;
		// TODO: What if the model only has two dimensions for position
		if (initial.size() > 2) {
		  initial.at(2) = new_plans.at(i).at(0).z;
		}
		if (initial.size() > 3 && new_plans.at(i).size() >= 2) {
		  // Set the proper heading if available. TODO: make this configurable (sometimes the original heading has to be kept)
		  initial.at(3) = new_plans.at(i).at(0).getHeadingTo(new_plans.at(i).at(1));
		}
		particles.at(i)->getModel()->setInitialState(initial);
		
	}
}

vector< std::vector< double > > Simulator::getStates() const
{
	vector<vector<double> > ret;
	
	for (unsigned int i = 0; i < particles.size(); i++) {
		ret.push_back(particles.at(i)->getState());
	}
	
	return ret;
}

bool Simulator::setStates(const std::vector<std::vector<double> > &new_states) {
	bool ret_val = true;
	
	if (new_states.size() != particles.size()) {
		ret_val = false;
	} else {
		for (unsigned int i = 0; i < particles.size(); i++) {
			particles.at(i)->setState(new_states.at(i));
		}
	}
	
	return ret_val;
}

ParseBlock* Simulator::toBlock()
{
  ParseBlock *ret_val = new ParseBlock;
  
  // Set properties:
  ostringstream T_str;
  T_str << T;
  ret_val->setProperty("T", T_str.str());
  ret_val->setBlock("config", config.toBlock());
  
  if (detector != NULL) {
    ret_val->setProperty("collision_detector_type", detector->getType());
  }
  
//   setFlightPlans(getFlightPlans());
  
  // Set particles:
  for (unsigned int k = 0; k < particles.size(); k++) {
    ParseBlock *par_block =  particles.at(k)->toBlock();
    par_block->setProperty("id", identificators.at(k));
    ret_val->setBlock("uav", par_block);
  }
  
  return ret_val;
}

bool Simulator::checkFlightPlan(const vector< FlightPlan >& fp, const std::vector< double >& new_geometries)
{
  std::vector<vector<double> > old_geos = geometries;
  setGeometries(new_geometries); // Use the new geometries
  
  bool ret_val = checkFlightPlan(fp);
  
  setGeometries(old_geos); // Then restore the class to its original state
  
  return ret_val;
}


bool Simulator::checkFlightPlan(const std::vector<FlightPlan> &fp) 
{
  bool ret_val = true;
  
  if (detector != NULL) {
    vector<vector<double> > final_positions;
    vector<vector<double> > initial_positions;
    
    if (config.debug) {
      cout << "Particles size = " << particles.size() << "\tFlight plan size: " << fp.size() << endl;
    }
  
    for (unsigned int i = 0; i < particles.size(); i++) {
      vector<double> init_pos;
      vector<double> last_pos;
      const functions::Point3D &last_wp = fp.at(i).at(fp.at(i).size() - 1);
      const functions::Point3D &first_wp = fp.at(i).at(0);
      last_pos.push_back(last_wp.x);
      last_pos.push_back(last_wp.y);
      last_pos.push_back(last_wp.z);
      init_pos.push_back(first_wp.x);
      init_pos.push_back(first_wp.y);
      init_pos.push_back(first_wp.z);
      
      final_positions.push_back(last_pos);
      initial_positions.push_back(init_pos);
    }
    
    // The collision check with initial and final position has to succeed with the double geometry
    // 
    ret_val = !collisionCheck(final_positions) && !collisionCheck(initial_positions);
    
    if (config.debug) {
      cout << "Simulator::checkFlightPlan --> checking the plan: " << functions::printToStringVector<>(fp) << endl;
      cout << "Simulator::checkFlightPlan --> checking the initial position: " << functions::printMatrix(initial_positions) << endl;
      cout << "Simulator::checkFlightPlan --> checking the final position: " << functions::printMatrix(final_positions) << endl;
      if (ret_val) {
	cout << "Simulator::checkFlightPlan --> no collisions. " << endl;
      } else {
	cout << "Simulator::checkFlightPlan --> collisions detected." << endl;
      }
    }
  }
  
  return ret_val;
}

bool Simulator::check1vsAll(const FlightPlan& fp, bool& collision)
{
  bool ret = true;
  bool ended = false;
  int uavs = particles.size();
  int cont = 0;
  collision = false;
  
  traj.at(0).clear();
  
  if (particles.size() > 0 || particles.at(0) == NULL) {
    Particle *part = particles.at(0);
    part->getController()->setFlightPlan(fp); // actualize
    
    // Initialize particle
    part->getModel()->setState(part->getModel()->getInitialState());
    part->getController()->setState(part->getModel()->getInitialState());
    part->getController()->reset();
    part->setStopped(false);
    
    // Main loop
    while( ret && !ended && (cont < config.max_cont || config.max_cont == 0)){
      vector<vector <double> > position; // Will store all objects to be checked
      bool new_col = false;
    
      if (config.debug) {
	cout << "Simulator::Run --> T:  " << cont * T << " ";
      }
      
      // Simulate and add to the particle's trajectory
      part->runOneStep();
      ended = part->isStopped();
      traj.at(0).push_back(part->getState());
      if (config.debug) {
	cout << " " << "State of UAV 0 " << functions::printVector(particles[0]->getState()) << endl;
      }
    
      // CD check
      int index_;
      for (int i = 0; i < uavs && ret; i++) {
	vector<functions::RealVector> &curr_traj = traj.at(i);
	index_ = functions::minimum<int>(cont, curr_traj.size() - 1);
	vector<double> pos(curr_traj.at(index_).begin(), curr_traj.at(index_).begin() + 3);
	position.push_back(pos);
      }
      new_col = collisionCheck1vsAll(position);
      
      if (cont == 0 && new_col) {
	if (config.debug)
	  cout << "bool Simulator::check1vsAll --> Initial condition conflictive!!\n";
      } else if ( config.debug && new_col) {
	cout << "Simulator::check1vsAll --> Collision detected\n";
      }
      collision |= new_col;
//       ret = false;
      cont++;
    } 
  } else {
    ret = false;
  }
  
  return ret;
}


bool Simulator::collisionCheck(const std::vector<std::vector<double> > &position) const
{
  bool ret_val = false;
  
  if (detector != NULL) {
    ret_val = detector->detectCollision(position, geometries);
    if (ret_val) {
//       cout << "Vehicle collision. Position " << functions::printVector(position.at(0)) << "\n";
    }
  }
  
  if (obs != NULL && !ret_val) {
    for (unsigned int i = 0; i < position.size(); i++) {
      functions::RealVector v (position.at(i));
      ret_val = obs->collisionCheck(v);
      if (ret_val) {
// 	cout << "Obstacle collision. UAV = " << i << ". Position = " << v.toString() << "\n";
      }
    }
     
  }
  
  return ret_val;
}

bool Simulator::collisionCheck1vsAll(const std::vector<std::vector<double> > &position) const
{
  bool ret_val = false;
  
  if (detector != NULL) {
    ret_val = detector->detectCollision1vsAll(position, geometries);
//     if (ret_val) {
//       cout << "Vehicle collision. Position " << functions::printVector(position.at(0)) << "\n";
//     }
  }
  
  if (obs != NULL && !ret_val && position.size() > 0) {
    functions::RealVector v (position.at(0));
    ret_val = obs->collisionCheck(v);
//     if (ret_val) {
//       cout << "Obstacle collision. Position" << v.toString() << "\n";
//     }
  }
  
  return ret_val;
}

void Simulator::setGeometries(const vector< vector< double > >& new_geo)
{
  for (unsigned int i = 0; i < howManyUAVs(); i++) {
    Particle *p = particles.at(i);
    p->setGeometry(new_geo.at(i));
  }
  setGeometries();
}

void Simulator::setGeometries(const vector< double >& new_geo)
{
  for (unsigned int i = 0; i < howManyUAVs(); i++) {
    Particle *p = particles.at(i);
    p->setGeometry(new_geo);
  }
  setGeometries();
}

void Simulator::expandGeometries(double dist)
{
  for (unsigned int i = 0; i < particles.size() && detector != NULL; i++) {
    vector<double> aux = particles[i]->getGeometry();
    detector->expandGeometry(aux, dist);
    particles[i]->setGeometry(aux);
  }
}




vector<simulator::FlightPlan> Simulator::importCatec(string filename)
{
  vector<vector<double> > mat;
  bool ok = functions::getMatrixFromFile(filename, mat);
  vector<simulator::FlightPlan> ret;
  
  functions::FormattedTime t;
  t.getTime();
  
  obs = NULL;
  
  for (unsigned int i = 0; i < particles.size() && ok; i++) {
    FlightPlan fp;
    fp.importFromCatec(mat, i, t);
    ret.push_back(fp);
    if (fp.size() > 0) {
      vector<double> v = particles.at(i)->getState();
      v[0] = fp.at(0).x;
      v[1] = fp.at(0).y;
      v[2] = fp.at(0).z;
      if (v.size() > 2) {
	v[3] = fp.at(0).getHeadingTo(fp.at(1));
      }
      particles.at(i)->setState(v);
    }
  }
  
  return ret;
}

Simulator::Simulator()
{
  detector = NULL;
  obs = NULL;
}

}