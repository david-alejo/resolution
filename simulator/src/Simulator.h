#ifndef SIM_SIMULATOR_H
#define SIM_SIMULATOR_H

#include "CD.h"
#include "Particle.h"
#include "ParticleFactory.h"
#include "FlightPlan.h"
#include "CDFactory.h"
#include <vector>
#include <string>
#include <sparser/all.h> 
#include "ObstacleDetector.h"
#include "ParticleFilter.h"

namespace simulator {
	struct SimConfig {
		bool debug;
		bool collision_stop;
		int max_cont;
		bool max_time;
		double time_coefficent;
		
		SimConfig();
		
		void parseConfig(ParseBlock &config_block);
		
		ParseBlock *toBlock() const;
		
		private:
			
		void init();
	};
	
class Simulator {
  public:
  //! @brief Recommended constructor: Constructor from filename
  //! @param filename Filename
  Simulator(const std::string &filename);
  
  //! @brief Another recommended constructor: Constructor from parse block
  //! @param filename Filename
  Simulator(ParseBlock &block);
  
  //! @brief Default destructor
  virtual ~Simulator();
  
  //! @brief Runs the simulation
  //! @param collision True if a collision has been detected in the system
  //! @retval true The simulation has ended successfully
  //! @retval false Could not simulate the system
  virtual bool run(bool &collision);
  
  //! @brief Reruns the simulation with new plans
  //! @param collision True if a collision has been detected in the system
  //! @param plans The new plans.
  //! @retval true The simulation has ended successfully
  //! @retval false Could not simulate the system
  virtual bool run(bool &collision, const std::vector<FlightPlan> &plans);
  
  //! @brief Runs the simulation with Montecarlo simulation 
  //! This can be useful in simulations with stochastic models (such as ModelSimpleUAVPolarWind)
  //! Can also be useful when uncertainties in the localization are modeled. (TODO)
  //! @param collision True if a collision has been detected in the system
  //! @param n_particles The number of particles in the simulation.
  //! @param max_dev Return a vector with the max deviation of each particle
  //! @param save_traj If true, the trajectory of each particle is saved (useful for representation purposes)
  //! @retval true The simulation has ended successfully
  //! @retval false Could not simulate the system
  virtual bool montecarlo(bool& collision, int n_particles, functions::RealVector& max_dev, 
			  bool save_traj = false, bool collision_stop = true);
  
  //! @brief Runs the simulation with Montecarlo simulation 
  //! This can be useful in simulations with stochastic models (such as ModelSimpleUAVPolarWind)
  //! Can also be useful when uncertainties in the localization are modeled. (TODO)
  //! @param collision True if a collision has been detected in the system
  //! @param n_particles The number of particles in the simulation.
  //! @param max_dev Return a vector with the max deviation of each particle
  //! @param save_traj If true, the trajectory of each particle is saved (useful for representation purposes)
  //! @retval true The simulation has ended successfully
  //! @retval false Could not simulate the system
  virtual bool montecarlo_1_vs_all(bool &collision, int n_particles, functions::RealVector &max_dev, bool save_traj = false);
  
  //!  @BRIEF Exports the last simulated trajectory to a file
  //! @param filename The name of the file to export
  bool exportTrajectory(const std::string &filename) const;
  
  //! @brief Returns the plans that have been commanded to the UAVs
  //! @return These flight plans
  std::vector<FlightPlan> getFlightPlans() const;
  
  //! @brief Sets the plans to be commanded to the UAVs
  //! @param new_plans These flight plans
  void setFlightPlans(const std::vector <FlightPlan> &new_plans);
  
  inline int howManyUAVs() const { 
	  return particles.size();
  }
  
  inline Particle* getParticle(int particle_number) {
	  return particles.at(particle_number);
  }
  
  //! @brief Translate the content into a parse block
  ParseBlock *toBlock();
  
  inline void setCollisionStop(bool new_val) {
    config.collision_stop = new_val;
  }
  
  inline SimConfig &getConfig() {
    return config;
  }
  
  inline const std::vector<std::vector<double> > &getGeometries() const {
    return geometries;
  }
  
  //! @brief Gets the minimum distance between any pair of UAVs after a simulation
  //! @return The minimum miss distance
  inline double getMinDistance() const {return min_distance;}
  
  //! @brief Performs a check in the first and last state of the flight plans. Configurable (initial and/or goal).
  //! @param fp The flight plans of each UAV
  //! @retval true The final state has no collisions
  //! @retval false The final state has collisions
  bool checkFlightPlan(const std::vector<FlightPlan> &fp);
  
  //! @brief Performs a check in the first and last state of the flight plans. Configurable geometries and other things.
  //! @param fp The flight plans of each UAV
  //! @retval true The final state has no collisions
  //! @retval false The final state has collisions
  bool checkFlightPlan(const std::vector<FlightPlan> &fp, const std::vector<double> &new_geometries);
  
  inline CD *getCollisionDetector() {return detector;}
  
  void expandGeometries(double dist);
		      
  std::vector< FlightPlan > importCatec(string filename);
    
    
  ObstacleDetector *getObstacleDetector() const { return obs;}
  
  //! @brief Checks for collision the change of flight plan of UAV 1 taking into account the other as static obstacles
  //! @param fp New flight plan of UAV 1
  //! @PARAM collision Returns whether a collision has been found or not
  //! @retval true The simulation was performed successfully
  //! @retval false Problems were found while performing the simulation
  bool check1vsAll(const FlightPlan &fp, bool &collision);
  
  inline void setTrajectories(const std::vector<std::vector<functions::RealVector > > &new_traj) {
    traj = new_traj;
  }
   
  inline std::vector<std::vector<functions::RealVector > > &getTrajectories() {
    return traj;
  }
  
protected:
  //! Configuration parameters. Loadable from sparser Block
  SimConfig config;
  
  CD *detector; // Collision detector
  
  ObstacleDetector *obs;
  
  std::vector<std::vector<double> > geometries; // The geometries of each particle
  
  std::vector<Particle *> particles; // Will be used to simulate the system.
  
  std::vector<std::string> identificators; // Identificators of each vehicle
  
  std::vector<std::vector<functions::RealVector > > traj; // Trajectory of the system in the last simulation
  
  std::vector<std::vector<double> > max_devs; // Evolution of the max_dev over the time
  
  std::vector<std::vector<std::vector<functions::RealVector> > > monte_traj; // Montecarlo Evolution of the system
  
  double T; // Sample time. It will be loaded from file and will override the Particle, Controller and Model sample time
  
  double min_distance; // Saves the minimum miss distance in the last performed simulation
  
  //! @brief Initializes the function loading data from file
  //! @param filename Filename
  //! @retval true The file has been successfully loaded
  //! @retval false Problems while loading data from files
  virtual bool init(ParseBlock& data);
  
  virtual bool init(ParseBlock& data, ParticleFactory *p);
  
  //! @brief Makes all pointers point to NULL
  virtual void pointersToNULL();
  
  //! @brief Returns a pointer to a sparser's Checker. It is necessary to load data from file
  //! @return The Checker
  virtual Checker *getChecker() const;
  
  //! @brief Deletes all the fields of the class
  virtual void dispose();

  //! @brief Initializes the trajectory attribute
  void initializeTrajectories();
  
  //! @brief Initializes the montecarlo trajectory attribute
  void initializeMonteCarloTrajectories(int n_particles);
  
  //! @brief Initializes the montecarlo trajectory attribute
  void initializeMonteCarloTrajectory(int n_particles, int uav);


  void printParticleState() const;

  //! @brief Returns the state of the models of each particle.
  //! @return A vector with the states
  virtual std::vector<std::vector<double> > getStates() const;

  //! @brief Sets the states of the models of all particles.
  //! @retval true The new states have been changed successfully.
  //! @retval false Some dimensions in the vector mismatch
  virtual bool setStates(const std::vector<std::vector<double> > &new_states);
  
  //! @brief Performs a check in the current state.
  //! @retval false The state has no collisions
  //! @retval true The state has collisions
  virtual bool collisionCheck(const std::vector< std::vector< double > >& position) const;
  
  //! @brief Performs a check of UAV 1 vs all the rest and obstacles.
  //! @retval false The state of UAV 1 has no collisions
  //! @retval true The state of UAV 1 has collisions
  bool collisionCheck1vsAll(const std::vector<std::vector<double> > &position) const;
  
  //! @brief Gets the geometries of all particles
  void setGeometries();
  
  //! @brief Gets the geometries of all particles
  //! @param new_geo A matrix containing the new geometries
  void setGeometries(const std::vector<std::vector<double> > &new_geo);
  
  //! @brief Gets the geometries of all particles. Useful when all vehicles share the same geometry
  //! @param new_geo A vector containing the new geometries
  void setGeometries(const std::vector<double> &new_geo);
  
  //! @brief Default constructor used internally for inheritance purporses
  Simulator();
};




}

#endif //SIM_SIMULATOR_H