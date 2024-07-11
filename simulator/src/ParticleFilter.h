#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <list>
#include <vector>
#include <string>
#include <cmath>
#include "boost/random.hpp"
#include "Particle.h"
#include "Models.h"
#include "functions/RandomNumberGenerator.h"
#include "functions/RealVector.h"


using namespace std;


namespace simulator {
class ParticleFilter{
  public:
  
  /*! @brief Constructor
  @param modelType String with the name of the model type (ModelSimpleUAV...)
  @param num Number of particles of the particle filter
  */
  ParticleFilter(ParseBlock& data);
  
  ParticleFilter(ParseBlock& uav_data, int n_particles, double N_th = 0.0);
  
  ParticleFilter(const Particle *particle, int n_particles, double N_th = 0.0);
  
  /*! @brief Destructor, deletes the Particle list */
  ~ParticleFilter();
  
  
  void init(ParseBlock &data, int n_particles);
  
  void init(const Particle *particle, int n_particles);
  
  /*! @brief Runs one step of all the particles, modifying its states
  @return error (0=OK)
  */
  int runOneStep();
  
  /*! @brief Gets all the particles states in a list
  @return stateList List with all the particles' states (vector <double> *), NULL if the particle is stopped
  */
  std::vector< std::vector<double> *> getAllParticlesStates() const ;
  
  /*! @brief Sets the control vector of all particles
  @param control vector of control references
  */
  void setControl(const std::vector < double > &control);
  
  
  /*! @brief Calculates weights of each particle and Neff (effective number of particles)
  @param ref Reference state vector. Particles nearer to this state are more probable to persist.
  */
  void calculateWeights(const std::vector <double> &ref);
  void calculateWeights();
  
  /*! @brief Makes the resampling process
  
  */
  void resample(const std::vector <double> &ref);
  
  inline double getMaxDev() const {return max_dev;}
  
  inline std::vector<functions::RealVector> getMeanTrajectory() const {return mean_traj;}
  inline functions::RealVector getMeanPosition() const {
    if (mean_traj.size() > 0) {
      return mean_traj.at(mean_traj.size() - 1);
    }else {
      functions::RealVector v;
      return v;
    }
  }
  
  inline Particle *getParticle(unsigned int n_par) {
    if (n_par < particlesList->size()) {
      return particlesList->at(n_par);
    } else {
      return NULL;
    }
  }
  
  void endOnFinalWaypoint();
  
  protected:
  ParticleFilter(){};
  //!Hiding Copy constructor
  ParticleFilter (const ParticleFilter &that){};
  //! Hiding assignment operator
  ParticleFilter & operator = (const ParticleFilter &that){
    *this = that;
    return *this;
  }
  
  //! @brief Gets the sparser checker to verify the integrity of the data
  //! @return The checker.
  Checker *getChecker();
  
	    
  private:

  vector <Particle *> *particlesList;
  double Nth;
  void eraseParticlesList();
  void initializeParticles(ParseBlock& b, unsigned int n_particles);
  std::vector<functions::RealVector> mean_traj;
  double max_dev;

};
}

#endif //#ifndef PARTICLE_FILTER_H
