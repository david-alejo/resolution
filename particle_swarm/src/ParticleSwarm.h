/*
    <This is a simple implementation of the Particle Swarm algorithm useful when the objective is to minimize
     a real function. The input parameter is a real vector. >
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

#ifndef PARTICLESWARM_H
#define PARTICLESWARM_H

#include "SwarmParticle.h"
#include <vector>
#include <functions/RealVector.h>
#include <functions/RandomNumberGenerator.h>
#include <string>
#include <boost/function.hpp>

namespace particle_swarm {

class ParticleSwarm
{
  public:
    //! @brief Recommended constructor with all necessary parameters
    //! @param population The number of particles stored in the algorithm
    //! @param iterations The number of iterations performed in the first exploration
    //! @param upper Upper bound vector
    //! @param lower Lower bound vector
    //! @param speed Speed bound vector
    //! @param evaluator The function that will be used for evaluation purposes
    //! @param acceleration_multiplier Multiplies the changes in speed each iteration
    //! @param inertia_weight The next speed is the current*inertia_weight + changes
    ParticleSwarm(int population, const functions::RealVector &upper, const functions::RealVector &lower,
		    const functions::RealVector &speed, boost::function1<double, functions::RealVector &> &evaluator, 
		    bool debug = false, double r0 = 0.1, double phi0 = 0.9, double inertia_weight = 1.0);
    
    //! @brief Destructor
    ~ParticleSwarm();
    
    //! @brief Performs an exploration of the desired number of iterations (incrementally)
    //! @param iterations The desired number of iterations to perform
    void explore(int iterations);
    
    //! @brief Brief information about the class
    //! @return A string that represents the information stored in this class
    std::string toString() const;
    
    //! @brief Gives information about the population
    //! @return A string with the information of every particle
std::string populationToString() const;
    
    //! @brief Retrieves the best position so far
    //! @return A vector with the best position
    inline functions::RealVector getBestPosition() const {
	    return best.getPosition();
    }
    
    //! @brief Gets the best cost so far
    //! @return The best cost
    double bestCost() const {
	    return best.getCost();
    }
    
    double getBestCost() const {
      return best.getCost(); 
    }
    
    inline void setDebug(bool new_dbg) {
	    debug = new_dbg;
    }
    
    //! @brief Performs one step in the exploration
    void oneStep();
    
    bool setParticle(unsigned int j, const functions::RealVector &v);
    
    void setSaveEvolution(bool new_value) {
      save_evolution = new_value; // Stores the positions of all particles in each iteration into an array
    }
    bool exportEvolution(const std::string &filename) const;
	  
  protected:
  // The set of particles
  std::vector<SwarmParticle> particles;
  // Stores the best particle so far
  SwarmParticle best;
  // The function that will be used in order to evaluate the fitness of each particle
  boost::function1<double, functions::RealVector&> evaluator;
  
  std::vector<std::vector<functions::RealVector> > evolution;
  bool save_evolution;
  
  bool debug;
  
  int evaluation_counter;
  
  // Bounds stuff
  functions::RealVector upper_bounds;
  functions::RealVector lower_bounds;
  functions::RealVector speed_bounds;
  
  // The current iteration
  int current_iteration;
  
  double r0, phi0, inertia_weight; // Acceleration parameters
  
  //! @brief Generates a random population with uniform distribution in the search space (taking into account the bounds)
  //! @param size The population size
  void randomPopulation(int size);
  
  //! @brief Frees all resources that have been dynamically gotten.
  void dispose();
};

}

#endif // PARTICLESWARM_H
