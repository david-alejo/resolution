/*
    A standard vector particle that will be used in the particle swarm optimizator
    Copyright (C) <2011>  <David Alejo Teissiere>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOURealVector ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef PARTICLE_H__SW
#define PARTICLE_H__SW

#include <boost/function.hpp>
#include <functions/RealVector.h>
#include <functions/RandomNumberGenerator.h>
#include <string>

namespace particle_swarm {

//! @brief This class represents an abstract particle. The method evaluate calls a boost function
class SwarmParticle
{
  public:
    SwarmParticle();
    
    SwarmParticle(functions::RealVector& data, functions::RealVector& initial_speed, 
		  functions::RealVector& upper_bounds, functions::RealVector& lower_bounds, 
		  functions::RealVector& up_speed, 
		  const boost::function1< double, functions::RealVector& > &evaluator);
    
    //! @brief Performs a iteration of the particle
    //! @param best_position The global best position achieved so far
    void iterate(const functions::RealVector &global_best_position, double r0, double phi0, 
		  double inertia_weight);
    
    //! @brief Evaluates the function
    void evaluate(); 
    
    //! @brief Cost getter
    inline double getCost() const {
	    return cost;
    }
    
    //! @brief Cost setter
    inline void setCost(double new_cost) {
      cost = new_cost;
    }
    inline functions::RealVector getPosition() const { return position; };
    
    inline bool setPosition(const functions::RealVector &v) {
      bool ret_val = false;
      if (position.size() == v.size()) {
	ret_val = true;
	position = v;
      }
      return ret_val;
    }
		
		std::string toString() const;
		
  protected:
    double cost; // Minimum cost
    double best_cost; // Minimum cost achieved by the particle
    
    functions::RealVector position; // Current position
    functions::RealVector best_position; // Stores the best position of the particle so far
    functions::RealVector speed; // Current speed
    functions::RealVector upper_bounds, lower_bounds; // Restrictions in the population
    functions::RealVector speed_bound;
    
    boost::function1<double, functions::RealVector &> evaluator;
    
    void correctBounds();
    
    void init();
  private:
    functions::RealVector inertia_weight;
};

}

#endif // PARTICLE_H
