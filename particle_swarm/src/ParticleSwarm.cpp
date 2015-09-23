/*
    <one line to give the program's name and a brief idea of what it does.>
    copyright (C) <year>  <name of author>

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

#include "ParticleSwarm.h"
#include "boost/random/uniform_real.hpp"
#include <sstream>
#include <functions/RandomNumberGenerator.h>
#include <functions/functions.h>

using namespace functions;
using namespace std;

namespace particle_swarm {
	
ParticleSwarm::ParticleSwarm(int population, const RealVector& upper, const RealVector& lower, const RealVector& speed,  boost::function1< double, RealVector& >& evaluator, bool debug, double r0, double phi0, double inertia_weight)
{
	// Default parameters:
	current_iteration = 0;
	this->r0 = r0;
	this-> phi0 = phi0;
	this->upper_bounds = upper;
	this->lower_bounds = lower;
	this->evaluator = evaluator;
	this->speed_bounds = speed;
	this->debug = debug;
	this->inertia_weight = inertia_weight;
	evaluation_counter++;
	
	gen = new RandomNumberGenerator;
	
	
	// Explore!!
	randomPopulation(population);
// 	explore(iterations);
	
	save_evolution = false;
}

void ParticleSwarm::explore(int iterations)
{
	for (int i = 0; i < iterations; i++) {
		oneStep();
	}
}

void ParticleSwarm::oneStep()
{
  vector<functions::RealVector> v;
  for (unsigned int j = 0; j < particles.size(); j++) {			
    particles.at(j).iterate(best.getPosition(), r0, phi0, *gen, inertia_weight);
    evaluation_counter++;
    // Checker whether there exists a new best or not
    if (particles.at(j).getCost() < best.getCost()) {
      best = particles.at(j);
    }
    
    if (save_evolution) {
      v.push_back(particles.at(j).getPosition());
    }
  }
  if (debug) {
    cout << populationToString();
  }
  if(save_evolution) {
    evolution.push_back(v);
  }
}

void ParticleSwarm::randomPopulation(int size)
{
	particles.clear();
	
	if (gen == NULL) {
	  cerr << "ParticleSwarm::randomPopulation --> Unexpectedly, random number generator has not been initialized.\n";
	  return;
	}
// 	randomize();
	
	for (int i = 0; i < size; i++) {
		RealVector v, speed;
		for (int j = 0; j < lower_bounds.size(); j++) {
			double length = upper_bounds.at(j) - lower_bounds.at(j);
			
			v.push_back(lower_bounds.at(j) +  gen->rnd01() * length);
			// Random speed
			
			length = 2 * speed_bounds.at(j);
			speed.push_back(gen->rnd01() * length - speed_bounds.at(j));
		}
		SwarmParticle p(v, speed, lower_bounds, upper_bounds, speed_bounds, evaluator);
		evaluation_counter++;
		
		if (debug) {
			cout << "Created Particle: " << p.toString() << "\n";
		}
		particles.push_back(p);
		
		if (i == 0) {
			best = p;
		} else {
			if (best.getCost() > p.getCost()) {
				best = p;
			}
		}
	}
}

std::string ParticleSwarm::toString() const
{
	ostringstream os;
	
	os << "ParticleSwarm::toString --> Lowest cost: " << best.getCost() << "\t";
	os << "Best position: " << best.getPosition().toString() << endl;
	os << "Bounds: " << lower_bounds.toString() << "\t" << upper_bounds.toString() << endl;
	os << "Number of evaluations so far: " << evaluation_counter << endl;
	
	return os.str();
}

std::string ParticleSwarm::populationToString() const
{
	ostringstream os;
	
	for (unsigned int i = 0; i < particles.size(); i++) {
		os << particles.at(i).toString() << endl;
	}
	
	return os.str();
}

ParticleSwarm::~ParticleSwarm()
{
  dispose();
}

void ParticleSwarm::dispose()
{
  delete gen;
  gen = NULL;
  evolution.clear();
}

bool ParticleSwarm::setParticle(unsigned int j, const RealVector& v)
{
  bool ret_val = particles[j].setPosition(v);
  particles[j].evaluate();
 
  // Checker whether there exists a new best or not
  if (particles.at(j).getCost() < best.getCost()) {
    best = particles.at(j);
  }
		  
  return ret_val;
}

bool ParticleSwarm::exportEvolution(const string& filename) const
{
  bool ret_val = true;
  
  for (unsigned int i = 0; i < evolution.size() && ret_val; i++) {
    ostringstream os;
    ostringstream name;
    
    for (unsigned int j = 0; j < evolution.at(i).size(); j++) {
      const functions::RealVector &v = evolution.at(i).at(j);
      os << v.toMatlabString() << endl;
    }
    name << filename << "_" << i;
    ret_val = functions::writeStringToFile(name.str(), os.str());
  }
  
  return ret_val;
}


}
