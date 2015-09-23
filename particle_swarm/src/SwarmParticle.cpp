/*
    <This class gives a representation of a particle a particle swarm implementation.>
    Copyright (C) <2011>  <David Alejo Teissiere>

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

#include "SwarmParticle.h"
#include <sstream>
#include <functions/functions.h>
#include <functions/RandomNumberGenerator.h>

using namespace functions;
using namespace std;

namespace particle_swarm {
	
SwarmParticle::SwarmParticle() {
	
}
	
SwarmParticle::SwarmParticle(RealVector& data, RealVector& initial_speed, RealVector& lower_bounds, RealVector& upper_bounds, RealVector& up_speed, const boost::function1< double, RealVector& > &evaluator)
{
	this->position = data;
	this->speed = initial_speed;
	this->evaluator = evaluator;
	this->lower_bounds = lower_bounds;
	this->upper_bounds = upper_bounds;
	this->speed_bound = up_speed;
	init();
	evaluate();
	best_cost = cost;
}

	
void SwarmParticle::iterate(const RealVector& global_best_position, double r0, double phi0, RandomNumberGenerator &gen, double inertia_weight)
{
	// Actualize the position
	position = position + speed; 
	
	// And then the speed
	RealVector ran1(position.size(), true);
	RealVector ran2(position.size(), true);

	speed = speed * inertia_weight  + ( (global_best_position - position).componentProduct(ran1) * r0 +
				(best_position - position).componentProduct(ran2) * phi0 );


	// Correct the possible violations of the bounds
	correctBounds();
	evaluate();
}

	
void SwarmParticle::evaluate()
{
	cost = evaluator(this->position);
	if (cost < best_cost) {
		best_cost = cost;
		best_position = position;
	}
}

void SwarmParticle::correctBounds()
{
	for (unsigned int i = 0; i < position.size(); i++) {
		position[i] = minimum(upper_bounds.at(i), position.at(i));
		position[i] = maximum(lower_bounds.at(i), position.at(i));
		speed[i] = minimum(speed_bound.at(i), speed.at(i));
		speed[i] = maximum(-speed_bound.at(i), speed.at(i));
	}
}

string SwarmParticle::toString() const
{
	ostringstream os;
	
	os << "Position: " << position.toString() << "\t";
	os << "Speed: " << speed.toString() << "\t";
	os << "Cost: " << cost << "\t";
	os << "Best cost: " << best_cost << "\t";
	
	return os.str();
}

void SwarmParticle::init()
{
	cost = 1e30;
	best_cost = 1e30;
	best_position = position;
}



}
