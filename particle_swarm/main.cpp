#include <iostream>

#include "src/ParticleSwarm.h"
#include <functions/RealVector.h>
#include <functions/ArgumentData.h>
#include <math.h>


using namespace std;
using namespace particle_swarm;
using namespace functions;


double evaluator(RealVector &v);

int main(int argc, char **argv) {
	RealVector up;
	RealVector low;
	RealVector sp;
	double r0 = 0.1;
	double phi0 = 0.9;
	double population = 10;
	double iterations = 10;
	int dim = 3;
	
	ArgumentData arg(argc, argv);
	
	if ( arg.isOption("r0") ) {
		arg.getOption("r0", r0);
	}
	if ( arg.isOption("phi0") ) {
		arg.getOption("phi0", phi0);
	}
	if ( arg.isOption("population") ) {
		arg.getOption("population", population);
	}
	if ( arg.isOption("iterations") ) {
		arg.getOption("iterations", iterations);
	}
	
	
	for (int i = 0; i < dim; i++) {
		up.push_back(10.0);
		low.push_back(0.0);
		sp.push_back(1.0);
	}
	boost::function1<double, RealVector&> evalu(&evaluator);
		
	ParticleSwarm p (population, up, low, sp, evalu ,false, r0, phi0 );
	p.setParticle(0, up * 0.5);
	p.explore(iterations);
  
	std::cout << "Last population: " << p.populationToString() << endl;
	std::cout << "Result: " << p.toString()<< std::endl;
	
  return 0;
}


double evaluator(RealVector& v) {
  RealVector obj(v.size());
  for (unsigned int i = 0; i < v.size(); i++) {
    obj[i] = 5.0;
  }
	
  return v.distance(obj) + 2*sin(v.norm() * 10);
}
