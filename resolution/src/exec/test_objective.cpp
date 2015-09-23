#include "../CRAlgorithmFactory.h"

#include <iostream>
#include <CRAlgorithm.h>
#include <functions/ArgumentData.h>

using namespace resolution;
using namespace std;

int main(int argc, char **argv) {
	
	CRAlgorithmFactory alg_fac;
	
	if (argc < 2) {
		cerr << "Use: " << argv[0] << " <data_filename> <vector to be evaluated>\n";
		return -1;
	}
	
	functions::ArgumentData arg(argc, argv);
	functions::RealVector v;
	for (unsigned int i = 2; i < arg.size(); i++) {
	  istringstream is(arg.at(i));
	  double aux;
	  is >> aux;
	  v.push_back(aux);
	}
	
	CRAlgorithm *algorithm = alg_fac.createFromFile(arg.at(1));
	if (algorithm != NULL) {
		double cost = algorithm->testObjective(v);
		cout << "Cost calculated. Obtained cost: " << cost << "\n";
	} else {
	  cerr << "Could not create the algorithm instance of problem.\n";
	}
	delete algorithm;
	
	return 0;
}