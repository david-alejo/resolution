#include "../CRAlgorithmFactory.h"

#include <iostream>
#include <CRAlgorithm.h>
#include <FlightPlanChecker.h>
#include <functions/ArgumentData.h>

using namespace resolution;
using namespace std;
using namespace simulator;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Use: " << argv[0] << " <data_filename> [<data_filename> ...]\n";
    return -1;
  }
  functions::ArgumentData arg(argc, argv);
  
  CRAlgorithmFactory alg_fac;

  for (unsigned int i = 1; i < arg.size(); i++) {

    CRAlgorithm *algorithm = alg_fac.createFromFile(arg.at(i));
    if (algorithm != NULL) {
      Simulator *sim = algorithm->getSimulator();
      if (FlightPlanChecker::checkFlightPlans(sim->getFlightPlans(), sim)) {
	cout << "The initial plans have been checked and no collision has been found.\n";
      } else {
	cout << "The initial plans have been checked and a collision has been found.\n";
      }
    } else {
      cerr << "Could not create the algorithm instance of problem" << i + 1 << ". Skipping it.\n";
    }
    delete algorithm;
  }
  return 0;
}