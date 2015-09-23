//! This utility will generate random tests to both GA and Particle Swarm

#include <stdio.h>
#include <string>
#include <vector>
#include <sys/time.h>
#include "CRAlgorithmFactory.h"
#include "CRAlgorithm.h"
#include <functions/ArgumentData.h>

#define MAX_CONT 100

using namespace std;
using namespace resolution;
using simulator::FlightPlan;

//! @brief Generates random flightplans for all the controlled uavs
//! @return The FlightPlans of all controlled uavs. They are not actual solution necessarily
bool generateRandomTest(const std::string &filename_in, const std::string &filename_out, int way_dim = 2);

//!Debugging method
bool tryToFiles(const std::vector< FlightPlan >& fps, const std::string &pr);
  
int main(int argc, char **argv) {
  int repetitions = 1;
  int problem_dimension = 2;
  
  if (argc < 3) {
    cerr << "Use: " << argv[0] << " <input_data_filename> <output_data_filename>\n";
    cerr << "Options: \n";
    cerr << "\t --repetitions \t Indicates the number of times that the scenario is generated. Defaults to 1.\n";
    cerr << "\t --dimension \t Indicates the dimension of the problem (2 or 3). Defaults to 2.\n";
    return -1;
  }
  
  functions::ArgumentData arg(argc, argv);
  
  int offset = 0;
  
  if (arg.isOption("repetitions") ) {
    
    arg.getOption("repetitions", repetitions);
    cout << "Number of repetitions --> " << repetitions << endl;
  }
  if (arg.isOption("dimension") ) {
    arg.getOption("dimension", problem_dimension);
    cout << "Problem dimension --> " << problem_dimension << endl;
  }
  if (arg.isOption("offset") ) {
    arg.getOption("offset", offset);
    cout << "Offset --> " << offset << endl;
  }
  
  for (int i = offset; i < repetitions; i++) {
    ostringstream os;
    os << arg.at(2);
    if (repetitions > 1) {
      os << i + 1;
    }
    int cont = 0;
    
    // Generate the problem
    while (!generateRandomTest(arg.at(1), os.str(), problem_dimension) && cont < MAX_CONT) {
      cont++;
    }
    
    if (cont <  MAX_CONT) {
      cout << "Random Test " << i + 1 << " created successfully.\n";
    }
  }
  
  return 0;
}


bool generateRandomTest(const std::string& filename_in, const std::string& filename_out, int way_dim){
  bool ret = false;
  bool error = false;
  bool found = false;
  bool timeout = false;
  bool collision = true;
  long time_elapsed =0.0;  
  
  struct timeval t1,t2;
  gettimeofday(&t1,NULL);
  
  vector<vector<double> > iicc;
  CRAlgorithmFactory alg_fac;

  CRAlgorithm *alg = NULL;
  try {
    alg = alg_fac.createFromFile(filename_in);
  } catch (exception &e) {
    cerr << "Exception thrown in problem.Content: "<< e.what() << endl;
    error = true;
  }
    
  if (alg != NULL && !error) {
  
    found = false;
    timeout = false;
    while (!(found || timeout)) { //While a solution is not found ( or too much time )
      if (alg->generateRandomProblem(way_dim)) { // Random problem successfully generated
	alg->saveProblem(filename_out);
	found = true;
	ret = true;
      } else {
	found = false;
      }
      gettimeofday(&t2, NULL);
      time_elapsed = (t2.tv_sec -t1.tv_sec)*1000.0 + (t2.tv_usec -t1.tv_usec)/1000.0; //in miliseconds
      if(time_elapsed > 1000000){
	timeout = true;
      }
    }
  }

  delete alg;
  return ret;
  
}

