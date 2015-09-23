// This executable gives the distance point by point between two trajectories.
// Optionally, it can cut the loaded trajectories in order 

#include "ArgumentData.h"
#include "UAVTrajectory.h"
#include "functions.h"
#include <iostream>

using namespace std;
using namespace functions;

int main(int argc, char **argv) {
  ArgumentData arg(argc, argv);
  
  if (argc < 4) {
    cout << "Use: " << arg.at(0) << " <traj file 1> <traj file2> [options]";
    return -1;
  }
  
  UAVTrajectory traj1(arg[1]);
  UAVTrajectory traj2(arg[1]);
  
  UAVTrajectory& ref1 = traj1;
  UAVTrajectory& ref2 = traj2;
  
  if (arg.isOption("cut1")) {
    cout << "Cutting first trajectory.\n";
    // We will get the information
    vector<string> v;
    arg.getOption("cut1", v);
    int begin, end;
    
    if (v.size() < 2) {
      cerr << "Error: cut1 option has to contain at least 2 parameters\n";
      return -1;
    }
    
    // Get the parameters into int values
    istringstream is(v[0]);
    is >> begin;
    istringstream is2(v[1]);
    is2 >> end;
    UAVTrajectory cut1 = ref1.cut(begin, end);
    ref1 = cut1;
  }
  if (arg.isOption("cut2")) {
    cout << "Cutting second trajectory.\n";
    // We will get the information
    vector<string> v;
    arg.getOption("cut2", v);
    int begin, end;
    
    if (v.size() < 2) {
      cerr << "Error: cut2 option has to contain at least 2 parameters\n";
      return -1;
    }
    
    // Get the parameters into int values
    istringstream is(v[0]);
    is >> begin;
    istringstream is2(v[1]);
    is2 >> end;
    UAVTrajectory cut2 = ref2.cut(begin, end);
    ref2 = cut2;
  }
  
  vector<double> d;
  ref1.distance(ref2, d);
  
  cout << "Distance vector:\n";
  cout << printVector(d);
  
  if (arg.isOption("export")) {
    string filename;
    if (!arg.getOption<string>("export", filename)) {
      cerr << "Export option has to contain at least one parameter.\n";
      return -2;
    }
    if (!vectorToMatlabFile(filename, d)) {
      cerr << "Errors found while writing the distance to a file.\n";
      return -3;
    }
    
  }
  
  return 0;
}
