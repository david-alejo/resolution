#include <stdio.h>
#include <sstream>
#include <iostream>
#include <sparser/all.h>
#include <string>
#include <vector>
#include "Simulator.h"

#include <functions/ArgumentData.h>

#include <iostream>                        // for std::cout

using namespace std;
using namespace simulator;
using namespace functions;

void print_message(const char *c);

int main(int argc, char **argv) {
  if (argc < 2) {
	  print_message(argv[0]);
	  return -1;
  }
  
  string filename(argv[1]);
  
//   try {
    cout << "Loading \"" << filename << "\" file...\n";
    Simulator s(filename);
    
    bool collision;
    ArgumentData arg(argc, argv);
    
    if (arg.isOption("montecarlo")) {
      int n_particles;
      arg.getOption("montecarlo",n_particles);
      bool collision;
      functions::RealVector v;
      if (s.montecarlo(collision, n_particles, v)) {
	cout << "The montecarlo simulation has been done successfully.\n";
	if (collision) {
	  cout << "New collision has been found.\n";
	} else {
	  cout << "No collisions have been found.\n";
	}
      }
    } else if (! arg.isOption("no-run")) {
      s.run(collision);
    
      s.exportTrajectory("traj.m");
    
      cout << "Simulations done. Congratulations. Trajectory exported to traj.m !!\n";
      if (collision) {
	    cout << "Colision found.\n";
      }
    
      int uavs = s.howManyUAVs();
    
    
      if (arg.isOption("rerun")) {
	s.run(collision);
	cout << "Simulation rerunned.\n";
	if (collision) {
	  cout << "Collision found in the new run.\n";
	} else {
	  cout << "No collisions found in the new run.\n";
	}
      }
    
      if ( arg.isOption("new_simulation") ){
	cout << "Enter new flight plans, please. N_uavs = " << uavs<< endl;
    
	vector<FlightPlan> plans;
	for (int i = 0; i < uavs; i++) {
	  FlightPlan plan;
	  for (int j = 0; j < 2; j++) {
	    cout << "Enter the point " << j+1 << " of uav " << i + 1 <<endl;
    
	    double x;
	    cin >> x;
	    double y;
	    cin >> y;
	    double z;
	    cin >> z;
	    Point3D p(x, y , z);
	    plan.push_back(p);
	  }
	  plans.push_back(plan);
	}
	  
	if (s.run(collision, plans)) {
	  cout << "The new simulation has been done successfully.\n";
	  if (collision) {
	    cout << "New collision has been found.\n";
	  } else {
	    cout << "No collisions have been found.\n";
	  }
	}
      }
      if (arg.isOption("write_test")) {
	string filename = "write_test";
	
	cout << "Executing write test.\n";
	
	if (arg.getOption<string>("write_test", filename)) {
	  cout << "Changing the filename to " << filename << endl;
	}
	
	
	ofstream out;
	
	try {
	  out.open(filename.c_str());
	  // Print the model content
	  ParseBlock *b = s.toBlock();
	  out << *b;
	  delete b;
	  
	  out.close();
	} catch (std::exception &e) {
	  cerr << "Catched a exception while writing the info to file.\n";
	  return -1;
	}
      }
    }
		
//   } catch (std::exception &e) {
// 	  cout << "Error while performing the simulations.   Exception content: " << e.what() << endl;
//   }
  
  return 0;
}

void print_message(const char *c)
{
	cout << "Usage: " << c << " <path of the input file> [--new_simulation] [--montecarlo]" << endl;
	cout << "This test performs a simulation with the data contained in the input file.\n";
	cout << "Options: \n";
	cout << "\t new_simulation Performs another simulation after the first one. The program will request the new plans to the user.\n";
	cout << "\t montecarlo <n_particles> Perform a montecarlo simulation of the system. \n";
}
