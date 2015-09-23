// This executable loads a flight plan from file and 
// prints its content to stdout.
// Opcionally it can export its content to a kml file

#include "UAVFlightPlan.h"
#include "functions/ArgumentData.h"
#include "functions/Point3D.h"
#include <iostream>
#include <string>
#include <vector>

using namespace functions;
using namespace UAVFlightPlan;
using namespace std;

int main(int argc, char **argv) {
  ArgumentData arg(argc,argv);
  
  if (argc < 4) {
    cout << "Use: " << arg[0] << " <relative_flight_plan_file> <center_file> <qgc_file> [" << endl;
    return -1; 
  }
  
  UAVFlightPlan::UAVFlightPlan fp;
  UAVFlightPlan::EarthLocation center(arg.at(2));
  fp.fromRelativeFlightPlan(arg.at(1), center);
  
  if (arg.isOption("loop")) {
    fp.setLoop(true);
  }
  
  cout << fp.toString() << endl;
  
  fp.toQGCFile(arg[3]);
    
  return 0;
}
