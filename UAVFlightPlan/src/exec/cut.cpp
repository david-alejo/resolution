// This executable cuts the trajectory of one log. The result trajectory has all samples between the initial and the
// final.

#include "functions/ArgumentData.h"
#include "../UAVTrajectory.h"
#include "functions/functions.h"
#include <iostream>
#include <vector>

using namespace std;
using namespace functions;
using UAVFlightPlan::UAVTrajectory;

int main(int argc, char **argv) {
  ArgumentData arg(argc, argv);
  
  if (argc < 5) {
    cout << "Use: " << arg.at(0) << " <traj file in> <traj file out> <init sample> <end sample> [<options>]";
    return -1;
  }
  
  UAVTrajectory traj1(arg[1]);
  
  int begin, end;
  
  if ( !arg.getParameter(3, begin) || !arg.getParameter(4, end) ) {
    cerr << "Error: no cut parameters\n";
    return -1;
  }
  
  UAVTrajectory cut = traj1.cut(begin, end);
  
  double shift_north = 0.0;
  double shift_east = 0.0;
  if (arg.isOption("shift_north") ) {
    arg.getOption("shift_north", shift_north);
    
  }
  if (arg.isOption("shift_east") ) {
    arg.getOption("shift_east", shift_east);
  }
  
  cut.shift(shift_north, shift_east);
  
  ofstream ofile(arg.at(2).c_str());
  
  ofile << cut.toString();
  
  ofile.close();
  
  return 0;
}
