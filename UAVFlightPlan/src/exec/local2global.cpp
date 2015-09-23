// Translates a UAV Navigation flight plan into a KML file

#include "UAVTrajectory.h"
#include <functions/ArgumentData.h>
#include <functions/functions.h>
#include <iostream>

using UAVFlightPlan::UAVTrajectory;
using functions::ArgumentData;
using std::cout;
using std::endl;

int main(int argc, char **argv) {
  ArgumentData arg(argc, argv);
  
  if (argc < 2) {
    cout << "Use: " << arg[0] << " <input_file> <output_file>\n";
    return -1;
  }
  
  UAVTrajectory traj(arg.at(1), UAVFlightPlan::UAVTrajectory::MATLAB_LOCAL);
  
  int begin = 0;
  int end = traj.size();
  double shift_north = 0.0;
  double shift_east = 0.0;
  double shift_altitude = 0.0;
  
  if ( arg.isOption("begin") ) {
    arg.getOption<int>("begin", begin);
  }
  
  if ( arg.isOption("end") ) {
    arg.getOption<int>("end", end);
  }
  if (arg.isOption("shift_north") ) {
    arg.getOption("shift_north", shift_north);
    
  }
  if (arg.isOption("shift_east") ) {
    arg.getOption("shift_east", shift_east);
    
  }
  if (arg.isOption("shift_altitude") ) {
    arg.getOption("shift_altitude", shift_altitude);
    
  }
  
  traj.shift(shift_north, shift_east, shift_altitude);

  if (! functions::writeStringToFile(arg.at(2), traj.toMatlab(NULL, begin, end)) ) {
    std::cerr << "Errors found while writing the global MATLAB file.\n";
    return -2;
  }
  
  return 0;
}
  
