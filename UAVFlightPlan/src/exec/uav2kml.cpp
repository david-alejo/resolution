// Translates a UAV Navigation flight plan into a KML file

#include "UAVTrajectory.h"
#include <functions/ArgumentData.h>
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
  
  UAVTrajectory::FileType ft = UAVTrajectory::UAV_NAV;
  if (arg.isOption("relative")) {
    ft = UAVFlightPlan::UAVTrajectory::MATLAB_LOCAL;
  }
  
  if (arg.isOption("matlab")) {
    ft = UAVFlightPlan::UAVTrajectory::MATLAB_GLOBAL;
  }
  
  UAVTrajectory::ExportType et = UAVTrajectory::LINE;
  if (arg.isOption("point")) {
    et = UAVTrajectory::POINT;
  }
  
  UAVTrajectory traj(arg.at(1), ft);
  
  int begin = 0;
  int end = traj.size();
  double shift_north = 0.0;
  double shift_east = 0.0;
  
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
  
  traj.shift(shift_north, shift_east, 0.0);
#ifdef USE_KML
  if (!traj.exportKMLFile(arg.at(2), begin, end, et) ) {
    std::cerr << "Errors found while writing the KML file.\n";
    return -2;
  }
#else
  std::cerr << "Could not export to KML file, edit config.h and rebuild if compiling in a Linux envorinment.\n";
#endif
  
  return 0;
}
