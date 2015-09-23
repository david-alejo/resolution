// This file creates an obstacle from relative coordinates given the experiment center.

#include "functions/ArgumentData.h"
#include "UAVObstacle.h"
#include "functions/functions.h"
#include <iostream>

using namespace functions;
using namespace UAVFlightPlan;
using namespace std;

int main(int argc, char **argv) {
	ArgumentData arg(argc, argv);
	
	if (argc < 4) {
		cout << "Use: " << argv[0] << " <obstacle file> <center file> <Out kml_file>\n";
		return -1;
	}
	
	// Create the location
	EarthLocation center(arg[2]);
	// Load the obstacle data
	vector<double> v = getVectorFromFile(arg[1]);
	double radius, north_shift, east_shift, altitude_shift;
	
	if (v.size() < 3) {
		cout << "Error while opening the obstacle file. Not enough elements.\n";
		return -2;
	}
	
	
	
	if (v.size() == 3) {
		radius = v.at(2);
	} else {
		radius = v.at(3);
	}
	
	if (arg.isOption("rotate")) {
	  double d;
	  if (arg.getOption("rotate", d)) {
	    DegMinSec deg(d);
	    rotateVector(v, deg);
	    cout << "Rotated vector: " << printVector(v) << endl;
	  } else {
	    cout << "Warning: no angle specified in the rotate option.\n";
	  }
	
	}
	
	
	north_shift = v.at(1);
	east_shift = v.at(0);
	
	// Create the obstacle
	UAVObstacle obs(center.getLatitude(), center.getLongitude() ,center.getAltitude(), radius);
	// Shift the obstacle with the position data
	obs.shift(north_shift, east_shift, altitude_shift);
	
	// Aditional shift
	if (arg.isOption("shift2")) {
	  double d;
	  if (arg.getOption("shift2", d)) {
	    obs.shift(0.0, d, 0);
	    
	  } else {
	    cout << "Warning: no distance specified in shift2 option.\n";
	  }
	
	}
	
	// Export the result to the desired KML file.
#ifdef USE_KML
	if (!obs.exportKMLFile(arg[3])) {
		cout << "Could not export data.\n";
		return -3;
	}
#else
	cerr << "Could not export to KML file, edit config.h and rebuild if compiling in a Linux envorinment.\n";
#endif
	
	return 0;
}
