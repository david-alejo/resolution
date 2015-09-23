// This file loads a flight plan in relative coordinates and converts it to Earth coordinates

#include "earthlocation.h"
#include "UAVFlightPlan.h"
#include <functions/functions.h>
#include <iostream>
#ifdef USE_KML
#include "kml/engine.h"
#include "kml/base/file.h"
#endif
#include <math.h>
#include <stdio.h>
#include <functions/ArgumentData.h>

using namespace std;
using namespace functions;
using UAVFlightPlan::EarthLocation;
using UAVFlightPlan::UAVWaypoint;

void fancy_test();

void rotate_plan(vector<double> &plan, double rad, double x_c, double y_c);
void shift_plan(vector<double> &plan, double x_shift, double y_shift);
double get_min_y(const vector<double> &plan);

int main(int argc, char **argv) {
	if (argc == 1) {
		fancy_test();
	} else if (argc > 3) {
		ArgumentData arg(argc, argv);
		
		// Loads a bidimensional flight plan in meters and converts it to coordinates
		vector<double> flight_plan_meters = functions::getVectorFromFile(argv[1]);
		vector<double> c = functions::getVectorFromFile(argv[2]);
		
		bool reverse =false;
		
		if (arg.isOption("reverse")) {
		  reverse = true;
		  // In reverse mode, first coordinate is north shift and second is east shift
		}
		
		if (arg.isOption("rotate")) {
			double rad = 0.0;
			arg.getOption("rotate", rad);
			
			double x_c = 0.0;
			double y_c = 0.0;
			
			vector<string> params_rotate;
			arg.getOption("rotate", params_rotate);
			
			
			if (params_rotate.size() == 3) {
				x_c = atof(params_rotate.at(1).c_str());
				y_c = atof(params_rotate.at(2).c_str());
			}
			cout << "Rotating. Angle = " << rad << "\t Center coords: (" << x_c << ", " << y_c << ")\n";
		
			cout << "Original flight plan: " << printVector(flight_plan_meters) << endl;
			rotate_plan(flight_plan_meters, rad, x_c, y_c);
			
			double min_y = get_min_y(flight_plan_meters);
			
			shift_plan(flight_plan_meters, 0, -min_y - 300);
			
			cout << "Rotated flight plan: " << printVector(flight_plan_meters) << endl;

		}
		
		if (arg.isOption("shift")) {
			cout << "Shifting flight plan: " << endl;
			cout << "Original flight plan: " << functions::printVector(flight_plan_meters) << endl;
			
			vector<string> params_shift;
			arg.getOption("rotate", params_shift);
			
			shift_plan(flight_plan_meters, atof(params_shift.at(0).c_str()), atof(params_shift.at(1).c_str()) );
			cout << "Shifted flight plan: " << functions::printVector(flight_plan_meters) << endl;
		}
						
		
		EarthLocation center(c.at(0), c.at(1));
		UAVFlightPlan::UAVFlightPlan flight_plan;
		
		
		int vector_increment = 2;
		
		if (arg.isOption("vector_increment")) {
			arg.getOption("vector_increment", vector_increment);
		}
		
		for (int i = 0; i < flight_plan_meters.size() - 1; i += vector_increment) {
			UAVWaypoint curr_way(center.getLatitude(), center.getLongitude());
			if (!reverse) {
			  curr_way.shift(flight_plan_meters.at(i + 1), flight_plan_meters.at(i) ); // First coord --> East, Second --> North
			} else {
			  curr_way.shift(flight_plan_meters.at(i), flight_plan_meters.at(i + 1) ); // First coord --> North, Second --> East
			}
			curr_way.setStringType(EarthLocation::EL_DEGMIN);
			if (vector_increment = 3) {
			  curr_way.setAltitude(flight_plan_meters.at(i + 2));
			}
			flight_plan.push_back(curr_way);
		}
		
		cout << "Flight plan in latitude and longitude.\n";		cout << flight_plan.toString();
		cout << endl;

		// Write KML
#ifdef USE_KML
		try {
			if (!flight_plan.exportToKMLFile(arg.at(3))) {
				cerr << "error: could not create kml file" << endl;
			}
		}  catch (exception &e) {
			cerr << "error: could not create kml file" << endl;
		}
#else
	cerr << "Could not export to KML file, edit config.h and rebuild if compiling in a Linux envorinment.\n";
#endif

	} else {
		cout << "This file loads a flight plan in relative coordinates and converts it to Earth coordinates in kml format\n";
		cout << "Use: " << argv[0] << " [<flight_plan_file> <Center coordinates> <kml output file>] [Options]\n";
		cout << " Options: \n";
		cout << " --rotate <rotation_angle> [<x_c> <y_c>]"<< endl;
		cout << " --shift <shift_x(meters)> <shift_y(meters)>]"<< endl;
		cout << " --vector_increment <dimension of each input data>"<< endl;
		cout << " --reverse. If present, first coordinate indicates north shift. Otherwise, first coordinate is east shift.\n";
	}
	
	
	
	return 0;
}

void fancy_test()
{
	EarthLocation loc1(37.14677, -5.78727 );
	 
	cout << "Departure location: " << loc1.toString() << endl;
	
	EarthLocation loc2(loc1);
	loc2.shift(700, 0.0);
	
	cout.precision(15);
	
	cout << "Center location: " << loc2.toString() << endl;
	cout << "Center location (decimal degrees): " << loc2.getLatitude() << ", " ;
	cout << loc2.getLongitude()<< endl;
	
	cout << "Distance between two points: " << loc1.distance(loc2) << endl;
}

void rotate_plan(vector<double> &plan, double rad, double x_c, double y_c) {
	shift_plan(plan, -x_c, -y_c);
	
// 	cout << "Shifted flight plan: " << aux::printVector(plan) << endl;
	
	for (int i = 0; i < plan.size() - 1; i += 2) {
		double x_old = plan.at(i);
		double y_old = plan.at(i + 1);
		
// 		cout << "x_old = " << x_old << "\t y_old = " << y_old << "\t";
		
		plan[i] = x_old * cos(rad) - y_old * sin(rad);
		plan[i + 1] = x_old * sin(rad) + y_old * cos(rad);
		
// 		cout << "x_new = " << plan[i] << "\t y_new = " << plan[i+1] << endl;
	}
	
// 	cout << "Rotated 1 flight plan: " << aux::printVector(plan) << endl;
	
	shift_plan(plan, x_c, y_c);
}

void shift_plan(vector<double> &plan, double x_shift, double y_shift) {
	for (int i = 0; i < plan.size() - 1; i += 2) {
		plan[i] += x_shift;
		plan[i + 1] += y_shift;
	}
	
}

double get_min_y(const vector<double> &plan) {
	double min = 1e30;
	
	for (int i = 0; i < plan.size() - 1; i += 2) {
		min = functions::minimum(plan.at(i + 1), min);
	}
	
	return min;
}
