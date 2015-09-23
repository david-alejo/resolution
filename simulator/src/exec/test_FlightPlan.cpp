#include "FlightPlan.h"
#include "functions/ArgumentData.h"
#include "functions/functions.h"
#include <boost/lexical_cast.hpp>
#include <iostream>

using namespace simulator;
using namespace functions;
using namespace std;
using boost::lexical_cast;

int main(int argc, char **argv) {
  ArgumentData arg(argc, argv);
  
  if (arg.size() < 2) {
    cerr << "Usage: " << arg[0] << "<input filename> [--export_reference <radius> <ascending_rate> <reference filename>]\n";
    return -1;
  }
  
  FlightPlan fp(arg[1]);
  
  cout << "Flight plan: \n" << fp.toString() << endl;
  
  if (arg.isOption("export_reference")) {
    vector<string> v_s;
    arg.getOption("export_reference", v_s);
    if (v_s.size() >= 3) {
      try {
	vector<Point3D> traj = fp.getReferenceTrajectory(lexical_cast<double>(v_s[0]),lexical_cast<double>(v_s[1]));
      
      
      Point3D p;
      ostringstream os;
      for (unsigned int i = 0; i < traj.size(); i++) {
	os << traj[i].toString(false) << endl;
      }
      if (!functions::writeStringToFile(v_s[2], os.str())) {
	cerr << "Error while writing the reference trajectory.\n";
      } else{
	cout << "Reference trajectory successfully written.\n";
      }
      } catch (exception &e) {
	cerr << "Could not interpret the command line associated to export_reference \n";
      }
    }
  }
  return 0;
}