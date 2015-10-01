#include <ArgumentData.h>
#include <iostream>
#include "functions.h"
#include "Point3D.h"

using namespace functions;
using namespace std;

void show_usage(const ArgumentData &arg);

int main (int argc, char **argv) {

	ArgumentData args(argc, argv);
	
	if (args.size() == 1 ) {
		// Only one argument --> the exec filename. 
		show_usage(args);
	}
	
	cout << "Arguments: " << args.toString() << endl;
	
	if ( args.isOption("Point3D") ) {
		
		cout << "Making Point3D test.\n";
		Point3D p(0.72, -1.47, 0.7);
		Point3D p1(3, 1, 0.7);
		Point3D p2(-3, -1, 0.7);
		cout << "Heading " << p.toString() << " = " << p.getHeading() << endl;
		cout << "Heading from " << p1.toString() << " to ";
		cout << p2.toString() << " = " << p1.getHeadingTo(p2) << endl;
		cout << "Delta Heading from " << p1.toString() << " to ";
		cout << p2.toString() << " = " << p1.getDeltaHeading(p2) << endl;
	}
	
	if (args.isOption("HessianPlane")) {
		cout << "Making HessianPlane test.\n";
	}
	
	if (args.isOption("RandomNumber")) {
		cout << "Making RandomNumber test.\n";
	}
	
	if (args.isOption("integer-option")) {
		cout << "Integer option test.\n";
		int number;
		if ( args.getOption<int>("integer-option", number) ) {
			cout << "Number catched successfully. Result = " << number << endl;
		} else {
			cout << "Could not catch the number." << endl;
		}
	}
	
	if (args.isOption("double-option")) {
		cout << "Double option test.\n";
		double number;
		if ( args.getOption<double>("double-option", number) ) {
			cout << "Number catched successfully. Result = " << number << endl;
		} else {
			cout << "Could not catch the number." << endl;
		}
	}
	
	if (args.isOption("multiple-arguments")) {
		cout << "Multiple arguments option detected.";
		vector<string> arguments;
		args.getOption("multiple-arguments", arguments);
		cout << " Arguments size = " << arguments.size() <<  ". These arguments are (testing printVector): ";
		cout << printVector(arguments);
		cout << endl;
	}
	
	if (args.isOption("load_matrix")) {
	  cout << "Load matrix option detected.\n";
	  string file;
	  args.getOption("load_matrix", file);
	  std::vector<vector<double> > v;
	  if (getMatrixFromFile(file, v)) {
	    cout << "Matrix loaded successfully. N_rows = " << v.size() <<". ";
	    int cols = 0;
	    if (v.size() > 0) {
	      cols = v.at(0).size();
	    }
	    cout << "Ncols = " << cols << endl;
	  }
	}
	
	return 0;
}


void show_usage(const ArgumentData &arg)
{
	cout << "Usage: " << arg[0] << " [--Point3D] [--HessianPlane] [--RandomNumber] [--load_matrix]" << endl;
}
