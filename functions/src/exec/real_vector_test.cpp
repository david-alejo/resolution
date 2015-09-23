
#include <ArgumentData.h>
#include <iostream>
#include "RealVector.h"

using namespace functions;
using namespace std;

void show_usage(const ArgumentData &arg);

int main (int argc, char **argv) {

	ArgumentData args(argc, argv);
	
	if (args.size() > 2 ) {
		// Only one argument --> the exec filename. 
		show_usage(args);
	}
	
	
	cout << "Real vector test. Note that the format of a vector is (n1, n2, ..., nm).\n";
	cout << "Input the vector 1. ";
	char line[200];
	cin.getline(line, 200);
	string s(line);
	RealVector v1(s);
	cout <<  "Now the second vector (with same size). ";
	cin.getline(line, 200);
	string s2(line);
	RealVector v2(s2);
	cout << "v1 = " << v1.toString() << "\tv2 = " << v2.toString() << endl;
	cout << "v1 + v2 = " << (v1 + v2).toString() << "\tv1 - v2 = " << (v1 - v2).toString() << endl;
	cout << "Checking distance to segment (v1, v2). enter a third vector (same dimension). ";
	cin.getline(line, 200);
	string s3(line);
	RealVector v3(s3);
	cout << "Distance to the segment: " << v3.distanceToSegment(v1, v2) << endl;
	
	cout <<  "\t norm(v1) = " << v3	.norm() << " v1 " << v1.toString() << "\t norm(v2) = " << v2.norm() << endl;
	cout << "v1" << v1.toString() << "\t v1.norm() = " << v1.norm()<< endl;
	
	cout << "Angles between vector: v1 to v2 -->" << v1.angle(v2) << endl;
	cout << "Angles between vector: v2 to v1 -->" << v2.angle(v1) << endl;
	cout << "Angles between vector: v1 to v3 -->" << v1.angle(v3) << endl;
	cout << "Angles between vector: v3 to v2 -->" << v3.angle(v2) << endl;

	if (v1.size() == v3.size() && v1.size() == 3) {
	  cout << "Cross product test: v3xv1 = " << v3.crossProduct(v1).toString() << endl;
	}
	if (v1.size() == v3.size()) {
	  cout << "Component product test: v3xv1 = " << v3.componentProduct(v1).toString() << endl;
	  }

	cout << "Random Vector Generation test. Enter a size: ";
	int tam;
	cin >> tam;
	RealVector v4(tam, true);
	cout << "Generated vector: " << v4.toString() << endl;

	
	return 0;
}


void show_usage(const ArgumentData &arg)
{
	cout << "Usage: " << arg[0] << " [<input_file>]" << endl;
}
