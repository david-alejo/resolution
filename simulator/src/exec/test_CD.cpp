// This file will contain tests to CD and its children classes (only SiCoDe yet)

#include "CDFactory.h"
#include <functions/functions.h>
#include <functions/ArgumentData.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <cstring>

using namespace std;
using namespace simulator;

bool load_positions(const string &file, std::vector<std::vector<double> > & pos);
bool load_geometry(const string& file, std::vector< double >& geo);
void overlap_test();
void print_error_message(const char* order);
bool do_test(const string& pos_file, const string& geo_file, const string& cd_type);

using functions::ArgumentData;

int main(int argc, char **argv) {
	bool overlap = false;
	bool multitest = false;
	int n_test = 0;
	bool extended = false;
	string cd_type = "SiCoDe";
	
	ArgumentData arg(argc, argv);
	
	if (arg.isOption("multitest")) {
		arg.getOption("multitest", n_test);
		cout << "Doing multitest. Number of tests: " << n_test << endl;
		multitest = true;
		
	} else if (arg.isOption("overlap")) {
		overlap = true;
	} else if (arg.isOption("extended")) {
	  extended = true;
	} else if (arg.size() < 3) {
		print_error_message(argv[0]);
		return -1;
	}
	
	if (arg.isOption("type")) {
	  arg.getOption("type", cd_type);
	}
	
	// Overlap test
	if (overlap) {
		cout << "Doing Overlap test\n";
		overlap_test();
	} else if (multitest) {
		cout << "Doing SiCoDe multi test\n";
		
		for (int i = 1; i <= n_test && n_test > 0; i++) {
			ostringstream pos_file;
			pos_file << "Position" << i;
			ostringstream geo_file;
			geo_file << "Geometry" << i;
			
			cout << "Position file: " << pos_file.str() << "\t";
			cout << "Geometry file: " << geo_file.str() << "\t";
			
			do_test(pos_file.str().c_str(), geo_file.str().c_str(), cd_type);
		}
	} else {
		cout << "Doing SiCoDe single test\n";
		do_test(arg.at(1), arg.at(2), cd_type);
	}
	
	return 0;
}

bool load_positions(const string &file, vector< std::vector< double > >& pos) {
	bool ret_val = true;
	
	std::vector<double> vec;
	std::string filename(file);
	
	try { 
		vec = functions::getVectorFromFile(filename);
	} catch (...) {
		cerr << "load_positions() --> Error while loading file.\n";
		ret_val = false;
	}
	pos.clear();
	
	for ( int obj = 0; obj * 3 + 2 < vec.size() && ret_val; obj++ ) {
		vector<double> position;
		int i;
		
		for (i = 0; i < 3; i++) {
			position.push_back(vec[obj * 3 + i]);
		}
		
		pos.push_back(position);
	}
	
	return ret_val; 
}

bool load_geometry(const string &filename, std::vector< double >& geo)
{
	SiCoDe aux;
	
	return aux.loadGeometryFromFile(filename.c_str(), geo);
}


void overlap_test()
{
	float x1,x2,x3,x4;
	bool ex = false;
	
	while (!ex) {
		cout << "Enter the limits of two intervals [x1 x2] [x3 x4] (-1 -1 -1 -1 to exit)--> ";
		cin >> x1 >> x2 >> x3 >> x4;
		if (x1 == x2 && x3 == x4) {
			ex = true;
		} else {
			std::pair<double, double> i1, i2;
			i1.first = x1;
			i1.second = x2;
			i2.first = x3;
			i2.second = x4;
			if (SiCoDe::overlap(i1, i2)) {
				cout << "The two intervals overlap\n";
			} else {
				cout << "The two intervals do not overlap\n";
			}
		}
	}
}

void print_error_message(const char *order) {
	cerr << "Usage: " << order << " <position_filename> <geometry_filename>\n";
	cerr << " Or: " << order << " overlap\n";
	cerr << " Or: " << order << " --multitest <number_of_tests>. \n";
	cerr << "This test loads Positionx and Geometryx where x starts by 1 and goes to the desired number.\n";
}

bool do_test(const string &pos_file, const string &geo_file, const string &cd_type)
{
	bool ret_val = true;
	vector<vector <double> > position;
	vector<double> geo;
	
	if (!load_positions(pos_file, position)) {
		ret_val = false;
		cerr << "Error: while reading positions file\n";
	}
	if (!load_geometry(geo_file, geo)) {
		cout << "Error: while reading geometry file\n";
		ret_val = false;
	}
	
	if (ret_val) {
		cout << "Positions: ";
		for (unsigned int i = 0; i < position.size(); i++) {
			cout << i<< ": " << functions::printVector(position[i]) << "\t";
		}
		cout << endl;
		
		
		cout << "Geometry: ";
			cout << functions::printVector(geo) << endl;
			
		cout << "Type: " << cd_type << endl;
		
		CDFactory fac;
		CD *c = fac.create(cd_type);
	
		
		if (c != NULL && c->detectCollision(position, geo)) {
			cout << "Collision detected in the system\n";
		
		} else if (c != NULL) {
			cout << "No collisions detected in the system.\n"; 
		} else {
		  cout << "Could not create the collision detector\n";
		}
	}
	
	return ret_val;
}

