#include <stdio.h>
#include <sstream>
#include <iostream>
#include <sparser/all.h>
#include <string>
#include <vector>
#include "ParticleFilter.h"

#include <functions/ArgumentData.h>

#include <iostream>                        // for std::cout

using namespace std;
using namespace simulator;
using namespace functions;

void print_message(const char *c);

int main(int argc, char **argv) {
	if (argc < 2) {
		print_message(argv[0]);
		return -1;
	}
	
	string filename(argv[1]);
	
	int max_steps = 1000;
	
	try {
		cout << "Loading \"" << filename << "\" file...\n";
		ParseBlock b;
		b.load(filename.c_str());
		ParticleFilter pf(b);
		
		bool collision;
		ArgumentData arg(argc, argv);
		
		int steps = 0;
		
		while (pf.runOneStep() == 0 && steps < max_steps) {
		  steps++;
		}
		cout << "Simulation done. Steps: " << steps << endl;
		cout << "Maximum deviation: " << pf.getMaxDev() << endl;
		
	}catch (std::exception &e) {
		cerr << "Error while performing the particle filter.   Exception content: " << e.what() << endl;
		
	}
	
	return 0;
}

void print_message(const char *c)
{
	cout << "Usage: " << c << " <path of the input file> " << endl;
	cout << "This test performs a MonteCarlo simulation with the data contained in the input file.\n";
}
