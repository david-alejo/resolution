#include <stdio.h>
#include <sstream>
#include <fstream>
#include <sparser/all.h>
#include <string>
#include <vector>
#include "CRAlgorithmFactory.h"
#include "functions/functions.h"

using namespace std;
using namespace resolution;

void print_message(const char *c, string filename);

struct test {
	string test_file;
	string output_file;
	bool change_name;
	int repeat;
};

int main(int argc, char **argv) {
	ParseBlock data;
	Checker *checker = new Checker;
	checker->addProperty("output_file", new NTimes(1) ); 
	checker->addProperty("test_file", new NTimes(1) );
	vector<test> test_info;
	string filename = "multitest_data";
	
	if (argc > 2) {
		
		delete checker;
		print_message(argv[0], filename);
		return -1;
	}
	
	if (argc == 2) {
		string aux(argv[1]);
		filename = aux;
	}
	
	try {
		cout << "Loading \"" << filename << "\" file...\n";
		data.load(filename.c_str());
		if (!data.hasBlock("test")) {
			cout << "No test blocks in the input file. Aborting.\n";
			print_message(argv[0], filename);
			delete checker;
			exit(1);
		}
		ParseBlock::Blocks *tests = data.getBlocks("test");
		ParseBlock::Blocks::iterator it;
		
		for ( it = tests->begin(); it != tests->end(); it++) {
			(*it)->checkUsing(checker);
			test aux;
			aux.output_file = (**it)("output_file").as<string>();
			aux.test_file = (**it)("test_file").as<string>();
			
			if ( (*it)->hasProperty("repeat") ) {
				aux.repeat = (**it)("repeat").as<int>();
			} else {
				aux.repeat = 1;
			}
			
			if ( (*it)->hasProperty("different_names")) {
			  aux.change_name = (**it)("different_names").as<bool>();
			} else {
			  aux.change_name = false;
			}
			
			test_info.push_back(aux);
		}
	} catch(std::runtime_error &e){
		cout << "Error while loading the input file. Error: " << e.what()<< endl;
		print_message(argv[0], filename);
		delete checker;
		exit(1);
	}
	
	CRAlgorithmFactory alg_fac;
	
	for (unsigned int i = 0; i < test_info.size(); i++) {
		cout << "\nTest number " << i + 1 << endl;
		StatisticVector stats;
		bool error = false;
		
		for (int k = 0; k < test_info[i].repeat && !error; k++) {
			cout << "Repetition number: " << k + 1;
			ostringstream name;
			name << test_info[i].test_file;
		
			if (test_info[i].change_name) {
			  name << k + 1;
			}
			cout << ". Test filename: " << name.str() << endl;
			
			CRAlgorithm *algorithm = alg_fac.createFromFile(name.str());
			if (algorithm != NULL) {
				stats.push_back(algorithm->execute());
				cout << "Algorithm " << i + 1 << ". Repetition " << k + 1 << " executed.\n";
			} else {
				error = true;
				cerr << "Errors while executing the test number " << i + 1 << ". Repetition " << k + 1 << endl;
			}
			delete algorithm;
		}
		
		// Represent the data obtained in the simulation
		if (!error) {
		  try {
		    functions::writeStringToFile(test_info[i].output_file, stats.toString());
		  }  catch (exception &e) {
		    cerr << "Error while exporting the results. Content: " << e.what() << " \n";
		  }
		}
	}
	
	cout << "Simulations done. Congratulations!!\n";
	
	delete checker;
	
	return 0;
}

void print_message(const char *c, string filename)
{
	cout << "Usage: " << c << " [<path of the input file>]" << endl;
	cout << "This file loads the tests indicated in \"" << filename << "\" file. " << endl;
	cout << "Each test has to have a location of the input data, the number of tests and the location";
	cout << "where the results will be moved. \n";
	cout << "If no input file is specified the default is \"multitest_data\".\n";
}
