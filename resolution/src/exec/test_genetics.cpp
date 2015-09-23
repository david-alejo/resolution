#include "../CRAlgorithmFactory.h"

#include <iostream>
#include <CRAlgorithm.h>
#include <functions/ArgumentData.h>

using namespace resolution;
using namespace std;

int main(int argc, char **argv) {
  
  CRAlgorithmFactory alg_fac;
  
  if (argc < 2) {
	  cerr << "Use: " << argv[0] << " <data_filename> [<data_filename> ...]\n";
	  return -1;
  }
  
  functions::ArgumentData arg(argc, argv);
  
  for (unsigned int i = 1; i < arg.size(); i++) {
  
// 		try {
    CRAlgorithm *algorithm = alg_fac.createFromFile(arg.at(i));
    if (algorithm != NULL) {
      algorithm->execute();
      cout << "Algorithm executed.\n";
    } else {
	    cerr << "Could not create the algorithm instance of problem" << i + 1<<". Skipping it.\n";
    }
    delete algorithm;
// 		} catch (exception &e) {
// 			cerr << "Exception thrown in problem " << i  << ".Content: "<< e.what() << endl;
// 		}
  }
  
  return 0;
}