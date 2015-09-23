// This executable loads a flight plan from file and 
// prints its content to stdout.
// Opcionally it can export its content to a kml file

#include "UAVFlightPlan.h"
#include "functions/ArgumentData.h"
#include <iostream>
#include <string>

using namespace functions;
using namespace std;

int main(int argc, char **argv) {
  ArgumentData arg(argc,argv);
  
  if (argc < 2) {
    cout << "Use: " << arg[0] << " <fp file> [options]" << endl;
    return -1; 
  }
  
  if (arg.isOption("help")) {
    cout << "Use: " << arg[0] << " <fp file> [options]" << endl << endl;
    cout << "Options: " << endl;
    cout << "--kml <filename>" << endl;
    return 0;
  }
  
  UAVFlightPlan::PLAN_TYPE type = UAVFlightPlan::QGC;
  
  if (arg.isOption("xml"))  {
    type = UAVFlightPlan::XML;
  }
  
  UAVFlightPlan::UAVFlightPlan fp(arg[1], type);
  
  cout << fp.toString() << endl;
  
  if (arg.isOption("kml")) {
    int begin = 0;
    int end = -1;
    
    if ( arg.isOption("begin") ) {
      arg.getOption<int>("begin", begin);
    }
  
    if ( arg.isOption("end") ) {
      arg.getOption<int>("end", end);
    }
    
    string file;
#ifdef USE_KML
    cout << " Exporting to KML file: " << file << endl;
    
    arg.getOption("kml", file);
    fp.exportToKMLFile(file, begin, end);
#else
    cerr << "Could not export to KML file, edit config.h and rebuild if compiling in a Linux envorinment.\n";
#endif
  }
  
  return 0;
}
