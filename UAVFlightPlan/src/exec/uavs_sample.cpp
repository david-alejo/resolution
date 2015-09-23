#include "../earthlocation.h"
#include "../UAVTrajectory.h"
#include "../general.h"
#include "functions/ArgumentData.h"
#include "functions/functions.h"
#include <iostream>
#include <vector>
#ifdef USE_KML
#include <kml/base/file.h>
#include <kml/dom.h>
#include <kml/engine.h>
#include <kml/convenience/convenience.h>

using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::PointPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
#endif

using namespace UAVFlightPlan;
using functions::ArgumentData;
using namespace std;


int main(int argc, char **argv) {
  ArgumentData arg(argc, argv);
  
  if (argc < 4) {
    cout << "Use: " << arg[0] << " <base_output_filename (without extension) > <sample_time> <input_file 1> <input_file_2> ...\n";
    return -1;
  }
  
  vector<UAVTrajectory> UAVs;
  double T;
  if ( !arg.getParameter<double>(2, T)) {
    cout << "Use: " << arg[0] << " <base_output_filename (without extension) > <sample_time> <input_file 1> <input_file_2> ...\n";
    return -1;
  } else {
    cout << "Sample time = " << T << endl; 
  }
  
  // Load 
  for ( int i = 3; i < argc; i++) {
    UAVs.push_back(UAVTrajectory(arg.at(i)));
  }
  
 #ifdef USE_KML
  KmlFactory* factory = KmlFactory::GetFactory();
  bool end = false;
  int n_file = 0;
  for (double t = 0.0; !end ; n_file++, t += T) {
    kmldom::DocumentPtr doc = factory->CreateDocument();
    
    for (int i = 0; i < UAVs.size(); i++) {
      ostringstream name;
      name << "UAV " << i + 1;
      EarthLocation e;
      
      if ( UAVs.at(i).getStateFromTime(t, e)) {
	doc->add_feature(e.getKMLPlacemark(name.str()));
      } else {
	end = true;
      }
    }
    
    // Write instant to file
    if (!end) {
      cout << "Writing file " << n_file + 1 << endl;
      cout << "Time = " << t << endl;
      ostringstream name;
      name << arg.at(1) << functions::numberToString(n_file, 2) << ".kml";
    
    
      // Finally create the kml
      KmlPtr kml = factory->CreateKml();
      kml->set_feature(doc);
    
      if (!UAVFlightPlan::writeKMLToFile(name.str(), kml)) {
	std::cerr << "Errors found while writing the KML file.\n";
	return -2;
      }
    }
    
    
  }
#else
  std::cerr << "Could not export to KML file, edit config.h and rebuild if compiling in a Linux envorinment.\n";
#endif
  
  return 0;
}
