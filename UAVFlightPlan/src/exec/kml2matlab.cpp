#include <UAVTrajectory.h>
#include <functions/ArgumentData.h>
#ifdef USE_KML
#include <kml/base/file.h>
#endif

using namespace UAVFlightPlan;
using namespace std;
using functions::ArgumentData;


void showMessage(std::string s) {
  cout << "Use: " << s << " <input kml file> <output matlab file>" << endl;
}

int main(int argc, char **argv) {
  ArgumentData arg(argc,argv);

#ifndef USE_KML
  cerr << "Could not load a KML file, edit config.h and rebuild if compiling in a Linux envorinment.\n";
  return 1;
#else
  
  if (arg.isOption("help")) {
    showMessage(arg.at(0));
    cout << "Options: " << endl;
    cout << "--center <latitude> <longitude>" << endl;
    cout << "--relative" << endl;
    cout << "--begin <init_sample>" << endl;
    cout << "--end <last_sample>" << endl;
    return 0;
  }
  
  if (argc < 3) {
    showMessage(arg.at(0));
    
    return -1; 
  }
  
  EarthLocation *center = NULL;
  UAVTrajectory *traj = NULL;
  if (arg.isOption("center")) {
    vector<string> ops;
    center = new EarthLocation;
    arg.getOption("center", ops);
    
    if (ops.size() > 1) {
      
      double lati, longi;
      istringstream is(ops.at(0));
      is >> lati;
      istringstream is2(ops.at(1));
      is2 >> longi;
      
      center->setLatitude(lati);
      center->setLongitude(longi);
      cout << "Center: " << center->toString() << endl;
    }
  } 

  traj = new UAVTrajectory(arg.at(1), UAVTrajectory::KML);
  
  int begin = 0;
  int end = traj->size();
  
  if ( arg.isOption("begin") ) {
    arg.getOption<int>("begin", begin);
  }
  
  if ( arg.isOption("end") ) {
    arg.getOption<int>("end", end);
  }
  
  

  
  if (arg.isOption("relative") && traj->size() > 0) {
    delete center;
    center = new EarthLocation(traj->at(0)); 
  }
  
  if (traj != NULL && kmlbase::File::WriteStringToFile(traj->toMatlab(center,begin, end), arg.at(2))) {
    cout << "File created successfully.\n";
  } else {
    cerr << "Errors while writing the file.\n";
  }
  
  delete traj, center;
  
  return 0;
#endif
}
