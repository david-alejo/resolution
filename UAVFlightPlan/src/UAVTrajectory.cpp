#include "UAVTrajectory.h"
#include "earthlocation.h"
#include "functions/DegMinSec.h"
#include "functions/functions.h"
#include <vector>
#include <iostream>
#include "config.h"
#include <sparser/all.h>

#ifdef USE_KML
#include "kml/base/file.h"
#include "kml/base/math_util.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/convenience/convenience.h"
#include "kml/base/string_util.h"

using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::LineStringPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
#endif

using namespace std;
using functions::DegMinSec;

namespace UAVFlightPlan {

UAVTrajectory::UAVTrajectory(const std::string& filename, UAVTrajectory::FileType t): std::vector<EarthLocation>()
{
	init();
	
	switch (t) {
	  case UAV_NAV: 
	    #ifdef USE_XML
	  loadUAVNavigationFile(filename);
#else
	    cerr << "UAVFlightPlan library not configured to load UAV NAV files.\n";
#endif

	    break;
	  case KML:
#ifdef USE_KML
	  loadKML(filename);
#else
	  cerr << "UAVFlightPlan library not configured to load KML files.\n";
#endif
	  break;
	  case MATLAB_LOCAL:
	    
	    fromLocalTrajectory(filename);
	    break;
	  case MATLAB_GLOBAL:
	  default:
	    fromGlobalTrajectory(filename);
	    
	}
}

UAVTrajectory::UAVTrajectory(): vector<EarthLocation>() {
  init();
}
	
#ifdef USE_XML
bool UAVTrajectory::loadUAVNavigationFile(const std::string& filename)
{
	clear();
	vector<vector<double> > vec;
	bool ret = true;
	int matrix_cols = 4;

	ifstream filestr;
	char buff[32768];
	
	try {
		filestr.open( filename.c_str() );
		filestr.exceptions(ifstream::failbit | ifstream::badbit);
		filestr.getline(buff, 32767); // Discard the first line
		filestr.close();
		
	} catch (exception &e) {
	  ret = false;
	  cerr << "UAVTrajectory::loadUAVNavigationFile--> Errors found while reading the trajectory file.\n";
	}
	
	try {
	  string st(buff);
	  istringstream iss(st);
	  string aux;
	  iss >> aux;
	  if ( aux == "MTIME" ) {
	    matrix_cols = 4;
	    cout << "Loading trajectory with mission file.\n";
	  } else if (aux == "LAT") {
	    matrix_cols = 3;
	    cout << "Loading trajectory without mission file.\n";
	  } else {
	    cerr << "Data could not be recognized\n";
	    ret = false;
	  }
	} catch (exception &e) {
	  cerr << "UAVTrajectory::loadUAVNavigationFile --> Error while reading the trajectory.\n";
	}
	
	if (!functions::getMatrixFromFile(filename, matrix_cols, vec) && ret) {
		ret = false;
		cerr << "UAVTrajectory::loadUAVNavigationFile--> Errors found while reading the trajectory file.\n";
	} else {
	  for (unsigned int i = 0; i < vec.size(); i++) {
	    if (matrix_cols == 3) {
	      DegMinSec lat(vec[i][0], functions::DegMinSec::RADIANS);
	      DegMinSec lon(vec[i][1], functions::DegMinSec::RADIANS);
	      EarthLocation aux(lat, lon, vec[i][2]);
	      aux.setStringType(EarthLocation::EL_RAD);
	      push_back(aux);
	    } else {
	      DegMinSec lat(vec[i][1], functions::DegMinSec::RADIANS);
	      DegMinSec lon(vec[i][2], functions::DegMinSec::RADIANS);
	      EarthLocation aux(lat, lon, vec[i][3], vec[i][0]);
	      aux.setStringType(EarthLocation::EL_RAD);
	      push_back(aux);
	    }
	  }
	}
	
	return ret;
}
#endif

#ifdef USE_KML
bool UAVTrajectory::loadKML(const string& filename)
{
  string data, errors;
  
  clear();
  vector<vector<double> > vec;
  bool ret = false;
  int matrix_cols = 4;

  ifstream filestr;
  char buff[32768];
  
  if (kmlbase::File::ReadFileToString(filename, &data)) {
    kmlengine::KmlFilePtr kml_file = KmlFile::CreateFromParse(data, &errors);
    ret = errors.empty();
    cout << "Errors: " << errors << endl;
    
    if (ret) {
      bool error = false;
      kmldom::ElementPtr root = kml_file->get_root();
      kmlengine::ElementVector placemarks;
      kmlengine::GetElementsById(root, kmldom::Type_Placemark, &placemarks);
      kmldom::LineStringPtr line = kmldom::AsLineString(placemarks.at(0));
//       if ( line != NULL) {
// 	for (unsigned int i = 0; i < line->get_coordinates(); i++)
//       } else {
      
        for (int i=0; i < placemarks.size(); i++) {
// 	cout << "UAVTrajectory::loadKML --> parsing element " << i << endl;
	
	  EarthLocation e(placemarks.at(i));
	  push_back(e);
//         }
      }
    }
  } else {
    cerr << "Could not load the desired file.\n";
  }
  
  return ret;
}


bool UAVTrajectory::exportKMLFile(const std::string &filename, int begin, int end, UAVTrajectory::ExportType type) const {
  bool ret = true;
  
  checkBounds(begin, end);
  
  // Create document. This is necessary to handle styles
  kmldom::DocumentPtr doc = factory->CreateDocument();
  switch (type) {
    case UAVTrajectory::LINE:
      doc->add_feature(getKMLPlaceMarck(filename, begin, end));  // kml takes ownership.
      break;
      
    case UAVTrajectory::POINT:
    default:
      vector<PlacemarkPtr> v = getKMLPlaceMarcks(filename, begin, end);
      for (unsigned int i = 0; i < v.size(); i++) {
	doc->add_feature(v.at(i));
      }
  }

  if (style) {
    // Generates the styles and retale them to the doc if the option is choosen
    addKMLStyle(doc);
  }
  
  // Create the kml pointer with the document
  KmlPtr kml = factory->CreateKml();
  kml->set_feature(doc);
  
  // Now the file and serialize it to string
  KmlFilePtr kmlfile = KmlFile::CreateFromImport(kml);
  if (!kmlfile) {
    cerr << "error: could not create kml file" << endl;
    return false;
  }
  std::string kml_data;
  kmlfile->SerializeToString(&kml_data);
  
  // Once we get the string --> write it to a file
  if (!kmlbase::File::WriteStringToFile(kml_data, filename.c_str())) {
    cerr << "error: write of " << filename << " failed" << endl;
    ret = false;
  }
  
  return ret;
}
#endif

string UAVTrajectory::toString() const {
  ostringstream os;
  
  os << "MTIME\tLAT LON ALT" << endl;
  
  for (unsigned int i = 0; i < size(); i++) {
    os << at(i).toString();
    os << endl;
  }
  
  return os.str();
}

string UAVTrajectory::toMatlab(const EarthLocation *center, unsigned int begin, int end) const
{
  ostringstream os;
  unsigned int end_ = end;
  if (end < 0) {
    end_ = size();
  }
  
  for (unsigned int i = begin; i < end_; i++) {
    if (center == NULL) {
      os << at(i).toMatlab();
    } else {
      os << at(i).toMatlab(*center);
    }
    os << endl;
  }
  
  return os.str();
}

#ifdef USE_KML
PlacemarkPtr UAVTrajectory::getKMLPlaceMarck(const std::string &place_name, int begin, int end) const {
  LineStringPtr linestring = getKMLLineString(begin, end);
  
  // Create the placemark containing the linestring
  PlacemarkPtr ret = factory->CreatePlacemark();
  ret->set_geometry(linestring);
  ret->set_name(place_name);
  ret->set_styleurl("#linestylemap");
    
  return ret;
}

vector<PlacemarkPtr> UAVTrajectory::getKMLPlaceMarcks(const std::string &place_name, int begin, int end) const {
  vector<PlacemarkPtr> ret;
  for(unsigned int i = begin; i < end; i++) {
    ostringstream os;
    os << place_name << i + 1;
    PlacemarkPtr aux = at(begin).getKMLPlacemark(os.str());
    ret.push_back(aux);
  }
  return ret;
}

LineStringPtr UAVTrajectory::getKMLLineString(int begin, int end) const {
  
  CoordinatesPtr coords = factory->CreateCoordinates();
  LineStringPtr linestring = factory->CreateLineString();

  checkBounds(begin, end);
  
  // First create the vector with the coordinates
  for (unsigned int i = begin; i < end; i++) {
    coords->add_latlngalt(at(i).getLatitude(), at(i).getLongitude(), at(i).getAltitude());
  }
  
  // Then assing them to the linestring
  linestring->set_coordinates(coords);
	
  return linestring;
}

void UAVTrajectory::addKMLStyle(kmldom::DocumentPtr &doc) const
{
	// Create the style 
	kmldom::StylePtr normal = factory->CreateStyle();
  normal->set_id("normal");
  kmldom::LineStylePtr linestyle = factory->CreateLineStyle();
  linestyle->set_width(5.0);
	kmlbase::Color32 red("red");
	linestyle->set_color(red);
  normal->set_linestyle(linestyle);
  doc->add_styleselector(normal);
	
	kmldom::StyleMapPtr stylemap = factory->CreateStyleMap();
  stylemap->set_id("linestylemap");
  kmldom::PairPtr pair = factory->CreatePair();
  pair->set_key(kmldom::STYLESTATE_NORMAL);
  pair->set_styleurl("#linestyle");
  stylemap->add_pair(pair);
	
	doc->add_styleselector(stylemap);
}
#endif

void UAVTrajectory::init() {
  style = false;
#ifdef USE_KML
  factory = KmlFactory::GetFactory();
#endif
}

UAVTrajectory UAVTrajectory::cut(int begin, int end) const {
  checkBounds(begin, end);
  UAVTrajectory ret;
  
  double init_time = at(begin).getTime();
  
  for (int i = begin; i < end; i++) {
    
    ret.push_back(at(i));
  }
  
  ret.shift(0.0, 0.0, 0.0, -init_time);
  
  return ret;
}

void UAVTrajectory::checkBounds(int &begin, int &end) const {
  if (end < 0 || end > (int)size()) {
    end = size(); 
  }
  
  if (begin < 0 || begin > (int)size()) {
    begin = 0;
  }
}

void UAVTrajectory::distance(const UAVTrajectory &traj, std::vector< double > &dist) {
  dist.clear(); // First clear the previous contents of dist vector
  
  for (int i = 0; i < traj.size() && i < size(); i++) {
    dist.push_back( at(i).distance(traj.at(i)) );
  }
}

void UAVTrajectory::shift(double north, double east, double alt, double time) {
  for (int i = 0; i < size(); i++) {
    at(i).shift(north,east,alt, time);
  }
}

bool UAVTrajectory::getStateFromTime(double time, EarthLocation &e) {
  bool found = false;
  
  if ( time < at(0).getTime() || size() == 0) {
    
    return false;
  }
  
  for (int i = 0; i < size() && !found ; i++) {
    if ( at(i).getTime() == time) {
      e = at(i);
      found = true;
    } else if (at(i).getTime() > time && i > 0) {
      // We have to interpolate
      EarthLocation &ant = at(i - 1);
      EarthLocation &pos = at(i);
      e = ant.interpolate(pos, time);
      found = true;
    }
   
  }
  
  return found;
}

bool UAVTrajectory::fromLocalTrajectory(const string& filename)
{
  bool ret_val = true;
  
  ParseBlock data;
  Checker *check = new Checker;
  
  check->addProperty("data_filename", new NTimes(1));
  check->addProperty("center", new NTimes(1));
  
  vector<vector<double> > matrix;
  EarthLocation center;
  
  try {
    data.load(filename.c_str());
    data.checkUsing(check);
    std::vector<double> aux = data("center").as<vector<double> >();
    center = EarthLocation(aux);
    ret_val = functions::getMatrixFromFile(data("data_filename").value, matrix);
    if (ret_val) {
      ret_val = matrix.size() > 0 && matrix.at(0).size() > 2;
    }
  } catch (std::exception &e) {
    cerr << "UAVTrajectory::fromLocalTrajectory --> Excepcion catched while getting the local trajectory from file. Content: " << e.what() << endl;
    ret_val = false;
  }
  
  if (ret_val) {
    for (unsigned int i = 0; i < matrix.size(); i++) {
      EarthLocation aux;
      aux.fromRelative(matrix.at(i), center, true);
      push_back(aux);
    }
  }
  
  return ret_val;
}

bool UAVTrajectory::fromGlobalTrajectory(const string& filename)
{
  bool ret_val = true;
  
  ParseBlock data;
  Checker *check = new Checker;
  
  check->addProperty("data_filename", new NTimes(1));
  check->addProperty("center", new NTimes(1));
  
  vector<vector<double> > mat;
  bool ret = functions::getMatrixFromFile(filename, mat);
  
  if (ret) {
    init(mat);
    ret = size() != 0;
  }
  
  return ret;
}

void UAVTrajectory::init(vector< std::vector< double > >& v)
{
  clear();
  for (unsigned int i = 0; i < v.size(); i++) {
    EarthLocation e(v.at(i));
    push_back(e);
  }
}



}
