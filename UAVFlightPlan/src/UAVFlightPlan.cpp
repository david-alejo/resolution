/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "UAVFlightPlan.h"
#ifdef USE_XML
#include <libxml++/libxml++.h>
#endif
#include <exception>
#include <iostream>
#include <functions/functions.h>

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

using namespace std;

namespace UAVFlightPlan {
  std::string UAVFlightPlan::wp_id = "dtFP"; // XML id of each waypoint
  std::string UAVFlightPlan::parent_id = "DocumentElement"; // XML id of the flight plan
  
  std::string UAVFlightPlan::qgc_id = "QGC WPL ";
  
  UAVFlightPlan::UAVFlightPlan() {
		init();
	}
  
  UAVFlightPlan::UAVFlightPlan(const string &s, PLAN_TYPE type, bool loop) :loop_(loop) {
#ifdef USE_XML
    if (type == XML) {
      loadXMLFile(s);
      
    }
#else
    if (type == QGC) {
      fromQGCFile(s);
    }
#endif
  }
#ifdef USE_XML
  bool UAVFlightPlan::loadXMLFile(const string &s):loop_(loop)  {
    clear();
    bool ret_val = true;
    
    try {
      xmlpp::DomParser parser;
      //parser.set_validate();
      parser.set_substitute_entities(false);
      parser.parse_file(s);
      if(parser)
      {
	//Walk the tree:
	const xmlpp::Node* pNode = parser.get_document()->get_root_node(); //deleted by DomParser.
	
	xmlpp::Node::NodeList n_list(pNode->get_children());
	
	xmlpp::Node::NodeList::iterator it;
	UAVWaypoint curr_wp;
	int cont = 0;
	
	for (it = n_list.begin(); it != n_list.end() && ret_val; it++, cont++) {
	  
	  
	  
	  if ( (*it)->get_name() == wp_id ) {
	    // First step: Document class
	    // Now we will search the waypoints (its name will be equal to wp_id
	    ret_val = curr_wp.fromXMLNode(*it);
	    if (ret_val) {
	      curr_wp.setID(cont);
	      curr_wp.setActive(cont == 0);
	      push_back(curr_wp);
	    } else {
	      cout << "UAVFlightPlan::loadXMLFile --> Error in one waypoint element.\n";
	    }
	  }
	}
	
	
      }
    } catch(const std::exception& ex) {
      ret_val = false;
      std::cout << "UAVFlightPlan::loadXMLFile --> Exception caught: " << ex.what() << std::endl;
    }
    
    return ret_val;
  }
#endif
  std::string UAVFlightPlan::toString() const {
    ostringstream os;
    
    os << "Flight plan content: " << endl;
    
    const_iterator it;
    for (it = begin(); it != end(); it++) {
      os << it->toString() << endl;
      
    }
    
    return os.str();
  }

#ifdef USE_KML
  bool UAVFlightPlan::exportToKMLFile(const std::string &f, int begin_, int end_) const {
    bool ret_val = true;
    
    if (end_ < 0 || end_ > size()) {
      end_ = size(); 
    }
  
    if (begin_ < 0 || begin_ > size()) {
      begin_ = 0;
    }
    
    const_iterator it = begin();
    
    // Advance to the first waypoint to save
    int i = 0;
    for (; i < begin_; i++, it++);

    KmlFactory* factory = KmlFactory::GetFactory();
    
    
		kmldom::DocumentPtr doc = factory->CreateDocument();
		
    for (int j = 1; i < end_ && it != end(); j++,i++, it++) {
			ostringstream name_;
			name_ << "Waypoint " << j;
			// Create a <Point> with <coordinates> from the given Vec3.
			kmlbase::Vec3 v(it->getLongitude(), it->getLatitude(), it->getAltitude());
			kmldom::PointPtr point = kmlconvenience::CreatePointFromVec3(v);
			PlacemarkPtr place = factory->CreatePlacemark();
			place->set_geometry(point);
			doc->add_feature(place);
    }
    
    
    
    // Finally create the kml
    KmlPtr kml = factory->CreateKml();
    kml->set_feature(doc);
    
    // Then the file
    KmlFilePtr kmlfile = KmlFile::CreateFromImport(kml);
    if (!kmlfile) {
      cerr << "error: could not create kml file" << endl;
      return false;
    }
    
    // And write it
    std::string kml_data;
    kmlfile->SerializeToString(&kml_data);
    if (!kmlbase::File::WriteStringToFile(kml_data, f.c_str())) {
      cerr << "error: write of " << f << " failed" << endl;
      ret_val = false;
    }
    
    return ret_val;
  }
#endif
  
  void UAVFlightPlan::init() {
		loop_ = false;
	}
	
	void UAVFlightPlan::getVectors(vector<double> &lat_, vector<double> &lon_) const
	{
		lat_.clear();
		lon_.clear();
		
		const_iterator it = begin();
		for ( ; it != end(); it++) {
			lat_.push_back(it->getLatitude());
			lon_.push_back(it->getLongitude());
		}
	}
	

  std::string UAVFlightPlan::toQGCString(unsigned int version) const {
    ostringstream os;
    uint cont = 0;
    
    os << qgc_id  << version << endl;
    UAVFlightPlan::const_iterator it = begin();
    
    for (;it != end(); it++) {
//        it->setID(cont);
       os << it->toQGroundControl(cont) << endl;
    }
    
    if (loop_) {
      os << UAVWaypoint::jumpCommand(cont, 0, 10) << endl;
      cont++;
      os << UAVWaypoint().toQGroundControl(cont) << endl;
    }
    return os.str();
  }
	
  bool UAVFlightPlan::toQGCFile(const string& filename, unsigned int version) const
  {
    return functions::writeStringToFile(filename, toQGCString(version));
  }
  
  UAVFlightPlan::UAVFlightPlan(const vector< functions::Point3D >& fp, const EarthLocation& e, bool loop, std::vector<bool> reach_altitude):loop_(loop)
  {
    
    for (unsigned int i = 0; i < fp.size(); i++) {
      EarthLocation p(e);
      p.shift(fp.at(i).x, fp.at(i).y, fp.at(i).z);
      
      UAVWaypoint w(p.getLatitude(), p.getLongitude(), p.getAltitude());
      w.setID(i);
      w.setActive(i == 1 || fp.size() == 1); // Make active allways the second waypoint or the first if it is the only one
      bool r = false;
      if (i < reach_altitude.size()) {
	r = reach_altitude.at(i);
      }
      w.setReachAltitude(r);
      
      push_back(w);
    }
  }
  
bool UAVFlightPlan::fromQGCFile(const string& filename)
{
  bool ret = true;
  clear();
  
  try {
    string s = functions::loadStringFile(filename);
    ret = fromQGCString(s);
  } catch (exception &e) {
    ret = false;
  }
  
  return ret;
}

bool UAVFlightPlan::fromQGCString(const string& data)
{
  istringstream iss(data);
  bool ret = true;
  char buffer[32768];
  int cont = 0;
  bool end = false;
  while (iss.getline(buffer, 32678) && !end && ret) {
    string s(buffer);
    if (cont == 0) {
      cout << s.substr(0, s.size() - 1) << "."<< endl; 
      if (s.substr(0, qgc_id.size()) != qgc_id) {
	  end = true;
	ret = false;
      }
    } else {
      UAVWaypoint w;
      end = !w.fromQGroundControl(s);
      if (!end) {
	push_back(w);
      }
    }
    cont++;
  }
  return ret;
}

vector< functions::Point3D > UAVFlightPlan::toRelativeFlightPlan(EarthLocation& center) const
{
  vector<functions::Point3D> ret;
  
  const_iterator it = begin();
  for (;it != end(); it++) {
    EarthLocation curr;
    curr.setAltitude(it->getAltitude());
    curr.setLatitude(it->getLatitude());
    curr.setLongitude(it->getLongitude());
    functions::Point3D x(curr.toRelative(center));
    ret.push_back(x);
  }
  
  return ret;
}

bool UAVFlightPlan::fromRelativeFlightPlan(const string& s, EarthLocation& center)
{
  clear();
  bool ret = false;
 
  vector<vector<double> > mat;
  
  ret = functions::getMatrixFromFile(s, mat);
  
  if (ret) {
    for (unsigned int i = 0; i < mat.size(); i++) {
      if (mat.at(i).size() > 1) {
	EarthLocation loc(center);
	loc.shift(mat.at(i).at(0), mat.at(i).at(1) );
	if (mat.at(i).size() > 2) {
	  loc.setAltitude(mat.at(i).at(2));
	}
	UAVWaypoint way(loc.getLatitude(), loc.getLongitude(), loc.getAltitude(), loc.getAltitude());
	if (i == 1) 
	  way.setActive(true);
	way.setID(i);
	push_back(way);
      }
      
    }
  }
  
  return ret;
}
  
} // namespace
