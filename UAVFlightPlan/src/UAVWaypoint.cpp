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

#include "UAVWaypoint.h"
#include <map>
#include <exception>
#include <sstream>
#include <iostream>
#ifdef USE_XML
#include <libxml++/libxml++.h>
#endif
#include <functions/functions.h>

using namespace std;

namespace UAVFlightPlan {
  std::string UAVWaypoint::xml_id("dtFP"); // XML id of each waypoint
  std::string UAVWaypoint::alt_id("Alt"); // XML id of altitude above SL
  std::string UAVWaypoint::lat_id("Lat"); // XML id of latitude
  std::string UAVWaypoint::lon_id("Lon"); // XML id of longitude
  std::string UAVWaypoint::agl_id("AGL"); // XML id of altitude above GL
  std::string UAVWaypoint::dist_id("Distance"); // XML id of the distance to the next WO
  std::string UAVWaypoint::head_id("Heading"); // XML id of the heading to the next WO
  uint UAVWaypoint::MAVLINK_WAYPOINT = 16;
  unsigned char UAVWaypoint::REACH_ALTITUDE_MASK = 0X10;
  uint UAVWaypoint::MAV_DO_JUMP = 177;
  uint UAVWaypoint::MAV_CMD_CONDITION_CHANGE_ALT = 113;
  uint UAVWaypoint::MAV_UNLIMITED_LOITER = 17;
  
  UAVWaypoint::UAVWaypoint(double lat, double lon, double alt, double agl, double distance, double heading, double tolerance) :
  EarthLocation(lat, lon , alt)
  {
    init(lat, lon, alt, agl, distance, heading);
  }
#ifdef USE_XML
  UAVWaypoint::UAVWaypoint(const xmlpp::Node *node) : EarthLocation() {
    fromXMLNode(node);
    setStringType(EL_ALTITUDE);
    setRepresentTime(false);
  }
#endif
  
  void UAVWaypoint::init(double lat, double lon, double alt, double agl, double distance, double heading, double tolerance) {
    EarthLocation::init(lat, lon, alt);
    this->AGL = agl;
    this->distance = distance;
    this->heading = heading;
    this->tolerance = tolerance;
    setStringType(EL_ALTITUDE);
    setRepresentTime(false);
    active = false;
  }
#ifdef USE_XML
  bool UAVWaypoint::fromXMLNode(const xmlpp::Node *node) {
    bool ret_val = true;
    init();
    map<string, bool> init_map;
    
    init_map[lat_id] = false;
    init_map[lon_id] = false;
    init_map[alt_id] = false;
    init_map[agl_id] = false;
    init_map[dist_id] = false;
    init_map[head_id] = false;
    
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end() && ret_val; ++iter)
    {   
      string curr_name((*iter)->get_name());
      
      try {
	// See what element is and fill the proper term
	if (curr_name == lat_id) {
	  //Recurse through child nodes:
	  std::istringstream iss(getNodeContents(*iter));
	  iss >> lat;
	  init_map[lat_id] = true;
	} else if (curr_name == lon_id) {
	  std::istringstream iss(getNodeContents(*iter));;
	  iss >> lon;
	  init_map[lon_id] = true;
	} else if (curr_name == alt_id) {
	  std::istringstream iss(getNodeContents(*iter));
	  iss >> altitude;
	  init_map[alt_id] = true;
	} else if (curr_name == agl_id) {
	  std::istringstream iss(getNodeContents(*iter));
	  iss >> AGL;
	  init_map[agl_id] = true;
	} else if (curr_name == dist_id) {
	  std::istringstream iss(getNodeContents(*iter));
	  iss >> distance;
	  init_map[dist_id] = true;
	} else if (curr_name == head_id) {
	  std::istringstream iss(getNodeContents(*iter));
	  iss >> heading;
	  init_map[head_id] = true;
	}
      } catch (exception &e) {
// 	ret_val = false;
      }
      
    }
    
    ret_val &= init_map[lat_id] && init_map[lon_id] && init_map[alt_id] && init_map[agl_id] && init_map[dist_id] && init_map[head_id];
    
    return ret_val;
  }
#endif
  
  std::string UAVWaypoint::toString() const {
    std::ostringstream os;
    
    os << EarthLocation::toString() << ", ";
    os << AGL << ", " << distance << ", " << heading;
    
    return os.str();
  }
#ifdef USE_XML
  std::string UAVWaypoint::getNodeContents(const xmlpp::Node* node) {
    std::ostringstream os;

    xmlpp::Node::NodeList l = node->get_children();
    xmlpp::Node::NodeList::iterator it = l.begin();
    
    for ( ; it != l.end(); it++) {
      xmlpp::ContentNode *cont = dynamic_cast<xmlpp::ContentNode *>(*it);
      if (cont) {
	os << cont->get_content();
      }
    }
    
    return os.str();
  }
#endif
bool UAVWaypoint::fromQGroundControl(const string& data)
{
  bool ret = true;
  
  istringstream iss(data);
  std::vector<double> v = functions::getVectorFromStream(iss);
  
  try {
    lat = v.at(8);
    lon = v.at(9);
    altitude = v.at(10);
    yaw = v.at(7);
    tolerance = v.at(6);
    active = v.at(1) == '1';
    id = v.at(0);
    unsigned int bit_data = v.at(5);
    reach_altitude = (bit_data && REACH_ALTITUDE_MASK) != 0;
  } catch (exception &e) {
    ret = false;
  }
  
  return ret;
}

std::string UAVWaypoint::toQGroundControl(uint& nextwp) const
{
  ostringstream os;
  char spacer = '\t';
  
  os.precision(10);
  
  if (!reach_altitude) {
    
  
  os << nextwp << spacer;
  os << ((active)?1:0) << spacer;
  os << 0 << spacer;
  
  os << MAVLINK_WAYPOINT << spacer;
  } else {
    // First the condition
    
    os << nextwp << spacer;
    os << ((active)?1:0) << spacer;
    os << 0 << spacer;
    os << MAV_CMD_CONDITION_CHANGE_ALT << spacer;
    os << climb_rate << spacer;
    os << "0" << spacer << "0" << spacer;
    os << yaw << spacer; // copters
    os << lat << spacer;
    os << lon << spacer;
    os << altitude << spacer;
    os << 1 << endl; // Always autocontinue
    // Then the jump 
    nextwp++;
    os << jumpCommand(nextwp, nextwp + 2, 10) << endl;
    nextwp++;
    os << nextwp << spacer;
    os << ((active)?1:0) << spacer;
    os << 0 << spacer;
    os << MAV_UNLIMITED_LOITER << spacer;
  }
  os << "0" << spacer; // Time in waypoint (delay, only for copters)
  os << "0" << spacer; // Acceptance radius?
  os << "0" << spacer; // Orbit radius for loiter
  os << yaw << spacer; // copters
  os << lat << spacer;
  os << lon << spacer;
  os << altitude << spacer;
  os << 1; // Always autocontinue
  
  nextwp++;
  
  
  return os.str();
}

string UAVWaypoint::jumpCommand(uint command_id, uint target_id, uint n_times)
{
  ostringstream os;
  char spacer = '\t';
  
  os << command_id << spacer; // Number of the command
  os << 0 <<  spacer;
  os << 0 << spacer;
  os << MAV_DO_JUMP << spacer; // Command number in the MAVLINK protocol
  os << target_id << spacer; // Target ID?
  os << n_times << spacer; // n_times
  for (uint i = 0; i < 5; i++) 
    os << 0 << spacer;
  
  os << 1 ; // Continue executing
  
  return os.str();
}


  
}
