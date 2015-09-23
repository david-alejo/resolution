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

#ifndef UAVWAYPOINT_H
#define UAVWAYPOINT_H

#include "config.h"
#include "earthlocation.h"
#ifdef USE_XML
#include <libxml++/libxml++.h>
#endif
#include <string>

namespace UAVFlightPlan {
class UAVWaypoint:public EarthLocation
{
  public:
  //! @brief Common constructor
  //! @param lat Latitude in decimal degrees
  //! @param lon Longitude in decimal degrees
  //! @param alt Altitude in meters above sea level
  //! @param agl Altitude in meters above ground level
  //! @param distance Distance to the next waypoint
  //! @param heading Heading to the next waypoint
  UAVWaypoint(double lat = 0.0, double lon = 0.0, double alt = 0.0, double agl = 0.0, double distance = 0.0, double heading = 0.0, double tolerance = 30.0);
  
  //! @brief Constructor from XML node
#ifdef USE_XML
  UAVWaypoint(const xmlpp::Node *node);
#endif
  
  //! @brief Represents the content of the class in deg min sec way
  //! @return The string that represents the contents
  virtual std::string toString() const;
  
  //! @brief Gets all WP data from one node and initialices the arguments with them
    //! @param node The XML node
    //! @retval true Data was retrieved successfully
    //! @retval false Errors while retrieving data
#ifdef USE_XML
    bool fromXMLNode(const xmlpp::Node* node);
#endif
  
  //! @brief Gets all WP data from a text line of a QGroundControl WPL 120 file
  //! @retval true Data was retrieved successfully
  //! @retval false Errors while retrieving data
  bool fromQGroundControl(const std::string &data);
  
  //! @brief Exports all WP data to a QGroundControl WPL 120 file
  //! @retval true Data was written successfully
  //! @retval false Errors while writing data
  //! @param nextwp Out parameter with the id of the next waypoint (some commands have more than one MAVLINK action)
  std::string toQGroundControl(uint &nextwp) const;
  
  //! @brief Accesor to active
  inline void setActive(bool new_active) { active = new_active; };
  
  //! @brief Accesor to ID
  inline void setID(int new_id) { id = new_id; };
  
  //! @brief accesor to Reach Altitude
  inline void setReachAltitude(bool value) { reach_altitude = value; }
  
  inline bool getReachAltitude() const {return reach_altitude;} 
  
  static std::string jumpCommand(uint command_id, uint target_id, uint n_times);
  
  inline void setClimbRate(double c) { climb_rate = c;}
  
  inline double getClimbRate() const { return climb_rate; }
   
  protected:
    // Basic added info to the location
    double heading; // Heading to the next waypoint in degrees
    double AGL; // Altitude from ground level
    double distance; // Distance to the following waypoint
    double tolerance; // Tolerance in meters
    bool active; // Indicates if it is the next waypoint to follow
    int id; // The ID of the waypoint
    // Additional behaviour initialized to a default value in the constructor, needs to be set separately
    bool reach_altitude;
    double climb_rate;
    
    static std::string xml_id; // Identifier of the XML element that represents a waypoint
    static std::string alt_id; // Identifier of the XML element that represents the altitude above SL
    static std::string lat_id; // Identifier of the XML element that represents the latitude
    static std::string lon_id; // Identifier of the XML element that represents the longitude
    static std::string agl_id; // Identifier of the XML element that represents the altiude above GL
    static std::string dist_id; // Identifier of the XML element that represents the distance to next WP
    static std::string head_id; // Identifier of the XML element that represents the heading
    
public:
    static uint MAVLINK_WAYPOINT;
    static uint MAV_UNLIMITED_LOITER;
    static uint MAV_DO_JUMP;
    static uint MAV_CMD_CONDITION_CHANGE_ALT;
    static unsigned char REACH_ALTITUDE_MASK;
    
protected:
    
    //! @brief Saves the information received
    //! @param lat Latitude in decimal degrees
    //! @param lon Longitude in decimal degrees
    //! @param alt Altitude in meters above sea level
    //! @param agl Altitude in meters above ground level
    //! @param distance Distance to the next waypoint
    //! @param heading Heading to the next waypoint
    void init(double lat = 0.0, double lon = 0.0, double alt = 0.0, double agl = 0.0, double distance = 0.0, double heading = 0.0, double tolerance = 30.0);
    
#ifdef USE_XML
    //! @brief Gets the contents from an identified XML node
    //! @param node The identified node
    //! @return A string with the whole content of the node
    std::string getNodeContents(const xmlpp::Node* node);
#endif
};

}

#endif // UAVWAYPOINT_H
