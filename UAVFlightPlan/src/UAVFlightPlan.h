/*
    <This class contains a UAV Navigation flight plan.>
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

#ifndef UAVFLIGHTPLAN_H
#define UAVFLIGHTPLAN_H

#include "config.h"
#include "UAVWaypoint.h"
#include "earthlocation.h"
#include <list>
#include <vector>
#include <string>
#include <functions/Point3D.h>

namespace UAVFlightPlan {
  
typedef enum {
  XML,
  QGC
}PLAN_TYPE;

class UAVFlightPlan:public std::list<UAVWaypoint>
{
  public:
		//! @brief Default Constructor
    UAVFlightPlan();
		
    //! @brief Constructor from XML file
    //! @param s Input filename
    UAVFlightPlan(const std::string &s, PLAN_TYPE type = QGC, bool loop = false);
    
    //! @brief Constructor from a usual flight plan data
    UAVFlightPlan(const std::vector< functions::Point3D >& fp, const EarthLocation& e, bool loop = false, std::vector< bool > reach_altitude = std::vector<bool>());
    
    //! @brief Loads a flight plan described in UAV Navigation XML format
    //! @param s Input filename
    bool loadXMLFile(const std::string &s);
    
    //! @brief Represents the information of the class in a string
    //! @return A string with the content
    std::string toString() const;

#ifdef USE_KML
    //! @brief Exports the information to a KML file
    //! @param f Filename
    //! @retval true The file has been exported successfully
    //! @retval false Errors while exporting the file
    bool exportToKMLFile(const std::string &f, int begin = 0, int end = -1) const;
#endif
		
    void getVectors(std::vector<double> &lat_, std::vector<double> &lon_) const;

    //! @brief Translates the info into a QGC string with the format of a QGC File
    //! @param version The version that will be saved in the heading of the file
    //! @return The string with the QGC file info
    std::string toQGCString(unsigned int version = 120) const;
    
    bool toQGCFile(const std::string &filename, unsigned int version = 120) const;

    //! @brief Loads a QGC flight plan file
    //! @param filename The name of the file
    //! @retval true success
    //! @retval false error
    bool fromQGCFile(const std::string &filename);
    
    //! @brief Gets a QGC flight plan string
    //! @param data A string that contains the data
    //! @retval true success
    //! @retval false error
    bool fromQGCString(const std::string &data);
    
    std::vector<functions::Point3D> toRelativeFlightPlan(EarthLocation &center) const;
    
    bool fromRelativeFlightPlan(const std::string &s, EarthLocation &center);
   
    inline bool getLoop() const {return loop_;}
    
    inline void setLoop(bool loop) { loop_ = loop; }
    
  protected:
    static std::string wp_id;
    static std::string parent_id;
    static std::string qgc_id;
    
    bool loop_;
		
    void init();
};

}

#endif // UAVFLIGHTPLAN_H

