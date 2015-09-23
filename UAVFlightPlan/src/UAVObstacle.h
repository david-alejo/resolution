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

#ifndef UAVOBSTACLE_H
#define UAVOBSTACLE_H

#include "earthlocation.h"
#include "config.h"

namespace UAVFlightPlan {

class UAVObstacle:public EarthLocation
{
	public:
		//! @brief Constructor with parameters
		//! @param lat Latitude in degrees
		//! @param lon Longitude  in degrees
		//! @param alt Altitude in meters above sea level
		//! @param rad Obstacle radius
    UAVObstacle(double lat = 0.0, double lon = 0.0, double alt = 0.0, double rad = 0.0);

#ifdef USE_KML
		//! @brief Exports a KML file with a circle that represents the obstacle
		bool exportKMLFile(const std::string filename);
#endif
	
	protected:
		double radius;
};

}

#endif // UAVOBSTACLE_H
