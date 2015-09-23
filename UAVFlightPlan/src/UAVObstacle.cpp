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

#include "UAVObstacle.h"
#include "functions/functions.h"
#ifdef USE_KML
#include <kml/convenience/convenience.h>
#include <kml/dom.h>
#endif
#include "general.h"

#define KML_SEGMENTS 1500

namespace UAVFlightPlan {

#ifdef USE_KML
bool UAVObstacle::exportKMLFile(const std::string filename)
{
	kmldom::KmlFactory *fac = kmldom::KmlFactory::GetFactory();
	kmldom::CoordinatesPtr coord = kmlconvenience::CreateCoordinatesCircle(lat, lon, radius, KML_SEGMENTS);
	kmldom::LinearRingPtr ring = fac->CreateLinearRing();
	ring->set_coordinates(coord);
	kmldom::PlacemarkPtr place = kmlconvenience::CreateBasicPolygonPlacemark(ring);
	place->set_name(filename);
	
	
	kmldom::KmlPtr kml = fac->CreateKml();
	kml->set_feature(place);
	
	return writeKMLToFile(filename, kml);
}
#endif

UAVObstacle::UAVObstacle(double lat, double lon, double alt, double rad): EarthLocation(lat, lon, alt)
{
	radius = rad;
}

	
}
