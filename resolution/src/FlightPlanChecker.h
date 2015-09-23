/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  sinosuke <email>

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


#ifndef FLIGHTPLANCHECKER_H
#define FLIGHTPLANCHECKER_H

#include <simulator/Simulator.h>
#include <vector>

namespace resolution {

typedef std::pair<double, int> ArrivalEvent;
typedef std::vector<ArrivalEvent> EventVector;

bool eventComparator(const ArrivalEvent &first, const ArrivalEvent &second);

class FlightPlanChecker
{
  public:
  //! @brief Checks the safety of the plans taking into account collision between UAVs.
  static bool checkFlightPlans(const std::vector<simulator::FlightPlan> &plans, simulator::Simulator *sim);
  
  static bool checkFlightPlans(const simulator::FlightPlan& fp1, const simulator::FlightPlan& fp2, 
			       const simulator::Particle *uav1, const simulator::Particle *uav2, const simulator::CD *checker);
  
  //! @brief Rearranges the vents by increasing time
  static void arrangeEvents(EventVector &ev);
  
private:
  //! @brief This is the fundamental functions. Performs the check between two positions in time by making simple comparisions.
  // TODO: differe
  static bool segmentCheck(functions::RealVector pos1_0, functions::RealVector pos1_1, functions::RealVector pos2_0, functions::RealVector pos2_1, std::vector< double > geo1, std::vector< double > geo2, const simulator::CD *checker);
};

}

#endif // FLIGHTPLANCHECKER_H
