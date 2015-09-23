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


#include "FlightPlanChecker.h"

using namespace resolution;
using simulator::FlightPlan;
using simulator::Particle;
using std::vector;
using std::pair;
using functions::RealVector;

bool resolution::eventComparator(const ArrivalEvent& first, const ArrivalEvent& second)
{
  return first.first <= second.first;
}


bool FlightPlanChecker::checkFlightPlans(const std::vector< FlightPlan >& fp, simulator::Simulator *sim)
{
  bool ret_val = true;
  // We have to assure that no pair of vehicles collide:
  for (unsigned int i = 0; i < fp.size() - 1 && ret_val; i++) {
    for (unsigned int j = i + 1; j < fp.size() && ret_val; j++) {
      ret_val = FlightPlanChecker::checkFlightPlans(fp.at(i), fp.at(j), sim->getParticle(i), sim->getParticle(j), sim->getCollisionDetector());
    }
  }
  
  return ret_val;
}


bool FlightPlanChecker::checkFlightPlans(const FlightPlan& fp1, const FlightPlan& fp2, 
					 const Particle *uav1, const Particle *uav2, const simulator::CD *checker)
{
  bool ret_val = true;
  
  if (fp1.size() < 2 || fp2.size() < 2) {
    // First check --> the flight plans have to include at least initial and final position (segment)
    return ret_val;
  }
  
  // First we have to get the ETAs and translate them into Events
  vector<double> ETA1 = fp1.getArrivalTimes();
  vector<double> ETA2 = fp2.getArrivalTimes();
  
  vector<ArrivalEvent> events;
  
  
  for (unsigned int i = 0; i < ETA1.size(); i++) {
    events.push_back(std::make_pair(ETA1.at(i), 1));
  }
  for (unsigned int i = 0; i < ETA2.size(); i++) {
    events.push_back(std::make_pair(ETA2.at(i), 2));
  }
  arrangeEvents(events);
  
  RealVector pos1_0(fp1.at(0));
  RealVector pos2_0(fp2.at(0));
  int cont_1 = 0;
  int cont_2 = 0;
  double last_t;
  RealVector pos1_1(fp1.at(cont_1 + 1));
  RealVector pos2_1(fp2.at(cont_2 + 1));
  
  for (unsigned int i = 0; i < events.size() && ret_val && (cont_1 < ETA1.size() || cont_2 < ETA2.size()) ; i++) {
    if (cont_1 + 1 < fp1.size()) {
      pos1_1 = fp1.at(cont_1 + 1);
    }
    if (cont_2 + 1 < fp2.size()) {
      pos2_1 = fp2.at(cont_2 + 1);
    }
    // Call the segment checker
    if (events.at(i).second == 1 ) {
      // Changes the first uav --> interpolate the second (where will it finish?)
      if (cont_2 < ETA2.size()) {
	RealVector inc_pos = pos2_1 - pos2_0;
	pos2_1 = pos2_0 + inc_pos * (ETA1.at(cont_1) - last_t) / (ETA2.at(cont_2) - last_t);
      } else {
	pos2_1 = pos2_0; // check with the last position
      }
      ret_val = segmentCheck(pos1_0, pos1_1, pos2_0, pos2_1, uav1->getGeometry(), uav2->getGeometry(), checker);
      last_t = ETA1.at(cont_1);
      cont_1++; // The first UAV is the one that has reached the waypoint
    } else {
      if (cont_1 < ETA1.size()) {
	RealVector inc_pos = pos1_1 - pos1_0;
	pos1_1 = pos1_0 + inc_pos * (ETA2.at(cont_2) - last_t) / (ETA1.at(cont_1) - last_t);
      } else {
	pos1_1 = pos1_0; // check with the last position
      }
      
      ret_val = segmentCheck(pos1_0, pos1_1, pos2_0, pos2_1, uav1->getGeometry(), uav2->getGeometry(), checker);
      last_t = ETA2.at(cont_2);
      cont_2++;
    }
    
//     ret_val = segmentCheck(pos1_0, pos1_1, pos2_0, pos2_1, uav1->getGeometry(), uav2->getGeometry(), checker);
    // Actualize the initial positions
    pos1_0 = pos1_1;
    pos2_0 = pos2_1;
  }
  
  return ret_val;
}

void FlightPlanChecker::arrangeEvents(EventVector& ev)
{
  std::sort(ev.begin(), ev.end(), &eventComparator);
}


bool FlightPlanChecker::segmentCheck(functions::RealVector pos1_0, functions::RealVector pos1_1, functions::RealVector pos2_0, functions::RealVector pos2_1, std::vector< double > geo1, std::vector< double > geo2, const simulator::CD *checker)
{
  return !checker->segmentCollision(pos1_0, pos1_1, pos2_0, pos2_1, geo1, geo2);
}
