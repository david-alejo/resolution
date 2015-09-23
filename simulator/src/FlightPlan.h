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

#ifndef _SIMULATOR_FLIGHTPLAN_H
#define _SIMULATOR_FLIGHTPLAN_H

#include <vector>
#include <functions/Point3D.h>
#include <functions/FormattedTime.h>

#include <sparser/all.h>


namespace simulator {
class FlightPlan:public std::vector<functions::Point3D>
{
  public:
    //!  @brief Default Constructor
    FlightPlan();
    FlightPlan(bool flag_4d);
    
    //! @brief Constructor from std vector of double
    //! @param vec The vector of double. Has to have dimension n*3
    FlightPlan(const std::vector<double> &vec);
    
    FlightPlan(const FlightPlan &other);

    
    //! @brief Constructor from file
    //! @param filename Filename
    FlightPlan(const std::string &filename);
    
    //! @brief Constructor from parse block
    //! @param filename Filename
    FlightPlan(ParseBlock &block);
    
    FlightPlan& operator = (const FlightPlan &other);
    
    //! @brief Returns the distance to be travel
    //! @ret The distance
    double distance() const;
    
    //! @brief Gets the Checker that will decide if the data will be valid
    //! @return A pointer to the Checker object.
    Checker *getFlightPlanChecker();
    
    //! @brief Return a string formatted for representation purposes
    //! @return The formatted string
    std::string toString() const;
    
    //! @brief Translate the content into a parse block
    virtual ParseBlock *toBlock() const;
    
    //! @brief Return a string with into Matlab format
    //! @return The formatted string
    std::string toMatlab(const std::string &matrix_name) const;
    
    //! @brief Return a string with into Latex format
    //! @return The formatted string
    std::string toLatex(const std::string &caption, const std::string &label, double scale = 1.0) const;
    
    //! @brief Inserts a waypoint at the end of the FP
    void push_back4d(const functions::Point3D& p, const functions::FormattedTime& ETA);
    
    //! @brief Inserts a waypoint p with ETA after position pos
    void insert4d(iterator pos, const functions::Point3D &p, const functions::FormattedTime& ETA);
    
    //! @brief Gets the ETA of waypoint n_waypoint
    functions::FormattedTime getETA(unsigned int n_waypoint) const {
      if (n_waypoint < size() && n_waypoint >= 0 && is4d) {
	return ETA_vector.at(n_waypoint);
      }
      functions::FormattedTime t;
      t.getTime();
      return t;
      
    }
    
    //! @brief Returns the turning angles between edges of the path
    //! @return A string with the angles
    std::string printAngles() const;

    //! @brief Checks for differences between two flight plans
    //! @param other The other FP
    //! @param first_way The first waypoint to Check (defaults to 0)
    //! @param min_dist The minimum distance between WPs to be considered different (defaults to 0.1)
    bool isDifferent(const FlightPlan &other, uint first_way = 0, double min_dist = 0.1) const;
    
    //! @brief Gets the cruise speed of the plan
    double getCruiseSpeed() const {return cruise_speed;}
    //! @brief Sets the cruise speed of the plan
    void setCruiseSpeed(double new_speed) {cruise_speed = new_speed;}
    
    inline bool is4d_() const {return is4d;}
    
    //! @brief Constructor from Point3D
    FlightPlan(const std::vector< functions::Point3D >& __a);
    
    //! Gets the ETA of each waypoint
    std::vector<double> getArrivalTimes() const;
    
    //! @brief Imports a plan in CATEC's format
    void importFromCatec(const std::vector< std::vector< double > >& aux, int id, const functions::FormattedTime& t);
    
    //! @brief Imports a flight plan from LAAS format
    bool importFromLAAS(const string& filename);
    
    std::string toCatec(int ID) const;
    
    void setReachAltitude(int i, bool ra) {
      if (i > size()){
	return;
      }
      if (reach_altitude.size() <= i) {
	for (unsigned int j = reach_altitude.size(); j < size(); j++) {
	  if ( j== i) {
	    reach_altitude.push_back(ra);
	  } else {
	    reach_altitude.push_back(false);
	  }
	
	}
      } else {
	reach_altitude.at(i) = ra;
      }
    }
    
    bool getReachAltitude(int i) const {
      bool ret_val = false;
      if (i < size() && i < reach_altitude.size()) {
	ret_val = reach_altitude.at(i);
      }
      return ret_val;
    }
    
    std::vector<bool> getReachAltitude() const { return reach_altitude;}
    
    //! @brief Sets the climb rate of waypoint pos
    inline void setClimbRate(unsigned int pos, double value) {
      for (unsigned int i = climb_rate.size(); i <= pos; i++) {
	climb_rate.push_back(0.0);
      }
      climb_rate.at(pos) = value;
    }
    
    //! @brief Gets the climb rate of the flight plan
    inline double getClimbRate(unsigned int i) const {
      double ret_val = 0.0;
      
      if (i < climb_rate.size() ) {
	ret_val = climb_rate.at(i);
      }
      
      return ret_val;
    }
    
    //! @brief Exports the flight plan into a reference trajectory composed by straight lines and arcs of circumferences
    //! @param res The resolution in m of the trajectory
    //! @param radius The radius of the circumferences
    vector< functions::Point3D > getReferenceTrajectory(double radius, double ascending_rate, double delta_x = 0.01) const;

  protected:
    bool load(const std::string &fileName) throw();
    
    
    
    // Stores the ETA of each waypoint. Necessary in 4D plans.
    std::vector<functions::FormattedTime> ETA_vector;
    
    bool is4d;
    std::vector<bool> reach_altitude;
    std::vector<double> climb_rate;
    
    //! Initializing method from a parseblock. Needs versión 0.1.7 of sparser
    //! @param parser The block where the waypoints are saved
    //! @retval true The data has been loaded successfully
    //! @retval false Could not retrieve the data
    bool init(ParseBlock& parser);
    
    //! @brief Basic initiallization
    void init();
    
    double cruise_speed;
    static const double default_cruise_speed = 20.0;
};

}

#endif // _SIMULATOR_FLIGHTPLAN_H
