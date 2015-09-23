#ifndef UPDRAFT_H__
#define UPDRAFT_H__

#include <functions/RealVector.h>
#include <functions/FormattedTime.h>
#include <sparser/all.h>
#include <string>

namespace simulator {

class Updraft {
public:
  Updraft();
  Updraft(const functions::RealVector &location, double wind_speed, double max_height, double radius);
  Updraft(const Updraft &other);
  Updraft(ParseBlock &data);
  
  void init();
  
  functions::RealVector getLocation() const;
  
  functions::RealVector getLocation(functions::FormattedTime &t) const;
  
  double getWindSpeed() const;
  
  inline double getMaxHeight() const {return max_height;}
  
  double setWindSpeed(double new_speed) { 
    wind_speed = new_speed;
  }
  
  //! @brief Updates parameters and catches the modification date.
  double updateParameters(const functions::RealVector &new_center, double new_speed, const functions::RealVector new_drift);
  
protected:
  
  functions::RealVector location; // TODO: Use it in order to get better estimates of position
  double wind_speed; // Initial 
  
public:
  
  functions::RealVector drift;
  functions::RealVector initial_location;
  functions::RealVector updated_location;
  functions::FormattedTime initial_time;
  functions::FormattedTime last_update;
  
  double max_height;
  double life_time;

  // The following two things are used in A* algorithm
  int max_loops;
  double radius;
  int graph_points;
  
  static Checker *getChecker();
  
  std::string toString() const;
  
  ParseBlock *toBlock() const;
  
  std::vector<functions::RealVector> getGraphLocations() const;
};

}

#endif
