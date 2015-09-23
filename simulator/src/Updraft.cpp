#include "Updraft.h"
#include <functions/functions.h>

using namespace std;
using namespace functions;

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace simulator {

Updraft::Updraft()
{
  init();
}

Updraft::Updraft(const functions::RealVector& location, double wind_speed, double max_height, double radius)
{
  init();
  this->location = location;
  this->wind_speed = wind_speed;
  this->max_height = max_height;
  this->radius = radius;
}

void Updraft::init()
{
  radius = 50.0; // Standard radius
  max_height = 0.0;
  wind_speed = 0.0;
  graph_points = 4;
  max_loops = 3;
  life_time = 900.0; // Less than 15 minutes
  for (int i = 0; i < 2; i++) {
    location.push_back(0.0);
    initial_location.push_back(0.0);
  }
  initial_time.getTime();
  last_update = initial_time;
}


Updraft::Updraft(const Updraft& other)
{
  init();
  location = other.location;
  wind_speed = other.wind_speed;
  max_height = other.max_height;
  radius = other.radius;
  max_loops = other.max_loops;
  graph_points = other.graph_points;
  initial_time = other.initial_time;
  initial_location = other.initial_location;
  drift = other.drift;
  life_time = other.life_time;
  last_update = other.last_update;
}

Updraft::Updraft(ParseBlock& data)
{
  Checker *check = getChecker();
  init();
  try {
    data.checkUsing(check);
    location = data("location").as<vector<double> >();
    initial_location = location;
    wind_speed = data("wind_speed").as<double>();
    max_height = data("max_height").as<double>();
    if (data.hasProperty("radius")) {
      radius = data("radius").as<double>();
    }
    if (data.hasProperty("max_loops")) {
      max_loops = data("max_loops").as<int>();
    }
    if (data.hasProperty("graph_points")) {
      graph_points = data("graph_points").as<int>();
    }
    
    if (data.hasProperty("drift")) {
      drift = data("drift").as<std::vector<double> >();
    } else {
      RealVector aux(2);
      drift = aux;
    }
    
    if (data.hasProperty("life_time")) {
      life_time = data("life_time").as<double>();
    }
    
  } catch (std::exception &e) {
    std::cerr << "AlgorithmConfig (initializer) --> failed to get the data from block. Content: ";
    std::cerr << e.what() << "\n";
  }
  delete check;
}

string Updraft::toString() const
{
  ostringstream os;
  
  os << "Location: " << location.toString() << "  ";
  os << "Wind speed: " << wind_speed << "  ";
  os << "Radius: " << radius << "  ";
  os << "Max Height: " << max_height << "  ";
  os << "Drift (m/s): " << drift.toString() << "  ";
  os << "Initial time (or detection time): " << initial_time.getFormattedTime() << "  ";
  os << "Life time: " << life_time << "  ";
//   os << "Graph points: " << graph_points << endl;
//   os << "Max loops: " << max_loops << endl;
  
  return os.str();
}

Checker* Updraft::getChecker()
{
  Checker *check = new Checker;
  check->addProperty("location", new NTimes(1));
  check->addProperty("wind_speed", new NTimes(1));
  check->addProperty("max_height", new NTimes(1));

  return check;
}

ParseBlock* Updraft::toBlock() const
{

  ParseBlock *target_block = new ParseBlock;
  target_block->setProperty("location", location.toMatlabString());
  target_block->setProperty("wind_speed", functions::numberToString(wind_speed));
 target_block->setProperty("radius", functions::numberToString(radius));
  target_block->setProperty("max_height", functions::numberToString(max_height));
  target_block->setProperty("drift", drift.toMatlabString());
//  target_block->setProperty("max_loops", functions::numberToString(max_loops));
//  target_block->setProperty("graph_points", functions::numberToString(graph_points));
  
  return target_block;
}

vector< RealVector > Updraft::getGraphLocations() const
{
  vector<RealVector> v;
  RealVector shift;
  shift.push_back(radius);
  shift.push_back(0.0);
  double angle = 2 * M_PI / graph_points;
  double x, y;
  
  for (int i = 0; i < graph_points; i++) {
    RealVector aux = location;
    aux = aux + shift;
    v.push_back(aux);
    x = shift.at(0);
    y = shift.at(1);
    shift.at(0) = x * cos(angle) - y * sin(angle);
    shift.at(1) = y * cos(angle) + x * sin(angle);
  }
  
  return v;
}

RealVector Updraft::getLocation() const
{
  FormattedTime t;
  t.getTime();
  return getLocation(t);
}


// TODO: use the last updated location (location field)
RealVector Updraft::getLocation(FormattedTime& t) const
{
  if (drift.size() != 0) {
    return location;
  }
  
//   double delta_t = t - initial_time;
  double delta_t = t - last_update;
  
  return location + drift * delta_t;
}

double Updraft::getWindSpeed() const {
  FormattedTime t1;
  t1.getTime();
  double delta_t = t1 - initial_time;
  double inc = functions::maximum(0.0, delta_t) / life_time;
  return wind_speed * (1 - inc);
}

double Updraft::updateParameters(const RealVector& new_center, double new_speed, const RealVector new_drift)
{
  last_update.getTime();
  location = new_center;
  wind_speed = new_speed;
  drift = new_drift;
}



} // namespace simulator
