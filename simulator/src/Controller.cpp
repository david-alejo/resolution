#include "Controller.h"

namespace simulator {
  
void Controller::init()
{

}

void Controller::init(const FlightPlan& wpl, const std::vector< double >& initial_state)
{
  init();
  fp = wpl;
  stateVector = initial_state;
  nextWpIndex = 0;
  finalWaypointReached = false;
  double curDist = updateWaypoint();
  lastSign = (curDist < 0)?-1:1;
}
  
void Controller::dispose()
{
  delete planeVec;
  controlVector.clear();
  stateVector.clear();
  parameterVector.clear();
}

  
}