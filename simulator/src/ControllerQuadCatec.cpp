#include "ControllerQuadCatec.h"
#include <iostream>
#include <sstream>
#include <functions/functions.h>

using namespace std;
using namespace functions;

namespace simulator {

	const double ControllerQuadCatec::defaultGoalDistance = 0.0;
	const double ControllerQuadCatec::WP_TOO_NEAR = 0.1; // in meters

ControllerQuadCatec::ControllerQuadCatec(){
  pointersToNull();
  init();
}

void ControllerQuadCatec::pointersToNull(){
  planeVec = NULL;
}

ControllerQuadCatec * ControllerQuadCatec::createFromFile(const std::string& fileName) const {
  //! \Note Dummy
  ControllerQuadCatec *ret = NULL;
	
	try {
		ParseBlock data;
		data.load(fileName.c_str());
		ret = createFromBlock(data);
	} catch (std::runtime_error &e) {
		std::cout << "ControllerQuadCatec::createFromFile --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
	
	return ret;
}

ControllerQuadCatec* ControllerQuadCatec::createFromBlock(ParseBlock& data) const
{
	ControllerQuadCatec *ret = new ControllerQuadCatec();
	Checker *check = controllerFileChecker();
	
	try {	
		data.checkUsing(check);
		FlightPlan init_plan(data["flight_plan"]);
		std::vector <double>initial_state;
		
		initial_state.push_back(init_plan[0].x);
		initial_state.push_back(init_plan[0].y);
		initial_state.push_back(init_plan[0].z);
		ret->init(init_plan, initial_state);
		
		if (data.hasProperty("min_wp_dist")) {
		  ret->min_wp_dist = data("min_wp_dist").as<double>();
		}
		
	} catch (std::runtime_error &e) {
		std::cout << "ControllerQuadCatec::createFromBlock --> Error while getting data from block: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
	
	delete check;
	
	return ret;
}

Checker *ControllerQuadCatec::controllerFileChecker() const {
  //! \Note Dummy
  Checker *ret = new Checker();
	ret->addBlock("flight_plan", new NTimes(1));
	FlightPlan fp;
	ret->addChecker("flight_plan", fp.getFlightPlanChecker());
	return ret;
}

ControllerQuadCatec::~ControllerQuadCatec(){
  dispose();
}

void ControllerQuadCatec::init(){
  planeVec = NULL;
  nextWpIndex = 1; 
  lastSign = 0;
  goalDistance = defaultGoalDistance;
  finalWaypointReached = false;
  controlVector.clear();
  stateVector.clear();
  controlVector = vector <double>(controlSize,0.0);
  stateVector = vector <double>(stateSize,0.0);
  parameterVector = vector <double>(parameterSize,0.0);
  min_wp_dist = 0.5;
  t = 0;
}

void ControllerQuadCatec::init(const FlightPlan& wpl, const std::vector<double> &initial_state)
{
  init();
  fp = wpl;
  stateVector = initial_state;
  nextWpIndex = 0;
  finalWaypointReached = false;
  double curDist = updateWaypoint();
  lastSign = (curDist < 0)?-1:1;
}

void ControllerQuadCatec::dispose () throw () {
  fp.clear();
  init();
  Controller::dispose();
}

ControllerQuadCatec *ControllerQuadCatec::clone () const { 
  ControllerQuadCatec *ret= new ControllerQuadCatec();
  ret->init(*this);
  
  return ret;
}

void ControllerQuadCatec::init ( const ControllerQuadCatec &other) {
  this->finalWaypointReached = other.finalWaypointReached;
  this->nextWpIndex = other.nextWpIndex;
  this->stateVector = other.stateVector;
  this->parameterVector = other.parameterVector;
  this->controlVector = other.controlVector;
	if (other.planeVec == NULL) {
		this->planeVec = NULL;
	} else {
		this->planeVec = other.planeVec->clone();
	}
  this->lastSign = other.lastSign;
  this->fp = other.fp;
  this->goalDistance = other.goalDistance;
  
  this->min_wp_dist = other.min_wp_dist;
  this->t = other.t;
}


std::string ControllerQuadCatec::toString() const {
  using std::endl;
  std::ostringstream os;
  os << "ControllerQuadCatec:" << endl;
  os << "Waypoints: ";
  FlightPlan::const_iterator it;
  for(it = fp.begin(); it != fp.end(); it++){
    os << " " << (*it).toString() << endl;
  }
  os << " next wp:" << endl;
  os << " " << fp[nextWpIndex].toString() << endl;
  os << " goal plane: " << endl;		
  os << " " << planeVec->toString() << endl;
  os << endl;
  
  return os.str();
}

void ControllerQuadCatec::update(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control){
  parameterVector=_parameter;
  stateVector=_state;
  controlVector=_control;
  update();
}


void ControllerQuadCatec::update(){
  t += T;
  update_nextWpIndex();
  update_control();
  update_parameter();
#ifdef DEBUG_WITH_MATLAB
  
  ofstream mat_sta("sta.mutlab",ios_base::app);
  ofstream mat_par("par.mutlab",ios_base::app);
  ofstream mat_con("con.mutlab",ios_base::app);
  ofstream mat_wps("wps.mutlab",ios_base::app);
  vector<double>::iterator it;
  for(it= controlVector.begin(); it!=controlVector.end() ; it++){
    mat_con << *it << "\t";
  }
  mat_con << std::endl;
  for(it= stateVector.begin(); it!=stateVector.end() ; it++){
    mat_sta << *it << "\t";
  }
  mat_sta << std::endl;
  for(it= parameterVector.begin(); it!=parameterVector.end() ; it++){
    mat_par << *it << "\t";
  }
  mat_par << std::endl;
  
  mat_wps << wpVec[nextWpIndex].x << " " << wpVec[nextWpIndex].y << " " <<  wpVec[nextWpIndex].z << std::endl;
//   mat_wps << nextWpIndex << std::endl;
  
#endif //DEBUG_WITH_MATLAB
  
}
void ControllerQuadCatec::update_parameter(){
//   parameterVector [0] = speed;
}


void ControllerQuadCatec::update_control(){
  //! Here comes the control logic!
  //    Control: [ c_phi c_v c_theta ]
  Point3D goal = fp[nextWpIndex];
  Point3D p(stateVector.at(0), stateVector.at(1), stateVector.at(2));
  controlVector[0] = p.getHeadingTo(goal);
  double &min_speed = parameterVector.at(3);
  double &max_speed = parameterVector.at(4);
  
  
  if (!fp.is4d_()) {
    controlVector[1] = fp.getCruiseSpeed(); // TODO: adapt for speed profiles and 4d flight plans
  } else {
    double t_to_ETA = fp.getETA(nextWpIndex) - fp.getETA(0) - t;
    if (t_to_ETA < 0) {
      controlVector[1] = max_speed;
    } else {
      double dist_wp = p.distance(goal);

      if (dist_wp > min_wp_dist) {
	controlVector[1] = dist_wp / t_to_ETA;
	controlVector[1] = saturate(controlVector[1], min_speed, max_speed);
      }
    }
  }
  controlVector[2] = atan2(goal.z - p.z, p.distance2d(goal));
}
  

bool ControllerQuadCatec::update_nextWpIndex(){
  Point3D curPoint(stateVector[0], stateVector[1], stateVector[2]);
  bool changed = false;
  double curDist = curPoint * (*(planeVec->planeVec)) + planeVec -> p; 

  //NOTE: Crossing a plane translated goalDistance.
  if( (curDist > 0 && (goalDistance - curDist) < 0 && lastSign > 0)  ||
      (curDist < 0 && (curDist + goalDistance) > 0 && lastSign < 0) ||
      (curDist > 0 && lastSign < 0)  ||
      (curDist < 0 && lastSign > 0)
  ) {
    // Update the plane
    curDist = updateWaypoint(false);
    changed = true;
  }
  
  lastSign = (curDist > 0) ? 1: -1;
  return changed;
}

double ControllerQuadCatec::updateWaypoint(bool vertical_plane) {
  Point3D curPoint(stateVector[0], stateVector[1], stateVector[2]);
  Point3D prevWp(fp[nextWpIndex]);
  
  // Plane crossed: actualize the waypoint index
  double next_wp_dist;
  do {
    nextWpIndex++;
    
    if (nextWpIndex < (int)fp.size()) {
      next_wp_dist = prevWp.distance(fp[nextWpIndex]);
//       cout << "Next wp dist." << next_wp_dist <<"\n";
    }
    
  } while (nextWpIndex < (int)fp.size() && next_wp_dist < min_wp_dist);
  if (nextWpIndex  >= (int)fp.size()) {
	nextWpIndex--;
	finalWaypointReached = true;
  }
  
  double x_diff = fp[nextWpIndex].x - stateVector[0];
  double y_diff = fp[nextWpIndex].y - stateVector[1];
  double dist_xy = sqrt( x_diff * x_diff + y_diff * y_diff ); 

  //To avoid unexpected behavior when two or more waypoints are in the same xy loc.
  while (dist_xy < WP_TOO_NEAR && !finalWaypointReached){
    nextWpIndex++;
    
    if (nextWpIndex >= (int)fp.size()) {
      nextWpIndex--;
      finalWaypointReached = true;
    }
    x_diff = fp[nextWpIndex].x - stateVector[0];
    y_diff = fp[nextWpIndex].y - stateVector[1];
    dist_xy = sqrt( x_diff * x_diff + y_diff * y_diff ); 
  }
  
  // Free memory related to the previous plane
  delete planeVec;
//   if (vertical_plane) {
//     prevWp.z = fp[nextWpIndex].z; //! \note The plane has to be vertical
//   }
  planeVec = new HessianPlane(prevWp, fp[nextWpIndex]);
  
  return curPoint * (*(planeVec->planeVec)) + planeVec -> p; // Returns the distance to the plane
}


void ControllerQuadCatec::setFlightPlan(const FlightPlan &nwfp){
  delete planeVec;
  pointersToNull();
  init(nwfp, stateVector);
}

FlightPlan ControllerQuadCatec::getFlightPlan() const
{
  return fp;
}

int ControllerQuadCatec::checkPlanFeasibility(const simulator::FlightPlan& _plan) const
{
  int ret = 0;
  for (unsigned int i = 1; i < _plan.size() && ret == 0; i++) {
    
    if (!_plan.is4d_()) {
      ret = checkTwoWpFeasibility(_plan.at(i-1), _plan.at(i));
    } else {
      ret = checkTwoWpFeasibility4d(_plan.at(i-1), _plan.at(i), _plan.getETA(i - 1), _plan.getETA(i));
    }
  }
  
  return ret;
}

int ControllerQuadCatec::checkPlanFeasibility() const
{
  return checkPlanFeasibility(fp);
}

int ControllerQuadCatec::checkTwoWpFeasibility(const functions::Point3D& a, const functions::Point3D& b) const
{
  //makes this kind of things happen
  if(a.distance(b) < WP_TOO_NEAR){
    return 1;
  }
  return 0;
}

int ControllerQuadCatec::checkTwoWpFeasibility4d(const functions::Point3D &a, const functions::Point3D &b, FormattedTime t_a, FormattedTime t_b) const
{
  double t = t_b - t_a;
  double v = a.distance(b) / t; // Mean speed 
  
  if (v < parameterVector.at(3) || v > parameterVector.at(4)) {
    return 3; // The ETA is unreacheable at the necessary times
  }
  
  return checkTwoWpFeasibility(a, b);
}


ParseBlock* ControllerQuadCatec::toBlock()
{
  ParseBlock *ret_val = new ParseBlock;
  
  ret_val->setBlock( "flight_plan", fp.toBlock());
  ret_val->setProperty("controller_type", getType());
  ret_val->setProperty("min_wp_dist", functions::numberToString( min_wp_dist));
  
  return ret_val;
}



	
}