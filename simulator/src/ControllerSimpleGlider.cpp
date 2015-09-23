#include "ControllerSimpleGlider.h"
#include <iostream>
#include <sstream>
#include <functions/functions.h>

#define DEBUG_WITH_MATLAB

using namespace std;
using namespace functions;

namespace simulator {

const double ControllerSimpleGlider::defaultGoalDistance = 0.0;
const double ControllerSimpleGlider::WP_TOO_NEAR = 0.1; // in meters

ControllerSimpleGlider::ControllerSimpleGlider(){
  pointersToNull();
  init();
}

void ControllerSimpleGlider::pointersToNull(){
  planeVec = NULL;
}

ControllerSimpleGlider * ControllerSimpleGlider::createFromFile(const std::string& fileName) const {
  //! \Note Dummy
  ControllerSimpleGlider *ret = NULL;
	
	try {
		ParseBlock data;
		data.load(fileName.c_str());
		ret = createFromBlock(data);
	} catch (std::runtime_error &e) {
		std::cout << "ControllerSimpleGlider::createFromFile --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
	
	return ret;
}

ControllerSimpleGlider* ControllerSimpleGlider::createFromBlock(ParseBlock& data) const
{
	ControllerSimpleGlider *ret = new ControllerSimpleGlider();
	Checker *check = controllerFileChecker();
	
	try {	
		data.checkUsing(check);
		FlightPlan init_plan(data["flight_plan"]);
		std::vector <double>initial_state;
		
		initial_state.push_back(init_plan[0].x);
		initial_state.push_back(init_plan[0].y);
		initial_state.push_back(init_plan[0].z);
		ret->init(init_plan, initial_state);
		ret->speed = data("speed").as<double>();
		
		if (data.hasProperty("min_wp_dist")) {
		  ret->min_wp_dist = data("min_wp_dist").as<double>();
		}
		
		if (data.hasProperty("z_tolerance")) {
		  ret->z_tolerance = data("z_tolerance").as<double>();
		}
		
	} catch (std::runtime_error &e) {
		std::cout << "ControllerSimpleGlider::createFromBlock --> Error while getting data from block: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
	
	delete check;
	
	return ret;
}

Checker *ControllerSimpleGlider::controllerFileChecker() const {
  //! \Note Dummy
  Checker *ret = new Checker();
	ret->addProperty("speed", new NTimes(1));
	ret->addProperty("min_wp_dist", new NTimes(1));
	ret->addBlock("flight_plan", new NTimes(1));
	FlightPlan fp;
	ret->addChecker("flight_plan", fp.getFlightPlanChecker());
	return ret;
}

ControllerSimpleGlider *ControllerSimpleGlider::create(const std::vector <double> &def){
  if(def.size() % 3 != 1){
    throw (std::runtime_error("ControllerSimpleGlider creation exception"));
  }
  
  ControllerSimpleGlider * ret = new ControllerSimpleGlider();
  double _speed;
  FlightPlan _wps;
  std::vector<double>::const_iterator wpit=def.begin();
  
  _speed = *wpit;
  wpit++;
  for (; wpit!=def.end();wpit+=3 ){
    _wps.push_back( Point3D( *wpit,*(wpit+1),*(wpit+2) ) );
  }
  
  ret->init();
  ret->init(_wps, def);
  ret->speed=_speed;
  
  return ret;
   
}


ControllerSimpleGlider::~ControllerSimpleGlider(){
  dispose();
}


void ControllerSimpleGlider::init(){
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
  
  z_tolerance = 10.0;
  min_wp_dist = 50;
  first_time = true;
  
}

void ControllerSimpleGlider::init(const FlightPlan& wpl, const std::vector<double> &initial_state)
{
  init();
  fp = wpl;
  stateVector = initial_state;
  nextWpIndex = 0;
  finalWaypointReached = false;
  double curDist = updateWaypoint();
  lastSign = (curDist < 0)?-1:1;
}

void ControllerSimpleGlider::dispose () throw () {
  fp.clear();
  delete planeVec;
  init();
	controlVector.clear();
	stateVector.clear();
	parameterVector.clear();
	
}

ControllerSimpleGlider *ControllerSimpleGlider::clone () const { 
  ControllerSimpleGlider *ret= new ControllerSimpleGlider();
  ret->speed = speed;
  ret->finalWaypointReached = finalWaypointReached;
  ret->nextWpIndex = nextWpIndex;
  ret->stateVector = stateVector;
  ret->parameterVector = parameterVector;
  ret->controlVector = controlVector;
	if (planeVec == NULL) {
		ret->planeVec = NULL;
	} else {
		ret->planeVec = planeVec->clone();
	}
  ret->lastSign = lastSign;
  ret->fp = fp;
  ret->goalDistance = goalDistance;
  ret->z_tolerance = z_tolerance;
  ret->min_wp_dist = min_wp_dist;
  ret->first_time = first_time;
  return ret;
}


std::string ControllerSimpleGlider::toString() const {
  using std::endl;
  std::ostringstream os;
  os << "ControllerSimpleGlider." << endl;
  os << " waypoints:";
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

void ControllerSimpleGlider::update(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control){
  parameterVector=_parameter;
  stateVector=_state;
  controlVector=_control;
  update();
}


void ControllerSimpleGlider::update(){
    
  update_nextWpIndex();
  update_control();
  update_parameter();
#ifdef DEBUG_WITH_MATLAB
  if (first_time) {
    ofstream mat_sta("sta.mutlab");
    ofstream mat_par("par.mutlab");
    ofstream mat_con("con.mutlab");
    ofstream mat_wps("wps.mutlab");
    first_time = false;
  } 
  
  ofstream mat_sta("sta.mutlab",std::fstream::app | std::fstream::out);
  ofstream mat_par("par.mutlab",std::fstream::app | std::fstream::out);
  ofstream mat_con("con.mutlab",std::fstream::app | std::fstream::out);
  ofstream mat_wps("wps.mutlab",std::fstream::app | std::fstream::out);
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
  
  if (nextWpIndex < fp.size()) {
    mat_wps << fp[nextWpIndex].x << " " << fp[nextWpIndex].y << " " <<  fp[nextWpIndex].z << std::endl;
  } else {
    mat_wps << "0 0 0 " << std::endl;
  }
//   mat_wps << nextWpIndex << std::endl;
  mat_wps.close();
  mat_sta.close();
  mat_con.close();
  mat_par.close();
  
#endif //DEBUG_WITH_MATLAB
  
}
void ControllerSimpleGlider::update_parameter(){
  parameterVector [0] = speed;
}


void ControllerSimpleGlider::update_control(){
    //! Here comes the control logic!
//    Control: [ c_phi c_h ]
      controlVector[1] = atan2(fp[nextWpIndex].y - stateVector[1],
			                         fp[nextWpIndex].x - stateVector[0]);
      controlVector[0] = fp[nextWpIndex].z;
}
  

bool ControllerSimpleGlider::update_nextWpIndex(){
  Point3D curPoint(stateVector[0], stateVector[1], stateVector[2]);
  bool changed = false;
  
  if (nextWpIndex < fp.size()) {
    Point3D nextWp(fp.at(nextWpIndex));
    double curDist = curPoint.distance2d(nextWp);

    if (curDist < min_wp_dist) {
      
      if (!fp.getReachAltitude(nextWpIndex) || abs(curPoint.z - nextWp.z) < z_tolerance) {
	// DEBUG
	cout << "Waypoint "<< nextWpIndex << " Reached!";
	curDist = updateWaypoint();
	changed = true;
	
	// DEBUG
	cout << ". Next waypoint. Next_wp = " << nextWpIndex << ". Reach altitude: " << functions::boolToString(fp.getReachAltitude(nextWpIndex)) << endl;
	
      }
    }
  }
  
  return changed;
}

double ControllerSimpleGlider::updateWaypoint(bool vertical_plane) {
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
    
  } while (nextWpIndex < (int)fp.size() && next_wp_dist < min_wp_dist && !fp.getReachAltitude(nextWpIndex));
  if (nextWpIndex  >= (int)fp.size()) {
	nextWpIndex--;
	finalWaypointReached = true;
  }
  
  double x_diff = fp[nextWpIndex].x - stateVector[0];
  double y_diff = fp[nextWpIndex].y - stateVector[1];
  double dist_xy = sqrt( x_diff * x_diff + y_diff * y_diff ); 

  //To avoid unexpected behavior when two or more waypoints are in the same xy loc.
  while (!finalWaypointReached && dist_xy < WP_TOO_NEAR && !fp.getReachAltitude(nextWpIndex)  ){
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
  if (vertical_plane) {
    prevWp.z = fp[nextWpIndex].z; //! \note The plane has to be vertical
  }
  planeVec = new HessianPlane(prevWp, fp[nextWpIndex]);
  
  return curPoint * (*(planeVec->planeVec)) + planeVec -> p; // Returns the distance to the plane
}


  void ControllerSimpleGlider::setFlightPlan(const FlightPlan &nwfp){
		delete planeVec;
		pointersToNull();
		init(nwfp, stateVector);
	}
	
FlightPlan ControllerSimpleGlider::getFlightPlan() const
{
	return fp;
}

int ControllerSimpleGlider::checkPlanFeasibility(const simulator::FlightPlan& _plan) const
{
	int ret = 0;
	for (unsigned int i = 1; i < _plan.size() && ret == 0; i++) {
		ret = checkTwoWpFeasibility(_plan.at(i-1), _plan.at(i));
	}
	
	return ret;
}

int ControllerSimpleGlider::checkPlanFeasibility() const
{
	return checkPlanFeasibility(fp);
}

int ControllerSimpleGlider::checkTwoWpFeasibility(const functions::Point3D& a, const functions::Point3D& b) const
{
  double x_diff_wp=b.x - a.x;
  double y_diff_wp=b.y - a.y;
  
  return 0;
}

ParseBlock* ControllerSimpleGlider::toBlock()
{
  ParseBlock *ret_val = new ParseBlock;
  
  ostringstream os;
  os << speed;
  ret_val->setProperty("speed", os.str());
  ret_val->setBlock( "flight_plan", fp.toBlock());
  ret_val->setProperty("controller_type", getType());
  ret_val->setProperty("z_tolerance", functions::numberToString(z_tolerance));
  ret_val->setProperty("min_wp_dist", functions::numberToString(min_wp_dist));
//   ret_val->setProperty("initial_state", functions::printVector(stateVector));
  
  return ret_val;
}

	
}