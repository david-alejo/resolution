#include "ControllerQuad.h"
#include <iostream>
#include <sstream>
#include <functions/functions.h>

using namespace std;
using namespace functions;

namespace simulator {

	const double ControllerQuad::defaultGoalDistance = 0.0;
	const double ControllerQuad::WP_TOO_NEAR = 0.1; // in meters

ControllerQuad::ControllerQuad(){
  pointersToNull();
  init();
}

void ControllerQuad::pointersToNull(){
  planeVec = NULL;
}

ControllerQuad * ControllerQuad::createFromFile(const std::string& fileName) const {
  //! \Note Dummy
  ControllerQuad *ret = NULL;
	
	try {
		ParseBlock data;
		data.load(fileName.c_str());
		ret = createFromBlock(data);
	} catch (std::runtime_error &e) {
		std::cout << "ControllerQuad::createFromFile --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
	
	return ret;
}

ControllerQuad* ControllerQuad::createFromBlock(ParseBlock& data) const
{
	ControllerQuad *ret = new ControllerQuad();
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
		std::cout << "ControllerQuad::createFromBlock --> Error while getting data from block: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
	
	delete check;
	
	return ret;
}

Checker *ControllerQuad::controllerFileChecker() const {
  //! \Note Dummy
  Checker *ret = new Checker();
	ret->addBlock("flight_plan", new NTimes(1));
	FlightPlan fp;
	ret->addChecker("flight_plan", fp.getFlightPlanChecker());
	return ret;
}

// ControllerQuad *ControllerQuad::create(const std::vector <double> &def){
//   if(def.size() % 3 != 1){
//     throw (std::runtime_error("ControllerQuad creation exception"));
//   }
//   
//   ControllerQuad * ret = new ControllerQuad();
//   double _speed;
//   FlightPlan _wps;
//   std::vector<double>::const_iterator wpit=def.begin();
//   
//   _speed = *wpit;
//   wpit++;
//   for (; wpit!=def.end();wpit+=3 ){
//     _wps.push_back( Point3D( *wpit,*(wpit+1),*(wpit+2) ) );
//   }
//   _wps.setCruiseSpeed(_speed);
//   ret->init();
//   ret->init(_wps, def);
//   
//   
//   return ret;
//    
// }


ControllerQuad::~ControllerQuad(){
  dispose();
}


void ControllerQuad::init(){
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
  
  #ifdef DEBUG_WITH_MATLAB
  ofstream mat_sta("sta.mutlab");
  ofstream mat_par("par.mutlab");
  ofstream mat_con("con.mutlab");
  ofstream mat_wps("wps.mutlab");
  #endif // DEBUG_WITH_MATLAB
}

void ControllerQuad::init(const FlightPlan& wpl, const std::vector<double> &initial_state)
{
  init();
  fp = wpl;
  stateVector = initial_state;
  nextWpIndex = 0;
  finalWaypointReached = false;
  double curDist = updateWaypoint();
  lastSign = (curDist < 0)?-1:1;
}

void ControllerQuad::dispose () throw () {
  fp.clear();
  init();
  Controller::dispose();
}

ControllerQuad *ControllerQuad::clone () const { 
  ControllerQuad *ret= new ControllerQuad();
  ret->init(*this);
  
  return ret;
}

void ControllerQuad::init (const ControllerQuad &other) {
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

std::string ControllerQuad::toString() const {
  using std::endl;
  std::ostringstream os;
  os << "ControllerQuad:" << endl;
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

void ControllerQuad::update(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control){
  parameterVector=_parameter;
  stateVector=_state;
  controlVector=_control;
  update();
}


void ControllerQuad::update(){
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
void ControllerQuad::update_parameter(){
//   parameterVector [0] = speed;
}


void ControllerQuad::update_control(){
    //! Here comes the control logic!
//    Control: [ c_phi c_v c_h ]
      controlVector[0] = atan2(fp[nextWpIndex].y - stateVector[1],
			                         fp[nextWpIndex].x - stateVector[0]);
      
      if (!fp.is4d_()) {
	controlVector[1] = fp.getCruiseSpeed(); // TODO: adapt for speed profiles and 4d flight plans
      } else {
	double t_to_ETA = fp.getETA(nextWpIndex) - fp.getETA(0) - t;
	if (t_to_ETA < 0) {
	  controlVector[1] = parameterVector.at(5);
	} else {
	  Point3D p(stateVector.at(0), stateVector.at(1), fp[nextWpIndex].z);
	  double dist_xy_wp = p.distance(fp[nextWpIndex]);

	  if (dist_xy_wp > min_wp_dist) {
	    controlVector[1] = dist_xy_wp / t_to_ETA;
	    controlVector[1] = saturate(controlVector[1], parameterVector.at(4), parameterVector.at(5));
	  }
	}
      }
      controlVector[2] = fp[nextWpIndex].z;
}
  

bool ControllerQuad::update_nextWpIndex(){
  Point3D curPoint(stateVector[0], stateVector[1], stateVector[2]);
  bool changed = false;
  double curDist = curPoint * (*(planeVec->planeVec)) + planeVec -> p; 

	//NOTE: Crossing a plane translated goalDistance.
  if( curDist > 0 && (goalDistance - curDist) < 0 && lastSign >0  ||
  	    curDist < 0 && (curDist + goalDistance) > 0 && lastSign <0 ||
  	    curDist > 0 && lastSign < 0  ||
  	    curDist < 0 && lastSign > 0
  ) {
    // Update the plane
    curDist = updateWaypoint();
    changed = true;
  }
  
  lastSign = (curDist > 0) ? 1: -1;
  return changed;
}

double ControllerQuad::updateWaypoint(bool vertical_plane) {
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
  if (vertical_plane) {
    prevWp.z = fp[nextWpIndex].z; //! \note The plane has to be vertical
  }
  planeVec = new HessianPlane(prevWp, fp[nextWpIndex]);
  
  return curPoint * (*(planeVec->planeVec)) + planeVec -> p; // Returns the distance to the plane
}


  void ControllerQuad::setFlightPlan(const FlightPlan &nwfp){
		delete planeVec;
		pointersToNull();
		init(nwfp, stateVector);
	}
	
FlightPlan ControllerQuad::getFlightPlan() const
{
	return fp;
}

int ControllerQuad::checkPlanFeasibility(const simulator::FlightPlan& _plan) const
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

int ControllerQuad::checkPlanFeasibility() const
{
	return checkPlanFeasibility(fp);
}

int ControllerQuad::checkTwoWpFeasibility(const functions::Point3D& a, const functions::Point3D& b) const
{
  double x_diff_wp=b.x - a.x;
  double y_diff_wp=b.y - a.y;
  double dist_xy_wp=sqrt( x_diff_wp*x_diff_wp + y_diff_wp*y_diff_wp );
  double max_z_diff_wp = parameterVector[2] *  dist_xy_wp / fp.getCruiseSpeed() ;
  //To avoid unexpected behavior when two or more waypoints are in the same xy loc.
  //Policy changed to throw exception whenever a plan has waypoints that 
  //makes this kind of things happen
  if(dist_xy_wp < WP_TOO_NEAR){
    return 1;
  }else if( abs(a.z - b.z) > max_z_diff_wp ){
    return 2;
  }
  return 0;
}

int ControllerQuad::checkTwoWpFeasibility4d(const functions::Point3D &a, const functions::Point3D &b, FormattedTime t_a, FormattedTime t_b) const
{
  double x_diff_wp=b.x - a.x;
  double y_diff_wp=b.y - a.y;
  double dist_xy_wp=sqrt( x_diff_wp*x_diff_wp + y_diff_wp*y_diff_wp );
  double t = t_b - t_a;
  double v = dist_xy_wp / t; // Mean speed 
  
  if (v < parameterVector.at(4) || v > parameterVector.at(5)) {
    return 3; // The ETA is unreacheable at the necessary times
  }
  
  double max_z_diff_wp = parameterVector[2] *  dist_xy_wp / fp.getCruiseSpeed() ;
  //To avoid unexpected behavior when two or more waypoints are in the same xy loc.
  //Policy changed to throw exception whenever a plan has waypoints that 
  //makes this kind of things happen
  if(dist_xy_wp < WP_TOO_NEAR){
    return 1; // The waypoints are too near in xy coords
  }else if( abs(a.z - b.z) > max_z_diff_wp ){
    return 2; // Too much z varation compared to xy variation
  }
  return 0;
}


ParseBlock* ControllerQuad::toBlock()
{
  ParseBlock *ret_val = new ParseBlock;
  
  ret_val->setBlock( "flight_plan", fp.toBlock());
  ret_val->setProperty("controller_type", getType());
  ret_val->setProperty("min_wp_dist", functions::numberToString( min_wp_dist));
  
  return ret_val;
}



	
}