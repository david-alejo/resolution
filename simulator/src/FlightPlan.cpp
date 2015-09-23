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

#include "FlightPlan.h"
#include "functions/functions.h"
#include "functions/RealVector.h"
#include <math.h>

using namespace functions;
using namespace std;

namespace simulator {
FlightPlan::FlightPlan():vector<Point3D>() {
  init();
}
  
FlightPlan::FlightPlan(bool flag4d):vector<Point3D>()
{
  this-> is4d = flag4d;
}
	
FlightPlan::FlightPlan(const std::vector< double >& vec ): vector<Point3D>()
{
	init();
}

FlightPlan::FlightPlan(const std::string& filename): vector<Point3D>()
{
	load(filename);
}

FlightPlan::FlightPlan(ParseBlock& block): vector<Point3D>()
{
	init(block);
}

FlightPlan::FlightPlan(const simulator::FlightPlan& other): vector< functions::Point3D, std::allocator< functions::Point3D > >(other)
{
  this->is4d = other.is4d;
  ETA_vector = other.ETA_vector;
  this->cruise_speed = other.cruise_speed;
  this->reach_altitude = other.reach_altitude;
  this->climb_rate = other.climb_rate;
}


FlightPlan::FlightPlan(const vector< Point3D >& __a): vector< functions::Point3D, std::allocator< functions::Point3D > >(__a)
{
  init();
}

void FlightPlan::init()
{
  is4d = false;
  cruise_speed = default_cruise_speed;
}



bool FlightPlan::load(const std::string& fileName) throw()
{
	bool ret_value = true;
	
	try {
		ParseBlock flight_plan_data;
		flight_plan_data.load(fileName.c_str());
		init(flight_plan_data);
	} catch (std::runtime_error &e) {
		std::cout << "FlightPlan::load --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
    
	return ret_value;
}

// TODO: Make it possible to read a ETA vector with two components (or to include two time components specifying the time data)
bool FlightPlan::init(ParseBlock& parser) {
  init();
  bool ret_value = true;
  ETA_vector.clear();
	
  Checker *check = getFlightPlanChecker();
	
  try {
    parser.checkUsing(check);

    ParseBlock::Properties *ways = parser.getProperties("waypoint");
    ParseBlock::Properties::iterator it = ways->begin();
    
    string catec_file;
    if (parser.hasProperty("laas_file")) {
      catec_file = parser("laas_file").as<string>();
      importFromLAAS(catec_file);
    } 
		
    FormattedTime t;
    t.getTime();
    int cont = 0;

    if (parser.hasProperty("cruise_speed")) {
      cruise_speed = parser("cruise_speed").as<double>();
    }
		
    for (;it != ways->end() && catec_file == ""; it++) {
      vector<double> curr_way = (*it) -> as<vector<double> >();
      bool added = false;
      push_back(Point3D(curr_way));
      added = true;
      cont++;
			
      if (added && curr_way.size() > 3) {
	is4d = true;
	ETA_vector.push_back(t + curr_way.at(3));
      }
    }
    if (parser.hasProperty("reach_altitude")) {
      string ra = parser("reach_altitude").value;
      reach_altitude.clear();
      for (unsigned int i = 0; i < ra.size();i++) {
	if (ra.at(i) == '1') {
	  reach_altitude.push_back(true);
	} else {
	  reach_altitude.push_back(false);
	}
      }
    }
  } catch (std::exception &e) {
    cerr << "FlightPlan::init --> Error while intializing from a ParseBlock. Exception content: ";
    cerr << e.what() << "\n";
    throw(e);
  }

  delete check;
  return ret_value;
}

Checker *FlightPlan::getFlightPlanChecker() {
	Checker *ret = new Checker();
	
	ret->addProperty("waypoint", new OneOrMore());
	
	return ret;
}

double FlightPlan::distance() const
{
	double distance_ = 0;
	for (unsigned int i = 1; i < size(); i++) {
		distance_ += at(i).distance(at(i - 1));
	}
	
	return distance_;
}

string FlightPlan::toString() const
{
	ostringstream os;
	
	for (unsigned int i = 0; i < size(); i++) {
		os << "Waypoint " << i << ": (" << at(i).x;
		os << ", " << at(i).y;
		os << ", " << at(i).z;
		if (is4d) {
		  os << ", " << ETA_vector.at(i).getFormattedTime();
		}
		os << ")\t";
	}
	
	return os.str();
}

ParseBlock* FlightPlan::toBlock() const
{
  ParseBlock *ret_val = new ParseBlock;
  
  ret_val->setProperty("cruise_speed", functions::numberToString(cruise_speed));
  
  for (unsigned int i = 0; i < size(); i++) {
    string add(at(i).toString(false));
    if (is4d) {
      add.append(" ");
      add.append(ETA_vector.at(i).getFormattedTime());
    }
    ret_val->setProperty("waypoint", add);
    
    
  }
  
    
  return ret_val;
}


string FlightPlan::toMatlab(const std::string& matrix_name) const
{
	ostringstream os;
	os << matrix_name << " = [";
	
	for (unsigned int i = 0; i < size(); i++) {
		os << at(i).x << " " << at(i).y << " " << at(i).z;
		
		if (is4d) {
		  os <<  " " << ETA_vector.at(i).getFormattedTime();
		}
		
		if (i < size() - 1) {
			os << "; ";
		}
	}
	os << "];";
	
	return os.str();
}


FlightPlan& FlightPlan::operator=(const simulator::FlightPlan& plan)
{
	if (this != &plan) {
	
		clear();
		ETA_vector.clear();
		is4d = plan.is4d;
	
		for (unsigned int i = 0; i < plan.size(); i++) {
			this->push_back(plan.at(i));
			if (is4d) {
			  ETA_vector.push_back(plan.ETA_vector.at(i));
			}
			this->cruise_speed = plan.cruise_speed;
		}
		this->reach_altitude = plan.reach_altitude;
		this->climb_rate = plan.climb_rate;
	}
	
	return *this;
}

void FlightPlan::insert4d(FlightPlan::iterator pos, const Point3D& p, const FormattedTime &ETA)
{
  functions::FormattedTime ctime;
  ctime.getTime();
  if ( size() != ETA_vector.size() ) {
    ETA_vector.clear();
    for (unsigned int i = 0; i < size(); i++) {
      ETA_vector.push_back(ctime);
    }
  }
  is4d = true;
  
  vector< Point3D >::iterator it = begin();
  vector<functions::FormattedTime>::iterator ETA_it = ETA_vector.begin();
  for (; it != pos; it++, ETA_it++);
 
  ETA_vector.insert(ETA_it, ETA);
  insert(pos, p);
}

void FlightPlan::push_back4d(const functions::Point3D& p, const FormattedTime &ETA)
{
  functions::FormattedTime ctime;
  ctime.getTime();
  if ( size() != ETA_vector.size() ) {
    ETA_vector.clear();
    for (unsigned int i = 0; i < size(); i++) {
      ETA_vector.push_back(ctime);
    }
    
  }
  is4d = true;
  ETA_vector.push_back(ETA);
  push_back(p);
}

string FlightPlan::printAngles() const
{
  ostringstream os;
  
  for (unsigned int i = 1; i < size() - 1; i++) {
    RealVector v1(at(i - 1));
    RealVector v2(at(i));
    RealVector v3(at(i + 1));
    
    RealVector e1 = v2 - v1;
    RealVector e2 = v3 - v2;
    
    os << e1.angle(e2) << "\t";
  }
  
  return os.str();
}

string FlightPlan::toLatex(const string &caption, const string &label, double scale) const
{
  ostringstream os;
  
  os << "\\begin{table} %[h!]" << endl;
  os << "\\caption{" << caption << "}" << endl;
  os << "\\centering" << endl;
  os << "\\label{" << label << "}" << endl;
  os << "\\scalebox{" << scale << "} {                % change table size" << endl;
  os << "\\begin{tabular}{*{" ;
  if (is4d) {
    os << "4";
  } else {
    os << "3";
  }
  os << "}{|c}|c|} \\hline" << endl;
  os << "{Node} & X $(m)$ & Y $(m)$ & Z $(m)$ & Time $(s)$ \\\\ \\hline" << endl;
  
  for (unsigned int i = 0; i < size(); i++) {
    
    os << i + 1 << "& " << at(i).x << " & " << at(i).y << " & " << at(i).z;
    if (is4d) {
      os << " & " << ETA_vector.at(i);
    }
    os << " \\\\ \\hline" << endl;
  }

  os << "\\end{tabular}" << endl;
  os << "} % Scalebox" << endl;
  os << "\\end{table}" << endl;
  
  return os.str();
}

bool FlightPlan::isDifferent(const simulator::FlightPlan& other, uint first_way, double min_dist) const {
  bool ret_val = !(other.size() == size());

  for (unsigned int i = first_way; i < size() && !ret_val; i++) {
	  if ( at(i).distance(other.at(i)) > min_dist) {
		  ret_val = true;
	  }
  }

  return ret_val;
}

std::vector< double, std::allocator< double > > FlightPlan::getArrivalTimes() const
{
  vector<double> ret_val;
  double calc = 0.0;
  
  for (unsigned int i = 1; i < size(); i++) {
    if ( !is4d) {
      // If is not a 4d plan --> estimate the arrival time
      calc += at(i - 1).distance(at(i)) / cruise_speed;
    } else {
      // If it is a 4d plan --> get it from the already estimated arrival times :)
      calc = getETA(i) - getETA(0);
    }
    ret_val.push_back(calc);
  }
  
  return ret_val;
}

string FlightPlan::toCatec(int ID) const
{
  ostringstream os;
  
  double v;
  for (unsigned int i = 0; i < size(); i++) {
    os << ID << " ";
    os << at(i).toString(false) << " ";
    if (is4d) {
      if (i > 0) {
	v =  at(i).distance(at(i - 1)) / (getETA(i) - getETA(i - 1));
      } else {
	v = 0.0;
      }
      os << " " << v << endl;
    }
  }
  
  return os.str();
}


bool FlightPlan::importFromLAAS(const string& filename)
{
  bool ret_val = true;
  
  vector<vector<double> > matrix;
  
  functions::getMatrixFromFile(filename, matrix);
  
  if ( matrix.size() >0) {
    if (matrix.at(0).size() == 3) {
      // 3D plan (x, y, z) {
      for (unsigned int i = 0; i < matrix.size(); i++) {
	Point3D aux(matrix.at(i));
	
	if (i == 0) {
	  push_back(aux);
	} else if (at(size() - 1).distance(aux) > 0.001) {
	  push_back(aux);
	}
      }
	
      
    } else {
      // 4D plan --> the first coord is the time
      FormattedTime t;
      t.getTime();
      vector<double> aux(3);
      
      for (unsigned int i = 0; i < matrix.size(); i++) {
	
	for (unsigned int j = 1; j < 4; j++) {
	  aux[j - 1] = matrix.at(i).at(j);
	}
      
	Point3D aux_p(aux);
	if (i == 0) {
	  push_back4d(aux_p, t + matrix.at(i).at(0));
	} else if (at(size() - 1).distance(aux) > 0.01) {
	  push_back4d(aux_p, t + matrix.at(i).at(0));
	}
      }
    }
  }
  
  return ret_val;
}

void FlightPlan::importFromCatec(const vector<vector<double> > &aux, int id, const FormattedTime &t)
{
  clear();
  FormattedTime curr_time = t;
  for (unsigned int i = 0; i < aux.size(); i++) {
    const vector<double> &cur = aux.at(i);
    if ( (int)cur.at(0) == id) {
      vector<double> aux(3);
      for (unsigned int j = 1; j < 4; j++) {
	aux.at(j - 1) = cur.at(j);
      }
      Point3D cp(aux);
      
      if (size() > 0) {
	curr_time = curr_time + cp.distance(at(size() - 1)) / cur.at(4);
      }
      if ( cp.distance(at(size() - 1)) > 0.01 ) {
	push_back4d(aux, curr_time);
      }
    }
  }
}

vector<Point3D> FlightPlan::getReferenceTrajectory(double radius, double ascending_rate, double delta_x) const
{
  Point3D aux, shift, aux2, center;
  vector<Point3D> ret;
  int max_cont = 10000;
  int cont = 0;
  double delta_head;
  
  double shift_z_abs = 0.0;
  if (cruise_speed > 0.0001) {
    shift_z_abs = ascending_rate * (delta_x / cruise_speed);
  }
  
  // Copy the first point (starting point)
  if (size() > 0) {
    aux = at(0);
    
    double d_min, alpha, head;
    Point3D d1, d2;
    ret.push_back(aux);
    for (unsigned int i = 1; i < size(); i++) {
      alpha = 0.0;
      d1 = at(i) - aux;
      if(i < size() - 1) {
	// There is at least one waypoint left --> check for turns
	
	d2 = at(i + 1) - at(i);
	alpha = (M_PI - fabs(d1.getHeadingTo(d2))) * sign(d1.getHeadingTo(d2));
      } 
      
      // TODO: implement the circumference arc
      
      head = d1.getHeading();
      shift.x = cos(head) * delta_x;
      shift.y = sin(head) * delta_x;
      shift.z = (d1.z > 0)?shift_z_abs:-shift_z_abs; // TODO: calculate the height changes
      // If alpha < 0.01 --> No turns --> follow straight line to the next wp
      double min_dist = fabs((abs(alpha) < 0.01)?0.0:radius / tan(alpha * 0.5));
      
      // If the distance is longer than the separation between distance --> reduce the radius
      double lesser_dist = minimum(d1.norm2d(), d2.norm2d());
      double radius_effective = radius;
      
      if (min_dist > lesser_dist) {
	// Reduce radius
	min_dist = lesser_dist;
	radius_effective = min_dist * tan(alpha * 0.5);
      }
      for (cont = 0;aux.distance2d(at(i)) > maximum(delta_x, min_dist) && cont < max_cont;cont++, aux += shift) {
	if (shift.z > 0) {
	  aux.z = functions::minimum(at(i).z, aux.z);
	} else {
	  aux.z = functions::maximum(at(i).z, aux.z);
	}
	ret.push_back(aux);
      }
      if (fabs(alpha) < 0.01) {
// 	aux = at(i);
// 	ret.push_back(at(i));
      } else {
	// Put the arc of circumference --> first get the location of the radius and the angles of circumference
	delta_head = delta_x / radius_effective;
	// Calculate the center of the circumference 
	// It is displaced from aux perp to the curr heading a distance radius
	Point3D radius_vector(-sin(d1.getHeading())*sign(alpha), 
			      cos(d1.getHeading())*sign(alpha), 
			      0.0);
	radius_vector = radius_vector * radius_effective;
	Point3D center = aux + radius_vector;
	double inc_head = delta_head;
	double phi = (-radius_vector).getHeading();
	for (;inc_head < fabs(d1.getHeadingTo(d2)) - delta_head;inc_head += delta_head) {
	  aux.x = center.x + radius_effective * cos(phi + inc_head * sign(alpha));
	  aux.y = center.y + radius_effective * sin(phi + inc_head * sign(alpha));
	  aux.z += shift.z;
	  if (shift.z > 0) {
	    aux.z = functions::minimum(at(i).z, aux.z);
	  } else {
	    aux.z = functions::maximum(at(i).z, aux.z);
	  }
	  ret.push_back(aux);
	}
      }
    }
  }
  
  return ret;
}


	
}