/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

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


#include "ModelSimpleGlider.h"
#include "ThermalModelFactory.h"
#include <iostream>
#include <functions/functions.h>

using namespace std;
using namespace simulator;
using namespace functions;

const double ModelSimpleGlider::defaultTimeStep = 0.01;
const double ModelSimpleGlider::defaultInitialTime = 0.00;

int ModelSimpleGlider::runOneStep(bool first_state){
	//Calculate the derivative terms
	
	double x_dot, y_dot;
	double v, phi_dot, h_dot;
	const double &phi = state.at(3);
	
	phi_dot = (control.at(1) - phi); // First order dynamics
	phi_dot = reduceAngle(phi_dot); // Avoiding phic-phi_o to go out of the -PI,PI range
	phi_dot = parameter.at(1) * phi_dot;
	v = parameter.at(0);
			
	x_dot = v * cos( phi );
	y_dot = v * sin( phi );
	
	if (first_state) {
	  initial_state = state; 
	}

	h_dot = getHDot();

	//Modifying the current state vector
	state[0]+= x_dot * ts;	// x
	state[1]+= y_dot * ts;	// y
	state[2]+= h_dot * ts;	// h
	state[3] += phi_dot * ts;
// 	state[3] = reduceAngle(phi_dot); // Avoiding phic-phi_o to go out of the -PI,PI range
	
	sim_t += ts;
	
	return 0;
}

int ModelSimpleGlider::runNSteps(int n){
	//This is trivial, but other models may have different implementations for this.
	for (int  i = 0; i < n; i++){
	  runOneStep();
	}
	return 0;
}
ModelSimpleGlider* ModelSimpleGlider::create(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control) const{
  ModelSimpleGlider *ret= new ModelSimpleGlider();
  ret->init(_parameter,_state,_control);
  return ret;

}

void ModelSimpleGlider::init(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control){
  parameter = _parameter;
  control = _control;
  state = _state;
  ts = defaultTimeStep;
  ctime = defaultInitialTime;
  initial_state = _state;
  vertical_wind_speed = 0.0;
  descending_rate = parameter.at(0) * tan(parameter.at(2));
  modelType = "ModelSimpleGlider";
  thermal = NULL;
  sim_t = 0.0;
}

ModelSimpleGlider::~ModelSimpleGlider(){
  initial_state.clear();
	parameter.clear();
	state.clear();
	control.clear();
	delete thermal;
}

ModelSimpleGlider* ModelSimpleGlider::clone() const {
  ModelSimpleGlider *ret= new ModelSimpleGlider();
  ret->parameter = parameter;
  ret->control = control;
  ret->state = state;
  ret->ts = ts;
  ret->ctime = ctime;
  ret->initial_state = initial_state;
  ret->modelType = modelType;
  ret->descending_rate = descending_rate;
  ret->thermal = NULL;
  ret->sim_t = sim_t;
  if (thermal != NULL) {
    ret->thermal = thermal->clone();
  } 
    
  return ret;
}

ModelSimpleGlider* ModelSimpleGlider::createFromBlock(ParseBlock& modelData) const
{
	string f_controller,f_model;
	ModelSimpleGlider *ret = new ModelSimpleGlider();
	Checker *particleChecker = getGliderChecker();
	

	
	try {
		modelData.checkUsing(particleChecker);
		vector<double> control(controlSize, 0.0);
		ret->init(modelData("parameters").as<vector<double> >(), modelData("initial_conditions").as<vector<double> >(), control);
		ret->ts = modelData("T").as<double>();
		
		ret->thermal = ThermalModelFactory::createFromBlock(modelData["wind_model"]); // Create the thermal wind model
		
		// DEBUG
		if (ret->parameter.size() != parameterSize || ret->state.size() != stateSize) {
			cout << "Wrong Parameter size = " << ret->parameter.size() << endl;
			cout << "Wrong Initial state size = " << ret->state.size() << endl;
			throw(std::runtime_error("Bad vector size"));
		}
	} catch (std::runtime_error &e) {
		std::cout << "ModelSimpleGlider::createFromBlock --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		delete ret;
		throw(e);
	}
	catch (std::runtime_error *e) {
		std::cout << "ModelSimpleGlider::createFromBlock --> Error while loading data from file: ";
		std::cout << e->what() << std::endl;
		delete ret;
		throw(*e);
	}
	delete particleChecker;
	
	return ret;
}


ModelSimpleGlider* ModelSimpleGlider::createFromFile(const std::string& fileName) const {
  
	
	ModelSimpleGlider *ret = NULL;
  
  try {
		ParseBlock modelData;
		modelData.load(fileName.c_str());
		ret = createFromBlock(modelData);
	} catch (std::runtime_error &e) {
		std::cout << "ModelSimpleGlider::create_from_file --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
    
  return ret;
}

Checker *ModelSimpleGlider::getGliderChecker() const {

	Checker *checker = new Checker;
	checker->addProperty("parameters", new NTimes(1) );
	checker->addProperty("initial_conditions", new NTimes(1) );
	
	return checker;
}

ParseBlock* ModelSimpleGlider::toBlock() const
{
  ParseBlock *block = Model::toBlock();
  
  block->setBlock("wind_model", thermal->exportThermalData());
  
  return block;
}

double ModelSimpleGlider::getFinalZ(const Point3D& p1, const Point3D& p2)
{
  Point3D position(p1);
  double head = p1.getHeadingTo(p2);
  double final_t = p1.distance2d(p2) / parameter.at(0);
  Point3D v(cos(head), sin(head), 0.0);
  v = v * parameter.at(0);
  
  std::vector<double> ant_state = state;
  
  state[0] = position.x;
  state[1] = position.y;
  state[2] = position.z;
  
  for (double t = 0.0; t < final_t; t += ts) {
//     delta_p.z = 
    state[0] = position.x;
    state[1] = position.y;
    state[2] = position.z;
    v.z = getHDot();
    position = position + v * ts;
    
  }
  
  state = ant_state;
  
  return position.z;
}

double ModelSimpleGlider::getHDot() const
{
  double vertical_wind_speed = 0.0;
	
  if (thermal != NULL) {
    ThermalModelTime *tmt = dynamic_cast<ThermalModelTime *>(thermal);
    if (tmt != NULL) {
      vertical_wind_speed = tmt->getVerticalWindSpeed(state, sim_t);
    } else {
      vertical_wind_speed = thermal->getVerticalWindSpeed(state);
    }
  }
	
  return -descending_rate + vertical_wind_speed;
}


void ModelSimpleGlider::init()
{
  thermal = NULL;
  Model::init();
}

