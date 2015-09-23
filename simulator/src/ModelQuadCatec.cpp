/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "ModelQuadCatec.h"
#include <cmath>
#include <sparser/all.h>
#include <functions/functions.h>

using namespace std;
// using std::cout;
// using std::endl;

namespace simulator {
  
  const double ModelQuadCatec::defaultTimeStep = 0.01;
  const double ModelQuadCatec::defaultInitialTime = 0.00;
  
int ModelQuadCatec::runOneStep(bool first_state)
{
  //Calculate the derivative terms
  double x_dot, y_dot, phi_dot, v_dot;
  double v, phi, h_dot;
  double &alpha_phi = parameter.at(0);
  double &alpha_v = parameter.at(1);
  double &max_h_dot = parameter.at(2);
  double &v_min = parameter.at(3);
  double &v_max = parameter.at(4);
  double &theta = control.at(2);
  
  // Avoiding phic-phi_o to go out of the -PI,PI range
  phi = state.at(3);
  double delta_phi = control.at(0) - phi;
  while ( fabs ( delta_phi ) > M_PI) {
	  delta_phi = delta_phi + ( delta_phi > 0.0 ? -1.0 : 1.0 ) * 2.0 * M_PI;
  }
  
  v = state.at(4);
  v_dot = alpha_v * (control.at(1) - v);
  v = functions::saturate(v, v_min, v_max);
  double v_xy = v * cos(theta);
  x_dot = v_xy * cos(phi);
  y_dot = v_xy * sin(phi);
  phi_dot = alpha_phi * delta_phi; 
  
  if (first_state) {
    initial_state = state; 
  }
  h_dot =  v * sin(theta);

  //Constraint of h_dot:
  h_dot = functions::saturate(h_dot, -max_h_dot, max_h_dot);
  
  //Modifying the current state vector
  state[0]+= x_dot * ts;	// x
  state[1]+= y_dot * ts;	// y
  state[2]+= h_dot * ts;	// h
  state[3]+= phi_dot * ts; // gamma
  state[4]+= v_dot * ts; // v
  
  return 0;
}

ModelQuadCatec::ModelQuadCatec()
{

}

ModelQuadCatec::~ModelQuadCatec()
{
  initial_state.clear();
  parameter.clear();
  state.clear();
  control.clear();
}



ModelQuadCatec* ModelQuadCatec::create(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control) const
{
  ModelQuadCatec *ret= new ModelQuadCatec();
  ret->init(_parameter,_state,_control);
  return ret;
}

void ModelQuadCatec::init(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control)
{
  parameter = _parameter;
  state = _state;
  control = _control;
  ts = defaultTimeStep;
  ctime = defaultInitialTime;
  initial_state = _state;
  modelType = "ModelQuadCatec";
}


ModelQuadCatec* ModelQuadCatec::createFromBlock(ParseBlock& block) const
{
  string f_controller,f_model;

	ModelQuadCatec *ret = new ModelQuadCatec();
	Checker *checker = getQuadChecker();
	try {
		block.checkUsing(checker);
		vector<double> control(controlSize, 0.0);
		ret->init(block("parameters").as<vector<double> >(), block("initial_conditions").as<vector<double> >(), control);
		ret->ts = block("T").as<double>();
		
		// DEBUG
		if (ret->parameter.size() != parameterSize || ret->state.size() != stateSize) {
			cout << "Parameter size = " << ret->parameter.size() << endl;
			cout << "Initial state size = " << ret->state.size() << endl;
			throw(std::runtime_error("Bad vector size"));
		}
	} catch (std::runtime_error &e) {
		std::cout << "ModelQuadCatec::createFromBlock --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		delete ret;
		throw(e);
	}
	delete checker;
	
	return ret; 
}

ModelQuadCatec* ModelQuadCatec::createFromFile(const std::string& fileName) const
{
  ModelQuadCatec *ret = NULL;
  
  try {
		ParseBlock modelData;
		modelData.load(fileName.c_str());
		ret = createFromBlock(modelData);
	} catch (std::runtime_error &e) {
		std::cout << "ModelQuadCatec::create_from_file --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
    
  return ret;
}

int ModelQuadCatec::runNSteps(int n)
{
  //This is trivial, but other models may have different implementations for this.
  for (int  i = 0; i < n; i++){
    runOneStep();
  }
  return 0;
}

ModelQuadCatec* ModelQuadCatec::clone() const
{
  ModelQuadCatec *ret= new ModelQuadCatec();
  ret->parameter = parameter;
  ret->control = control;
  ret->state = state;
  ret->ts = ts;
  ret->ctime = ctime;
  ret->initial_state = initial_state;
  ret->modelType = modelType;
  return ret;
}

Checker *ModelQuadCatec::getQuadChecker() const {

	Checker *checker = new Checker;
	checker->addProperty("parameters", new NTimes(1) );
	checker->addProperty("initial_conditions", new NTimes(1) );
	
	return checker;
}

double ModelQuadCatec::getMaxSpeed() const
{
  return parameter.at(parameterSize - 1);
}

double ModelQuadCatec::getMinSpeed() const
{
  return parameter.at(parameterSize - 2);
}

}