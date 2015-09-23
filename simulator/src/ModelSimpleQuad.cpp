
#include "ModelSimpleQuad.h"

using namespace std;
#include <boost/concept_check.hpp>
#include <functions/functions.h>

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace simulator {

//Constants:

	const double ModelSimpleQuad::defaultTimeStep = 0.01;
	const double ModelSimpleQuad::defaultInitialTime = 0.00;

int ModelSimpleQuad::runOneStep(bool first_state){
	//Calculate the derivative terms
	
	double x_dot, y_dot;
	double v, phi, h_dot;
	phi = 	control.at(0); //No dynamics
	// Avoiding phic-phi_o to go out of the -PI,PI range
	phi = functions::reduceAngle(phi);
	v = 	parameter.at(0);
			
	x_dot = v * cos( phi );
	y_dot = v * sin( phi );
	
	if (first_state) {
	  initial_state = state; 
	}

	
	h_dot=  (parameter.at(1) / ts) * ( control.at(1) - state.at(2) );

	//Constraint of h_dot:
 	if ( fabs( h_dot ) > parameter.at(2) ) {
     		h_dot = parameter.at(2) * ( h_dot > 0.0 ? 1.0 : -1.0 );
	}
	//Modifying the current state vector
	state[0]+= x_dot * ts;	// x
	state[1]+= y_dot * ts;	// y
	state[2]+= h_dot * ts;	// h
}

int ModelSimpleQuad::runNSteps(int n){
	//This is trivial, but other models may have different implementations for this.
	for (int  i = 0; i < n; i++){
	  runOneStep();
	}
}
ModelSimpleQuad* ModelSimpleQuad::create(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control) const{
  ModelSimpleQuad *ret= new ModelSimpleQuad();
  ret->init(_parameter,_state,_control);
  return ret;

}

void ModelSimpleQuad::init(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control){
  parameter = _parameter;
  control = _control;
  state = _state;
  ts = defaultTimeStep;
  ctime = defaultInitialTime;
  initial_state = _state;
  modelType = "ModelSimpleQuad";
}

ModelSimpleQuad::ModelSimpleQuad(){
  
}
ModelSimpleQuad::~ModelSimpleQuad(){
  initial_state.clear();
	parameter.clear();
	state.clear();
	control.clear();
}

ModelSimpleQuad* ModelSimpleQuad::clone() const {
  ModelSimpleQuad *ret= new ModelSimpleQuad();
  ret->parameter = parameter;
  ret->control = control;
  ret->state = state;
  ret->ts = ts;
  ret->ctime = ctime;
  ret->initial_state = initial_state;
  ret->modelType = modelType;
  return ret;
}

ModelSimpleQuad* ModelSimpleQuad::createFromBlock(ParseBlock& modelData) const
{
	string f_controller,f_model;

	ModelSimpleQuad *ret = new ModelSimpleQuad();
	Checker *particleChecker = getQuadChecker();
	

	
	try {
		modelData.checkUsing(particleChecker);
		vector<double> control(controlSize, 0.0);
		ret->init(modelData("parameters").as<vector<double> >(), modelData("initial_conditions").as<vector<double> >(), control);
		ret->ts = modelData("T").as<double>();
		
		// DEBUG
		if (ret->parameter.size() != parameterSize || ret->state.size() != stateSize) {
			cout << "Parameter size = " << ret->parameter.size() << endl;
			cout << "Initial state size = " << ret->state.size() << endl;
			throw(std::runtime_error("Bad vector size"));
		}
	} catch (std::runtime_error &e) {
		std::cout << "ModelSimpleQuad::createFromBlock --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		delete ret;
		throw(e);
	}
	delete particleChecker;
	
	return ret;
}


ModelSimpleQuad* ModelSimpleQuad::createFromFile(const std::string& fileName) const {
  
	
	ModelSimpleQuad *ret = NULL;
  
  try {
		ParseBlock modelData;
		modelData.load(fileName.c_str());
		ret = createFromBlock(modelData);
	} catch (std::runtime_error &e) {
		std::cout << "ModelSimpleQuad::create_from_file --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
    
  return ret;
}

Checker *ModelSimpleQuad::getQuadChecker() const {

	Checker *checker = new Checker;
	checker->addProperty("parameters", new NTimes(1) );
	checker->addProperty("initial_conditions", new NTimes(1) );
	
	return checker;
}

}