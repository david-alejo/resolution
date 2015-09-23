#ifndef MODEL_SIMPLE_QUAD_H
#define MODEL_SIMPLE_QUAD_H
#include <vector>
#include <cmath>
#include <sparser/all.h>
#include <iostream>
#include "Model.h"

namespace simulator {

/*! @brief Model of a Simple Quadrotor

This model assumes that the quadrotor is always in horizontal  position
State: [ x y h ]
x: x-axis position in meters[m] 
y: x-axis position in meters[m] 
h: height in meters[m]

Control: [ c_phi c_h ]
c_phi: commanded heading
c_h: commanded height

Parameters: [ v_xy k_phi k_h max_phidot max_hdot]
v_xy: Speed of the quadrotor in the x-y plane
k_h: constant of the height state equation
max_hdot: Maximum height derivative


*/

class ModelSimpleQuad: public Model {
		
	private:
	static const double defaultTimeStep;
	static const double defaultInitialTime;
	
	
	//!Size of state vector
	static const int stateSize = 3; // x y h
	//!Size of control vector
	static const int controlSize = 2; // 
	//!Size of parameters vector
	static const int parameterSize = 3;
	
	
	Checker *getQuadChecker() const;
	
	public:	
	
	/*!
	
	*/
	ModelSimpleQuad *create(const std::vector< double >& _parameter,
		  const std::vector< double >& _state,
		  const std::vector< double >& _control) const;
	
		  
	ModelSimpleQuad *createFromFile(const std::string &fileName) const;
	
	ModelSimpleQuad *createFromBlock(ParseBlock &block) const;
		  
	ModelSimpleQuad *clone() const;
	
	//!Class destructor
	~ModelSimpleQuad();
	
	/*! @brief runs one step of the model, modifying its states
	
	*/
	int runOneStep(bool first_state = false);
	
	/*! @brief runs N steps of the model, modifying its states
	
	*/
 	int runNSteps(int n);

	/*! @brief returns the number of dimensions of the state
	@return state dimensions
	 */
	inline int getStateSize() const{return stateSize;};
	
	/*! Returns the number of dimensions of the control
	@return control vector dimensions
	 */
	inline int getControlSize() const{return controlSize;};
	
	/*!@brief Returns the number of dimensions of the parameter
	@return parameter dimensions
	 */
	inline int getParameterSize() const{return parameterSize; };
	
	inline virtual double getMaxSpeed() const {return parameter.at(0);}
	
	inline virtual double getMinSpeed() const {return parameter.at(0);}
	
  private: 
		
	
      
	void init(const std::vector< double >& _parameter,
		  const std::vector< double >& _state,
		  const std::vector< double >& _control);
		  
	ModelSimpleQuad();
	friend class ModelFactory;
};
}


#endif //#ifndef MODEL_SIMPLE_QUAD_H
