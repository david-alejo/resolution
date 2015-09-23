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


#ifndef MODELQUAD_H__
#define MODELQUAD_H__

#include "Model.h"

namespace simulator {

class ModelQuad:public Model
{
  private:
	static const double defaultTimeStep;
	static const double defaultInitialTime;
	
	
	//!Size of state vector
	static const int stateSize = 5; // x, y, h, gamma, v, 
	//!Size of control vector
	static const int controlSize = 3; // gamma_c, v_c, z_c
	//!Size of parameters vector
	static const int parameterSize = 6; // alpha (gamma, v, h), max_h_dot, v_min, v_max)
	
	
	Checker *getQuadChecker() const;
	
	public:	
	
	/*!
	
	*/
	ModelQuad *create(const std::vector< double >& _parameter,
		  const std::vector< double >& _state,
		  const std::vector< double >& _control) const;
	
		  
	ModelQuad *createFromFile(const std::string &fileName) const;
	
	ModelQuad *createFromBlock(ParseBlock &block) const;
		  
	ModelQuad *clone() const;
	
	//!Class destructor
	~ModelQuad();
	
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
	
	inline virtual double getMaxSpeed() const {return parameter.at(5);}
	inline virtual double getMinSpeed() const {return parameter.at(4);}
	
  private: 
	void init(const std::vector< double >& _parameter,
		  const std::vector< double >& _state,
		  const std::vector< double >& _control);
		  
	ModelQuad();
	friend class ModelFactory;
};

}

#endif // MODELQUAD_H

