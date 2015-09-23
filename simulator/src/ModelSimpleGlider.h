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


#ifndef MODELSIMPLEGLIDER_H
#define MODELSIMPLEGLIDER_H

#include "Model.h"
#include <sparser/all.h>
#include "ThermalModel.h"

namespace simulator {

class ModelSimpleGlider: public Model
{
  private:
	static const double defaultTimeStep;
	static const double defaultInitialTime;
	
	
	//!Size of state vector
	static const int stateSize = 4; // x y h phi
	//!Size of control vector
	static const int controlSize = 2; // ??? phi_c
	//!Size of parameters vector
	static const int parameterSize = 3; // v alpha_phi
	
	double descending_rate;
	double vertical_wind_speed;
    double sim_t;
	
	
	Checker *getGliderChecker() const;
	
	virtual void init(const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control);
	


public:
  	ModelSimpleGlider() { init(); }
    
      /*!
	
	*/
	virtual ModelSimpleGlider *create(const std::vector< double >& _parameter,
		  const std::vector< double >& _state,
		  const std::vector< double >& _control) const;
	
		  
	virtual ModelSimpleGlider *createFromFile(const std::string &fileName) const;
	
	virtual ModelSimpleGlider *createFromBlock(ParseBlock &block) const;
		  
	virtual ModelSimpleGlider *clone() const;
	
	virtual ParseBlock *toBlock() const;
	
	//!Class destructor
    virtual ~ModelSimpleGlider();
	
	/*! @brief runs one step of the model, modifying its states
	
	*/
	virtual int runOneStep(bool first_state = false);
	
	/*! @brief runs N steps of the model, modifying its states
	
	*/
 	virtual int runNSteps(int n);

	/*! @brief returns the number of dimensions of the state
	@return state dimensions
	 */
	inline int getStateSize() const{return stateSize;}
	
	/*! Returns the number of dimensions of the control
	@return control vector dimensions
	 */
	inline int getControlSize() const{return controlSize;}
	
	/*!@brief Returns the number of dimensions of the parameter
	@return parameter dimensions
	 */
	inline int getParameterSize() const{return parameterSize; }
	
	inline ThermalModel *getThermalModel() {return thermal;}
	
	inline virtual double getMaxSpeed() const {return parameter.at(0);}
    inline virtual double getMinSpeed() const {return parameter.at(0);}
    
    double getFinalZ(const functions::Point3D &p1, const functions::Point3D &p2); 
    
    void init();
	
protected:
	ThermalModel *thermal;
	
	double getHDot() const;
};

} // namespace simulator

#endif // MODELSIMPLEGLIDER_H
