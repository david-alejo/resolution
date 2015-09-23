#ifndef MODEL_SIMPLE_UAV_POLAR_WIND_H
#define MODEL_SIMPLE_UAV_POLAR_WIND_H
#include <vector>
#include <cmath>
#include <iostream>
#include "boost/random.hpp"

#include "Model.h"

namespace simulator {

class ModelSimpleUAVPolarWind: public Model{
  typedef  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > norm_rand;
	typedef  boost::variate_generator<boost::mt19937&, boost::random::weibull_distribution<> > wei_rand;
  
	private:
	  static const double defaultTimeStep;
	static const double defaultInitialTime;

	//!Size of state vector
	static const int stateSize = 6; // x y v theta h h_dot 
	//!Size of control vector
	static const int controlSize = 3; // v_ref theta_ref h_ref
	//!Size of parameters vector
	static const int parameterSize = 11; // alpha_v alpha_theta alpha_h alpha_h_dot v_min v_max
	
	public:	
	
	/*! ModelSimpleUAVPolarWind() constructor (private, so allways use new and create)
	@param pState (optional) The initial state vector. Default all 0. 
	[ ]
	@param pTs (optional) The intial Time Step in seconds. Default 0.1s
	@param pCtime (optional) The initial time in seconds. Default 0s.
	
	*/
	ModelSimpleUAVPolarWind(const std::vector<double> pState = std::vector<double>(),const double pTs = 0.1 , const double pCtime = 0.0);
	
	Model *create(const std::vector<double> &_parameter,
			      const std::vector<double> &_state,
			      const std::vector<double> &_control) const;
	
	Model *createFromFile(const std::string &filename) const;
	
	virtual Model *createFromBlock(ParseBlock &block) const;
	
	//!Class destructor
	~ModelSimpleUAVPolarWind();
	
	/*! @brief runs one step of the model, modifying its states
	@return error (0=OK)
	*/
	int runOneStep(bool first_state = false);
	
	/*! runNSteps: runs N steps of the model, modifying its states
	@return error (0=OK)
	*/
 	int runNSteps(int n);


	/*! returns the number of dimensions of the state
	@return state dimensions
	 */
	inline int getStateSize() const{return stateSize;};
	
	/*! Returns the number of dimensions of the control
	@return control dimensions
	 */
	inline int getControlSize() const{return controlSize;};
	
	/*! Returns the number of dimensions of the parameter
	@return parameter dimensions
	 */
	inline int getParameterSize() const{return parameterSize; };
	
	inline virtual double getMaxSpeed() const {return 0.0;}
	
	inline virtual double getMinSpeed() const {return 0.0;}
	
	void getWeibullParameters();
	
	Model *clone() const;
	
protected:
  void init(const std::vector< double >& _parameter,
		  const std::vector< double >& _state,
		  const std::vector< double >& _control);
  
  void getWindSpeeds(double &v_x, double &v_y);
  
  double k,c;
  
  boost::normal_distribution<> *normal_dist; 
  boost::random::weibull_distribution<> *weibull_dist; 
	
	friend class ModelFactory;
};

} // namespace simulator

#endif //#ifndef MODEL_SIMPLE_UAV_POLAR_WIND_H