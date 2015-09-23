#ifndef MODEL_H
#define MODEL_H
#include <vector>
#include <string>
#include <iostream>
#include <sparser/ParseBlock.h>

namespace simulator {
/*! Model class
	Pure abstract Class (interface). All models have to inherit from it.

*/

class Model{ 
	public:
	//!@brief Destructor
	virtual ~Model(){};
	/*!@brief "Pure Virtual Constructor (PVC)" 
	@return Pointer to the new Model
	*/
	virtual Model *create(const std::vector<double> &_parameter,
			      const std::vector<double> &_state,
			      const std::vector<double> &_control) const = 0; 
	
	/*!@brief "Recommended Virtual Constructor (RVC)"
	@return Pointer to the new Model
	*/
	virtual Model *createFromFile(const std::string &fileName) const =0;
	
	
	/*!@brief "Recommended Virtual Constructor (RVC)"
	@return Pointer to the new Model
	*/
	virtual Model *createFromBlock(ParseBlock &fileName) const = 0;
	
	//! @brief Translate the content into a parse block
	virtual ParseBlock *toBlock() const;
	
	//! @brief Returns a string that represents the content of the class
	//! @return The string
	std::string toString() const;
			      
	/*!@brief Returns a deep copy of this object
	@return Pointer to the new Model
	*/
	virtual Model *clone() const = 0;
	
	/*! @brief runs one step of the model, modifying its states
	
	*/
	virtual int runOneStep(bool first_state = false) = 0;
	
	/*!@brief runs N steps of the model, modifying its states
	
	*/
	virtual int runNSteps(int n) =0;

	/*! @brief returns the current state vector of the model
	
	 */
	inline std::vector<double> getState() const{return state; };
	
	virtual std::vector<double> getInitialState() const;
	
	/*!@brief returns the number of dimensions of the state
	@return State dimensions
	*/
	virtual int getStateSize() const = 0;
	
	/*! @brief sets the current state vector. Useful for Initial Conditions.
	@param state: The state vector that will replace the current one
	*/
	inline void setState(const std::vector<double> _state){state=_state; };
	
	/*! @brief sets the current state vector and the initial state vector. Useful for Initial Conditions.
	@param state: The state vector that will replace the current one
	*/
	inline virtual void setInitialState(const std::vector<double> state) {
	  initial_state = state;
	  this->state = state;
	}
	
	/*! Returns the current control vector of the model
	*/
	inline std::vector<double> getControl() const{return control;};
	
	/*! Sets the current control vector. Useful for Initial Conditions.
	@param _control: The control vector that will replace the current one
	 */
	inline void setControl(const std::vector<double> _control){control = _control; };
	
	/*! @brief Returns the number of dimensions of the control
	@return control dimensions
	 */
	virtual int getControlSize() const =0;
	
	/*!@brief Returns the current parameter vector of the model
	
	 */
	inline std::vector<double> getParameter() const{return parameter; };
	
	/*! @brief Returns the number of dimensions of the parameter vector
	@return parameters dimensions
	 */
	virtual int getParameterSize() const = 0;
	
	/*!@brief Sets the current parameter vector. Useful for Initial Conditions.
	@param _parameter The parameter vector that will replace the current one
	 */
	inline void setParameter(const std::vector<double> _parameter){parameter = _parameter; };
	
	/*! @brief Gets the current time step
	@return time step of the model in seconds [s]
	*/
	inline double getTimeStep() const {return ts;}
	
	/*! @brief Sets the current time step
	@param ts: time step that will replace the current one, in seconds [s]
	*/
	inline void setTimeStep(const double ts) {this->ts = ts;}
	
	
	/*! @brief Returns the type of the model
	@return string that identifies the type of the model
	*/
	inline std::string getType() const {return modelType;}
	
	/*! For any other functionality that is not modeled in this interface
	@param pointer A pointer to something
	@param type Type of the pointer, in order to identify it
	
	 */
	virtual void specialFeature(void ** pointer, const char *type ){};
	
	/*! Returns the time of the model
	 */
	inline double getTime() const {return ctime;}
	
	/*!  Sets the time of the model
	@param ti: new time (in seconds)
	 */
	inline void setTime(const double ti) {ctime = ti;}
	
	//! @brief Useful accesors necessary in order to make simple calculations
	virtual double getMaxSpeed() const = 0;
	virtual double getMinSpeed() const = 0;
	
	 friend class ModelFactory;
	 
	protected:
	//!ctime: Current time
	double ctime;
	//!ts: Current time step
	double ts;
	
	//!Strings that identifies the model type
	std::string modelType;
	  
	 //!state: Current state vector
	std::vector<double> state;
	
	//!control: Current control vector
	std::vector<double> control;
	  
	//! initial_state: Necessary for saving purposes (toBlock function)
	 std::vector<double> initial_state;
	 
	 //!params: array of parameters
	std::vector<double> parameter;
	
	virtual void init() {
	  state.clear();
	  control.clear();
	  parameter.clear();
	  modelType = "Uninitialized";
	  ts = 0.0;
	  ctime = 0.0;
	}
};
}

#endif //#ifndef MODEL_H