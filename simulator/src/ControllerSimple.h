#ifndef __CONTROLLER_SIMPLE_H__
#define __CONTROLLER_SIMPLE_H__

// #define DEBUG_WITH_MATLAB
#include <vector>
#include <sstream>
#include <cmath>
#include <iostream>
#include <map>
#include <sparser/all.h>
#include "FlightPlan.h"
#include "functions/Point3D.h"
#include "functions/HessianPlane.h"
#include "Model.h"
#include "Controller.h"

namespace simulator {

/** @brief Simple controller that uses ModelSimpleUAV and a simple waypoint ControllerFactory

This was created for use in CATEC testbed, but may be reused.

The contoller is created with 
create(const std::vector <double> &def)
where
def = [ speed x0 y0 z0 x1 y1 z1 ...]

speed is the speed in the x_y plane. 
x0,y0,z0 are the coordinates of the quadrotor at init
x1,y1,z1... are the coordinates of the waypoints to follow
  
 */
class ControllerSimple : public Controller{
	public:
		static const int stateSize = 3;
		static const int controlSize = 2;
		static const int parameterSize = 3;
		static const double defaultGoalDistance;
		static const double WP_TOO_NEAR; // in meters

		
		/*!FMS preferred constructor.
		\param def The first value is the speed. The rest are 3D waypoints
		*/
		ControllerSimple *create(const std::vector <double> &def);
		
		/*! Creation from file.
		
		*/
		ControllerSimple *createFromFile(const std::string &fileName)const;
		
		/*! Creation from parse block.
		
		*/
		ControllerSimple *createFromBlock(ParseBlock &fileName)const;
		
// 		void initialize(const std::vector < double > &ic);

		//!Destructor
		~ControllerSimple();
		
		//! Only clone is allowed
		ControllerSimple * clone () const;  
			
		//!Returns a string representation of the object
		std::string toString() const;
		
		virtual void setFlightPlan(const FlightPlan &new_plan);
		
		virtual FlightPlan getFlightPlan() const;
		
		inline virtual int getCurrentWaypoint() const { return nextWpIndex; }

		//!@brief Updates parameter, state and control vectors and calls update
		void update(const std::vector<double> &_parameter,
			const std::vector<double> &_state,
			const std::vector<double> &_control);
		
		//!@brief Updates all FMS based on the current parameter, state and control 
		void update();
		
		//!@brief Returns the value of finalWaypointReached.
		//! @return True if final waypoint has been reached
		inline bool hasReachedFinalWaypoint() const{return finalWaypointReached;};
		
		inline void set_lastSign(int val){lastSign=val;};
		inline void setControlVector(const std::vector<double> &v){controlVector=v;};
		inline void setStateVector(const std::vector<double> &v){stateVector=v;};
		inline int getLastSign(void)const {return lastSign;};
		inline std::vector <double> getControlVector() const {return controlVector;};
		inline std::vector <double> getStateVector() const {return stateVector;};
		inline std::vector <double> getParameterVector() const {return std::vector<double>();};
		
		//!@brief Check if a flight is feasible a priori
		//! @param _plan The plan to check for viability
		//! @retval 2 Two waypoints are too far in Z-axis
		//! @retval 1 Two waypoints are too near in X-Y plane
		//! @retval 0 The plan is feasible
		int checkPlanFeasibility(const FlightPlan &_plan) const; 
		
		//!@brief Check if the current plan is feasible a priori
		//! @retval 2 Two waypoints are too far in Z-axis
		//! @retval 1 Two waypoints are too near in X-Y plane
		//! @retval 0 The plan is feasible
		int checkPlanFeasibility() const; 
		
		//!@brief Check if two points are feasible to be consecutively in a plan
		//! @retval 2 Two points are too far in Z-axis
		//! @retval 1 Two points are too near in X-Y plane
		//! @retval 0 The plan is feasible
		int checkTwoWpFeasibility(const functions::Point3D &a,const functions::Point3D &b) const ;
		
		inline virtual void reset() {
			delete planeVec;
			std::vector<double> aux = stateVector;
			init(fp, aux);
		};
		
		//! @brief Translate the content into a parse block
		virtual ParseBlock *toBlock();
		
		virtual std::string getType() const { return "ControllerSimple"; };
		
		inline virtual double calculateETA() const { return fp.distance() / speed;}
		
		virtual void setState(const std::vector<double> &st) {
		  stateVector = st;
		}

	protected:
		//! Speed of the controller in the x-y plane (to set in v_xy parameter)
		double speed;
		
		//! Distance for circle arc turn 
		double goalDistance;
		
		//!@brief Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//!@brief init without arguments -> all internal data to empty or zero values
		void init();
	
		//!@brief calls sets and inits the index and planes with the given waypoints
		void init(const FlightPlan &wpl, const std::vector<double> &initial_state);
		
		//!@brief Update the State of WP
		//!@return True if changed
		bool update_nextWpIndex();
		
		//!@brief Updates the control signals to the model
		void update_control();
		
		//!@brief Updates the parameters that will be sent to the model
		void update_parameter();
		
		//! Checker for use FMS files with sparser 
		Checker *controllerFileChecker()const;
		
		//! @brief Updates the next waypoint number and the plane to be crossed. 
		//! If necessary raises the final waypoint flag.
		//! @param vertical_plane If true the plane is forced to be vertical
		//! @return The distance to the new plane
		virtual double updateWaypoint(bool vertical_plane = true);

		//!@brief Hidden default constructor (use preferred one always)
		ControllerSimple();
		//!@brief Hidden Copy constructor (use clone always)
		ControllerSimple (const ControllerSimple &that);
		//!@brief Hidden assignment operator (use clone always)
		ControllerSimple & operator = (const ControllerSimple &that);
		//!@brief Sets all pointers to Null
		void pointersToNull();

		friend class ControllerFactory;

};

}



#endif //__CONTROLLER_SIMPLE_H__
