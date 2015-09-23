#ifndef __CONTROLLER_SIMPLE_GLIDER_H__
#define __CONTROLLER_SIMPLE_GLIDER_H__

// #define DEBUG_WITH_MATLAB
#include <vector>
#include <sstream>
#include <cmath>
#include <iostream>
#include <map>
#include <sparser/all.h>
#include "FlightPlan.h"
#include "functions/Point3D.h"
#include "Model.h"
#include "Controller.h"

namespace simulator {

/** @brief Simple controller that uses ModelSimpleGlider

The contoller is created with 
create(const std::vector <double> &def)
where
def = [ speed x0 y0 z0 x1 y1 z1 ...]

speed is the speed in the x_y plane. 
x0,y0,z0 are the coordinates of the glider at init
x1,y1,z1... are the coordinates of the waypoints to follow
  
 */
class ControllerSimpleGlider : public Controller{
	public:
		static const int stateSize = 4;
		static const int controlSize = 2;
		static const int parameterSize = 3;
		static const double defaultGoalDistance;
		static const double WP_TOO_NEAR; // in meters

		
		/*!FMS preferred constructor.
		\param def The first value is the speed. The rest are 3D waypoints
		*/
		ControllerSimpleGlider *create(const std::vector <double> &def);
		
		/*! Creation from file.
		
		*/
		ControllerSimpleGlider *createFromFile(const std::string &fileName)const;
		
		/*! Creation from parse block.
		
		*/
		ControllerSimpleGlider *createFromBlock(ParseBlock &fileName)const;
		
// 		void initialize(const std::vector < double > &ic);

		//!Destructor
		~ControllerSimpleGlider();
		
		//! Only clone is allowed
		ControllerSimpleGlider * clone () const;  
			
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
		
		
		
		//! @brief Translate the content into a parse block
		virtual ParseBlock *toBlock();
		
		virtual std::string getType() const { return "ControllerSimpleGlider"; };
		
		inline virtual double calculateETA() const { return fp.distance() / speed;} // TODO: include the thermal ascent
		
		virtual void setState(const std::vector<double> &st) {
		  stateVector = st;
		}

	protected:
		//! Speed of the controller in the x-y plane (to set in v_xy parameter)
		double speed;
		
		//! Flag that indicates if the altitude indicated in the waypoint has to be reached (i.e. inside a thermal)
		double z_tolerance;
		
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
		ControllerSimpleGlider();
		//!@brief Hidden Copy constructor (use clone always)
		ControllerSimpleGlider (const ControllerSimpleGlider &that);
		//!@brief Hidden assignment operator (use clone always)
		ControllerSimpleGlider & operator = (const ControllerSimpleGlider &that);
		//!@brief Sets all pointers to Null
		void pointersToNull();

		friend class ControllerFactory;
		bool first_time;
};

}



#endif //__CONTROLLER_SIMPLE_GLIDER_H__
