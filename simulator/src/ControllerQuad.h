#ifndef __CONTROLLER_QUAD_H__
#define __CONTROLLER_QUAD_H__

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

/** @brief Simple controller that uses ModelQuad and a simple waypoint ControllerFactory 
 */
class ControllerQuad : public Controller{
	public:
		static const int stateSize = 5;
		static const int controlSize = 3;
		static const int parameterSize = 4;
		static const double defaultGoalDistance;
		static const double WP_TOO_NEAR; // in meters

		
		/*! Creation from file.
		
		*/
		ControllerQuad *createFromFile(const std::string &fileName)const;
		
		/*! Creation from parse block.
		
		*/
		ControllerQuad *createFromBlock(ParseBlock &fileName)const;
		
// 		void initialize(const std::vector < double > &ic);

		//!Destructor
		~ControllerQuad();
		
		//! Only clone is allowed
		ControllerQuad * clone () const;  
			
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
		
		//!@brief Check if two points are feasible to be consecutively in a plan
		//! @retval 3 Impossible to satisfy the time limits
		//! @retval 2 Two points are too far in Z-axis
		//! @retval 1 Two points are too near in X-Y plane
		//! @retval 0 The plan is feasible
		int checkTwoWpFeasibility4d(const functions::Point3D &a, const functions::Point3D &b, 
			                    functions::FormattedTime t_a, functions::FormattedTime t_b) const;
		
		
		//! @brief Translate the content into a parse block
		virtual ParseBlock *toBlock();
		
		virtual std::string getType() const { return "ControllerQuad"; };
		
		inline virtual double calculateETA() const { 
		  if (fp.is4d_()) {
		    return fp.getETA(fp.size() - 1) - fp.getETA(0);
		  } else {
		    return fp.distance() / fp.getCruiseSpeed();
		  }
		}
		virtual void setState(const std::vector<double> &st) {
		  stateVector = st;
		}

	protected:
		//! Distance for circle arc turn 
		double goalDistance;
		
		double min_wp_dist;
		
		double t;
		
		//! Flag that shows that the final waypoint has been reached
		bool finalWaypointReached;
		
		//!@brief Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//!@brief init without arguments -> all internal data to empty or zero values
		void init();
	
		//!@brief calls sets and inits the index and planes with the given waypoints
		void init(const FlightPlan &wpl, const std::vector<double> &initial_state);
		
		//! @brief Copy constructor helper
		void init (const ControllerQuad &other);
		
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
		double updateWaypoint(bool vertical_plane = true);

		//!@brief Hidden default constructor (use preferred one always)
		ControllerQuad();
		//!@brief Hidden Copy constructor (use clone always)
		ControllerQuad (const ControllerQuad &that);
		//!@brief Hidden assignment operator (use clone always)
		ControllerQuad & operator = (const ControllerQuad &that);
		//!@brief Sets all pointers to Null
		void pointersToNull();

		friend class ControllerFactory;

protected:
    
};

}



#endif //__CONTROLLER_SIMPLE_H__
