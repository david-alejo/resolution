#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include <string>
#include <vector>
#include "FlightPlan.h"
#include "functions/HessianPlane.h"

namespace simulator{

class Controller{
 
  public:
    //! Pure Virtual Contructor. Controllers are complex enough to be always created from a config file
    virtual Controller *createFromFile(const std::string &fileName) const = 0;
		
		//! Pure Virtual Contructor. Controllers are complex enough to be always created from a Parse Block
    virtual Controller *createFromBlock(ParseBlock &block) const = 0;

    //! Pure Virtual Constructor. Uses 'def' to define the object
//     virtual Controller *create(const std::vector<double> &def) = 0;
    
    virtual FlightPlan getFlightPlan() const = 0;
		
    virtual void setFlightPlan(const FlightPlan &new_plan) = 0;
		
    //! Getter of the current control vector
    virtual std::vector<double> getControlVector() const = 0;
    //! Getter of the current parameter vector (some controllers may change it)
    virtual std::vector<double> getParameterVector()const = 0;
		
    //!@brief Check if a flight is feasible a priori
    //! @param _plan The plan to check for viability
    //! @retval 2 Two waypoints are too far in Z-axis
    //! @retval 1 Two waypoints are too near in X-Y plane
    //! @retval 0 The plan is feasible
    virtual int checkPlanFeasibility(const FlightPlan &_plan) const = 0; 
		
    //!@brief Check if the current plan is feasible a priori
    //! @retval 2 Two waypoints are too far in Z-axis
    //! @retval 1 Two waypoints are too near in X-Y plane
    //! @retval 0 The plan is feasible
    virtual int checkPlanFeasibility() const = 0; 
		
    //! Updates the Controller with new information (usually called before using getters)
    virtual void update(const std::vector<double> &_parameter,
			const std::vector<double> &_state,
			const std::vector<double> &_control)=0;
    //!Destructor
    virtual ~Controller(){};

    //!Returns a deep copy of this object
    virtual Controller *clone() const = 0;
    friend class ControllerFactory;
		
    virtual void reset() {
      delete planeVec;
      std::vector<double> aux = stateVector;
      init(fp, aux);
    };
    
    //! @brief Translate the content into a parse block
    virtual ParseBlock *toBlock() = 0;
    
    virtual std::string getType() const = 0;
    
    virtual std::string toString() const = 0;
    
    virtual bool hasReachedFinalWaypoint() const = 0;
    
    virtual int getCurrentWaypoint() const = 0;
    
    virtual double calculateETA() const = 0;
    
    inline void setT(double T_) { T = T_; }

    virtual void setState(const std::vector<double> &st) = 0;
    
    virtual void dispose();
    
protected:
  double min_wp_dist;
  double T;
  //! Flag that shows that the final waypoint has been reached
  bool finalWaypointReached;
  //! (State of WP)Last sign of distance to the planeVec initially 0
  int lastSign;	
  
  //! (State of WP)Index of the waypoint that the UAV is following right now
  int nextWpIndex;
  
  //! Hessian form of the plane that defines the arrival to a waypoint
    functions::HessianPlane *planeVec;
  
  //!Current state vector \NOTE: State: [ x y h phi ]
  std::vector<double> stateVector;
		
  //!Current control vector \NOTE: control: [ c_phi c_h ]
  std::vector <double> controlVector;
		
  //!Current paramater vector \NOTE: control: [ v_xy k_phi k_h max_phidot max_hdot]
  std::vector <double> parameterVector;
		
  //! Reference waypoints list	
  FlightPlan fp;
  
  //! Basic initialization
  virtual void init();
  
  //!@brief calls sets and inits the index and planes with the given waypoints
  void init(const FlightPlan &wpl, const std::vector<double> &initial_state);
  
  virtual double updateWaypoint(bool vertical_plane = true) = 0;
};

}

#endif //_CONTROLLER_H_