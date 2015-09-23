
/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#ifndef CRALGORITHM_H_H
#define CRALGORITHM_H_H

#include "AlgorithmConfig.h"
#include "CostConfig.h"
#include "CRAlgorithmStatistics.h"
#include <simulator/Simulator.h>
#include <sparser/all.h>
#include <simulator/FlightPlan.h>
#include <vector>
#include <functions/RealVector.h>
#include <simulator/ObstacleDetector.h>
#include <ga/GARealGenome.h>
#include <ga/GASelector.h>

namespace resolution {
  
  typedef enum { SPEED, COURSE, LEVEL }ManoeuvreType;
	
class CRAlgorithm
{
public:	
  virtual CRAlgorithmStatistics execute() = 0;
  
  std::vector<simulator::FlightPlan> getSolution() const;
  
  //! @brief Initializes the algorithm from a file with the common information (controlled UAVs vector, initial flight plans, etc)
  //! @param block The Parse Block that contains the file info
  //! @param config The configuration that has to be previously loaded from block
  //! @return A pointer to the algorithm
  virtual void init(ParseBlock &b, AlgorithmConfig *config);
  
  virtual ~CRAlgorithm();
  
  //! @brief Exports the solution flight plans to a file
  //! @param filename The name of the exported file
  //! @param sol The solution flight plans
  //! @retval true The write operation has been done successfully
  //! @retval false Error while exporting the file
  bool exportSolution(const std::string &filename, const std::vector<simulator::FlightPlan> &sol);
  
  //! @brief Generates Random Problem. Modifies the initial conditions and the goal waypoints
  //! @brief in order to generate a conflictive situation. 
  bool generateRandomProblem(int waypoint_dimension);
  
  //! @brief Saves the current problem to solve. 
  //! @retval true The problem has been solved successfully
  //! @retval false Errors while saving the problem
  bool saveProblem(const std::string& filename);
  
  virtual std::string getType() const = 0;
  
  inline AlgorithmConfig &getConfig() const {return *config;}
  
  inline simulator::Simulator *getSimulator() {return sim;}
  
  //! @brief Checks for conflicts in two different plans
  void checkFlightPlans(const simulator::FlightPlan& fp1, const simulator::FlightPlan& fp2, unsigned int uav1, unsigned int uav2);
  
    double testObjective(const functions::RealVector& v);
  
protected:
  AlgorithmConfig *config;
  ParseBlock block; // Used to save the data again
  simulator::Simulator *sim;
  std::vector<simulator::FlightPlan> initial_plans;
  std::vector<simulator::FlightPlan> solution;
  // Vector that indicates wich UAVs are controlled. If empty, all UAVs are controlled
  std::vector<bool> controlled_UAVs;
  // Stores the number of UAVs of the system (controlled and not controlled)
  int n_uavs;
  bool solved;
  
  
  std::vector<std::map<int, int> > speed_to_index; // Speed of waypoint n --> gene k
  
  void calculateSpeedMap(const std::vector<simulator::FlightPlan> &fp);
  
  //! @brief Creates an Algorithm from a file. Necessary
  //! @param block The Parse Block that contains the file info
  //! @return A pointer to the algorithm
  virtual CRAlgorithm *createFromBlock(ParseBlock &block) const = 0;
  
  //! @brief Gets the checker that will validate the algorithm input file
  //! @return The checker
  virtual Checker *getAlgorithmChecker() const;
  
  //! \brief Clears all the info of the base class
  virtual void dispose();
  
  //! @brief Initializes all the pointer of the base class to NULL
  virtual void pointersToNULL();
  
  inline int howManyControlledUAVs() const {
	  int ret_val = 0;
	  for (unsigned int i = 0; i < controlled_UAVs.size(); i++) {
		  if (controlled_UAVs[i]) {
			  ret_val++;
		  }
	  }
	  return ret_val;
  }
  
   //! @brief transaltes the ID of the controlled UAV to the ID of the simulator
  unsigned int getControledUAVID(unsigned int n_controlled_uav) const {
    if (controlled_UAVs.size() == 0) {
      return n_controlled_uav; // If the controlled UAVs is not present --> the ID is the same
    }

    unsigned int ret_val = 0;
    unsigned int count = 0;
    while (ret_val < controlled_UAVs.size() && count < n_controlled_uav) {
      
      ret_val++;
      count = (controlled_UAVs.at(ret_val)) ? count + 1 : count;
    }
    
    return ret_val;
    
  }
  
  bool getControlledUAVs(ParseBlock &block);
  
  friend class CRAlgorithmFactory;
  
  //! @brief Gets the problems bound (useful in waypoint change problems)
  //! @param vec A vector that will get the bounds
  virtual void getBounds(std::vector<double> &upper, std::vector<double> &lower) {
    upper.clear();
    lower.clear();
    CostConfig &config = dynamic_cast<CostConfig&>(*this->config);
    
    upper = config.upper_bounds;
    lower = config.lower_bounds;
  }
  
  //! @brief Restores the initial plans that were loaded.
  void restoreInitialPlans();
  
  //! @brief Calculates how many altitude levels will be possible in the 3D exploration
  //! @param lower Lowest allowed altitude
  //! @param up Greatest allowed altitude
  //! @param mid The default altitude that has be included in the list. Has to remain into [lower, up]
  //! @param step The step between two consecutive altitude levels
  //! @param ret An array that will contain the levels
  //! @return The number of levels
  int calculateLevels(float lower, float up, float mid, double step, float *&ret) const;
  
  //! @brief Translates the content of the class into a ParseBlock in order to write it to a file
  //! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
  virtual ParseBlock *toBlock() const;
  
  //! @brief Calls the simulator class with the desired flight plans
  //! @param fp The flight plans
  //! @param colli Collision?
  //! @retval true The simulation has been called successfully
  //! @retval false Problems while simulating the system
  virtual bool simulateSystem(std::vector< simulator::FlightPlan > fp, bool &colli);
  
  //! @brief gets the dimension of the problem
  virtual int getProblemDimensionWithoutTimeExploration() const;
  
  virtual int getProblemDimension() const;
  
  //! @brief gets the dimension of the problem with only one intermediate waypoint
  virtual int getProblemDimensionOneIntermediate() const;
  
  // ---- EVOLUTIONARY RELATED FUNCTIONS ---- TODO: make a subclass from these functions and pack it to that class.
  // TODO: crswarm and crgenetics must inherit from that subclass
  
  int iteration;
  int msg; // Controls the messages given by the cost evaluation functions
  
  //! @brief Saves the evolution data in a structure
  //! @param filename The place to save
  //! @param evo The evolution data to be saved
  bool saveEvolutionData(const std::string &filename, const std::vector<EvolutionData>& evo);

  //! @brief Calculates the proper objective as readed in the configuration file
  //! @return The objective function value
  double calculateObjectiveVec(const functions::RealVector& vec);
  
  //! @brief Calculates the proper objective as readed in the configuration file
  //! @return The objective function value
  double calculateObjective(const std::vector<simulator::FlightPlan> &fp);
  
  //! @brief Calculates the distance objective plus penalty (reduced objective)
  //! @return The objective function value
  double calculateReducedObjective(const std::vector<simulator::FlightPlan> &fp);
  
  //! @brief Calculates the difficult objective (takes into account the minimum miss distance) (min_dist objective)
  //! @return The objective function value
  double calculateMinDistObjective(const std::vector<simulator::FlightPlan> &fp);
  
  double calculateSpeedObjective(const std::vector<simulator::FlightPlan> &fp);
  
  //! @brief Calculates the cost depending on the variation on the speed in two consecutive segments of the flight plan
  double calculateDeltaSpeedObjective(const std::vector<simulator::FlightPlan> &fp);

  //! @brief Calculates the objective without simulation
  double calculateNoSimObjective(const std::vector<simulator::FlightPlan> &fp);
  
  double calculateManoeuvreObjective(const functions::RealVector& vec);
  
  double calculateObjectiveOne(const functions::RealVector& vec);
  
  //! @brief Converts the genome information into flight plans of each UAV
  //! @NOTE The flight plan of a UAV is a vector of Point3D generated by adding intermediate waypoints
  //! @param gen The genome to translate
  //! @return A vector with the Point3D properly constructed. 
  std::vector<simulator::FlightPlan> getFlightPlan(const functions::RealVector &vec) const;
  
  //! @brief Converts the genome information into flight plans of each UAV
  //! @NOTE The flight plan of a UAV is a vector of Point3D generated by modifying the intermediate waypoints
  //! @param gen The genome to translate
  //! @return A pointer with the Point3D properly constructed. 
  std::vector<simulator::FlightPlan> getModifiedFlightPlan(const functions::RealVector &vec) const;
  

  //! @brief Converts the genome information of MS-PSO into flight plans of each UAV
  //! @NOTE The flight plan of a UAV is a vector of Point3D generated by modifying the intermediate waypoints
  //! @param gen The genome to translate
  //! @return A vector with all the plans
  std::vector<simulator::FlightPlan> getManeouvreSelectionFlightPlan(const functions::RealVector &vec) const;
  
  //! @brief Gets the flight plan of the problem when one is plannimng vs the rest of UAVs (ICRA2011)
  //! @return The flight plan
  simulator::FlightPlan get1vsAllFlightPlan(const functions::RealVector &vec) const;

  //! @brief Prepare MS-PSO bounds (it's simple: all parameters are normalized and there are M(N+1) params (M = uavs, N = Intermediate ways)
  void prepareMSPSOBounds(std::vector< double >& upper, std::vector< double >& lower, std::vector< double >& speed);
  
  //! @brief Adds the proper bounds to the problem
  void prepareTimeBounds(std::vector<double> &upper, std::vector<double> &lower,std::vector<double> &speed);
  
  //! @brief Modifies the bounds to adapt them to the problem
  void prepareModifiedBounds(std::vector<double> &upper, std::vector<double> &lower,std::vector<double> &speed);
  
  //! @brief Adds the proper bounds to the problem with maintaining all eta of each vehicle
  void prepareMaintainETABounds(std::vector<double> &upper, std::vector<double> &lower, std::vector<double> &speed);
  
  //! @brief Calculates an additional penalty which is proportional to the variation of the speed in consecutive segments
  double deltaSpeedPenalty(const std::vector<simulator::FlightPlan> &fp) const;

  //! @brief Gets the initial plans with ETA information (necessary to calculate ETA Penalty)
  void getInitialPlans4D();
  
  //! @brief Calculates the ETA penalty due to the modifications in the trajectory 
  double ETAPenalty(const std::vector<simulator::FlightPlan> &fp) const;
  
  //! @brief Calculates the initial index of the information related to the cont_uav CONTROLLED UAV (does not directly take into account the ID of the UAV)
  unsigned int getManouvreSelectionIndex(int cont_uav) const {
    CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
    return cont_uav * (conf.intermediate_waypoints + 1); // initial index
  }
  
  ManoeuvreType getManoeuvreType(double genome_value) const ;
  
  void updateDeltaETA(EvolutionData &d) const;
  
  void getBounds();
  
  //! @brief Checks whether the problem to solve is valid (dimension of the bounds, etc)
  bool checkProblem() const;
  
  // Custom genetic operators
  friend float CRALgorithmObjective(GAGenome &);
  friend float CRALgorithmOneObjective(GAGenome &c);
  friend void GAPopulationInitializer(GAPopulation &);
  friend void GAGenomeRandomInitializer(GAGenome& g);
  friend int GAGenomeUniformWaypointMixer(const GAGenome &dad, const GAGenome &mom, GAGenome *g1, GAGenome *g2);
  friend int GARealGaussianMaMutator(GAGenome &gen, float pmut);
  friend int GARealUniformCrossoverCheck(const GAGenome& a, const GAGenome& b,
			    GAGenome* c, GAGenome* d);
  friend int GARealGaussianMutatorCheck(GAGenome& g, float pmut);
};


}


#endif // CRALGORITHM_H_H
