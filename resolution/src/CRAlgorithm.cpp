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
    with this program;  if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "CRAlgorithm.h"
#include "functions/RandomNumberGenerator.h"
#include "functions/functions.h"
#include "CostConfig.h"
#include "FlightPlanChecker.h"

using namespace simulator;
using namespace functions;
using namespace std;

namespace resolution {
	
std::vector< simulator::FlightPlan > CRAlgorithm::getSolution() const
{
	return solution;
}

void CRAlgorithm::dispose()
{
	delete sim;
	delete config;
	
	CRAlgorithm::pointersToNULL();
}



void CRAlgorithm::pointersToNULL()
{
	sim = NULL;
	config = NULL;
}

void CRAlgorithm::init(ParseBlock& block, AlgorithmConfig *conf)
{
  config = conf;
  Checker *check = getAlgorithmChecker();

  try {
    block.checkUsing(check);
    sim = new simulator::Simulator(block["simulator"]);
    
    getInitialPlans4D();
    
    cout << "Plans: cruise speed " <<initial_plans.at(0).getCruiseSpeed() << endl;
    
    solution.clear();
    n_uavs = sim->howManyUAVs();
    getControlledUAVs(block);
    msg = 0;
    iteration = 0;
    solved = false;
  } catch (std::exception &e) {
    std::cerr << "CRAlgorithm::createFromBlock --> Error while catching data. Exception content: " << e.what() <<"\n";
    throw(e);
  }

  delete check;
}

void CRAlgorithm::getInitialPlans4D()
{
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  initial_plans.clear();
  vector<FlightPlan> ip = sim->getFlightPlans();
  
  for (unsigned int i = 0; i < ip.size(); i++) {
    const FlightPlan &fp = ip.at(i); 
    FlightPlan np;
    np.setCruiseSpeed(fp.getCruiseSpeed());
    FormattedTime t = conf.init_time;
    np.push_back4d(fp.at(0), t);
    for (unsigned int j = 1;j < fp.size();j++) {
      t = t + fp.at(j).distance2d(fp.at(j - 1)) / fp.getCruiseSpeed();
      np.push_back4d(fp.at(j), t);
    }
    
    initial_plans.push_back(np);
  }
//   cout << printToStringVector(initial_plans) << endl;
}


Checker* CRAlgorithm::getAlgorithmChecker() const
{
	Checker *ret = new Checker;
	
	ret->addBlock("simulator", new NTimes(1));
	ret->addBlock("config", new NTimes(1));
	
	return ret;
}

bool CRAlgorithm::getControlledUAVs(ParseBlock &block)
{
	controlled_UAVs.clear();
	controlled_UAVs = std::vector<bool>(n_uavs, true); // If not a thing is indicated --> all uavs are controlled
	
	bool ret_val = true;
	
	std::vector <double> vec;
	
	try {
		if (block.hasProperty("controlled_uavs")) {
			vec = block("controlled_uavs").as<std::vector<double> >();
		}
			
	} catch (...) {
			std::cerr << "CRGenetics::calculateControlledVector() --> Error while parsing the controlled vector.\n";
			ret_val = false;
	}
		
	for (int i = 0; i < controlled_UAVs.size() && ret_val && vec.size() > 0; i++) {
		controlled_UAVs[vec[i]] = false; // If the UAV does not appear in the list --> it is not controlled
	}
		
	for (unsigned int i = 0; i < vec.size() && ret_val; i++) {
		if (vec[i] < controlled_UAVs.size()) {
			controlled_UAVs[(int)vec[i] - 1] = true;
		}
	}
	
	if (config->debug) {
		std::cout << "CRGenetics::calculateControlledVector --> process finished " ;
		if (ret_val) {
			cout << "successfully.";
		} else {
			cout << "with errors.";
		}
		cout << " Number of controlled UAVs: " << howManyControlledUAVs() << "." << endl;
	}
	
	return ret_val;   
}

CRAlgorithm::~CRAlgorithm()
{
  dispose();
}

bool CRAlgorithm::exportSolution(const std::string& filename, const std::vector< FlightPlan >& sol)
{
	bool ret_val = true;
	
	ofstream file(filename.c_str());
	
	try {
		if (file.is_open()) {
			for (int i = 0; i < sol.size(); i++) {
				ostringstream os;
				os << "plan{" << i + 1 << "}";
				file << sol.at(i).toMatlab(os.str()) << endl;
			}
		} else {
			ret_val = false;
		}
		file.close();
	} catch (std::exception &e) {
		cerr << "CRAlgorithm::exportSolution --> Errors while exporting solution.\n";
	}
	
	return ret_val;
}

int CRAlgorithm::calculateLevels(float lower, float up, float mid, double step, float *&ret) const
{
	delete ret;
	
	float aux_array[100];
	
	int cont = 0;
	
	for (float aux = mid; aux >= lower && cont < 100; aux -= step, cont++) {
		aux_array[cont] = aux;
	}
		
	for (float aux = mid + step; aux <= up && cont < 100; aux += step, cont++) {
		aux_array[cont] = aux;
	}
	
	ret = new float[cont];
	for (int i = 0; i < cont; i++) {
		ret[i] = aux_array[i];
	}
		
	return cont;
}

bool CRAlgorithm::generateRandomProblem(int waypoint_dimension)
{
  bool ret_val = false;
  
  vector<double> lb, hb;
  getBounds(hb, lb);
  RandomNumberGenerator ran;
  
  double steps_x = 1.0; //meters (total must be multiplo) 
  double steps_y = 1.0; //meters (total must be multiplo) 
  double steps_z = 1.0; //meters (total must be multiplo) 
  double x_chunks= (hb[0]-lb[0])/steps_x ;
  double y_chunks= (hb[1]-lb[1])/steps_y ;
  double z_chunks;
  if (waypoint_dimension == 3) {
    z_chunks= (hb[2]-lb[2])/steps_z ;
  }
  
  
  vector<FlightPlan> new_plans(sim->getFlightPlans());
  
  double x,y,z;
  
  while (!ret_val) {
    bool plan_ok = false;
    
    while (!plan_ok) {
    
      for (unsigned int i = 0; i < sim->howManyUAVs(); i++) {
	FlightPlan aux_plan;
	
	aux_plan.setCruiseSpeed(initial_plans.at(i).getCruiseSpeed()); // The cruise speed has to be the same
	
	//***** Initial Point   
	//Chosing from the different posible entry planes ( x=lb, x=hb, y=lb, y=hb )
	// Discretized square perimeter
	//           
	//   _______2______  
	//   |             | 
	//   |             | 
	//  3|             | 1
	//   |             | 
	//   |             | 
	//   |_____________| 
	//          0 
	int in_side = floor(ran.rnd01()*4.0);
	switch (in_side){
	  case 0:
	    x = floor(ran.rnd01() * x_chunks) * steps_x + lb[0]; //random number of chunks * chunk_size
	    y= lb[1];
	    break;
	  case 1:
	    x = hb[0];
	    y = floor(ran.rnd01() *y_chunks) * steps_y + lb[1];
	    break;
	  case 2:
	    x = - floor(ran.rnd01() *x_chunks) * steps_x + hb[0];
	    y = hb[1];
	    break;
	  case 3:
	    x = lb[0];
	    y = - floor(ran.rnd01() *y_chunks) * steps_y + hb[1];
	    break;
	  default:
	    break;
	}
          
	if (waypoint_dimension == 3) {
	  z = floor(ran.rnd01() * z_chunks ) *steps_z + lb[2]; 
	} else {
	  z = new_plans[i].at(0).z;
	}
	aux_plan.push_back(Point3D(x,y,z));
	
	int out_side = floor(ran.rnd01()*4.0);
      
	while (out_side == in_side) {
	  out_side = floor(ran.rnd01()*4.0);
	}
      
      
	switch (out_side){
	  case 0:
	    x = floor(ran.rnd01() * x_chunks) * steps_x + lb[0]; //random number of chunks * chunk_size
	    y= lb[1];
	    break;
	  case 1:
	    x = hb[0];
	    y = floor(ran.rnd01() *y_chunks) * steps_y + lb[1];
	    break;
	  case 2:
	    x = - floor(ran.rnd01() *x_chunks) * steps_x + hb[0];
	    y = hb[1];
	    break;
	  case 3:
	    x = lb[0];
	    y = - floor(ran.rnd01() *y_chunks) * steps_y + hb[1];
	    break;
	  default:
	    break;
	}
      
	if (waypoint_dimension == 3) {
	  z = floor(ran.rnd01() * z_chunks ) *steps_z + lb[2]; 
	} else {
	  z = new_plans[i].at(0).z;
	}
	// Add the point to the plan
	aux_plan.push_back(Point3D(x,y,z));
	new_plans[i] = aux_plan;
	
	
	// Set the initial conditions of the model
	Particle *p = sim->getParticle(i);
	std::vector<double> st = p->getState();
	st.at(0) = aux_plan.at(0).x;
	st.at(1) = aux_plan.at(0).y;
	if (st.size() > 2) {
	  st.at(2) = aux_plan.at(0).z;
	}
	if (p->getModel()->getType() == "ModelQuadCatec") {
	  // Initial orientation
	  st.at(3) = aux_plan.at(0).getHeadingTo(aux_plan.at(1));
	  st.at(4) = 0.0;
	}
	p->getModel()->setInitialState(st);
	p->getModel()->setState(st);
      }
      if (config->debug) {
	cout << "CRAlgorithm::generateRandomProblem --> checking plans." << functions::printToStringVector(new_plans)<<endl;
      }
      if (config->double_geometry.size() > 0) {
	plan_ok = sim->checkFlightPlan(new_plans, config->double_geometry);
      } else {
	plan_ok = sim->checkFlightPlan(new_plans);
      }
      if (config->debug) {
	cout << "CRAlgorithm::generateRandomProblem --> plan checked!. Result: " << functions::boolToString(plan_ok) <<"\n";
      }
      
    }
    
    if (config->debug) {
      cout << "CRAlgorithm::generateRandomProblem --> Geometries after plan checked: " << printMatrix(sim->getGeometries()) << endl; 
    }
//     if (config->debug) {
//       simulator::SimConfig &sim_conf = sim->getConfig();
//       sim_conf.debug = true;
//     }
    
    bool collision;
    if (config->debug) {
      cout << "CRAlgorithm::generateRandomProblem --> Got the plan. Checking for collision... ";
    }
    
    bool old_debug = config->debug;
    config->debug = false;
    if (sim->run(collision, new_plans) ) {
      config->debug = old_debug;
      if (collision) {
	if (config->debug) {
	  cout << "Found!. Solving the problem. ";
	}
	initial_plans = new_plans;
	// A problem has been identified. Try to execute the algorithm
	CRAlgorithmStatistics stat = execute();
	if (stat.getSolved()) {
	  // If the problem was solved --> Houston we got a random problem!!
	  if (config->debug) {
	    cout << "Solved!.\n";
	  }
	  ret_val = true;
	} else {
	  if (config->debug) {
	    cout << "NOT solved!.\n";
	  }
	}
      } else {
	if (config->debug) {
	  cout << "NOT found!.\n";
	}
      }
    }
  }
  
  return ret_val;
}

bool CRAlgorithm::saveProblem(const std::string& filename) 
{
  restoreInitialPlans();
  ParseBlock *info_block = toBlock();
  bool ret_val = true;
  
  
  try {
    ofstream ofs;
    
    ofs.open(filename.c_str());
    
    if (ofs.is_open()) {
      ofs << *info_block;
      ofs.close();
    } else {
      ret_val = false;
      cerr << "CRAlgorithm::saveProblem --> could not open the file.\n";
    }
  } catch (exception &e) {
    ret_val = false;
    cerr << "CRAlgorithm::saveProblem --> caught an exception while writing the problem\n";
  }
  
  // Free memory
  delete info_block;
  
  return ret_val;
}

bool CRAlgorithm::saveEvolutionData(const std::string &filename, const std::vector<EvolutionData>& evo)
{
	bool ret_val = true;
	
	// Get the vectors to write
	vector<float> ex_time, cost;
	for (unsigned int i = 0; i < evo.size(); i++) {
		ex_time.push_back( evo[i].t);
		cost.push_back(evo[i].cost);
	}
		
	// Open the file
	ofstream file(filename.c_str());
		
	try {
		file << functions::vectorToMatlabString<float>("execution_time", ex_time)<<endl;
		file << functions::vectorToMatlabString<float>("cost", cost) << endl;
			
		for (int i = 0; i < evo.size(); i++) {
			for (int j = 0; j < evo[i].plan.size(); j++) {
				ostringstream vec_name;
				vec_name << "plan{" << i + 1 << "}" << "(" << j + 1 << ", :, :)";
				
				file << evo[i].plan[j].toMatlab(vec_name.str())<<endl;
			}
		}
		
	} catch (...) {
		ret_val = false;
	}
	
	file.close();
	
	return ret_val;
}

// TODO: develop saveProblem
ParseBlock *CRAlgorithm::toBlock() const {
    ParseBlock *block = new ParseBlock;
    block->setBlock("config", config->toBlock());
    block->setBlock("simulator", sim->toBlock());
    
    return block;
}

void CRAlgorithm::restoreInitialPlans()
{
  for (int i = 0; i < initial_plans.size(); i++) {
    sim->getParticle(i)->getController()->setFlightPlan(initial_plans.at(i));
  }
}

double CRAlgorithm::calculateObjective(const vector<simulator::FlightPlan> &fp) {
  double ret_val = 0.0;
  static bool show_err = true;
  
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  if (conf.objective_type == "reduced") {
    ret_val = calculateReducedObjective(fp);
    if (msg == 0) {
      cout << "CRAlgorithm::calculateObjective --> Using reduced objective.\n";
      msg = 1;
    }
  } else if (conf.objective_type == "min_dist") {
    ret_val = calculateMinDistObjective(fp);
    if (msg == 0) {
      cout << "CRAlgorithm::calculateObjective --> Using min distance objective.\n";
      msg = 1;
    }
  } else if (conf.objective_type == "speed") {
    ret_val = calculateSpeedObjective(fp);
    if (msg == 0) {
      cout << "CRAlgorithm::calculateObjective --> Using speed objective. Value: " << ret_val << "\n";
      msg = 1;
    }
  } else if (conf.objective_type == "delta_speed") {
    ret_val = calculateDeltaSpeedObjective(fp);
//     if (msg == 0) {
      cout << "CRAlgorithm::calculateObjective --> Using delta speed objective. Value: " << ret_val << "\n";
      msg = 1;
//     }
  } else if (conf.objective_type == "adaptative") {
    if (msg == 0) {
      cout << "CRAlgorithm::calculateObjective --> Using adaptative objective.\n";
      msg = 1;
    }
    if (solved || iteration == 0) {
      if (msg == 1) {
	cout << "CRAlgorithm::calculateObjective --> Adaptative: using reduced objective.\n";
	msg = 2;
      } else if (msg == 3) {
	cout << "CRAlgorithm::calculateObjective --> Adaptative: using reduced objective again.\n";
	msg = 4;
      }
      ret_val = calculateReducedObjective(fp);
    } else {
      if (msg == 2) {
	cout << "CRAlgorithm::calculateObjective --> Adaptative: using minimum distance objective.\n";
	msg = 3;
      }
      ret_val = calculateMinDistObjective(fp);
    }
  } else if (conf.objective_type == "no_sim") {
    if (msg == 0) {
      cout << "CRAlgorithm::calculateObjective --> Using no sim objective.\n";
      msg = 1;
    }
    ret_val = calculateNoSimObjective(fp);
  } else if (conf.objective_type == "eta") { 
    ret_val = conf.speed_factor * ETAPenalty(fp); // TODO: new parameter
    ret_val += calculateReducedObjective(fp);
  } else {
    if (show_err) {
      cerr << "CRAlgorithm::calculateObjective --> Error: not the objective type was not recognized. \n";
      show_err = false;
    }
    ret_val = calculateReducedObjective(fp);
  }
  
  return ret_val;
}

double CRAlgorithm::calculateObjectiveVec(const functions::RealVector& vec) {
  double ret_val;
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  if (conf.objective_type == "manoeuvre") {
    ret_val = calculateManoeuvreObjective(vec);
  } else {
  
    vector<FlightPlan> fps = getFlightPlan(vec);
    ret_val = calculateObjective(fps);
  }
  return ret_val;
}

double CRAlgorithm::calculateObjectiveOne(const RealVector& vec)
{
  double ret_val = 0.0;
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  FlightPlan plan = get1vsAllFlightPlan(vec);
  ret_val += plan.distance(); // Distance cost
  
  bool collision;
  
  if (!sim->check1vsAll(plan, collision) || collision) {
    // A collision has been produced --> add penalty
    ret_val += conf.collision_penalty;
  }
  
  if (config->debug) {
    cout << "Plan = " << plan.toString() << endl;
    cout << "Cost value: " << ret_val << "\t Collision = " << functions::boolToString(collision) << endl;
  }
  
  return ret_val;
}

FlightPlan CRAlgorithm::get1vsAllFlightPlan(const RealVector& vec) const
{
  FlightPlan plan;
  plan.push_back(initial_plans.at(0).at(0));
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  // Get the flight plan
  for (int j = 0; j < conf.intermediate_waypoints; j++) {
    // Calculate the initial index in the vector
    functions::Point3D new_way;
  
    if (conf.waypoint_dimension == 2) {
      new_way.init(vec.at(0), vec.at(1), plan.at(0).z);
    } else if (conf.waypoint_dimension == 3) {
      new_way.init(vec.at(0), vec.at(1), vec.at(2));
    }
    plan.push_back(new_way);
  }
  plan.push_back(initial_plans.at(0).at(1)); // TODO: no time exploration and initial plan of one goal waypoint
  return plan;
}


double CRAlgorithm::calculateReducedObjective(const vector<simulator::FlightPlan> &fps)
{
	double objective = 0.0; 
	CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
	
	sim->setCollisionStop(true); // if a collision is detected -->stop simulation
	
	if (conf.debug) {
		cout << "CRAlgorithm::calculateObjective --> simulating " << functions::printToStringVector(fps) <<endl;
	}
	
	
	
	for (int i = 0; i < n_uavs; i++) {
		if (controlled_UAVs[i]) {
			objective += fps[i].distance();
		}
		if (conf.debug) {
			cout << "Fp_" << i + 1 << " dist: " << fps[i].distance() << "    ";
		
		}
	}
	if (conf.debug) {
		cout << endl;
	}
	
	bool coll;
	if (simulateSystem(fps,coll)) {
		if (coll) {
			if (conf.debug) {
				cout << "CRAlgorithm::calculateObjective --> adding penalty: " << conf.collision_penalty << endl;
			}
				
			objective += conf.collision_penalty;
		} else {
		  solved = true;
		}
	} else {
		objective += 2.0 * conf.collision_penalty;
	}
	
	return objective;
}

double CRAlgorithm::calculateMinDistObjective(const vector<simulator::FlightPlan> &fps)
{
  double objective = 0.0;
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  if (sim != NULL) {
    sim->setCollisionStop(false);
    
    if (conf.debug) {
      cout << "CRAlgorithm::calculateMinDistObjective --> simulating. " <<endl ; //<< functions::printToStringVector(fps) <<endl;
    }
    
    for (int i = 0; i < n_uavs; i++) {
		if (controlled_UAVs[i]) {
			objective += fps[i].distance();
		}
		if (conf.debug) {
// 			cout << "Fp_" << i + 1 << " dist: " << fps[i].distance() << "    ";
		
		}
	}
	if (conf.debug) {
		cout << endl;
	}
	
	bool coll;
	if (simulateSystem(fps, coll)) {
	  double addition = 0.0;
	  if (coll) {
	    double sec_dist = sim->getGeometries()[0][0];
	    if (conf.debug) {
	      cout << "CRAlgorithm::calculateMinDistObjective --> min_dist = " << sim->getMinDistance() << "\t";
	    }
	    addition = conf.collision_penalty * (1 - sim->getMinDistance()/sec_dist);
	    // Assure a minimum penalty
	    addition = functions::maximum(0.0, addition); // Prevent negative penalities
	    objective += addition;
	    if (conf.debug) {
	      cout << "CRAlgorithm::calculateMinDistObjective --> adding penalty: " << addition << endl;
	    }
	  } else {
	    // No collisions, so the problem has been solved
	    solved = true;
	  }
	} else {
	  cout << "CRAlgorithm::calculateMinDistObjective --> Error. Adding double penalty. " << endl;
	  objective += 2.0 * conf.collision_penalty;
	}
    
  } else {
    cerr << "CRAlgorithm::calculateMinDistObjective --> Error: simulator not configured. This should not happen.\n";
  }
  
  return objective;
}

double CRAlgorithm::calculateSpeedObjective(const vector< FlightPlan >& fp)
{
  double ret_val = calculateReducedObjective(fp);
  
  for (unsigned int i = 0; i < fp.size(); i++) {
    const simulator::FlightPlan &curr_plan = fp.at(i);
    if (curr_plan.is4d_()) {	
      // Get the amount of time that has changed and then multiply by the cruise speed
      double t = curr_plan.getETA(curr_plan.size() - 1) - curr_plan.getETA(0);
      ret_val += fabs(t * initial_plans.at(i).getCruiseSpeed() - initial_plans.at(i).distance());
    }
  }
  
//   cout << "CRAlgorithm::calculateSpeedObjective --> Value: " << ret_val << " ";

  return ret_val;
}

double CRAlgorithm::calculateDeltaSpeedObjective(const vector<FlightPlan> &fp) {
  double ret_val = calculateReducedObjective(fp);
  ret_val += deltaSpeedPenalty(fp);
  
  return ret_val;
}

double CRAlgorithm::calculateManoeuvreObjective(const functions::RealVector& vec)
{
  double cost = 0.0;
  if (config->debug) {
//       cout << "CRAlgorithm::calculateManoeuvreObjective --> evaluating: " << vec.toString()<< endl;
    }
  
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  int index;
  vector<FlightPlan> fps = getFlightPlan(vec);
  for (unsigned int i = 0, cont_uav = 0; i < n_uavs; i++) {
    FlightPlan &fp = fps.at(i);
    if (conf.manoeuvre_selection) {
      if (controlled_UAVs.size() <= i || controlled_UAVs.at(i)) {
	index = getManouvreSelectionIndex(cont_uav);
	cont_uav++;
	
	switch (getManoeuvreType(vec.at(index))) {
	  case SPEED:
	    cost += abs(initial_plans.at(i).getETA(initial_plans.at(i).size() - 1) - fp.getETA(fp.size() - 1))*initial_plans.at(i).getCruiseSpeed();
// 	    cout << "CRAlgorithm::calculateManoeuvreObjective --> ETA difference: " << abs(initial_plans.at(i).getETA(initial_plans.at(i).size() - 1) - fp.getETA(fp.size() - 1))<< endl;
	    break;
	  case COURSE:
	    cost += fp.distance() - initial_plans.at(i).distance();
	    break;
	    
	  case LEVEL:
	    cost += (fp.distance() - initial_plans.at(i).distance()) * conf.speed_factor;
	    break;
	}
      }
    }  else {
      // For default, get the increment of the flight plan in meters
      cost += fp.distance() - initial_plans.at(i).distance();
    }
  }
  if (!FlightPlanChecker::checkFlightPlans(fps, sim)) {
    if (config->debug) {
//       cout << "CRAlgorithm::calculateManoeuvreObjective --> adding penalty: " << conf.collision_penalty << endl;
    }
    cost += conf.collision_penalty;
  }
  
  return cost;
}


double CRAlgorithm::ETAPenalty(const vector< FlightPlan >& fp) const
{
  double ret_val = 0.0;
  
  vector<double> ETA;
  
  for (unsigned int i = 0; i < fp.size(); i++) {
    ETA.push_back(fp.at(i).getETA(fp.at(i).size() - 1) - fp.at(i).getETA(0));
  }
  
  ret_val = functions::std_dev(ETA);
  
  
  return ret_val;
}


double CRAlgorithm::deltaSpeedPenalty(const vector<FlightPlan> &fp) const {
  double ret_val = 0.0;
  CostConfig &config = dynamic_cast<CostConfig &>(*this->config);
  
  for (unsigned int i = 0; i < fp.size(); i++) {
//     cout << "i = " << i << "\t control = " <<getControledUAVID(i) <<endl;
    const simulator::FlightPlan &curr_plan = fp.at(i);
    if (curr_plan.is4d_()) {
      // Get the previous speed and current speed
      double v_ant = (curr_plan.at(1).distance(curr_plan.at(0)))/(curr_plan.getETA(1) - curr_plan.getETA(0));
      for (int j = 1; j < curr_plan.size() - 1; j++) {
	double v = v_ant;
// 	if (curr_plan.getETA(j + 1) - curr_plan.getETA(j) > 1e-100) {
	  v = (curr_plan.at(j + 1).distance(curr_plan.at(j)))/(curr_plan.getETA(j + 1) - curr_plan.getETA(j));
	  ret_val += config.speed_factor * pow(v - v_ant, 2.0); // And then add the difference by three
// 	  cout << "v = " << v << " \t v_ant = " << v_ant << endl;
// 	}
// 	const vector<double> &param = sim->getParticle(getControledUAVID(i))->getModel()->getParameter();
// 	if (v < 0.0 || v > param.at(param.size() - 1) ) {
// 	  // Add penalty because of unfeasible plan_ok
// 	  ret_val += config.collision_penalty * 2.0;
// 	}
// 	
	
	v_ant = v;
      }
    }
  }
  
  return ret_val;
}

vector< FlightPlan > CRAlgorithm::getFlightPlan(const functions::RealVector& vec) const
{
  vector<FlightPlan> ret = initial_plans;
  CostConfig &config = dynamic_cast<CostConfig &>(*this->config);
  int cont_uav = 0;
  
  if (config.manoeuvre_selection) {
    return getManeouvreSelectionFlightPlan(vec);
  }
  
  if (config.modify_trajectory) {
    return getModifiedFlightPlan(vec);
  }
  
  for (int i = 0; i < n_uavs; i++) {
    if (controlled_UAVs.at(i)) {
      // The UAV is controlled so we have to update its plan
      FlightPlan plan;
      functions::FormattedTime t = config.init_time;
      if (config.time_exploration) {
	plan.push_back4d(ret.at(i).at(0), t);
      } else {
	plan.push_back(ret.at(i).at(0));
      }
    
    
      for (int j = 0; j < config.intermediate_waypoints; j++) {
	// Calculate the initial index in the vector
	int index = cont_uav * config.intermediate_waypoints * config.waypoint_dimension + j * config.waypoint_dimension;
	functions::Point3D new_way;
      
	if (config.waypoint_dimension == 2) {
	  new_way.init(vec.at(index), vec.at(index + 1), plan.at(0).z);
	} else if (config.waypoint_dimension == 3) {
	  new_way.init(vec.at(index), vec.at(index + 1), vec.at(index + 2));
	}
	if (config.time_exploration) {
	  double v, dist;
	  if (config.time_exploration_type == INDEPENDENT_VELOCITY ) {
	    // The following count is necessary in order to get the proper ETA included in the "genome" information
	    v = vec.at(vec.size() - (config.intermediate_waypoints + 1) * (howManyControlledUAVs() - cont_uav));
	    dist = plan.at(plan.size() - 1).distance2d(new_way);
	  } else {
	    // The following count is necessary in order to get the proper ETA included in the "genome" information
	    v = vec.at(vec.size() - (config.intermediate_waypoints) * (howManyControlledUAVs() - cont_uav));
	    dist = plan.at(plan.size() - 1).distance2d(new_way);
	  }
	  t = t + dist / v;
	  plan.push_back4d(new_way, t);
	} else {
	  plan.push_back(new_way);
	}
      }
      if (config.time_exploration) {
	if (config.time_exploration_type == INDEPENDENT_VELOCITY) {
	  double v = vec.at(vec.size() - (config.intermediate_waypoints + 1) * (howManyControlledUAVs() - cont_uav));
	  double dist = plan.at(plan.size() - 1).distance2d(initial_plans.at(i).at(1));
	  t = t + dist/v;
	  plan.push_back4d(ret.at(i).at(1), t);
	} else {
	  const vector<double> &param = sim->getParticle(getControledUAVID(i))->getModel()->getParameter();
	  const FlightPlan &ip = initial_plans.at(i);
	  FormattedTime desired_ETA = config.init_time + ip.distance()/ip.getCruiseSpeed();
	  double dis = ret.at(i).at(1).distance2d(plan.at(plan.size() - 1)); 
	  double v = dis / (desired_ETA - plan.getETA(plan.size() - 2));
	  v = saturate( v, param.at(param.size() - 2), param[param.size() - 1]);
	
	  FormattedTime real_ETA = plan.getETA(plan.size() - 1) + v * dis;
	  plan.push_back4d(ret.at(i).at(1), real_ETA);
	}
      } else {
	plan.push_back(ret.at(i).at(1)); // Note: TODO: It is assumed flight plan of longitude 1 when adding waypoints!!!!
      }
      
      // Once we have calculated the plan, change it in the proper place
      ret[i] = plan;
      // Actualize the controlled uav counter
      cont_uav++;
    }
  }
  
  return ret;
}

vector< FlightPlan > CRAlgorithm::getManeouvreSelectionFlightPlan(const RealVector& vec) const
{
  vector<FlightPlan> ret;
  CostConfig &config = dynamic_cast<CostConfig &>(*this->config);
  int cont_uav = 0;
  
  
  
  if (config.debug) {
//     cout << "CRAlgorithm::getManoeuvreSelectionFlightPlan --> Individual = " << vec.toString() << endl;
  }
  
  for (int i = 0; i < n_uavs; i++) {
    if (controlled_UAVs.at(i)) {
      // The UAV is controlled so we have to update its plan
      FlightPlan plan;
      functions::FormattedTime t = config.init_time;
      plan.push_back4d(initial_plans.at(i).at(0), t);
      plan.setCruiseSpeed(initial_plans.at(i).getCruiseSpeed());
      
      int index = getManouvreSelectionIndex(cont_uav); // initial index

      double course = initial_plans.at(i).at(0).getHeadingTo(initial_plans.at(i).at(1));
      
      Point3D inc_xy = (initial_plans.at(i).at(1) - initial_plans.at(i).at(0)) / (config.intermediate_waypoints + 1);
      inc_xy.z = 0.0;
      Point3D u_inc = inc_xy / inc_xy.norm();
      Point3D p_ant = plan.at(0);
      Point3D p; 
      double course_change;
      double min_speed = sim->getParticle(cont_uav)->getModel()->getMinSpeed();
      double max_speed = sim->getParticle(cont_uav)->getModel()->getMaxSpeed();
      
      for (int j = 0; j < config.intermediate_waypoints; j++) {
	// Calculate the initial index in the vector. Genome structre: [s1 w1 ... wn] [s2 w1 ... wn] ... [sm w1 .. wn]  (n = intermediate_waypoints, m = controlled uavs)
	double speed = plan.getCruiseSpeed();
	
	
	switch (getManoeuvreType(vec.at(getManouvreSelectionIndex(cont_uav)))) {
	  case SPEED:
	    // Speed manoeuvre
	    speed = min_speed + vec.at(index + j + 1) * (max_speed - min_speed);
	    p = p_ant + inc_xy;
	  
	    if (config.debug) {
// 	      cout << "CRAlgorithm::getManeouvreSelectionFlightPlan --> Speed manoeuvre. Speed = " << speed <<  endl;
	    }
	    break;
	  case COURSE:
	    // Course manoeuvre 
	    course_change = (vec.at(index + j + 1) - 0.5) * config.max_course * 2.0;
	    course += course_change;
	  
// 	    if (config.debug) {
// 	      cout << "CRAlgorithm::getManeouvreSelectionFlightPlan --> Course manoeuvre. Course = " << course << ". Course change = " << course_change <<  endl;
// 	    }
	    p.x = cos(course);
	    p.y = sin(course);
	    p.z = 0.0;
	    p = p_ant + p * initial_plans.at(i).distance() / (cos(course_change) * (config.intermediate_waypoints + 1));
	    if (config.debug) {
// 	      cout << "Course maneuver. New way: " << p.toString() << "\t pant = " << inc_xy.toString() << endl;
	    }
	    break;
	  default:
	    if (config.debug) {
// 	      cout << "CRAlgorithm::getManeouvreSelectionFlightPlan --> Altitude manoeuvre. Altitude = ";
	    }
	    
	    // Altitude manoeuvre
	    p = p_ant + inc_xy;
	    p.z = config.min_z + vec.at(index + j + 1) * (config.max_z - config.min_z);
	    if (config.debug) {
// 	      cout << p.z << "max_z = " << config.max_z << " value = " << vec.at(index+j+1) << endl;
	    }
	}
	
	t = t + (p.distance2d(p_ant)) / speed;
	plan.push_back4d(p, t);
	p_ant = p;
      }
      // Then consider the last waypoint
      p = initial_plans.at(i).at(initial_plans.at(i).size() - 1); // The last waypoint is the same as the original last waypoint
      t = t + (p.distance2d(p_ant)) / initial_plans.at(i).getCruiseSpeed();
      plan.push_back4d(p, t);
      // Add the finished plan to the returned vector
      cont_uav++;
      ret.push_back(plan);
    }
    
  }
  
  return ret;
}


vector< FlightPlan > CRAlgorithm::getModifiedFlightPlan(const functions::RealVector& vec) const
{
  vector<FlightPlan> ret = initial_plans;
  CostConfig &config = dynamic_cast<CostConfig &>(*this->config);
  int cont_uav = 0;
  int iw = 0;
  for (int i = 0; i < n_uavs; i++) {
    if (controlled_UAVs.at(i)) {
      // The UAV is controlled so we have to update its plan
      FlightPlan plan;
      if (!config.time_exploration) {
	plan.push_back(ret.at(i).at(0));
      } else {
	plan.push_back4d(ret.at(i).at(0), config.init_time);
      }
      functions::FormattedTime t = config.init_time;

      FlightPlan init_plan = initial_plans.at(i);
      const int &dim = config.waypoint_dimension;
      int index;
      for (int j = 1; j < init_plan.size(); j++) {
	index = iw * dim + (j - 1) * dim;
	functions::Point3D new_way(initial_plans.at(i).at(j));
	
	if (config.waypoint_dimension == 2 && j < init_plan.size() - 1) {
	  // Makes a sample in a square centered in the original waypoint with size defined by the bounds
	  Point3D aux(vec.at(index), vec.at(index + 1), 0.0);
	  new_way = new_way + aux;
	} else if (config.waypoint_dimension == 3) {
	  Point3D aux(vec.at(index), vec.at(index + 1), vec.at(index + 2));
	  new_way = new_way + aux;
	}
	
	if (config.time_exploration) {
	  double dist = plan.at(j - 1).distance(new_way);
	  if (j != init_plan.size() - 1 || config.time_exploration_type == INDEPENDENT_VELOCITY) {
	    // The following calculation is necessary in order to get the proper ETA included in the "genome" information
	    double v = vec.at(speed_to_index.at(i).at(j));
	    t = t + dist/v;
	  } else {
	    double delta_t = init_plan.getETA(init_plan.size() - 1) - t;
	    
	    // Calculate the speed necessary to meet the ETA
	    double speed = functions::minimum(dist / delta_t, sim->getParticle(i)->getModel()->getMaxSpeed()) ;
	    if (speed < 0.0) {
	      speed = sim->getParticle(i)->getModel()->getMaxSpeed();
	    }
	    t = t + dist / speed;
	  }
	  plan.push_back4d(new_way, t);
	} else {
	  plan.push_back(new_way);
	}
      }
      
      
      // Once we have calculated the plan, change it in the proper place
      ret[i] = plan;
      // Actualize the controlled uav counter
      cont_uav++;
      // Actualize the intermediate waypoints so far
      iw += initial_plans.at(i).size() - 2; // ????
    }
  }
  
  return ret;
}



bool CRAlgorithm::simulateSystem(vector< FlightPlan > fp, bool& colli)
{
	return sim->run(colli, fp);
}

void CRAlgorithm::prepareModifiedBounds(std::vector<double> &upper, std::vector<double> &lower, std::vector<double> &speed)
{
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  vector<double> n_upper, n_lower, n_speed;
  
  calculateSpeedMap(initial_plans);
  
  for (int i = 0; i < sim->howManyUAVs(); i++) {
    if (controlled_UAVs.size() == 0 ||  controlled_UAVs.at(i)) {
      Particle *p = sim->getParticle(i); // Get the current particle
      
      const FlightPlan &i_plan = initial_plans.at(i);
      int int_ways = i_plan.size() - 2;
      int dim = conf.waypoint_dimension;
      int cont_uav = 0;
      int old_index;
      
      // First the geometric bounds
      for (int j = 0; j < i_plan.size(); j++) {
	old_index = cont_uav * dim;
	if (controlled_UAVs[i]) {
	  for (int k = 0; k < dim; k++) {
	    n_upper.push_back(upper.at(k));
	    n_lower.push_back(lower.at(k));
	    n_speed.push_back(speed.at(k));
	  }
	}
      }
      
      // Then the speed bounds
      
      for (int j = 1; j < i_plan.size(); j++) {
	
	if (j == i_plan.size() - 1 && conf.time_exploration_type == MANTAIN_ETA) {
	  continue;
	}
	if (controlled_UAVs[i]) {
	  if (j == 1 || speed_to_index.at(i).at(j) != speed_to_index.at(i).at(j - 1) ) {
	    n_upper.push_back(p->getModel()->getMaxSpeed());
	    n_lower.push_back(p->getModel()->getMinSpeed());
	    n_speed.push_back((p->getModel()->getMaxSpeed() - p->getModel()->getMinSpeed()) / 10.0);
	  }
	}
      }
    }
  }
  upper = n_upper;
  lower = n_lower;
  speed = n_speed;
  
  if (conf.debug) {
    cout << "Modified bounds: \n Upper:" << functions::printVector(n_upper) << endl;
    cout << "Lower:" << functions::printVector(n_lower) << endl;
    cout << "Speed:" << functions::printVector(n_speed) << endl;
  }
}

void CRAlgorithm::prepareTimeBounds(std::vector<double> &upper, std::vector<double> &lower, std::vector<double> &speed)
{
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  for (int i = 0; i < sim->howManyUAVs(); i++) {
    if (controlled_UAVs.size() == 0 ||  controlled_UAVs.at(i)) {
      Particle *p = sim->getParticle(i); // Get the current particle
      for (int i = 0; i < conf.intermediate_waypoints + 1; i++) {
	upper.push_back(p->getModel()->getMaxSpeed());
	lower.push_back(p->getModel()->getMinSpeed());
	speed.push_back((p->getModel()->getMaxSpeed() - p->getModel()->getMinSpeed()) / 5.0);
      }
    }
  }
}

void CRAlgorithm::prepareMaintainETABounds(std::vector< double >& upper, std::vector< double >& lower, std::vector< double >& speed)
{
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  for (int i = 0; i < sim->howManyUAVs(); i++) {
    if (controlled_UAVs.size() == 0 ||  controlled_UAVs.at(i)) {
      Particle *p = sim->getParticle(i); // Get the current particle
      for (int i = 0; i < conf.intermediate_waypoints; i++) {
	upper.push_back(p->getModel()->getMaxSpeed());
	lower.push_back(p->getModel()->getMinSpeed());
	speed.push_back((p->getModel()->getMaxSpeed() - p->getModel()->getMinSpeed()) / 5.0);
      }
    }
  }
}

void CRAlgorithm::prepareMSPSOBounds(std::vector<double> &upper, std::vector<double> &lower, std::vector<double> &speed)
{
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  upper.clear();
  lower.clear();
  speed.clear();
  
  for (int i = 0; i < getProblemDimension(); i++) {
    upper.push_back(1);
    lower.push_back(0);
    speed.push_back(0.1);
  }
}


void CRAlgorithm::calculateSpeedMap(const vector<FlightPlan> &fp) {
  int curr_index = 0;
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  // Calculate the index related to the changes in trajectory
  for (unsigned int i = 0; i< fp.size(); i++) {
    curr_index += (fp.at(i).size() - 2) * conf.waypoint_dimension;
  }
  
  // The next index belong to speed items
  for (unsigned int i = 0;i < fp.size(); i++) {
    const FlightPlan &cp = fp.at(i);
    double curr_dist = 0.0;
    map<int, int> wp_to_gen;
    
    wp_to_gen.insert(make_pair(0, curr_index));
    
    // If we want to meet the ETA, at last the last waypoint has to be dependent
    int last =  (conf.time_exploration_type == INDEPENDENT_VELOCITY) ? cp.size() : cp.size() - 1;
    
    for (int j = 1;j < last; j++) {
      curr_dist += cp.at(j).distance(cp.at(j - 1));
      
      if (curr_dist > conf.min_control_dist) {
	curr_dist = 0.0;
	wp_to_gen.insert(make_pair(j, curr_index));
	curr_index++;
      }
    }
    speed_to_index.push_back(wp_to_gen);
    
    
  }
  
  if (conf.debug) {
    cout << "Speed to Index: " << endl;
    for (unsigned int i = 0; i < speed_to_index.size(); i++) {
      cout << i << "\t";
      for (unsigned int j = 0; j < speed_to_index.at(i).size(); j++) {
	
	cout << "(" << j << ", " << speed_to_index.at(i)[j] << ") ";
      }
      cout << endl;
    }
  }
}

double CRAlgorithm::calculateNoSimObjective(const vector< FlightPlan >& fp)
{
  double ret_val = 0.0;
  CostConfig &conf= dynamic_cast<CostConfig&>(*(this->config));
  
  if (!FlightPlanChecker::checkFlightPlans(fp, sim)) {
    ret_val += conf.collision_penalty;
  }
  
  for (int i = 0; i < n_uavs; i++) {
    if (controlled_UAVs[i]) {
      ret_val += fp[i].distance();
    }
  }
  
  if (conf.time_exploration) {
//     ret_val += deltaSpeedPenalty(fp);
    ret_val += conf.speed_factor * ETAPenalty(fp);
  }
  
  return ret_val;
}

double CRAlgorithm::testObjective(const RealVector& v)
{
  return calculateObjective(getFlightPlan(v));
}

ManoeuvreType CRAlgorithm::getManoeuvreType(double genome_value) const
{
  CostConfig &config= dynamic_cast<CostConfig&>(*(this->config));
  const double first = (config.waypoint_dimension == 3)?0.333333:0.5;
  const double second = (config.waypoint_dimension == 3)?0.666667:1.1;

  ManoeuvreType ret = LEVEL;
  if (genome_value < first) {
    ret = SPEED;
  } else if (genome_value < second) {
    ret = COURSE;
  } 
  return ret;
}

void CRAlgorithm::getBounds()
{
  CostConfig &config= dynamic_cast<CostConfig&>(*(this->config));
  
  vector<double> new_upper, new_lower, new_speed;
  
  if (config.intermediate_waypoints > 1 && config.upper_bounds.size() != getProblemDimensionWithoutTimeExploration() && config.upper_bounds.size() == getProblemDimensionOneIntermediate()) {
    
    // Copy the bounds for each intermediate waypoint
    for (unsigned int j = 0; j < n_uavs; j++) {
      int index = j * config.waypoint_dimension;
      for (unsigned int i = 0; i < config.intermediate_waypoints; i++) {
	for (unsigned int k = 0; k < config.waypoint_dimension; k++) {
	  new_upper.push_back(config.upper_bounds.at(index + k));
	  new_lower.push_back(config.lower_bounds.at(index + k));
	  new_speed.push_back(config.speed_bounds.at(index + k));
	}
      }
    }
    config.upper_bounds = new_upper;
    config.lower_bounds = new_lower;
    config.speed_bounds = new_speed;
  }
  
  if (config.manoeuvre_selection) {
	prepareMSPSOBounds(config.upper_bounds, config.lower_bounds, config.speed_bounds);
      } else if (config.modify_trajectory) {
	prepareModifiedBounds(config.upper_bounds, config.lower_bounds, config.speed_bounds);
      } else if(config.time_exploration) {
	switch (config.time_exploration_type) {
	  case INDEPENDENT_VELOCITY:
	    prepareTimeBounds(config.upper_bounds, config.lower_bounds, config.speed_bounds);
	    break;
	  case MANTAIN_ETA:
	    prepareMaintainETABounds(config.upper_bounds, config.lower_bounds, config.speed_bounds);
	    break;
	}
      }
  
}

int CRAlgorithm::getProblemDimensionWithoutTimeExploration() const {
  CostConfig &config = dynamic_cast<CostConfig&>(* (this->config));
  
  return howManyControlledUAVs() * config.waypoint_dimension * config.intermediate_waypoints;
}

int CRAlgorithm::getProblemDimensionOneIntermediate() const {
  CostConfig &config = dynamic_cast<CostConfig&>(* (this->config));
  
  return howManyControlledUAVs() * config.waypoint_dimension;
}

int CRAlgorithm::getProblemDimension() const
{
  CostConfig &config = dynamic_cast<CostConfig&>(* (this->config));
  
  int ret;
  
  if (config.manoeuvre_selection) {
    ret = howManyControlledUAVs() * (config.intermediate_waypoints + 1);
  } else {
    ret = getProblemDimensionWithoutTimeExploration();
    if (config.time_exploration) {
      ret += howManyControlledUAVs() * (config.intermediate_waypoints + 1);
    }
  }
  
  return ret;
}

bool CRAlgorithm::checkProblem() const
{
  bool ret = true;
  CostConfig &config = dynamic_cast<CostConfig&>(*this->config);
  
  int problem_dim = getProblemDimensionWithoutTimeExploration();
  int pd = getProblemDimensionOneIntermediate();
  if (config.upper_bounds.size() < problem_dim || 
      config.lower_bounds.size() < problem_dim ||
      config.speed_bounds.size() < problem_dim) {
    
    if (config.upper_bounds.size() < pd || 
      config.lower_bounds.size() < pd ||
      config.speed_bounds.size() < pd) {
      ret = false;
    }
  }
  
  return ret;
}

void CRAlgorithm::updateDeltaETA(EvolutionData& current) const
{
  current.sum_delta_ETA = 0.0;
  CostConfig &config = dynamic_cast<CostConfig&>(*(this->config));
  if (config.time_exploration) {
    for (unsigned int i = 0; i < current.plan.size(); i++) {
      FlightPlan &p = current.plan.at(i);
      current.sum_delta_ETA += fabs(p.getETA(p.size() - 1) - p.getETA(0));
      
    }
    current.sum_delta_ETA = current.sum_delta_ETA / (double)howManyControlledUAVs();
  }
  if (config.time_exploration && config.time_exploration_type == MANTAIN_ETA) {
    for (unsigned int i = 0; i < current.plan.size(); i++) {
      FlightPlan &p = current.plan.at(i);
      for (unsigned int j = 1; j < p.size(); j++) {
	double v = ( p.at(j).distance(p.at(j - 1)) )/(p.getETA(j) - p.getETA(j - 1));
	current.speed_in_sector.push_back(v);
      }
    }
  }

  if (config.time_exploration) {
    for (unsigned int i = 0; i < current.plan.size(); i++) {
      const FlightPlan &p = current.plan.at(i);
      const FlightPlan &ip = initial_plans.at(i);
      current.sum_delta_ETA += fabs(p.getETA(p.size() - 1) - p.getETA(0) - ip.distance() / ip.getCruiseSpeed() );
    }
    current.sum_delta_ETA = current.sum_delta_ETA / (double)howManyControlledUAVs();
  }
      
}


}