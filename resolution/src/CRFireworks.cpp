/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2015  sinosuke <email>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "CRFireworks.h"
#include "FireworksConfig.h"
#include "functions/RandomNumberGenerator.h"
#include "functions/functions.h"

using functions::RealVector;
using functions::RandomNumberGenerator;

namespace resolution {

CRFireworks::CRFireworks():CRAlgorithm()
{
  pointersToNULL();
}

CRFireworks* CRFireworks::clone() const
{

}

void CRFireworks::init(ParseBlock& block, AlgorithmConfig* conf)
{
  resolution::CRAlgorithm::init(block, conf);
}

string CRFireworks::toString() const
{
  ostringstream os;
  
  os << "CRFireworks." << endl;
  
  return os.str();
}


CRAlgorithm* CRFireworks::createFromBlock(ParseBlock& block) const
{
  CRAlgorithm* ret = new CRFireworks();
  Checker *check = getAlgorithmChecker();
  
  try {
    block.checkUsing(check);
    FireworksConfig f_conf;
    AlgorithmConfig *conf = f_conf.createAlgorithmConfig(block["config"]);
    ret->init(block, conf);
  } catch (exception &e) {
    cerr << "CRGenetics::createFromBlock --> error while loading data from block\n";
    delete check;
    throw(e);
  }
  
  delete check;
  
  return ret;
}

CRAlgorithmStatistics CRFireworks::execute()
{
  CRAlgorithmStatistics stat;
  FireworksConfig &c = dynamic_cast<FireworksConfig&>(* (this->config));
  int i, j , k;
  
  dimension = getProblemDimension();
  
  solved = false;
  msg = 0;
  evaluations = 0;
  
  // Some checks in order to make sure that the parameters are correct
  
  if (!checkProblem() && !c.manoeuvre_selection) {
    cerr << "CRFireworks::run --> Errors while checking the integrity of the problem. Aborting.\n";
    stat.setError(true);
    return stat;
  }
  
  struct timeval t1, t2;
  gettimeofday(&t1, NULL);

  // Make the MonteCarlo simulation obtaining the mean trajectories and the stdDev of the particles set
  // Make a first simulation to check if there exist some collisions in the system
  if (c.debug) {
    cout << "CRFireworks::run --> Simulating system in order to check for collisions...\n";
  }
  
  bool collision;
  float min_distance;
  if ( sim->run(collision) ) { // Simulating the whole system saving trajectory.
    if (!collision) {
      cout << "CRFireworks::run --> No collisions found. It is not necessary to run the PS algorithm.\n";
      stat.setSolved(false);
      stat.setCollisionDetected(false);
    } else {
      stat.setCollisionDetected(true);
      cout << "CRAlgorithm::run --> Collisions found, initiating the CR Algorithm\n";
  
      buffer = 0;
    
      allocateMemory();
    
      int &n = c.population;
    
      // If 4d plan is selected, we have to prepare the bounds
      CRAlgorithm::getBounds();
      if (c.debug) {
	cout << "CRFireworks::execute() --> upper_bounds = " << functions::printVector(c.upper_bounds) << endl;
	cout << "CRFireworks::execute() --> lower_bounds = " << functions::printVector(c.lower_bounds) << endl;
      }
    
      // Initialize population (TODO: select the correct initialization)
      double min,max,sum;
      int min_pos;
      min = INF;
      for (i = 0;i < n;i++)
      {
	for (j = 0;j < dimension;j++)
	{
	  fireworks[i][j] = RandomNumberGenerator::getRandomNumber(c.lower_bounds.at(j), c.upper_bounds.at(j));
	}
	evaluations++;
	fitnesses[i] = calculateObjectiveVec(RealVector(fireworks[i], dimension));
	if (c.debug) {
	  cout << "CRFireworks::execute --> Initial position " << i;
	  cout << ". Position: " << RealVector(fireworks[i], dimension).toString() << endl;
	}
	if (fitnesses[i] < min) {
	  min = fitnesses[i];
	  min_pos = i;
	}
      }
      
      // The evolution begins
      
      int counter,idx;
      int nn,count,index;
      int iter;
      // For saving relevant evolution data
      vector<EvolutionData> ev_data;
      RealVector curr_best = RealVector(fireworks[min_pos], dimension);
      bool end_time = false;
      for (iter = 1;iter < c.generations && !end_time;iter++)
      {
	EvolutionData current;
	
	current.cost = min;
	gettimeofday(&t2, NULL);
	current.t = functions::calculateLapseTime(t1, t2);
	current.plan = getFlightPlan(curr_best);
	updateDeltaETA(current);
	ev_data.push_back(current);
	
	// Getting the minimum and maximum
	min = max = fitnesses[0];
	for(i = 1;i < n;i++)
	{
	  if(min > fitnesses[i])
	  {
	    min = fitnesses[i];				
	  }
	  
	  if (max < fitnesses[i])
	  {
	    max = fitnesses[i];
	  }
	}

	sum = 0;
	for (i = 0;i < n;i++)
	{
	  sum += (max - fitnesses[i]);
	}

	
	// Calculating the number of sparks for each value
	counter = n + c.mm;
	for(i = 0;i < n;i++)
	{
	  nums[i] = (int)(c.m * ((max - fitnesses[i] + EPS)/(sum+EPS)));
	  nums[i] = functions::saturate(nums[i], c.am, c.bm);
	  counter += nums[i];
	}

	sum = 0;
	for (i = 0;i < n;i++)
	{
	  sum += (fitnesses[i] - min);
	}

	// As[i] is the amplitude of the explosion (for each coordinate)
	for(i = 0;i < n;i++)
	{
	  for (j = 0; j < dimension; j++) {
	    As[i][j] = (c.upper_bounds.at(j) + c.lower_bounds.at(j)) * 0.5 * ((fitnesses[i] - min + EPS)/(sum + EPS));
	    As[i][j] *= c.amplitude_mult;
	    
	    // Minimum amplitude check
	    As[i][j] = functions::maximum(As[i][j], c.A_min);
	  }
	}

	// Copy the current population (fireworks) to the buffer (positions)
	for (idx = 0;idx < n;idx++)
	{
	  memcpy(positions[idx], fireworks[idx], sizeof(double) * dimension);			
	  vals[idx] = fitnesses[idx];
	}
	
	// The fireworks start. Create mm explosions
	for (i = 0;i < c.mm;i++)  {
	  memset(flags,0,sizeof(int)*dimension);			
	  nn = RandomNumberGenerator::getRandomNumber(0, dimension)+1;
	  count = 0;
	  while (count < nn)
	  {
	    index = RandomNumberGenerator::getRandomNumber(0,dimension);
	    if (flags[index] != 1)
	    {
	      flags[index] = 1;
	      count++;
	    }
	  }

	  // Select an individual at random
	  int id = RandomNumberGenerator::getRandomNumber(0,n);
	  // Select the magnitude of the explosion (always the same distribution?)
	  double g = RandomNumberGenerator::randomGaussian(1.0, 1.0);
	  
	  // Do the explosion in the selected dimensions
	  for (j = 0;j < dimension;j++)
	  {
	    if (flags[j] == 1)
	    {
	      double &LBOUND = c.lower_bounds.at(j);
	      double &UBOUND = c.upper_bounds.at(j);
	      positions[idx][j] = fireworks[id][j]*g;
	      
	      // Correct the bounds
	      if (c.new_mapping) {
		positions[idx][j] = correct_bounds_EFWA(positions[idx][j], LBOUND, UBOUND);
	      } else {
		correct_bounds_FWA(positions[idx][j], LBOUND, UBOUND);
	      }
	    } else {
	      // Do not modify this dimension
	      positions[idx][j] = fireworks[id][j];
	    }
	  }
	  
	  // Evaluate the new particles
	  evaluations++;
	  vals[idx] = calculateObjectiveVec(RealVector(positions[idx],dimension));
	  idx++;
	}
	
	for (i = 0;i < n;i++) {
	  for (k = 0;k < nums[i];k++,idx++)
	  {
	    // Get the dimensions to modify
	    memset(flags,0,sizeof(int) * dimension);
	    nn = RandomNumberGenerator::getRandomNumber(0.0, dimension) + 1;
	    count = 0;
	    while (count < nn)
	    {
	      index = RandomNumberGenerator::getRandomNumber(0.0, dimension);
	      if (flags[index] != 1)
	      {
		flags[index] = 1;
		count++;
	      }
	    }

	    double h;
	    for (j = 0;j < dimension;j++) {
	      if (flags[j] == 1)
	      {
		h =  As[i][j] * RandomNumberGenerator::getRandomNumber(-1.0, 1.0);
		double &LBOUND = c.lower_bounds.at(j);
		double &UBOUND = c.upper_bounds.at(j);
		
		positions[idx][j] = fireworks[i][j] + h;
		
		// Correct the bounds
		if (c.new_mapping) {
		  positions[idx][j] = correct_bounds_EFWA(positions[idx][j], LBOUND, UBOUND);
		} else {
		  correct_bounds_FWA(positions[idx][j], LBOUND, UBOUND);
		}
	      } else  {
		positions[idx][j] = fireworks[i][j];
	      }
	    }
	    evaluations++;
	    vals[idx] = calculateObjectiveVec(RealVector(positions[idx], dimension));
	  }
	}

	// Selection operator calls
	if (c.efwa_selection) {
	  EFWASelection(counter);
	} else {
	  FWASelection(counter);
	}
	  
	
	// Actualize curr_best ( the selection algorithm must be elistist and assure that the best individual
	// is in the first position of the vector)
	curr_best = RealVector(fireworks[0], dimension);
	min = fitnesses[0];
	
	// See if the time has expired
	gettimeofday(&t2, NULL);
	if (c.max_time > 0.0 && functions::calculateLapseTime(t1, t2) > c.max_time) {
	  end_time = true;
	}
      } // end of the for of the iterations

      // Get the results of the last iteration
      EvolutionData current;
	
      current.cost = min;
      gettimeofday(&t2, NULL);
      current.t = functions::calculateLapseTime(t1, t2);
      current.plan = getFlightPlan(curr_best);
      updateDeltaETA(current);
      ev_data.push_back(current);
      
      
      double rs = fitnesses[0];

//       if (x != NULL)
//       {
// 	memcpy(x,fireworks[0],sizeof(double)*dimension);		
//       }
      // Generate statistics
      if (c.export_evolution) {
	if (saveEvolutionData(c.evolution_file, ev_data)) {
	  if (c.debug) {
	    cout << "CRFireworks::run --> Evolution saved.\n";
	  } 
	} else {
	  cerr << "CRFireworks::run --> Could not save evolution data.\n";
	}
      }
      if (c.export_all_evolution) {
	// TODO:
// 	algorithm.exportEvolution(c.evolution_file);
      }
      stat.setEvolutionData(ev_data);
      
//       cout << algorithm.toString() << endl;
      // Calculate spended time and show it
//       gettimeofday(&t2,NULL);
      cout << "CRFireworks::run --> Spended time = " << functions::showTime(t1,t2) << endl;
      cout << "Minimum cost: " << rs << "\t Evaluations: " << evaluations << endl;
      stat.setEvaluations(evaluations);
      stat.setExecutionTime(functions::calculateLapseTime(t1, t2));
      stat.setMinObjetive(rs);

      // Get the minimum objetive value and store it for statistical purposes...
      // Show the best flight plan
      sim->setCollisionStop(false);
      bool collision, ok;
      
      vector<double> aux(RealVector(fireworks[0], dimension));
      vector<simulator::FlightPlan> fp = getFlightPlan(aux);
      
      if (c.best_individual.size() > 0) {
	functions::writeStringToFile(c.best_individual, functions::printVector(aux));
      }
      
      // Contract the geometries
      sim->expandGeometries(-c.geometry_expansion);

      try {
	      ok = simulateSystem(fp, collision);
      } catch(...) {
	      ok = false;
      }
      
      if (c.export_solution) {
	      if (exportSolution(c.solution_filename, fp)) {
		      cout << "Solution exported successfully.\n";
	      }
      }
      if (c.export_trajectories) {
	if (sim->exportTrajectory(c.trajectory_filename)) {
	  cout << "Trajectories exported successfully.\n";
	}
      }

      
      if (collision || !ok) {
	if (!ok) {
	  cerr << "CRFireworks::run --> Error while simulating the best flight plan\n";
	}
	stat.setSolved(false);
	cout  << "CRFireworks::run --> The problem was not solved.\n"; // Min distance = " << min_d << "\n";
      } else {
	cout  << "CRFireworks::run --> The problem was solved successfully.\n";// Min distance = " << min_d << "\n";
	stat.setSolved(true);
      }
      
      if (c.export_catec != "") {
	cout << "CRFireworks::execute --> Exporting the flight plan to " << c.export_catec << endl;
	ostringstream os;
	for (unsigned int i = 0; i < fp.size(); i++) {
	  
	  os << fp.at(i).toCatec(i) << endl;
	}
	
	if (!functions::writeStringToFile(c.export_catec, os.str())) {
	  cerr << "CRFireworks::execute --> could not export the plan in Catec format. " << endl;
	}
      }
      
      cout << endl << endl;
    }
  } else {
    cerr << "CRFireworks::run --> An error was found while simulating the system. Aborting.\n";
    stat.setError(true);
  }
  
  return stat;
}


CRAlgorithmStatistics CRFireworks::execute_one_vs_all(vector< vector< RealVector > >& traj, bool initialize)
{
  CRAlgorithmStatistics stat;
  
  return stat;
}

ParseBlock* CRFireworks::toBlock() const
{
  ParseBlock* ret = CRAlgorithm::toBlock();
  
  ret->setProperty("algorithm", getType());
  
  return ret;
}

void CRFireworks::pointersToNULL()
{
    resolution::CRAlgorithm::pointersToNULL();
    result = NULL;
    fireworks = NULL;
    fitnesses = NULL;
    positions = NULL;
    vals = NULL;
    Rs = NULL;
    As = NULL;
    nums = NULL;
    flags = NULL;
}

void CRFireworks::allocateMemory()
{
  FireworksConfig &c = dynamic_cast<FireworksConfig&>(* (this->config));
  int &n = c.population;
  
  //allocate memory
  fireworks = (double **)calloc(n, sizeof(double*));
  As = (double **)calloc(n,sizeof(double*));
  int i,j,k;
  for (i = 0;i < n;i++)
  {
    fireworks[i] = (double *)calloc(dimension,sizeof(double));
    As[i] = (double *)calloc(dimension,sizeof(double));
  }
  fitnesses = (double *)calloc(n, sizeof(double));

  buffer = (n + c.mm + n * c.bm);
  positions = (double **)calloc(buffer, sizeof(double*));	
  for (i = 0; i < buffer; i++)
  {
    positions[i] = (double*)calloc(dimension, sizeof(double));
  }
  vals = (double *)calloc(buffer,sizeof(double));
  
  nums = (int *)calloc(n,sizeof(int));
  flags = (int*)calloc(dimension,sizeof(int));
  Rs = (double *)calloc(buffer,sizeof(double));	
}

void CRFireworks::freeMemory()
{
  if (this->config == NULL) {
    return;
  }
  FireworksConfig &c = dynamic_cast<FireworksConfig&>(* (this->config));
  int &n = c.population;
  int i;
  if (fireworks != NULL) {
    //free memory
    for (i = 0;i < n;i++)
    {
      free(fireworks[i]);
      free(As[i]);
    }
    free(fireworks);
    free(As);
    
    free(fitnesses);
    for (i = 0;i < buffer && positions != NULL;i++)
    {
	    free(positions[i]);
    }
    free(positions);
    free(vals);
    
    free(nums);
    free(flags);
    free(Rs);
  }
  pointersToNULL();
}

CRFireworks::~CRFireworks()
{
  freeMemory();
}

double CRFireworks::correct_bounds_FWA(double position, double LBOUND, double UBOUND) const
{
  if (position < LBOUND)
  {
    double tt = abs(position);
    while(tt > 0)
    {
      tt -= (UBOUND-LBOUND);
    }
    tt += (UBOUND-LBOUND);
    position = LBOUND + tt;
  } else if (position>UBOUND) {						
    double tt = abs(position);
    while(tt > 0)
    {
      tt -= (UBOUND-LBOUND);
    }
    tt += (UBOUND-LBOUND);
    position = LBOUND + tt;
  }
  return position;
}

double CRFireworks::correct_bounds_EFWA(double position, double LBOUND, double UBOUND) const
{
  if (position < LBOUND || position > UBOUND)
  {
    position = LBOUND + RandomNumberGenerator::getRandomNumber() * (UBOUND - LBOUND);
  }
  return position;
}

void CRFireworks::FWASelection(int population)
{
  int index;
  double minima = getMinima(index, population);
  int i,j,k;
  FireworksConfig &c = dynamic_cast<FireworksConfig&>(* (this->config));
  
  // Elitism
  fitnesses[0] = minima;
  memcpy(fireworks[0],positions[index],sizeof(double) * dimension);	

  // Get the weights for the Roulette Wheel taking into account the coverage of the search space
  double ss = 0;
  for (i = 0;i < population;i++)
  {
      // This is for more coverage (original FWA)
    for (j = 0;j < population;j++)
    {
      double tem = 0.0;
      
      for(k = 0;k < dimension;k++)
      {
	tem += (positions[i][k] - positions[j][k])*(positions[i][k] - positions[j][k]);
      }
      Rs[i] += sqrt(tem);
    }
    ss += Rs[i];
  }

  for (i = 0;i < population;i++)
  {
    Rs[i] = Rs[i]/ss;			
  }

  // Select the individuals of the next generation by roulette wheel
  for (i = 1;i < c.population;i++)
  {
    double r = RandomNumberGenerator::getRandomNumber();
    int id = 0;
    double s = 0;
    for (id = 0;id < population;id++)
    {
      s += Rs[id];
      if (s > r)
      {
	break;
      }
    }
    fitnesses[i] = vals[id];
    memcpy(fireworks[i],positions[id],sizeof(double) * dimension);
  }
}

void CRFireworks::EFWASelection(int population)
{
  int index;
  int i, j, k;
  double minima = getMinima(index, population);
  FireworksConfig &c = dynamic_cast<FireworksConfig&>(* (this->config));
  
  // Elitist: always gets the minimum
  fitnesses[0] = minima;
  memcpy(fireworks[0],positions[index],sizeof(double) * dimension);	

  // Get the weights for the Roulette Wheel
  double ss = 0;
  for (i = 0;i < population;i++)
  {
    if (fitnesses[i] - minima > 0.01) {
      Rs[i] = 1/(fitnesses[i] - minima + EPS);
    } else{
      Rs[i] = 0.000001;
    }
    // Till here
    
    ss += Rs[i];
  }

  for (i = 0;i < population;i++)
  {
    Rs[i] = Rs[i]/ss;			
  }

  // Select the individuals of the next generation by roulette wheel
  for (i = 1;i < c.population;i++)
  {
    double r = RandomNumberGenerator::getRandomNumber();
    int id = 0;
    double s = 0;
    for (id = 0;id < population;id++)
    {
      s += Rs[id];
      if (s > r)
      {
	break;
      }
    }
    fitnesses[i] = vals[id];
    memcpy(fireworks[i],positions[id],sizeof(double) * dimension);
  }
}


double CRFireworks::getMinima(int& index, int counter)
{
  //find the minima
  int i;
  index = 0;
  double minima = vals[0];
  for(i = 1;i < counter;i++)
  {
    if(minima > vals[i])
    {
      minima = vals[i];
      index = i;
    }
  }
  return minima;
}


}