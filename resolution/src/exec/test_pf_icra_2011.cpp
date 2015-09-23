#include "../CRAlgorithmFactory.h"

#include <iostream>
#include <vector>
#include <CRAlgorithm.h>
#include <functions/ArgumentData.h>
#include <functions/FormattedTime.h>
#include <functions/functions.h>
#include <simulator/Simulator.h>
#include <simulator/ParticleFilter.h>

using namespace resolution;
using namespace std;
using namespace functions;
using namespace simulator;

/*
 * This application does the following:
 * 
  1. Application intitialization
  2. Simulation of the uncontrolled vehicles (2-N)
  3. Monte-Carlo simulation of UAV 1 (init: 1 candidate (start->goal), afterwards, N candidates). Get the max_dev
  4. Collision detection (1 vs all) (with radius UAV1 + max_dev)
  5.------- if collision detected
    6. genetic algorithm call (includes Collision Detection with maxdev of the previous) (obtains candidates)
    7. Go to 2.
  ------- else
  END. 
  
  */

struct results {
  vector<double> montecarlo;
  vector<double> ga;
  double time;
  vector<double> cost;
  vector<int> collision;
  uint id;
  
  string toString() {
    ostringstream os, end_name;
    end_name << "{" << id << "}";
    os << vectorToMatlabString("montecarlo" + end_name.str(),  montecarlo) << endl;
    os << vectorToMatlabString("ga" + end_name.str(),  ga) << endl;
    os << vectorToMatlabString("cost" + end_name.str(),  cost) << endl;
    os << vectorToMatlabString("collision" + end_name.str(),  collision) << endl;
    os << "time" << end_name.str() << " = " << time << endl;
    
    
    return os.str();
  }
};

results solveProblem(ArgumentData &arg, int max_cont, int id, int particles);

int main(int argc, char **argv) {
  if (argc < 3) {
    cerr << "Use: " << argv[0] << " <gen_data_filename> <output_data_filename> [--max_cont <max_cont>]\n";
    cerr << "Options: \n";
    return -1;
  }
  
  int max_cont = 5;
  
  functions::ArgumentData arg(argc, argv);
  
  int repeats = 1;
  int particles = 100;
  
  if (arg.isOption("repeat")) {
    arg.getOption("repeat", repeats);
    cout << "Setting repetitions: " << repeats << endl;
  }
  if (arg.isOption("max_cont")) {
    arg.getOption("max_cont", max_cont);
    cout << "Setting max_cont: " << max_cont << endl;
  }
  if (arg.isOption("particles")) {
    arg.getOption("particles", particles);
    cout << "Setting particles: " << particles << endl;
  }
  
  
  ostringstream result_string;
  
  for (int i = 0; i < repeats; i++) {
    cout << endl << "Iteration: " << i << endl << endl;
    result_string << solveProblem(arg, max_cont, i + 1, particles).toString() << endl;
    
  }
  
  if (writeStringToFile(arg.at(2), result_string.str())) {
    cout << "Results exported successfully to: " << arg.at(2) << endl;
  } else {
    cerr << "Could not write the results to: " << arg.at(2) << endl;
    return -1;
  }
  
  
  return 0;
}

results solveProblem(ArgumentData& arg, int max_cont, int id, int particles)
{
  results ret_val;
  ret_val.id = id;
  
  FormattedTime t0, t1, t2;
  
  t0.getTime();
  t1.getTime();
 // 1. Montecarlo simulation --> First make a montecarlo simulation of the whole system
  Simulator *s = NULL;
  try {
    ParseBlock b;
    b.load(arg[1].c_str());
    
    s = new Simulator(b["simulator"]);
  } catch (exception &e) {
    cerr << "Error while loading the file. Content: " << e.what() << endl;
  }
  
  bool collision;
  RealVector max_dev;
  CRAlgorithmFactory alg_fac;
  s->montecarlo(collision, particles, max_dev, false, false);
  t2.getTime();
  cout << "First simulation done. Spended time: " << t2 - t1 << endl;
  ret_val.montecarlo.push_back(t2 - t1);
  
  cout << "Max deviations: " << max_dev.toString() << endl;
  
  if (arg.isOption("export_original")) {
    string s_;
    arg.getOption("export_original", s_);
    cout << "Exporting original trajectories to: " << s_ << endl;
    s->exportTrajectory(s_);
  }
  
  if (!collision) {
    cout << "No collision has been found.\n";
    return ret_val;
  }

  CRAlgorithm *alg = alg_fac.createFromFile(arg.at(1));
  CRGenetics *algorithm = dynamic_cast<CRGenetics *>(alg);
  
  int cont = 0;
  while (cont < max_cont && algorithm != NULL) {
    double expansion = max_dev.at(0) + max_dev.at(1);
    if (!collision) {
      expansion = max_dev.at(0);
      cout << "No collision in the last solution --> setting the expansion to: " << expansion << endl;
    } else {
      cout << "Collision --> setting the expansion to: " << expansion << endl;
    }
    algorithm->getSimulator()->expandGeometries(expansion);
    vector<double > original_geo = algorithm->getSimulator()->getGeometries().at(0);
    
    t1.getTime();
    CRAlgorithmStatistics stats = algorithm->execute_one_vs_all(s->getTrajectories(), cont == 0);
    t2.getTime();
    
    cout << "Algorithm executed. Expended time: " << t2 - t1 << "\n";
    cout << "New plan: " << endl,
    cout << algorithm->getSimulator()->getFlightPlans().at(0).toString() << endl; 
    ret_val.ga.push_back(t2 - t1);
    ret_val.cost.push_back(stats.getMinObjetive());
    
    t1.getTime();
    // Set the new plan and simulate again!
    s->setFlightPlans(algorithm->getSimulator()->getFlightPlans());
    s->montecarlo_1_vs_all(collision, particles, max_dev, false);
    t2.getTime();
    
    cout << "Montecarlo executed. Expended time: " << t2 - t1 << ". ";
    ret_val.montecarlo.push_back(t2 - t1);
    
    if (collision) {
      cout << "Collision found\n";
      ret_val.collision.push_back(1);
    } else {
      cout << "NO Collision found\n";
      ret_val.collision.push_back(0);
    }
    cout << "Max deviations: " << max_dev.toString() << endl;
    
    cont++;
    algorithm->getSimulator()->expandGeometries(-expansion);
  }
  
  t2.getTime();
  cout << endl << "TOTAL Time: " << t2 - t0 << endl;
  ret_val.time = t2-t0;
  
  if (algorithm == NULL) {
    cerr << "Could not create the algorithm instance of the problem\n";
  }
  
  // Free memory
  delete algorithm;
  delete s;
  
  return ret_val;
}

