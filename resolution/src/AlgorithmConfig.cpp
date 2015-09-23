#include "AlgorithmConfig.h"
#include <functions/functions.h>

namespace resolution {
	void AlgorithmConfig::init()
{
  debug = false;
  export_solution = false;
  export_trajectories = false;
  solution_filename = "solution.m";
  trajectory_filename = "trajectory.m";
  export_catec = "";
  double_geometry.clear();
  geometry_expansion = 0.0;
}

void AlgorithmConfig::init(ParseBlock& block)
{
  // Set the default parameters
  init();

  try {
    if (block.hasProperty("debug")) {
      debug = block("debug").as<bool>();
    }
    if (block.hasProperty("export_solution")) {
      export_solution = block("export_solution").as<bool>();
    }
    if (block.hasProperty("export_catec")) {
      export_catec = block("export_catec").value;
    }
    if (block.hasProperty("export_trajectories")) {
      export_trajectories = block("export_trajectories").as<bool>();
    }
    if (block.hasProperty("solution_filename")) {
      solution_filename = block("solution_filename").value;
    }
    if (block.hasProperty("trajectory_filename")) {
      trajectory_filename = block("trajectory_filename").value;
    }
    if (block.hasProperty("double_geometry")) {
      double_geometry = block("double_geometry").as<std::vector<double> >();
    }
    if (block.hasProperty("geometry_expansion")) {
      geometry_expansion = block("geometry_expansion").as<double>();
    }
  } catch (std::exception &e) {
    std::cerr << "AlgorithmConfig (initializer) --> failed to get the data from block. Content: ";
    std::cerr << e.what() << "\n";
  }
}

AlgorithmConfig::~AlgorithmConfig()
{
  dispose();
}

void AlgorithmConfig::dispose()
{
  init();
}

ParseBlock* AlgorithmConfig::toBlock() const
{
  ParseBlock *block = new ParseBlock;
  
  block->setProperty("export_trajectories", functions::boolToString(export_trajectories));
  block->setProperty("trajectory_filename", trajectory_filename);
  block->setProperty("export_solution", functions::boolToString(export_solution));
  block->setProperty("solution_filename", solution_filename);
  block->setProperty("debug", functions::boolToString(debug));
  block->setProperty("double_geometry", functions::printVector(double_geometry));
  block->setProperty("geometry_expansion", functions::numberToString(geometry_expansion));
  
  return block;
}

	
}