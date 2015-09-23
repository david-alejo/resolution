#ifndef __ALGORITHM_CONFIG__H
#define __ALGORITHM_CONFIG__H

#include <sparser/all.h>

namespace resolution {

class AlgorithmConfig {
  public:
  // ----------- Flags -----
  bool export_trajectories;
  std::string trajectory_filename;
  bool debug; // If true --> prints debug information      
  bool export_solution; // If true the solution waypoints are exported with trajectory_filename
  std::string solution_filename;
  std::vector<double> double_geometry; // This geometry will only be used in the generation of random problems
  double geometry_expansion;
  std::string export_catec;

  //! @brief creates a new configuration class for an algorithm. To be implemented in a final class
  //! @param block The block where the data is allocated
  virtual AlgorithmConfig* createAlgorithmConfig(ParseBlock &block) = 0;
  
  virtual ~AlgorithmConfig();
  
  //! @brief sets all parameters to their defaut values
  virtual void init();
  
  //! @brief initializer from parseblock
  //! @param block The block where the data is allocated
  virtual void init(ParseBlock &block);
  
  //! @brief Translates the content of the class into a ParseBlock in order to write it to a file
  //! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
  virtual ParseBlock *toBlock() const;

protected:
  //! @brief Frees all fields that are necessary to free
  virtual void dispose();
};

}

#endif // __ALGORITHM_CONFIG__H