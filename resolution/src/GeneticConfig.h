#ifndef __GENETIC_CONFIG_H__
#define __GENETIC_CONFIG_H__

#include "CostConfig.h"

namespace resolution {

class GeneticConfig:public CostConfig {
	public:
	// ----------------------- DATA ---------------------
	// Parameter section
	int max_checks; // Maximum number of check in the initialization phase
	int custom_evolution;
	
	// Operator section
	std::string initializer_type;
	std::string crossover_type;
	std::string mutator_type;
	double pMutation;
	double mutation_dev;
	double pCrossover;
	
	double search_ratio;
	
	// -------------------- FUNCTIONS -------------
	
	//! @brief Creates an instance of the algorithm configuration
	//! @param block The parseblock data
	//! @return The instance
	virtual AlgorithmConfig* createAlgorithmConfig(ParseBlock &block);
	
	//! @brief All data in the class to the default values
	virtual void init();
	
	//! @brief Translates the content of the class into a ParseBlock in order to write it to a file
	//! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
	virtual ParseBlock *toBlock() const;
	
	protected:
	//! @brief Returns an instance of Checker necessary in order to check the block contents
	//! @return The checker
  Checker* getChecker();
	
	virtual void dispose();
	
	virtual void init(ParseBlock &block);
};

}

#endif // __GENETIC_CONFIG_H__