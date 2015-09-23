#ifndef __SWARM_CONFIG_H__
#define __SWARM_CONFIG_H__

#include "CostConfig.h"

namespace resolution {

class SwarmConfig:public CostConfig {
	public:
	// ----------------------- DATA ---------------------
	// Parameter section
	
	std::string initializer_type;
	double search_ratio; // Useful in the random initialization
	double max_checks;
	double r0, phi0; // Parameters of PS algorithm
	double inertia_weight;
	
	// -------------------- FUNCTIONS -------------
	
	//!@brief destructor
	~SwarmConfig();
	
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
		
  
	//! @brief initializer from parseblock
	//! @param block The block where the data is allocated
	virtual void init(ParseBlock &block);
	
	//! @brief Frees all fields that are necessary to free
	virtual void dispose();
};

}

#endif // __SWARM_CONFIG_H__