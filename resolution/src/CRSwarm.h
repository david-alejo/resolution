#ifndef __CRSWARM_H__
#define __CRSWARM_H__

#include <string>

#include "CRAlgorithm.h"
#include "CRAlgorithmFactory.h"
#include "CRAlgorithmStatistics.h"
#include "SwarmConfig.h"
#include "EvolutionData.h"
#include <particle_swarm/ParticleSwarm.h>

/** @brief This class implements the Particle Swarm algorithm that uses the custom particle swarm, and the Collision Detector.
 */
namespace resolution {

	class CRAlgorithmFactory;
	
	
class CRSwarm:public CRAlgorithm {
	public:
		//!Destructor
		~CRSwarm();
		
		//! @brief Creates an Algorithm from a file. Necessary
		//! @param block The Parse Block that contains the file info
		//! @return A pointer to the algorithm
		CRAlgorithm* createFromBlock(ParseBlock& block) const;
		
		//! @brief Prefer clone to copy constructor.
		CRSwarm * clone () const;  
			
		//! @brief Returns a string representation of the object
		std::string toString() const;
		
		//! @brief Runs the algorithm in the specified folder
		CRAlgorithmStatistics execute();
		
protected:
	virtual void init(ParseBlock &block, AlgorithmConfig *conf);
	
	//! Default Constructor (do not use)
		CRSwarm();
	
		// Dimension of the problem
		int problem_dimension;
		
		//! @brief Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//! @brief init without arguments -> all internal data to empty or zero values
		virtual void init();
		
		//! @brief Makes all pointers that are in the class point to NULL.
		void pointersToNULL();
		
		//! @brief Gets the problems bound (useful in waypoint change problems)
		//! @param upper A vector that will get the upper bounds
		//! @param lower A vector that will get the lower bounds
		virtual void getBounds(std::vector<double> &upper, std::vector <double> &lower);
		
		//! @brief Translates the content of the class into a ParseBlock in order to write it to a file
		//! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
		virtual ParseBlock *toBlock() const;	
		
		virtual inline std::string getType() const {
		  return "Swarm";
		}
		
		
		
		friend class CRAlgorithmFactory;
};

}

#endif //__CRSWARM_H__
