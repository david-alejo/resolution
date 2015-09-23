#ifndef __PF_SEED_H__
#define __PF_SEED_H__

#include "Particle.h"
/** @brief Interface (Pure abstract class) that all Particle filter seeds have to implement

 */
namespace simulator {
class PFSeed{
	public:
 		//! Returns a vector of <i>num</i> particles. YOU MUST <i>delete</i> IT AND ITS CONTENT
		virtual std::vector< Particle *> getParticlesList(const int num ) const = 0;
 	
	
};

}

#endif //__PF_SEED_H__
