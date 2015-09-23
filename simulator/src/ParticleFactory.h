#ifndef _PARTICLE_FACTORY_H_
#define _PARTICLE_FACTORY_H_
#include "Particles.h"
#include <sparser/all.h>

/*!@brief Particle Factory
In order to add another model, add another 'else if' clause to each method
*/

namespace simulator {
class ParticleFactory {
  public:
    
    virtual Particle* create(const std::string &particleType, const Controller &ctr, const Model &mod, double w);
    virtual Particle* createFromFile(const std::string &particleType,const std::string & fileName);
    virtual Particle* createFromBlock(const std::string &particleType,ParseBlock &block);
		
    virtual Particle* createFromBlock(ParseBlock &block);
};
}

#endif //_PARTICLE_FACTORY_H_