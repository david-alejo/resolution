#include "ParticleFactory.h"
/*!\TODO: Think if it deserves to template these factories
The call will be something like
ParticleFactory<ParticleSimple> pFac;
pFac.create(const Controller &ctr, const Model &mod, double w);

and the code:
< class > T;
create ... {
  T clase;
  clase.create(...)
  }
*/

namespace simulator {
Particle* ParticleFactory::create(const std::string &particleType,  const Controller &ctr, const Model &mod, double w){
  if(particleType=="ParticleSimple"){
    ParticleSimple p;
    return p.create(ctr,mod,w);
  }//else if (particleType== "ParticleANOTHER"){return ... }
  
  return NULL;
}

Particle* ParticleFactory::createFromFile(const std::string& particleType, const std::string& fileName){
  if(particleType=="ParticleSimple"){
    ParticleSimple p;
    return p.createFromFile(fileName);
  }//else if (particleType== "ParticleANOTHER"){return ... }
  return NULL;
  
}

Particle* ParticleFactory::createFromBlock(const std::string& particleType, ParseBlock& block){
  if(particleType=="ParticleSimple"){
    ParticleSimple p;
    return p.createFromBlock(block);
  }//else if (particleType== "ParticleANOTHER"){return ... }
  return NULL;
  
}

Particle* ParticleFactory::createFromBlock(ParseBlock& block)
{
  string particleType;
  
  try {
    if (block.hasProperty("particle_type")) {
      particleType = block("particle_type").as<string>();
    } else if (block.hasProperty("type")) {
      particleType = block("type").as<string>();
    } else {
      std::cerr << "ParticleFactory::createFromBlock(ParseBlock& block) --> type not found\n";
      return NULL;
    }
  } catch (std::exception &e) {
    
    std::cerr << "ParticleFactory::createFromBlock(ParseBlock& block) --> exception found\n";
    return NULL;
  }
  
  return createFromBlock(particleType, block);
}


}
