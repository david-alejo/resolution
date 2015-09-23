#ifndef PARTICLE_SIMPLE_H
#define PARTICLE_SIMPLE_H
#include <vector>
#include <string>
#include <stdio.h>
#include <sparser/all.h>
#include "Particle.h"
#include "ModelFactory.h"
#include "ControllerFactory.h"


namespace simulator {
class ParticleSimple: public Particle {
  
  public:
    
    /*! PVC implementation */
    virtual Particle *create(const Controller &con, const Model &mod,double w = 0.0) const;
    
    /*! Constructor from file. See test01/Particle.in to see an example */
    virtual Particle *createFromFile(const std::string &fileName) const;
		
		//! Constructor from parse block 
		virtual Particle *createFromBlock(ParseBlock &block) const;
		
    //! Checker to use sparser with the Particle data
    virtual Checker *particleFileChecker()const ;

       
    ~ParticleSimple();       
    /*! @brief runs one step of the model, modifying its states and maybe its parameters
    
    */
    virtual int runOneStep();
    
    /*! @brief returns the current state vector of the particle
    @return state The container for the state vector to be returned
    */
    virtual std::vector<double> getState() const;
		
    virtual void setState(const std::vector<double> &new_state);
    
    /*! @brief Sets the model of the particle
    @param pModel Model object reference 
    */
    virtual void setModel(Model *pModel);
    
    /*! @brief Gets the model of the particle
    @return Model object pointer
    */
    virtual Model *getModel();
		
    //! @brief Gets the controller of the particle
    //! @return The pointer to the controller object
    virtual Controller *getController();
		
    virtual void setController(Controller* nCon);
		
    /*! @brief Sets the weight of the particle
    @param w Particle weight
    */
    inline virtual void setWeight(const double w) { weight = w;};
    
    /*! @brief Gets the weight of the particle
    @return Weight of the particle
    */
    inline virtual double getWeight() const {return weight; };
    
    virtual void dispose();
    
    virtual void init();

    inline virtual std::string getType() const {return particleType; };

    void specialFeature(void **pointer, const char *type );
    
    inline virtual bool isStopped() const{return stopped;};
		
		inline virtual void setStopped(bool new_value) {
			if (controller != NULL) {
				controller->reset();
			}
			stopped = new_value;};
    virtual void stopParticle();
    virtual ParticleSimple *clone() const;
    
    virtual void resumeParticle();
		
		
    //! @brief Returns a string that represents the content of the ParticleFactory
    //! @return The string
    virtual std::string toString() const;

    //! @brief Translate the content into a parse block
    virtual ParseBlock *toBlock();

  protected:
    //!True if the particle is stopped
    bool stopped;
    //! Model of the particle
    Model *model;
    //! Controller of the particle
    Controller *controller;
    //! Particle weight for Particle filter use
    double weight;
    //! This flag stops, or not the simulation when reaching the final waypoint
    bool stoppable;
  
    
    //! Init method
    virtual void init(const Controller& fms, const Model& mod, double w );
    
		static const std::string particleType;
		
		//! Default constructor (private)
    ParticleSimple ();
    //!Hiding Copy constructor
    ParticleSimple (const ParticleSimple &that);
    //! Hiding assignment operator
    ParticleSimple & operator = (const ParticleSimple &that);
    
    //! @brief Set pointers to null
    virtual void pointersToNULL();
    
    virtual void setTimeStep(double T) {
      if (model != NULL) {
	model->setTimeStep(T);
      }
      if (controller != NULL) {
	controller->setT(T);
      }
    }
    
  
  friend class ParticleFactory;
};

}
#endif //#ifndef PARTICLE_SIMPLE_H
