#ifndef PARTICLE_H
#define PARTICLE_H
#include <vector>
#include <string>
#include <stdio.h>
#include "Models.h"
#include "Controllers.h"

namespace simulator {

class Particle{
  protected:
		
		std::vector<double> geometry;
    
     
  public:
    static const int NORMAL = 0;
    static const int WAYPOINT_CHANGED = 1;
    static const int LAST_WAYPOINT_REACHED = 2;
    
    
    /*! @brief "Pure Virtual Constructor". Do not ate the system
			bool run(bool &collision);use real constructors.
    @param ctr Controller of the particle
    @param mod Model of the particle
    */
    virtual Particle *create(const Controller &ctr, const Model &mod, double weight =0.0 ) const =0;
		
		//! @brief "Pure Virtual Constructor" from file. Uses the method createFrom Block.
		//! See test01/Particle.in to see an example */
    Particle *createFromFile(const std::string &fileName) const;
		
		/*! @brief "Pure Virtual Constructor" from parse block. See test01/Particle.in to see an example */
		Particle *createFromBlock(ParseBlock &block) const;

    
    /*! @brief runs one step of the model, modifying its states
     *  @return A control value.
     *  @retval NORMAL Nothing special happened
     *  @retval WAYPOINT_CHANGED The waypoint has been reached
     *  @retval LAST_WAYPOINT_REACHED All the flight plan has been completed
    */
    virtual int runOneStep() = 0;
    
    /*! @brief returns the current state vector of the particle
    @return state The container for the state vector to be returned
    */
    virtual std::vector<double> getState() const = 0;
		
		virtual void setState(const std::vector<double> &new_state) = 0;
    
    /*! @brief Sets the model of the particle
    @param pModel Model object reference 
    */
    virtual void setModel(Model *pModel)=0;
    
    /*! @brief Gets the model of the particle
    @return Model object pointer
    */
    virtual Model *getModel()= 0;
		
    //! @brief Gets the controller of the particle
    //! @return The pointer to the controller object
    virtual Controller *getController() = 0;
    
    virtual void setController(Controller *nCon) = 0;
    
    /*! @brief Sets the weight of the particle
    @param w Particle weight
    */
    virtual void setWeight(const double w) = 0;
    
    /*! @brief Gets the weight of the particle
    @return Weight of the particle
    */
    virtual double getWeight() const = 0;
    
    /*! @brief Returns the particle type name
    @return String that identifies the type of the model
    */    
    virtual std::string getType() const = 0;
		
		//! @brief Sets the geometry
		inline void setGeometry(const std::vector<double> new_geo) {
			geometry = new_geo;
		}
		
		inline std::vector<double> getGeometry() const {
			return geometry;
		}
		
		
    
    /*! @brief For any other functionality that is not modeled in this interface
	@param pointer A pointer to something
	@param type Type of the pointer, in order to identify it
	
	*/
    virtual void specialFeature(void ** pointer, const char *type ) = 0;
    
    //! Returns true if the particle is stopped
    virtual bool isStopped() const = 0;
		
		virtual void setStopped(bool new_value) = 0;
    //! Returns a deep copy of this object
    virtual Particle *clone() const = 0;
    //! Stops the particle
    virtual void stopParticle()=0;  
    //! Resumes the particle 
    virtual void resumeParticle() = 0;
   
    //! @brief Translate the content into a parse block
    //! @return A pointer to the block that contains the info
    virtual ParseBlock *toBlock() = 0;
    
    virtual void setTimeStep(double T) = 0;

    virtual ~Particle(){};
    
  
    friend class ParticleFactory;
};
}

#endif //#ifndef PARTICLE_H
