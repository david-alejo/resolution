#ifndef __PF_NORMAL_SEED_H__
#define __PF_NORMAL_SEED_H__

#include <string>
#include <vector>

#include <sstream>
#include <sparser/all.h>
#include "Particle.h"
#include "PFSeed.h"
#include "boost/random.hpp"

/** @brief Particle filter seed using normal distribution

 */
namespace simulator {
class PFNormalSeed : public PFSeed{
  typedef  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > norm_rand;
	  
  private:
  
  //! Internal data
    std::vector < std::vector <double> > *sigma;
    std::vector < std::vector <double> > *mean;
    boost::mt19937 *generator;
    std::vector < boost::normal_distribution <> *> *normal_dist_state;
    std::vector < boost::normal_distribution <> *> *normal_dist_control;
    std::vector < boost::normal_distribution <> *> *normal_dist_parameter;
    
    ParseBlock data;
			  
  protected:
    //! Default constructor is protected, use another one please
    PFNormalSeed(){init();}
		    
    //! Disposal of resources (erases contents and restart)
    void dispose () throw ();
    
    //! init without arguments -> all internal data to empty or zero values
    void init();
    
    /** Initializing method
      * 
    */
    void init(const std::string &modelType, const std::vector< std::vector<double> > &me , const std::vector< std::vector<double> > &sd);	
					  
  public:
    /**Preferred constructor 
      * @param data ParseBlock of data from file
      */
    PFNormalSeed(ParseBlock &data);
    
    //!Destructor
    ~PFNormalSeed();
    
    //! Prefer clone to copy constructor.
    PFNormalSeed * clone() const;  
	    
    //!Returns a string representation of the object
    std::string toString();
    
    //!Returns a pointer to a vector of <i>num</i> particles, using a Normal distribution with the given means and std deviations to randomly generate the state, control and parameter vectors. YOU MUST <i>delete</i> IT 
    std::vector< Particle *> getParticlesList(const int num );
	  
  private:
    //! Hiding default shallow copy of copy constructor
    PFNormalSeed (const PFNormalSeed &that);
    //! Hiding assignment operator
    PFNormalSeed & operator = (const PFNormalSeed &that);
};
}
#endif //__PF_NORMAL_SEED_H__ 
