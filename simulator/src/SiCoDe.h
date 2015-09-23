#ifndef _SICODE_SICODE_
#define _SICODE_SICODE_

#include <vector>
#include "CD.h"

namespace simulator {


/** @brief SiCoDe (Simple Collision Detection) Collection of simple collision detection methods
  
  This simple library provides simple collision detection methods for pairs of objects.


*/

typedef std::pair<std::vector <double>, std::vector<double> > box;

class SiCoDe:public CD{
  public:
  /** @brief Detects collisions between a set of boxes with the same orientation in space
  @param boxes Vector of of boxes. Each box is defined as a pair of vectors of 3 doubles [x_min,y_min,z_min],[x_max,y_max,z_max]
  @return Vector of pairs of the indexes of the colliding boxes, if any
  */
  static std::vector< std::pair <unsigned int, unsigned int> > allignedBoxesCollide(const std::vector<box> &boxes);
	
	
  static bool allignedBoxesCollide(const box &box1, const box &box2);
  
  
	//! @brief Inherited method from CD. Detects collisions using alligenedboxescollide
	//! @param position Position of the center of each box
	//! @param geometry Group of boxes. 3 coords per box: x, y and z edge sizes. Same geo 4 all. Any extra value will be ignored
    virtual bool detectCollision(const std::vector< std::vector< double > >& position, const std::vector< std::vector< double > >& geometry) const;
		
    //! @brief Inherited method from CD. Detects collisions using alligenedboxescollide
    //! @param position Position of the center of each box
    //! @param geometry Group of boxes. 3 coords per box: x, y and z edge sizes. Same geo 4 all. Any extra value will be ignore
    virtual bool detectCollision(const std::vector< std::vector< double>, std::allocator< std::vector< double> > >& position, const std::vector<double>& geometry) const;
    
    //! @brief Inherited method from CD. Detects collisions using alligenedboxescollide, but only 1 vs all!! (the first position)
    //! @param position Position of the center of each box
    //! @param geometry Group of boxes. 3 coords per box: x, y and z edge sizes. Same geo 4 all. Any extra value will be ignore
    virtual bool detectCollision1vsAll(const std::vector< std::vector< double > >& position, const std::vector< std::vector< double > >& geometry) const;
    
    //! @brief This method has to be implemented in every functional CD. 
    //! @brief and the obstacles that have been added. Does not add this trajectory to the system.
    //! @param position Position of all bodies to be checked
    //! @param geometry One common geometry of the bodies
    //! @retval true Some collisions have been detected
    //! @retval false The system is collision-free
    virtual bool segmentCollision(const functions::RealVector &p1_0, const functions::RealVector &p1_1,
				  const functions::RealVector &p2_0, const functions::RealVector &p2_1,
				  const functions::RealVector &geo1, const functions::RealVector &geo2) const; 

		//! @brief Checks if the intervals overlap
		//! @param first_interval First interval to check. The first value has to be lower than the second
		//! @param second_interval Second interval to check. The first value has to be lower than the second
		//! @retval true The intervals overlap
		//! @retval false The intervals do not overlap
		inline static bool overlap(const std::pair<double, double> &first_interval, const std::pair<double, double> &second_interval);
		
    inline virtual std::string getType() const {
      return "SiCoDe";
    }
    
    virtual void expandGeometry(std::vector<double> &geo, double dist) const;
};

bool SiCoDe::overlap(const std::pair< double, double >& fi, const std::pair< double, double >& se)
{
  return ( fi.first <= se.second) && ( fi.second >= se.first);
}

}

#endif //_SICODE_SICODE_
