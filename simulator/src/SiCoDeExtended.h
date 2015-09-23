#ifndef _SICODE_SICODE_EXTENDED
#define _SICODE_SICODE_EXTENDED

#include <vector>
#include "SiCoDe.h"

namespace simulator {


/** @brief SiCoDe (Simple Collision Detection) Collection of simple collision detection methods
  
  This simple library provides simple collision detection methods for pairs of objects.


*/

class SiCoDeExtended:public SiCoDe{
  public:
	static bool allignedBoxesCollide(const box &box1, const box &box2);
  
  
	//! @brief Inherited method from CD. Detects collisions using alligenedboxescollide
	//! @param position Position of the center of each box
	//! @param geometry Group of boxes. 3 coords per box: x, y and z edge sizes. Same geo 4 all. Any extra value will be ignored
    virtual bool detectCollision(const std::vector< std::vector< double > >& position, const std::vector< std::vector< double > >& geometry) const;
		
		//! @brief Inherited method from CD. Detects collisions using alligenedboxescollide
	//! @param position Position of the center of each box
	//! @param geometry Group of boxes. 6 coords per box: x, y and z edge sizes and then the x, y and z offsets from the center of the robot.
    virtual bool detectCollision(const std::vector< std::vector< double>, std::allocator< std::vector< double> > >& position, const std::vector<double>& geometry) const;
    
    //! @brief Inherited method from CD. Detects collisions using alligenedboxescollide, but only 1 vs all!! (the first position)
    //! @param position Position of the center of each box
    //! @param geometry Group of boxes. 3 coords per box: x, y and z edge sizes. Same geo 4 all. Any extra value will be ignore
    virtual bool detectCollision1vsAll(const std::vector< std::vector< double > >& position, const std::vector< std::vector< double > >& geometry) const;
    
    // TODO: implement this
    virtual bool segmentCollision(const functions::RealVector &p1_0, const functions::RealVector &p1_1,
				  const functions::RealVector &p2_0, const functions::RealVector &p2_1,
				  const functions::RealVector &geo1, const functions::RealVector &geo2) const; 

    inline virtual std::string getType() const {
      return "SiCoDeExtended";
    }
    
    virtual void expandGeometry(std::vector<double> &geo, double dist) const;
};


}

#endif //_SICODE_SICODE_EXTENDED
