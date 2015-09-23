#ifndef _COLLISION_DETECTOR_FACTORY_H_
#define _COLLISION_DETECTOR_FACTORY_H_
#include <vector>
#include <string>
#include "CDs.h"
#include <sparser/all.h>

namespace simulator {

/*!@brief CD Factory
In order to add another collision detector, add another 'else if' clause to each method
*/
class CDFactory {
  public:
		//! @brief Creates a new collision detector from his type
		//! @param detector_type The type of collision detector
		//! @return the pointer
    CD* create(const std::string &detector_type);
		
		CD *getPointerFromType(const std::string& type);
};

}

#endif //_MODEL_FACTORY_H_