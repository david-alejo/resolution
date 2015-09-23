#ifndef _THERMAL_MODEL_FACTORY_H_
#define _THERMAL_MODEL_FACTORY_H_
#include <vector>
#include <string>
#include "Thermals.h"
#include <sparser/all.h>

namespace simulator {

/*!@brief Thermal Model Factory
In order to add another model, add another 'else if' clause to each method
*/
class ThermalModelFactory {
  public:
		//! @brief Creates a model from file
		//! @param modelType The type of the model to be created
		//! @param block The block where the info is located
		static ThermalModel* createFromBlock(ParseBlock& block);
};

}

#endif //_MODEL_FACTORY_H_
