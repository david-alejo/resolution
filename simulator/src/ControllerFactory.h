#ifndef _CONTROLLER_FACTORY_H_
#define _CONTROLLER_FACTORY_H_
#include <vector>
#include <string>
#include "Controllers.h"
#include <sparser/all.h>

namespace simulator {

/*!@brief Controller Factory
In order to add another model, add another 'else if' clause to each method
*/
class ControllerFactory {
  public:
    Controller* create(const std::string &modelType, std::vector <double> );
		
		//! @brief Creates a model from file
		//! @param modelType The type of the model to be created
		//! @param fileName The file where the data is allocated
    Controller* createFromFile(const std::string& modelType, const std::string& fileName);
		
		//! @brief Creates a model from file
		//! @param modelType The type of the model to be created
		//! @param block The block where the info is located
		Controller* createFromBlock(const std::string& modelType, ParseBlock& block);
};

}

#endif //_MODEL_FACTORY_H_
