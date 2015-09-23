#ifndef _MODEL_FACTORY_H_
#define _MODEL_FACTORY_H_
#include "Models.h"
#include <sparser/all.h>

namespace simulator {

/*!@brief Model Factory
In order to add another model, add another 'else if' clause to each method
*/
class ModelFactory {
  public:
    
    Model* create(const std::string &modelType,
		  const std::vector<double> &_parameter,
		  const std::vector<double> &_state,
		  const std::vector<double> &_control);
			
    Model* create_from_file(const std::string &modelType,const std::string & fileName);
		
    Model* create_from_block(const std::string &modelType, ParseBlock &block);
  
};

}

#endif //_MODEL_FACTORY_H_