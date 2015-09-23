#include "ModelFactory.h"

namespace simulator {

Model* ModelFactory::create(const std::string &modelType, const std::vector< double >& _parameter, const std::vector< double >& _state, const std::vector< double >& _control){
  if(modelType=="ModelSimpleQuad"){
    ModelSimpleQuad m;
    return m.create(_parameter,_state,_control);
  } else if (modelType=="ModelSimpleGlider"){
    ModelSimpleGlider m;
    return m.create(_parameter,_state,_control);
  } //else if (modelType== "ModelANOTHER"){return ... }
  
  return NULL;
}

Model* ModelFactory::create_from_file(const std::string &modelType, const std::string& fileName){
  if(modelType=="ModelSimpleQuad"){
    ModelSimpleQuad m;
    return m.createFromFile(fileName);
  } else if (modelType== "ModelSimpleGlider") {
    ModelSimpleGlider m;
    return m.createFromFile(fileName);
  } else if (modelType == "ModelQuad") {
    ModelQuad m;
    return m.createFromFile(fileName);
  } else if (modelType == "ModelQuadCatec") {
    ModelQuadCatec m;
    return m.createFromFile(fileName);
  } else if (modelType == "ModelSimpleUAV") {
    ModelSimpleUAV m;
    return m.createFromFile(fileName);
  } else if (modelType == "ModelSimpleUAVPolarWind") {
    ModelSimpleUAVPolarWind m;
    return m.createFromFile(fileName);
  }
  
  
  return NULL;
}

Model* ModelFactory::create_from_block(const std::string &model_type, ParseBlock& block)
  {
  if (model_type == "ModelSimpleQuad") {
    ModelSimpleQuad m;
    return m.createFromBlock(block);
  } else if (model_type == "ModelSimpleGlider") {
    ModelSimpleGlider m;
    return m.createFromBlock(block);
  } else if (model_type == "ModelQuad") {
    ModelQuad m;
    return m.createFromBlock(block);
  } else if (model_type == "ModelQuadCatec") {
    ModelQuadCatec m;
    return m.createFromBlock(block);
  } else if (model_type == "ModelSimpleUAV") {
    ModelSimpleUAV m;
    return m.createFromBlock(block);
  } else if (model_type == "ModelSimpleUAVPolarWind") {
    ModelSimpleUAVPolarWind m;
    return m.createFromBlock(block);
  }
	
	return NULL;
}

}
