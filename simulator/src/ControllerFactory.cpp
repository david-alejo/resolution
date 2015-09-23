#include "ControllerFactory.h"

#include <iostream>

namespace simulator {
	
Controller* ControllerFactory::createFromFile(const std::string &modelType, const std::string& fileName){
  if(modelType=="ControllerSimple"){
    ControllerSimple m;
    return m.createFromFile(fileName);
  } else if (modelType== "ControllerSimpleGlider") {
    ControllerSimpleGlider m;
    return m.createFromFile(fileName);
  } else if (modelType== "ControllerSimpleUAV") {
    ControllerSimpleUAV m;
    return m.createFromFile(fileName);
  } else if (modelType== "ControllerQuad") {
    ControllerQuad m;
    return m.createFromFile(fileName);
  } else if (modelType== "ControllerQuadCatec") {
    ControllerQuadCatec m;
    return m.createFromFile(fileName);
  } else if (modelType== "ControllerQuadCatecEasy") {
    ControllerQuadCatecEasy m;
    return m.createFromFile(fileName);
  } else {
		std::cerr << "ControllerFactory::createFromFile -->Unrecognized controller type.\n";
	}
  
  return NULL;
}
Controller* ControllerFactory::create(const std::string& modelType, std::vector < double > _def){
  if(modelType=="ControllerSimple"){
    ControllerSimple m;
    return m.create(_def);
    
  } else if (modelType== "ControllerSimpleGlider"){
    ControllerSimpleGlider m;
    return m.create(_def);
  } //else if (modelType== "ControllerANOTHER"){return ... }
  else {
		std::cerr << "ControllerFactory::createFromFile -->Unrecognized controller type.\n";
	}
  return NULL;
  
}

Controller* ControllerFactory::createFromBlock(const std::string& modelType, ParseBlock& block)
{
  if(modelType=="ControllerSimple"){
    ControllerSimple m;
    return m.createFromBlock(block);
  } else if (modelType== "ControllerSimpleGlider"){
    ControllerSimpleGlider m;
    return m.createFromBlock(block);
  } else if (modelType== "ControllerSimpleUAV"){
    ControllerSimpleUAV m;
    return m.createFromBlock(block);
  } else if (modelType== "ControllerQuad") {
    ControllerQuad m;
    return m.createFromBlock(block);
  } else if (modelType== "ControllerQuadCatec") {
    ControllerQuadCatec m;
    return m.createFromBlock(block);
  } else if (modelType== "ControllerQuadCatecEasy") {
    ControllerQuadCatecEasy m;
    return m.createFromBlock(block);
  } else {
		std::cerr << "ControllerFactory::createFromFile -->Unrecognized controller type.\n";
	}
  
  return NULL;
}


}