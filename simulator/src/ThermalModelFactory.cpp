#include "ThermalModelFactory.h"

#include <iostream>

namespace simulator {

ThermalModel* ThermalModelFactory::createFromBlock(ParseBlock& block)
{
  std::string modelType = block("type").as<std::string>();
  if(modelType=="Simple"){
    ThermalModelSimple m;
    return m.createModel(block);
  } else if (modelType== "File"){
    ThermalModelFile m;
    return m.createModel(block);
  } else if (modelType== "Time"){
    ThermalModelTime m;
    return m.createModel(block);
  } else {
    std::cerr << "ThermalModelFactory::createFromFile -->Unrecognized ThermalModel type.\n";
  }
  
  return NULL;
}


}