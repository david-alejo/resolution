#include "CDFactory.h"

#include <iostream>

namespace simulator {
	
CD* CDFactory::create(const std::string& detector_type){
  CD *ret = getPointerFromType(detector_type);
	
	if ( ret == NULL ) {
		std::cerr << "CDFactory:: -->Unrecognized controller type.\n";
	}
	
	return ret;
}
	
CD *CDFactory::getPointerFromType(const std::string &detector_type) {
	CD *ret = NULL;
	
	if ( detector_type == "SiCoDe") {
		ret = new SiCoDe();
	} else if (detector_type== "SiCoDeExtended") {
	    ret = new SiCoDeExtended();
	}
	//else if (detector_type== "CDANOTHER"){return ... }
		
  return ret;
}


}
