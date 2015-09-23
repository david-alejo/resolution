#include "CD.h"

namespace simulator {

bool CD::loadGeometryFromFile(const std::string& filename, std::vector< double >& geometry) const
{
	bool ret_val = false;
	std::ifstream file;
	file.open(filename.c_str());
	
	geometry.clear();
	
	if (file.is_open()) {
		double aux;
		file >> aux;
		while (!file.eof()) {
			geometry.push_back(aux);
			file >> aux;
		}
		
		file.close();
		ret_val = true;
	}
	
	return ret_val;
}

}
