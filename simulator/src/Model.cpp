#include "Model.h"
#include "functions/functions.h"

using namespace std;

namespace simulator {
std::vector< double > Model::getInitialState() const
{
	return initial_state;
}

std::string Model::toString() const
{
	ostringstream os;
	
	os << "Model Type: " << modelType << "\t";
	os << "State = " << functions::printVector(state) << " \t";
	os << "Parameters = " << functions::printVector(parameter) << " \t";
	os << "Control = " << functions::printVector(control) << " \t";
	
	return os.str();
}

ParseBlock *Model::toBlock() const {
  ParseBlock *ret_val = new ParseBlock;
  ret_val->setProperty("parameters", functions::printVector(parameter));
  ret_val->setProperty("initial_conditions", functions::printVector(initial_state));
  ret_val->setProperty("model_type", modelType);
  ostringstream os;
  os << ts;
  ret_val->setProperty("T", os.str());
  
  return ret_val;
}

}