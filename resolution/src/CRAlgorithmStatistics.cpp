#include "CRAlgorithmStatistics.h"

#include <sstream>
#include <iostream>
#include <functions/functions.h>

using namespace std;

namespace resolution {

CRAlgorithmStatistics::CRAlgorithmStatistics()
{
	collision_detected = false;
	solved = false;
	error = false;
	execution_time = 0.0;
	min_objetive = 0.0;
	evo.clear();
}

CRAlgorithmStatistics::~CRAlgorithmStatistics()
{
	evo.clear();
}


void CRAlgorithmStatistics::setCollisionDetected(bool new_cd)
{
	collision_detected = new_cd;
}

void CRAlgorithmStatistics::setError(bool new_error)
{
	error = new_error;
}

void CRAlgorithmStatistics::setSolved(bool new_solved)
{
	solved = new_solved;
}


void CRAlgorithmStatistics::setExecutionTime(float new_ex)
{
	execution_time = new_ex;
}

void CRAlgorithmStatistics::setMinObjetive(float new_obj)
{
	min_objetive = new_obj;
}

std::string CRAlgorithmStatistics::toString() const
{
	ostringstream os;
	
	os << "Execution time = " << execution_time << endl;
	os << "Minimum objective = " << min_objetive << endl;
	os << "Flags: Error = " << functions::boolToString(error) << endl;
	os << "Evaluations: " << evaluations << endl;
	
	return os.str();
}

std::string CRAlgorithmStatistics::toMatlab() const
{
	ostringstream os;
	
	os << execution_time << " ";
	os << min_objetive << " ";
	
	if (evo.size() > 0 && evo.at(0).sum_delta_ETA != 0.0) {
	 os << evo.at(evo.size() - 1).sum_delta_ETA << " ";
	}
	
	return os.str();
}

string StatisticVector::toString() const
{
  ostringstream ret;
  
  ret << "% The first column is the spended time and the second is the cost. \n";
  ret << "% If present, the third is the mean of the absolute value of differences between the original ETA and the ETA of the best individual. \n";
  
  ret << "result = [ " ;
  for(unsigned int k = 0; k < size(); k++) {
    ret << at(k).toMatlab();
    if (k < size() - 1) {
      ret << "; ";
    } else {
      ret << "]\n";
    }
  }
  unsigned int size_speed = 0;
  for (unsigned int i = 0; i < size() && size_speed == 0; i++) {
    if (at(i).getEvolutionData().size() > 0) {
      size_speed = at(i).getEvolutionData().at(0).speed_in_sector.size();
    }
  }
  std::vector<double> cero(size_speed, 0.0);
  if (size_speed > 0) {
    
    ret << "best_speed_in_sector = [" ;
    for(unsigned int k = 0; k < size(); k++) {
      const std::vector<EvolutionData> &evo = at(k).getEvolutionData();
      std::vector <double> represent;
      if (evo.size() > 0) {
	represent = evo.at(evo.size() - 1).speed_in_sector;
      } else {
	
	represent = cero;
      }
      ret << functions::printVector(represent);
      if (k < size() - 1) {
	ret << "; ";
      } else {
	ret << "]\n";
      }
    }
  }
  
  for(unsigned int k = 0; k < size(); k++) {
    std::vector<EvolutionData> evo_data = at(k).getEvolutionData();
				
    ret << "evo_cost{" << k + 1 << "} = [";
    for (unsigned int m = 0; m < evo_data.size(); m++) {
      ret << evo_data.at(m).cost << " ";
    }
    ret << "]\n";
				
    ret << "evo_time{" << k + 1 << "} = [";
    for (unsigned int m = 0; m < evo_data.size(); m++) {
      ret << evo_data.at(m).t << " ";
    }
    ret << "]\n";
    
    if (evo_data.size() > 0 && evo_data.at(0).sum_delta_ETA != 0.0) {
      ret << "evo_delta_ETA{" << k + 1 << "} = [";
      for (unsigned int m = 0; m < evo_data.size(); m++) {
	ret << evo_data.at(m).sum_delta_ETA << " ";
      }
      ret << "]\n";
    }
    
    if (evo_data.size() > 0 && evo_data.at(0).speed_in_sector.size() > 0) {
      ret << "evo_speed_in_sector{" << k + 1 << "} = [";
      for (unsigned int m = 0; m < evo_data.size(); m++) {
	ret << functions::printVector(evo_data.at(m).speed_in_sector);
	
	if (k < size() ) {
	  ret << "; ";
	}
      }
      ret << "]\n";
    }
    
    
  }
  
  return ret.str();
}


} // namespace resolution
