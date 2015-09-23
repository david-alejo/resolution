#ifndef CR_ALGORITHM_STATISTICS_H
#define CR_ALGORITHM_STATISTICS_H

#include <string>
#include "EvolutionData.h"

namespace resolution {

class CRAlgorithmStatistics {
	public:
	CRAlgorithmStatistics();
	
	~CRAlgorithmStatistics();
	
	void setError(bool new_error);
	void setCollisionDetected(bool new_cd);
	void setSolved(bool new_solved);
	inline bool getError() const { return error;}
	inline bool getCollisionDetected() const { return collision_detected;}
	inline bool getSolved() const { return solved;}
	
	void setExecutionTime(float new_ex);
	void setMinObjetive(float new_obj);
	inline float getExecutionTime()  const {return execution_time;}
	inline float getMinObjetive() const {return min_objetive;}
	inline void setEvaluations(const double eva) {evaluations = eva;}
	
	//! @brief Shows the information in a fancy way
	//! @return the string
	std::string toString() const;
	
	//! @brief Exports the exec time and the objective to a MATLAB like (the numbers separated by space)
	//! @return the string
	std::string toMatlab() const;
	
	//! @brief Gets the evolution data
	inline std::vector<EvolutionData> getEvolutionData() const {
		return evo;
	}
	
	//! @brief Sets the evolution data
	inline void setEvolutionData(const std::vector<EvolutionData> &data) {
		evo = data;
	}
	
	// ------------------ Attributes -----------------------
	private:
	bool solved; // If true --> a collision was detected and the problem was solved
	bool collision_detected; // If false --> no collision have been detected in the initial flight plan
  bool error;
	float execution_time;
	float min_objetive;
	int evaluations;
	std::vector<EvolutionData> evo;
};

class StatisticVector:public std::vector<CRAlgorithmStatistics> {
public:
  std::string toString() const;
};

}

#endif