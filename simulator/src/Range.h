#ifndef SIM_RANGE_H__
#define SIM_RANGE_H__

#include <vector>
#include <string>

namespace simulator {

//! @class SimpleRange Implements the simplest representation of a range (does not distinguish wether it is open or closed)
class SimpleRange : public std::pair<double, double> {
public:
  SimpleRange();
  SimpleRange(double low, double high);
  SimpleRange intersect(const SimpleRange &other) const;
  inline bool isEmpty() const {
    return first == second;
  }
  inline bool equal(const SimpleRange &other) const {
    return first == other.first && second == other.second;
  }
  //! @brief If the ranges intersect --> returns the union of the ranges. If not, returns an empty range
  SimpleRange makeUnion(const SimpleRange &other) const;
  
  //! @brief Represents the range in an user friendly way ;)
  std::string toString() const;
};

//! @class Range Extends the SimpleRange function in order to 
class Range : public std::vector<SimpleRange> {
public:
  Range();
  Range(double low, double high);
  Range intersect(const Range &other) const;
  Range unite(const Range &other) const;
  void push_back_check(const SimpleRange &push); // Pushes back if not empty and taking into account the already existing ranges
  
  std::string toString() const;
  
  bool equal(const simulator::Range& other) const;
  
protected:
//   void reduce(); Not necessary for the moment TODO: develop need when needed
};

}

#endif