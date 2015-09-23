#include "Range.h"
#include <functions/functions.h>

namespace simulator {

// SIMPLERANGE functions. TODO: MOVE TO FUNCTIONS?
SimpleRange::SimpleRange()

{
 first = 0.0;
 second = first;
}


SimpleRange::SimpleRange(double low, double high)
{
  if (low > high) {
    double aux = low;
    low = high;
    high = aux;
  }
  first = low;
  second = high;
}

SimpleRange SimpleRange::intersect(const SimpleRange& other) const
{
  SimpleRange aux;
  if (other.first <= first || isnan(other.first)) {
    if (other.second > first) {
      aux.first = first;
      aux.second = functions::minimum(other.second, second);
    }
  } else {
    aux = other.intersect(*this);
  }
  return aux;
}

SimpleRange SimpleRange::makeUnion(const SimpleRange& other) const
{
  SimpleRange aux;
  
  if (!intersect(other).isEmpty()) {
    aux.first = functions::minimum(first, other.first);
    aux.second = functions::maximum(second, other.second);
  }
  return aux;
}

std::string SimpleRange::toString() const
{
  std::ostringstream os;
  
  
  os << "[" << first << ", " << second << "]";
  
  return os.str();
}


/// RANGE functions. TODO: MOVE TO FUNCTIONS?

Range Range::intersect(const simulator::Range& other) const
{
  Range ret_val;
  for (unsigned int i = 0; i < size(); i++) {
    for (unsigned int j = 0; j < other.size(); j++) {
      SimpleRange aux_range = at(i).intersect(other.at(j));
      ret_val.push_back_check(aux_range);
    }
  }
  return ret_val;
}


Range::Range(double low, double high)
{
  SimpleRange aux(low, high);
  push_back_check(aux);
}

// TODO: make it better (now does not consider repeated ranges)
Range Range::unite(const Range& other) const
{
  Range aux;
  for (unsigned int i = 0; i < size(); i++) {
    aux.push_back_check(at(i));
  }
  for (unsigned int j = 0; j < other.size(); j++) {
    aux.push_back_check(other.at(j));
  }
  
  return aux;
}

void Range::push_back_check(const SimpleRange& push)
{
  if (push.isEmpty()) {
    // First check --> do not aggregate empty ranges
    return;
  }
  Range::iterator it = begin();
  
  SimpleRange aux = push;
  for (unsigned int i = 0; i < size(); i++, it++) {
    if (!push.intersect(at(i)).isEmpty()) {
      // If there is an intersection --> the candidate to push back is the union and the old range has to be deleted
      aux = aux.makeUnion(at(i));
      erase(it);
      i--;
      it = begin() + i;
    }
  }
  
  push_back(aux);
}


Range::Range()
{
}

std::string Range::toString() const
{
  std::ostringstream os;
  
  for (unsigned int i = 0; i < size(); i++) {
    os << at(i).toString();
    if (i < size() - 1) {
      os << " U ";
    }
  }
  
  if (empty()) {
    os << "Empty";
  }
  
  return os.str();
}

bool Range::equal(const Range& other) const
{
  bool ret_val = size() == other.size();
  for (unsigned int i = 0; i < size() && ret_val; i++) {
    bool found = false;
    for (unsigned int j = 0;j < size() && !found; j++) {
      found = at(i).equal(other.at(j));
    }
    ret_val = found;
  }
  
  return ret_val;
}


}