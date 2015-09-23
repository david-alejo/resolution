#include "Range.h"
#include "functions/ArgumentData.h"
#include "functions/RandomNumberGenerator.h"
#include <iostream>

using namespace simulator;
using namespace functions;
using namespace std;

int main (int argc, char **argv) {
  ArgumentData arg(argc, argv);
  int num_ranges = 5;
  double min = 0;
  double max = 100;
  
  RandomNumberGenerator rnd;
  
  if (arg.size() >= 2) {
    istringstream is(arg.at(1));
    int aux;
    try {
      is >> aux;
      num_ranges = aux;
    } catch (exception &e) {}
  }
  if (arg.size() >= 4) {
    istringstream is(arg.at(2));
    istringstream is_2(arg.at(3));
    double aux;
    try {
      is >> aux;
      min = aux;
      is_2 >> aux;
      max = aux;
    } catch (exception &e) {}
  }
  
  std::vector<Range> r;
  
  for (int i = 0; i < num_ranges; i++) {
    // Get a range
    Range aux(floor(rnd.rnd(min, max)), floor(rnd.rnd(min,max)));
    // And output it to the screen
    cout << "Range " << i + 1 << " = " << aux.toString() << endl;
    r.push_back(aux);
  }
  
  Range curr_union;
  for (int i = 0; i < num_ranges - 1; i++) {
    // Get a range
    curr_union = curr_union.unite(r.at(i));
    // And output it to the screen
    cout << "Intersect " << i + 1 << " and " << i + 2 << ". " << r.at(i).intersect(r.at(i + 1)).toString() << "\t";
    cout << "Curr union: " << curr_union.toString() << endl;
  }
  
  
  cout << "Performing self-intersect check...";
  if (r.at(0).intersect(r.at(0)).equal(r.at(0))) {
    cout << "Passed!\n";
  } else {
    cout << "Failed!\n";
  }
  
  
  
  
  return 0;
}