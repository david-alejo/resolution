#include "SimpleGraph.h"

#include <iostream>
#include <functions/RealVector.h>

using namespace simple_graph;
using namespace std;
using namespace functions;

#include <string.h>

int main (int argc, char **argv) {
  bool symmetric = false;
  if (argc > 1) {
    if ( strcmp(argv[1], "symmetric") == 0) {
      cout << "Symmetric test.\n";
      symmetric = true;
    }
  }
  
  SimpleGraph<RealVector, double> graph(symmetric);
  
  // Define the vertices
  RealVector location;
  location.push_back(0.0);
  location.push_back(0.0);

  graph.addVertex(location);
  location[0] = 1000.0;
  graph.addVertex(location);
  location[1] = 1000.0;
  graph.addVertex(location);
  location[0] = 0.0;
  graph.addVertex(location);

  // Define the edges
  graph.addEdge(0, 1, 1000.0);
  graph.addEdge(0, 2, 1500.0);
  graph.addEdge(0, 3, 1000.0);
  
  cout << graph.toString() << endl;
}