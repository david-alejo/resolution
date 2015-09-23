#include "SiCoDeExtended.h"
#include <iostream>
#include <functions/functions.h>

namespace simulator {

using namespace std;
using functions::RealVector;

bool SiCoDeExtended::detectCollision1vsAll(const vector< vector< double > >& position, const vector< vector< double > >& geometry) const
{
  bool error = false;
  bool ret_val = false;
  
  for (unsigned int box1 = 0; box1 + 5 < geometry[0].size() && ! error && !ret_val; box1 += 6) {
    vector<double> min_edge(position[0]);
    vector<double> max_edge(position[0]);
    for (unsigned int j = 0; j < 3 && !error; j++) {
      min_edge[j] += -geometry[0][box1 + j] * 0.5 + geometry[0][box1 + j + 3];
      max_edge[j] += geometry[0][j + box1] * 0.5 + geometry[0][box1 + j + 3];
    }
    for (unsigned int j = 1; j < position.size() && !error && !ret_val; j++) {
      for (unsigned int box_2 = 0; box_2 + 5 < geometry[j].size() && !error && !ret_val; box_2 += 6) {
	vector<double> min_edge_2(position[j]);
	vector<double> max_edge_2(position[j]);
	
	if (position[0].size() != 3 || position[j].size() != 3 ) {
		cerr << "SiCoDeExtended::detectCollision --> Error: position or geometry sizes aren't equal to 3\n";
		
	} else {
	  for (unsigned int k = 0; k < 3; k++) {
	    min_edge_2[k] += -geometry[j][box_2 + k] * 0.5 + geometry[j][box_2 + k + 3];
	    max_edge_2[k] += geometry[j][box_2 + k] * 0.5 + geometry[j][box_2 + k + 3];
	  }
	
	  box box_a(min_edge, max_edge);
	  box box_b(min_edge_2, max_edge_2);
				
	  if (!error) {
	    // Detect collisions between a box of ith UAV and other box of jth UAV
	    ret_val |= SiCoDe::allignedBoxesCollide(box_a,box_b);
// 		    if (ret_val) {
// 		      cout << "Min egde = " << functions::printVector(min_edge) << "\t max edge = " << functions::printVector(max_edge) << endl;
// 		      cout << "Min egde_2 = " << functions::printVector(min_edge_2) << "\t max edge = " << functions::printVector(max_edge_2) << endl;
// 		    }
					
	  } // if (!error)
	} // else
      } // for (box_2)
    } // for(j)
  } // for(box_1)
	
  return !error && ret_val;
}


bool SiCoDeExtended::detectCollision(const std::vector< std::vector< double > >& position, const std::vector< std::vector< double > >& geometry) const
{
  bool error = false;
  bool ret_val = false;
  
  for (unsigned int i = 0; i < position.size() && !error; i++) {
    for (unsigned int box1 = 0; box1 + 5 < geometry[i].size() && ! error && !ret_val; box1 += 6) {
      vector<double> min_edge(position[i]);
      vector<double> max_edge(position[i]);
      for (unsigned int j = 0; j < 3 && !error; j++) {
	min_edge[j] += -geometry[i][box1 + j] * 0.5 + geometry[i][box1 + j + 3];
	max_edge[j] += geometry[i][j + box1] * 0.5 + geometry[i][box1 + j + 3];
      }
      for (unsigned int j = i + 1; j < position.size() && !error && !ret_val; j++) {
	for (unsigned int box_2 = 0; box_2 + 5 < geometry[j].size() && !error && !ret_val; box_2 += 6) {
	  vector<double> min_edge_2(position[j]);
	  vector<double> max_edge_2(position[j]);
	  
	  if (position[i].size() != 3 || position[j].size() != 3 ) {
		  cerr << "SiCoDeExtended::detectCollision --> Error: position or geometry sizes aren't equal to 3\n";
		  
	  } else {
	    for (unsigned int k = 0; k < 3; k++) {
	      min_edge_2[k] += -geometry[j][box_2 + k] * 0.5 + geometry[j][box_2 + k + 3];
	      max_edge_2[k] += geometry[j][box_2 + k] * 0.5 + geometry[j][box_2 + k + 3];
	    }
	  
	    box box_a(min_edge, max_edge);
	    box box_b(min_edge_2, max_edge_2);
				  
	    if (!error) {
	      // Detect collisions between a box of ith UAV and other box of jth UAV
	      ret_val |= SiCoDe::allignedBoxesCollide(box_a,box_b);
// 		    if (ret_val) {
// 		      cout << "Min egde = " << functions::printVector(min_edge) << "\t max edge = " << functions::printVector(max_edge) << endl;
// 		      cout << "Min egde_2 = " << functions::printVector(min_edge_2) << "\t max edge = " << functions::printVector(max_edge_2) << endl;
// 		    }
					  
	    } // if (!error)
	  } // else
	} // for (box_2)
      } // for(j)
    } // for(box_1)
  } // for(i)
	
  return !error && ret_val;
}

bool SiCoDeExtended::detectCollision(const std::vector< std::vector< double > >& position, const std::vector< double >& geometry) const
{
	
	bool error = false;
	bool ret_val = false;
	
	for (unsigned int i = 0; i < position.size() - 1 && !error && !ret_val; i++) {
		for (unsigned int box1 = 0; box1 + 5 < geometry.size() && ! error && !ret_val; box1 += 6) {
			vector<double> min_edge(position[i]); // Will be the opposite edges of the box of the ith UAV
			vector<double> max_edge(position[i]);
			
			if (position[i].size() != 3 ) {
				cerr << "SiCoDeExtended::detectCollision --> Error: position size aren't equal to 3\n";
				error = true;
			} else {
				for (unsigned int k = 0; k < 3; k++) {
					min_edge[k] += -geometry[k + box1] * 0.5 + geometry[box1 + k + 3];
					max_edge[k] += geometry[k + box1] * 0.5 + geometry[box1 + k + 3];
				}
			}
			
			
			
			for (unsigned int j = i + 1; j < position.size() && !error && !ret_val; j++) {
				for (unsigned int box_2 = 0; box_2 + 5 < geometry.size() && !error && !ret_val; box_2 += 6) {
					// Will be the opposite edges of the box of the jth UAV
					vector<double> min_edge_2(position[j]);
					vector<double> max_edge_2(position[j]);
				
					if (position[j].size() != 3 ) {
						cerr << "SiCoDeExtended::detectCollision --> Error: position size aren't equal to 3\n";
						error = true;
					} else {
						for (unsigned int k = 0; k < 3; k++) {
							min_edge_2[k] += -geometry[box_2 + k] * 0.5 + geometry[box_2 + k + 3];
							max_edge_2[k] += geometry[box_2 + k] * 0.5 + geometry[box_2 + k + 3];
						}
						
					}
					box box_a(min_edge, max_edge);
					box box_b(min_edge_2, max_edge_2);
					
					if (!error) {
						// Detect collisions between a box of ith UAV and other box of jth UAV
						ret_val |= SiCoDe::allignedBoxesCollide(box_a,box_b);
						
						if (ret_val) {
							#ifdef CD_DEBUG
							cout << "Collision detected between object " << i + 1 << "and " << j + 1<< "\t";
							cout << "Boxes: " << box1 / 3 + 1 << " and " << box_2 / 3 + 1 << "respectively.\n";
							cout << "Edges box 1: (" << aux::printVector(min_edge) << ") and (" << aux::printVector(max_edge);
							cout << ")\nEdges box 2: (" << aux::printVector(min_edge_2);
							cout << ") and (" << aux::printVector(max_edge_2)<< ")\n";
							#endif
						}
					}
				}
			}
		}
	}

	return ret_val || error;
}

void SiCoDeExtended::expandGeometry(std::vector< double >& geo, double dist) const
{
  for (unsigned int i = 0; i + 5 < geo.size(); i+=6) {
    for (unsigned int j = 0; j < 3; j++) {
      geo[i + j] += dist;
    }
  }
}

bool SiCoDeExtended::segmentCollision(const functions::RealVector& p1_0, const functions::RealVector& p1_1, const functions::RealVector& p2_0, 
				      const functions::RealVector& p2_1, const functions::RealVector& geo1, const functions::RealVector& geo2) const
{
    // We have to solve a linear inequaty involving absolute values. Once for each coordinate

  if (p1_0.size() != 3 || p1_1.size() != 3 || p2_0.size() != 3 || p2_1.size() != 3) {
    cerr << "SiCoDeExtended::segmentCollision --> Error: position sizes aren't equal to 3\n";
    return false;
  }
  bool ret_val = false;
  
  for (unsigned int box1 = 0; box1 + 5 < geo1.size() && !ret_val; box1 += 6) {
    for (unsigned int box2 = 0; box2 + 5 < geo2.size() && !ret_val; box2 += 6) {
      // We want to do this by calling the parent function
      
      functions::RealVector p1_0_new = p1_0;
      functions::RealVector p1_1_new = p1_1;
      functions::RealVector p2_0_new = p2_0;
      functions::RealVector p2_1_new = p2_1;
      functions::RealVector box_1_offset(3);
      functions::RealVector box_2_offset(3);
      functions::RealVector new_box_1(3);
      functions::RealVector new_box_2(3);
      
      // Translate the geometries
      for (unsigned int i = 0; i < 3; i++) {
	box_1_offset[i] = geo1[box1 + 3 + i];
	box_2_offset[i] = geo2[box2 + 3 + i];
	new_box_1[i] = geo1[box1 + i];
	new_box_2[i] = geo2[box2 + i];
      }
      
      
      // Actualize the variables
      p1_0_new = p1_0 + box_1_offset;
      p1_1_new = p1_1 + box_1_offset;
      p2_0_new = p2_0 + box_2_offset;
      p2_1_new = p2_1 + box_2_offset;
      
      // And call the parent function
      ret_val = SiCoDe::segmentCollision(p1_0_new, p1_1_new, p2_0_new, p2_1_new, new_box_1, new_box_2);
    }
  }
  
  return ret_val;
}


}
