#include "SiCoDe.h"
#include <iostream>
#include <functions/functions.h>
#include "Range.h"

namespace simulator {

using namespace std;
using functions::RealVector;

std::vector< std::pair< unsigned int, unsigned int > > SiCoDe::allignedBoxesCollide(const std::vector< box >& boxes){
  
  std::vector<std::pair<unsigned int, unsigned int> > ret;
//   std::vector<std::pair<unsigned int, unsigned int> >::iterator pit;
  std::vector< std::pair < std::vector< double >, std::vector< double > > >::const_iterator bit1=boxes.begin();
  std::vector< std::pair < std::vector< double >, std::vector< double > > >::const_iterator bit2;
  unsigned int i=0;
  unsigned int j=0;
  for(;bit1!=boxes.end()-1; bit1++,i++){
    for(bit2=bit1+1;bit2!=boxes.end(); bit2++,j++){
			// Create intervals
			if ( SiCoDe::allignedBoxesCollide(*bit1, *bit2)) {
				ret.push_back(std::pair <unsigned int,unsigned int> (i,j));
			}
      
    }
  }
  
  return ret;
}

bool SiCoDe::allignedBoxesCollide(const box& box1, const box& box2)
{
	std::pair<double, double> x1_int(box1.first[0], box1.second[0]);
	std::pair<double, double> x2_int(box2.first[0], box2.second[0]);
	std::pair<double, double> y1_int(box1.first[1], box1.second[1]);
	std::pair<double, double> y2_int(box2.first[1], box2.second[1]);
	std::pair<double, double> z1_int(box1.first[2], box1.second[2]);
	std::pair<double, double> z2_int(box2.first[2], box2.second[2]);
	
	return overlap(x1_int, x2_int) && overlap(y1_int, y2_int) && overlap(z1_int, z2_int);
}


bool SiCoDe::detectCollision(const std::vector< std::vector< double > >& position, const std::vector< std::vector< double > >& geometry) const
{
  vector<pair<vector<double>, vector<double> > > v;
  bool error = false;
  
  for (unsigned int i = 0; i < position.size() && !error; i++) {
    vector<double> min_edge(position[i]);
    vector<double> max_edge(position[i]);
    
    if (position[i].size() != 3 || geometry[i].size() != 3) {
      cerr << "SiCoDe::detectCollision --> Error: position or geometry sizes aren't equal to 3\n";
      cerr << "position[" << i << "] = " << functions::printVector(position[i]) << "\t";
      cerr << "geometry[" << i << "] = " << functions::printVector(geometry[i]) << "\t";
    }
    
    for (unsigned int j = 0; j < min_edge.size() && !error; j++) {
      min_edge[j] -= geometry[i][j] * 0.5;
      max_edge[j] += geometry[i][j] * 0.5;
    }
    
    v.push_back(pair<vector<double>, vector<double> >(min_edge, max_edge));
  }
  
  return !error && !SiCoDe::allignedBoxesCollide(v).empty();
}

bool SiCoDe::detectCollision1vsAll(const vector< vector< double > >& position, const vector< vector< double > >& geometry) const
{
  bool ret_val = false;
  vector<double> min_edge(position[0]); // Will be the opposite edges of the box of the ith UAV
  vector<double> max_edge(position[0]);
  for (unsigned int k = 0; k < 3; k++) {
    min_edge[k] -= geometry[0][k] * 0.5;
    max_edge[k] += geometry[0][k] * 0.5;
  }
  
  for (unsigned int i = 1; i < position.size() && !ret_val; i++) {
    vector<double> min_edge_2(position[i]); // Will be the opposite edges of the box of the ith UAV
    vector<double> max_edge_2(position[i]);
    for (unsigned int k = 0; k < 3; k++) {
      min_edge_2[k] -= geometry[i][k] * 0.5;
      max_edge_2[k] += geometry[i][k] * 0.5;
      box box_a(min_edge, max_edge);
      box box_b(min_edge_2, max_edge_2);
      ret_val = SiCoDe::allignedBoxesCollide(box_a, box_b);
    }
  }
  return ret_val;
}


bool SiCoDe::detectCollision(const std::vector< std::vector< double > >& position, const std::vector< double >& geometry) const
{
	
	bool error = false;
	bool ret_val = false;
	
	for (unsigned int i = 0; i < position.size() - 1 && !error && !ret_val; i++) {
		for (unsigned int box1 = 0; box1 + 2 < geometry.size() && ! error && !ret_val; box1 += 3) {
			vector<double> min_edge(position[i]); // Will be the opposite edges of the box of the ith UAV
			vector<double> max_edge(position[i]);
			
			if (position[i].size() != 3 ) {
				cerr << "SiCoDe::detectCollision --> Error: position size aren't equal to 3\n";
				error = true;
			} else {
				for (unsigned int k = 0; k < 3; k++) {
					min_edge[k] -= geometry[k + box1] * 0.5;
					max_edge[k] += geometry[k + box1] * 0.5;
				}
			}
			
			
			
			for (unsigned int j = i + 1; j < position.size() && !error && !ret_val; j++) {
				for (unsigned int box_2 = 0; box_2 + 2 < geometry.size() && !error && !ret_val; box_2 += 3) {
					// Will be the opposite edges of the box of the jth UAV
					vector<double> min_edge_2(position[j]);
					vector<double> max_edge_2(position[j]);
				
					if (position[j].size() != 3 ) {
						cerr << "SiCoDe::detectCollision --> Error: position size aren't equal to 3\n";
						error = true;
					} else {
						for (unsigned int k = 0; k < 3; k++) {
							min_edge_2[k] -= geometry[box_2 + k] * 0.5;
							max_edge_2[k] += geometry[box_2 + k] * 0.5;
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

bool SiCoDe::segmentCollision(const RealVector& p1_0, const RealVector& p1_1, 
			      const RealVector& p2_0, const RealVector& p2_1, 
			      const RealVector& geo1, const RealVector& geo2) const
{
  // We have to solve a linear inequaty involving absolute values. Once for each coordinate
  bool ret_val = false;
  RealVector p = p2_0 - p1_0; // Difference
  RealVector v = p2_1 - p2_0 + p1_0 - p1_1;
  
  if (p1_0.size() != 3 || p1_1.size() != 3 || p2_0.size() != 3 || p2_1.size() != 3) {
	  cerr << "SiCoDe::segmentCollision --> Error: position sizes aren't equal to 3\n";
	  return false;
	}
  
  for (unsigned int box1 = 0; box1 + 2 < geo1.size() && !ret_val; box1 += 3) {
    for (unsigned int box2 = 0; box2 + 2 < geo2.size() && !ret_val; box2 += 3) {
      Range curr_range(0, 1.0); // We assume that the time goes between 0 and 1.0
      bool possible_collision = true; // Suppose collision, if there is one coordinate that is not conflictive --> do not keep searching
      for (unsigned int i = 0; i < p1_0.size() && possible_collision; i++) {
	// The separation is the half sum of the geometries
	double sep = (geo1.at(box1 + i) + geo2.at(box2 + i)) / 2;
	// Get the coordinates that we are checking
	double vx = v.at(i);
	double px = p.at(i); 
	if (vx < 0.0) {
	  vx *= -1;
	  px *= -1; // Change the sign if necessary (it does not affect because of the absolute value
	}
	// Assuming MRU we can retrieve the range of t where it is accomplished abs (px + vx * t) <= sep
	if (vx != 0.0) {
	  Range r( -(px + sep)/vx, (sep - px)/vx);
	  curr_range = curr_range.intersect(r);
	  possible_collision = !curr_range.empty(); // If the range is empty --> no collision (ret_val = true indicates collision)
	} else {
	  // If there is no relative movement in the coordinate, a collision is possible only when the coordinates are close enough
	  possible_collision = abs(px) < sep;
	}
      }
      ret_val = possible_collision; 
      
    }
  }
  
  return ret_val;
}

void SiCoDe::expandGeometry(std::vector< double >& geo, double dist) const
{
  for (unsigned int i = 0; i < geo.size(); i++) {
    geo[i] += dist;
  }
}


}
