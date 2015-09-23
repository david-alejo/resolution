/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "ObstacleDetector.h"

using namespace simulator;

ObstacleDetector::ObstacleDetector(ParseBlock& b)
{
  cd = NULL;
  Checker *check;
  try {
    check = getChecker();
    ParseBlock::Properties *obs = b.getProperties("obstacle");
    ParseBlock::Properties::iterator it = obs->begin();
    
    for (;it != obs->end(); it++) {
      std::vector<double> v = (*it)->as<std::vector<double> >();
      if (v.size() == 3) {
	obstacles.push_back(v);
      }
    }
  }catch (std::exception &e) {
    std::cerr << " ObstacleDetector::ObstacleDetector --> could not initialize the class.\n";
  }
}

ObstacleDetector::~ObstacleDetector()
{
  dispose();
}

bool ObstacleDetector::collisionCheck(const functions::RealVector& pos) const
{
  bool ret_val = false;
  functions::Point3D p(pos.at(0), pos.at(1), 0.0);
  for (unsigned int i = 0; i < obstacles.size() && !ret_val; i++) {
    // Check colls in 2D
    functions::Point3D p1 (obstacles.at(i).at(0), obstacles.at(i).at(1), 0);
    ret_val = p.distance(p1) < obstacles.at(i).at(2);
  }
  return ret_val;
}

// TODO: make the cd call
bool ObstacleDetector::collisionCheck(const functions::RealVector& pos, const std::vector< double >& geo) const
{
  return collisionCheck(pos);
}

void ObstacleDetector::dispose()
{
  geometries.clear();
  obstacles.clear();
  delete cd;
}

Checker* ObstacleDetector::getChecker() const
{
  Checker *check = new Checker;
  check->addProperty("obstacle", new OneOrMore());
  
  return check;
}
