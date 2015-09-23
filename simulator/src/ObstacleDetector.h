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


#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

#include<sparser/all.h>
#include <functions/RealVector.h>
#include "CD.h"

namespace simulator {

class ObstacleDetector
{
  
public:
  ObstacleDetector(ParseBlock &b);
  
  ~ObstacleDetector();
  
  //!@brief Collision check without CD
  bool collisionCheck(const functions::RealVector &pos) const;
  
  //! @brief Collision check by using CD
  bool collisionCheck(const functions::RealVector &pos, const std::vector<double> &geo) const;
  
  std::vector<functions::RealVector > &getObstacles() {return obstacles;}
  
protected:
  std::vector<functions::RealVector > obstacles; // Obstacles in environment (center and radius)
  std::vector<std::vector<double> > geometries; // Geometries when using a CD
  
  Checker *getChecker() const;
  
  void dispose();
  
  CD *cd;
};

}

#endif // OBSTACLEDETECTOR_H
