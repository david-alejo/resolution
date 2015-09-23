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


#include "ControllerQuadCatecEasy.h"

namespace simulator {
ControllerQuadCatecEasy::ControllerQuadCatecEasy()
{
  ControllerQuadCatec::init();
}

ControllerQuadCatecEasy::ControllerQuadCatecEasy(const ControllerQuadCatecEasy& that)
{
  init(that);
}

ParseBlock* ControllerQuadCatecEasy::toBlock()
{
    ParseBlock *ret = simulator::ControllerQuadCatec::toBlock();
    
    ret->setProperty("type", getType());
    
    return ret;
}

bool ControllerQuadCatecEasy::update_nextWpIndex()
{
  bool ret_val = false;
  functions::Point3D p (stateVector);
  if (p.distance(fp.at(nextWpIndex)) < min_wp_dist) {
    updateWaypoint(false);
    ret_val = true;
  }
  return ret_val;
}

double ControllerQuadCatecEasy::updateWaypoint(bool vertical_plane)
{
    return simulator::ControllerQuadCatec::updateWaypoint(vertical_plane);
}

ControllerQuadCatecEasy::~ControllerQuadCatecEasy()
{
  ControllerQuadCatec::dispose();
}

  
}