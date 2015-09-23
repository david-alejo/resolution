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


#ifndef CONTROLLERQUADCATECEASY_H
#define CONTROLLERQUADCATECEASY_H

#include "ControllerQuadCatec.h"

namespace simulator {

class ControllerQuadCatecEasy: public ControllerQuadCatec
{
  //! @brief Translate the content into a parse block
  virtual ParseBlock *toBlock();
  
  virtual std::string getType() const { return "ControllerQuadCatecEasy"; };
  
    virtual ~ControllerQuadCatecEasy();
  
protected:
  //!@brief Update the State of WP
  //!@return True if changed
  virtual bool update_nextWpIndex();
  
  //! @brief Updates the next waypoint number and the plane to be crossed. 
  //! If necessary raises the final waypoint flag.
  //! @param vertical_plane If true the plane is forced to be vertical
  //! @return The distance to the new plane
  virtual double updateWaypoint(bool vertical_plane = true);
  
  //!@brief Hidden default constructor (use preferred one always)
    ControllerQuadCatecEasy();
  //!@brief Hidden Copy constructor (use clone always)
  ControllerQuadCatecEasy (const ControllerQuadCatecEasy &that);
  //!@brief Hidden assignment operator (use clone always)
  ControllerQuadCatecEasy & operator = (const ControllerQuadCatecEasy &that);
  
    friend class ControllerFactory;
};

} // namespace simulator

#endif // CONTROLLERQUADCATECEASY_H
