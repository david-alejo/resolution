/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  sinosuke <email>

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


#ifndef __THERMALMODELFILE_H
#define __THERMALMODELFILE_H

#include <functions/RealVector.h>
#include <sparser/all.h>
#include <vector>
#include "ThermalModel.h"

namespace simulator {

typedef std::vector<std::vector<double> > WindMatrix;

class ThermalModelFile:public ThermalModel
{
public:
    virtual ThermalModel *createModel(ParseBlock &b) const;

    //! @return The vertical speed due to the thermal in 3D position v
    virtual double getVerticalWindSpeed(const functions::RealVector &v) const;

    ThermalModel *clone() const;
    
    virtual ParseBlock *exportThermalData() const;
    
    virtual ~ThermalModelFile();

protected:
  bool loadData(const std::string &filename);

  Checker *getChecker() const;
  
  virtual double getVerticalWindSpeed(const functions::RealVector &v, const WindMatrix &data) const;

  std::string filename;
  WindMatrix data;
  double resolution;
};

}

#endif // __THERMALMODELFILE_H