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


#ifndef THERMALMODELSIMPLE_H
#define THERMALMODELSIMPLE_H

#include "ThermalModel.h"
#include <functions/RealVector.h>
#include <sparser/all.h>
#include <vector>
#include "Updraft.h"

namespace simulator {

class ThermalModelSimple:public ThermalModel
{
  public:
    //! @return The vertical speed due to the thermal in 3D position v
    virtual double getVerticalWindSpeed(const functions::RealVector &v) const;

	virtual ThermalModel *createModel(ParseBlock &b) const;
    
    ThermalModel *clone() const;
    
    ~ThermalModelSimple();
    
    ParseBlock *exportThermalData() const;
    
  protected:
    virtual Checker* getChecker() const;
    
    virtual void dispose();

	void init(ParseBlock &b);
    
    std::vector<Updraft> updraft_vector;
};
}

#endif // THERMALMODELSIMPLE_H
