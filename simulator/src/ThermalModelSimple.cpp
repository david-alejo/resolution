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


#include "ThermalModelSimple.h"
#include <iostream>

using namespace std;

namespace simulator {
ThermalModel *ThermalModelSimple::createModel(ParseBlock& b) const
{
  
  ThermalModel *r = new ThermalModelSimple;
  ThermalModelSimple *tms = dynamic_cast<ThermalModelSimple *>(r);

  tms->init(b);
  return r;
}

void ThermalModelSimple::init(ParseBlock &b) {
  Checker *check = getChecker();
  try {
    b.checkUsing(check);
    
    // Get updraft info
    ParseBlock::Blocks *updrafts = b.getBlocks("updraft");
	
    updraft_vector.clear();
    ParseBlock::Blocks::iterator it = updrafts->begin();
    for ( ; it != updrafts->end(); it++) {
      Updraft aux(**it);
      updraft_vector.push_back(aux);
    }
  } catch (exception &e) {
    cerr << "ThermalModelSimple::ThermalModelSimple --> Error: could not load the data.\n";
    throw(e);
  }
  
  delete check;
}

ThermalModelSimple::~ThermalModelSimple()
{
  dispose();
}

ThermalModel *ThermalModelSimple::clone() const
{

  ThermalModel *r = new ThermalModelSimple;
  ThermalModelSimple *tms = dynamic_cast<ThermalModelSimple *>(r);
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
    Updraft aux(updraft_vector.at(i));
    tms->updraft_vector.push_back(aux);
  }

  return r;
}


void ThermalModelSimple::dispose()
{
  updraft_vector.clear();
}

double ThermalModelSimple::getVerticalWindSpeed(const functions::RealVector& v) const
{
     double ret_val = 0.0;
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
    functions::RealVector vec2d;
    vec2d.push_back(v.at(0));
    vec2d.push_back(v.at(1));
    
    const Updraft &u = updraft_vector.at(i);
    
    if (vec2d.distance(u.getLocation()) < u.radius && v.at(2) < u.max_height) {
      ret_val += u.getWindSpeed();
    }
  }
  
  return ret_val;
}


Checker* ThermalModelSimple::getChecker() const
{
  Checker *check = ThermalModel::getChecker();
  check->addBlock("updraft", new OneOrMore());
  check->addChecker("updraft", Updraft::getChecker());
  return check;
}

ParseBlock *ThermalModelSimple::exportThermalData() const
{
  ParseBlock* block = new ParseBlock;
  for (unsigned int i = 0; i < updraft_vector.size(); i++) {
    block->setBlock("updraft", updraft_vector.at(i).toBlock());  
  }

  return block;
}


  
}