/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#include "CRAlgorithmFactory.h"
#include <iostream>
#include "CRAlgorithms.h"

using namespace std;

namespace resolution {
CRAlgorithmFactory::CRAlgorithmFactory()
{
  
}

CRAlgorithm* CRAlgorithmFactory::createFromFile(const std::string& filename)
{
  CRAlgorithm *ret = NULL;
  ParseBlock config;
  
  Checker *check = getAlgorithmChecker();
  
  try {
    config.load(filename.c_str());
    config.checkUsing(check);
    
    if ( config("algorithm").value == "Genetic" ) {
      CRGenetics gen;
      ret = gen.createFromBlock(config);
    } else if ( config("algorithm").value == "ParticleSwarm" ) {
      CRSwarm swarm;
      ret = swarm.createFromBlock(config);
    } else if ( config("algorithm").value == "Fireworks" ) {
      CRFireworks fireworks;
      ret = fireworks.createFromBlock(config);
    }
  } catch (exception &e) { 
    cerr << "CRAlgorithmFactory::createFromFile --> Error while loading data. Content:" << e.what() << endl;
  }
  
  delete check;
  
  return ret;
}

Checker* CRAlgorithmFactory::getAlgorithmChecker() const
{
  Checker *ret = new Checker();
  
  ret->addProperty("algorithm", new NTimes(1));
  ret->addBlock("config", new NTimes(1));
  
  return ret;
}

}