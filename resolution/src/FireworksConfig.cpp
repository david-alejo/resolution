/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2015  sinosuke <email>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "FireworksConfig.h"
#include <functions/functions.h>

namespace resolution {

FireworksConfig::FireworksConfig()
{
  init();
}

AlgorithmConfig* FireworksConfig::createAlgorithmConfig(ParseBlock& block)
{
  AlgorithmConfig *c_ = new FireworksConfig;
  FireworksConfig *c = dynamic_cast<FireworksConfig *>(c_);
  
  Checker *check = getChecker();
  init();
  try {
    block.checkUsing(check);
    c->init(block);
    
    c->am = block("am").as<int>();
    c->bm = block("bm").as<int>();
    c->m = block("m").as<int>();
    c->mm = block("mm").as<int>();
    
    if (block.hasProperty("amplitude_mult")) {
      c->amplitude_mult = block("amplitude_mult").as<double>();
    }
    if (block.hasProperty("A_min")) {
      c->A_min = block("A_min").as<double>();
    }
    if (block.hasProperty("new_mapping")) {
      c->new_mapping = block("new_mapping").as<bool>();
    }if (block.hasProperty("efwa_selection")) {
      c->efwa_selection = block("efwa_selection").as<bool>();
    }
  } catch (std::runtime_error &e) {
	  std::cerr << "SwarmConfig::init() --> error while loading data from block. Content: " << e.what() << std::endl;
	  throw(e);
  }
  delete check;
  return c_;
}

void FireworksConfig::init()
{
    CostConfig::init();
    am = 2;
    bm = 40;
    mm = 5;
    m = 50;
    amplitude_mult = 1.0;
    A_min = 0.5;
    new_mapping = true; // Use EFWA by default
    efwa_selection = true; // Use EFWA by default
}

ParseBlock* FireworksConfig::toBlock() const
{
    ParseBlock *ret = resolution::CostConfig::toBlock();
    
    ret->setProperty("am", functions::numberToString(am));
    ret->setProperty("bm", functions::numberToString(bm));
    ret->setProperty("mm", functions::numberToString(mm));
    ret->setProperty("m", functions::numberToString(m));
    ret->setProperty("amplitude_mult", functions::numberToString(amplitude_mult));
    
    return ret;
}

void FireworksConfig::dispose()
{
    resolution::CostConfig::dispose();
}

Checker* FireworksConfig::getChecker()
{
  Checker *check = CostConfig::getConfigChecker();
  
  check->addProperty("am", new NTimes(1));
  check->addProperty("bm", new NTimes(1));
  check->addProperty("mm", new NTimes(1));
  check->addProperty("m", new NTimes(1));
  
  return check;
}

void FireworksConfig::init(ParseBlock& block)
{
  init();
  CostConfig::init(block);
}


}