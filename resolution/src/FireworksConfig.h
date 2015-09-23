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

#ifndef FIREWORKSCONFIG_H
#define FIREWORKSCONFIG_H

#include "CostConfig.h"
#include <boost/concept_check.hpp>

namespace resolution{

class FireworksConfig : public CostConfig
{
public:
  int am; // minimum sparks
  int bm; // Maximum sparks
  int mm; 
  int m;
  double amplitude_mult; // Maximum amplitude of sparks multiplicator (usually is the half of the search space)
  double A_min; // Minimum amplitude of a spark
  bool new_mapping;
  bool efwa_selection;
  
  FireworksConfig();
  
  // -------------------- FUNCTIONS -------------
      
  //! @brief Creates an instance of the algorithm configuration
  //! @param block The parseblock data
  //! @return The instance
  virtual AlgorithmConfig* createAlgorithmConfig(ParseBlock &block);
  
  //! @brief All data in the class to the default values
  virtual void init();
  
  //! @brief Translates the content of the class into a ParseBlock in order to write it to a file
  //! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
  virtual ParseBlock *toBlock() const;
    
protected:
  //! @brief Returns an instance of Checker necessary in order to check the block contents
  //! @return The checker
  Checker* getChecker();
  
  virtual void dispose();
  virtual void init(ParseBlock &block);
};

}

#endif // FIREWORKSCONFIG_H
