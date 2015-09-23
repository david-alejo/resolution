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

#ifndef CRFIREWORKS_H
#define CRFIREWORKS_H

#include "CRAlgorithm.h"

//constant variables
#ifndef PI
#define PI (3.14159265358979323846264338327950288)
#endif
#ifndef E
#define E  (2.71828182845904523536028747135266249)
#endif
#ifndef INF
#define INF (1e38)
#endif
#ifndef EPS
#define EPS (1e-38)
#endif

namespace resolution {

//! @class CRFireworks
//! @brief Uses the implementation of FWA library to solve the path planning problem
class CRFireworks : public CRAlgorithm
{
public: 
    virtual CRAlgorithm* createFromBlock(ParseBlock& block) const;
    
    CRFireworks* clone() const;
    
    virtual CRAlgorithmStatistics execute();
    
    CRAlgorithmStatistics execute_one_vs_all(std::vector<std::vector <functions::RealVector> > &traj, bool initialize);
    
    std::string toString() const;
    
    virtual inline std::string getType() const {
      return "Fireworks";
    }
    
    ~CRFireworks();
  
protected:
    // Stuff necessary for the FWA library
    double **fireworks,*fitnesses, **positions,*vals;
    double *Rs;
    double **As;
    double *result;
    int dimension;
    int *nums;	
    int *flags;
    int buffer;
    int evaluations;
  
    void pointersToNULL();
    
    CRFireworks();
    
    virtual void init(ParseBlock &block, AlgorithmConfig *conf);
    
    //! @brief Translates the content of the class into a ParseBlock in order to write it to a file
    //! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
    virtual ParseBlock *toBlock() const;
    
    //! @brief Allocates the memory for the FWA execution
    void allocateMemory();
    
    //! @brief Frees the memory after the FWA execution
    void freeMemory();
    
    //! @BRIEF Implements the mapping operator of original FWA (as found in FWA C library)
    double correct_bounds_FWA(double position, double LBOUND, double UBOUND) const;
    
    //! @brief Implements the mapping operator of EFWA as specified in Zheng (13) "Enhanced Fireworks Algorithm"
    double correct_bounds_EFWA(double position, double LBOUND, double UBOUND) const;
    
    //! @brief Performs the selection of the population as EFWA
    //! @param population The individuals to be considered
    void EFWASelection(int population);
    
    //! @brief Performs the selection of the population as FWA (as implemented in FWA C Library)
    //! @param population The individuals to be considered
    void FWASelection(int population);
    
    //! @brief Finds the minima of the population in vals
    //! @param idx Out: returns the index
    //! @param counter The number of population to be taken into account
    //! @return The minimum objective value so far
    double getMinima(int& index, int counter);
    
    
    friend class CRAlgorithmFactory;

};

}

#endif // CRFIREWORKS_H
