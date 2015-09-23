#ifndef __CRGENETICS_H__
#define __CRGENETICS_H__

#include <string>
#include <ga/ga.h>
#include <ga/GARealGenome.h>
#include <ga/std_stream.h>
#include "CRAlgorithm.h"
#include "CRAlgorithmFactory.h"
#include "CRAlgorithmStatistics.h"
#include "GeneticConfig.h"
#include "EvolutionData.h"

/** @brief This class implements the complete CR Algorithm that uses GA, Particle Filter and the Collision Detector.
TODO Implement CheckGen
 */
namespace resolution {

  class CRAlgorithmFactory;

class CRGenetics:public CRAlgorithm {
public:
  //!Destructor
  ~CRGenetics();
  
  //! @brief Creates an Algorithm from a file. Necessary
  //! @param block The Parse Block that contains the file info
  //! @return A pointer to the algorithm
  CRAlgorithm* createFromBlock(ParseBlock& block) const;
  
  //! @brief Prefer clone to copy constructor.
  CRGenetics * clone () const;  
	  
  //! @brief Returns a string representation of the object
  std::string toString() const;
  
  //! @brief Runs the algorithm in the specified folder
  CRAlgorithmStatistics execute();
  
  CRAlgorithmStatistics execute_one_vs_all(std::vector<std::vector <functions::RealVector> > &traj, bool initialize);
  
  //! @brief Checks that the genome values meet the constrains
  //! @param arg1 The genome to be checked
  void checkGenomeBounds(GARealGenome &arg1);

  virtual inline std::string getType() const {
    return "Genetic";
  }
    
protected:

  //! @brief Sets the bounds in the GALIB format
  void setBounds();
    
  virtual void init(ParseBlock &block, AlgorithmConfig *conf);

  // Custom genetic operators
  friend float CRALgorithmObjective(GAGenome &);
  friend float CRALgorithmOneObjective(GAGenome &c);
  friend void GAPopulationInitializer(GAPopulation &);
  friend void GAGenomeRandomInitializer(GAGenome& g);
  friend int GAGenomeUniformWaypointMixer(const GAGenome &dad, const GAGenome &mom, GAGenome *g1, GAGenome *g2);
  friend int GARealGaussianMaMutator(GAGenome &gen, float pmut);
  friend int GARealUniformCrossoverCheck(const GAGenome& a, const GAGenome& b,
		      GAGenome* c, GAGenome* d);
  friend int GARealGaussianMutatorCheck(GAGenome& g, float pmut);

  //! Default Constructor (do not use)
  CRGenetics();

  // ---------- Internal data ---------------------
  
  // -------------------- GAlib things -----------------------
  // Bounds of the population (Allele set definition)
  GARealAlleleSetArray *bounds;
  // Genetic Algorithm
  GAGeneticAlgorithm *algorithm;
  // Width of the genome (depends on the configuration parameters)
  int genome_width;
  // Folder where all the configuration files are archived
  std::string folder;
  // Number of the current iteration
  int iterations;
  
  //! @brief Disposal of resources (erases contents and restart)
  void dispose () throw ();
  
  //! @brief init without arguments -> all internal data to empty or zero values
  virtual void init();
  
  //! @brief Makes all pointers that are in the class point to NULL.
  void pointersToNULL();
  
  // ----------------- Aux functions -----------------
  
  //! @brief Converts the genome information into flight plans of each UAV
  //! @NOTE The flight plan of a UAV is a vector of Point3D
  //! @param gen The genome to translate
  //! @return A pointer with the Point3D properly constructed. This pointer has to be deleted
  std::vector<simulator::FlightPlan> getGeneticFlightPlan(const GARealGenome &gen) const;
  
  //! @brief Gets the initial length of the flight plan.
  //! @return A vector that contains the length of the flight plan of each UAV.
  std::vector<float> getInitialWaypointLength() const;
  
    //! @brief Checks that the plans that are represented by a genome are vald
  //! @param gen The gen to be checked
  //! @retval true All plans are valid
  //! @retval false There are invalid plans
  bool checkGen(const GARealGenome &gen) const;

  //! @brief Gets the checker that will validate the algorithm input file
  //! @return The checker
  virtual Checker *getAlgorithmChecker() const;
  
  //! @brief Gets the problems bound (useful in waypoint change problems)
  //! @param vec A vector that will get the bounds
  virtual void getBounds(std::vector<double> &upper, std::vector <double> &lower);
  
  //! @brief Translates the content of the class into a ParseBlock in order to write it to a file
  //! @return A pointer to the ParseBlock that has to be freed (note that ParseBlocks that belong to another are automatically freed)
  virtual ParseBlock *toBlock() const;
  
  static functions::RealVector geneToVector(const GARealGenome &gen);
  
  //! @brief Returns the parameters in the GA format according to the configuration data
  GAParameterList getGeneticParams();
  
  //! @brief Sets the crossover, mutation and initialization operators according to the config data
  void setGeneticOperators(GARealGenome& genome);
  
  void setGeneticParameters();
  
  void customEvolution(resolution::CRAlgorithmStatistics& ret, timeval& t1, bool initialize);
  
  friend class CRAlgorithmFactory;
};

}

#endif //__CRALGORITHM_H__
