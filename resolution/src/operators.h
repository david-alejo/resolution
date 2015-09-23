#include "CRGenetics.h"
#include <ga/GARealGenome.h>
#include <ga/GASelector.h>

namespace resolution {

//! @brief Represents the info of one gene to a string
//! @param gen The gene to represent
//! @return String representing the genome
std::string geneToString(const GARealGenome &gen);

//! @brief Calculates the fitness of a genome. I have not managed to include this in CRAlgorithm class.
//! \NOTE It is necessary that the user data of the genome contain the pointer to the desired CRAlgorithm class.
//! @param c Target genome 
//! @return A float value meaning the objetive function. Less objetive --> better genome
float CRALgorithmObjective(GAGenome &c);

//! @brief Calculates the fitness of a genome when performing the CA maneuver with the other UAVs as static obstacles
//! It behaves in a similar fashion as the algorithm explained in ICRA 2011.
//! \NOTE It is necessary that the user data of the genome contain the pointer to the desired CRAlgorithm class.
//! @param c Target genome 
//! @return A float value meaning the objetive function. Less objetive --> better genome
float CRALgorithmOneObjective(GAGenome &c);

//! @brief Initialize the population in order to sample the genome space sampling the hyperdiagonal.
//! @BRIEF Same indications that objetive function
//! @param p The population to be initialized
void GAPopulationInitializer(GAPopulation &p);

//! @brief Stocastic generator. Generates intermediate waypoints with a gaussian distribution centered in
//! @brief the midpoint of init & goal points. With a determinate(config file) std deviation.
//! @param g The genome to be initialized
void GAGenomeRandomInitializer(GAGenome& g);

//! @brief Mixes each pair of waypoints of the parents taking a point of the segment that joins those WPs.
//! @brief Uses uniform distribution on this segment.
//! @param dad One parent
//! @param mom The other parent
//! @param g1 Pointer to one children
//! @param g1 Pointer to the other children
int GAGenomeUniformWaypointMixer(const GAGenome& dad, const GAGenome& mom, GAGenome* g1, GAGenome* g2);

//! @brief Performs a uniform crossover between two genes checking the validity of all resulting plans
//! @brief Uses uniform distribution on this segment.
//! @param dad One parent
//! @param mom The other parent
//! @param g1 Pointer to one children
//! @param g1 Pointer to the other children
int GARealUniformCrossoverCheck(const GAGenome& a, const GAGenome& b,
				  GAGenome* c, GAGenome* d) ;

int GARealGaussianMaMutator(GAGenome &, float);

int GARealGaussianMutatorCheck(GAGenome& g, float pmut);

int getAlleleNumber(float value, const GARealAlleleSet &set);

}