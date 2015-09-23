#include "operators.h"
#include "functions/functions.h"

using namespace std;
using simulator::FlightPlan;

namespace resolution {

	vector<float> alleleSetToVector(GAAlleleSet<float> set);
void checkGenBounds(GARealGenome &gen);

	
// ---------------- Genetic functions --------------
float CRALgorithmObjective(GAGenome& c)
{
	float objective = 0.0;
	float min_distance;
	
	GARealGenome &gen = (GARealGenome &)c;
	CRAlgorithm *t = (CRAlgorithm*) gen.userData();
	
	if (t == NULL) {
		cerr << "Error: objective --> User data is not configured\n";
		return 0.0;
	} else {
		functions::RealVector v = CRGenetics::geneToVector(gen);
		objective = t->calculateObjectiveVec(v);
	}
	
	return objective;
}

float CRALgorithmOneObjective(GAGenome& c)
{
  float objective = 0.0;
  float min_distance;
  
  GARealGenome &gen = (GARealGenome &)c;
  CRAlgorithm *t = (CRAlgorithm*) gen.userData();
  
  if (t == NULL) {
    cerr << "Error: objective --> User data is not configured\n";
    return 0.0;
  } else {
    functions::RealVector v = CRGenetics::geneToVector(gen);
    objective = t->calculateObjectiveOne(v);
  }
  
  return objective;
}


// For deterministic initialization
void GAPopulationInitializer(GAPopulation& p)
{
	CRGenetics *t = (CRGenetics *)p.userData();
	if (t == NULL) {
		cerr << "Error: GAPopulationInitializer. User data not configured.\n";
		return;
	}
	
	GeneticConfig &config = static_cast<GeneticConfig&>(* (t->config));

	if (config.debug) {
		cout << "GAPopulationInitializer --> Genome width = " << t->genome_width << endl;
		cout << "GAPopulationInitializer --> Population size = " << p.size() << endl;
	}
	int size = p.size();
	
	int i;
  for (i = 0; i < size; i++) {
    GARealGenome &gaux =(GARealGenome &) p.individual(i);
		float *aux = new float[t->genome_width];
		
    for (int j = 0; j < gaux.length(); j++){
			
			if (gaux.alleleset(j).type() == 2) {
				// Non discrete alleleset
				float incre = (gaux.alleleset(j).upper() - gaux.alleleset(j).lower() )/ p.size(); 
			
				if (j % 2 == 1) {
					aux[j] =  gaux.alleleset(j).lower() + incre * i; 
				} else {
					aux[j] =  gaux.alleleset(j).upper() - incre * i; 
				}
			} else {
				// Discrete alleleset--> levels
				aux[j] = gaux.alleleset(j).allele(GARandomInt(0, gaux.alleleset(j).size() - 1));
			}
    }
    gaux = aux;
		
		delete[] aux;
  }
}

void GAGenomeRandomInitializer(GAGenome& g)
{
  GARealGenome &gen = (GARealGenome &)g;
  CRGenetics *t = (CRGenetics *)g.userData();
  
  if (t == NULL) {
    cerr << "Error: GAGenomeRandomInitializer. User data not configured.\n";
    return;
  }
  
  float *au = new float[t->genome_width];
  
  GeneticConfig &c = static_cast<GeneticConfig&>(* (t->config));
//   std::vector<float> waypoint_length = t->getInitialWaypointLength();
  if (c.debug) {
    cout << "GAGenomeRandomInitializer --> Initializing. Upper bounds = ";
    cout << functions::printVector(c.upper_bounds) << endl;
  }
  
//   cout << "Random: ";
  for (int cont = 0; cont < c.lower_bounds.size(); cont++) {
    au[cont] = GARandomFloat(c.lower_bounds.at(cont), c.upper_bounds.at(cont));
//     cout  << au[cont] << ", ";
  }
//   cout << endl;
  
  
  gen = au;

//   checkGenBounds(gen);
  delete[] au;
}

// --------------------------       Variants of the Crossover functions ------------------- ///

int GAGenomeUniformWaypointMixer(const GAGenome& dad, const GAGenome& mom, GAGenome* g1, GAGenome* g2)
{
  GARealGenome &d = (GARealGenome &)dad;
  GARealGenome &m = (GARealGenome &)mom;
  
  GARealGenome *gr1 = (GARealGenome *)g1;
  GARealGenome *gr2 = (GARealGenome *)g2;
  
  CRGenetics *t = (CRGenetics *)(d.userData());
  GeneticConfig &c = dynamic_cast<GeneticConfig&>(*t->config);
  
  float *au = new float[t->genome_width];
  float *au2 = new float[t->genome_width];
  
  GARealGenome &ret1 = *gr1;
  GARealGenome &ret2 = *gr2;
  
  int cont_uav = 0;
  
  bool debug = c.debug;
  
  for (int j = 0; j < t->n_uavs; j++) { 
    
    if (!t->controlled_UAVs[j]) {
	    continue;
    }
    
    
    
    for (int k = 0; k < c.intermediate_waypoints; k++) {
      bool ok = false;
      
      for (int i = 0; i < c.waypoint_dimension; i++) {
	// Calculate the index of the gen to modify
	int index = (cont_uav * c.intermediate_waypoints + k)  * c.waypoint_dimension + i; 
    
	if (index >= t->genome_width) {
	  cerr << "Warning: GAGenomeUniformWaypointMixer --> index exceeded genome_width" << endl;
	  continue;
	}
  
	// The coordinate of the waypoint has to lie between the coordinates of the two parents
	au[index] = GARandomFloat(d.gene(index), m.gene(index));
	au2[index] = GARandomFloat(d.gene(index), m.gene(index));
    
      }
    }
    cont_uav++;
  }
  
  ret1 = au;
  ret2 = au2;
  
  delete[] au;
  delete[] au2;
  
  if ( debug ) {
	  cout << "GAGenomeUniformWaypointMixer INIT --> Ret1 contents: " << geneToString(ret1) << "\t Ret2 = "<< geneToString(ret2) << endl;
  }
  
  return 2;
}

int GARealUniformCrossoverCheck(const GAGenome& a, const GAGenome& b,
				  GAGenome* c, GAGenome* d) {
  int ret;
	bool ok = false;
	CRGenetics *t = (CRGenetics *)a.userData();
	GeneticConfig &conf = static_cast<GeneticConfig&>(* (t->config));
	int max_checks = conf.max_checks;
	
	GARealGenome *h1 = (GARealGenome *)c;
	GARealGenome *h2 = (GARealGenome *)d;
	do {
		ret = GA1DArrayGenome<float>::UniformCrossover(a,b,c,d);
		
		GARealGenome *h1 = (GARealGenome *)c;
		GARealGenome *h2 = (GARealGenome *)d;
		
		if (h1 != NULL) {
			ok = t->checkGen(*h1);
		}
		if (h2 != NULL && ok) {
			ok &= t->checkGen(*h2);
		}
		
		max_checks--;
	} while (!ok && max_checks >= 0);
	
	if (!ok) {
		ret = 0;
	}
	
	return ret;
}


// ---------- Mutators ---------------------

int GARealGaussianMaMutator(GAGenome &gen, float pmut) {
	GARealGenome &g = (GARealGenome &)gen;
	CRGenetics *t = (CRGenetics *)gen.userData();
	GeneticConfig &c = static_cast<GeneticConfig&>(* (t->config));
	bool debug = c.debug;
	
	int muts = 0;
	
	if (pmut < 0) 
		return muts;
	
	pmut = (pmut > 1)?1:pmut;
	
// 	pmut = 1; 
	if ( debug ) {
		cout << "GARealGaussianMaMutator INIT --> Initial gen contents: " << geneToString(g) << "\t pmut = "<< pmut << endl;
	}
	
	for (int i = 0; i < g.size(); i++) {
		if (GAFlipCoin(pmut)) {
			muts++;
		
		
			if (g.alleleset(i).type() == GAAllele::BOUNDED) {
				g.gene(i, g.gene(i) + (GARandomFloat() - 0.5));
			} else {
				float aux = GARandomFloat();
				g.gene(i, g.alleleset(i).allele(GARandomInt(0, g.alleleset(i).size() - 1)));
			}
		}
	}
	
	if (debug) {
		cout << "GARealGaussianMaMutator --> RetValue = " << muts << endl;
				cout << geneToString(g) << endl;
	}
	
	return muts;
}

int 
GARealGaussianMutatorCheck(GAGenome& g, float pmut){
	int ret = 0;
	bool ok;
	
	
	
	GARealGenome &gen = (GARealGenome &)g;
	CRGenetics *t = (CRGenetics *)g.userData();
	GeneticConfig &c = static_cast<GeneticConfig&>(* (t->config));
	bool debug = c.debug;
	
	if ( debug ) {
		cout << "GARealGaussianMaMutator INIT --> Initial gen contents: " << geneToString(gen) << "\t pmut = "<< pmut << endl;
	}
	
	int check_counter = c.max_checks;
	do {
		for (int i = 0; i < gen.size(); i++) {
		  if (GARandomInt() < pmut) {
		    gen[i] += GAGaussianFloat(c.mutation_dev);
		    ret++;
		  }
		}
		ok = t->checkGen(gen);
		check_counter--;
	} while (!ok && check_counter >= 0);
	
	checkGenBounds(gen);
	
	if (!ok) {
		ret = 0;
	}
	
	if (debug) {
		cout << "GARealGaussianMaMutator --> RetValue = " << ret << endl;
				cout << geneToString(gen) << endl;
	}
	
	return ret;
}


// ---------------- Aux Functions --------------------------

int getAlleleNumber(float value, const GARealAlleleSet &set) {
	int ret_val = 0;
	bool located = false;
	
	for ( ; ret_val < set.size() && !located; ret_val++) {
		
	}
	
	if (located) {
		ret_val--;
	} else {
		ret_val = -1;
	}
	
}

vector<float> alleleSetToVector(GAAlleleSet<float> set) {
	vector<float> ret;
	
	for (unsigned int i = 0; i < set.size(); i++) {
		ret.push_back(set.allele(i));
	}
	
	return ret;
}

std::string geneToString(const GARealGenome& gen)
{
	ostringstream os;
	
	for (unsigned int i = 0; i < gen.size(); i++) {
		os << gen.gene(i) << "\t";
	}
	
	return os.str();
}

void checkGenBounds(GARealGenome &gen) {
	for (int i = 0; i < gen.size(); i++) {
		if (gen.alleleset(i).type() == GAAllele::BOUNDED) {
			gen.gene(i, functions::minimum<float>( gen.gene(i), gen.alleleset(i).upper()));
			gen.gene(i, functions::maximum<float>( gen.gene(i), gen.alleleset(i).lower()));
		}
		
	}
	
}

} // namespace resolution