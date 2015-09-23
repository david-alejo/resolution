#include "GeneticConfig.h"
#include "functions/functions.h"

using namespace functions;

namespace resolution {
	AlgorithmConfig *GeneticConfig::createAlgorithmConfig(ParseBlock& block)
{
	AlgorithmConfig *ret = new GeneticConfig();
	ret->init(block);
	
	return ret;
}

void GeneticConfig::init()
{
	CostConfig::init();
	
	// Default values
	initializer_type = "Random";
	crossover_type = "OnePoint";
	mutator_type = "Gaussian";
	custom_evolution = true;
	search_ratio = 1.0;
	max_checks = 50;
	pMutation = 0.05;
	mutation_dev = 1.0;
	pCrossover = 0.8;
}

	
void GeneticConfig::init(ParseBlock& block)
{
	Checker *check = getChecker();
	init();
	try {
		block.checkUsing(check);
    resolution::CostConfig::init(block);
		
		

		if (block.hasProperty("initializer_type")) {
			initializer_type = block("initializer_type").value;
		}
		if (block.hasProperty("crossover_type")) {
			crossover_type = block("crossover_type").as<string>();
		}
		if (block.hasProperty("custom_evolution")) {
			custom_evolution = block("custom_evolution").as<bool>();
		}
		
		if (block.hasProperty("mutator_type")) {
			mutator_type = block("mutator_type").as<string>();
		}
		if (block.hasProperty("search_ratio")) {
			search_ratio = block("search_ratio").as<double>();
		}
		
		if (block.hasProperty("max_checks")) {
			max_checks = block("max_checks").as<int>();
		}
		
		if (block.hasProperty("mutation_probability")) {
		  pMutation = block("mutation_probability").as<double>();
		}
		
		if (block.hasProperty("mutation_deviation")) {
		  mutation_dev = block("mutation_deviation").as<double>();
		}
		
		if (block.hasProperty("crossover_probability")) {
		  pCrossover = block("crossover_probability").as<double>();
		}
		
	} catch (std::exception &e) {
		std::cerr << "GeneticConfig::init() --> error while loading data from block\n";
		throw(e);
	}
	delete check;
}

Checker* GeneticConfig::getChecker()
{
	Checker *ret = new Checker;

	return ret;
}

void GeneticConfig::dispose() {
  CostConfig::dispose();
  init();
  
}

ParseBlock* GeneticConfig::toBlock() const
{
    ParseBlock *ret = resolution::CostConfig::toBlock();
    
    ret->setProperty("initializer_type", initializer_type);
    ret->setProperty("crossover_type", crossover_type);
    ret->setProperty("mutator_type", mutator_type);
    ret->setProperty("custom_evolution", boolToString(custom_evolution));
    ret->setProperty("search_ratio", numberToString(search_ratio));
    ret->setProperty("max_checks", numberToString(max_checks));
    ret->setProperty("mutation_probability", numberToString(pMutation));
    ret->setProperty("crossover_probability", numberToString(pCrossover));
    ret->setProperty("mutation_deviation", numberToString(mutation_dev));
    
    return ret;
}


} // namespace resolution