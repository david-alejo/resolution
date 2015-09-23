#include "SwarmConfig.h"
#include "functions/functions.h"

namespace resolution {
	AlgorithmConfig *SwarmConfig::createAlgorithmConfig(ParseBlock& block)
{
	AlgorithmConfig *ret = new SwarmConfig();
	ret->init(block);
	
	return ret;
}

void SwarmConfig::init()
{
	CostConfig::init();
	search_ratio = 1.0;
	max_checks = 50;
	r0 = 0.1;
	phi0 = 0.9;
	inertia_weight = 1.0;
	upper_bounds.clear();
	lower_bounds.clear();
	speed_bounds.clear();
}

	
void SwarmConfig::init(ParseBlock& block)
{
	Checker *check = getChecker();
	init();
	try {
		block.checkUsing(check);
		resolution::CostConfig::init(block);
		
		if (block.hasProperty("initializer_type")) {
			initializer_type = block("initializer_type").value;
		}
		if (block.hasProperty("search_ratio")) {
			search_ratio = block("search_ratio").as<double>();
		}
		if (block.hasProperty("phi0")) {
			phi0 = block("phi0").as<double>();
		}
		if (block.hasProperty("r0")) {
			r0 = block("r0").as<double>();
		}
		if (block.hasProperty("inertia_weight")) {
		  inertia_weight = block("inertia_weight").as<double>();
		}
		
	} catch (std::runtime_error &e) {
		std::cerr << "SwarmConfig::init() --> error while loading data from block. Content: " << e.what() << std::endl;
		throw(e);
	}
	delete check;
}

Checker* SwarmConfig::getChecker()
{
	Checker *ret = new Checker;

	return ret;
}

SwarmConfig::~SwarmConfig()
{
	dispose();
}

void SwarmConfig::dispose()
{
  CostConfig::dispose();
  init();
	
}

ParseBlock* SwarmConfig::toBlock() const
{
    ParseBlock *ret = resolution::CostConfig::toBlock();
    
    ret->setProperty("initializer_type", initializer_type);
    ret->setProperty("search_ratio", functions::numberToString(search_ratio));
    ret->setProperty("r0", functions::numberToString(r0));
    ret->setProperty("phi0", functions::numberToString(phi0));
    ret->setProperty("inertia_weight", functions::numberToString(inertia_weight));
    
    return ret;
}



} // namespace resolution
