#include "PFNormalSeed.h"
#include "ParticleFactory.h"

using namespace std;

namespace simulator {

PFNormalSeed::PFNormalSeed(){
	init(mT,me,sd);
	
}

PFNormalSeed::PFNormalSeed(const std::string &modelFile, const std::string &icFile) throw(std::runtime_error){
	string mT;
	vector < vector <double> > me;
	vector < vector <double> > sd;
	
	vector< double > stateMean;
	vector< double > stateDev;
	vector< double > controlMean;
	vector< double > controlDev;
	vector< double > paramMean;
	vector< double > paramDev;
	
	try {
		ParseBlock icData,modelData;
		Checker *icChecker=ICFileChecker();
		Checker *modelChecker=modelFileChecker();
		icData.load(icFile.c_str());
		icData.checkUsing(icChecker);
		stateMean.push_back(icData("x").as<double>() );
		stateMean.push_back(icData("y").as<double>() );
		stateMean.push_back(icData("v").as<double>() );
		stateMean.push_back(icData("phi").as<double>() );
		stateMean.push_back(icData("h").as<double>() );
		stateMean.push_back(icData("h_dot").as<double>() );
		stateMean.push_back(icData("ws_me").as<double>() );
		stateMean.push_back(icData("ws_sd").as<double>() );
		stateMean.push_back(icData("wd_me").as<double>() );
		stateMean.push_back(icData("wd_sd").as<double>() );
		
		stateDev.push_back(icData("dx").as<double>() );
		stateDev.push_back(icData("dy").as<double>() );
		stateDev.push_back(icData("dv").as<double>() );
		stateDev.push_back(icData("dphi").as<double>() );
		stateDev.push_back(icData("dh").as<double>() );
		stateDev.push_back(icData("dh_dot").as<double>() );
		stateDev.push_back(icData("dws_me").as<double>() );
		stateDev.push_back(icData("dws_sd").as<double>() );
		stateDev.push_back(icData("dwd_me").as<double>() );
		stateDev.push_back(icData("dwd_sd").as<double>() );

		controlMean.push_back(icData("cv").as<double>() );
		controlMean.push_back(icData("cphi").as<double>() );
		controlMean.push_back(icData("ch").as<double>() );
		controlDev.push_back(icData("dcv").as<double>() );
		controlDev.push_back(icData("dcphi").as<double>() );
		controlDev.push_back(icData("dch").as<double>() );
		modelData.load(modelFile.c_str());
		modelData.checkUsing(modelChecker);
		mT = modelData("name").as<string>();
		
		
		paramMean.push_back(modelData("alpha_v").as<double>() );
		paramMean.push_back(modelData("alpha_a").as<double>() );
		paramMean.push_back(modelData("alpha_h").as<double>() );
		paramMean.push_back(modelData("alpha_h_dot").as<double>() );
		paramMean.push_back(modelData("c").as<double>() );
		paramMean.push_back(modelData("v_min").as<double>() );
		paramMean.push_back(modelData("v_max").as<double>() );
		
		paramDev.push_back(modelData("dalpha_v").as<double>() );
		paramDev.push_back(modelData("dalpha_a").as<double>() );
		paramDev.push_back(modelData("dalpha_h").as<double>() );
		paramDev.push_back(modelData("dalpha_h_dot").as<double>() );
		paramDev.push_back(modelData("dc").as<double>() );
		paramDev.push_back(modelData("dv_min").as<double>() );
		paramDev.push_back(modelData("dv_max").as<double>() );
		
		
	} catch (std::runtime_error &e) {
		cout << "PFNormalSeed::PFNormalSeed--> Error while loading data from file: ";
		cout << e.what() << endl;
		throw(e);
	}
	
	me.push_back(stateMean);
	me.push_back(paramMean);
	me.push_back(controlMean);
	sd.push_back(stateDev);
	sd.push_back(paramDev);
	sd.push_back(controlDev);
	
	
	init(mT,me,sd);
	
}

Checker *PFNormalSeed::modelFileChecker() {

	
	Checker *checker = new Checker;
	checker->addProperty("name", new NTimes(1) );
	checker->addProperty("v_min", new NTimes(1) );
	checker->addProperty("v_max", new NTimes(1) );
	checker->addProperty("c", new NTimes(1) );
	checker->addProperty("alpha_v", new NTimes(1) );
	checker->addProperty("alpha_a", new NTimes(1) );
	checker->addProperty("alpha_h", new NTimes(1) );
	checker->addProperty("alpha_h_dot", new NTimes(1) );
	checker->addProperty("dv_min", new NTimes(1) );
	checker->addProperty("dv_max", new NTimes(1) );
	checker->addProperty("dc", new NTimes(1) );
	checker->addProperty("dalpha_v", new NTimes(1) );
	checker->addProperty("dalpha_a", new NTimes(1) );
	checker->addProperty("dalpha_h", new NTimes(1) );
	checker->addProperty("dalpha_h_dot", new NTimes(1) );
	
	return checker;
}

Checker *PFNormalSeed::ICFileChecker() {
  // Initials conditions for ModelSimpleUAVWind
  Checker *checker = new Checker;
  
  // Position data
  checker->addProperty("x", new NTimes(1) );
  checker->addProperty("y", new NTimes(1) );
  checker->addProperty("v", new NTimes(1) );
  checker->addProperty("phi", new NTimes(1) );
  checker->addProperty("h", new NTimes(1) );
  checker->addProperty("h_dot", new NTimes(1) );
  checker->addProperty("ws_me", new NTimes(1) );
  checker->addProperty("ws_sd", new NTimes(1) );
  checker->addProperty("wd_me", new NTimes(1) );
  checker->addProperty("wd_sd", new NTimes(1) );
  
  // Derivative data
  checker->addProperty("dx", new NTimes(1) );
  checker->addProperty("dy", new NTimes(1) );
  checker->addProperty("dv", new NTimes(1) );
  checker->addProperty("dphi", new NTimes(1) );
  checker->addProperty("dh", new NTimes(1) );
  checker->addProperty("dh_dot", new NTimes(1) );
  checker->addProperty("dws_me", new NTimes(1) );
  checker->addProperty("dws_sd", new NTimes(1) );
  checker->addProperty("dwd_me", new NTimes(1) );
  checker->addProperty("dwd_sd", new NTimes(1) );
  
  // Limits
  checker->addProperty("cv", new NTimes(1) );
  checker->addProperty("cphi", new NTimes(1) );
  checker->addProperty("ch", new NTimes(1) );
  
  checker->addProperty("dcv", new NTimes(1) );
  checker->addProperty("dcphi", new NTimes(1) );
  checker->addProperty("dch", new NTimes(1) );
  
  return checker;
}


PFNormalSeed::~PFNormalSeed(){
	dispose();
}



void PFNormalSeed::init(){
	
	modelType = NULL;
	mean = NULL;
	sigma = NULL;
	
}

void PFNormalSeed::init(const std::string &mT, const std::vector< std::vector<double> > &me , const std::vector< std::vector<double> > &sd){
	modelType = new string(mT);
	mean = new vector < vector <double> >(me.size());
	sigma = new vector < vector <double> > (sd.size());
	for (uint i=0; i< me.size(); i++){
		(*mean) [i] = me[i];
	}
	for (uint i=0; i< sd.size(); i++){
		(*sigma) [i] = sd[i];
	}
	//!random number generator
	generator= new boost::mt19937;
	normal_dist_state = new vector < boost::normal_distribution <> *>();
	normal_dist_control = new vector < boost::normal_distribution <> *>();
	normal_dist_parameter = new vector < boost::normal_distribution <> *>();
	for (uint i = 0; i < ((*mean)[0]).size(); i++){
		normal_dist_state->push_back ( new boost::normal_distribution<>(me[0][i],sd[0][i]) );
	}
	for (uint i = 0; i < ((*mean)[1]).size(); i++){
		normal_dist_parameter->push_back ( new boost::normal_distribution<>(me[1][i],sd[1][i]) );
	}
	for (uint i = 0; i < ((*mean)[2]).size(); i++){
		normal_dist_control->push_back ( new boost::normal_distribution<>(me[2][i],sd[2][i]) );
	}
}

void PFNormalSeed::dispose () throw () {
	delete modelType;
	delete mean;
	delete sigma;
	delete generator;
	if(normal_dist_state){
		vector < boost::normal_distribution <> *>:: iterator it;
		for(it=normal_dist_state->begin(); it!=normal_dist_state->end(); it++ ){
			delete (*it);
		}
		delete normal_dist_state;
	}
	if(normal_dist_control){
		vector < boost::normal_distribution <> *>:: iterator it;
		for(it=normal_dist_control->begin(); it!=normal_dist_control->end(); it++ ){
			delete (*it);
		}
		delete normal_dist_control;
	}
	if(normal_dist_parameter){
		vector < boost::normal_distribution <> *>:: iterator it;
		for(it=normal_dist_parameter->begin(); it!=normal_dist_parameter->end(); it++ ){
			delete (*it);
		}
		delete normal_dist_parameter;
	}
	init();
}
		
PFNormalSeed::PFNormalSeed(const PFNormalSeed &that )
{
	init( *(that.modelType), *(that.mean), *(that.sigma));
};

PFNormalSeed &PFNormalSeed::operator = (const PFNormalSeed &that){
	if (this == &that) {
		return *this;
	}
	dispose(); 
	init( *(that.modelType), *(that.mean), *(that.sigma) );
	return *this;
}

PFNormalSeed *PFNormalSeed::clone () const { 
  return NULL;
// 	return new PFNormalSeed ( *modelType, *mean, *sigma);
}


std::vector< Particle *> PFNormalSeed::getParticlesList(const int num ) {
	double weight = 1.0/num;
	
	vector<double> ic;
	
	//Preparing the distributions
	vector < Particle *> li;
	vector < double > aux_ic(normal_dist_state->size());
	for (int i = 0; i< num; i++){
		for( uint j = 0; j < normal_dist_state->size(); j++ ){
			norm_rand aux(norm_rand( *generator, * ( (*normal_dist_state)[j] ) ) );
			aux_ic[j] = aux();
		}
		ParticleFactory pf;
		li.push_back(pf.createFromBlock(data));
	}
	
	return li;
}

std::string PFNormalSeed::toString() {
	using std::endl;
	std::ostringstream os;
	os << "PFNormalSeed: "<< endl;
	os << " Initialization values (mean, std deviation):";
	vector <string> title(3);
	title[0] = " State:";
	title[1] = " Control:";
	title[2] = " Parameter:";
	for ( uint i =0 ; i < 3 ; i++ ){
		os  << title[i] << endl;
		
		for ( uint j = 0; j < ( (*mean)[i] ).size() ; j++ ){
			os << " (" << (*mean)[i][j]<< ", " << (*sigma)[i][j]<<")\t";
			
		}
		os << endl;
	}
	os << endl;
	
	return os.str();
}

}
