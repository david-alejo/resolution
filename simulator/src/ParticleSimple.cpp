#include "ParticleSimple.h"
#include "functions/functions.h"
#include <sstream>

using std::ostringstream;

namespace simulator {

const std::string ParticleSimple::particleType = "ParticleSimple";


Particle* ParticleSimple::create(const simulator::Controller& con, const simulator::Model& mod, double w) const{
  ParticleSimple *ret =new ParticleSimple();
  ret->init(con, mod, w);  
  return ret;
}

ParticleSimple::ParticleSimple() {
	init();
}

void ParticleSimple::pointersToNULL(){
  model = NULL;
  controller = NULL;
}


Particle* ParticleSimple::createFromFile(const std::string& fileName) const {
  Particle * ret = NULL;
//   string route= aux::extract_route_path(fileName);
  try {
		ParseBlock particleData;
		particleData.load(fileName.c_str());
		ret = createFromBlock(particleData);
	} catch (std::runtime_error &e) {
		std::cout << "ParticleSimple::create_from_file --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
  
  return ret;
}

Particle* ParticleSimple::createFromBlock(ParseBlock& particleData) const
{
  double w;
  ParticleSimple * ret = new ParticleSimple();
	string controller_type, model_type;
	
	Checker *particleChecker=particleFileChecker();
  try {
		particleData.checkUsing(particleChecker);
		
		// Creating model
		ModelFactory mFac;
		Model *model = NULL;
		model_type = particleData["model"]("model_type").as<string>();
		if (particleData["model"].hasProperty("model_file")) {
			string f_model = particleData["model"]("model_file").as<string>();
			model = mFac.create_from_file(model_type, f_model);
		} else {
			model = mFac.create_from_block(model_type, particleData["model"]);
		}
		w = particleData("weight").as<double>();
		
		// Creating controller
		ControllerFactory cFac;
		Controller *control = NULL;
		controller_type = particleData["controller"]("controller_type").as<string>();
		if (particleData["controller"].hasProperty("controller_file")) {
			string f_controller = particleData["controller"]("controller_file").as<string>();
			control = cFac.createFromFile(controller_type, f_controller);
		} else {
			control = cFac.createFromBlock(controller_type, particleData["controller"]);
		}
		control->setT(model->getTimeStep());
		
		if (model != NULL && control != NULL) {
			ret->init(*control, *model, w);
		} else {
			std::runtime_error e("ParticleSimple::createFromBlock() --> Error while creating the model and the controller.\n");
			throw(e);
		}
		
		delete model;
		delete control;
		
		// Check if the geometry has been defined
		if ( particleData.hasProperty("geometry") ) {
			ret->geometry = particleData("geometry").as<std::vector<double> >();
// 			std::cout << "Geometry: " <<  particleData("geometry").value << std::endl;
		} else {
		  ret->geometry.clear();
		}
		ret->stoppable = true;
		if (particleData.hasProperty("stoppable")) {
		  ret->stoppable = particleData("stoppable").as<bool>();
		}
		
		
		
	} catch (std::runtime_error &e) {
		std::cerr << "ParticleSimple::create_from_block --> Error while loading data from block: ";
		std::cerr << e.what() << std::endl;
		throw(e);
	}
	delete particleChecker;
  
  return ret;
}


Checker *ParticleSimple::particleFileChecker() const {
	Checker *model_checker = new Checker;
	model_checker->addProperty("model_type", new NTimes(1) );
	
	Checker *controller_checker = new Checker;
	controller_checker->addProperty("controller_type", new NTimes(1) );
	
	Checker *particle_checker = new Checker;
	particle_checker->addBlock("model", new NTimes(1) );
	particle_checker->addChecker("model", model_checker);
	particle_checker->addBlock("controller", new NTimes(1) );
	particle_checker->addChecker("controller", controller_checker);
	particle_checker->addProperty("weight", new NTimes(1) );
	
	return particle_checker;
}


void ParticleSimple::init(const Controller& fms, const Model& mod, double w ){
  
  controller =fms.clone();
  model = mod.clone();
  if(!controller || !model){
    throw( std::runtime_error ("ParticleSimple initialization error") );
    
  }
    
  stopped = false;
  weight = w;
//   controller->initialize(model->getState());
  
}



void ParticleSimple::init(){
  controller = NULL;
  model = NULL;
  weight = 0.0;
  stopped = false;
  geometry.clear();
}

void ParticleSimple::dispose () {
  delete model;
	model = NULL;
  delete controller;
	controller = NULL;
	geometry.clear();
// 	std::cout  << "ParticleSimple::dispose --> end.\n";
}

std::vector<double> ParticleSimple::getState() const {
  return model->getState();
} 

void ParticleSimple::setState(const std::vector< double >& new_state)
{
	if (new_state.size() == model->getState().size()) {
		model->setState(new_state);
	}
}


int ParticleSimple::runOneStep (){
  int ret_val = ParticleSimple::NORMAL;
  if(!stopped){
    controller->update(model->getParameter(),model->getState(),model->getControl());
    std::vector <double> aux; 
    aux = controller->getParameterVector();
    if (!aux.empty()){
      model->setParameter(aux);
    }
    aux = controller->getControlVector();
    if (!aux.empty()){ 
      model->setControl(aux);
    }
    model->runOneStep();
    if( controller->hasReachedFinalWaypoint()){
	stopped = stoppable;
	ret_val = Particle::LAST_WAYPOINT_REACHED;
    }
  } else {
    ret_val = Particle::LAST_WAYPOINT_REACHED;
  }
  
  return ret_val;
}

Model* ParticleSimple::getModel() {
  return model;
} 

void ParticleSimple::setModel (Model *pModel){
  delete model;
  
   model = pModel;
 
 }

void ParticleSimple::specialFeature(void **pointer, const char *type ){
  return model->specialFeature(pointer,type);
}


ParticleSimple *ParticleSimple::clone() const {
  ParticleSimple *ret = new ParticleSimple();
	
  ret->stopped = stopped;
  ret->stoppable = stoppable;
	
  if (model != NULL) 
		ret->model = model->clone();
	else 
		ret->model = NULL;
	
	if (controller != NULL) {
		ret->controller = controller->clone();
	} else {
		ret->controller = NULL;
	}
	
  ret->weight = weight;
  ret->geometry = geometry;
	
  return ret;
}
void ParticleSimple::stopParticle(){
  stopped=true;
}
void ParticleSimple::resumeParticle(){
  stopped=false; 
}
ParticleSimple::~ParticleSimple(){
  dispose();
}
string ParticleSimple::toString() const
{
	ostringstream os;
	
	os << "Particle state: ";
	
	if ( stopped ) {
		os << "Stopped";
	} else{
		os << "Running";
	}
	
	os << "\tParticle type: " << particleType;
	os << "\nModel: " << model->toString();
	os << "\nController: " << controller->toString();
	
	return os.str();
}

ParseBlock* ParticleSimple::toBlock()
{
  ParseBlock *ret_val = new ParseBlock;
  
  // Put blocks
  ret_val->setBlock("controller", controller->toBlock());
  ret_val->setBlock("model", model->toBlock() );
  
  // Then properties
  ret_val->setProperty("geometry", functions::printVector(geometry));
  ostringstream s_weight;
  s_weight << weight;
  ret_val->setProperty("weight", s_weight.str());
  ret_val->setProperty("particle_type", getType());
  
  return ret_val;
}

Controller* ParticleSimple::getController()
{
	return controller;
}

void ParticleSimple::setController(Controller* nCon)
{
  delete controller;
  controller = nCon;
}


}
