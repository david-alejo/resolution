#include "ParticleFilter.h"
#include "ParticleFactory.h"

//TODO: see ParticleFilter.h

#include <functions/functions.h>

using functions::RandomNumberGenerator;

namespace simulator {

ParticleFilter::ParticleFilter(ParseBlock &data):max_dev(0.0){
  
  Checker *check = getChecker();
  try {
    data.checkUsing(check);
    particlesList = new vector < Particle * >();
    Nth = data("Nth").as<int>();
    init(data["uav"], data("particles").as<int>());
  } catch (exception &e) {
    cerr << "Catched an exception while creating the particle filter. Content: " << e.what() << "\n";
    delete check;
    throw(e);
  }
  delete check;
}

ParticleFilter::ParticleFilter(ParseBlock& uav_data, int n_particles, double N_th):Nth(N_th),max_dev(0.0)
{
  Nth = 0.0;
  try {
    particlesList = new vector < Particle * >();
    init(uav_data, n_particles);
  } catch (exception &e) {
    cerr << "Catched an exception while creating the particle filter. Content: " << e.what() << "\n";
  }
}

ParticleFilter::ParticleFilter(const Particle* particle, int n_particles, double N_th):Nth(N_th)
{
  particlesList = new vector < Particle * >();
  init(particle, n_particles);
}


void ParticleFilter::init(const Particle* particle, int n_particles)
{
  eraseParticlesList();
  particlesList = new vector < Particle *>;
  max_dev = 0.0;
  
  for (unsigned int i = 0; i < n_particles; i++) {
    particlesList->push_back(particle->clone());
  }
}


void ParticleFilter::init(ParseBlock& data, int n_particles)
{
  max_dev = 0.0;
  mean_traj.clear();
  initializeParticles(data, n_particles);
}


Checker* ParticleFilter::getChecker()
{
  Checker *ret = new Checker();
  
  ret->addBlock("uav", new NTimes(1));
  ret->addProperty("particles", new NTimes(1));
  ret->addProperty("Nth", new NTimes(1));
  
  return ret;
}


void ParticleFilter::initializeParticles(ParseBlock& b, unsigned int n_particles)
{
  ParticleFactory pf;
  
  eraseParticlesList();
  particlesList = new vector < Particle *>;
  
  for (unsigned int i = 0; i < n_particles; i++) {
    particlesList->push_back(pf.createFromBlock(b));
  }
}


int ParticleFilter::runOneStep(){
	
  vector< Particle *>::iterator it;
  int ret = 1, act = 1;
  
  int cont = 0;
  functions::RealVector mean;
  double curr_max_dev = 0.0;
  
  //! Step the alive particles
  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
    act = (*it)->runOneStep();
    ret = (act == 0) ? 0 : ret; // If any particle is alive --> keep going
    if (cont == 0) {
      mean = (*it)->getState();
    } else {
      mean = mean + (*it)->getState();
    }
    cont++;
  }
  
  mean = mean / cont;
  
  //! Get the max deviation position
  double aux;
  for(it = particlesList->begin(); it != particlesList->end(); it++ ) {
    functions::RealVector a((*it)->getState());
    functions::Point3D av1(a.at(0), a.at(1), a.at(2));
    functions::Point3D b(mean.at(0), mean.at(1), mean.at(2));
    aux = b.distance(av1);
    curr_max_dev = functions::maximum(aux, curr_max_dev);
  }
  
  max_dev = functions::maximum(curr_max_dev, max_dev);
  mean_traj.push_back(mean);
  
  return ret;
}

ParticleFilter::~ParticleFilter(){
	eraseParticlesList();
}

void ParticleFilter::eraseParticlesList(){
	if (particlesList != NULL) {
	  vector< Particle *>::iterator it;
	  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
	    delete( *it );
	  }
	}
	delete particlesList;
	particlesList = NULL;
}

vector< vector<double> *> ParticleFilter::getAllParticlesStates() const {
  vector< vector<double> *> states;
  vector< Particle *>::iterator it;
  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
    if ( (*it)->isStopped() ){
      states.push_back( NULL );
    } else {
      states.push_back( new vector<double> ((*it)->getState()) );
    }
  }
  
  return states;
}

void ParticleFilter::setControl(const std::vector<double> &control){
  vector< Particle *>::iterator it;
  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
    (*it)->getModel()->setControl(control);
  }
	  
}

void ParticleFilter::calculateWeights(){
  calculateWeights((*particlesList)[0]->getModel()->getState()); 
}

void ParticleFilter::calculateWeights(const std::vector <double> &ref){
  vector< Particle *>::iterator it;
  double den, weightSum = 0.0;
  vector <double> auxState;
  //Calculating weights using euclidean distance of the 2 first statesand assuming proposal=prediction function
  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
    auxState = ((*it)->getModel())->getState();
    den= sqrt( (auxState[0]-ref[0]) * (auxState[0]-ref[0]) +
      (auxState[1]-ref[1]) * (auxState[1]-ref[1]) );
    if (den==0.0) {
      den=1.0e-8;
    }
    (*it)->setWeight ( (*it)->getWeight()*(1.0/den) );
    weightSum += (*it)->getWeight();
  }	
  
  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
    (*it)->setWeight ( (*it)->getWeight() /weightSum );
  }
			
	/*TODO:Consider using normal pdf represnting the real p(z|x) :
	%Weight function is p(z|x)
	if isempty(traj)
	w_xya=normpdf([particle{q}(end,1) particle{q}(end,2) particle{q}(end,4)],[particle{1}(end,1) particle{1}(end,2) particle{1}(end,4)],sens_sig);
	else
	w_xya=normpdf([particle{q}(end,1) particle{q}(end,2)],[traj(minN,1) traj(minN,2)],sens_sig(1:2));
	end
	w(q)=w(q)*(prod(w_xya));
    
	wpsum= wpsum + w(q);
	end

	%normalizing
	w=w/wpsum;
	*/
	
  //Calculating N_eff
  double N_eff,N_eff_inv = 0.0;
  for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
    N_eff_inv +=  (*it)->getWeight() * (*it)->getWeight();
  }
  
  N_eff = N_eff_inv==0.0?10e10:1.0/N_eff_inv;
  
  //Normalizing Nth
  double N_th = Nth * particlesList->size();
  
  if (N_eff < N_th){
    //NOTE: commented for debug
// 		resample(ref);
  }
}

void ParticleFilter::resample(const std::vector <double> &ref){
  //function [particle pset wind_n w]=resamplingAndSim(num_par,particle,wp,wind_n,w,meas_time,Th,obstacles,traj)

  double rndNum,u;
  RandomNumberGenerator r;
  rndNum = r.getRandomNumber();
  
  const double numberOfParticles = particlesList->size();	
  double m_1=1.0/numberOfParticles;	
  rndNum *= m_1;
  vector < Particle *> *newParts = new vector < Particle *>;
  double c =  (*particlesList)[0]->getWeight();
  unsigned int i= 0;
  for (int m=0; m< numberOfParticles;m++) {
    u = rndNum + m*m_1;
    while( u > c){
      i++;
      c= c + (*particlesList)[i]->getWeight(); 
    }
    newParts->push_back( (*particlesList)[i]->clone());
  }
  
  
  eraseParticlesList();
  particlesList= newParts;
}

void ParticleFilter::endOnFinalWaypoint(){

//   vector< Particle *>::iterator it;
  
//   for(it=particlesList->begin(); it!=particlesList->end(); it++ ){
//     (*it)->getModel()->specialFeature((void **)&wpc,"getWpc");
//     (*it)->stopConnect(wpc, SIGNAL(finalWaypointReached()) );
//   }
}

}