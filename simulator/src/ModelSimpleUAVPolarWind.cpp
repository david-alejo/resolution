#include "ModelSimpleUAVPolarWind.h"
#include <functions/RandomNumberGenerator.h>
#include <cmath>

using namespace std;

namespace simulator {

//Constants:
const double ModelSimpleUAVPolarWind::defaultTimeStep = 0.01;
const double ModelSimpleUAVPolarWind::defaultInitialTime = 0.00;

Model* ModelSimpleUAVPolarWind::clone() const
{
  ModelSimpleUAVPolarWind *ret= new ModelSimpleUAVPolarWind();
  ret->setParameter(parameter);
  ret->setControl(control);
  ret->setState(state);
  ret->setTimeStep(ts);
  ret->setTime(ctime);
  ret->setInitialState(initial_state);
  ret->k = k;
  ret->c = c;
  ret->normal_dist= new boost::normal_distribution<>();
  ret->weibull_dist = new boost::random::weibull_distribution<>();
//   ret->modelType = modelType;
  return ret;
}


ModelSimpleUAVPolarWind::ModelSimpleUAVPolarWind(const std::vector<double> pState,
			      const double pTs, const double pCtime) {
  ts = pTs; //setting time step
  ctime = pCtime; //setting current time
  k = -1.0;
  c = -1.0;
  if( pState.empty() ){//Setting Default state vector if not given
    
    // [x y v phi h h_dot]
    state.push_back(0.0);
    state.push_back(0.0);
    state.push_back(25.0);
    state.push_back(0.0);
    state.push_back(200.0);
    state.push_back(0.0);	
  } else { //setting state vector to the given one
    state = pState;
  }
  
  //setting default control
  control.clear();
  control.push_back(25.0);
  control.push_back(0.5);
  control.push_back(200.0);
  
  //setting default parameter
  parameter.clear();
  parameter.push_back(1.5);
  parameter.push_back(1.0);
  parameter.push_back(0.3364);
  parameter.push_back(1.4680);
  parameter.push_back(0.3);
  parameter.push_back(20.0);
  parameter.push_back(40.0);
  
  normal_dist= new boost::normal_distribution<>();
  weibull_dist = new boost::random::weibull_distribution<>();
}

Model* ModelSimpleUAVPolarWind::create(const vector< double >& _parameter, const vector< double >& _state, const vector< double >& _control) const
{
  ModelSimpleUAVPolarWind* ret = new ModelSimpleUAVPolarWind(_state);
  ret->setParameter(_parameter);
  ret->setControl(_control);
  ret->getWeibullParameters();
  ret->normal_dist = new boost::normal_distribution<>();
  ret->weibull_dist = new boost::random::weibull_distribution<>();
  
  return ret;
}


ModelSimpleUAVPolarWind::~ModelSimpleUAVPolarWind(){
	state.clear();
	control.clear();
	parameter.clear();
	
}

Model* ModelSimpleUAVPolarWind::createFromFile(const string& filename) const
{
  Model *ret = NULL;
  
  try {
		ParseBlock modelData;
		modelData.load(filename.c_str());
		ret = createFromBlock(modelData);
	} catch (std::runtime_error &e) {
		std::cout << "ModelSimpleUAVPolarWind::create_from_file --> Error while loading data from file: ";
		std::cout << e.what() << std::endl;
		throw(e);
	}
    
  return ret;
}

Model* ModelSimpleUAVPolarWind::createFromBlock(ParseBlock& block) const
{
  ModelSimpleUAVPolarWind *ret = new ModelSimpleUAVPolarWind;
  string f_controller,f_model;
  
  try {
    vector<double> control(controlSize, 0.0);
    ret->init(block("parameters").as<vector<double> >(), block("initial_conditions").as<vector<double> >(), 
	      control);
    ret->setTimeStep(block("T").as<double>());
    ret->getWeibullParameters();
    // DEBUG
//     if (ret->parameter.size() != parameterSize || ret->state.size() != stateSize) {
//       cout << "Parameter size = " << ret->parameter.size() << endl;
//       cout << "Initial state size = " << ret->state.size() << endl;
//       throw(std::runtime_error("Bad vector size"));
//     }
  } catch (std::runtime_error &e) {
    std::cout << "ModelSimpleUAVPolarWind::createFromBlock --> Error while loading data from file: ";
    std::cout << e.what() << std::endl;
    delete ret;
    throw(e);
  }
//   delete checker;

  
  
  return ret; 
}

void ModelSimpleUAVPolarWind::init(const vector< double >& _parameter, const vector< double >& _state, const vector< double >& _control)
{
  parameter = _parameter;
  state = _state;
  control = _control;
  ts = defaultTimeStep;
  ctime = defaultInitialTime;
  initial_state = _state;
  modelType = "ModelSimpleUAVPolarWind";
  getWeibullParameters();
}



int ModelSimpleUAVPolarWind::runOneStep(bool first){
	//Calculate the derivative terms
	
	double x_dot, y_dot, v_dot, h_dot_dot, phi_dot;
	double x, y, v, phi, h, h_dot;
	
	if (first) {
	  initial_state = state;
	}
	
	x = 	state[0];
	y = 	state[1];
	v = 	state[3];
	phi = 	state[4];
	h = 	state[2];
	h_dot = state[5];
			
			
			
	x_dot = v * cos( phi );
	y_dot = v * sin( phi );

	// Avoiding phic-phi_o to go out of the -PI,PI range
	while ( fabs ( control[1] - phi ) > M_PI) 
    		phi = phi + ( control[1] - phi > 0.0 ? 1.0 : -1.0 ) * 2.0 * M_PI;

	phi_dot = parameter[1] * ( control[1] - phi );
	v_dot = parameter[0] * ( control[0] - v );
	h_dot_dot= -parameter[3] * h_dot + parameter[2] * ( control[2] - h );

	//Constraint of phi_dot:
	if ( fabs( phi_dot ) > parameter[4] ) 
    		phi_dot = parameter[4] * ( phi_dot > 0.0 ? 1.0 : -1.0 );
	

	//Speed constraint
	if ( v < parameter[5] ){
    		if ( v_dot < 0.0 ){
        		v_dot=0.0;
		}
	} else if ( v > parameter[6] ){
    		if ( v_dot > 0.0 ){
        		v_dot=0.0;
		}
	}
    	
    	double x_w, y_w;
	getWindSpeeds(x_w, y_w);
	
	
	//Modifying the current state vector
	state[0]= x + (x_dot + x_w) * ts;	// x
	state[1]= y + (y_dot + y_w) * ts;	// y
	state[3]= v + v_dot * ts;	// v
	state[4]= phi + phi_dot * ts;	// phi
	state[2]= h + 0.5 * h_dot_dot * ts * ts + h_dot * ts;	// h
	state[5]= h_dot + h_dot_dot * ts;			// h_dot
	return 0;
}

void ModelSimpleUAVPolarWind::getWeibullParameters()
{
  double &mean = parameter[7];
  double &std_dev = parameter[8];
  if (k < 0.0 || c < 0.0) {
    k = pow(mean / std_dev, 1.086);
    c = mean / std::tgamma(1 + k);
  }
  cout << "ModelSimpleUAVPolarWind::getWeibullParameters. k = " << k << "\t c = " << c << endl;
}


void ModelSimpleUAVPolarWind::getWindSpeeds(double& v_x, double& v_y)
{
  double wind_speed;
  functions::RandomNumberGenerator rnd;
  *weibull_dist= boost::random::weibull_distribution<>(k, c);
  wei_rand n_rnd1( *(rnd.getGen()),*weibull_dist);
  wind_speed = n_rnd1();
  
  //NOTE: For a Weibull distribution W(k,c), use uniform distribution U(0,1) and apply :
  // c·(-ln( U(0,1) ) ) ^ (1/c)
  
  //Wind direction
  double wind_direction;
  
  *normal_dist= boost::normal_distribution<>(parameter[9],parameter[10]);
  norm_rand n_rnd2( *(rnd.getGen()),*normal_dist);
  wind_direction = n_rnd2(); 
// 	cout << wind_direction << endl;
  
  v_x = wind_speed * cos(wind_direction);
  v_y = wind_speed * sin(wind_direction);
}


int ModelSimpleUAVPolarWind::runNSteps(int n){

	double x_dot, y_dot, v_dot, h_dot_dot, phi_dot;
	double x, y, v, phi, h, h_dot;
	
	x = state[0];
	y = state[1];
	v = state[2];
	phi = state[3];
	h = state[4];
	h_dot = state[5];
	
	for (int it=0; it< n; it++){
		x_dot = v * cos( phi );
		y_dot = v * sin( phi );
	
		// Avoiding phic-phi_o to go out of the -PI,PI range
		while ( fabs ( control[1] - phi ) > M_PI) 
			phi = phi + ( control[1] - phi > 0.0 ? 1.0 : -1.0 ) * 2.0 * M_PI;
	
		phi_dot = parameter[1] * ( control[1] - phi );
		v_dot = parameter[0] * ( control[0] - v );
		h_dot_dot= -parameter[3] * h_dot + parameter[2] * ( control[2] - h );
	
		//Constraint of phi_dot:
		if ( fabs( phi_dot ) > parameter[4] ) 
			phi_dot = parameter[4] * ( phi_dot > 0.0 ? 1.0 : -1.0 );
		
	
		//Speed constraint
		if ( v < parameter[5] ){
			if ( v_dot < 0.0 ){
				v_dot=0.0;
			}
		}
		else if ( v > parameter[6] ){
			if ( v_dot > 0.0 ){
				v_dot=0.0;
			}
		}
		

	
		//Modifying the current state vector
		state[0]= x + x_dot * ts;	// x
		state[1]= y + y_dot * ts;	// y
		state[2]= v + v_dot * ts;	// v
		state[3]= phi + phi_dot * ts;	// phi
		state[4]= h + 0.5 * h_dot_dot * ts * ts + h_dot * ts;	// h
		state[5]= h_dot + h_dot_dot * ts;	// h_dot
	}

	return 0;
}

}
