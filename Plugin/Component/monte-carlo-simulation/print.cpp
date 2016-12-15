#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../plugin.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#define handle_error_en(en, msg) \
do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#include "common.h"

#ifdef USE_GSL
#include "Random.h"
Random::ParamMap parameter_generator;
#else
#error Failing build: This plugin should be built with randomization support from GSL. Set 'USE_GSL' to ON to correct build.
#endif


//                NAME                  DOF                 RANDOM VALUE        DEFAULT
std::vector<std::string> get_sample_options(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  // Get current state
  std::map<std::string,Ravelin::VectorNd> joint_position, joint_velocity;
  ctrl->get_joint_value(Pacer::Robot::position,joint_position);
  ctrl->get_joint_value(Pacer::Robot::velocity,joint_velocity);
  
  Ravelin::VectorNd base_position, base_velocity;
  ctrl->get_base_value(Pacer::Robot::position,base_position);
  ctrl->get_base_value(Pacer::Robot::velocity,base_velocity);
  
  Ravelin::VectorNd base_position_spatial(6);
  base_position_spatial.segment(0,3) = base_position.segment(0,3);
  Ravelin::Origin3d rpy;
  Ravelin::Quatd(base_position[3],base_position[4],base_position[5],base_position[6]).to_rpy(rpy);
  base_position_spatial.segment(3,6) = rpy;
  
  joint_position["BODY0"] = base_position_spatial;
  joint_velocity["BODY0"] = base_velocity;
  OUT_LOG(logINFO) << "Positions: \n" << joint_position;
  OUT_LOG(logINFO) << "Velocities: \n" << joint_velocity;
  
  std::vector<std::string> PARAMETER_ARGV;
  // NOTE: parallelize this loop
#ifdef USE_GSL
  Random::ParamValueMap generated_params;
  
  OUT_LOG(logDEBUG)  << ">> Random::generate_parameters";
  Random::generate_parameters(parameter_generator,ctrl,generated_params);
  OUT_LOG(logDEBUG)  << "<< Random::generate_parameters";
  
  for(Random::ParamValueMap::iterator it = generated_params.begin(); it != generated_params.end();it++){
    OUT_LOG(logDEBUG) << it->first ;
    
    std::vector<std::string> params;
    boost::split(params, it->first, boost::is_any_of("."));
    OUT_LOG(logDEBUG) << params;
    
    OUT_LOG(logDEBUG) << "--"<< it->first << " " << it->second;
    
    PARAMETER_ARGV.push_back("--" + it->first);
    OUT_LOG(logDEBUG) << "Vector size: " << it->second.size();
    
    bool is_x_state = (params.back().compare("x") == 0);
    bool is_xd_state = (params.back().compare("xd") == 0);
    for (int i=0;i<it->second.size(); i++) {
      double value = it->second[i];
      
      if (is_x_state) {
        value += joint_position[params.front()][i];
      } else if (is_xd_state) {
        value = joint_velocity[params.front()][i] * (value+1.0);
      }
      // Convert to command line argument
      PARAMETER_ARGV.push_back(SSTR(value));
      OUT_LOG(logDEBUG) << "Value (" << i << ") : " << value;
      
    }
  }
#else
  OUT_LOG(logDEBUG)  << ">> no random generator used";
#endif
  OUT_LOG(logDEBUG) << "Parameters: " << PARAMETER_ARGV;
  
  return PARAMETER_ARGV;
}

#ifdef USE_THREADS
pthread_mutex_t _sample_processes_mutex;
#else
#warning This plugin should be buit with threading support, please build with 'USE_THREADS' turned ON.
#endif

long long unsigned int sample_idx=0;
int NUM_SAMPLES=0;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  std::string SAMPLE_BIN("sample.bin");
  ctrl->get_data<std::string>(plugin_namespace+".executable", SAMPLE_BIN);
  
  if (!getenv("PACER_COMPONENT_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");
  
  std::string pPath(getenv("PACER_COMPONENT_PATH"));
  SAMPLE_BIN = pPath + "/" + SAMPLE_BIN ;
  
  std::vector<std::string> SAMPLE_ARGV = get_sample_options();
  
  OUT_LOG(logDEBUG) << "Parameters: " << SAMPLE_ARGV;
  
  bool STAND_ROBOT = false;
  ctrl->get_data<bool>(plugin_namespace+".standing",STAND_ROBOT);
  if(STAND_ROBOT)
    SAMPLE_ARGV.push_back("--stand");
  
  bool EXPORT_XML = false;
  ctrl->get_data<bool>(plugin_namespace+".output-model",EXPORT_XML);
  if(EXPORT_XML){
    SAMPLE_ARGV.push_back("--xml");
  }
  
  SAMPLE_ARGV.push_back("--no-pipe");
  
  SAMPLE_ARGV.push_back("--sample");
  SAMPLE_ARGV.push_back(SSTR(sample_idx++));
  
  SAMPLE_ARGV.push_back("--duration");
  SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".duration")));
  
  SAMPLE_ARGV.push_back("--stepsize");
  SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".dt")));
  
  if(ctrl->get_data<bool>(plugin_namespace+".display"))
    SAMPLE_ARGV.push_back("--display");
  
  std::string s;
  for (std::vector<std::string>::const_iterator i = SAMPLE_ARGV.begin(); i != SAMPLE_ARGV.end(); ++i)
    s += *i + " ";
  
  char * message = (char *) s.c_str();
  message[s.size()-1] = 0;
  std::cout << SAMPLE_BIN << " " << message << std::endl;
  
  if (sample_idx >= NUM_SAMPLES) {
    throw std::runtime_error("DONE!");
  }
  return;
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  // install sighandler to detect when gazebo finishes
  
  OUT_LOG(logDEBUG) << "Importing sources of uncertainty";
#ifdef USE_GSL
  Random::create_distributions("uncertainty",ctrl,parameter_generator);
#endif
  ctrl->get_data<int>(plugin_namespace+".max-samples", NUM_SAMPLES);
  
  return;
}

void destruct(){
}
