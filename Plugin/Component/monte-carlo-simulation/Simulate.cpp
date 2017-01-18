#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../plugin.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#define INIT_SIM

#define OUT_LOG(x) std::cout << std::endl


#include "service.h"
using namespace Pacer::Service;

#ifdef USE_GSL
#include "Random.h"
Random::ParamMap parameter_generator;
#else
#error Failing build: This plugin should be built with randomization support from GSL. Set 'USE_GSL' to ON to correct build.
#endif

bool GET_DATA_ONLINE = false;

//                NAME                  DOF                 RANDOM VALUE        DEFAULT
void get_sample_options(std::vector<std::string>& PARAMETER_ARGV){
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
}

std::vector<Ravelin::Pose3d> completed_poses;

struct SampleConditions{
  //////////////////////////////////////////////
  ///// Set when worker process is initied /////
  pid_t   pid;
  std::string worker_port;
  
  Client client;
  
  //////////////////////////////////////////////
  ///// Set each time a new sample begins  /////
  int     sample_number;
  double  start_time;
  
  bool  active;
  bool  restart;
  bool  inited;
  
#ifdef USE_THREADS
  pthread_mutex_t mutex;
#else
#warning This plugin should be buit with threading support, please build with 'USE_THREADS' turned ON.
#endif
};

std::vector<SampleConditions> sample_processes;

int NUM_THREADS = 1, NUM_SAMPLES = 1,NUM_WORKING = 100;
double START_TIME = 0;
int sample_idx=0;

#include "simulate-message.cpp"

//#define ARM
//#define QUAD

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  ///////////////////////////////////////////////
  ///////// Visualize final data   //////////////
  
  if (t < START_TIME) {
    return;
  }
  
  std::vector<double> col = ctrl->get_data<std::vector<double> >(plugin_namespace+".color");
  static Ravelin::Vector3d color(&col[0]);
  OUT_LOG(logINFO) << "NUM end messages: " << sample_messages.size() << " out of " << sample_idx << " samples that have been run. " << std::endl;
  for (int i=0; i<sample_messages.size(); i++) {
    OUT_LOG(logINFO) << " end pose from message: " << sample_messages[i]  << std::endl;
    //double stod (const string&  str, size_t* idx = 0);
#ifdef ARM
    Ravelin::Origin3d position(std::stod(sample_messages[i][3]),std::stod(sample_messages[i][4]),std::stod(sample_messages[i][5]));
    OUT_LOG(logINFO) << "position: " << position << std::endl;
    Ravelin::Pose3d completed_pose(Ravelin::Quatd::identity(),position,Pacer::GLOBAL);
    VISUALIZE(CPOSE(completed_pose,color));
#endif
#ifdef QUAD
    Ravelin::Origin3d position(std::stod(sample_messages[i][3]),std::stod(sample_messages[i][4]),std::stod(sample_messages[i][5]));
    Ravelin::Quatd orient = Ravelin::Quatd::rpy(std::stod(sample_messages[i][6]),std::stod(sample_messages[i][7]),std::stod(sample_messages[i][8]));
    OUT_LOG(logINFO) << "pose: " << position << " , " << orient << std::endl;
    Ravelin::Pose3d completed_pose(orient,position,Pacer::GLOBAL);
    VISUALIZE(CPOSE(completed_pose,color));
#endif
  }
  
  int available_threads = 0;
  for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
    if(!sample_processes[thread_number].active)
      available_threads++;
  }
  
  OUT_LOG(logDEBUG) << "num sample processes = " << sample_processes.size();

  OUT_LOG(logDEBUG) << "sample: " << sample_idx << " out of " << NUM_SAMPLES << " total samples: " << sample_idx/NUM_SAMPLES ;
  
  int active_threads = NUM_THREADS-available_threads;
  OUT_LOG(logINFO) << "num threads =  " << NUM_THREADS << ", available threads = " << available_threads << ", active_threads = " << active_threads;

  OUT_LOG(logINFO) << "Active processes: " << active_threads << " out of " << NUM_WORKING << " allowed simultaneous processes.";
  
  if ( active_threads < NUM_WORKING && sample_idx < NUM_SAMPLES ){
    for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
      if(sample_processes[thread_number].active || sample_processes[thread_number].restart)
        continue;
      
#ifndef INIT_SIM // NOT USED
      {
        std::vector<std::string> SIM_ARGV;
        SIM_ARGV.push_back("moby-driver-options");
        SIM_ARGV.push_back("--moby");
        
        double dt_sample = 0;
        ctrl->get_data<double>(plugin_namespace+".dt",dt_sample);
        SIM_ARGV.push_back("s="+SSTR(dt_sample));
        
        static std::string pacer_interface_path(getenv ("PACER_SIMULATOR_PATH"));
        OUT_LOG(logINFO) << "PACER_INTERFACE_PATH: " << pacer_interface_path << std::endl;

        int visualize_step = 0;
        bool OUTPUT_VIZ = ctrl->get_data<int>(plugin_namespace+".visualize-step",visualize_step);
        if(OUTPUT_VIZ){
          SIM_ARGV.push_back("v="+SSTR(visualize_step));
          SIM_ARGV.push_back("y=osg");
        }

        SIM_ARGV.push_back("p="+pacer_interface_path+"/libPacerMobyPlugin.so");
        bool DISPLAY_MOBY = false;
        ctrl->get_data<bool>(plugin_namespace+".display",DISPLAY_MOBY);
        if(DISPLAY_MOBY){
          SIM_ARGV.push_back("r");
        }
        
        std::string model_name("model.xml");
        ctrl->get_data<std::string>(plugin_namespace+".moby-model",model_name);
        SIM_ARGV.push_back(model_name);
        
        std::string s;
        for (int i=0; i<SIM_ARGV.size(); i++) {
          s += ( i != 0 )? " " + SIM_ARGV[i] : SIM_ARGV[i];
        }
        
        OUT_LOG(logINFO) << "New Sample: " << sample_idx << " on thread: " << thread_number << " Starting at t = " << t
        << "\nHas simulator options (set online):\n" << SIM_ARGV;

        std::cerr << "New Sample: " << sample_idx << " on thread: " << thread_number << " Starting at t = " << t
        << "\nHas simulator options (set online):\n" << SIM_ARGV << std::endl;
        
        OUT_LOG(logINFO) << "New Sample: " << sample_idx << " on thread: " << thread_number << " at port: [" << sample_processes[thread_number].worker_port << "] Starting at t = " << t << std::endl <<
        " message (main thread): " << s << std::endl;
        
        // expects reply
        sample_processes[thread_number].client.request( s );
        // ... holding
        
        OUT_LOG(logINFO) << "reply to simulator options " << s << std::endl;
        
        if(s.compare("started") != 0){
          throw std::runtime_error("Message not recieved!");
        }
      }
#endif
      
      
      std::vector<std::string> SAMPLE_ARGV;
      SAMPLE_ARGV.push_back("moby-model-update-options");
      
      get_sample_options(SAMPLE_ARGV);
      
      OUT_LOG(logDEBUG) << "Parameters: " << SAMPLE_ARGV;
      
      bool STAND_ROBOT = false;
      ctrl->get_data<bool>(plugin_namespace+".standing",STAND_ROBOT);
      if(STAND_ROBOT)
        SAMPLE_ARGV.push_back("--stand");
      
      bool EXPORT_XML = false;
      ctrl->get_data<bool>(plugin_namespace+".output-model",EXPORT_XML);
      if(EXPORT_XML)
        SAMPLE_ARGV.push_back("--xml");
      
      SAMPLE_ARGV.push_back("--sample");
      SAMPLE_ARGV.push_back(SSTR(sample_idx));
      
      double duration_sample = 100;
      ctrl->get_data<double>(plugin_namespace+".duration",duration_sample);
      SAMPLE_ARGV.push_back("--duration");
      SAMPLE_ARGV.push_back(SSTR(duration_sample));
      
      // concatenate string into message
      std::string s;
      for (int i = 0; i < SAMPLE_ARGV.size(); i++)
        s += ( i != 0 )? " " + SAMPLE_ARGV[i] : SAMPLE_ARGV[i];
      
      OUT_LOG(logINFO) << "New Sample: " << sample_idx << " on thread: " << thread_number << " Starting at t = " << t;
      OUT_LOG(logINFO) <<  "\nHas simulator options:\n" << SAMPLE_ARGV;
      
      OUT_LOG(logINFO) << "New Sample: " << sample_idx << " on thread: " << thread_number << " Starting at t = " << t;
      OUT_LOG(logINFO) << " message (main thread): " << s;
      
      // expects reply
      sample_processes[thread_number].client.request( s );      // ... holding
      OUT_LOG(logINFO) << "reply to sample options: " << s ;
      
      if(s.compare("simulating") != 0){
        throw std::runtime_error("Message not recieved!");
      }
#ifdef USE_THREADS
      pthread_mutex_lock(&sample_processes[thread_number].mutex);
#endif
      OUT_LOG(logINFO) << "setting up sample_processes, and starting thread worker for : " << thread_number << std::endl;
      
      sample_processes[thread_number].start_time = t;
      sample_processes[thread_number].sample_number = sample_idx++;
      sample_processes[thread_number].active = true;
      
      start_worker_thread(thread_number);
#ifdef USE_THREADS
      pthread_mutex_unlock(&sample_processes[thread_number].mutex);
#endif
      
      if(sample_idx == NUM_SAMPLES)
        break;
    }
    
    
    
  }
  
  if (available_threads == 0 && sample_idx < NUM_SAMPLES) {
    OUT_LOG(logINFO) << "No threads are free ( " << available_threads << " out of " << NUM_THREADS << " ), some samples ( " << sample_idx << " out of " << NUM_SAMPLES << " ).";
  } else if (available_threads == 0 && sample_idx == NUM_SAMPLES) {
    OUT_LOG(logINFO) << "No threads are free ( " << available_threads << " out of " << NUM_THREADS << " ), all samples ( " << sample_idx << " out of " << NUM_SAMPLES << " ) have been started.";
  } else if(available_threads == NUM_THREADS  && sample_idx == NUM_SAMPLES){
    OUT_LOG(logINFO) << "All threads are free ( " << available_threads << " out of " << NUM_THREADS << " ), all samples ( " << sample_idx << " out of " << NUM_SAMPLES << " ) have been started";
    OUT_LOG(logINFO) << "Experiment Complete";
      exit(0);
  } else if (available_threads > 0 && sample_idx == NUM_SAMPLES) {
    OUT_LOG(logINFO) << "Some threads are free ( " << available_threads << " out of " << NUM_THREADS << " ), all samples ( " << sample_idx << " out of " << NUM_SAMPLES << " ) have been started.";
  }
  
  return;
}

//#define USE_SIGHANDLER
void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

#ifdef RUNNING_SERVICE
  int PID = getpid();
  std::string port_id = "pacer-robot-" + SSTR(PID);
  Server server = Server(port_id);
#endif
    //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  // install sighandler to detect when gazebo finishes
  
  OUT_LOG(logDEBUG) << "Importing sources of uncertainty";
#ifdef USE_GSL
  Random::create_distributions("uncertainty",ctrl,parameter_generator);
#endif
  ctrl->get_data<int>(plugin_namespace+".max-threads", NUM_THREADS);
  ctrl->get_data<int>(plugin_namespace+".max-working", NUM_WORKING);
  NUM_WORKING = std::min(NUM_THREADS,NUM_WORKING);
  OUT_LOG(logDEBUG) << NUM_WORKING << " threads are allowed to work simultaneously" ;

  ctrl->get_data<int>(plugin_namespace+".max-samples", NUM_SAMPLES);
  
  /////////////////////////////////
  ////  START WORKER PROCESSES ////
  OUT_LOG(logDEBUG) << "Starting " << NUM_THREADS << " threads" ;

  worker_threads.resize(NUM_THREADS);
  for(int thread_number=0;thread_number<NUM_THREADS;thread_number++){
    OUT_LOG(logDEBUG) << "Starting thread #" << thread_number;
    OUT_LOG(logDEBUG) << "adding process to sample_processes: " << sample_processes.size();

    sample_processes.push_back(SampleConditions());
    sample_processes.back().active = false;
    sample_processes.back().restart = true;
#ifdef USE_THREADS
    pthread_mutex_unlock(&sample_processes[thread_number].mutex);
#endif
    spawn_process(thread_number);
  }
  
  ctrl->get_data<bool>(plugin_namespace+".get-data-online", GET_DATA_ONLINE);
  ctrl->get_data<double>(plugin_namespace+".start-at-time", START_TIME);
  
#ifdef USE_SIGHANDLER
  OUT_LOG(logINFO) << "INIT: register_exit_sighandler";
  
  register_exit_sighandler();
  OUT_LOG(logINFO) << "INIT: start_process_spawner_thread";
  
  start_process_spawner_thread();
#endif
  
  return;
}

void destruct(){
#ifdef USE_SIGHANDLER
  action.sa_sigaction = NULL;  // might not be SIG_DFL
  sigaction( SIGCHLD, &action, NULL );
#endif
}

#ifdef INIT_SIM
#undef INIT_SIM
#endif
