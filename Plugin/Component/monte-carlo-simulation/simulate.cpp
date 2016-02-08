#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../plugin.h"

#include "common.h"

#ifndef USE_GSL
#error Failing This plugin should be buit with randomization support from GSL please set 'USE_GSL' to ON.
#endif

#include <Pacer/Random.h>

bool GET_DATA_ONLINE = false;

Random::ParamMap parameter_generator;

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
  Random::ParamValueMap generated_params;
  
  OUT_LOG(logDEBUG)  << ">> Random::generate_parameters";
  Random::generate_parameters(parameter_generator,ctrl,generated_params);
  OUT_LOG(logDEBUG)  << "<< Random::generate_parameters";
  
  for(Random::ParamValueMap::iterator it = generated_params.begin(); it != generated_params.end();it++){
    std::vector<std::string> params;
    boost::split(params, it->first, boost::is_any_of("."));
    OUT_LOG(logDEBUG) << "--"<< it->first << " " << it->second;
    PARAMETER_ARGV.push_back("--" + it->first);
    for (int i=0;i<it->second.size(); i++) {
      double value = it->second[i];
      
      if (params.back().compare("x") == 0) {
        value += joint_position[params.front()][i];
      } else if (params.back().compare("xd") == 0) {
        value = joint_velocity[params.front()][i] * (value+1.0);
      }
      // Convert to command line argument
      PARAMETER_ARGV.push_back(SSTR(value));
    }
  }
  return PARAMETER_ARGV;
}

std::vector<Ravelin::Pose3d> completed_poses;
#ifdef USE_THREADS
pthread_mutex_t _sample_processes_mutex;
#else
#warning This plugin should be buit with threading support, please build with 'USE_THREADS' turned ON.
#endif

const int READ_INDEX  = 0;
const int WRITE_INDEX = 1;
struct SampleConditions{
  //////////////////////////////////////////////
  ///// Set when worker process is initied /////
  pid_t   pid;
  // [ READ WRITE ]
  int     PARENT_TO_CHILD[2];
  int     CHILD_TO_PARENT[2];
  
  //////////////////////////////////////////////
  ///// Set each time a new sample begins  /////
  int     sample_number;
  double  start_time;
  bool  restart;
  bool  inited;
};

typedef std::pair<int, SampleConditions> Sample;
std::map<int, SampleConditions> sample_processes;
std::set<int> available_threads;

int NUM_THREADS = 1, NUM_SAMPLES = 1;
std::string TASK_PATH = "./";
long long unsigned int sample_idx=0;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  for (int i=0; i<completed_poses.size(); i++) {
    std::cout << "Visualizing end pose: " << completed_poses[i];

    Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Pose(completed_poses[i],1.0)));
  }
  
  OUT_LOG(logINFO) << "Active processes: " << ( NUM_THREADS-available_threads.size() ) << " out of " << NUM_THREADS << " allowed simultaneous processes.";
  
  if ( available_threads.size() > 0 && sample_idx < NUM_SAMPLES){
#ifdef USE_THREADS
    pthread_mutex_lock(&_sample_processes_mutex);
#endif
    for(std::set<int>::iterator it = available_threads.begin(); it != available_threads.end();){
      int thread_number = (*it);
      SampleConditions& sc = sample_processes[thread_number];
      
      std::vector<std::string> SAMPLE_ARGV = get_sample_options();
      
      SAMPLE_ARGV.push_back("--sample");
      SAMPLE_ARGV.push_back(SSTR(sample_idx));
      
      SAMPLE_ARGV.push_back("--duration");
      SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".duration")));
      
      bool EXPORT_XML = false;
      ctrl->get_data<bool>(plugin_namespace+".output-model",EXPORT_XML);
      if(EXPORT_XML){
        SAMPLE_ARGV.push_back("--xml");
      }
      
      SAMPLE_ARGV.push_back("--stepsize");
      SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".dt")));
      
      if(ctrl->get_data<bool>(plugin_namespace+".display")){
        SAMPLE_ARGV.push_back("--display");
      }
      
      char message[MESSAGE_SIZE];
      int message_size = 0;
      for(int i=0;i<SAMPLE_ARGV.size();i++){
        sprintf(&message[message_size]," %s ",SAMPLE_ARGV[i].c_str());
        message_size += SAMPLE_ARGV[i].size()+1;
      }
      
      OUT_LOG(logINFO) << "New Sample: " << sample_idx << " on thread: " << thread_number << "Starting at t=" << t
                     << "\nHas simulator options:\n" << SAMPLE_ARGV;
      
      std::cout << "New Sample: " << sample_idx << " on thread: " << thread_number << "Starting at t=" << t << std::endl;

      
      write(sc.PARENT_TO_CHILD[WRITE_INDEX],message,sizeof(message));
      sc.start_time = t;
      sc.sample_number = sample_idx++;
      available_threads.erase(it++);
      if(sample_idx == NUM_SAMPLES)
        break;
    }
#ifdef USE_THREADS
    pthread_mutex_lock(&_sample_processes_mutex);
#endif
    
    
  } else if (available_threads.size() == 0 && sample_idx < NUM_SAMPLES) {
    OUT_LOG(logINFO) << "All threads are working: sample " << sample_idx << " out of " << NUM_SAMPLES << ".";
  } else if (available_threads.size() == 0 && sample_idx == NUM_SAMPLES) {
    OUT_LOG(logINFO) << "All threads are working and all samples have been started.";
    if (!GET_DATA_ONLINE)
      ctrl->close_plugin(plugin_namespace);
  } else if (available_threads.size() > 0 && sample_idx == NUM_SAMPLES) {
    OUT_LOG(logINFO) << "Not threads are working but all samples have been started.";
    if (!GET_DATA_ONLINE)
      ctrl->close_plugin(plugin_namespace);
  }
  
  if(sample_idx == NUM_SAMPLES && available_threads.size() == NUM_THREADS){
    OUT_LOG(logINFO) << "Experiment Complete";
    ctrl->close_plugin(plugin_namespace);
  }
  
  return;
}

#include "select.cpp"

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  // install sighandler to detect when gazebo finishes
  
  OUT_LOG(logDEBUG) << "Importing sources of uncertainty";
  Random::create_distributions("uncertainty",ctrl,parameter_generator);
  
  /////////////////////////////////
  ////  START WORKER PROCESSES ////
  for (int i=0; i<NUM_THREADS; i++) {
    available_threads.insert(i);
    SampleConditions sc;
    sc.inited = false;
    sc.restart = true;
    sample_processes[i] = sc;
  }
  
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
  ctrl->get_data<bool>(plugin_namespace+".get-data-online", GET_DATA_ONLINE);
  std::cout << "Sleeping before actually starting anything" << t << std::endl;

  start_process_spawner_thread();

  register_exit_sighandler();

  sleep(5);
  std::cout << "AFTER: Sleeping before actually starting anything " << t << std::endl;

  start_listener_thread();
  

  // wait for all new forked processes to start;
  
  return;
}

void destruct(){
  //  _close(ALL_PIPE_FDS);
//-      action.sa_sigaction = NULL;  // might not be SIG_DFL
//-      sigaction( SIGCHLD, &action, NULL );
}
