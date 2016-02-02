#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../plugin.h"

#include "common.h"
#include <Pacer/Random.h>
#include <boost/algorithm/string.hpp>

#include <string>
#include <sstream>
//#define SSTR( x ) ( std::ostringstream() << std::dec << x ).str()

bool get_data_online = false;

template <typename T>
static std::string SSTR(T x)
{
  std::ostringstream oss;
  oss << std::dec << x;
  return oss.str();
}

Random::ParamMap parameter_generator;

//                NAME                  DOF                 RANDOM VALUE        DEFAULT
#include <signal.h>
#include <unistd.h>
#include <map>
#include <time.h>
#ifdef USE_THREADS
pthread_mutex_t _sample_processes_mutex;
#else
#warning This plugin should be buit with threading support, please build with 'USE_THREADS' turned ON.
#endif

struct SampleConditions{
  pid_t   pid;
  double  start_time;
  int     pipe_fd[2];
};

std::vector<Ravelin::Pose3d> completed_poses;
std::map<pid_t, SampleConditions> sample_processes;
//-----------------------------------------------------------------------------
// Signal Handling : Simulation process exit detection and
//-----------------------------------------------------------------------------
void exit_sighandler( int signum, siginfo_t* info, void* context ) {
  // returns time in mu_seconds
  //  double time_elapsed = ( (double) info->si_status ) *  (double) 1.0e-6;
  
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  std::map<pid_t, SampleConditions>::iterator it = sample_processes.find(info->si_pid);
  {
    SampleConditions& sc = it->second;
    char buf[256];
    FILE * file_stream = fdopen(sc.pipe_fd[0],"r");
    read(sc.pipe_fd[0], buf, 256);
    close(sc.pipe_fd[0]);
    std::string message(buf);
    std::vector<std::string> values;
    boost::split(values, message, boost::is_any_of(" "));
    //    std::cout << "Process " << info->si_pid << " output message: " << message << std::endl;
    double start_time = sc.start_time;
    double time_elapsed = atof(values[0].c_str());
    completed_poses.push_back(Ravelin::Pose3d(Ravelin::Quatd(atof(values[4].c_str()), atof(values[5].c_str()), atof(values[6].c_str()), atof(values[7].c_str())),
                                              Ravelin::Origin3d( atof(values[1].c_str()), atof(values[2].c_str()), atof(values[3].c_str()))));
    std::cout << "Sim ("<< info->si_pid <<") timeline: " << start_time << "  |======" << time_elapsed << "======>  " << (start_time+time_elapsed) << std::endl;
    //    Ravelin::Origin3d rpy;
    //    completed_poses.back().q.to_rpy(rpy);
    //    std::cout << "Sim ("<< info->si_pid <<") Orientation: " << rpy << std::endl;
    
  }
  sample_processes.erase(it);
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
  //  std::cout << "Sim ("<< info->si_pid <<") exited!" << std::endl;
}


std::vector<std::string> SAMPLE_ARGV;
int NUM_THREADS = 1, NUM_SAMPLES = 1;
std::string TASK_PATH = "./";

struct sigaction action;
long long unsigned int sample_idx=0;
void loop(){
  for (int i=0; i<completed_poses.size(); i++) {
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Pose(completed_poses[i],1.0)));
  }
  
  
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  OUT_LOG(logINFO) << "Active processes: " << sample_processes.size() << " out of " << NUM_THREADS << " allowed simultaneous processes.";
  //    int started = 0;
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  int sample_processes_size = sample_processes.size();
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
  
  while ( sample_processes_size < NUM_THREADS && sample_idx<NUM_SAMPLES){
    
    // CREATE A NEW SAMPLE
    sample_idx++;
    OUT_LOG(logINFO) << "New Sample: " << sample_idx;
    
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
    
    
    // Before forking, set up the pipe;
    SampleConditions sc;
    sc.start_time = t;
    pipe(sc.pipe_fd);
    OUT_LOG(logDEBUG)  << "Sample output FDs: " << sc.pipe_fd[1] << " --> " << sc.pipe_fd[0] << std::endl;
    OUT_LOG(logDEBUG) << "Forking Process!";
    
    // Run each sample as its own process
    pid_t pid = fork();
    if( pid < 0 ) {
      // fork failed
      OUT_LOG(logERROR) << "Forking Process!";
      
      throw std::runtime_error("Fork failed!");
    }
    
    if( pid == 0 ) {
      ////////////////////////////////////////////////////
      /// ---------- START CHILD PROCESS ------------- ///
      // Child process is write-only, closes receiving end of pipe
      close(sc.pipe_fd[0]);
      
      pid = getpid();
      
      OUT_LOG(logDEBUG) << "Started Sample ("<< sample_idx <<") with PID ("<< pid <<")";
      
      SAMPLE_ARGV.push_back("--pipe");
      SAMPLE_ARGV.push_back(SSTR(sc.pipe_fd[1]));
      
      SAMPLE_ARGV.push_back("--sample");
      SAMPLE_ARGV.push_back(SSTR(sample_idx));
      // Add PARAMETER_ARGV to SAMPLE_ARGV
      SAMPLE_ARGV.insert(SAMPLE_ARGV.end(), PARAMETER_ARGV.begin(), PARAMETER_ARGV.end());
      
      char* const* exec_argv = param_array(SAMPLE_ARGV);
      OUT_LOG(logINFO) << "Moving working directory to: " << TASK_PATH;
      OUT_LOG(logINFO) << ".. then Executing " << SAMPLE_ARGV << std::endl;
      
      chdir(TASK_PATH.c_str());
      execv( SAMPLE_ARGV.front().c_str() , exec_argv );
      ///////////////////////////////////////////////////
      /// ---------- EXIT CHILD PROCESS ------------- ///
      ///////////////////////////////////////////////////
      
      // NOTE: unreachable code (memory leak)
      // TODO: clean up argv
      for ( size_t i = 0 ; i <= SAMPLE_ARGV.size() ; i++ )
        delete [] exec_argv[i];
      
      // This code should be unreachable unless exec failed
      perror( "execve" );
      throw std::runtime_error("This code should be unreachable unless execve failed!");
      /// ---------- END CHILD PROCESS ------------- ///
      //////////////////////////////////////////////////
    }
    ////////////////////////////////////////////////////////
    /// ---------- CONTINUE PARENT PROCESS ------------- ///
    // Parent is read-only, closing sending end of pipe
    close(sc.pipe_fd[1]);
    
    sc.pid = pid;
    
    // Before anything else, register child process in signal map
#ifdef USE_THREADS
    pthread_mutex_lock(&_sample_processes_mutex);
#endif
    sample_processes[sc.pid] = sc;
    sample_processes_size = sample_processes.size();
#ifdef USE_THREADS
    pthread_mutex_unlock(&_sample_processes_mutex);
#endif
    
    //      OUT_LOG(logINFO) << "Plugin sleeping to permit Sample (" << sample_idx << ") with PID ("<< pid <<") to start";
    //
    //      // yield for a short while to let the system start the process
    struct timespec req,rem;
    req.tv_sec = 0;
    req.tv_nsec = 1;
    nanosleep(&req,&rem);
  }
  
  if(sample_processes_size == 0){
    OUT_LOG(logINFO) << "Experiment Complete";
    if (get_data_online) {
      // uninstall sighandler
      //    action.sa_handler = SIG_DFL;
      action.sa_sigaction = NULL;  // might not be SIG_DFL
      sigaction( SIGCHLD, &action, NULL );
      // close self
    }
//    ctrl->close_plugin(plugin_namespace);
  }
  
  sleep(1);
  return;
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  OUT_LOG(logDEBUG) << "Installing signal handler";
  //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  // install sighandler to detect when gazebo finishes
  // TODO: make sighandler class more compatible with using a member function
  ctrl->get_data<bool>(plugin_namespace+".get-data-online", get_data_online);

  if(get_data_online){
    memset( &action, 0, sizeof(struct sigaction) );
    //  action.sa_handler = exit_sighandler;
    action.sa_sigaction = exit_sighandler; // NEW
    sigaction( SIGCHLD, &action, NULL );
  }
  
  OUT_LOG(logDEBUG) << "Importing sources of uncertainty";
  Random::create_distributions("uncertainty",ctrl,parameter_generator);
  
  std::string SAMPLE_BIN("sample.bin");
  ctrl->get_data<int>(plugin_namespace+".max-threads", NUM_THREADS);
  ctrl->get_data<int>(plugin_namespace+".max-samples", NUM_SAMPLES);
  ctrl->get_data<std::string>(plugin_namespace+".executable", SAMPLE_BIN);
  ctrl->get_data<std::string>(plugin_namespace+".task-directory", TASK_PATH);
  
  if (!getenv("PACER_COMPONENT_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");
  
  std::string pPath(getenv("PACER_COMPONENT_PATH"));
  SAMPLE_BIN = pPath + "/" + SAMPLE_BIN ;
  
  OUT_LOG(logDEBUG) << "MC-Simulation executable: " << SAMPLE_BIN;
  
  //#ifdef __LINUX__
  //  SAMPLE_ARGV.push_back("nice");
  //  SAMPLE_ARGV.push_back("-n");
  //  SAMPLE_ARGV.push_back("20");
  //#else
  //  // Start process with niceness
  //  SAMPLE_ARGV.push_back("nice");
  //  SAMPLE_ARGV.push_back("-n");
  //  SAMPLE_ARGV.push_back("20");
  //#endif
  
  SAMPLE_ARGV.push_back(SAMPLE_BIN);
  
  SAMPLE_ARGV.push_back("--duration");
  SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".duration")));
  
  bool EXPORT_XML = false;
  ctrl->get_data<bool>(plugin_namespace+".output-model",EXPORT_XML);
  if(EXPORT_XML){
    SAMPLE_ARGV.push_back("--xml");
  }
  
  SAMPLE_ARGV.push_back("--stepsize");
  SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".dt")));
  
  if(ctrl->get_data<bool>(plugin_namespace+".display"))
    SAMPLE_ARGV.push_back("--display");
  
  return;
}
