#include <Pacer/controller.h>
#include "../plugin.h"

std::vector<std::string> SAMPLE_ARGV;

//typedef std::map<std::string,std::vector<std::vector<double> > > DataMap;
typedef std::map<std::string,boost::shared_ptr<Generator> > ParamMap;
ParamMap parameter_generator;

#include "common.h"
#include "random.h"

// create distribution map


#include <signal.h>
#include <unistd.h>
#include <map>
#include <time.h>
std::map<int, unsigned> sample_processes;
//  boost::mutex sample_processes_mutex;
//-----------------------------------------------------------------------------
// Signal Handling : Simulation process exit detection and
//-----------------------------------------------------------------------------
//void gazebo_c::exit_sighandler( int signum ) { // for action.sa_handler
void exit_sighandler( int signum, siginfo_t* info, void* context ) {
  // for action.sa_sigaction
  
//      sample_processes_mutex.lock();
  // update exit condition variable
  sample_processes[info->si_pid] = 0;
//      sample_processes_mutex.unlock();
  
  //  assert( signum );  // to suppress compiler warning of unused variable
}


struct sigaction action;
long long unsigned int sample_idx=0;
void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  sample_idx++;
  if(sample_idx<=NUM_SAMPLES){
    std::vector<std::string> PARAMETER_ARGV;
    // NOTE: parallelize this loop
    for(ParamMap::iterator it = parameter_generator.begin(); it != parameter_generator.end();
        it++){
      char buffer[10];
      sprintf(buffer, "%f",it->second->generate());
      
      PARAMETER_ARGV.push_back("--" + it->first);
      PARAMETER_ARGV.push_back(buffer);
    }

    if ( sample_processes.size() >= NUM_THREADS )
      
    // Run each sample as its own process
    pid_t pid = fork();
    if( pid < 0 ) {
      // fork failed
      throw std::runtime_error("Fork failed!");
    }
    
    if( pid == 0 ) {
      ////////////////////////////////////////////////////
      /// ---------- START CHILD PROCESS ------------- ///
      pid = getpid();
      
      OUT_LOG(logDEBUG) << "Starting Sample ("<< sample_idx <<") with PID ("<< pid <<")\n";
      
      char buffer[10];
      sprintf(buffer, "%u",sample_idx);
      
      SAMPLE_ARGV.push_back("--sample");
      SAMPLE_ARGV.push_back(buffer);
      // Add PARAMETER_ARGV to SAMPLE_ARGV
      SAMPLE_ARGV.insert(SAMPLE_ARGV.end(), PARAMETER_ARGV.begin(), PARAMETER_ARGV.end());
      
      char* const* exec_argv = param_array(SAMPLE_ARGV);
      
      //    execve( GZSERVER_BIN, exec_argv, exec_envars );
      execve( SAMPLE_BIN , exec_argv, NULL );
      
      // NOTE: unreachable code (memory leak)
      // TODO: clean up argv
      for ( size_t i = 0 ; i <= SAMPLE_ARGV.size() ; i++ )
        delete [] exec_argv[i];
      
      // This code should be unreachable unless exec failed
      perror( "execve" );
      exit( EXIT_FAILURE );
      /// ---------- END CHILD PROCESS ------------- ///
      //////////////////////////////////////////////////
    }
    
    ////////////////////////////////////////////////////////
    /// ---------- CONTINUE PARENT PROCESS ------------- ///
    
    // Before anything else, register child process in signal map
    sample_processes[pid] = sample_idx;
    
    OUT_LOG(logDEBUG) << "Plugin sleeping to permit Sample (" << sample_idx << ") with PID ("<< pid <<") to start";
    // yield for a short while to let the system start the process
    struct timespec nanosleep_req;
    struct timespec nanosleep_rem;
    nanosleep_req.tv_sec = 0;
    nanosleep_req.tv_nsec = 1;
    nanosleep(&nanosleep_req,&nanosleep_rem);
    
    // Make room for new simulations once one has completed
    typedef std::map<int, unsigned> pid_completed_map;
    
    // NOTE: Do not parallelize this
    //          sample_processes_mutex.lock();
    for(pid_completed_map::iterator it = sample_processes.begin(); it != sample_processes.end();it++){
      if( it->second == 0 ) {
        OUT_LOG(logDEBUG) << "Detected Sample ("<< it->first << ") Exit\n";
        sample_processes.erase(it);
        break;
      }
    }
    //          sample_processes_mutex.unlock();
    return;
  } else {
    OUT_LOG(logDEBUG) << "Experiment Complete";
    // uninstall sighandler
    //    action.sa_handler = SIG_DFL;
    action.sa_sigaction = NULL;  // might not be SIG_DFL
    sigaction( SIGCHLD, &action, NULL );
    // close self
    ctrl->close_plugin(plugin_namespace);
    return;
  }
}

void setup(){
  //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  // install sighandler to detect when gazebo finishes
  // TODO: make sighandler class more compatible with using a member function
  memset( &action, 0, sizeof(struct sigaction) );
  //  action.sa_handler = exit_sighandler;
  action.sa_sigaction = exit_sighandler; // NEW
  sigaction( SIGCHLD, &action, NULL );
  
  OUT_LOG(logDEBUG) << "signal handler installed";
  return;
}