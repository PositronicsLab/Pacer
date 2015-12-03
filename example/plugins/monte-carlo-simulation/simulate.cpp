#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../plugin.h"

#include "common.h"
#include "random.h"

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

//                NAME                  DOF                 RANDOM VALUE        DEFAULT
typedef std::pair<std::vector<boost::shared_ptr<Generator> >, std::vector<double> > ParamDefaultPair;
typedef std::map<std::string, ParamDefaultPair > ParamMap;
ParamMap parameter_generator;

void parse_distribution(const std::string& parameters,std::vector<boost::shared_ptr<Generator> >& generator){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  std::vector<double> min, max, mu, sigma;
  int N = 0;
  bool has_min = false, has_max = false, has_mu = false, has_sigma = false;
  
  std::string distribution_type;
  if(!ctrl->get_data<std::string>(parameters+".distribution", distribution_type))
    throw std::runtime_error("there is no default value OR distribution params for this!");
    
  OUT_LOG(logDEBUG) << parameters << " has a distribution of type: " << distribution_type;
  
  if (has_min = ctrl->get_data<std::vector<double> >(parameters+".min", min)){
    OUT_LOG(logDEBUG) << "min = " << min;
    N = min.size();
  }
  if (has_max = ctrl->get_data<std::vector<double> >(parameters+".max", max)){
    OUT_LOG(logDEBUG) << "max = " << max;
    N = max.size();
  }
  if (has_mu = ctrl->get_data<std::vector<double> >(parameters+".mu", mu)){
    OUT_LOG(logDEBUG) << "mu = " << mu;
    N = mu.size();
  }
  if (has_sigma = ctrl->get_data<std::vector<double> >(parameters+".sigma", sigma)){
    OUT_LOG(logDEBUG) << "sigma = " << sigma;
    N = sigma.size();
  }
  
  generator.resize(N);
  for (int i=0;i<N;i++) {
    generator[i] = boost::shared_ptr<Generator>(new Generator());
    if(distribution_type.compare("gaussian") == 0){
      if (has_max && has_min && has_mu && has_sigma) {
        generator[i]->set_gaussian(mu[i],sigma[i],min[i],max[i]);
      } else if (has_max && has_min && !has_mu && !has_sigma) {
        generator[i]->set_gaussian_from_limits(min[i],max[i]);
      } else if (has_max && !has_min && !has_mu && !has_sigma) {
        generator[i]->set_gaussian_from_limits(-max[i],max[i]);
      } else if (!has_max && !has_min && has_mu && has_sigma) {
        generator[i]->set_gaussian(mu[i],sigma[i]);
      } else {
        throw std::runtime_error("Not a valid set of params for a GAUSSIAN distribution!");
      }
    } else if(distribution_type.compare("uniform") == 0){
      if (has_max && has_min) {
        generator[i]->set_uniform(min[i],max[i]);
      } else if (has_max && !has_min){
        generator[i]->set_uniform(-max[i],max[i]);
      } else {
        throw std::runtime_error("Not a valid set of params for a UNIFORM distribution!");
      }
    } else {
      throw std::runtime_error("Not a valid distribution!");
    }
  }
}

// create distribution map
void create_distributions(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  // Initialize Parameter distributions
  
  // FOR EACH UNCERTAINTY (manufacturing ,state)
  std::vector<std::string> uncertainty_names;
  if(ctrl->get_data<std::vector<std::string> >("uncertainty.id",uncertainty_names)){
    for (int i=0; i<uncertainty_names.size();i++) {
      std::string& uncertainty_name = uncertainty_names[i];
      
      // FOR EACH TYPE (joint, link)
      std::vector<std::string> type_names;
      if(ctrl->get_data<std::vector<std::string> >("uncertainty."+uncertainty_name+".id",type_names)){
        for (int j=0; j<type_names.size();j++) {
          std::string& type_name = type_names[j];
          
          // FOR EACH OBJECT (joint name, link name)
          std::vector<std::string> object_names;
          if(ctrl->get_data<std::vector<std::string> >("uncertainty."+uncertainty_name+"."+type_name+".id",object_names)){
            for (int k=0; k<object_names.size();k++) {
              std::string& object_name = object_names[k];
              
              std::vector<std::string> value_names;
              if(ctrl->get_data<std::vector<std::string> >("uncertainty."+uncertainty_name+"."+type_name+"."+object_name+".id",value_names)){
                for (int l=0; l<value_names.size();l++) {
                  std::string& value_name = value_names[l];
                  
                  // Create vector of generators and default values
                  std::vector<boost::shared_ptr<Generator> > generator;
                  std::vector<double>                        default_value;
                  // try to use default value
                  if(!ctrl->get_data<std::vector<double> >("uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name,default_value)){
                    // if we're here then there are sub-tags to this parameter (generator params)
                    parse_distribution("uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name,generator);
                    
                    OUT_LOG(logDEBUG) << "Created generator for uncertain parameter: "<< object_name << "." << value_name;
                    OUT_LOG(logDEBUG) << "\t FROM: uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name;
                  } else {
                    OUT_LOG(logDEBUG) << "Used default for uncertain parameter: "<< object_name << "." << value_name;
                    OUT_LOG(logDEBUG) << "\t FROM: uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name;
                    OUT_LOG(logDEBUG) << "\t default: " << default_value;
                  }
                  
                  
                  // error check
                  if( (generator.empty() && default_value.empty()) || (!generator.empty() && !default_value.empty()))
                    throw std::runtime_error("there are default values AND distribution params for this value!");
                  
                  parameter_generator[object_name+"."+value_name] = ParamDefaultPair(generator,default_value);
                }
              }
            }
          }
        }
      }
    }
  }
}

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


std::vector<std::string> SAMPLE_ARGV;
int NUM_THREADS = 1, NUM_SAMPLES = 1;
std::string SAMPLE_BIN = "sample.bin";

struct sigaction action;
long long unsigned int sample_idx=0;
void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  sample_idx++;
  if(sample_idx<=NUM_SAMPLES){
    OUT_LOG(logDEBUG) << "Sample: " << sample_idx;

    if ( sample_processes.size() >= NUM_THREADS ){
      OUT_LOG(logDEBUG) << "Active Threads: " << sample_processes.size();

      std::vector<std::string> PARAMETER_ARGV;
      // NOTE: parallelize this loop
      for(ParamMap::iterator it = parameter_generator.begin(); it != parameter_generator.end();
          it++){
        OUT_LOG(logDEBUG) << "--"<< it->first;
        PARAMETER_ARGV.push_back("--" + it->first);
        if (it->second.first.empty()) {
          for (int i=0;i<it->second.second.size(); i++) {
            double value = it->second.second[i];
            OUT_LOG(logDEBUG) << " " << value;
            PARAMETER_ARGV.push_back(SSTR(value));
          }
        } else { // Generators created for variable
          for (int i=0;i<it->second.first.size(); i++) {
            double value = it->second.first[i]->generate();
            OUT_LOG(logDEBUG) << " " << value;
            // Convert to command line argument
            PARAMETER_ARGV.push_back(SSTR(value));
          }
        }
      }
      
      OUT_LOG(logDEBUG) << "Forking Process!";

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
        
        OUT_LOG(logDEBUG) << "Started Sample ("<< sample_idx <<") with PID ("<< pid <<")";
                
        SAMPLE_ARGV.push_back("--sample");
        SAMPLE_ARGV.push_back(SSTR(sample_idx));
        // Add PARAMETER_ARGV to SAMPLE_ARGV
        SAMPLE_ARGV.insert(SAMPLE_ARGV.end(), PARAMETER_ARGV.begin(), PARAMETER_ARGV.end());
        
        char* const* exec_argv = param_array(SAMPLE_ARGV);
        
        //    execve( GZSERVER_BIN, exec_argv, exec_envars );
        execve( SAMPLE_BIN.c_str() , exec_argv, NULL );
        
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

    }

    
    // Make room for new simulations once one has completed
    typedef std::map<int, unsigned> pid_completed_map;
    
    // NOTE: Do not parallelize this
    //          sample_processes_mutex.lock();
    for(pid_completed_map::iterator it = sample_processes.begin(); it != sample_processes.end();it++){
      if( it->second == 0 ) {
        OUT_LOG(logDEBUG) << "Detected Sample ("<< it->first << ") Exit";
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
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  OUT_LOG(logDEBUG) << "Installing signal handler";
 //-----------------------------------------------------------------------------
  // Multi-core execution : Simulation process spawner
  //-----------------------------------------------------------------------------
  // install sighandler to detect when gazebo finishes
  // TODO: make sighandler class more compatible with using a member function
  memset( &action, 0, sizeof(struct sigaction) );
  //  action.sa_handler = exit_sighandler;
  action.sa_sigaction = exit_sighandler; // NEW
  sigaction( SIGCHLD, &action, NULL );
  
  OUT_LOG(logDEBUG) << "Importing sources of uncertainty";
  create_distributions();
  
  ctrl->get_data<int>(plugin_namespace+".max-threads", NUM_THREADS);
  ctrl->get_data<int>(plugin_namespace+".max-samples", NUM_SAMPLES);
  ctrl->get_data<std::string>(plugin_namespace+".executable", SAMPLE_BIN);
  
  if (!getenv("PACER_PLUGIN_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");
  
  std::string pPath(getenv("PACER_PLUGIN_PATH"));
  SAMPLE_BIN = SAMPLE_BIN + "/" + pPath;
  
  OUT_LOG(logDEBUG) << "MC-Simulation executable: " << SAMPLE_BIN;

  SAMPLE_ARGV.push_back(SAMPLE_BIN);
  SAMPLE_ARGV.push_back("--duration");
  
  SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".duration")));
  SAMPLE_ARGV.push_back("--stepsize");
  SAMPLE_ARGV.push_back(SSTR(ctrl->get_data<double>(plugin_namespace+".dt")));
  
  if(ctrl->get_data<bool>(plugin_namespace+".display"))
    SAMPLE_ARGV.push_back("--display");
  
  return;
}