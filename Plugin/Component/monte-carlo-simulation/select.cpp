////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CHILD LISTENER  ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifdef USE_THREADS
#include <pthread.h>
pthread_t process_spawner_thread;
#endif


////////////////////////////////////////////////////////////////////////////////
////////////////////////////// NEW PROCESS SPAWNER /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::string TASK_PATH = "./";
static void spawn_process(int thread_number){
  OUT_LOG(logINFO) << "PROCESS SPAWNER: spawn_process(.)";
  
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  std::string SAMPLE_BINARY("sample.bin");
  ctrl->get_data<std::string>(plugin_namespace+".executable", SAMPLE_BINARY);
  ctrl->get_data<std::string>(plugin_namespace+".task-directory", TASK_PATH);
  
  if (!getenv("PACER_COMPONENT_PATH"))
  throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");
  
  std::string pPath(getenv("PACER_COMPONENT_PATH"));
  std::string SAMPLE_BIN = pPath + "/" + SAMPLE_BINARY ;
  
  OUT_LOG(logDEBUG) << "MC-Simulation executable: " << SAMPLE_BIN;
  
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
    
    OUT_LOG(logDEBUG) << "In child process for thread ("<< thread_number <<") with PID ("<< pid <<")";
    
//    OUT_LOG(logINFO) << "Moving working directory to: " << TASK_PATH;
    OUT_LOG(logINFO) << ".. then Executing " << SAMPLE_BIN << std::endl;
    
//    chdir( TASK_PATH.c_str() );
    execl( SAMPLE_BIN.c_str() , SAMPLE_BINARY.c_str() , TASK_PATH.c_str() ,  NULL );
    ///////////////////////////////////////////////////
    /// ---------- EXIT CHILD PROCESS ------------- ///
    ///////////////////////////////////////////////////
    
    // This code should be unreachable unless exec failed
    perror( "execl" );
    throw std::runtime_error("This code should be unreachable unless execl failed!");
    /// ---------- END CHILD PROCESS ------------- ///
    //////////////////////////////////////////////////
  }
  ////////////////////////////////////////////////////////
  /// ---------- CONTINUE PARENT PROCESS ------------- ///
  OUT_LOG(logINFO) << "Plugin sleeping to permit thread (" << thread_number << ") with PID ("<< pid <<") to start";
  tick();
//#ifdef USE_THREADS
//  pthread_mutex_lock(&sample_processes[thread_number].mutex);
//#endif
  sample_processes[thread_number].pid = pid;
  
  std::string port_id_simulation = "pacer-planner-" + SSTR(pid);
  sample_processes[thread_number].worker_port = port_id_simulation;
  
  OUT_LOG(logINFO) << "Client connecting to server port " << port_id_simulation << " on thread (" << thread_number << ") with PID ("<< pid <<").  Server must be inited by now!";
  
  sample_processes[thread_number].client = Client(port_id_simulation);
  
  std::string request_reply = "PlanningService-->SimulationService: sending request to SimulationService!";
  sample_processes[thread_number].client.request( request_reply );

  
#ifdef INIT_SIM // if i can ever get this to work
  
  std::vector<std::string> SIM_ARGV;
  SIM_ARGV.push_back("--moby");
  SIM_ARGV.push_back("-s="+SSTR(dt_sample));
  SIM_ARGV.push_back("-p="+pacer_interface_path+"/libPacerMobyPlugin.so");
  bool DISPLAY_MOBY = false;
  ctrl->get_data<bool>(plugin_namespace+".display",DISPLAY_MOBY);
  if(DISPLAY_MOBY){
    SIM_ARGV.push_back("-r");
  }
  
  std::string model_name("model.xml");
  ctrl->get_data<std::string>(plugin_namespace+".moby-model",model_name);
  SIM_ARGV.push_back(model_name);
  
  std::string s;
  for(int i = 0; i < SAMPLE_ARGV.size(); i++)
  s += ( i != 0 )? " " + SAMPLE_ARGV[i] : SAMPLE_ARGV[i];
  
  OUT_LOG(logINFO) << "New Sample: " << sample_idx << " on thread: " << thread_number << " Starting at t = " << t
  << "\nHas simulator options:\n" << SAMPLE_ARGV;
  
  std::cout << "New Sample: " << sample_idx << " on thread: " << thread_number << " Starting at t = " << t << std::endl <<
  " message (main thread): " << s << std::endl;
  
  // expects reply
  client.request( s );
  // ... holding
  if(message != "started"){
    throw std::runtime_error("Message not recieved!");
  }
  
#endif
  sample_processes[thread_number].restart = false;
  sample_processes[thread_number].active = false;
//#ifdef USE_THREADS
//  pthread_mutex_unlock(&sample_processes[thread_number].mutex);
//#endif
  
}

static void *process_spawner_loop(void* data){
  OUT_LOG(logINFO) << "PROCESS SPAWNER: process_spawner_loop(.)";
  
  while(1){
    for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
      if(sample_processes[thread_number].restart){
        spawn_process(thread_number);
      }
      tick(0,1000);
    }
  }
}

static void start_process_spawner_thread(){
  OUT_LOG(logINFO) << "PROCESS SPAWNER: start_process_spawner_thread(.)";
  
#ifdef USE_THREADS
  static int iret = pthread_create( &process_spawner_thread, NULL,&process_spawner_loop,(void*)NULL);
  if(iret)
  {
    throw std::runtime_error("Error - pthread_create() return code: " + SSTR(iret));
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////  WORKER THREADS  //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifdef USE_THREADS
#include <pthread.h>
std::vector<pthread_t> worker_threads;
#endif

std::vector< std::vector<std::string> > sample_messages;

static void *thread_worker(void *threadid)
{
  long thread_number;
  thread_number = (long)threadid;
  
  std::string request_reply = "PlanningService-->SimulationService: Start this simulation: TODO: POLICY INFO";
  sample_processes[thread_number].client.request( request_reply );
  
  while (sample_processes[thread_number].active) {
    std::string request_reply = "PlanningService-->SimulationService: Give me simulation data once you're done";
    sample_processes[thread_number].client.request( request_reply );
    std::vector<std::string> message;
    
    boost::split(message, request_reply, boost::is_any_of(" "));
    if(message.size() > 1){
      sample_messages.push_back(message);
      sample_processes[thread_number].active = 0;
    } else {
      sample_processes[thread_number].active = 0;
      sample_processes[thread_number].restart = 1;
    }
  }
  return (void *) 0;
}

void start_worker_thread(int thread_number){
  OUT_LOG(logINFO) << "PROCESS SPAWNER: start_process_spawner_thread(.)";
  
#ifdef USE_THREADS
  // start worker thread to check on process
  static int iret = pthread_create( &worker_threads[thread_number], NULL,&thread_worker, (void *)thread_number);
  if(iret)
  {
    throw std::runtime_error("Error - pthread_create() return code: " + SSTR(iret));
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// EXIT SIGNAL HANDLER /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define handle_error_en(en, msg) \
do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#include <signal.h>
void exit_sighandler( int signum, siginfo_t* info, void* context ) {
  std::cout << "exit_sighandler entered, something crashed! " << info->si_pid << std::endl;
  for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
    if(sample_processes[thread_number].pid == info->si_pid){
      OUT_LOG(logINFO) << "Process on thread (" << thread_number << ") with PID ("<< sample_processes[thread_number].pid <<") crashed!";
      sample_processes[thread_number].restart = true;
      sample_processes[thread_number].active = false;
    }
  }
}

struct sigaction action;
void register_exit_sighandler(){
  memset( &action, 0, sizeof(struct sigaction) );
  //  action.sa_handler = exit_sighandler;
  action.sa_sigaction = exit_sighandler; // NEW
  sigaction( SIGCHLD, &action, NULL );
}
