//#include <sstream>
//#include <vector>
//#include <sys/types.h>
//#include <sys/mman.h>
//#include <stdio.h>
//#include <sys/wait.h>
//#include <atomic>
//#include <cstring>
//#include <signal.h>
//#include <unistd.h>
//#include <map>
//#include <time.h>

#define MAX_TIMER_EVENTS 100

char spstr[512];

fd_set                         pending_fds;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool select( void ) {
//  __select( subscribed_fds, pending_fds );

  int max_fd = 0;

  assert( sample_processes.size() > 0 );

  FD_ZERO( &pending_fds );
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  BOOST_FOREACH(Sample sample, sample_processes){
    SampleConditions& sc = sample.second;
     if(sc.restart){
       continue;
     }

    max_fd = std::max( max_fd, sc.CHILD_TO_PARENT[READ_INDEX] );
    FD_SET( sc.CHILD_TO_PARENT[READ_INDEX], &pending_fds );
    OUT_LOG(logDEBUG2) << "checking -- fd: " << sc.CHILD_TO_PARENT[READ_INDEX];
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif

  OUT_LOG(logDEBUG2) << "max_fd: " << max_fd ;

  if( select( max_fd + 1, &pending_fds, NULL, NULL, NULL ) == -1 ) {

    if( LOG(logDEBUG2) ){
      char buf[16];
      if( errno == EBADF ) 
        sprintf( buf, "EBADF" );
      else if( errno == EINTR ) 
        sprintf( buf, "EINTR" );
      else if( errno == EINVAL ) 
        sprintf( buf, "EINVAL" );
      else if( errno == ENOMEM ) 
        sprintf( buf, "ENOMEM" );
      else 
        sprintf( buf, "UNKNOWN" );

      sprintf( spstr, "ERROR : (coordinator.cpp) select() failed calling __select(...) : errno[%s]", buf );
      OUT_LOG(logDEBUG2) << spstr;
    }

    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
int read_notifications( std::vector<std::string>& messages , std::vector<int>& completed_threads) {
  char message[MESSAGE_SIZE];
  int message_count = 0;
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  BOOST_FOREACH(Sample sample, sample_processes){
    int              thread_number = sample.first;
    SampleConditions& sc    = sample.second;
    int fd;
    // predator controller
    if( FD_ISSET( sc.CHILD_TO_PARENT[READ_INDEX], &pending_fds ) != 0 ) {
      fd = sc.CHILD_TO_PARENT[READ_INDEX];
      if( read(fd, message, sizeof(message)) == -1 ) {
        message[0] = '\0';
        OUT_LOG(logDEBUG2) << "Message failed to read from thread = "<<thread_number<<" FD: "<< sc.CHILD_TO_PARENT[READ_INDEX];
      } else {
        message_count += 1;
        OUT_LOG(logDEBUG2) << "Message read from thread = "<<thread_number<<": "<<message;
      }
      messages.push_back(std::string(message));
      completed_threads.push_back(thread_number);
      // Close test
    }
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
  return message_count;
}


//-----------------------------------------------------------------------------
bool init_pipe( int& read_fd, int& write_fd, bool write_blocking = false ) {
  int flags;
  int fd[2];

  if( pipe( fd ) != 0 ) {
    return false;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  if( write_blocking )
    fcntl( fd[1], F_SETFL, flags );
  else
    fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], read_fd ) == -1 ) {
    close( fd[0] );
    close( fd[1] );
    return false;
  }
  if( dup2( fd[1], write_fd ) == -1 ) {
    close( read_fd );
    close( fd[1] );
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------

bool init_pipe( int fd[2], bool write_blocking = false ) {
  int flags;
  
  if( pipe( fd ) != 0 ) {
    return false;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  if( write_blocking )
    fcntl( fd[1], F_SETFL, flags );
  else
    fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );
  
  return true;
}

void listen_for_child_return() {
  if(select()){
    std::vector<std::string> messages;
    std::vector<int> completed_threads;
    int n_messages = read_notifications( messages , completed_threads );
#ifdef USE_THREADS
    pthread_mutex_lock(&_sample_processes_mutex);
#endif
    for (int i=0; i<n_messages; i++) {
      int thread_number = completed_threads[i];
      SampleConditions& sc = sample_processes[thread_number];

      if(messages[i].empty()){
        std::cerr <<"Thread "<< thread_number <<  " Sample ("<< sc.sample_number <<") returned an empty message!" << std::endl;
        continue;
      }

      double start_time = sc.start_time;
      OUT_LOG(logINFO) <<  "Thread "<< thread_number <<  " Sample ("<< sc.sample_number <<") returned with message: " << messages[i];
      std::cout << "Thread "<< thread_number <<  " Sample ("<< sc.sample_number <<") returned with message: " << messages[i] << std::endl;

      std::vector<std::string> values;
      boost::split(values, messages[i], boost::is_any_of(" "));
      
      double time_elapsed = atof(values[0].c_str());
      completed_poses.push_back(Ravelin::Pose3d(Ravelin::Quatd(atof(values[4].c_str()), atof(values[5].c_str()), atof(values[6].c_str()), atof(values[7].c_str())),
                                                Ravelin::Origin3d( atof(values[1].c_str()), atof(values[2].c_str()), atof(values[3].c_str()))));
      OUT_LOG(logINFO) << "Thread "<< thread_number << " Sample ("<< sc.sample_number <<") timeline: " << start_time << "  |======" << time_elapsed << "======>  " << (start_time+time_elapsed);
      std::cout << "Thread "<< thread_number << " Sample ("<< sc.sample_number <<") timeline: " << start_time << "  |======" << time_elapsed << "======>  " << (start_time+time_elapsed) << ".  CURRENT TIME: " << t << std::endl;
      available_threads.insert(thread_number);
    }
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
}

static void *listen_for_child_return_loop(void* data){
  while(1){
    listen_for_child_return();
  }
}

#ifdef USE_THREADS
#include <pthread.h>
pthread_t listener_thread;
pthread_t process_spawner_thread;
#endif

static void start_listener_thread(){
#ifdef USE_THREADS
  if (GET_DATA_ONLINE){
    static int iret = pthread_create( &listener_thread, NULL,&listen_for_child_return_loop,(void*)NULL);
    if(iret)
    {
      fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
      exit(1);
    }
  }
#endif
}

static void spawn_process(SampleConditions& sc, int thread_number){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

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
  
  std::vector<std::string> SAMPLE_ARGV;
  
  //  #ifdef __LINUX__
  //    SAMPLE_ARGV.push_back("nice");
  //    SAMPLE_ARGV.push_back("-n");
  //    SAMPLE_ARGV.push_back("-19");
  //  #else
  // Start process with niceness
  //    SAMPLE_ARGV.push_back("nice");
  //    SAMPLE_ARGV.push_back("-n");
  //    SAMPLE_ARGV.push_back("-19");
  //  #endif
  
  SAMPLE_ARGV.push_back(SAMPLE_BIN);
  
  //////////////////////////////
  //////  SET UP PIPES /////////
  // Before forking, set up the pipe;
  //    if(!init_pipe(sc.PARENT_TO_CHILD[READ_INDEX],sc.PARENT_TO_CHILD[WRITE_INDEX], false )){
  //      throw std::runtime_error("Could not open PARENT_TO_CHILD pipe to worker process.");
  //    }
  //
  //    if(!init_pipe(sc.CHILD_TO_PARENT[READ_INDEX],sc.CHILD_TO_PARENT[WRITE_INDEX], false )){
  //      throw std::runtime_error("Could not open CHILD_TO_PARENT pipe to worker process.");
  //    }
  //
  if(!init_pipe(sc.PARENT_TO_CHILD, false )){
    throw std::runtime_error("Could not open PARENT_TO_CHILD pipe to worker process.");
  }
  
  if(!init_pipe(sc.CHILD_TO_PARENT, false )){
    throw std::runtime_error("Could not open CHILD_TO_PARENT pipe to worker process.");
  }
  
  
  OUT_LOG(logDEBUG)  << "(CHILD) " << sc.CHILD_TO_PARENT[WRITE_INDEX] << " --> " << sc.CHILD_TO_PARENT[READ_INDEX] << " (PARENT)";
  OUT_LOG(logDEBUG)  << "(PARENT) " << sc.PARENT_TO_CHILD[WRITE_INDEX] << " --> " << sc.PARENT_TO_CHILD[READ_INDEX] << " (CHILD)";
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
    // Child process is write-only, closes receiving end of pipe
    close(sc.PARENT_TO_CHILD[WRITE_INDEX]);
    close(sc.CHILD_TO_PARENT[READ_INDEX]);
    
    pid = getpid();
    
    OUT_LOG(logDEBUG) << "Started Thread ("<< thread_number <<") with PID ("<< pid <<")";
    
    SAMPLE_ARGV.push_back("--readpipe");
    SAMPLE_ARGV.push_back(SSTR(sc.PARENT_TO_CHILD[READ_INDEX]));
    
    SAMPLE_ARGV.push_back("--writepipe");
    SAMPLE_ARGV.push_back(SSTR(sc.CHILD_TO_PARENT[WRITE_INDEX]));
    
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
    perror( "execv" );
    throw std::runtime_error("This code should be unreachable unless execve failed!");
    /// ---------- END CHILD PROCESS ------------- ///
    //////////////////////////////////////////////////
  }
  ////////////////////////////////////////////////////////
  /// ---------- CONTINUE PARENT PROCESS ------------- ///
  // Parent is read-only, closing sending end of pipe
  close(sc.PARENT_TO_CHILD[READ_INDEX]);
  close(sc.CHILD_TO_PARENT[WRITE_INDEX]);
  
  sc.pid = pid;
  
  // Before anything else, register thread number with SampleConditions
  sample_processes[thread_number] = sc;
  
  OUT_LOG(logINFO) << "Plugin sleeping to permit thread (" << thread_number << ") with PID ("<< pid <<") to start";
  struct timespec req,rem;
  req.tv_sec = 0;
  req.tv_nsec = 1;
  nanosleep(&req,&rem);
}

static void close_process(SampleConditions& sc){
  OUT_LOG(logDEBUG)  << "CLOSING: (CHILD [already closed]) " << sc.CHILD_TO_PARENT[WRITE_INDEX] << " --> " << sc.CHILD_TO_PARENT[READ_INDEX] << " (PARENT [closing now])";
  close(sc.CHILD_TO_PARENT[READ_INDEX]);
  OUT_LOG(logDEBUG)  << "CLOSING: (PARENT [closing now]) " << sc.PARENT_TO_CHILD[WRITE_INDEX] << " --> " << sc.PARENT_TO_CHILD[READ_INDEX] << " (CHILD [already closed])";
  close(sc.PARENT_TO_CHILD[WRITE_INDEX]);
}

static void process_spawner(){
  for(std::map<pid_t, SampleConditions>::iterator it = sample_processes.begin();it != sample_processes.end();it++)
  {
    SampleConditions& sc = it->second;
     if(sc.restart){
       if(sc.inited){
         close_process(sc);
       }
       spawn_process(sc,it->first);
       sc.restart = false;
       sc.inited = true;
     }
  }
}

static void *process_spawner_loop(void* data){
  while(1){
    process_spawner();
//    sleep_duration(0.5);
    sleep(1);
  }
}

static void start_process_spawner_thread(){
#ifdef USE_THREADS
    static int iret = pthread_create( &process_spawner_thread, NULL,&process_spawner_loop,(void*)NULL);
    if(iret)
    {
      fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
      exit(1);
    }
#endif
}
#include <signal.h>
void exit_sighandler( int signum, siginfo_t* info, void* context ) {
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  std::map<pid_t, SampleConditions>::iterator it = sample_processes.find(info->si_pid);
  {
    SampleConditions& sc = it->second;
     if(sc.pid == info->si_pid){
       sc.restart = true;
     }
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
}

struct sigaction action;
void register_exit_sighandler(){
  memset( &action, 0, sizeof(struct sigaction) );
  //  action.sa_handler = exit_sighandler;
  action.sa_sigaction = exit_sighandler; // NEW
  sigaction( SIGCHLD, &action, NULL );
}
