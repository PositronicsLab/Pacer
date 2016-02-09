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


////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PIPE READING  /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool select( const std::vector<int>& checked_fds , fd_set& pending_fds ) {
//  __select( subscribed_fds, pending_fds );

  int max_fd = 0;

  int num_fds = 0;
  
  FD_ZERO( &pending_fds );

  for(int i=0;i<checked_fds.size();i++){
    max_fd = std::max( max_fd, checked_fds[i] );
    FD_SET( checked_fds[i] , &pending_fds );
    num_fds += 1;
    OUT_LOG(logINFO) << "LISTENER: checking -- fd: " << checked_fds[i];
  }

  OUT_LOG(logINFO) << "LISTENER: max_fd: " << max_fd ;

  if( select( max_fd + 1, &pending_fds, NULL, NULL, NULL ) == -1 ) {

    if( LOG(logINFO) ){
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

      sprintf( spstr, "ERROR : LISTENER: (select.cpp) select() failed calling __select(...) : errno[%s]", buf );
      OUT_LOG(logINFO) << spstr;
    }

    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
int read_notifications( const std::vector<int>& checked_threads ,  const std::vector<int>& checked_fds , const fd_set& pending_fds ,
                       std::vector<std::string>& messages , std::vector<int>& completed_threads, std::vector<int>& error_threads ) {
  for(int i=0;i<checked_fds.size();i++){
    char message[MESSAGE_SIZE];
    int fd;
    // predator controller
    if( FD_ISSET( checked_fds[i], &pending_fds ) != 0 ) {
      if( read(checked_fds[i], message, sizeof(message)) == -1 ) {
        message[0] = '\0';
        OUT_LOG(logINFO) << "LISTENER: Message failed to read from thread = "<< checked_threads[i] <<" FD: "<< checked_fds[i];
        error_threads.push_back(checked_threads[i]);
      } else {
        messages.push_back(std::string(message));
        completed_threads.push_back(checked_threads[i]);
        OUT_LOG(logINFO) << "LISTENER: Message read from thread = "<< checked_threads[i] <<": "<<message;
      }
    }
  }

  return messages.size();
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

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CHILD LISTENER  ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void listen_for_child_return() {
  std::vector<int> checked_fds,checked_threads;
  fd_set pending_fds;
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
    SampleConditions& sc = sample_processes[thread_number];
    if(sc.restart || !sc.inited || !sc.active){
      continue;
    }
    checked_fds.push_back(sc.CHILD_TO_PARENT[READ_INDEX]);
    checked_threads.push_back(thread_number);
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_sample_processes_mutex);
#endif
  if (checked_fds.empty()) {
    return;
  }

  if(select(checked_fds,pending_fds)){
    std::vector<std::string> messages;
    std::vector<int> completed_threads;
    std::vector<int> error_threads;
    int n_messages = read_notifications( checked_threads, checked_fds,pending_fds, messages , completed_threads, error_threads );
#ifdef USE_THREADS
    pthread_mutex_lock(&_sample_processes_mutex);
#endif
    for(int i=0;i<error_threads.size();i++){
      int thread_number = error_threads[i];
      sample_processes[thread_number].restart = true;
      sample_processes[thread_number].active = false;
    }
    for(int i=0;i<completed_threads.size();i++){
      int thread_number = completed_threads[i];
      SampleConditions& sc = sample_processes[thread_number];

      if(messages[i].empty()){
        // Something bad happened
         OUT_LOG(logINFO) << "Thread "<< thread_number <<  " Sample ("<< sc.sample_number <<") returned an empty message!";
        sc.active = false;
        sc.restart = true;
//        throw std::runtime_error("returned an empty message!");
      }

      double start_time = sc.start_time;
      
      OUT_LOG(logINFO) <<  "Thread "<< thread_number <<  " Sample ("<< sc.sample_number <<") returned with message: " << messages[i];

      std::vector<std::string> values;
      // NOTE: trailing spaces will cause errors in boost::split
      boost::split(values, messages[i], boost::is_any_of(" "));
      OUT_LOG(logINFO) << "Split message: " << values;
      
      // NOTE: this will have to change if I change message size
      if(values.size() != 8){
        // Something bad happened
        OUT_LOG(logINFO) << "Thread "<< thread_number <<  " Sample ("<< sc.sample_number <<") returned a missized message!";
        sc.active = false;
        sc.restart = true;
//        throw std::runtime_error("Recieved bad message from child.");
      }
      
      double time_elapsed = atof(values[0].c_str());
      completed_poses.push_back(Ravelin::Pose3d(Ravelin::Quatd(atof(values[4].c_str()), atof(values[5].c_str()), atof(values[6].c_str()), atof(values[7].c_str())),
                                                Ravelin::Origin3d( atof(values[1].c_str()), atof(values[2].c_str()), atof(values[3].c_str()))));
      OUT_LOG(logINFO) << "Thread "<< thread_number << " Sample ("<< sc.sample_number <<") timeline: " << start_time << "  |======" << time_elapsed << "======>  " << (start_time+time_elapsed);
      sc.active = false;
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
    static int iret = pthread_create( &listener_thread, NULL,&listen_for_child_return_loop,(void*)NULL);
    if(iret)
    {
      fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
      exit(1);
    }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////// NEW PROCESS SPAWNER /////////////////////////////
////////////////////////////////////////////////////////////////////////////////


static void spawn_process(SampleConditions& sc, int thread_number){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  std::string SAMPLE_BIN("sample.bin");
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
//      SAMPLE_ARGV.push_back("nice");
//      SAMPLE_ARGV.push_back("-n");
//      SAMPLE_ARGV.push_back("-19");
//  //  #endif
  
  SAMPLE_ARGV.push_back(SAMPLE_BIN);
  
  // Before anything else, register thread number with SampleConditions
  sample_processes[thread_number] = SampleConditions();

  if(!init_pipe(sample_processes[thread_number].PARENT_TO_CHILD, false )){
    throw std::runtime_error("Could not open PARENT_TO_CHILD pipe to worker process.");
  }
  
  if(!init_pipe(sample_processes[thread_number].CHILD_TO_PARENT, false )){
    throw std::runtime_error("Could not open CHILD_TO_PARENT pipe to worker process.");
  }
  
  
  OUT_LOG(logDEBUG)  << "(CHILD) " << sample_processes[thread_number].CHILD_TO_PARENT[WRITE_INDEX] << " --> " << sample_processes[thread_number].CHILD_TO_PARENT[READ_INDEX] << " (PARENT)";
  OUT_LOG(logDEBUG)  << "(PARENT) " << sample_processes[thread_number].PARENT_TO_CHILD[WRITE_INDEX] << " --> " << sample_processes[thread_number].PARENT_TO_CHILD[READ_INDEX] << " (CHILD)";
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
    close(sample_processes[thread_number].PARENT_TO_CHILD[WRITE_INDEX]);
    close(sample_processes[thread_number].CHILD_TO_PARENT[READ_INDEX]);
    
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
  close(sample_processes[thread_number].PARENT_TO_CHILD[READ_INDEX]);
  close(sample_processes[thread_number].CHILD_TO_PARENT[WRITE_INDEX]);
  
  sample_processes[thread_number].pid = pid;
  
  sample_processes[thread_number].restart = false;
  sample_processes[thread_number].inited = true;
  sample_processes[thread_number].active = false;
  
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
  for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
    SampleConditions& sc = sample_processes[thread_number];
     if(sc.restart){
       if(sc.inited){
         close_process(sc);
       }
       spawn_process(sc,thread_number);
     }
  }
}

static void *process_spawner_loop(void* data){
  while(1){
#ifdef USE_THREADS
    pthread_mutex_lock(&_sample_processes_mutex);
#endif
    process_spawner();
#ifdef USE_THREADS
    pthread_mutex_unlock(&_sample_processes_mutex);
#endif
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// EXIT SIGNAL HANDLER /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <signal.h>
void exit_sighandler( int signum, siginfo_t* info, void* context ) {
  std::cout << "exit_sighandler entered, something crashed! " << info->si_pid << std::endl;
#ifdef USE_THREADS
  pthread_mutex_lock(&_sample_processes_mutex);
#endif
  for(int thread_number=0;thread_number<sample_processes.size();thread_number++){
     if(sample_processes[thread_number].pid == info->si_pid){
       OUT_LOG(logINFO) << "Process on thread (" << thread_number << ") with PID ("<< sample_processes[thread_number].pid <<") crashed!";
       sample_processes[thread_number].restart = true;
       sample_processes[thread_number].active = false;
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
