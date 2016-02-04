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
    SampleConditions& conditions = sample.second;

    max_fd = std::max( max_fd, conditions.CHILD_TO_PARENT[READ_INDEX] );
    FD_SET( conditions.CHILD_TO_PARENT[READ_INDEX], &pending_fds );
    OUT_LOG(logDEBUG2) << "checking -- fd: " << conditions.CHILD_TO_PARENT[READ_INDEX];
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
    SampleConditions conditions    = sample.second;
    int fd;
    // predator controller
    if( FD_ISSET( conditions.CHILD_TO_PARENT[READ_INDEX], &pending_fds ) != 0 ) {
      fd = conditions.CHILD_TO_PARENT[READ_INDEX];
      if( read(fd, message, sizeof(message)) == -1 ) {
        message[0] = '\0';
        OUT_LOG(logDEBUG2) << "Message failed to read from thread = "<<thread_number<<" FD: "<< conditions.CHILD_TO_PARENT[READ_INDEX];
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

#ifdef USE_THREADS
#include <pthread.h>
pthread_t listener_thread;
#endif

void listen_for_child_return() {
  if(select()){
    std::vector<std::string> messages;
    std::vector<int> completed_threads;
    int n_messages = read_notifications( messages , completed_threads );
    for (int i=0; i<n_messages; i++) {
      int thread_number = completed_threads[i];
      SampleConditions& sc = sample_processes[thread_number];
      double start_time = sc.start_time;
      
      std::vector<std::string> values;
      boost::split(values, messages[i], boost::is_any_of(" "));
      //    std::cout << "Process " << info->si_pid << " output message: " << message << std::endl;
      
      double time_elapsed = atof(values[0].c_str());
      completed_poses.push_back(Ravelin::Pose3d(Ravelin::Quatd(atof(values[4].c_str()), atof(values[5].c_str()), atof(values[6].c_str()), atof(values[7].c_str())),
                                                Ravelin::Origin3d( atof(values[1].c_str()), atof(values[2].c_str()), atof(values[3].c_str()))));
      std::cout << "Sample ("<< sc.sample_number <<") timeline: " << start_time << "  |======" << time_elapsed << "======>  " << (start_time+time_elapsed) << std::endl;
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

