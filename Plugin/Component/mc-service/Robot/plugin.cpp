#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../../plugin.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

bool read_notifications(){
  
}

bool init_server_connection() {
  return true;
}

inline void listen_for_child_return() {
  
}

void *listen_for_child_return_loop(void* data){
  OUT_LOG(logINFO) << "LISTENER: listen_for_child_return_loop()";
  while(1){
    listen_for_child_return();
  }
}

#ifdef USE_THREADS
pthread_t listener_thread;
#endif

void start_listener_thread(){
  OUT_LOG(logINFO) << "LISTENER: start_listener_thread()";
  
  static int iret = -1;
#ifdef USE_THREADS
  iret = pthread_create( &listener_thread, NULL,&listen_for_child_return_loop,(void*)NULL);
#endif
  if(iret)
  {
    fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
    throw runtime_error("Failed to create listener thread");
  }
}

//////////////////////////////////////////////////////////////////////////////
////////////////////// CONTROLLER CODE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//////////////////////////////////////////////////////////////////////////////

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  // Get system state
  
  // Get proposed policies
  
  // send current state & policies to server
  
  // check if there is an answer
  
  // if answer set new policy
  
  // else
  return;
}


void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  // communicate to PlannerService that you want it to start up
  
  // wait on PlannerService reply that all SimulatorServices have started
  
  return;
}

void destruct(){
#ifdef USE_THREADS
  pthread_cancel(listener_thread);
#endif
}



